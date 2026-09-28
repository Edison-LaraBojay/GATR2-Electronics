#!/usr/bin/env bash
# run_perf.sh
# Runs the perf scenarios against one build and writes JSON results, the
# test conditions and a markdown summary into an output folder. Builds
# nothing: configure tools/perf against the tree to measure first.
#
#   tools/perf/run_perf.sh --build <dir> --src <naviGATR tree> --out <dir>
#       [--label name] [--repeats 3] [--first 1] [--duration 20] [--browser-duration 15]
#       [--config config/demo/synthetic_field_demo.xml] [--port 18765]
#       [--cdp-port 19333] [--only server|browser] [--subscribe '<json>']
#       [--browser-scenarios idle,orbit,idle_cpu4,orbit_cpu4]
#
# --build holds navigatr_inspect_bench, navigatr_cdp_bench and navigatr
# (a standalone tools/perf build puts navigatr under navigatr/). --src is
# the naviGATR tree the build came from; the config path is relative to it.
# Server scenarios run in-process (exact host-clock ages). Browser
# scenarios run the navigatr binary on --port and a headless Chrome/Edge
# on --cdp-port. Both ports must be free: a browser run is refused when
# something already answers there, and navigatr_cdp_bench checks that the
# server's session and the browser's DevTools URL are the ones this run
# started. Other load on the machine skews timing: the CPU load is sampled
# before and after every run into conditions.txt. Exit status 1 when any
# run failed (each failure is a FAILED line in conditions.txt).

set -u

BUILD=""
SRC=""
OUT=""
LABEL=""
REPEATS=3
FIRST=1
DURATION=20
BDURATION=15
CONFIG="config/demo/synthetic_field_demo.xml"
PORT=18765
CDP_PORT=19333
ONLY=""
SUBSCRIBE=""
BSCEN="idle,orbit,idle_cpu4,orbit_cpu4"

while [ $# -gt 0 ]; do
    case "$1" in
        --build) BUILD="$2"; shift 2;;
        --src) SRC="$2"; shift 2;;
        --out) OUT="$2"; shift 2;;
        --label) LABEL="$2"; shift 2;;
        --repeats) REPEATS="$2"; shift 2;;
        --first) FIRST="$2"; shift 2;;
        --duration) DURATION="$2"; shift 2;;
        --browser-duration) BDURATION="$2"; shift 2;;
        --config) CONFIG="$2"; shift 2;;
        --port) PORT="$2"; shift 2;;
        --cdp-port) CDP_PORT="$2"; shift 2;;
        --only) ONLY="$2"; shift 2;;
        --subscribe) SUBSCRIBE="$2"; shift 2;;
        --browser-scenarios) BSCEN="$2"; shift 2;;
        -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0;;
        *) echo "unknown argument $1"; exit 2;;
    esac
done
if [ -z "$BUILD" ] || [ -z "$SRC" ] || [ -z "$OUT" ]; then
    echo "need --build, --src and --out"; exit 2
fi
mkdir -p "$OUT"

# MinGW binaries need the ucrt64 runtime first on PATH (0xc0000139 otherwise)
if [ -n "${MSYSTEM:-}" ] || [ -n "${WINDIR:-}" ]; then
    for d in "${MINGW_BIN:-}" /c/msys64/ucrt64/bin; do
        if [ -n "$d" ] && [ -f "$d/libstdc++-6.dll" ]; then PATH="$d:$PATH"; export PATH; break; fi
    done
fi
native() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else echo "$1"; fi; }

find_bin() {
    for c in "$@"; do
        if [ -f "$c" ] || [ -f "$c.exe" ]; then echo "$c"; return 0; fi
    done
    return 1
}
BENCH=$(find_bin "$BUILD/navigatr_inspect_bench" "$BUILD/tools/perf/navigatr_inspect_bench") || { echo "no navigatr_inspect_bench in $BUILD"; exit 2; }
CDPB=$(find_bin "$BUILD/navigatr_cdp_bench" "$BUILD/tools/perf/navigatr_cdp_bench") || { echo "no navigatr_cdp_bench in $BUILD"; exit 2; }
NAV=$(find_bin "$BUILD/navigatr/navigatr" "$BUILD/navigatr") || { echo "no navigatr in $BUILD"; exit 2; }
REPORT=$(find_bin "$BUILD/navigatr_perf_report" "$BUILD/tools/perf/navigatr_perf_report") || REPORT=""

BROWSER=""
for b in "${CHROME_BIN:-}" \
    "/c/Program Files/Google/Chrome/Application/chrome.exe" \
    "/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe" \
    "$(command -v google-chrome 2>/dev/null)" "$(command -v chromium 2>/dev/null)"; do
    if [ -n "$b" ] && [ -x "$b" ]; then BROWSER="$b"; break; fi
done

COND="$OUT/conditions.txt"
NAV_PID=""
BR_PID=""
# children go with us on any exit
cleanup() {
    [ -n "$BR_PID" ] && kill "$BR_PID" 2>/dev/null
    [ -n "$NAV_PID" ] && kill "$NAV_PID" 2>/dev/null
}
trap cleanup EXIT
trap "exit 130" INT TERM
load_sample() {
    if command -v typeperf >/dev/null 2>&1; then
        typeperf "\\Processor(_Total)\\% Processor Time" -sc 3 2>/dev/null | grep '^"[0-9]' | tr '\n' ' '
    elif [ -r /proc/loadavg ]; then
        cat /proc/loadavg
    fi
}
echo "----" >> "$COND"
{
    echo "label: $LABEL"
    echo "date: $(date '+%Y-%m-%d %H:%M:%S %z')"
    echo "build: $(native "$BUILD")"
    echo "src: $(native "$SRC")"
    echo "config: $CONFIG"
    echo "repeats: $FIRST..$((FIRST + REPEATS - 1)), server duration ${DURATION}s, browser duration ${BDURATION}s"
    echo "browser: $BROWSER"
    echo "browser scenarios: $BSCEN; inspect port $PORT, DevTools port $CDP_PORT"
    echo "uname: $(uname -a)"
    echo "cpus: $(nproc 2>/dev/null)"
    if command -v powershell >/dev/null 2>&1; then
        powershell -NoProfile -Command "\$p=Get-CimInstance Win32_Processor; \$o=Get-CimInstance Win32_OperatingSystem; \$g=Get-CimInstance Win32_VideoController; 'cpu: ' + \$p.Name; 'os: ' + \$o.Caption + ' ' + \$o.Version; 'ram_gb: ' + [math]::Round(\$o.TotalVisibleMemorySize/1MB,1); foreach(\$x in \$g){ 'gpu: ' + \$x.Name + ', refresh ' + \$x.CurrentRefreshRate + ' Hz' }; 'power_plan: ' + ((powercfg /getactivescheme) -join ' ')" 2>/dev/null | tr -d '\r'
    elif [ -r /proc/cpuinfo ]; then
        grep -m1 'model name' /proc/cpuinfo
    fi
} >> "$COND"

FAILED=0
fail() {
    echo "FAILED: $*" >> "$COND"
    echo "FAILED: $*" >&2
    FAILED=$((FAILED + 1))
}

# true when something already accepts connections on 127.0.0.1:$1. curl
# exit 7 is "could not connect"; Windows takes about 2 s to refuse a
# loopback connect, hence the 5 s limit.
port_taken() {
    curl -s -m 5 -o /dev/null "http://127.0.0.1:$1/" 2>/dev/null
    [ $? -ne 7 ]
}

run_server() {
    local r="$1"
    local sub=()
    if [ -n "$SUBSCRIBE" ]; then sub=(--subscribe "$SUBSCRIBE"); fi
    rm -f "$OUT/server_r$r.json"
    echo "load before server run $r: $(load_sample)" >> "$COND"
    "$BENCH" --config "$(native "$SRC/$CONFIG")" --scenario all --duration "$DURATION" \
        --warmup 6 "${sub[@]}" --out "$(native "$OUT/server_r$r.json")" > "$OUT/server_r$r.txt" 2>&1
    local rc=$?
    echo "server run $r exit $rc" >> "$COND"
    if [ "$rc" -ne 0 ]; then fail "server run $r: navigatr_inspect_bench exit $rc (see server_r$r.txt)"; fi
    echo "load after server run $r: $(load_sample)" >> "$COND"
}

# stops this run's navigatr and browser, and headless children that
# outlived the launcher (matched by this run's profile dir only)
stop_browser_run() {
    local r="$1" nav_pid="$2" br_pid="$3" prof="$4"
    if [ -n "$br_pid" ]; then
        for _ in $(seq 1 25); do
            kill -0 "$br_pid" 2>/dev/null || break
            sleep 0.2
        done
        kill "$br_pid" 2>/dev/null
    fi
    [ -n "$nav_pid" ] && kill "$nav_pid" 2>/dev/null
    sleep 1
    if [ -n "$br_pid" ] && command -v powershell >/dev/null 2>&1; then
        local tag
        tag="$(basename "$OUT")/.chrome-profile-r$r"
        powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { \$_.ProcessId -ne \$PID -and \$_.CommandLine -like '*$tag*' } | ForEach-Object { Stop-Process -Id \$_.ProcessId -Force -ErrorAction SilentlyContinue }" 2>/dev/null
    fi
    for _ in 1 2 3 4 5; do
        rm -rf "$prof" 2>/dev/null && break
        sleep 1
    done
    NAV_PID=""
    BR_PID=""
}

run_browser() {
    local r="$1"
    rm -f "$OUT/browser_r$r.json"
    if [ -z "$BROWSER" ]; then fail "browser run $r: no browser found"; return; fi
    # never measure a server or a browser that was already there
    if port_taken "$PORT"; then
        fail "browser run $r: 127.0.0.1:$PORT already answers; not measuring whatever holds it (pick --port)"
        return
    fi
    if port_taken "$CDP_PORT"; then
        fail "browser run $r: DevTools port $CDP_PORT already in use, a stale browser? (pick --cdp-port)"
        return
    fi
    echo "load before browser run $r: $(load_sample)" >> "$COND"
    ( cd "$SRC" && exec "$NAV" "$CONFIG" --inspect-port "$PORT" ) > "$OUT/navigatr_r$r.log" 2>&1 &
    local nav_pid=$!
    NAV_PID=$nav_pid
    local prof="$OUT/.chrome-profile-r$r"
    local ok=0
    for _ in $(seq 1 100); do
        kill -0 "$nav_pid" 2>/dev/null || break
        if curl -s -m 5 "http://127.0.0.1:$PORT/api/health" >/dev/null 2>&1; then ok=1; break; fi
        sleep 0.2
    done
    # the session navigatr printed; cdp_bench checks /api/hello against it
    local sess
    sess=$(tr -d '\r' < "$OUT/navigatr_r$r.log" | sed -n 's/^profile .* session \([^ ]*\).*$/\1/p' | head -n 1)
    if [ "$ok" != 1 ] || ! kill -0 "$nav_pid" 2>/dev/null || [ -z "$sess" ]; then
        fail "browser run $r: navigatr did not come up on port $PORT (see navigatr_r$r.log)"
        stop_browser_run "$r" "$nav_pid" "" "$prof"
        return
    fi
    rm -rf "$prof"
    "$BROWSER" --headless=new --remote-debugging-port="$CDP_PORT" --user-data-dir="$(native "$prof")" \
        --no-first-run --no-default-browser-check --disable-background-networking \
        --disable-component-update --disable-sync --disable-extensions --mute-audio \
        --window-size=1600,900 about:blank \
        > "$OUT/browser_r$r.log" 2>&1 &
    local br_pid=$!
    BR_PID=$br_pid
    # the DevTools URL this browser printed; cdp_bench checks the port serves it
    local bws=""
    for _ in $(seq 1 100); do
        # the background launch may not have created the log yet
        if [ -f "$OUT/browser_r$r.log" ]; then
            bws=$(tr -d '\r' < "$OUT/browser_r$r.log" | sed -n 's/^DevTools listening on \(ws:[^ ]*\).*$/\1/p' | head -n 1)
        fi
        [ -n "$bws" ] && break
        kill -0 "$br_pid" 2>/dev/null || break
        sleep 0.2
    done
    if [ -z "$bws" ]; then
        fail "browser run $r: the browser did not open DevTools on port $CDP_PORT (see browser_r$r.log)"
        stop_browser_run "$r" "$nav_pid" "$br_pid" "$prof"
        return
    fi
    "$CDPB" --cdp "127.0.0.1:$CDP_PORT" --url "http://127.0.0.1:$PORT/" --server "127.0.0.1:$PORT" \
        --expect-session "$sess" --expect-browser-ws "$bws" \
        --scenarios "$BSCEN" --duration "$BDURATION" --warmup 8 --label "$LABEL r$r" \
        --close-browser 1 --out "$(native "$OUT/browser_r$r.json")" > "$OUT/browser_r$r.txt" 2>&1
    local rc=$?
    echo "browser run $r exit $rc (server session $sess)" >> "$COND"
    if [ "$rc" -ne 0 ]; then fail "browser run $r: navigatr_cdp_bench exit $rc (see browser_r$r.txt)"; fi
    stop_browser_run "$r" "$nav_pid" "$br_pid" "$prof"
    echo "load after browser run $r: $(load_sample)" >> "$COND"
}

for r in $(seq "$FIRST" $((FIRST + REPEATS - 1))); do
    if [ "$ONLY" != "browser" ]; then echo "server run $r"; run_server "$r"; fi
    if [ "$ONLY" != "server" ]; then echo "browser run $r"; run_browser "$r"; fi
done

if [ -n "$REPORT" ]; then
    "$REPORT" "$(native "$OUT")=${LABEL:-$(basename "$OUT")}" > "$OUT/summary.md" 2>&1
    echo "summary: $OUT/summary.md"
fi
echo "done: $OUT"
if [ "$FAILED" -gt 0 ]; then
    echo "$FAILED run(s) FAILED; see $COND" >&2
    exit 1
fi
