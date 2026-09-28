#!/usr/bin/env bash
# viewer_smoke.sh
# Headless-browser smoke test for the inspection viewer. Starts the runtime
# with the inspection service on a loopback port, waits for /api/health,
# checks the HTTP documents (hello and the inspect/2 features, the full
# /api/snapshot with its trail, capture status), loads the page in headless
# Edge/Chrome, and checks that the live feed reached the browser:
# data-live-seen, data-snapshots (state messages under inspect/2),
# data-frames and an empty data-errors in the dumped DOM, plus the inspect/2
# attributes the viewer reports (data-contract, data-mode, data-diags,
# data-attitude-status) and a screenshot for a human to look at. A new
# attribute the page does not report is a warning, a wrong value a failure.
# Builds nothing; point it at a binary you already built. Host check only:
# it says nothing about the robot hardware.
#
#   tools/viewer_smoke.sh [binary] [config] [--port N] [--out DIR]
#                         [--attitude-badge assumed|valid|stale|any] [--keep]
#
# Defaults: build-viewer/navigatr.exe (or build/navigatr[.exe]),
# config/demo/synthetic_field_demo.xml, port 8791, out in a temp dir.
# Exit 0 and print PASS on success, non-zero and FAIL otherwise. Runs on
# Windows Git Bash (Edge/Chrome) and on Linux (chromium/google-chrome).

set -u

# --- locate the repo (pi/naviGATR) relative to this script ---
here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/.." && pwd)"
cd "$repo"

BIN=""
CONFIG="config/demo/synthetic_field_demo.xml"
PORT=8791
OUTDIR=""
ATT_BADGE="any"
KEEP=0

while [ $# -gt 0 ]; do
    case "$1" in
        --port) PORT="$2"; shift 2;;
        --out) OUTDIR="$2"; shift 2;;
        --attitude-badge) ATT_BADGE="$2"; shift 2;;
        --keep) KEEP=1; shift;;
        -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0;;
        *) if [ -z "$BIN" ] && [ -z "${BIN_SET:-}" ] && [[ "$1" != *.xml ]]; then BIN="$1"; BIN_SET=1;
           else CONFIG="$1"; fi; shift;;
    esac
done

# --- find the binary ---
if [ -z "$BIN" ]; then
    for c in build-viewer/navigatr.exe build-viewer/navigatr build/navigatr.exe build/navigatr ./navigatr.exe ./navigatr; do
        if [ -x "$c" ] || [ -f "$c" ]; then BIN="$c"; break; fi
    done
fi
if [ -z "$BIN" ] || { [ ! -x "$BIN" ] && [ ! -f "$BIN" ]; }; then
    echo "FAIL: navigatr binary not found (pass it as the first argument)"; exit 2
fi

# A MinGW-built navigatr.exe needs libstdc++/libgcc/winpthread on PATH or it
# exits 0xc0000139 before printing anything. Prepend the ucrt64 runtime when
# it is present (no-op on Linux/the Pi). The matching toolchain must precede
# Git Bash's incompatible DLLs even when a DLL is already visible on PATH.
if [ -n "${MSYSTEM:-}" ] || [ -n "${WINDIR:-}" ]; then
    for d in "${MINGW_BIN:-}" /c/msys64/ucrt64/bin /c/msys64/mingw64/bin; do
        if [ -n "$d" ] && [ -d "$d" ] && [ -f "$d/libstdc++-6.dll" ]; then
            PATH="$d:$PATH"; export PATH
            break
        fi
    done
fi

# --- find a headless browser ---
BROWSER=""
for b in \
    "${CHROME_BIN:-}" \
    "/c/Program Files/Google/Chrome/Application/chrome.exe" \
    "/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe" \
    "$(command -v google-chrome 2>/dev/null)" \
    "$(command -v chromium 2>/dev/null)" \
    "$(command -v chromium-browser 2>/dev/null)"; do
    if [ -n "$b" ] && [ -x "$b" ]; then BROWSER="$b"; break; fi
done 2>/dev/null
if [ -z "$BROWSER" ]; then
    echo "FAIL: no headless browser found (set CHROME_BIN)"; exit 2
fi

# --- output directory ---
if [ -z "$OUTDIR" ]; then
    OUTDIR="$(mktemp -d 2>/dev/null || echo "${TMPDIR:-/tmp}/viewer_smoke.$$")"
    mkdir -p "$OUTDIR"
fi
mkdir -p "$OUTDIR"
OUTDIR="$(cd "$OUTDIR" && pwd)"
DOM="$OUTDIR/dom.html"
SHOT="$OUTDIR/viewer.png"
ERR="$OUTDIR/browser.err.txt"
NAVLOG="$OUTDIR/navigatr.log"
# Always create a new profile; never recursively delete a caller-derived path.
UD="$(mktemp -d "$OUTDIR/browser-profile.XXXXXX")" || exit 2

winpath() { if command -v cygpath >/dev/null 2>&1; then cygpath -w "$1"; else echo "$1"; fi; }

URL="http://127.0.0.1:$PORT/"
echo "binary   $BIN"
echo "config   $CONFIG"
echo "browser  $BROWSER"
echo "url      $URL"
echo "out      $OUTDIR"

# --- start the runtime ---
"$BIN" "$CONFIG" --inspect-port "$PORT" --cycles 30000 > "$NAVLOG" 2>&1 &
NAV_PID=$!
BROWSER_PID=""
WATCHDOG_PID=""

cleanup() {
    if [ -n "$WATCHDOG_PID" ]; then kill "$WATCHDOG_PID" >/dev/null 2>&1; fi
    if [ -n "$BROWSER_PID" ]; then kill "$BROWSER_PID" >/dev/null 2>&1; fi
    kill "$NAV_PID" >/dev/null 2>&1
    wait "$NAV_PID" 2>/dev/null
    # Retain the isolated profile with the other smoke artifacts for diagnosis.
}
trap cleanup EXIT

# --- wait for the service ---
ready=0
for i in $(seq 1 60); do
    if curl -s "http://127.0.0.1:$PORT/api/health" 2>/dev/null | grep -q '"ok":true'; then ready=1; break; fi
    if ! kill -0 "$NAV_PID" 2>/dev/null; then
        echo "FAIL: navigatr exited before serving; log:"; tail -5 "$NAVLOG"; exit 1
    fi
    sleep 0.5
done
if [ "$ready" -ne 1 ]; then
    echo "FAIL: /api/health did not come up; log:"; tail -5 "$NAVLOG"; exit 1
fi
HEALTH="$(curl -s "http://127.0.0.1:$PORT/api/health")"
echo "health   $HEALTH"
HELLO_DOC="$(curl -s "http://127.0.0.1:$PORT/api/hello" 2>/dev/null)"
# a configuration without camera sensors is not expected to deliver frames
CAMERAS=1
if echo "$HELLO_DOC" | grep -q '"camera_sensors":\[\]'; then CAMERAS=0; fi
# the feed contract the server speaks; an inspect/1 server has no contract in health
CONTRACT="$(echo "$HELLO_DOC" | grep -o '"contract":"[^"]*"' | head -1 | sed 's/"contract":"//;s/"$//')"
echo "contract $CONTRACT"

# --- HTTP documents (no browser) ---
http_fail=0
if [ "$CONTRACT" = "navigatr.inspect/2" ]; then
    echo "$HELLO_DOC" | grep -q '"features":{"state_hz":' \
        || { echo "  x hello has no features block"; http_fail=1; }
    SNAP_DOC="$(curl -s "http://127.0.0.1:$PORT/api/snapshot" 2>/dev/null)"
    echo "$SNAP_DOC" | grep -q '"type":"snapshot","contract":"navigatr.inspect/1"' \
        || { echo "  x /api/snapshot is not the inspect/1 snapshot document"; http_fail=1; }
    echo "$SNAP_DOC" | grep -q '"trail":\[' \
        || { echo "  x /api/snapshot has no trail"; http_fail=1; }
    echo "$SNAP_DOC" | grep -q '"attitude":{"valid":[a-z]*,"assumed_level":[a-z]*,"status":"' \
        || { echo "  x robot.attitude has no status"; http_fail=1; }
    CAP_CODE="$(curl -s -o "$OUTDIR/capture_status.json" -w '%{http_code}' \
        "http://127.0.0.1:$PORT/api/capture/status" 2>/dev/null)"
    echo "capture  status HTTP $CAP_CODE $(head -c 120 "$OUTDIR/capture_status.json" 2>/dev/null)"
    grep -q '"available":' "$OUTDIR/capture_status.json" 2>/dev/null \
        || { echo "  x /api/capture/status has no status object"; http_fail=1; }
fi

# --- headless load: virtual time so the socket delivers before the DOM is
#     dumped; a screenshot for a human; swiftshader so WebGL works without a
#     GPU (falls back cleanly if the machine has one). ---
# A live WebSocket/animation can prevent some Chromium versions from
# completing --dump-dom despite the virtual-time budget. Bound the process
# with a real-time watchdog; never leave validation running indefinitely.
# The DOM dump and the screenshot are two runs: with both flags in one run
# this Chrome captures the page before the feed script started, so the
# dumped status never leaves "connecting".
BROWSER_TIMEOUT_SECONDS="${NAVIGATR_BROWSER_TIMEOUT_SECONDS:-60}"
case "$BROWSER_TIMEOUT_SECONDS" in
    ''|*[!0-9]*|0) echo "FAIL: NAVIGATR_BROWSER_TIMEOUT_SECONDS must be positive"; exit 2;;
esac
UD_SHOT="$(mktemp -d "$OUTDIR/browser-profile.XXXXXX")" || exit 2

# browser_run <profile dir> <stdout file> <flags...>: exit status, 124 on timeout
browser_run() {
    local profile="$1" out="$2"
    shift 2
    local flag="$profile/browser-timeout"
    "$BROWSER" --headless=new --disable-gpu --use-angle=swiftshader --enable-unsafe-swiftshader \
        --no-first-run --disable-extensions --disable-background-networking --disable-breakpad \
        --user-data-dir="$(winpath "$profile")" \
        --window-size=1400,900 --virtual-time-budget=10000 "$@" "$URL" > "$out" 2>> "$ERR" &
    BROWSER_PID=$!
    (
        sleep "$BROWSER_TIMEOUT_SECONDS"
        if kill -0 "$BROWSER_PID" >/dev/null 2>&1; then
            echo "browser exceeded ${BROWSER_TIMEOUT_SECONDS}s real-time limit" > "$flag"
            kill "$BROWSER_PID" >/dev/null 2>&1
            sleep 2
            kill -KILL "$BROWSER_PID" >/dev/null 2>&1
        fi
    ) &
    WATCHDOG_PID=$!
    wait "$BROWSER_PID"
    local rc=$?
    BROWSER_PID=""
    kill "$WATCHDOG_PID" >/dev/null 2>&1
    wait "$WATCHDOG_PID" 2>/dev/null
    WATCHDOG_PID=""
    if [ -f "$flag" ]; then
        echo "FAIL: $(cat "$flag")"
        rc=124
    fi
    return $rc
}

: > "$ERR"
browser_run "$UD" "$DOM" --dump-dom
BROWSER_EXIT=$?
browser_run "$UD_SHOT" /dev/null --screenshot="$(winpath "$SHOT")"
SHOT_EXIT=$?
[ "$SHOT_EXIT" -eq 0 ] || echo "  ! screenshot run exited $SHOT_EXIT"

STATUS_TAG="$(grep -o '<div id="status"[^>]*>' "$DOM" | head -1)"
attr() { echo "$STATUS_TAG" | grep -o "data-$1=\"[^\"]*\"" | head -1 | sed "s/data-$1=\"//;s/\"$//"; }

STATE="$(attr state)"
SNAPS="$(attr snapshots)"
FRAMES="$(attr frames)"
ERRORS="$(attr errors)"
LIVE_SEEN="$(attr live-seen)"
ATT="$(attr attitude)"
WEBGL="$(attr webgl)"
# inspect/2 viewer attributes
PAGE_CONTRACT="$(attr contract)"
MODE="$(attr mode)"
DIAGS="$(attr diags)"
TRAIL="$(attr trail)"
ATT_STATUS="$(attr attitude-status)"
SMOOTHING="$(attr smoothing)"

echo "browser exit $BROWSER_EXIT"
echo "status   state=$STATE live-seen=$LIVE_SEEN snapshots=$SNAPS frames=$FRAMES attitude=$ATT webgl=$WEBGL"
echo "feed     contract=$PAGE_CONTRACT mode=$MODE diags=$DIAGS trail=$TRAIL attitude-status=$ATT_STATUS smoothing=$SMOOTHING"
[ -n "$ERRORS" ] && echo "errors   $ERRORS"
[ -f "$SHOT" ] && echo "shot     $SHOT ($(wc -c < "$SHOT") bytes)"

# --- verdict ---
fail=$http_fail
num() { case "$1" in ''|*[!0-9]*) echo 0;; *) echo "$1";; esac; }
has() { echo "$STATUS_TAG" | grep -q "data-$1=\""; }
if [ "$CONTRACT" = "navigatr.inspect/2" ]; then
    if has contract; then
        [ "$PAGE_CONTRACT" = "$CONTRACT" ] \
            || { echo "  x page reports contract '$PAGE_CONTRACT', server speaks $CONTRACT"; fail=1; }
    else
        echo "  ! page does not report data-contract"
    fi
    if has mode; then
        [ "$MODE" = "live" ] || { echo "  x page mode is '$MODE', expected live"; fail=1; }
    else
        echo "  ! page does not report data-mode"
    fi
    if has diags; then
        [ "$(num "$DIAGS")" -ge 1 ] || { echo "  x diags=$DIAGS (need >= 1)"; fail=1; }
    else
        echo "  ! page does not report data-diags"
    fi
    if has attitude-status; then
        case "$ATT_STATUS" in
            measured|stale|assumed_level|unavailable) ;;
            *) echo "  x attitude-status '$ATT_STATUS' is not a server status"; fail=1;;
        esac
    else
        echo "  ! page does not report data-attitude-status"
    fi
fi
if [ -z "$STATUS_TAG" ]; then echo "  x no #status element in the DOM"; fail=1; fi
if [ "$LIVE_SEEN" != "1" ] && [ "$STATE" != "live" ]; then
    echo "  x feed never reached live (state=$STATE, live-seen=$LIVE_SEEN)"; fail=1
fi
if [ "$(num "$SNAPS")" -le 5 ]; then echo "  x snapshots=$SNAPS (need > 5)"; fail=1; fi
if [ "$CAMERAS" -eq 1 ] && [ "$(num "$FRAMES")" -lt 1 ]; then echo "  x frames=$FRAMES (need >= 1)"; fail=1; fi
if [ -n "$ERRORS" ]; then echo "  x page errors present"; fail=1; fi
if [ "$BROWSER_EXIT" -ne 0 ]; then echo "  x browser exited unsuccessfully"; fail=1; fi
if [ "$WEBGL" != "1" ]; then echo "  x 3D view did not initialize"; fail=1; fi
if [ ! -s "$SHOT" ]; then echo "  x screenshot missing"; fail=1; fi
case "$ATT_BADGE" in
    assumed) [ "$ATT" = "assumed_level" ] || { echo "  x attitude badge is '$ATT', expected assumed_level"; fail=1; };;
    valid)   [ "$ATT" = "valid" ] || { echo "  x attitude badge is '$ATT', expected valid"; fail=1; };;
    stale)   [ "$ATT" = "stale" ] || { echo "  x attitude badge is '$ATT', expected stale"; fail=1; };;
    any) ;;
    *) ;;
esac

if [ "$fail" -eq 0 ]; then
    echo "PASS"
    exit 0
fi
echo "FAIL"
exit 1
