#!/usr/bin/env bash
# e5 CLI smoke: runs one navigatr.exe through fixed cases, writes raw and
# normalized output. Usage: e5_cli_smoke.sh <navigatr.exe> <out_dir> <port>
set -u
EXE=$1
OUT=$2
PORT=$3
NAV=/c/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/wt-navi/pi/naviGATR
export PATH=/c/msys64/ucrt64/bin:$PATH
mkdir -p "$OUT"
cd "$NAV" || exit 1

run() {
    local name=$1
    shift
    "$EXE" "$@" > "$OUT/$name.out" 2> "$OUT/$name.err"
    echo "exit $?" > "$OUT/$name.code"
}

run help --help
run badopt --bogus
run badreplay --replay foo
run missing does_not_exist.xml
run inline --inline --cycles 20 --inspect-port "$PORT"
run threaded --cycles 60 --inspect-port "$PORT"
EMPTY="$OUT/empty.bin"
: > "$EMPTY"
REPLAY=(--replay "pico_uart=$EMPTY" --replay "brain_usb=$EMPTY")
run brainlink_inline config/override/brain_profile_usb.xml "${REPLAY[@]}" --inline --cycles 5 --inspect-port "$PORT"
run brainlink_threaded config/override/brain_profile_usb.xml "${REPLAY[@]}" --cycles 20 --inspect-port "$PORT"

# normalized: hex ids, and every number on timing-dependent lines
for f in "$OUT"/*.out "$OUT"/*.err; do
    sed -E \
        -e 's/digest [0-9a-f]{16}/digest H/; s/session [0-9a-f]{16}/session H/' \
        -e '/^worker /s/[0-9]+(\.[0-9]+)?/N/g' \
        -e 's#http://127\.0\.0\.1:[0-9]+#URL#' \
        "$f" > "$f.norm"
done
