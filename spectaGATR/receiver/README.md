# spectagatr receiver

Laptop program between the robot and the browser viewer. It reads pose lines (PROTOCOL.md,
`SGP1 ...`) from a V5 controller, a Brain USB port, the built-in sim or a replay, and serves the
viewer (`web/`), the field definitions (`fields/`) and a WebSocket feed on
`http://127.0.0.1:8790/`. Structure: [PIPELINE.md](PIPELINE.md).

## Build and run

```
cmake -S spectaGATR -B build-spectagatr -G "MinGW Makefiles"     # Windows, ucrt64 GCC
cmake --build build-spectagatr --parallel
ctest --test-dir build-spectagatr --output-on-failure
build-spectagatr/receiver/spectagatr --source sim                 # opens the browser
cmake --install build-spectagatr --prefix C:/spectagatr           # bin/, share/spectagatr/
```

Each build copies `web/` and `fields/*.json` next to the program. The program looks for them
there, then in `../share/spectagatr/`, then in the source tree. On MinGW it links statically, so
it runs without the ucrt64 DLLs on PATH. Nothing is fetched at runtime.

## Options

| Option | Default | Meaning |
|---|---|---|
| `--source auto\|controller\|brain\|sim\|file` | auto | where poses come from |
| `--port NAME` | search | this serial port (COM7, /dev/ttyACM1) |
| `--list-ports` | | print USB serial ports with their VEX role, then exit |
| `--sim figure8\|circle\|square\|attitude\|restart\|dropout` | figure8 | sim motion (implies `--source sim`) |
| `--rate HZ` / `--seed N` | 10 / 1 | sim rate (1..50) and session seed |
| `--replay PATH` (`-` = stdin) | | replay raw bytes (implies `--source file`) |
| `--replay-rate-bps N` / `--loop` | 12800 / off | replay pace; start the file again at its end |
| `--http-port N` / `--bind ADDR` | 8790 / 127.0.0.1 | viewer address |
| `--field NAME` | override | field shown first (`fields/<name>.json`) |
| `--web DIR` / `--fields DIR` | next to the program | viewer files, field definitions |
| `--stale-ms MS` | 750 | pose older than this is stale |
| `--robot F,B,L,R[,H]` | 0.1685,0.1685,0.1465,0.1465,0.25 | robot box in meters |
| `--no-browser` | | only print the URL |

Ctrl+C stops it.

## Sources

- **auto**: a V5 controller if one is plugged in, else a Brain user port. Searches about once a
  second until one appears.
- **controller**: CDC2 polling of the controller's port. It sends only SYSTEM_VERSION,
  USER_READ (channel 1) and RADIO_STATUS: never stdin bytes, never a radio channel switch.
  `status.detail` shows the link ("controller: Brain linked by radio, radio quality 90" or
  "controller: no Brain linked"). The RADIO_STATUS layout comes from vexide and is unverified.
  Trailing NUL bytes of each reply are dropped (the Brain may pad replies; a NUL inside a line
  would drop it as truncated).
- **brain**: the Brain's USB user port, read raw. It never writes to it: stdin bytes like
  `pRc` change PROS serial settings for the whole program.
- **sim**: deterministic motion inside the field, sent as real pose lines through the same
  scanner, decoder and tracker. `attitude` stands still and sweeps yaw +90, roll +20 (left side
  up), pitch +20 (nose down) degrees, one at a time, to check axis signs. `restart` starts a new
  session every 15 s with a pose reset (epoch) between. `dropout` goes silent 3 s of every 10 s.
- **file**: bytes from a file or stdin at `--replay-rate-bps` (paced by elapsed time), whatever they hold (raw text, PROS COBS
  packets). Link `open` while bytes flow, `searching` at the end. `--loop` rewinds a file and
  starts ordering over.

## Port selection

Roles come from USB ids, never from port names or COM numbers: Brain 2888:0501 interface 2 is
the user port, interface 0 the system port (uploads; never opened), controller 2888:0503. Two
controllers or two Brains are refused with both listed; pick one with `--port`. A `--port` that
the listing does not know is read raw like a Brain port (read-only either way).

## Reconnect

On an unplug or read error the link shows `error` with the reason, then `searching`, and the
same or another device is opened again, without restarting the program. A line cut by the
disconnect is dropped, not joined to the next bytes. If the Brain program kept running, its
poses continue (same session, newer seq) and the trail goes on; a new Brain program run (new
session) or a pose reset (new epoch) breaks the trail. The browser reconnects by itself.

## Status, stale and age

- `pose_age_ms` is how long ago this receiver accepted the newest pose, on its own monotonic
  clock. It is the local receive age, not the end-to-end latency from the Brain's measurement
  to the screen, which is not measured. The Brain's `t_ms` is passed on, never compared with
  laptop time.
- `stale` is true with no pose yet, an age over `--stale-ms`, or a link that is not `open`.
- `counts` has every drop reason: bad_crc, malformed, out_of_range, repeated, out_of_order,
  truncated, too_long, garbage.
- A slow browser never builds a queue: each gets the newest pose and status (a restart pose is
  kept in order), and a client that stops reading for 5 s is closed.

## Limitations

- Polling the controller while the Brain's USB port feeds a Pi through the retired communiGATR
  link (Pi fallback, commit 10c568e) is untested and may steal bytes from that link (one VEXos
  stdout FIFO). Use one mode or
  the other.
- Platforms: Windows built and tested here (host tests; sim, replay and auto-search runs of the
  program). No VEX device was attached: controller, radio and Brain USB behavior is tested
  against fakes only. Linux (x86_64) and macOS (arm64) were only cross-compiled (zig, clang
  18; macOS against a hand-written IOKit declaration shim, not the SDK), never run.
