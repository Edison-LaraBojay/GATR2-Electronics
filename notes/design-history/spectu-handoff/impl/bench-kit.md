# Bench kit (B1-B13): implementation log

2026-09-29. Workstream key `bench-kit`. Worktree `S/wt-spectu` (detached at ae457ee), where
`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
Edited only `spectuGATR/bench/**` (new; the folder the task names, in no other workstream's list).
Builds only in `S/sp-build/bench-kit-*`. Nothing committed; `git add spectuGATR/bench` (index
only, 43 files staged as A). No other worktree file touched.

## What was built

| Path under `spectuGATR/bench/` | What |
|---|---|
| `BENCH.md` | the kit: build, conventions, troubleshooting, then one section per test B1-B13: goal, what it settles (decision and code knob), needs, exact steps, what to record, a sheet, pass line, outcome |
| `README.md` | signpost (PIPE-1: bench and tools get a README, no PIPELINE.md) |
| `brain/bench_main.cpp` | PROS entry points; a screen menu picks LINES, POSE or FLOOD at start |
| `brain/lines_program.cpp` | LINES: COBS off, raw `T<seq> <ms> <free>[ <pad>]` lines (settings plain 10 ms, pad20, pad200, bin), `S` stdin counts each second, `M<setting>` markers, `BIN<seq>:` + 255 raw bytes; stdin reader counting `P<n>`, `L1<n>`, `L2<n>`, bad lines; screen with write times, BLOCKED, console FIFO free (`vexSerialWriteFree(1)`) |
| `brain/pose_program.cpp` | POSE: the real Publisher + `StdioTransport` (budget 8000/1024/50 Hz), a new pose at 5/10/20/50 Hz picked on screen, event `bench.rate` per pick; joystick change intervals in 10 s windows as `bench.joy_*` channels and on screen (B4) |
| `brain/flood_program.cpp` | FLOOD: SG1 pose lines (protocol `FrameBuilder` + `encodeLine`) written one by one to `/ser/sgtr` at ~2 kB/s, each write timed and its return counted; `printf` heartbeat timed; BLOCKED flags; FIFO free |
| `brain/bench_screen.{h,cpp}`, `brain/programs.h` | screen rows and a touch button bar; program entry points |
| `brain/window_stats.h` | fixed-window p50/p95/max, std only (host-tested; meant for the W6 bench pages too) |
| `host/*` | `spectugatr_bench`: `ports`, `record`, `analyze`, `channel`, `turnaround` |
| `host/recorder.*` | one device into log entries: Brain user port reads (+ `P<n>` stdin lines for B1); controller SYSTEM_VERSION until linked, USER_READ with one request outstanding and per-poll round trip, RADIO_STATUS every 2 s, `L1<n>`/`L2<n>` stdin writes for B1 |
| `host/bench_requests.*` | bench-only CDC2 requests: stdin write in the two encodings, FILE_CTRL radio channel |
| `host/bench_log.*` | the text log (one line per event, flushed per line, re-analyzable) |
| `host/transport_report.*` | per source: bytes, 0x00, 512-multiple reads and the gap after, USER_READ RTT, empty/late/timeouts, trailing 0x00 and odd lengths, stdin write answers per encoding, link and radio changes |
| `host/lines_report.*` | LINES output per setting (missing, repeats, pad, BIN intact, 0x00 inside/between lines, COBS-framed lines, Brain stdin counts) and the B1 routing verdict (duplicate, split, exclusive, mixed) |
| `host/sg1_report.*` | W5 `Decoder` over the data; segments at `bench.rate`; poses/s, pose seq skipped, frames missed, one-way age, B/s; every channel, `spectu.*` counters, events |
| `host/turnaround.*` | Pi request-read to reply-write times from a naviGATR capture's `transport_bytes.csv` (B8) |
| `host/devices.*`, `host/command_line.*`, `host/stats.*`, `host/main.cpp` | real ports (role or path, DTR on/off, reopen), arguments, summaries and min-offset ages |
| `CMakeLists.txt`, `tests/CMakeLists.txt` | standalone project over `../protocol` and `../host`; static exe on MinGW; 40 tests, explicit list |

Test to kit:

| Test | Brain | Host |
|---|---|---|
| B1 | LINES plain | `record --brain-user --controller --stdin --expect lines` (+ `analyze` of a Pi log and a laptop log) |
| B2 | POSE | `record --controller --expect sg1`, `--read-length`, `channel` |
| B3 | LINES pad20/pad200/bin | `record --controller --expect lines` |
| B4 | POSE (joystick windows) | `record --controller`, `--poll-gap 0`, `channel` |
| B5 | FLOOD | screen; `record` as control |
| B6 | POSE | `record --no-dtr` vs default |
| B7 | operaGATR (needs W6 page) + POSE for hold-back | `record --brain-user --expect sg1` |
| B8 | locaGATR (needs W6 page) | Pi capture + `turnaround` |
| B9 | any | `ports` + OS commands |
| B10 | LINES plain | `record --brain-user --expect lines` (COBS-framed lines) |
| B11 | FLOOD | `record` before and after 60 s without a reader |
| B12 | operaGATR (needs W6 page) | - |
| B13 | - | existing `navigatr_inspect_bench --connect` |

## Decisions and deviations (with reasons)

1. **Brain programs live in `spectuGATR/bench/brain/`, compiled by W4's example through a small
   hook** (described below, not applied). The task prefers bench modes in W4/W6 apps or examples
   over new PROS projects, and forbids editing their files; so the code sits in the kit folder
   (the PROS build already compiles `spectuGATR/protocol` from the top-level folder the same way)
   and the example builds it in place of its `main.cpp` with `pros make BENCH=1`. No new PROS
   project.
2. **One Brain build, picked at runtime on the screen**, not a compile-time mode per test: one
   upload for B1-B6, B10, B11; no stale objects when switching (the object names would not change
   with a mode macro); B5 needs choosing without a controller.
3. **Three programs cover eight tests.** LINES: B1, B3, B10. POSE: B2, B4 (the proposal runs B4
   "during B2"), B6, B7 hold-back. FLOOD: B5, B11.
4. **FLOOD writes its own SG1 lines** (same `/ser/sgtr` open and activate as `StdioTransport`):
   B5/B11 need each write's duration and return; `StdioTransport` ignores the return (W4
   decision 15) and exposes only `slow_writes`.
5. **LINES `BIN` lines carry every byte value except 0x0A**, so line assembly holds; 0x00 and
   0x80-0xFF are in. COBS is off in LINES (as communiGATR sets it), so any 0x00 a receiver sees
   there is padding or damage.
6. **B1 host A is the laptop by default** (a second cable into the Brain user port): one clock,
   one log, one command. The routing verdict needs only the Brain's seq numbers, so the Pi as
   host A stays possible: `analyze` takes one log per host.
7. **B1 stdin through the controller: one line per 100 ms, the two encodings in turn, a
   USER_READ between writes, 50 ms write timeout.** The proposal says every 100 ms in both
   encodings; each encoding now goes every 200 ms. Reason: if the controller never answers a
   write (plausible: pros-cli never sends one, cargo-v5 #34), a pair every period at the normal
   250 ms timeout starved polling completely (found while writing the test; test
   `SlowStdinWritesNeverStarvePolling`).
8. **Host tool in C++ on the W3 platform layer and the W5 codec/decoder**, not Python: serial
   access on Windows needs SetupAPI/COMM calls the platform layer already has; one static exe;
   the same decoder as the receiver; it also builds on the Pi. No Python in the kit.
9. **Own controller poll loop instead of `runControllerSource`**: the bench needs every poll's
   round trip and the raw reply boundaries (trailing 0x00 per reply for B3); W5's source reports
   the last RTT once a second and hands on bytes without reply boundaries. It reuses W5's
   `CdcReplyScanner`, parsers and request builders.
10. **Bench-only CDC2 writes (`bench_requests.*`)**: stdin write `[channel, len, data]` in both
    encodings and FILE_CTRL radio channel. W5's `cdc2` stays read-only as designed. The channel
    request bytes equal the pros-cli-computed constants in disc:controller-host s2.1 (test
    `RadioChannelBytesMatchPros`). BENCH.md states the receiver never sends either.
11. **One-way age = (rx - sent) minus the minimum within +-5 s, per session**: a whole-run minimum
    would read clock drift as delay (50 ppm is 15 ms over a 5 min run).
12. **Standalone CMake project at `spectuGATR/bench`** (adds `../protocol` and `../host` with
    `SPECTUGATR_BUILD_TESTS` off) instead of an option in W0's top-level `CMakeLists.txt`: no W0
    file touched. The tests compile W5's two sim fakes (`fake_controller.cpp`, `fake_port.cpp`)
    as code under test.
13. **Namespaces `spectugatr::bench` (tool) and `spectugatr::bench::brain` (Brain programs)**,
    not `spectugatr::bench::host`: inside that, `host::` would name the bench namespace and hide
    `spectugatr::host`.
14. **B7, B8, B12 need Brain numbers the apps do not show** (link RTT and robot age windows, USB
    stats, publisher stats, drive period): described below as a W6 change; BENCH.md lists them
    under "Not in place yet" and in each test's Needs.
15. **Not in the kit:** the B8 Pico to Pi minimum latency (needs a Pico pin pulse and a Pi edge
    time, or a logic analyzer; BENCH.md says so); B7's Pi parse time and line gaps (need the Pi's
    SG1 split, Phase B). B13 uses the existing `navigatr_inspect_bench --connect`, no new code.
16. **Pass lines** are mine where the proposal gives none (for example B2: 95 % of the rate
    delivered, 1 % frames missed; B13: one way 2 ms mean / 3 ms p95, the margin the 4.4 budget
    leaves over the modeled 9.9 / 12.7). Each is stated in its sheet next to the outcome it
    drives.
17. **`WindowStats` tests use a window of 100**: with a window of 4, GCC 15 at -O2 raises a
    false `-Warray-bounds` inside `std::sort`'s introsort path; the Brain's windows are 2048.

## Changes needed in other workstreams' files (described, not made)

**W4, `make BENCH=1`** (verified in a scratch mirror, `S/sp-build/bench-kit-pros`, diffs in
`hook_makefile.diff` and `hook_mk.diff` there):

`brain/spectuGATR/examples/standalone/Makefile`, after `GATR2_BRAIN_LIBS := spectugatr`:
```make

# Bench programs in place of src/main.cpp: make BENCH=1 (spectuGATR/bench/BENCH.md).
ifdef BENCH
EXCLUDE_SRCDIRS  += $(SRCDIR)/main.cpp
GATR2_BRAIN_LIBS += spectugatr_bench
endif
```
(`EXCLUDE_SRCDIRS` is PROS common.mk's filter-out list for `src/` objects.)

`brain/gatr2_brain.mk`, before `GATR2_SRC := ...`:
```make
GATR2_SRC_spectugatr_bench := \
	$(GATR2_ROOT)/spectuGATR/bench/brain/bench_main.cpp \
	$(GATR2_ROOT)/spectuGATR/bench/brain/bench_screen.cpp \
	$(GATR2_ROOT)/spectuGATR/bench/brain/flood_program.cpp \
	$(GATR2_ROOT)/spectuGATR/bench/brain/lines_program.cpp \
	$(GATR2_ROOT)/spectuGATR/bench/brain/pose_program.cpp
```
Optional: one line in the example's README: "`pros make BENCH=1` builds the bench programs
instead (spectuGATR/bench/BENCH.md)."

**W6, bench page and pose cap (B7, B8, B12):**
- `brain/robot/gatr2_robot.h`: `kSpectuBenchPage = false` and `kSpectuUsbPoseHz = 25.0`.
- operaGATR `createLink`: copy `kUsbRelayBudget`, set `pose_hz = kSpectuUsbPoseHz`, pass it to
  `usbLane` (B7 run 3 sets 50).
- operaGATR `display()`, with `kSpectuBenchPage`, rows 6-9 (or a second page) show:
  - `polls/s`, `timeouts` (`ClientStats` deltas over 1 s), round trip p50/p95/max (one sample of
    `ProsLinkStatus::state.round_trip` per new `state.received_at`, a `WindowStats<256>`);
  - `robot_age_ms` p50/p95 (same samples) and state interval p95/max (`received_at` deltas);
  - `ProsLinkStatus::usb`: `frames_out`, `expired`, `short_writes`, `aux_lines`, `aux_short`;
  - `spectugatr::stats(*g_viz)`: `lines`, `slow_writes`, `next_line_max_us`,
    `events_contended`, `events_dropped` (spectuGATR on);
  - drive step period p50/p95/max and steps over 15 ms: a `ProsDriveConfig::observer` wrapper
    recording `now - last` per step into a window under a `pros::Mutex` (drive task writes, the
    background task reads), then calling the DriveFeed observer when spectuGATR is on. It must
    work with spectuGATR off (B12 run 1).
- locaGATR: the round trip, robot age and state interval rows (B8).
- Both include `spectuGATR/bench/brain/window_stats.h` (`$(GATR2_ROOT)` is on the include path).
- `docs/brain_setup.md` section 11 could link `spectuGATR/bench/BENCH.md` where it names B5 and
  B7.

**W0:** `spectuGATR/README.md` could list `- bench/: hardware bench kit B1-B13
([bench/BENCH.md](bench/BENCH.md))`.

## Commands and results (Git Bash, `export PATH=/c/msys64/ucrt64/bin:$PATH`)

```
# host tool, Debug
cmake -S S/wt-spectu/spectuGATR/bench -B S/sp-build/bench-kit-host -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src
cmake --build S/sp-build/bench-kit-host -j16 --clean-first      -> exit 0, 0 warnings
ctest --test-dir S/sp-build/bench-kit-host -j16                  -> 100% passed, 40/40
S/sp-build/bench-kit-host/tests/spectugatr_bench_tests.exe --gtest_shuffle --gtest_repeat=5
                                                                 -> 5 x PASSED 40
# Release (S/sp-build/bench-kit-release, same flags, -DCMAKE_BUILD_TYPE=Release, --clean-first)
                                                                 -> exit 0; 40/40 passed;
   1 warning: W0 protocol/src/doc_assembler.cpp:46 -Wmaybe-uninitialized (W0 review S3)
objdump -p spectugatr_bench.exe | grep "DLL Name"                -> system DLLs only (static)
spectugatr_bench.exe ports                                       -> "0 USB serial ports" (no VEX
                                                                    device on this box)
```

Warning gate (GCC 15.2, -O2, `-Wall -Wextra -Wpedantic -Wshadow -Wundef -Wold-style-cast
-Wextra-semi -Wmissing-declarations -Wzero-as-null-pointer-constant -Wnon-virtual-dtor
-Woverloaded-virtual -Wcast-align -Wfloat-conversion` plus the CMAKE-13 `-Werror=` list, gtest as
`-isystem`), every `bench/host` and `bench/tests` .cpp: 21 compiles, 0 warnings in bench files;
3 hits, all W0's `frame.h:61` `-Wshadow`. `-Wconversion -Wsign-conversion` over `bench/host`: 0
(after replacing three `x += cond ? 1 : 0`). Logs: `S/sp-build/bench-kit-gate/`.

Cross-compile (zig 0.13.0 from `S/sp-build/w3-platform-zig`, W3's macOS shim), bench host + the
receiver libraries it links (platform, sources, decode, protocol), `-O1` with the clang subset of
the gate: x86_64-linux-gnu, aarch64-linux-gnu, aarch64-macos, 117 compiles, 0 warnings, 0 errors;
`spectugatr_bench` linked for x86_64 and aarch64 Linux (ELF checked with `file`). Not run.
Log: `S/sp-build/bench-kit-xcheck/`.

Mutations (`S/sp-build/bench-kit-tools/mutate.sh`, one at a time on a copy, results
`S/sp-build/bench-kit-mut.results.txt`), 16/16 caught: RTT always late, no poll between stdin
writes, no relink after timeouts, no poll gap, no pad check, 0x00 in lines not counted, no COBS
strip, split threshold, pose seq skips not counted, no rate split, no 512 gap, no trailing-0x00
count, no same-time write dedupe (turnaround), whole-run minimum offset (first survived; the
drift test now runs slow and fast clocks), stdin ACK ignored, status line breaks kept.

PROS (arm-none-eabi 13.3.1, kernel files copied from `/c/GATR2-Electronics/brain/operaGATR`),
scratch mirror of `brain/spectuGATR`, `brain/gatr2_brain.mk`, `spectuGATR/protocol`,
`spectuGATR/bench/brain` with the two hooks applied (`S/sp-build/bench-kit-tools/pros_mirror.sh`):
```
make BENCH=1       -> exit 0, 24 units (19 spectugatr + 5 bench), src/main.cpp not built,
                      0 warnings, hot image 24.62 KB text
make (no BENCH)    -> exit 0, the example as W4 logged it: 18.91 KB text
make BENCH=1 GATR2_WARNFLAGS="<the host gate set minus -Wpedantic, plus the -Werror= list>"
                   -> exit 0; warnings only in PROS kernel headers and W0 frame.h:61;
                      0 in bench files
```
`vexSerialWriteFree` links through `libpros.a` (checked with `arm-none-eabi-nm`).

Probes (scratch, `S/sp-build/bench-kit-probe/`):
- `pose_probe.cpp`: the real W4 `Publisher` in fake time with the POSE budget, 5 Hz then 50 Hz,
  one `bench.*` channel; its SG1 lines as a bench log, then `spectugatr_bench analyze
  pose_probe.log --expect sg1`: segments 5 Hz (4.96 poses/s, gap p50 200 ms) and 50 Hz (50.05/s,
  gap p50 20 ms), 0 frames missed, `bench.rate` x2, the channel series, `spectu` counters.
- `b1_probe.py` (scratch only): a split B1 log; `analyze --expect lines` gives `B1 routing:
  split`, the per-encoding stdin answers and the Brain stdin line.

Format and text: clang-format 22.1.8 (`C:/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/
clang-format.exe`, repo `.clang-format`) `-i`, then `--dry-run --Werror` over every bench .h/.cpp:
exit 0. Every bench file plain ASCII (perl byte check) with LF endings. pros-cli source
(`S/spectu/pros-serial/pros-cli`) checked for `pros make BUILD_ARGS` and `pros upload --slot
--name --after`.

## Open issues

- Nothing ran on a Brain, controller, radio or Pi. The kit's pass lines are proposals to confirm
  with the first results.
- The W4 hook and the W6 bench page are not applied: `pros make BENCH=1` fails until the hook
  lands; B7, B8, B12 have only their host parts until the page lands.
- B7's Pi side waits for the Pi's SG1 split (Phase B). B8's Pico to Pi latency is not covered.
- The tool's POSIX serial path has never run (W3's pty tests have not run either); the first
  `spectugatr_bench ports` on the Pi exercises it.
- W5's fake controller answers neither stdin writes nor FILE_CTRL, so the tool's handling of real
  answers to those is untested; they are logged raw (`wa`, `to`), which is the B1 and B2 evidence.
- The controller's stdin encodings and the RADIO_STATUS layout are unconfirmed (vexide's reading);
  B1 run 2 and the B2 `radio` lines are where they get checked.
