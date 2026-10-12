# W5-core receiver: implementation log

Workstream key `w5-core`. Owns `spectuGATR/host/{sources,decode,combine,capture}/**`,
`spectuGATR/tests/host/**`, and appends to `spectuGATR/host/CMakeLists.txt`. Worktree `S/wt-spectu`,
index only (`git add`), never committed.

`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`

## Status

- **Done and host-tested on Windows 11** (2026-09-29).
  - Debug: 259/259 (106 protocol + 34 platform + 119 host). 0 warnings.
  - Release: 259/259. 1 warning, W0's `doc_assembler.cpp:46` (W0 review S3), none in W5 files.
  - Host suite shuffled: Debug x5 and Release x3, 119/119 each time.
  - UBSan traps + `_GLIBCXX_ASSERTIONS`: 119/119.
  - 30 mutations, all caught.
- **Linux and macOS: cross-compiled only** (zig, clang 18), never run.
- **No hardware claims.** Every source test runs against fakes in fake time. No VEX device was
  attached; nothing here says how a real controller or Brain behaves (bench B1-B7, B9).
- Not built, per IMPL_A: the navigatr WebSocket source (D19 default c2), the HTTP server, main.

### Resume note

- A first run (session cut off at about 12:23) wrote `sources/` (source, vex_port, brain_usb, cdc2,
  controller, sim fakes), `decode/decoder.*`, `combine/clock_mapping.*` and `combine/combiner.h`.
  It left no combiner.cpp, capture, tests, CMake lines or log.
- Snapshot of those files: `S/sp-build/w5-core-orig/`.
- This run reviewed them against PROPOSAL 4.1, 5.3, 5.6, 5.8, 7.1 W5 and disc:controller-host s2,
  s9, reworked them (Deviations 1, 2, 5-8), and wrote everything else.
  - Kept unchanged: `vex_port.*`, `fake_port.*`, `clock_mapping.cpp`.

## What exists

Libraries (include root `spectuGATR/`, `#include "host/decode/decoder.h"`), one per stage:

| Target | Namespace | Holds | Links |
|---|---|---|---|
| `spectugatr_sources` | `spectugatr::host::sources` | `chunk.h` (`ReceiverUs`, `SourceKind`, `SourceChunk`), `source.h` (`Port`, `PortOpener`, `SourceHooks`, `SourceStatus`, `TransportStats`, `RadioStatus`), `vex_port` (real opener: role + identity, or `--device`), `brain_usb` (`runBrainUsbSource`), `cdc2` (requests, `CdcReplyScanner`, parsers), `controller` (`runControllerSource`) | platform |
| `spectugatr_decode` | `...::decode` | `Decoder`: `LineScanner` -> `decodeFrame` -> frame_seq per session -> `DocAssembler` -> `decodeDocument`; `FrameItem`, `DocumentItem`, `DecodeStats` | protocol only |
| `spectugatr_combine` | `...::combine` | `OneWayMinOffset`; `Combiner` (`receive`, `portStatus`, `tick`, `lateJoin`, `primaryBrain`); `coreScene` | decode, sources |
| `spectugatr_capture` | `...::capture` | `CaptureRecorder` (rolling log, triggers, finished captures), `exportCapture` (bundle files), `tables` (CSV columns and rows), `JsonText` | combine |
| `spectugatr_sources_sim` (tests only) | `...::sources::sim` | `FakeClock`, `ScriptedPort`, `FakeController`, `FakeBrainStdio`, `brain_records.h` builders | sources, protocol |

Docs: `PIPELINE.md` in each of `sources/`, `decode/`, `combine/`, `capture/`.

Wiring for the Phase B main (what `tests/host/receiver_gtest.cpp` does):

```text
source thread: runBrainUsbSource | runControllerSource (hooks.on_chunk, hooks.on_status)
combine thread: recorder.record(chunk); items = combiner.receive(chunk)
                faults in items -> recorder.fault(fault); recorder.tick(rx_us)
                recorder.record(status); combiner.portStatus(status)
                every ~100 ms: combiner.tick(now); recorder.tick(now)
server: items -> view/1 (SourceUpdate = source, ForwardedRecord = record messages,
        ForwardedDocument = doc); new client -> combiner.lateJoin(now)
capture: CaptureRecorder(config, [&]{ return combiner.lateJoin(now); }); exportCapture -> zip
```

Tests (`spectugatr_host_tests`, 119, explicit list in `tests/host/CMakeLists.txt`, TIMEOUT 60):
Cdc2 12, ControllerSource 15, BrainUsbSource 6, Fakes 10, Decoder 15, ClockMapping 6, Combiner 18,
CoreScene 12, CaptureRecorder 12, Bundle 11, Receiver 2 (end to end through both sources).

TEST-10 fault patterns covered: source stops (stale, lost, last pose kept with age), identity change
mid-flow (session restart clears state, clock and partial documents; a new session's chunks never
complete an old document), no input (idle relink, liveness), stalled consumer (log byte bound held
with 5000 pushes), id wrap (frame_seq, publisher ms clock), enum names complete, overflow reports
itself (evictions, capture byte limit, CSV bound, all counted), upstream garbage (CDC2 resync, bad
lines, random bytes), field absent (no field, no estimate for another field, empty cells), late
replies after a timeout.

## Deviations and decisions (with reasons)

Reworked from the first run:
1. **The combiner owns one `Decoder` per source; `Combiner::receive(SourceChunk)`.** The first run's
   `receive(src, items, stats, rx_us)` let a caller pass stats read before the push; my own test
   harness did exactly that through argument evaluation order (found by a failing test). Owning the
   decoder also lets a port reopen drop the partial line (`resetTransport` on a change into kOpen,
   not on the controller's repeated kOpen stats reports). decode stays a standalone library.
2. **`sources/chunk.h` split from `source.h`** so decode needs no platform headers; one CMake
   library per stage, so the Pi adapter (W7) links `spectugatr_decode` and the protocol only.
3. **Controller: late USER_READ replies are delivered, in FIFO order, during any exchange.** The
   first version counted a reply that did not answer the current request as bad and dropped it,
   losing Brain bytes after a timeout (found when mutations m14 and m16 survived; tests
   `LateReplyKeepsTheFifoOrder`, `ALateUserReadDuringARadioQueryStillDelivers`). The fake's reads
   now return every ready reply, as a USB read returns what is buffered.
4. **Controller: RADIO_STATUS every 2 s while linked** (5.3 "Radio link quality ... at 0.2-1 Hz").
   The payload layout is vexide's reverse engineering, unverified (B2). A NACK or no reply leaves
   `radio` absent; a missing radio reply never counts toward the relink.
5. **Controller: 2 s without Brain bytes sends SYSTEM_VERSION again**, so a dropped radio link shows
   as `kNoBrainLinked` at the port, not only through liveness. The publisher's 1 s liveness frame
   keeps a live Brain from idling that long. Status reports are sent only on a change of state or
   detail, plus stats at most once a second.
6. **CDC2 scanner skips replies nobody asked for** (a command other than SYSTEM_VERSION or 0x56),
   so a stray `AA 55` cannot swallow up to 255 following bytes.
7. **brain_usb counts reads of a 512 B multiple, and such a read followed by a timeout** (3.5 B7:
   the host hold-back of exact-512 B transfers).

Receiver rules (5.6) where the proposal left room:
8. **Viewer rules 2, 3, 6, 7, 11 as C++ `coreScene()`**, as the task asks. The proposal gives them
   to the viewer (W8); this is the reference behavior the viewer must match, and W7 can use it.
   `PiLink` (pi_instance, link session) is an input though Phase A has no Pi source (D19 c2).
9. **A newer major is refused only after 3 such frames in a row with no good frame between**, and
   the next good frame clears it. W0 review S1: a single corrupted version digit decodes as "newer".
10. **One Brain per robot name** (rule 8): the later-accepted source is refused, with the reason.
    Re-checked on every SESSION, so a refused source takes over once the first is not delivering
    (lost). An empty robot name is a name.
11. **Rule 12 dedupe is against the kind's current document** (and a DEF's current content), not
    every id ever seen. Going back to an earlier field (A, B, A) forwards A again, so a viewer that
    joined late never lacks the current field. VIEW.md says "an id not seen before"; W0 should
    write this reading into VIEW.md. The export dedupes the same way.
12. **Pose freshness without a declared period** (SESSION can be up to a 10 s carousel away on the
    radio) uses 2.5x the recent interval between new, not held, poses; the 250 ms floor stays.
13. **Clock mapping resets on a new session** (a new publisher clock origin). `uncertainty_us` is
    1000 (ms resolution); one_way_min ages are lower bounds, and a pose's mapped time is clamped to
    its receipt.
14. **Faults:** brain_lost (live or stale to lost, not for a refused source), session_restart,
    gap_burst (5 frame_seq gaps in 2 s), loss_spike (a `spectu.*.dropped` channel up by 5 between
    samples). Names from W4's counter channels.

Capture (5.8):
15. **The log holds `LogEntry` = SourceChunk | SourceStatus | SourceFault**, so port states and
    faults are the loss records next to frame_seq gaps. An entry costs its bytes plus 64.
16. **State from before the window**: `Combiner::lateJoin` at the trigger (a `StateSource` callback,
    called only when a capture starts). Items received before the window become `before_window`
    rows and resolve names and the SESSION for later rows of that session; items received inside
    it come from the log. Without it a radio capture would often lack DEF names and the field.
17. **Export returns `BundleFiles` {name, data}** (the shape of naviGATR's `ZipEntry`). The zip and
    the HTTP route wait for W2's extraction (Phase B), as do its `csvEscape` and `JsonWriter`: this
    run has a local `csvField` and a small `JsonText` (API-6: swap them for the extracted ones
    then). JSON escapes bytes >= 0x7F as `\u00XX`, since wire text is bytes, not always UTF-8.
18. **Bundle layout:** `metadata.json` first; `<src>_{session,pose,def,samples,event,estimate,path,
    path_segments,names,loss}.csv`; `field_revisions.csv`; `fields/<FIELD_ID hex8>@<rev>.csv`
    (object rows, CSV rather than JSON). Every row starts with rx_us, src, session, sent_ms,
    frame_seq, placement, field_id, field_revision, before_window. Port rows only on a change.
    metadata.json: schema, wire and view versions, the capture and its bounds, config, units and
    frames (5.2), sources[] {in_window stats, publisher_counters, segments with a clock mapping per
    session, at_trigger state and since-start stats}, defs, files.
19. **Automatic triggers default to all four faults**, cooldown 120 s, 6 an hour, newest 3 kept
    (naviGATR's capture numbers). Phase B's main can narrow them.

Layout and tests:
20. **Fakes live in `host/sources/sim/`** (TEST-4: a sim folder beside the component; `tests/`
    keeps only `*_gtest.cpp`), including `brain_records.h`, shared by 6 test files. The fakes have
    their own tests (TEST-5), one against the W0 golden COBS packet.
21. **`host/PIPELINE.md` not written**: it is outside W5-core's files. Phase B's receiver doc
    should list the four stage docs under `## Inside`.
22. **Not done here:** SYS_STATUS identity through the controller (B9; `--identity` is not in phase
    1); view/1 JSON (server, Phase B).

## Open issues

- **W0:** `doc_assembler.cpp:46` Release warning (S3) still present; `frame.h:61` `kNoSession`
  `-Wshadow` shows in every file that includes frame.h under the gate flags (m4).
- **W0, VIEW.md:** `source.stats` has no fields for `TransportStats` extras (radio status,
  late_replies, reads_of_512, holdback_suspect); adding them keeps `/1`. Record Deviation 11's
  dedupe reading.
- **Phase B:** the main's thread wiring (one combine thread fed by the source threads through a
  bounded queue), the view/1 writer, the zip, `--capture-dir`, and swapping `JsonText`/`csvField`
  for W2's extracted writer.
- **Bench:** real controller replies (B1-B6), the RADIO_STATUS layout, whether USER_READ is answered
  with no Brain linked, NUL padding (B3), the 512 B hold-back (B7), device identities (B9).
- **Brain (455) and naviGATR (626) suites not rerun by W5-core:** no file they compile changed
  (nothing outside `spectuGATR/` references `spectuGATR/host` or these targets; grep checked).

## Verification: exact commands and results (Git Bash)

```text
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake -S S/wt-spectu/spectuGATR -B S/sp-build/w5-core-host -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug \
      -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src           -> exit 0
cmake --build S/sp-build/w5-core-host -j16 --clean-first                             -> exit 0, 0 warnings
ctest --test-dir S/sp-build/w5-core-host -j16 --output-on-failure                     -> 259/259 passed
(same, -DCMAKE_BUILD_TYPE=Release, S/sp-build/w5-core-release)                        -> 259/259 passed;
                                                  1 warning, W0 doc_assembler.cpp:46 -Wmaybe-uninitialized
S/sp-build/w5-core-host/tests/host/spectugatr_host_tests.exe --gtest_shuffle --gtest_repeat=5
                                                                                     -> 5 x PASSED 119
S/sp-build/w5-core-release/tests/host/spectugatr_host_tests.exe --gtest_shuffle --gtest_repeat=3
                                                                                     -> 3 x PASSED 119
cmake ... -B S/sp-build/w5-core-ubsan "-DCMAKE_CXX_FLAGS=-fsanitize=undefined -fsanitize-trap=undefined
      -D_GLIBCXX_ASSERTIONS"; build spectugatr_host_tests; run                        -> PASSED 119
```
Logs: `S/sp-build/w5-core-host.{cfg,build,final.build,final.test}.log`,
`S/sp-build/w5-core-release.{cfg,final.build,final.test}.log`, `S/sp-build/w5-core-ubsan.*.log`.

Warning gate (GCC 15.2, `-O0` and `-O2`, the draft standard's host gate set plus the CMAKE-13
`-Werror=` list, gtest as `-isystem`), every W5 .cpp including tests, 58 compiles,
`S/sp-build/w5-core-gate/gcc.log`: 0 warnings in W5 files; 30 `-Wshadow` hits, all W0's
`frame.h:61`. `-Wconversion -Wsign-conversion` over the library sources: 0.

Cross-compile (zig 0.13.0, from W3's `S/sp-build/w3-platform-zig/`), x86_64-linux-gnu,
aarch64-linux-gnu, aarch64-macos, every library and sim .cpp, `-O1 -Wall -Wextra -Wshadow -Wundef
-Wold-style-cast -Wextra-semi -Wzero-as-null-pointer-constant` plus `-Werror=` return-type, switch,
uninitialized: 54 compiles, 0 warnings, 0 errors (`S/sp-build/w5-core-xcheck/xcheck.log`). Not run.

Mutations (TEST-12), one at a time on a copy in `S/sp-build/w5-core-mut/` (`mutate.sh`, logs per
mutation), each caught by at least one test: no assembler reset on a new session, no missed-frame
count, no clock reset on restart, repeated DEF forwarded, repeated document forwarded, no robot
refusal, stale at the lost threshold, transport reset on every kOpen report, age from receipt,
field shown for any definition id, Pi always combined, placement ignored, relink one timeout late,
any extended reply accepted (m14), only the first reply delivered (m16), late replies dropped,
no CDC2 crc check, no pre-window copy, no cooldown, context rows inside the window, export
without document dedupe, port rows on every status, hold-back uncounted, clock window never
trimmed, newer major accepted, no loss spike, no gap burst, declared pose period ignored, CSV
never quoted, JSON never escaped. m14 and m16 first survived; Deviation 3 fixed the defect they
exposed and added the two tests that now catch them.

Format and text: `clang-format` 22.1.8 (`C:/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/
clang-format.exe`) `--style=file -i`, then `--dry-run --Werror` over every W5 .h/.cpp: exit 0.
Every W5 file is plain ASCII with LF endings (`file`).

git (worktree index only): `git add spectuGATR/host/{sources,decode,combine,capture}
spectuGATR/tests/host spectuGATR/host/CMakeLists.txt`; 53 paths staged as `A`. No commit.
