# Fix round 1 (verify-review-r1 findings 1-6): log

2026-09-29. Workstream key `fix-r1`. Worktree `S/wt-spectu` (detached at ae457ee), where
`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
Builds, probes, mutations and the rig only in `S/sp-build/fix-r1-*`. `C:/GATR2-Electronics` read
only (PROS kernel files copied out). `S/wt-navi` not touched. Nothing committed; the three new
files are intent-to-add (`git add --intent-to-add`), nothing else was staged.

## Status

All six findings fixed or built, host-tested only. Nothing ran on a Brain, controller, radio or Pi.

| Suite | Result |
|---|---|
| spectuGATR host, Debug (clean rebuild) and Release (fresh) | 263/263 both (259 + 4 new); 0 warnings in both |
| brain host, Debug (clean rebuild) and Release (fresh) | 536/536 both (527 + 9 new); Debug 0 warnings; Release only the existing `investiGATR/src/planner.cpp:206` -Warray-bounds |
| naviGATR, this worktree's old layout | 626/626, 0 warnings |
| PROS locaGATR, operaGATR, spectuGATR off and on | 4 builds, exit 0, 0 warnings |
| review probes (`probe.exe`, `probe2.exe`) rebuilt against the fix | 3/3 pass (all 3 failed before) |
| mutations (combine 6, transmit loop 5) | 11/11 caught |
| 7.2 e2e rig, 600 runs | built and run; results under Finding 6 |

## Finding 1: a same-robot takeover now moves the core scene (rule 3)

- `Combiner::primaryBrain()`: the first accepted used source still owns the core scene, and keeps
  it while stale. Once it is not delivering (lost), a used, delivering source with the same known
  robot name takes over (earliest accepted among them). Unknown or different names never take
  over, so `ThePrimaryBrainIsKeptWhileLost` (two robots) is unchanged.
- When the first Brain comes back it owns the scene again: it is still the earliest accepted, and
  rule 8 refuses the other on that source's next frame.
- New test `CoreScene.TheSameRobotThroughAnotherSourceTakesOverFromALostBrain`: the probe's
  scenario (USB first, controller refused, USB port closed, controller frame): scene on the
  controller, live, its pose (x 3000); USB back: scene on USB with its new pose.
- Helpers `used()` and `acceptedBefore()` (anonymous namespace) replace the inline checks.
- Docs: `combiner.h` and `core_scene.h` comments; `host/combine/PIPELINE.md` rules.

## Finding 2: a second Brain's records no longer go out before its refusal (rule 8)

- `frame()` takes the frame's SESSION first (`keepSession`), then reviews refusal, then forwards
  records. The publisher's order (POSE ahead of SESSION) no longer leaks. `record()` no longer
  reviews on SESSION. When the review clears a refusal, the replay already carried this frame's
  SESSION, so the loop skips it (no duplicate).
- A later source is held while its own robot name, or the earlier delivering source's, is not
  known yet. It is refused with `"waiting for robot names, <src> already delivers"` and
  re-reviewed on each of its frames. So a receiver that joins mid-stream no longer forwards a
  duplicate source until its SESSION carousel repeat.
- `refusalFor` treats a source not yet accepted as later than every accepted one.
  `accepted_at_us` is set only after the review, so a refused source's acceptance time is when it
  was first used, not its first frame. Before, a refused source got `accepted_at_us` at its first
  frame; that value only fed the order used here.
- `reviewRefusal` now returns "the refusal changed" (it still emits the update and replays when
  cleared). `frame()` emits its state update only when the review did not.
- Tests:
  - `ASecondBrainForTheSameRobotIsRefused` now builds `[POSE, SESSION]`, the publisher's order;
  - new `ALaterSourceIsHeldUntilItsRobotNameIsKnown`: pose before SESSION is held with the reason;
    SESSION "chomp" clears it and replays SESSION then the held pose; SESSION "gatr" turns it
    into the second-Brain refusal;
  - new `ALaterSourceIsHeldUntilTheEarlierOneNamesItsRobot`;
  - new `ALoneSourceWithoutARobotNameIsUsed`: a single source is never held.
- Docs: `host/combine/PIPELINE.md` rule 8 text; `protocol/VIEW.md` `refused` (a later Brain whose
  name is not known yet; a refusal can clear and the state is sent again).

Deviation, with reason: while held, SAMPLES and EVENT records are not forwarded, the same as for
any refused source. SESSION, DEFs, documents and the latest pose are kept and replayed when it is
used, and every chunk is still in the capture log. The alternative (a buffer of held records)
would add a bounded queue for a case that lasts at most one SESSION carousel (2 s USB, 10 s
radio), and only when two Brains feed one receiver.

## Finding 3: `doc_assembler.cpp` optimized-build warning (W0 review S3)

- Took the W0 review's tested fix (`S/sp-build/w0-review-warn/da_fix3.cpp`): `Partials` is a plain
  `std::array<Partial, kDocKindCount>`, `total == 0` means empty; `reset()` assigns an empty array.
  No behavior change.
- `g++ -std=c++17 {-O0,-O1,-O2,-O3,-Os} -Wall -Wextra -Werror=uninitialized
  -Werror=maybe-uninitialized -c doc_assembler.cpp`: rc 0 and no warning at every level (was an
  error at -O2).
- Release builds: spectuGATR 0 warnings (was 1); brain only the existing planner.cpp one.

## Findings 4 and 5: PROTOCOL.md versioning (W0 review S1, S2)

- S1: the streak rule is written down; no code change. Versioning now says:
  - a v1 reader drops a newer frame with `kNewerVersion` and counts it;
  - the version is read before the crc, so one corrupted version digit also reads as newer;
  - a receiver refuses the source, with "publisher speaks vN, this receiver reads v1", only after
    3 newer frames in a row with no good v1 frame between, and the next good frame clears it.
- Deviation, with reason: I chose the streak rule over freezing a crc check ahead of the version.
  - W0's decode order reads byte 0 first so that a newer major may change sizes and the crc; the
    test `Frame.NewerVersionIsRefusedAndNamed` decodes a 3-byte v2 frame.
  - Freezing the crc trailer would constrain every later version.
  - The receiver side already exists and is tested (W5 deviation 9,
    `ANewerMajorIsRefusedAfterAStreakAndUsedAfterAGoodFrame`).
- S2: "Frozen for every version: the line format, byte 0, and byte 1 bit 0 as the durable flag";
  new flag bits are 1 to 7.
- Same sentence family, W0 review m2: the Line section's "hex characters 6 and 7 after the marker"
  now reads "hex digits 2 and 3 after `SG1:` (`kFlagsHexOffset`, 6 from the start of the marker)".
- Version history: a dated sub-bullet, "no wire change".

## Finding 6: the aux lane has a host test, and the 7.2 e2e rig exists

### (a) The transmit loop is portable and host-tested

- New `brain/communiGATR/include/communigatr/usb_transmit.h` and `src/usb_transmit.cpp`:
  - `UsbTransmitter` holds the one-slot link queue, the aux source, the last aux write time and
    the counters;
  - `loop(io)` writes the link frames waiting, then at most one aux line, then any link frame
    queued meanwhile;
  - `UsbTransmitIo` (write, millis, micros, stopRequested) is the transmit task's stream and clock.
  - `UsbFrame`, `FrameQueue`, `kAuxLineMax` and `AuxLineSource` moved there from
    `pros_usb_port.{h,cpp}`. `pros_usb_port.h` includes it, so every existing include keeps
    working.
- `ProsUsbPort::Io` implements `UsbTransmitIo` with `::write`, `pros::millis`, `pros::micros` and
  its `stopping` flag. The task runs `tx.loop(*this); pros::delay(2)`.
  - Same order, same expiry, one write per line, same counters (`ProsUsbStats` reads them from the
    transmitter), and the stop check stays per link frame.
  - The two line buffers (517 B) moved from the transmit task's stack into the heap-allocated `Io`.
  - Hot image: locaGATR 46.56 to 47.08 KB, operaGATR 77.98 to 78.49 KB (spectuGATR off).
- The integration header `spectugatr/integrations/communigatr.h` now includes `usb_transmit.h`
  (the portable types it uses) instead of `pros_usb_port.h`.
- Build lists: `brain/communiGATR/CMakeLists.txt` (library and test list), `brain/gatr2_brain.mk`
  (communigatr entry). naviGATR compiles none of these files; no change there.
- Tests `communiGATR/tests/usb_transmit_gtest.cpp` (9), against a recording stream whose write
  hook stands in for the poll task:
  - `WithoutAnAuxSourceALoopWritesJustTheLinkFrames` (behavior with spectuGATR unused);
  - `LinkFramesGoFirstThenOneAuxLineThenFramesQueuedMeanwhile`;
  - `AtMostOneAuxLinePerLoop`;
  - `TheSourceGetsThePreviousLineWriteTime` (0 after a loop with no line);
  - `AnExpiredFrameIsDroppedUnwritten`;
  - `ShortWritesAreCountedPerLane`;
  - `ALineOverTheCapIsNotWritten`;
  - `OneFrameWaitsAtATime`;
  - `AStopWritesNoMoreLinkFrames`.
- Docs `docs/communigatr.md`: layer table row, a line in "Observers and the aux lane", test table
  rows for `usb_transmit_gtest.cpp` and the missing `client_observers_gtest.cpp` (W6's file).

### (b) The 7.2 e2e rig extension (scratch, not repo code)

`S/sp-build/fix-r1-e2e/`: `src/aux_e2e.cpp`, `CMakeLists.txt` (worktree sources by path, fresh
`navigatr_core` from this worktree's pi/naviGATR), `aggregate.py`, `sweep_j500.csv`,
`sweep_j500.md`.

- Starting point: `design/sim/sg1_sweep.cpp`. It runs the real Client, LinkDriver, NG1 codec,
  Motion, planner and naviGATR System (`brain_profile_usb.xml`); the Pico and Brain links are
  memory links; one simulated clock with a 100 us tick. The RS-485 wire and the synthetic aux
  generator were dropped.
- The aux source is now the apps' own: a `spectugatr::Publisher` on the sim clock, `LinkFeed`
  observers on the Client, and `usbLane(publisher, budget)`. There are 8 rig channels (16 for
  x4) sampled every control tick, so the samples lane is busy too.
- The Brain transmit model (my [inference] from disc:pros-serial s1; nothing is measured):
  - the Client's `write` queues into the real `UsbTransmitter`;
  - the transmit task runs `loop()` and then waits 2 ms after the loop ends;
  - a write blocks until the whole line fits the 2047 B PROS buffer, while every other actor keeps
    running;
  - every 2 ms the daemon moves min(buffered, free) into a 2048 B VEXos FIFO;
  - the FIFO sends one character per `char_us`, then the old fixed line delay applies (usb2, usb3,
    usb5);
  - the Pi reads the SG1 lines as console text (no W7 split yet).
- Sweep: 3 lines x char time {0, 5, 10, 20, 40} us x TELEMETRY {off, 100 ms} x aux {off,
  relay4k = `kUsbRelayBudget` (the apps), direct8k = `kDirectUsbBudget`, x4_16k = 16000 B/s,
  2048 burst, pose 100 Hz, samples 50 Hz, carousel 0.5 s} x Pi phase {0, 400, 800, 1200, 1600}
  us. Jitter 0.5 ms, 3 s settle, 20 s window: 600 runs, 1 min 52 s wall, 0 errors.
- Acceptance per cell (mean of 5 phases, against aux off in the same cell): polls/s and poll-gap
  p95 within 1%, and no new timeouts.

Change vs aux off, polls/s % / poll-gap p95 %, TELEMETRY off , 100 ms:

| budget | char us | usb2 | usb3 | usb5 |
|---|---|---|---|---|
| relay4k (apps) | 0-10 | 0/0 , 0/0 | 0/0 , 0/0 | 0/0 , 0/0 |
| relay4k (apps) | 20 | 0/0 , 0/0 | 0/0 , 0/0 | -0.2/0 , -0.3/0 |
| relay4k (apps) | 40 | -0.1/0 , -0.1/0 | -0.4/0 , -0.8/0 | **-1.4/0 , -2.0/0** |
| direct8k | 20 | -0.1/0 , -0.2/0 | 0/0 , -0.1/0 | -0.1/0 , **-1.2/+0.9** |
| direct8k | 40 | -0.1/0 , -0.3/0 | -0.9/0 , **-3.2/+4.8** | **-4.1/+16.9 , -8.3/+3.6** |
| x4_16k | 20 | 0/0 , 0/0 | 0/0 , 0/0 | -0.1/0 , **-1.9/0** |
| x4_16k | 40 | **-1.0/+4.0 , -4.8/+18.0** | **-2.4/+38.0 , -5.6/+4.8** | **-6.8/+16.9 , -7.7/0** |

- All 600 runs: 0 timeouts, 0 blocked writes (the 2047 B buffer never filled), 0 expired link
  frames, 0 full-slot writes, 0 `spectu.slow_writes`.
- The cost is head-of-line delay in the character pipe. For example, usb5 at 40 us the link egress
  (client write to the last character leaving the Brain) has max 10.7 ms with relay4k, against
  2.7 ms with aux off.
- Verdict:
  - the apps' budget passes in every cell up to 20 us per character (50 kB/s), and on usb2 and
    usb3 up to 40 us;
  - it fails only at 40 us on usb5, where polls/s drops by 1.4% (TELEMETRY off) and 2.0%
    (100 ms), with p95 and timeouts unchanged;
  - 13 of 90 on-cells fail; 11 of them are the direct8k and x4 sensitivity budgets. direct8k is
    meant for a Brain with no Pi link on USB.
- Rig sanity: with aux off and 0 us the rig gives 50.00 polls/s (TELEMETRY off) and 43.25
  (100 ms). The design rig without a Brain model gave 50.0 and 43.93.

What this does not settle, for Phase B validation and the bench:
- the real V5 character time and flush behavior (B7). The model's 2 ms flush and FIFO sizes come
  from PROS 4.2.2 code reading;
- the Pi side after W7's SG1 split: rerun this rig then, because the Pi read path changes;
- the 7.2 latprobe rerun with the SG1 lane before any latency step.

## Decisions and deviations (summary)

1. Takeover only by the same known robot name, only once the first is not delivering (Finding 1).
2. Hold instead of buffer for a source whose robot is not known yet; its samples and events of the
   hold window are not forwarded (Finding 2).
3. The acceptance time of a refused source is when it is first used (Finding 2).
4. Streak rule documented; no crc-first decode (Finding 4).
5. The transmit loop was extracted into communiGATR `src/` instead of adding a PROS-only test
   hook. The rig runs the same code (Finding 6a).
6. The e2e rig is built now, in Phase A scratch. Its W7-dependent rerun is recorded as Phase B
   (Finding 6b).
7. Files outside a single workstream's list, as the task allows: W0 (`doc_assembler.*`,
   PROTOCOL.md, VIEW.md), W5 (combine, tests/host), W6 (communiGATR port and CMake, the
   integration header, docs/communigatr.md), W4's `gatr2_brain.mk` (one line).
8. clang-format 22.1.8 (pip; `/c/msys64/ucrt64/bin/clang-format` does not exist):
   - new files and the W0/W5 files: fully formatted, `--dry-run --Werror` clean;
   - `pros_usb_port.cpp`: my hunks clean with `clang-format-diff`. The file as a whole was not
     clang-clean before (W6 note 18), and `pros_usb_port.h` is left in its file style.

## Still open from verify-review-r1 (minor, not in this task)

- 7: no test feeds real Publisher output through the host Decoder and Combiner (it would have
  caught Finding 2's record order).
- 8: code comments cite PROPOSAL sections; `spectuGATR/PIPELINE.md` Status is stale;
  `host/PIPELINE.md` is Phase B.
- 9: `FrameError::kNoSession` shadows `protocol::kNoSession` under -Wshadow.
- 10: `usbLane()` claims the publisher even on RS-485.
- 11: `parseUserRead` does not check the channel byte.
- 12: EventQueue retry drop bound.

## Commands and results (Git Bash, `export PATH=/c/msys64/ucrt64/bin:$PATH`)

```
# host builds (G = S/nav-base/_deps/googletest-src), keys fix-r1-{spectu,brain,navi}
cmake -S S/wt-spectu/<spectuGATR|brain|pi/naviGATR> -B S/sp-build/fix-r1-<key> -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$G
cmake --build S/sp-build/fix-r1-spectu -j16 --clean-first    -> exit 0, 0 warnings
ctest --test-dir S/sp-build/fix-r1-spectu -j16 --output-on-failure -> 263/263 passed
cmake --build S/sp-build/fix-r1-brain -j16 --clean-first     -> exit 0, 0 warnings
ctest --test-dir S/sp-build/fix-r1-brain -j16 --output-on-failure  -> 536/536 passed
cmake --build S/sp-build/fix-r1-navi -j8                     -> exit 0, 0 warnings
ctest --test-dir S/sp-build/fix-r1-navi -j8 --output-on-failure    -> 626/626 passed
(same, -DCMAKE_BUILD_TYPE=Release, fresh dirs fix-r1-spectu-rel, fix-r1-brain-rel)
  spectu -> exit 0, 0 warnings, 263/263 passed
  brain  -> exit 0, warnings only investiGATR/src/planner.cpp:206 -Warray-bounds (existing), 536/536
Logs: S/sp-build/fix-r1-{spectu,brain,navi}{,-rel}.*.log

# CMAKE-13 gate, GCC 15.2, -O2 -Wall -Wextra plus -Werror= return-type, switch,
# implicit-fallthrough, format, uninitialized, maybe-uninitialized, delete-non-virtual-dtor,
# return-local-addr, mismatched-new-delete, shift-count-negative, shift-count-overflow, div-by-zero
combiner.cpp, core_scene.cpp, doc_assembler.cpp, combiner_gtest.cpp, core_scene_gtest.cpp,
usb_transmit.cpp, usb_transmit_gtest.cpp                     -> rc 0, 0 warnings each
usb_transmit.cpp with -Wshadow                               -> 0 warnings

# review probes, rebuilt against fix-r1-spectu (S/sp-build/fix-r1-probe)
probe.exe  -> Probe.SameRobotTakeoverMovesTheCoreScene OK ("primaryBrain = controller,
              scene.status = live, pose.x_mm = 3000"); SecondBrainPosesBeforeItsSessionAreForwarded OK (0)
probe2.exe -> Probe2.PoseAheadOfSessionInOneFrameLeaks OK (0 records)

# mutations (scratch copies S/sp-build/fix-r1-mut{,-brain}; mutate.sh), each caught:
m1 no takeover -> TheSameRobotThroughAnotherSourceTakesOverFromALostBrain
m2 keepSession removed -> 4 CoreScene/Receiver tests
m2b SESSION reviewed in frame order (the old code) -> ASecondBrainForTheSameRobotIsRefused,
    ALaterSourceIsHeldUntilItsRobotNameIsKnown
m3 no hold -> both ALaterSourceIsHeld... tests
m4 SESSION forwarded again after the replay -> ALaterSourceIsHeldUntilItsRobotNameIsKnown
m5 a never-accepted source not treated as later -> 3 tests
t1 no second link pass, t2 aux first -> LinkFramesGoFirstThenOneAuxLineThenFramesQueuedMeanwhile
t3 write time not passed -> TheSourceGetsThePreviousLineWriteTime
t4 two aux lines per loop -> 4 tests;  t5 no expiry -> AnExpiredFrameIsDroppedUnwritten
restored copies -> 123/123 host, 9/9 UsbTransmit

# PROS (arm-none-eabi 13.3.1), scratch mirror S/sp-build/fix-r1-pros (worktree brain/, translaGATR/,
# spectuGATR/protocol plus firmware/ and include/pros copied from /c/GATR2-Electronics/brain/<app>)
bash S/sp-build/fix-r1-pros.sh  (make clean; PATH="$TC/bin:$PATH" make -j16 per app)
locaGATR  off -> exit 0, 0 warnings, hot 47.08 KB;  on -> exit 0, 0 warnings, 69.17 KB
operaGATR off -> exit 0, 0 warnings, hot 78.49 KB;  on -> exit 0, 0 warnings, 103.85 KB
(on = kSpectuEnabled true in the mirror's gatr2_robot.h only). Logs S/sp-build/fix-r1-logs/

# e2e rig
cmake -S S/sp-build/fix-r1-e2e -B S/sp-build/fix-r1-e2e/build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release ...
cmake --build S/sp-build/fix-r1-e2e/build -j16 --target aux_e2e
  -> exit 0; warnings only the existing planner.cpp:206 -Warray-bounds
./aux_e2e.exe sweep 500 > ../sweep_j500.csv                  -> 600 rows, 0 ERROR, 1 min 52 s
python3 ../aggregate.py ../sweep_j500.csv > ../sweep_j500.md

# text
clang-format 22.1.8 --dry-run --Werror: every new or W0/W5 file touched clean; pros_usb_port.cpp
  hunks clean through clang-format-diff
byte scan of the 19 touched files: no bytes outside tab, LF, 0x20-0x7E; no CR
git add --intent-to-add brain/communiGATR/{include/communigatr/usb_transmit.h,src/usb_transmit.cpp,
  tests/usb_transmit_gtest.cpp}
```

## Files touched

- spectuGATR:
  - `host/combine/combiner.{h,cpp}`, `host/combine/core_scene.h`, `host/combine/PIPELINE.md`;
  - `tests/host/combiner_gtest.cpp`, `tests/host/core_scene_gtest.cpp`;
  - `protocol/src/doc_assembler.cpp`, `protocol/include/spectugatr/protocol/doc_assembler.h`;
  - `docs/PROTOCOL.md`, `protocol/VIEW.md`.
- brain:
  - new `communiGATR/include/communigatr/usb_transmit.h`, `communiGATR/src/usb_transmit.cpp`,
    `communiGATR/tests/usb_transmit_gtest.cpp`;
  - `communiGATR/include/communigatr/pros_usb_port.h`, `communiGATR/pros/pros_usb_port.cpp`,
    `communiGATR/CMakeLists.txt`, `gatr2_brain.mk`;
  - `spectuGATR/integrations/include/spectugatr/integrations/communigatr.h`.
- docs: `docs/communigatr.md`.
