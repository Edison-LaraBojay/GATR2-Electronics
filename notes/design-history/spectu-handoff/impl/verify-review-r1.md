# Verify review, round 1 (adversarial): log

2026-09-29. Workstream key `verify-review-r1`. Read-only on the worktree `S/wt-spectu`: nothing
there was edited, staged or committed. Builds and probes only in `S/sp-build/verify-review-r1-*`,
where `S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
`C:/GATR2-Electronics` read only (kernel files copied out, git diff/show only). `S/wt-navi` not
touched.

Scope: all of Phase A (W0, W3, W4, W5-core, W6) against IMPL_A.md and PROPOSAL 5.1-5.8, 7.1,
7.2, checks (a) to (g) of the task.

## Verdict

pass = false. No blocking defect in the Brain side; two should-fix defects in the receiver's
combine rules (found and reproduced by scratch probes), three W0 review items still open (one of
them a new optimized-build warning that fails under the approved `-Werror=` list), and one
validation gap (7.2 aux-lane e2e). Details under Findings.

## What was checked, and how

| Check | Method | Result |
|---|---|---|
| (a) publisher call paths | read publisher.cpp, slot.h, event_queue.*, next_line.cpp, take_slots.cpp, carousel.cpp, transport.cpp, stdio_transport.cpp, the three integrations, pros_usb_port.cpp, pros_drive.cpp | producer calls copy into slots or the bounded FIFO; no lock, no I/O, no allocation after init (the integrations reserve at init; paths over 33 segments allocate in the drive task, documented, and the planner there already allocates). Observers run under the link mutex but only copy. DriveObserver runs with no lock held. Event FIFO: bounded CAS loops, stale-claim break; the documented residual (a writer stalled over 50 ms can tear one event) stands |
| (b) behavior with spectu unused | `git diff HEAD` of communiGATR, actuGATR, investiGATR, apps; old vs new ProsUsbPort transmit loop by hand | empty hooks change nothing: the transmit loop is equivalent with no aux source; `robotStatus` keeps LinkDriver's order; ProsDrive gets the same config. The only unconditional change is the approved D9 fix. Aux lane: one write per line (no split), link frames drained before and after the one aux line, lines <= 249 chars, 4000 B/s and 512 burst |
| (b) drift vs the user's tree | `git log`/`git diff ae457ee HEAD` in C:/GATR2-Electronics; `git merge-file` into scratch | user tree is 3 commits plus uncommitted edits past ae457ee; 5 files overlap Phase A (drive_requests.h, operaGATR main.cpp, gatr2_robot.h, docs/actugatr.md, docs/brain_setup.md); all 5 merge with 0 conflicts. Counts (455 brain tests) are for ae457ee, not the user's HEAD |
| (c) D9 | scratch copy with the ae457ee `planned_` gate put back into motion.cpp, built `actugatr_tests` and `spectugatr_integrations_tests` | the 3 D9 tests fail (A cancel during halt, B refused replan, C new command during halt) plus `PathFeed.MotionReplacementDuringHaltEndsTheOldPath`; all pass on the real tree. Logic reviewed: `reported_` set only when a sink got a plan, cleared once in start() (kReplaced, old command id) and finish() |
| (d) encoder/decoder | read publisher_gtest.cpp | every record kind (SESSION, POSE, DEF, SAMPLES, EVENT, DOC_CHUNK) and the three documents the Brain makes go through `nextLine` into the protocol decoder the host wraps; names documents are never produced. No test feeds real Publisher output into the host `Decoder`/`Combiner` (minor) |
| (e) combine rules | read combiner.*, core_scene.*, decoder.*; scratch probes | rules 1, 2, 3, 4, 5, 6, 7, 10, 11, 12 and liveness match 5.6; no Pi pose path exists. Rule 8 has two defects (Findings 1, 2) |
| (f) CDC2 | cdc2.* vs discovery/controller-host.md s2, s2.1 | request and reply layouts, VarU16, CRC-16/XMODEM BE, residue check, SYSTEM_VERSION product/flags bytes, USER_READ data offset (ecmd, ack, channel), RADIO_STATUS field order all match; tests pin the pros-cli request bytes |
| (g) style | read headers and PIPELINE.md files; clang-format; byte scan | namespaced (only PROS/host mains global), aliases, struct for data, classes where there is state; explicit test lists; clang-format 22.1.8 dry-run clean on every new .h/.cpp; all 234 changed files plain ASCII, LF. Minor doc items (Finding 8) |

## Findings

### Should-fix

1. **Rule 8 takeover leaves the core scene on the lost Brain.** `Combiner::primaryBrain()` picks the
   smallest `accepted_at_us` among non-refused sources, lost ones included
   (`spectuGATR/host/combine/combiner.cpp:175-190`). When Brain A (brain_usb) is lost and the refused
   Brain B of the same robot (controller) is accepted again, the combiner forwards B's records
   (W5 deviation 10), but B's `accepted_at_us` is its first frame, later than A's, so
   `primaryBrain()` and `coreScene()` stay on A: a dimmed, lost pose while the same robot is live
   on B. This contradicts rule 3 ("recovery snaps back") and deviation 10. The existing test
   `CoreScene.ThePrimaryBrainIsKeptWhileLost` uses two different robot names, so it does not see
   this. Probe `S/sp-build/verify-review-r1-probe/probe_gtest.cpp`: prints
   `primaryBrain = brain_usb, scene.status = lost, pose.x_mm = 1000` after the controller took
   over. Fix idea: among sources of the primary's robot name, prefer a delivering one.
2. **A second Brain's records are forwarded before its refusal.** `refusalFor` needs the source's
   robot name, which only SESSION carries, and `record()` forwards records in frame order
   (`combiner.cpp:228-256`). The publisher packs POSE ahead of SESSION (`next_line.cpp:417-418`),
   so the second Brain's first frame `[POSE, SESSION]` forwards its pose before the SESSION refuses
   it (probe `probe2_gtest.cpp`: 1 record forwarded from a source that ends the call refused).
   When the receiver joins mid-stream, SESSION arrives only on the carousel (2 s USB, 10 s radio),
   so every record of a duplicate source is forwarded until then. The unit test
   `ASecondBrainForTheSameRobotIsRefused` builds `[SESSION, POSE]`, the opposite of the real order.
   Fix idea: decide refusal per frame before forwarding (scan the frame for SESSION first), and
   hold records of a source with no robot name while another delivering source exists.
3. **W0 review S3 still open: new warning in optimized builds.** GCC 15.2 `-O2` and `-O3` warn
   `-Wmaybe-uninitialized` through `spectuGATR/protocol/src/doc_assembler.cpp:46`
   (`std::optional<Partial>`). IMPL_A says zero new warnings; with the approved CMAKE-13
   `-Werror=uninitialized` the file fails to compile at `-O2` (checked:
   `g++ -O2 -Wall -Wextra -Werror=uninitialized -c doc_assembler.cpp` -> error, rc 1). W0 review
   gave a tested fix (a plain array with `total == 0` as empty). Reported by W0 review, W4, W5
   and W6; nobody owned the fix.
4. **W0 review S1 still open: PROTOCOL.md and the receiver disagree on newer versions.**
   `docs/PROTOCOL.md` Versioning says a v1 reader refuses a newer frame; W5 refuses only after 3
   such frames in a row (`CombineConfig::refuse_newer_after`, deviation 9) because one corrupted
   hex digit reads as a newer version (29 of 1,260 in the W0 review). Either freeze a CRC check
   ahead of the version test, or write the streak rule into PROTOCOL.md (W0 file).
5. **W0 review S2 still open: the durable bit is not frozen.** PROTOCOL.md freezes only the line
   format and byte 0, yet the Pi relay (W7) reads byte 1 bit 0 without decoding. One line in the
   Versioning section.
6. **7.2 aux-lane e2e not run; the aux-lane loop has no host test.** The ProsUsbPort transmit
   ordering (link frames, one aux line, link frames) is PROS only (W6 deviation 19) and the 7.2
   e2e rig extension (Brain tx model: link lane first, capped aux lane, 2047 B FIFO; acceptance
   polls/s and poll-gap p95 within 1% of off, 0 new timeouts) does not exist (no change under
   `pi/`, W6 open issues). If this belongs to Phase B validation, record it there; the acceptance
   gate for (b) is otherwise unmeasured.

### Minor

7. No test runs real `Publisher` output through the host `Decoder` and `Combiner` (the protocol
   round trip exists; a cross-library one would have caught Finding 2's record order).
8. Docs and comments:
   - code comments cite out-of-repo documents (`PROPOSAL 5.6`, `disc:controller-host s2`):
     combiner.h:2, core_scene.h:2, recorder.h:2, cdc2.h:3-4, controller.h:3, cdc2_gtest.cpp:2,
     combine/PIPELINE.md:20. A repo reader cannot follow them; cite PROTOCOL.md/VIEW.md or state
     the rule;
   - `spectuGATR/PIPELINE.md` Status still says "designed ... protocol/ is host-tested";
   - `spectuGATR/host/PIPELINE.md` does not exist yet (W5 deviation 21, Phase B).
9. W0 review m4: `frame.h:61` `FrameError::kNoSession` shadows `protocol::kNoSession` under
   `-Wshadow` (reproduced).
10. `usbLane()` claims the Publisher even if the LinkConfig is RS-485 (the port then ignores
    `usb_aux`), which would lock out `StdioTransport`. The apps only call it with `kUseUsb`; a
    one-line note in communigatr.h would do.
11. `parseUserRead` does not check the channel byte (payload[2] == 1).
12. EventQueue: when the ring is full and the owner is preempted between its dequeue CAS and the
    seq release, a producer's retry loop can drop up to 7 more old events before giving up.
    Bounded and rare.

### Checked and fine

- Publisher memory fixed at construction; producer calls never wait; one owner per Publisher;
  budgets and caps as 5.3; D17 honored (stdio only when `kSpectuStdio != kOff`, only without the
  USB link).
- Apps: one producer task per slot (pose, placement, field, estimate from the poll task; path
  and drive channels from the drive task; tilt from the app/background task). Motion is only
  driven from the drive task.
- Hooks: `LeaveTheExchangeUnchanged` compares request sequences with and without observers.
- CDC2 source never writes to the Brain; replies scanned with bounded buffers.

## Commands and results (Git Bash, `export PATH=/c/msys64/ucrt64/bin:$PATH`)

```
# host, Debug, from the worktree
cmake -S S/wt-spectu/brain -B S/sp-build/verify-review-r1-brain -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src
cmake --build ... -j8                      -> exit 0, 0 warnings
ctest ... -j8 --output-on-failure          -> 527/527 passed (455 + 51 W4 + 21 W6)
(same for S/wt-spectu/spectuGATR -> S/sp-build/verify-review-r1-spectu)
                                           -> exit 0, 0 warnings; 259/259 passed
(same for S/wt-spectu/pi/naviGATR -> S/sp-build/verify-review-r1-navi)
                                           -> exit 0, 0 warnings; 626/626 passed

# D9 on the old gate (scratch copy S/sp-build/verify-review-r1-d9src, motion.cpp gated on planned_)
cmake --build S/sp-build/verify-review-r1-d9 --target actugatr_tests spectugatr_integrations_tests
actugatr_tests.exe                -> 94 passed, 3 failed: Motion.CancelDuringReplanHaltClearsThePath,
                                     Motion.RefusedReplanClearsThePath,
                                     Motion.NewCommandBeforeItsPlanClearsTheOldPath
spectugatr_integrations_tests.exe -> 12 passed, 1 failed: PathFeed.MotionReplacementDuringHaltEndsTheOldPath

# combine probes (S/sp-build/verify-review-r1-probe, linked to the Debug host libraries)
probe.exe  -> Probe.SameRobotTakeoverMovesTheCoreScene FAILED (primaryBrain = brain_usb, lost)
              Probe.SecondBrainPosesBeforeItsSessionAreForwarded FAILED (1 record forwarded)
probe2.exe -> Probe2.PoseAheadOfSessionInOneFrameLeaks FAILED (1 record forwarded, source refused)

# W0 items
g++ -std=c++17 -O2|-O3 -Wall -Wextra -c protocol/src/doc_assembler.cpp -> 1 -Wmaybe-uninitialized each
g++ -std=c++17 -O2 -Wall -Wextra -Werror=uninitialized -c doc_assembler.cpp -> error, rc 1
g++ -std=c++17 -O2 -Wall -Wextra -Wshadow -c protocol/src/frame.cpp -> frame.h:61 -Wshadow

# PROS (arm-none-eabi 13.3.1), scratch mirror S/sp-build/verify-review-r1-pros (worktree copy plus
# kernel files copied from /c/GATR2-Electronics/brain/<app>)
locaGATR  make -j8 -> exit 0, 0 warnings;  operaGATR make -j8 -> exit 0, 0 warnings
kSpectuEnabled = true in the mirror's gatr2_robot.h, make clean, make:
locaGATR  -> exit 0, 0 warnings;  operaGATR -> exit 0, 0 warnings, 103.34 KB hot text

# drift vs the user's tree (merge into scratch only)
git merge-file -p <user file> <ae457ee file> <worktree file> for the 5 overlapping files -> 0 conflicts

# text
clang-format 22.1.8 (pip) --style=file --dry-run --Werror on every new .h/.cpp -> 0 failures
byte scan of all 234 changed files: no bytes outside tab, LF, 0x20-0x7E; no CR
```

Release builds (`-DCMAKE_BUILD_TYPE=Release`, `S/sp-build/verify-review-r1-{brain,spectu}-rel`):

```
brain   build exit 0; warnings: investiGATR/src/planner.cpp:206 -Warray-bounds (existing code),
        protocol/src/doc_assembler.cpp:46 -Wmaybe-uninitialized (Finding 3); none elsewhere
        ctest -> 527/527 passed
spectu  build exit 0; warning: protocol/src/doc_assembler.cpp:46 -Wmaybe-uninitialized only
        ctest -> 259/259 passed
```

Nothing here ran on a Brain, controller, radio or Pi.
