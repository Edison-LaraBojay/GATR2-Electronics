# Phase C integration notes (lead)

## Open items found during Phase B

1. **Planner start and goal next to walls or landmarks.**
   - The circle model (R = enclosing radius + clearance) refuses any start whose origin is within R of the boundary or of a box. VEX robots start touching the wall, so avoiding mode would refuse the first command of a match.
   - Proposed fix (investiGATR, a follow-up to Worker C):
     - **Start escape.** When the circle start is blocked but the exact footprint at the start heading is clear, try straight escapes along the current heading (forward, and backward if reverse is allowed). Check the swept exact rectangle; holonomic drives may also try a sideways escape. The first escape that reaches a circle-free point becomes a leading translate segment.
     - **Goal approach.** The same idea, mirrored: approach the goal along its final heading from a circle-free point, with the swept exact rectangle checked.
     - Report kStartBlocked or kGoalBlocked only when no escape exists.
   - Tests: start against each wall; goal beside a landmark; escape blocked by a second box.
2. **brain_link_e2e.** v4 test in pi/navigatr/tests that links the real communiGATR core (client, link driver) against the real Pi System over memory links. Worker B adds the CMake entries.
3. **Apps.** Shared robot description in `brain/robot/gatr2_robot.h` (the include dir is already in gatr2_brain.mk); testing and localization-test rewrites.
4. **gatr2_brain.mk communigatr list** from Worker A's progress, section PROS SOURCES.
5. **docs:** docs/actugatr.md, docs/README.md, root README, brain/README.md.

6. communiGATR CMake: list the lead files wheel_calibration, startup_placement (A3 told) and link_events (not told; add in Phase C) with their tests.

## Review of lead code (agent a7d57187126d8a213), 2026-09-27 21:xx, all handled
1. ProsDrive busy reads looked idle -> redesign: DriveRequests mailbox (actuGATR/src/drive_requests.cpp, 5 tests), requests latest-wins, ids from ProsDrive via Motion::setNextId, status(DriveSnapshot&) bool, pending goTo shows kWaiting. Drive task is the only owner user; no lock during step.
2. gatr2_brain.mk communigatr list: still open (after A3), plus lead files wheel_calibration, startup_placement, link_events.
3. WheelCalibration rotation bias: constructor takes TrackingWheel, lever removed; max_rotation 0.5 deg (lib default and app). Test AllowedRotationIsRemovedFromTheWheelTravel.
4. Lock across path sink: gone with the mailbox (only the drive task calls Motion).
5. Replaced command path clear: Motion::start clears. Test ReplacedCommandClearsItsPath.
6. Turn drift vs translation tolerances: turn uses hypot(position, tracking). Test TurnAcceptsWhatAFinishedTranslationLeaves.
7. LinkEvents: last nonzero session/pi, last valid state, calibrating entry from any waiting state. Tests SessionGapHidesNothing, CalibratingEntersThroughAnyWaitingState.
8. testing checkConfig (ports, drivetrain, follower, motion, hypot(tol) < clearance); ~ProsDrive quits the task cooperatively (bounded) then stops the drive; stale disable flag replaced by sequence ordering.
Harness: $S/lead-harness3 -> $S/lead-build3: 115/115 (actugatr + integration + calibration + link_events). ARM syntax ok for pros_drive, drive_requests, motion, follower, wheel_calibration, link_events.

## Phase C progress (lead), 2026-09-27 ~21:50
- gatr2_brain.mk communigatr list = A3 PROS SOURCES (+ lead files); actugatr += drive_requests. localization-test usb_bench_driver.{h,cpp} deleted (tracked in git).
- PROS builds (toolchain make.exe directly; `pros make` from Git Bash cannot find the toolchain and wrote a stray `~/PROS` dir, removed): localization-test and testing both link, 0 warnings. Command: cd brain/<app>; PROS_TOOLCHAIN=$P/pros-toolchain-windows/usr PATH=$P/pros-toolchain-windows/usr/bin:$PATH make.exe -j16 (P = %APPDATA%/Code/User/globalStorage/sigbots.pros/install).
- Apps: busy ProsLinkStatus keeps the last view (no false link-lost events); startup readiness from A's Readiness (sensorsReady / calibrating incl. CalibrationFailed); kStartupWaitSeconds 90 (cold Pi boot) in both apps.
- Start pose placeholder moved to (1.2, 1.8, 0): (0.6, 0.6) is a closed corner pocket for avoiding with the 0.46 m footprint + 0.06 clearance. testing: kDirectGoal (1.2, 2.3, 90), kAvoidGoal (2.4, 1.8, 0), landmark RedGoal2West (object 6) + (0.45, 0, 180).
- v4 e2e test pi/navigatr/tests/brain_link_e2e_gtest.cpp (rewritten; real Client/LinkDriver/NG1 codec/Motion/Drive/GeometricPlanner vs System from brain_profile_usb.xml). Lead harness $S/lead-e2e -> $S/lead-e2e-build: 6/8 pass. Failing, both B4 (spec 8.8): WheelReadingsReportRawTravel (Pi READ_WHEELS NotReady), PulledCableStopsTheMoveAndTheLinkResumes (bench model drops wheel travel over a VEX IMU gap: 0.18 m error, still Ready).
- TODO after B's pipeline: add target brain_link_e2e_tests to pi/navigatr/tests/CMakeLists.txt (sources as in $S/lead-e2e/CMakeLists.txt, NAVIGATR_CONFIG_DIR); docs/brain_setup.md section 8 cable row per B4 outcome.
- Deployment order (D1): new Pico firmware sends only v2 frames; old Pi drops them. Pi first, then flash Pico. Added to brain_setup.md section 2; put in final report.
- A FIX applied in apps (23:xx): wheel capture via WheelTicket/wheels(ticket); VEX recal via VexImuRecalibration in both apps (testing refuses tests and zeroes sticks meanwhile); gatr2_brain.mk += vex_imu_recalibration.cpp; both PROS builds clean.
- USER RULE 23:20 (spec 8.10): any used sensor stale > 250 ms or identity change -> pose invalid, place again; unused Pico IMU never matters. e2e updated (Brain restart and still pulls now expect NeedsPlacement; Pico drop case; 0.1 s blip keeps the pose). brain_setup.md section 8 rewritten. TODO: docs/communigatr.md lines ~643 and ~706 (A's, branch done) still say the Pi holds the pose / keeps the placement across a Brain restart -> lead fixes after B4 lands. Also e2e Brain restart assumes the idempotent profile path (no upload).
- 23:30 user: keep the viewer additions (5.5) in B5.
- 23:38: e2e 9/9 pass against B4 in-progress tree (READ_WHEELS + 8.10 sensor loss working). Run needs /c/msys64/ucrt64/bin on PATH (else silent DLL failure).
- 00:25 pre-gate (independent of B5), fresh lead dirs, 0 warnings everywhere:
  brain (repo brain/CMakeLists): 407/407; common/tests: 221/221; pico/tests: 110/110; pio run -e hat2_bno08x -e hat2_asm330: both SUCCESS (RAM 16368/10920 B, flash 94328/86272 B); PROS localization-test + testing link clean (last 23:2x). Pi navigatr + e2e: after B5/fix.
- 01:30 B5 done (511/511). Lead: gatr2_robot::kSendProfile switch (false = XML-configured Pi works with the apps; calibration apply refused); Y/CONTROL 3/4 'pose holds' text fixed in brain_setup.md and common/frames.h comments; both PROS apps rebuilt clean. B review running.
- 01:55 opcontrol bug (reported by user): A/X/Y goTo replaced by the same-cycle manual(zero) from a stale snapshot (my mailbox refactor regression). Fix: actugatr::moving/sendManual(demand, requested_this_cycle, snapshot) + fresh status read before the manual decision in testing opcontrol. Tests: DriveRequests.CenteredSticksNeverCancelANewCommand, SticksTakeOverAndIdleSendsZero, ZeroDemandAfterARequestReplacesIt. brain 410/410, 0 warnings; PROS testing links clean.
- 02:55 FINAL GATE (fresh lead dirs, 0 warnings): brain 410/410; common 221/221; pi/navigatr 532/532 incl. brain_link_e2e_tests 9 (target added to pi/navigatr/tests/CMakeLists.txt); pico 110/110; pio both SUCCESS; PROS both apps link. Workflow wf_bd73f72e-259 completed (15 agents, 0 errors).
