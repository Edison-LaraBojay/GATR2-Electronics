# W6 Brain integrations: implementation log

2026-09-29. Workstream key `w6-integrations`. Worktree `S/wt-spectu` (detached at ae457ee), where
`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
Builds only in `S/sp-build/w6-integrations-*`. Nothing committed; new files are intent-to-add
(`git add --intent-to-add`, index only).

## What was built

Neutral hooks (empty by default, then nothing changes):

| Library | Change |
|---|---|
| investiGATR `path.h` | `PathState` {active, replaced, completed, cancelled, failed}, `PathContext` {state, note, frame, field_generation, reference, has_destination, destination}; `PathSink::reportPath(command, path, context)` (widened once) |
| actuGATR `motion.{h,cpp}` | D9 fix: `reported_` ("a path of this command went to the sink") tracked apart from `planned_`; `clearReportedPath(state, note)` from `start()` (kReplaced) and `finish()` (completed, cancelled, failed with `toString(reason)` as the note); `pathContext()` |
| actuGATR `path_tee.h` (new, header only) | `PathTee`: one PathSink feeding two |
| actuGATR `drive_requests.h`, `pros_drive.{h,cpp}` | `using DriveObserver = std::function<void(const DriveSnapshot&, Seconds)>`; `ProsDriveConfig::observer`, called each step after the snapshot is published, no lock held |
| communiGATR `client.{h,cpp}` | `ClientEvent`, `StateObserver`, `FieldObserver`, `ClientEventObserver`, `ClientObservers`; `ClientConfig::observers`. on_state at the end of `handleState`, on_field at the end of `publish`, events at HELLO Ok, Pi restart (before session loss), `loseSession`, `settlePlacement`, profile newly applied |
| communiGATR `link_driver.{h,cpp}` | `robotStatus(client, now, accept_configured_anchor)` extracted from `LinkDriver::robot()` (same checks, same order; robot() uses it); `reportPath` takes the context and ignores it |
| communiGATR `pros_usb_port.{h,cpp}` | `AuxLineSource`, `kAuxLineMax` (256); ctor takes the source; transmit loop: link frames, then at most one aux line, then any link frame queued meanwhile, then the 2 ms idle; the source gets the previous line's write time (`pros::micros`); `ProsUsbStats::aux_lines`, `aux_short` |
| communiGATR `pros_link.{h,cpp}` | `LinkConfig::usb_aux` passed to `ProsUsbPort`; `reportPath` forwards the context |

Integrations, `brain/spectuGATR/integrations/` (library `spectugatr_integrations`):

| File | Holds |
|---|---|
| `include/spectugatr/integrations/communigatr.h`, `src/communigatr.cpp` | `LinkFeed` (+ `LinkFeedConfig`): `observers()`; `onState` -> `updatePlacementRef` + `updatePose(PoseSample)` (status via `robotStatus`, `measured_at` = received_at - robot age, spread = round trip + `mapping_floor`), `poseUnavailable` only on a status change; `onField` -> `updateField` only when the map id changes (rebuilt from the Pi map records), then `updateFieldEstimate` (errors from nominal taken in mm/cdeg, age = Pi age + transit); `onEvent` -> `link.session_opened`, `link.session_lost` (plus pose unavailable, no source), `link.pi_restarted`, `link.placement`, `link.profile_applied`. `usbLane(publisher, budget)`: claims, returns an `AuxLineSource` (nextLine within cap and budget, `recordWrite` for the previous line only) |
| `include/.../actugatr.h`, `src/actugatr.cpp` | `PathFeed` (a PathSink): plan -> kActive document, end/replacement -> that state with no segments, target = context destination, reference, note. `DriveFeed` (+ `observer()`): channels `motion.state`, `motion.distance_error`, `motion.heading_error`, `motion.cross_track`, `drive.vx/vy/omega`, `drive.saturation`, `drive.rpm_1..N` sampled only on change; events `motion.state_change` (arg state), `motion.failed` (arg reason), `drive.fault` (arg fault) |
| `include/.../investigatr.h`, `src/investigatr.cpp` | `poseOf`, `segmentsOf` (turn direction as a sign) |
| `CMakeLists.txt`, `PIPELINE.md`, `tests/*_gtest.cpp` | library, explicit test list, pipeline doc |

Apps and config:
- `brain/robot/gatr2_robot.h`: `SpectuStdio {kOff, kRadio, kUsbCable}`, `kSpectuEnabled = false`,
  `kSpectuRobotName = "gatr2"`, `kSpectuStdio = kOff`, `kPathReportWithSpectu = false`.
- `brain/operaGATR/src/main.cpp`: `createViz()` (one function-local static Publisher on
  `prosClock`) in `initialize()` before `createLink`; `createLink` adds the `LinkFeed` observers and,
  on USB, `usbLane(kUsbRelayBudget)`; without USB `startStdio()` only when `kSpectuStdio != kOff`
  (radio budget or direct USB budget); `pathSink()`: ProsLink when off, PathFeed when on
  (PATH_REPORT stopped, D4 b), `PathTee(ProsLink, PathFeed)` with `kPathReportWithSpectu`;
  `DriveFeed` as `ProsDriveConfig::observer`; `publishTilt()` in the background task.
- `brain/locaGATR/src/main.cpp`: the same without Motion; `createViz()` in `initialize()` before the
  task that creates the link; tilt from the main loop.
- Docs: `docs/communigatr.md` (LinkConfig `usb_aux`, ClientConfig `observers`, reportPath
  signature, usb-tx row, new section "Observers and the aux lane"), `docs/actugatr.md` (path sink
  bullet, DriveObserver bullet), `docs/brain_setup.md` (settings row, section 11).

Tests added (21): actuGATR `motion_gtest` 5 (`PathReportsCarryTheirContext`,
`PathTeeFeedsBothSinks`, and the three D9 cases `CancelDuringReplanHaltClearsThePath`,
`RefusedReplanClearsThePath`, `NewCommandBeforeItsPlanClearsTheOldPath`); communiGATR
`client_observers_gtest` 3 (`SeeStatesFieldsAndEvents`, `ProfileAppliedOncePerSession`,
`LeaveTheExchangeUnchanged`: the same scripted exchange with and without observers gives the Pi
identical request op and id sequences); integrations 13 (`LinkFeed` 6 incl.
`DefinitionIsThePiMapByteForByte` = the 7.2 field_id round trip for 0, 1, 17, 128 objects,
`EstimateKeepsThePiPosesAndAges` incl. a heading wrap; `UsbLane` 3: claim once, cap and budget,
write times; `PathFeed` 3 incl. Motion + replacement during a replan halt and behind a tee;
`DriveFeed` 1).

## Decisions and deviations (with reasons)

1. **investiGATR `path.h` edited** (not in the W6 list): `PathSink` lives there, and 5.4 says
   widen it once. Callers updated: LinkDriver, ProsLink, the actuGATR test sink, 5 call sites in
   `link_driver_gtest.cpp` (pass `{}`). No default argument on the virtual.
2. **`PathContext` adds `has_destination` + `destination`** beyond 5.4's {state, note, frame,
   field generation, reference}: the path document's target is Motion's resolved destination,
   which no sink can see otherwise. `field_generation` is "last planned or checked on" (Motion
   updates `plan_generation_` when revalidating). The publisher resolves the path's field_id
   from its own field slot (W4), so the integration does not use frame or field_generation.
3. **No `kCleared` in `investigatr::PathState`**: Motion never clears a path without ending or
   replacing its command.
4. **`StateObserver` takes `const Client&`**, not `const StateSample&`: the pose status needs the
   profile and placement state held by the Client. `robotStatus()` was extracted so the viewer's
   status is exactly `LinkDriver::robot()`'s (existing LinkDriver status tests cover it; the
   observer test also compares both).
5. **New `ClientEvent` enum**, not the app-level `LinkEvent` (that one is derived by apps from
   snapshots and means other things). kProfileApplied fires on the transition only (the state
   repeats every 20 ms).
6. **Integration layout**: one header and source per integrated library,
   `integrations/include/spectugatr/integrations/{communigatr,actugatr,investigatr}.h`, instead of
   the sketch's `integrations/communigatr/usb_lane.h`: the include prefix must not collide with
   communiGATR's own `communigatr/`. Namespaces are the proposal's
   `spectugatr::integrations::{communigatr, actugatr, investigatr}`; inside them the GATR types
   are reached through `::`-qualified `using` aliases.
7. **Classes where there is state**: `LinkFeed` (event ids, last status, last map id, reusable
   field buffers), `DriveFeed` (ids, last values), `PathFeed` (a PathSink with a reusable
   update); `usbLane` is a free function whose lambda owns its bucket and line buffer.
8. **Drive channels only on change** ("actual new states only"; a still robot costs no bytes).
   The event is `motion.state_change`, not `motion.state`: channels and events share one DEF
   name space and the channel already has that name.
9. **Unavailable pose only on a status change**; on session loss the last pose goes out as no
   source. A link timeout without session loss just stops new poses (the receiver's stale rule).
10. **Estimate age** = Pi age + (completed_at - snapshot_after), as `LinkDriver::field`; unknown
    snapshot time -> not supplied (65535 ms on the wire). Invalid estimates publish no error.
11. **`mapping_floor` = 13 ms**, the upper end of the host probe's 3-13 ms (a wider interval is
    the honest side); placeholder until B8.
12. **Tilt** is published with `kNotSupplied`: the publisher stamps the call, which is when the
    IMU is read (`RobotAttitude` carries no stamp).
13. **`kSpectuEnabled` defaults to false**: the apps are unchanged by default (nothing ran on a
    Brain, B7 unmeasured, PATH_REPORT stops when on). One constant turns it on. The Publisher is
    constructed only when on (disabled apps do not allocate its ~69 KB).
14. **`kSpectuStdio` is an enum** (`kOff`, `kRadio`, `kUsbCable`): the budget depends on who
    reads the console. Default `kOff` per D17; the doc row says why.
15. **D4(b) in the apps**: PATH_REPORT (the op and `ProsLink::reportPath`) is kept; operaGATR
    picks Motion's sink: ProsLink when spectuGATR is off, PathFeed when on,
    `PathTee(ProsLink, PathFeed)` with `kPathReportWithSpectu`. TELEMETRY unchanged (D11 a).
16. **Files outside the W6 list, needed to build**: `brain/gatr2_brain.mk` (W4's: entries
    `spectugatr_communigatr` and `spectugatr_actugatr` plus the integrations include dir),
    `brain/spectuGATR/CMakeLists.txt` (W4's: the `add_subdirectory(integrations)` W4 anticipated,
    guarded by `TARGET communigatr AND TARGET actugatr` so the publisher still builds alone),
    `brain/locaGATR/Makefile` and `brain/operaGATR/Makefile` (`GATR2_BRAIN_LIBS`; W4 left these to
    the coordinator), `brain/communiGATR/CMakeLists.txt` (the new test file). Two mk entries, not
    one, so locaGATR (no actuGATR) compiles only the link integration.
17. **Tests in `brain/spectuGATR/integrations/tests/`**, one level below `brain/spectuGATR/tests/`
    (W4's): integrations is its own library with its own CMake and PIPELINE.md. Each test file
    has its own small decode helper (tests/ holds test mains only).
18. **clang-format 22.1.8** (pip, `C:/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/clang-format.exe`;
    `/c/msys64/ucrt64/bin/clang-format` does not exist): new files fully formatted and
    `--dry-run --Werror` clean. Existing files are not clang-clean in the repo (W0 note 18), so
    only my hunks were formatted (`clang-format-diff`), and only where the result matches the
    rest of the file (motion.cpp, motion_gtest.cpp one hunk, link_driver.cpp). In motion.h,
    pros_usb_port.h, pros_link.cpp and the test RecordingSink a partial format would mix access
    specifier and constructor styles; those lines keep the file's style.
19. **ProsUsbPort's aux lane is PROS only**: compiled in both apps with 0 warnings and reviewed,
    not host-tested. The lane's own logic (claim, cap, budget, write times) is host-tested
    through `usbLane`. The Brain-side transmit model belongs to the 7.2 e2e rig extension.
20. **kReplaced and latest-wins**: when the new command plans in the same drive step, its kActive
    replaces the kReplaced in the publisher's path slot before a pull (W4 decision 8); the viewer
    then sees the new path replace the old. kReplaced itself goes out when the new command waits.
    `MotionReplacementDuringHaltEndsTheOldPath` pulls between the two, as the transport does.
21. **RS-485 with spectuGATR on and `kSpectuStdio = kOff`**: the feeds run but nothing owns the
    publisher, so nothing is sent (latest-wins slots, bounded FIFO).

## Commands and results (Git Bash, `export PATH=/c/msys64/ucrt64/bin:$PATH`)

```
# baseline, before any W6 edit (W4 files present)
cmake -S S/wt-spectu/brain -B S/sp-build/w6-integrations-base -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src
cmake --build S/sp-build/w6-integrations-base -j16     -> exit 0, 0 warnings
ctest --test-dir S/sp-build/w6-integrations-base -j16   -> 100% passed, 506/506

# with W6 (same flags, -B S/sp-build/w6-integrations-host), final tree
cmake --build ... -j16                                  -> exit 0, 0 warnings
ctest --test-dir S/sp-build/w6-integrations-host -j16 --output-on-failure
                                                        -> 100% passed, 527/527 (506 + 21)
spectugatr_integrations_tests.exe --gtest_shuffle --gtest_repeat=3   -> 13 passed, 3 times
communigatr_tests.exe --gtest_filter=ClientObservers.* --gtest_repeat=3 -> 3 passed, 3 times

# Release (-B S/sp-build/w6-integrations-release -DCMAKE_BUILD_TYPE=Release)
cmake --build ...   -> exit 0; warnings only the known ones: investiGATR planner.cpp
                       -Warray-bounds (existing code), protocol doc_assembler.cpp
                       -Wmaybe-uninitialized (W0 review S3); none from W6 files
ctest               -> 527/527 passed

# naviGATR, this worktree's pi/naviGATR (compiles communiGATR, investiGATR, actuGATR sources)
cmake -S S/wt-spectu/pi/naviGATR -B S/sp-build/w6-integrations-navi (same flags)
cmake --build ... -j16   -> exit 0, 0 warnings
ctest ... -j16           -> 100% passed, 626/626
(rebuilt after the last whitespace edits: link_driver.cpp, motion.cpp recompiled, 0 warnings,
 626/626 again)

# D9 mutation check: motion.cpp with the old planned_ gate put back, then restored (cmp ok)
actugatr_tests --gtest_filter=Motion.*  -> 26 passed, 3 failed: exactly
  CancelDuringReplanHaltClearsThePath, RefusedReplanClearsThePath,
  NewCommandBeforeItsPlanClearsTheOldPath
spectugatr_integrations_tests           -> 12 passed, 1 failed:
  PathFeed.MotionReplacementDuringHaltEndsTheOldPath

# PROS (arm-none-eabi 13.3.1, PROS 4.2.2 kernel files copied from /c/GATR2-Electronics)
TC=/c/Users/mbes1/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr
export PROS_TOOLCHAIN="$(cygpath -w $TC)"; cd <app>; make clean; PATH="$TC/bin:$PATH" make -j16
locaGATR   -> exit 0, 46 units, 0 warnings, hot image 46.56 KB text (spectuGATR off)
operaGATR  -> exit 0, 61 units, 0 warnings, hot image 77.98 KB text (spectuGATR off)
brain/spectuGATR/examples/standalone (W4's, rechecked after the mk edit)
           -> exit 0, 20 units, 0 warnings, 18.91 KB (as W4 logged)
with kSpectuEnabled = true (temporary edit, restored):
locaGATR   -> exit 0, 0 warnings, 68.65 KB;  operaGATR -> exit 0, 0 warnings, 103.34 KB
make clean in all three; kernel copies (firmware/, include/pros/) removed again

# text
clang-format 22.1.8 --dry-run --Werror on every new .h/.cpp -> clean
non-printable or non-ASCII bytes in added lines of every touched file: 0; LF endings
```

## Open issues

- Nothing ran on a Brain, Pi, controller or radio. The aux lane's Brain-side cost (B7), the
  mapping floor (B8), stdio with no reader (B5) are unmeasured.
- Until W7's SG1 split in `pros_usb_link`, the Pi drops SG1 lines as console text; they share
  its per-cycle read budget (the lane caps bursts at 512 characters).
- The 7.2 e2e rig extension (Brain tx model with the aux lane) and the B-series bench programs
  and sheets (7.1 "Bench kit") were not part of this task.
- `brain/README.md` and `brain/robot/README.md` do not mention spectuGATR (in no Phase A list).
- Existing C++ files are not clang-format clean repo-wide (a coding-standard question).
