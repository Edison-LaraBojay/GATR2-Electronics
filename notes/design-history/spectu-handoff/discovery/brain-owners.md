# Discovery: Brain-side ownership of spectuGATR data and integration points

Date 2026-09-28. Read-only on the repo. Basis tags: [code] file:line checked
in the working tree, [measured-host] run here, [source] external primary,
[inference], [unknown]. No hardware; nothing below is physical behavior.
Paths are relative to C:/GATR2-Electronics unless absolute.

Probe used for the [measured-host] items (scratch only):
- source: C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/spectu/brain-owners/src/probe.cpp
- build: cmake -S <scratch>/src -B <scratch>/build -G "MinGW Makefiles" (brain/ added as a subdir, current tree), cmake --build --target probe
- run: <scratch>/build/probe.exe (ucrt64 first on PATH)

## 0. Summary of ownership

| Data | Owner that first knows it | Task (operaGATR) | Consumers today |
|---|---|---|---|
| Pi pose (raw wire) | communigatr::Client::handleState | communigatr poll task, under link mutex | LinkDriver::robot -> Motion, app screens |
| Pose status/frame in SI | communigatr::LinkDriver::robot (computed on demand) | caller's task | Motion::update (drive task), display/readView (app tasks) |
| Field map+estimate (accepted) | Client::publish -> FieldPublication | poll task, under link mutex | LinkDriver::field -> Motion field_ copy |
| Field the planner used | actugatr::Motion::plan (field_, plan_generation_) | drive task | nobody outside Motion (private) |
| Plan accept/replace/clear/cancel | actugatr::Motion (start/plan/finish) | drive task | PathSink = ProsLink -> PATH_REPORT |
| Command id handout / pending goTo | actugatr::DriveRequests | app task (ProsDrive mutex) | snapshot() |
| Execution (MotionStatus, DriveStatus, mode) | Motion / Drive / DriveOwner | drive task | ProsDrive snapshot -> telemetryOf (app bg task) |
| Attitude (roll, pitch) | app-owned ProsVexImu::attitude | app task (pulled) | TELEMETRY -> Pi estimator tilt + viewer |
| Link lifecycle events | derived by app from ProsLinkStatus (LinkEvents) | app task | Brain screen only |

investiGATR owns none of the live data: it owns types, the StateSource/PathSink
interfaces and the planner (pure function). [code] brain/investiGATR/include/investigatr/state_source.h:1-4, path.h:43-49, planner.h:1-3

## 1. Current pose

### 1.1 Where it arrives
- GET_STATE reply handled in Client::handleState. [code] brain/communiGATR/src/client.cpp:616-645
  - stores StateSample{valid, pi_instance, session, state (BrainState), received_at, round_trip} [code] client.cpp:626-631, client.h:165-172
  - received_at = poll clock `now`; round_trip = write-return to reply. [code] client.cpp:351-353
- Runs inside Client::poll, called only by ProsLink::run in the "communigatr" task, priority TASK_PRIORITY_DEFAULT+1 (=9), period 2 ms, while holding the link mutex. [code] brain/communiGATR/pros/pros_link.cpp:145-166, 122-123; pros_link.h:44-46; brain/operaGATR/include/pros/rtos.h:63
- Poll period of GET_STATE: 20 ms. [code] client.h:49

### 1.2 What the wire pose carries (BrainState) [code] translaGATR/frames.h:353-369
| Field | Meaning |
|---|---|
| x_mm, y_mm i32; heading_cdeg i32 (-18000,18000] | field frame, CCW from +x |
| robot_flags | PoseValid, Localized, AgeKnown, AnchorCommand, AnchorConfigured (frames.h:137-143) |
| robot_age_ms u16 | Pi cycle time minus measurement time (Pi computes clampAgeMs(in.now - measuredAtHost)) [code] pi/naviGATR/src/impl/publishing/brain_link_publisher.cpp:287-290 |
| odometry_epoch, anchor_revision u32 | placement identity (low 32 bits) |
| map_id, estimate_id | field document identities |
| health, calibration, profile_* | status |

NOT on the wire: any measurement timestamp, any pose sequence number, any
Pi clock value. "The brain link sends ages, not timestamps." [code] pi/naviGATR/docs/coordinates.md:149-151

### 1.3 Identity / timing / validity as the Brain holds it
- Session: Pi-issued u32 at HELLO Ok; pi_instance random per Pi process. [code] client.cpp:446-448; frames.h:378
- Validity: LinkDriver::robot status order kNoLink, kNoProfile, kCalibrating, kUnplaced (incl. placement pending), kNoPose, kValid. [code] brain/communiGATR/src/link_driver.cpp:54-93; state_source.h:16-23
- Age at query: robot_age_ms/1000 + round_trip + (now - received_at). [code] link_driver.cpp:89
  - Derived Brain-clock measurement time: t_meas >= received_at - round_trip - robot_age_ms/1000 (so the age is an upper bound) [inference from link_driver.cpp:89 and docs/communigatr.md:498-500]. Caveat: the Pi fills the reply from the cycle's `in.now` (cycle start) [code] brain_link_publisher.cpp:415-418; if a cycle starts before the request arrives the bound can be short by up to one Pi cycle [inference, needs Pi-side check].
- Placement/frame identity: tuple (pi_instance, session, odometry_epoch, anchor_revision) mapped to a Brain-local FrameGeneration counter, remembering only 4 tuples. [code] link_driver.cpp:90-91, 215-229; link_driver.h:66-84
  - FrameGeneration is process-local and not stable across Brain reboots: a receiver cannot compare it with anything else; spectuGATR must carry the raw tuple (or its own placement id), not this number [inference].
- Placement confirmation: PlacementStatus kApplied only when a later state shows the anchor. [code] client.cpp:637-640
- Clock: ProsLink::now() = pros::micros()*1e-6 (s since PROS init). [code] pros_link.cpp:135-137. pros::millis() (used for telemetry stamp_ms) has the same documented epoch ("since PROS initialized"). [code] brain/operaGATR/include/pros/rtos.h:207-241
- Duplicate detection: none possible from the wire; a repeated Pi pose shows only as a larger age [inference from frames.h:353-369]. "Publish actual new states" needs either a pose sequence/measurement id added to GET_STATE (wire change) or a Brain heuristic (same x,y,h and t_meas estimate within round-trip jitter) [inference].

### 1.4 Who reads it
- Motion::update -> source_->robot(now) every drive step (10 ms). [code] brain/actuGATR/src/motion.cpp:388; pros_drive.h:27
  - through ProsLink::robot, which takes the link mutex with call_timeout_ms (20 ms) [code] pros_link.cpp:233-240; pros_link.h:46
- operaGATR display() (background task) [code] brain/operaGATR/src/main.cpp:235; locaGATR readView [code] brain/locaGATR/src/main.cpp:131
- Raw unplaced heading for wheel calibration: ProsLinkStatus.heading [code] pros_link.cpp:215-218

### 1.5 Pose from the user's own localization
- The only abstraction is investigatr::StateSource {robot(now), field(out)} [code] state_source.h:38-48. Host sims implement it (SimulatedState) [code] brain/actuGATR/sim/simulated_state.h:18-34. actuGATR Motion takes any StateSource [code] motion.h:140-141, 154.
- It is an investiGATR type: a standalone spectuGATR user must not need it. The integration adapter (optional) can publish from StateSource, but the core API takes plain values [inference].

## 2. Field acquisition

### 2.1 Transfer and assembly
- Map read when state.map_id != 0 and != cached map id. [code] client.cpp:1255-1258, 991-1005
- Estimate read when map current, estimate_id new, and field_period (0.5 s) passed since the last read started. [code] client.cpp:1260-1276, 1006-1020; client.h:59
- DocAssembly: same kind/id, repeated total_len+crc, contiguous, crc checked; partial never readable. [code] brain/communiGATR/include/communigatr/doc_assembly.h:1-6; src/doc_assembly.cpp:38-69
- Map complete -> map_cache_ (validated, crc == map_id), estimate in progress dropped. NOT published yet. [code] client.cpp:673-687
- Estimate complete + validated against cached map -> publish(): new FieldPublication generation (map + estimate pair, pi_instance, session, completed_at, snapshot_after). [code] client.cpp:716-731, 1296-1315; client.h:176-189
- Session loss keeps the published pair and the cached map. [code] client.cpp:1099-1131
- Sizes: map 24 + 28/object, estimate 24 + 20/object, max 128 objects (3608 / 2584 bytes). [code] translaGATR/link_documents.h:154-174, 227-247

### 2.2 Accepted vs pending
| State | Where visible |
|---|---|
| Pi announces new map_id | state_.state.map_id [code] client.cpp:629 |
| Map being read | FieldSyncStatus.map_reading/map_received [code] client.cpp:266-273 |
| Map complete but not accepted (no estimate yet) | fieldSync().map_id != field().map_id [code] client.cpp:268, 680-687 |
| Accepted | FieldPublication.generation (Brain-local counter) [code] client.cpp:1297 |
| Used by the planner | Motion::field_ copy, pulled each drive step; plan() records plan_generation_ = field_.generation (private) [code] motion.cpp:371-373, 319; motion.h:215-219 |

- The Brain's accepted field can lag the Pi by >= field_period (0.5 s) plus transfer; a new map is invisible to the Brain until its first estimate completes [code] client.cpp:680-687, 1260-1276. This is exactly the revision a receiver must not replace with the Pi's newer one.
- If ProsLink::field() cannot take the mutex, Motion keeps its previous copy [code] pros_link.cpp:242-249; so "accepted" (FieldPublication) and "used" (Motion::field_) can differ transiently [inference].
- Direct plans never read the field [code] investigatr/planner.h:37. The field only matters for avoiding plans and object references.

### 2.3 Field identity and content the Brain holds
- Map identity: map_id = crc32 of document; revision u16 declared in field definition [code] investigatr/field.h:15-21; link_documents.h:154, 187-195
- Estimate identity: (pi_instance, estimate_id); estimate header names map_id, odometry_epoch, anchor_revision [code] link_documents.h:253-260; docs/communigatr.md:448-449
- Per object: id, kind landmark/fixed, flags obstacle/estimated/reference, nominal pose, box (object frame), estimate source none/nominal/observed, valid, pose, age [code] investigatr/field.h:43-57; link_driver.cpp:125-154
- Bounds (robot footprint must stay inside) [code] field.h:23-29
- NOT held on the Brain: object names, tag mounts, heights/3D, field art. Names exist only as compile-time generated references (operaGATR include/field_references.h, map 0xF6B3F2C9 rev 1) [code] brain/operaGATR/include/field_references.h:1-16
- Standalone users have no field source at all today; they must supply geometry directly [inference].

## 3. Plans and paths

### 3.1 Who does what
| Action | Code | Task |
|---|---|---|
| Hand out command id, latest-wins request | DriveRequests::goTo [code] brain/actuGATR/src/drive_requests.cpp:8; ProsDrive::goToDirect/Avoiding [code] actuGATR/pros/pros_drive.cpp:26-46 | caller (app / opcontrol / autonomous) under ProsDrive mutex (5 ms bound) |
| Pending goTo shown as waiting | DriveRequests::snapshot [code] drive_requests.cpp:41 | reader |
| Start (replace) | apply() -> DriveOwner::goTo* -> Motion::start [code] drive_requests.cpp:63-75; motion.cpp:167-202 | drive task |
| Replace clears old path (only if planned_) | [code] motion.cpp:172-174 | drive task |
| Accept plan (first, replan, correction) | Motion::plan -> follower_.start, sink_->reportPath(cmd, path) [code] motion.cpp:315-327 | drive task |
| Replan trigger (field correction) -> halt | [code] motion.cpp:343-361, 435-441 | drive task |
| Correction after settling short | planned_=false, next update plans again [code] motion.cpp:474-477 | drive task |
| Cancel | Motion::cancel -> finish(kCancelled, kCancelledByCaller) [code] motion.cpp:208-212; from DriveOwner::disable/manual [code] drive_owner.cpp:31-44; ProsDrive::cancel (mutex or atomic forced stop) [code] pros_drive.cpp:59-68, 91-95 | drive task |
| End (completed/failed/cancelled) clears path (only if planned_) | Motion::finish [code] motion.cpp:229-238 | drive task |
| Source changed | Motion::setSource -> kSourceChanged [code] motion.cpp:220-227 | caller |

- Cancel reasons are not distinguished: B button, manual takeover, competition disabled() all give kCancelledByCaller [code] drive_owner.cpp:31-44; operaGATR main.cpp:406-410, 459-461.
- Single sink pointer [code] motion.h:161, 189. operaGATR sets it to ProsLink [code] operaGATR main.cpp:170. locaGATR has no Motion.

### 3.2 Path representation [code] investigatr/path.h:15-41
- Path{mode kDirect|kAvoiding, segments}; PathSegment{kind kTurn|kTranslate, start Pose, end Pose, reverse, turn_direction +1 CCW/-1 CW, exact}. Field frame, meters, radians.
- Turn-only segments exist: non-holonomic legs are turn+translate; a holonomic pure heading change is one kTurn [code] docs/investigatr.md:173-182, 230-235.
- Frame of a path: Motion captures robot.frame at the first usable update and fails with kFrameChanged if it changes [code] motion.cpp:415-423. Not passed to PathSink.
- Field used for the path: field_.generation/map at plan time [code] motion.cpp:319. Not passed to PathSink.
- Target: MotionStatus.destination/has_destination (resolved, field frame) [code] motion.h:92-93; motion.cpp:320-321. Reference object id is private (reference_) [code] motion.h:194.

### 3.3 What PATH_REPORT sends today
- LinkDriver::reportPath: points = every segment start and every translation end, rounded mm, consecutive repeats dropped; headings, turn direction, reverse, exact and segment kinds are dropped; mode 0 none/1 direct/2 avoiding [code] link_driver.cpp:158-183
- Client::reportPath: >13 points thinned evenly to 13 (first/last kept); latest replaces unsent; one attempt; dropped without session [code] client.cpp:213-246; frames.h:247
- Scheduled at priority 9 of 10, only when the state poll is not due [code] client.cpp:1021-1027; client.h:8-14
- Pi stores it in CommandState.path (latest only, reset per session) and inspection reads it [code] pi/naviGATR/src/impl/commands/brain_link_commands.cpp:577-603, 783; src/state/command_state.h:71-83; src/inspection/inspection_document.cpp:1408
- Called from: Motion::start/plan/finish -> ProsLink::reportPath (drive task, takes link mutex, 20 ms bound) [code] pros_link.cpp:251-258

[measured-host] probe.exe:
```
E turn-only path: have=1 mode=1 points=1
F 40-segment path (21 vertices): points=13 first=(500,500) last=(2500,500)
  (500,500) (700,500) (800,900) (1000,900) (1200,900) (1300,500) (1500,500) ...
```
- E: an in-place rotation becomes a single point; the rotation is invisible.
- F: true zigzag vertices are (500,500),(600,900),(700,500),(800,900)...; the thinned polyline draws (500,500)->(700,500), a line the robot never drives (can cross obstacles on screen).

### 3.4 Existing lifecycle gap (defect, display only)
Motion::finish and Motion::start clear the reported path only when planned_ is
true [code] motion.cpp:172-174, 230-237. planned_ is false during a replan
halt and before a correction replan [code] motion.cpp:435-441, 474-477, so the
last reported path is never cleared in those windows.

[measured-host] probe.exe (StraightPlanner + SimulatedState, current tree):
```
A cancel during halt: state=cancelled ... last=(cmd 1, segs 1)      <- no clear
B replan refused after halt: state=failed reason=no path ... last=(cmd 1, segs 1)  <- no clear
C new command during halt (before its plan): state=waiting ... last=(cmd 1, segs 1)
D cancel while planned (control): state=cancelled ... last=(cmd 1, segs 0)  <- clear sent
```
No existing test covers A/B/C (motion_gtest.cpp:506-532 only cover the planned case).
Minimal fix for any integration: track "a path of this command was reported"
separately from planned_. spectuGATR updatePath must not inherit this gap.

## 4. Execution / actuation
- MotionState {idle, waiting, running, settling, completed, cancelled, failed}; MotionReason 25 values [code] motion.h:45-81
- MotionStatus: command_id, state, reason, mode, destination, reference_source, segment/count, distance_error, heading_error (dest minus robot), cross_track (left positive), elapsed, plans, blocking [code] motion.h:87-103; follower.h:54
- DriveStatus: fault, stopped, command (applied body command after desaturation), wheels (m/s), saturation, motor_rpm per group (targets, positive forward) [code] brain/actuGATR/include/actugatr/drive.h:37-47; src/drive.cpp:26-78
- DriveFault {none, wrong frame, non-finite, unsupported motion, stale} [code] drive.h:21-27; FollowFault {none, invalid path, tracking error} [code] follower.h:47
- DriveMode {disabled, manual, navigate} [code] drive_owner.h:17; step [code] drive_owner.cpp:62-81
- DriveSnapshot {motion, drive, mode} published every drive step under ProsDrive mutex [code] pros_drive.cpp:79-112; drive_requests.h:17-21
- telemetryOf(snapshot) -> BrainTelemetry motion/wheels groups, wire units, saturation [code] actuGATR/src/telemetry.cpp:52; telemetry.h:23
- Motor feedback: none. No actual velocity, current, temperature or motor fault is read anywhere in brain/ (grep for get_actual_velocity/get_temperature/get_current_draw/get_faults found nothing outside PROS headers) [code]. Only targets exist. ProsDrive::lockTimeouts is the only drive-task health counter [code] pros_drive.h:60.
- Motor commands are written only by Drive via MotorOutput in the drive task [code] drive_owner.h:1-5; pros_motor_output.cpp:117-135.

## 5. Attitude
- ProsVexImu::attitude(): pros::Imu get_roll/get_pitch in degrees, IMU frame, stamp_ms = pros::millis() at read; invalid on error/calibrating/non-finite. No yaw is read. [code] brain/communiGATR/pros/pros_vex_imu.cpp:37-48; pros_vex_imu.h:30
- robotAttitudeFromVex(vex, mount_yaw_deg): rotates the up vector by the mount yaw, exact; roll (-pi,pi], pitch [-pi/2,pi/2]; invalid on non-finite [code] communiGATR/src/attitude.cpp:30-61; attitude.h:31-40
- Convention: robot +x fwd, +y left, +z up; roll about +x positive left side up; pitch about +y positive nose down. VEX convention match is UNVERIFIED on hardware [code] attitude.h:5-13; frames.h:281-283
- kVexImuMountYawDeg = 0.0 PLACEHOLDER [code] brain/robot/gatr2_robot.h:49-53
- Heading/yaw: VEX rotation is CW positive in PROS; the Brain negates it to CCW for the bench IMU [code] pros_vex_imu.cpp:26. The pose heading comes from the Pi (fused), not the VEX yaw [code] link_driver.cpp:88.
- So Attitude{roll, pitch, yaw} on the Brain mixes two sources and two times: roll/pitch from the Brain IMU at pros::millis(), yaw from the Pi pose at its derived age [inference]. Timing must be kept per component, or yaw must be declared "from pose".
- Attitude is pulled by the apps (they own g_vex): locaGATR main.cpp:75-80; operaGATR main.cpp:142-147.
- IMPORTANT: TELEMETRY attitude is also a Pi estimator input, not display only. With a Brain VEX IMU profile the Pi profile builder wires an Attitude observation from TELEMETRY into the estimator (measured while fresh, assumed level after 250 ms) [code] pi/naviGATR/src/runtime/brain_profile_builder.cpp:537-538, 605-610; brain_profile_builder.h:97; src/impl/localization/brain_imu_wheel_bench.cpp:204-224; src/resources/brain_imu_bench.h:6-13. Moving attitude off TELEMETRY into a viz-only stream would degrade the Pi's tilt (camera association uses attitude policy) [code] src/impl/world_estimation/tag_mount_association.cpp:68-74.
  - Doc/code contradiction: "localization never reads it" in translaGATR/frames.h:268-269, docs/communigatr.md:707-709, communigatr/attitude.h:2-3, gatr2_robot.h:51-52, 56-57 vs the Pi code above.
- Pi times Brain attitude by Pi arrival, not Brain stamp [code] brain_imu_wheel_bench.cpp:215.

## 6. The two apps

### 6.1 operaGATR [code] brain/operaGATR/src/main.cpp
| Task | Created | Period / prio | Does |
|---|---|---|---|
| PROS init | initialize() 390-404 | once | createLink, createDrive (Motion, sink=ProsLink 170, ProsDrive 175) |
| communigatr poll | ProsLink::start (pros_link.cpp:122) | 2 ms / 9 | client.poll under link mutex |
| communigatr-usb-tx/rx | ProsUsbPort::open (pros_usb_port.cpp:205-214) | - / 8 | USB only: stdout named stream, stdin reader |
| actugatr drive | ProsDrive ctor (pros_drive.cpp:8-13) | 10 ms / 9 | requests, owner.step (Motion+Drive), publish snapshot |
| "testing ui" background | 403 | 20 ms (380) / 8 | link start, LinkEvents, startup placement, TELEMETRY every 100 ms (366-374), display every 100 ms (376-379) |
| opcontrol | PROS | 10 ms (521) | buttons, sticks -> ProsDrive, VEX recal (blocks ~1 s, 493) |
| autonomous | PROS | - | runDirect/runAvoiding + waitFor polling (414-421, 202-221) |

### 6.2 locaGATR [code] brain/locaGATR/src/main.cpp
| Task | Created | Period | Does |
|---|---|---|---|
| "locaGATR" | initialize() 671-674 | 20 ms (665; robot_config.h:41) | link start, readView, events, startup placement, pages, VEX recal (blocks ~1 s, 614), TELEMETRY attitude every 100 ms (650-658), display every 100 ms (660-664) |
| communigatr poll (+USB) | as above | 2 ms | as above |
| opcontrol | idle loop 680-684 | 20 ms | nothing |

### 6.3 Mutexes and hazards
- Link mutex: every ProsLink public call, bounded 20 ms; the poll task holds it for the whole client.poll [code] pros_link.cpp:64-82, 155-162; pros_link.h:46
- ProsDrive mutex: request post/status copy, bounded 5 ms [code] pros_drive.h:29; pros_drive.cpp:28-110
- The drive task (10 ms period) already takes the link mutex up to 2-3 times per step (field, robot, reportPath), each bounded 20 ms [code] motion.cpp:371, 388; pros_link.cpp:233-258. A worst case wait already exceeds the drive period [inference]. The publisher must add no blocking wait here.
- PROS deletes competition tasks at mode changes without releasing held mutexes; a deleted holder leaks the mutex forever [code] docs/communigatr.md:203-207; pros_drive.h:7-12. No publisher lock may be taken from opcontrol/autonomous unless it cannot leak (lock-free mailbox, or try-lock never held across a deletable point).
- std::shared_ptr is not safe across tasks on the PROS toolchain [code] docs/communigatr.md:208-209.
- Precedent for nonblocking cross-task handoff: ProsUsbPort lock-free SPSC queues [code] brain/communiGATR/include/communigatr/pros_usb_port.h:1-9.

### 6.4 Where telemetry is built/sent now
- operaGATR sendTelemetry: ProsDrive::status copy -> telemetryOf -> setAttitude -> stamp_ms -> ProsLink::reportTelemetry, background task, 100 ms grid [code] operaGATR main.cpp:276-288, 366-374
- locaGATR sendTelemetry: attitude only, main loop, 100 ms grid [code] locaGATR main.cpp:83-91, 650-658
- Client side grid/slot rules [code] client.cpp:840-846, 961-975, 1033-1044; period from gatr2_robot::kTelemetryPeriodMs=100 [code] gatr2_robot.h:60-61; locaGATR main.cpp:70; operaGATR main.cpp:137

### 6.5 Single publisher: owner and reference passing (proposal input)
- Natural owner: the app (main.cpp), like g_link/g_task: one static spectugatr publisher constructed in initialize() before createLink/createDrive [inference; pattern at operaGATR main.cpp:41-52, 390-404].
- References (all optional, via our libraries' neutral hooks so communiGATR/actuGATR do not depend on spectuGATR):
  - communiGATR: an observer in LinkConfig/ClientConfig called in the poll task on new state / new FieldPublication / session and placement changes (precedent: ClientConfig::bench_imu std::function called in the poll task) [code] client.h:67-68; client.cpp:1062-1067
  - actuGATR: a richer path observer than PathSink (needs lifecycle reason, frame, field generation) set on Motion; an execution observer in ProsDriveConfig called after owner_.step [code hook points] motion.cpp:172-174, 229-238, 325-327; pros_drive.cpp:100-110
  - app: attitude (owns g_vex), custom channels, program lifecycle (initialize/disabled/autonomous/opcontrol)
- Integrations (brain/spectuGATR/integrations/) translate investiGATR/communiGATR/actuGATR types into spectuGATR types [inference].

## 7. Proposed publisher methods: call sites

All must be nonblocking; "blocking OK" means a bounded wait would be tolerable, which none of these are.

| Method | Call site (file:line) | Task / prio / period | Locks held at the call | Blocking acceptable |
|---|---|---|---|---|
| updatePose (Pi-fed) | after Client::handleState stores state_ [code] client.cpp:626-635 (or ProsLink::run after client_.poll, pros_link.cpp:158) | communigatr / 9 / 2 ms (20 ms state period) | link mutex | No: stalls all link I/O and every ProsLink caller |
| updatePose (validity/frame as used) | LinkDriver::robot [code] link_driver.cpp:54-93 is on demand; for "what the robot used" hook Motion::update after robot() [code] motion.cpp:388 | drive / 9 / 10 ms | none (ProsLink lock released) | No: drive loop |
| updatePose (standalone) | user's own loop | user task | user's | No |
| updateField (accepted) | Client::publish [code] client.cpp:1296-1315 | communigatr | link mutex | No; copy only on generation change (<= ~6.2 KB) |
| updateField (pending map) | handleMapChunk complete [code] client.cpp:680-687 | communigatr | link mutex | No |
| updateField (planner used) | Motion::plan [code] motion.cpp:315-319 | drive | none | No |
| updatePath (accept) | Motion::plan [code] motion.cpp:325-327 | drive | none | No |
| updatePath (replace) | Motion::start [code] motion.cpp:172-174 | drive (via apply) | none | No |
| updatePath (clear on end/cancel) | Motion::finish [code] motion.cpp:229-238 (fix planned_ gap, 3.4) | drive | none | No |
| updatePath (pending request) | DriveRequests::goTo [code] drive_requests.cpp:8 | caller task, e.g. opcontrol | ProsDrive mutex | No: deletable task holding a mutex |
| sample (execution) | ProsDrive::run after owner_.step [code] pros_drive.cpp:100-110 | drive / 9 / 10 ms | none (between takes) | No |
| sample (attitude) | app loop where attitude() is read [code] operaGATR main.cpp:142-147, 276-288; locaGATR main.cpp:75-91 | background 20 ms / locaGATR 20 ms | none | No |
| sample (link stats) | app loop reading ProsLink::status [code] operaGATR main.cpp:334; locaGATR main.cpp:114 | app task | none | No |
| events: session/Pi restart/placement/profile | client.cpp:446-454 (HELLO Ok), 1102-1148 (loseSession), 1150-1156 (settlePlacement), 1214-1221 (profile applied), 1163-1167 (control) | communigatr | link mutex | No |
| events: link lost/restored etc. (derived) | LinkEvents::update [code] communiGATR/src/link_events.cpp:71; called at operaGATR main.cpp:347, locaGATR main.cpp:553 | app task | none | No |
| events: motion lifecycle | Motion::start/finish/plan/halt [code] motion.cpp:167-238, 435-441 | drive | none | No |
| events: drive fault / mode | Drive::stop/apply [code] drive.cpp:26-78; DriveOwner [code] drive_owner.cpp:31-81 | drive | none | No |
| lifecycle: program/competition | initialize/disabled/autonomous/opcontrol [code] operaGATR main.cpp:390-423; locaGATR main.cpp:671-684 | PROS tasks (deletable) | none | No |

Implication [inference]: the publisher API must be a copy-into-mailbox call
(latest-wins slots for live state, bounded queue for events) with all
encoding and I/O on one publisher-owned task. Field and path payloads should
be copied into fixed-capacity storage without heap allocation in the drive and
poll tasks.

## 8. Units and conventions on the Brain

| Item | Convention | Ref |
|---|---|---|
| Library units | m, rad, s, m/s, rad/s | [code] investigatr/geometry.h:11-15; docs/actugatr.md:25-30 |
| Field frame | origin inside bottom-left corner (audience view), +x right, +y up/toward 0-deg wall, fixed, no alliance flip | [code] docs/investigatr.md:28-30; pi/naviGATR/docs/coordinates.md:11, 18-20; gatr2_robot.h:9-11 |
| Heading | CCW from +x, wrapAngle (-pi, pi] | [code] geometry.h:2-3, 33-34; coordinates.md:22-24 |
| Robot frame | +x forward, +y left, +z up; origin the Pi-reported point | [code] geometry.h:2-3; coordinates.md:12 |
| Wire pose | mm i32, centidegrees (-18000, 18000], ages ms u16 | [code] frames.h:353-369; coordinates.md:25-27 |
| TELEMETRY | mm, mm/s, cdeg, cdeg/s, rpm x10, i16 saturated | [code] frames.h:268-285; actuGATR/src/telemetry.cpp |
| Attitude | roll about +x left-up positive, pitch about +y nose-down positive | [code] attitude.h:5-7; frames.h:281-283 |
| VEX IMU rotation | PROS CW positive, negated to CCW mdeg | [code] pros_vex_imu.cpp:26 |
| Body command | vx fwd, vy left, omega CCW | [code] docs/actugatr.md:30; chassis.h:22-27 |
| Brain clock | ProsLink::now s since PROS init (micros); stamp_ms = millis | [code] pros_link.cpp:135-137; rtos.h:207-241 |
| Plan mode numbering | investiGATR 0 direct/1 avoiding; PATH_REPORT 1 direct/2 avoiding | [code] path.h:33; frames.h:224-228 |

## 9. Standalone API dependency boundary

Must NOT appear in the public spectuGATR Brain API (core include/, src/, pros/):
- investiGATR: Pose, Point, Seconds/Meters/Radians aliases, FrameGeneration, Field, FieldObject, Bounds, Box, MapIdentity, MapId, ObjectId, EstimateSource, Path, PathSegment, SegmentKind, PlanMode, CommandId, PathSink, StateSource, RobotState, RobotStatus, Reference, MotionModel/Footprint.
- communiGATR: Client, StateSample, FieldPublication, FieldSyncStatus, BenchImuSample, VexAttitude, RobotAttitude, ProsLink/ProsLinkStatus, LinkEvent/LinkSnapshot, Readiness, PlacementStatus, ProfileStatus, ProsVexImu.
- actuGATR: MotionStatus/State/Reason, DriveStatus, DriveFault, DriveSnapshot, DriveMode, ChassisCommand, WheelSpeeds, FollowOutput.
- translaGATR Brain-link types (BrainState, BrainTelemetry, PathPoint, field document records): these are the Brain<->Pi contract, not the viz contract [inference].
- PROS headers outside the pros/ backend.

Minimal own types [inference]:
- Units: explicit field names with unit suffix or local aliases (meters, radians, seconds).
- Pose2 {x_m, y_m, heading_rad}; Attitude {roll_rad, pitch_rad, yaw_rad} with per-component "supplied" flags (heading-only = tilt not supplied).
- Stamp {clock domain id, seconds} (Brain uptime domain; source ages kept as ages when the source gives ages).
- PoseValidity enum + short reason (maps from RobotStatus); PlacementId (opaque 64-128 bit; integration packs pi_instance/session/epoch/anchor, standalone user counter).
- FieldDefinition {field id, revision, bounds, objects {id, optional name, kind, flags, nominal Pose2, box}}; FieldEstimate {field ref, estimate id, placement id, objects {id, valid, source, pose, age}}.
- PathUpdate {path/command id, lifecycle (active, replaced, cleared, completed, cancelled, failed + reason text), mode label, segments {turn|translate, start, end, reverse, turn direction}, target optional, placement id, field ref}.
- Channel {name, unit, type, group}; Sample {channel, value, stamp}; Event {name, severity, text, stamp}.
- SourceIdentity {robot name, source kind "brain", boot/session nonce}; ProsLink::nonce shows how to make a boot nonce [code] pros_link.cpp:391-404.

## 10. Other contradictions / notes
- Pi attitude use vs "localization never reads it" docs: section 5.
- docs/communigatr.md:498-500 "No clock value of one device is subtracted from another's" holds on the Brain [code] link_driver.cpp:89.
- CommandState.path on the Pi is inspection-only today [code] brain_link_commands.cpp:577-603; inspection_document.cpp:1408 (only reader found by grep).
- PROS build: libraries are listed explicitly in brain/gatr2_brain.mk:21-60 (GATR2_BRAIN_LIBS); a spectugatr lib fits as another entry. CI does not build PROS programs [code] brain/README.md:25-26.

## Open questions / needs bench test
1. Wire: add a pose sequence or Pi measurement id to GET_STATE so the Brain can publish only new poses (no duplicates) and give receivers an exact identity? Otherwise dedupe is heuristic.
2. Pi cycle vs request arrival: can the GET_STATE reply use a cycle that started before the request arrived (age bound short by one cycle)? Needs Pi-side check.
3. Attitude: keep TELEMETRY attitude to the Pi (functional tilt input) regardless of spectuGATR; decide whether spectuGATR attitude is a second copy of the same sample.
4. VEX roll/pitch sign and mount yaw: UNVERIFIED on hardware (docs/brain_setup.md section 10).
5. Motion path lifecycle gap (3.4): fix in actuGATR as part of integration, or only in the viz adapter? It also affects today's PATH_REPORT display.
6. Which pose does the viz show: every GET_STATE reply (poll task), or the pose Motion actually used (drive task)? They differ by up to one drive period and by lock misses.
7. Field names/tags/3D for standalone mode have no Brain source; is id + geometry enough, or does the Brain publisher need an optional name table (e.g. from generated field_references.h)?
8. Real V5 cost of the extra copies in the poll/drive tasks and any new publisher task at priority <= 8: unmeasured; host only.
9. Competition task deletion: confirm the chosen publisher handoff cannot leak a lock when opcontrol/autonomous is deleted mid-call (bench test with mode switches on a V5).
