# actuGATR

Brain-side movement for a PROS program. actuGATR turns a destination into
motor commands:

- It resolves the destination against a reference.
- It asks investiGATR for a path the drivetrain can execute.
- It follows the path from pose feedback.
- It drives tank or mecanum wheels through the V5 motors' own velocity loops.

Robot state and the field come from any `investigatr::StateSource`; on the
robot that is communiGATR's `ProsLink`.

```
investiGATR  shared types, references, planner
   ^      ^
   |      |
communiGATR   actuGATR   (neither depends on the other)
```

Sources: [brain/actuGATR](../brain/actuGATR). Planner and shared types:
[investiGATR](investigatr.md). The Pi link: [communiGATR](communigatr.md).
Robot setup, calibration and the test programs: [Brain setup](brain_setup.md).

## Units and frames

- SI units throughout: m, rad, s, m/s, rad/s.
- **Field frame:** +x right, +y up on the field diagram, heading CCW from +x.
- **Robot frame:** +x forward, +y left, origin the point the Pi reports.
- `ChassisCommand` names its frame. Body commands are `vx` forward m/s, `vy` left m/s and `omega` CCW rad/s. `toBody()` rotates a field command into the body frame.

## Movement commands

```cpp
#include "actugatr/motion.h"
#include "field_references.h" // generated: struct Field : investigatr::FieldReferences

motion.goToDirect({1.0, 0.5, 0.0});                       // field origin
motion.goToAvoiding({-0.4, 0.0, 0.0}, Field::BlueGoal2East);
motion.goToDirect({0.3, 0.0, 0.0}, Field::RobotAtStart);  // relative to where it started
```

`goToDirect(destination, relative_to = Field::Origin, options)` and
`goToAvoiding(...)` take the same reference argument:

- **The destination** (x, y, heading) is expressed in the reference frame, and the reference translation and rotation are applied exactly once.
- **Origin** is virtual. It is not an obstacle and needs no map.
- **Object references** name a map object by its stable id and the map id they were generated from.
  - A reference from another field definition fails with `kMapMismatch` instead of picking the wrong object.
  - The tables are generated from the Pi's field definition (`navigatr_field_refs`), so season names live in the application, not in the libraries.
- **`RobotAtStart`** is the robot pose when the command begins.

### Direct and avoiding

**Direct** plans a straight approach shaped for the drivetrain and never looks at obstacles:
- a tank turns in place, drives, and turns to the final heading;
- a mecanum drive translates while turning.

It needs no field, landmarks or AprilTags.

**Avoiding** needs a complete field from the source:
- It plans around the field's obstacle boxes and inside its boundary.
- It reports `kStartBlocked`, `kGoalBlocked`, `kStartOutOfBounds`, `kGoalOutOfBounds` or `kNoPath` explicitly, and never falls back to direct.
- Without a field it waits up to `input_wait_timeout`, then fails with `kFieldUnavailable`.

### Options

| Field | Default | Meaning |
|---|---|---|
| `timeout` | 0 | seconds from the first update; 0 = `MotionConfig::default_timeout` |
| `require_observed_reference` | true | object references wait for an observed estimate; false also accepts the nominal map pose |
| `reference_max_age` | 0 | observed estimate age limit, 0 = none |
| `allow_reverse` | true | non-holonomic drives may back up instead of turning around |
| `speed_scale` | 1 | fraction (0, 1] of the model speed limits |

### Status

`motion.status()` reports:
- `command_id`, `state` (`kIdle`, `kWaiting`, `kRunning`, `kSettling`, `kCompleted`, `kCancelled`, `kFailed`) and `reason`;
- the resolved `destination` and the reference `source` (nominal or observed);
- the segment index and count, distance, heading and cross-track errors, `elapsed`, the number of `plans`, and the `blocking` object.

`toString()` gives short names for the screen.

### Command rules

- Each command replaces the previous one at once and gets a new id. Ids never repeat within a program run.
- `cancel()` ends the active command with `kCancelledByCaller`. Nothing ever resumes on its own.
- **Waiting before the first plan**, while the robot state is unusable (not connected, stale, no profile, calibrating, unplaced):
  - the motors stop;
  - after `input_wait_timeout` the command fails with the matching reason (`kInputUnavailable`, `kNoProfile`, `kCalibrating`, `kPlacementRequired`).
- **Losing the state after the first plan:**
  - if the link is lost, the command fails at once with `kInputLost`;
  - if the state is stale while still connected, the motors stop, and after `input_loss_timeout` the command fails with `kInputLost`.
- **Frame change:** the field frame is captured at the first usable update. A different frame later (Pi restart, reinitialization, new placement, new profile) fails with `kFrameChanged`.
- **Field corrections:** a new field generation re-resolves an object reference and revalidates the remaining avoiding path with the planner's `clear()`.
  - A move beyond `replan_distance`/`replan_heading`, or a blocked path, replans.
  - The replan first brakes until the pose is steady (`halt_*`), then plans from where the robot stopped.
  - The command keeps its id and its deadline. Smaller moves keep the current path.
- **Settle and correct:** after the follower settles, errors above `final_position_tolerance`/`final_heading_tolerance` plan a correction.
- **Plan limit:** first plans, replans and corrections share `max_plans`; beyond it the command fails with `kPlanLimit`.
- **Path sink:** `setPathSink()` receives every plan and a clear when the command ends. `ProsLink` forwards them to the Pi viewer.

## Following

The followers execute a path one segment at a time:

- A **translate** segment runs to its end, slowing toward it, and a **turn** segment runs to its heading, before the next segment starts. The executed motion therefore stays on the planned segments and never cuts a corner.
- **Speed profile:** along-track speed is the smaller of the PID on the remaining distance and the stopping-distance limit `sqrt(2 a d)`. Output accelerations are slew limited by the model limits.
- **`DifferentialFollower`** (tank): heading PID on the travel direction plus cross-track PID. It drives backward on reverse segments, and pauses translation beyond `max_heading_error`.
- **`HolonomicFollower`** (mecanum): field-frame velocity along the segment plus a cross-track correction. The heading moves with progress along the segment. It holds position during turns.
- **Tracking tolerance:** drift beyond `tracking_tolerance` from a segment fails the path (`kTrackingError`). The planner's clearance only covers motion near the segments, so keep `tracking_tolerance` below the model `clearance`. A turn allows drift from its point up to hypot(`position_tolerance`, `tracking_tolerance`), the most a finished translation can leave; the testing program refuses to start the drive unless that is below `kClearance`.

## Drivetrains

```cpp
actugatr::TankConfig tank;
tank.left.count  = 3;
tank.left.motors[0] = {11, true}; // Smart Port, reversed
// ...
tank.track_width = 0.30;           // driven wheel contact spacing
tank.wheels.wheel_diameter = 0.1016;
tank.wheels.gear_ratio     = 0.6;  // wheel turns per motor turn
tank.wheels.cartridge      = actugatr::Cartridge::kBlue;
```

Configs and kinematics:
- `TankConfig` and `MecanumConfig` list motor groups in kinematics order: left, right; or front left, front right, rear left, rear right. Each motor has a port and a reversal flag.
- The driven wheels are separate from the tracking wheels, whose geometry belongs to the robot profile.
- `motionModel(config, footprint, clearance, limits)` gives the planner its capabilities:
  - tank: non-holonomic, turns in place, may reverse;
  - mecanum: holonomic.
  - The speed limits are clamped to what the wheels reach.
- `TankKinematics` refuses any sideways command.
- `MecanumKinematics` is the X-roller model: strafing left runs the front-left and rear-right wheels backward.

`Drive` turns a body command into motor targets:
- Wheel speeds are desaturated together, which keeps direction and curvature.
- Motor rpm = wheel speed / (pi * diameter) * 60 / gear_ratio.
- **Control method:** the V5 smart motor's internal velocity loop (`move_velocity`), using its own encoder as feedback.
- Pose feedback is the follower's outer loop.
- **Immediate stop:** `Drive` stops the motors at once, with the configured `StopMode`, for:
  - a field-frame or non-finite command;
  - motion the kinematics refuses;
  - a command older than `command_timeout`;
  - wheel speeds all below `stop_below`.

## Ownership

`DriveOwner` is the one writer of drive commands. Each `step()` applies the current mode's command:

| Mode | Command |
|---|---|
| `kNavigate` | `Motion::update` |
| `kManual` | the manual demand in physical units: `manual_speed` m/s and `manual_omega` rad/s at full stick; strafe only on holonomic drives |
| `kDisabled` | a stop |

- A manual demand older than `manual_timeout` stops the drive.
- Manual input and disabling cancel navigation.

`ProsDrive` runs `DriveOwner::step` in its own task (10 ms). Only that task touches the owner, so planning, path reports and Pi link reads all happen there:
- Application calls post a request (`DriveRequests`: a goTo, a manual demand or a stop; the latest wins) and read the last published `DriveSnapshot` under a mutex held only to copy.
- `goToDirect`/`goToAvoiding` return the command id at once; until the task takes the request, `status()` shows it waiting under that id, so a test never reads as idle.
- Every take is bounded (`lock_timeout_ms`). A busy call returns 0 or false; `status()` then leaves the caller's copy unchanged.
- PROS deletes competition tasks at mode changes without releasing their mutexes. If that ever leaks this mutex, the task keeps stepping without new requests and counts lock timeouts: the current command still runs closed loop, and a manual demand goes stale and stops. A stop that cannot take the mutex is carried by an atomic, ordered against posted requests.

`PortMap` checks every active Smart Port device: drive motors, the VEX IMU when it is the IMU source, and the RS-485 link when selected. A conflict names both devices. `distinctPorts()` checks constant lists in a `static_assert`.

## Telemetry

`telemetryOf(snapshot)` (`actugatr/telemetry.h`) turns a `DriveSnapshot` into the motion and wheels groups of a TELEMETRY for the Pi viewer and recordings. It is display and recording only: nothing reads it back and it takes no lock. The caller sets `stamp_ms` and the attitude group and sends it through communiGATR ([Telemetry](communigatr.md#telemetry)). operaGATR sends one every 100 ms from the drive task's last published snapshot.

| Field | From | Wire unit |
|---|---|---|
| `flags` | `kTelemetryMotion` always; `kTelemetryTarget` (bit 3) exactly when `motion.has_destination`; `kTelemetryWheels` when the drive has wheel groups | bits, see [communiGATR telemetry](communigatr.md#telemetry) |
| `command_id` | `motion.command_id` | 0 = none |
| `motion_state`, `motion_reason` | `motion.state`, `motion.reason` | tables below |
| `plan_mode` | `motion.mode` | 0 direct, 1 avoiding |
| `segment`, `segment_count` | `motion` | count, at most 255 |
| `target_x_mm`, `target_y_mm`, `target_heading_cdeg` | `motion.destination`, field frame, latest resolution | mm; centidegrees in (-18000, 18000]. Valid only with `kTelemetryTarget`, so a destination at the field origin reads (0, 0, 0) with the bit set. Without a resolved destination (idle, waiting for a reference, or ended before resolving) the bit is clear and all three are 0. Once resolved it stays set after the command ends, until the next command |
| `cmd_vx_mm_s`, `cmd_vy_mm_s`, `cmd_omega_cdeg_s` | `drive.command`: the body command the drive applied, after desaturation (the manual demand in manual mode) | mm/s, centidegrees/s CCW |
| `cross_track_mm`, `distance_error_mm` | `motion` | mm |
| `heading_error_cdeg` | `motion.heading_error`, destination minus robot | centidegrees in (-18000, 18000] |
| `drive_fault` | `drive.fault` | table below |
| `wheel_count`, `wheel_rpm_x10[]` | `drive.motor_rpm`, one per wheel group | motor rpm x 10; positive drives the robot forward, before per-motor reversal |

- Values are rounded to the nearest unit and saturated to the field: 16-bit fields to -32768..32767 (about 32.8 m, 32.8 m/s, 327 deg/s, 3276 rpm). Non-finite values are sent as 0.
- Wheel groups are in kinematics order: tank left, right; mecanum front left, front right, rear left, rear right. While the drive is stopped every group reads 0: a commanded stop, not missing data. The group is absent only before the drive task has published a snapshot.
- `DriveStatus::motor_rpm` holds the targets `Drive` last sent to the motors.

`MotionState` (`motion_state`):

| Value | Name | Meaning |
|---|---|---|
| 0 | idle | no command yet |
| 1 | waiting | for usable state, the field or the reference |
| 2 | running | following the path |
| 3 | settling | at the end, settling within the tolerances |
| 4 | completed | ended at the destination |
| 5 | cancelled | ended by a cancel or a mode change |
| 6 | failed | ended, see the reason |

`MotionReason` (`motion_reason`):

| Value | Name | Value | Name |
|---|---|---|---|
| 0 | none | 13 | reference unavailable |
| 1 | invalid command | 14 | unsupported model |
| 2 | invalid config | 15 | start out of bounds |
| 3 | input unavailable | 16 | start blocked |
| 4 | no robot profile | 17 | goal out of bounds |
| 5 | calibrating | 18 | goal blocked |
| 6 | placement required | 19 | no path |
| 7 | input lost | 20 | tracking error |
| 8 | frame changed | 21 | plan limit |
| 9 | field unavailable | 22 | timed out |
| 10 | map mismatch | 23 | source changed |
| 11 | unknown reference | 24 | cancelled by caller |
| 12 | not a reference | | |

`PlanMode` (`plan_mode`): 0 direct, 1 avoiding. This is the investiGATR value; a PATH_REPORT's `path_mode` numbers them differently (1 direct, 2 avoiding).

`DriveFault` (`drive_fault`):

| Value | Name | Meaning |
|---|---|---|
| 0 | none | |
| 1 | wrong frame | a field frame command reached the drive |
| 2 | non-finite command | |
| 3 | unsupported motion | e.g. sideways on a tank |
| 4 | stale command | older than `command_timeout`; also a stale manual demand |

The names are the `toString()` texts. `telemetry_gtest.cpp` fails if a value changes, so these tables and the Pi's names stay in step.

## Tuning

Units are physical, so gains transfer between robots with similar dynamics:

| Setting | Unit | Effect |
|---|---|---|
| `FollowerConfig::along` | (m/s) per m remaining | approach speed near the end of a segment |
| `FollowerConfig::cross` | differential (rad/s) per m, holonomic (m/s) per m | pull back onto the segment |
| `FollowerConfig::heading` | (rad/s) per rad | heading hold while translating |
| `FollowerConfig::turn` | (rad/s) per rad | turns in place |
| `position_tolerance`, `heading_tolerance`, `settle_time` | m, rad, s | when a path ends |
| `tracking_tolerance` | m | allowed drift from a segment; keep below the planner clearance |
| `MotionLimits` | m/s, m/s^2, rad/s, rad/s^2 | speed and acceleration caps, clamped per drivetrain |
| `MoveOptions::speed_scale` | fraction | slower runs without retuning |

Start slow (`speed_scale` 0.3), raise `turn` until turns settle without
overshoot, then `along`, then `cross`. The testing program shows the errors
live; see [Brain setup](brain_setup.md).

## Swerve

Not implemented. The extension points are:
- a `Kinematics` with steering state;
- `MotionModel::min_turn_radius`. Planners refuse a nonzero value today with `kUnsupportedModel`.

## Build and tests

PROS builds compile actuGATR through
[brain/gatr2_brain.mk](../brain/gatr2_brain.mk). Host build and tests:

```sh
export PATH=/c/msys64/ucrt64/bin:$PATH   # Windows Git Bash
cmake -S brain -B build-brain -G "MinGW Makefiles"
cmake --build build-brain -j8
ctest --test-dir build-brain --output-on-failure
```

- `actugatr_tests` cover kinematics signs and units, coupled limits, drive stops, followers closed loop through a drivetrain sim, Motion semantics, drive ownership, port checks, and the TELEMETRY groups (units, rounding, wrapping, saturation, the target bit with a missing destination and one at the origin, motor targets, the enum values above). A Motion test checks the target bit through a real command: clear while the reference is unresolved, set from the first plan through completion, clear for the next command.
- `actugatr_integration_tests` run Motion with the real planner, a tank and a mecanum sim. They check the true rectangular footprint against every obstacle at every simulated step, through turns, corners and a replan after a field correction.

## Limits

- Simulation is not hardware. The V5 velocity loop, wheel slip, battery sag and real latency are not modeled beyond a first-order wheel lag.
- Every vertex is a stop: slower than blended paths, but the executed motion stays on segments the planner checked.
- The pose feedback rate and latency are whatever the Pi link delivers. Keep `max_pose_age` near 0.25 s.
- No dynamic obstacles, other robots or game objects.
