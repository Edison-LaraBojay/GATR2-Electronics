# investiGATR

investiGATR is the Brain's waypoint navigation and motion control library. It
is portable C++17 (it also compiles as gnu++20 in PROS) in namespace
`investigatr`, with no dependency on PROS, serial hardware, Navigatr packets,
motors or threads. A `Navigator` reads poses from an abstract `InputSource`
and returns a platform independent forward/turn demand; the application mixes
it into motor commands.

Status: implemented and host tested on a simulated tank drive. Gains,
tolerances, motor directions and timing have not been validated on the robot.

| Path | Contents |
|---|---|
| `brain/investiGATR/include/investigatr/` | public headers |
| `brain/investiGATR/src/` | library sources (host and PROS) |
| `brain/investiGATR/sim/` | host only drivetrain sim and `SimulatedSource` |
| `brain/investiGATR/tests/` | host tests |

## Units and frames

- Meters, radians, seconds everywhere.
- Field frame: heading counterclockwise from field +x. Robot frame: +x
  forward, +y left. A `Pose` is the robot origin in the field frame.
- `compose(a, b)` is `a * b`: b's translation rotated by `a.heading`, headings
  added and wrapped. `wrapAngle` returns (-pi, pi].
- Every estimate in one `InputSnapshot` shares the coordinate frame named by
  its `frame` generation.

## Headers

| Header | Contents |
|---|---|
| `geometry.h` | `Seconds`, `Meters`, `Radians`, `kPi`, `Pose`, `wrapAngle`, `compose` |
| `input.h` | `LandmarkId`, `FrameGeneration`, `RobotEstimate`, `LandmarkSource`, `LandmarkStatus`, `LandmarkEstimate`, `InputRequest`, `InputSnapshot`, `InputSource` |
| `drive.h` | `DriveCommand`, `TankOutput`, `mixTank`, `slewLimit` |
| `pid.h` | `PidGains`, `Pid` |
| `navigator.h` | `Destination`, `Waypoint`, `Path`, `CommandId`, `MotionOptions`, `MotionState`, `MotionReason`, `MotionStatus`, `NavigatorConfig`, `Navigator`, `isTerminal`, `toString` |
| `odometry_source.h` | `OdometrySource` |

## Navigator

```cpp
#include "investigatr/navigator.h"

investigatr::Navigator navigator(source, config);   // source: any InputSource

// Commands return at once. Each one replaces the previous command.
investigatr::CommandId id = navigator.goTo({1.2, 0.6, 0.0});

// Control task, fixed period, the only motor writer.
const investigatr::DriveCommand demand = navigator.update(now_s);
const investigatr::TankOutput   tank   = investigatr::mixTank(demand);
// tank.left / tank.right in [-1, 1]: apply motor directions and the voltage limit.
```

| Call | Effect |
|---|---|
| `goTo(field_pose, options)` | drive to a field pose |
| `goToRelative(landmark_id, landmark_local_offset, options)` | drive to `compose(T_field_landmark, offset)` |
| `goToRobotRelative(offset, options)` | drive to `compose(robot pose when it starts, offset)` |
| `follow(path, options)` | ordered waypoints; the three calls above are one-stop-waypoint paths |
| `cancel()` | active command becomes `kCancelled` / `kCancelledByCaller` |
| `setSource(source)` | old source gets `request({})`; active command becomes `kCancelled` / `kSourceChanged` |
| `status()` | `MotionStatus` of the current (or last) command |
| `update(now)` | samples the source, returns the drive demand |
| `Navigator::valid(config, &why)` | config validation, `why` names the first failed rule |

`DriveCommand{forward, turn}`: fractions in [-1, 1], +forward drives robot +x,
+turn rotates counterclockwise. `mixTank` gives `left = forward - turn`,
`right = forward + turn`, both scaled together so neither exceeds 1.

`MotionOptions{timeout, require_observed_landmark}`: `timeout` 0 uses
`default_timeout`; `require_observed_landmark` (default true) refuses nominal
landmark estimates.

The Navigator is not thread safe. One task owns it and calls `update()` at a
fixed period; other tasks reach it through that owner's mutex. `update()` does
not block as long as the source's `request()` and `latest()` do not.

### Waiting on a command

```cpp
const investigatr::CommandId id = navigator.goTo(goal);   // under the owner's mutex
for (uint32_t waited = 0; waited < limit_ms; waited += 10) {
    const investigatr::MotionStatus s = owner.status();   // copy under the mutex
    if (s.command_id != id) break;                        // replaced by a newer command
    if (investigatr::isTerminal(s.state)) break;          // kCompleted, kCancelled, kFailed
    pros::delay(10);
}
```

### Command semantics

- Replacement: a new command replaces any command at once. It gets a new id,
  PIDs reset and timers restart. Output slew memory carries over, so a
  replacement does not jerk the drivetrain.
- `cancel()`: zero output from the next `update()`. No effect on a terminal
  command. Terminal states (`kCompleted`, `kCancelled`, `kFailed`) persist until
  a new command; nothing ever resumes, including after input returns.
- Invalid config, empty path, non-finite pose, landmark id 0 or a negative
  timeout: `kFailed` / `kInvalidCommand` at once.
- Timing starts at the command's first `update()`. The timeout covers the whole
  command including waiting. Landmark and robot-relative resolution waits are
  measured from that waypoint's activation.
- Order inside `update()`: dt; terminal returns zero; timeout; frame check;
  input usability; landmark resolution; phase and control; completion.
- `dt = now - previous update`, clamped to [0, `max_dt`]. A gap longer than
  `max_dt` also resets the PIDs and the settle timer.
- `source.request(current need)` runs on every `update()`.

### MotionStatus

| Field | Meaning |
|---|---|
| `command_id` | id of the command this status belongs to |
| `state` | `kIdle`, `kWaiting`, `kTurning`, `kDriving`, `kAligning`, `kCompleted`, `kCancelled`, `kFailed` |
| `reason` | why a command ended early (below), `kNone` otherwise |
| `waypoint_index`, `waypoint_count` | active waypoint and path length |
| `has_destination`, `destination` | resolved field destination of the active waypoint |
| `distance_error`, `bearing_error`, `heading_error` | robot to destination, direction to it minus heading, destination heading minus heading |
| `elapsed` | seconds since the command's first update |

| Reason | When |
|---|---|
| `kInvalidCommand` | bad command or config |
| `kInputUnavailable` | no usable robot pose within `input_wait_timeout` of the start |
| `kInputLost` | `connected` false after usable input, or pose stale longer than `input_loss_timeout` |
| `kFrameChanged` | snapshot frame generation differs from the one captured at the start |
| `kLandmarkUnknown` | source reports `kUnknownLandmark` for the requested id |
| `kLandmarkUnsupported` | source reports `kUnsupported` |
| `kLandmarkUnavailable` | no usable landmark estimate within `input_wait_timeout` of the waypoint's activation |
| `kLandmarkJump` | landmark estimate moved more than the jump limit |
| `kTimedOut` | command timeout |
| `kSourceChanged` | `setSource()` while active |
| `kCancelledByCaller` | `cancel()` |

### Destinations and paths

- `Destination::field(pose)`: absolute field pose of the robot origin.
- `Destination::relative(id, offset)`:
  `T_field_destination = T_field_landmark * T_landmark_destination`. The
  landmark pose from the source is the full physical landmark pose (heading is
  its field orientation); the offset is expressed in the landmark frame and is
  applied exactly once. No camera mounting offset is applied here.
- `Destination::robotRelative(offset)`: resolved once, from the first usable
  robot pose after the waypoint activates. Later pose changes do not move it.
- `Path` is `std::vector<Waypoint>`; `Waypoint{destination, stop}`. The last
  waypoint always stops. Pass-through waypoints ignore their heading and
  advance when the robot is within `waypoint_pass_radius`, or when it passes
  the waypoint's perpendicular while driving toward it. A stop waypoint settles
  before the path advances.
- Routes are caller defined. There is no obstacle routing.

### Landmark relative waypoints

When a relative waypoint is active, `request()` asks for its landmark. The
landmark estimate for that id decides:

| Status | Navigator |
|---|---|
| `kUnknownLandmark` | `kFailed` / `kLandmarkUnknown` at once |
| `kUnsupported` | `kFailed` / `kLandmarkUnsupported` at once |
| `kPending`, `kUnavailable`, `kStale`, `kNotRequested`, or not usable | `kWaiting`, zero output, until usable or `input_wait_timeout` from activation (`kLandmarkUnavailable`) |
| `kAvailable` and usable | resolved |

Usable means status `kAvailable`, the id matches, source is not `kNone`, the
source is allowed (`kNominal` only with `require_observed_landmark` false),
and an observed estimate with a known age is no older than `max_landmark_age`.

After resolution the destination follows the landmark without restarting the
command or its timeout:

- Each accepted candidate is compared with the previous one of the same
  source. A change beyond `max_landmark_jump` (m) or
  `max_landmark_jump_heading` (rad) fails with `kLandmarkJump`.
- A change from nominal to observed is accepted within
  `max_nominal_correction` / `max_nominal_correction_heading`, else
  `kLandmarkJump`. Nominal candidates after an observed one are ignored.
- The destination moves toward the latest accepted candidate by at most
  `landmark_follow_speed * dt` and `landmark_follow_turn_rate * dt`.
- Missing or unusable updates keep the last destination (same frame).

## Input contract

```cpp
class InputSource {
public:
    virtual void          request(const InputRequest& request) = 0;   // nonblocking, idempotent
    virtual InputSnapshot latest(Seconds now) = 0;                     // nonblocking, ages relative to now
};
```

- `InputRequest{landmark, landmark_id}` is what the active command needs. It
  is sent on every `update()`; an idle Navigator sends `{}`.
- `InputSnapshot`:

| Field | Meaning |
|---|---|
| `frame` | coordinate frame generation, 0 = no frame. Changes on every discontinuity (restart, re-anchor, re-align); never reused for a different frame by the same source instance |
| `connected` | the source is alive and delivering data now |
| `link_age` | seconds since the source last heard from its provider |
| `robot.valid` | a real pose exists in `frame` |
| `robot.pose` | robot origin, field frame |
| `robot.age` | seconds since the pose was measured (not since it arrived) |
| `landmark.id` | the requested id, 0 when none requested |
| `landmark.status` | `LandmarkStatus` above |
| `landmark.source` | `kNone`, `kNominal` (configured map pose) or `kObserved` (estimated) |
| `landmark.pose` | physical landmark pose, field frame, same frame as the robot |
| `landmark.age_known`, `landmark.age` | observation age when known |

The robot input is usable iff `robot.valid`, `frame != 0`, `connected`,
`robot.age <= max_pose_age` and `link_age <= max_link_age`. Then:

- Never had usable input: `kWaiting`, zero output; after `input_wait_timeout`,
  `kInputUnavailable`.
- Had usable input and `connected` is false: `kInputLost` at once.
- Had usable input, still connected, pose invalid or stale: `kWaiting`, zero
  output; resumes only within `input_loss_timeout` in the same frame, else
  `kInputLost`.
- The first usable snapshot captures the frame. Any later snapshot with a
  nonzero, different frame fails with `kFrameChanged`, even while waiting.

Invalid, stale or missing input always produces exactly `{0, 0}`.

## Input sources

### Driver

`communigatr::Driver` (brain/communiGATR) implements
`InputSource` over the brain link. Its frame generation changes whenever the
Pi instance, Brain session, odometry epoch or anchor revision changes; its
ages include the measured round trip. See
[communigatr.md](communigatr.md).

### OdometrySource

Dead reckoning from left/right travel and an optional absolute heading.

| Call | Effect |
|---|---|
| `OdometrySource(track_width)` | measuring wheel spacing, used when no heading is supplied |
| `update(now, left_m, right_m)` | cumulative travel in meters |
| `update(now, left_m, right_m, heading)` | plus an absolute heading (any zero); turn comes from the heading |
| `align(pose, now)` | pose in a known field frame, new frame generation |
| `alignFrom(snapshot, max_age, now)` | align from a usable snapshot no older than `max_age`; false and no change otherwise |
| `invalidate()` | back to no pose, frame 0 |
| `aligned()`, `pose()`, `frame()` | state |

No pose before `align()`. `connected` equals aligned. `robot.age` and
`link_age` are the time since the last `update()`, so an odometry source that
stops being updated goes stale. Non-finite readings (for example `PROS_ERR_F`
while the IMU calibrates) are ignored and the pose ages. Landmark requests
report `kUnsupported`. Frame generations are unique per instance.

Brain motor encoders and the V5 inertial sensor, as a sketch (it compiles
against kernel 4.2.2; it has not run on the robot). Ports, gearing, wheel size
and track width are placeholders. `get_position()` reads the group's first
motor. The IMU reports clockwise positive, hence the sign:

```cpp
#include "investigatr/odometry_source.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

pros::MotorGroup left_drive({1, 2}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::MotorGroup right_drive({-3, -4}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Imu        imu(10);

constexpr double kWheelCircumference = 0.2593;       // placeholder, m
constexpr double kWheelPerMotorTurn  = 36.0 / 48.0;  // placeholder gear ratio
constexpr double kTrackWidth         = 0.30;         // placeholder, m
constexpr double kMetersPerDegree    = kWheelCircumference * kWheelPerMotorTurn / 360.0;

investigatr::OdometrySource odometry(kTrackWidth);

// Control task, every period, before navigator.update(now).
void sampleOdometry() {
    const double now     = pros::micros() / 1e6;
    const double left_m  = left_drive.get_position() * kMetersPerDegree;
    const double right_m = right_drive.get_position() * kMetersPerDegree;
    const double heading = -imu.get_rotation() * investigatr::kPi / 180.0;
    odometry.update(now, left_m, right_m, heading);
}
```

`align()` maps the IMU heading onto the field heading, so the IMU zero does not
matter. Calibrate the IMU (`imu.reset()`) before aligning.

### SimulatedSource and DifferentialDriveSim (host only)

`sim/simulated_source.h`: `SimulatedSource` reports truth fed through
`setRobot(pose, now)`, with `setLatency`, `setExtraAge`, `setRobotValid`,
`setConnected`, `setLinkAge`, `setFrame` (default 1), and a landmark table
(`setLandmark(id, SimulatedLandmark{status, source, pose, age_known, age})`,
missing ids report `kUnknownLandmark`). `lastRequest()` and `requestCount()`
expose what the Navigator asked for.

`sim/differential_drive_sim.h`: `DifferentialDriveSim` integrates a
`TankOutput` with `DifferentialDriveConfig{track_width 0.30 m,
max_wheel_speed 1.2 m/s, wheel_time_constant 0.05 s, origin_offset}`.
`pose()` is the robot origin; `leftTravel()`/`rightTravel()` feed
`OdometrySource`. It is a test model, not a robot model.

### Writing an InputSource

1. `request()` and `latest()` must return at once. Do I/O in another task and
   copy a cached snapshot under a mutex.
2. Use meters, radians and the field frame above. Put the robot origin, not a
   sensor, in `robot.pose`.
3. Report `robot.valid = false` whenever there is no real pose in a known
   frame. Never fill in a guessed or old pose.
4. Give every coordinate frame a nonzero generation and change it on every
   discontinuity. Estimates in one snapshot must share it.
5. Ages are measurement ages relative to the `now` passed to `latest()`, in the
   Brain clock. Convert other clocks through measured offsets or round trips,
   never by subtracting unrelated clock values. Keep `link_age` separate.
6. For the requested landmark, report a status. Landmark poses are physical
   landmark poses with full field heading; the Navigator applies the offset.

```cpp
class MySource : public investigatr::InputSource {
public:
    void request(const investigatr::InputRequest& request) override { wanted_ = request; }

    investigatr::InputSnapshot latest(investigatr::Seconds now) override {
        investigatr::InputSnapshot s;
        if (!have_pose_) {
            return s;   // invalid, frame 0
        }
        s.frame       = frame_;
        s.connected   = true;
        s.link_age    = now - heard_at_;
        s.robot.valid = true;
        s.robot.pose  = pose_;
        s.robot.age   = now - measured_at_;
        if (wanted_.landmark) {
            s.landmark.id     = wanted_.landmark_id;
            s.landmark.status = investigatr::LandmarkStatus::kUnsupported;
        }
        return s;
    }
    // ...
};
```

## Control law

Tank drive, forward driving only. With `d` the distance to the destination
point, `eb = wrap(bearing to it - heading)`, `ef = wrap(destination heading -
heading)` and `along = d * cos(eb)`:

- Phase selection (command start, waypoint advance, leaving `kWaiting`,
  leaving `kAligning`): stop waypoint and `d <= arrive_distance` gives
  Aligning; else `|eb| > turn_exit` gives Turning; else Driving.
- Turning: forward 0, `turn = turn_pid(eb)`. `|eb| <= turn_exit` goes to
  Driving; a stop waypoint within `arrive_distance` goes to Aligning. Beyond
  pi - 5 deg the previous turn direction is kept.
- Driving: `forward = clamp(drive_pid(along_error) * max(0, cos eb), 0,
  max_forward)`, where `along_error = along` plus, for a pass-through
  waypoint, the remaining path length to the next stop waypoint.
  `turn = heading_pid(eb)` while `d > near_distance`, else 0. `|eb| >
  turn_in_place_threshold` with `d > near_distance` goes to Turning. For a
  stop waypoint, `d <= arrive_distance` goes to Aligning, and `along <=
  arrive_distance` (at the perpendicular) goes to Aligning if `d <=
  position_tolerance`, else back to phase selection.
- Aligning: forward 0, `turn = turn_pid(ef)`. `d > position_tolerance` goes
  back to phase selection.
- Completion: in Aligning, `d <= position_tolerance` and `|ef| <=
  heading_tolerance` held continuously for `settle_time`. The settle timer
  restarts on leaving that condition, any unusable cycle, any phase change
  and a dt gap.
- Static friction: nonzero `forward` below `min_forward` is raised to it while
  Driving; nonzero `turn` below `min_turn` is raised while Turning and while
  Aligning outside `heading_tolerance`.
- PID: `kP`, `kI`, `kD`, `integral_limit` (max |I term|, 0 disables it),
  `output_limit`. dt 0 is P only; no derivative on the first sample after a
  reset; angular PIDs differentiate `wrap(e - e_prev)`. PIDs reset on command
  start, waypoint change and phase change.
- Output: clamp to `max_forward` / `max_turn`, then slew limit increases of
  |value| per second. Decreases are immediate and a sign reversal drops to 0
  first. Idle, waiting, terminal and unusable input return exactly `{0, 0}`
  and clear the slew memory.

## Configuration

`NavigatorConfig` fields. Defaults are tuning starting points checked on the
host sim, not robot measurements.

| Field | Unit | Default | Meaning |
|---|---|---|---|
| `position_tolerance` | m | 0.03 | completion radius; leaving it restarts approach |
| `arrive_distance` | m | 0.02 | approach target, must be < `position_tolerance` |
| `heading_tolerance` | rad | 0.035 | completion heading error, > 0 |
| `settle_time` | s | 0.25 | time inside both tolerances before completion |
| `near_distance` | m | 0.10 | no heading correction or turn-in-place closer than this |
| `waypoint_pass_radius` | m | 0.15 | pass-through advance radius, >= `near_distance` |
| `turn_in_place_threshold` | rad | 0.5 | Driving to Turning above this bearing error |
| `turn_exit` | rad | 0.1 | Turning to Driving below this, < `turn_in_place_threshold` |
| `drive_pid` | per m | kP 2.0, kI 0, kD 0.1, output 1 | forward from along-track error |
| `heading_pid` | per rad | kP 1.5, kI 0, kD 0.05, output 1 | turn while driving |
| `turn_pid` | per rad | kP 1.2, kI 0, kD 0.06, output 1 | turn in place (Turning, Aligning) |
| `max_forward`, `max_turn` | fraction | 0.8, 0.7 | output clamps, in (0, 1] |
| `forward_slew`, `turn_slew` | fraction/s | 2.0, 3.0 | increase limits, 0 = none |
| `min_forward`, `min_turn` | fraction | 0, 0 | static friction floors |
| `max_pose_age` | s | 0.25 | oldest usable robot pose |
| `max_link_age` | s | 0.25 | oldest usable link |
| `input_wait_timeout` | s | 2.0 | wait for first pose, or a landmark after activation |
| `input_loss_timeout` | s | 0.3 | tolerated stale pose after usable input |
| `default_timeout` | s | 10.0 | command timeout when `MotionOptions::timeout` is 0, > 0 |
| `max_landmark_age` | s | 1.5 | oldest usable observed landmark |
| `max_landmark_jump`, `max_landmark_jump_heading` | m, rad | 0.15, 0.25 | same-source change limit |
| `max_nominal_correction`, `max_nominal_correction_heading` | m, rad | 0.5, 0.5 | nominal to observed change limit |
| `landmark_follow_speed` | m/s | 0.3 | destination follow rate |
| `landmark_follow_turn_rate` | rad/s | 0.6 | destination follow turn rate |
| `max_dt` | s | 0.1 | dt clamp and gap threshold, > 0 |

Validation (`Navigator::valid`): every value finite and >= 0, the ordering
rules in the table, PID output limits > 0, `max_forward` and `max_turn` in
(0, 1], `min_forward <= max_forward`, `min_turn <= max_turn`,
`default_timeout > 0`, `max_dt > 0`. A Navigator with an invalid config fails
every command with `kInvalidCommand`.

### Tuning

1. Motor directions first, on blocks: `mixTank({0.3, 0})` must drive the robot
   forward, `mixTank({0, 0.3})` must turn it counterclockwise seen from above.
2. `turn_pid` with `goToRobotRelative({0, 0, kPi / 2})`: raise kP until the
   turn is quick, add kD to remove overshoot. Then `heading_pid` and
   `drive_pid` on straight `goTo` runs. Start every kI at 0; add a small kI
   with an `integral_limit` only for a persistent offset.
3. `min_forward` / `min_turn`: the smallest demand that moves the robot from
   rest on the field surface.
4. `max_forward`, `max_turn` and the slews for traction and brownout limits.
5. Tolerances: turning in place moves an origin that is not at the turning
   center by up to twice that offset, so `position_tolerance -
   arrive_distance` must exceed twice the origin to turning center offset
   plus coast, or Aligning and Turning can alternate until the timeout.
   `heading_tolerance` must be larger than the heading the robot can resolve.
6. `max_pose_age` must cover the poll period plus the response time plus the
   typical Pi pose age (for Navigatr: 20 ms poll + round trip + Pi age). There
   is no latency compensation: the controller acts on the pose as reported.
   On the host sim the defaults complete every test goal with up to 100 ms of
   pose latency.

## Fallback and source switching

Independent sources do not create backup localization by themselves. A
fallback must supply a real pose in a known frame.

1. While Navigatr is usable, the application may re-align odometry from a
   fresh Navigatr snapshot, preferably while the robot is stationary:
   `odometry.alignFrom(navigatr.latest(now), config.max_pose_age, now)`. It
   refuses invalid, disconnected, frame 0 or stale snapshots. It backdates the
   pose by the snapshot age but cannot add motion during that age, which is
   why stationary alignment is preferred.
2. On Navigatr loss the active command fails (`kInputLost`) or waits; it never
   continues on another source by itself.
3. The application switches explicitly with `navigator.setSource(odometry)`.
   An active command is cancelled with `kSourceChanged`; the old source gets
   `request({})`.
4. The application issues new commands. Field destinations stay meaningful
   only because step 1 put odometry in the Navigatr field frame.
   Landmark-relative commands fail with `kLandmarkUnsupported` on odometry.
5. Switching back is the same: `setSource(navigatr)`, then new commands. The
   Navigatr frame may have changed meanwhile.

Never align from stale data and never invent a pose when Navigatr disappears.
`tests/odometry_source_gtest.cpp` runs this procedure on the sim.

## Build and tests

Host (from the repository root):

```
cmake -S brain -B build-brain
cmake --build build-brain -j
ctest --test-dir build-brain --output-on-failure
```

`brain/CMakeLists.txt` is the host project for both Brain libraries. It uses a
system googletest when found, else fetches v1.14. Targets: `investigatr`
(library), `investigatr_sim` (host sim), `investigatr_tests`. On Windows with
MSYS2, put `/c/msys64/ucrt64/bin` first on `PATH` and add
`-G "MinGW Makefiles"` to the configure line.

PROS: `brain/gatr2_brain.mk` compiles the library sources in place; see
[communigatr.md](communigatr.md) for the import steps.

| Test file | Covers |
|---|---|
| `geometry_gtest.cpp` | wrap across +-pi, compose, rotated offsets |
| `pid_gtest.cpp` | reset, dt 0, first sample, integral clamp, wrapped derivative |
| `drive_gtest.cpp` | tank mixing, increase-only slew, reversal |
| `navigator_gtest.cpp` | convergence from several headings and across +-pi, final heading, heading-only goals, origin offset, overshoot, lateral offset, rotated landmark offset, landmark drift, shift and jump, nominal and observed, statuses, robot-relative, paths and missed pass radius, settling, replacement, cancel, timeout, missing, stale and lost input, frame change, source switch, slew, minimum outputs, validation |
| `odometry_source_gtest.cpp` | no pose before align, generations, integration, heading sensor, ages, alignment rules, explicit fallback |

## Limitations

- Tank drive only, driving forward only. Reversing to a destination is a
  turn around.
- Pass-through corners sharper than `turn_exit` stop forward motion and turn
  in place.
- A landmark waypoint is requested when it activates, so the robot waits
  there until the source reports the landmark.
- No obstacle routing, no motion profiling, no latency compensation.
- The Navigator is single threaded by design; the application owns locking.
- Not validated on the robot.
