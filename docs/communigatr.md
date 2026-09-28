# communiGATR

`brain/communiGATR` is the Brain side of the Pi link, brain link v4
([interfaces.md](interfaces.md#brain-link-v4)). It:

- opens and keeps a session with the Pi over USB or RS-485;
- uploads the Brain's robot profile and follows its application;
- reads the field map and field estimates in chunks and publishes them whole;
- gives planning and control the robot state and the field as an
  `investigatr::StateSource`, and forwards planned paths to the Pi viewer;
- places the robot, runs calibration and recovery actions on the Pi, and reads
  raw wheel travel for calibration;
- sends best-effort telemetry (VEX IMU tilt, movement status, wheel targets)
  for the Pi viewer and recordings;
- sums up what the link, profile, sensors, calibration and placement still
  wait for.

It does not plan routes ([investiGATR](investigatr.md)) or drive motors
([actuGATR](actugatr.md)). Robot setup, calibration procedures and the two
test programs are in [Brain setup](brain_setup.md).

Status: host tested against a fake Pi over a simulated RS-485 bus and a
simulated USB console. The PROS sources (`pros/`) compile only in the PROS
build. Nothing here has been validated on the robot, on the V5 USB console or
on real RS-485 hardware.

| Path | Contents |
|---|---|
| `include/communigatr/` | public headers; `pros_*.h` are PROS only |
| `src/` | portable sources, host and PROS |
| `pros/` | PROS only sources, never built on the host |
| `sim/` | host only fake Pi, RS-485 bus, USB console and test rig |
| `tests/` | host tests |

Namespace `communigatr`. Time is `Seconds` (double) everywhere; the portable
classes take the time as an argument and have no clock, thread or PROS
dependency.

## Layers

```
application (locaGATR, testing)
  actugatr::Motion, planner      use StateSource and PathSink only
  ProsLink                       PROS: poll task, bounded mutex, either transport
    LinkDriver                   StateSource + PathSink: SI units, status, frames, field
      Client                     protocol: session, scheduling, retries, profile,
                                 documents, placement, control, wheels, path,
                                 telemetry
        BytePort                 nonblocking bytes
          ProsUsbPort            V5 USB user console, NG1 lines, two I/O tasks
          ProsSerialPort         V5 smart port, generic serial, RS-485 adapter
  ProsVexImu                     Brain VEX IMU sample carried by GET_STATE, and
                                 roll and pitch for TELEMETRY
```

| Class or function | Header | Portable | Role |
|---|---|---|---|
| `BytePort` | `byte_port.h` | yes | nonblocking `read`/`write` interface |
| `Client` | `client.h` | yes | brain link v4 state machine; all I/O in `poll(now)` |
| `LinkDriver` | `link_driver.h` | yes | `StateSource` and `PathSink` over a client; no I/O |
| `RobotProfile`, `makeProfileDocument` | `robot_profile.h` | yes | robot profile in SI units, wire document, shared check |
| `DocAssembly` | `doc_assembly.h` | yes | one READ_DOC document, bounded and checked |
| `readinessOf`, `Readiness` | `readiness.h` | yes | readiness summary and decoded health |
| `encodeUsbLine`, `UsbLineDecoder` | `usb_line.h` | yes | NG1 line codec |
| `VexImuRecalibration` | `vex_imu_recalibration.h` | yes | Brain VEX IMU calibration gated by the Pi's stationary check |
| `robotAttitudeFromVex`, `setAttitude` | `attitude.h` | yes | VEX IMU roll and pitch to the robot frame; TELEMETRY attitude group |
| `WheelCalibration` | `wheel_calibration.h` | yes | per-wheel travel scale trials (application helper) |
| `StartupPlacement` | `startup_placement.h` | yes | once-per-start placement policy (application helper) |
| `LinkEvents` | `link_events.h` | yes | recovery history for displays (application helper) |
| `ProsLink` | `pros_link.h` | PROS | owns port, client and driver; poll task; both transports |
| `ProsUsbPort` | `pros_usb_port.h` | PROS | `BytePort` over the V5 USB console |
| `ProsSerialPort` | `pros_serial_port.h` | PROS | `BytePort` over a smart port in generic serial mode |
| `ProsVexImu` | `pros_vex_imu.h` | PROS | VEX IMU as the Brain bench IMU source |

## PROS use

A program creates one `ProsLink` and keeps it for its whole run. This is what
both test programs do, with the robot description from
[brain/robot/gatr2_robot.h](../brain/robot/gatr2_robot.h):

```cpp
#include "communigatr/pros_link.h"
#include "communigatr/pros_vex_imu.h"
#include "gatr2_robot.h"

std::unique_ptr<communigatr::ProsVexImu> g_vex;  // kept for the program's life
std::unique_ptr<communigatr::ProsLink>   g_link;

void initialize() {
    communigatr::LinkConfig config;
    if (gatr2_robot::kUseUsb) {
        config.transport = communigatr::Transport::kUsb;
    } else {
        config.transport  = communigatr::Transport::kSmartPort;
        config.smart_port = gatr2_robot::kLinkPort;
        config.baud       = gatr2_robot::kLinkBaud;
    }
    config.profile = gatr2_robot::profile();          // RobotProfile, SI units
    if (gatr2_robot::usesVexImu()) {
        g_vex.reset(new communigatr::ProsVexImu(gatr2_robot::kVexImuPort));
        communigatr::ProsVexImu* vex = g_vex.get();
        config.client.bench_imu      = [vex] { return vex->sample(); };
    }
    g_link.reset(new communigatr::ProsLink(config));
    g_link->start();  // false: task not created (errno set); call again later
}

// Any task, every loop:
//   const communigatr::ProsLinkStatus s = g_link->status();   // readiness, profile, stats
//   const investigatr::RobotState r = g_link->robot(communigatr::ProsLink::now());
//   if (r.valid()) { ... r.pose, r.age, r.frame ... }
```

- `start()` only creates the poll task. The task opens the transport and
  reopens it while it is closed; nothing in `initialize()` waits for the Pi.
- Pass `ProsLink::now()` (seconds since PROS start, from `pros::micros()`) to
  `robot()` and to every controller that uses the link. Ages and link timing
  are only meaningful on that clock.
- actuGATR's `Motion` takes the `ProsLink` as its `StateSource` and
  `PathSink` ([actuGATR](actugatr.md)).
- The planner and the robot state never depend on the VEX IMU: `ProsVexImu`
  only feeds the GET_STATE sample, and only for a profile whose IMU source is
  the Brain VEX IMU.

### LinkConfig

| Field | Default | Meaning |
|---|---|---|
| `transport` | `kUsb` | `kUsb`: V5 USB user console. `kSmartPort`: RS-485 adapter on a smart port |
| `smart_port` | 0 | `kSmartPort` only: V5 port 1..21 |
| `baud` | 115200 | `kSmartPort` only: must match the Pi serial resource |
| `poll_period_ms` | 2 | poll task period, at least 1 |
| `task_priority` | `TASK_PRIORITY_DEFAULT + 1` | poll task priority |
| `call_timeout_ms` | 20 | bounded mutex wait of every public call |
| `profile` | empty | `RobotProfile` to upload; when set it replaces `client.profile`. Empty and no `client.profile`: the Pi runs its own XML localization |
| `client` | `ClientConfig{}` | see [ClientConfig](#clientconfig); `bench_imu` runs in the poll task under the link mutex |
| `driver` | `LinkDriverConfig{}` | `accept_configured_anchor`, see [Robot state](#robot-state) |

The transport choice is explicit and has a matching Pi configuration:
`brain_profile_usb.xml` (`pros_usb_link`) for USB and `brain_profile_rs485.xml`
(`linux_serial_link`) for RS-485. There is no automatic choice or failover.
The transport never changes the robot profile or the localization.

### ProsLink calls

Every call takes the link mutex with a bounded wait (`call_timeout_ms`) and
copies values out. When the mutex is busy the call gives a safe answer and
counts it in `status().link.call_lock_misses`.

| Call | Effect | Busy answer |
|---|---|---|
| `start()` | create the poll task; true once started | |
| `now()` (static) | poll task clock, seconds | |
| `status()` | `ProsLinkStatus`, below | `busy` true; only `started`, `transport`, `port_open`, `link` and `usb` current |
| `robot(now)` | `investigatr::RobotState` | status `kNoLink` |
| `field(out)` | copy the newest complete field into `out` unless it already holds that generation | `out` unchanged; true when it holds a field |
| `reportPath(command, path)` | planned path to the Pi viewer, best effort | dropped |
| `place(pose)` | SET_POSE ticket, 0 when refused | 0 |
| `placement(ticket)` | `PlacementStatus` | `kPending` for a nonzero ticket (ask again) |
| `recalibrate()`, `reinitialize()`, `reinitImu()`, `restartAcquisition()` | CONTROL ticket, 0 when refused | 0 |
| `control(ticket)` | `ControlStatus` | `kPending` for a nonzero ticket |
| `requestWheels()` | READ_WHEELS ticket, 0 when refused (no session, another read pending) | 0 |
| `wheels(ticket)` | `WheelStatus` with the readings | `kPending` for a nonzero ticket |
| `wheelReadings()` | readings of the latest Ok reply of any read, by value | `busy` set, sequence 0, result NotReady |
| `setProfile(profile)` | replace the configured profile at run time | false |
| `profile()` | the configured `RobotProfile` | empty profile |
| `resubmitProfile()` | upload and apply again, clearing a settled rejection | false |
| `reportTelemetry(t)` | TELEMETRY for the Pi viewer, best effort, see [Telemetry](#telemetry) | dropped, false |

### ProsLinkStatus

| Field | Meaning |
|---|---|
| `busy` | the mutex was not taken in time; the fields below `port_open` are not current |
| `started`, `transport`, `port_open` | poll task running, selected transport, port open |
| `ready`, `connected`, `link_age` | session with a state reply; a reply within `link_timeout`; time since the last reply (infinity without a session) |
| `session`, `pi_instance`, `error`, `peer_version` | current session, Pi instance, last incompatibility and the Pi's version |
| `readiness`, `summary` | [readiness](#readiness) state and its details (IMU use, decoded health, calibration, Brain IMU calibrating, localized, pose valid) |
| `profile` | `ProfileStatus`: sync state, id, reason, detail, last result, bytes the Pi holds |
| `state` | latest GET_STATE Ok of this session, raw wire units |
| `telemetry_unsupported` | the Pi refused TELEMETRY in this session (an older Pi); nothing more is sent until a new session |
| `heading_valid`, `heading` | raw Pi heading, placed or not, while the Pi has a pose; used by wheel calibration |
| `field_sync` | transfer progress: complete map id, map bytes read, estimate id being read |
| `field_generation`, `map_id`, `estimate_id`, `field_age` | published field and time since its estimate completed |
| `stats`, `link`, `usb` | `ClientStats`, `ProsLinkStats`, `ProsUsbStats` counters |

`ProsLinkStats`: `poll_lock_misses` (poll cycles skipped), `call_lock_misses`
(busy answers), `opens` (transport open attempts), `serial_closes` (smart
port closed after repeated errors).

### Tasks and locks

| Task | Priority | Role |
|---|---|---|
| `communigatr` | `task_priority` | opens and reopens the transport, then `client.poll(now)` under the mutex every `poll_period_ms` |
| `communigatr-usb-tx` | `TASK_PRIORITY_DEFAULT` | USB only: writes NG1 lines |
| `communigatr-usb-rx` | `TASK_PRIORITY_DEFAULT` | USB only: reads stdin, decodes lines |

- Each poll cycle takes the mutex with a wait of one period. A miss skips
  the cycle and counts in `poll_lock_misses`.
- Opening runs outside the mutex: only the poll task touches the port, and a
  smart port open may wait about 100 ms for the port to settle.
- PROS deletes competition tasks (autonomous, opcontrol) at mode changes
  without releasing mutexes they hold. Because every wait is bounded, a
  deleted holder costs skipped cycles and busy answers, all counted, but the
  link cannot recover until the program restarts. Run loops that call the
  link in your own tasks, as both test programs do.
- `std::shared_ptr` is not shared between tasks: the PROS toolchain has no
  atomic reference counts. `ProsLink` hands out value copies only.
- The destructor stops the poll task within 250 ms and deletes it only as a
  last resort. Keep a started link for the program's life.

### HELLO nonce

The client asks for a nonce once per new HELLO. `ProsLink` mixes, with a
murmur3 finalizer, `pros::micros()` at construction, at `start()`, at each
transport open and at the call; battery voltage and current; an object and a
stack address; and a call counter. The nonce only has to differ from the Pi's
last four opening nonces. A collision costs one `kResultStale` and a new
nonce; a 0 or a repeat is replaced by the client.

## Transports

`BytePort` is the seam: `read` returns what is waiting (0 none, negative on
error) and `write` queues a whole frame or nothing. Neither blocks.

### USB (`ProsUsbPort`)

Each link frame travels as one NG1 line, `"NG1:" + uppercase hex + "\n"`
([USB NG1 envelope](interfaces.md#usb-ng1-envelope)). The line codec
(`usb_line.h`) is portable and follows the Pi's `pros_usb_link` parser: the
last marker in a line counts, text before it is ignored, a trailing `\r` is
stripped, the digits are uppercase, even in number and 2..256 long, and any
fault drops the whole line. A line longer than 516 characters is dropped.

- `open()` opens the PROS named stream `/ser/ngtr`, activates it, disables
  PROS output COBS, and starts whichever I/O task is not running. Any failed
  step returns false (errno set) and is retried by the poll task.
- COBS off applies to all PROS output in the program, so console text
  (`printf`) arrives plain and the Pi skips it. Each NG1 line is written with
  one call, which the kernel keeps whole between other console output.
- Input comes from stdin. Kernel 4.2.2 appends a NUL after the data it
  returns, so one byte of the read buffer stays free.
- The client's `write` only pushes onto a one-slot queue; a full slot is a
  failed attempt that the client counts. Decoded frames wait in a four-slot
  queue. Both queues are lock-free, one producer and one consumer.
- A queued frame older than `clamp(response_timeout * 800, 1, 10000)` ms (48
  ms by default) is dropped unwritten, so a stalled writer never replays an
  expired request.
- `SERCTL_NOBLKWRITE` has no effect in kernel 4.2.2, and USB writes can block
  while no host is reading. Only the transmit task makes USB calls, so a
  blocked write stalls that task alone: the poll task, the program and
  battery-only operation carry on. Program `printf` calls may still block the
  task that makes them while no host reads; keep console output low or off.
- The port stays open across unplugging; the Brain sees timeouts until the
  cable is back. The Pi reopens its device at most once a second and does
  not answer the first request after reopening (it still applies it); the
  next request, a resend or a new poll, gets an answer.

### RS-485 (`ProsSerialPort`)

- `open()` enables generic serial on the smart port, then sets the baud and
  flushes, retrying every 2 ms for up to 100 ms while the port settles. It
  returns false when PROS refuses, and can be called again.
- `read` returns only bytes already received. `write` is all or nothing: the
  whole frame must fit the output FIFO, else it is refused (not an error).
- After 50 consecutive read or write errors the port counts as closed
  (`serial_closes`), and the poll task reopens it at most once a second.
- The V5 smart port is assumed to switch its own RS-485 direction. Not
  validated.

## Robot profile

The Brain owns the robot's localization description and sends it to the Pi as
a typed document; the Pi never receives XML or file paths
([Robot profile exchange](interfaces.md#robot-profile-exchange)).

```cpp
#include "communigatr/robot_profile.h"

communigatr::TrackingWheel forward;
forward.encoder_port   = 0;        // naviGATR encoder port: 0 J2, 1 J3, 2 J4
forward.radius         = 0.024;    // PROVISIONAL
forward.counts_per_rev = 4000;     // encoder shaft
forward.x = 0.0; forward.y = 0.15; // contact point, robot frame (PLACEHOLDER)
forward.angle          = 0.0;      // measures forward travel

communigatr::TrackingWheel sideways = forward;
sideways.encoder_port = 1;
sideways.x = 0.15; sideways.y = 0.0;   // PLACEHOLDER
sideways.angle = investigatr::kPi / 2; // measures leftward travel

communigatr::RobotProfile p;
p.topology       = communigatr::LocalizationTopology::kTwoWheelImu;
p.wheels         = {forward, sideways};
p.imu_source     = communigatr::ImuSource::kBrainVex;
p.vex_smart_port = 1;
p.footprint      = {0.23, 0.23, 0.23, 0.23}; // front, back, left, right (PLACEHOLDER)
```

The values above are the placeholders of the current bench; the real ones
live in `gatr2_robot.h` ([Where settings live](brain_setup.md#1-where-settings-live)).

### Fields

| `RobotProfile` | Wire | Rule |
|---|---|---|
| `topology` | topology | see the table below |
| `wheels` | wheel records | 2 or 3 by topology |
| `imu_source` | imu_source | `kNone`, `kPico`, `kBrainVex` |
| `imu_port` | imu_port | Pico IMU port with `kPico` (0 on HAT v2), else 0 |
| `vex_smart_port` | vex_smart_port | 1..21 with `kBrainVex`, else 0 |
| `imu_invert` | imu_flags bit 0 | `kPico` only: Pico yaw sign flipped |
| `calibration.window` | calibration_window_ms | Pi IMU bias stationary window; 0 = Pi default, else 0.5..20 s |
| `calibration.still_rate` | still_rate_cdps | gyro rate still counted as still, rad/s; 0 = default, else 0.1..20 deg/s |
| `calibration.still_travel` | still_travel_um | per-wheel travel still counted as still over the window; 0 = default, else 20..5000 um |
| `footprint` | footprint_*_um | distance from the origin to each side, 0..2 m, nonzero length and width |
| `cameras` | camera records | up to 4, distinct Pi camera slots, mount within 2 m and 360 deg |

| `TrackingWheel` | Wire | Rule | Kind |
|---|---|---|---|
| `encoder_port` | encoder_port | distinct per wheel; the Pi checks it is wired | encoder |
| `counts_per_rev` | counts_per_rev | encoder shaft counts, 1..1000000 | encoder |
| `reversed` | flags bit 0 | encoder polarity: positive counts mean travel against `angle` | encoder |
| `gear_ratio` | gear_micro | encoder turns per wheel turn, 0.1..10 | encoder |
| `radius` | radius_um | 1..200 mm | geometry |
| `x`, `y` | x_um, y_um | contact point, robot frame, within 1 m | geometry |
| `angle` | angle_mdeg | measuring direction, CCW from +x, within 360 deg | geometry |
| `travel_scale` | travel_scale_ppm | measured distance correction, 0.9..1.1; 1.0 uncalibrated | empirical |

The Pi applies each kind once: counts per revolution, gearing and polarity in
the encoder sensor; radius and direction in the observation model; the travel
scale as a factor on that travel. A measured travel test sees only the product
of radius, gearing and travel scale, so keep radius and gearing physical and
put the measured correction in `travel_scale`; its range is narrow on purpose.

| Topology | Wheels | IMU source | Observability |
|---|---|---|---|
| `kTwoWheelImu` | 2 | `kPico` or `kBrainVex` | independent directions |
| `kTwoForwardWheelImu` | 2 | `kPico` or `kBrainVex` | both angles 0 or 180 deg; no sideways travel assumed |
| `kThreeWheel` | 3 | `kNone` or `kPico` | the three wheels resolve x, y and rotation |

A Brain VEX IMU with three wheels is refused (`ImuCombination`): its samples
run on the Brain clock and cannot be fused as an independent observation. The
Pico IMU chip (BNO08X or ASM330) is fixed by the Pico firmware build;
`kPico` only selects the Pico IMU port.

### Documents and the shared check

- `makeProfileDocument(profile)` converts to wire units (micrometers,
  millidegrees, ratios x 1e6), rounding to the nearest unit, and runs the
  shared `translagatr::validateRobotProfile`, the same check the Pi runs first. The
  result carries the bytes, or the first failure as a `ProfileReason` and the
  wheel or camera index. A value that does not fit the wire reports the
  reason of its group.
- `toProfileDoc` does the same into a `translagatr::RobotProfileDoc`.
- `profileId(doc)` is the document's CRC-32, the id the Pi reports.
- `profileReasonName(reason)` gives a short name for the screen.
- A profile that fails the Brain check is `ProfileSync::kInvalid` with its
  reason and is never sent. The Pi's own capability checks (ports wired, IMU
  port present, Brain IMU mailbox configured, camera slots) can still reject
  it.

## Profile sync

With a profile configured the client keeps the Pi running exactly that
document:

```
first GET_STATE of a session
  Pi runs this id   -> kApplied, nothing uploaded
  otherwise         -> kWriting: PROFILE_WRITE chunks of up to 106 bytes
                    -> kApplying: PROFILE_APPLY, again every pending_retry on Pending
                    -> kApplied: a state showing it applied (after an APPLY Ok,
                       the next state poll)
                       or kRejected (ProfileRejected: the Pi's reason and detail)
```

| `ProfileSync` | Meaning |
|---|---|
| `kNone` | no profile configured; the Pi uses its XML localization |
| `kInvalid` | failed the Brain side check; never sent |
| `kWaiting` | no state in this session yet |
| `kWriting` | sending the document |
| `kApplying` | PROFILE_APPLY sent, the Pi answered Pending, or Ok and no state shows it yet |
| `kApplied` | a state reply shows the Pi running this profile |
| `kRejected` | refused; settled until the session changes or `resubmitProfile()` |

- Only a state reply makes a profile `kApplied`. An APPLY Ok can arrive while
  the newest state still predates the Pi's swap and carries the old
  profile's epoch and placement; the client sends no more APPLY and waits for
  the next state poll. So whenever `profileApplied()` is true, the state that
  `robot()` and readiness read describes this profile. A state after the Ok
  that shows another profile restarts the upload.
- The upload resumes where the Pi's staging ends (`received`) within a
  session, and starts over when the Pi reports a gap or other bytes.
- A state poll goes between profile requests after a timeout. InvalidArgument
  three times in a row (staging lost or replaced) settles as `kRejected` with
  that result, so there is no retry storm.
- The Pi is authoritative: a state that shows another profile applied, or
  this one not applied, restarts the upload unless a rejection is settled.
- A rejection is remembered by the Pi per id. `resubmitProfile()` with the
  same document gets the same answer; change the profile to clear it.
- `profileConfigured()`, `profileApplied()` and `profile()` report it.

### Runtime change

`setProfile(profile)` (on `ProsLink` and `LinkDriver`; `Client::setProfile`
takes the document) replaces the configured profile, for example when
locaGATR applies a wheel calibration:

- A profile the Brain check refuses returns false and the running profile
  stays. With no valid profile running, the refused one is kept as `kInvalid`
  so its reason shows.
- The same document again returns true and changes nothing.
- A new id goes through the sync above. The Pi applies it as a new profile:
  a new odometry epoch, history cleared, unplaced, any unapplied SET_POSE
  withdrawn. The program must place the robot again; nothing places
  automatically.

## Field map and estimate

The Pi publishes the field map (kind 1, `doc_id` = `map_id`) and field
estimate snapshots (kind 2) as documents; GET_STATE names the current ids
([Field documents](interfaces.md#field-documents-read_doc)).

**Map**
- Read when the state's `map_id` is nonzero and differs from the complete map
  held. Chunks of up to 96 bytes are assembled by `DocAssembly` into a buffer
  of `kFieldMapMaxLen` (128 objects, 3608 bytes) allocated once.
- Every chunk must name the same kind and id, repeat the first chunk's
  `total_len` and `crc32`, start where the previous one ended and stay inside
  `total_len`. The complete document must match its CRC, have CRC-32 equal to
  `map_id`, and pass `validateFieldMap`.
- The complete map is cached by `map_id` across sessions and Pi restarts (not
  across Brain restarts). A new map id replaces it.

**Estimate**
- Read when the map is current, the state's `estimate_id` is new, and
  `field_period` (0.5 s) has passed since the last read started; a read in
  progress continues.
- It must name the held map and pass `validateFieldEstimate` against it: the
  same count and ids in map order, fixed objects nominal, source none exactly
  when not valid.
- Stale (the Pi no longer holds that id) restarts from the newest id at once.
  Three failures of one kind in a row wait `transfer_backoff` (1 s).
  Unavailable waits `field_period`.
- Estimate ids restart in each Pi process, so the client keys the published
  estimate by (`pi_instance`, `estimate_id`).

**Publication**
- A map and an estimate checked against it are published together as one
  `FieldPublication` generation. A partial document is never visible.
- A session loss drops documents in progress and keeps the published pair.

### Field for planning (`LinkDriver::field`)

`field(out)` copies the newest publication into an `investigatr::Field`:

- mm to m, centidegrees to rad wrapped to (-pi, pi];
- the boundary, every object with kind, obstacle, estimated and reference
  flags, nominal pose, collision box, estimate source, validity and pose;
- `generation` changes with every publication, and `out` is left alone when
  it already holds it;
- `frame` numbers the estimate's anchor (`pi_instance`, `session`,
  `odometry_epoch`, `anchor_revision`) the same way `robot()` numbers the
  robot's, so `field.frame == robot.frame` exactly when the estimate is
  under the robot's current anchor;
- `received_at` is when the estimate completed.

Observed ages. The Pi takes a new estimate only when its content changes, so
an estimate's `age_ms` is the age at the snapshot. The client records the
send time of the last state poll that did not show the id yet
(`snapshot_after`), and the driver reports `age_ms + (completed_at -
snapshot_after)` at `received_at`, an upper bound. An estimate that was
already current at the first state of a session has an unknown snapshot time
and an infinite observed age, so consumers that bound the age treat it as
stale until the Pi takes a new one.

Landmark counts come from the Pi's field definition: 0 to 128 objects, one
READ_DOC chunk for small maps and up to 38 for the largest.

## Robot state

`LinkDriver::robot(now)` (and `ProsLink::robot`) returns an
`investigatr::RobotState`. The status is the first unmet condition:

| Status | Condition |
|---|---|
| `kNoLink` | not connected: no session with a state reply, or no reply within `link_timeout` |
| `kNoProfile` | a profile is configured and not applied |
| `kCalibrating` | Pi calibration running, waiting for stillness, or waiting for data |
| `kUnplaced` | not localized, the anchor is not accepted, or a placement is in flight |
| `kNoPose` | the Pi has no pose or its age is unknown |
| `kValid` | pose, age and frame are filled |

- Units: mm to m, centidegrees to rad wrapped to (-pi, pi].
- Age: `robot_age_ms / 1000 + round_trip + (now - received_at)`, where
  `round_trip` is the matched GET_STATE's write to its reply. No clock value
  of one device is subtracted from another's.
- Frame: a number that changes whenever (`pi_instance`, `session`,
  `odometry_epoch`, `anchor_revision`) changes: a Pi restart or reset, a new
  Brain session, a new odometry epoch on the Pi (new profile,
  reinitialization, source restart), or a placement. Any change is a
  coordinate discontinuity; actuGATR fails a running command with
  `kFrameChanged`.
- Anchors: a pose anchored by a Brain SET_POSE (any session of this Pi
  instance) is accepted. `LinkDriverConfig::accept_configured_anchor` also
  accepts the Pi's configured initial placement; it is off by default.
- A failed Pi calibration (`kCalibrationFailed`) does not block the pose;
  readiness shows it.

## Readiness

`readinessOf(client, now, accept_configured_anchor)`, also
`LinkDriver::readiness(now)` and `ProsLinkStatus::readiness`, gives the first
unmet condition in this order. Both programs show it; it never acts.

| `Readiness` | Condition | What it takes |
|---|---|---|
| `kConnecting` | no state reply yet since start | the Pi service, the link |
| `kReconnecting` | had one; link or session lost now | automatic |
| `kProfileRejected` | refused by the Brain check or the Pi | fix the profile (reason and detail in `ProfileStatus`) |
| `kProfilePending` | configured profile not applied yet | automatic |
| `kSensorsUnavailable` | profile encoders not fresh, or the profile's IMU source not fresh or failed | Pico link, encoders, IMU |
| `kSensorsInitializing` | Pico IMU starting or aligning, Brain VEX IMU calibrating, or placed with no pose yet | hold still |
| `kWaitingStill` | Pi IMU calibration saw movement | hold the robot still |
| `kCalibrating` | Pi IMU calibration collecting a window, or waiting for data | hold still |
| `kCalibrationFailed` | no qualified window within the Pi's bound | hold still, then recalibrate |
| `kNeedsPlacement` | not placed, anchor not accepted, or a placement in flight | place the robot |
| `kReady` | placed pose from the Pi | |

- Only the configured profile's IMU source counts. An absent or failed Pico
  IMU never holds back a Brain VEX IMU profile; a three-wheel profile without
  IMU checks none. Without a Brain profile the Pi XML decides and the IMU is
  not checked.
- `LinkReadiness` also carries the IMU use, the decoded health bits
  (`HealthBits`: encoders fresh, IMU fresh, vision, bias calibrated, Pico
  link, IMU initializing, IMU failed, stationary), the calibration state
  (`calibrationName`), whether the Brain IMU is calibrating, localized and
  pose valid.

## Placement

`place(pose)` (`ProsLink`, `LinkDriver`) sends the robot origin's field pose
as SET_POSE, rounded to mm and centidegrees with the heading in (-18000,
18000]. It returns a ticket, or 0 when refused: no session with a state
reply, the configured profile not applied, another placement pending, or a
pose that is not finite or outside the wire range. Only the latest ticket is
tracked.

| `PlacementResult` | Meaning |
|---|---|
| `kPending` | queued, in flight, or Ok and waiting for a state that shows its anchor |
| `kApplied` | a state after the Ok reports the Ok's anchor revision |
| `kRejected` | error result in `PlacementStatus::result` (NotReady when the Pi has no applied profile) |
| `kTimedOut` | `placement_attempts` or `placement_deadline` used up; outcome unknown, it may still apply |
| `kSessionLost` | the session ended first; never resent |

- SET_POSE is resent with the same bytes on timeout, before any other
  request (a newer id would make it stale on the Pi), and after
  `pending_retry` on Pending. With a Brain bench IMU one state poll goes
  between Pending resends, so the Pi keeps receiving IMU samples.
- The robot reads `kUnplaced` from submission until the ticket settles, so a
  pose from before the placement is never used after it.
- Program start and recovery are different: `StartupPlacement` (application
  helper) allows one automatic placement at the configured start pose per
  program start, after the link, profile and sensors are ready. After a
  reconnect, a Pi restart, a profile change or a reinitialization the
  program shows Needs placement and waits for the operator
  ([Placement](brain_setup.md#6-placement)).

## Control: calibration and recovery actions

| Call | Action | On the Pi | Pose |
|---|---|---|---|
| `recalibrate()` | 1 | stationary check, then restart the IMU bias calibration; Ok with calibration none for a Brain VEX IMU profile | holds |
| `reinitialize()` | 2 | stationary check, then a localization reset: new odometry epoch, unplaced | placement needed |
| `reinitImu()` | 3 | Pico IMU reinitialization, then recalibration; needs a Pico IMU profile and v2 Pico firmware | holds |
| `restartAcquisition()` | 4 | stationary check, then the Pico zeroes its counters under a new acquisition epoch; localization rebases | holds |

Each returns a ticket, or 0 when refused (no session with a state reply, or
another control pending). `control(ticket)` (`controlStatus` on `Client`)
gives:

| `ControlResult` | Meaning |
|---|---|
| `kPending` | queued, in flight, or the Pi answered Pending (Pico or calibration working) |
| `kOk` | done; `calibration` is the Pi calibration state after it |
| `kFailed` | ran and failed; `detail` is a `translagatr::ControlDetail` (PicoLink, ImuAbsent, ImuUnused, PicoRefused, TimedOut, Calibration) |
| `kNotStationary` | the robot moved in the Pi's stationary window; nothing started |
| `kNotReady` | the Pi has no applied robot profile |
| `kRejected` | another error result, in `result` |
| `kTimedOut` | before any reply: no reply to `control_attempts` sends or within `control_deadline` of the first send. After a reply: no final result within `control_wait` (20 s) of the first send. Outcome unknown |
| `kSessionLost` | the session ended first; never resent |

- A CONTROL is resent with the same request id and bytes, never a new id.
  Before the first reply the Pi may not have it, so a timeout resends it at
  once, before any other request. Once the Pi has answered it holds the
  record and answers that id after newer ones, so a timeout or a failed write
  resends it after `control_retry` (0.1 s) with state polls in between, and
  while the Pi answers Pending it is asked again every `control_retry`. Only
  `control_wait` or a session loss ends it then: a pulled cable costs time,
  not the outcome. The Pi deduplicates it, so a lost reply never runs the
  action twice or restarts a Pico command.
- The stationary condition is the Pi's (over the last 300 ms, every profile
  encoder moved less than 1 mm and, with a Pico IMU, the yaw rate stayed
  under 2 deg/s). The Brain never assumes stillness from zero motor
  commands.

### Brain VEX IMU recalibration

VEX firmware calibrates the Brain VEX IMU (`pros::Imu::reset`); the Pi judges
stillness. `VexImuRecalibration` joins the two so the calibration never
starts while the robot moves:

1. `begin(link.recalibrate())`: CONTROL recalibrate. For a Brain VEX IMU
   profile the Pi runs its stationary check and calibrates nothing itself.
2. The Pi's Ok: `update()` returns `kStart`; the program calls
   `ProsVexImu::recalibrate()` and passes its result to `started()`.
   NotStationary ends the run as `kMoving` with nothing started; any other
   answer, or none, as `kRefused`.
3. `kCalibrating` until the IMU sample stops reporting calibrating, then
   `kDone` with a valid sample, else `kImuFailed` (also when the IMU does not
   report calibrating within 1 s of the start, or calibrates longer than the
   limit, 10 s by default).

```cpp
communigatr::VexImuRecalibration g_recal;   // one per program

// On the button:
if (!g_recal.begin(g_link->recalibrate())) { /* refused: no link, or a control pending */ }

// Every loop, in the program's task (recalibrate() may block about 1 s):
if (g_recal.active()) {
    const auto s = g_recal.update(g_link->control(g_recal.ticket()), g_vex->sample(),
                                  communigatr::ProsLink::now());
    if (s == communigatr::VexRecalibrationState::kStart) {
        g_recal.started(g_vex->recalibrate(), communigatr::ProsLink::now());
    }
}
// Screen: communigatr::toString(g_recal.state())
```

- `ProsVexImu::recalibrate()` alone starts the calibration at once with no
  stillness check; programs call it only from `kStart`.
- The check covers the moment before the start. The VEX calibration then
  takes about 2 s, and the helper does not watch the wheels during it; keep
  the robot still until it shows done. Samples are invalid meanwhile, so the
  Pi invalidates the pose (a used sensor lost, spec rule in
  [Brain setup section 8](brain_setup.md#8-sensor-loss-and-recovery)); place
  the robot again afterwards.
- With a Pico IMU profile, CONTROL recalibrate itself is the recalibration
  (the Pi's windowed bias calibration); the helper is not used.
- The Pi applies no bias to the VEX source, so the calibration is never
  applied twice.

## Wheel readings

`requestWheels()` asks for one READ_WHEELS read (scheduled after control and
the due state poll) and returns a ticket, or 0 when refused: no session, or
another read pending. `wheels(ticket)` (`wheelStatus` on `Client`) follows it
until it settles; only the latest ticket is tracked.

| `WheelResult` | Meaning |
|---|---|
| `kPending` | queued or in flight |
| `kOk` | answered; `WheelStatus::readings` holds this read |
| `kRejected` | error result in `WheelStatus::result`: NotReady without an applied profile, Unavailable on a Pi config without a Brain profile |
| `kTimedOut` | no reply to the one send (lost request or reply, failed write, cable out); ask again |
| `kSessionLost` | the session ended first |

A read is sent once and never resent; a caller that needs it asks again with
a new ticket.

| `WheelReadings` | Meaning |
|---|---|
| `sequence` | Ok replies so far, 0 = none |
| `received_at`, `round_trip` | poll time of the reply and its request's round trip |
| `result` | in `wheelReadings()`: the last reply's result |
| `count`, `wheels[]` | one `translagatr::WheelReading` per profile wheel, in profile order |
| `busy` | `ProsLink::wheelReadings()` only: the link was busy and nothing above is current |

`wheelReadings()` keeps the readings of the latest Ok reply of any read, for
display. A calibration step uses the readings of its own ticket.

A reading has the port, fresh and valid flags, the encoder discontinuity
counter, raw counts, `travel_um` (counts per revolution, gearing, polarity and
radius applied, travel scale not) and the Pi-side age. `WheelCalibration`
(application helper) turns two readings around a measured push into a
proposed `travel_scale`, with its rejection rules
([Per-wheel calibration](brain_setup.md#4-per-wheel-calibration)).

## Path reports

`reportPath(command, path)` (`PathSink`) sends the plan's vertices for the Pi
viewer: every segment start and translation end, in field mm, repeats
removed. More than 13 points are thinned evenly to 13 with the first and last
kept. An empty path clears the report. It is best effort: one attempt, a newer
report replaces an unsent one, and it is dropped without a session. It never
changes localization.

## Telemetry

`reportTelemetry(t)` (`Client` and `ProsLink`) sends a
`translagatr::BrainTelemetry` (TELEMETRY, op 12,
[interfaces.md](interfaces.md#brain-link-v4)) for the Pi viewer and capture
recordings. The Pi records and shows it; its localization, placement and
profile never read it, and nothing on the Brain waits for it.

| Group (flag) | Fields | Source |
|---|---|---|
| attitude (`kTelemetryAttitude`) | `roll_cdeg`, `pitch_cdeg`, robot frame | `ProsVexImu::attitude()` through `robotAttitudeFromVex` |
| motion (`kTelemetryMotion`) | command id, state, reason, plan mode, segment, destination, applied chassis command, errors, drive fault | `actugatr::telemetryOf`, see [actuGATR telemetry](actugatr.md#telemetry) |
| wheels (`kTelemetryWheels`) | `wheel_count`, `wheel_rpm_x10` | motor velocity targets, same place |
| target (`kTelemetryTarget`, bit 3) | not a group: marks the motion group's `target_*` as a resolved destination | set by `telemetryOf` exactly when `MotionStatus::has_destination` |

Without `kTelemetryTarget` the motion group's `target_x_mm`, `target_y_mm`
and `target_heading_cdeg` are 0 and mean nothing (idle, waiting for a
reference); with it, (0, 0, 0) is a real target at the field origin. The bit
is only meaningful with `kTelemetryMotion`. A Pi that predates the bit keeps
it in `flags` and ignores it.

`stamp_ms` is the Brain clock (`pros::millis()`) when the telemetry was
assembled. A group whose flag is clear is zero on the wire and ignored. The
attitude group is absent while the VEX IMU calibrates, is missing or reports
an error, and always with a Pico IMU profile (no VEX IMU is read then).

Rules:

- Best effort: one attempt, never resent; a newer report replaces an unsent
  one (`telemetry_replaced`).
- On average at most one send per `telemetry_period` (0.1 s). Sends sit on a
  grid of that period. A send up to half a period late keeps the grid, so two
  sends can be as close as half a period, and a program that reports once per
  period never has a report replaced by drift.
- Lowest priority, after the path report, and only ever in the first request
  slot after a state reply, the earliest point of the poll's idle window. A
  report that falls due in the middle of the window waits for the next reply.
- On time it is not a waiting transfer: it goes only when the state poll is
  not due and nothing else waits, and never counts toward the four-poll rule.
- Overdue: when a state exchange plus the 5 ms gap fills the whole 20 ms
  poll period (a round trip of about 15 ms or more), the poll is due in
  every slot and the on-time slot never comes. A report half a period past
  its grid time then takes the first slot after a state reply from the due
  poll or a waiting transfer (`telemetry_overdue`). That is at most one such
  exchange per 1.5 periods and at most one exchange between two polls; a
  transfer that already yielded its four polls still goes first. Without
  this rule a slow link sent no telemetry at all while `telemetry_replaced`
  grew.
- Dropped at once (false, `telemetry_dropped`): no session, flag bits other
  than the four above, a wheels group of more than 6 wheels, or the Pi
  refused TELEMETRY in this session.
- An older Pi answers UnsupportedOp; a Pi that refuses the body answers
  InvalidArgument. Either sets `telemetryUnsupported()` for the session and
  counts `telemetry_refused`; the session, polling and every other request
  go on. Unlike every other op, UnsupportedOp to TELEMETRY is not a terminal
  incompatibility. From another `pi_instance` it is a Pi restart. A new
  session tries once more.
- A pending report is dropped with its session; a report of one session never
  goes out in the next. A report still unsent two periods after it was made
  (a link outage) is dropped too: it would reach the Pi looking fresh.

**Cost.** With one request outstanding, a TELEMETRY exchange (68-byte request,
19-byte reply) holds the link for its round trip plus the 5 ms gap, and the
next state poll moves back by about that much: 10 to 15 ms at the default
fake timing, more on a slower link, where one exchange takes as long as a
poll's. Host runs only (V5 timing is not measured), a report every 100 ms as
the programs send, polls counted over the same time without and with
telemetry:

| Setup (host) | Telemetry sent | State polls | Longest wait for a send |
|---|---|---|---|
| RS-485 bus, default timing, 5 s | 50 of 50 (on time) | 243 to 217 (11% fewer) | 116 ms |
| fake USB, default timing, 5 s | 50 of 50 (on time) | 243 to 224 (8% fewer) | 114 ms |
| fake USB, turnaround 8 ms, 5 s | 50 of 50 (on time) | 243 to 205 (16% fewer) | 118 ms |
| fake USB, turnaround 11 ms, 5 s | 32 of 50 (overdue) | 237 to 205 (14% fewer) | 166 ms |
| fake USB, turnaround 25 ms, 5 s | 29 of 50 (overdue) | 142 to 113 (20% fewer) | 180 ms |
| RS-485 bus, reply after 8 ms, 5 s | 32 of 50 (overdue) | 232 to 200 (14% fewer) | 171 ms |
| real Pi runtime (`brain_link_e2e` harness, `brain_profile_usb.xml`), 2 ms each way, 10 s | 100 of 100 | 497 and 500 (no change) | 124 ms |
| same, 3, 4, 5 or 6 ms each way | 63 of 100 (overdue) | 489 to 437 (11% fewer) | 160 ms |
| same, 1 to 4 ms each way, random per line | 79 of 100 | 490 to 431 (12% fewer) | 162 ms |

At the default timing the longest gap between polls grew from 21 ms to 32 ms
(RS-485) and 30 ms (USB). Before the overdue rule the slow rows sent 0 or 1
reports in the whole run, and the random 1 to 4 ms row waited up to 370 ms
(5 waits over 250 ms). With it every row stays under the Pi's 250 ms
attitude staleness limit. The Brain VEX IMU samples ride on GET_STATE and
slow by the same fraction as the polls.
`gatr2_robot::kSendTelemetry = false` (or a longer `telemetry_period`)
removes or lowers the cost.

**Programs.** Both send every `gatr2_robot::kTelemetryPeriodMs` (100 ms) when
`kSendTelemetry` is set:

- operaGATR, from its background task: motion and wheels from the drive
  task's last published snapshot (`ProsDrive::status`, a copy under the drive
  mutex; nothing is held while the telemetry is built or sent), plus the VEX
  IMU attitude. The snapshot is at most one drive period (10 ms) older than
  `stamp_ms`.
- locaGATR: the attitude only, and only with the VEX IMU profile.

Both show the robot-frame tilt on the Brain screen and whether the Pi refused
TELEMETRY.

### Attitude

```cpp
#include "communigatr/attitude.h"

const communigatr::VexAttitude   vex = g_vex->attitude(); // PROS roll and pitch, degrees
const communigatr::RobotAttitude a =
    communigatr::robotAttitudeFromVex(vex, gatr2_robot::kVexImuMountYawDeg);
translagatr::BrainTelemetry t;
communigatr::setAttitude(t, a); // flag and centidegrees; invalid clears the group
t.stamp_ms = pros::millis();
g_link->reportTelemetry(t);
```

- `ProsVexImu::attitude()` reads `pros::Imu::get_roll()` and `get_pitch()`.
  It is invalid while the IMU calibrates, is missing, reports an error or
  returns `PROS_ERR_F`.
- `robotAttitudeFromVex(roll_deg, pitch_deg, mount_yaw_deg)` builds the up
  direction from the IMU angles, rotates it by the mount yaw about +z and
  reads roll and pitch back in the robot frame. It is exact, with no small
  angle approximation: roll in (-180, 180], pitch in [-90, 90] degrees.
  Non-finite input is invalid.
- Convention (translaGATR): roll about robot +x (forward), positive left side
  up; pitch about robot +y (left), positive nose down. The VEX angles are
  assumed to follow the same convention in the IMU's own frame. This is
  UNVERIFIED on hardware; the bench check is in
  [Brain setup](brain_setup.md#10-telemetry-and-attitude).
- The mount yaw is `gatr2_robot::kVexImuMountYawDeg`: the direction of the
  IMU's +x axis in the robot frame, CCW from forward, usually 0, 90, 180 or
  270. PLACEHOLDER. The IMU must lie flat. It changes only the telemetry,
  never the heading or localization.
- Only tilt: no height or vertical position is derived.

## Reconnect and recovery

A timeout means the link was lost, not that a device restarted. Restarts are
seen from identities: the Pi's `pi_instance`, the session, the odometry
epoch, the anchor revision, and on the Pi the Pico's boot and epochs.

| Event | What the Brain does |
|---|---|
| USB cable out and back in | the session stays; status reads not connected after `link_timeout`; replies resume in the same session. With a Brain VEX IMU profile an outage over 250 ms stops the IMU samples, so the Pi invalidates the pose: placement needed |
| RS-485 port errors | the port closes after 50 errors in a row and reopens at most once a second |
| Pi silent (process stalled) | as a cable cut; the session is never dropped for silence alone |
| Brain program restart | new client, new session, no command carried over; the first state shows the profile applied, so nothing is uploaded. With a Brain VEX IMU profile the IMU samples stopped during the restart, so the pose is invalid: placement needed (startup placement does it at program start) |
| Pi restart or reset | the new `pi_instance` (or UnknownSession) ends the session: in-flight placement and control become `kSessionLost` and are never resent; new session, profile uploaded again, placement needed |
| Profile upload cut | resumes at the Pi's `received` in the same session, from 0 in a new one; a Pi restart mid-upload starts over |
| Map or estimate transfer cut | the partial document is dropped; the read starts again; the published field stays |
| Pico restart or acquisition restart | a used sensor restarted: the Pi invalidates the pose (new odometry epoch, unplaced); placement needed |
| Unsupported version or op | terminal error: the session is dropped and HELLO repeats every `hello_backoff`, no retry storm. Except TELEMETRY: an older Pi's UnsupportedOp stops telemetry for the session and keeps the session |

What never happens automatically:
- a movement command resuming after any of these (actuGATR fails it);
- a placement after a reconnect, Pi restart, profile change or
  reinitialization;
- a resend of a SET_POSE or CONTROL from a lost session;
- a recalibration while the robot moves: the Pi refuses CONTROL recalibrate,
  and `VexImuRecalibration` starts the Brain VEX IMU calibration only after
  the Pi's Ok.

A CONTROL the Pi has answered survives a cable pull in the same session: its
request id is asked again until the final result, up to `control_wait`.

## Timing

**Scheduling.** One request outstanding. Before each send the client drains
its receive buffer and resets its frame reader. Priority:

1. HELLO while no session is open.
2. Placement.
3. Control.
4. Profile sync.
5. The due state poll (every `state_period`, 20 ms).
6. READ_WHEELS.
7. Map chunk.
8. Estimate chunk.
9. Path report.
10. Telemetry, only in the first slot after a state reply.

Items 6 to 10 go while the state poll is not due. When exchanges outlast the
poll period the poll is always due, so a waiting transfer goes after 4 due
polls in a row instead of never, and a telemetry report half a period late
goes ahead of 5 to 9 in the first slot after a state reply (see
[Telemetry](#telemetry)). In host tests on the default RS-485 bus,
state polls stay at most 43 ms apart while a 128-object map moves.

**Timeout per request.**

```
T = response_timeout + byte_time * max(0, request_frame + max_reply_frame(op) - 85)
```

With the defaults (60 ms, 10/115200 s per byte) T is 60 ms for HELLO,
SET_POSE, GET_STATE, PROFILE_APPLY and CONTROL, 61.1 ms for READ_WHEELS,
65.6 ms for READ_DOC, and at most 65.9 ms (a full PROFILE_WRITE chunk).
`Client::responseTimeout(op, len)` computes it. The budget behind it is in
[Bus ownership and timing](interfaces.md#bus-ownership-and-timing). The same
formula is used on USB.

**Retries.**
- HELLO: same bytes after every timeout, no limit (the normal connecting
  state). Stale means a new nonce.
- SET_POSE and CONTROL: same bytes, as above.
- READ_WHEELS and PATH_REPORT: one send, never resent.
- Everything else (GET_STATE, profile writes and applies, READ_DOC) is
  idempotent by content and gets a new request id each time.
- Request ids count from 1 per program start and wrap 65535 to 1.

**Pose age.** A pose is typically `Pi age + round trip + up to state_period
+ request_gap` old when used; a poll cycle adds up to `poll_period_ms`.
actuGATR's input age limits must allow that.

### ClientConfig

| Field | Default | Meaning |
|---|---|---|
| `response_timeout` | 0.060 s | base of the per-request timeout, from the write call's return |
| `byte_time` | 10/115200 s | per byte beyond the 85-byte v3 budget pair |
| `request_gap` | 0.005 s | after a reply or a timeout |
| `state_period` | 0.020 s | GET_STATE poll period |
| `link_timeout` | 0.25 s | connected while a correlated reply is this recent |
| `pending_retry` | 0.020 s | SET_POSE and PROFILE_APPLY resend after Pending |
| `placement_attempts` | 10 | sends per placement |
| `placement_deadline` | 1.0 s | first send through the confirming state |
| `control_attempts` | 5 | sends per control before its first reply |
| `control_deadline` | 1.0 s | first send through its first reply |
| `control_retry` | 0.1 s | CONTROL resend after Pending, or after a lost reply once answered |
| `control_wait` | 20 s | first send through the final result, once the Pi answered |
| `hello_backoff` | 1.0 s | HELLO period after an unsupported version or op |
| `field_period` | 0.5 s | between field estimate reads |
| `transfer_backoff` | 1.0 s | after 3 failed transfers of one kind in a row |
| `telemetry_period` | 0.1 s | TELEMETRY sends on a grid of this period, never closer than half of it |
| `profile` | none | `ProfileDocument`; `LinkConfig::profile` fills it from a `RobotProfile` |
| `bench_imu` | empty | Brain bench IMU sample for every GET_STATE; empty sends flags 0 |

`BenchImuSample`: `valid`, `stamp_ms` (Brain clock at the read), `rotation_mdeg`
(continuous, CCW), and `calibrating` (not sent; readiness uses it). The Pi
pairs these samples with Pico encoder data by arrival time only; that is a
bench approximation and does not synchronize the Brain and the Pico clocks.

`ClientStats` counts requests, resends, replies, timeouts, uncorrelated and
bad frames, drained bytes, read and write errors, sessions opened and lost,
Pi restarts, stale HELLOs, unexpected replies, profile writes, document
chunks, rejects and stale answers, maps and estimates completed, and path
reports sent and dropped, and TELEMETRY sent (and of those, sent overdue),
replaced, dropped and refused.

## Client (portable)

```cpp
Client     client(port, nonce_function, ClientConfig{});
LinkDriver driver(client);  // StateSource and PathSink, no I/O
client.poll(now);           // every few ms from one task; never blocks
```

`poll(now)` reads and correlates replies, times out the outstanding request,
then sends at most one request; `now` also stands for the write call's return.
A reply is accepted only when its op and request id match the outstanding
request and, for HELLO, the nonce, else the session. Only a correlated reply
can signal a Pi restart. `ProsLink` wraps exactly this pair; host tests and
the Pi end-to-end test use them directly.

## PROS packaging

A PROS project compiles the libraries in place from this repository through
[brain/gatr2_brain.mk](../brain/gatr2_brain.mk). Nothing is copied: the PROS
build and the host build use the same files.

| Variable | Default | Meaning |
|---|---|---|
| `GATR2_ROOT` | `../..` | repository root as a path relative to the PROS project |
| `GATR2_BRAIN_LIBS` | `investigatr communigatr actugatr` | libraries to compile; `communigatr` and `actugatr` need `investigatr` |
| `GATR2_SRC_communigatr` | explicit list | `src/*.cpp` and `pros/*.cpp` of communiGATR, `translaGATR/frame_codec.cpp`, `translaGATR/link_documents.cpp` |
| `GATR2_WARNFLAGS` | `-Wall -Wextra` | warnings for library sources |

The fragment is included twice from the project Makefile. Before `common.mk`
it adds the library objects (`bin/gatr2/<path>.o`) to the link and the include
paths (the three libraries' `include/`, `brain/robot` and the repository
root, so `#include "translaGATR/frame_codec.h"` works). After `common.mk` it adds
one compile rule per source with the project's flags and a dependency file
next to the object.

Import into another PROS project:

1. Start from a kernel 4.2.2 project without liblvgl, for example
   `pros c new-project <dir> v5 4.2.2 --no-default-libs`.
2. In its Makefile, in the user section:

   ```make
   EXTRA_CXXFLAGS=-D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING
   C_STANDARD:=gnu17
   CXX_STANDARD:=gnu++20
   GATR2_ROOT ?= ../..
   GATR2_BRAIN_LIBS := investigatr communigatr actugatr
   ```

3. Replace its last line, `-include ./common.mk`, with:

   ```make
   include $(GATR2_ROOT)/brain/gatr2_brain.mk
   -include ./common.mk
   include $(GATR2_ROOT)/brain/gatr2_brain.mk
   ```

4. Include `"communigatr/pros_link.h"` and use it as in [PROS use](#pros-use).
   `brain/locaGATR` and `brain/operaGATR` are complete examples.
5. Build with `pros make`. Build and upload steps for this repository's
   programs: [Build and upload](brain_setup.md#2-build-and-upload).

Notes:

- `GATR2_ROOT` must be relative. A drive letter path (`C:/...`) breaks the
  `-iquote` flags in the toolchain's shell.
- A new library source must be added to its list in `gatr2_brain.mk`, and to
  `pi/naviGATR/tests/CMakeLists.txt` when the Pi end-to-end test needs it.
  Headers need no listing.
- The fragment has its own compile rule: `common.mk`'s dependency steps
  assume sources under `src/` and would write over library sources.
- The template defaults `gnu23` and `gnu++26` are rejected by the PROS arm gcc
  13.3, and the kernel headers need C++20. Library code is C++17 on the host
  and compiles as gnu++20 here.
- Without liblvgl `pros::lcd` prints nothing; use `pros::screen::print`.
- Linking uses `--gc-sections`: everything listed is compiled, so errors
  still show, but unused code stays out of the image.
- Windows, Git Bash: the PROS CLI needs `PROS_TOOLCHAIN` set to the toolchain
  folder in Windows form; `PATH` alone is not enough.

  ```
  TC="$HOME/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr"
  export PROS_TOOLCHAIN="$(cygpath -w "$TC")"
  ```

PROS library templates (`IS_LIBRARY`) are not used: a template needs a second
copy of the sources and a `pros c apply` in every consumer after each change.
Compiling in place keeps one copy, and every PROS build uses the host-tested
sources.

## Host build and tests

The host project is `brain/CMakeLists.txt`, with the commands in
[brain/README.md](../brain/README.md). The codec's own tests are in
`translaGATR/tests` (`cmake -S translaGATR/tests`).

| Target | Contents |
|---|---|
| `communigatr` | `src/*.cpp`, `translaGATR/frame_codec.cpp`, `translaGATR/link_documents.cpp`; links `investigatr_types` |
| `communigatr_fakes` | `sim/`: `FakePi`, `FakeBus`, `FakeUsb`, `LinkRig` |
| `communigatr_tests` | the test files below, listed explicitly |

Fakes (host only):

- `FakePi`: the Pi's v4 rules over the real codec: sessions and dedupe,
  placement with an apply delay, profile staging and apply with capability
  rejection and apply delay, field documents with any object count (including
  maps of many chunks), estimate replacement and Stale, control with
  NotStationary, Pending and Pico failures, READ_WHEELS, path reports,
  TELEMETRY kept and answered header only (`setTelemetryResult` for a
  refusing Pi, `setUnsupportedOp(kOpTelemetry)` for a Pi from before it),
  restart, other versions and unsupported ops.
- `FakeBus`: a timed half-duplex RS-485 bus with byte airtime, Brain
  latencies, the Pi reply window, scripted faults (dropped, duplicated,
  corrupt, fragmented, truncated, delayed) and collision detection.
- `FakeUsb`: the V5 USB console with NG1 lines through the real line codec
  both ways, latencies, console text mixed in, a cable that can be pulled,
  and the Pi's no-answer-after-reopen rule.
- `LinkRig`: a fake Pi behind either transport, a `Client` and `LinkDriver`,
  a stepped clock, and `rebootBrain()`.

| Test file | Covers |
|---|---|
| `usb_line_gtest.cpp` | NG1 encoding and refusals, carriage return, text before the marker and the last marker, bad digits drop the line, digit and line limits, reset, round trip of every frame size |
| `fake_pi_gtest.cpp` | the fake Pi's session, dedupe, placement, profile staging and rejection memory, documents, control and path rules |
| `client_gtest.cpp` | readiness only after a correlated state, fresh state ids, fragmented, truncated, corrupt, duplicate and dropped replies, same-byte placement and control retries, uncorrelated and drained bytes, Pi restart, unknown session, lost placement never resent, stale HELLO and repeated nonces, Brain reboot, placement applied only after its anchor, placement expiry and replacement, bench IMU samples, control Pending and Failed, an answered control through more lost replies than `control_attempts` and through failed writes, wheel read tickets (Ok, one at a time, NotReady, lost send, failed write, session loss), path thinning, request id wrap, unsupported version and op, link loss, write failures |
| `client_timing_gtest.cpp` | every op's timeout meets the budget, replies anywhere in the window, retry after the latest possible reply, gap and poll period, transfers only while the poll is not due, polling stays responsive during transfers, a too short timeout collides |
| `robot_profile_gtest.cpp` | SI to wire units, rounding and angle wrapping, the three kinds of wheel value kept apart, gearing, scale and calibration ranges, supported topologies, Brain side rejections with the Pi's reason codes, document id |
| `doc_assembly_gtest.cpp` | documents of every size up to capacity, chunk sizes, inconsistent chunks, first chunk bounds, CRC, restart |
| `profile_sync_gtest.cpp` | chunked upload through Pending, lost chunks resumed, idempotent Brain restart, changed profile needs placement, rejection settled, Pi without Brain profiles, Brain-invalid profile never sent, Pi restart, upload interrupted across sessions, replaced staging, an APPLY Ok that counts only with a state showing it |
| `field_sync_gtest.cpp` | every object count arrives whole and checked, no field, inconsistent chunks, malformed map, estimate for another map or count, replaced estimate Stale, backoff, field period, map cache across sessions and Pi restarts, new map, interrupted transfers, Unavailable |
| `link_driver_gtest.cpp` | status order, configured anchors, pending placement hides the pose, units and ages, placement rounding, frame numbers, field in SI units, field frame equals robot frame under one anchor, pre-session estimate age, path reports |
| `recovery_gtest.cpp` | on both transports: Brain restart keeps profile, placement and map; restarts during profile upload or apply; placement refused before the profile; control through lost replies runs once; a Pending control across a 1.5 s cut ends Ok with one request id and one execution; an answered control ends at `control_wait` while cut; a wheel read lost to a cut settles and the next one reads; Pi restart; cable cut resumes the same session; interrupted transfers and profile exchange; replies from before a cut ignored; Pico restart changes the frame, not the placement; explicit reinitialize; IMU reinit followed to its outcome; interrupted commands never resumed. USB only: console text and prefixes ignored |
| `readiness_gtest.cpp` | names and health bits, connecting then reconnecting, profile pending and rejected, Brain-refused profile, VEX profile ignores the Pico IMU, Pico IMU health and every calibration state, three wheels without IMU, placement in flight and first pose, no Brain profile |
| `profile_change_gtest.cpp` | on both transports: new travel scale applied as a new profile that needs placement, same profile, Brain-refused profile, invalid first profile, change during an upload or while the Pi applies; with Pi apply delays 0 to 5, neither a runtime change nor a Brain restart with an edited profile ever shows the old placement as valid, ready or localized |
| `vex_imu_recalibration_gtest.cpp` | the VEX calibration starts only after the Pi's Ok; movement, refusals and lost answers start nothing; an IMU that does not start, does not finish or ends invalid; on both transports against the fake Pi |
| `wheel_calibration_gtest.cpp`, `startup_placement_gtest.cpp`, `link_events_gtest.cpp` | the application helpers |
| `attitude_gtest.cpp` | VEX roll and pitch to the robot frame at mounts 0, 90, 180 and 270 and others, combined tilts round trip, the exact gravity rotation against the small angle swap, non-finite input, the centidegree group |
| `client_telemetry_gtest.cpp` | the fake Pi's TELEMETRY rules; on both transports: values arrive intact, the target bit arrives as sent (set with a target at the origin, clear without one), latest wins, one send per period on a grid, state polls keep at least 85% of their rate, an older or refusing Pi stops telemetry for the session and keeps it, a new session tries again, a Pi restart drops the unsent report, a report held through an outage is dropped; on slow links (USB turnaround 11 ms, bus reply after 8 ms) at least 30 sends in 5 s, no wait over 250 ms, never closer than half a period, one exchange at most between two polls, at least 80% of the polls, a waiting transfer and telemetry both served; on the bus log: on the default bus never with the poll due, on a slow bus overdue, and on both only in the first slot after a state reply with the longest poll gap bounded by one TELEMETRY exchange and the gap; a lost report never resent; bodies the codec cannot carry and unassigned flag bits refused |

`pi/naviGATR/tests/brain_link_e2e_gtest.cpp` (target `brain_link_e2e_tests` in
`pi/naviGATR/tests/CMakeLists.txt`, run by the Pi test build) exercises the
real Pi runtime from `brain_profile_usb.xml` against the real `Client`,
`LinkDriver`, NG1 codec, actuGATR `Motion` and the planner, with a drivetrain
sim as the truth: profile upload, placement, direct, avoiding and landmark
moves on the transferred map, wheel readings, a travel scale change, Brain and
Pi restarts, and used sensors dropping out.

## Limitations and unvalidated items

- Host tested only. Not validated: the V5 USB console under load and across
  unplugging, PROS stdin behavior, the smart port settle retries and reopen,
  the V5 smart port's RS-485 direction switching (an echoed request would only
  count as a bad frame), V5 serial latency, task timing and stack sizes under
  load, and `pros::Imu::reset` timing.
- The VEX IMU sample carries the Brain read time, not a measurement time, and
  a repeated IMU value counts as a new sample at the Pi. This is the bench
  arrival-time approximation.
- VEX roll and pitch: the sign conventions and which IMU axis each angle is
  about are UNVERIFIED on hardware, and `kVexImuMountYawDeg` is a
  PLACEHOLDER; see the bench check in
  [Brain setup](brain_setup.md#10-telemetry-and-attitude). An IMU mounted on
  its side or upside down is not supported.
- TELEMETRY delays some state polls: 6 to 11% fewer at the default fake
  timing, up to 20% on slow fakes, where it also sends fewer reports (about
  6 per second instead of 10); see [Telemetry](#telemetry). Not measured on
  the V5.
- One placement, one control and one wheel read at a time; only the latest
  ticket of each is tracked.
- `VexImuRecalibration` checks stillness before the VEX calibration starts,
  not during its roughly 2 s.
- No latency compensation: ages are reported, not used to predict the pose.
- A competition task deleted while it holds the link mutex leaves the link
  answering busy until the program restarts.
