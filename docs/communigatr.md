# communiGATR

`brain/communiGATR` is the Brain side of the brain link v3 described in
[interfaces.md](interfaces.md#brain-link-v3). It gives an investiGATR
`Navigator` the Pi's robot pose and landmark estimates as an ordinary
`InputSource`, and lets the application place the robot on the field.

Status: implemented and host tested against a fake Pi on a simulated
half-duplex bus. The PROS sources compile and link against PROS kernel 4.2.2.
Nothing here has been validated on the robot or on real RS-485 hardware.

| Path | Contents |
|---|---|
| `include/communigatr/` | public headers; `pros_*.h` are PROS only |
| `src/` | portable sources (host and PROS): `client.cpp`, `driver.cpp` |
| `pros/` | PROS only sources: `pros_serial_port.cpp`, `pros_driver.cpp` |
| `sim/` | host only fake Pi, fake half-duplex bus, test rig |
| `tests/` | host tests |

Namespace `communigatr`. Time is `Seconds` (double) everywhere.

## Layers

```
investigatr::Navigator
  -> InputSource&
       ProsDriver              PROS: poll task, one mutex, owns everything below
         Driver                InputSource: units, validity, ages, frames, landmarks
           Client              protocol: sessions, scheduling, retries, correlation
             BytePort          nonblocking bytes
               ProsSerialPort  V5 smart port, generic serial
```

| Class | Header | Portable | Role |
|---|---|---|---|
| `BytePort` | `byte_port.h` | yes | nonblocking `read`/`write` interface |
| `Client` | `client.h` | yes | brain link v3 state machine; all I/O in `poll(now)` |
| `Driver` | `driver.h` | yes | `investigatr::InputSource` over a client; no I/O |
| `ProsSerialPort` | `pros_serial_port.h` | PROS | `BytePort` over the PROS `serial_*` C API |
| `ProsDriver` | `pros_driver.h` | PROS | owns port, client and driver; poll task; mutex; `InputSource` by delegation |

The portable classes have no clock, thread or PROS dependency. Time is passed
in, so the same code runs in host tests and on the Brain.

## PROS use

```cpp
#include "investigatr/navigator.h"
#include "communigatr/pros_driver.h"

using communigatr::ProsDriver;

ProsDriver*             navigatr  = nullptr; // created once, kept for the program's life
investigatr::Navigator* navigator = nullptr;

void initialize() {
    communigatr::ProsDriverConfig config;
    config.port = 10;      // PLACEHOLDER: smart port wired to the RS-485 link
    config.baud = 115200;  // must match the Pi serial resource
    navigatr  = new ProsDriver(config);
    navigator = new investigatr::Navigator(*navigatr);
    navigatr->start();

    // Bounded waits; never hang in initialize().
    const double connect_end = ProsDriver::now() + 3.0;
    while (!navigatr->status().connected && ProsDriver::now() < connect_end) {
        pros::delay(10);
    }
    const auto   ticket    = navigatr->submitPlacement({0.0, 0.0, 0.0}); // starting pose
    const double place_end = ProsDriver::now() + 2.0;
    while (navigatr->placementResult(ticket) == communigatr::PlacementResult::kPending &&
           ProsDriver::now() < place_end) {
        pros::delay(10);
    }
}

// Control task, fixed period, the only motor writer:
//   const investigatr::DriveCommand demand = navigator->update(ProsDriver::now());
```

- `start()` enables generic serial on the port, sets the baud rate, clears the
  buffers and starts the poll task. It returns false when PROS refuses the
  port (errno set) and can be called again.
- Pass `ProsDriver::now()` (seconds since PROS start, from `pros::micros()`)
  to `Navigator::update`. Ages and link timing are only meaningful when the
  Navigator and the poll task use the same clock.
- The Navigator itself is not thread safe; one control task owns it.

### ProsDriverConfig

| Field | Default | Meaning |
|---|---|---|
| `port` | 0 | V5 smart port 1..21 wired to the RS-485 link; must be set |
| `baud` | 115200 | must match the Pi serial resource |
| `poll_period_ms` | 2 | poll task period, at least 1 |
| `task_priority` | `TASK_PRIORITY_DEFAULT + 1` | poll task priority |
| `client` | `ClientConfig{}` | see [Client](#client) |
| `driver` | `DriverConfig{}` | see [Driver](#driver) |

### ProsDriver calls

Every call takes the wrapper's mutex; the poll task takes the same mutex
around each `poll`. No call waits for the bus.

| Call | Effect |
|---|---|
| `start()` | open the port and start the poll task |
| `now()` (static) | poll task clock, seconds |
| `request(InputRequest)`, `latest(now)` | `InputSource`, forwarded to the driver |
| `submitPlacement(Pose)` | forwarded to the driver |
| `placementResult(ticket)`, `placementStatus(ticket)` | forwarded to the driver |
| `status()` | `ProsDriverStatus`: started, ready, connected, link_age, session, pi_instance, error, peer_version, selection, stats |

The destructor stops the poll task. Construct the wrapper once and keep it; it
is not meant to be created per command.

### HELLO nonce

The client asks for a nonce once per new HELLO. `ProsDriver` mixes, with a
murmur3 style finalizer: `pros::micros()` at construction, at `start()` and at
the call; battery voltage and current; the object's address and a stack
address; and a call counter. Timing is the main entropy: boot time to the
first HELLO varies with device enumeration and `initialize()` work. The
addresses only differ between builds. The nonce only needs to differ from the
Pi's last four opening nonces. A collision costs one `kResultStale` and a retry
with a new nonce; a 0 or a repeat is replaced by the client.

## Driver

```cpp
Driver driver(client);            // client polled by its owner
Driver driver(client, {true});    // also accept configured anchors
```

`Driver(Client&, const DriverConfig& = {})` reads the client's
latest state; it never polls. Whoever owns the client calls `client.poll(now)`
every few ms (`ProsDriver` does this).

| Call | Effect |
|---|---|
| `request(InputRequest)` | wanted landmark: ids 1..255 are sent as the wire id; `{}` or any other id releases the Pi selection |
| `latest(now)` | `InputSnapshot` built from the latest GET_STATE of the current session |
| `submitPlacement(Pose)` | SET_POSE in wire units; ticket, or 0 when another placement is pending or the pose is not finite or outside the wire range |
| `placementResult(ticket)`, `placementStatus(ticket)`, `placementPending()` | from the client |
| `client()`, `config()` | access |

`DriverConfig::accept_configured_anchor` (default false): also accept a Pi pose
whose field anchor came from its `<InitialPlacement>` rather than from a Brain
SET_POSE.

### Units

Wire to snapshot: mm to m, centidegrees to rad (wrapped to (-pi, pi]), ms to s.
Placement: m to mm and rad to centidegrees, rounded to the nearest unit, heading
normalized to (-18000, 18000].

### Robot estimate

`robot.valid` requires all of:

- a GET_STATE Ok of the current session;
- `kRobotPoseValid`, `kRobotLocalized` and `kRobotAgeKnown`;
- an accepted anchor: `kRobotAnchorCommand` (a SET_POSE from any session of
  this Pi instance), or `kRobotAnchorConfigured` with
  `accept_configured_anchor`;
- no placement pending: from `submitPlacement` until the ticket is terminal,
  including the phase where the Pi answered Ok and the driver waits for a state
  that shows the placement's anchor.

Pose and age are filled only when valid. Status and health bits are never
treated as acknowledgements.

Ages:

```
robot.age    = robot_age_ms / 1000    + round_trip + (now - received_at)
landmark.age = landmark_age_ms / 1000 + round_trip + (now - received_at)
```

`round_trip` is the matched GET_STATE's write to its reply; `received_at` is
the poll time that handled the reply. The Pi takes its ages at its cycle start,
so the sum is an upper bound only to within the Pi cycle processing time (a
few ms). Replies are noticed up to one poll period late, which makes ages
slightly conservative. No clock values from different devices are subtracted.

### Frame generation

`frame` is 0 while there is no state of the current session. Otherwise it is a
number, starting at 1, that changes whenever the identity
(`pi_instance`, `session`, `odometry_epoch`, `anchor_revision`) of the state
changes. Any change is a coordinate discontinuity: a Navigator that captured
the old frame fails with `kFrameChanged` or, when the link is down,
`kInputLost`. Generations are unique per driver instance.

Changes come from a Pi restart or reset (instance), a new Brain session
(session, including a Brain reboot), a Pico restart (odometry epoch), and an
applied placement (anchor revision).

### Link

- `connected`: session open, a GET_STATE Ok received in it, and the last
  correlated reply within `link_timeout` (0.25 s).
- `link_age`: now minus the last correlated reply; infinity without a session.

### Landmark statuses

Checked in this order:

| Condition | `status` |
|---|---|
| no landmark requested | `kNotRequested`, id 0 |
| requested id 0 or above 255 (no wire id) | `kUnknownLandmark` |
| not connected | `kStale` |
| SELECT not acknowledged, or no state since the ack | `kPending` |
| SELECT answered `kResultUnknownLandmark` | `kUnknownLandmark` |
| SELECT answered `kResultLandmarkUnsupported` (Pi world estimation is noop) | `kUnsupported` |
| state carries the id with source none | `kUnavailable` |
| source nominal | `kAvailable`, `kNominal`, `age_known` false |
| source observed | `kAvailable`, `kObserved`, `age_known` true |

The landmark pose is the physical landmark pose from the same state as the
robot pose, so it shares its frame. The Navigator applies the destination
offset once. The Navigator requests a landmark only while a landmark waypoint
is active; `request({})` releases the selection on the Pi.

## Client

```cpp
Client client(port, nonce_function, ClientConfig{});
client.poll(now);   // every few ms, never blocks
```

`poll(now)` reads and correlates replies, times out the outstanding request,
then sends at most one request. `now` also stands for the write call's return.

| ClientConfig | Default | Meaning |
|---|---|---|
| `response_timeout` | 0.060 s | from the write return; see the budget in [interfaces.md](interfaces.md#bus-ownership-and-timing) |
| `request_gap` | 0.005 s | after a reply or a timeout |
| `state_period` | 0.020 s | GET_STATE poll period |
| `link_timeout` | 0.25 s | connected while a correlated reply is this recent |
| `pending_retry` | 0.020 s | SET_POSE resend after `kResultPending` |
| `placement_attempts` | 10 | sends per placement |
| `placement_deadline` | 1.0 s | from the first send through the GET_STATE that confirms the Ok |
| `select_attempts` | 5 | sends per SELECT transaction |
| `select_retry` | 0.5 s | wait before a new SELECT after a failed one |
| `hello_backoff` | 1.0 s | HELLO period after an unsupported version or op |

Scheduling, one request outstanding: HELLO while no session is open; then
placement > landmark selection > GET_STATE poll. Before each send the client
drains its receive buffer and resets its frame reader. Request ids count from 1
per boot and wrap 65535 to 1.

Retries:

- HELLO: same bytes every timeout plus gap while the Pi is silent (the normal
  connecting state, no attempt limit). `kResultStale` means a new nonce.
- SET_POSE: same bytes on timeout, and `pending_retry` after `kResultPending`,
  within `placement_attempts` and `placement_deadline`.
- SELECT: same bytes on timeout within `select_attempts`, then a new
  transaction after `select_retry`.
- GET_STATE: never resent; every poll has a new request id.

### Placement tickets

`submitPlacement(x_mm, y_mm, heading_cdeg)` returns a ticket, or 0 while
another placement is pending. Only the latest ticket is tracked; older tickets
report `kNone`. A ticket submitted with no session waits for the first
session.

| `PlacementResult` | Meaning |
|---|---|
| `kPending` | queued, in flight, or Ok and waiting for a GET_STATE that shows its anchor |
| `kApplied` | a GET_STATE after the Ok reports `anchor_revision` at or after the Ok's |
| `kRejected` | error result; `PlacementStatus::result` has it |
| `kTimedOut` | attempts or deadline used up; outcome unknown, it may still apply |
| `kSessionLost` | the session ended first; never resent |

`PlacementStatus` also carries the `odometry_epoch` and `anchor_revision` from
the Ok reply. A timed out placement that applies later changes the anchor
revision, so the driver reports a new frame generation.

### Selection, readiness, errors

- `selectLandmark(id)` sets the wanted wire id (0 = none); the driver calls it
  from `request()`. `selection()`: `kNotRequested`, `kPending`, `kActive`,
  `kUnknownLandmark`, `kUnsupported`. Unknown and unsupported are settled until
  the wanted id or the session changes.
- `ready()`: session open and a GET_STATE Ok received in it.
- `error()`: `kUnsupportedVersion` or `kUnsupportedOp`. Terminal: the session
  is dropped and HELLO repeats every `hello_backoff`. Cleared when ready again.
  `peerVersion()` gives the Pi's version.
- `stats()`: `requests`, `resends`, `replies`, `timeouts`, `uncorrelated`,
  `bad_frames`, `drained_bytes`, `read_errors`, `write_errors`, `sessions`,
  `session_losses`, `pi_restarts`, `stale_hellos`, `unexpected`.

## Sessions and restarts

A session is one Brain application run. The protocol rules are in
[interfaces.md](interfaces.md#sessions); this is what the Brain side does.

Brain reboot (the Pi keeps running):

1. The new program creates a new client: no session, request ids from 1, no
   cached input. `latest()` reports frame 0, not connected, robot invalid.
2. HELLO with a fresh nonce opens a new session. The Pi clears the old
   session's selection, target latch and duplicate records; localization is
   untouched.
3. The first GET_STATE Ok makes the client ready. If the Pi's anchor came from
   a SET_POSE of an earlier boot, the pose is valid at once, in a new frame
   generation.
4. The application's starting SET_POSE applies even when its request id and
   pose equal the old boot's, because dedupe and placement identity include
   the session.
5. Delayed replies to the old session fail correlation and are counted.
   Delayed old-session requests get `kResultUnknownSession` on the Pi.
6. Nothing resumes: the Navigator is new, and only the new program issues
   commands.

Pi restart or `System::reset()` (the Brain stays on):

- The next correlated reply carries a different `pi_instance` (or
  `kResultUnknownSession`). The client drops the session: cached state is
  cleared, so the driver reports frame 0 and not connected; a pending
  placement becomes `kSessionLost` and is never resent; the selection is
  cleared.
- An active Navigator command fails with `kInputLost`.
- The client opens a new session. The restarted Pi has no anchor, so the robot
  stays invalid until the application submits a new placement.

Link loss (cable, Pi process stalled):

- No correlated replies: after `link_timeout` the driver reports not
  connected, the robot pose ages out, and the landmark is `kStale`. An active
  command fails with `kInputLost`.
- The session stays open. When replies resume in the same session the frame
  generation is unchanged and the driver is connected again. Nothing resumes;
  the application must issue a new command.

Placement acknowledgement:

- `kResultOk` from the Pi means localization applied that placement. The
  client still waits for a GET_STATE that shows its anchor before `kApplied`,
  and the driver reports the robot invalid until then, so a pose from before
  the placement is never used after `kApplied`.
- `kTimedOut` does not mean "not applied". Check the robot pose, or place
  again.

## Timing

- `ProsDriver` polls every 2 ms at `TASK_PRIORITY_DEFAULT + 1`. `poll` never
  waits.
- `response_timeout` must satisfy the budget in
  [interfaces.md](interfaces.md#bus-ownership-and-timing):
  `T >= A_req + W + A_rep + R + L`. With the defaults, V5 plus Pi latency and
  margin `L` may be up to 12.4 ms. Poll period granularity adds up to one
  period to the measured round trip.
- A robot pose is typically `Pi age + round trip + up to state_period +
  request_gap` old when used. `NavigatorConfig::max_pose_age` (0.25 s) must be
  at least that, and `max_link_age` at least `state_period + response_timeout`.
- A GET_STATE Ok reply is 59 bytes, 5.1 ms at 115200 baud.

## PROS packaging and import

A PROS project compiles the libraries in place from this repository through
[brain/gatr2_brain.mk](../brain/gatr2_brain.mk). Nothing is copied: the PROS
build and the host build use the same source files.

| Variable | Default | Meaning |
|---|---|---|
| `GATR2_ROOT` | `../..` | repository root, as a path relative to the PROS project |
| `GATR2_BRAIN_LIBS` | `investigatr communigatr` | libraries to compile; `communigatr` needs `investigatr` |
| `GATR2_SRC_investigatr` | explicit list | `brain/investiGATR/src/*.cpp` |
| `GATR2_SRC_communigatr` | explicit list | `brain/communiGATR/src/*.cpp`, `brain/communiGATR/pros/*.cpp`, `common/frame_codec.cpp` |
| `GATR2_WARNFLAGS` | `-Wall -Wextra` | warnings for library sources |

The fragment is included twice. Before `common.mk` it adds the library
objects (`bin/gatr2/<path>.o`) to the link and the include paths
`brain/investiGATR/include`, `brain/communiGATR/include` and the repository
root. After `common.mk` it adds one compile rule per source, with the
project's `CXXFLAGS` and a dependency file next to the object, so a header
change rebuilds what uses it.

Import into another PROS project:

1. Start from a kernel 4.2.2 project without liblvgl, for example
   `pros c new-project <dir> v5 4.2.2 --no-default-libs`.
2. In its Makefile, in the user section:

   ```make
   EXTRA_CXXFLAGS=-D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING
   C_STANDARD:=gnu17
   CXX_STANDARD:=gnu++20
   GATR2_ROOT ?= ../..
   GATR2_BRAIN_LIBS := investigatr communigatr
   ```

3. Replace its last line, `-include ./common.mk`, with:

   ```make
   include $(GATR2_ROOT)/brain/gatr2_brain.mk
   -include ./common.mk
   include $(GATR2_ROOT)/brain/gatr2_brain.mk
   ```

4. Include `"investigatr/navigator.h"` and `"communigatr/pros_driver.h"` and
   use them as in [PROS use](#pros-use). `brain/testing` is a complete example.
5. Build with `pros make`.

Notes:

- `GATR2_ROOT` must be relative. A drive letter path (`C:/...`) breaks the
  `-iquote` flags in the toolchain's shell and the headers are not found.
- A new library source file must be added to its list in `gatr2_brain.mk`.
  Headers need no listing.
- The fragment uses its own compile rule. The dependency steps in `common.mk`
  (`DEPFLAGS`, `MAKEDEPFOLDER`, `RENAMEDEPENDENCYFILE`) assume sources under
  `src/`; for a source elsewhere their `mv` would write the dependency file
  over the `.cpp`.
- The template defaults `gnu23` and `gnu++26` are rejected by the PROS arm gcc
  13.3, and the kernel headers need C++20. Library code is C++17 on the host
  and compiles as gnu++20 here.
- Without liblvgl `pros::lcd` prints nothing; use `pros::screen::print`. The
  `-D` flag above silences the kernel's lcd deprecation warning.
- Linking uses `--gc-sections`: library code the program never calls is
  compiled, so errors still show, but left out of the image.

PROS library templates (`IS_LIBRARY`) are not used. A template packages
sources from the project's own `src/` and headers from `include/<LIBNAME>`.
These libraries live in `brain/` and are built and tested on the host with
CMake, so a template would need a second copy of the sources, a template
rebuild for every change and `pros c apply` in every consumer. Compiling in
place keeps one copy, and every PROS build uses the tested sources.

Verified with PROS CLI 3.5.6, arm gcc 13.3.1 and kernel 4.2.2: `pros make` in
`brain/testing`, and a copy of that project outside the repository layout with
a relative `GATR2_ROOT`, both without warnings.

## Testing application and hardware bring-up

[brain/testing](../brain/testing/README.md) is a minimal PROS program using
both libraries. It has been built with `pros make`, never run on a Brain.

| Callback | What it does |
|---|---|
| `initialize()` | creates `ProsDriver` and `DriveControl`, checks the `NavigatorConfig`, opens the link, waits up to 3 s for `connected`, submits `kStartPose`, waits up to 2 s for the placement result and shows it; without a link it places nothing |
| `autonomous()` | the demo: `goTo(kDemoGoal)`, then `follow(kDemoPath)`, then `goToRelative(kDemoLandmarkId, kDemoLandmarkOffset)` |
| `opcontrol()` | manual driving; button A runs the demo, B cancels it |
| `disabled()` | `stop()`: cancels the command and stops the motors |
| `competition_initialize()` | nothing |

`DriveControl` (`include/drive_control.h`) owns the Navigator and the drive
motors:

- One task every 10 ms at `TASK_PRIORITY_DEFAULT + 1` is the only code that
  writes the drive motors. Modes: disabled (zero), navigate (Navigator
  demand), manual (latest manual demand).
- Each period: `Navigator::update(ProsDriver::now())` in every mode, so the
  status stays current; `mixTank`; each side times `kMaxVoltageMv` through
  `move_voltage`. Zero on both sides calls `brake()`, which acts per
  `kBrakeMode`. Motor directions come from signed ports.
- `goTo`, `goToRelative` and `follow` switch to navigate. `manual(demand)`
  switches to manual and cancels navigation. `stop()` cancels, clears the
  manual demand and brakes at once in the caller's task. Every call takes one
  mutex and returns at once.

Each demo step uses `MotionOptions{kDemoTimeout, kDemoRequireObserved}`. Its
wait loop compares `status().command_id` with the step's id and gives up one
second after the motion timeout. A step that fails, is aborted or runs out
stops the drive and ends the demo. `autonomous()` needs a competition switch
or field control; without one PROS runs `opcontrol()` after `initialize()`,
and button A runs the same demo.

| Control | Effect |
|---|---|
| left stick Y | forward, + drives robot +x |
| right stick X | turn, right = clockwise |
| A | run the demo; manual driving pauses until it ends |
| B | cancel the demo: the motors stop at once |

| Screen line | Shows |
|---|---|
| 0 | title |
| 1 | `init: placement applied, result 0`, `init: no link, robot not placed`, `init: port N not opened, errno E` or `init: navigator config invalid: <why>` |
| 2 | `link up s <session> pi <pi_instance> <link age> ms`, or `link down ready R err E tx <requests> rx <replies>` |
| 3 | `pose <x> <y> m <heading> deg <age> ms f<frame>`, or `pose invalid f<frame>` |
| 4 | `cmd <id> <state> <reason> <distance error> m` |
| 5 | demo step: `goal: running`, `path: completed`, `landmark: failed landmark unknown`, `goal: stopped` |

Lines 2 to 4 refresh every 100 ms while `initialize()`, the demo or
`opcontrol()` is waiting.

### Configuration

Everything robot specific is in `include/robot_config.h`, namespace
`robot_config`. Values marked PLACEHOLDER are not measured.

| Name | Meaning |
|---|---|
| `kLeftMotorPorts`, `kRightMotorPorts` | smart ports; a negative port reverses that motor |
| `kGearset`, `kBrakeMode` | motor cartridge; behavior at zero demand |
| `kMaxVoltageMv` | voltage at full demand, at most 12000 |
| `kNavigatrPort`, `kNavigatrBaud` | smart port wired to the RS-485 link; baud equal to the Pi resource's `<Baud>` |
| `kStartPose` | starting placement, field frame |
| `navigatorConfig()` | `NavigatorConfig` overrides: tolerances, output limits, minimum outputs, PID gains ([tuning](investigatr.md#tuning)) |
| `kDemoGoal`, `kDemoPath` | demo goal and path, field frame |
| `kDemoLandmarkId`, `kDemoLandmarkOffset` | landmark wire id (1..255), equal to a Pi `FieldObject` `wire_id`; wanted robot pose in the landmark frame |
| `kDemoRequireObserved` | false also accepts the Pi's nominal (map) landmark pose |
| `kDemoTimeout` | motion timeout per demo step |
| `kDemoButton`, `kCancelButton`, `kStickDeadband` | controller |

### Set up and build

The kernel files (`firmware/`, `include/pros/`) are not in git. After a clone,
in `brain/testing`:

```
pros c apply kernel@4.2.2 --force-apply --no-download
git checkout -- .gitignore
pros make
```

- The first command restores the kernel files offline from the local PROS
  template cache. Without `--force-apply` the CLI reports the kernel as
  installed and restores nothing. On a machine without kernel 4.2.2 in its
  cache, run it without `--no-download` (network; not verified here).
- The CLI keeps `Makefile`, `src/main.cpp` and `include/main.h`, but replaces
  `.gitignore` with its template copy, which drops the `firmware/` and
  `include/pros/` lines; the checkout puts it back.
- With the kernel files missing, `make` stops with a message naming these
  commands.
- Windows, Git Bash: `pros make` needs `PROS_TOOLCHAIN` set to the toolchain
  folder in Windows form; adding it to `PATH` is not enough.

  ```
  TC="$HOME/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr"
  export PROS_TOOLCHAIN="$(cygpath -w "$TC")"
  ```

  Plain `make` works with `PATH="$TC/bin:$PATH"`. The toolchain's `make` and
  shell are MSYS programs with their own root, so do not pass `/c/...` paths
  to them.
- Output goes to `bin/` and `.d/` (ignored); `make clean` removes both. The hot
  image is about 24 KB of code; the cold package (kernel, libc) about 1.3 MB.
- Upload with `pros upload` or the PROS VS Code extension (not exercised
  here).

Pi side: a profile with the `brain_link` command collection and publishing on
one serial resource, run threaded, with the baud equal to `kNavigatrBaud`; see
[Connect the Brain](../pi/navigatr/docs/setup.md#connect-the-brain). The
landmark step needs a `FieldObject` whose `wire_id` equals `kDemoLandmarkId`;
no supplied profile has one.

### Hardware bring-up checklist

In order. Keep the robot on blocks (wheels off the ground) until step 2
passes, keep the controller in hand, and start with a low `kMaxVoltageMv`.

1. Ports: drive motors on `kLeftMotorPorts` and `kRightMotorPorts`, the RS-485
   link on `kNavigatrPort`, `kGearset` equal to the fitted cartridges. Check
   on the Brain's device screen.
2. Motor directions, on blocks, in `opcontrol()`: a small push forward on the
   left stick turns every wheel in the robot's forward direction; the right
   stick to the right drives the left side forward and the right side
   backward (clockwise seen from above). Negate the port of a wrong motor.
   Releasing the sticks stops the motors.
3. Link: line 2 shows `link up` with a session within a second of the Pi
   running. `link down` with `tx` rising and `rx` at 0 means nothing answers:
   check the Pi profile pairing, baud, wiring, the DriverEnable GPIO and the
   Pi's link counters ([Check the link](../pi/navigatr/docs/setup.md#check-the-link)).
   `err 1`: the Pi speaks another brain link version; `err 2`: it does not
   know one of the requests.
4. Placement: line 1 shows `init: placement applied`, and line 3 shows
   `kStartPose`, an age well under `max_pose_age` (0.25 s) and a frame number.
   `timed out`: the placement was not confirmed within the placement deadline
   (for example no valid pose on the Pi because the Pico is not streaming); it
   may still apply later. `rejected`:
   see the result code in [interfaces.md](interfaces.md#brain-link-v3).
   `no link`: fix step 3 and restart the program; nothing is placed without
   the link.
5. Pose signs: push the robot straight forward by hand at heading 0: x grows.
   Turn it counterclockwise seen from above: the heading grows. Otherwise fix
   the Pi's wheel and IMU signs
   ([Measure the wheels and IMU sign](../pi/navigatr/docs/setup.md#measure-the-wheels-and-imu-sign));
   the Brain does not correct them.
6. Brain reboot: power cycle the Brain while the Pi keeps running. Line 2
   shows a new session with the same `pi_instance`, and the placement applies
   again. A Pi restart changes `pi_instance` instead.
7. First motion, off blocks, on clear floor: put `kDemoGoal` a short distance
   straight ahead and press A. Line 4 shows the phases (`turning`, `driving`,
   `aligning`) and ends with `completed`. Press B during a move: the motors
   stop at once and line 5 shows `goal: stopped`. Disabling from the
   competition switch also stops them. Then tune in the order of
   [tuning](investigatr.md#tuning).
8. Path: `kDemoPath` in clear floor space. The robot passes the first points
   and settles only at the last.
9. Landmark: map `kDemoLandmarkId` on the Pi. `landmark unknown`: no mapping
   for that id. `landmark unsupported`: the profile's world estimation is
   noop. `landmark unavailable`: no usable estimate in time; with
   `kDemoRequireObserved` the camera must see the landmark, without it the
   map pose is accepted. Measure the final robot pose against the landmark.
10. Link loss: unplug the link cable during a move. The robot stops within
    about 0.25 s (`max_pose_age`, `link_timeout`) and the command fails with
    `input lost`. Plug it back in: nothing resumes; run the demo again.

### Not validated

The program has only been built. Not validated on hardware: motor ports,
directions, gearset, brake mode and voltage limit; gains, tolerances and
timeouts; the demo destinations; controller mapping and screen layout; the
10 ms control task and 2 ms poll task under load on the V5; the V5 smart port's
RS-485 direction handling and serial latency; placement and landmark behavior
against a real Pi; `pros upload`.

## Host build and tests

The host project is `brain/CMakeLists.txt` (see
[investigatr.md](investigatr.md#build-and-tests) for the commands).

| Target | Contents |
|---|---|
| `communigatr` | `src/client.cpp`, `src/driver.cpp`, `common/frame_codec.cpp`; links `investigatr` |
| `communigatr_fakes` | `sim/`: `FakePi`, `FakeBus`, `LinkRig` |
| `communigatr_tests` | the tests below |

Fakes (host only):

- `FakePi`: the Pi's brain link rules (sessions, dedupe, placement, selection,
  restart) over the real common codec, with a scripted robot state, a landmark
  table and a configurable placement delay.
- `FakeBus`: a timed half-duplex bus with byte airtime, Brain latencies, the
  Pi reply window, scripted faults (dropped request or reply, duplicate,
  corrupt, fragmented, truncated then valid, delayed) and a transmission log
  with collision detection.
- `LinkRig`: fake Pi, fake bus and a client on a stepped clock;
  `rebootBrain()` replaces the client as a power cycle does.

| Test file | Covers |
|---|---|
| `fake_pi_gtest.cpp` | the fake Pi's session, dedupe, placement, selection and restart rules |
| `client_gtest.cpp` | readiness after a correlated state, fresh GET_STATE ids, fragmented, truncated, corrupt, duplicate and dropped replies, same-byte retries, stale replies (old id, old session, old nonce, bytes before send), Pi restart, unknown session, in-flight placement lost and never resent, Brain reboot, placement applied only after its anchor, request id wrap, stale HELLO, unsupported version and op, link loss, write failures, selection statuses and retries |
| `client_timing_gtest.cpp` | the default timeout meets the budget, no Brain transmission while a reply can start, retry after the latest possible reply, gap and poll period, a too short timeout collides |
| `driver_gtest.cpp` | empty start, units, placement range and heading normalization, validity flags and anchor rules, pose gated while a placement is pending, timed out placement, ages, frame generations, landmark statuses, stale measurement and link loss seen by a Navigator, nothing resumes |
| `equivalence_gtest.cpp` | one Navigator scenario (absolute goal, landmark relative goal, waypoint path) on `SimulatedSource` and on `Driver` through the fake Pi; both complete at the same pose |

The real Pi runtime is tested against the real `Client`, `Driver` and
`Navigator` in `pi/navigatr/tests/brain_link_e2e_gtest.cpp`
(`ctest -R BrainLinkEndToEnd` in the Navigatr build): Brain reboot while the Pi
keeps running, Pi restart and reset, placement retries, and a placement
followed at once by a command. That test compiles the Brain sources from an
explicit list in `pi/navigatr/tests/CMakeLists.txt`, so a new or renamed
library source must be added there as well as in `gatr2_brain.mk`.

## Limitations and unvalidated items

- Host tested only. Not validated: the V5 smart port switching its own RS-485
  direction, whether it hears its own transmission (echoed request frames
  would only count as `bad_frames`), real V5 serial write and read latency,
  PROS task timing under load, and every robot-specific value.
- Landmark wire ids are 1..255.
- One placement at a time; only the latest ticket is tracked.
- No latency compensation: ages are reported, not used to predict the pose.
