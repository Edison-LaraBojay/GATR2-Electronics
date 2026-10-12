# Manual mecanum drive and encoder localization demo

**Choose the mode before building:** `kControllerViewerTest` in
[include/localization_config.h](include/localization_config.h) selects the
controller-only simulated-pose test when `true`, or the normal localization
and manual-drive program below when `false`. See section 9 for the test.

```
tracking wheels -> Pico (hat2_encoder_demo) -UART-> Pi encoder-relay -USB-> Brain localization-demo -> spectaGATR
                                                                              ^ VEX IMU, port 20
```

The Pico counts two encoder channels and sends a 16 byte record 50 times a
second. The Pi forwards each record unchanged, as one `ENC1:` hex text line.
The Brain combines the counts with the VEX IMU heading, computes the pose and
publishes it with `viz.updatePose`. The same Brain program drives eight motors,
two per mecanum wheel, using the controller. The left stick translates the
robot and the right stick X rotates it. This is manual, robot-relative drive;
there is no autonomous driving or path following.

**Check motor reversals before driving.** The configured pairs are front-left
1/2, front-right 3/4, back-left 5/6 and back-right 7/8. Their reversal signs
have not been verified. Zero is an unset port and keeps drive disabled.
The complete hardware route has not been tested.

| Part | Where |
|---|---|
| Record format and codec | [encoder-record/encoder_record.h](../../encoder-record/encoder_record.h) |
| Pico firmware | [pico/aggreGATR](../../pico/aggreGATR), default env `hat2_encoder_demo`, `src/main.cpp` |
| Pi relay | [pi/encoder-relay](../../pi/encoder-relay) |
| Brain program | this folder. Math: `src/odometry.cpp`. Record handling: `src/encoder_tracker.cpp`, `src/localization.cpp` |
| Calibration and start pose | [include/localization_config.h](include/localization_config.h) |
| Motor ports, reversals and manual-drive settings | [include/drive_config.h](include/drive_config.h) |

Hardware: Pi HAT v2. Sideways wheel on encoder channel 0 (GP0/GP1, J2).
Forward wheel on channel 1 (GP2/GP3, J3). Pico UART TX GP16 and RX GP17 to the
Pi at 115200 baud, `/dev/ttyAMA0` on the Pi. Brain USB to a Pi USB-A port
(data cable). VEX IMU on Smart Port 20. These match the archived profile
(commit 10c568e) and the HAT v2 pin map, with no conflicts.

## 1. Pico

USB to the laptop, from `pico/aggreGATR`:

```sh
pio run
pio run -t upload
```

The default `hat2_encoder_demo` environment builds the encoder sender from
`src/main.cpp`. Its settings are in `src/encoder_config.h` under `encoder_cfg`.
It has no IMU code, needs no receiver, and never resets its counts while running.
Every boot picks a new random run id, so the Brain can detect a restart.
`hat2_bno08x` and `hat2_asm330` are optional local sensor monitor environments;
select them explicitly only when testing a Pico-connected IMU.
`pio device monitor` shows the encoder status twice a second:

```
encoder demo run=3F2A seq=1234 ch0=-152 ch1=8801 sent=1235 skipped=0
```

## 2. Brain

Motor settings are in [include/drive_config.h](include/drive_config.h). Each wheel has
two independently signed Smart Ports in `drive_config`:

| Setting | Wheel, viewed from the robot | Smart Ports |
|---|---|---|
| `kFrontLeft` | Front left motor pair | 1, 2 |
| `kFrontRight` | Front right motor pair | 3, 4 |
| `kBackLeft` | Back left motor pair | 5, 6 |
| `kBackRight` | Back right motor pair | 7, 8 |

Update these assignments if the wiring changes. Use a negative port to
reverse that individual motor; paired motors can need different signs
because of their mounting or gearing. The absolute port numbers must be
unique, in 1 through 21, and different from the IMU port in
`localization_config.h` (currently 20). Invalid configuration disables drive.
Localization still runs with unset or invalid motor ports.

`kMotorRpm` defaults to 600 RPM, matching the prior robot configuration;
confirm the installed motor cartridges. `kDeadband` is 8 controller units and
`kMaxVoltage` is 12000 mV. The mixer preserves the requested translation and
rotation proportions when limiting wheel commands.
The drive assumes the usual X mecanum roller layout when viewed from above.
Motor braking defaults to coast, allowing the hand-push localization checks.

Kernel files are not in git. From this folder, apply kernel 4.2.2 from the
existing PROS cache if `firmware/` or `include/pros/` is missing:

```sh
pros c apply kernel@4.2.2 --force-apply --no-download
```

This requires kernel 4.2.2 already in the local PROS cache. If PROS changes
`.gitignore`, restore the repository's ignore rules while preserving any
local edits. Then, with the Brain on the laptop USB:

```sh
pros make
pros upload --slot 4 --name localization-demo --after screen
```

Move the Brain USB cable back to the Pi afterwards.

### Controller and motor checks

| Input | Motion |
|---|---|
| Left stick Y | Forward/backward |
| Left stick X | Strafe left/right |
| Right stick X | Turn left/right |

Forward and strafe follow the robot's current orientation. Pose and heading
are for localization and display; they do not transform or steer the manual
drive command. Commands inside the deadband produce no movement.

Keep the sticks centered during startup. Drive waits for IMU calibration and
the first fresh encoder baseline, then requires the sticks to be centered
before it arms. Disabled mode, autonomous mode, loss of controller connection
and controller read errors stop the motors. Center the sticks again after
resuming or reconnecting. Once startup succeeds, a later encoder-link loss
holds the pose and makes the viewer stale, while manual robot-relative drive
remains available.

A motor-command failure stops all drive motors and latches a drive fault.
Check the wiring and port configuration, then restart the Brain program.

For the first motor check, support the robot with all drive wheels clear of
the floor. Use small stick inputs and verify both motors on every wheel work
together; a forward command should turn all four wheels forward. Correct
individual port signs before a floor test. Then check slow
forward, strafe and rotation commands in a clear area. Motor wiring,
reversals and roller orientation still need hardware checks. While supported,
also verify motors stop on controller disconnect and disabled/autonomous mode,
and require centered sticks when control resumes.

## 3. Pi relay

Copy the sources from the laptop (PowerShell, in `C:\GATR2-Electronics`):

```powershell
tar.exe -czf "$env:TEMP\encoder-relay.tar.gz" encoder-record pi/encoder-relay
scp "$env:TEMP\encoder-relay.tar.gz" USER@PI_HOST:/tmp/
```

On the Pi:

```sh
mkdir -p ~/encoder-demo && tar -xzf /tmp/encoder-relay.tar.gz -C ~/encoder-demo
cd ~/encoder-demo
cmake -S pi/encoder-relay -B build-relay -DCMAKE_BUILD_TYPE=Release
cmake --build build-relay -j2
sudo cmake --install build-relay            # /usr/local/bin/encoder-relay
```

Manual run, with the old service stopped (section 5):
`encoder-relay`. The options are `--uart PATH` (default `/dev/ttyAMA0`),
`--baud N` (115200), `--brain auto|PATH`, `--echo-brain` and `--status-s N` (status
log period, default 10 s). Each device is opened exclusively: a second relay
reports `Device or resource busy` and keeps retrying. With `--brain auto`,
the relay picks the Brain's user port: USB 2888:0501, interface 02. It never
picks the system/upload port, whatever its ttyACM number. With two Brains, or to
force a port, pass `--brain /dev/serial/by-id/...-if02`.

The relay keeps only the newest unsent record: counts are cumulative, so no
travel is lost. It reopens either device one second after a failure. It flushes
stale bytes on every open, and it drops queued output when the Brain stops
taking input for 0.5 s. A reconnect therefore never delivers a backlog.

## 4. Start on boot

```sh
sudo cp ~/encoder-demo/pi/encoder-relay/encoder-relay.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now encoder-relay
journalctl -u encoder-relay -f
```

systemd restarts the relay 2 s after a crash. The unit runs as a dynamic user in
group `dialout`.

## 5. Stop the old service first

The old Pi runtime (`navigatr`) uses the same UART and Brain USB port:

```sh
sudo systemctl disable --now navigatr
```

The new unit also has `Conflicts=navigatr.service`. Starting either service stops
the other.

## 6. Calibration and start pose

Edit [include/localization_config.h](include/localization_config.h), then
rebuild and upload. Each value is labelled with its source:

- Start pose: `kInitialXInches`, `kInitialYInches`, `kInitialHeadingDegrees`
  (default 7 in, 24 in, 0 deg, from the archive). Place the robot there before
  starting the program.
- Wheels: diameter about 48 mm and 4000 counts per wheel turn are UNVERIFIED.
  4000 is the x4 decoder count, every A and B edge, for an encoder at the AS5047P
  default of 1000 pulses per rev. It is not the pulses-per-rev figure.
- Scale: the archived 23 inch push tests read about half the true distance
  (forward x2.10, sideways x1.97). Scale carries that correction until the real
  cause, counts per rev or diameter, is found. If you fix either one, set scale
  back to 1.0 and measure again.
- Offsets: forward wheel 0.1115 m right of center, sideways wheel 0.011 m ahead
  of center (UNVERIFIED).
- Directions: both -1, because the archive marks both encoders reversed.

To measure scale, keep the controller sticks centered and push the robot
23 in along a tape, straight and without turning. Then set
`scale = old_scale * true_distance / shown_distance`.

## 7. Startup messages

Brain screen, in order:

1. `IMU calibrating: keep the robot still` (about 2 s). `IMU NOT FOUND on its
   port, retrying` means check port 20.
2. `Waiting for encoder records from the Pi` until a record arrives that is
   at most 250 ms old and establishes the encoder and IMU baselines. Motors
   stay stopped until this startup step completes.
3. Status rows:
   - IMU state;
   - `Encoders ok: run 3F2A seq ..., last advance`;
   - the raw counts;
   - the pose in inches and degrees;
   - the lost-data line;
   - the link counters;
   - the spectaGATR line count;
   - drive status, including an unset or invalid motor configuration.

Relay log:

```
encoder-relay: uart /dev/ttyAMA0 at 115200 baud, brain auto
encoder-relay: uart /dev/ttyAMA0 open
encoder-relay: brain /dev/ttyACM1 open
encoder-relay: pico run 3F2A
encoder-relay: status uart open brain open | records 500 lines 498 superseded 2 ...
```

What happens to the pose:

| Event | Effect |
|---|---|
| First record | Sets the encoder and IMU baselines. It is not counted as travel. |
| Duplicate record, or one up to 256 records older that arrives late | Ignored. |
| Link gap over 250 ms | `ENCODERS STALE: no records, pose held`, and the viewer goes stale. On reconnect, odometry rebases and the gap counter rises. |
| Pico restart | Odometry rebases with no jump. The restart counter rises. |
| Jump (seq far out of order, or more than 20000 counts in one step) | Odometry rebases. The jump counter rises. |
| Invalid IMU reading | Nothing is integrated. Odometry rebases when the IMU is valid again. |

When any of those counters is nonzero, the screen shows
`UNCERTAIN: restarts .. gaps .. jumps .. imu ..`. During those events, x and y
keep their last value, but travel during the event is lost. The heading stays
correct after a gap or a Pico restart, because the IMU stays connected. To trust
the position again, restart the program with the robot at the start pose.

Only a pose backed by fresh encoder records and a valid IMU reading goes to
spectaGATR. While the encoders are stale, the viewer marks the pose stale.
After the first successful startup baseline, localization faults do not
disable manual robot-relative drive; the displayed position can then miss
travel until the program is restarted at the configured start pose.

## 8. Sign tests

Start the program at heading 0, keep the controller sticks centered, and
push by hand:

- Forward: x grows, y stays. If x shrinks, set `w.forward.direction` to +1.
- Left: y grows, x stays. If y shrinks, set `w.sideways.direction` to +1.
- Spin counterclockwise (seen from above): heading increases. After one full
  turn back to the start, x and y should be within about an inch of where they
  started, for a slow spin. The IMU is read when a record is processed, 10 to 30
  ms after the counts were taken, so a fast spin (about 1 turn per second) can
  leave roughly half an inch of error each time it starts or stops. If the position traces a circle while spinning, check the sign of
  each offset: forward wheel right of center is negative, sideways wheel ahead
  of center is positive.

## 9. Visualization

Build the laptop receiver as in [spectaGATR/receiver](../../spectaGATR/receiver/README.md).

### Isolate the controller link with a simulated pose

Set `kControllerViewerTest = true` in
[include/localization_config.h](include/localization_config.h). This mode
publishes a circle near field center using the same spectaGATR publisher as
localization. It never starts the encoder receiver, initializes the drive,
reads or calibrates the IMU, or runs motor control. The Brain screen labels
the pose as simulated and shows the publisher's line and write-error counts.

In Windows CMD, with the Brain connected directly to the laptop for upload:

```bat
cd /d C:\GATR2-Electronics\brain\localization-demo
pros make
pros upload --slot 5 --name controller-viz-test --after screen
```

Use an unused slot; slot 5 here keeps the normal localization program in
slot 4. Disconnect the Brain's USB cable completely after uploading. Power
the Brain with its battery, link the controller by radio, connect the
controller to the laptop by USB, and start `controller-viz-test` on the Brain
in operator control. No Pi or sensors are needed. Close other serial terminals
and any running spectaGATR receiver, then launch in Windows CMD:

```bat
cd /d C:\GATR2-Electronics
build-spectagatr\receiver\spectagatr.exe --source controller --debug-link --radio-download
```

`--radio-download` moves the controller radio to the download channel (robot
disabled while it switches; see the receiver README for what is unverified).
Measured 2026-10-10: on the normal pit channel the controller answers about 3
times a second with 4 poses per reply, so the viewer updates about 3 times a
second however the receiver polls; on the download channel 10 Hz arrives one
pose at a time with 110 to 180 ms gaps. `--debug-link` prints a `delivery`
line once a second: expect `rate_hz` near 10, `seq_lost` 0 and
`poses_per_reply` 1.0.

At `http://127.0.0.1:8790/`, expect LIVE with a continuously moving circle.
If the Brain's line counter keeps increasing but the viewer stays WAITING,
capture the receiver terminal output. If this works, repeat with the Pi
connected to the Brain and its relay running to test USB/radio coexistence.
The pose remains simulated in both cases. The `--echo-brain` SSH route has
been observed working; simultaneous USB/radio viewing still needs this test.

Set `kControllerViewerTest = false`, rebuild, and upload to return to real
localization. Simply selecting an existing normal program in another slot
also restores its behavior without a rebuild. No Pi or Pico changes are needed.

### Real localization

**Controller route (UNTESTED with the Pi on the Brain USB).** Plug the controller
into the laptop, then run `spectagatr --source controller --debug-link
--radio-download`, as in the simulated-pose test above. The Brain may hand
each stdout byte to only one host. If so, the Pi, which reads and discards the
Brain's console output, would take the pose lines from the controller. Test it
first:

1. The Brain screen line `spectaGATR: lines` should grow.
2. The viewer should update at about 10 Hz without gaps.
3. In the relay status, `brain_bytes` grows when the Pi receives the Brain's
   output.

If lines grow but the viewer shows nothing or gaps, use the route below.

**Through the Pi (no laptop-to-controller cable).** The controller can stay
linked to the Brain for manual drive. Stop the service and pipe the Brain's
output from the relay to the receiver. Use `cmd.exe` or Git Bash: PowerShell
5.1 corrupts binary pipes.

```bat
ssh USER@PI_HOST "sudo systemctl stop encoder-relay"
ssh USER@PI_HOST "encoder-relay --echo-brain" | build-spectagatr\receiver\spectagatr --source file --replay -
```

The relay exits when the pipe closes (Ctrl+C). Afterwards:
`ssh USER@PI_HOST "sudo systemctl start encoder-relay"`.

## Tests and status

Host tests (Windows, MinGW GCC 15.2) for the codec, odometry, record handling,
relay forwarding and port discovery, plus an end-to-end path test:

```sh
cmake -S encoder-record -B build-er && cmake --build build-er && ctest --test-dir build-er
cmake -S brain/localization-demo/tests -B build-ld && cmake --build build-ld && ctest --test-dir build-ld
cmake -S pi/encoder-relay -B build-rl -DENCODER_RELAY_BUILD_TESTS=ON && cmake --build build-rl && ctest --test-dir build-rl
```

| Check | Status |
|---|---|
| Pico default `hat2_encoder_demo` build and optional IMU builds | builds, no warnings |
| Brain PROS 4.2.2 build | builds, no warnings |
| Host tests (codec, odometry, record handling, relay, port discovery, end-to-end) | pass |
| Relay `main.cpp` and `serial_port.cpp` | Not built on Linux. Syntax-checked only, against stub POSIX headers. |
| Anything on hardware: motor ports/reversals, eight-motor mecanum motion and stops, Pico UART, relay on the Pi, Brain stdin input, IMU signs, the controller route next to the Pi USB link | Not run. Configure motor ports before driving. |
