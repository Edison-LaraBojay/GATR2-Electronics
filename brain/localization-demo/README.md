# Encoder localization demo

```
tracking wheels -> Pico (hat2_encoder_demo) -UART-> Pi encoder-relay -USB-> Brain localization-demo -> spectaGATR
                                                                              ^ VEX IMU, port 20
```

The Pico counts two encoder channels and sends a 16 byte record 50 times a
second. The Pi forwards each record unchanged, as one `ENC1:` hex text line.
The Brain combines the counts with the VEX IMU heading, computes the pose and
publishes it with `viz.updatePose`. Nothing moves the robot: push it by hand.

| Part | Where |
|---|---|
| Record format and codec | [encoder-record/encoder_record.h](../../encoder-record/encoder_record.h) |
| Pico firmware | [pico/aggreGATR](../../pico/aggreGATR), env `hat2_encoder_demo`, `src/encoder_demo/` |
| Pi relay | [pi/encoder-relay](../../pi/encoder-relay) |
| Brain program | this folder. Math: `src/odometry.cpp`. Record handling: `src/encoder_tracker.cpp`, `src/localization.cpp` |
| Calibration and start pose | [include/localization_config.h](include/localization_config.h) |

Hardware: Pi HAT v2. Sideways wheel on encoder channel 0 (GP0/GP1, J2).
Forward wheel on channel 1 (GP2/GP3, J3). Pico UART TX GP16 and RX GP17 to the
Pi at 115200 baud, `/dev/ttyAMA0` on the Pi. Brain USB to a Pi USB-A port
(data cable). VEX IMU on Smart Port 20. These match the archived profile
(commit 10c568e) and the HAT v2 pin map, with no conflicts.

## 1. Pico

USB to the laptop, from `pico/aggreGATR`:

```sh
pio run -e hat2_encoder_demo -t upload
```

This replaces the sensor firmware. The default envs (`hat2_bno08x`, `hat2_asm330`)
are unchanged. The firmware has no IMU code, needs no receiver, and never resets
its counts. Every boot picks a new random run id, so the Brain can detect a
restart. `pio device monitor` shows the status twice a second:

```
encoder demo run=3F2A seq=1234 ch0=-152 ch1=8801 sent=1235 skipped=0
```

## 2. Brain

Kernel files are not in git. Copy `firmware/` and `include/pros/` from
`brain/spectaGATR-demo` into this folder, or run
`pros c apply kernel@4.2.2 --force-apply --no-download` here and then
`git checkout -- .gitignore`. Then, with the Brain on the laptop USB:

```sh
pros make
pros upload --slot 4 --name localization-demo --after screen
```

Move the Brain USB cable back to the Pi afterwards.

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

To measure scale, push the robot 23 in along a tape, straight and without
turning. Then set `scale = old_scale * true_distance / shown_distance`.

## 7. Startup messages

Brain screen, in order:

1. `IMU calibrating: keep the robot still` (about 2 s). `IMU NOT FOUND on its
   port, retrying` means check port 20.
2. `Waiting for encoder records from the Pi` until a record arrives that is
   at most 250 ms old.
3. Status rows:
   - IMU state;
   - `Encoders ok: run 3F2A seq ..., last advance`;
   - the raw counts;
   - the pose in inches and degrees;
   - the lost-data line;
   - the link counters;
   - the spectaGATR line count.

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

## 8. Sign tests

Start the program at heading 0 and push by hand:

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

**Controller route (UNTESTED with the Pi on the Brain USB).** Plug the controller
into the laptop, then run `spectagatr --source controller`. The Brain may hand
each stdout byte to only one host. If so, the Pi, which reads and discards the
Brain's console output, would take the pose lines from the controller. Test it
first:

1. The Brain screen line `spectaGATR: lines` should grow.
2. The viewer should update at about 10 Hz without gaps.
3. In the relay status, `brain_bytes` grows when the Pi receives the Brain's
   output.

If lines grow but the viewer shows nothing or gaps, use the route below.

**Through the Pi (no controller).** Stop the service and pipe the Brain's
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
| Pico `hat2_encoder_demo` build (default envs still build) | builds, no warnings |
| Brain PROS 4.2.2 build | builds, no warnings |
| Host tests (codec, odometry, record handling, relay, port discovery, end-to-end) | pass |
| Relay `main.cpp` and `serial_port.cpp` | Not built on Linux. Syntax-checked only, against stub POSIX headers. |
| Anything on hardware: Pico UART, relay on the Pi, Brain stdin input, IMU signs, the controller route next to the Pi USB link | Not run. |
