# VEX IMU bench fallback

Quick push-by-hand test: the Pico supplies encoder channels 0 and 1, the Brain
supplies continuous VEX IMU rotation, and the Pi returns the ordinary robot pose
and serves the field viewer. The Pico BNO08X may remain unavailable. No camera
or path planner is used.

This mode pairs the latest readings by their arrival on the Pi. It does not
synchronize the Brain and Pico clocks. Use slow translation and turns; delay
and jitter can create position error. Sideways motion is assumed zero.

## Brain

Use the existing `brain/localization-test` PROS project. Its
`include/robot_config.h` selects:

- `kUseVexImuBench = true`.
- VEX IMU on Smart Port **1**.
- Pi link on Smart Port **10**.
- Initial field position and heading in `kStartX`, `kStartY`, and
  `kStartHeadingDegrees`.

From `brain/localization-test`, build and upload with PROS (not PlatformIO):

```sh
pros make
pros upload --slot 2 --name localization-test --after screen
```

Choose a different slot if needed. For a fresh checkout, restore the PROS kernel
as described in the [app README](../../../brain/localization-test/README.md#build-and-upload-the-brain-app).
Keep the robot still
while the VEX IMU calibrates. Its local rotation is displayed on the Brain even
if the Pi link has not connected, so the two pieces can be checked separately.
The Brain converts PROS clockwise-positive rotation to counterclockwise-positive
rotation before sending it. The app never commands motors.

## Pi

From `pi/navigatr` on the Pi, using the updated source:

```sh
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_TESTS=OFF
cmake --build build-bench -j2
./build-bench/navigatr --config_file=config/override/diagnostics/bench_vex_imu.xml
```

The single [bench configuration](../config/override/diagnostics/bench_vex_imu.xml)
is runnable without completing the usual robot template. Edit its values:

| Setting | Bench default |
|---|---|
| Pico UART | `/dev/ttyAMA0`, 115200 |
| Brain UART | `/dev/ttyAMA5`, 115200, driver-enable GPIO 6 |
| Encoder counts/revolution | 4000 for each wheel |
| Wheel radius | 0.0254 m: **unmeasured two-inch-diameter default** |
| Wheel lateral offsets | +0.15 m and -0.15 m: **unmeasured defaults** |
| Wheel direction | `positive` for both; change if forward rolling decreases counts |

Actual wheel radius, direction and offsets are still needed for meaningful
distance measurements. `calibration_status` is only a reader annotation.
Serial device names and the GPIO number must match the Pi's configured UARTs
and GPIO numbering; see [Pi access](../../../docs/pi_setup.md). This fallback
still requires a working Brain–Pi physical link.

The Brain opens a session, supplies IMU samples, and requests the initial pose.
If the initial connection window expires, press controller A or the screen's
placement button after it connects. Stale/missing IMU or wheel data stops motion
updates. A source restart or calibration change rebases measurement differences;
it does not replay motion from the gap.

## Viewer

From the viewing computer:

```sh
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
```

Open `http://127.0.0.1:8765/`. The field and robot pose use the normal viewer.
Landmarks remain nominal; no camera/AprilTag correction or measured live tilt is
provided by this bench path.

## Return to the BNO08X setup

Set `kUseVexImuBench = false`, rebuild/upload the Brain app, and use the normal
parallel-wheel Pi profile. The bench mode adds an optional state request carrying
IMU data; ordinary Brain requests keep their existing format. Both Brain and Pi
must be rebuilt to use the extension.
