# Localization test

A standalone PROS app for checking naviGATR localization with the robot on
the bench or floor. It uses communiGATR to set the robot's field pose and
read it back, shows live status on the Brain, and optionally logs to the USB terminal.
Move the robot by hand; this app has no motor control or navigation commands.

The current bench fallback is selected by `kUseVexImuBench = true`: VEX IMU on
port 1, Pi link on port 10. Follow the [VEX IMU bench guide](../../pi/navigatr/docs/vex_imu_bench.md)
and use `bench_vex_imu.xml` on the Pi. It uses approximate arrival timing and
editable, unmeasured wheel defaults. The Brain displays the VEX IMU reading
even before its Pi link connects. For the original BNO08X setup described below,
set `kUseVexImuBench = false` and rebuild.

## Configure

Edit [include/robot_config.h](include/robot_config.h):

- `kNavigatrPort = 10`: the confirmed V5 Smart Port connected to the Pi's RS-485 link.
- `kNavigatrBaud = 115200`: must match the Pi's `brain_uart` resource.
- `kStartX`, `kStartY`, `kStartHeadingDegrees`: where the robot is when you start
  the program. Defaults are `(0, 0, 0)` for a bench test. Position is meters;
  heading is entered in degrees and converted to radians for the driver.

These are **field coordinates**. In the field diagram, +x points right, +y
points up, and heading 0 faces +x; counterclockwise turns increase heading.
The robot's reported point is the origin configured in the Pi's wheel geometry,
usually the turning center. Changing sides does not change the coordinate axes.

## Prepare the Pi

Follow the [parallel-wheel bring-up guide](../../pi/navigatr/docs/parallel_wheel_bringup.md)
for the Pico firmware, gyro calibration, wheel measurements and Pi serial setup.
Use **profile A**, with two parallel tracking wheels and the IMU.

The real robot file must exist before launching that profile. Copy
[`gatr2_parallel_wheels_bno08x.xml.in`](../../pi/navigatr/config/shared/robots/gatr2_parallel_wheels_bno08x.xml.in)
to `gatr2_parallel_wheels_bno08x.xml` in the same folder and replace every
`@...@` token with a measured or confirmed value. The template is intentionally
not runnable. Remaining tokens are wheel geometry and wheel directions. It
already uses 4000 counts/revolution for default AS5047P encoders coupled 1:1
to the wheels, and IMU `invert="false"` for the Pico's aligned yaw.
No camera calibration is needed for profile A.

From `pi/navigatr` on the Pi, after building:

```sh
./build/navigatr --config_file=config/override/diagnostics/parallel_wheels_bno08x.xml
```

Use the normal threaded runtime with the Brain link. Keep the robot stationary
and level for about six seconds: at least two for Pico startup gravity
alignment, then about four for Pi gyro bias calibration. The IMU itself can
be mounted sideways or upside down. Both wheels
measure forward/backward travel: this configuration assumes no sideways motion
and cannot measure sideways slip or a sideways push.

## Build and upload the Brain app

Open **this folder** as the PROS project; it already contains `project.pros`
and the application entry points. Use a terminal with the PROS CLI and ARM
toolchain available. From the repository root:

```sh
cd brain/localization-test
pros c apply kernel@4.2.2 --force-apply
pros make
pros upload --slot 2 --name localization-test --after screen
```

The kernel restore is needed on a fresh checkout. If already cached, append
`--no-download` to the `pros c apply` command to restore offline. Kernel headers,
firmware libraries and build output are ignored by Git. The app compiles the
shared Brain libraries directly from this repository, so keep its folder here.

The example upload uses Brain program slot **2**; choose another slot if needed.
Start `localization-test` on the Brain. USB logs default off for battery-only use;
the Brain display and Pi link stay active. For text output, set `kUsbDebug = true`
in `robot_config.h`, rebuild/upload, and connect a USB terminal:

```sh
pros terminal
```

## Run the test

1. Place the robot at the configured starting pose and start the Brain program.
   The app opens port 10 and polls in a background task, including while disabled.
2. It waits up to ten seconds for a connected session and submits the starting
   pose once. `Placement: applied` means the Pi acknowledged it and a subsequent
   state confirmed the anchor. Placement and valid sensor data are separate:
   wait for `Pose: LIVE` before evaluating motion.
3. Push forward and rotate. Read x/y in meters and heading in degrees. The
   display also shows measurement age, connection state and reply/timeout counts.
4. To repeat, place the robot at the configured starting pose again and press
   **controller A** or tap the **bottom screen button**. This changes the Pi's
   field anchor; it does not recalibrate the IMU or reset encoder hardware.

When enabled, USB logging runs once per second. `LIVE` requires a connected
link, valid pose and measurement age at most 250 ms. A cached pose can still be
displayed as `STALE`; its numbers are not a current measurement. Missing poses
show dashes, not fabricated zeroes.

If startup cannot connect, the display stays responsive and the driver keeps
polling. Once connected, press A/tap to place. Disconnects and Pi restarts do
not automatically resend the starting pose. A placement timeout has an unknown
outcome: check the robot's position before explicitly submitting another one.
Restarting the Brain program starts a new initialization and starting-pose attempt.

For the 3D viewer, forward its port from the computer:

```sh
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
```

Open `http://127.0.0.1:8765/`. The field and robot display work with profile A;
the camera panel has no camera configured. The Brain screen and viewer read the
same Pi localization. Testing on an open floor does not enforce field obstacles.
Startup gravity alignment supplies yaw for a fixed IMU mounting; it does not
send live pitch/roll, so the model remains level.

## Code to change

[src/main.cpp](src/main.cpp) contains the whole test. The driver calls are:

```cpp
driver.start();
auto ticket = driver.submitPlacement(robot_config::kStartPose);
auto result = driver.placementResult(ticket);
auto sample = driver.latest(communigatr::ProsDriver::now());
// sample.robot.pose: x/y in meters, heading in radians.
// Check sample.connected, sample.robot.valid and sample.robot.age together.
```

The driver owns serial polling; the application owns placement and display.
It uses investiGATR's shared pose types without constructing a Navigator.
See [communiGATR](../../docs/communigatr.md) for the full driver API.

Hardware operation must still be checked on the robot; compiling and host
protocol tests do not validate the physical RS-485 link or sensor calibration.
