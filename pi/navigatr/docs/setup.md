# Set up and run Navigatr

Start here to bring up a Raspberry Pi 4, Pico, IMU, tracking wheels, and camera,
then see robot localization and landmark estimates in the browser.

**The primary path is a Brain-profiled config**: the robot is described in
Brain C++ and sent to the Pi as a robot profile, so the Pi XML holds no robot
geometry. That path is the next section. The rest of this guide covers the
XML-configured profiles (**three tracking wheels + IMU + camera**, and the
**two parallel tracking wheels + BNO08X** [bring-up guide](parallel_wheel_bringup.md)),
which still run and are needed for a camera. Their XML files are templates:
actual wheel measurements, camera calibration, mounting, and starting position
must be supplied.

## Brain-profiled robot (primary)

This runs the current bench setup (forward wheel on encoder port 0, sideways
wheel on port 1, Brain VEX IMU, USB) and every other supported setup the Brain
selects: two wheels or two forward wheels with the Pico IMU or the VEX IMU, and
three wheels with or without the Pico IMU. No camera or AprilTags are needed.

1. Prepare the Pi and Pico as in [Prepare the Pi and Pico](#prepare-the-pi-and-pico).
   The Pico UART is `/dev/ttyAMA0`; RS-485 also needs UART5 and the DE GPIO.
2. Build without the camera backend and install the service with the config
   that matches the Brain's link (see [Pi setup](../../../docs/pi_setup.md)):

   ```sh
   cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_TESTS=OFF -DNAVIGATR_WITH_LIBCAMERA=OFF
   cmake --build build-bench -j2
   sudo bash tools/install_service.sh --user "$USER" --config config/override/brain_profile_usb.xml
   ```

   For RS-485 use `config/override/brain_profile_rs485.xml` instead. The two
   differ only in the Brain link; this is the one setting the Brain cannot
   change for the Pi.
3. Before a Brain connects the Pi waits: the log and the viewer show no pose,
   profile none, and the raw encoder counts in the sources table.
4. Start the Brain program (`brain/localization-test` or `brain/testing`, see
   [Brain setup](../../../docs/brain_setup.md)). It sends the profile; the Pi
   logs `event: profile <id> applied (...)` with the wheels, IMU and footprint,
   or the refusal reason. Hold the robot still while the IMU starts.
5. The Brain places the robot at its start pose. The viewer's Brain link panel
   then reads `ready`.

Edit robot geometry, encoder settings, the IMU source and the footprint in
[brain/robot/gatr2_robot.h](../../../brain/robot/gatr2_robot.h), then rebuild
and upload the Brain program; no Pi change or restart. Details, refusals,
calibration and recovery: [Brain robot profiles](brain_profile.md).

## Choose your configuration

| Hardware | Main configuration template | Robot measurement template | Pipeline template |
|---|---|---|---|
| Any supported setup, robot described on the Brain | [brain_profile_usb.xml](../config/override/brain_profile_usb.xml) or [brain_profile_rs485.xml](../config/override/brain_profile_rs485.xml) | none: the Brain sends it | inside the main file |
| Three wheels + IMU + camera | [three_wheel_imu_camera.xml.in](../config/override/diagnostics/three_wheel_imu_camera.xml.in) | [gatr2_as5047_imu.xml.in](../config/shared/robots/gatr2_as5047_imu.xml.in) | [three_wheel_imu_camera_pipeline.xml.in](../config/override/diagnostics/three_wheel_imu_camera_pipeline.xml.in) |
| Two parallel wheels + BNO08X, no camera | [parallel_wheels_bno08x.xml](../config/override/diagnostics/parallel_wheels_bno08x.xml) | [gatr2_parallel_wheels_bno08x.xml.in](../config/shared/robots/gatr2_parallel_wheels_bno08x.xml.in) | [parallel_wheels_bno08x_no_camera.xml](../config/shared/pipelines/parallel_wheels_bno08x_no_camera.xml) |
| Two parallel wheels + BNO08X + camera | [parallel_wheels_bno08x_camera.xml](../config/override/diagnostics/parallel_wheels_bno08x_camera.xml) | same | [parallel_wheels_bno08x_camera.xml](../config/shared/pipelines/parallel_wheels_bno08x_camera.xml) |

The camera profiles use [gatr2_front_camera.xml.in](../config/shared/robots/gatr2_front_camera.xml.in),
and all use the nominal [Override field definition](../config/override/field.xml).
The three-wheel template uses encoders **A + B + C**. The parallel-wheel
profiles use encoder channels 0 and 1 only, share one localization fragment,
get their starting field pose from the Brain, and need no third wheel; follow
the [parallel-wheel guide](parallel_wheel_bringup.md) for them.

For an XML-configured profile, follow these sections in order:

1. [Prepare the Pi and Pico](#prepare-the-pi-and-pico).
2. [Build and check the viewer](#build-and-check-the-viewer).
3. [Create your robot configuration](#create-your-robot-configuration).
4. [Measure the wheels and IMU sign](#measure-the-wheels-and-imu-sign).
5. [Bring up and calibrate the camera](#bring-up-and-calibrate-the-camera).
6. [Configure camera mounting and field placement](#configure-camera-mounting-and-field-placement).
7. [Run and check the robot](#run-and-check-the-robot).
8. [Connect the Brain](#connect-the-brain).
9. [Find the outputs](#find-the-outputs).

## Prepare the Pi and Pico

Install Raspberry Pi OS with SSH enabled and configure the account, hostname,
and network. The camera backend targets the libcamera stack used by Bookworm
and newer Raspberry Pi OS releases. Connect from your computer:

```sh
ssh <user>@<pi-host>
```

On the Pi, install the build and camera tools and obtain this repository:

```sh
sudo apt update
sudo apt install git build-essential cmake pkg-config libcamera-dev rpicam-apps libgtest-dev
git clone <repository-clone-url> GATR2-Electronics
cd GATR2-Electronics/pi/navigatr
```

An existing `rpicam-apps-lite` installation is also sufficient for headless
camera checks. Unless a block says otherwise, subsequent Pi commands run from
`GATR2-Electronics/pi/navigatr`. Reuse an existing checkout instead of cloning
over it. See [Pi access](../../../docs/pi_setup.md) for SSH troubleshooting.

### Pico firmware and wiring

Install [PlatformIO for VS Code](https://docs.platformio.org/en/stable/integration/ide/vscode.html)
on the development computer, open this repository's `pico` project, and run from
its PlatformIO terminal:

```sh
cd pico
pio run -e hat2_bno08x
pio run -e hat2_bno08x --target upload
```

Use the board revision's schematic and [Pico pin map](../../../pico/src/board.h)
for connections. The firmware reads the encoder A/B quadrature channels and the
IMU through the Pico: `hat2_bno08x` for the BNO08X, `hat2_asm330` for the
ASM330. It sends binary telemetry to the Pi at 115200 baud. [Pico firmware](../../../pico/README.md) and
[hardware interfaces](../../../docs/hardware.md) describe the pin map and protocol;
the [Pico link](pico_link.md) describes its identity, status and commands.
Update the Pi software before flashing new Pico firmware: the new firmware
sends sensor frames that older Pi builds drop.

The BNO08X learns a fixed up axis during a stationary, level startup and
projects gyro XYZ onto it, supporting sideways or upside-down IMU mounting.
ASM330 firmware still reads physical gyro Z. Both send yaw rate without live
roll/pitch; the camera profiles use the documented level assumption until an
attitude source is added. Startup alignment does not compensate dynamic rocking.
Camera mounting pitch/roll are still configured and applied.

### Pi UART configuration

Use `sudo raspi-config` to disable the serial login console and enable serial
hardware. For the HAT's Brain UART, add the following to
`/boot/firmware/config.txt` and reboot:

```ini
enable_uart=1
dtoverlay=uart5
```

Check the resulting device names:

```sh
ls -l /dev/serial* /dev/ttyAMA*
```

The robot templates select `/dev/ttyAMA0` for the Pico and `/dev/ttyAMA5` for the
Brain. Confirm these against the actual GPIO routing and OS device mapping and
edit each resource's `<Device path="..."/>`. `/dev/serial0` identifies the
primary UART and may resolve to a different device. On a Pi 4, assigning PL011
UART0 to the primary pins can require a Bluetooth/UART overlay choice; follow
the [Raspberry Pi UART documentation](https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts)
for the wiring you actually use.

If the serial devices belong to `dialout`, grant the runtime account access and
log out and back in:

```sh
sudo usermod -aG dialout "$USER"
```

The Brain link also uses GPIO6 as the RS-485 DriverEnable, through sysfs
(`/sys/class/gpio`). That needs GPIO access for the runtime account; serial
group membership alone does not provide it, and the configured number may need
the kernel's sysfs offset (see [Connect the Brain](#connect-the-brain)). For
initial viewer-only bring-up, select `<CommandCollection type="noop"/>` and
`<Publishing type="noop"/>` together in your completed pipeline file (the
brain link slots are a pair) and remove the `brain_uart` resource from your
completed robot file. This avoids opening or driving a Brain link you are not
testing.

## Build and check the viewer

Build on the Pi with the camera backend enabled:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_WITH_LIBCAMERA=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

If the Pi runs out of memory while compiling, use `-j2` or `-j1`. Ordinary
desktop builds can run simulation but do not provide Pi camera capture;
Windows build commands are in the [README](../README.md#build).

First verify the software without hardware:

```sh
./build/navigatr --config_file="config/demo/synthetic_field_demo.xml"
```

On your computer, open a second terminal for the SSH tunnel:

```sh
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
```

Open `http://127.0.0.1:8765/` in your computer's browser. You should see the 3D
field, moving robot and trail, nominal and estimated goals, and a synthetic
camera image with real detector output. The browser performs the 3D rendering;
the Pi needs no desktop. Keep the tunnel running. Ctrl-C stops Navigatr cleanly.

## Create your robot configuration

Copy only the set you need. These commands preserve an existing destination;
edit the completed `.xml` files, keeping the `.xml.in` templates for reference.

For **three wheels + IMU + camera**:

```sh
cp -n config/shared/robots/gatr2_as5047_imu.xml.in config/shared/robots/gatr2_as5047_imu.xml
cp -n config/shared/robots/gatr2_front_camera.xml.in config/shared/robots/gatr2_front_camera.xml
cp -n config/override/diagnostics/three_wheel_imu_camera_pipeline.xml.in config/override/diagnostics/three_wheel_imu_camera_pipeline.xml
cp -n config/override/diagnostics/three_wheel_imu_camera.xml.in config/override/diagnostics/three_wheel_imu_camera.xml
```

For **two parallel wheels + BNO08X**, see the
[parallel-wheel guide](parallel_wheel_bringup.md#3-fill-in-the-robot-description).
That template needs only wheel geometry and wheel-direction tokens filled in.
It uses 4000 counts/revolution for default AS5047P settings with direct 1:1
coupling, and IMU `invert="false"` for the Pico's aligned yaw. Check the encoder
count when testing; change it if its settings or gearing differ.

Replace every `@...@` parameter in the selected files. A `.xml.in` file or an
unresolved parameter is not runnable. `calibration_status` is only a note for
the reader; setting it to `verified` does not validate or enable anything.

| File | Information you put there |
|---|---|
| Main `*_imu_camera.xml` | Referenced files, loop/inspection rates, visible robot-body dimensions. |
| Robot `gatr2_*as5047_imu.xml` | Serial ports, encoder CPR/sign, gyro sign, wheel radius/position/direction. |
| Camera `gatr2_front_camera.xml` | Camera selection, capture mode, lens intrinsics, camera mounting, tag corner size. |
| Local `*_imu_camera_pipeline.xml` | Selected wheels, gyro bias settings, estimator, initial field pose, history, association gates, brain link slots. |
| `config/override/field.xml` | Nominal field and landmark/tag-mount geometry. Competition displacements are estimated at runtime. |

The main document references the robot and camera fragments separately. Paths
are relative to the XML containing them. Keep this directory layout or update
the references when moving a file. Inline sections and `file="..."` references
are supported as described in [configuration](configuration.md).

## Measure the wheels and IMU sign

Choose one fixed robot origin near the nominal turning center. Use this same
point for wheel geometry, camera mounting, initial field placement, and the
reported robot position. Robot axes are **+x forward, +y left, +z up**; heading
is positive counterclockwise viewed from above.

| XML value | How to determine it |
|---|---|
| Sensor `counts_per_revolution` | Counts emitted by the Pico for one shaft revolution, including its quadrature decoding. Confirm by several full turns; do not substitute an encoder datasheet number without checking the configured output. |
| Sensor `invert` | Electrical sign normalization. Rotate in the defined positive shaft direction and check the reported direction. |
| Wheel `radius_m` | Effective rolling radius under robot load. Measure travel over several turns; radius = travel / (2 pi x revolutions). |
| Wheel `position_x_m`, `position_y_m` | Wheel contact-point coordinates relative to the robot origin. |
| Wheel `measurement_angle_deg` | Rolling direction measured from robot +x toward +y: 0 forward, 90 left. |
| Wheel `direction` | Whether positive calibrated shaft rotation measures travel along (`positive`) or opposite (`negative`) that rolling direction. |
| IMU `invert` | A counterclockwise robot turn must produce positive yaw rate. Keep `false` with aligned BNO08X firmware; physical gyro-Z firmware depends on mounting. |

Check electrical inversion and geometric direction together; avoid changing both
to compensate for the same error. Names such as `left_wheel` are references,
not an assumed geometry: the numeric position and direction define the model.
Two wheels must observe different translation directions and require the gyro
constraint, unless both measure forward: then the model builds only with an
explicit `<LateralMotion assume="zero"/>`, which leaves sideways motion
unmeasured. The supplied three-wheel hardware template also uses the gyro
constraint.

At startup leave the robot stationary and level until `tracking_motion` reports ready. The
templates collect 200 gyro samples for bias calibration; with 50 Hz telemetry
this is roughly four seconds after usable samples begin. BNO08X first needs at
least two seconds and 200 stationary acceleration reports for startup alignment,
so allow about six seconds total. The [bring-up guide](parallel_wheel_bringup.md#5-check-the-sensors-in-this-order)
describes its stationarity checks. Motion can restart calibration. Gravity
supplies up; the Brain's placement supplies field heading. The templates select `planar_motion_integrator`; the
[fusion guide](localization_fusion.md) describes the input and noise settings
for `weighted_planar_fusion`. Do not add a second
independent gyro contribution when it is already included in the wheel model.

## Bring up and calibrate the camera

### Check capture before adding metric calibration

Use the [Raspberry Pi camera software](https://www.raspberrypi.com/documentation/computers/camera_software.html)
to check discovery:

```sh
rpicam-hello --list-cameras
```

Use camera auto-detection or the module's documented overlay. The
[camera backend guide](pi_camera_setup.md) covers device selection, libcamera,
supported capture format, and startup diagnostics. Stop any other program using
the camera before starting Navigatr.

Edit [gatr2_front_camera_uncalibrated.xml](../config/shared/robots/gatr2_front_camera_uncalibrated.xml):
select `Device index`, `Capture width_px`, `height_px`, and `frame_rate_hz`.
The starting selection is 1280 x 960 at 30 Hz, `pixel_format="Y8"`; confirm the
installed camera supports the requested output and rate. Record the actual
camera ID reported in diagnostics, particularly with multiple cameras attached.

```sh
./build/navigatr --config_file="config/override/diagnostics/live_camera_inspection.xml"
```

Through the same SSH tunnel, the camera panel should show images and decoded tag
IDs. Intrinsics and camera mounting are not needed for this 2D check. This profile
does not produce robot localization or measured landmark positions.

### Capture calibration images from the same camera path

Use the included [calibration helper](../tools/calibrate_camera.py), which runs
OpenCV on your computer or on the Pi. It does not become part of the C++ runtime.
OpenCV's [calibration tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
explains the camera matrix and five distortion coefficients; its
[pattern guide](https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html)
describes printable targets.

Use a flat chessboard with known square size. `--cols` and `--rows` count
**inner corners**, not squares. For example, a board of 10 x 7 squares has
9 x 6 inner corners. Measure the printed square size instead of trusting printer
scaling. Keep the same lens, focus, capture size, rate, and camera/backend settings
for calibration and operation. Changing the sensor crop can invalidate calibration
even when output dimensions match. Focus control is not exposed in the current
XML; a changing-focus camera needs its focus behavior resolved before trusting
one calibration.

Stop Navigatr and, in `live_camera_inspection.xml`, set:

```xml
<Inspection enabled="true" bind="127.0.0.1" port="8765"
            snapshot_hz="20" preview_hz="5"
            preview_quality="100" preview_max_width="1280"/>
```

Use your **actual capture width** for `preview_max_width`, then restart the live
camera profile. The default 640-pixel preview is too small to calibrate a
1280-pixel stream directly. The HTTP image route uses these XML settings;
changing the browser's preview controls or adding width query parameters does
not change that route. It supplies unannotated JPEGs from the actual detector
frames, including frames containing no AprilTags. JPEG is lossy even at quality
100, but setting the width this way avoids resizing. The helper rejects images
whose dimensions do not match the requested calibration dimensions.

On your **viewing computer**, with a checkout of this repository and the SSH
tunnel running, change to `pi/navigatr`. Create an isolated Python environment:

```sh
python3 -m venv build-camera-tools/venv
./build-camera-tools/venv/bin/python -m pip install opencv-python-headless numpy
```

On **Windows PowerShell**, use:

```powershell
py -3 -m venv build-camera-tools/venv
.\build-camera-tools\venv\Scripts\python.exe -m pip install opencv-python-headless numpy
```

In the following commands, replace `python` with that environment's Python
executable. For the 1280 x 960 example:

```sh
python tools/calibrate_camera.py capture --url "http://127.0.0.1:8765/api/frame.jpg?camera=front_camera" --width 1280 --height 960 --output-dir build-camera-tools/front-images
```

Move the board to different positions, distances, and tilts, including the image
edges. Hold it still and press Enter for each capture; enter `q` when finished.
Aim for 20-30 clear, varied views. The helper requires at least ten distinct
usable images, but image count alone does not establish a good calibration.
It refuses an existing output directory; use a new directory for another run.

For a measured 25 mm square, 9 x 6 inner-corner board:

```sh
python tools/calibrate_camera.py calibrate --images build-camera-tools/front-images --width 1280 --height 960 --cols 9 --rows 6 --square-size-m 0.025 --calibration-id front-1280x960-run1 --frame-id front_camera_engineering --output-dir build-camera-tools/front-calibration
```

Substitute your own board dimensions and square size. The outputs are:

- `calibration.xml`: a complete camera `<Calibration>` fragment.
- `calibration.json`: camera matrix, distortion coefficients, image size,
  reprojection errors, and input provenance for review.

Review rejected views and per-view errors; repeat with better coverage or sharper
images when needed. A low fitting error alone does not verify physical range
accuracy. Keep the images and report associated with the calibration ID.

### Put calibration into the camera XML

In your completed `gatr2_front_camera.xml`:

1. Set `Device` and `Capture` to the exact selection used to collect the images.
2. Replace the whole placeholder `<Calibration>...</Calibration>` with
   `calibration.xml`. Copy it to the Pi if calibration ran on your computer.
3. Keep the `Extrinsic frame_id="front_camera_engineering"` binding, and measure
   the mounting frame as described below.
4. Set the detector family's `detection_size_m` to the measured detector-corner
   edge. For the current field, verify the `tagCircle21h7` family and physical
   size using [field assets](field_assets.md). This is not the carrier-plate width.
   Update the matching `TagMount` sizes in the field definition consistently.

The XML matrix mapping is `fx_px = K[0,0]`, `fy_px = K[1,1]`, `cx_px = K[0,2]`,
and `cy_px = K[1,2]`. Distortion order is `k1, k2, p1, p2, k3`. The helper emits
these named attributes directly. Use the original calibration matrix, not one
computed for a subsequently cropped or undistorted display image. Calibration
board poses returned by the fit do not establish the camera's robot mount.

You can restore the live inspection profile's smaller preview width/quality
after capture. Your main robot profiles use a lower-cost viewer preview while
detection still receives full capture frames.

## Configure camera mounting and field placement

The camera template contains:

```xml
<Frame id="front_camera_engineering" parent_frame_id="robot_body">
    <PoseOfChildInParent x_m="..." y_m="..." z_m="..."
                        roll_deg="..." pitch_deg="..." yaw_deg="..."/>
</Frame>
```

Replace each template token with the measured pose of the **optical center**
relative to your fixed robot origin:

| Attribute | Convention |
|---|---|
| `x_m` | Forward offset; negative behind the origin. |
| `y_m` | Left offset; negative to the right. |
| `z_m` | Height above the robot frame's ground-plane origin. |
| `yaw_deg` | Positive looks left, measured from robot forward. |
| `pitch_deg` | Positive looks down. |
| `roll_deg` | Right-hand rotation about the camera's forward axis; follow the configured Euler convention. |

The configured rotation is `Rz(yaw) * Ry(pitch) * Rx(roll)`. A forward-mounted
camera changes position as the robot turns; the runtime accounts for that offset
using the robot pose at image exposure. Its reported robot x/y remains the robot
origin. See [coordinates](coordinates.md) for the full transform conventions.

In your completed `*_imu_camera_pipeline.xml`, fill:

```xml
<InitialPlacement x_m="..." y_m="..." heading_deg="..."/>
```

This sits inside `Localization`. It describes the robot origin's approximate
starting pose on the field, not the camera location. For the supplied Override
field: origin is the inside bottom-left corner of the audience-view drawing,
+x goes right, +y goes toward the top, and +90 degrees faces +y. Set the real
starting pose each run as needed. With no placement and no placement command,
local odometry alone cannot provide the field prior needed for association.

The main profile's `Inspection/RobotBody` dimensions only affect the drawn box.
Set length, width, height, and the box center's forward/left offset from the
robot origin (`origin_x_m`, `origin_y_m`). They do not substitute for wheel or
camera mounting calibration.

## Run and check the robot

Check your completed XML files for remaining `@...@` values. The camera and
robot files must exist under the names referenced by your main profile.
Place the robot at its configured starting pose, keep it level and stationary
through startup alignment and gyro bias calibration, and run **one** of:

```sh
./build/navigatr --config_file="config/override/diagnostics/three_wheel_imu_camera.xml"
./build/navigatr --config_file="config/override/diagnostics/parallel_wheels_bno08x_camera.xml"
```

Open the same tunneled browser URL. Confirm the source status, localization
readiness, robot position, camera frame sequence, and tag association results.
Startup warnings matter: an unavailable camera or serial link can leave the
process and viewer running while that source supplies no measurements.

Perform short measured checks before relying on estimates:

1. Move forward a known distance: the robot should move in its heading direction
   by that distance. Check a counterclockwise turn, and with three wheels a
   sideways displacement (parallel wheels cannot measure one).
2. Rotate around the robot origin while viewing a stationary landmark: its
   estimated field position should remain approximately fixed. This checks
   mounting offset, wheel geometry, signs, and timing together.
3. View a landmark from several distances and headings. Compare its estimated
   pose against nominal and inspect rejection reasons, frame age, and pose age.
4. If detection work falls behind, inspect worker timing and dropped snapshots.
   Tune detector settings or choose another capture mode; recalibrate when image
   geometry changes. Pose-history coverage must span image-processing latency.

The pipeline exposes `History retention_s`, capacity, and interpolation-gap
limits. A longer retention window does not make an old measurement new. Field
objects retain earlier observations; use the viewer's observation age to judge
freshness. The [camera hardware checks](pi_camera_setup.md#not-verified-on-hardware)
cover exposure timestamps, capture rate, and shutdown behavior that host tests
cannot establish.

### Choose the default main file

To make a completed profile the default for this build:

```sh
cmake -S . -B build "-DNAVIGATR_DEFAULT_CONFIG=config/override/diagnostics/three_wheel_imu_camera.xml"
cmake --build build -j4
./build/navigatr
```

Use the parallel-wheel profile path instead for that robot. `--config_file` overrides the
default for one run. The chosen filename is compiled in; its XML is read each
startup. Editing calibration/configuration requires a restart, not a rebuild.
`./build/navigatr --help` prints the selected default. Ctrl-C stops the service,
workers, and camera cleanly.

## Connect the Brain

The V5 Brain runs a PROS program built on
[communiGATR](../../../docs/communigatr.md). The Brain asks and the Pi answers,
one request at a time, over the Brain's USB port (`pros_usb_link`) or the
HAT's RS-485 link; the Pi never sends unasked.
[Brain link v4](../../../docs/interfaces.md) defines the
requests, sessions, and the timing budget. The Pi side is implemented and host
tested with fake links and clocks; it has not run against a real Brain yet.
The Brain-profiled configs set up both slots as below plus the profile
elements; see [Brain robot profiles](brain_profile.md).

### Pipeline

Every three-wheel and parallel-wheel robot profile selects the brain link in
both slots (the camera inspection profile uses `noop`):

```xml
<CommandCollection type="brain_link">
    <Serial resource_id="brain_uart"/>
    <Reply window_ms="40" turnaround_guard_us="1000"/>   <!-- optional; the defaults -->
</CommandCollection>

<Publishing type="brain_link">
    <Serial resource_id="brain_uart"/>                   <!-- the same resource -->
    <Health fresh_ms="150">                              <!-- optional -->
        <Encoder sensor_id="tracking_encoder_a"/>
        <Gyro sensor_id="robot_imu"/>
        <BiasCal function_id="tracking_motion"/>
    </Health>
    <Field resource_id="override_field"                  <!-- optional -->
           estimate_period_ms="200"/>
    <Pico resource_id="pico_telemetry"/>                 <!-- optional, Pico health bits -->
</Publishing>
```

With a Brain profile, `Health` takes only `fresh_ms`: the encoders and IMU it
reports on follow the running profile. `<BenchImu>` on the CommandCollection
receives the Brain VEX IMU samples, and `<Pico>` there enables the Pico IMU
reinitialize and acquisition restart controls.

Checked when the profile loads:

- `brain_link` command collection and `brain_link` publishing come as a pair on
  the same `Serial` resource. Select `noop` for both to run without a Brain.
- `window_ms > 0` and `0 <= turnaround_guard_us < window_ms * 1000`.
- `1000 / Loop rate_hz <= window_ms / 2`: with the default 40 ms window the
  loop must run at 50 Hz or faster. The supplied profiles run at 100 Hz.
- `Field` names a `field_map` with a `revision`, a `Boundary` and a `wire_id`
  on every landmark; `estimate_period_ms` is positive. See
  [landmarks](landmarks.md#field-documents).

`Reply` bounds when a reply may start: no earlier than `turnaround_guard_us`
after the read that completed the request, and no later than `window_ms` after
the last read of the previous cycle. A reply that would start later is dropped
and the Brain retries. Change the window only together with the Brain's
response timeout (budget formula in the interface document).

Run the executable in its default threaded mode. `--inline` runs world
estimation between a request and its reply, so replies can miss the window;
the executable warns when it is used with the brain link. Each reply blocks the
estimation worker for its airtime at 115200 baud: 5.1 ms for a state reply,
up to 11.1 ms for a full 128-byte document chunk.

### Serial resource

The robot file declares the link:

```xml
<Resource id="brain_uart" type="linux_serial_link">
    <Device path="/dev/ttyAMA5"/>
    <Baud value="115200"/>
    <DriverEnable gpio="6"/>
</Resource>
```

With `DriverEnable` the link is half duplex: DE is low while idle, high only
while the Pi sends a reply, and released once the UART transmitter is empty
plus a two-character guard. Errors and timeouts also release it. The optional
`post_guard_us` and `tx_margin_us` attributes and the failure results are in
[linux_serial_link](../../../docs/navigatr_resources.md#linux_serial_link).

`gpio` is the sysfs number. Some kernels offset it: compare
`cat /sys/class/gpio/gpiochip*/base`; with a base of 512, GPIO6 is
`gpio="518"`. When the GPIO cannot be opened the profile still loads, with a
warning ending in `half-duplex writes will fail`, and every reply then fails
(counted as `tx_errors`).

### Placement and sessions

Only the three-wheel camera profile has an `InitialPlacement`; a Brain SET_POSE
re-anchors it. The other hardware profiles get the starting field pose from
the Brain's SET_POSE when its program initializes. The Pi answers `Pending`
until localization has applied exactly that placement, then `Ok`.

A Brain reboot opens a new session. That withdraws an unapplied SET_POSE, but
it never moves the robot; only a SET_POSE does. A Pi restart or `reset()`
changes `pi_instance`, so the Brain opens a new session.
Localization restarted too, so the robot is unplaced until a placement applies
again: the profile's `InitialPlacement` where configured, otherwise a new
SET_POSE from the Brain program.

With a Brain profile, SET_POSE answers NotReady until a profile is applied, a
new profile leaves the robot unplaced, and a source the profile uses that
drops (stale over 250 ms or restarted) ends the placement: the Brain must place
again. So does motion a model had to discard (a gap past its limit, or
movement while the gyro bias calibrates). A Brain VEX IMU rides on the Brain
link, so with a VEX profile a Brain restart or a link outage over 250 ms needs
a new placement. See
[Brain robot profiles](brain_profile.md#sensor-loss-and-recovery).

### Field documents on the supplied profiles

Every supplied `brain_link` profile publishes the Override field with
`<Field resource_id="override_field" estimate_period_ms="200"/>`. GET_STATE
reports its `map_id` and the newest `estimate_id`, and READ_DOC serves both
documents in chunks.

| Profile | World estimation | Field estimate records |
|---|---|---|
| `brain_profile_usb`, `brain_profile_rs485`, `three_wheel_imu` (`three_wheel_imu_no_correction.xml`), `parallel_wheels_bno08x` (`parallel_wheels_bno08x_no_camera.xml`), `bench_vex_imu`, `bench_vex_imu_usb` | `noop` | every object nominal |
| `three_wheel_imu_camera`, `parallel_wheels_bno08x_camera` | `apriltag` | each landmark nominal until the camera observes it in the current odometry epoch, then observed with its age |

Obstacles (loaders, toggles) are always nominal. See
[landmarks](landmarks.md#field-documents) for the rules and
[field assets](field_assets.md#planning-data) for the collision boxes.

### Check the link

On exit the executable prints the brain link counters once a request has
arrived (`link brain_uart ... requests, replies, ...`). The inspection
snapshot carries the same counters in `diagnostics.estimation.links[]`, the
Brain session in `command`, and in `brain_link` the link state, the state block
the Brain reads, the profile, the wheel readings and the reported path; the
viewer's Brain link panel shows them with a readiness line. See
[inspection](inspection.md). The service log prints lifecycle events as
`event: ...` lines (`journalctl -u navigatr -f`).

| Counter | Meaning |
|---|---|
| `requests` stays 0 | Nothing decodes: device path, baud, wiring, transceiver, or the V5 port. |
| `unanswered` | A request already waiting at the first read after start or reset (applied, not answered; the Brain retries and gets the duplicate answer), or one followed by more bytes in the same cycle. |
| `expired` | The reply window was missed; check `workers.estimation` overruns. |
| `input_pending` | The Brain was still sending when the Pi was about to reply. |
| `tx_errors` | The write failed; check the DriverEnable warning. |
| `late_release` | DE was released later than planned, for example the Pi was preempted. |

### Hardware checks still to do

None of these has been checked on the robot:

- The V5 smart port switches its own RS-485 direction (assumed).
- The sysfs GPIO number of DE on the installed kernel.
- On a scope: DE low while idle, high only during a reply, released after the
  last stop bit.
- `TIOCSERGETLSR` transmitter-empty reporting on `/dev/ttyAMA5`. Without it
  the link falls back to `tcdrain`, which has no deadline of its own.
- Real turnaround and reply latency against the Brain's response timeout.
- If navigatr is killed (SIGKILL, crash) during a reply, DE stays high and
  blocks the Brain until navigatr starts again and drives it low.

## Find the outputs

| Output | Where it goes |
|---|---|
| Robot pose, velocity, attitude status, and recent trail | `RobotStateFeed`; the browser shows localization and pose age. |
| Landmark estimates and their observation/association diagnostics | `FieldSnapshot`; the viewer compares solid observed estimates with ghost nominal objects. |
| Detection images | Camera panel with frame-matched overlays; `/api/frame.jpg?camera=front_camera` supplies the unannotated JPEG. |
| Configuration/geometry and runtime snapshots | Inspection HTTP `/api/hello`, `/api/snapshot`, and live `/ws`; see [inspection](inspection.md). |
| Robot pose, anchor identity, health bits, and the field documents for the Brain | `brain_link` command collection and publishing on `brain_uart` answer each Brain request once and send nothing unasked; positions are millimeters and headings centidegrees on the wire. See [Connect the Brain](#connect-the-brain). |

The supplied hardware profiles select `brain_link` command collection and
publishing and `noop` target resolution. A GET_STATE reply carries the robot
pose, its measurement age, the anchor identity, the configured health bits, and
the ids of the field map and estimate documents, which READ_DOC serves in
chunks with every configured object. Full field heading
is the wire convention; the viewer separately shows heading error from nominal.
See [landmarks and reporting](landmarks.md#brain-output) and the
[wire interface](../../../docs/interfaces.md).

The Pi estimates state and answers requests; Brain code owns robot movement
([investiGATR](../../../docs/investigatr.md) and
[communiGATR](../../../docs/communigatr.md)). The
[bench example](../../../bench/rs485_link/README.md) is a separate
byte-transfer test.

Snapshots and images are not automatically recorded to disk. The calibration
helper explicitly saves its images and reports; runtime estimates otherwise
remain in memory. Stopping or resetting the program clears runtime estimates
and history. It does not write observed displacements back into `field.xml`.

## Troubleshooting

| Symptom | Check |
|---|---|
| Template/token or missing-file error | Complete the selected `.xml.in` files, save `.xml`, and verify each relative reference. |
| Unknown `WorldEstimation` or estimator type | Executable and XML must come from the same source revision; rebuild after source changes. |
| Camera backend not compiled | Configure with `-DNAVIGATR_WITH_LIBCAMERA=ON` on the Pi. |
| Camera unavailable or busy | Startup diagnostic, `rpicam-hello --list-cameras`, selected camera ID, and another process holding capture. |
| Camera works, metric pose unavailable | Intrinsics absent, invalid capture/calibration dimensions, or tag size not supplied. |
| Calibration helper rejects image dimensions | Set XML Inspection preview width to the capture width, restart, and confirm the camera's actual resolution. |
| Calibration finds too few boards | Inner-corner counts, sharpness, complete board visibility, square pattern, varied views. |
| Robot frozen or bias never ready | Pico UART mapping/permissions, firmware output, sensor health, configured channels, and robot motion during calibration. |
| Tags decode but goals stay nominal | InitialPlacement, camera mounting, tag family/size, history availability, and the association rejection reason. |
| Wrong displacement or turn direction | Encoder CPR/sign, wheel radius/geometry, gyro sign, and one shared robot origin. |
| No browser connection | Running inspection service, matching port, SSH tunnel, and another process already using the port. |
| Brain gets no replies | Threaded mode (no `--inline`), the `brain_uart` path and baud, a DriverEnable warning at startup (sysfs `gpio` number, GPIO access), transceiver wiring, then the link counters in [Connect the Brain](#check-the-link). Over USB: the Pi config must be `brain_profile_usb.xml` and exactly one Brain plugged in. |
| Profile refused | The `event: profile ... refused` log line and the viewer name the reason; fix it in `gatr2_robot.h`. An encoder port or IMU the Pi config does not wire is refused, not guessed. |
| Pose keeps becoming invalid | `event: sensor lost: ...` names the source; a VEX profile loses its IMU with the Brain link. |

For the exhaustive parameter list, use [calibration inventory](calibration_inventory.md).
