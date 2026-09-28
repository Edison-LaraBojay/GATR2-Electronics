# Bring up two parallel wheels + BNO08X

Pi HAT v2, a Raspberry Pi Pico, a GY-BNO08X IMU, and two tracking wheels that
both measure forward/backward travel, configured in Pi XML. Get localization
working first with profile A, then add the camera with profile B. Neither
profile uses a third wheel or encoder channel 2.

**The primary path is a Brain profile.** The same hardware runs from
`brain_profile_usb.xml` or `brain_profile_rs485.xml` with a Brain profile of
two forward wheels and the Pico IMU: the Pi then builds the same models
(`tracking_wheel_motion` with the heading constraint and zero lateral motion)
from the Brain's description, and a geometry change is a Brain upload. See
[Brain robot profiles](brain_profile.md). The current Brain programs always
send a profile; these XML profiles refuse it (reason "not accepted"), and the
Brain then refuses to place, so use them with a Brain client built without a
profile or for camera work. The wiring, Pico and sensor-check sections below
apply to both paths.

| Profile | Main file | Adds |
|---|---|---|
| A: wheels + IMU | [parallel_wheels_bno08x.xml](../config/override/diagnostics/parallel_wheels_bno08x.xml) | Localization, pose history, viewer, field map for display. World estimation is the registered noop. |
| B: A + camera | [parallel_wheels_bno08x_camera.xml](../config/override/diagnostics/parallel_wheels_bno08x_camera.xml) | The front camera, calibration, mount, AprilTag detector and world estimation. |

Both profiles include one shared [localization fragment](../config/shared/pipelines/parallel_wheels_bno08x_localization.xml)
and one [robot description](../config/shared/robots/gatr2_parallel_wheels_bno08x.xml.in),
so wheel and IMU calibration is entered once.

## What runs now and what waits for measurements

| File | State |
|---|---|
| `config/override/diagnostics/parallel_wheels_bno08x.xml` | Complete. Resolves once the robot file below exists; until then it fails and names that file. |
| `config/override/diagnostics/parallel_wheels_bno08x_camera.xml` | Complete. Also needs the camera file below and a libcamera build. |
| `config/shared/pipelines/parallel_wheels_bno08x_*.xml` | Complete. No measured values in them. |
| `config/shared/robots/gatr2_parallel_wheels_bno08x.xml.in` | **Template.** Wheel radius, offsets and directions need measurements. Default AS5047P CPR is 4000; aligned IMU yaw uses `invert="false"`. |
| `config/shared/robots/gatr2_front_camera.xml.in` | **Template.** Camera selection, intrinsics and mounting are unmeasured. |
| `config/demo/synthetic_field_demo.xml` | Runnable now without hardware, for checking the build and the viewer. |
| Pico firmware `hat2_bno08x` | Builds. Not yet run on the robot. |

Nothing here has been validated on hardware. The software tests use synthetic
data only.

## 1. Wiring

Encoders, on the HAT:

| Channel | HAT connector | Pico pins |
|---|---|---|
| 0 | J2 | A = GP0 (pin 1), B = GP1 (pin 2) |
| 1 | J3 | A = GP2 (pin 4), B = GP3 (pin 5) |

BNO08X, by the module's printed labels:

| Module | Pico | Route |
|---|---|---|
| VCC | 3.3 V | J7 pin 7 |
| GND | GND | J7 pin 1 or 8 |
| SCL (SPI SCK) | GP10, pin 14 | J7 pin 5 |
| SDA (SPI MISO) | GP8, pin 11 | J7 pin 3 |
| ADO (SPI MOSI) | GP11, pin 15 | J7 pin 4 |
| CS | GP9, pin 12 | J7 pin 6 |
| INT | GP22, pin 29 | J7 pin 2 |
| RST | GP12, pin 16 | added wire |
| PS0/WAKE | GP13, pin 17 | added wire |
| PS1 | 3.3 V | added wire |

The Pico-Pi UART is unchanged: Pico TX GP16 (pin 21), RX GP17 (pin 22), 115200
baud, `/dev/ttyAMA0` on the Pi. Pin definitions live in
[pico/src/board.h](../../../pico/src/board.h).

## 2. Build and flash the Pico

From the repository on the development computer:

```sh
cd pico
pio run -e hat2_bno08x
```

Flash it by hand. Either hold BOOTSEL while plugging in the Pico's USB and copy
`pico/.pio/build/hat2_bno08x/firmware.uf2` onto the `RPI-RP2` drive, or run
`pio run -e hat2_bno08x --target upload` with the board connected. The
`hat2_asm330` environment builds the older ASM330 IMU instead.

Flash this build even if the Pico already has firmware. The previous firmware
gated every frame on `Serial1.availableForWrite() >= n`, which is always false
on this Arduino core, so it never transmitted.

The BNO08X supplies acceleration at 100 Hz and uncalibrated gyro XYZ at 200 Hz.
During a stationary, level startup, the Pico learns the up direction in the
sensor's frame, then projects gyro XYZ onto it. Sideways and upside-down fixed
IMU mountings work with the same positive counterclockwise yaw convention.
Each 50 Hz frame carries the mean projected rate, with bias left for the Pi
to calibrate. During alignment, missing data or resets, frames keep flowing
with the gyro bit clear. Reset recovery repeats alignment. No live roll or
pitch is sent, and the fixed axis does not compensate dynamic rocking.

## 3. Fill in the robot description

On the Pi, from `pi/navigatr`:

```sh
cp -n config/shared/robots/gatr2_parallel_wheels_bno08x.xml.in config/shared/robots/gatr2_parallel_wheels_bno08x.xml
```

Edit the `.xml` and replace every `@...@` value. Never put zero in just to make
it parse. First choose the robot origin: the point the reported pose refers to.
The drivetrain's turning center is best, because a turn in place then leaves
the reported position still. Axes are +x forward, +y left.

| Token | Value |
|---|---|
| `MEASURE_WHEEL_A_Y_M`, `MEASURE_WHEEL_B_Y_M` | Lateral offset of each wheel's contact point from the origin, left positive. This is the lever arm that removes turning from the wheel travel, so measure it carefully. |
| `MEASURE_WHEEL_A_X_M`, `MEASURE_WHEEL_B_X_M` | Forward offset. It does not affect the forward-only solve, but record it. |
| `MEASURE_WHEEL_A_LOADED_RADIUS_M`, `..._B_...` | Effective rolling radius under load. Start from calipers, then tune it with the push test below. |
| `CONFIRM_WHEEL_A_DIRECTION`, `..._B_...` | `positive` when the count increases as the robot drives forward, else `negative`. This is the only sign knob per wheel; encoder `invert` stays false. |

Wheel A is channel 0 (J2), wheel B is channel 1 (J3).
Both encoders are configured for **4000 counts/revolution**, the AS5047P
default with x4 decoding and direct 1:1 wheel coupling
([datasheet, page 19](https://www.mouser.com/datasheet/2/588/AS5047P_DS000324_3_00-1843440.pdf)).
Change this value if the encoder settings or gearing differ; confirm it in the
one-revolution test. Keep IMU `invert="false"`: the Pico's gravity projection
already establishes positive counterclockwise yaw.

## 4. Build and run profile A

On the Pi, from `pi/navigatr`:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_WITH_LIBCAMERA=ON
cmake --build build -j4
./build/navigatr --config_file=config/override/diagnostics/parallel_wheels_bno08x.xml
```

Use `-j2` if the Pi runs out of memory. Profile A does not need libcamera, but
building with it now avoids a rebuild for profile B. Run the threaded default,
not `--inline`, when the Brain is connected.

Open the viewer from the development computer through SSH:

```sh
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
```

Then browse to `http://127.0.0.1:8765/`. The field and the nominal landmark
positions (labelled "nominal") show immediately. With no Brain placement yet,
the robot is drawn grey and labelled "not placed", and the status bar reads
"not placed: odometry frame": that pose is odometry from where localization
started, drawn from the field origin, not a field position. The camera panel
reads "no camera configured".

## 5. Check the sensors, in this order

1. **Hold stationary and level for alignment and bias.** Wait until the
   localization table shows `tracking_motion` ready, normally about 6 s.
   Pico alignment requires at least 2 s and 200 acceleration samples with
   quiet, fresh gyro XYZ. Its checks allow gravity magnitude within 0.5 m/s^2
   of 9.80665, acceleration-vector variation up to 0.25 m/s^2, and gyro magnitude
   up to 0.10 rad/s; motion restarts collection. The Pi then collects 200 yaw
   samples, about 4 s at 50 Hz, restarting its bias collection if the wheels
   move. Until then no usable pose is produced. If it never becomes ready,
   check `robot_imu` freshness, IMU wiring, RST/PS0 wires and firmware.
2. **Encoder counts and wheel signs.** Lift the robot so the IMU stays still.
   Spin only wheel A forward by hand: x must increase (by half that wheel's
   travel, since the two wheels are averaged). If x decreases, flip
   `CONFIRM_WHEEL_A_DIRECTION`. Repeat for wheel B. If nothing moves, check the
   encoder's sensor state in the sources table and the J2/J3 wiring.
3. **CPR.** Still lifted, turn one wheel exactly one marked revolution: x
   should change by pi times the radius. Check the radius, gearing and encoder
   settings if the result differs substantially from this prediction.
4. **IMU sign.** On the floor, rotate the robot counterclockwise seen from
   above by about 90 degrees: heading must increase by about 90. If it
   decreases, check that the aligned BNO08X firmware is flashed, `invert` is
   `false`, and startup alignment happened with the robot level and still.
5. **Rotation and wheel offsets.** Turn in place several full turns about the
   origin. The reported position should stay nearly still. If it swings around a
   circle, the lateral offsets or a wheel sign are wrong.
6. **Translation and radius.** Push the robot straight 2 m along a tape: x
   should read 2.000 m with heading unchanged. Scale the radius by true/reported
   distance and repeat.
7. **Known limitation.** Push the robot sideways: the pose does not move. Both
   wheels measure forward only, so the localization assumes zero sideways
   motion. Sideways slip or pushing is not measured and is not corrected; the
   camera does not fix it either, because landmark estimates are never fed back
   into localization.

Restart navigatr after editing the XML. Each start calibrates the gyro bias
again; restarting the Pico or BNO08X also repeats startup gravity alignment.
A Pico reboot, or an IMU restart that the Pico firmware reports (its IMU
epoch), invalidates the Pi's bias and collects it again; older firmware shows a
hub reset only as a gap, which keeps the old bias. After remounting the IMU,
restart it and localization, then repeat the level, stationary startup. The
Brain's placement supplies field heading because gravity only defines up.

## 6. Brain placement

The profiles have no `InitialPlacement`. The Brain sends the robot's starting
FIELD pose (x, y, heading) over the brain link when its program initializes,
and waits until the Pi reports that localization applied it. From then on the
viewer draws the robot orange at its field pose. A new Brain session (a Brain
reboot) never moves the robot and never resumes an old movement; only a new
placement relocates it.

The field axes come from [field.xml](../config/override/field.xml) and never
change with the starting side. Starting on the other side means sending a
different x, y and heading, not mirroring the axes. The Brain programs'
starting pose is `kStartPose` in
[brain/robot/gatr2_robot.h](../../../brain/robot/gatr2_robot.h) (meters,
degrees counterclockwise from field +x). It is a placeholder until you measure
where the robot sits. See [Brain setup](../../../docs/brain_setup.md) for the
Brain side.

In profile A world estimation is off: the field documents the Brain reads keep
every landmark at its nominal pose, so landmark-relative moves use the nominal
map. A nominal landmark drawn in the viewer is never an observation.

## 7. Add the camera: profile B

Fill in the camera description the same way:

```sh
cp -n config/shared/robots/gatr2_front_camera.xml.in config/shared/robots/gatr2_front_camera.xml
```

It needs the capture mode, the lens calibration and the camera mount relative
to the same robot origin. [Camera setup](pi_camera_setup.md) and
[setup](setup.md#bring-up-and-calibrate-the-camera) describe capture checks and
calibration. Then run:

```sh
./build/navigatr --config_file=config/override/diagnostics/parallel_wheels_bno08x_camera.xml
```

The localization is byte for byte the same as profile A. The camera panel shows
the preview with tag overlays bound to the exact frame they were decoded from.
The field view adds solid "est." landmarks for observed estimates next to the
nominal ghosts. Each detection uses the robot pose from the pose history at the
frame's exposure time and the configured camera mount. Missing tilt is assumed
level; the Pico sends no roll or pitch. Detection runs in its own worker, so a
slow or failed camera never stalls localization; a camera fault shows in the
sources table.

The Brain reads every landmark from the field documents that the Publishing
section of
[the camera pipeline](../config/shared/pipelines/parallel_wheels_bno08x_camera.xml)
serves. A landmark stays nominal until the camera observes it; see
[landmarks](landmarks.md#field-documents).
Nominal map poses are never accepted as observed landmarks by default.

A camera configured with world estimation off still shows its preview, marked
"preview only, detection not run".

## Troubleshooting

- **No encoder or gyro data.** Check `pico_uart` in the sources table and
  reflash the firmware (section 2).
- **`tracking_motion` never ready.** Startup alignment or bias calibration is
  incomplete, or an IMU report is stale. Keep the robot level and stationary
  for about 6 s and check `robot_imu`.
- **Pose jumps or drifts during turns.** Check the lateral offsets and wheel
  signs with the turn-in-place test.
- **Heading drifts while still.** The robot moved during calibration; restart
  while holding it still.
