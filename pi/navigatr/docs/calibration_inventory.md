# Calibration inventory

For commands, calibration software, and complete three-wheel camera profiles,
start with [Set up and run Navigatr](setup.md); for the two parallel wheels +
BNO08X profiles, with the [parallel-wheel bring-up](parallel_wheel_bringup.md).

**Brain-profiled configs (the primary path).** With `brain_profile_usb.xml` or
`brain_profile_rs485.xml` the robot's measurements live on the Brain, in
[brain/robot/gatr2_robot.h](../../../brain/robot/gatr2_robot.h), and reach the
Pi as the robot profile; the XML rows below for wheels, encoders and the IMU
sign apply only to the XML-configured profiles. The measurements are the same
ones, split three ways and applied once each ([Brain robot profiles](brain_profile.md#corrections-each-applied-once)):

| Item | Where it goes | How to obtain | Status |
|---|---|---|---|
| Encoder: counts per revolution, polarity (`reversed`), gearing | `gatr2_robot.h`, each `TrackingWheel` | Counts the Pico reports for one encoder shaft turn; roll the wheel along its measuring direction and confirm the travel is positive; gearing is encoder turns per wheel turn | 4000 counts, not reversed, gear 1: defaults to confirm |
| Geometry: radius, mounting position `x`, `y`, measuring angle | same | Loaded radius from a long push (as below); position of the contact point from the robot origin; angle 0 forward, 90 left | radius 0.024 m PROVISIONAL, positions UNMEASURED |
| Empirical: `travel_scale` per wheel | same | The wheel calibration page of `brain/localization-test` (READ_WHEELS against a measured push, three agreeing trials); see [Brain setup](../../../docs/brain_setup.md#4-per-wheel-calibration) | 1.0 until calibrated; never hides a wrong radius or count |
| IMU source and port, VEX Smart Port, Pico IMU inversion | same (`kSetup`, `kVexImuPort`) | Which IMU is fitted; the Pico chip is fixed by its firmware build | VEX IMU on port 1 for the bench |
| Footprint | same, `kFootprint` | Measure the robot outline about the origin | PLACEHOLDER |
| Start pose | same, `kStartPose` | Tape the starting pose on the field | PLACEHOLDER |
| Stationary window and bias defaults | Pi: `BrainProfile/Calibration`; a profile's calibration window, still rate and still travel override them | Record the robot standing still and compare gyro noise and wheel jitter with `still_rate_dps` (1) and `still_travel_m` (0.001) | defaults, not validated against the real sensors |
| Sensor loss limit | Pi: `BrainProfile/Timing sensor_loss_ms` (250), `on_sensor_loss` | Watch `event: sensor lost` and `motion lost` lines during normal driving; too tight a limit unplaces on ordinary poll gaps. It is also the VEX bench model's longest step. The other limits (`Calibration max_gap_ms` 250, sensor `stale_after_ms` 250, the Pico link's 250 ms gyro accumulator) still end continuity when shorter, so raising it alone never lets a gap go unmeasured | default, not validated against real poll gaps and UART jitter |
| Three-wheel fusion noise | Pi: `BrainProfile/Fusion` | as for `weighted_planar_fusion` below | PLACEHOLDER |

Every measurement the runtime still needs, where it goes, how to get it,
and what depends on it. Status words in this inventory and XML
`calibration_status` attributes are notes for the reader only. The runtime
does not read the labels, require them, or change behavior based on them.
`provisional` describes an unchecked value and `verified` a checked value;
`UNCONFIGURED` can remind the reader that work remains. Actual `.xml.in`
templates and unresolved `@TOKEN@` parameter values are rejected. Required
numbers must be finite and usable, and camera calibration must match the
capture mode. A label cannot supply or validate missing measurements.

Which profile uses which file:

| Profile | Robot fragments | Field | Pipeline |
|---|---|---|---|
| `override/brain_profile_usb.xml`, `override/brain_profile_rs485.xml` | none: devices and wired ports only; the robot comes from the Brain profile | `override/field.xml` (served to the Brain) | inline |
| `override/diagnostics/three_wheel_imu.xml.in` | `gatr2_as5047_imu.xml` | none | `three_wheel_imu_no_correction.xml` |
| `override/diagnostics/parallel_wheels_bno08x.xml` | `gatr2_parallel_wheels_bno08x.xml`, completed from `.xml.in` | `override/field.xml` (display only) | `parallel_wheels_bno08x_no_camera.xml` + shared `parallel_wheels_bno08x_localization.xml` |
| `override/diagnostics/parallel_wheels_bno08x_camera.xml` | `gatr2_parallel_wheels_bno08x.xml` + `gatr2_front_camera.xml` | `override/field.xml` | `parallel_wheels_bno08x_camera.xml` + the same shared localization |
| `override/diagnostics/three_wheel_imu_camera.xml.in` | `gatr2_as5047_imu.xml` + `gatr2_front_camera.xml` | `override/field.xml` | local `three_wheel_imu_camera_pipeline.xml`, completed from `.xml.in` |
| `override/diagnostics/live_camera_inspection.xml` | `gatr2_front_camera_uncalibrated.xml` | `override/field.xml` | `camera_inspection_only.xml` |
| `demo/synthetic_field_demo*.xml` | `synthetic_rig*.xml` (nothing measured) | `override/field.xml` | `synthetic_field_demo.xml` |
| `demo/synthetic_fusion_demo.xml` | `synthetic_rig.xml` (nothing measured) | `override/field.xml` | `synthetic_fusion_demo.xml` |

## Inventory

The parallel-wheel robot template (`gatr2_parallel_wheels_bno08x.xml.in`) has
only `forward_wheel_a`/channel 0 and `forward_wheel_b`/channel 1, both at
measurement angle 0, with tokens `@MEASURE_WHEEL_A_*@`, `@MEASURE_WHEEL_B_*@`,
`@CONFIRM_WHEEL_A_DIRECTION@`, `@CONFIRM_WHEEL_B_DIRECTION@`,
and no other measurement tokens. Encoder CPR is 4000 for default AS5047P
settings and direct 1:1 wheel coupling; verify the physical count and change
it if settings or gearing differ. Encoder `invert` is fixed false so each
wheel has one sign knob, `direction`. IMU `invert` is also false: Pico startup
gravity alignment establishes positive counterclockwise yaw for any fixed
mounting. See the [bring-up guide](parallel_wheel_bringup.md) for the encoder
datasheet and startup checks. The rows below name the three-wheel template's
tokens; its CPR and sign placeholders remain configurable. Each
three-wheel camera main profile also has `Inspection/RobotBody` display
dimensions and body-center offsets relative to the robot origin; these do not
affect estimates.

| Item | Where it goes | How to obtain | Status | Gates |
|---|---|---|---|---|
| Tracking wheel loaded radius, one per wheel | `shared/robots/gatr2_as5047_imu.xml.in`, `Resource[wheel_geometry]/Wheel[left_wheel,right_wheel,rear_wheel]` `radius_m` (`@MEASURE_*_WHEEL_LOADED_RADIUS_M@`) | Push the robot a taped distance of several meters on field tiles, `distance / (counts / CPR) / 2 pi`; repeat both directions, average | placeholder | every wheel profile (all four diagnostics templates); wrong radius scales odometry |
| Tracking wheel position, one per wheel | same element, `position_x_m`, `position_y_m` (`@MEASURE_*_WHEEL_X_M@`, `_Y_M@`) | Measure from the surveyed robot origin to the wheel contact point, +x forward, +y left | placeholder | heading from wheel differences (three-wheel), lateral compensation of the rear wheel; for parallel wheels `position_y_m` is the lever arm that removes turning from forward travel (x unused) |
| Tracking wheel rolling angle and direction | same element, `measurement_angle_deg`, `direction` (`@MEASURE_*_WHEEL_ANGLE_DEG@`, `@CONFIRM_*_WHEEL_DIRECTION@`) | Angle of the rolling direction in the body frame (0 forward, 90 left); push forward or left and confirm the count sign is positive, else `direction="negative"` | placeholder | motion sign; a wrong direction inverts the odometry axis |
| Wheel `calibration_status` | same elements; optional reader annotation | Record measurement progress if useful; for example `verified` once a square drive returns to start within tolerance | reader note | nothing; runtime does not read it |
| Encoder counts per revolution, one per channel | same file, `Sensor[tracking_encoder_a,b,c]/Calibration` `counts_per_revolution` (`@VERIFY_ENCODER_*_CPR@`) | AS5047 configuration as decoded by the Pico (see `pico/src`), confirmed by ten hand turns of the wheel | placeholder | distance scale, with radius |
| Encoder invert, one per channel | same, `invert` (`@CONFIRM_ENCODER_*_INVERT@`) | Turn the wheel forward, confirm counts increase; `true` if they decrease | placeholder | motion sign |
| IMU yaw sign | same file, `Sensor[robot_imu]/Calibration` `invert` (`@CONFIRM_IMU_CCW_POSITIVE_INVERT@`) | Rotate the robot counterclockwise from above; `gyro_z` must read positive, else `true` | placeholder | heading constraint sign; gyro bias calibration is automatic (a qualified stationary window, `bias_samples` samples over `window_ms`) |
| Camera device index | `shared/robots/gatr2_front_camera.xml.in`, `Resource[front_camera_device]/Device` `index` (`@SELECT_CAMERA_INDEX@`) | `rpicam-hello --list-cameras` on the Pi | placeholder (0 selected in `gatr2_front_camera_uncalibrated.xml`) | the camera opens at all |
| Capture mode and rate | same, `Capture` `width_px`, `height_px`, `frame_rate_hz` (`@SELECT_CAPTURE_*@`); `pixel_format` is fixed `Y8` | Pick a listed sensor mode; the calibration must be done at this same mode, the resource refuses a mismatch | placeholder (1280 x 960 at 30 Hz selected, not calibrated, in the uncalibrated fragment) | frame timing budget; the live inspection profile runs on the selection alone |
| Intrinsics | same, `Calibration/Intrinsics` `calibrated_width_px`, `calibrated_height_px`, `fx_px`, `fy_px`, `cx_px`, `cy_px`, `k1`, `k2`, `p1`, `p2`, `k3` (`@CALIBRATED_*@`), model fixed `brown_conrady` | Chessboard or ChArUco calibration at the capture mode from frames captured on the Pi (OpenCV `calibrateCamera` on a laptop is fine); 20+ views covering the corners | placeholder (absent by design in the uncalibrated fragment) | any metric tag pose: without it every decode is 2D, association rejects it, no landmark evidence |
| Calibration RMS and run id | same, `rms_reprojection_px` (`@CALIBRATION_RMS_PX@`), `Calibration` `calibration_id` (`@CALIBRATION_RUN_ID@`) | Reported by the calibration; name the run by date | placeholder | numeric calibration metadata; an optional `calibration_status` label has no runtime effect |
| Camera mount pose | same file, `Resource[robot_geometry]/Frame[front_camera_engineering]/PoseOfChildInParent` `x_m`, `y_m`, `z_m`, `roll_deg`, `pitch_deg`, `yaw_deg` (`@MEASURE_CAMERA_*@`), `calibration_status` | Measure from the robot origin to the optical center (+x forward, +y left, +z up; positive pitch looks down, positive yaw looks left); check by observing a tag at a taped field pose and comparing the implied landmark pose in the viewer | placeholder (the uncalibrated fragment declares no frame) | `tag_mount_association` needs the camera frame in the robot frame map; the camera diagnostic profiles |
| Detector corner size | same file, `Resource[tag_detector]/Family` `detection_size_m` (`@MEASURE_DETECTOR_CORNER_EDGE_SIZE_M@`); also every `TagMount` in `override/field.xml` | Measure the printed pattern side on a physical goal; the corner square is 5/9 of it for tagCircle21h7 | provisional (0.01761272 inferred from the CAD, see [field assets](field_assets.md)) | range scale of every metric tag pose; change the field file and the detector together |
| Tag family | same `Family` `name`; every `TagMount` `family` | Run `live_camera_inspection.xml` at a goal and see whether ids 0-4 decode as tagCircle21h7 | provisional (inferred; the manual never names it) | any decode at all |
| Initial robot placement | `Localization/InitialPlacement` in the local `override/diagnostics/*_imu_camera_pipeline.xml.in` templates; the parallel-wheel and Brain-profiled configs have none and take the pose from the Brain (`kStartPose` in `brain/robot/gatr2_robot.h`) | Tape the starting pose on the field, enter the robot origin's field coordinates (x, y, heading counterclockwise from +x) | per run; camera pipeline templates have required placement tokens, synthetic demos supply their own placement | field poses and association priors: without placement localization stays odometry-only and no landmark evidence is accepted |
| Field map geometry | `override/field.xml`, nine `NominalPose` and 36 `TagMount` elements | Nominal CAD positions are sufficient for association; optional physical checks can improve nominal accuracy. Competition displacement is estimated rather than configured in advance | provisional reader annotation | nominal association priors; the association translation gate (0.5-0.75 m) tolerates placement differences |
| BNO08X startup up axis | Pico `GravityAlignment`, automatic at startup and IMU reset | Keep the robot level and stationary: at least 2 s and 200 acceleration samples; fresh gyro XYZ at most 0.10 rad/s, acceleration magnitude 9.80665 +/- 0.5 m/s^2, vector variation at most 0.25 m/s^2. On localization startup, Pi bias collection adds a stationary window (2 s by default) after yaw becomes available; an IMU restart the Pi can see (a Pico reboot, or an IMU epoch change from firmware that reports one) invalidates the Pi bias and repeats it. | automatic; hardware validation pending | projected yaw availability; permits sideways/upside-down fixed IMU mounting |
| Attitude source | would be a `Sensor type="attitude_channel"` bound to a resource output, an `Observation type="attitude_reference"` and an `Estimator/Attitude` reference (see `synthetic_rig.xml` and `synthetic_field_demo.xml`) | No live attitude report exists in firmware. BNO08X acceleration at 100 Hz and uncalibrated gyro XYZ at 200 Hz supply a fixed startup up axis and projected yaw only; ASM330 reads physical gyro Z. Pico `SensorSample` carries `enc[3]`, `gyro_z` and optional `accel[2]` (`common/frames.h`); the reserved accel fields remain absent. Live tilt needs additional acquisition and protocol work; see [attitude follow-up](attitude_firmware_followup.md). The runtime attitude path is exercised by the synthetic rig. | absent | measured tilt in association (`Attitude policy="assume_level"` otherwise) and the viewer's attitude badge; nothing else |
| Camera exposure timestamps | no configured value; `CameraFrameData.exposureAt`, `exposure_uncertainty_ms`, `exposure_time_reliable` from the libcamera backend | On the Pi, compare the reported exposure time with the host monotonic clock and a known event (a flashed LED), confirm the uncertainty bound and the latency; see [Pi camera setup](pi_camera_setup.md) | untested on hardware | pose-at-exposure lookups (`History max_interpolation_gap_ms`) and every association decision |
| Odometry and gyro noise | `Localization/Estimator[weighted_planar_fusion]/Motion/Noise` and `Heading/Noise` in a pipeline fragment (`synthetic_fusion_demo.xml` carries placeholders); units in [localization fusion](localization_fusion.md) | Wheel translation and rotation error per meter and per radian from repeated known-path drives against tape; gyro angle random walk and residual bias from a long stationary recording after `bias_samples` calibration | placeholder | the fusion weights and the reported `odom_covariance`; the pose itself still follows the wheels and gyro |
| Pi link parameters | `gatr2_as5047_imu.xml.in`, `gatr2_parallel_wheels_bno08x.xml.in`, `Resource[pico_uart]`, `Resource[brain_uart]` | Already known: `/dev/ttyAMA0` 115200, `/dev/ttyAMA5` 115200 (`docs/hardware.md`) | verified | serial transport |
| Brain link DriverEnable and reply timing | same files, `Resource[brain_uart]/DriverEnable` `gpio` (optional `post_guard_us`, `tx_margin_us`); the pipeline's `CommandCollection[brain_link]/Reply` `window_ms`, `turnaround_guard_us` (defaults 40 ms, 1000 us) | GPIO6 is the HAT's DE pin (`docs/hardware.md`); `gpio` is its sysfs number, so compare `/sys/class/gpio/gpiochip*/base` on the installed kernel (a base of 512 gives 518). On a scope, DE low while idle and high only during a reply; replies inside the Brain's response timeout. See [Connect the Brain](setup.md#connect-the-brain) | pin known; sysfs number, turnaround and timing not validated on hardware | every Brain reply; an unreachable GPIO is a startup warning and every reply fails |
| Field object wire ids and collision boxes | `override/field.xml`: `wire_id` on every `Landmark` and `Obstacle`, `CollisionBox` sizes, `Boundary`, `revision`; regenerate `brain/testing/include/field_references.h` after an edit | Collision boxes from the manual and CAD ([field assets](field_assets.md#planning-data)); measure the goal base and the loader and toggle footprints on a real field | provisional, derived not measured | Brain planning obstacles and named references |

Not in the table because nothing in the runtime consumes them yet: the
loader, toggle and tape positions (display only, from the manual), and the
robot body size in `Inspection/RobotBody` (cosmetic).
