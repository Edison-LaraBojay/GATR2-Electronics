# Calibration inventory

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
| `override/diagnostics/two_wheel_bno08x.xml.in`, `three_wheel_bno08x.xml.in` | `gatr2_as5047_bno08x.xml` | none | `*_no_correction.xml` |
| `override/diagnostics/two_wheel_bno08x_camera.xml.in`, `three_wheel_bno08x_camera.xml.in` | `gatr2_as5047_bno08x.xml` + `gatr2_front_camera.xml` | `override/field.xml` | `*_camera_diagnostic.xml` |
| `override/diagnostics/live_camera_inspection.xml` | `gatr2_front_camera_uncalibrated.xml` | `override/field.xml` | `camera_inspection_only.xml` |
| `demo/synthetic_field_demo*.xml` | `synthetic_rig*.xml` (nothing measured) | `override/field.xml` | `synthetic_field_demo.xml` |

## Inventory

| Item | Where it goes | How to obtain | Status | Gates |
|---|---|---|---|---|
| Tracking wheel loaded radius, one per wheel | `shared/robots/gatr2_as5047_bno08x.xml.in`, `Resource[wheel_geometry]/Wheel[left_wheel,right_wheel,rear_wheel]` `radius_m` (`@MEASURE_*_WHEEL_LOADED_RADIUS_M@`) | Push the robot a taped distance of several meters on field tiles, `distance / (counts / CPR) / 2 pi`; repeat both directions, average | placeholder | every wheel profile (all four diagnostics templates); wrong radius scales odometry |
| Tracking wheel position, one per wheel | same element, `position_x_m`, `position_y_m` (`@MEASURE_*_WHEEL_X_M@`, `_Y_M@`) | Measure from the surveyed robot origin to the wheel contact point, +x forward, +y left | placeholder | heading from wheel differences (three-wheel), lateral compensation of the rear wheel |
| Tracking wheel rolling angle and direction | same element, `measurement_angle_deg`, `direction` (`@MEASURE_*_WHEEL_ANGLE_DEG@`, `@CONFIRM_*_WHEEL_DIRECTION@`) | Angle of the rolling direction in the body frame (0 forward, 90 left); push forward or left and confirm the count sign is positive, else `direction="negative"` | placeholder | motion sign; a wrong direction inverts the odometry axis |
| Wheel `calibration_status` | same elements; optional reader annotation | Record measurement progress if useful; for example `verified` once a square drive returns to start within tolerance | reader note | nothing; runtime does not read it |
| Encoder counts per revolution, one per channel | same file, `Sensor[tracking_encoder_a,b,c]/Calibration` `counts_per_revolution` (`@VERIFY_ENCODER_*_CPR@`) | AS5047 configuration as decoded by the Pico (see `pico/src`), confirmed by ten hand turns of the wheel | placeholder | distance scale, with radius |
| Encoder invert, one per channel | same, `invert` (`@CONFIRM_ENCODER_*_INVERT@`) | Turn the wheel forward, confirm counts increase; `true` if they decrease | placeholder | motion sign |
| IMU yaw sign | same file, `Sensor[robot_imu]/Calibration` `invert` (`@CONFIRM_IMU_CCW_POSITIVE_INVERT@`) | Rotate the robot counterclockwise from above; `gyro_z` must read positive, else `true` | placeholder | heading constraint sign; gyro bias calibration is automatic (`bias_samples`) |
| Camera device index | `shared/robots/gatr2_front_camera.xml.in`, `Resource[front_camera_device]/Device` `index` (`@SELECT_CAMERA_INDEX@`) | `rpicam-hello --list-cameras` on the Pi | placeholder (0 selected in `gatr2_front_camera_uncalibrated.xml`) | the camera opens at all |
| Capture mode and rate | same, `Capture` `width_px`, `height_px`, `frame_rate_hz` (`@SELECT_CAPTURE_*@`); `pixel_format` is fixed `Y8` | Pick a listed sensor mode; the calibration must be done at this same mode, the resource refuses a mismatch | placeholder (1280 x 960 at 30 Hz selected, not calibrated, in the uncalibrated fragment) | frame timing budget; the live inspection profile runs on the selection alone |
| Intrinsics | same, `Calibration/Intrinsics` `calibrated_width_px`, `calibrated_height_px`, `fx_px`, `fy_px`, `cx_px`, `cy_px`, `k1`, `k2`, `p1`, `p2`, `k3` (`@CALIBRATED_*@`), model fixed `brown_conrady` | Chessboard or ChArUco calibration at the capture mode from frames captured on the Pi (OpenCV `calibrateCamera` on a laptop is fine); 20+ views covering the corners | placeholder (absent by design in the uncalibrated fragment) | any metric tag pose: without it every decode is 2D, association rejects it, no landmark evidence |
| Calibration RMS and run id | same, `rms_reprojection_px` (`@CALIBRATION_RMS_PX@`), `Calibration` `calibration_id` (`@CALIBRATION_RUN_ID@`) | Reported by the calibration; name the run by date | placeholder | numeric calibration metadata; an optional `calibration_status` label has no runtime effect |
| Camera mount pose | same file, `Resource[robot_geometry]/Frame[front_camera_engineering]/PoseOfChildInParent` `x_m`, `y_m`, `z_m`, `roll_deg`, `pitch_deg`, `yaw_deg` (`@MEASURE_CAMERA_*@`), `calibration_status` | Measure from the robot origin to the optical center (+x forward, +y left, +z up; positive pitch looks down, positive yaw looks left); check by observing a tag at a taped field pose and comparing the implied landmark pose in the viewer | placeholder (the uncalibrated fragment declares no frame) | `tag_mount_association` needs the camera frame in the robot frame map; the camera diagnostic profiles |
| Detector corner size | same file, `Resource[tag_detector]/Family` `detection_size_m` (`@MEASURE_DETECTOR_CORNER_EDGE_SIZE_M@`); also every `TagMount` in `override/field.xml` | Measure the printed pattern side on a physical goal; the corner square is 5/9 of it for tagCircle21h7 | provisional (0.01761272 inferred from the CAD, see [field assets](field_assets.md)) | range scale of every metric tag pose; change the field file and the detector together |
| Tag family | same `Family` `name`; every `TagMount` `family` | Run `live_camera_inspection.xml` at a goal and see whether ids 0-4 decode as tagCircle21h7 | provisional (inferred; the manual never names it) | any decode at all |
| Initial robot placement | `Localization/InitialPlacement` in a pipeline fragment, or the brain init command over `brain_uart` (`CommandFrame` init pose) | Tape the starting pose on the field, enter it in field coordinates (x, y, heading counterclockwise from +x) | per run; the diagnostic pipelines carry none, the synthetic demo carries one | field poses and association priors: without a placement localization stays odometry-only and no landmark evidence is accepted |
| Field map geometry | `override/field.xml`, nine `NominalPose` and 36 `TagMount` elements | Nominal CAD positions are sufficient for association; optional physical checks can improve nominal accuracy. Competition displacement is estimated rather than configured in advance | provisional reader annotation | nominal association priors; the association translation gate (0.5-0.75 m) tolerates placement differences |
| Attitude source | would be a `Sensor type="attitude_channel"` bound to a resource output, an `Observation type="attitude_reference"` and an `Estimator/Attitude` reference (see `synthetic_rig.xml` and `synthetic_field_demo.xml`) | None exists in firmware: the Pico `SensorSample` carries `enc[3]`, `gyro_z` and `accel[2]` only (`common/frames.h`); the ASM330LHHG1 is six-axis but `pico/src/imu.cpp` reads gyro Z alone and there is no attitude report. Live tilt needs a new Pico report (out of scope, see [attitude follow-up](attitude_firmware_followup.md)). The runtime path is exercised by the synthetic rig | absent | measured tilt in association (`Attitude policy="assume_level"` otherwise) and the viewer's attitude badge; nothing else |
| Camera exposure timestamps | no configured value; `CameraFrameData.exposureAt`, `exposure_uncertainty_ms`, `exposure_time_reliable` from the libcamera backend | On the Pi, compare the reported exposure time with the host monotonic clock and a known event (a flashed LED), confirm the uncertainty bound and the latency; see [Pi camera setup](pi_camera_setup.md) | untested on hardware | pose-at-exposure lookups (`History max_interpolation_gap_ms`) and every association decision |
| Pi link parameters | `gatr2_as5047_bno08x.xml.in`, `Resource[pico_uart]`, `Resource[brain_uart]` | Already known: `/dev/ttyAMA0` 115200, `/dev/ttyAMA5` 115200 with RS-485 enable GPIO 6 (`docs/hardware.md`) | verified | serial transport |

Not in the table because nothing in the runtime consumes them yet: the
loader, toggle and tape positions (display only, from the manual), and the
robot body size in `Inspection/RobotBody` (cosmetic).
