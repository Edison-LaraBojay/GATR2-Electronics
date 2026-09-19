# Attitude: what the firmware would have to send

The runtime carries body attitude end to end: an `attitude_channel` sensor
takes an `AttitudeSample` (a unit quaternion of the body in a named reference,
with quality and a source epoch), the `attitude_reference` observation function
forwards fresh samples, the `planar_motion_integrator` keeps the measured tilt
beside its planar heading with its own age, the pose history interpolates it by
slerp, and the tag association uses the tilt at exposure time when it exists.
All of that is exercised by the synthetic rig (`config/demo/`), which is the
only attitude source in the repository today.

## What the live path has

`common/frames.h` defines these optional Pico sensor-frame fields:

| Field | Meaning | Used for |
|---|---|---|
| `gyro_z` | mdeg/s, raw, bias not removed | planar heading increment (`imu_heading_increment`, `HeadingConstraint`) |
| `accel[2]` | mg, two axes | supported by the decoder, not emitted by current firmware or exposed as a Pi resource output |

The Pico reads only the gyro Z axis of the ASM330LHHG1 (`pico/src/imu.cpp`;
the accelerometer is switched off). A yaw-rate-only path cannot produce roll or
pitch, and inferring tilt from two acceleration axes without a model of the
robot's own acceleration would be a guess dressed as a measurement. The runtime
therefore reports attitude unavailable on live profiles, draws the robot level
with the `assumed` label, and the association's `assume_level` policy marks the
evidence.

## What is missing, precisely

To get live tilt through the existing contracts, the firmware and the shared
wire protocol need:

1. A new telemetry report (new frame type or new fields, versioned) carrying
   either the six raw axes at the sample rate (three gyro, three accel, with
   the same device timestamp and sequence as the encoder frame) or a fused
   orientation (quaternion w, x, y, z in fixed point) plus a quality word and a
   filter-epoch counter that bumps whenever the on-device filter restarts.
2. The ASM330LHHG1 driver enabling the accelerometer and the remaining gyro
   axes with a declared full scale and output data rate.
3. On the Pi, either a `pico_attitude_channel` sensor that turns the fused
   report into `AttitudeSample`s (the natural fit for the existing
   `attitude_channel` contract, which already applies the mounting
   calibration), or a small complementary or Madgwick filter behind a new
   observation function if only raw axes are sent. Either way the sensor
   mounting orientation on the robot must be calibrated
   (`<Mounting calibration_status=... roll_deg pitch_deg yaw_deg/>`).
4. Bench verification: static tilt table checks against a level and a known
   incline, and dynamic checks that the tilt stays consistent with the wheel
   odometry heading during turns.

Until then the planar viewer and localization are complete without it, and no
live run claims measured attitude.
