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
| `gyro_z` | mdeg/s, bias not removed; BNO08X projects gyro XYZ onto startup up, ASM330 sends physical gyro Z | planar heading increment (`imu_heading_increment`, `HeadingConstraint`) |
| `accel[2]` | mg, two axes | supported by the decoder, not emitted by current firmware or exposed as a Pi resource output |

The BNO08X driver (`pico/src/imu_bno08x.cpp`) reads acceleration XYZ at 100 Hz
and uncalibrated gyro XYZ at 200 Hz. Its stationary, level startup alignment
learns a fixed up axis, allowing arbitrary fixed mounting for yaw projection.
That axis stays fixed while driving; it supplies neither live tilt nor dynamic
rocking compensation. The ASM330LHHG1 driver (`pico/src/imu_asm330.cpp`) reads
physical gyro Z with its accelerometer disabled.

Both paths send yaw only. Live profiles report attitude unavailable, draw the
robot level with the `assumed` label, and mark association evidence with the
`assume_level` policy. A live tilt estimate must account for the robot's own
acceleration rather than treating every acceleration reading as gravity.

## What is missing, precisely

To get live tilt through the existing contracts, the firmware and the shared
wire protocol need:

1. A new telemetry report (new frame type or new fields, versioned) carrying
   either the six raw axes at the sample rate (three gyro, three accel, with
   the same device timestamp and sequence as the encoder frame) or a fused
   orientation (quaternion w, x, y, z in fixed point) plus a quality word and a
   filter-epoch counter that bumps whenever the on-device filter restarts.
2. Acquisition for the chosen IMU: BNO08X already reads all six raw axes, so
   expose them with their timestamps or enable and decode a suitable fused
   orientation report. ASM330LHHG1 needs its accelerometer and remaining gyro
   axes enabled with a declared full scale and output data rate.
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

The planar viewer and localization work without a live attitude source. Keep
the robot level and stationary during BNO08X startup alignment; the Brain's
placement supplies its field heading. No live run claims measured attitude.
