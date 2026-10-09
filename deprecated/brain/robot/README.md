# robot

`gatr2_robot.h`: this robot's localization description, shared by the Brain
programs (tracking wheels, encoders, IMU source, footprint, Pi link, start
pose), plus the telemetry switch and the VEX IMU mounting for the viewer's
tilt. The Brain sends the localization part to the Pi as the robot profile on
connect.
Program-specific settings (drive motors, gains, tests, controls) stay in each
program's `include/robot_config.h`.

Calibration and tuning: [docs/brain_setup.md](../../docs/brain_setup.md).
