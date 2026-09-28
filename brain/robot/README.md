# robot

`gatr2_robot.h`: this robot's localization description, shared by the Brain
programs (tracking wheels, encoders, IMU source, footprint, Pi link, start
pose). The Brain sends it to the Pi as the robot profile on connect.
Program-specific settings (drive motors, gains, tests, controls) stay in each
program's `include/robot_config.h`.

Calibration and tuning: [docs/brain_setup.md](../../docs/brain_setup.md).
