# IMU

[All PCBs](../README.md)

A dedicated inertial sensor board for estimating the robot's attitude, with
heading as its primary role. Gyroscope measurements describe how the robot is
rotating and support heading estimation alongside the tracking wheels.

The broader goal is to account for roll and pitch as well, including the robot
tilting or rocking during movement. That orientation matters when interpreting
camera observations. The PCB supplies sensor measurements; firmware and the
localization software turn those measurements into attitude estimates.

The separate board connects to the acquisition hardware over SPI, allowing its
mounting position to be chosen independently of the Pi HAT. Sensor selection,
connector details, and implementation status are documented per revision.

## Iterations

| Revision | Changes and status |
|---|---|
| [IMU v1](IMU_v1/README.md) | Initial ASM330-family design. Assembly failed because the sensor footprint was mirrored. |
| [IMU v2](IMU_v2/README.md) | Footprint correction in progress, with routing and mounting-hole improvements under consideration. |
