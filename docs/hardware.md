# Hardware parts and rationale

The [PCB overview](../pcb/README.md) describes each board family, iteration, and
reported test status. Open the relevant revision's schematic for connector pins,
component values, and assembly choices; those can differ between revisions.
This page connects the hardware to the checked-in acquisition and Pi runtime.

## IMU

The [IMU board](../pcb/IMU/README.md) uses the ASM330LHHG1 for inertial sensing,
primarily heading. The Pico's [driver](../pico/src/imu.cpp) communicates over SPI1
and currently reads only gyro Z. It configures:

| Firmware setting | Value |
|---|---|
| Gyro output data rate | 208 Hz |
| Gyro full scale | 2000 degrees/second |
| Conversion to wire units | 70 millidegrees/second per LSB |
| Block-data update and address increment | Enabled |
| Accelerometer | Disabled |

The Pi's `pico_imu_channel` converts yaw rate and accumulated rotation into radians,
then localization observation models estimate bias and use heading increments.
The protocol defines optional accel XY fields, but the current firmware does not
populate them. No quaternion/roll/pitch report exists. See the
[attitude follow-up](../pi/navigatr/docs/attitude_firmware_followup.md) for the
additional acquisition and protocol work needed for measured tilt.

The present estimator does not periodically correct robot heading from field
landmarks: landmark position estimation uses robot localization as an input.
`kStatusGyroHealthy` reports configured sensor freshness, not an implemented
comparison between gyro and wheel-derived heading. Sensor specifications alone
do not establish assembled-robot drift or alignment accuracy.

## Tracking wheels

The [magnetic encoder board](../pcb/MagneticEncoder/README.md) supports compact
custom tracking-wheel assemblies using contactless magnetic rotation sensing.
The Pico counts A/B quadrature for three channels. Its
[pin configuration](../pico/src/config.h) assigns channels to GP0/1, GP2/3, and
GP4/5; pin and connector wiring must match the chosen HAT revision.

Counts per revolution and electrical sign are sensor calibration. Effective
wheel radius, position, and rolling direction belong to `wheel_geometry`, used
by `tracking_wheel_motion` to calculate body movement. Verify the actual encoder
configuration and counted edges rather than assuming every assembly has the same
counts per revolution. Unpowered tracking wheels measure ground movement but
still depend on contact, mounting rigidity, and calibration.

## Pi HAT

The [Pi HAT](../pcb/PiHat/README.md) mounts through the Pi's 40-pin header to reduce
loose wiring and provide a stable connection with dedicated sensor ports. The
family README compares iterations; each revision records its power arrangement,
regulator, peripheral connectors, and validation status.

The transceiver fitted to v2 was ST3485ECDR, although its saved schematic retains
the earlier THVD1410 selection. V3 records ST3485ECDR in its schematic. Use the
revision documentation when ordering or assembling rather than assuming all
boards share a BOM.

## Pico-to-Pi telemetry

The Pico sends sensor frames at 50 Hz on UART0 (Serial1), GP16 TX / GP17 RX,
115200 baud. The Pi profile selects the Linux device path. The current template
uses `/dev/ttyAMA0`; confirm the enabled UART and device mapping on the deployed
Pi. The [wire specification](interfaces.md) defines masks, integer units,
timestamps, and packet layouts.

## RS-485 to the V5 Brain

The HAT routes Pi UART5 through a half-duplex transceiver:

| Signal | Pi GPIO | 40-pin header pin |
|---|---|---|
| UART5 TX to transceiver DI | 12 | 32 |
| UART5 RX from transceiver RO | 13 | 33 |
| Transceiver DE and /RE enable | 6 | 31 |

The enable line is pulled down on the board. The checked-in Linux serial resource
sets its configured `DriverEnable` GPIO high while open and low on destruction.
There is no transmit/receive turnaround implementation; with DE and /RE tied
together, that keeps this physical link in transmit mode. The runtime has a
command-frame parser, but receive traffic on this shared link still needs the
direction-control implementation and hardware validation.

The [bench example](../bench/rs485_link/README.md) exercises Pi-to-Brain byte
transfer separately from the production codec. It does not validate command
return traffic. [Pi setup](pi_setup.md) covers access and UART provisioning.

## Camera and Pi runtime

The runtime targets a Raspberry Pi 4 and uses C++, libcamera, and the bundled
AprilRobotics detector. There is no Python/OpenCV localization process or EKF in
the current executable. The implemented planar estimator integrates configured
wheel-motion and heading observations.

Camera capture produces grayscale images for tag decoding. Intrinsics, detected
corner size, and rigid camera mounting are required for metric field estimates.
An uncalibrated inspection profile supports viewing images and decoded IDs first.
The browser renders the 3D field on the viewing computer, reached through an SSH
port forward to the Pi's inspection service.

See [camera setup](../pi/navigatr/docs/pi_camera_setup.md) for supported capture
configuration and the remaining hardware checks. Pi throughput, capture timing,
memory use, and physical alignment accuracy have not been established by host
simulation tests. Measure them on the selected camera, mode, and robot.
