# Hardware parts and rationale

The [PCB overview](../pcb/README.md) describes each board family, iteration, and
reported test status. Open the relevant revision's schematic for connector pins,
component values, and assembly choices; those can differ between revisions.
This page connects the hardware to the checked-in acquisition and Pi runtime.

## IMU

The robot's IMU is a GY-BNO08X breakout on SPI1 (mode 3, at most 3 MHz). The
Pico's [BNO08X driver](../pico/aggreGATR/src/imu_bno08x.cpp) runs CEVA's sh2 library over a
non-blocking SPI layer. It enables acceleration report `0x01` at 100 Hz and
uncalibrated gyro report `0x07` at 200 Hz. At startup or IMU reset, at least two
seconds of stationary, level robot data establish sensor-space up. The driver
projects gyro XYZ onto that fixed axis, allowing sideways and upside-down IMU
mounts. Each 50 Hz frame carries the mean projected yaw rate in millidegrees
per second, with bias left for the Pi. The gyro bit stays clear during alignment
or stale reports. Recovery retries do not stop encoders or UART frames.

Pi HAT v2 wiring, by the module's printed labels:

| Module | Pico | Route |
|---|---|---|
| VCC / GND | 3.3 V / GND | J7 pin 7 / J7 pin 1 or 8 |
| SCL (SCK) | GP10, pin 14 | J7 pin 5 |
| SDA (MISO) | GP8, pin 11 | J7 pin 3 |
| ADO (MOSI) | GP11, pin 15 | J7 pin 4 |
| CS | GP9, pin 12 | J7 pin 6 |
| INT | GP22, pin 29 | J7 pin 2 |
| RST | GP12, pin 16 | added wire |
| PS0/WAKE | GP13, pin 17 | added wire |
| PS1 | 3.3 V | added wire |

PS0 and PS1 must be high through reset to select SPI; the driver resets the hub
with PS0 held high, then uses PS0 as WAKE. The interrupt line is required: the
v3 HAT's six-pin IMU connector drops it. Pins live in
[board.h](../pico/aggreGATR/src/board.h).

The earlier [IMU board](../pcb/IMU/README.md) with the ASM330LHHG1 stays
selectable with the `hat2_asm330` build ([driver](../pico/aggreGATR/src/imu_asm330.cpp)):
208 Hz, 2000 degrees/second full scale, 70 millidegrees/second per LSB, newest
sample per frame, accelerometer disabled.

Keep the robot level and still at a full startup: about two seconds for BNO08X
alignment, then the Pi's gyro bias window (2 s by default with a Brain profile,
200 samples, about 4 s, in the XML parallel-wheel profiles). The
[bring-up guide](../pi/naviGATR/docs/parallel_wheel_bringup.md) and
[Brain robot profiles](../pi/naviGATR/docs/brain_profile.md#calibration-and-stationary-handling)
describe the stationarity checks. The fixed axis does not compensate later
rocking, and the Brain's placement still supplies field heading. The IMU chip
(BNO08X or ASM330) is chosen by the Pico firmware build; a Brain profile only
selects Pico IMU port 0.

The Pi's `pico_imu_channel` converts yaw rate and accumulated rotation into radians,
then localization observation models estimate bias and use heading increments.
The protocol defines optional accel XY fields, but the current firmware does not
populate them: BNO08X acceleration is used internally for startup alignment.
No quaternion/roll/pitch report exists. See the
[attitude follow-up](../pi/naviGATR/docs/attitude_firmware_followup.md) for the
additional acquisition and protocol work needed for measured tilt.

The present estimator does not periodically correct robot heading from field
landmarks: landmark position estimation uses robot localization as an input.
The Brain link's gyro-fresh health bit reports sensor freshness, not an
implemented comparison between gyro and wheel-derived heading. Sensor specifications alone
do not establish assembled-robot drift or alignment accuracy.

## Tracking wheels

The [magnetic encoder board](../pcb/MagneticEncoder/README.md) supports compact
custom tracking-wheel assemblies using contactless magnetic rotation sensing.
The Pico counts A/B quadrature for three channels. Its
[pin map](../pico/aggreGATR/src/board.h) assigns channels to GP0/1, GP2/3, and GP4/5,
which are J2, J3 and J4 on the v2 HAT (logical encoder ports 0, 1 and 2); pin
and connector wiring must match the chosen HAT revision. The current bench has
a forward-measuring wheel on port 1 (J3) and a sideways wheel on port 0 (J2),
described on the Brain ([gatr2_robot.h](../brain/robot/gatr2_robot.h)); the
earlier two parallel wheels have their own
[XML bring-up](../pi/naviGATR/docs/parallel_wheel_bringup.md).

Counts per revolution, gearing and encoder polarity are sensor calibration.
Effective wheel radius, position, rolling direction and the measured travel
scale belong to the wheel model that turns wheel travel into body movement:
the Brain profile, or `wheel_geometry` in an XML profile. Verify the actual encoder
configuration and counted edges rather than assuming every assembly has the same
counts per revolution. The parallel-wheel AS5047P template uses its default
4000 counts/revolution with x4 decoding and direct 1:1 wheel coupling; see the
[bring-up guide](../pi/naviGATR/docs/parallel_wheel_bringup.md#3-fill-in-the-robot-description).
Unpowered tracking wheels measure ground movement but
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
115200 baud. The Pi profile selects the Linux device path. The current configs
use `/dev/ttyAMA0`; confirm the enabled UART and device mapping on the deployed
Pi. The [wire specification](interfaces.md) defines masks, integer units,
timestamps, and packet layouts.

The link is two way: the Pico also sends status frames (boot identity, epochs,
IMU state) and reads Pi commands (reinitialize the IMU, restart acquisition) on
GP17. See [Pico link](../pi/naviGATR/docs/pico_link.md). The Pi to Pico
direction on the HAT wiring has not been checked on hardware.

## USB to the V5 Brain

The current bench bypasses the HAT's RS-485 circuit: a data cable from a Pi
USB-A port to the Brain's micro-USB port, with the Brain on its V5 battery. The
Pi's `pros_usb_link` finds the Brain's USB user interface and wraps each frame
in an `NG1:` hexadecimal line; see
[USB bench](../pi/naviGATR/docs/usb_localization_bench.md). Choosing USB or
RS-485 needs the matching Pi config (`brain_profile_usb.xml` or
`brain_profile_rs485.xml`) and never changes localization geometry.

## RS-485 to the V5 Brain

The HAT routes Pi UART5 through a half-duplex transceiver:

| Signal | Pi GPIO | 40-pin header pin |
|---|---|---|
| UART5 TX to transceiver DI | 12 | 32 |
| UART5 RX from transceiver RO | 13 | 33 |
| Transceiver DE and /RE enable | 6 | 31 |

The enable line is pulled down on the board. With `<DriverEnable gpio="6"/>` the
Linux serial resource runs the link half duplex: it drives DE low at open so the
Pi listens, raises it only while it sends a reply, and drives it low again once
the UART reports its transmitter empty and a two-character guard has passed.
Errors and timeouts also release DE. The Brain is the only initiator and the Pi only
answers requests; the [interface spec](interfaces.md) defines bus ownership and
timing, and [navigatr resources](navigatr_resources.md#linux_serial_link) the
transmit sequence. The V5 smart port is assumed to switch its own RS-485
direction.

Hardware checks, not validated on the robot yet: the sysfs GPIO number (newer
kernels can offset it, for example 512 + 6), `TIOCSERGETLSR` and turnaround
timing on the Pi UART, DE behavior, and the V5 port's direction handling. If
the runtime is killed (SIGKILL, crash) while transmitting, DE stays high and
blocks the Brain until the runtime starts again and drives it low.

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

See [camera setup](../pi/naviGATR/docs/pi_camera_setup.md) for supported capture
configuration and the remaining hardware checks. Pi throughput, capture timing,
memory use, and physical alignment accuracy have not been established by host
simulation tests. Measure them on the selected camera, mode, and robot.
