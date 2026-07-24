# Hardware parts and rationale

Why each part was chosen and which of its specs the design actually depends on.

This is not a BOM. Per board BOMs are exported from the KiCad schematic into
`pcb/<board>/`, so the schematic stays the single source of truth for what gets
ordered. A hand maintained parts table drifts from the schematic and then the
wrong part gets ordered. What a schematic cannot record is why, which is what
this page is for.

Datasheets are linked, not committed. They are multi megabyte binaries that
cannot be diffed and would bloat the repository permanently.

## IMU

| Field | Value |
|-------|-------|
| Part | ASM330LHHG1 |
| Vendor | STMicroelectronics |
| Role | Yaw rate for heading |
| Interface | SPI |
| Datasheet | https://www.st.com/resource/en/datasheet/asm330lhhg1.pdf |

Automotive qualified 6-axis inertial module. Only the gyroscope Z axis feeds
fusion. The accelerometer is not used, see the accelerometer note below.

### One IMU, not three

An earlier idea was three IMUs averaged for redundancy and lower noise. Rejected.

Averaging N independent gyros reduces angle random walk by the square root of N,
so three parts would improve bias instability from about 0.04 deg/hour to about
0.023 deg/hour. Over a two minute match, 0.04 deg/hour accumulates roughly
0.0013 degrees of drift. That is already far below anything measurable on this
robot, and landmark corrections zero the accumulated error periodically anyway.
Three parts would have cost board area, money, and two extra sets of decoupling
capacitors to improve a term that was never the limiting factor.

The errors that actually matter are gyro scale factor calibration, mounting
rigidity, vibration, and temperature drift. None of those improve by adding
more IMUs to the same board.

Redundancy is not lost. Yaw rate derived from the differential between the
parallel tracking wheels cross checks the gyro, and disagreement is what
`kStatusGyroHealthy` reports.

### Settings the firmware depends on

| Setting | Value | Reason |
|---------|-------|--------|
| Full scale | 2000 dps | Headroom for collision spikes; 1000 dps would also work |
| Sensitivity | 70 mdps/LSB | Integer, see below |
| Output data rate | 208 Hz | Comfortably above the 50 Hz frame rate |
| BDU | enabled | Prevents reading a high byte and low byte from different samples |

Full scale is chosen partly so the sensitivity is a whole number of
millidegrees per second per LSB. The wire format carries yaw rate as an integer
in mdeg/s and forbids floats, so a scale like 70 or 35 converts with a single
multiply and no rounding. The 125, 250, and 500 dps settings have fractional
sensitivities (4.375, 8.75, 17.5) and would force either float math or a
lossy conversion.

### Accelerometer

Present on the part but unused. Position from double integrated acceleration
drifts as the square of time, and the tracking wheels already measure
displacement directly and far better. `kSensorAccelXY` is reserved on the wire
so the bit and width exist if a future use appears, such as slip or impact
detection, but nothing populates it.

## Tracking wheel encoders

| Field | Value |
|-------|-------|
| Part | AS5047P |
| Role | Wheel displacement |
| Interface | ABI quadrature |
| Resolution | 4000 counts per revolution |

Magnetic rotary encoders read in ABI quadrature mode, not SPI. Three channels
are wired so a second parallel wheel can be enabled for redundancy and fault
detection through the sensor mask without any wire format change.

The 4000 CPR figure is where the `counts` unit in the sensor frame comes from.
Converting counts to millimeters needs the wheel diameter, which is a
calibration constant measured on the bench rather than a datasheet number.

Tracking wheels are unpowered and spring loaded. No motor torque tries to spin
them past the ground, which is why they slip far less than drive motor encoders.

## Where the wire units come from

`docs/interfaces.md` fixes the units on the wire. Their origin is here.

| Wire unit | Origin |
|-----------|--------|
| counts | AS5047P 4000 CPR, raw quadrature |
| mdeg/s | Gyro full scale sensitivity, 70 mdps/LSB at 2000 dps |
| mm | Calibrated from counts using measured wheel diameter |
| mg | Accelerometer full scale, unused |

## RS-485 link to the brain

| Field | Value |
|-------|-------|
| Part | ST3485ECDR |
| Vendor | STMicroelectronics |
| Role | Pi UART to RS-485 differential, into a V5 smart port |
| Interface | Standard 8-pin 485 pinout, 3.3V, half duplex |
| Datasheet | https://www.st.com/resource/en/datasheet/st3485e.pdf |

The Pi transmits over UART5 (GPIO12/13). The transceiver converts that to the
A/B differential pair, which plugs into a V5 smart port. V5 smart port data
lines are RS-485, so no extra hardware is needed on the brain side and Chomp
reads it as generic serial.

| Transceiver pin | Connects to | Note |
|-----------------|-------------|------|
| DI (4) | Pi GPIO12 / TXD5 (RSTX) | driver input |
| RO (1) | Pi GPIO13 / RXD5 (RSRX) | receiver output, unused, link is one way |
| DE (3) and /RE (2) | Pi GPIO6 (EN) | tied together, 10k pulldown to GND |
| A (6), B (7) | V5 smart port | differential pair |
| VCC (8) | +3.3V | |
| GND (5) | GND | |

DE and /RE share one EN line, so the idle state (pulled low) is receive and
driver disabled. The Pi drives EN high to transmit. The link is one way, Pi to
brain, so EN stays high in operation. The pulldown means a dead or unbooted Pi
leaves the bus idle instead of jamming it.

Any 3.3V standard-pinout 485 transceiver is interchangeable here, the pinout is
the common MAX485 layout.

## Camera

| Field | Value |
|-------|-------|
| Part | IMX296 (intended, not finalized) |
| Interface | CSI |
| Type | Global shutter, monochrome |

Global shutter avoids the rolling-shutter skew a moving robot would put into tag
corners, and bearing accuracy depends on corner positions. Monochrome is enough
for AprilTags, the detector runs on grayscale, and it is cheaper and lower
bandwidth than color. See the Pi platform note for why the vision workload is
light on memory.

## Pi platform

| Field | Value |
|-------|-------|
| Board | Raspberry Pi 4 |
| RAM | 2 GB |
| OS | Pi OS Bookworm, headless (Lite) |

2GB is enough. The vision plus fusion app is a few hundred MB resident: OpenCV
and Python are the bulk, one mono frame is about 1.6 MB, the EKF is negligible.
AprilTag detection is classical computer vision with no model weights, so it is
memory light. The real constraint is CPU time per frame, tuned with resolution
and frame rate, not RAM. 4GB was considered but the runtime never needs it, and
the things that could (source builds, a desktop) are avoided by using prebuilt
packages and developing on a laptop. RAM is soldered, so the only argument for
4GB is reuse in a heavier future project.

### Pi GPIO usage

| GPIO | Pin | Use |
|------|-----|-----|
| 12 | 32 | UART5 TXD, RSTX to transceiver DI |
| 13 | 33 | UART5 RXD, RSRX from transceiver RO (unused) |
| 6 | 31 | RS-485 EN (DE + /RE), 10k pulldown |

UART5 needs `dtoverlay=uart5` and presents as `/dev/ttyAMA5`. See
`docs/pi_setup.md`.

## To be filled in

Add a section and the reasoning as each is chosen.

- Regulators and protection on the Pi HAT
- Connectors for the encoder harness
