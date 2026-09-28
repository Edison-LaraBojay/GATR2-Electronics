# GATR2 Electronics

Electronics, firmware, shared protocols, Raspberry Pi sensing software, and V5
Brain navigation libraries for the GATR2 VEX robot.

```text
encoders / IMU -> RP2040 acquisition -> Raspberry Pi estimation <-> V5 Brain
camera / other Pi-connected sensors ----------^
```

The Pi combines timestamped sensor measurements into robot localization and
information about a requested physical landmark. The Brain asks for them over a
request/reply link and owns destinations, alignment behavior, mechanisms, and
motor control.

## PCBs

The [PCB overview](pcb/README.md) links each board family. Its README explains
the general purpose, and the iteration folders document changes, test results,
and KiCad project files.

| Board family | Purpose |
|---|---|
| [Pi HAT](pcb/PiHat/README.md) | Plugs into the Pi's 40-pin header to reduce loose wiring and provide dedicated sensor and Brain connections. |
| [IMU](pcb/IMU/README.md) | Inertial measurements for attitude estimation, primarily heading. |
| [Magnetic encoder](pcb/MagneticEncoder/README.md) | Contactless rotation sensing for compact custom tracking-wheel assemblies. |

## Pi runtime

Start with [Set up and run Navigatr](pi/naviGATR/docs/setup.md) for Pi/Pico setup,
building, camera calibration, mounting measurements, two- and three-wheel IMU +
camera configurations, and the live field viewer.

The [Navigatr overview](pi/naviGATR/README.md) is the entry point for the Pi
runtime design, implementation coverage, and build commands. Its design uses
one program with independently scheduled localization and landmark-estimation
pipelines, standard stage inputs and outputs, and timestamped pose history.
Neither pipeline requires a particular sensor family or relative execution rate.

Robot position and field-object positions use configured field coordinates.
Field estimation starts with nominal landmark poses and can update them from
accepted observations. The Pi sends nothing unasked: each Brain request gets
one reply. A state reply carries the robot pose with its measurement age and
anchor identity; the field map (objects and their obstacle boxes) and the
current field-object estimates, labeled nominal or observed, are read as
chunked documents. Headings on the wire are full field headings, and the Brain
applies its own destination offset. The inspection viewer also displays
heading differences from nominal orientation. See the landmark documentation
for selection and retention behavior, and the
[brain link](docs/interfaces.md) for the wire contract.

- [Architecture and scheduling](pi/naviGATR/docs/architecture.md)
- [Coordinates and heading](pi/naviGATR/docs/coordinates.md)
- [Landmark reporting and retention](pi/naviGATR/docs/landmarks.md)
- [Inspection service and browser viewer](pi/naviGATR/docs/inspection.md)
- [Pi camera setup and libcamera build](pi/naviGATR/docs/pi_camera_setup.md)

A hardware-free demo (`pi/naviGATR/config/demo/`) drives the whole runtime
from a synthetic rig and serves the field viewer on loopback; the README above
has the run and SSH port-forward commands.

## Brain

[`brain/`](brain/README.md) holds portable C++ libraries with host tests and
the PROS programs that compile them in place.

- [investiGATR](docs/investigatr.md): path planning, direct and around the
  field's obstacle boxes, and the shared Brain types.
- [communiGATR](docs/communigatr.md): the Brain side of the Pi link over USB or
  RS-485: robot profile upload, field map, placement, calibration and recovery.
- [actuGATR](docs/actugatr.md): `goToDirect` and `goToAvoiding`, path
  following, tank and mecanum drives.

The Brain describes the robot (tracking wheels, IMU, footprint) and sends it
to the Pi as a robot profile; see [Brain setup](docs/brain_setup.md) for
configuration, calibration and the two test programs. The libraries are host
tested and compile in the PROS build; none of the Brain code has run on the
robot yet.

## Repository map

- [`pcb/`](pcb/) - KiCad boards, symbols, footprints, and hardware revisions.
- [`pico/aggreGATR/`](pico/aggreGATR/README.md) - RP2040 acquisition firmware.
- [`pi/naviGATR/`](pi/naviGATR/README.md) - Pi sensing and estimation runtime, with the spectaGATR browser viewer.
- [`brain/`](brain/README.md) - V5 Brain libraries (investiGATR, communiGATR, actuGATR) and PROS programs (locaGATR, operaGATR).
- [`translaGATR/`](translaGATR/) - shared wire codec used by the Brain, Pi and Pico.
- [`bench/`](bench/) - host and hardware bring-up utilities.
- [`docs/`](docs/) - supporting hardware, interface, and setup material; [component names](docs/naming.md).

## Hardware and bring-up references

- [Contributing](CONTRIBUTING.md)
- [Hardware architecture](docs/hardware.md)
- [Shared interfaces and wire frames](docs/interfaces.md)
- [Raspberry Pi setup](docs/pi_setup.md)

Physical values require measurement or independent verification. A `.xml.in`
configuration contains unresolved values and is not a deployment profile.
