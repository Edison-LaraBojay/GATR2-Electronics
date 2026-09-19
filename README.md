# GATR2 Electronics

Electronics, firmware, shared protocols, and Raspberry Pi sensing software for
the GATR2 VEX robot.

```text
encoders / IMU -> RP2040 acquisition -> Raspberry Pi estimation -> V5 Brain
camera / other Pi-connected sensors ----------^
```

The Pi combines timestamped sensor measurements into robot localization and
information about a requested physical landmark. The Brain owns destinations,
alignment behavior, mechanisms, and motor control.

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

The [Navigatr overview](pi/navigatr/README.md) is the entry point for the Pi
runtime design, implementation coverage, and build commands. Its design uses
one program with independently scheduled localization and landmark-estimation
pipelines, standard stage inputs and outputs, and timestamped pose history.
Neither pipeline requires a particular sensor family or relative execution rate.

Robot position and landmark position use configured field coordinates. The
landmark report identifies one requested reference and its rotation away from
nominal field orientation, accounting for declared object symmetry. Static field
definitions remain separate from the recommended cache of accepted measured
object estimates. Observations update that cache; reporting selects the requested
reference, and selection prioritizes processing work.

- [Architecture and scheduling](pi/navigatr/docs/architecture.md)
- [Coordinates and heading](pi/navigatr/docs/coordinates.md)
- [Landmark reporting and retention](pi/navigatr/docs/landmarks.md)
- [Inspection service and browser viewer](pi/navigatr/docs/inspection.md)
- [Pi camera setup and libcamera build](pi/navigatr/docs/pi_camera_setup.md)

A hardware-free demo (`pi/navigatr/config/demo/`) drives the whole runtime
from a synthetic rig and serves the field viewer on loopback; the README above
has the run and SSH port-forward commands.

## Repository map

- [`pcb/`](pcb/) - KiCad boards, symbols, footprints, and hardware revisions.
- [`pico/`](pico/) - RP2040 acquisition firmware.
- [`pi/navigatr/`](pi/navigatr/) - Pi sensing and estimation runtime.
- [`brain/`](brain/) - V5-side integration surface.
- [`common/`](common/) - shared framing and wire codecs.
- [`bench/`](bench/) - host and hardware bring-up utilities.
- [`docs/`](docs/) - supporting hardware, interface, and setup material.

## Hardware and bring-up references

- [Contributing](CONTRIBUTING.md)
- [Hardware architecture](docs/hardware.md)
- [Shared interfaces and wire frames](docs/interfaces.md)
- [Raspberry Pi setup](docs/pi_setup.md)

Physical values require measurement or independent verification. A `.xml.in`
configuration contains unresolved values and is not a deployment profile.
