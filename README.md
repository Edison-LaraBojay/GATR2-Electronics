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

The [PCB overview](pcb/README.md) lists the board revisions, what each does,
and the changes between them. Each revision folder has a short README and links
to its KiCad project, schematic, and layout.

| Board family | Purpose | Where to start |
|---|---|---|
| Pi HAT | Plugs into the Pi's 40-pin header to reduce loose wiring and provide dedicated sensor and Brain connections. | [v3](pcb/PiHat_v3/README.md) is in progress; [v2](pcb/PiHat_v2/README.md) has passed initial power-up; [v1](pcb/PiHat_v1/README.md) demonstrated tracking-wheel acquisition and Brain communication. |
| IMU | Separate inertial sensor board with an SPI connection. | [v1](pcb/IMU_v1/README.md) failed assembly due to a mirrored footprint; [v2](pcb/IMU_v2/README.md) is the correction effort, in progress. |
| Magnetic encoder | Measures tracking-wheel rotation using an AS5047P encoder. | [v1](pcb/MagneticEncoder_v1/README.md). |

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

- [Hardware architecture](docs/hardware.md)
- [Shared interfaces and wire frames](docs/interfaces.md)
- [Raspberry Pi setup](docs/pi_setup.md)

Physical values require measurement or independent verification. A `.xml.in`
configuration contains unresolved values and is not a deployment profile.
