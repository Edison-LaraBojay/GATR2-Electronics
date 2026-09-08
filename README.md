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
