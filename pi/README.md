# Raspberry Pi sensing runtime

[`navigatr`](navigatr/README.md) combines timestamped measurements into a robot
pose and an estimate of the physical landmark requested by the V5 Brain. The
Brain chooses the reference and owns alignment, destinations, and motor control.

The runtime design uses independently scheduled localization and landmark
pipelines in one program. Each pipeline has defined inputs and outputs; its
implementation determines which sensors and algorithms it uses.

Start with the [runtime overview](navigatr/README.md), then read:

- [Architecture and scheduling](navigatr/docs/architecture.md)
- [Coordinates, heading, and measurement time](navigatr/docs/coordinates.md)
- [Landmark reporting and retention](navigatr/docs/landmarks.md)

The overview includes implementation coverage and build commands.
