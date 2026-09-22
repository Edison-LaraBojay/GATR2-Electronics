# Raspberry Pi sensing runtime

[`navigatr`](navigatr/README.md) combines timestamped measurements into a robot
pose and configured field-object estimates. Target resolution can latch a desired
robot pose for a requested target; the V5 Brain owns motor control.

The runtime uses independently scheduled estimation and field
pipelines in one program. Each pipeline has defined inputs and outputs; its
implementation determines which sensors and algorithms it uses.

Start with [Set up and run Navigatr](navigatr/docs/setup.md) for the complete
hardware, calibration, configuration, build, and viewer workflow. The
[runtime overview](navigatr/README.md) describes implementation coverage; then read:

- [Architecture and scheduling](navigatr/docs/architecture.md)
- [Coordinates, heading, and measurement time](navigatr/docs/coordinates.md)
- [Field estimates, targets, and reporting](navigatr/docs/landmarks.md)

The overview includes implementation coverage and build commands.
