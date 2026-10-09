# Documentation

## Current libraries and hardware

- [investiGATR](investigatr.md): pose inputs, field geometry and planning.
- [actuGATR](actugatr.md): following, control and drivetrain adapters.
- [Brain libraries](../brain/README.md): host build and integration boundaries.
- [Pico acquisition](../pico/aggreGATR/README.md): encoder and IMU firmware.
- [PCB overview](../pcb/README.md): sensor boards and historical designs.
- [Contribution conventions](contributing/README.md): PCB families and iterations.
- [Component names](naming.md): active, legacy and deprecated components.

## Legacy communication

[communiGATR](communigatr.md) and its [wire specification](interfaces.md)
describe the retained Pi-link implementation. They are references for the
upcoming redesign, not instructions for an implemented MCU link.

## Deprecated applications

[deprecated/](../deprecated/README.md) indexes the retired Pi runtime, viewer,
test apps, Pi HATs and deployment instructions. No replacement MCU localization
or end-to-end robot application exists yet.
