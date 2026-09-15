# Pi HAT v1

[All PCBs](../README.md) · [Next revision: v2](../PiHat_v2/README.md)

Initial Raspberry Pi 4 carrier, built to get tracking-wheel acquisition and
V5 Brain communication working while reducing loose wiring. The HAT plugs into
the Pi's 40-pin header and carries a Raspberry Pi Pico and dedicated harness
connectors.

## Design

- Two copper layers, with the circuitry drawn in one main schematic.
- Three tracking-wheel ports and an SN65HVD11HD RS-485 Brain interface.
- Vertical JST-XH 2.50 mm harness connectors.
- A two-pin 5 V/ground connector, intended for external power input.
- Sensor 3.3 V supplied by the Pico; there is no separate HAT LDO.
- No dedicated IMU port. The SPI IMU connector first appears in v2.

## Bench results

The designer reports successful tracking-wheel acquisition and communication
with the Brain. The board powered up and operated without observed component
damage. These results cover the functions exercised on this revision.

## Files

- [KiCad project](RP4Hat/RP4Hat.kicad_pro)
- [Schematic](RP4Hat/RP4Hat.kicad_sch)
- [PCB layout](RP4Hat/RP4Hat.kicad_pcb)
- [Custom Pi symbols](RP4Pinout/)

This is an earlier design; current HAT work is in [v3](../PiHat_v3/README.md).
See the [PCB overview](../README.md) for library-path and revision-label notes.
