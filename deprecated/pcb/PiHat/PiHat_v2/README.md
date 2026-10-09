# Pi HAT v2

[Pi HAT overview](../README.md) · [All PCBs](../../README.md) · [Previous: v1](../PiHat_v1/README.md) · [Next: v3](../PiHat_V3/README.md)

Second Raspberry Pi/Pico carrier design, extending the tracking-wheel and
V5 Brain connections with a dedicated SPI IMU connection, its own 3.3 V
regulator, and a four-layer layout.

## Changes from v1

- Moves from two copper layers to four. The saved front-to-back arrangement is
  signal / GND / power / signal: `In1.Cu` carries GND, while `In2.Cu` has a 5 V
  plane and a local 3.3 V zone.
- Adds an AP2112K-3.3 LDO so the sensor supply no longer relies on the Pico's
  3.3 V output.
- Adds an eight-pin SPI IMU connector carrying the SPI bus, interrupt, and power.
- Changes the harness connectors from vertical JST-XH 2.50 mm to horizontal
  JST-PH 2.00 mm footprints.
- Uses the Pi's USB-C input for system power and the HAT's two-pin 5 V/ground
  connector for a fan. USB-C is on the Pi; the HAT connector is not labeled
  with a power direction.

## Assembly substitution

The saved schematic specifies THVD1410 for the RS-485 transceiver.
**ST3485ECDR** was substituted during assembly because the originally selected
part was out of stock and the ST part was easier to source. This
assembly choice is not yet reflected in the v2 schematic; v3 records it.

## Bench results

Initial power-up completed without observed component damage, and the 3.3 V
LDO operates. Full board testing is still pending; these
results do not establish tracking-wheel, IMU, or Brain-interface operation on v2.

## Files

- [KiCad project](RP4Hat/RP4Hat.kicad_pro)
- [Root schematic](RP4Hat/RP4Hat.kicad_sch)
- [PCB layout](RP4Hat/RP4Hat.kicad_pcb)
- [Saved Gerber archive](gerbers.zip)
- [Custom Pi symbols](RP4Pinout/) and [imported regulator assets](parts/AP2112K-3.3TRG1/)

This is the predecessor to the in-progress [v3](../PiHat_V3/README.md).
See the [Pi HAT overview](../README.md) for revision-label notes and the
[PCB overview](../../README.md) for library-path notes.
