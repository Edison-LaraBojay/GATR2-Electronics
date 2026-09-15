# IMU v1

[All PCBs](../README.md) · [Next project: v2](../IMU_v2/README.md)

**Failed assembly.** The designer reports that the component's bottom-view
pin arrangement was used for the PCB footprint without the required change of
view. The resulting footprint was mirrored, so the device could not be
correctly soldered onto the board. This revision and its Gerber archive retain
that error; the correction is being developed in v2.

Standalone 3.3 V inertial sensor board using the custom ASM330 symbol. It
connects to the carrier over an eight-pin horizontal JST-PH connector carrying
SPI, an interrupt signal, power, and ground.

The [hardware rationale](../../docs/hardware.md) identifies the intended part as
ASM330LHHG1. The schematic's custom symbol does not record the complete ordering
code in its value field.

## Files

- [KiCad project](IMU_v1/IMU_v1.kicad_pro)
- [Schematic](IMU_v1/IMU_v1.kicad_sch)
- [PCB layout](IMU_v1/IMU_v1.kicad_pcb)
- [Saved Gerber archive](IMU_v1/gerbers.zip)
- [Custom symbol](IMU_v1/ASM330.kicad_sym) and [footprint library](IMU_v1/ASM330.pretty/)

The schematic and layout are retained as the record of the failed iteration.
See [v2](../IMU_v2/README.md) for the correction effort.
