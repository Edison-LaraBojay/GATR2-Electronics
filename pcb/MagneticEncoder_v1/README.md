# Magnetic encoder v1

[All PCBs](../README.md)

AS5047P-ATSM magnetic rotary encoder board for tracking-wheel displacement.
Its four-pin horizontal JST-PH harness carries 3.3 V, A, B, and ground to the
acquisition board. Separate solder connections expose the SPI signals.

The wheel interface uses A/B quadrature; the encoder's index/PWM pin is
unconnected in this design. This is the only magnetic encoder revision in the
repository.

## Files

- [KiCad project](MagneticEncoder.kicad_pro)
- [Schematic](MagneticEncoder.kicad_sch)
- [PCB layout](MagneticEncoder.kicad_pcb)
- [Footprint library](MagnetEncoderFootprint.pretty/)
- [Symbol library table](sym-lib-table)

The AS5047P symbol library entry uses a workstation-specific path, although the
schematic contains an embedded copy of the symbol. See the
[PCB overview](../README.md) for library-path and fabrication-status notes.
