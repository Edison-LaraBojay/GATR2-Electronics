# Magnetic encoder

[All PCBs](../README.md)

A magnetic rotary encoder board for measuring tracking-wheel rotation. The
design aims to provide a compact alternative to a VEX rotation sensor, sized
for custom tracking-wheel assemblies.

A magnet rotates with the wheel shaft while the sensor reads its angle without
physical contact. The sensing element requires no rubbing contacts or geared
coupling; wheel and bearing friction still depend on the mechanical assembly.
This supports a compact tracking wheel with low mechanical resistance.

The encoder's A/B quadrature signals let the acquisition hardware count movement
and determine direction. With the wheel diameter and placement configured, the
localization software uses those readings to estimate robot motion.

## Iterations

| Revision | Design and status |
|---|---|
| [Magnetic encoder v1](MagneticEncoder_v1/README.md) | AS5047P board with an A/B quadrature harness and separate SPI connections. Schematic and layout are available. |
