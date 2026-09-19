# Pi HAT

[All PCBs](../README.md)

A carrier board that plugs directly into the Raspberry Pi's 40-pin header to
minimize loose wiring and provide a stable connection. Dedicated ports connect
the robot's sensors and peripherals without a collection of individual jumper
wires between the Pi and the acquisition hardware.

The HAT brings together the Pi, Pico acquisition module, tracking-wheel
connections, and the V5 Brain communication interface. Later revisions add an
IMU connection, sensor power regulation, fan power, and provisions for other
peripherals. Port choices can evolve with the needs of future robots and
seasons.

Mechanical clearance, power distribution, and readable schematics are part of
the design alongside the electrical interfaces. Each iteration documents its
specific connections, layout changes, and bench results.

## Iterations

| Revision | Changes and status |
|---|---|
| [Pi HAT v1](PiHat_v1/README.md) | Initial two-layer design. Tracking-wheel acquisition and Brain communication tested successfully. |
| [Pi HAT v2](PiHat_v2/README.md) | Four layers, a dedicated 3.3 V LDO, and an SPI IMU port. Initial power-up and LDO operation confirmed; full testing pending. |
| [Pi HAT v3](PiHat_V3/README.md) | In progress: revised routing, separate power input/fan output, a camera-ribbon slot, and schematic sheets. Audio and LED interfaces are planned. |

Revision names follow the folders. The older board silkscreens use different
labels: `PiHat_v1` says `V0`, and `PiHat_v2` says `V1`.
