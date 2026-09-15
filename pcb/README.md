# PCBs

Custom boards for the GATR2 sensing system: a Raspberry Pi HAT, an SPI inertial
sensor board, and magnetic tracking-wheel encoders. **Pi HAT v3 is the revision
currently being worked on.**

The Pi HAT was created to minimize loose wiring and provide a stable connection
through the Raspberry Pi's 40-pin header. The remaining cables plug into ports
chosen for the robot's sensors and peripherals. Those ports can change with the
needs of later robots and seasons.

## Board revisions

| Revision | Purpose and changes | Design state |
|---|---|---|
| [Pi HAT v3](PiHat_v3/README.md) | Revises routing and regulator placement; adds separate 5 V input/fan output and a camera-ribbon slot; organizes the interfaces into schematic sheets. Audio and LED interfaces are planned. | In progress. |
| [Pi HAT v2](PiHat_v2/README.md) | Moves to four layers; adds a 3.3 V LDO and SPI IMU port; uses Pi USB-C power and the two-pin connector for a fan. | Initial power-up and LDO operation confirmed; full validation pending. |
| [Pi HAT v1](PiHat_v1/README.md) | Two-layer Pi/Pico carrier for tracking wheels and Brain communication, with external 5 V input and no dedicated IMU port. | Tracking-wheel acquisition and Brain communication tested successfully. |
| [IMU v1](IMU_v1/README.md) | ASM330-family sensor board with an eight-pin SPI/interrupt connector. | Failed assembly: the footprint was mirrored. |
| [IMU v2](IMU_v2/README.md) | Intended to correct the v1 footprint, with routing and mounting-hole improvements under consideration. | In progress; the saved schematic and layout are still empty. |
| [Magnetic encoder v1](MagneticEncoder_v1/README.md) | AS5047P tracking-wheel board with an A/B quadrature harness and separate SPI connections. | Schematic and layout. |

Physical results above are the board designer's reported bench experience.
Revision READMEs separate those results from saved circuit details and planned
work. Gerber archives are retained for Pi HAT v2 and IMU v1; the IMU v1 archive
predates the footprint correction.

Revision names here follow the directory names. The older Pi HAT silkscreen
labels differ: the `PiHat_v1` board says `V0`, and `PiHat_v2` says `V1`.

## Working with a board

Open the `.kicad_pro` linked from that revision's README. Keep its schematic,
layout, and accompanying libraries together. The existing revision directories
are retained so each design can be opened in its original project structure.

The Pi HAT symbol tables and the magnetic encoder's symbol table still contain
workstation-specific library paths. The Pi HAT's custom symbols are included in
its `RP4Pinout/` directory; library paths may need updating when editing on
another machine. Project locations and library notes are linked from each
revision README.

When a revision changes, update its short description and the comparison with
the previous revision. Record fabrication or test results alongside that
revision when available. The schematic remains the source for the BOM and
connector pin assignments.

## Related documentation

- [Hardware parts and rationale](../docs/hardware.md)
- [RP2040 acquisition firmware](../pico/README.md)
- [Pi sensing and estimation software](../pi/navigatr/README.md)
- [Shared interfaces](../docs/interfaces.md)
