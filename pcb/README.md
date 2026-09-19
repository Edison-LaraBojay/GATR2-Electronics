# PCBs

Custom boards for the GATR2 sensing system. Each board folder explains its
general purpose and contains its numbered iterations. Each iteration has its
own README describing changes, status, and KiCad project files.

## Boards

| Board | Purpose |
|---|---|
| [Pi HAT](PiHat/README.md) | A stable connection through the Pi's 40-pin header, with dedicated sensor and peripheral ports to reduce loose wiring. |
| [IMU](IMU/README.md) | Inertial measurements for robot attitude estimation, primarily heading. |
| [Magnetic encoder](MagneticEncoder/README.md) | Contactless rotation sensing for compact custom tracking-wheel assemblies. |

## Working with a board

Choose a board, then an iteration, and open the `.kicad_pro` linked from its
README. Keep the schematic, layout, and accompanying libraries together.

The Pi HAT symbol tables and the magnetic encoder's symbol table still contain
workstation-specific library paths. The Pi HAT's custom symbols are included in
its `RP4Pinout/` directory; library paths may need updating when editing on
another machine. Project locations and library notes are linked from each
revision README.

When a revision changes, update its short description and the comparison with
the previous revision. Record fabrication or test results alongside that
revision when available. The schematic remains the source for the BOM and
connector pin assignments.

For new boards and iterations, follow the
[PCB contribution guide](../docs/contributing/README.md).

## Related documentation

- [Hardware parts and rationale](../docs/hardware.md)
- [RP2040 acquisition firmware](../pico/README.md)
- [Pi sensing and estimation software](../pi/navigatr/README.md)
- [Shared interfaces](../docs/interfaces.md)
