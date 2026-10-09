# PCBs

Each active board family has a purpose README and numbered iterations with
schematics, layouts, libraries and design/test notes.

| Board | Purpose |
|---|---|
| [IMU](IMU/README.md) | Inertial measurements for attitude and heading. |
| [Magnetic encoder](MagneticEncoder/README.md) | Compact tracking-wheel encoders. |
| [Pi HAT](PiHat/README.md) | Deprecated Pi/Pico carrier; complete design history retained. |

The complete [Pi HAT family](PiHat/README.md) is deprecated
with the Pi-based system. Its design history remains available for reference.

Choose an iteration and open its `.kicad_pro`. Keep project files and custom
libraries together. The magnetic encoder symbol table contains a historical
workstation-specific library path that may need configuring on another machine.

Follow the [PCB contribution guide](../docs/contributing/README.md) for new
boards and iterations. [Pico acquisition](../pico/README.md) remains active.
