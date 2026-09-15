# Pi HAT v3

[All PCBs](../README.md) · [Previous revision: v2](../PiHat_v2/README.md)

**In progress.** This is the current Raspberry Pi/Pico carrier revision for the
tracking-wheel, SPI IMU, and V5 Brain connections. The schematic and PCB are
still being developed.

## Changes from v2

- Revises routing with closer attention to signal return paths and LDO
  placement. The saved layout includes a 3.3 V copper pour on the back layer;
  routing and copper distribution remain under development.
- Organizes the interfaces into separate schematic sheets, with one
  tracking-wheel sheet reused for three channels.
- Changes the SPI IMU connector from eight pins to six pins, removing the
  interrupt connection and one ground pin.
- Records ST3485ECDR in the schematic, matching the designer's substitution
  during v2 assembly.
- Provides separate 5 V/ground connectors for HAT power `IN` and fan power
  `OUT`. Both purposes are identified in the root schematic.
- Adds an internal camera-ribbon slot to avoid the sharp bend required by the
  earlier HAT. The designer derived its dimensions from a Raspberry Pi 4 model
  in Fusion; the slot is present in the saved board outline.

## Power and peripheral plans

The intended system supply is an external battery, BMS, and buck converter
feeding regulated 5 V into the HAT's input. This revision is intended to use
that connection for power instead of the Pi's USB-C input. The battery, BMS, and
buck converter are external to the HAT; the onboard AP2112K remains a 3.3 V LDO.

Planned additions are an audio amplifier, a speaker connection, and an LED port
controlled through the Pico. Audio is primarily for spoken debugging feedback,
such as successful IMU initialization; LEDs provide visual feedback and
lighting. Optional Bluetooth speaker use is a future integration idea.
These audio and LED circuits are not yet present in the saved schematic.

The mechanical source model is available in the local workspace as
`models/Raspberry Pi 4 Model B.STEP`; it is not tracked in Git. The camera-slot
dimensions and mechanical fit still belong to the ongoing v3 design work.

## Files

- [KiCad project](RP4Hat/RP4Hat.kicad_pro)
- [Root schematic](RP4Hat/RP4Hat.kicad_sch): Pi/Pico connections and the top-level wiring.
- [PCB layout](RP4Hat/RP4Hat.kicad_pcb)
- [Custom Pi symbols](RP4Pinout/) and [imported regulator assets](parts/AP2112K-3.3TRG1/)

The root schematic currently instantiates these interface sheets:

| Sheet | Role |
|---|---|
| [TrackingWheel](RP4Hat/TrackingWheel.kicad_sch) | Reused for tracking-wheel channels 0, 1, and 2. |
| [SPI_IMU](RP4Hat/SPI_IMU.kicad_sch) | External SPI IMU connection. |
| [VexBrainPort](RP4Hat/VexBrainPort.kicad_sch) | RS-485 transceiver and V5 Brain connector. |

Open the root project to follow these connections in context. See the
[PCB overview](../README.md) for library-path notes.
