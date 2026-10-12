# Documentation

## Active demo and hardware

- [Drive and localization demo](../brain/localization-demo/README.md): start here for manual eight-motor mecanum drive, Pico firmware, Pi relay, Brain odometry, calibration and visualization.
- [Encoder record](../encoder-record/README.md): shared record format and codec.
- [Pi encoder relay](../pi/encoder-relay/README.md): forwards encoder records from UART to Brain USB.
- [spectaGATR](../spectaGATR/README.md): live pose viewer ([protocol](../spectaGATR/docs/PROTOCOL.md)).
- [Brain programs and publisher](../brain/README.md): localization program, pose publisher and host tests.
- [Pico acquisition](../pico/aggreGATR/README.md): encoder and IMU firmware.
- [PCB overview](../pcb/README.md): sensor boards and historical designs.
- [Contribution conventions](contributing/README.md): PCB families and iterations.
- [Component names](naming.md): active and retired components.

## Retired system

The former planning and motor-control libraries, old Brain test applications,
and old Pi runtime and link are retired. Previous versions remain in Git
history. The active demo uses a small encoder relay on the Pi and computes
pose on the Brain alongside manual mecanum drive; its hardware route is still
unverified. Configure the eight motor ports and reversals before driving.
