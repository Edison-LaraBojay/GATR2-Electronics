# Documentation

## Runtime and configuration

- [Set up and run Navigatr](../pi/navigatr/docs/setup.md): complete Pi/Pico bring-up,
  camera calibration, robot geometry, two- and three-wheel camera configurations,
  and the live viewer.
- [Navigatr](../pi/navigatr/README.md): build, run, demo, and implementation coverage.
- [Configuration](../pi/navigatr/docs/configuration.md): default profile, command-line
  overrides, inline XML, and referenced fragments.
- [Architecture](../pi/navigatr/docs/architecture.md): construction, stage contracts,
  localization, field estimation, and workers.
- [Coordinates](../pi/navigatr/docs/coordinates.md): frames, heading, camera offsets,
  measurement time, and pose history.
- [Landmarks and targets](../pi/navigatr/docs/landmarks.md): association, retained
  field estimates, target resolution, and the actual Brain output.
- [Resources](navigatr_resources.md) and [sensors](navigatr_sensors.md): registered
  implementations and their configuration contracts.
- [Inspection](../pi/navigatr/docs/inspection.md): browser field viewer, camera
  previews, and inspection protocol.

## Hardware and deployment

- [PCB overview](../pcb/README.md): board families, revisions, and validation status.
- [Hardware](hardware.md): sensor roles, firmware settings, and communication wiring.
- [Pico firmware](../pico/README.md): acquisition and sensor telemetry.
- [Wire interfaces](interfaces.md): sensor, pose, and command frames.
- [Pi access](pi_setup.md): SSH and provisioning.
- [Pi camera setup](../pi/navigatr/docs/pi_camera_setup.md): libcamera build and
  capture checks.
- [Field assets](../pi/navigatr/docs/field_assets.md): nominal Override geometry and
  its CAD sources.
- [Calibration inventory](../pi/navigatr/docs/calibration_inventory.md): required
  robot measurements and template values.
- [Attitude firmware follow-up](../pi/navigatr/docs/attitude_firmware_followup.md):
  the work needed to provide live roll and pitch.
- [RS-485 bench test](../bench/rs485_link/README.md): independent transmit bring-up.

[Contribution conventions](contributing/README.md) describe the repository layout
and how to add board families and revisions. Host tests and synthetic demos verify
software behavior; hardware validation status is recorded in the relevant board
and deployment documents.
