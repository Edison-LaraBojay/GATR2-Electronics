# Documentation

## Runtime and configuration

- [Set up and run Navigatr](../pi/navigatr/docs/setup.md): complete Pi/Pico bring-up,
  camera calibration, robot geometry, three-wheel camera configurations, and the
  live viewer.
- [Parallel-wheel bring-up](../pi/navigatr/docs/parallel_wheel_bringup.md): the
  current robot's systems test, two parallel wheels + BNO08X, localization
  first and camera second.
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

## Brain

- [Brain setup](brain_setup.md): where robot settings live, building and
  uploading, the localization test and drive test programs, wheel and IMU
  calibration, placement, recovery, and what is not yet hardware validated.
- [investiGATR](investigatr.md): path planning and the shared Brain types
  (field, state source, references, motion model, paths); planner method,
  exits by walls, cost and limits.
- [communiGATR](communigatr.md): the Pi link over USB or RS-485: sessions,
  robot profile, field transfer, placement, calibration, recovery, timing,
  PROS packaging.
- [actuGATR](actugatr.md): movement commands, path following, tank and
  mecanum drives, drive ownership, tuning.
- [Brain folder](../brain/README.md): libraries, PROS programs, and the host
  build.

## Hardware and deployment

- [PCB overview](../pcb/README.md): board families, revisions, and validation status.
- [Hardware](hardware.md): sensor roles, firmware settings, and communication wiring.
- [Pico firmware](../pico/README.md): acquisition and sensor telemetry.
- [Wire interfaces](interfaces.md): Pico sensor frames and control, and the brain
  link v4 request/reply protocol (sessions, profile, documents, bus timing).
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
