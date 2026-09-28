# Documentation

## Runtime and configuration

- [Set up and run Navigatr](../pi/naviGATR/docs/setup.md): complete Pi/Pico bring-up,
  camera calibration, robot geometry, three-wheel camera configurations, and the
  live viewer.
- [Parallel-wheel bring-up](../pi/naviGATR/docs/parallel_wheel_bringup.md): the
  current robot's systems test, two parallel wheels + BNO08X, localization
  first and camera second.
- [Navigatr](../pi/naviGATR/README.md): build, run, demo, and implementation coverage.
- [Configuration](../pi/naviGATR/docs/configuration.md): default profile, command-line
  overrides, inline XML, and referenced fragments.
- [Architecture](../pi/naviGATR/docs/architecture.md): construction, stage contracts,
  localization, field estimation, and workers.
- [Coordinates](../pi/naviGATR/docs/coordinates.md): frames, heading, camera offsets,
  measurement time, and pose history.
- [Landmarks and targets](../pi/naviGATR/docs/landmarks.md): association, retained
  field estimates, target resolution, and the actual Brain output.
- [Resources](navigatr_resources.md) and [sensors](navigatr_sensors.md): registered
  implementations and their configuration contracts.
- [Inspection](../pi/naviGATR/docs/inspection.md): the spectaGATR browser
  viewer, the `navigatr.inspect/2` feed (state, diag, telemetry,
  instrumentation), latency terms, live graphs and replay.
- [Capture and export](../pi/naviGATR/docs/capture.md): bounded Pi-side
  recording with manual and automatic triggers, the ZIP of CSVs and its
  metadata.
- [Camera preview](../pi/naviGATR/docs/camera_preview.md): live camera
  pictures on the Brain-profile USB setup without field correction.
- [Pico link](../pi/naviGATR/docs/pico_link.md): Pico identity, status,
  commands, the optional diagnostic frame and the link instrumentation.
- [Viewer perf harness](../pi/naviGATR/tools/perf/README.md): the inspection
  and browser latency baseline and how to rerun it.

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
- [Pi camera setup](../pi/naviGATR/docs/pi_camera_setup.md): libcamera build and
  capture checks.
- [Field assets](../pi/naviGATR/docs/field_assets.md): nominal Override geometry and
  its CAD sources.
- [Calibration inventory](../pi/naviGATR/docs/calibration_inventory.md): required
  robot measurements and template values.
- [Attitude firmware follow-up](../pi/naviGATR/docs/attitude_firmware_followup.md):
  the work needed to provide live roll and pitch.
- [RS-485 bench test](../bench/rs485_link/README.md): independent transmit bring-up.

[Component names](naming.md) lists every -GATR name, what it does and why
it fits, and the renames with migration steps.

[Contribution conventions](contributing/README.md) describe the repository layout
and how to add board families and revisions. Host tests and synthetic demos verify
software behavior; hardware validation status is recorded in the relevant board
and deployment documents.
