# communiGATR

Legacy Brain side of the Pi link (brain link v4), retained pending redesign.
The Pi runtime and viewer are archived. This library still implements their
protocol; it is not the future MCU pose backend. The transport abstractions
(`BytePort` and the serial adapter) are reusable independently of the protocol.
Field geometry belongs on the Brain; the future MCU supplies pose only.

The retained client runs over USB or RS-485. It sends the
robot profile, reads the field map and estimates, places the robot, runs
calibration and recovery actions, and gives planning and control the robot
state as an investiGATR `StateSource` and legacy field data through the separate
`FieldSource` interface. It does not plan or drive.

- `Client`: the protocol over a `BytePort`, polled with a time; no I/O of its
  own, no threads.
- `LinkDriver`: separate `StateSource`, `FieldSource` and optional `PathSink`
  interfaces over the legacy client, in SI units.
- `RobotProfile`: the robot description, checked with the Pi's rules before
  upload.
- `readiness`: the legacy link status summary.
- `ProsLink` (PROS): one class for both transports, with its own poll task
  and bounded locks. `ProsVexImu` (PROS): the Brain VEX IMU bench source.
- `attitude`: VEX IMU roll and pitch in the robot frame, for TELEMETRY.
- `VexImuRecalibration`: starts the VEX IMU calibration only after the Pi
  reports the robot still.
- `wheel_calibration`, `startup_placement`, `link_events`: application
  helpers.

Folders:

- `include/communigatr/`: public headers (`pros_*.h` are PROS only)
- `src/`: portable sources, host and PROS
- `pros/`: PROS only sources, never built on the host
- `sim/`: host only fake Pi, RS-485 bus, USB console and test rig
- `tests/`: host tests

Docs:

- [communiGATR](../../docs/communigatr.md): API, profile, field sync,
  readiness, recovery, timing, PROS packaging, tests.
- [Wire interface](../../docs/interfaces.md): brain link v4 and documents.
- [actuGATR](../../docs/actugatr.md): movement on top of this link.
