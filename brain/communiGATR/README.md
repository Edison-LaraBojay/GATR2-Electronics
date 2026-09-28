# communiGATR

Brain side of the Pi link (brain link v4) over USB or RS-485. It sends the
robot profile, reads the field map and estimates, places the robot, runs
calibration and recovery actions, and gives planning and control the robot
state as an investiGATR `StateSource`. It does not plan or drive.

- `Client`: the protocol over a `BytePort`, polled with a time; no I/O of its
  own, no threads.
- `LinkDriver`: `StateSource` and `PathSink` over the client, in SI units.
- `RobotProfile`: the robot description, checked with the Pi's rules before
  upload.
- `readiness`: the status summary both programs show.
- `ProsLink` (PROS): one class for both transports, with its own poll task
  and bounded locks. `ProsVexImu` (PROS): the Brain VEX IMU bench source.
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
- [Brain setup](../../docs/brain_setup.md): where settings live, build and
  upload, calibration, placement.
- [actuGATR](../../docs/actugatr.md): movement on top of this link.
