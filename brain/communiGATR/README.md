# communiGATR

Brain side of the Navigatr brain link v4. `Client` runs the protocol over a
`BytePort`: sessions, request scheduling and retries, robot profile upload
and runtime change, field map and estimate transfer, placement, control,
wheel readings and path reports. `LinkDriver` is the investiGATR `StateSource`
and `PathSink` over the client. `RobotProfile` is the Brain robot
configuration in SI units, checked with the Pi's rules before upload.
`readiness` is the one status summary both apps show. `usb_line` is the NG1
line codec for the V5 USB console. `wheel_calibration`, `startup_placement`
and `link_events` are application helpers for calibration, start-up
placement and recovery history.

PROS programs use `ProsLink` (both transports, its own poll task, bounded
locks) and `ProsVexImu` for the Brain VEX IMU bench source.

- `include/communigatr/`: public headers (`pros_*.h` are PROS only)
- `src/`: portable sources, host and PROS
- `pros/`: PROS only sources, never built on the host
- `sim/`: host only fake Pi, RS-485 bus, USB console and test rig
- `tests/`: host tests

Client, driver, restarts, timing and PROS use:
[docs/communigatr.md](../../docs/communigatr.md). Wire protocol:
[docs/interfaces.md](../../docs/interfaces.md).
