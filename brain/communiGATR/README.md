# communiGATR

Brain side of the Navigatr brain link v3. `Client` runs the protocol
(sessions, request scheduling, retries, reply correlation) over a `BytePort`;
`Driver` is an investiGATR `InputSource` over the client; `ProsDriver`
and `ProsSerialPort` run it on a V5 smart port from a PROS task.

- `include/communigatr/`: public headers (`pros_*.h` are PROS only)
- `src/`: portable sources, host and PROS
- `pros/`: PROS only sources, never built on the host
- `sim/`: host only fake Pi, half-duplex bus and test rig
- `tests/`: host tests

Driver, client, restarts, timing and PROS use:
[docs/communigatr.md](../../docs/communigatr.md). Wire protocol:
[docs/interfaces.md](../../docs/interfaces.md).
