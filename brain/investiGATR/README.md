# investiGATR

Portable waypoint navigation and motion control for the Brain: a `Navigator`
over an abstract `InputSource`, returning forward/turn demand for a tank drive.
No PROS, serial or motor dependencies.

- `include/investigatr/`: public headers
- `src/`: library sources, host and PROS
- `sim/`: host only drivetrain sim and `SimulatedSource`
- `tests/`: host tests

API, input contract, tuning, fallback and build: [docs/investigatr.md](../../docs/investigatr.md).
