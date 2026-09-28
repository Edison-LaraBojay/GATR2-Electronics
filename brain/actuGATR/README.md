# actuGATR

Brain-side movement: `Motion` (`goToDirect`, `goToAvoiding`), path followers,
tank and mecanum drive kinematics, the drive owner and PROS motor adapters.
Plans come from investiGATR; robot state and the field come from any
`investigatr::StateSource` (communiGATR over the Pi link).

- `include/actugatr/`: public headers
- `src/`: portable sources, host and PROS
- `pros/`: PROS-only sources (motor output, drive task)
- `sim/`: host drivetrain sim, simulated state source, test planner
- `tests/`: host tests; `trajectory_gtest.cpp` runs with the real planner

Usage, tuning and limits: [docs/actugatr.md](../../docs/actugatr.md).
