# brain

V5 Brain software. The libraries are portable C++ with host tests; PROS
programs compile them in place from this folder.

- [`investiGATR/`](investiGATR/README.md): path planning (direct and obstacle
  avoiding) and the shared Brain types: field, state source, references,
  motion model, paths.
- [`communiGATR/`](communiGATR/README.md): the Pi link over USB or RS-485:
  sessions, robot profile upload, field map and estimates, placement,
  calibration and recovery, wheel calibration.
- [`actuGATR/`](actuGATR/README.md): movement: `goToDirect` and
  `goToAvoiding`, path following, tank and mecanum drives, the drive task.
- [`robot/`](robot/README.md): this robot's description (tracking wheels,
  IMU, footprint, link, start pose), shared by the programs.
- [`localization-test/`](localization-test/README.md): motor-free bench
  program: placement, pose, IMU and wheel calibration, recovery history.
- [`testing/`](testing/README.md): drive program: direct, avoiding and
  landmark tests, manual driving, tuning.
- `gatr2_brain.mk`: make fragment that compiles the libraries into a PROS
  project.
- `CMakeLists.txt`: host build and tests of the three libraries.

Host build and tests, from the repository root (the CI job `brain` runs the
same commands; CI does not build the PROS programs):

```
cmake -S brain -B build-brain
cmake --build build-brain -j
ctest --test-dir build-brain --output-on-failure
```

Setup, calibration, tuning and the test programs:
[Brain setup](../docs/brain_setup.md). Libraries:
[investiGATR](../docs/investigatr.md), [communiGATR](../docs/communigatr.md),
[actuGATR](../docs/actugatr.md), [wire interface](../docs/interfaces.md).
Nothing here has run on the robot yet.
