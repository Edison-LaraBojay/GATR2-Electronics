# brain

V5 Brain software. The libraries are portable C++ with host tests; PROS
programs compile them in place from this folder.

- [`investiGATR/`](investiGATR/README.md): waypoint navigation and motion
  control over an abstract `InputSource`.
- [`communiGATR/`](communiGATR/README.md): brain link v3 client and the
  Navigatr `InputSource`, with the PROS serial port and task wrapper.
- [`testing/`](testing/README.md): minimal PROS application using both
  libraries.
- [`localization-test/`](localization-test/README.md): PROS application that
  places the robot and reads back the Navigatr pose, without motor control.
- `gatr2_brain.mk`: make fragment that compiles the libraries into a PROS
  project.
- `CMakeLists.txt`: host build and tests of both libraries.

Host build and tests, from the repository root (the CI job `brain` runs the
same commands; CI does not build the PROS programs):

```
cmake -S brain -B build-brain
cmake --build build-brain -j
ctest --test-dir build-brain --output-on-failure
```

Docs: [investiGATR](../docs/investigatr.md),
[communiGATR](../docs/communigatr.md) (including PROS import and hardware
bring-up), and the [wire interface](../docs/interfaces.md#brain-link-v3).
Nothing here has run on the robot yet.
