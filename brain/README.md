# Brain programs and publisher

The active robot program is the
[encoder localization demo](localization-demo/README.md). The Pico sends
encoder counts through the Pi relay; the Brain combines them with its VEX
IMU heading, computes the pose and publishes it to spectaGATR. The program
never commands motors. Its guide covers the complete setup, calibration and
hardware checks.

- [localization-demo](localization-demo/README.md): encoder and IMU odometry
  in a PROS program, with portable host tests.
- [spectaGATR](spectaGATR/README.md): publishes a caller-supplied pose to the
  spectaGATR laptop viewer; example PROS project in
  [spectaGATR-demo](spectaGATR-demo/README.md).
- `gatr2_brain.mk`: compiles the spectaGATR publisher into a PROS project.

The Pico supplies counts, the Pi forwards records, and the Brain owns pose
estimation. Field rendering belongs to the laptop viewer. The complete
localization route has not been tested on hardware.

The former planning and motor-control libraries and old Brain test
applications are retired. Previous versions remain in Git history.

Build and test the portable publisher and localization code:

```sh
cmake -S brain -B build-brain
cmake --build build-brain --parallel
ctest --test-dir build-brain --output-on-failure
```

Standalone localization tests and PROS builds are documented in the
[demo guide](localization-demo/README.md).
