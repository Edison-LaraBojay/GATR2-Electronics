# navigatr

Estimation runtime for the Pi. The semantic pipeline is fixed; XML is the
complete declaration of which implementations fill it, resources included.
Architecture in `docs/navigatr.md`, registered type catalogs in
`docs/navigatr_resources.md` and `docs/navigatr_sensors.md`.

## Build

```
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Needs a C++17 compiler and CMake 3.16+. gtest comes from the system when
installed, otherwise it is fetched. tinyxml2 is vendored in `third_party/`.

## Run

```
./build/navigatr config/navigatr.xml
./build/navigatr config/three_wheel_imu_no_landmark_correction.xml
./build/navigatr config/two_wheel_imu_no_landmark_correction.xml
./build/navigatr config/navigatr.xml --replay pico_uart=capture.bin --cycles 1000
```

The `*_no_landmark_correction.xml` configurations are the odometry bring-up
rigs: wheels plus the IMU heading constraint, everything else an explicit
noop. The `*_apriltag_landmark_correction.xml.in` files are intentionally
non-runnable calibration templates: every `@...@` token must be replaced
with a measured value (never zero to make parsing pass) and every
`calibration_status` raised before saving them as runnable `.xml`.
`--replay` swaps a declared resource id for a capture file through the same
decoders; `--allow-provisional` accepts `calibration_status="provisional"`
for bench use.

Everything here is C++. The XML files are configuration data, CMake is build
glue, and that is the whole list.
