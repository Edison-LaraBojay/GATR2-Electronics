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
./build/navigatr config/tracking_only.xml
./build/navigatr config/navigatr.xml --replay pico_uart=capture.bin --cycles 1000
```

`tracking_only.xml` is the bench rig: two wheels plus IMU heading, everything
else an explicit noop. `--replay` swaps a declared resource id for a capture
file through the same decoders.

Everything here is C++. The XML files are configuration data, CMake is build
glue, and that is the whole list.
