# GATR2 Electronics

Sensor electronics and an encoder localization demo for a VEX robot.

The active path is tracking wheels -> Pico encoder firmware -> Pi relay ->
Brain localization demo -> spectaGATR. The Brain combines encoder counts with
its VEX IMU heading to compute the pose. The demo never commands motors;
push the robot by hand.

Start with the [localization demo guide](brain/localization-demo/README.md)
for wiring, builds, calibration and visualization. The software builds and
host tests pass; the complete route has not been tested on hardware.

## Active components

| Component | Purpose |
|---|---|
| [Localization demo](brain/localization-demo/README.md) | Brain odometry from two tracking wheels and the VEX IMU. Main setup guide. |
| [Encoder record](encoder-record/README.md) | Shared encoder record format and codec. |
| [Pico firmware](pico/aggreGATR/README.md) | `hat2_encoder_demo` sends encoder counts; sensor acquisition environments are also retained. |
| [Pi encoder relay](pi/encoder-relay/README.md) | Forwards Pico UART records to the Brain USB user port. |
| [spectaGATR](spectaGATR/README.md) | Brain pose publisher, laptop receiver and browser field view. |
| [IMU boards](pcb/IMU/README.md) | Inertial sensor board designs. |
| [Magnetic encoder boards](pcb/MagneticEncoder/README.md) | Tracking-wheel encoder designs. |

## Brain host build

```sh
cmake -S brain -B build-brain
cmake --build build-brain --parallel
ctest --test-dir build-brain --output-on-failure
```

The [localization demo guide](brain/localization-demo/README.md) covers its
firmware builds and host tests for the encoder codec, odometry and relay.

## Retired system

The former planning and motor-control libraries, old Brain test applications,
and old Pi runtime and link are retired. Their previous versions remain in
Git history. PCB design history stays tracked with the hardware.

See [documentation](docs/README.md), [PCB overview](pcb/README.md), and
[contribution conventions](CONTRIBUTING.md).
