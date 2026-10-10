# GATR2 Electronics

Sensor electronics, microcontroller acquisition firmware, and portable C++
planning and drive-control libraries for a VEX robot.

The active direction is a microcontroller supplying robot pose to the Brain.
The Brain owns field geometry, path planning and motor control. Replacement
MCU localization and communication are not implemented yet; there is currently
no supported end-to-end robot application.

## Active components

| Component | Purpose |
|---|---|
| [spectaGATR](spectaGATR/README.md) | Live pose viewer: Brain publisher library, laptop receiver (controller or Brain USB), browser field view. |
| [investiGATR](docs/investigatr.md) | Geometry, field models, direct and obstacle-avoiding planning; separate pose and field inputs. |
| [actuGATR](docs/actugatr.md) | Following, PID control, tank and mecanum drive adapters. |
| [aggreGATR](pico/aggreGATR/README.md) | RP2040 encoder and IMU acquisition, retained for firmware rework. |
| [IMU boards](pcb/IMU/README.md) | Inertial sensor board designs. |
| [Magnetic encoder boards](pcb/MagneticEncoder/README.md) | Tracking-wheel encoder designs. |

## Host build

```sh
cmake -S brain -B build-brain
cmake --build build-brain --parallel
ctest --test-dir build-brain --output-on-failure
```

Firmware builds are documented in [pico/aggreGATR](pico/aggreGATR/README.md).
Host simulation establishes software behavior, not hardware readiness.

## Retired system

The retired Pi runtime, its viewer, the former robot test applications, and
the communiGATR/translaGATR Pi link are not distributed in the active tree.
Their previous versions remain in Git history. The deprecated
[Pi HAT designs](pcb/PiHat/README.md) stay tracked with the other PCB families.

See [documentation](docs/README.md), [PCB overview](pcb/README.md), and
[contribution conventions](CONTRIBUTING.md).
