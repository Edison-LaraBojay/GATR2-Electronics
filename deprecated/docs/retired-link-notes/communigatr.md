# communiGATR

Legacy Brain-side client for the retired Pi server, retained pending redesign.
It is portable C++ around a transport abstraction; its application protocol is
still specific to the previous naviGATR system. It does not implement a new
Pico-to-Brain connection.

## Existing layers

- `BytePort`: byte I/O interface; PROS USB and RS-485 implementations live in
  `brain/communiGATR/pros/`.
- `Client`: legacy sessions, request/reply, profile and field synchronization,
  calibration/recovery, and telemetry messages.
- `LinkDriver`: converts the legacy protocol into SI units and source status.
- `ProsLink`: PROS tasks and transport management around the client.
- `translaGATR`: the formats and codecs used by that client.

`LinkDriver` and `ProsLink` implement `investigatr::StateSource` for robot pose
and `investigatr::FieldSource` separately for their historical field transfer.
The future MCU source will only need the pose interface; the Brain supplies
its field independently. Generic path observation remains optional.

## Build status

The retained host tests use a fake Pi; they do not establish a working new MCU
link. Build them with the other Brain libraries:

```sh
cmake -S brain -B build-brain
cmake --build build-brain --parallel
ctest --test-dir build-brain --output-on-failure
```

To build/test only planning and control, configure with
`-DGATR2_BUILD_COMMUNIGATR=OFF`. A PROS project can select libraries through
[gatr2_brain.mk](../brain/gatr2_brain.mk).

Historical deployment examples use the retired combined pose/field source
interface and old applications; they require adaptation.
See [wire formats](interfaces.md), [planning](investigatr.md), and
[control](actugatr.md).
