# Component names

| Component | Status and responsibility |
|---|---|
| `spectaGATR` | Active: live pose viewer (laptop receiver, browser view, wire format). |
| `brain/spectaGATR` | Active: pose publisher library for PROS programs. |
| `brain/localization-demo` | Active: manual eight-motor mecanum drive, encoder and VEX IMU pose estimation, and pose publishing. |
| `encoder-record` | Active: encoder record format and codec shared across the demo. |
| `pi/encoder-relay` | Active: forwards Pico encoder records from UART to Brain USB. |
| `pico/aggreGATR` | Active: default `hat2_encoder_demo` encoder sender in `src/main.cpp`; optional IMU sensor monitors retained in `src/sensor_monitor/`. |
| `pcb/PiHat` | Deprecated Pi HAT board family; design history retained. |

The [localization demo](../brain/localization-demo/README.md) is the main
setup guide. The Pico counts encoder edges, the Pi forwards records, the Brain
computes pose with its IMU heading, and spectaGATR displays it. The controller
also commands the Brain's mecanum drive after startup calibration and an
encoder baseline. Configure motor ports and reversals first; the complete
route has not been tested on hardware.

In `pico/aggreGATR`, plain `pio run` and `pio run -t upload` select the encoder
sender. The `hat2_bno08x` and `hat2_asm330` sensor monitors require an explicit
environment selection.

The former `investiGATR` planning and `actuGATR` motor-control libraries,
old Brain test applications, `naviGATR` runtime and `communiGATR`/`translaGATR`
Pi link are retired; previous versions remain in Git history.
