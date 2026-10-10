# Component names

| Component | Status and responsibility |
|---|---|
| `spectaGATR` | Active: live pose viewer (laptop receiver, browser view, wire format). |
| `brain/spectaGATR` | Active: pose publisher library for PROS programs. |
| `brain/spectaGATR-demo` | Active: example PROS program for the publisher; never moves motors. |
| `brain/localization-demo` | Active: encoder and VEX IMU pose estimation; never moves motors. |
| `encoder-record` | Active: encoder record format and codec shared across the demo. |
| `pi/encoder-relay` | Active: forwards Pico encoder records from UART to Brain USB. |
| `pico/aggreGATR` | Active: `hat2_encoder_demo` encoder firmware, with sensor acquisition environments retained. |
| `pcb/PiHat` | Deprecated Pi HAT board family; design history retained. |

The [localization demo](../brain/localization-demo/README.md) is the main
setup guide. The Pico counts encoder edges, the Pi forwards records, the Brain
computes pose with its IMU heading, and spectaGATR displays it. The complete
route has not been tested on hardware.

The former `investiGATR` planning and `actuGATR` motor-control libraries,
old Brain test applications, `naviGATR` runtime and `communiGATR`/`translaGATR`
Pi link are retired; previous versions remain in Git history.
