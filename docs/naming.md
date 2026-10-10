# Component names

| Component | Status and responsibility |
|---|---|
| `spectaGATR` | Active: live pose viewer (laptop receiver, browser view, wire format). |
| `brain/spectaGATR` | Active: pose publisher library for PROS programs. |
| `brain/spectaGATR-demo` | Active: example PROS program for the publisher; never moves motors. |
| `brain/investiGATR` | Active: planning and shared pose, field and path types. |
| `brain/actuGATR` | Active: following and motor control. |
| `pico/aggreGATR` | Retained acquisition firmware, pending rework. |
| `pcb/PiHat` | Deprecated Pi HAT board family; design history retained. |

The new MCU localization implementation is not named or implemented. The target
split is MCU pose estimation, Brain-owned field/planning and Brain motor control.
Camera work is deferred.

The retired naviGATR runtime, communiGATR/translaGATR Pi link and associated
applications are not included in the active tree; previous versions remain
in Git history.
