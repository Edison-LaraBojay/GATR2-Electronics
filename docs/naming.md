# Component names

| Component | Status and responsibility |
|---|---|
| `brain/investiGATR` | Active: planning and shared pose, field and path types. |
| `brain/actuGATR` | Active: following and motor control. |
| `pico/aggreGATR` | Retained acquisition firmware, pending rework. |
| `brain/communiGATR` | Legacy Pi-link client and transports, pending redesign. |
| `translaGATR` | Legacy packet formats, framing and codecs used by communiGATR. |
| `deprecated/pi/naviGATR` | Retired Pi localization and world estimation. |
| `deprecated/pi/naviGATR/spectaGATR` | Retired viewer, also called spectuGATR. |
| `deprecated/brain/operaGATR` | Retired driving test app. |
| `deprecated/brain/locaGATR` | Retired localization/calibration test app. |
| `deprecated/pcb/PiHat` | Historical Pi HAT board family. |

The new MCU localization implementation is not named or implemented. The target
split is MCU pose estimation, Brain-owned field/planning and Brain motor control.
Viewer and camera work are deferred.
