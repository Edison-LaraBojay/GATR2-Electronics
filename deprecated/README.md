# Deprecated system

These components are retired until further notice. They are retained as source,
hardware and test-history references, excluded from active builds and CI.
Historical instructions and code may depend on the former repository layout;
this directory is not maintained as a standalone build.

| Location | Archived material |
|---|---|
| [pi/naviGATR](pi/naviGATR/README.md) | Entire Pi runtime, XML, tools, tests and docs. |
| [spectaGATR](pi/naviGATR/spectaGATR/) | Browser viewer/assets, also called spectuGATR. |
| [brain/operaGATR](brain/operaGATR/README.md) | Former driving test app. |
| [brain/locaGATR](brain/locaGATR/README.md) | Former localization/calibration test app. |
| [brain/robot](brain/robot/README.md) | Former robot profile and temporary calibration. |
| [brain/actuGATR](brain/actuGATR/) | Retired Pi-viewer telemetry adapter and tests. |
| [pico/aggreGATR](pico/aggreGATR/) | Retired Pico-to-Pi communication code. |
| [pcb/PiHat](pcb/PiHat/README.md) | Pi HAT iterations and KiCad projects/libraries. |
| [bench](bench/README.md) | Former Pi/Brain RS-485 bring-up tools. |
| [docs](docs/) | Former setup and architecture references. |

Active acquisition is at [pico/aggreGATR](../pico/aggreGATR/README.md); active
planning/control libraries are under [brain](../brain/README.md).
communiGATR and translaGATR remain outside this directory as legacy code pending
their redesign. New Pico-to-Brain localization is not implemented.
