# Deprecated system

This is a local-only source and architecture reference, ignored by Git. Keep
its contents when cleaning the repository. PCB designs remain tracked under
[pcb/PiHat](../pcb/PiHat/README.md).

These components are retired until further notice. They are kept as source,
hardware and test-history references, and are excluded from active builds and CI.

The Pi/USB system (Pico -> Pi naviGATR -> Brain over USB, with the old Pi viewer)
is the demonstration fallback. [FALLBACK.md](FALLBACK.md) has the exact procedure and
shows what has been verified.

- The Pi runtime and the Brain apps `operaGATR` and `locaGATR` depended on
  communiGATR and translaGATR, which are now archived here too. Their former
  in-place build instructions no longer apply. Use the pinned historical
  checkout in FALLBACK.md for a complete fallback build.
- The Pi-talking Pico firmware does not build here. Build it from commit `10c568e`.
- Other archived instructions, such as the docs under `pi/naviGATR/docs` and `docs/`,
  still use the former paths, such as `pi/naviGATR` and `brain/operaGATR`.

| Location | Archived material |
|---|---|
| [FALLBACK.md](FALLBACK.md) | Procedure for the Pi/USB fallback. |
| [pi/naviGATR](pi/naviGATR/README.md) | Entire Pi runtime, XML, tools, tests and docs. |
| [spectaGATR](pi/naviGATR/spectaGATR/) | Old browser viewer, compiled into the Pi runtime. It is not the new top-level spectaGATR. |
| [brain/operaGATR](brain/operaGATR/README.md) | Former driving test app; the fallback Brain app. |
| [brain/locaGATR](brain/locaGATR/README.md) | Former localization/calibration test app. |
| [brain/robot](brain/robot/README.md) | Former robot profile and temporary calibration. |
| [brain/actuGATR](brain/actuGATR/README.md) | Retired path following, drive control, motor adapters and tests. |
| [brain/investiGATR](brain/investiGATR/README.md) | Retired geometry, field and path planner implementation and tests. |
| [brain/actuGATR-telemetry](brain/actuGATR-telemetry/) | Earlier Pi-viewer telemetry adapter and tests. |
| [brain/communiGATR](brain/communiGATR/README.md) | Retired Pi-link client, USB/RS-485 adapters and tests. |
| [translaGATR](translaGATR/README.md) | Retired wire formats, codecs and tests. |
| [pico/aggreGATR](pico/aggreGATR/) | Retired Pico-to-Pi communication code (partial snapshot). |
| [bench](bench/README.md) | Former Pi/Brain RS-485 bring-up tools. |
| [docs](docs/) | Former setup and architecture references. |

Active acquisition is in [pico/aggreGATR](../pico/aggreGATR/README.md). The active
Brain code is the pose publisher and the motor-free localization demo under
[brain](../brain/README.md).
The current encoder demo uses its own record format, independently of the
archived communication libraries. A replacement MCU-localization link is not
implemented by this archive.
