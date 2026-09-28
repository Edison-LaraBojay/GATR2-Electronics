# Component names

The GATR2 software components carry "-GATR" names. Each is a real English
agent noun ending in "-ator" that says what the component does:
navigator, investigator, communicator, actuator. A folder gets such a name
only when it is a component with one job and the word fits. Descriptions
(`brain/robot`), resources (`models/`), references (`docs/`), and hardware
(`pcb/`) keep plain names.

| Folder | Namesake | What it does |
|---|---|---|
| `pi/naviGATR/` | navigator | Pi runtime: fuses the Pico encoders and the IMUs into the robot pose, estimates field objects, answers the Brain. It finds where the robot is. |
| `pi/naviGATR/spectaGATR/` | spectator | Browser viewer served by naviGATR. It watches the robot live, and on replay, but never commands it. |
| `brain/investiGATR/` | investigator | Brain planner. It works out a collision-free route through the field before the robot moves. |
| `brain/communiGATR/` | communicator | Brain side of the Pi link: sessions, robot profile, field transfer, placement, control. |
| `brain/actuGATR/` | actuator | Brain movement: path following, tank and mecanum drives, the drive task. It turns plans into motor commands. |
| `brain/locaGATR/` | locator | Motor-free bench program. It places the robot and shows where naviGATR locates it, with calibration and recovery pages. |
| `brain/operaGATR/` | operator | Operator-control drive program (PROS `opcontrol`): direct, avoiding and landmark tests, manual driving and tuning. |
| `pico/aggreGATR/` | aggregator | RP2040 firmware. It gathers the encoder counts and IMU readings into timed frames for the Pi. |
| `translaGATR/` | translator | Shared wire codec compiled by the Brain, Pi and Pico. It translates between in-memory structs and the bytes every link carries. Namespace `translagatr` (was `gatr2`). |

## Renames on 2026-09-28

| Before | After |
|---|---|
| `pi/navigatr/` | `pi/naviGATR/` (capitalized like the other components) |
| `pi/navigatr/viewer/` | `pi/naviGATR/spectaGATR/` |
| `common/` | `translaGATR/` |
| namespace `gatr2` (wire codec) | namespace `translagatr` |
| `brain/localization-test/` | `brain/locaGATR/` |
| `brain/testing/` | `brain/operaGATR/` |
| `pico/` (PlatformIO project) | `pico/aggreGATR/` (`pico/` is now the platform folder, like `brain/` and `pi/`) |

Unchanged on purpose:
- **Namespaces `navigatr`, `investigatr`, `communigatr`, `actugatr`:** already match their folders.
- **`gatr2_robot`:** the robot description, not a component.
- **`GATR2_*` macros and the `gatr2_brain.mk` fragment:** project-wide, not one component.
- **Executable and target names (`navigatr`, `navigatr_core`, test targets):** so service files, scripts and build commands keep working.
- **Browser URLs:** the viewer is still served at `/`.
- **`bench/`:** mixed host bring-up utilities, not one component.

## After pulling the rename

- **Build directories.** CMake and PlatformIO build folders made before the rename point at old paths: reconfigure them, or delete and rebuild. The in-repo `build*` folders are yours and were left alone.
- **PROS programs.** Build them from `brain/locaGATR` and `brain/operaGATR`. The upload name is now `locaGATR` / `operaGATR` (see [Brain setup](brain_setup.md)); remove the old slots on the Brain if you like.
- **Pico.** Open `pico/aggreGATR` as the PlatformIO project. `cd pico/aggreGATR`, then `pio run`.
- **Pi.** After copying the source, the runtime lives in `pi/naviGATR`. Linux paths are case sensitive: update any shell history, scripts or service `WorkingDirectory` that still say `pi/navigatr`, then reinstall the service with `tools/install_service.sh`.
- **git on Windows.** `pi/navigatr` to `pi/naviGATR` is a case-only rename. With `core.ignorecase` git does not see it on its own. Record it with:
  ```
  git mv pi/navigatr pi/naviGATR_tmp
  git mv pi/naviGATR_tmp pi/naviGATR
  ```
  Close any terminal or program whose current folder is inside it first.
