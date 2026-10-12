# naviGATR on a microcontroller: planning notes

Local only; never committed (excluded through `.git/info/exclude`).
Status: preliminary. Nothing changes until the PCB and hardware exist. The Pi naviGATR stays as it is for testing.

## Why
- Slim the robot down. No Linux, no SD card, no Pi power budget. Instant on, fewer parts, cheaper.
- Lower latency: acquisition and estimation run on one chip, one hop fewer before the Brain.
- The core question is "where am I". "Where are things around me" (field correction) may be too much for what we need.
- The two questions are separate systems. They are coupled only when relative observations are turned into field positions through the pose.

## Target shape
- naviGATR becomes pure-localization firmware on a compact microcontroller, cheaper than a Pi 4 and Pico-sized. It:
  - reads the encoders and the IMU;
  - estimates the pose;
  - answers the Brain over RS-485.
- aggreGATR's acquisition folds into it.
- Config still comes from the Brain (the robot profile), so there is no XML on the chip.
- The Brain-side link code does not change: the Brain does not care who answers GET_STATE.
- Visualization: spectuGATR in Brain-only mode, since the Brain owns the robot scene. The chip's diagnostics reach the viewer through the Brain as named channels (`viz.sample(...)`).
- Field correction, if ever wanted again, becomes an optional separate component (estimaGATR, "estimator"). It would take the pose from naviGATR and send landmark errors (dx, dy, dtheta) to the Brain.

## Chip options (prices rough, from memory; check before buying)

| Option | Core | Float | RAM | Encoders | Rough cost | Notes |
|---|---|---|---|---|---|---|
| RP2350 (Pico 2) | 2x Cortex-M33, 150 MHz | single in hardware, doubles accelerated | 520 KB | PIO (what aggreGATR uses now) | ~$5 board, ~$1 chip | Same footprint and pinout as the Pico; firmware and PCB carry over |
| STM32G4 / STM32H7 | M4F 170 MHz / M7 480 MHz | single / double | 128 KB / 1 MB | hardware encoder timers | ~$5-20 | Best peripherals (hardware RS-485 DE); heavier toolchain |
| Teensy 4.0 | Cortex-M7, 600 MHz | double | 1 MB | 4 hardware quadrature decoders | ~$25-30 | Huge headroom, tiny module; hard to put the chip down |
| ESP32-S3 | 2x LX7, 240 MHz | single | 512 KB | pulse counters | ~$5-10 | Has WiFi (practice only; likely not match-legal) |

Recommendation: RP2350.
- It fixes the RP2040's missing floating-point unit (the Pi math is double-heavy).
- Encoder PIO, IMU drivers and the translaGATR codec carry over.
- It is pin-compatible if we keep a module, and the chip can go down directly on a custom PCB.
- Compute is not the limit: odometry plus fusion is a few hundred float operations per update.

## What ports from the Pi code (sizes at ae457ee)
- Localization math: about 4,200 lines.
- Brain link server (commands, publishing): about 2,100 lines.
- Profile, calibration, sensor loss and stationary precheck: about 1,200 lines.
- Pose history: about 700 lines. Only needed if something aligns camera frames in time.
- The Pi code is desktop-style C++ (registries, variants, heap), so this is an embedded rewrite reusing the algorithms. Estimate: 3-4k lines of firmware.
- After the restructure, the Pi pipeline folders are the reference to port from.

## Hardware and PCB implications
- The Brain link runs RS-485 through a smart port, so the RS-485 transceiver must be routed to the MCU's UART.
  - The Brain's USB port is a USB device, so the MCU would have to be a USB host. That is possible but fiddly, so it is not the plan.
- The MCU can be a module or placed directly on the board (the most compact option).
- IMU on SPI; encoders on PIO or timer inputs.

## Risks
- Rewrite effort, and slower iteration than on Linux. Host tests still work, as aggreGATR's do today.
- Less debug visibility: no Pi-side capture. Diagnostics go through the Brain, or through the MCU's own USB during bench work.
- Field correction is gone unless estimaGATR comes back.

## Decision gate: is localization alone enough?
Run this with the current stack before committing to anything:
1. On the Pi, set world estimation to `noop` (the config already supports it).
2. Run the typical autonomous routes several times.
3. Tape-measure the final pose error against the intended end pose.
4. Record the drift against time and distance.
5. Compare the error with the tolerances that matter for scoring.

If the error is inside tolerance, field correction can go. Cheap middle ground: wall resets with ToF or distance sensors. That is localization corrected against known walls, not world estimation, and it could run on the MCU too.

## Keeping the current naviGATR (when the switch happens)
- `git mv pi/naviGATR pi/naviGATR-v0` keeps the history with no duplicate files.
- A git tag `navigatr-v0` on that commit is an exact, buildable snapshot forever.
- `pi/naviGATR-v0/README.md`: DEPRECATED, frozen at tag `navigatr-v0`, how to build it, and how to revive world estimation.
- Excluded from CI, so shared changes (e.g. translaGATR) do not force maintaining it.
- To revive field correction:
  - the restructure puts it all in `pipeline/world_estimation/` behind one contract (sensors plus pose history in, `LandmarkError` out);
  - AprilTag lives in `estimators/apriltag/`;
  - lift that folder into a standalone estimaGATR fed by the MCU's poses.

## Nice to have: FPV camera and recorded runs (not mandatory)
- No processing; just a first-person view to watch and record runs.
- Keep it separate from the localization MCU. Video and localization are independent, and an RP2350-class chip cannot do useful video.
- Options:
  - an ESP32-S3 camera board (~$10-15, JPEG straight from the sensor);
  - a Pi Zero 2 W with a camera (~$15 plus the camera, hardware H.264).
  - Either records to its own microSD with timestamps.
- The VEX radio cannot carry video (hundreds of bytes per second). Live FPV needs WiFi, which is fine in practice and likely not match-legal; check the manual. Recording locally sidesteps it.
- Recording runs: spectuGATR capture records the telemetry and the camera records video. Replay plays both on one timeline, lined up by one shared sync mark (e.g. the Brain's match-start event plus a visible LED blink).
- Live in practice: a small receiver "camera source" could feed the viewer's existing camera panel.

## Relation to the work running now
- The naviGATR restructure is still worth finishing: it makes the Pi code readable as the porting reference and isolates world estimation for a possible revival.
- spectuGATR is unaffected. The Brain side works with or without a Pi, and Brain-only mode is already supported.
- Open: the spectuGATR Phase B naviGATR adapter (Pi instrumentation, relaying the Brain's lines through the Pi). Minimal or complete while the Pi is the test rig? Leaning minimal.

## Open questions
- Chip final pick, and module vs chip-down.
- PCB plan: RS-485 to the MCU, IMU and encoder routing, power.
- The result of the localization-only accuracy test.
- Whether estimaGATR is ever needed, and where it would run.
- Whether to add ToF wall resets to the MCU localization.
