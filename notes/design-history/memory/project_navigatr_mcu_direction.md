---
name: project-navigatr-mcu-direction
description: "2026-09-29 direction: naviGATR to become pure-localization microcontroller firmware; current Pi naviGATR (with field correction) kept later as deprecated naviGATR-v0"
metadata:
  node_type: memory
  type: project
  originSessionId: 827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b
  modified: 2026-10-09T23:43:01.269Z
---

The user's direction (2026-09-29). This is not started; the Pi naviGATR stays as-is for testing now.

**Goal: slim down.**
- naviGATR becomes pure localization on a compact microcontroller: cheaper than a Pi 4, Pico-sized. It acquires encoders and IMU, estimates the pose, and talks RS-485 to the Brain. aggreGATR's acquisition folds into it.
- Field correction (camera/AprilTag world estimation) may be dropped as "too much".
- Chip discussed: RP2350 (Pico 2) is recommended. It has hardware float, 520 KB of RAM, the same footprint as the Pico, and PIO encoders. Alternatives: STM32G4/H7, Teensy 4.0.
- Before committing: measure localization-only accuracy on the real robot with world estimation set to `noop`.

**Preserving the old code.** When the switch happens:
- `git mv pi/naviGATR pi/naviGATR-v0` (the user's words: "naviGATR-deprecated ... version as is 0"), plus a git tag `navigatr-v0` and a DEPRECATED README;
- v0 is excluded from CI.
- The field correction to revive later is `pipeline/world_estimation/` (a clean contract after the restructure). It could come back as a standalone estimaGATR that takes poses from the microcontroller naviGATR.

**Why:** fewer parts, instant on, lower latency. The Brain already owns the config (the robot profile) and the visualization (spectuGATR), so the Brain side does not change.

**How to apply:**
- Don't invest in Pi-only naviGATR features beyond what testing needs.
- The spectuGATR Phase B naviGATR adapter should be minimal or skipped (ask the user). A microcontroller naviGATR can send diagnostics through the Brain as named channels.

Related: [[project-navigatr-restructure]], [[project-spectugatr]], [[feedback-system-philosophy]]

**2026-10-09 update:** the "naviGATR-v0 + tag" plan is superseded. The Pi naviGATR, communiGATR and translaGATR are retired; they live in Git history (pinned fallback commit 10c568e) plus a local-only, gitignored `deprecated/`. See [[feedback-codex-authorized]].

**2026-09-29 addendum:**
- All of this is preliminary. It needs PCB changes and hardware the user does not have, so make NO changes now.
- Nice-to-have (not mandatory): an FPV camera view in the visualizer and recorded runs, with no processing.
  - Plan: keep it separate from the localization MCU (a tiny camera board such as an ESP32-S3 cam or a Pi Zero 2 W that records locally).
  - spectuGATR replay would sync the video to the captured telemetry through a shared sync mark.
  - An optional receiver "camera source" could feed the existing camera panel live in practice (WiFi; likely not match-legal).
