---
name: project-overview
description: "GATR2-Electronics — custom localization stack for a UF VEX U robot; three-device pipeline, wire-contract-driven design"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0693582d-d417-490c-8cb4-1a0c07d0b810
  modified: 2026-07-19T04:59:07.926Z
---

GATR2-Electronics is the custom electronics/embedded stack for GATR2, a University of Florida VEX U robot (Override season, 2026-2027). Owned by the electronics sub-team. Replaces stock V5 odometry with a custom localization system.

**Pipeline (narrow seams):** sensors -> Pico (RP2040) -> Pi 4 -> V5 brain -> motors

- **Pico (RP2040):** hard real-time raw sensing. Reads 3x AS5047P magnetic encoders (ABI quadrature, 4000 CPR) + eventually an IMU. Timestamps on its own clock (the fusion time reference). Ships sensor frames over UART to Pi at 50 Hz. Deliberately dumb — no float pose math, no fusion, raw gyro rate on the wire. PlatformIO, earlephilhower core.
- **Pi 4:** soft real-time vision + fusion. Pi OS Bookworm, IMX296 global-shutter mono camera over CSI. AprilTag detection (tagCircle21h7 this season) + EKF fusion (3-6 state, 50 Hz). Produces pose frames, sends to brain over RS-485 (one-way, DE held high). Python (chosen for vision iteration speed).
- **V5 brain:** behavior, the master. PROS library named Chomp (namespace `chomp::`, headers `brain/include/chomp/`). Treats Pi as untrusted/possibly-absent sensor; control loop never blocks. Robot must stay fully drivable with Pi dead (non-negotiable). Eventually distributed as a PROS template zip.

**Responsibility boundaries:** Electronics owns "where am I", programming owns "what to do". Chomp knows tags + field geometry, ZERO game knowledge (words "goal"/"score" in Chomp = boundary violation). No visual odometry — encoders/gyro own short-term deltas, vision gives absolute corrections. Nothing fused in two places.

**Longer-term:** Chomp may be open-sourced (MIT) as a generic VEX coprocessor-localization library. Keep brain lib dependent only on the wire contract, never Pi internals. Keep Chomp free of PROS template deps (no LemLib, no EZ-Template).

Note: user said this overview is a generic summary and some things are subject to change. See [[project-wire-format]], [[feedback-design-principles]], [[project-current-state]].
