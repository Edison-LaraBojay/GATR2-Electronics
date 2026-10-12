---
name: project-spectagatr-standalone
description: "2026-10-09 direction change - spectaGATR is a standalone pose visualizer (Brain publisher -> controller/USB -> laptop receiver -> browser); Pi system moved to deprecated/ as tomorrow's fallback"
metadata:
  node_type: memory
  type: project
  originSessionId: 827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b
  modified: 2026-10-10T07:17:54.003Z
---

On 2026-10-09 the user replaced the spectuGATR plan ([[project-spectugatr]], now superseded) with
**spectaGATR**, spelled exactly that way.

**The product:**
- The laptop owns the field and renders it.
- The Brain publishes an already-computed pose through `viz.updatePose(x, y, heading | Attitude{roll, pitch, yaw})`.
- The route is Brain -> controller radio -> USB -> laptop C++ receiver -> browser, or direct Brain USB.
- It carries no old debugger panels and no planning or motor control.

**Layout:**
- `spectaGATR/` holds the protocol, receiver, web, fields and docs.
- `brain/spectaGATR/` is the library; `brain/spectaGATR-demo/` is a PROS example that never moves motors.

**Wire format:** `SGP1 ...*CRC` text lines, mm and centidegrees, documented in spectaGATR/docs/PROTOCOL.md.

**Repo state:**
- Commit 20f20e2 (the user's "refactor") moved the Pi naviGATR, the old viewer, operaGATR/locaGATR/robot and the Pi-talking Pico code under `deprecated/`.
- The active Pico firmware is acquisition-only.
- The last pre-refactor known-good state is 10c568e: VEX IMU on port 20, forward encoder 1, sideways encoder 0, USB link.

**Why:** a demo on 2026-10-10. MCU localization and RS-485 are not ready, so the Pi/USB system must stay usable as the fallback.

**State at the end of 2026-10-09, all uncommitted:**
- Implemented and host-verified: protocol 9 tests, receiver 158, publisher 30, browser 27. The end-to-end replay passes. The PROS demo builds.
- Fallback: `deprecated/FALLBACK.md`. The pinned 10c568e route is recommended.
- DECIDED by the user on 2026-10-09: `deprecated/` stays local and gitignored (Codex did it with authorization), the PCB designs stay tracked under `pcb/`, and communiGATR and translaGATR are retired. The fallback is the pinned commit 10c568e. Tracked docs reference retired code by commit, not by `deprecated/` path. See [[feedback-codex-authorized]].

**2026-10-10 hardware finding (controller radio):**
- Measured about 230 B/s through the controller on the pit channel (about 3.3 Hz of 10). Poses arrived up to about 14 s old because the PROS and VEXos buffers filled.
- Fixed by session -62:
  - the publisher writes only while the VEXos FIFO holds 128 B or less (`vexSerialWriteFree`);
  - receiver additions: `--radio-download` (opt-in), `--user-read-length`, `--debug-link`.
- Bench, later on 2026-10-10:
  - The pit channel costs about 235 ms of dead time per USER_READ. Read length 0 gives 224 B replies, about 520 B/s and bursty 6-7 Hz.
  - The download channel (`--radio-download`) gives 10 Hz with no loss and 28-47 ms round trips. The user said "far better".
  - Default `--user-read-length` is now 0; `--pipeline` is experimental.
  - The gate self-calibrates its empty FIFO size and has not yet been uploaded to the Brain. Tests are spectaGATR 187.
- The tree changed: `brain/spectaGATR-demo` was removed in favour of `brain/localization-demo`, which has a `kControllerViewerTest` mode; investiGATR and actuGATR are retired. Tests are now spectaGATR 184 and brain 74 (30 of them Specta).

**How to apply:**
- Never claim that the controller polling and the Brain-Pi USB link coexist; that is untested.
- Keep the new code free of any build dependency on `deprecated/`.
- The handoff docs are in `specta-handoff/` in the project memory dir.
