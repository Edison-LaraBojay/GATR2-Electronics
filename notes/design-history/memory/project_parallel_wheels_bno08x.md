---
name: project-parallel-wheels-bno08x
description: "2026-09-26 parallel-wheel + BNO08X systems-test work (session 827c2c12, \"d1\"/\"61\"): what landed, uncommitted, and what still waits on the brain_link session"
metadata:
  node_type: memory
  type: project
  originSessionId: 827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b
  modified: 2026-09-26T19:57:06.267Z
---

As of 2026-09-26 ~16:30, all UNCOMMITTED in the working tree (user said do not commit).

**Hardware facts (user-confirmed):** Pi HAT v2; two PARALLEL tracking wheels, both forward-measuring, on encoder channels 0 (J2) and 1 (J3); GY-BNO08X on SPI1 (SCK GP10, MISO GP8, MOSI GP11, CS GP9, INT GP22, RST GP12 and PS0/WAKE GP13 as added wires, PS1 to 3.3 V). Supersedes the old forward+lateral two-wheel description.

**Landed:**
- pico/: board.h (GATR2_HAT_REV) + IMU build selection (envs hat2_bno08x default, hat2_asm330); vendored CEVA sh2 v1.4.0 in pico/third_party with our non-blocking SPI HAL (imu_bno08x.cpp); uncalibrated gyro report 0x07 at 200 Hz averaged per 50 Hz frame (gyro_window), bno08x_supervisor recovery with backoff; pico/tests host gtests (26). Fixed a real bug: Serial1.availableForWrite() returns 0/1 on this core, so the old firmware never sent frames.
- navigatr: tracking_wheel_motion `<LateralMotion assume="zero"/>` opt-in (forward-only wheels, dx = mean(travel_i + y_i*dtheta), dy = 0, needs HeadingConstraint). Profiles override/diagnostics/parallel_wheels_bno08x{,_camera}.xml (.xml, no tokens) + shared robot template gatr2_parallel_wheels_bno08x.xml.in + shared token-free Localization fragment; no InitialPlacement (Brain supplies placement). Deleted the old two_wheel_imu* forward+lateral templates.
- Raw camera preview without detection: System::publishDetectionFrames publishes has_observations=false frames; inspection JSON gained has_observations/has_trace; viewer shows unplaced (RobotState.initialized false) robot grey, nominal landmark rows, no frame wait without cameras.
- Docs: pi/navigatr/docs/parallel_wheel_bringup.md plus localized edits.

**Verified on the live tree after peer "ca" went green (2026-09-26 ~16:40):** navigatr 342/342 (incl. my 16: ParallelWheels x11, ParallelWheelProfiles x5 with an end-to-end Brain HELLO/SET_POSE/SELECT test on profile A), investiGATR 68/68, pico both envs build + 26/26 host tests. Peer swept my pipelines to brain_link (CommandCollection + Publishing on brain_uart, loop 100 Hz). Inspection robot section now exports placement_origin/session/sequence.

**Not verifiable here:** headless-browser viewer smoke (Chrome and Edge both time out on this box, baseline identical); anything on hardware. communiGATR (Brain driver, renamed from navigatr-interface) was mid-rename and not built by me.

**Superseded outside this session (~16:57, apparently a Codex session run by the user):** the BNO08X driver now enables accel 0x01 + uncalibrated gyro XYZ and projects gyro onto the gravity-up axis learned during a level, stationary startup (gravity_alignment.*, bno08x_reports.*), so mounting orientation doesn't matter. The robot template now fixes CPR at 4000 and IMU invert false, and the bring-up guide was rewritten to match. Re-verified at 17:25: pico both envs build, 53/53 host tests, navigatr 351/351.

See [[feedback-conventions]], [[reference-navigatr-windows-build]].
