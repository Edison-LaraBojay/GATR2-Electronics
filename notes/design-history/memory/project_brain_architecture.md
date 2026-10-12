---
name: project-brain-architecture
description: "2026-09-27 integrated Brain/Pi architecture + calibration/recovery addition (communiGATR/investiGATR/actuGATR, Brain RobotProfile, field docs, v4 link, Pico identity/control); lead session 827c2c12; handoff dir brain-arch-handoff"
metadata:
  node_type: memory
  type: project
  originSessionId: 827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b
  modified: 2026-09-28T06:56:06.239Z
---

Two user requests, both 2026-09-27, copied verbatim to `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/brain-arch-handoff/`:
- `requirements.md`: the architecture.
- `requirements_2.md`: calibration, per-wheel tuning and recovery.

Handoff folder contents: `spec.md` is the contract; section 8.x holds binding amendments, 8.7 the latest. Also `inventory/` (the Phase A reports), `progress/wA..wD.md` (per worker), `stage_results/` (finished stage reports) and `phase_c.md` (lead's open items).

**Done (lead):**
- common/ v4 contract, plus the 8.2 contract: sensor frame v2, Pico command 0x12 and status 0x13, READ_WHEELS, and a 32-byte profile with gear and travel_scale.
- investiGATR shared types.
- brain/actuGATR complete: 79 host tests including trajectory clearance with the real planner.
- communiGATR wheel_calibration and startup_placement: 12 tests, built only in the lead harness `$S/lead-harness` (`$S/lead-build`).
- `brain/robot/gatr2_robot.h` (shared profile), `brain/testing/include/robot_config.h`, gatr2_brain.mk, docs/actugatr.md.
- PicoTelemetry now implements PicoControl (minimal); `pi/navigatr/src/resources/pico_control.h` is the contract.

**Worker stages:**
- Finished in the first workflow wf_e8d66a45-c7c, which stopped when the session ended: A1, A2, B1, B2, C1, C2.
- Relaunched as wf_86d9db27-ed6; the computer powered off twice (a ventilation problem, per the user) and it was relaunched as wf_71552465-ffb, which hit the account session limit (HTTP 429), and then again as wf_bd73f72e-259 (task wzuyyh92n, script scratchpad/lead/phase_b2.js), three branches. The user wants full build parallelism, no -j limits:
  - A3 (PROS ProsLink, common tests update), then A4 docs.
  - B3 (profile application), B4 (calibration/recovery), B5 (configs/inspection/docs).
  - C review and fix, then D1 (Pico firmware), D2 (Pi Pico link).
  - Each branch ends with a review and a fix pass.

**Phase C status 2026-09-27 21:50:** lead review findings fixed (DriveRequests mailbox in ProsDrive, calibration rotation lever, LinkEvents gaps); both PROS apps build via toolchain make.exe; v4 e2e test written (6/8, the 2 failures are Pi items handed to B4 as spec 8.8); start pose placeholder moved to (1.2, 1.8) because (0.6, 0.6) is a closed planner pocket. Details in phase_c.md.

**DONE 2026-09-28 ~03:00.** Final gate: brain 410, common 221, pi 532 (incl. brain_link_e2e_tests), pico 110, pio both envs, both PROS apps link; all 0 warnings. Final report: brain-arch-handoff/final_report.md. User rules adopted: a used sensor stale over 250 ms or restarted invalidates the pose (spec 8.10), no Pico CONFIGURE (8.9), viewer additions kept. Their external IMU is broken; the VEX IMU over USB is the working setup. Nothing committed by the lead (the user committed d4bc875 and f657cdf themselves), nothing flashed.

**Lead to do (Phase C, historical):**
- Both Brain apps: localization-test with calibration and recovery UI; testing with Motion and the tests.
- A v4 brain_link_e2e test.
- PROS builds of both apps.
- docs/brain_setup.md.
- Review of the lead's own code (the first reviewer stalled).
- Final report.

**Key facts:**
- PROS has no gthreads: no shared_ptr sharing across tasks and no std::mutex; use bounded pros::Mutex takes.
- The circle planner cannot pass between diagonal Override goals for an 18 in robot.
- Constraints: no commit, flash, deploy or PCB changes; -Wall -Wextra with no -Werror.

See [[project-investigatr-brain-link]], [[reference-navigatr-windows-build]].
