# Memory Index

- [Project Overview](project_overview.md) — GATR2-Electronics: custom localization stack for UF VEX U robot; Pico->Pi->brain pipeline
- [Wire Format](project_wire_format.md) — the frame contract in translaGATR/frames.h (was common/); framing, fixed-point, mask-walking, generated frames.py
- [Navigatr Architecture](project_navigatr_architecture.md) — Pi pipeline framework: stage contract, typed maps, XML config, registry names; 08-05 vision/frames/targets build
- [Design Principles](feedback_design_principles.md) — extract-don't-invent interfaces, no speculative abstraction, degrade-don't-hang, narrow seam
- [Conventions](feedback_conventions.md) — C++17 warnings-as-errors, tests-next-to-code, explicit test lists, README/CI rules, navigatr C++-only
- [Comment Style](feedback_comment_style.md) — user's explicit rules: terse, self-describing, plain ASCII, no AI-ish overexplaining
- [Testing Strategy](project_testing_strategy.md) — tentative sim/test philosophy: seam-boundary injection, log/replay, synthetic vs real truth, eyeball-first
- [Current State](project_current_state.md) - 2026-09-14: localization/vision/viewer build in progress (workers, AprilTag, synthetic rig, inspection contract done; server/viewer/libcamera/field agents running); all uncommitted
- [investiGATR + brain link v3](project_investigatr_brain_link.md) - 2026-09-26: implemented (communiGATR rename done); review + fix phase in progress; user's own test is brain/locaGATR (was localization-test)

- [Parallel wheels + BNO08X](project_parallel_wheels_bno08x.md) - 2026-09-26 systems-test work: LateralMotion opt-in, A/B profiles, BNO08X firmware, raw preview; uncommitted; verified on live tree 342/342
- [Windows Build](reference_navigatr_windows_build.md) — navigatr build/test recipe on this box: ucrt64 first on PATH, MinGW Makefiles, 0xc0000139 fix
- [Brain/Pi architecture](project_brain_architecture.md) - DONE 2026-09-28: communiGATR/investiGATR/actuGATR, Brain RobotProfile, sensor-loss rule; final report in brain-arch-handoff/
- [Viz + diagnostics phase 1](project_viz_diagnostics.md) - 2026-09-28 inspect/2, capture, TELEMETRY, Pico diag done host-only; audit deferred until asked; -GATR renames
- [spectuGATR](project_spectugatr.md) - SUPERSEDED 2026-10-09 by spectaGATR standalone (next line)
- [spectaGATR standalone](project_spectagatr_standalone.md) - 2026-10-09 standalone pose viewer (SGP1 lines, receiver, browser); Pi fallback = pinned commit 10c568e; deprecated/ local-only
- [Codex is authorized](feedback_codex_authorized.md) - Codex restructures are user-authorized (local-only deprecated/, retired communiGATR/translaGATR); never undo them
- [Coding standard](project_coding_standard.md) - 2026-09-28 drafting GATR2 coding rules .md to apply system-wide; CI to be rebuilt around it
- [System philosophy](feedback_system_philosophy.md) - user's core rules: season-reusable pipelines, no over-generalization, standard I/O, readability over speed, struct/class/functional, PIPELINE.md
- [naviGATR restructure](project_navigatr_restructure.md) - 2026-09-28 approved layout; PAUSED 2026-09-29 by user (functionally same); WIP patch saved; naviGATR now under deprecated/
- [naviGATR MCU direction](project_navigatr_mcu_direction.md) - 2026-09-29 plan: naviGATR -> pure-localization MCU (RP2350 likely); Pi naviGATR kept later as naviGATR-v0 + tag; Phase B adapter minimal
- [Encoder localization demo](project_encoder_localization_demo.md) - 2026-10-09 Pico->Pi relay->Brain odometry->spectaGATR demo; host tested only, uncommitted
- [Controller radio link](project_controller_radio_link.md) - 2026-10-10 measured: pit channel ~3 updates/s, download channel clean 10 Hz; use --radio-download for demos

- [Push-everything handoff](project_push_everything_handoff.md) - 2026-10-12 laptop reset: push ALL local-only dirs per PUSH-EVERYTHING-HANDOFF.md; GATR2 section 1 is STAGED, user runs commit+push (classifier blocks it); pi/navigatr is empty; never secrets.local
