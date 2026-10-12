---
name: project-spectugatr
description: "2026-09-28 spectuGATR (viz product) replaces spectaGATR scope; Brain owns core scene (pose/field/path), Pi only instrumentation; design proposal first, no code until approved"
metadata:
  node_type: memory
  type: project
  originSessionId: 827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b
  modified: 2026-09-28T16:09:04.108Z
---

2026-09-28: the user sent a complete replacement spec (verbatim at `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/spectu-handoff/requirements.md`).

- spectuGATR = the whole viz product: shared browser viewer (spectaGATR migrates into it), portable C++ receiver (Win/mac/Linux, CMake, prebuilt packages), shared contracts, Brain publisher lib `brain/spectuGATR`.
- **Ownership flip:** the Brain owns the core scene (pose incl. attitude, field it uses, paths/targets, execution). naviGATR only adds Pi instrumentation (UART, raw sensors, pins, calibration, estimator, camera).
- Receiver modes: Brain only, Brain + naviGATR, naviGATR only. naviGATR-only must never substitute a Pi pose.
- The standalone route is Brain -> VEX radio -> controller USB -> laptop receiver.
- The Brain API is `viz.updatePose(x, y, heading | Attitude{roll, pitch, yaw})`, `updateField`, `updatePath`, `sample`, with no central polling coordinator.

**Order:**
1. Design proposal only (inventory, ownership diagram, topologies, answers on wireless/coexistence, rate/budget, latency, field transfer, name, API, evidence, plan, decisions).
2. After the user approves: implement, validate, then the whole-system read-only audit (report only).

**Why:** the user wants Brain-authoritative viz usable without a Pi. Their concerns: link budget (TELEMETRY already costs 6-20% of state polls in host sims), the extra round trip, and USB stream ownership (communiGATR owns stdin, `/ser/ngtr`, COBS disabled globally).

**How to apply:** no repo edits until approval. Label evidence as host-sim vs hardware. If Brain authority conflicts with the latency target, say so explicitly; never silently restore Pi-authoritative pose.

Related: [[project-viz-diagnostics]], [[project-brain-architecture]]

**Status 2026-09-28 ~15:50:**
- The design proposal (spectu-handoff/PROPOSAL.md) is APPROVED with all recommended defaults.
- Phase A (protocol, Brain publisher, integrations + D9 fix, host platform, receiver core, bench kit) runs as workflow wf_e70fce5a-ce5 in worktree `<scratchpad>/wt-spectu`. It produces spectu-handoff/phaseA.patch and REPORT_A.md.
- Phase B (viewer move, server extraction, naviGATR adapter, receiver main, viewer multi-source, packaging) waits for the naviGATR restructure patch. Then validation, then the whole-system read-only audit.
