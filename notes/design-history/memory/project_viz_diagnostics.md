---
name: project-viz-diagnostics
description: "2026-09-28 visualizer + diagnostics phase 1 (inspect/2, capture, instrumentation, TELEMETRY, Pico diag) done host-only; audit phase 2 deferred until user asks; -GATR renames"
metadata:
  node_type: memory
  type: project
  originSessionId: 827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b
  modified: 2026-09-28T11:14:38.438Z
---

Phase 1 (visualizer/diagnostics) implemented 2026-09-28, committed by the user as 29b50b0. Handoff: `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/viz-handoff/` (requirements.md verbatim, spec.md, stage_results/, final_report.md).

- Feed contract `navigatr.inspect/2` (state/diag/telemetry/instrumentation/capture split, latest-wins slots, ping/pong ack window); `/api/snapshot` still inspect/1.
- Pi DiagnosticsHub + LinkMonitor + CaptureService (ZIP `gatr2.capture/1`); Brain TELEMETRY op 12; Pico diag frame 0x14.
- Everything is host-validated only; VEX roll/pitch sign convention and `kVexImuMountYawDeg` are UNVERIFIED placeholders.

**Phase 2 = read-only audit of the ENTIRE system (report only, no fixes). User said: do not start it until they ask.**

**Renames** (docs/naming.md): naviGATR, spectaGATR, aggreGATR, translaGATR (was `common/`, namespace `translagatr`), locaGATR, operaGATR. Committed by the user as 29b50b0 (08:47). The case rename pi/navigatr -> pi/naviGATR is staged in the git index only (423 R100, via update-index, backup at scratchpad/git-index.backup); the disk folder is still lowercase because this Claude session started inside it and holds it.

**Why:** the user will review the audit separately; the renames were an explicit request.
**How to apply:** do not run the audit unprompted. When it is asked for, it covers the whole system, not just the viz work. Keep `-GATR` names only where the word fits.

Related: [[project-brain-architecture]], [[reference-navigatr-windows-build]]
