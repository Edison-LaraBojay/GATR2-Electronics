---
name: project-investigatr-brain-link
description: investiGATR + brain link v3 PAUSED MID-IMPLEMENTATION 2026-09-26 14:34; navigatr does not build; resume from handoff dir with the v2 workflow script (continuation prompts); peer session d1 owns pico/BNO files
metadata:
  node_type: memory
  type: project
  originSessionId: ffabe616-f541-4ae8-9301-414e9aa793ac
  modified: 2026-09-27T00:47:43.367Z
---

User request 2026-09-26: implement investiGATR (Brain nav lib) + Navigatr integration; do NOT commit. The user ended the session at 14:34 to restart about 20 min later, so work is PAUSED MID-CHANGE.

**Handoff files (durable, outside repo):** `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/investigatr-handoff/`
- `requirements.md`: the user's full verbatim request.
- `spec.md`: design contract v2, revised after the 4-lens adversarial review. Its `$S` points at the OLD session scratchpad; use the new session's scratchpad for build dirs.
- `design_review.txt`: the review findings.
- `implement_workflow.js`: the parallel workflow script. Roles A1 common, A2 investiGATR, B1a Pi half-duplex, C2a PROS scaffold, B2a client, B1b Pi core, B2b driver, C2b PROS app, B1docs, C1 e2e, C3 docs/CI.
- `tree_status_at_pause.txt`: the tree state at pause.

**Tree at pause (nothing finished or verified; no agent returned a result):**
- common/ is partly converted to v3: frames.h, frame_codec.*, and ~1000 lines of new tests. It is not verified.
- Pi transport is partial: half_duplex.h/.cpp and serial_link/serial_links/serial_port edits; half_duplex_gtest.cpp was not yet written. pi CMakeLists and tests CMakeLists were edited.
- brain/investiGATR: most of it is written (navigator.cpp ~670 lines, sim, odometry). No tests yet.
- brain/testing does not exist yet.
- Result: navigatr does NOT build (vex_brain*.cpp still use the removed PoseFrame/CommandFrame).

**How to resume:** relaunch `implement_workflow.js` via Workflow with the S constant changed to the new scratchpad. The CONTINUATION note is already on A1/A2/B1a; add it to C2a too, since the old C2a ran ~5 min. Old-session resumeFromRunId does not work across sessions. The googletest source may still be at the old scratchpad `nav-base/_deps/googletest-src`; otherwise FetchContent downloads it. Baseline before the change: navigatr 282/282 tests passed. After implementation: run the verification gate (common, navigatr, brain ctest, pros make), then a multi-lens review workflow, then message the peer.

**Resumed 2026-09-26 ~15:15 (session 73859862):** relaunched as run wf_ddbef890-f1c using `investigatr-handoff/implement_workflow_resume2.js` (new scratchpad `.../73859862-8cc3-4c8a-a408-13cc873c10fd/scratchpad`, with the spec and googletest source copied there; CONTINUATION notes on A1/A2/B1a/C2a). Peer d1 has ended, and three new idle peers (6d, 61, f2) exist. If this run is also interrupted, check the journal.jsonl under that session's subagents/workflows for finished roles and hand-author a continuation.

**16:30 rename (user decision):** brain/navigatr-interface is now **brain/communiGATR**. Namespace `communigatr`, headers under include/communigatr/, classes Client, Driver, ProsDriver (plus ProsDriverConfig/ProsDriverStatus), ProsSerialPort; doc docs/communigatr.md; make lib name `communigatr`. "Navigatr" stays the name of the Pi runtime. The user is writing their OWN barebones Brain test against communiGATR, and agents must never touch it (spec 0.2). Verified after the rename: brain ctest 133/133 and pros make clean; common 73/73 and navigatr 326/326 earlier. Workflow relaunched from `investigatr-handoff/implement_workflow_resume3.js` (task wd3er1xn3, same run id; A1..B2b cached). Remaining roles: C2b PROS app, B1docs, C1 e2e, C3 docs/CI. After that: the verification gate, then the review workflow.

**~17:00: IMPLEMENTATION WORKFLOW COMPLETE** (all 11 roles). Agents reported navigatr 350/350 (8 new BrainLinkEndToEnd tests), brain 137/137, common 73/73, and a clean `pros make` of the brain/testing demo app (DriveControl + main.cpp). brain/testing stays the demo app (user decision). The user's own barebones test is **brain/localization-test/** (theirs; never touch). Agent results are dumped in the scratchpad (results_so_far.txt, result_b2b.txt, results_final.txt). Running now: independent gate (task bhfxwkzxv) and the 7-lens review with adversarial verify (run wf_88ffceb1-f72). Known small follow-ups:
- radToCdeg -18000
- FieldObject object_id not validated
- vision-alive bit flickers
- root .gitignore lacks brain/*/firmware
- pico_telemetry next()
- `pros c apply --force-apply` rewrites .gitignore

Next: fix the confirmed findings, re-gate, then write the final report.

**17:13: account usage limit hit (429; session limit resets 18:30 ET).** The gate was already GREEN: common 73, brain 137, navigatr 350, pros make clean. Six review agents failed and the review workflow was stopped. Resume it after 18:30 with `Workflow({scriptPath: <session workflows/scripts/review-investigatr-brain-link-wf_88ffceb1-f72.js>, resumeFromRunId: "wf_88ffceb1-f72"})`; if that path is refused, copy the script into the scratchpad first. Fixed by hand meanwhile (navigatr 350 and brain 137 still green):
- radToCdeg maps -18000 to 18000, with a new test BrainLinkState.WireHeadingStaysInsideItsRange.
- pico_telemetry drains with next().
- Root .gitignore gains the brain/*/firmware, include/pros, bin and .d entries.
- brain/CMakeLists.txt adds communiGATR unconditionally.

**20:20 review relaunched** by session 827c2c12 (the parallel-wheel session) at the user's request, as run wf_1e521a91-11d. All 7 lenses of wf_88ffceb1-f72 had failed or stopped with no results. It targets committed HEAD (882c95f, 132f78f "janky fix", 9a8f790) and includes the op 5 bench IMU wire extension; the bench localization model and the uncommitted bench edits are out of scope. Script: `C:/Users/mbes1/.claude/projects/C--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/workflows/scripts/review-investigatr-brain-link-resume-wf_1e521a91-11d.js`.

**Review wf_1e521a91-11d COMPLETE (~20:50):** 14/14 agents; 18 confirmed, which dedupe to 9 issues, and 5 refuted. Full JSON: session 827c2c12 scratchpad review_result.json.
- high: ProsSerialPort::open calls serial_set_baudrate right after serial_enable against a PROS type cache the daemon refreshes every 2 ms, so it gets ENODEV. brain/testing calls start() once, never retries, and the link stays down after a Brain power-up.
- medium: bench_imu client polls op 5 with a new rid between a timed-out SET_POSE and its retry, so the retry is Stale and the placement kRejected.
- medium: PROS task_delete of a competition task holding the DriveControl/ProsDriver mutex deadlocks the drive task (motors keep their last voltage).
- low: op5/bench_imu missing from interfaces.md/communigatr.md; brace/alignment style in the bench additions; tcdrain fallback unbounded, and CRTSCTS not cleared; a queued placement with no session never times out, and the docs initialize() example is unsafe; brain/testing comment about the anchor; the navigator perpendicular advance never fires with default config (docs + test).
Fixes NOT applied yet; the user decides.

Still open: FieldObject object_id validation (needs field objects passed through SlotInitializationContext) and the vision-alive flicker (a user decision).

**Key design:** see spec.md. In brief: brain link v3 is request 0x10 / reply 0x11 with CRC-16, HELLO/SET_POSE/SELECT/GET_STATE, Pi-assigned sessions, pi_instance, per-session dedupe, and placement identity (origin, session, sequence). The Pi uses brain_link Commands + Publishing and real half duplex. PROS builds need gnu++20 (the template's gnu++26 fails on arm gcc 13.3), and library sources compile in place via brain/gatr2_brain.mk (never reuse common.mk's dependency rules on them).

**Coordination update (15:30):** d1 resumed as peer **gatr2-electronics-61**. It is implementing BNO08X, parallel wheels, A/B profiles (Brain SET_POSE placement, no InitialPlacement), and viewer/preview work. It now also owns viewer/**, inspection_service.*, inspection_state.h, and parallel_wheels_gtest; it is deleting the old two-wheel templates. In system.cpp it takes only publishDetectionFrames; in inspection_document.cpp we have only the command and links sections. It relies on RobotState.initialized keeping its meaning. All of this is recorded as spec section 0.1 (scratchpad copy). Message 61 when common/ and Pi brain_link are green, with the final brain_link XML schema and command/links JSON names.

**Coordination (original):** peer Claude session "gatr2-electronics-d1" owns pico/** (BNO08X), tracking_wheel_motion.*, register_localization.cpp, and new two-parallel-wheel profiles. Shared docs get localized edits only. Its requests are folded into the spec: require_observed_landmark defaults to true; kResultLandmarkUnsupported when world estimation is noop; landmark status enum; goToRobotRelative. At pause it was warned that navigatr is broken mid-change. Message it when common/ + Pi brain_link + brain/ are green.

See [[project-wire-format]] (pose/command frames being retired), [[reference-navigatr-windows-build]], [[feedback-conventions]].
