---
name: project_push_everything_handoff
description: "2026-10-12 laptop is about to be wiped; user wants EVERYTHING in GATR2-Electronics pushed, including deprecated/ (supersedes the local-only rule); plan file in repo root; section 1 staged 2026-10-12, commit+push left for the user"
metadata:
  node_type: memory
  type: project
  originSessionId: b09f0a6f-981b-4ac6-87b0-d1f7091a1001
  modified: 2026-10-12T07:25:40.902Z
---

On 2026-10-12 the user decided to factory-reset this laptop and asked that every repo be fully pushed first,
**including `deprecated/`, `pi/navigatr/`, `notes/`, `models/` and `scratchpad/`**, which were local-only until now.
This supersedes the "deprecated/ stays local" part of [[feedback_codex_authorized]] and [[project_spectagatr_standalone]].
`secrets.local` is still never committed.

**Why:** the machine is being wiped; anything not on GitHub is lost. The user also wants the reference material
(deprecated/) reachable online while working on the structural redesign from a new device.

**How to apply:** follow `C:\GATR2-Electronics\PUSH-EVERYTHING-HANDOFF.md` top to bottom (it lists every repo,
its exact dirty state, and the un-ignore steps). `custom_tank_drive` and `Configurable-Swerve` are trash; skip them.
The wider keep/reinstall list lives in `C:\Users\mbes1\OneDrive\Desktop\LAPTOP-RESET\`.

**Status of section 1 (GATR2-Electronics), 2026-10-12:**
- 1b was already done by the user: commit `99feff4 new shi` holds the demo/receiver work and `scratchpad/`.
- 1c is STAGED but not committed: `.gitignore` no longer ignores `/deprecated/` and ignores `__pycache__/`;
  `.git/info/exclude` no longer hides `notes/`; staged = 691 files under `deprecated/`, `notes/navigatr_mcu_plan.md`,
  3 `models/*.step` (force-added, largest 32.7 MB), `PUSH-EVERYTHING-HANDOFF.md`, and deletions of `debug.log`,
  three 0-byte files in `brain/localization-demo/` and `spectaGATR/tools/__pycache__/`.
- The Claude auto-mode permission classifier refused both `git commit` and `git push` for this set, so the user runs
  them: `git commit -m "Track retired software, notes and 3D models"` then `git push origin main`.
- Findings that correct the survey: `pi/navigatr/` holds NO source, only empty dirs and the ignored 240 MB `build/`
  (the source is `deprecated/pi/naviGATR/`); the 66 files still ignored inside `deprecated/` are PROS kernel copies,
  `bin/`, `.d/` and one `.vscode` file, not source (691 + 66 = 757 on disk); CI uses explicit `cmake -S` paths and
  will not build `deprecated/`.
- Open question for the user: `pcb/PiHat/PiHat_V3/New folder/RaspberryPi4Hat.step` (6 MB, 2026-09-26) is ignored
  by the pcb `*.step` rule but looks like the user's own export, not a vendor model. Not added.
- Sections 2 and 3 (other repos, backup script) not started by this session.
