# Handoff: push everything before the laptop reset

Written 2026-10-12 by a Claude session that surveyed this laptop. The user is about to wipe the machine and
wants **every repo fully on GitHub, including the content that is currently local-only on purpose.**
Work through this top to bottom. Nothing here is speculative; every path was checked on disk.

User decisions already made (do not re-ask):
- Push everything in GATR2-Electronics, **including `deprecated/`** (the old "keep it local" rule in memory is superseded by this).
- `custom_tank_drive` and `Configurable-Swerve` are trash. Do not touch them.
- `ALM-Autonomous-Lawn-Mower` is the live lawn-mower repo. `AutonMower` is its predecessor.
- `secrets.local` is never committed.

## 1. C:\GATR2-Electronics (origin = github.com/Edison-LaraBojay/GATR2-Electronics, branch main, in sync with origin as of survey)

State at survey time: 46 modified, 15 deleted (spectaGATR-demo, encoder_demo), 16 untracked, 0 ahead of origin.

### 1a. Clean up junk first
- `brain/localization-demo/cd`, `brain/localization-demo/pros`, `brain/localization-demo/set` are **empty 0-byte files** created 2026-10-10 01:46 by a mistyped shell command. Delete them.
- `spectaGATR/tools/__pycache__/` - do not commit. Add `__pycache__/` to `.gitignore`.
- `debug.log` at repo root - check what it is; probably delete.
- `build/`, `build-spectagatr*/`, `build-printables/` are ignored already. Leave them.

### 1b. Commit the ordinary working-tree changes
The modified/deleted/untracked set is the localization-demo + spectaGATR receiver + aggreGATR work from 2026-10-09/10
(see memory files `project_encoder_localization_demo.md`, `project_spectagatr_standalone.md`, `project_controller_radio_link.md`).
Untracked files that belong in: `brain/localization-demo/{include/drive.h,include/drive_config.h,include/mecanum.h,src/drive.cpp,src/mecanum.cpp,tests/mecanum_gtest.cpp}`,
`pico/aggreGATR/src/encoder_config.h`, `pico/aggreGATR/src/sensor_monitor/`, `spectaGATR/receiver/feed/delivery_meter.{h,cpp}`,
`spectaGATR/receiver/tests/delivery_meter_gtest.cpp`, `scratchpad/`.
Run the existing tests before committing if the build is quick (`reference_navigatr_windows_build.md` has the recipe: ucrt64 first on PATH, MinGW Makefiles).
If something fails, commit anyway with a message saying so. Getting it off the machine matters more than green CI today.

### 1c. Bring the local-only content under git
| Path | Why it is not in git | What to do |
|---|---|---|
| `deprecated/` (31 MB, 757 files: brain, pi/naviGATR, translaGATR, pico, docs, bench, FALLBACK.md, README.md) | `.gitignore` line `/deprecated/` | Delete that line and its comment from `.gitignore`, then `git add deprecated`. No nested `.git`, no file over 50 MB, no `build/` inside. Verify with `git add -n deprecated | wc -l` (expect ~757). |
| `pi/navigatr/` (196 KB without `build/`; `build/` is 240 MB and already ignored) | never `git add`ed; not matched by any ignore rule | `git add pi/navigatr` (build/ stays out automatically). Older versions exist in history at 10c568e etc.; the current tree does not. |
| `notes/navigatr_mcu_plan.md` | `.git/info/exclude` has `/notes/` | Remove that line from `.git/info/exclude`, `git add notes`. |
| `models/` (3 STEP files, 37 MB total, largest under 50 MB) | `.gitignore` `*.step`/`*.STEP` | `git add -f models/*.step models/*.STEP`. GitHub's hard limit is 100 MB per file; these are fine. |
| `scratchpad/` (5.7 MB) | untracked | `git add scratchpad`. |
| `.claude/settings.local.json` | ignored | optional. It is only a permissions allowlist. Skip. |
| `secrets.local` | ignored on purpose | **do not add.** It is copied by the backup script instead. |
| `pcb/*/gerbers/`, `*.step` under `pcb/` | ignored by design (regenerable / manufacturer models) | leave as is unless the user says otherwise. |

Also check `git status --ignored` afterwards for anything else that looks like source and is still excluded.

### 1d. Commit and push
- One commit for 1b, one for 1c is fine. Messages: terse, plain ASCII, no AI-style prose (memory `feedback_comment_style.md`).
- `git push origin main`.
- Verify: `git status --porcelain` shows nothing except `secrets.local`-style ignored items; `git log origin/main..main` is empty; `git ls-files deprecated | wc -l` is ~757.
- CI (`.github/workflows/ci.yaml`) does not reference `deprecated/`, but it was modified in the working tree. Read the diff before committing; if it runs tests on globs that will now pick up `deprecated/`, scope them.

## 2. Other repos (all under C:\Users\mbes1\OneDrive\Desktop unless noted)

| Repo | State at survey | Action |
|---|---|---|
| `C:\ALM-Autonomous-Lawn-Mower` (Edison-LaraBojay/ALM-Autonomous-Lawn-Mower) | 2 modified (`motherboard.kicad_pro/.kicad_sch`), 10 untracked (BatteryVoltageSense, CurrentSense, MotorController, MotorPort, VoltageSense, VoltageSenseMotor, untitled .kicad_sch; `DRV/`; `H-BridgeIC.kicad_sym`; `fp-lib-table`) | `git add -A pcb`, commit, push. `DRV/` holds the DRV8262 symbol/step; include it. |
| `C:\Users\mbes1\AutonMower` (Edison-LaraBojay/AutonMower) | 1 modified `MotorControllerV2/CurrentSense.kicad_sch` | commit, push. |
| `ProBallers` (Edison-LaraBojay/ProBallers2025) | 8 modified (autons, main, scoring, subsystems, project.pros) | commit, push. |
| `MecanumDrive` (Edison-LaraBojay/MecanumDrive) | 2 modified hpp + 1 stash | commit; then `git stash show -p stash@{0}`, if it has real work `git stash pop`, commit separately; push. |
| `Swerve\GATR2-Swerve-Bot` (GATR-VEX/GATR2-Swerve-Bot) | 2 modified (robot1.hpp/.cpp) + 3 stashes | same pattern. If stashes conflict, commit them to a branch `stash-backup` and push that. |
| `running-power-meter` (NotMercury/Power-Meter) | `test_run.ino` modified + untracked `src/GradientEstimator/` | commit, push. If no push rights, push to a fork under Edison-LaraBojay. |
| `Lab11\LAB11-ELB-LO` | clean, 1 stash | pop + commit + push only if the stash looks like real work. |
| `VexPlanner`, `FinalProject`, `GATR2-RP4`, `OneDrive\Desktop\GATR2-Electronics` | clean or build artefacts only | nothing. |
| `custom_tank_drive`, `Configurable-Swerve` | - | **skip, user says trash.** |

Note: `C:\Users\mbes1` itself has a stray `.git` whose remote is MecanumDrive (accidental `git init`/clone in the home dir). Ignore it; it is not a real repo.

## 3. Not git: copy, don't push
`C:\JuniorDesign\PCBDemo\IntruderPCB` (20 MB, no repo) and the EEL3211C coursework are handled by
`C:\Users\mbes1\OneDrive\Desktop\LAPTOP-RESET\backup.ps1`. Remind the user to run it; do not run it yourself.

## 4. Done when
Every repo in sections 1-2 (except the two trashed ones) reports `git status` clean apart from ignored build output and `secrets.local`,
and `git log origin/<branch>..HEAD` is empty for each. Then tell the user it is safe to reset.
