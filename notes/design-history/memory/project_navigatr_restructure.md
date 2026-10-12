---
name: project-navigatr-restructure
description: 2026-09-28 naviGATR folder restructure (core/, pipeline/<stage>/, estimators/, system.cpp split, LandmarkError) running in a git worktree; result is a patch to apply to the main tree
metadata:
  type: project
---
Approved by the user on 2026-09-28. The spec is `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/restructure/SPEC.md`.

**Layout:**
- `src/main.cpp`, `cli/`, and `core/` (build.cpp, run.cpp, thin system.h).
- `pipeline/{framework,resources,sensors,commands,localization,world_estimation,targets,publishing}/`. Each stage has contract.h, stage.cpp and plug-in folders named after their XML element (never `impl`).
- `brain_profile/`, `config/`, `io/`, `time/`, `math/`, and `debug/{inspection,capture,diagnostics}`.

**World estimation:**
- Exactly one Estimator; no fusion yet.
- Output is `LandmarkError{dx, dy, dtheta}` from the map nominal, in field axes.
- The Brain wire is unchanged (full poses).
- Tag traces move to `estimators/apriltag/`.

**Execution:**
- Workflow wf_eace851a-eba runs in the detached git worktree `<scratchpad>/wt-navi`, at ae457ee.
- Scratchpad = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
- Steps: move, then split, then world estimation, then PIPELINE.md docs, then two verifiers and a fix loop.
- Output: `restructure/restructure.patch` and `REPORT.md`. The patch is applied to the main tree only after the agents reading it finish. Never commit.

**Why:** the user found naviGATR unreadable (the main pipeline was buried in runtime/system.cpp, and folders like core/app/contracts were grab bags).

**How to apply:** after verification, apply the patch to C:/GATR2-Electronics. Then remove the worktree with `git worktree remove`. Later work (spectuGATR, the coding standard) uses the new paths. Related: [[feedback-system-philosophy]], [[project-spectugatr]], [[project-coding-standard]].

**Addendum (2026-09-28 16:30):** after the current run, steps 5-7 run in the same worktree per restructure/SPEC_ADDENDUM.md:
- make_* for every stage plus verb-named callables, and main split into parse/build/run/report;
- namespaces per folder;
- doc updates.

After that, the patch is re-packaged. The whole-repo clang-format pass (D-25) runs on the main tree only after both the restructure and spectuGATR Phase A patches are applied.
