# Restructure addendum (user decisions 2026-09-28 ~16:30)

Runs AFTER the current restructure run (steps 1-4, verify, package) finishes, in the same worktree `S/wt-navi`, continuing from its state. The user's words and the full decision text are in `coding-standard/requirements.md` ("User decisions on the standard").

## Steps
**Step 5, stage builders and entry names (D-5 + F4 + F11). Behavior-free.**
- `make_commands`, `make_targets`, `make_publishing` in their stage folders, so every stage has a `make_*`.
- Every stage and sub-step returns a callable with a descriptive `using` alias. Construction in core/build.cpp and the run loop in core/run.cpp bind them to verbs: read_resources, read_sensors, collect_commands, localize, estimate_world, resolve_targets, publish.
- Inside world_estimation/estimators/apriltag: extract_tags, associate_tags, fold_landmarks.
- Existing classes are wrapped by their make_* (the lambda captures the instance). Internals are not rewritten.
- `main.cpp` becomes parse, build, run, report: runInline and runThreaded move to core/run.cpp, and printing moves to cli/report.cpp (F11).

**Step 6, namespaces mirror folders (D-6). Behavior-free.** Mapping:
- `navigatr::<area>` for cli, core, config, io, time, math, brain_profile, inspection, capture, diagnostics (debug/ adds no level);
- `navigatr::framework`;
- `navigatr::<stage>` (pipeline/ adds no level);
- one more level for self-contained family / sensor-group folders (world_estimation::apriltag, resources::cameras / pico / links / geometry / tag_detectors / synthetic);
- grouping folders add no level.

Use explicit using-declarations or aliases at namespace scope for cross-namespace names. No `using namespace` in headers. Watch for ADL and overload-resolution changes: tests must be identical.

**Step 7, docs:** update the PIPELINE.md files and the architecture doc for the new entry names and namespaces.

**Then:**
- Verify: fresh builds, the same test names passing, viewer smoke, and an adversarial behavior review as before.
- Re-package: a cumulative `restructure.patch` plus a REPORT.md update.

## Formatting (D-25)
The whole-repo reformat is NOT done in this worktree. It runs as a separate format-only pass on the main tree after the restructure patch and the spectuGATR Phase A patch are both applied, so it cannot conflict with them.
