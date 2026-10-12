# naviGATR restructure spec (2026-09-28; user said "go ahead with that clean up then of the repo and especially world estimation with the shit I proposed")

All the user's own words and the discussion are in `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/coding-standard/requirements.md` (the sections from "The user's philosophy" onward). This file is the distilled, approved decision list. Where they differ, this file wins.

## Where the work happens
- **Worktree:** `S/wt-navi` (detached HEAD at ae457ee). All edits happen ONLY there.
  - Never touch `C:/GATR2-Electronics`, the user's working tree; other agents are reading it.
  - `S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
- **git:**
  - In the worktree you may use `git mv` and `git add` (index only).
  - NEVER commit, push, stash, reset or checkout another ref. Never touch the main repo's refs.
- **Builds:** only in `S/wt-build/<your-key>/`. Git Bash recipe:
  - `export PATH=/c/msys64/ucrt64/bin:$PATH`
  - `cmake -S S/wt-navi/pi/naviGATR -B <dir> -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src`
  - `cmake --build <dir> -j16`
  - `ctest --test-dir <dir> -j16 --output-on-failure`
  - Test executables need ucrt64 first on PATH or they exit silently with 0xc0000139.
- **Test baseline:** naviGATR has 626 tests at ae457ee, all passing.
  - After every step, the same test names must pass, plus any new tests.
  - Record `ctest -N` lists before and after, and diff them.
- **Scope:** `pi/naviGATR/**`, plus docs that reference its paths: `docs/*.md`, root `README.md`, `pi/naviGATR/README.md`, `pi/naviGATR/docs/*.md`, `.github/workflows/ci.yaml` only if a path in it changes.
  - Do not touch brain/, pico/, translaGATR/ or pcb/ except for doc path references.
  - `pi/naviGATR/spectaGATR/`, `tools/` and `config/` keep their places, except `field_refs` (see below).

## Approved target layout for `pi/naviGATR/src/`

```
main.cpp                 tiny entry: parse args, build, run
cli/                     command-line parsing (from app/). field_refs.cpp is a separate generator program: move it to pi/naviGATR/tools/field_refs/ and update its CMake target
core/                    the engine and nothing else: builds the pipeline and runs it
  system.h               thin facade: build, start, stop, reset, snapshots
  build.cpp              System::build calling each stage's make_* in order
  run.cpp                the two workers and step() (latest-wins handoff)
pipeline/                the stages, in run order, each the same shape
  framework/             stage machinery: function registry, typed payloads, payload descriptors, records, ids, function status, execution context (from old core/)
  resources/  sensors/  commands/  localization/  world_estimation/  targets/  publishing/
    each stage folder:
      PIPELINE.md        a few lines (the docs step writes these)
      contract.h         the stage's standard input/output (from contracts/<x>.h)
      stage.h/.cpp       its make_* builder (from runtime/<x>_stage.*); stages without a builder skip this
      <plug-ins>/        implementations plus their register_*.cpp, in a folder whose name is a readable plural noun for what the plug-ins are (estimators, collectors, resolvers, publishers, channels, observation_models); a stage with many plug-ins groups them by family instead (resources/cameras, resources/pico, ...). Never a mechanical plural of the XML element that reads badly, never a stutter (resources/resources), NEVER a folder named impl.
      generic helpers    that belong to the stage (catalogs, stores, state types the stage owns, payloads it produces) sit at the stage level, not in the plug-in folder
brain_profile/           Brain-driven reconfiguration: profile build (runtime/brain_profile_builder.*), profile host/prepare/apply/swap/waiting, sensor loss and recovery, stationary precheck (split out of system.cpp)
config/                  XML parsing (unchanged)
io/                      serial ports and byte streams (from transport/)
time/                    host clock, clock sync, time types (from old core/)
math/                    unchanged
debug/                   inspection/, capture/, diagnostics/ moved as subfolders (spectuGATR will reshape later)
```

Other rules:
- The old folders `app/`, `contracts/`, `payloads/`, `state/`, `runtime/`, `impl/`, `resources/` (top level), `transport/` and the old `core/` all disappear.
  - Each file moves to the stage or place that owns it.
  - A type used by several stages lives with its producer. Truly shared machinery goes in `pipeline/framework/`, `time/` or `math/`.
- `impl/noop/noops.*`: split so each stage's noop sits with that stage's plug-ins (a small split is fine).
- Namespaces are UNCHANGED (`navigatr`). Mirroring namespaces is a separate later decision.
- Include paths stay relative to `src/` (e.g. `#include "pipeline/localization/contract.h"`).

## world_estimation (the user's model, confirmed)
- The purpose, for its PIPELINE.md: the goal is lining up relative to field objects, not absolute field truth.
  - Localization error appears in both the landmark error and the robot pose, so it cancels for targets relative to a landmark.
  - Residual error: perception error (AprilTag detection, calibration, mount geometry), plus localization drift since the last sighting. Re-resolving landmark-anchored targets every cycle keeps the drift small.
- Input: the SensorMap (plus pose history, robot state, previous field). The XML picks EXACTLY ONE `<Estimator type="...">`, a sensor group with its own config (apriltag today).
- No fusion step and no multiple estimators. Keep the "exactly one Estimator" rule.
- Layout:
  - `pipeline/world_estimation/`:
    - `contract.h` (from contracts/field_estimation.h);
    - generic step contracts (perception, association);
    - `evidence.h`: FieldObjectPoseEvidence and its set only;
    - field state (FieldState, `LandmarkError`);
    - generic evidence folding (landmark estimator, sensor-agnostic);
    - `stage.h/.cpp`.
  - `pipeline/world_estimation/estimators/apriltag/`: the apriltag world estimator, tag observation extraction, tag mount association, tag observation payloads, and `TagAssociationTrace`/`TagAssociationTraceSet` (moved OUT of the generic evidence header), plus its register file.
  - The world-estimation noop goes in `estimators/noop/`.
- OUTPUT CHANGE (S2, behavior-identical):
  - Each estimated field object exposes `struct LandmarkError { Meters dx; Meters dy; Radians dtheta; }`: a displacement in FIELD axes plus a rotation from the map's nominal pose. (0, 0, 0) means the field is as built. The map's heading zero is an authoring convention with no meaning.
  - The estimated field pose equals nominal plus error: position = nominal + (dx, dy); heading = wrap(nominal heading + dtheta). It must be numerically identical to today's estimated pose.
  - Keep the existing odometry-frame retention and anchor/epoch re-expression semantics (FieldObjectState keeps `T_odom_object`, and the field pose is derived under the current anchor). The error is derived from that derived field pose and the map nominal, wherever the field pose is derived today.
  - Add ONE helper that applies a landmark's error to a nominal target defined relative to that landmark: rotate the target's offset from the landmark's nominal position by dtheta, then translate by (dx, dy); target heading = nominal target heading + dtheta. Consumers use it in place of hand-rolled composition (target resolution).
  - Inspection's `heading_error_deg` and `displacement_m` read from the error directly.
  - The Brain wire format is UNCHANGED: field documents still carry full poses, computed as nominal plus error. Whether the wire switches to the error form is an open decision; do not do it.
  - New unit tests:
    - (0, 0, 0) is the identity;
    - the helper equals the old pose composition, over random cases incl. wrap near +-pi;
    - the error round-trips with the field pose.

## Code style for new or touched code
- Use `using` aliases liberally for non-trivial types in signatures, members, containers, callbacks, IDs and units; never hide ownership or pointer-ness.
- `struct` for data-only types, `class` for data plus behavior. Prefer free functions over classes made for their own sake.
- Comments: plain ASCII, terse self-describing labels, not AI-ish, no over-explaining.
- clang-format 22.1.8 with the repo `.clang-format` (at `/c/msys64/ucrt64/bin/clang-format`). Format only files you touched.

## Behavior rule
- Step 1 (move) and step 2 (split) change NO logic. Only file locations, include paths, CMake lists, and the code moved between files in the split.
- Step 3 (world estimation) is representation-only and numerically identical.
- Any test outcome change is a bug to fix, not to accept.
