Stage C1 is complete. `GeometricPlanner`, `PathPlanner::clear`, `toString(PlanStatus)` and `footprintClearance` are in the new `investigatr` target. All 74 investiGATR tests pass. One thing in your actuGATR code needs a change: as written, an avoiding move can fail near an obstacle even though its planned route is still clear.

## Integration issue in `brain/actuGATR/src/motion.cpp`
- **The problem:** `Motion::remaining()` replaces the current segment's start with the measured pose before `needsReplan()` calls `planner_.clear(...)`.
- **Why it fails:** `clear()` is exact against the grown boxes. Avoiding routes run only `vertex_margin` (5 mm) outside those boxes, along box sides and at corners. `FollowerConfig::tracking_tolerance` is 0.04 m, so a drift of more than 5 mm toward a box puts the measured origin inside a grown box.
- **What happens:** the next field generation makes `clear()` false. The replan then starts inside a grown box, returns `kStartBlocked`, and the command fails.
- **Suggested fix:** call `clear()` on the remaining planned segments unmodified. Tracking error is what `model.clearance` covers, so keep `tracking_tolerance` below it. Replan from the measured pose only when the planned remainder is no longer clear.
- **Related:** a replan from within a few mm of a corner vertex can begin with a very short leg. It is kept because it is needed to stay clear, so a tank turns to face it first.

## What changed (all in files I own)
- **`brain/investiGATR/CMakeLists.txt`:** new STATIC target `investigatr` (`src/collision.cpp`, `src/planner.cpp`), PUBLIC link to `investigatr_types`. `investigatr_tests` links it and lists 8 test files explicitly. communiGATR's existing `target_link_libraries(communigatr PUBLIC investigatr)` now finds a real target.
- **New:** `src/collision.h` (private helpers, reached by a quoted include), `src/collision.cpp` (includes `footprintClearance`), `src/planner.cpp`.
- **Tests:** added `field_gtest`, `motion_model_gtest`, `path_gtest`, `reference_gtest`, `clearance_gtest`, `planner_gtest`, `planner_random_gtest`; extended `geometry_gtest` with inverse, between and finite.
- **Docs:** rewrote `docs/investigatr.md` (types, references, the method and why it was chosen, limits, `clear()`, `footprintClearance`, config, cost, tests). It links to `docs/actugatr.md` and `docs/communigatr.md`. Updated `brain/investiGATR/README.md`.
- No public header, name, signature or type source changed. Your `gatr2_brain.mk` already lists `collision.cpp` and `planner.cpp`.

## Planner behavior your code relies on
- **Check order:**
  1. Unknown mode, non-finite pose, invalid model or invalid planner config gives `kInvalidRequest`.
  2. A non-holonomic model without turn-in-place, or `min_turn_radius > 0`, gives `kUnsupportedModel` in both modes.
- **Direct mode:** ignores `field` entirely.
  - Non-holonomic: turn, translate, turn. Each leg is driven backward when that means less total turning; ties go forward.
  - Holonomic: one translation, with the heading moving to the goal heading along it.
  - Moves shorter than `min_segment` and turns smaller than `min_turn` are dropped. `kOk` can come with an **empty path**, which the follower must treat as done. A dropped move leaves the path ending at the start position.
- **Avoiding mode:**
  - Returns `kNoField` for a null field, generation 0, non-finite or empty bounds, or a malformed obstacle.
  - R is `enclosingRadius(model)`. Obstacles are placed at `pose` when `valid`, else `nominal`.
  - Start is checked against the bounds first (`blocking` 0), then the grown boxes (`blocking` = lowest id). The goal is checked the same way.
  - The straight line is tried first and builds no graph. Otherwise A* runs on the visibility graph. A graph larger than `max_vertices` (start and goal included) gives `kNoPath`.
  - Every path gets a final self-check, and there is no fallback to direct.
- **Avoiding shapes:** a tank route alternates turns and translations, with reverse chosen over the whole route. A holonomic route is translations only, with heading linear in distance. A single short leg is dropped; in a multi-leg route every leg is kept.
- **`clear()`:** false for an invalid model or unusable field, true for an empty path. Turns are checked as points.
- **`footprintClearance`:** `distance` is the exact signed distance minus `margin`, where margin is a rounded grow. `nearest` is 0 when a bound is nearest (bounds win ties). Invalid input is reported as not clear.

## Build and test commands (Git Bash, ucrt64 first on PATH)
```
cmake -S /c/GATR2-Electronics/brain -B "$S/wC-brain" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$S/nav-base/_deps/googletest-src"
cmake --build "$S/wC-brain" -j8 --target investigatr_tests
ctest --test-dir "$S/wC-brain" --output-on-failure -j8 -R "Planner|Clearance|Reference|Geometry|Field\.|MotionModel|Path\."
```

| Check | Result |
|---|---|
| Host build (g++ 15.2, `-Wall -Wextra`) | 0 warnings |
| ctest | 74/74 passed |
| Random avoiding test (300 fields × 6 plans) | 1161 ok (360 with detours), 222 blocked, 193 out of bounds, 224 no path; 128 of the no-path results confirmed by an independent grid search |
| ARM syntax check (arm-none-eabi-g++ 13.3.1, gnu++20) of all 8 investiGATR sources | clean |
| 13 deliberately injected bugs, on a scratch copy in `$S/wC-mut` | all 13 caught by the tests |

The one change the tests don't catch is removing the planner's final self-check. That is expected, because the path construction already guarantees what it checks.

On the host, a plan on a field shaped like this season's (17 elements) takes about 11–18 µs; 127 boxes (510 vertices) takes about 7 ms. The V5 will be much slower, so if you plan inside a control task, keep `max_vertices` small enough.

## Remaining gaps
- **Your docs index:** the investiGATR line in `docs/README.md` is stale. Suggested text: "investiGATR: path planning and the shared Brain types: field, state source, references, motion model, paths; planner method and limits".
- **Conservative robot outline:** planning treats the robot as a circle, so long thin robots may be refused narrow gaps. This is documented.
- **Enum gap:** `PlanStatus` has no separate value for "graph too large"; it reports `kNoPath`, as the `planner.h` comment already says.
- **Formatting:** clang-format was applied to the new tests. The source files keep your style (multi-line short functions, one-line case labels), which clang-format 22 would reflow; CI does not enforce it.
- **Hardware:** none of this is validated on the robot.

The progress file with full details is `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/brain-arch-handoff/progress/wC.md`.

Files are in `C:/GATR2-Electronics/`:
- `brain/investiGATR/CMakeLists.txt`
- `brain/investiGATR/src/collision.h`
- `brain/investiGATR/src/collision.cpp`
- `brain/investiGATR/src/planner.cpp`
- `brain/investiGATR/tests/*_gtest.cpp`
- `docs/investigatr.md`
- `brain/investiGATR/README.md`