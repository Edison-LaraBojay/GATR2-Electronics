Stage C2 is complete. All 98 investiGATR tests pass (0 warnings), and your actuGATR tests pass against the new planner: `actugatr_tests` 73/73 and `actugatr_integration_tests` 6/6.

One result needs your attention first. With the placeholder robot in `brain/robot/gatr2_robot.h` (0.46 m square) and `kClearance` 0.06, the planner treats the robot as a circle of radius R = 0.385 m. On the Override field that robot cannot get out of the corner areas in avoiding mode. Details are in the second section.

## What changed (all in files I own, under `C:/GATR2-Electronics/`)

**`brain/investiGATR/include/investigatr/path.h`**
- Added `bool exact = false;` to `PathSegment`, exactly as spec 8.1 says. This is the only change to a shared type.

**`brain/investiGATR/include/investigatr/planner.h`**
- Comments only. No names, signatures or defaults changed.

**`brain/investiGATR/src/planner.cpp`**
- **Order of checks at each end:**
  1. The robot must fit inside a circle of radius R that is clear of everything.
  2. If not, the exact robot outline is checked. Touching a wall or box is allowed; overlapping is refused with the right status and `blocking`.
  3. If either end fails the circle check and no turn is needed, a single straight exact move is used. Start and goal at the same pose gives an empty path, even against a wall.
  4. Otherwise the planner tries a **start escape**. It is the shortest straight move of at most 2R that reaches clear space: forward, backward (when reverse is allowed), or sideways for holonomic drives. It becomes the first segment, with no turn before it.
  5. The **goal approach** is the same idea, mirrored. It runs along the goal heading, so there is no turn at the goal.
  6. An escape or approach in line with the next leg is merged into one exact segment.
- **Obstacles are now grown into octagons instead of rectangles.** The sides move out by R and the corners are cut at 45°. This overstates obstacles by at most 8.3% of R, against 41% before. With rectangles, the diagonal gaps between neighbouring goals were closed for any robot wider than about 0.29 m.
- **Other changes:**
  - Corners in a straight line are merged into one leg.
  - The graph stops growing as soon as it reaches `max_vertices`.
  - Obstacle crossing tests use precomputed edges and a quick bounding-box check first, which made the worst case about 20 times faster.

**`brain/investiGATR/src/collision.h` and `collision.cpp`** (private)
- Distance between convex polygons, the swept shape of a moving robot, the octagon, and the obstacle crossing tests. Removed two helpers nothing uses any more.

**Tests**
- New `tests/planner_exit_gtest.cpp` (12 tests):
  - starts against each of the four walls, facing away, along and toward the wall, and in a corner;
  - goals beside a landmark, facing it and with the robot's back to it;
  - escapes blocked by a second box, and by a small post in the way out;
  - the 2R escape limit;
  - how `clear()` handles exact segments.
- New `tests/planner_edge_gtest.cpp` (12 tests):
  - boxes touching at a corner, collinear edges, a long row of boxes;
  - start equal to goal, heading wrap at ±π, boxes crossing the boundary;
  - overlapping boxes forming walls and a closed ring, zero-size boxes;
  - fields the robot barely fits or doesn't fit, same result every run, and a 128-box map.
- `planner_gtest.cpp` updated for octagons and escapes.
- The random test now also starts and ends next to walls and boxes (300 fields × 8 plans).
- Both new files are listed in `CMakeLists.txt`. No new files in `src/`, so `gatr2_brain.mk` is unchanged.

**Docs**
- `docs/investigatr.md` rewritten. It covers:
  - the shared types, references with `Field::Origin`, the generated table and map-id checking;
  - the planner method and why it was chosen, direct vs avoiding;
  - tank vs holonomic path shaping, status codes, and escapes and approaches;
  - the `clear()` rules, cost with measured numbers, and limits;
  - the test table.
- `brain/investiGATR/README.md` updated.

## Things you need to know

1. **Your placeholder robot can't use avoiding mode in the corner areas.**
   - **Cause:** the diagonal gaps between goals only open when R < 0.313 m, and your robot's R is 0.385 m.
   - **Effect:** avoiding moves from the corner areas fail with `kNoPath` or a start refusal.
   - **Measured on the field's collision boxes (corner area to the center):**

     | Robot side | Largest clearance that still routes |
     |---|---|
     | 0.381 m | 0.04 m |
     | 0.40 m | 0.02 m |
     | 0.457 m (18") | none |

   - **Options:**
     - use the real robot's footprint, if it is smaller;
     - use a smaller clearance;
     - use direct moves in those areas.

     An 18" robot would need a planner that accounts for heading, which is not implemented.

2. **Spec 8.1 deviation.** The spec says the swept outline is grown by `model.clearance` everywhere. Taken literally, that refuses every escape for a robot touching the wall. Instead the path keeps the clearance everywhere except next to its start and goal poses. There it may come no nearer to anything than those poses already are. This is documented and recorded under CONTRACT ISSUES.

3. **`clear()` from a measured pose.** It still suits your `Motion::remaining()`:
   - The first segment's start may already be a millimetre into a wall, as long as the move doesn't go deeper.
   - Ordinary segments near a corner can still fail on tracking error alone. When they do, the replan now escapes along the heading (if a way out exists) instead of failing with `kStartBlocked`.
   - Exact segments keep only `model.clearance`, so keep `tracking_tolerance` below it.

4. **Planning time.** Measured on the host with -O2, not on the V5:

   | Case | Time per plan |
   |---|---|
   | Override field | 0.06–0.55 ms |
   | Worst case at the default `max_vertices` 512 (63 boxes, no route) | 18 ms |
   | 128 boxes with every vertex kept (limit raised), routed / no route | 22 ms / 97 ms |
   | Over the default limit, refused | about 0.5 ms |

   The V5 will be much slower. Keep `max_vertices` small if you plan inside a control task.

5. **Your docs index.** The investiGATR line in `docs/README.md` is still the old text. Suggested: "investiGATR: path planning and the shared Brain types: field, state source, references, motion model, paths; planner method, exits by walls, cost and limits".

## Commands run (Git Bash, ucrt64 first on PATH, `$S` = scratchpad)

```
cmake -S /c/GATR2-Electronics/brain -B "$S/wC-brain" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$S/nav-base/_deps/googletest-src"
cmake --build "$S/wC-brain" -j8 --target investigatr_tests --clean-first
"$S/wC-brain/investiGATR/investigatr_tests.exe"
cmake --build "$S/wC-brain" -j8 --target actugatr_tests actugatr_integration_tests communigatr_tests
ctest --test-dir "$S/wC-brain" --output-on-failure -j8 -R "Planner|Clearance|Reference|Geometry|Field\.|MotionModel|Path\."
```

| Check | Result |
|---|---|
| `investigatr_tests` | 98/98 passed, 0 warnings |
| ctest with the filter above | 104/104 (the pattern also matches 6 of your tests) |
| Your actuGATR tests | `actugatr_tests` 73/73, `actugatr_integration_tests` 6/6 |
| `communigatr_tests` | compiles with the new `PathSegment` |
| Random avoiding run | ok 1409 (826 detours, 336 escapes, 308 approaches), blocked 418, out of bounds 362, no path 211 (69 confirmed by an independent grid search) |
| ARM syntax check (arm-none-eabi-g++ 13.3.1, gnu++20) of all 8 sources | clean |
| 23 deliberately injected bugs in the new code, on a scratch copy in `$S/wC-mut` | all 23 caught; 5 needed new tests to catch them |

All owned files are LF line endings and plain ASCII.

## Remaining gaps
- Planning treats the robot as a circle (quantified above); a heading-aware planner is not implemented.
- There is only one straight escape or approach per end, at most 2R long. Turning in place near a wall means driving out and back in.
- Toggles are treated as full-height obstacles; there is no height modelling and no moving obstacles.
- Planning time on the V5 is not measured.
- Nothing here is validated on the robot.

The progress file is `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/brain-arch-handoff/progress/wC.md`.