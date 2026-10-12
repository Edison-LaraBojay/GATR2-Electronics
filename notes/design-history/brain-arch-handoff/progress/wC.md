# Worker C progress (investiGATR planner)

## Stage C1: planner (spec 6.1)

### Done
- New target `investigatr` (STATIC: src/collision.cpp, src/planner.cpp), PUBLIC link to
  `investigatr_types`. communiGATR's existing `target_link_libraries(communigatr PUBLIC investigatr)`
  now resolves to it.
- src/collision.h (private header, quoted include from src/*.cpp; works with -iquote PROS builds):
  rotated rectangle helpers, obstacle pose rule (estimate when valid else nominal), field
  usability check, open-interior point and segment tests, exact signed rectangle distance.
- src/collision.cpp: `footprintClearance` (exact: separating axes for overlap depth, vertex-edge
  distance when apart, plus bounds; distance = exact signed distance minus margin, i.e. margin is a
  rounded grow; nearest = lowest distance, bound wins ties and reports 0).
- src/planner.cpp: `toString(PlanStatus)`, `GeometricPlanner::plan`, `GeometricPlanner::clear`.
- Tests (all listed explicitly in brain/investiGATR/CMakeLists.txt): geometry (added inverse,
  between, finite), field, motion_model, path, reference, clearance, planner, planner_random.
- No header, public name or signature changed. No type source changed.

### Planner behavior (for the lead)
- Order of checks: mode/finite poses/valid(model)/valid config -> kInvalidRequest; then
  (!holonomic && !turn_in_place) || min_turn_radius > 0 -> kUnsupportedModel (both modes).
- Direct: ignores `field` completely (may be null, never checked). Non-holonomic: turn, translate,
  turn; reverse picked per leg by least total turning (start turn + final turn), forward on ties.
  Holonomic: one translate, heading start -> goal along it. A translation < min_segment is dropped
  (path then ends at the start position); turns < min_turn dropped. Path may be EMPTY with kOk
  (already there): follower must treat that as done.
- Avoiding: field null, generation 0, non-finite/empty bounds, or an obstacle with non-finite
  pose/box or negative size -> kNoField. R = enclosingRadius(model). Start: bounds (shrunk by R)
  first -> kStartOutOfBounds (blocking 0), then grown boxes -> kStartBlocked (blocking = first
  object in field order, i.e. lowest id). Same for goal. Straight segment tried first (builds no
  graph, ignores max_vertices); else A* on the visibility graph; graph larger than max_vertices
  (start and goal count) -> kNoPath. Final self check of every segment and turn point; never falls
  back to direct.
- Avoiding shaping: non-holonomic turn/translate alternation with reverse chosen by a 2-state DP
  over legs (least total turning, forward on ties); holonomic translate-only, heading linear in
  path length. A single short leg (< min_segment) is dropped; in a multi-leg route every leg is
  kept even if short, since each is needed to stay clear.
- Only objects with `obstacle` set count. Pose used: `pose` when `valid`, else `nominal`.
- clear(path, field, model): false for invalid model or unusable field; else every segment start
  and end inside bounds shrunk by R and the segment not crossing any grown box interior. Empty
  path -> true. Turn segments are point checks.
- Cost: override-like field (17 boxes) about 11-18 us per plan on the host (-O2/-Os); 127 tiny
  boxes (510 vertices) about 7 ms on the host. Expect roughly 20-50x slower on the V5.

### Notes for the lead (integration risks)
- IMPORTANT, actuGATR src/motion.cpp `Motion::remaining()` + `needsReplan()`: it replaces the
  current segment start with the MEASURED pose before `clear()`. clear() is exact against the
  grown boxes, and avoiding routes run only `vertex_margin` (5 mm) outside them along box sides
  and at corners. With FollowerConfig::tracking_tolerance 0.04, a drift of more than 5 mm toward
  a box puts the measured origin inside a grown box: clear() is false on the next field
  generation, the replan starts inside a grown box and returns kStartBlocked, and the command
  fails although the planned route is still clear. Recommended: check the remaining PLANNED
  segments unmodified (tracking error is what model.clearance covers; keep tracking_tolerance
  below it), and replan from the measured pose only when those are no longer clear. A replan
  from a pose within a few mm of a corner vertex can also start with a very short leg (kept,
  since it is needed to stay clear; non-holonomic then turns to face it).
- footprintClearance margin is a rounded grow (Minkowski disk). Every pose on a planned path has
  footprintClearance(pose, footprint, model.clearance, field).distance >= 0 (random test checks
  this for sampled poses along translations and turns).
- max_vertices overflow reports kNoPath (no separate status exists).

### CONTRACT ISSUES
- none blocking.
- Minor (no action needed): PlanStatus has no "graph too large" value; max_vertices overflow is
  kNoPath as the planner.h comment already says.

### Test results
- Build: `cmake --build $S/wC-brain -j8 --target investigatr_tests` (host g++ 15.2, Debug,
  -Wall -Wextra): 0 warnings.
- `investigatr_tests.exe`: 74 tests, 74 passed (11 suites). Random avoiding run: 1161 ok (360
  multi-leg detours), 222 blocked, 193 out of bounds, 224 no path (128 confirmed by an independent
  grid search).
- ARM syntax check (arm-none-eabi-g++ 13.3.1, gnu++20, -Wall -Wextra) of all 8 investiGATR
  sources: clean.
- Mutation check on a scratch copy ($S/wC-mut): 13 injected defects (no segment test, SAT-only
  distance, nominal-only poses, footprint side ignored, footprint not about origin, A* without
  visibility, corners pulled in, wrong reverse heading, DP without reverse, no bounds check,
  corners outside region kept, bound not shrunk, always straight) all killed; only the removal of
  the redundant final self check survives (expected).

- docs/investigatr.md rewritten (types, references, planner method and why, limits, clear(),
  footprintClearance, config, cost, tests) and brain/investiGATR/README.md updated. Links to
  docs/actugatr.md (lead's, for the Motion API and following) and docs/communigatr.md.
- Lead's docs/README.md index line for investiGATR is stale ("navigation library: API, input
  contract, ... control law"); suggested text: "investiGATR: path planning and the shared Brain
  types: field, state source, references, motion model, paths; planner method and limits".
- clang-format 22.1.8 applied to the new tests; src files match the lead's style (multi-line
  short functions, one-line case labels), which clang-format would collapse or split.
- brain/gatr2_brain.mk already lists collision.cpp and planner.cpp; nothing needed there.

### In progress / next
- nothing; stage C1 complete.

### Risks
- Visibility graph plus inflated rectangles is conservative: long thin robots and narrow gaps may
  be refused (documented).

## Stage C2: harden, start escape / goal approach (spec 8.1), docs

### Done
- `PathSegment::exact` added exactly as spec 8.1 (additive, default false). No other public
  name or signature changed; planner.h comments only.
- Start escape and goal approach (spec 8.1). Order per end: circle test; if it fails, exact
  footprint (contact allowed, overlap refused with the right status and blocking); then
  - straight exact move start->goal when either end fails the circle test and no turn is needed
    (headings within min_turn; non-holonomic: goal on the start heading line). Same pose gives
    an empty path (already there, even touching a wall);
  - escape: forward, backward (reverse allowed; always for holonomic), left/right (holonomic),
    shortest <= 2R to a point vertex_margin past the last grown boundary that passes the circle
    test, with a clear exact sweep; first segment, no turn before it;
  - approach: mirrored, runs along the goal heading, no turn at the goal;
  - exits collinear with the next leg at one heading are joined into one exact segment.
- Exact sweep rule (sweepFree; used by plan() and clear()): hull of the footprint at both ends at
  a fixed heading; end footprint must not overlap anything; the sweep keeps model.clearance from
  every box and bound, except that the path's FIRST segment may start and its LAST segment end
  nearer than that, and then the sweep may come no nearer than that pose already is. clear()
  refuses an exact translation whose start and end headings differ.
- Octagon growth: grown boxes are octagons (sides at R, corners cut at 45 degrees tangent to the
  rounded Minkowski sum; <= 8.3 % of R overstatement) instead of rectangles (41 %). Vertices are
  the 8 octagon vertices at R + vertex_margin. Stored as half-planes plus a bounding box.
- straighten(): collinear route corners merged when the joined leg is clear.
- max_vertices: vertex list stops growing at the bound (memory O(V + boxes)).
- collision: convex polygon signed distance (SAT + vertex-edge), swept hull, octagon, Region.
  Removed the now unused Rect inside/crosses (private header).
- Tests: new tests/planner_exit_gtest.cpp (12 tests) and tests/planner_edge_gtest.cpp (12
  tests incl. PlannerCost), both listed in CMake; planner_gtest updated for octagons and exits;
  planner_random_gtest now puts ends by walls and boxes (300 fields x 8 plans).
- docs/investigatr.md rewritten; brain/investiGATR/README.md updated.
- All owned files LF, plain ASCII; clang-format applied to tests, new src code formatted by hand
  in the lead's style (short functions and case labels kept).

### Test results
- `cmake --build $S/wC-brain -j8 --target investigatr_tests --clean-first`: 0 warnings
  (g++ 15.2, Debug, -Wall -Wextra).
- `investigatr_tests.exe`: 98 tests, 98 passed (14 suites). Random avoiding run: ok 1409
  (detoured 826, escaped 336, arrived 308), blocked 418, bounds 362, no path 211 (69 confirmed
  by grid search).
- Lead's tests built in my dir against this planner: actugatr_tests 73/73,
  actugatr_integration_tests 6/6. communigatr_tests compiles.
- ARM syntax check (arm-none-eabi-g++ 13.3.1, gnu++20, -Wall -Wextra) of all 8 investiGATR
  sources: clean.
- Mutation run ($S/wC-mut): 23 distinct defects injected in the C2 code, all killed (4 needed
  added tests: loose ends, end overlap, holonomic left exit, straighten; 1 the 2R limit).
- Cost, host -O2, one core (not V5): Override-like field 0.06-0.55 ms per plan; worst case at the
  default max_vertices 512 (63 boxes, V 506, no route) 18 ms; 128 boxes with V 1026 (bound
  raised) routed 22 ms, no route 97 ms; over the default bound refused in about 0.5 ms.

### CONTRACT ISSUES / notes for the lead
- Spec 8.1 says the swept rectangle is "grown by model.clearance" everywhere. Taken literally
  that refuses every escape for a robot touching the wall (the grown start overlaps the wall).
  Implemented instead: clearance everywhere except next to the path's start and goal poses,
  where the sweep may come no nearer than those poses already are. Documented.
- IMPORTANT (lead's placeholder robot): brain/robot/gatr2_robot.h kFootprint 0.46 m square with
  robot_config kClearance 0.06 gives R = 0.385. On the Override collision boxes the diagonal
  goal gaps pass only for R < 0.313, so the corner pockets are closed: avoiding moves out of a
  corner area (and wall starts there) fail with kNoPath or a start refusal. Measured routes
  pocket->center: side 0.381 m ok up to clearance 0.04; 0.40 m ok at 0.02; 0.457 m never.
  Options: real (smaller) footprint, smaller clearance, or direct moves there; a heading-aware
  planner would be needed for an 18 in robot (not implemented).
- Motion::remaining() still replaces the current segment's start with the measured pose. With
  exact exits that is supported (first-segment allowance); for ordinary segments near a
  corner it can still force a replan on tracking error alone; the replan now escapes along the
  heading instead of failing kStartBlocked, when an exit exists.
- Exact segments keep only model.clearance (less next to start/goal): keep tracking_tolerance
  below clearance; a robot starting against a wall touches it if it drifts backward.
- docs/README.md investiGATR index line is still the old text. Suggested: "investiGATR: path
  planning and the shared Brain types: field, state source, references, motion model, paths;
  planner method, exits by walls, cost and limits".
- PlanStatus still has no "graph too large" value (kNoPath), as the header says.

### Risks
- Circle model is conservative (quantified above). Octagon overstatement <= 8.3 % of R.
- V5 planning time not measured; typical field plans should be a few ms to tens of ms there.
- Not validated on the robot.

## Stage FIX (reviewer findings on C1-C2)

### Reproduced
- All three findings reproduced with the reviewer's probe binaries on the unchanged code
  ($S/rev2-WorkerC-probe2): pocket escape and approach give kNoPath; 3 m wall slide and 2.1 m
  barrier run keep sub-clearance far from the ends; 0.0079 rad heading jump into an exact approach
  with 1.18 mm goal overlap at the arrival heading.

### Done (all three fixed; session restarted mid-stage, partial edits inspected and finished)
- F1 (false kNoPath): findExits keeps every exit: each move the model allows, and along it the
  first point of every circle-free stretch within 2R whose sweep is clear. shortestRoute is a
  multi-source, multi-target A*: g(escape end) = escape length, virtual goal behind every approach
  start at cost + approach length; the direct-pair shortcut uses the same costs. Pocket escape and
  approach now route (backing out west).
- F2 (allowance far from the ends): sweepFree splits a sweep at 2R from a loose start and 2R from a
  loose goal; only pieces within 2R of a loose end get min(clearance, that end's gap); the rest
  keeps the full clearance. Applies to plan() and clear(). A long wall ride is refused (tank along
  a wall: kStartOutOfBounds; holonomic: out, across, in); the barrier start backs out and routes
  around. Behaviour chosen: the reviewer's option 1 (bounded allowance), no wall-riding moves.
- F3 (heading jump into an exact move): shape() keeps the turn onto an exact approach whenever
  > 1e-9 rad; min_turn bounded to 0.1 rad in validConfig (kInvalidRequest above); straightExact
  keeps the start heading. Found and fixed one more path in this stage: joinExact merged a leg
  entered after a dropped (< min_turn) turn into the exact approach, giving one exact segment
  entered up to min_turn off its heading (probe: start (0.6, 2, 0.005), goal touching a box: 5
  mrad jump). joinExact now requires the entry heading (start heading or previous segment end) to
  equal the leg heading.
- Tests: planner_exit_gtest new EveryExitIsTried (pocket, escape and approach), LaterStretches
  AlongOneMove (goal moved off the heading line so the exit logic is really exercised),
  ShortestWholeRoute (exit lengths count; escape and approach), AllowanceOnlyNearTheEnds (wall
  slide lengths around 2R and 4R, barrier), ClearKeepsTheClearanceAwayFromTheEnds,
  TurnOntoAnApproachIsKept (+ the join case). expectExecutable in planner, exit, edge tests and
  executionProblem in the random test require a heading jump <= 1e-9 into every exact segment.
  Random test: 15 % of trials start within min_turn of the line to the goal; refusals checked by
  an independent exit search; kNoPath checked by a grid search from the exit points. Planner
  InvalidRequests covers min_turn 0.1 ok / 0.11 refused.
- planner.h comments only (clear() allowance within 2R, min_turn <= 0.1, search from every exit).
  No name or signature changed.
- docs/investigatr.md: start and goal steps, the exact rule (2R), shaping (exact entered at its
  heading), completeness claim reworded, clear() section (actuGATR passes unmodified planned
  segments, spec 8.7), min_turn bound, cost (re-measured), exit limits, test tables.

### Test results (FIX)
- `cmake --build $S/wC-brain -j8 --target investigatr_tests --clean-first`: 0 warnings.
- `investigatr_tests.exe` / ctest -R "^(Planner|Clearance|Reference|Geometry|Field\.|MotionModel|
  Path\.)": 103/103 passed, 14 suites. Random avoiding run: ok 1409 (detoured 787, escaped 332,
  arrived 316), blocked 398, bounds 400 (no exit confirmed 243), no path 193 (searched 120, from or
  to exits 57).
- In my dir against the current tree: actugatr_tests 81/81, actugatr_integration_tests 6/6,
  communigatr_tests 195/195.
- ARM syntax (arm-none-eabi-g++ 13.3.1 gnu++20 -Wall -Wextra) of all 8 investiGATR sources: clean.
- Reviewer probes on the final code ($S/wC-fixprobe): pocket escape/approach ok, assembled path
  clear; HeavyFuzz 3960 ok, 0 failures, 0 heading jumps into exact segments, 0 sub-clearance points
  > 2R from both ends; LongSubClearanceSlideAlongWall now refused (it asserted kOk only to show the
  bug).
- Mutations ($S/wC-mut2/mutate.py, scratch copies): one exit per end, unbounded allowance, dropped
  turn before approach, min_turn unbounded, first stop only, escape lengths ignored, join entry
  unchecked, straight move skipped: all killed. Approach lengths ignored survives: at a to terminal
  the A* heuristic already equals its approach length, so it differs only when a terminal on
  another ray is nearer with a much shorter approach (near-equivalent).
- Cost (-O2, host loaded by other builds): Override-like 0.003-0.3 ms; pocket exits 0.03 ms; 63
  boxes worst case 10-12 ms; 128 boxes routed 18-22 ms, no route 54-74 ms; over the bound
  0.4-0.9 ms. No regression against C2.

### Notes for the lead
- Planner behaviour change: a GeometricPlannerConfig with min_turn > 0.1 rad now gives
  kInvalidRequest (nobody outside investiGATR sets the config today).
- Straight exact moves longer than 4R along a wall the robot starts against are refused; a tank
  parallel to a wall within R cannot leave it except within that bound.
- Exact segments are always entered at their own heading, so a follower can execute them without
  a heading correction first.
