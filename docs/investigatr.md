# investiGATR

investiGATR is the Brain's path planner and the home of the types the Brain
libraries share. It is portable C++17 in namespace `investigatr` (it also
compiles as gnu++20 in PROS), with no PROS, serial, motor or thread
dependency.

- It resolves destinations against the field origin, a map object or the robot
  pose at command start.
- It plans a complete path, direct or obstacle avoiding, for any motion model,
  and reports explicit success or failure.
- It does not follow paths, run control loops or write motors. That is
  actuGATR: see [actugatr.md](actugatr.md) for the movement commands
  (`goToDirect`, `goToAvoiding`), following, replanning and drivetrains.
- communiGATR implements the state source over the Pi link: see
  [communigatr.md](communigatr.md).

Status: implemented and host tested. Not validated on the robot.

| Path | Contents |
|---|---|
| `brain/investiGATR/include/investigatr/` | public headers |
| `brain/investiGATR/src/` | library sources, host and PROS |
| `brain/investiGATR/tests/` | host tests |

## Units and frames

- Meters, radians, seconds.
- Field frame: origin at the inside bottom-left field corner in the audience
  view, +x right, +y toward the 0-degree wall, heading CCW from +x.
- Robot frame: +x forward, +y left, origin at the reported robot point. A
  `Pose` is that origin in the field frame.
- `wrapAngle` returns (-pi, pi]. `compose(a, b)` is `a * b`: b's translation
  rotated by `a.heading`, headings added and wrapped. `inverse(p)` undoes it,
  and `between(a, b)` is b expressed in a's frame.

## Targets and headers

| CMake target | Sources | Contents |
|---|---|---|
| `investigatr_types` | geometry, field, state_source, reference, motion_model, path | shared types, reference resolution |
| `investigatr` | collision, planner (links `investigatr_types`) | `GeometricPlanner`, `footprintClearance` |

| Header | Contents |
|---|---|
| `geometry.h` | unit aliases, `FrameGeneration`, `Point`, `Pose`, `wrapAngle`, `compose`, `inverse`, `between`, `finite` |
| `field.h` | `ObjectId`, `MapId`, `MapIdentity`, `Bounds`, `Box`, `FieldObject`, `Field`, `boxCorners` |
| `state_source.h` | `RobotStatus`, `RobotState`, `StateSource` |
| `reference.h` | `Reference`, `FieldReferences`, `ResolveStatus`, `ReferencePolicy`, `Resolved`, `resolve` |
| `motion_model.h` | `Footprint`, `MotionLimits`, `MotionModel`, `enclosingRadius`, `valid` |
| `path.h` | `CommandId`, `SegmentKind`, `PathSegment`, `PlanMode`, `Path`, `PathSink` |
| `planner.h` | `PlanStatus`, `PlanRequest`, `PlanResult`, `PathPlanner`, `GeometricPlannerConfig`, `GeometricPlanner`, `Clearance`, `footprintClearance` |

## Shared types

### Field

A `Field` is one complete map plus one complete estimate snapshot. Sources
never hand out a partly assembled field.

| Field | Meaning |
|---|---|
| `generation` | changes with the map or the estimates, 0 = no field |
| `map` | `MapId` (crc32 of the Pi's map document) and the declared revision |
| `bounds` | rectangle the robot footprint must stay inside |
| `objects` | every configured object, sorted by id |
| `frame` | robot field frame the observed estimates are in |
| `received_at` | source clock time the snapshot completed |

`FieldObject`:

- `id`: stable semantic id from the field definition (the `wire_id`), not an
  AprilTag id. Printed tag ids repeat; object ids do not.
- `kind`: landmark or fixed.
- `obstacle`: the box is a planning obstacle. Objects without it are ignored
  by planning; tape and decorations are never obstacles.
- `estimated`: world estimation may correct the pose.
- `reference`: usable as a movement reference.
- `nominal`: field pose from the field definition.
- `box`: rectangle in the object frame. `center` offsets and turns it, so an
  asymmetric element can be enclosed; `length` runs along the box x. A box
  covers the element's largest horizontal extent at any height.
- `source`, `valid`, `pose`, `age_known`, `age`: the current estimate.

Planning places each obstacle box at `pose` when `valid`, else at `nominal`.
`boxCorners(box, owner_pose, corners)` gives the four field frame corners,
counterclockwise.

### State source

`StateSource` is what planning and control read. Both calls are nonblocking.

- `robot(now)`: the newest `RobotState`: pose, age, frame generation, link
  state, and a `RobotStatus` saying why a pose is or is not usable (`kValid`,
  `kNoLink`, `kNoProfile`, `kCalibrating`, `kUnplaced`, `kNoPose`).
- `field(out)`: copies the newest complete field into `out` unless `out`
  already holds that generation; true when `out` holds a complete field.

### References

A destination is a pose relative to a `Reference`:

| Reference | Pose measured from |
|---|---|
| `Reference::origin()` | the field origin, a virtual reference and never an obstacle |
| `Reference::object(id, map)` | a map object's current estimate |
| `Reference::robotAtStart()` | the robot pose when the command begins |

`FieldReferences` supplies `Origin` and `RobotAtStart`. Season names live in
a table generated from the Pi field definition, never in the libraries:

```cpp
// brain/operaGATR/include/field_references.h, generated by navigatr_field_refs
// from pi/naviGATR/config/override/field.xml. Do not edit.
struct Field : investigatr::FieldReferences {
    static constexpr investigatr::MapId kMapId    = 0xF6B3F2C9;
    static constexpr uint16_t           kRevision = 1;
    static constexpr investigatr::Reference NeutralGoal0Center =
        investigatr::Reference::object(5, kMapId);
    // ... one per reference object
};
// Field::Origin and Field::RobotAtStart come from FieldReferences.
```

- Every object reference carries the `MapId` it was generated for. A field
  with another map id gives `kMapMismatch`, so a reference can never pick the
  wrong object after the field definition changes.
- Regenerate the table after a planning edit to the field definition; a Pi
  test fails while the checked-in table is stale. See
  `pi/naviGATR/docs/field_assets.md`.

`resolve(reference, relative, field, robot_at_start, robot_frame, policy, now)`
applies the reference exactly once: `destination = compose(reference_pose,
relative)`.

| `ResolveStatus` | When |
|---|---|
| `kOk` | resolved |
| `kNoField` | object reference and no complete field |
| `kMapMismatch` | reference made for another map |
| `kUnknownObject` | no such object |
| `kNotReference` | object not marked as a reference |
| `kNoEstimate` | object has no valid estimate |
| `kFrameMismatch` | observed estimate in another robot frame (`robot_frame` 0 skips this) |
| `kNotObserved` | estimated object with only a nominal pose while `policy.require_observed` |
| `kStale` | observed estimate older than `policy.max_age` (0 = no limit) |

Origin and RobotAtStart need no field. Fixed objects are always nominal and
pass `require_observed`.

### Motion model

Drivetrain adapters build a `MotionModel`; planning never looks at drivetrain
names.

| Field | Meaning |
|---|---|
| `holonomic` | translation direction independent of heading |
| `turn_in_place` | rotates about the origin without translating |
| `reverse` | non-holonomic: may drive backward along its heading |
| `min_turn_radius` | steering limit, 0 = none; nonzero is refused by the planner |
| `footprint` | distance from the robot origin to each side (front, back, left, right) |
| `clearance` | margin around the footprint for tracking and estimate error |
| `limits` | max speed, acceleration, angular rate and angular acceleration, for followers |

`enclosingRadius(model)` is the farthest footprint corner from the origin plus
`clearance`. A circle of that radius about the origin holds the footprint at
every heading. `valid(model, &why)` checks finite, non-negative sides with an
area and positive limits.

### Paths

A `Path` is a list of typed segments in the field frame, and its `mode`.

- `kTurn`: rotate in place at `start` from `start.heading` to `end.heading`,
  in `turn_direction` (+1 CCW, -1 CW, always the shorter way).
- `kTranslate`: straight line from `start` to `end`. Non-holonomic: the
  heading is the travel direction, plus pi when `reverse`. Holonomic: the
  heading moves linearly from `start.heading` to `end.heading` along the leg.
- `exact`: set on translations whose swept footprint was checked exactly
  rather than with the enclosing circle (exits by walls and boxes, see
  below). Their heading is constant. `clear()` checks them the same way.

A follower finishes each segment before starting the next. That is what makes
a collision-free plan a collision-free trajectory: no corner cutting between
segments. `length()` sums translations. `PathSink::reportPath` receives every
new plan for display; communiGATR forwards it to the Pi viewer.

## Planner

```cpp
#include "investigatr/planner.h"

investigatr::GeometricPlanner planner;          // GeometricPlannerConfig defaults

investigatr::PlanRequest request;
request.mode  = investigatr::PlanMode::kAvoiding;
request.start = robot.pose;                     // field frame
request.goal  = resolved.destination;
request.model = model;                          // from the drivetrain adapter
request.field = &field;                         // complete field from the StateSource

const investigatr::PlanResult result = planner.plan(request);
if (result.status != investigatr::PlanStatus::kOk) {
    // investigatr::toString(result.status); result.blocking names the object
}
```

A planner holds no robot state; each plan depends only on its request. The
same request always gives the same path.

| `PlanStatus` | When |
|---|---|
| `kOk` | `path` is complete; it may be empty when the robot is already there |
| `kInvalidRequest` | unknown mode, non-finite start or goal, invalid model or planner config |
| `kUnsupportedModel` | non-holonomic without `turn_in_place`, or `min_turn_radius > 0` |
| `kNoField` | avoiding with no field, generation 0, non-finite or empty bounds, or a malformed obstacle |
| `kStartOutOfBounds`, `kGoalOutOfBounds` | the footprint there crosses a bound, or it is within R of one and no straight exit gets clear; `blocking` 0 |
| `kStartBlocked`, `kGoalBlocked` | the footprint there overlaps a box, or it is within R of one and no straight exit gets clear; `blocking` = that box's id (lowest when several) |
| `kNoPath` | no route between the ends, or the graph would exceed `max_vertices` |

Overlaps are reported first, start before goal, then missing exits, start
before goal. Avoiding mode never falls back to a direct path.

### Direct mode

No field is needed and none is checked, even when one is passed. Direct mode
does not avoid anything.

- Non-holonomic: turn to face the goal, translate, turn to the goal heading.
  With `model.reverse` and `request.allow_reverse`, the leg is driven backward
  when that needs less total turning (start turn plus final turn); ties drive
  forward.
- Holonomic: one translation, heading moving to the goal heading along it; a
  pure heading change is one turn.
- Translations shorter than `min_segment` and turns smaller than `min_turn`
  are dropped. A dropped translation leaves the path within `min_segment` of
  the goal; an empty `kOk` path means already there.

### Avoiding mode

**Configuration space.**
- R = `enclosingRadius(model)`: the footprint's farthest corner plus
  `clearance`.
- Each obstacle box, at its estimate or nominal pose, grows by R into an
  octagon: its sides move out by R and its corners are cut at 45 degrees,
  tangent to the rounded shape. The octagon holds every point within R of the
  box and overstates it by at most 8.3 percent of R, at its vertices. (A
  plain rectangle overstates corners by 41 percent of R, which closes the
  diagonal gaps between this season's goals; see Limits.)
- The free region is the bounds shrunk by R.
- While the origin is in the free region and outside every octagon, the
  footprint plus clearance is clear at every heading. Turns in place and
  heading changes along a leg therefore need no separate check.

**Graph and search.**
- Vertices: the two route ends, and the octagon vertices for R +
  `vertex_margin`. Vertices outside the free region or strictly inside another
  octagon are dropped, so overlapping boxes act as one wall.
- Edges: segments that do not cross the interior of any octagon. Touching a
  side or passing through a vertex is not a crossing.
- The straight segment between the route ends is tried first; when clear it
  is the answer and no graph is built.
- Otherwise A* on length with the Euclidean heuristic, visibility tested
  lazily. Corners in line with their neighbours are merged into one leg.

**Start and goal.** Each end first gets the circle test above. An end that
fails it is refused only when its exact footprint overlaps something, or when
no exit exists:
1. Exact footprint: the footprint rectangle at the end pose must not cross a
   bound or overlap a box. Contact is allowed, so a robot placed touching the
   wall is accepted.
2. Straight exact move: when either end fails the circle test, and the move
   needs no turn (headings within `min_turn`; for a non-holonomic model the
   goal lies on the start heading line), one translation at the start heading
   whose swept footprint passes the exact rule below is the whole path. Start
   and goal at the same pose give an empty path.
3. Start escapes: straight moves at the start heading, forward, backward (when
   reverse is allowed; always for holonomic models), and for holonomic models
   left and right, at most 2R long, with their swept footprint clear. Along
   each move every stretch of points passing the circle test counts, and the
   escape ends `vertex_margin` past the grown boundary where the stretch
   begins. Every escape is kept, not only the shortest: a short one may lead
   into a closed pocket. An escape is the first segment, with no turn before
   it.
4. Goal approaches: the same, mirrored: points behind, in front of (reverse)
   or beside (holonomic) the goal along the goal heading, each with the
   straight move into the goal. The approach was swept at the goal heading
   only, so the robot turns to it before the approach however small the turn.
   A non-holonomic approach then runs along the goal heading, with no turn at
   the goal.
5. The route search starts from every escape end (or the start) and ends at
   any approach start (or the goal), counting the exit lengths, so it returns
   the shortest whole path. An exit collinear with the next leg at the same
   heading is joined with it into one exact segment, when the robot enters
   that leg at its heading.

Exit segments carry `exact`. Their swept footprint (the hull of the footprint
at both ends, at a fixed heading) keeps `clearance` from every box and bound
side, with one allowance: within 2R of the path's start pose (on its first
segment) and of its goal pose (on its last), it may come as near to each box
and bound side as that pose already is, when that is nearer than `clearance`.
Farther than 2R from both it keeps the full `clearance`. A long straight move
along a wall or box the robot starts against is therefore refused, and the
planner escapes, routes around with the clearance and approaches instead. The
end of every exact segment must not overlap anything.

**Shaping.** Per leg, as in direct mode:
- Non-holonomic: turn, translate, turn, ..., final turn. Reverse is chosen
  over the whole route: least total turning, forward on ties.
- Holonomic: translations only, the heading moving linearly with distance
  along the route to the goal heading. Exits keep their end's heading.
- A route of one leg shorter than `min_segment` is dropped (not before an
  approach). In a route of several legs every leg is kept, because each is
  needed to stay clear.
- An exact segment is always entered at its own heading: the turn onto an
  approach is never dropped, and a leg entered after a dropped turn is never
  joined into an exact segment.

**Self-check.** Every segment of the shaped path goes through the same test
as `clear()` before the plan is returned.

**Why this method.** The field is a 3.6 m square with tens of convex
elements, and the robot follows turn-drive-turn legs.
- Exact for its model: shortest routes among convex polygons bend only at
  their vertices, so the visibility graph finds a route whenever the grown
  configuration space connects the route ends (to within `vertex_margin`).
  With exits, it searches from every escape to every approach. `kNoPath`
  means there is no such route; a way out that is not one straight move at
  the end heading is not searched (see Limits).
- Bounded: see Cost. No grid resolution and no sampling.
- Deterministic: the same request gives the same path.
- Few waypoints: straight legs between corners suit both turn-drive-turn and
  holonomic followers, and every vertex is a place the robot may turn.

### clear() and field corrections

`clear(path, field, model)` runs the planner's tests over every segment:
- ordinary segments: start and end inside the free region, no octagon
  interior crossed; turns are point checks;
- `exact` translations: the swept footprint as above, with the same allowance
  within 2R of the path's start pose (first segment) and goal pose (last
  segment).

It is false for an invalid model or an unusable field, and true for an empty
path. A new field generation that moves an obstacle onto the route makes it
false, and the follower replans.

actuGATR passes the remaining planned segments unmodified, not rebased on the
measured pose: routes run just outside the grown boxes, so small drift would
read as blocked. Tracking error is what `model.clearance` covers; keep the
follower's tolerances below it (see [actugatr.md](actugatr.md)). A measured
start a millimeter into a wall is still accepted as long as the move does not
go deeper, so a replan from there starts with an escape.

### footprintClearance

`footprintClearance(robot, footprint, margin, field)` tests the exact footprint
rectangle at `robot` against every obstacle box and the bounds, for monitoring
and tests.

- Boxes: separating axes find overlap and the penetration depth; separated
  boxes use the exact vertex to edge distance.
- Bounds: the smallest distance from a footprint corner to a bound side.
- `distance` = the smallest signed distance minus `margin` (a rounded grow),
  negative when overlapping; `nearest` = that object's id, 0 for a bound;
  `clear` = `distance >= 0`.
- Invalid input (non-finite pose, negative sides or margin, unusable field)
  is not clear.

Every pose on an ordinary avoiding segment, turns included, has
`footprintClearance(pose, model.footprint, model.clearance, field).distance >= 0`
up to rounding; exact segments follow the rule above.

### Configuration

| `GeometricPlannerConfig` | Default | Meaning |
|---|---|---|
| `vertex_margin` | 0.005 m | vertices and exit ends pushed out beyond the grown boxes |
| `max_vertices` | 512 | graph bound, `kNoPath` beyond it |
| `min_segment` | 0.005 m | shorter single translations are dropped |
| `min_turn` | 0.01 rad | smaller turns are dropped, never the turn onto an exact approach; at most 0.1 rad, else `kInvalidRequest` |

### Cost

- Vertices: V <= min(`max_vertices`, exit ends + 8 x boxes). The route ends
  are the start or every escape end, and the goal or every approach start.
  The vertex list stops growing at `max_vertices`, so memory is O(V + boxes).
- Search: each vertex is expanded once and each pair is tested at most once,
  so at most V(V-1)/2 segment tests, each against every box (a bounding box
  check first): O(V^2 x boxes). The minimum search adds O(V^2).
- Exits: up to 4 moves per end; along each, one candidate per grown box it
  leaves, each with an O(boxes) sweep.

Measured on the host (g++ 15.2 -O2, one core, other builds running), not on
the V5:

| Case | Time per plan |
|---|---|
| Override field (17 elements), 0.30 and 0.381 m robots, tank and holonomic: wall starts, goal sides, corner to corner, refusals | 0.003 to 0.3 ms |
| Closed pocket: escape or approach through a channel, the shorter exit leading nowhere | 0.03 ms |
| 63 boxes, every vertex kept (V = 506), no route: the worst case at the default bound | 10 to 12 ms |
| 128 boxes, every vertex kept (V = 1026, bound raised), routed / no route | 18 to 22 ms / 54 to 74 ms |
| 128 boxes over the default bound | refused in 0.4 to 0.9 ms |

The V5 CPU is much slower than the host; plan outside a tight control loop
or keep `max_vertices` small enough that the worst case fits the budget.

### Limits

- **Conservative circle.** The planner treats the robot as a circle of radius
  R. On this season's field the diagonal gaps between neighbouring goals
  (box corners 0.63 m apart) pass only when R < 0.313 m. Measured on the
  Override collision boxes, a square robot routes from a corner pocket to the
  center at side 0.381 m with 0.04 m clearance, or 0.40 m with 0.02 m, and no
  wider. A 0.457 m (18 in) square robot cannot leave a corner pocket in
  avoiding mode at any clearance, although it would fit through those gaps
  facing along them. Heading-aware planning is not implemented; use direct
  moves there, or a smaller clearance.
- **Octagons** overstate the rounded obstacle by up to 8.3 percent of R at
  their vertices.
- **Exits** are straight moves at the end heading, at most 2R long, one at
  each end of a path. Every direction the model allows and every clear
  stretch along it is tried. Turning in place within R of a wall or box needs
  an exit first, so a pure turn there drives out and back in (or is refused
  for a tank that can only drive along the wall). A tank parallel to a wall
  within R cannot leave it; one straight move along it reaches a goal on that
  line only where the move is within 2R of the start or goal, or keeps the
  full clearance.
- **Planar only.** A box is an element's largest horizontal extent at any
  height; the toggles on the walls are full-height obstacles even though a low
  robot might pass under them. No CAD-mesh collision.
- **Known elements only.** Field elements and their corrections; no balls,
  opponents or other dynamic obstacles.
- **No curvature or speed planning.** `min_turn_radius` is refused; the
  follower profiles speed.
- **Zero-size obstacles.** The link's map validator rejects obstacles without
  a size; other sources may pass one, and the planner treats it as a point or
  a line grown like any box.
- Not validated on the robot.

## Build and tests

Host, from the repository root:

```
cmake -S brain -B build-brain
cmake --build build-brain -j --target investigatr_tests
ctest --test-dir build-brain --output-on-failure -R "Planner|Clearance|Reference|Geometry|Field\.|MotionModel|Path\."
```

On Windows with MSYS2, put `/c/msys64/ucrt64/bin` first on `PATH` and add
`-G "MinGW Makefiles"` to the configure line. PROS apps compile the sources in
place through `brain/gatr2_brain.mk`; its investigatr list names the six type
sources plus `src/collision.cpp` and `src/planner.cpp`.

| Test file | Covers |
|---|---|
| `geometry_gtest.cpp` | wrap, compose, inverse, between, finite |
| `field_gtest.cpp` | find by id, box corners with owner pose and offset |
| `motion_model_gtest.cpp` | enclosing radius about an off-center origin, validation |
| `path_gtest.cpp` | length |
| `reference_gtest.cpp` | origin, robot at start, object composed once, map identity, estimate policy, age, frame |
| `clearance_gtest.cpp` | exact distances (axis, corner, rotated), penetration, containment, bounds, offset origin, nearest id, estimate vs nominal, box offset |
| `planner_gtest.cpp` | direct and avoiding for tank and holonomic models, reverse choice, unsupported and invalid requests, rotated and offset boxes, bounds, blocked start and goal, no route, narrow gap, `max_vertices`, field correction and replan, `clear()` |
| `planner_exit_gtest.cpp` | starts against each wall facing away, along and toward it; a corner; goals beside a landmark facing it and backed onto it; escapes blocked by a second box or a post; the 2R exit limit; exact footprint refusals; straight exact moves; joined exits; `clear()` of exact segments from a measured pose and after corrections, and with the clearance away from the ends; every exit tried (a channel into a closed pocket, escape and approach); later stretches along one move; the shortest whole path counting exit lengths; the allowance only within 2R of the ends (along a wall, beside a barrier); the turn onto an approach kept, and a leg after a dropped turn not joined |
| `planner_edge_gtest.cpp` | boxes touching at a corner, collinear edges and vertices, a long row as one leg, repeated points, goal equal to start, heading wrap at +-pi, boxes across the boundary, merged boxes and a closed ring, zero-size boxes, fields the robot barely fits or does not fit, determinism, a 128-box map against the vertex bound |
| `planner_random_gtest.cpp` | 300 random fields x 8 plans (fixed seeds), with ends by walls and boxes and some starts within `min_turn` of the line to the goal: every path passes `clear()`, is executable with every exact segment entered at its own heading, keeps R on ordinary segments and the exact rule (per box and bound side, allowance only within 2R of the ends) on exits, and every sampled pose passes `footprintClearance`; refusals checked against their reason and an independent exit search, `kNoPath` against a grid search from the exit points; moving a box onto a path fails `clear()`; random direct paths; clearance against brute force |
