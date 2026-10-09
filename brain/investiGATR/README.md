# investiGATR

Brain path planning and the types the Brain libraries share: poses, the field,
the independent pose and field source interfaces, movement references, the
motion model and paths.
Plans direct or obstacle avoiding paths for tank and holonomic motion models,
with explicit refusals, and revalidates a path after field corrections. No
PROS, motors, following or threads; actuGATR follows the paths.

`StateSource::robot(now)` supplies pose, freshness and frame identity. A
localizer implementing it needs no field knowledge. `FieldSource::field(out)`
supplies Brain-owned static geometry and any field updates separately.
`PlanRequest` takes the start pose and field explicitly; `Motion` receives the
two sources independently. `PathSink` is an optional generic plan observer.
Static geometry and pose use the same field coordinates. Providers of observed
field updates must share the pose source's frame namespace and controller clock;
replacing a pose backend also requires rebinding those observations.

Build planner and control host tests without the legacy Pi link or its protocol:

```sh
cmake -S brain -B build/brain-core -DGATR2_BUILD_COMMUNIGATR=OFF
cmake --build build/brain-core
ctest --test-dir build/brain-core --output-on-failure
```

- `include/investigatr/`: public headers
- `src/`: library sources, host and PROS
- `tests/`: host tests

Types, planner method, exits by walls, cost, limits and tests:
[docs/investigatr.md](../../docs/investigatr.md).
