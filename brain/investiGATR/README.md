# investiGATR

Brain path planning and the types the Brain libraries share: poses, the field,
the state source interface, movement references, the motion model and paths.
Plans direct or obstacle avoiding paths for tank and holonomic motion models,
with explicit refusals, and revalidates a path after field corrections. No
PROS, motors, following or threads; actuGATR follows the paths.

- `include/investigatr/`: public headers
- `src/`: library sources, host and PROS
- `tests/`: host tests

Types, planner method, exits by walls, cost, limits and tests:
[docs/investigatr.md](../../docs/investigatr.md).
