# navigatr architecture

navigatr is the estimation program on the Pi. Sensors, sensor counts, wiring,
shared buses, algorithms, wheel layouts, cameras, world estimation, and
publishers change through XML configuration and separately registered
implementations, not through edits to the generic runtime.

Two deliberately different layers:

```text
Stable framework knowledge          Implementation-owned knowledge
    fixed semantic pipeline             SPI and serial configuration
    ids and references                  Pico protocol channels
    factory registration                tracking wheel geometry
    resource ownership                  camera and AprilTag behavior
    standard inputs and outputs         localization math
    status, timing, diagnostics         publisher wire formats
```

The framework knows that sensors, resources, preprocessing, localization, and
publishing exist. It does not know which particular sensors or algorithms
exist.

## Fixed semantic pipeline, hierarchical implementations

```text
Sensor Collection -> Command Collection -> Preprocessing
  -> Localization -> World Estimation -> Target Resolution -> Publishing
```

Resources initialize before runtime; they are not a pipeline step. The order
is the framework's, not the document's: slots may appear in any order in XML
and execute in this sequence (there is a test that proves it).

Each slot holds one selected implementation behind one contract, and that
implementation may be a leaf, an explicit noop, or a composite that
privately owns a nested pipeline. `configured_collection` nests a
PreprocessingMap of preprocessors; `landmark_world` nests observation
extraction (Perception contract), association (Association contract), and a
landmark estimator with an explicit commit policy. Nested children are
built from the same registry categories with explicit types, run only when
their parent runs, are reachable only through their parent, and can never
leak private intermediates: what crosses a slot boundary is exactly what
the parent declares it publishes. A child fault never partially commits the
parent output. The top-level sequence never changes and never becomes a
user-defined graph, and adding a new composite requires no coordinator
change.

Genericity is selective. Resources, sensors, preprocessing, localization,
and world estimation are open registry categories because the robot is
expected to vary there. Target Resolution and Publishing are focused domain
logic driven by configuration (the target set, the transports); they have
an explicit noop and a configured implementation, not an open plugin
ecosystem.

Each step has a standard input and output contract (`contracts/`); the only
cross-step data path is the standard result maps and states. Localization
runs before world estimation so association inside it works against the
current cycle's predicted pose; any future robot pose correction belongs
inside a localization composite, never bolted on from outside.

## Explicit selection

Every slot names its implementation with `type`, including intentional
absence:

```xml
<WorldEstimation type="noop"/>
```

Configuration errors, never silent behavior: missing slot, missing type,
unknown type, missing dependency, incompatible payload type, duplicate id,
duplicate output id. There are no implicit algorithm defaults, and each
category has its own registered noop with documented semantics (preprocessing
produces no artifacts, world estimation preserves the previous world and
publishes no evidence, target resolution preserves the previous target,
publishing publishes nothing successfully, command collection carries the
previous state forward). Composite children select their noops the same
way.

## Composed profiles

A deployable profile is a `<Configuration>` document naming its parts by
file: one Robot description (resources and sensors, every physical fact),
optional Field data, one Pipeline fragment. Paths resolve relative to the
referencing file; fragment roots are validated; missing files, repeated
includes, `.xml.in` templates, leftover `@...@` placeholder tokens, and
cross-file duplicate ids all fail with the include chain named. The
resolved profile carries its declared id and an FNV-1a digest over every
contributing file, so the running configuration is identifiable exactly.
Wheel geometry lives in the robot description as a `wheel_geometry`
resource; pipeline fragments only reference declared wheels and never
restate a measurement.

## Identity, type, references

| Concept | Purpose | Example |
|---------|---------|---------|
| type | selects a registered factory | `pico_encoder_channel` |
| id | one configured instance | `tracking_encoder_a` |
| sensor_id / resource_id / artifact_id / association_id | reference to an existing producer | `tracking_encoder_a` |
| label | local human-readable diagnostics | `left` |
| geometry / configuration | actual behavior | `position_y_m="0.130"` |

Ids are opaque. Nothing parses them for meaning, nothing searches producers
by implementation type, and renaming an id plus its references changes
nothing. In C++ every identifier is a strong type (`SensorId`, `ResourceId`,
`PreprocessorId`, `ArtifactId`, `FunctionKey`, ...) so a resource id cannot
be passed where a sensor id belongs.

## One FunctionRegistry

All factories, every category, live in one typed registry under opaque
registered names (`linux_serial_link`, `pico_encoder_channel`,
`tracking_wheel_odometry`, `noop`). A name is scoped by the factory
signature, which is what lets every category register its own `noop` while a
duplicate within a category stays impossible. Registration happens through
explicit `register_*` calls that live beside their implementations
(`impl/sensors/register_sensors.cpp`, ...), aggregated by `registerAll`;
nothing depends on static initializer order, and a registration collision
aborts at startup instead of silently keeping the first function. Unknown
keys and wrong-signature retrieval fail loudly, and holding the registry
grants no execution authority: the coordinator decides which category it
retrieves and when the result runs.

The runtime collections are the settled names: `ResourceMap` owns initialized
`ResourceInstance`s, `SensorMap` owns the id-keyed sensor executables in
deterministic order, `SensorResultsMap` owns the latest records, and the
configured preprocessing collection owns a `PreprocessingMap`.

## Lifecycle

```text
parse XML -> register factories -> index and build resources
  -> build sensors -> build pipeline slots, resolving every reference with
     payload compatibility -> freeze -> runtime cycles
```

Construction is atomic: any failure destroys the whole candidate and nothing
partial runs. Resource references resolve lazily with cycle detection, so
declaration order never matters and a dependency cycle reports the full
cycle. Shutdown is reverse dependency order: pipeline executables, then
sensors, then shared resources, via RAII.

Initialization errors carry the XML path, id, type, and the contracts
involved:

```text
/System/Pipeline/Preprocessing/Preprocessor[@id='tracking_motion']:
  ... sensor robot_imu produces sensor.imu_sample but ... requires
  sensor.encoder_sample
```

Parsing is strict where the framework owns schema: malformed numbers, NaN,
infinity, overflow, invalid enums, duplicate or unknown framework-owned
sections are errors, never silent defaults or clamps. Implementation-owned
subtrees stay open; each factory applies the same strictness to its own
schema.

## Data contracts

Records use fixed envelopes with open payloads. `TypedPayload` stores any
coherent struct, returns it only to a reader naming the exact type, and
carries a stable type name (`sensor.encoder_sample`) alongside the in-process
`type_index`; compiler type names are never serialized. Producers declare
`PayloadDescriptor`s at initialization, consumers bind against them
(`SensorCatalog::bind<EncoderSample>(...)`,
`SlotInitializationContext::requireArtifact(...)`), so impossible wiring dies
at build while the algorithms themselves know payload contracts, never
hardware.

Sensor records separate health from data: `Valid` with a publication is a new
sample, `Valid` without one is a healthy quiet cycle, and the stored record
keeps `measuredAt` (device clock), `receivedAt` (host clock, assigned by
Sensor Collection), and a sequence that increments only on new publications.
Nothing erases history; a slow camera does not disappear between frames.
Freshness is policy, not accident: a channel sensor whose link stays open but
goes silent turns `Unavailable` after its configured `stale_after_ms`, and
preprocessing consumes only currently healthy sources, with IMU integration
reseeding across outages longer than `max_gap_ms` instead of integrating
garbage. Clock domains are typed and never compared across; camera fusion
requires an explicit conversion service before it lands.

One fusion rule worth stating: when wheels can solve rotation themselves,
gyro fusion belongs in the solve (`HeadingConstraint`, which conditions the
translation on the gyro heading) rather than overriding heading after the
fact; the prediction-slot `Orientation` input exists for motion sources that
carry no heading of their own.

## Coordinate conventions

One project-owned set of axes, pinned by tests in `se3_gtest.cpp`. Any
heading taken from a VEX drawing is converted once into this convention.

```text
Field frame F (from Audience View)          y
    origin bottom-left inside the field     ^   top
    +x right, +y top, +z up                 |
    heading 0 = +x, +90 = +y, CCW           |
    positive from above                     +------> x
                                          origin   right
```

```text
robot body R: origin = the exact point RobotState tracks (drivetrain
    center of rotation or the chosen odometry reference, not
    automatically the geometric center); +x forward, +y left, +z up
engineering camera Ce: origin at the optical center; +x looking
    direction, +y camera-left, +z camera-up; a level forward camera is
    yaw 0 pitch 0 roll 0, aimed left is positive yaw, tilted down is
    positive pitch
canonical tag surface S: origin at the center of the detector's four
    pose-estimation corners; +x outward normal toward a viewer, +z the
    decoded printed top, +y right-handed completion
```

Every transform is written `T_A_B` (pose of B in A) and chains as
`T_A_C = T_A_B * T_B_C`. XML carries meters and degrees, applied as
`R = Rz(yaw) * Ry(pitch) * Rx(roll)`, right handed; C++ carries radians.
Camera and tag chains stay SE(3) end to end (`math/se3.h`) and project to
planar only after the chain is complete. Detector-native optical axes are
converted once inside perception with two fixed rotations; they never
appear as mysterious angle offsets in camera configurations.

## Odometry frame and field frame

The robot estimate is split: a smooth local odometry frame O that wheel and
IMU prediction updates continuously, and the corrected field frame F that
re-anchors it. `T_field_robot = T_field_odom * T_odom_robot`. A commanded
pose init changes only `T_field_odom`, so odometry-anchored data (latched
targets, exposure-time lookups) keeps its physical meaning across a field
re-anchor. `odometry_epoch` increments when O itself becomes discontinuous
(hard reset, device time regression from a Pico reboot); anything latched
in O is valid only while the epoch matches, and cancellation is the V1
policy. The framework keeps a short host-clock pose history so evidence
with an exposure timestamp is evaluated against the pose at exposure.

Pose history entries carry the time a pose was physically true, not the
loop time it was computed at: wheel data is device stamped, so the
estimator maintains a device-to-host clock mapping (`core/clock_sync.h`,
minimum-latency offset over a sliding window). Observations pair each
device stamp with the actual host receipt of that sample - artifacts carry
`receivedAt` for exactly this - so pipeline delay never masquerades as
clock offset, and the mapping declares itself valid only after a warm-up.
It resets with the odometry epoch when the device clock restarts, and an
epoch change also clears pose history outright: poses from a dead odometry
frame are not history. A clock-map revision that moves a mapped time
backward is clamped to the newest entry explicitly, keeping history
monotonic. Only valid poses enter history, and an exposure timestamp in
the future or further past the newest entry than sensor latency explains
is rejected, never clamped. Motion gates on evidence (maximum angular
speed) evaluate the yaw rate around the exposure time from that same
history, not the rate at processing time. Gyro data survives packet batching
because the telemetry decoder integrates every decoded packet into an
accumulated angle that consumers difference; a latest-rate snapshot alone
would silently drop rotation. Wheel/IMU startup is a lifecycle: while gyro
bias collection runs the wheel baselines rebase continuously, motion above
a small threshold restarts collection, and the first fused solve therefore
never combines stale wheel travel with a short gyro interval.

## Targets and correction gating

Navigation targets are configuration (`target_set` resource): a
landmark-relative target names a landmark, one of its approach frames, a
controlled robot frame (`front_contact`, `rear_contact`, ...), and the
desired controlled-frame pose; a robot-relative target names a delta
snapshotted once per new command sequence. The `target_tracker` world
prediction owns activation (edge triggered by the brain's select command),
the vision policy (`none`, `acquire_once`; `continuous` is reserved),
explicit timeout fallback, and generation stamping: evidence from a
previous target generation or odometry epoch is discarded, never applied.

Acquisition is transactional with one acceptance path. Accepted evidence
buffers privately inside target resolution, at most one candidate per
(camera, frame) - three mounts in one image are one observation, not three
- until the configured number of consistent results from distinct frames
arrives. The lock latches the target and records the window's mean landmark
pose in `TargetState`; that locked landmark is the only camera evidence the
world estimation estimator may fold (`commit="on_target_lock"`, once per
generation, on the following cycle). Evidence the resolver rejected or
never confirmed does not exist anywhere else, so it can never move a
landmark, and the `use_nominal_target` timeout fallback reads the immutable
field map nominal - zero visual correction with zero trace of unconfirmed
evidence. Activation, robot-relative
snapshotting, and association all require a valid robot estimate; a select
command during startup defers until localization is real instead of
latching zeros. The AprilTag detector itself is target gated
(`detect="on_demand"`): the camera stays warm, but detection runs only
while something is acquiring unless the diagnostic `detect="always"` mode
is configured.

Association accepts evidence only through, in order: detector quality
(hamming, decision margin; the reprojection-error and alternate-pose
ambiguity gates, when enabled, reject detections that do not report the
value rather than treating absent as zero), physical plausibility (the tag
in front of the camera, its outward normal toward the lens by at least the
configured `min_facing_cos`, within range), expected mount visibility from
the prior (in front, facing, projecting inside the calibrated image, at
least `min_projected_size_px` across), the translation and heading gates,
and a decisive combined-score margin over the runner-up. Anything less
abstains. Frame identity everywhere is (camera, sequence): sequences are
per device and never compared across cameras. Only the selected target's
landmark ever mutates, and the Localization estimate is untouchable from
outside its own slot: vision corrects or acquires the selected
landmark-derived target, never the wheel/IMU robot estimate.

## Placeholder policy

Runnable `.xml` contains only verified numeric values. `.xml.in` files are
intentionally non-runnable templates whose `@...@` tokens must be replaced
with measured values, never zero to make parsing pass. Calibration-critical
attributes are required (missing is an error, not a default), and elements
carrying measured geometry declare `calibration_status`: UNCONFIGURED is
always an error, provisional runs only under the explicit
`--allow-provisional` bench option, verified always runs.

## What stays implementation-specific

The generic runtime does not privilege AprilTags, wheel counts, left/right
names, a camera, an IMU, a field map, VEX wire ids, a publisher protocol, or
one world estimator. Field maps are a typed resource
(`field_map`) consumed only by implementations that reference them.
Brain wire object ids live in publisher and command configuration, not in
`WorldState`. Frame math (`T_a_b` compose/inverse, `FramedPose2D`) is shared
infrastructure; how it is used belongs to the selected implementations.

Adding next season's sensor, observation extractor, world estimator, or
nested classifier means: implement the standard interface, define private
payload and config types, register the factory, write the XML, add tests.
The executor, the generic builders, the registries, the map storage, and the
unrelated implementations do not change.

See `navigatr_resources.md` and `navigatr_sensors.md` for the registered
type catalogs, and `pi/navigatr/config/` for complete configurations.
