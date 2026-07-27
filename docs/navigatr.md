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

## Fixed semantic pipeline

```text
Sensor Collection -> Command Collection -> Preprocessing
  -> Localization Prediction -> Perception -> Association
  -> Pose Correction -> World Prediction -> Publishing
```

Resources initialize before runtime; they are not a pipeline step. The order
is the framework's, not the document's: slots may appear in any order in XML
and execute in this sequence (there is a test that proves it). A selected
implementation may contain a nested configurable collection, as
`preprocessing/configured_collection` does, but the top-level sequence never
changes and never becomes a user-defined graph.

Each step has a standard input and output contract (`contracts/`); the only
cross-step data path is the standard result maps. Localization prediction
runs before perception so association works against the current cycle's
predicted pose; corrections from associated evidence land in Pose Correction.

## Explicit selection

Every slot names its implementation with `type`, including intentional
absence:

```xml
<Perception type="perception/noop"/>
```

Configuration errors, never silent behavior: missing slot, missing type,
unknown type, missing dependency, incompatible payload type, duplicate id,
duplicate output id. There are no implicit algorithm defaults, and each
category has its own registered noop with documented semantics (preprocessing
produces no artifacts, pose correction passes the prediction through, world
prediction preserves the previous world, publishing publishes nothing
successfully, command collection carries the previous state forward).

## Identity, type, references

| Concept | Purpose | Example |
|---------|---------|---------|
| type | selects a registered factory | `sensor/pico_encoder_channel` |
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

All factories, every category, live in one typed registry under namespaced
keys (`resource/linux_serial_link`, `preprocessor/tracking_wheel_odometry`,
`perception/noop`). Registration happens through explicit `register_*` calls
at startup, aggregated by `registerAll`; nothing depends on static
initializer order. Unknown keys, duplicate registrations, and retrieval with
the wrong signature all fail loudly, and holding the registry grants no
execution authority: the coordinator decides which category it retrieves and
when the result runs.

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
Clock domains are typed and never compared across; camera fusion requires an
explicit conversion service before it lands.

## What stays implementation-specific

The generic runtime does not privilege AprilTags, wheel counts, left/right
names, a camera, an IMU, a field map, VEX wire ids, a publisher protocol, or
one world estimator. Field maps are a typed resource
(`resource/field_map`) consumed only by implementations that reference them.
Brain wire object ids live in publisher and command configuration, not in
`WorldState`. Frame math (`T_a_b` compose/inverse, `FramedPose2D`) is shared
infrastructure; how it is used belongs to the selected implementations.

Adding next season's sensor, perception algorithm, world predictor, or
nested classifier means: implement the standard interface, define private
payload and config types, register the factory, write the XML, add tests.
The executor, the generic builders, the registries, the map storage, and the
unrelated implementations do not change.

See `navigatr_resources.md` and `navigatr_sensors.md` for the registered
type catalogs, and `pi/navigatr/config/` for complete configurations.
