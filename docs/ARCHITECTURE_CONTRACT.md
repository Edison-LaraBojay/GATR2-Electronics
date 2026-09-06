# GATR2 / NaviGATR Architecture Contract

## Status and authority

This document defines the intended architecture. It is normative: code,
configuration, and tests are corrected toward this contract. Existing behavior
does not become correct merely because it already exists or passes its current
tests.

The words **must**, **must not**, **required**, and **explicit** describe
architectural requirements. Changing one of these requirements is a deliberate
architecture decision, not an incidental implementation refactor.

## Governing constraint: configurability

**Configurability is the governing architectural constraint of NaviGATR. It is
not an optional feature, convenience layer, or future improvement.**

NaviGATR is a generic localization and spatial-estimation stack. A deployment
may be as simple as tracking wheels plus an IMU, or may combine cameras, 2D or
3D lidar, thermal vision, ToF, UWB, and future sensors that are not represented
in this repository today. Whether a combination is useful is the profile
author's decision. The framework is responsible for composing it without
acquiring intimate knowledge of those choices.

Generic orchestration must not know:

- Which concrete sensors are installed.
- How many tracking wheels exist or what their labels mean.
- Which buses, pins, cameras, or coprocessors a sensor requires.
- Which concrete preprocessing or localization algorithm is selected.
- Whether AprilTags, lidar landmarks, thermal objects, or season-specific game
  elements exist.
- Whether a meaningful implementation or an explicit no-op occupies an
  optional semantic position.

XML determines composition. Plain registered names select concrete factories.
Concrete factories own their configuration schemas, dependency requirements,
payload types, validation, and contextual diagnostics. Generic orchestration
owns stable contracts, execution order, time, status propagation, and standard
containers.

An implementation that works for the current robot but embeds concrete sensor,
hardware, algorithm, or game-object knowledge in generic orchestration is
architecturally incorrect.

## The extension test

Every new resource, sensor, payload, algorithm, and field behavior must answer:

> Can this be added through a concrete type, factory, registration, XML
> configuration, standard record, and standard pipeline contract without
> modifying generic orchestration?

If not, either the stable contract is genuinely missing a universal concept or
concrete knowledge has leaked into a layer that does not own it. That decision
must be resolved explicitly.

## Initialization model

Resources are initialized dependencies, not sensor measurements and not a
runtime pipeline step.

```text
XML <Resources> -> registered resource factories -> ResourceMap
                                                        |
XML <Sensors>  -> registered sensor factories ----------+
                         |
                         v
                     SensorMap

XML pipeline selections -> registered algorithm factories
                         -> configured pipeline implementations
```

Examples of resources include:

- Pico serial connections.
- SPI and I2C buses.
- Camera devices.
- Network transports.
- Shared monotonic-clock services.
- Hardware accelerators.
- Recorded-data sources used by replay.

Multiple sensors may share a resource when the resource's concrete
implementation supports that use.

### Generic builder responsibilities

A generic builder validates only framework-owned structure. It may require:

- `type`, when it is needed to select a registered factory.
- `id`, when the constructed instance must be referenced later.
- Globally required uniqueness and reference rules.

It must not assert that a concrete implementation-specific attribute is
required, optional, or forbidden.

### Concrete factory responsibilities

The selected factory receives its own XML node and the standard construction
context. It owns:

- Its child attributes and nested XML schema.
- Concrete resource and payload types.
- Required dependencies.
- Configuration validation.
- Initialization behavior.
- Contextual failure messages.

After its factory succeeds, the runtime implementation may assume that the
factory-established invariants hold. An SPI sensor is allowed to know that it
requires `SpiBus`. A tracking-wheel preprocessor is allowed to know the labels
and geometry its own schema defines. That knowledge must not leak upward into
the generic builders or runtime loop.

## Standard containers and concrete values

Every generic boundary uses a standard container and standard record. The
value inside a record may be a concrete, implementation-defined type.

One payload is one typed object, not necessarily one scalar:

- `EncoderCount` may contain a count, direction, and hardware metadata.
- `ImuMeasurement` may contain calibrated angular and acceleration data.
- `LidarPointCloud` may contain hundreds of thousands of timestamped points.
- `ThermalImage` may contain an entire calibrated frame.
- A future user-defined structure may be registered without editing the
  generic containers.

Concrete consumers are expected to know the type they require. This is valid
intimate knowledge owned by that implementation. Generic orchestration
transports the record without interpreting its payload.

| Container | Standard value | Purpose |
|---|---|---|
| `ResourceMap` | `ResourceHandle` | Initialized, shareable dependencies keyed by `ResourceId`. |
| `SensorMap` | `SensorExecutable` | Configured sensor callables keyed by `SensorId`. |
| `SensorResultsMap` | `SensorRecord` | Retained sensor publications and their metadata. |
| `ArtifactMap` | `ArtifactRecord` | Preprocessed computational values. |
| `ObservationMap` | `ObservationRecord` or typed observation collection | Facts inferred from artifacts before association. |
| `AssociationMap` | `AssociationRecord` | Accepted observation-to-field-object relationships or explicit abstentions. |
| `FieldState` | `FieldObjectState` | Estimates of configured field objects, including landmarks. |

`ResourceMap` and `SensorMap` store active runtime objects. They do not store
sensor measurements. `SensorResultsMap` is the output of executing the
`SensorMap`.

### Standard record envelopes

Conceptually:

```text
SensorRecord
|- typed payload
|- device measurement timestamp
|- host receipt timestamp
|- sequence/generation
|- validity and freshness
|- health/status
`- diagnostics

ArtifactRecord / ObservationRecord
|- typed payload
|- source identity and provenance
|- measurement/exposure timestamp
|- sequence/generation
|- coordinate-frame information where applicable
|- validity and quality
`- diagnostics
```

An observation payload may contain a collection. A camera frame or lidar scan
can produce many observations. The framework must not invent a globally unique
map entry for every point merely to force all payloads into the same physical
shape.

### Type erasure and templates

Templates may be used internally for checked insertion and retrieval of a
payload or resource. Pipeline interfaces, record envelopes, and heterogeneous
maps remain stable and non-templated.

A closed central `variant` containing every concrete resource and payload type
is not acceptable. Adding a new implementation must not require adding its
types to a generic master list.

The intended usage is conceptually:

```text
concrete implementation requests EncoderCount
    -> receives EncoderCount or a structured lookup failure

concrete implementation requests SpiBus
    -> receives a shared SpiBus handle or a structured lookup failure

generic runtime
    -> never asks whether either value is EncoderCount or SpiBus
```

## Lookup and diagnostic ownership

Typed lookup must not terminate with an opaque `get failed`, `bad_any_cast`, or
equivalent generic exception. It returns either the requested value or a
structured failure.

Lookup may identify:

- The requested id is missing.
- The id exists but contains the wrong concrete type.
- The resource was configured but failed initialization.
- The value exists but is invalid, stale, or unavailable.

The concrete caller owns the useful explanation because it knows why the value
was required.

```text
resources.find<SpiBus>(configured_resource_id)
    -> success: shared SpiBus handle
    -> failure: structured missing/wrong-type/failed-resource result

make_spi_imu(...) adds domain context:
    "sensor 'chassis_imu' requires SPI bus 'pi_spi0', but no resource with
     that id was configured"

or:
    "sensor 'chassis_imu' expected resource 'pi_spi0' to be an SPI bus, but
     that id was constructed as type 'pico_serial'"
```

The XML parser owns XML syntax, a node tree, and source locations. It does not
own every registered implementation's semantic schema. Generic builders own
generic errors; concrete factories own implementation-specific errors. A
custom XML parser is unnecessary solely for good diagnostics if the chosen
parser preserves enough location and attribute information.

Diagnostics should identify, where applicable:

- Active profile.
- XML file and source location.
- Concrete registered type.
- Configured instance id.
- Referenced dependency id.
- Failure category and underlying cause.

### Initialization versus runtime failures

- Missing required resources, wrong resource types, invalid pin assignments,
  invalid geometry, and malformed concrete configuration are startup errors.
  The profile must not partially start.
- A resource that cannot initialize reports its concrete cause. A dependent
  factory adds the dependency chain and its own context.
- Runtime checksum failures, dropped packets, stale values, temporary outages,
  and invalid measurements are represented through standard status records.
- The concrete implementation assigns warning, degraded, or fault severity.
  Generic orchestration never fabricates replacement sensor data.

## Runtime architecture

Sensor acquisition and preprocessing form the common backbone. Motion
estimation and observation extraction then form separate logical lanes. They
merge when external evidence is associated and optionally used to correct the
robot estimate.

```text
SensorMap
    |
    v
Sensor Collection -> SensorResultsMap -> Preprocessing -> ArtifactMap
                                                          |         |
                                                          |         +-> Observation Extraction
                                                          |                    |
                                                          v                    v
                                                    Localization         ObservationMap
                                                          |                    |
                                                          v                    v
                                                Predicted RobotState      Association
                                                          |                    |
                                                          +---------+----------+
                                                                    |
                                                                    v
                                                       Robot Pose Correction
                                                                    |
                                                                    v
                                                       Corrected RobotState
                                                                    |
                                                                    v
                                                          Field Estimation
                                                                    |
                                                                    v
                                                               FieldState
                                                                    |
                                    CommandState + target definitions + robot state
                                                                    |
                                                                    v
                                                          Target Resolution
                                                                    |
                                                                    v
                                                         ResolvedTargetState
                                                                    |
                                                                    v
                                                               Publishing
```

Observation Extraction may run independently of the current pose prediction.
Association often requires the predicted robot pose to determine which
physical world object could have produced an observation. Robot Pose
Correction consumes the prediction it may correct; it is not an independent
replacement for Localization. Field Estimation runs before Target Resolution
so a newly committed landmark estimate can affect the resolved target in the
same logical update.

In the implemented runtime the top-level slots are Command Collection,
Preprocessing, Localization, Field Estimation, Target Resolution, and
Publishing. Observation Extraction and Association execute as private
children of the selected Field Estimation composite, behind the same
standard contracts drawn above; Robot Pose Correction belongs inside the
selected Localization implementation. A parent may be a leaf, an explicit
no-op, or a composite - the coordinator never learns which.

The architectural name is **Observation Extraction** rather than a
camera-specific interpretation of perception. It may produce AprilTag
observations, lidar features, UWB ranges, ToF surfaces, thermal detections, or
future observation types.

## Pipeline contracts

### Sensor Collection

```text
Input:  SensorMap + cycle context
Output: SensorResultsMap
```

It executes each configured sensor and retains its standard record. A sensor
executable has already captured its validated configuration, resource handles,
and implementation state.

The result distinguishes a new publication, no new publication, temporary
unavailability, invalid measurement, and hardware fault.

### Command Collection

```text
Input:  previous CommandState + configured transport + time
Output: CommandState
```

It applies new command generations. When no new command exists, it preserves
the previous command rather than manufacturing an empty command.

### Preprocessing

```text
Input:  SensorResultsMap + time + implementation state
Output: ArtifactMap
```

It converts sensor-native publications into computational artifacts. Examples
include A/B transitions to encoder counts, encoder counts to wheel travel, raw
IMU values to calibrated angular increments, camera frames to rectified frames,
and lidar packets to timestamped point clouds.

Generic preprocessing orchestration does not know which concrete artifacts an
implementation produces.

### Localization

```text
Input:  ArtifactMap + previous RobotState + command/time context
Output: Predicted RobotState
```

It estimates smooth robot motion and maintains the timestamped state history
required to evaluate delayed evidence at measurement or exposure time.

The simple initial implementation uses tracking-wheel motion artifacts and an
IMU heading constraint. Other registered implementations may use different
artifacts without changing the generic contract.

### Observation Extraction

```text
Input:  SensorResultsMap + ArtifactMap + time/context
Output: ObservationMap
```

It infers timestamped facts without deciding which configured or tracked world
object produced them.

### Association

```text
Input:  ObservationMap + Predicted RobotState + prior FieldState
Output: AssociationMap
```

It determines which physical field object and configured feature, if any,
could have produced an observation, and emits immutable associated evidence
with provenance (`FieldObjectPoseEvidence`). Abstention is a valid result
when the evidence is ambiguous or invalid. Association is target-blind:
navigation intent (requested object, allowed features, preferred sources,
activation time) is filtered in Target Resolution, never here.

### Robot Pose Correction

```text
Input:  Predicted RobotState + relevant artifacts/observations/associations
Output: Corrected RobotState
```

This position explicitly changes the robot estimate. Future implementations
may use a trusted absolute landmark map, lidar map matching, UWB anchors, or
another absolute source.

The initial AprilTag behavior does **not** correct the robot. Its configured
Robot Pose Correction implementation is explicitly `type="noop"`, whose output
is the unchanged predicted `RobotState`.

### Field Estimation

```text
Input:  previous FieldState + Corrected RobotState + observations/associations
Output: FieldState
```

It updates external state such as landmark estimates, tracked objects, or
occupancy. It does not own the robot pose, and it runs every cycle
regardless of navigation state: no target request can start, stop, gate, or
reset it. Estimates are field-framed and unbounded - shared global frame
error cancels during relative targeting, so an estimate is never clamped to
or rejected for the nominal field boundary. A fault in a child leaves the
prior FieldState untouched. Multiple accepted measurements of one object in
one cycle fuse deterministically, independent of iteration order.

### Target Resolution

```text
Input:  CommandState + configured target definitions
        + Corrected RobotState + FieldState + published evidence
Output: ResolvedTargetState
```

It converts a semantic target selection into a concrete navigation target.
Examples include a one-time robot-relative displacement, an absolute field
target, and an offset from an estimated landmark. All navigation-intent
filtering of generic evidence lives here: the requested field object, the
route's allowed feature instances and preferred source, freshness, and the
activation time (evidence measured before activation never acquires the
target). Selecting or clearing a target must not reset FieldState or
disable extraction and association.

### Publishing

```text
Input:  configured standard results and states
Output: configured side effects + publishing status
```

It publishes robot state, target state, health, and diagnostics. It must not
silently mutate estimation state.

## Explicit no-op behavior

Every semantic position is explicitly selected in a profile. Deliberate
absence is represented by a real registered no-op with the same standard
contract. Omitting a type must never silently select a meaningful algorithm.

This allows a minimal configuration to use:

```text
tracking wheels + IMU
    -> wheel/IMU preprocessing
    -> wheel/IMU localization
    -> no-op Observation Extraction
    -> no-op Association
    -> no-op Robot Pose Correction
    -> no-op Field Estimation
```

The architecture remains unchanged when more capable implementations are
selected later.

## Robot correction versus landmark estimation

An external observation constrains the relationship between robot and object.
It does not inherently say which estimate must move. The concrete estimator
must declare what is treated as the anchor.

### Trust robot odometry and estimate the landmark

This is the initial AprilTag policy:

```text
Localization:             wheel/IMU prediction
Observation Extraction:   AprilTag observations (continuous)
Association:              physical tag-mount association (continuous)
Robot Pose Correction:    explicit no-op
Field Estimation:         continuous landmark estimation
Target Resolution:        acquire-once landmark-relative target resolution
```

The observation is transformed as:

```text
T_odom_landmark =
    T_odom_robot * T_robot_camera * T_camera_tag * T_tag_landmark
```

Field Estimation folds accepted evidence into the landmark estimates every
cycle. Independently, after a configured number of fresh, mutually
consistent observations measured since target activation, Target Resolution
latches the navigation target from its own privately buffered evidence
window. Wheel/IMU odometry remains smooth and unchanged in both paths.

### Trust an absolute landmark and correct the robot

A future implementation may instead hold a verified landmark map fixed and use
the observation to estimate robot pose. That implementation belongs to a
robot-pose-correcting localization implementation and must explicitly
describe how it updates the robot state or field-to-odometry transform.

### Do not double-count evidence

The same observation must not independently move both robot and landmark unless
a concrete joint estimator explicitly models both states, their covariance,
and the shared measurement. Otherwise the observation is double-counted and
the resulting states may be inconsistent.

## Profiles, targets, and ids

A **profile** defines a complete deployed topology: resources, sensors,
geometry, algorithms, maps, and publishers. It is selected at process startup
and remains fixed for that run.

A **target** is a configured navigation intent within a profile. The Brain
selects it by stable wire id at runtime. Selecting a target does not select an
XML file, infer a wheel layout, or rebuild the pipeline.

- `type` is the plain registered name selecting a factory. It is not a source
  directory, class name, or architectural enumeration.
- `id` identifies one configured instance.
- `*_id` fields are opaque cross-references to configured instances.
- Meaning comes from the selected implementation and configuration, never from
  parsing an id string.

## Explicit implementation status

The contract describes the intended extensible system, not a claim that every
example sensor or algorithm is currently implemented. In particular, the
initial supported AprilTag direction is continuous field estimation and
acquire-once target resolution with robot pose correction absent (no-op). Real camera capture,
tag detection, calibration, and deployment protocols must be verified
independently before that profile is considered operational.
