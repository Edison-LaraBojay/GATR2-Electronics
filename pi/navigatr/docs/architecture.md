# Runtime architecture

Navigatr is one C++ executable. XML selects resource factories, sensor processors,
localization models, and the field/command/reporting implementations. The
[configuration guide](configuration.md) documents profile selection and file
composition; [resources](../../../docs/navigatr_resources.md) and
[sensors](../../../docs/navigatr_sensors.md) list registered implementations.

## Construction

```text
main XML -> resolve referenced fragments -> System
  make_resources -> ResourceStore + ResourceExecutor
  make_sensors -> SensorExecutor
  CommandCollection factory
  make_localization -> LocalizationExecutor + RobotStateFeed
  make_world_estimation -> WorldEstimationExecutor
  TargetResolution factory
  Publishing factory
```

Each builder parses configuration once, resolves typed references, and captures
initialized state. `FunctionRegistry` maps implementation names to factories by
factory signature. Multiple instances can select the same type. A factory must
copy the configuration it needs; XML node views expire after construction.

`ResourceStore` owns initialized objects such as links, camera devices, detector
handles, and immutable geometry. Its bindings are frozen after construction.
Resources with runtime outputs also provide an executable and declarations of
those outputs. Static field definitions, robot frames, wheel geometry, target
sets, and transport handles can be bound directly without emitting measurements.

Every top-level Pipeline slot must be present: `CommandCollection`,
`Localization`, `WorldEstimation`, `TargetResolution`, and `Publishing`.
Localization selects observation functions and an estimator inside its own
subtree; WorldEstimation selects exactly one `Estimator` by `id` and `type`
inside its subtree. The other slots select an explicit `type`, including `noop`
when unused.
Duplicate IDs, unresolved references, incompatible payloads, and invalid geometry
fail construction. A partially built system is discarded.

## Resource and sensor execution

The coordinator invokes aggregate executors:

```cpp
const ResourceMap& resources = execute_resources_(context);
const SensorMap& sensors = execute_sensors_(resources, context);
robot_ = execute_localization_(sensors, requests, context);
```

The resource executor calls each configured acquisition executable once per
invocation. `ResourceMap[resource_id].outputs[output_id]` addresses a measurement,
including sources with only one output. A Pico telemetry resource drains and
decodes its link once, exposing configured encoder and gyro channels separately.

Each sensor receives the complete read-only ResourceMap and uses its captured
bindings. Sensors convert wire units, normalize electrical signs, apply sensor
mounting corrections, or forward an already usable sample. For example, the
encoder channel produces accumulated shaft angle; the localization observation
model applies wheel radius and placement to obtain robot displacement.

`SensorMap[sensor_id]` and resource outputs use `MeasurementRecord`:

- Source health and a diagnostic are separate from the retained latest sample.
- A sample carries typed data, measurement time, actual upstream receipt time,
  sequence, epoch, and provenance.
- A healthy poll without new data preserves the sample without advancing its
  sequence. Faults do not erase its history.
- Measurement time belongs to the declared source clock. Receipt time belongs to
  the host clock; forwarding a sample does not replace its receipt with poll time.

The maps are retained stage results, not owners of device handles. Consumers must
check health, freshness, and sequence rather than treating every read as new data.

## Localization

```text
SensorMap
  -> configured RobotObservationFunctions
  -> RobotObservationMap
  -> one StateEstimator
  -> settle accepted/rejected observations
  -> publish RobotState + localization status + history
```

Localization owns observation state, estimator state, and history. It provides
`RobotStateFeed` for synchronized snapshots and timestamped lookups. Localization
is the only writer; consumers use its snapshot and lookup operations.

Current observation implementations are:

| Type | Inputs and result |
|---|---|
| `tracking_wheel_motion` | Configured wheel geometry and encoder sensors, optionally a gyro heading constraint; produces a body-motion increment over an interval. |
| `imu_heading_increment` | One IMU sensor and bias settings; produces a heading increment over an interval. |
| `attitude_reference` | One attitude sensor; produces a timestamped quaternion observation. |

Three suitably placed tracking wheels can solve planar motion. Two wheels need
a heading constraint. The model validates the geometry, aligns intervals, handles
source restarts, and does not bridge invalid spans. Gyro bias calibration belongs
to these observation models. Configuring the same gyro independently twice does
not create independent information: every sample carries the acquisition output
it came from (`Provenance.measurement`), and the estimators use that lineage,
not the configured sensor id, to detect overlapping contributors.

Observations remain pending until the estimator explicitly accepts or rejects
them. Functions receive the disposition through `settle`; this prevents a quiet
poll or retried observation from integrating the same movement twice. Payloads
are typed contracts, not arbitrary metadata that the estimator must interpret.

The implemented `planar_motion_integrator` selects one `BodyMotionIncrement`, an
optional `HeadingIncrement`, and an optional `AttitudeObservation`. A heading
supplies the rotation used for integration only when its support equals the
motion window exactly on one named clock; partial support from the window start
accumulates while the motion waits, bounded by `max_wait_ms`, and anything else
is rejected with the reason. The estimator checks measurement lineage overlap,
clock identity by name (a device domain alone identifies nothing), and
epoch continuity, integrates body motion into the previous odometry pose, and
derives velocity from displacement and elapsed time. These rules live in
`motion_step` and are shared with `weighted_planar_fusion`. It holds pose when no usable motion arrives. It is not a
multi-source statistical fusion filter and does not predict ahead using the
previous velocity. Another algorithm can implement the `StateEstimator` contract.

`weighted_planar_fusion` fills the same slot with an explicit uncertainty
model: configured one-sigma noise for the wheel increment and the gyro, a
precision-weighted rotation, translation re-solved through the wheel
geometry coupling the wheel model publishes, and a pose covariance
propagated with the heading-to-position coupling. Shared sources, interval
mismatches, repeated stamps and resets follow the integrator rules. See
[localization fusion](localization_fusion.md).

`RobotState` contains continuous odometry pose, a field anchor, velocity, yaw rate,
validity, effective measurement time, and separately timestamped attitude. Field
placement changes the anchor while preserving odometry. A hard reset changes the
odometry epoch. Only an advance in effective pose time enters pose history;
attitude has its own history so tilt updates do not invent new planar movement.
History lookup uses bounded storage, binary search, shortest-arc yaw interpolation,
and quaternion interpolation, with explicit age and gap failure results. Epoch
changes clear history, and association checks the returned odometry epoch.
See [coordinates](coordinates.md) for the transform and timing conventions.

## World estimation

```text
<WorldEstimation><Estimator id="goals" type="apriltag">...</Estimator></WorldEstimation>
  make_world_estimation -> WorldEstimationExecutor (one captured estimator)
  field_out = execute_world_({sensors, robot, history, previous_field, context})
```

The coordinator drives the selected estimator through one contract.
`FieldEstimationInput` carries the entire read-only `SensorMap`, the latest
`RobotState`, pose-history lookups, the previous `FieldState`, and the execution
context (host clock, invocation, diagnostics). `FieldEstimationOutput` carries
the updated `FieldState`, the observation and association maps the estimator
publishes, a status, and a diagnostic. The executor drops any observation or
association id the estimator never declared (or whose payload contradicts the
declaration) and records the run under `WorldEstimation/<id>` in the field
diagnostics. It never learns what steps run inside the estimator.

Exactly one `Estimator` is configured. `register_world_estimation` registers
`noop` (previous field carried forward, nothing published) and `apriltag`;
another selectable implementation needs its class, a registration line, and a
build entry, not a coordinator change.

The `apriltag` estimator privately owns a fixed serial subpipeline over one
camera, built from the field references it reads at construction:

```text
SensorMap -> ObservationExtraction (AprilTagObservationPerception) -> ObservationMap
          -> Association (TagMountAssociation, optional)          -> AssociationMap
          -> LandmarkEstimation (LandmarkEstimator)                -> FieldState
```

Observation extraction runs the detector over each new frame of the configured
camera. Association uses the exposure-time robot pose and attitude, camera
mount, configured tag mounts, and geometric gates to generate
`FieldObjectPoseEvidenceSet`; an optional trace retains candidate decisions for
inspection. Without an `Association` the decodes still publish (camera
inspection before the camera has a mount frame) and the landmark estimation
must be `commit="never"`. Payload types are checked when the estimator is
built; a step fault leaves the previous field untouched and publishes nothing.

Landmark estimation seeds all configured landmarks from the nominal field map.
With `commit="always"`, accepted evidence updates those objects; `commit="never"`
retains nominal/previous state while still publishing detections and associations.
The `blend` parameter controls interpolation from the previous estimate.
Observing a goal changes its estimate, not robot localization. World estimation
runs independently of whether a navigation target is requested.

`FieldSnapshot` contains FieldState, published observations/associations, timing,
and status. It is an immutable published snapshot. See
[landmarks](landmarks.md) for association, retention, and target/report semantics.

## Commands, target resolution, and publishing

Command collection produces `CommandState`: placement requests, requested object
ID and command sequence, and stream control. The `vex_brain_serial` implementation
parses the shared command-frame codec; diagnostic profiles can use `noop`.

`configured_targets` resolves targets from `target_set`. It can latch a desired
robot pose from a relative movement, a landmark estimate, or an `acquire_once`
visual acquisition. This stage owns target lifecycle; it does not alter the field
estimate. The Brain remains responsible for motor control.

The `vex_brain` publisher writes the existing pose frame. For a requested object,
a matching latched target takes precedence and supplies a desired robot pose.
Otherwise a configured FieldObject mapping supplies its estimated absolute pose.
Both use full field heading. The browser's heading error from nominal is a
separate inspection value, not the wire object's heading convention.

The Linux serial link's optional `DriverEnable` holds DE high while the link
exists. It does not implement half-duplex transmit/receive turnaround. A command
parser in software does not make the HAT's shared RS-485 path bidirectional.

## Scheduling and lifecycle

Default execution uses two workers:

1. The estimation worker polls resources and sensors, collects commands, updates
   localization, and runs target resolution/publishing against the newest field
   snapshot. It hands a copied SensorMap to the field worker through a bounded,
   latest-wins mailbox.
2. The field worker runs world estimation on the latest available sensor
   snapshot and publishes its result. Slow detection can skip intermediate
   snapshots without blocking the estimation worker.

Resource acquisition is still invoked from the estimation worker. The libcamera
backend receives camera requests asynchronously and exposes the newest frame;
there is no generic worker per resource or sensor. A newly added blocking resource
can therefore slow localization.

`--inline` runs estimation, field estimation, and reporting serially for replay
and tests. The optional inspection service reads published snapshots at its own
rate, serving JSON and JPEG previews over loopback HTTP/WebSocket. Slow clients
are skipped rather than stalling an estimator.

`stop()` joins workers before their dependencies are destroyed. `reset()` stops
workers, resets stages and shared resources, changes history/source epochs, clears
published state, and resumes workers when appropriate. Snapshot readers use
synchronized copies; inspection never calls mutable estimator implementations.

## Source map

| Area | Source |
|---|---|
| Coordinator and workers | [system.cpp](../src/runtime/system.cpp) |
| Resource/sensor executors | [resource_stage.cpp](../src/runtime/resource_stage.cpp), [sensor_stage.cpp](../src/runtime/sensor_stage.cpp) |
| Localization executor | [localization_stage.cpp](../src/runtime/localization_stage.cpp) |
| Localization contracts and payloads | [localization.h](../src/contracts/localization.h), [robot_observations.h](../src/payloads/robot_observations.h) |
| State estimators | [planar_motion_integrator.cpp](../src/impl/localization/planar_motion_integrator.cpp), [weighted_planar_fusion.cpp](../src/impl/localization/weighted_planar_fusion.cpp) |
| World estimation executor | [world_estimation_stage.cpp](../src/runtime/world_estimation_stage.cpp) |
| AprilTag world estimator | [apriltag_world_estimator.cpp](../src/impl/world_estimation/apriltag_world_estimator.cpp) |
| Target lifecycle | [configured_targets.cpp](../src/impl/target_resolution/configured_targets.cpp) |
| Brain output | [vex_brain.cpp](../src/impl/publishing/vex_brain.cpp) |
