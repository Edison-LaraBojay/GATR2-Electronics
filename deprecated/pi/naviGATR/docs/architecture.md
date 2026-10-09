# Runtime architecture

Navigatr is one C++ executable. XML selects resource factories, sensor processors,
localization models, and the field/command/reporting implementations. The
[configuration guide](configuration.md) documents profile selection and file
composition; [resources](../../../docs/navigatr_resources.md) and
[sensors](../../../docs/navigatr_sensors.md) list registered implementations.

With a Brain-profiled configuration (`<BrainProfile>` under `Localization`,
the primary path) the XML names only the Pi's devices, wired ports and model
tuning. The localization models come from the robot profile the Brain sends,
built at runtime from typed data; see [Brain robot profiles](brain_profile.md)
and the section below.

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
| `tracking_wheel_motion` | Configured wheel geometry and encoder sensors, optionally a gyro heading constraint and an explicit zero-lateral-motion assumption for forward-only wheels; produces a body-motion increment over an interval. |
| `imu_heading_increment` | One IMU sensor and bias settings; produces a heading increment over an interval. |
| `attitude_reference` | One attitude sensor; produces a timestamped quaternion observation. |
| `brain_imu_planar_bench`, `brain_imu_parallel_bench` | Two wheels and the Brain VEX IMU mailbox (bench, arrival-time pairing); produces a body-motion increment whose rotation is the VEX IMU's. |

Three suitably placed tracking wheels can solve planar motion. Two wheels need
a heading constraint. Wheels that all measure forward, such as two parallel
wheels, also need `<LateralMotion assume="zero"/>`: sideways motion is then
assumed, not measured, and each wheel's travel is corrected by its lateral
lever arm times the gyro rotation. The model validates the geometry, aligns intervals, handles
source restarts, and does not bridge invalid spans. Gyro bias calibration belongs
to these observation models. Configuring the same gyro independently twice does
not create independent information: every sample carries the acquisition output
it came from (`Provenance.measurement`), and the estimators use that lineage,
not the configured sensor id, to detect overlapping contributors.

Every IMU bias path and the stationary status share one `StationaryWindow`
(`impl/localization/stationary_window.h`): new samples only, restarted by a
gap, a source restart or movement, qualified by elapsed sample time
(`window_ms`, 2 s unless configured, never zero). The
first qualified window gives the gyro bias; later ones maintain it in
bounded steps; a gyro restart invalidates it. While a window stays
qualified the executor reports zero velocity and the stationary health bit;
the pose is left alone. This is gated stationary handling, not a zero
velocity filter update: no estimator here models velocity with uncertainty.
See [Brain robot profiles](brain_profile.md#calibration-and-stationary-handling).

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

## Commands and publishing

Command collection produces `CommandState`: the current Brain session, the
newest placement (`init_pose`, `init_session`, `init_sequence`), the robot
profile status, the latest reported path, a target selection that only
in-process callers set (`object_requested`, `object_wire_id`,
`object_sequence`), and the reply owed for this cycle's request
(`BrainReplyContext`). Implementations are
`brain_link` and `noop`; `noop` keeps the Pi standalone on a bench.

The Brain talks to the Pi with the request/reply protocol in
[Brain link v4](../../../docs/interfaces.md): the Brain asks,
the Pi answers each request at most once, and nothing is sent unasked. The
`brain_link` pair implements the Pi side:

- `brain_link` command collection drains its serial link once per cycle,
  stamps every read with the link's microsecond clock, and processes only the
  newest request the drain completed. It owns the session rules: a random
  nonzero session per HELLO, a HELLO retry answered with the same session
  until another request is accepted, the last four opening nonces rejected as
  stale, per-session request_id dedupe with 16-bit wraparound, and SET_POSE
  and CONTROL records so a retry is answered, never applied twice. It
  also holds `pi_instance`, a random nonzero id that is new for every process
  start and every `reset()`. It computes the reply window from its read
  timestamps; see [Bus ownership and timing](../../../docs/interfaces.md#bus-ownership-and-timing).
- A new session clears only client state: the dedupe records, the bench IMU
  mailbox and the reported path. It never touches `init_*`, localization,
  the profile or the field documents, so a new session alone never relocates
  the robot.
- `System::requestsFrom` passes the command placement to localization only
  while `init_session` equals the current session, so a new session withdraws
  a placement not applied yet. `PlacementEdge` applies a placement once per
  (origin, session, sequence) and records that identity in `RobotState`
  (`placement_origin` `command` or `configuration`, `placement_session`,
  `placement_sequence`). A configured `InitialPlacement` is
  (`configuration`, 0, 1).
- With a Brain profile host, PROFILE_WRITE stages the profile, PROFILE_APPLY
  hands it to the System, CONTROL (recalibrate, reinitialize, Pico IMU
  reinit, acquisition restart) and READ_WHEELS go to the running profile,
  and SET_POSE is NotReady until a profile runs. PATH_REPORT is stored for
  inspection only.
- `brain_link` publishing writes only when the command slot left a reply
  pending, once, through the link's windowed write. SET_POSE is `Ok` only
  when `RobotState` reports that exact placement applied, otherwise
  `Pending`. GET_STATE carries the robot pose and flags, the robot
  measurement age, health bits, the profile status, and the ids of the
  current field documents. With a `Field`, READ_DOC serves the field map
  and field estimate documents in chunks: every configured object, nominal
  or observed (see [landmarks](landmarks.md#field-documents)).

Construction enforces the pairing: `brain_link` command collection needs
`brain_link` publishing, and that publisher needs `brain_link` command
collection on the same `Serial` resource. A profile with `brain_link`
command collection must also satisfy `1000 / Loop rate_hz <= window_ms / 2`.
The factories read the loop rate, the command type and its serial resource,
and the Brain profile host from `SlotInitializationContext`.
Publishing receives the diagnostics through `PublishingInput` and counts its
writes in the link's `LinkStats`.

A `linux_serial_link` with `DriverEnable` runs RS-485 half duplex: it listens
while idle and drives the transceiver only while sending a reply, releasing it
once the transmitter is empty. The sequence is in `transport/half_duplex` over
a small port interface; see [linux_serial_link](../../../docs/navigatr_resources.md#linux_serial_link).

## Brain robot profiles

A Brain-profiled System starts waiting: noop localization, while resources,
the Brain link, world estimation, publishing and inspection run. On
PROFILE_APPLY the System checks this Pi's capabilities, writes Sensors and
Localization subtrees from fixed Pi templates (ids generated on the Pi; the
Brain never sends XML) and builds the candidate at once through the ordinary
factories, touching nothing that runs. The swap happens at a controlled
boundary in the `reset()` pattern: the main thread calls
`applyPendingProfile()` every 20 ms in worker mode (workers stop, the
candidate moves in, workers restart), and `step()` does it first inline. The
resources, the commands slot with its session and `pi_instance`, world
estimation, publishing, the feed and inspection survive. A new profile
advances the odometry epoch, clears history, leaves the robot unplaced and
withdraws every earlier placement request; re-applying the running profile
changes nothing. Inspection reads the running binding through one immutable
`BindingView`.

Every cycle the sources the running profile uses are watched
(`runtime/sensor_loss.h`): its encoders, and the Pico IMU or the Brain VEX
IMU mailbox when the profile uses one. A used source that is stale longer
than `sensor_loss_ms`, restarts, or (Pico IMU) reports itself not ready ends
pose continuity like a new profile does, and the Brain must place again.
After localization runs, the same happens when an observation function
reports a new dropped interval (`ObservationReadiness::dropped_intervals`):
measured motion it had to discard, such as a gap past its own limit or
movement while its gyro bias calibrated.
With `<Pico resource_id>` on the CommandCollection, CONTROL 3 and 4 run on the
Pico through the `PicoControl` contract. Lifecycle events go to a bounded log
that inspection shows. Details: [Brain robot profiles](brain_profile.md).

## Targets

`configured_targets` resolves targets from `target_set`. It can latch a desired
robot pose from a relative movement, a landmark estimate, or an `acquire_once`
visual acquisition. A selection activates the target with that target
`wire_id`; brain link v4 has no selection op, so only in-process callers
select today. Targets are Pi internal: inspection shows
them, the brain link never sends them. This stage owns target lifecycle; it
does not alter the field estimate. The Brain remains responsible for motor
control.

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

A Brain request is read by command collection and answered by publishing in
the same estimation cycle, from that cycle's robot state and the newest
completed field snapshot, so a reply never waits for camera processing. The
reply write blocks the estimation worker for about the frame airtime: up to
11.1 ms for a 128-byte reply at 115200 baud, about 13 ms with the transmit
margin. Over USB the write waits at most 5 ms of backpressure.

`--inline` runs estimation, field estimation, and reporting serially for replay
and tests. Field estimation then runs between the Brain request and its reply,
so the brain link needs the default threaded mode; the executable warns when
`--inline` is used with it. The optional inspection service reads published
snapshots at its own rate, serving JSON and JPEG previews over loopback
HTTP/WebSocket. Slow clients are skipped rather than stalling an estimator.

`stop()` joins workers before their dependencies are destroyed. `reset()` stops
workers, resets stages and shared resources, changes history/source epochs, clears
published state, and resumes workers when appropriate. For the brain link it
draws a new `pi_instance`, forgets the Brain session, and clears `CommandState`
(`init_sequence` restarts at 0); the Brain sees the new `pi_instance` and opens
a new session. A Brain-profiled System returns to waiting for a profile.
Snapshot readers use synchronized copies; inspection never calls mutable
estimator implementations.

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
| Brain requests and sessions | [brain_link_commands.cpp](../src/impl/commands/brain_link_commands.cpp), [command_state.h](../src/state/command_state.h) |
| Brain replies | [brain_link_publisher.cpp](../src/impl/publishing/brain_link_publisher.cpp) |
| Field documents | [field_documents.cpp](../src/impl/publishing/field_documents.cpp) |
| Brain profile templates and checks | [brain_profile_builder.cpp](../src/runtime/brain_profile_builder.cpp) |
| Stationary window and gyro bias | [stationary_window.cpp](../src/impl/localization/stationary_window.cpp) |
| Sensor loss and stationary precheck | [sensor_loss.cpp](../src/runtime/sensor_loss.cpp), [stationary_precheck.cpp](../src/runtime/stationary_precheck.cpp) |
| Inspection documents | [inspection_document.cpp](../src/inspection/inspection_document.cpp) |
| Half-duplex transmit | [half_duplex.cpp](../src/transport/half_duplex.cpp), [serial_port.cpp](../src/transport/serial_port.cpp) |
