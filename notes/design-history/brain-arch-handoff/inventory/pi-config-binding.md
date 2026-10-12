# naviGATR configuration binding and where a Brain-sent RobotProfile could be applied

Scope: `C:/GATR2-Electronics/pi/navigatr/src` at HEAD (`c2c5654`, tree clean). This was read-only: I edited no files and ran no builds. Paths below are relative to `pi/navigatr/src` unless they say otherwise.

---

## 0. Summary

- **The configuration is bound once, at construction.** `System::build` parses one merged `<System>` XML string. Each stage builder hands the implementation factories non-owning `ConfigNode` views, and the factories copy what they need into closures or objects. After that the `tinyxml2::XMLDocument` is destroyed (`runtime/system.cpp:96`; the comment at `:130-132` says so).
- **Nothing can be replaced at runtime.**
  - No `System` API replaces a stage or slot.
  - `app/main.cpp` never rebuilds the System.
  - `ResourceStore` has no mutation API after it is built.
  - The only lifecycle operation is `System::reset()`, and production code never calls it (only `tests/workers_gtest.cpp:625` and `tests/stages_gtest.cpp:656` do).
- **Robot geometry is copied into models, so changing a shared resource later has no effect.** `tracking_wheel_motion` copies the wheel geometry at `create()`; it does not keep the `WheelGeometryMap` pointer. Encoder CPR and invert are captured in the encoder sensor's closure. Any profile change therefore means rebuilding the objects that consume the geometry.
- **All runtime coupling between stages is by typed id plus payload type (map lookups), never by pointer.** None of the factories I checked keep raw pointers from the build context; they keep only `shared_ptr`s to resources (grep over `impl/`). This makes swapping the sensors, localization, world, target or publishing layer mechanically feasible.
- **The Brain session lives inside the System.** It is in `BrainLinkCommands`, which is the `commands_` slot. Both `System::reset()` and a full rebuild destroy it: a new `pi_instance` and `session_ = 0` (`impl/commands/brain_link_commands.cpp:102-118`, `:32-42`).
- **No `RobotProfile` exists anywhere in naviGATR.** The only "robot profile" text is prose in `docs/setup.md:365,479`.

---

## 1. Build sequence: XML to a running System

### 1.1 Entry (`app/main.cpp`)

1. `:92-93`: `FunctionRegistry functions; registerAll(functions);`
   - `runtime/register_all.cpp:9-17` calls, in order: resources, sensors, commands, localization, world_estimation, target_resolution, publishers.
   - Registry keys are scoped by signature `type_index` (`core/function_registry.h:31-45`). A duplicate registration aborts (`register_all.h:34-39`).
2. `:96-97`: `System::buildFromFile(command.config_path, functions, err, command.build)`.
   - `BuildOptions` holds only `std::map<std::string,std::string> replay` (`runtime/build_options.h:11-17`).
3. `:114-129`: `InspectionService::create(*system, inspection, err)` then `start`. The service stores a raw `System*` (`inspection/inspection_service.cpp:583`).
4. `:146-153`: `system->start(err)`, then the main thread sleeps 20 ms per iteration until SIGINT or SIGTERM. Inline mode is `:134-144`.
5. `:157-160`: `service->stop(); system->stop();`

### 1.2 File resolution (`config/composition.cpp`)

- `System::buildFromFile` (`system.cpp:63-77`) calls `resolveConfiguration(path, resolved, err)`, then `buildFromString(resolved.xml.c_str(), ...)`, then copies `id`, `name` and `digest` onto the System.
- `resolveConfiguration` (`composition.cpp:424-446`):
  - The `Resolver` expands `file=` includes at framework positions only (`childSection`, `:62-104`).
  - `rejectTokens` rejects `@...@` placeholders (`:288-307`).
  - `composeProfile` (`:328-407`) flattens a `<Configuration>` into `<System>`. `Robot/Resources`, `Robot/Sensors` and `Field/Resource` are copied into `System/Resources` and `System/Sensors`, and `Loop`, `Inspection` and `Pipeline` are deep-cloned.
  - The merged document is printed to a string (`:441-443`). The digest is FNV-1a over every file path and its bytes (`:204-205`, `contentDigest` `:411-418`).
- For a plain `<System>` document, `id = path`, the full path (`:438`). The comment at `system.h:156-158` says "file name", which does not match.
- **The string is parsed a second time** in `System::build`. `buildFromString` never resolves includes; `requireResolvedSystem` rejects any `file=` at framework positions (`composition.cpp:420-422`).

### 1.3 `System::build` (`runtime/system.cpp:92-273`)

1. `:94`: `session_id_ = newSessionId()`, random, one per build.
2. `:96-109`: parse; the root must be `System`; `requireResolvedSystem`. `ConfigNode root{root_e}`.
3. `:111-114`: the allowed root children are `Loop`, `Resources`, `Sensors`, `Pipeline`, `Inspection`.
4. `:116-125`: `Loop rate_hz`, default 100 (`system.h:219`), must be greater than 0.
5. `:126`: `parseInspectionConfig(root.child("Inspection"), inspection_, err)`. This includes `RobotBodyVisual`, the display-only footprint (`runtime/inspection_config.h:27-33,110-127`).
6. **Resources**, `:133-139`: `make_resources(root.child("Resources"), functions, options, &warnings_, err)`, then `resources_ = std::move(resources->store); execute_resources_ = std::move(resources->execute);`
   - `make_resources` (`runtime/resource_stage.cpp:87-144`):
     - `ResourceStoreBuilder::index` stores `{id, type, ConfigNode view}` for every `<Resource>` (`resources/resource_store.cpp:27-47`).
     - Replay overrides go through `overrideFactory` (`resource_stage.cpp:110-115`).
     - `buildAll` resolves dependencies lazily with cycle detection (`resource_store.cpp:70-138`).
     - Each factory runs once: `ResourceMakeFunction = std::function<ResourceInstance(const ConfigNode&, ResourceInitializationContext&, std::string& err)>` (`resource_store.h:48-49`).
     - Records that have an executable go into `ResourceCatalog` and `ResourceFunctions`. Configuration-only resources (wheel_geometry, robot_frame_map, field_map, target_set, brain_imu_bench) are skipped (`:125-141`), so they never appear in the `ResourceMap`.
7. **Sensors**, `:141-147`: `make_sensors(root.child("Sensors"), functions, resources_, execute_resources_.outputs(), &warnings_, err)`.
   - `sensor_stage.cpp:40-89`: one factory call per `<Sensor>`. `SensorMakeFunction = std::function<std::optional<SensorExecutable>(const ConfigNode&, SensorInitializationContext&, std::string&)>` (`contracts/sensor.h:47-48`).
   - The result's `outputPayload` goes into `SensorCatalog`.
8. `:149-159`: `Pipeline` must exist, with children `CommandCollection`, `Localization`, `WorldEstimation`, `TargetResolution`, `Publishing`.
9. `:161-166`: a `SlotInitializationContext` is built with `resources=&resources_`, `sensors=&execute_sensors_.outputs()`, `functions`, `warnings`, `loop_rate_hz`.
10. **CommandCollection**, `:205-212`: built through `buildSlot` (`:188-203`: `functions.find<MakeFunction>(type)` then `(*factory)(node, context, err)`). Afterwards `slot_context.commands_type` and `commands_serial` are filled in.
11. **Localization**, `:216-232`: `make_localization(localization_node, functions, execute_sensors_.outputs(), resources_, &warnings_, err)`, then `execute_localization_ = std::move(*localization); robot_ = execute_localization_.state();`. The observation-function ids and outputs are copied into `slot_context` (`:229-232`).
12. **WorldEstimation**, `:236-252`: `make_world_estimation(...)`. It builds its own fresh `SlotInitializationContext` (`world_estimation_stage.cpp:117-121`). Its declared observations and associations, plus the `world_estimation_noop` flag, are then copied into `slot_context`.
13. **TargetResolution and Publishing**, `:254-264`: through `buildSlot`. Then a pairing check: brain_link commands require brain_link publishing (`:265-269`).
14. `:271`: `estimation_stats_.setPeriodTarget(1000/loop_rate_hz_)`.

Construction is atomic: failure returns `nullptr` and the partial System is destroyed (`system.cpp:83-87`).

### 1.4 `make_localization` (`runtime/localization_stage.cpp:133-272`)

- `<Observation id type>`: `RobotObservationMakeFunction` (`contracts/localization.h:102-103`) gets `RobotObservationInitializationContext{sensors, resources, functions, warnings}` (`:145-149`). Duplicate function ids and duplicate output ids are rejected (`:164-196`).
- `<Estimator type>`: exactly one. `StateEstimatorInitializationContext.observations` is the declared outputs (`:257-261`).
- `<History>` becomes `PoseHistoryConfig` (`:209-226`). `<InitialPlacement>` becomes `configured_placement_{origin:"configuration", sequence:1}` (`:227-234`).
- `:270`: `executor.feed_ = std::make_shared<RobotStateFeed>(history)`. **Every build creates a new feed**, and with it an empty history and `RobotState{}` with `odometry_epoch = 0`.

### 1.5 What gets copied at `create()`

| Object | Reads | Kept after build |
|---|---|---|
| `make_wheel_geometry` (`impl/resources/wheel_geometry_resource.cpp:12-75`) | `<Wheel id sensor_id label radius_m position_x_m position_y_m measurement_angle_deg direction>`, all required | `shared_ptr<const WheelGeometryMap>` in the store; no executable; ignores its context (no dependencies) |
| `TrackingWheelMotion::create` (`impl/localization/tracking_wheel_motion.cpp:104-368`) | inline `TrackingWheel`, or `Wheels resource_id` plus `Use wheel_id` | **Copies.** `geometry` is a local `shared_ptr` (`:198-199`). `addWheel` (`:116-143`) copies `radius_m`, `ux = cos`, `uy = sin`, `k_m = x*uy - y*ux`, `sign`, and binds `TypedSensorBinding<EncoderSample>` by `SensorId`. The coupling terms and the observability solve are precomputed (`:310-355`). **No pointer to the WheelGeometryMap is kept.** |
| `BrainImuWheelBench::create` (`brain_imu_wheel_bench.cpp:19-78`) | `Wheels` (exactly 2) and `Imu resource_id` | Copies the wheel parameters (`:51-58`); keeps `shared_ptr<BrainImuBench>` |
| `make_pico_encoder_channel` (`impl/sensors/pico_channels.cpp:137-215`) | `Source resource_id output_id`, `Calibration counts_per_revolution invert`, `Freshness` | `radians_per_count = 2π/cpr × (invert ? -1 : 1)` is captured in a closure `State` (`:155-167`). The binding holds `(ResourceId, OutputId)`. |
| `make_pico_imu_channel` (`:217-275`) | `Calibration invert` | `sign` in the closure |
| `make_attitude_channel` (`impl/sensors/attitude_channel.cpp:15-153`) | `Mounting roll/pitch/yaw_deg` | `q_sensor_body` in the closure |
| `make_pico_telemetry` (`impl/resources/pico_telemetry.cpp:151-287`) | `<Output id channel="0..2" \| "imu">` | The mapping from output id to channel (the encoder logical port) is captured |
| `make_robot_frame_map` (`impl/resources/robot_frame_map.cpp:14-101`) | `<Frame id parent_frame_id><PoseOfChildInParent/>` | Chains resolved to `T_robot_frame`; `shared_ptr<const RobotFrameMap>`. **Kept** by `TagMountAssociation::frames_` (`tag_mount_association.h:73`), `target_set` (`target_set_resource.cpp:123,145`) and `synthetic_rig` (`synthetic_rig.h:114`) |
| Camera resource (`impl/resources/cameras.cpp:118-126,187-210`) | `Calibration/Extrinsic frame_id` | A `FrameId` stamped into every `CameraFramePayload`; the transform itself lives in RobotFrameMap |

---

## 2. Where robot geometry lives and how it reaches localization

| Profile item | XML today | C++ holder | Path to localization |
|---|---|---|---|
| Encoder logical port | `pico_telemetry <Output id="encoder_a" channel="0">`, then `Sensor <Source output_id="encoder_a">` | `ConfiguredOutput.encoder` (pico_telemetry.cpp:146-149); `TypedOutputBinding` | Wheel `sensor_id` gives `TypedSensorBinding<EncoderSample>`, read with `freshStored()` each cycle |
| CPR, electrical reversal | `Sensor/Calibration counts_per_revolution invert` | Encoder closure `radians_per_count` | Publishes `EncoderSample.angle_rad` (unwrapped radians) |
| Wheel radius, offsets (x, y), measurement angle, geometric direction | `wheel_geometry/Wheel`, or inline `TrackingWheel` | `WheelDecl` (resources/wheel_geometry.h:19-28), then copied into `TrackingWheelMotion::Wheel` (tracking_wheel_motion.h:79-100) | Solves in `run()` (tracking_wheel_motion.cpp:713-737) |
| 2-perpendicular vs 3-wheel vs 2-parallel | Which `Use wheel_id` entries, plus `HeadingConstraint`, plus `LateralMotion assume="zero"` | `wheels_`, `heading_.configured`, `zero_lateral_` | 3 wheels without a gyro use `solveFull`; with a gyro, `solvePlanar`; 2 parallel wheels need `LateralMotion` and use `solveForward` (`:320-345`). The bench types are `brain_imu_planar_bench` (2 non-parallel wheels) and `brain_imu_parallel_bench` |
| IMU source | (a) Pico gyro: `pico_imu_channel`, then `HeadingConstraint sensor_id` or `imu_heading_increment`; (b) Brain VEX IMU: `brain_imu_bench` resource plus `CommandCollection/BenchImu` plus the bench observation type; (c) attitude: `attitude_channel`, then `attitude_reference`, then Estimator `<Attitude>` | Sensor closures; `BrainImuBench` mailbox (resources/brain_imu_bench.h:9-37), written by commands (brain_link_commands.cpp:219-221) and read by the bench model | See the Source column |
| IMU invert or mount | `pico_imu_channel Calibration invert`; `attitude_channel Mounting` | Closures | — |
| Logical IMU port | **Does not exist.** Pico telemetry has exactly one `imu` channel (pico_telemetry.cpp:191-193); accel is decoded but not published | — | — |
| VEX Smart Port of the bench IMU | **Does not exist on the Pi.** The Pi gets only `imu_rotation_mdeg` and stamp/flags through `kOpGetStateWithImu`; the Brain side supplies the sample (`brain/communiGATR/src/client.cpp:409`, `config_.bench_imu()`) | — | — |
| Footprint | `Inspection/RobotBody`, display only | `InspectionConfig.robot_body` | **Not used by any estimator** |
| Camera mounts | `robot_frame_map Frame` plus camera `Extrinsic frame_id` | `RobotFrameMap` (shared) | Not localization. World estimation: TagMountAssociation resolves `T_robot_camera` (tag_mount_association.cpp:183); target_set validates controlled frames |

Two sign controls exist per wheel: sensor `invert` (electrical) and wheel `direction` (geometric). The template recommends leaving `invert=false` (`config/shared/robots/gatr2_parallel_wheels_bno08x.xml.in` comment).

---

## 3. Reset, generation and lineage mechanisms

**Record epochs (stage-owned).**
- `resetRecord` sets `epoch + 1` and clears the record (`runtime/record_bookkeeping.h:52-56`).
- `storePoll` stamps `StoredSample.epoch = record.epoch` and restarts `sequence` at 1 (`:42-49`).
- A new `SensorExecutor` or `ResourceExecutor` starts records at epoch 0 (`sensor_stage.cpp:11-16`, `resource_stage.cpp:12-25`).

**Upstream device epoch.**
- `PicoTelemetry::deviceEpoch()` increments when the device clock runs backwards (a Pico reboot) (`pico_telemetry.h:71-74`) and is published as `upstream.epoch`.
- `upstream.clock = telemetry->clockId()`, which is the serial resource id, for example `pico_uart` (`pico_telemetry.cpp:236-239`).
- The gyro accumulator epoch increments on gaps longer than `kGyroGapMs = 250`.

**Sensor restart detection.** `ChannelInput::fresh` sets `restart` when the record epoch or `upstream.epoch` changes (`pico_channels.cpp:77-91`):
- The encoder then increments `discontinuity_epoch` (`:188-191`).
- The IMU shifts `accumulated_epoch += restarts<<32` (`:254-263`).
- Sensors republish `upstream.epoch = stored.epoch + stored.upstream.epoch` (`:122`).

**Observation-model rebase.** `TrackingWheelMotion::run` calls `dropWindow` and rebases on any of the following (`tracking_wheel_motion.cpp:481-502`, `:580-625`):
- record epoch change
- `discontinuity_epoch` change
- clock-domain change
- non-positive interval
- gyro gap
- accumulator-epoch change
- gyro restart

**Clock identity.** `observationClock` returns `"host"` for host-domain stamps, otherwise the one named `Provenance.clock` (`motion_step.cpp:34-56`). If the motion clock identity changes after a pose exists, `odometry_epoch += 1` and the step is rejected (`:437-458`). If device time regresses, `odometry_epoch += 1` (`:463-479`).

**Lineage.** `Provenance.measurement` is carried unchanged through every derived record. `sharesMeasurement` (`motion_step.cpp:23-32`) stops the same gyro from being fused twice.

**History reset.** `RobotStateFeed::publish` clears history whenever `state.odometry_epoch` changes (`state/robot_state_feed.cpp:13`). `PoseHistory` also clears on an epoch change (`pose_history.cpp:28-49`).

**Localization reset.** `LocalizationExecutor::reset` (`localization_stage.cpp:112-131`):
- resets the models and the estimator;
- sets `state_ = RobotState{}` with `odometry_epoch = old+1`;
- publishes to the feed;
- re-arms the configured `InitialPlacement`.

**Placement.** `PlacementEdge::apply` (`motion_step.cpp:70-86`) deduplicates by `(origin, session, sequence)`. It sets `field_from_odom = pose ∘ inverse(odom_pose)`, `anchor_revision++`, `valid = initialized = true`, and records `placement_origin`, `placement_session` and `placement_sequence` in `RobotState` (`state/robot_state.h:62-68`).
- The dedupe state lives inside the estimator (`PlanarStepState.placement`, `motion_step.h:172-186`), not in the executor.
- `System::requestsFrom` forwards the command placement **every cycle** while `init_sequence != 0 && init_session == session` (`system.cpp:275-287`).
- `configured_placement_` applies only when there is no command placement (`localization_stage.cpp:64-68`).
- The noop estimator never applies a placement (`impl/noop/noops.cpp:22-30`), so SET_POSE stays Pending (`brain_link_publisher.cpp:280-285`).

**Field state.** Observed estimates are tagged with `odometry_epoch`. The publisher refuses them when the epoch does not match (`brain_link_publisher.cpp:240-243`).

**Wire exposure.** `odometry_epoch` and `anchor_revision` are cast to `uint32` and sent to the Brain (`brain_link_publisher.cpp:204-205`).

**Brain session.**
- `BrainLinkCommands` holds `pi_instance_`, which is new per construction and per `reset()`; `session_`; the nonce ring; and the `request_id` dedupe (`brain_link_commands.h:52-86`).
- HELLO opens a new session and resets the bench IMU mailbox (`brain_link_commands.cpp:240-266`).
- `BrainImuBench.epoch` increments on reset, on a session change and on stamp regression (`resources/brain_imu_bench.h:17-35`).

**System identity.** `session_id_` (per build), `reset_count_` (per `reset()`), and `configurationDigest`. Inspection clients use these to detect a restart (`inspection_document.cpp:382-389`).

**How a sensor-source change propagates today.**
- A source restart or reboot inside a fixed configuration propagates as: device epoch, then sensor restart, then model rebase, then possibly an estimator epoch bump.
- Changing *which* source feeds a model (a different sensor id, IMU type or wheel set) cannot happen at runtime; it needs a rebuild.
- If a rebuild changes the motion clock (for example the Pico device clock versus host-stamped bench samples), a surviving estimator treats it as a discontinuity. A new estimator starts fresh at epoch 0.

---

## 4. Rebuilding or replacing at runtime

**What exists:**
- `System::buildFromString` and `buildFromFile` are static factories only. Each call produces a new System with new devices and a new `session_id_`.
- `System::reset()` (`system.cpp:597-623`):
  - takes `lifecycle_mutex_`;
  - stops and joins both workers;
  - calls `resetStages()` (`:575-595`), which resets every executor, `commands_`, `target_resolution_`, `publishing_`, clears `robot_`, `command_`, `target_` and every snapshot, and clears `field_handoff_`;
  - increments `reset_count_` and restarts the workers.
  - This is the existing pattern to copy for a controlled swap: stop, mutate on one thread, restart.
- Executors are move-assignable; `system.cpp:139,147,227,247` do exactly that at build.
- `ResourceStoreBuilder::overrideFactory(const ResourceId&, ResourceMakeFunction, std::string&)` (`resource_store.h:97-100`) swaps the factory of an already-declared resource. Only replay uses it today.

**What does not exist:**
- No stage or slot replacement API.
- No reload in `main.cpp`.
- No `ResourceStore` mutation: `records_` is private and only the builder is a friend (`resource_store.h:80-81`).
- No owning or synthesized `ConfigNode`: it is a view over `const tinyxml2::XMLElement*` (`config/config_node.h:23-67`).
- No thread-safe System indirection for inspection.
- No way to seed `odometry_epoch`, `anchor_revision` or the placement identity into a new `LocalizationExecutor` (`state_` is private; only `make_localization` is a friend, `localization_stage.h:71-73`).

**Inspection coupling.** The inspection thread calls these System accessors without synchronization. They are safe today only because the members are immutable after build:
- `resources().records()` (`inspection_document.cpp:160,413,530,709`)
- `sensorCatalog().entries()` (`:556`)
- `localization().estimatorType()` (`:565`)
- `robotFeed()`, which copies the `feed_` shared_ptr member (`:568,575,610`)
- `warnings()` (`:587`)

`reset()` is safe for inspection because it does not reassign these members. **Any swap of the sensor, localization or resource members while inspection runs is a data race.**

**What survives what:**

| | `reset()` | Full rebuild |
|---|---|---|
| Serial and camera devices | kept open; executables reset | closed and reopened. `linux_serial_link` opens once; a failed open is a dead link unless `required` (`serial_links.cpp:111-121`). `pros_usb_link` retries every second (`pros_usb_link.cpp:142-146`) |
| Brain session | **lost** (commands reset) | **lost**; the first drain goes unanswered (`brain_link_commands.cpp:202-209`) |
| Inspection | kept | must be recreated; it holds a raw `System*` |
| `odometry_epoch` | +1 | restarts at 0 (wire regression) |
| `session_id_` | kept (`reset_count` +1) | new |

**To keep acquisition, comms and inspection alive while waiting for a profile, these must persist:**
- the `ResourceStore` device objects (pico_uart, pico_telemetry, the brain link, cameras, the detector, the field map) and `execute_resources_` records;
- `commands_` (`BrainLinkCommands`) and the `BrainImuBench` mailbox;
- the publisher's `SerialLink`;
- `InspectionService` and the System it points to;
- monotonic `odometry_epoch` and `anchor_revision` counters.

A waiting System can be built today:
- Localization with only `<Estimator type="noop"/>` is legal (`config/shared/pipelines/camera_inspection_only.xml`).
- Sensors are optional (`sensor_stage.cpp:48-50`).
- WorldEstimation and TargetResolution have noops.
- A brain_link publisher without `<Health>` needs no sensors.
- Raw encoder counts stay visible in source health at the resource level (`system.cpp:433-454`).
- Encoder sensors cannot be built without a CPR (`pico_channels.cpp:151-154`).

---

## 5. Main loop, tick order and loop-rate constraints

- **Worker mode.** `estimationWorker` (`system.cpp:687-710`):
  1. `estimationCycle`: resources, sensors, `commands_->run`, `execute_localization_(sensors, requestsFrom(command_), ctx)`, `publishSourceHealth` (`:327-342`).
  2. `field_handoff_.put(copy of sensor map)` into a latest-only slot.
  3. `reportingCycle`: target resolution, then publishing against the newest field snapshot (`:344-373`).
  4. Sleep until `next += period`. If it falls more than one period behind, it resyncs rather than bursting (`:702-706`).
- **Field worker.** `waitTake` with a 100 ms timeout, then `fieldCycle`, which reads `execute_localization_.feed()` at call time (`:375-409`, `:712-724`).
- **Inline mode.** `step()` is estimation, then field, then reporting (`:319-325`). `main.cpp:134-144` uses `sleep_until`. With brain_link it prints a warning (`:109-112`).
- **Main thread.** Only a 20 ms polling sleep (`main.cpp:150-153`). This is where an external "apply profile" trigger could run. The estimation worker cannot call `reset()` or a swap on itself: it would join itself.
- **Constraints:**
  - `rate_hz > 0`.
  - brain_link requires `1000/rate ≤ window_ms/2` (`brain_link_commands.cpp:74-82`); with the default 40 ms window that is at least 50 Hz. The default rate is 100 Hz.
  - The bench model's freshness gate is 200 ms (`brain_imu_wheel_bench.cpp:85-88`).
  - Tracking-wheel timing defaults to 20 ms tolerance and 500 ms max pending.
  - The History default is 5 s and 1024 entries.
  - The reply deadline is `previous_read_us + window_us_` (`brain_link_commands.cpp:210-212`). Any worker blackout longer than about 40 ms means requests in that span go unanswered and the Brain retries.

---

## 6. Options for a profile application boundary

These are recommendations, not existing code. All three assume the Pi keeps a Pi-local base XML for transports, device paths, cameras and the field, and turns the typed profile into configuration internally. The Brain never sends XML or paths.

**Common ingredients (all options):**
- **Profile intake.** Add a new op in `BrainLinkCommands::process` (`brain_link_commands.cpp:176-232`) that writes into a mailbox resource, following the `BrainImuBench` precedent (`register_resources.cpp:21-24`, and commands' `require` at `:90-94`). Unlike BrainImuBench, which is same-worker only, this mailbox needs a mutex because a non-worker thread reads it.
- **Synthesis.** Build a `tinyxml2::XMLDocument` in memory (`NewElement`/`SetAttribute`). The vendored tinyxml2 v11 prints doubles with `%.17g` (`third_party/tinyxml2/tinyxml2.cpp:585-587`), so a text round trip is lossless. Ids are generated on the Pi. The existing validation all runs unchanged: required attributes, direction enums, the observability solve, payload binding.
- **Base config.** Declare all Pico outputs (`encoder_a/b/c`, `imu`) in the base, so `pico_telemetry` never depends on the profile. Logical encoder ports then become the choice of which sensor `Source output_id` to bind.

### Option A: whole-System rebuild from a synthesized document, owned by a supervisor

**Touch points:**
- `main.cpp:96-160` becomes a supervisor. It builds a waiting System from the base (noop localization) and polls the mailbox. When a profile arrives it runs `service->stop(); system->stop();`, destroys the System, calls `buildFromString(merged)`, and recreates inspection (or refactors `InspectionService` to go through a lock-protected System handle; `inspection_service.cpp:194,583`).
- To avoid reopening devices, add adopted `ResourceInstance`s to `BuildOptions` (`runtime/build_options.h`) and apply them through `overrideFactory` next to replay (`resource_stage.cpp:110-115`). `ResourceInstance` is copyable and `shared_ptr`-based (`resource_instance.h:66-69`).
- Identity: `configuration_id_` and `digest` set from the profile hash (`contentDigest`, `composition.h:61`).

**Risks:**
- The Brain session is lost on every apply (new `pi_instance`, `session_ = 0`); communiGATR must re-HELLO.
- `odometry_epoch` and `anchor_revision` restart at 0 and go backwards on the wire.
- Browser clients are dropped unless the inspection indirection is added.
- If the build fails, a fallback rebuild to the waiting System is needed. The old System cannot stay running while a candidate that shares single-threaded devices is built (`resources/serial_link.h` thread-safety note).
- The blackout is the stop plus the full build, including detector and camera startup unless adopted.

### Option B (recommended): in-place swap of a profile-bound core inside the System, with a persistent shell

- **Persistent:** `resources_` and `execute_resources_` (base), `commands_`, `InspectionService`, `session_id_`, counters.
- **Replaceable:** a profile overlay of configuration-only resources (wheel_geometry, robot_frame_map), then `execute_sensors_`, `execute_localization_`, `execute_world_`, `target_resolution_` and `publishing_`.

**Touch points:**
- New `bool System::applyProfile(const tinyxml2::XMLElement& generated, std::string& err)` in `runtime/system.{h,cpp}`:
  - Build candidates on the caller's thread using the existing `make_sensors` / `make_localization` / `make_world_estimation` / `buildSlot` logic (factor `buildSlot` out of `build`, `system.cpp:171-203`).
  - Then run the `reset()` stop sequence (`:597-610`), move-assign the candidates, skip `commands_->reset()`, clear `field_`, `detection_frames_` and `field_handoff_` (as `resetStages` does, `:584-594`), and restart.
- Resource overlay: a small API on `ResourceStore` to compose base records with overlay records (`resource_store.h:51-82`). `wheel_geometry` and `robot_frame_map` have no dependencies (their factories ignore the context), so a separate `ResourceStoreBuilder` can build the overlay. `target_set` depends on RobotFrameMap and would have to move into the overlay too.
- Continuity: add `LocalizationExecutor::continueFrom(const RobotState&)` (`localization_stage.h:47-96`) that seeds `odometry_epoch = old+1`, carries `anchor_revision`, `field_from_odom` (compose so the field pose is continuous), `initialized` and the placement identity, and clears `measuredAt` and `measuredAtHost`. A set `measuredAt` would trigger a clock-change discontinuity on the first motion (`motion_step.cpp:437-458`).
- Placement filter: change `requestsFrom` (`system.cpp:275-287`) to forward a command placement only when `(init_session, init_sequence)` differs from `robot_.placement_session/sequence`. Without it, a fresh estimator's empty `PlacementEdge` re-applies the last SET_POSE to a new odometry origin and the pose jumps.
- Inspection safety: guard the swapped accessors (`resources()`, `sensorCatalog()`, `localization()`, `robotFeed()`, `warnings()`) with a mutex, or publish one immutable `shared_ptr` "binding view" that inspection reads.

**Risks:**
- A larger System surface.
- Every consumer of the old executors must be quiescent. The field worker reads `feed()`; the stop/join handles that.
- The new sensor records restart at epoch 0, so source-health ids and epochs visibly restart.
- The publisher must be rebuilt, because its `Health` and `BiasCal` references depend on the profile (`brain_link_publisher.cpp:62-107`).
- If the profile arrives after SET_POSE, the placement applies to the odometry origin at apply time. Ordering proposal: profile first, then SET_POSE, or have the Pi reject SET_POSE until a profile is applied.

### Option C: typed parameter objects handed to the factories (no XML synthesis)

**Touch points.** Split each factory into `parse(ConfigNode) → Config` and `build(const Config&, context)`:
- `TrackingWheelMotion::create` (`tracking_wheel_motion.cpp:104-368`)
- `make_pico_encoder_channel` and `make_pico_imu_channel` (`pico_channels.cpp:137-275`)
- `make_attitude_channel`
- `BrainImuWheelBench::create`
- the chain resolver in `make_robot_frame_map` (`:67-98`)

`WheelGeometryMap` and `RobotFrameMap` are already plain typed structs (`wheel_geometry.h:19-41`, `robot_frames.h:19-28`) and could be inserted directly as `ResourceInstance::asContract<const T>`.

The generic stage builders (`make_sensors`, `make_localization`) and the registry signatures (`contracts/localization.h:102-103,183-184`, `contracts/sensor.h:47-48`) are all `ConfigNode`-shaped. They would need typed overloads or a parallel path.

**Risks:**
- The most code churn.
- Two entry points per factory, with the risk that validation diverges.
- It works against the "each implementation owns its subtree" design.
- It still needs Option B's swap mechanics. By itself it solves only the content path, not the boundary.

**Suggested hybrid.** Use Option B's boundary. Insert `WheelGeometryMap` and `RobotFrameMap` from typed data (Option C for those two only), and synthesize only the small `Sensor` (Source plus Calibration) and `Localization` subtrees from a fixed set of Pi-owned templates:
- 2-perpendicular plus gyro
- 3-wheel with or without gyro
- 2-parallel plus gyro plus `LateralMotion`
- Brain-IMU bench

---

## 7. Explicit non-existence list

- No `RobotProfile` or profile-application code anywhere in naviGATR.
- No runtime stage or slot replacement.
- No config reload or SIGHUP handling.
- `System::reset()` is not used in production.
- No `ResourceStore` mutation, and no store layering or overlay.
- No `ConfigNode` without a tinyxml2 document behind it.
- No logical-IMU-port or VEX-Smart-Port concept on the Pi.
- No footprint used in estimation.
- No encoder zero-offset concept; encoders are only differenced.
- No carry-over of `odometry_epoch`, `anchor_revision` or the placement across a `LocalizationExecutor` or System instance.
- No synchronized System indirection for inspection.
- **No publication of *all* landmark estimates to the Brain.** `BrainLinkPublisher::state` fills exactly one landmark: the one the Brain selected with `object_wire_id` (`brain_link_publisher.cpp:226-255`). `FieldState.objects` holds all of them (`state/field_state.h`). Publishing them all needs new wire and publisher work, which is outside this area.