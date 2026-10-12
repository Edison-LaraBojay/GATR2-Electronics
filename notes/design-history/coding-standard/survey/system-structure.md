# Survey: system structure, pipelines, standard I/O, season reuse, and the naviGATR gripe

Input for the GATR2 coding standard. Read-only survey of `C:/GATR2-Electronics` at `ae457ee`
(clean tree, 2026-09-28). Every claim cites `path:line` with a short quote; paraphrase is marked.

Method:
- file and folder sizes from `git ls-files` and `wc -l`;
- class/struct shape from a libclang AST scan of every tracked `.h/.cpp` outside `third_party/` and
  the PROS kernel headers (718 type definitions);
- function size and complexity from `lizard` (NLOC, CCN);
- manual reading of every coordinator, stage, contract and registry file in naviGATR, and of the
  entry points of every other component.

Scratch scripts and raw output: `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/standard/system-structure/`
(`types_scan.py`, `types.json`, `move_map.py`, `move_map.txt`).

## 0. The short version

1. naviGATR's pipeline design is sound and already generic: stages, typed result maps, declared outputs,
   explicit registries, atomic build.
   - What is broken is **where it lives**. The per-cycle order is 121 lines (`system.cpp:1020-1140`)
     inside a 1450-line file.
   - 569 lines (39%) of that file are Brain-profile, Pico-control and recovery logic.
   - One stage is spread over 6 to 8 folders (`contracts/`, `runtime/`, `impl/<stage>/`, `impl/noop/`,
     `payloads/`, `state/`, `resources/`).
   - The only code map is at line 362 of a 381-line doc.
2. The fix is mostly **moves plus a split**, not a redesign:
   - numbered stage folders (`src/1_resources` ... `src/7_publishing`), each holding its contract,
     builder, registration, output types and `impl/`;
   - a `src/pipeline/` coordinator whose construction lives in `build.cpp` and whose per-cycle order
     lives in `cycle.cpp`;
   - a `src/brain_profile/` folder for the Brain-profile feature;
   - `system.cpp` split into 8 files **without changing a single function body**;
   - 142 of 201 src files move, touching 522 include lines in 166 files. Section 1.3 has the complete
     move map.
3. Standard I/O exists but is **uneven**.
   - Resources and sensors use a functional contract (a struct of `std::function`).
   - The other five stages use virtual interfaces.
   - Inputs mix `ExecutionContext` with loose `now` and `Diagnostics*`.
   - Three generic contracts carry Brain-link wire types: `CommandState`, `PublishingOutput` and
     `SlotInitializationContext`.
   - A new sensor, localization model or world estimator plugs in without touching the coordinator.
   - A new command source or publisher cannot: `system.cpp` string-compares `"brain_link"`.
4. Season leakage is small:
   - viewer id prefixes (`field_scene.js:293`, `:690`);
   - the primary robot configs live in the season folder `config/override/`;
   - season names appear in comment examples in generic headers.

   Implementation leakage (Brain link, Pico, HAT, AprilTag) into generic code is the bigger problem.
5. Over-generalization:
   - `SpiBus`/`SpiDevice` have zero production implementations.
   - `Perception`/`Association` have one private implementation each.
   - `BrainProfileHost` is a pure forwarding adapter.
   - `snapshot_hz` is set in 13 configs and drives nothing.
   - 9 inspection knobs are set by no config.
6. Types are already mostly right:
   - no data-only classes;
   - one static-only class (`HostClock`);
   - about 10 structs that behave like classes (mutators with invariants, pimpl `Impl` structs);
   - globals only where frameworks force them, plus one generated header;
   - the Pico has 7 top-level namespaces and no component root.
7. Pipeline docs exist but are scattered: README tables, an 84-line header comment in `system.h`,
   architecture.md, and header comments. There is no `PIPELINE.md` anywhere. A template and three
   drafts are in section 6.

## 1. naviGATR deep dive

### 1.1 The main pipeline, end to end

Construction (once):

| Step | Where | What |
|---|---|---|
| 1 | `src/app/main.cpp:93-94` | `navigatr::FunctionRegistry functions; navigatr::registerAll(functions);` |
| 2 | `src/runtime/register_all.cpp:9-17` | calls `register_resources`, `register_sensors`, `register_commands`, `register_localization`, `register_world_estimation`, `register_target_resolution`, `register_publishers` in that order |
| 3 | `src/impl/*/register_*.cpp` | e.g. `registerOrDie<SensorMakeFunction>(functions, "pico_encoder_channel", &make_pico_encoder_channel);` (`impl/sensors/register_sensors.cpp:12-13`) |
| 4 | `src/app/main.cpp:97-98` | `navigatr::System::buildFromFile(command.config_path, functions, err, command.build);` |
| 5 | `src/runtime/system.cpp:112-126` | `resolveConfiguration` (composed `<Configuration>` -> `<System>` XML), then `buildFromString` |
| 6 | `src/runtime/system.cpp:128-137` | `new System()`, private `build()`; failure destroys the candidate: `return nullptr;   // atomic: the partial candidate dies here` |
| 7 | `src/runtime/system.cpp:148-186` | parse, check root children `{"Loop", "Resources", "Sensors", "Pipeline", "Inspection", "Capture"}`, Loop, Inspection, Capture |
| 8 | `src/runtime/system.cpp:193-194` | `make_resources(root.child("Resources"), functions, resource_options, &warnings_, err)` -> `ResourceStore` + `ResourceExecutor` (`runtime/resource_stage.cpp:87-145`) |
| 9 | `src/runtime/system.cpp:201-203` | `make_sensors(...)` -> `SensorExecutor` (`runtime/sensor_stage.cpp:77-126`) |
| 10 | `src/runtime/system.cpp:231-237` | optional `<BrainProfile>`: `buildProfileHost` (`system.cpp:353-395`) |
| 11 | `src/runtime/system.cpp:259-279` | generic `buildSlot` lambda builds `CommandCollection` from the registry (uses the `Tag<T>` trick, `system.cpp:30-33`) |
| 12 | `src/runtime/system.cpp:292-295` | `profiled_ ? buildWaitingLocalization(err) : make_localization(...)` (`runtime/localization_stage.cpp:199-338`) |
| 13 | `src/runtime/system.cpp:317-318` | `make_world_estimation(...)` (`runtime/world_estimation_stage.cpp:71-143`) |
| 14 | `src/runtime/system.cpp:329-338` | `buildSlot("TargetResolution", ...)`, `buildSlot("Publishing", ...)` |
| 15 | `src/runtime/system.cpp:339-343` | pairing rule: `if (commands_type_.value == "brain_link" && publishing_type.value != "brain_link") {` |
| 16 | `src/runtime/system.cpp:349` | `capture_ = makeCaptureRecorder(*this, capture_config_);` |

Running:

| Mode | Where | Order |
|---|---|---|
| worker (default) | `main.cpp:167` `system->start(err)` -> `system.cpp:1354-1382` `startWorkers` spawns two threads | |
| estimation worker | `system.cpp:1411-1434` | `estimationCycle(now)` -> `field_handoff_.put(...)` -> `reportingCycle(now)` at the loop rate |
| field worker | `system.cpp:1436-1448` | `field_handoff_.waitTake(...)` -> `fieldCycle(*work.sensors, work.now)` (latest wins, `runtime/handoff.h:27` `LatestSlot`) |
| main thread | `main.cpp:171-177` | `system->applyPendingProfile();` and event printing every 20 ms |
| inline (`--inline`) | `main.cpp:154-165` -> `system.cpp:1022-1031` `step` | `estimationCycle`, `fieldCycle`, `reportingCycle` on one thread |

One cycle:

| Stage | Call site | Input -> output |
|---|---|---|
| 1 resources | `system.cpp:1039` `const ResourceMap& resource_map = execute_resources_(context);` | `ExecutionContext` -> `ResourceMap` |
| 2 sensors | `system.cpp:1040` `execute_sensors_(resource_map, context)` | `ResourceMap` -> `SensorMap` |
| 3 commands | `system.cpp:1045` `commands_->run({command_, now, &diagnostics_})` | `CommandState` -> `CommandState` |
| (glue) | `system.cpp:404-417` `requestsFrom(command_)` | `CommandState` -> `LocalizationRequests` (placement) |
| 4 localization | `system.cpp:1054` `robot_ = execute_localization_(sensor_map, requestsFrom(command_), context);` | `SensorMap` + requests -> `RobotState` (+ history via `RobotStateFeed`) |
| (Brain profile) | `system.cpp:1041-1062` | `precheck_.update`, `watchSensors`, `watchModels`, `noteRecovery`, `updatePicoOperation` interleaved with the stages |
| 5 world estimation | `system.cpp:1117-1118` `execute_world_({sensors, robot, *feed_, previous->field, context})` | `FieldEstimationInput` -> `FieldEstimationOutput` |
| 6 target resolution | `system.cpp:1069-1071` `target_resolution_->run(...)` | `TargetResolutionInput` -> `TargetState` |
| 7 publishing | `system.cpp:1075-1077` `publishing_->run(...)` | `PublishingInput` -> Brain reply (+ `BrainState` for inspection) |
| snapshots | `system.cpp:1080-1103`, `1128`, `1063` | `ReportingSnapshot`, detection frames, source health for inspection |

Inspection and capture only read published snapshots:
- `System::fieldSnapshot`, `reportingSnapshot`, `sourceHealth`, `diagnosticsSnapshot`, `detectionFrames`
  (`system.cpp:994-1018`);
- the inspection service is created in `main.cpp:128-138`.

### 1.2 Why it is hard to find

1. **No entry document points at code.**
   - `pi/naviGATR/README.md` is 278 lines. It opens with product description and a 15-row document table
     (`README.md:38-55`).
   - The best pipeline picture, "What runs" (`README.md:57-77`), names `make_resources -> make_sensors ->
     make_localization` but no file.
   - The only source map is at the end of a 381-line doc: `docs/architecture.md:362-381`
     ("| Coordinator and workers | [system.cpp](../src/runtime/system.cpp) |"). It is linked from the
     README table's 5th row.
   - This README also breaks the standing rule "READMEs are short signposts".
2. **The pipeline order is buried in the middle of a 1450-line file.**

   | `system.cpp` lines | Responsibility | Lines | Share |
   |---|---|---|---|
   | 1-76 | helpers (`checkChildren`, `newSessionId`, `elapsedMs`, `hexId`, `Tag`) | 76 | 5% |
   | 78-108 | `ProfileHost` adapter (Brain profile) | 31 | 2% |
   | 110-146 | ctor, `buildFromFile`, `buildFromString`, dtor | 37 | 3% |
   | 148-351 | `System::build`: parse, `make_*` in order, slots, capture | 204 | 14% |
   | 353-402 | Brain profile host build, waiting localization | 50 | 3% |
   | 404-417 | `requestsFrom`: commands -> localization glue | 14 | 1% |
   | 419-685 | profile prepare, CONTROL, Pico operations, READ_WHEELS | 267 | 18% |
   | 687-814 | sensor loss, model drops, unplacing, recovery events | 128 | 9% |
   | 816-908 | profile swap at the boundary, swap to waiting | 93 | 6% |
   | 910-936 | pause/resume workers | 27 | 2% |
   | 938-990 | binding view, events, capture faults | 53 | 4% |
   | 992-1018 | snapshot readers | 27 | 2% |
   | **1020-1140** | **the pipeline: `step`, `estimationCycle`, `reportingCycle`, `fieldCycle`** | **121** | **8%** |
   | 1142-1302 | inspection snapshot assembly (source health, detection frames) | 161 | 11% |
   | 1304-1448 | reset, start/stop, worker loops | 145 | 10% |

   - Brain-profile/Pico/recovery code totals 569 lines (39%). The pipeline plus its glue is 135 (9%).
   - Lizard: `System::build` is 171 NLOC, CCN 31. `noteRecovery` is CCN 26 and
     `publishDetectionFrames` is CCN 25.
3. **`system.h` is a 479-line god header.**
   - An 84-line header comment (`system.h:1-84`) covers construction, workers, threading, Brain
     profiles, recovery and Pico CONTROL.
   - The pipeline diagram is at lines 10-11: "Resources -> Sensors -> Command Collection ->
     Localization / -> World Estimation -> Target Resolution -> Publishing".
   - It defines 5 public data structs besides `System` (`FieldSnapshot` :134, `PicoOperation` :145,
     `ReportingSnapshot` :158, `BindingView` :171, `RuntimeEvent` :180).
   - libclang counts 70 data members and 75 methods on `System`.
4. **The `make_*` builders hide in a catch-all folder.**
   - `src/runtime/` holds 26 files with at least 7 roles:
     - stage builders: `*_stage.*`, 8 files;
     - catalogs and bookkeeping: 3;
     - the coordinator: 2;
     - registry: 2;
     - Brain-profile feature: `brain_profile_builder.*`, `sensor_loss.*`, `stationary_precheck.*`,
       `pico_control_ref.h`, 7 files;
     - inspection types: `inspection_config.h`, `inspection_state.h`;
     - worker plumbing: `handoff.h`;
     - `build_options.h`.
   - Only 4 of the 7 stages have a `make_*`. Commands, target resolution and publishing are built by an
     anonymous generic lambda (`system.cpp:259` `const auto buildSlot = [&](const char* slot_name,
     auto& target, auto make_tag,`), so `grep make_commands` finds nothing.
5. **One stage lives in 6 to 8 folders.** Localization today:
   - `contracts/localization.h`;
   - `runtime/localization_stage.{h,cpp}`;
   - `impl/localization/` (17 files);
   - `impl/noop/noops.cpp:19` (`NoopStateEstimator`);
   - `payloads/robot_observations.h`;
   - `state/{robot_state,robot_state_feed,pose_history,attitude,stillness}.*`;
   - `runtime/brain_profile_builder.*` (the templates that generate localization XML);
   - `runtime/system.cpp` (`requestsFrom`, `watchModels`).

   World estimation spans 8: `contracts/`, `runtime/`, `impl/world_estimation/`, `impl/noop/`,
   `payloads/`, `state/`, `config/field_map.*`, `resources/tag_detector.h`.

   Camera code is 10 files in 5 folders:
   - `impl/resources/{camera_capture.h,cameras.*,libcamera_camera.*}`;
   - `impl/sensors/camera_channel.*`;
   - `math/camera_model.h`;
   - `payloads/camera_frames.h`;
   - `resources/camera.h`.
6. **Three meanings of "resource" in three places.**
   - `src/resources/` = typed device contracts plus the store.
   - `src/impl/resources/` = implementations.
   - `src/contracts/resource.h` = the executable half.
   - The data contracts are split by an internal taxonomy (`contracts/` interfaces, `payloads/` payload
     types, `state/` stage outputs, `core/records.h` envelopes) instead of by stage.
7. **One concept, several names.**
   - Stage 5 is `WorldEstimation` in XML, the stage builder and `impl/world_estimation/`. Its contract is
     `FieldEstimation` (`contracts/field_estimation.h:58`, `FieldEstimationInput`). Its worker, cycle and
     state are "field" (`fieldWorker`, `fieldCycle`, `FieldState`, `FieldSnapshot`).
   - Stage 3 is `CommandCollection` in XML, `Commands` in the contract, `impl/commands/`.
   - Stage 7 is `Publishing`, registered by `register_publishers`.
   - Factories follow three schemes:
     - `make_pico_telemetry` (snake, resources and sensors);
     - `makeNoopCommands` (camel, noops, `impl/noop/noops.h:26`);
     - `BrainLinkCommands::create` (static member, every class-based implementation).
   - Registration mixes `registerAll` (`runtime/register_all.h:30`) with `register_resources` (`:21`).
   - The basename `records.h` exists twice (`core/records.h`, `diagnostics/records.h`).
8. **Dependencies run the wrong way.** Generic files include implementation headers:
   - `runtime/resource_stage.cpp:6` `#include "impl/resources/serial_links.h"`, for the `--replay`
     override at `:111-116`;
   - `runtime/pico_control_ref.h:12` `#include "impl/resources/pico_telemetry.h"`;
   - `inspection/inspection_document.cpp:17-18` includes `impl/publishing/field_documents.h` and
     `impl/resources/tag_detectors.h`;
   - `diagnostics/instrumentation.cpp:17` includes `impl/resources/pico_telemetry.h` and down-casts at
     `:231-232` (`std::shared_ptr<PicoTelemetry> findPico(const System& system) {`).

   A reader cannot trust the folder names to tell generic from specific.
9. **Observers outweigh the pipeline.**
   - `inspection/` is 5878 lines, `capture/` 3439 and `diagnostics/` 1342: 29% of `src/`'s 37112 lines.
   - `inspection/inspection_document.cpp` (2052 lines) is the largest file in the tree.
   - The stage builders are 128-340 lines each. The observers sit as peers of the stages with nothing
     marking them read-only.
10. **The pipeline changes shape at runtime, and the two modes order stages differently.**
    - A Brain profile replaces the sensor and localization executors in place (`system.cpp:844-846`
      `execute_sensors_.replaceProfile(std::move(c->sensors));`).
    - `step()` runs world estimation before target resolution in the same cycle (`system.cpp:1026-1030`).
      The workers run target resolution against the newest field snapshot the field worker has
      published, usually from an earlier cycle.
    - Both facts are documented only in prose: the `system.h` header comment (`:22-35`, `:48-63`) and
      `README.md:67-69`. Nothing in the code layout shows them.

### 1.3 Proposed restructure

Goals:
- the folder tree mirrors the pipeline;
- behavior stays identical;
- the system keeps its shape: same stages, contracts, registries, workers and namespace.

Target tree (numbered prefixes make any file listing show the stage order; the alternative is plain names plus
the order in `PIPELINE.md`, see open decision D1):

```text
pi/naviGATR/
  README.md                 signpost, about 30 lines: what, build, run, links
  PIPELINE.md               the whole runtime (draft in 6.4)
  src/
    app/                    main.cpp, command_line.*, build_config.h.in
    pipeline/               the coordinator
      system.h              class System, short header comment
      build.cpp             System::build: parse, make_* in order   <- "the make pipeline"
      cycle.cpp             step, estimationCycle, fieldCycle, reportingCycle   <- "the main pipeline"
      workers.cpp           start, stop, reset, the two worker loops
      snapshots.h/.cpp      what workers publish for readers
      events.h/.cpp         lifecycle events, capture faults, recovery notes
      slot_init.h, register_all.*, build_options.h, handoff.h, noops.*
    1_resources/            PIPELINE.md, resource_contract.h, resource_stage.*, resource_store.*,
                            resource_catalog.h, resource_instance.h, register_resources.cpp,
                            pico_telemetry_samples.h,
                            kinds/ (camera, serial_link, tag_detector, field_map, ...),
                            impl/{links,pico,camera,tags,static_data,sim}/
    2_sensors/              PIPELINE.md, sensor_contract.h, sensor_stage.*, sensor_catalog.h,
                            register_sensors.cpp, sensor_samples.h, camera_frames.h,
                            attitude_samples.h, impl/
    3_commands/             PIPELINE.md, commands_contract.h, command_state.h, register_commands.cpp, impl/
    4_localization/         PIPELINE.md, localization_contract.h, localization_stage.*,
                            register_localization.cpp, robot_state.h, robot_state_feed.*,
                            pose_history.*, attitude.h, stillness.h, robot_observations.h, impl/
    5_world_estimation/     PIPELINE.md, world_estimation_contract.h, world_estimation_stage.*,
                            register_world_estimation.cpp, field_state.h, tag_observations.h,
                            field_object_evidence.h,
                            impl/apriltag/ (PIPELINE.md, the 3-step subpipeline and its 2 step contracts)
    6_target_resolution/    PIPELINE.md, target_resolution_contract.h, target_state.h,
                            register_target_resolution.cpp, impl/
    7_publishing/           PIPELINE.md, publishing_contract.h, register_publishers.cpp, impl/
    brain_profile/          PIPELINE.md, brain_profile_contract.h, brain_profile_builder.*,
                            profile_swap.cpp, control.cpp, continuity.cpp, sensor_loss.*,
                            stationary_precheck.*, pico_control_ref.h, pico_operation.h
    inspection/ capture/ diagnostics/    read-only observers (unchanged, + inspection_config.h)
    core/ config/ math/ transport/       shared building blocks (unchanged, + core/record_bookkeeping.h)
  tools/field_refs/field_refs.cpp        was src/app/field_refs.cpp (a generator, not the app)
```

What a newcomer now reads, in order:
1. `PIPELINE.md`;
2. `src/app/main.cpp`;
3. `src/pipeline/build.cpp`;
4. `src/pipeline/cycle.cpp`;
5. any `src/N_stage/PIPELINE.md`.

#### Split of `system.cpp`

Every function moves **unchanged** as a `System::` member definition in another translation unit. C++
allows this, so it is a pure move with no signature change. Each anonymous-namespace helper has users in
only one destination file: `hexId` is used only by `prepareProfile` and `swapProfile`, `elapsedMs` only
by the workers, `kEventsKept` only by `noteEvent`, and `checkChildren`, `newSessionId` and `Tag` only by
`build`. Nothing is duplicated and the defined-symbol set stays identical.

| New file | From `system.cpp` lines | Content | About |
|---|---|---|---|
| `pipeline/build.cpp` | 1-76 (build helpers), 110-351 | ctor/dtor, `buildFromFile`, `buildFromString`, `build` | 290 |
| `pipeline/cycle.cpp` | 404-417, 1020-1140 | `requestsFrom`, `step`, `estimationCycle`, `reportingCycle`, `fieldCycle` | 140 |
| `pipeline/workers.cpp` | 910-936, 1304-1448 | `pauseWorkers`, `resumeWorkers`, `resetStages`, `reset`, `start`, `startWorkers`, `requestStop`, `stop`, `estimationWorker`, `fieldWorker` | 175 |
| `pipeline/snapshots.cpp` | 943-959, 992-1018, 1142-1302 | `publishBindingView`, `bindingView`, snapshot readers, `publishSourceHealth`, `publishDetectionFrames` | 210 |
| `pipeline/events.cpp` | 735-814, 961-990 | `noteRecovery`, `noteEvent`, `noteFault`, `events` | 110 |
| `brain_profile/profile_swap.cpp` | 78-108, 353-402, 419-473, 816-908, 938-941 | `ProfileHost`, `buildProfileHost`, `buildWaitingLocalization`, `prepareProfile`, `applyPendingProfile`, `swapProfile`, `swapToWaiting`, `profileBinding` | 235 |
| `brain_profile/control.cpp` | 475-685 | `controlProfile`, `submitPico`, `controlProgress`, `updatePicoOperation`, `readWheels` | 210 |
| `brain_profile/continuity.cpp` | 687-733 | `watchSensors`, `watchModels`, `losePlacement` | 50 |

`system.h` split, also move-only:
- `FieldSnapshot`, `ReportingSnapshot` and `BindingView` go to `pipeline/snapshots.h`, together with the
  current `runtime/inspection_state.h`.
- `RuntimeEvent` goes to `pipeline/events.h`.
- `PicoOperation` goes to `brain_profile/pico_operation.h`.
- The 84-line header comment becomes `PIPELINE.md` text. The header keeps a few label lines pointing to it.

#### Namespace changes

None. The runtime stays one component with one namespace, `navigatr`:
- class names are already unique and stage-prefixed (`LocalizationExecutor`, `SensorCatalog`);
- per-stage namespaces would touch every reference for little gain;
- folder names cannot be namespaces anyway once they carry digits.

Namespace fixes elsewhere are in section 5(d).

#### Complete move map (142 of 201 files; `move_map.py` checks full coverage)

Base names stay the same except:
- the 9 contract headers get a `_contract` suffix, which is free because their include lines change anyway;
- `inspection_state.h` becomes `snapshots.h`;
- `diagnostics/records.h` becomes `diag_records.h` (unique basename).

Git rename detection works because content is unchanged apart from include lines.

| Old | New |
|---|---|
| `src/app/field_refs.cpp` | `tools/field_refs/field_refs.cpp` |
| `src/config/field_map.cpp` | `src/1_resources/kinds/field_map.cpp` |
| `src/config/field_map.h` | `src/1_resources/kinds/field_map.h` |
| `src/contracts/association.h` | `src/5_world_estimation/impl/apriltag/association_contract.h` |
| `src/contracts/brain_profile.h` | `src/brain_profile/brain_profile_contract.h` |
| `src/contracts/commands.h` | `src/3_commands/commands_contract.h` |
| `src/contracts/field_estimation.h` | `src/5_world_estimation/world_estimation_contract.h` |
| `src/contracts/localization.h` | `src/4_localization/localization_contract.h` |
| `src/contracts/perception.h` | `src/5_world_estimation/impl/apriltag/perception_contract.h` |
| `src/contracts/publishing.h` | `src/7_publishing/publishing_contract.h` |
| `src/contracts/resource.h` | `src/1_resources/resource_contract.h` |
| `src/contracts/sensor.h` | `src/2_sensors/sensor_contract.h` |
| `src/contracts/slot_init.h` | `src/pipeline/slot_init.h` |
| `src/contracts/target_resolution.h` | `src/6_target_resolution/target_resolution_contract.h` |
| `src/diagnostics/records.h` | `src/diagnostics/diag_records.h` |
| `src/impl/commands/brain_link_commands.cpp` | `src/3_commands/impl/brain_link_commands.cpp` |
| `src/impl/commands/brain_link_commands.h` | `src/3_commands/impl/brain_link_commands.h` |
| `src/impl/commands/register_commands.cpp` | `src/3_commands/register_commands.cpp` |
| `src/impl/localization/attitude_reference.cpp` | `src/4_localization/impl/attitude_reference.cpp` |
| `src/impl/localization/attitude_reference.h` | `src/4_localization/impl/attitude_reference.h` |
| `src/impl/localization/brain_imu_wheel_bench.cpp` | `src/4_localization/impl/brain_imu_wheel_bench.cpp` |
| `src/impl/localization/brain_imu_wheel_bench.h` | `src/4_localization/impl/brain_imu_wheel_bench.h` |
| `src/impl/localization/imu_heading_increment.cpp` | `src/4_localization/impl/imu_heading_increment.cpp` |
| `src/impl/localization/imu_heading_increment.h` | `src/4_localization/impl/imu_heading_increment.h` |
| `src/impl/localization/motion_step.cpp` | `src/4_localization/impl/motion_step.cpp` |
| `src/impl/localization/motion_step.h` | `src/4_localization/impl/motion_step.h` |
| `src/impl/localization/planar_motion_integrator.cpp` | `src/4_localization/impl/planar_motion_integrator.cpp` |
| `src/impl/localization/planar_motion_integrator.h` | `src/4_localization/impl/planar_motion_integrator.h` |
| `src/impl/localization/register_localization.cpp` | `src/4_localization/register_localization.cpp` |
| `src/impl/localization/stationary_window.cpp` | `src/4_localization/impl/stationary_window.cpp` |
| `src/impl/localization/stationary_window.h` | `src/4_localization/impl/stationary_window.h` |
| `src/impl/localization/tracking_wheel_motion.cpp` | `src/4_localization/impl/tracking_wheel_motion.cpp` |
| `src/impl/localization/tracking_wheel_motion.h` | `src/4_localization/impl/tracking_wheel_motion.h` |
| `src/impl/localization/weighted_planar_fusion.cpp` | `src/4_localization/impl/weighted_planar_fusion.cpp` |
| `src/impl/localization/weighted_planar_fusion.h` | `src/4_localization/impl/weighted_planar_fusion.h` |
| `src/impl/noop/noops.cpp` | `src/pipeline/noops.cpp` |
| `src/impl/noop/noops.h` | `src/pipeline/noops.h` |
| `src/impl/publishing/brain_link_publisher.cpp` | `src/7_publishing/impl/brain_link_publisher.cpp` |
| `src/impl/publishing/brain_link_publisher.h` | `src/7_publishing/impl/brain_link_publisher.h` |
| `src/impl/publishing/field_documents.cpp` | `src/7_publishing/impl/field_documents.cpp` |
| `src/impl/publishing/field_documents.h` | `src/7_publishing/impl/field_documents.h` |
| `src/impl/publishing/register_publishers.cpp` | `src/7_publishing/register_publishers.cpp` |
| `src/impl/resources/apriltag_c.c` | `src/1_resources/impl/tags/apriltag_c.c` |
| `src/impl/resources/apriltag_c.h` | `src/1_resources/impl/tags/apriltag_c.h` |
| `src/impl/resources/camera_capture.h` | `src/1_resources/impl/camera/camera_capture.h` |
| `src/impl/resources/cameras.cpp` | `src/1_resources/impl/camera/cameras.cpp` |
| `src/impl/resources/cameras.h` | `src/1_resources/impl/camera/cameras.h` |
| `src/impl/resources/field_map_resource.cpp` | `src/1_resources/impl/static_data/field_map_resource.cpp` |
| `src/impl/resources/field_map_resource.h` | `src/1_resources/impl/static_data/field_map_resource.h` |
| `src/impl/resources/libcamera_camera.cpp` | `src/1_resources/impl/camera/libcamera_camera.cpp` |
| `src/impl/resources/libcamera_camera.h` | `src/1_resources/impl/camera/libcamera_camera.h` |
| `src/impl/resources/pico_telemetry.cpp` | `src/1_resources/impl/pico/pico_telemetry.cpp` |
| `src/impl/resources/pico_telemetry.h` | `src/1_resources/impl/pico/pico_telemetry.h` |
| `src/impl/resources/pros_usb_link.cpp` | `src/1_resources/impl/links/pros_usb_link.cpp` |
| `src/impl/resources/pros_usb_link.h` | `src/1_resources/impl/links/pros_usb_link.h` |
| `src/impl/resources/register_resources.cpp` | `src/1_resources/register_resources.cpp` |
| `src/impl/resources/robot_frame_map.cpp` | `src/1_resources/impl/static_data/robot_frame_map.cpp` |
| `src/impl/resources/robot_frame_map.h` | `src/1_resources/impl/static_data/robot_frame_map.h` |
| `src/impl/resources/serial_links.cpp` | `src/1_resources/impl/links/serial_links.cpp` |
| `src/impl/resources/serial_links.h` | `src/1_resources/impl/links/serial_links.h` |
| `src/impl/resources/synthetic_rig.cpp` | `src/1_resources/impl/sim/synthetic_rig.cpp` |
| `src/impl/resources/synthetic_rig.h` | `src/1_resources/impl/sim/synthetic_rig.h` |
| `src/impl/resources/tag_detectors.cpp` | `src/1_resources/impl/tags/tag_detectors.cpp` |
| `src/impl/resources/tag_detectors.h` | `src/1_resources/impl/tags/tag_detectors.h` |
| `src/impl/resources/target_set_resource.cpp` | `src/1_resources/impl/static_data/target_set_resource.cpp` |
| `src/impl/resources/target_set_resource.h` | `src/1_resources/impl/static_data/target_set_resource.h` |
| `src/impl/resources/wheel_geometry_resource.cpp` | `src/1_resources/impl/static_data/wheel_geometry_resource.cpp` |
| `src/impl/resources/wheel_geometry_resource.h` | `src/1_resources/impl/static_data/wheel_geometry_resource.h` |
| `src/impl/sensors/attitude_channel.cpp` | `src/2_sensors/impl/attitude_channel.cpp` |
| `src/impl/sensors/attitude_channel.h` | `src/2_sensors/impl/attitude_channel.h` |
| `src/impl/sensors/camera_channel.cpp` | `src/2_sensors/impl/camera_channel.cpp` |
| `src/impl/sensors/camera_channel.h` | `src/2_sensors/impl/camera_channel.h` |
| `src/impl/sensors/pico_channels.cpp` | `src/2_sensors/impl/pico_channels.cpp` |
| `src/impl/sensors/pico_channels.h` | `src/2_sensors/impl/pico_channels.h` |
| `src/impl/sensors/register_sensors.cpp` | `src/2_sensors/register_sensors.cpp` |
| `src/impl/target_resolution/configured_targets.cpp` | `src/6_target_resolution/impl/configured_targets.cpp` |
| `src/impl/target_resolution/configured_targets.h` | `src/6_target_resolution/impl/configured_targets.h` |
| `src/impl/target_resolution/register_target_resolution.cpp` | `src/6_target_resolution/register_target_resolution.cpp` |
| `src/impl/world_estimation/apriltag_tag_observation.cpp` | `src/5_world_estimation/impl/apriltag/apriltag_tag_observation.cpp` |
| `src/impl/world_estimation/apriltag_tag_observation.h` | `src/5_world_estimation/impl/apriltag/apriltag_tag_observation.h` |
| `src/impl/world_estimation/apriltag_world_estimator.cpp` | `src/5_world_estimation/impl/apriltag/apriltag_world_estimator.cpp` |
| `src/impl/world_estimation/apriltag_world_estimator.h` | `src/5_world_estimation/impl/apriltag/apriltag_world_estimator.h` |
| `src/impl/world_estimation/landmark_estimator.cpp` | `src/5_world_estimation/impl/apriltag/landmark_estimator.cpp` |
| `src/impl/world_estimation/landmark_estimator.h` | `src/5_world_estimation/impl/apriltag/landmark_estimator.h` |
| `src/impl/world_estimation/register_world_estimation.cpp` | `src/5_world_estimation/register_world_estimation.cpp` |
| `src/impl/world_estimation/tag_mount_association.cpp` | `src/5_world_estimation/impl/apriltag/tag_mount_association.cpp` |
| `src/impl/world_estimation/tag_mount_association.h` | `src/5_world_estimation/impl/apriltag/tag_mount_association.h` |
| `src/payloads/attitude_samples.h` | `src/2_sensors/attitude_samples.h` |
| `src/payloads/camera_frames.h` | `src/2_sensors/camera_frames.h` |
| `src/payloads/field_object_evidence.h` | `src/5_world_estimation/field_object_evidence.h` |
| `src/payloads/pico_telemetry_samples.h` | `src/1_resources/pico_telemetry_samples.h` |
| `src/payloads/robot_observations.h` | `src/4_localization/robot_observations.h` |
| `src/payloads/sensor_samples.h` | `src/2_sensors/sensor_samples.h` |
| `src/payloads/tag_observations.h` | `src/5_world_estimation/tag_observations.h` |
| `src/resources/brain_imu_bench.h` | `src/1_resources/kinds/brain_imu_bench.h` |
| `src/resources/camera.h` | `src/1_resources/kinds/camera.h` |
| `src/resources/pico_control.h` | `src/1_resources/kinds/pico_control.h` |
| `src/resources/resource_instance.h` | `src/1_resources/resource_instance.h` |
| `src/resources/resource_store.cpp` | `src/1_resources/resource_store.cpp` |
| `src/resources/resource_store.h` | `src/1_resources/resource_store.h` |
| `src/resources/robot_frames.h` | `src/1_resources/kinds/robot_frames.h` |
| `src/resources/serial_link.h` | `src/1_resources/kinds/serial_link.h` |
| `src/resources/spi_bus.h` | `src/1_resources/kinds/spi_bus.h` (or delete, see 4.2) |
| `src/resources/tag_detector.h` | `src/1_resources/kinds/tag_detector.h` |
| `src/resources/target_set.h` | `src/1_resources/kinds/target_set.h` |
| `src/resources/wheel_geometry.h` | `src/1_resources/kinds/wheel_geometry.h` |
| `src/runtime/brain_profile_builder.cpp` | `src/brain_profile/brain_profile_builder.cpp` |
| `src/runtime/brain_profile_builder.h` | `src/brain_profile/brain_profile_builder.h` |
| `src/runtime/build_options.h` | `src/pipeline/build_options.h` |
| `src/runtime/handoff.h` | `src/pipeline/handoff.h` |
| `src/runtime/inspection_config.h` | `src/inspection/inspection_config.h` |
| `src/runtime/inspection_state.h` | `src/pipeline/snapshots.h` |
| `src/runtime/localization_stage.cpp` | `src/4_localization/localization_stage.cpp` |
| `src/runtime/localization_stage.h` | `src/4_localization/localization_stage.h` |
| `src/runtime/pico_control_ref.h` | `src/brain_profile/pico_control_ref.h` |
| `src/runtime/record_bookkeeping.h` | `src/core/record_bookkeeping.h` |
| `src/runtime/register_all.cpp` | `src/pipeline/register_all.cpp` |
| `src/runtime/register_all.h` | `src/pipeline/register_all.h` |
| `src/runtime/resource_catalog.h` | `src/1_resources/resource_catalog.h` |
| `src/runtime/resource_stage.cpp` | `src/1_resources/resource_stage.cpp` |
| `src/runtime/resource_stage.h` | `src/1_resources/resource_stage.h` |
| `src/runtime/sensor_catalog.h` | `src/2_sensors/sensor_catalog.h` |
| `src/runtime/sensor_loss.cpp` | `src/brain_profile/sensor_loss.cpp` |
| `src/runtime/sensor_loss.h` | `src/brain_profile/sensor_loss.h` |
| `src/runtime/sensor_stage.cpp` | `src/2_sensors/sensor_stage.cpp` |
| `src/runtime/sensor_stage.h` | `src/2_sensors/sensor_stage.h` |
| `src/runtime/stationary_precheck.cpp` | `src/brain_profile/stationary_precheck.cpp` |
| `src/runtime/stationary_precheck.h` | `src/brain_profile/stationary_precheck.h` |
| `src/runtime/system.cpp` | split: `src/pipeline/{build,cycle,workers,snapshots,events}.cpp`, `src/brain_profile/{profile_swap,control,continuity}.cpp` |
| `src/runtime/system.h` | `src/pipeline/system.h` (+ `snapshots.h`, `events.h`, `brain_profile/pico_operation.h`) |
| `src/runtime/world_estimation_stage.cpp` | `src/5_world_estimation/world_estimation_stage.cpp` |
| `src/runtime/world_estimation_stage.h` | `src/5_world_estimation/world_estimation_stage.h` |
| `src/state/attitude.h` | `src/4_localization/attitude.h` |
| `src/state/command_state.h` | `src/3_commands/command_state.h` |
| `src/state/field_state.h` | `src/5_world_estimation/field_state.h` |
| `src/state/pose_history.cpp` | `src/4_localization/pose_history.cpp` |
| `src/state/pose_history.h` | `src/4_localization/pose_history.h` |
| `src/state/robot_state.h` | `src/4_localization/robot_state.h` |
| `src/state/robot_state_feed.cpp` | `src/4_localization/robot_state_feed.cpp` |
| `src/state/robot_state_feed.h` | `src/4_localization/robot_state_feed.h` |
| `src/state/stillness.h` | `src/4_localization/stillness.h` |
| `src/state/target_state.h` | `src/6_target_resolution/target_state.h` |

Unchanged (59 files): `src/app/*` except `field_refs.cpp`; `src/capture/*`; `src/config/{composition,config_node}.*`
and `pose3_config.h`; `src/core/*`; `src/diagnostics/*` except `records.h`; `src/inspection/*`; `src/math/*`;
`src/transport/*`. Also unchanged: `tests/` (flat per the convention, only include lines change),
`config/`, `spectaGATR/`, `third_party/`.

Also in the same pass:
- root `docs/navigatr_resources.md` and `docs/navigatr_sensors.md` move to `pi/naviGATR/docs/`. Those
  are the only naviGATR docs outside the component.
- 20 doc links to `src/` paths are updated.

#### Commit plan and how behavior stays identical

| # | Commit | Kind | Gate |
|---|---|---|---|
| 1 | Add root `PIPELINE.md`, `pi/naviGATR/PIPELINE.md` (current paths), trim `pi/naviGATR/README.md` to a signpost | docs only | link check |
| 2 | `runtime/` -> `pipeline/`, `brain_profile/`, and the `*_stage.*`/catalog files to their stage folders | move-only | A, B, C |
| 3 | Split `system.cpp`/`system.h` as tabled above | move-only (same bodies) | A, B, C |
| 4-10 | One commit per stage: its contract, state, payloads, registry and `impl/` into `src/N_stage/` | move-only | A, B, C |
| 11 | `docs/navigatr_*.md` into `pi/naviGATR/docs`, doc link updates, stage `PIPELINE.md` files | docs only | link check |

Gates:
- **A. Tests.**
  - Before commit 2, record `navigatr_tests --gtest_list_tests` and the full `ctest` result on the host.
    That is 617 `TEST` cases across 37 files, plus 10 in `brain_link_e2e_tests`.
  - Include the perf tools (`-DNAVIGATR_BUILD_PERF_TOOLS=ON`).
  - After each commit the list and the results must be identical.
  - Also run the `field_references` target: the regenerated `brain/operaGATR/include/field_references.h`
    must be byte-identical.
  - Also run `tools/viewer_smoke.sh`.
- **B. Symbol parity.**
  - `nm -C --defined-only libnavigatr_core.a`, reduced to sorted unique `type name` pairs, must be
    identical before and after.
  - Moves and the member-function split cannot change a symbol, because no helper is shared between the
    new files.
  - This catches an accidental edit that tests miss.
- **C. Diff shape.**
  - `git diff -M --stat` shows renames only.
  - `git diff -M` filtered to `^[-+]` lines, minus `#include` lines and `CMakeLists.txt`, must be empty.
    For the split commit it contains only the moved blocks, which `git diff --color-moved` shows as moved.
- **Pi build.** `libcamera_camera.cpp` compiles only with `-DNAVIGATR_WITH_LIBCAMERA=ON` on the Pi. The
  commit that moves `impl/resources/` must also build on the Pi, or its include edit is unverified.

Churn:
- 142 files move and 522 include lines change in 166 files (src, tests, tools/perf).
- Both CMake source lists change; they stay explicit, per the convention.
- 20 doc links change.
- No function body changes in commits 2-10.

Smaller alternative, if the whole move is too much at once: commits 1 to 3 alone (docs + `runtime/` +
the `system.cpp` split) fix the gripe itself, meaning where the main pipeline is and where the make
pipeline is. They touch about 35 files.

#### Follow-ups that change code (each its own commit with tests; not part of the move)

1. Extract a `ProfileSupervisor` class (namespace `navigatr`, `src/brain_profile/`). It owns the
   Brain-profile state System carries today:
   - `profile_config_`, `profile_functions_`, `base_catalog_`, `profile_host_`;
   - `candidate_`, `applied_`, `profile_pending_`, `profile_applies_`;
   - `precheck_`, `pico_`, `pico_op_`;
   - `sensor_loss_`, `loss_warned_`, `sensor_lost_`, `models_dropped_`, `seen_.stillness`;
   - see `system.h:386-444`.

   The five `if (profiled_)` hooks in `estimationCycle` (`system.cpp:1041-1057`) become one call. About
   20 of `System`'s 70 members leave.
2. Fix the dependency direction (section 1.2 item 8):
   - replay: take the replay factory from the registry by name (`file_replay_link`) or through
     `BuildOptions`;
   - `PicoControl`: let `ResourceStore` hand out a stored resource by a base contract, so
     `pico_control_ref.h` needs no concrete type;
   - `field_documents` and the family geometry: expose them as functions next to their kinds.
3. Take Brain-link wire types out of generic contracts (section 3.1).
4. Split `pipeline/noops.*` into one `impl/noop.cpp` per stage.
5. One name per stage: rename `FieldEstimation`/`FieldEstimationInput` to `WorldEstimation...`, and
   pick either `Commands` or `CommandCollection`. Rename-only commit, tests gate.

## 2. Every other component

| Component | Understandable from folders? | Pipeline / role | INPUT | OUTPUT | Defined in |
|---|---|---|---|---|---|
| `translaGATR/` | Yes: 3 header/codec pairs + `tests/` | wire codec, struct <-> bytes, no pipeline | structs (`SensorSample`, `BrainRequest`, `PicoCommand`, `RobotProfileDoc`, ...) | frame bytes, and back | `frames.h` (types, 580 lines), `frame_codec.h`, `link_documents.h` |
| `pico/aggreGATR/` | Mostly: flat `src/` of 30 files named by device | tick 50 Hz: sample encoders + IMU -> encode -> UART; idle slots: status, diag; commands in | quadrature pins, IMU on SPI1 (`board.h`), Pi commands 0x12 | frames 0x04, 0x13, 0x14 | `translaGATR/frames.h`; loop `src/main.cpp:319-356` |
| `brain/investiGATR/` | Yes, but the name hides half its job | planner + the shared Brain types | `PlanRequest` (`planner.h:31`) | `PlanResult` (`planner.h:40`) | `include/investigatr/*.h` |
| `brain/communiGATR/` | Yes: `include/src/pros/sim/tests` | `Client::poll` -> choose -> transmit -> handle reply -> `LinkDriver` | bytes via `BytePort` (`byte_port.h:12`), `RobotProfile` | `investigatr::StateSource`, `PathSink` (`link_driver.h:25`), readiness, events | `docs/communigatr.md:39-54` layer diagram |
| `brain/actuGATR/` | Yes | `Motion::update`: resolve reference -> plan -> follow -> kinematics -> drive -> motors | `StateSource` + `PathPlanner` (`motion.h:140`), `goToDirect/goToAvoiding` | `MotionStatus`, motor commands via `MotorOutput` (`motor_output.h:14`) | `include/actugatr/*.h` |
| `brain/robot/` | Yes: one header | robot description | measured values | `gatr2_robot::` constants, robot profile | `gatr2_robot.h` |
| `brain/locaGATR/`, `brain/operaGATR/` | Yes | PROS programs that wire link -> planner -> motion -> drive | controller, link | screen, motors | `operaGATR/src/main.cpp:120` `createLink`, `:149` `createDrive` |
| `pi/naviGATR/spectaGATR/` | Partly: 19 flat ES modules, no README | WebSocket feed -> store -> animation loop -> scene and panels (`app.js:1-21`) | `navigatr.inspect/2` feed, capture ZIPs | the page; capture control over HTTP | C++ writer `inspection/inspection_document.cpp` (2052 lines), prose `docs/inspection.md` (768 lines); no schema file |
| `pi/naviGATR/tools/` | No | mixed: camera calibration (Python), viewer tests (Python + JS), perf (C++), service install (sh) | varies | varies | per tool |
| `bench/` | Yes | hardware bring-up | - | - | `bench/README.md` |

Verdicts and fixes:

- **translaGATR.** Clear.
  - Only component without a README. The root README links the bare folder (`README.md:86`).
  - Include style differs by consumer:
    - 57 lines use `"translaGATR/frames.h"`-style paths (Pi, Brain);
    - 27 use bare `"frames.h"` (the Pico via `platformio.ini` `-I$PROJECT_DIR/../../translaGATR`, and translaGATR's own files and tests).

  Fix:
  - a 10-line README naming the three files and which device builds which (`link_documents.h:5`
    "Brain and Pi only; the Pico does not build this");
  - one include style.

- **aggreGATR.** Clear enough at this size.
  - `main.cpp` has no header comment. Its loop is the whole pipeline (`main.cpp:319-356`).
  - Seven top-level namespaces and no component root: `board` (`board.h:12`), `cfg` (`config.h:17`),
    `encoder` (`quadrature.h:8`), `imu` (`imu.h:11`), `bno08x` (`bno08x_reports.h:9`), `asm330`
    (`asm330_supervisor.h:12`), `pilink` (`pi_link.h:8`).

  Fix:
  - group `src/` into `imu/` (asm330_*, bno08x_*, gravity_alignment, gyro_window, shtp_header, imu*),
    `encoders/` (quadrature) and `link/` (pi_link, pico_commands, command_records, diag_report);
    `build_src_filter = +<*>` already recurses;
  - root namespace `aggregatr` with those as nested namespaces;
  - a 15-line `PIPELINE.md`.

- **investiGATR.**
  - Its README says "Brain path planning and the types the Brain libraries share" (`README.md:3`).
    Someone looking for `Pose`, `Field` or `StateSource` does not look in the planner.

  Fix (low churn): README/PIPELINE lists the two roles and the files of each. The alternative, a
  separate types library, is churn for a naming problem.

- **communiGATR.** The layering is clear and well documented. Two problems:
  - The link's own pipeline is concentrated in one class. `Client` has 73 members and 79 methods
    (libclang); `client.cpp` is 1351 lines, and `Client::choose` is 143 NLOC with CCN 56 (lizard).
  - "Application helpers" (`wheel_calibration`, `startup_placement`, `link_events`,
    `vex_imu_recalibration`, per `communiGATR/README.md`) sit in the library's public include folder.
    Both programs use `startup_placement`, `link_events` and `vex_imu_recalibration`; only locaGATR uses
    `wheel_calibration`.

  Also a stale name: `docs/communigatr.md:41` "application (locaGATR, testing)".

  Fix:
  - `PIPELINE.md` for the link;
  - `include/communigatr/app/` for the helpers.

- **actuGATR.** Clear, and it has two implementations of every interface (`Follower`, `Kinematics`,
  `MotorOutput` pros + sim, `PathPlanner` real + sim).

  Watch:
  - `Motion::update` is 117 NLOC, CCN 38;
  - `using investigatr::CommandId;` and five more using-declarations in a public header
    (`motion.h:29-34`) re-export names into `actugatr`.

  Fix: `PIPELINE.md`.

- **locaGATR / operaGATR.**
  - The README title is still the old name: `brain/locaGATR/README.md:1` "# Localization test".
  - Apps are single large `main.cpp` files: 684 and 523 lines; locaGATR `run()` has CCN 45, operaGATR
    `opcontrol` has CCN 32.
  - Wiring is readable (`createLink`, `createDrive`).

  Fix:
  - title;
  - optional split of screen pages into their own files.

- **spectaGATR.**
  - Names are good, but the input contract has no single home. It is defined by the C++ writer and a
    768-line prose doc.
  - Season id prefixes are hard-coded (section 4).

  Fix:
  - README + `PIPELINE.md`;
  - a field list per message type in `docs/inspection.md`, pointed to from both sides.

- **tools/ and the "naviGATR is C++ only" rule.** naviGATR contains 5 Python files
  (`tools/calibrate_camera.py`, `tools/test_calibrate_camera.py`, `tools/viewer/*.py`), 21 JS files and
  3 shell scripts. `field_refs.cpp` is a generator living in `src/app/`. Decision D3.

- **Cross-component build duplication.** Brain library sources are listed three times:
  - `brain/*/CMakeLists.txt`;
  - `brain/gatr2_brain.mk:25-...`;
  - `pi/naviGATR/tests/CMakeLists.txt:76-100`, which lists 23 Brain `.cpp` files for
    `brain_link_e2e_tests`.

  Fix: the Pi test links the Brain CMake targets instead.

- **config/.** `config/override/` is the season folder but holds the primary, season-independent robot
  configs too: `brain_profile_usb.xml`, `brain_profile_rs485.xml`, `diagnostics/*`. Section 4.1.

## 3. Standard INPUT/OUTPUT

### 3.1 The contracts today

| Stage | Input | Output | Seam | Factory signature | Neutral? |
|---|---|---|---|---|---|
| 1 resources | `const ExecutionContext&` | `ResourcePollResult` per resource -> `ResourceMap` (`core/records.h:115`) | `struct ResourceExecutable` of `std::function` (`contracts/resource.h:43`) | returns `ResourceInstance` (`resources/resource_store.h:51`) | yes |
| 2 sensors | `const ResourceMap&, const ExecutionContext&` | `PollResult` -> `SensorMap` (`core/records.h:102`) | `struct SensorExecutable` (`contracts/sensor.h:30`) | returns `std::optional<SensorExecutable>` (`sensor.h:47`) | yes |
| 3 commands | `CommandsInput{previous, now, Diagnostics*}` (`commands.h:22`) | `CommandsOutput{CommandState, status}` (`:28`) | `class Commands` (`:33`) | returns `std::unique_ptr<Commands>` (`:43`) | **no**: `CommandState` holds `BrainReplyContext` (`state/command_state.h:30-58`), `ProfileStatus`, `PathReport`, all wire codes; includes `translaGATR/frames.h` (`:17`) |
| 4 localization | `SensorMap`, `LocalizationRequests`, `ExecutionContext` (`localization_stage.h:52`) | `RobotState` + `RobotStateFeed` | inner: `class RobotObservationFunction` (`localization.h:79`), `class StateEstimator` (`:159`) with `StateEstimatorInput/Output` (`:133`, `:140`) | `unique_ptr` (`:114`, `:195`) | yes |
| 5 world estimation | `FieldEstimationInput{sensors, robot, history, previousField, context}` (`field_estimation.h:42`) | `FieldEstimationOutput{field, observations, associations, status, diagnostic}` (`:50`) | `class FieldEstimation` (`:58`) | `unique_ptr` (`:76`) | yes |
| 6 target resolution | `TargetResolutionInput` (8 fields, `target_resolution.h:30`) | `TargetResolutionOutput{target, status}` (`:41`) | `class TargetResolution` (`:46`) | `unique_ptr` (`:56`) | yes |
| 7 publishing | `PublishingInput` (10 fields, `publishing.h:27`) | `PublishingOutput{status, brain_state}` (`:40`) | `class Publishing` (`:48`) | `unique_ptr` (`:58`) | **no**: `std::optional<translagatr::BrainState> brain_state;` (`publishing.h:45`) |
| build context for 3, 5, 6, 7 | `SlotInitializationContext` (`slot_init.h:41`) | - | - | - | **no**: `FunctionKey commands_type; ResourceId commands_serial;` (`:61-62`), `BrainProfileHost* brain_profile` (`:66`) |

Inconsistencies a standard should remove:
1. **Two seam styles.**
   - Resources and sensors use a struct of `std::function`, which is the functional approach.
   - The other five use virtual classes.
   - Stateless noops exist only to satisfy the virtual interfaces (`impl/noop/noops.cpp:11-60`).
2. **Three factory return types**: `ResourceInstance`, `std::optional<Executable>` and
   `std::unique_ptr<Interface>`.
3. **Context passing.** Stages 1, 2, 4 and 5 get `ExecutionContext`. Stages 3 and 7 get loose
   `MonotonicTime now` + `Diagnostics*`. Stage 6 gets `now` and no diagnostics.
4. **Output health.** 5, perception and association carry `status` + `diagnostic`. 3, 6 and 7 carry
   `status` only.
5. **Four init-context types:** `ResourceInitializationContext`, `SensorInitializationContext`,
   `RobotObservationInitializationContext`/`StateEstimatorInitializationContext`, and
   `SlotInitializationContext`. The last has grown implementation fields.
6. **Duplicated machinery.** `ResourceFunctions` (`resource_stage.h:35-68`) and `SensorFunctions`
   (`sensor_stage.h:34-66`) are the same class with the id type substituted.

Brain side: explicit and stable, with small single-purpose interfaces:
- `StateSource` (`state_source.h:38`): `robot(now)`, `field(out)`;
- `PathPlanner` (`planner.h:46`): `plan`, `clear`;
- `PathSink` (`path.h:44`);
- `MotorOutput`;
- `BytePort`.

Pico: its I/O is the wire contract (`translaGATR/frames.h`).

### 3.2 Can another team add X without touching the pipeline?

| New thing | Touches | Verdict |
|---|---|---|
| Pi sensor channel | impl files, 1 line in `register_sensors.cpp`, 1 CMake line | yes |
| Pi device resource of a new kind (e.g. an I2C ToF) | a new kind header + impl + register + CMake | yes |
| Localization observation model or estimator, XML-configured | impl + `register_localization.cpp` | yes |
| ...selectable from the Brain robot profile | also `brain_profile_builder.cpp` topology table (`brain_profile_builder.h:49-62`) and the profile document in `translaGATR/link_documents.h` | no, 3 components |
| World estimator | impl + `register_world_estimation.cpp`; the file says so (`register_world_estimation.cpp:2-4` "Adding one means its implementation, a line here, and a build entry") | yes |
| Command source other than the Brain link | `system.cpp:339-343` pairing rule, `system.cpp:363` (`if (commands.attr("type") != "brain_link") {`), `main.cpp:118`, `slot_init.h:61-66`, `CommandState` | no |
| Publisher other than the Brain link | `PublishingOutput::brain_state`, the pairing rule | partly |
| Pico sensor | `translaGATR/frames.h` mask bit + codec, Pico firmware, `pico_telemetry` decode, a sensor channel | no, by design (the wire is the seam); needs a written procedure |
| Brain planner | implement `investigatr::PathPlanner`, pass it to `Motion` | yes |
| Brain state source (sim, other localizer) | implement `investigatr::StateSource` | yes |
| Drivetrain | implement `actugatr::Kinematics` (+ follower variant) | yes |

### 3.3 Proposed rule set (standard I/O)

- **IO-1 One contract file per stage.**
  - `<stage>_contract.h` holds exactly:
    - `struct <Stage>Input`;
    - `struct <Stage>Output`;
    - the declared-output struct;
    - the seam;
    - the init context;
    - the `using <Stage>MakeFunction = ...` alias.
  - Nothing else lives there, and nothing else defines stage I/O.
- **IO-2 Inputs are read-only views.**
  - Every input carries `const ExecutionContext& context` (host clock, cycle, diagnostics).
  - No loose `now` or `Diagnostics*`.
- **IO-3 Outputs carry health.** Every output has `FunctionStatus status` and `std::string diagnostic`.
  A failure is a status, never an empty result.
- **IO-4 Contracts are implementation-neutral.**
  - No wire types (`translagatr::`), device types, implementation names or string compares on a
    registered type name in contract files, stage builders or the coordinator.
  - Implementation extras travel as declared typed payloads in the stage's map, or stay private.
- **IO-5 Declared outputs.** An implementation declares its ids and payload types at build. The stage
  executor enforces them at runtime, as world estimation already does (`world_estimation_stage.h:21-24`).
- **IO-6 Adding an implementation is local.**
  - It touches `<stage>/impl/`, one registration line, one CMake line, a config example and a test.
  - If it needs more, the contract is missing something. Fix the contract first, in its own commit.
- **IO-7 Functional seam first.** New stages use the executable struct form (`std::function` run +
  reset + declarations), like `SensorExecutable`. Existing class seams stay until touched for another
  reason. A virtual interface is allowed when there are two implementations with shared state logic
  worth a base class.
- **IO-8 One factory shape.** `std::optional<<Stage>Executable> make_<registered_name>(const
  ConfigNode&, <Stage>InitContext&, std::string& err)`, a free function named after the XML `type`.
  This makes `grep make_<type>` land on it from the config.

### 3.4 Stage contract template (a template, not repo code)

```cpp
// xyz_contract.h
// Xyz stage: sensor map in, xyz state out.

#pragma once
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "core/execution_context.h"
#include "core/function_status.h"
#include "core/payload_descriptor.h"
#include "core/records.h"

namespace navigatr
{

class ResourceStore;
class SensorCatalog;

struct XyzInput {
    const SensorMap&        sensors;
    const XyzState&         previous;
    const ExecutionContext& context;   // host clock, cycle, diagnostics
};

struct XyzOutput {
    XyzState       state;
    FunctionStatus status = FunctionStatus::kOk;
    std::string    diagnostic;
};

struct XyzOutputDecl {
    XyzId             id;
    PayloadDescriptor payload;
};

struct XyzExecutable {
    std::vector<XyzOutputDecl>                outputs;
    std::function<XyzOutput(const XyzInput&)> run;
    std::function<void()>                     reset;   // may be empty
};

struct XyzInitContext {
    const ResourceStore*      resources = nullptr;
    const SensorCatalog*      sensors   = nullptr;
    std::vector<std::string>* warnings  = nullptr;
};

using XyzMakeFunction =
    std::function<std::optional<XyzExecutable>(const ConfigNode&, XyzInitContext&, std::string& err)>;

} // namespace navigatr
```

## 4. Season-specific leakage and over-generalization

### 4.1 Season (Override) knowledge outside season data

Field size, tag family and match timing are **not** hard-coded in pipeline code:
- no field dimension literal was found in `src/`, Brain libraries or Pico;
- tag families are configuration (`tag_detectors.h:7` shows it as XML);
- `docs/actugatr.md` states the rule "season names live in the application, not in the libraries".

Leaks found:

| Where | What | Severity |
|---|---|---|
| `spectaGATR/field_scene.js:293` | `shortId` strips `neutral_goal_`, `blue_goal_`, `red_goal_`, which are Override landmark id conventions | medium: generic viewer knows season ids |
| `spectaGATR/field_scene.js:690` | `const red = mean('red_'), blue = mean('blue_');` wall labels from id prefixes | low: VRC alliance convention, not season, but an id-spelling contract |
| `config/override/brain_profile_usb.xml`, `brain_profile_rs485.xml`, `brain_profile_usb_camera.xml`, `diagnostics/*` | robot and Pi device configs, season-independent, live in the season folder; `pi/naviGATR/README.md:16-21` calls them the primary configs | medium: next season they sit in last season's folder |
| `pi/naviGATR/CMakeLists.txt:224-239` | `field_references` target hard-codes `config/override/field.xml` -> `brain/operaGATR/include/field_references.h` | low: one path, but in the build file |
| `pi/naviGATR/tests/field_assets_gtest.cpp:49`, `:498-504`; `tests/CMakeLists.txt:63` | the generic runtime's test binary asserts the Override field file and the Brain app's generated header are current | low: right check, wrong place (season data test in the generic suite) |
| comment examples in generic headers | `apriltag_world_estimator.h:21` and `world_estimation_stage.h:9` `<Estimator id="goals"`; `brain_link_publisher.h` `override_field`; `field_map_resource.h:7` `center_goal`; `target_set_resource.h:11` `back_to_center_goal`; `composition.h:7-8` `override_three_wheel_diagnostic`; `field_state.h:3` "seeing a displaced goal moves the goal" | low: teaches season names as the norm |
| `brain/operaGATR/include/field_references.h:4` | `struct Field : investigatr::FieldReferences {` with `NeutralGoal4North` etc. | fine place (app), but global namespace, see 5(d) |

Implementation and hardware knowledge in generic pipeline code is the bigger issue for season reuse,
because it is what a new sensor or link would have to fight:

| Where | What |
|---|---|
| `system.cpp:339-343`, `:363`; `main.cpp:118` | string compares on `"brain_link"` in the coordinator and app |
| `system.h:145-154`, `:189-190` | `PicoOperation`, `kPicoOperationMs`, `kWheelFreshMs` (CONTROL 3/4, READ_WHEELS) on the coordinator |
| `state/command_state.h:30-110` | generic stage output shaped by the Brain link wire protocol |
| `contracts/publishing.h:12`, `:45` | `translaGATR/frames.h` and `BrainState` in the generic publishing contract |
| `contracts/slot_init.h:59-66` | `commands_type`, `commands_serial`, `brain_profile` in the generic build context |
| `runtime/brain_profile_builder.h:87` | `constexpr uint8_t kProfileEncoderPorts = 3;` (the HAT's port count) in runtime code |
| `runtime/pico_control_ref.h:12`, `runtime/resource_stage.cpp:6`, `inspection/inspection_document.cpp:17-18`, `diagnostics/instrumentation.cpp:17` | generic code includes concrete implementations |

### 4.2 Over-generalization

| Where | What | Evidence | Suggestion |
|---|---|---|---|
| `resources/spi_bus.h:35`, `:45` | `class SpiDevice`, `class SpiBus` | zero production implementations; only `tests/resources_gtest.cpp:36` `class FakeSpiDevice : public SpiDevice` and `:59` | delete until an SPI device exists on the Pi |
| `contracts/perception.h:34`, `contracts/association.h:39` | "internal step contracts" | one implementation each, "reachable only through this class; no other component can address them" (`apriltag_world_estimator.h:14-15`) | move beside apriltag (done by the move map); collapse to concrete classes when next touched |
| `contracts/brain_profile.h:40` + `system.cpp:79-108` | `BrainProfileHost` interface | one implementation, whose five methods each forward to a `System` private (`return system_.prepareProfile(...)`) | disappears with the `ProfileSupervisor` extraction |
| `contracts/target_resolution.h:5` | abstract slot + registry | the file itself: "not an open algorithm ecosystem"; the noop and `configured_targets` | keep (the noop serves bench configs) but say so in the rule: noop + one real = two |
| `core/function_registry.h:87-157` | one registry keyed by `(name, std::type_index)` holding `std::any` for 9 factory categories | every category already has its own typed signature | borderline; typed maps per stage would drop `std::any` and the `Tag<T>` trick (`system.cpp:30-33`) |
| `runtime/inspection_config.h:21-22`, `:82` | `snapshot_hz`: "still accepted and validated so older profiles load" | used only to echo into the document (`inspection_document.cpp:778`), set in 13 configs incl. `config/override/brain_profile_usb.xml:19` | remove the knob and the 13 attributes |
| `runtime/inspection_config.h:46-64` | `state_hz`, `diag_hz`, `max_clients`, `client_buffer_kb`, `reliable_kb`, `send_buffer_kb`, `ack_window_kb`, `stall_close_ms`, `static_root` | set by no config (tests set some) | keep only knobs with a documented tuning use; others become constants |
| `resources/target_set.h:19`; `target_set_resource.cpp:31-32` | `continuous` vision policy "reserved for a future implementation" | parsed only to be rejected | remove the enum value |
| `state/command_state.h:96-102` | `object_requested`, `object_wire_id`, `object_sequence`: "Brain link v4 has no selection op; only in-process callers set it" | no producer on the robot | keep only if a test harness needs it; say so in the contract |
| `world_estimation_stage.h:17-18` | "Exactly one Estimator is configured for now: combining several is a later concern" | section schema and id for one estimator | fine as long as nothing is built for "several" |

For balance: the Brain libraries have no single-implementation interfaces. Every seam there has a real and
a sim/test implementation.

## 5. Types

Scan: libclang, production code excluding tests, sims and tools unless stated.

| Component | classes | structs | pure interfaces | (a) data-only class | (b) struct with methods | (c) static-only | 40+ member classes |
|---|---|---|---|---|---|---|---|
| naviGATR src | 95 | 271 | 21 | 0 | 26 | 1 | 3 (`System` 70, `PicoTelemetry` 56, `CaptureRecorder` 45) |
| communiGATR | 14 | 40 | 1 | 0 (1 RAII `Lock`) | 2 | 0 | 1 (`Client` 73) |
| actuGATR | 16 | 21 | 3 | 0 | 0 | 0 | 0 |
| investiGATR | 4 | 31 | 3 | 0 | 3 | 0 | 0 |
| translaGATR | 1 | 18 | 0 | 0 | 0 | 0 | 0 |
| aggreGATR | 10 | 13 | 0 | 0 | 0 | 0 | 0 |
| locaGATR, operaGATR | 0 | 3 | 0 | 0 | 0 | 0 | 0 |

**(a) Classes that are data-only.** None in production code. The codebase already uses `struct` for plain
data. The rule needs no cleanup pass, only enforcement.

**(b) Structs that behave like classes.** Split the 31 hits:
- **Fine as structs: const query helpers, no invariant.** A rule should allow these explicitly:
  - `FieldMap::find` (`config/field_map.h:159`), `LandmarkDecl::findApproach` (`:106`);
  - `RobotFrameMap::find` (`resources/robot_frames.h:19`);
  - `TargetSet::findById` (`resources/target_set.h:90`);
  - `RobotState::fieldPose` (`state/robot_state.h:44`);
  - `MonotonicTime::isSet` (`core/time.h:23`);
  - `PayloadDescriptor::matches` (`core/payload_descriptor.h:13`);
  - `investigatr::RobotState::valid` (`state_source.h:27`), `investigatr::Field::find` (`field.h:59`),
    `Path::length` (`path.h:35`).
- **Should be classes: mutators that keep an invariant.**
  - `BrainImuBench` (`resources/brain_imu_bench.h:14`, `reset`/`accept`/`acceptAttitude` maintain
    sequence and epoch);
  - `PlacementEdge` (`impl/localization/motion_step.h:63`, `apply`/`reset`);
  - `PlanarStepState` (`motion_step.h:172`);
  - `Diagnostics::note` (`core/diagnostics.h:43`);
  - `DocBuildStats::add` (`inspection/inspection_stats.h:61`);
  - `SlotInitializationContext` (`contracts/slot_init.h:41`) has a `private:` section.
- **Should be classes: pimpl structs with many members and methods.**
  - `HttpServer` `Impl` (`inspection/http_server.cpp:401`) and `Client` (`:440`, 40 members);
  - `InspectionService` `Impl` (`inspection/inspection_service.cpp:250`, 33 members);
  - `LibcameraCamera` `Impl` (`impl/resources/libcamera_camera.cpp:132`, 40 members);
  - communiGATR `Io` (`pros/pros_usb_port.cpp:69`, `transmit`/`receive`).

**(c) Static-only classes, or classes where a function would do.**
- `HostClock` (`core/host_clock.h:16`) has only `static` `now`, `nowUs`, `origin`. It should be a
  namespace with free functions.
- Classes made for their own sake:
  - `System::ProfileHost` (`system.cpp:79-108`) only forwards.
  - The four field-less noop classes (`noops.cpp:11`, `:37`, `:47`, `:55`) exist to satisfy virtual
    seams. With a functional seam they are one lambda each.
  - `ResourceFunctions`/`SensorFunctions` are duplicated classes (section 3.1). One template or one free
    helper would do.

**(d) Code outside any namespace.**
- Globals that frameworks require are fine: `main`, Arduino `setup`/`loop` (`pico/aggreGATR/src/main.cpp:299`,
  `:319`), PROS `initialize`/`opcontrol`/... (`operaGATR/src/main.cpp:390`, `:423`) and gtest `TEST`
  bodies.
- Real misses:
  - `brain/operaGATR/include/field_references.h:4`: generated `struct Field` at global scope. The
    generator `fieldReferencesHeader` in `impl/publishing/field_documents.cpp` should wrap it in a
    namespace.
  - `brain/communiGATR/tests/client_telemetry_gtest.cpp:524`: `struct BusRun` global, in a test.
  - `pi/naviGATR/src/impl/resources/apriltag_c.{c,h}`: C; exempt as a C shim.
- No component root:
  - Pico has seven top-level namespaces (section 2).
  - Brain apps use `robot_config` twice (`locaGATR/include/robot_config.h:13`,
    `operaGATR/include/robot_config.h:24`). Separate programs, so acceptable.
- `using namespace` appears only in test files (`brain/*/tests/*`, `using namespace actugatr;`). There are
  no header `using namespace`, so the rule holds already.

## 6. Pipeline docs

### 6.1 What exists today

| Pipeline | Where it is explained | Problem |
|---|---|---|
| whole system | `README.md:7-10` (3-line diagram), `docs/interfaces.md:23-50` (wire view) | no single doc with stages, I/O, rates and failure behavior |
| naviGATR | `pi/naviGATR/README.md:57-87`; `system.h:1-84`; `docs/architecture.md` (381 lines); `pi/README.md` repeats the summary | scattered; no file paths; header comment far longer than the comment style allows |
| naviGATR localization | `contracts/localization.h:1-18`, `docs/architecture.md:82-163`, `docs/localization_fusion.md` | good content, three places |
| naviGATR world estimation (apriltag) | `impl/world_estimation/apriltag_world_estimator.h:1-43` | good and concise, but a header comment, not a doc |
| naviGATR Brain profile | `runtime/brain_profile_builder.h:1-66`, `docs/brain_profile.md` (384 lines), `system.h:48-83` | three places |
| communiGATR | `docs/communigatr.md:39-54` layers | good; stale "testing" name |
| actuGATR | `docs/actugatr.md:1-20` | good |
| aggreGATR | `pico/aggreGATR/README.md` | good, but protocol detail, not the loop |
| spectaGATR | `spectaGATR/app.js:1-21` | header comment only |

There is no `PIPELINE.md` anywhere. The content exists; it needs one concise place per pipeline that
points at code.

### 6.2 PIPELINE.md template

Rules for the template:
- about 60 lines at most;
- links to folders and files, never line numbers;
- updated in the same commit as the code it describes;
- plain ASCII;
- terse labels per the comment style.

```markdown
# <Name> pipeline

<What goes in, what comes out, two lines at most.>

## Start here
- <entry file>: <what it does>
- <construction file>: <what it builds, in order>
- <per-cycle file>: <the order each cycle>

## Input
- <what> from <who>: `<Type>` in [<file>](<path>)

## Output
- <what> to <who>: `<Type>` in [<file>](<path>)

## Stages
| # | Folder | In | Out |
|---|---|---|---|
| 1 | [<folder>/](<folder>/) | `<Type>` | `<Type>` |

## Adding an implementation
1. <files to add, where>
2. <registration line, where>
3. <build list line, where>
4. <config example and test, where>

## Season-specific
<what is, and where it lives; "nothing" is a valid answer>

## Threads and rates
- <thread>: <stages>, <rate>

## Failure behavior
- <input missing, stale, or device failing>: <what happens, who sees it>
```

### 6.3 Draft: system-wide (`PIPELINE.md` at the repo root)

```markdown
# GATR2 pipeline

Where the robot is and where things around it are, measured on custom
electronics and handed to the V5 Brain, which plans and drives.

## Flow
encoders, IMU -> Pico (aggreGATR) -UART-> Pi (naviGATR) <-Brain link-> Brain (communiGATR -> actuGATR -> motors)
                                 camera -^        |
                                                  +-HTTP/WebSocket-> browser (spectaGATR)

## Stages
| # | Component | In | Out | Contract |
|---|---|---|---|---|
| 1 | [pico/aggreGATR](pico/aggreGATR/) | 3 quadrature encoders, 1 IMU (BNO08X or ASM330, build time), Pi commands | sensor frames, status, diag | [translaGATR/frames.h](translaGATR/frames.h) |
| 2 | [pi/naviGATR](pi/naviGATR/PIPELINE.md) | Pico frames, camera frames, Brain requests and robot profile | robot pose, field-object estimates, Brain replies | [pi/naviGATR/PIPELINE.md](pi/naviGATR/PIPELINE.md) |
| 3 | [brain/communiGATR](brain/communiGATR/) | Brain link replies | robot state, field: `investigatr::StateSource` | [state_source.h](brain/investiGATR/include/investigatr/state_source.h) |
| 4 | [brain/investiGATR](brain/investiGATR/) | start, goal, motion model, field | path or refusal | [planner.h](brain/investiGATR/include/investigatr/planner.h) |
| 5 | [brain/actuGATR](brain/actuGATR/) | goToDirect, goToAvoiding, state, planner | motor commands, motion status | [motion.h](brain/actuGATR/include/actugatr/motion.h) |
| 6 | [pi/naviGATR/spectaGATR](pi/naviGATR/spectaGATR/) | inspect/2 feed, capture ZIPs | the viewer page | [inspection.md](pi/naviGATR/docs/inspection.md) |

Programs that assemble 3 to 5: [locaGATR](brain/locaGATR/) (bench, no motors),
[operaGATR](brain/operaGATR/) (drive). Every link's bytes: [translaGATR](translaGATR/),
spec in [docs/interfaces.md](docs/interfaces.md).

## Threads and rates
- Pico: sensor frame every 20 ms (50 Hz), UART 115200; status every 200 ms
- Pi: estimation loop at the config's Loop rate_hz (100 in the robot configs);
  field worker per new sensor snapshot, latest wins
- Brain: GET_STATE every 20 ms, field estimate every 0.5 s, TELEMETRY every 0.1 s
- viewer: state up to 30 Hz, diag 4 Hz, camera preview 5 Hz (defaults)

## Season-specific
- field: pi/naviGATR/config/override/field.xml (landmarks, tag mounts, wire ids, obstacles)
- field names for Brain programs: brain/operaGATR/include/field_references.h, generated from it
- program behavior and test destinations: brain/<program>/include/robot_config.h
Robot, not season: brain/robot/gatr2_robot.h (sent to the Pi as the robot profile),
Pi devices in the Pi XML, Pico pins in pico/aggreGATR/src/board.h.

## Failure behavior
- Pico IMU missing or aligning: gyro bit clear, encoders keep flowing
- Pi loses a used source (stale, restarted, rebooted): robot unplaced, Brain places again
- Brain link quiet: Brain reports no link; motion waits, then fails
- slow viewer: skipped, never waited for
- any device reboots mid-match: frames resync on the sync bytes
```

Facts in the draft, with sources:
- 50 Hz: `pico/aggreGATR/src/config.h:24` `constexpr uint32_t kSampleHz = 50;`
- 200 ms: `:30` `kStatusPeriodUs = 200000`
- Brain periods: `communigatr/client.h:49`, `:59`, `:61`
- viewer defaults: `runtime/inspection_config.h:47-49`
- 100 Hz: `config/override/brain_profile_usb.xml:18` `<Loop rate_hz="100"/>`
- IMU degrade: `pico/aggreGATR/README.md` ("A missing or failed IMU ... leaves the gyro bit clear")

### 6.4 Draft: `pi/naviGATR/PIPELINE.md` (paths as of today; the stage column becomes `src/N_stage/` after the move)

```markdown
# naviGATR pipeline

Pico, camera and Brain inputs in; robot pose, field-object estimates and
Brain replies out. One process, one XML configuration.

## Start here
- src/app/main.cpp: registerAll, System::buildFromFile, then workers or --inline
- System::build in src/runtime/system.cpp: the construction order
- System::step, estimationCycle, fieldCycle, reportingCycle in src/runtime/system.cpp: one cycle

## Input
- Pico frames: pico_telemetry resource on a serial link
- camera frames: libcamera_camera resource (Pi build) or synthetic_rig
- Brain requests and robot profile: brain_link CommandCollection on
  linux_serial_link (RS-485) or pros_usb_link (V5 USB)
- configuration: one Configuration or System XML under config/

## Output
- Brain replies (state block, field documents): brain_link Publishing
- inspection feed and viewer: src/inspection/, read-only
- capture bundles: src/capture/, read-only

## Stages
| # | XML | Builder | Out | Code |
|---|---|---|---|---|
| 1 | Resources | make_resources | ResourceMap | runtime/resource_stage.*, impl/resources/ |
| 2 | Sensors | make_sensors | SensorMap | runtime/sensor_stage.*, impl/sensors/ |
| 3 | CommandCollection | slot factory | CommandState | contracts/commands.h, impl/commands/ |
| 4 | Localization | make_localization | RobotState, pose history | runtime/localization_stage.*, impl/localization/ |
| 5 | WorldEstimation | make_world_estimation | FieldState, evidence | runtime/world_estimation_stage.*, impl/world_estimation/ |
| 6 | TargetResolution | slot factory | TargetState | contracts/target_resolution.h, impl/target_resolution/ |
| 7 | Publishing | slot factory | Brain reply | contracts/publishing.h, impl/publishing/ |

Build order is 1 to 7; any failure destroys the candidate and nothing runs.

## Adding an implementation
1. files in impl/<stage>/ with a factory
2. one registerOrDie line in impl/<stage>/register_<stage>.cpp
3. the .cpp in NAVIGATR_SOURCES in CMakeLists.txt
4. a config example and a test in tests/
Coordinator and stage builders stay as they are. Not yet true for
CommandCollection and Publishing: brain_link must be selected for both.

## Brain profile
With BrainProfile under Localization, stages 2 and 4 come from the robot
profile the Brain sends (runtime/brain_profile_builder.*) and swap in at a
boundary; until then localization is noop. Sensor loss, CONTROL and
READ_WHEELS belong to this feature. Details: docs/brain_profile.md.

## Season-specific
config/override/field.xml only. Nothing in src/.

## Threads and rates
- estimation worker, Loop rate_hz: 1, 2, 3, 4, then 6, 7 against the newest
  field snapshot; hands its sensor map to the field worker
- field worker, per sensor snapshot, latest wins: 5
- main thread: applies a pending Brain profile every 20 ms
- inspection service, capture recorder: own threads, read snapshots only
- --inline: 1 to 7 in order on one thread (System::step)

## Failure behavior
- configuration error: nothing runs, exit 1
- device fails to open: build warning, its outputs report no data
- used source lost (Brain profile): robot unplaced, event logged, Brain places again
- field worker behind: the pending snapshot is replaced and counted
- slow browser: skipped, never waited for
```

Sources:
- atomic build: `system.cpp:134`;
- open failure is a warning: `main.cpp:2-3` "A device that fails to open is a build warning and a runtime
  status";
- 20 ms main loop: `main.cpp:174-176`;
- latest wins: `runtime/handoff.h:5-9`;
- unplace: `system.h:64-73`.

### 6.5 Draft: a sub-pipeline (`src/5_world_estimation/impl/apriltag/PIPELINE.md`)

```markdown
# AprilTag world estimation

Camera frames in; landmark pose estimates and tag evidence out. One
selectable estimator (type apriltag); the coordinator sees only the
world estimation contract.

## Steps
| # | Step | In | Out | Code |
|---|---|---|---|---|
| 1 | ObservationExtraction | camera frame (SensorMap) | tag observations | apriltag_tag_observation.* |
| 2 | Association, optional | observations, pose at exposure, field map | landmark pose evidence | tag_mount_association.* |
| 3 | LandmarkEstimation | evidence | committed landmark estimates | landmark_estimator.* |

## Failure behavior
- a step fault: previous field kept, nothing published
- no Association: observations still publish, no landmark commits

## Season-specific
none here; tags, mounts and landmarks come from the field_map resource
```

Source: `impl/world_estimation/apriltag_world_estimator.h:1-43`, condensed.

## 7. Rules this survey proposes for the standard

Structure:
- **ST-1 Folder mirrors the pipeline.**
  - Every pipeline is one folder. Every stage is one subfolder holding its contract, builder,
    registration, output types and `impl/`.
  - A fixed stage order is shown by numeric folder prefixes (D1).
- **ST-2 PIPELINE.md.**
  - One for the repo, one per component pipeline, one per stage and per sub-pipeline, using the 6.2
    template.
  - Updated in the same commit as the code; reviewers check it.
- **ST-3 Entry points by name.**
  - Construction lives in `build.*`, the per-cycle order in `cycle.*`.
  - The per-cycle function reads top to bottom as the stage list and fits on one screen.
  - Feature logic calls out of it; it is never inlined.
- **ST-4 One responsibility per file.** The coordinator hosts no feature (Brain profile, device control,
  recovery, snapshot assembly). A file past about 800 lines, or a class past about 30 members, needs a
  stated reason in review.
- **ST-5 Dependency direction.**
  - Nothing outside an `impl/` folder includes from an `impl/` folder, except `register_<stage>.cpp` and
    `tools/`.
  - Enforceable with one grep in CI.
- **ST-6 No catch-all folders.** `runtime/`, `misc/`, `utils/` or `common/` are not allowed unless the
  folder holds one stated role.
- **ST-7 Unique basenames within a component.**
- **ST-8 Season data only in `config/<season>/` and app folders.**
  - Season-independent robot and device configs live outside the season folder.
  - Generic code, its tests and its comment examples use neutral names.
- **ST-9 Two implementations before an interface.**
  - A noop that serves a real configuration counts.
  - A test fake counts only at a hardware or process boundary.
  - Otherwise the code stays concrete.
- **ST-10 Every config knob is used.**
  - A knob is set by a shipped config, or documented as a tuning knob with a reason.
  - Legacy knobs are deleted, not kept "so older profiles load".

Standard I/O: IO-1 to IO-8 (section 3.3).

Types:
- **TY-1 struct vs class.**
  - `struct` is public data with at most const query helpers and no invariant.
  - `class` is private data plus the methods that keep its invariant.
  - A pimpl `Impl` with methods is a class.
- **TY-2 Static-only types become namespaces.** A type with only static members is a namespace with free
  functions.
- **TY-3 Functional first.** Prefer a free function or a struct of `std::function` over an interface class
  with one implementation.
- **TY-4 Namespaces.**
  - Every symbol is in a namespace named after its component (`navigatr`, `communigatr`, `aggregatr`,
    ...), generated code included.
  - Framework entry points are the only globals.
  - No `using namespace` in headers.
  - Using-declarations that re-export another component's names in a public header need a reason.

Naming:
- **NM-1 One name per concept** across XML element, folder, contract, class and doc (WorldEstimation, not
  also FieldEstimation).
- **NM-2 Factories are `make_<registered type>`** and registration functions are `register_<stage>`, so
  `grep make_<type>` goes from XML to code.

## 8. Open decisions for the user

- **D1** Numbered stage folders (`src/4_localization/`) or plain names with the order in `PIPELINE.md`?
  Numbers make the order visible in every file listing. The cost is digits in include paths.
- **D2** Do the full move (commits 1-11) or only the gripe fix (commits 1-3, about 35 files) now?
- **D3** naviGATR "C++ only" versus what is there:
  - Python (`tools/calibrate_camera.py`, viewer test harness);
  - JS (spectaGATR, which is its own component and served by the binary);
  - shell.

  Keep the rule for `src/` and list justified tool exceptions in `tools/README.md`?
- **D4** Move `config/override/brain_profile_*.xml` and `diagnostics/` out of the season folder to
  `config/robot/`? The Pi service and docs reference the current paths.
- **D5** Unify all stage seams on the functional executable form (IO-7) over time, or only for new stages?
