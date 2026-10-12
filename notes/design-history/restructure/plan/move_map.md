# naviGATR src move map

Planning step, read-only. Worktree `S/wt-navi` at ae457ee, nothing in it was changed.

- Inventory: 201 files under `pi/naviGATR/src`. 189 move, 12 stay (`config/` 7, `math/` 5).
- Machine-readable map: `plan/move_map.tsv`, 201 lines, `old<TAB>new`, relative to `pi/naviGATR/`, `same` for files that stay.
- Generator and checks: `scripts/move_map.py` (the map itself lives in it). Tables: `scripts/move_map_tables.py`.
- Include graph (who includes each src file, whole worktree): `logs/include_graph.txt`, `logs/include_graph.json`, from `scripts/include_graph.py`.
- Every reference that changes, line by line: `logs/move_map_refs.txt`.

Checks `scripts/move_map.py` ran on the map, all passing (errors: 0):
- every inventoried file is mapped exactly once (201/201), and nothing maps a file that does not exist;
- no two files get the same destination, and no noop split piece collides with a moved file;
- no destination uses a folder named `impl`;
- `app/ contracts/ payloads/ state/ runtime/ impl/ resources/ transport/`, the old `core/`, `capture/ diagnostics/ inspection/` are all empty afterwards.

Every naviGATR include of a src file is src-relative (for example `"core/time.h"`). None resolves relative to the including file, so a move only changes the include string.

## Resulting tree

```
pi/naviGATR/src/
  main.cpp
  cli/                              build_config.h.in, command_line.*
  core/                             system.h, system.cpp (split later), handoff.h
  pipeline/
    register_all.h, register_all.cpp
    framework/                      ids, records, typed_payload, payload_descriptor, function_registry,
                                    function_status, execution_context, diagnostics, slot_init,
                                    record_bookkeeping, build_options
    resources/                      contract.h, stage.*, resource_catalog, resource_store.*, resource_instance,
                                    resource types (camera, tag_detector, pico_control, pico_control_ref,
                                    brain_imu_bench, robot_frames, target_set, wheel_geometry),
                                    pico_telemetry_samples.h
      resources/                    <Resource type> plug-ins + register_resources.cpp (26 files)
    sensors/                        contract.h, stage.*, sensor_catalog.h,
                                    sensor_samples.h, attitude_samples.h, camera_frames.h
      sensors/                      <Sensor type> plug-ins + register_sensors.cpp
    commands/                       contract.h, command_state.h
      collectors/                   brain_link_commands.*, noop.*, register_commands.cpp
    localization/                   contract.h, stage.*, robot_state.h, robot_state_feed.*, pose_history.*,
                                    attitude.h, stillness.h, robot_observations.h, register_localization.cpp
      observations/                 tracking_wheel_motion, imu_heading_increment, attitude_reference,
                                    brain_imu_wheel_bench, stationary_window
      estimators/                   planar_motion_integrator, weighted_planar_fusion, motion_step, noop
    world_estimation/               contract.h, stage.*, perception.h, association.h, evidence.h,
                                    field_state.h, landmark_estimator.*
      estimators/                   register_world_estimation.cpp
        apriltag/                   apriltag_world_estimator.*, apriltag_tag_observation.*,
                                    tag_mount_association.*, tag_observations.h
                                    (+ tag trace header in step 3)
        noop/                       noop.*
    targets/                        contract.h, target_state.h
      resolvers/                    configured_targets.*, noop.*, register_target_resolution.cpp
    publishing/                     contract.h
      publishers/                   brain_link_publisher.*, field_documents.*, noop.*, register_publishers.cpp
  brain_profile/                    brain_profile.h, brain_profile_builder.*, sensor_loss.*, stationary_precheck.*
  config/                           unchanged
  io/                               byte_stream.h, file_stream.*, half_duplex.*, serial_port.*, serial_link.h, spi_bus.h
  time/                             time.h, host_clock.h, clock_sync.h
  math/                             unchanged
  debug/
    inspection/                     old inspection/ + inspection_config.h, inspection_state.h
    capture/                        old capture/
    diagnostics/                    old diagnostics/
pi/naviGATR/tools/field_refs/       field_refs.cpp
```

No stage builder exists for commands, targets or publishing: `System::build` builds those three slots itself (`buildSlot`, runtime/system.cpp:276, 329, 335), so those folders have no `stage.*`.

## Plug-in folders and the XML element behind each

| plug-in folder | XML element that selects the plug-in | where the builder looks it up | `type` values in config/** (count) |
|---|---|---|---|
| `pipeline/resources/resources/` | `<Resource type>` in `<Resources>` | runtime/resource_stage.cpp:99 | linux_serial_link 11, pico_telemetry 7, wheel_geometry 6, brain_imu_bench 5, apriltag_detector 4, robot_frame_map 4, libcamera_camera 3, pros_usb_link 3, synthetic_rig 2, field_map 1, memory_link 1 |
| `pipeline/sensors/sensors/` | `<Sensor type>` in `<Sensors>` | runtime/sensor_stage.cpp:96 | pico_encoder_channel 15, camera_frame 5, pico_imu_channel 4, attitude_channel 2 |
| `pipeline/commands/collectors/` | `<CommandCollection type>` (the slot element itself) | runtime/system.cpp:276, `type` read at :250 | brain_link 10, noop 4 |
| `pipeline/localization/observations/` | `<Observation type>` in `<Localization>` | runtime/localization_stage.cpp:223 | tracking_wheel_motion 6, attitude_reference 2, imu_heading_increment 1, brain_imu_parallel_bench 1, brain_imu_planar_bench 1 |
| `pipeline/localization/estimators/` | `<Estimator type>` in `<Localization>` | runtime/localization_stage.cpp:268 | planar_motion_integrator 7, noop 2, weighted_planar_fusion 1 |
| `pipeline/world_estimation/estimators/` | `<Estimator id type>` in `<WorldEstimation>` | runtime/world_estimation_stage.cpp:85 | noop 8, apriltag 6 |
| `pipeline/targets/resolvers/` | `<TargetResolution type>` (the slot element itself) | runtime/system.cpp:329 | noop 14 |
| `pipeline/publishing/publishers/` | `<Publishing type>` (the slot element itself) | runtime/system.cpp:335 | brain_link 10, noop 4 |

No `<Function>`, `<Command>` or `<Publisher>` element exists in config/** or in any builder. The three typed slots (CommandCollection, TargetResolution, Publishing) pick their plug-in with a `type` attribute on the slot element itself, so there is no separate plug-in element to pluralize (see D2).

## Decisions to confirm

The map applies each of these. Changing one is a one-line edit in `scripts/move_map.py`, then rerunning it.

- **D1. `<Resource>` and `<Sensor>` give doubled names.** The literal rule gives `pipeline/resources/resources/` and `pipeline/sensors/sensors/`, and the map uses them.
  - Alternative: family folders directly under the stage, for example resources `links/ pico/ camera/ apriltag/ geometry/ synthetic/`, with `register_resources.cpp` at stage level.
- **D2. The typed slots use agent nouns.**
  - The map uses `commands/collectors/`, `targets/resolvers/` and `publishing/publishers/`.
  - The literal plurals are `command_collections/` and `target_resolutions/`, and "Publishing" has no plural.
  - The agent nouns match the code: `register_publishers`, `BrainLinkPublisher`, and "resolver" in configured_targets.cpp and contracts/target_resolution.h:8.
- **D3. Plug-in folders are flat.** Implementations sit directly in the plug-in folder with their current basenames.
  - The only subfolders are `world_estimation/estimators/apriltag/` and `estimators/noop/`, as SPEC says (the sensor-group model).
  - The noops of the other stages are flat `noop.h/.cpp`.
  - Alternative: one subfolder per implementation in every stage.
- **D4. `register_world_estimation.cpp` goes in `world_estimation/estimators/`, not in `apriltag/`.** It registers noop as well as apriltag.
  - SPEC's apriltag bullet says "plus its register file". Move it one level down if the literal reading is wanted.
- **D5. `register_localization.cpp` is at localization stage level.** It registers both `<Observation>` and `<Estimator>` plug-ins.
  - Alternative: split it into `observations/register_observations.cpp` and `estimators/register_estimators.cpp`. That needs a new register_* function, so it is not done in the move.
- **D6. `register_all.*` goes in `pipeline/`, not `core/` or `framework/`.**
  - It is the stage list of plug-in registrations, and all 7 plug-in register files include it for `registerOrDie`.
  - In `pipeline/`, plug-ins never include `core/`, and `framework/` stays stage-agnostic.
- **D7. `build_options.h` goes in `pipeline/framework/`.**
  - Its only consumer below core is `make_resources` (replay map, diagnostics hub). cli/ and core/ also use it.
  - In `core/` or `cli/`, the resources stage would include upward.
- **D8. `serial_link.h` and `spi_bus.h` go in `io/`, not `pipeline/resources/`.**
  - `io/half_duplex.h` needs `SerialLink`, `ByteSpan` and `TransmitWindow`, and `commands/command_state.h` needs `TransmitWindow`.
  - Keeping both in `io/` means io never includes pipeline/.
- **D9. The sensor.* payloads live in `pipeline/sensors/`.** These are `sensor_samples.h`, `attitude_samples.h` and `camera_frames.h`.
  - They are the SensorMap output types, with stable names `sensor.encoder_sample`, `sensor.imu_sample`, `sensor.attitude_sample` and `sensor.camera_frame`. Downstream stages bind them by sensor id.
  - The camera resources and the synthetic rig already construct `CameraFramePayload` and raw `AttitudeSample`. So after the move, resource plug-ins include sensors headers: cameras.cpp and synthetic_rig.cpp.
  - Alternative: put `camera_frames.h` beside `resources/camera.h`.
  - `pico_telemetry_samples.h` (the `pico.*` names) stays with resources.
- **D10. `inspection_state.h` and `inspection_config.h` go in `debug/inspection/`, not `core/`.** They are inspection's data and policy, and core/ stays "build and run".
- **D11. Shared plug-in helpers sit in their plug-in folder, not at stage level.**
  - `stationary_window.*` goes in `observations/`: only observation models use it.
  - `motion_step.*` goes in `estimators/`: only the two planar estimators use it.
  - The stage builder uses neither.
- **D12. `field_documents.*` go in `publishing/publishers/`.**
  - They are the Brain wire encoding the brain_link publisher sends.
  - tools/field_refs and debug/inspection also include them.
- **D13. `field_refs` stays a top-level CMake target.** `add_executable(navigatr_field_refs tools/field_refs/field_refs.cpp)` stays in `pi/naviGATR/CMakeLists.txt`, and there is no new tools/field_refs/CMakeLists.txt.
  - The target name `navigatr_field_refs` and the `field_references` custom target are unchanged.
- **D14. Do the noop split in the move step.**
  - The TSV maps `noops.*` to its git mv target, `world_estimation/estimators/noop/noop.*`.
  - Moved alone, it would leave all five noops there, with every register file including a world-estimation header.
  - Doing the split in the same step avoids that state. It is code moved between files, no logic change.
- **D15. `slot_init.h` goes in `pipeline/framework/`.**
  - It is the build context every slot factory receives, and it accumulates declarations from earlier stages.
  - It includes `pipeline/localization/contract.h` for `RobotObservationOutputDecl`, a framework-to-stage include (see Layering notes).
- **D16. Line-1 filename comments.** Files that get a shared basename, and the 7 payload headers that already carry a folder, use the src-relative path, for example `// pipeline/commands/contract.h`.

## Noop split (impl/noop/noops.h, noops.cpp -> 5 pieces)

Today `noops.h` declares five factories and `noops.cpp` defines five classes in an anonymous namespace. Each piece takes one of each, unchanged:

| stage | header / source | factory (declared in .h) | class (anon ns in .cpp) | .h includes | registered in |
|---|---|---|---|---|---|
| commands | `pipeline/commands/collectors/noop.h`, `noop.cpp` | `makeNoopCommands` | `NoopCommands` | `pipeline/commands/contract.h`, `<memory>`, `<string>` | `pipeline/commands/collectors/register_commands.cpp` |
| localization | `pipeline/localization/estimators/noop.h`, `noop.cpp` | `makeNoopStateEstimator` | `NoopStateEstimator` (has `std::string type_`) | `pipeline/localization/contract.h`, `<memory>`, `<string>` | `pipeline/localization/register_localization.cpp` |
| world estimation | `pipeline/world_estimation/estimators/noop/noop.h`, `noop.cpp` (the git mv target) | `makeNoopFieldEstimation` | `NoopFieldEstimation` | `pipeline/world_estimation/contract.h`, `<memory>`, `<string>` | `pipeline/world_estimation/estimators/register_world_estimation.cpp` |
| targets | `pipeline/targets/resolvers/noop.h`, `noop.cpp` | `makeNoopTargetResolution` | `NoopTargetResolution` | `pipeline/targets/contract.h`, `<memory>`, `<string>` | `pipeline/targets/resolvers/register_target_resolution.cpp` |
| publishing | `pipeline/publishing/publishers/noop.h`, `noop.cpp` | `makeNoopPublishing` | `NoopPublishing` | `pipeline/publishing/contract.h`, `<memory>`, `<string>` | `pipeline/publishing/publishers/register_publishers.cpp` |

- **Includers of noops.h today (7):**
  - the 5 register files above;
  - noops.cpp;
  - tests/system_build_gtest.cpp:12.
- **system_build_gtest.cpp uses no noop symbol.** It gets `Commands`, `StateEstimator`, `FieldEstimation`, `TargetResolution`, `Publishing` and `SlotInitializationContext` through runtime/system.h.
  - Drop the include, or replace it with the stage contract headers it uses.
- **Header comment.** Each piece keeps its own line from the noops.h header block, for example `// CommandCollection noop: carries the previous command state forward`.
- **CMake.** `NAVIGATR_SOURCES` loses `src/impl/noop/noops.cpp` and gains the 5 `noop.cpp` paths.

## Tag trace split (note only; the world-estimation step does it)

- **What moves.** Four things leave `pipeline/world_estimation/evidence.h` (today payloads/field_object_evidence.h:54-87 and :34):
  - `TagAssociationCandidate`
  - `TagAssociationTrace`
  - `TagAssociationTraceSet`
  - `payload_names::kTagAssociationTraceSet`

  They go to a new header in `pipeline/world_estimation/estimators/apriltag/`, suggested name `tag_association_trace.h`.
- **What stays in evidence.h:** `FieldObjectPoseEvidence`, `FieldObjectPoseEvidenceSet` and `kFieldObjectPoseEvidenceSet`.
  - Its `pose_history.h` include exists only for `TagAssociationTraceSet::exposure_context` (`PoseSampleLookupResult`), so that include moves with the trace.
- **Users of the trace types that will need the new include (grep of `TagAssociation*|kTagAssociationTraceSet`):**
  - impl/world_estimation/tag_mount_association.cpp;
  - runtime/inspection_state.h (`DetectionFrameSnapshot::trace`), and through it inspection/inspection_document.cpp and runtime/system.cpp:1241;
  - tests/capture_transform_gtest.cpp, tests/synthetic_rig_gtest.cpp, tests/vision_targets_gtest.cpp.
- **Users of the generic evidence types, which keep evidence.h:**
  - configured_targets.cpp, apriltag_world_estimator.cpp, landmark_estimator.h/.cpp and tag_mount_association.cpp;
  - tests/capture_transform_gtest.cpp, tests/localization_fusion_gtest.cpp, tests/world_estimation_gtest.cpp.

## Non-src references that must change

Exact line lists are in `logs/move_map_refs.txt`.

**Includes (744 lines in 206 files, before the noop split):**
- src: 168 files, 534 lines;
- tests: 37 of the 38 `*_gtest.cpp`, 205 lines. transforms_gtest.cpp (math only) and inspection_test_client.h need nothing;
- tools/perf/inspect_bench.cpp: 5 lines (system.h, register_all.h, host_clock.h, inspection_document.h, inspection_service.h).

**CMake:**
- `pi/naviGATR/CMakeLists.txt`:
  - `NAVIGATR_SOURCES` lines 75-138: 64 paths change. The 3 `src/config/*.cpp` lines stay. Add the noop pieces (see Noop split);
  - line 193: libcamera `target_sources` becomes `src/pipeline/resources/resources/libcamera_camera.cpp`;
  - line 216: `configure_file(src/cli/build_config.h.in ...)`;
  - line 217: `add_executable(navigatr src/main.cpp src/cli/command_line.cpp)`;
  - line 227: `add_executable(navigatr_field_refs tools/field_refs/field_refs.cpp)`.
- `pi/naviGATR/tests/CMakeLists.txt:21`: `../src/app/command_line.cpp` becomes `../src/cli/command_line.cpp`.
- `pi/naviGATR/cmake/embed_assets.cmake:36`: the generated `viewer_assets.cpp` writes `#include "inspection/viewer_assets.h"`. It must write `debug/inspection/viewer_assets.h`, or the generated file fails to compile.
- `pi/naviGATR/tools/perf/CMakeLists.txt`: no change. It links `navigatr_core`, which exports `src/` as its include dir.

**Code comments naming old paths (not includes):**
- src/config/field_map.h:202 (`impl/publishing/field_documents.h`);
- src/inspection/inspection_service.h:33 (`capture/capture_http.h`);
- src/runtime/register_all.h:3 (`impl/resources/register_resources.cpp`);
- src/state/stillness.h:5 (`impl/localization/stationary_window.h`);
- line 1 of the 7 payload headers (`// payloads/<name>.h`).

**Line-1 filename comments of the 18 renamed files** (24 files counting the payload headers):
- contracts/{commands, field_estimation, localization, publishing, resource, sensor, target_resolution}.h become `contract.h`;
- runtime/{localization, resource, sensor, world_estimation}_stage.{h,cpp} become `stage.{h,cpp}`;
- impl/noop/noops.{h,cpp} become `noop.{h,cpp}`;
- payloads/field_object_evidence.h becomes `evidence.h`.

**Docs (link target, and link text where the file is renamed):**
- `pi/naviGATR/docs/architecture.md`:
  - 118 (`impl/localization/stationary_window.h`);
  - 277 (`transport/half_duplex` becomes `io/half_duplex`);
  - 299 (`runtime/sensor_loss.h`);
  - 366-381: the "where it lives" table, 22 links. Link text changes on 367 (resource_stage.cpp, sensor_stage.cpp), 368 (localization_stage.cpp), 369 (localization.h) and 371 (world_estimation_stage.cpp).
- `pi/naviGATR/docs/capture.md`: 18, 20, 172.
- `pi/naviGATR/docs/inspection.md`: 465.
- `pi/naviGATR/docs/landmarks.md`: 114, 165.
- `pi/naviGATR/docs/pi_camera_setup.md`: 10, 127.
- `pi/naviGATR/docs/pico_link.md`: 25, 176.
- `pi/naviGATR/third_party/README.md`: 35, 54.
- `docs/navigatr_resources.md`: 171.
- `docs/navigatr_sensors.md`: 129.

**No change needed:**
- `.github/workflows/ci.yaml`: it only runs `cmake -S pi/naviGATR`, build and ctest, and names no src path.
- Root `README.md`, `pi/naviGATR/README.md`, `docs/README.md`, `CONTRIBUTING.md`: no src paths.
- `.clang-format`, `.gitignore`, `.gitattributes`, `.vscode/settings.json`: no src paths.
- brain/, pico/, translaGATR/, bench/, pcb/: no naviGATR src path. They name only the target `navigatr_field_refs`, which is unchanged (brain/operaGATR/include/field_references.h:1, docs/actugatr.md:50, docs/investigatr.md:113).
- tests/CMakeLists.txt `brain_link_e2e_tests`: only brain/ paths.

## Name collisions and include ambiguities

- **Path collisions: none.** There are 201 distinct destinations, and the 8 new noop files are distinct from them.
- **Same basename in different folders.** No include is affected, because every include names its folder:
  - `contract.h` x7 (commands, localization, publishing, resources, sensors, targets, world_estimation);
  - `stage.h` x4, `stage.cpp` x4 (localization, resources, sensors, world_estimation);
  - `noop.h` x5, `noop.cpp` x5 (commands/collectors, localization/estimators, world_estimation/estimators/noop, targets/resolvers, publishing/publishers);
  - `records.h` x2 (`pipeline/framework/records.h` for record envelopes, `debug/diagnostics/records.h` for diagnostic event records).
- **Build check for the shared .cpp basenames.** `navigatr_core` will contain 4 `stage.cpp` and 5 `noop.cpp`. This is new: today no two .cpp files share a basename.
  - CMake keeps each object under its source folder, and GNU `ar qc` keeps duplicate member names, so this should link.
  - The step-1 build and link will confirm it.
- **Near-names worth a glance:**
  - `pipeline/framework/diagnostics.h` (per-slot counters) vs the `debug/diagnostics/` folder;
  - `pipeline/resources/camera.h` vs `resources/resources/cameras.h`;
  - `tag_detector.h` vs `resources/tag_detectors.h`;
  - `target_set.h` vs `resources/target_set_resource.h`.
- **New folder names vs system headers.** `src/time/`, `src/io/` and `src/debug/` are new (`src/math/` already exists).
  - With `-I src`, `<time.h>` or `<io.h>` (MinGW) looks for `src/time.h` or `src/io.h`, which do not exist. So nothing is shadowed.
  - Do not add a file named `src/time.h`, `src/io.h` or `src/math.h`.
- **Include order.** The repo `.clang-format` is LLVM-based, so SortIncludes is on and changed include strings re-sort within their block.
  - A header that is not self-contained can then fail to compile. The fix is adding its missing include, which is still logic-free.
- **Path length on this box:**
  - longest new source path: 208 chars (`.../src/pipeline/world_estimation/estimators/apriltag/apriltag_tag_observation.cpp`);
  - longest object dependency file under `S/wt-build/move-map/`: 241 chars, which is under Windows' 260-character path limit (MAX_PATH).

## Layering notes (created or kept by the move; no action in step 1)

- `pipeline/framework/slot_init.h` includes `pipeline/localization/contract.h`.
- `math/transforms.h` and `config/field_map.h` include `pipeline/framework/ids.h`.
- `math/camera_model.h` includes `pipeline/resources/camera.h`.
- Resource plug-ins include sensors payloads (D9):
  - `cameras.cpp` includes `camera_frames.h`;
  - `synthetic_rig.cpp` includes `camera_frames.h` and `attitude_samples.h`.
- `pipeline/resources/pico_control_ref.h` includes its own plug-in, `resources/resources/pico_telemetry.h`.
- The commands and publishing plug-ins include `brain_profile/brain_profile.h`:
  - brain_link_commands.h;
  - brain_link_publisher.h.
- These pipeline files include `debug/diagnostics/hub.h`:
  - robot_state_feed.cpp;
  - brain_link_commands.cpp, brain_link_publisher.cpp;
  - pico_telemetry.cpp, pros_usb_link.cpp.
- `core/system.cpp` and `debug/inspection/inspection_state.h` use apriltag-group types:
  - `TagObservationSet` from step 1;
  - `TagAssociationTraceSet` from step 3.
  - Inspection's `DetectionFrameSnapshot` is tag-shaped. This is for spectuGATR and the world-estimation step.

## Tables by new folder

Columns:
- **old**: relative to `src/`.
- **new file**: the basename in the folder named by the heading.
- **included by**: current includers across the worktree, as count (src, tests, tools). Up to 3 are named, by their current basenames. The full lists are in `logs/include_graph.txt`.

### src/ (1)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| app/main.cpp | main.cpp | 0 | tiny entry (SPEC: src/main.cpp) |

### src/cli/ (3)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| app/build_config.h.in | build_config.h.in | 0 | configure_file template for the default config path; only main.cpp reads it (generated navigatr_build_config.h) |
| app/command_line.cpp | command_line.cpp | 0 | command-line parsing (SPEC cli/) |
| app/command_line.h | command_line.h | 3 (src 2, tests 1): command_line.cpp, main.cpp, command_line_gtest.cpp | command-line parsing (SPEC cli/) |

### tools/field_refs/ (1)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| app/field_refs.cpp | field_refs.cpp | 0 | separate generator program (SPEC: tools/field_refs/) |

### src/core/ (3)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| runtime/handoff.h | handoff.h | 2 (src 1, tests 1): system.h, workers_gtest.cpp | LatestSlot/WorkerStats: the run loop's latest-wins handoff between the two workers |
| runtime/system.cpp | system.cpp | 0 | engine; SPEC maps it to core/system.cpp until the split step |
| runtime/system.h | system.h | 32 (src 7, tests 24, tools 1) | engine facade; SPEC maps it to core/system.h until the split step |

### src/pipeline/ (2)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| runtime/register_all.cpp | register_all.cpp | 0 | calls every stage's register_* in run order; plug-in index for the whole pipeline |
| runtime/register_all.h | register_all.h | 39 (src 9, tests 29, tools 1) | declares every register_* plus registerOrDie; included by every plug-in register file |

### src/pipeline/framework/ (11)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| runtime/build_options.h | build_options.h | 3 (src 3): command_line.h, resource_stage.h, system.h | BuildOptions: build-time policy from the CLI, consumed by make_resources; keeps stages from including core/ |
| core/diagnostics.h | diagnostics.h | 13 (src 9, tests 4) | per-slot FunctionStats/LinkStats sink of ExecutionContext; stage machinery |
| core/execution_context.h | execution_context.h | 8 (src 8) | SPEC framework: execution context |
| core/function_registry.h | function_registry.h | 11 (src 10, tests 1) | SPEC framework: function registry |
| core/function_status.h | function_status.h | 9 (src 9) | SPEC framework: function status |
| core/ids.h | ids.h | 25 (src 25) | SPEC framework: ids |
| core/payload_descriptor.h | payload_descriptor.h | 8 (src 8) | SPEC framework: payload descriptors |
| runtime/record_bookkeeping.h | record_bookkeeping.h | 2 (src 2): resource_stage.cpp, sensor_stage.cpp | poll-to-record rules shared by the resource and sensor stages |
| core/records.h | records.h | 20 (src 19, tests 1) | SPEC framework: records (ResourceMap/SensorMap envelopes) |
| contracts/slot_init.h | slot_init.h | 6 (src 6) | SlotInitializationContext: build context every slot factory receives |
| core/typed_payload.h | typed_payload.h | 1 (src 1): records.h | SPEC framework: typed payloads |

### src/pipeline/resources/ (16)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| resources/brain_imu_bench.h | brain_imu_bench.h | 9 (src 7, tests 2) | resource type brain_imu_bench (registered inline in register_resources.cpp); produced by the resources stage |
| resources/camera.h | camera.h | 14 (src 9, tests 5) | Camera resource contract; produced by camera resources |
| contracts/resource.h | contract.h | 3 (src 3): resource_instance.h, resource_catalog.h, resource_stage.h | resources stage contract (SPEC contracts/<x>.h -> contract.h) |
| resources/pico_control.h | pico_control.h | 5 (src 5) | PicoControl resource contract; produced by pico_telemetry |
| runtime/pico_control_ref.h | pico_control_ref.h | 3 (src 3): brain_link_publisher.cpp, inspection_document.cpp, system.cpp | resolves <Pico resource_id> to a PicoControl from the store; resource-reference helper |
| payloads/pico_telemetry_samples.h | pico_telemetry_samples.h | 6 (src 5, tests 1) | pico.* payloads the pico_telemetry resource produces |
| runtime/resource_catalog.h | resource_catalog.h | 7 (src 6, tests 1) | catalog of resource outputs the stage owns |
| resources/resource_instance.h | resource_instance.h | 7 (src 7) | type-erased resource holder the store owns |
| resources/resource_store.cpp | resource_store.cpp | 0 | store the stage owns |
| resources/resource_store.h | resource_store.h | 31 (src 27, tests 4) | store the stage owns |
| resources/robot_frames.h | robot_frames.h | 6 (src 5, tests 1) | RobotFrameMap resource data; produced by robot_frame_map |
| runtime/resource_stage.cpp | stage.cpp | 0 | make_resources builder (SPEC stage.cpp) |
| runtime/resource_stage.h | stage.h | 2 (src 2): resource_stage.cpp, system.h | make_resources builder (SPEC stage.h) |
| resources/tag_detector.h | tag_detector.h | 5 (src 2, tests 3) | TagDetector resource contract; produced by apriltag_detector |
| resources/target_set.h | target_set.h | 5 (src 2, tests 3) | TargetSet resource data; produced by target_set |
| resources/wheel_geometry.h | wheel_geometry.h | 4 (src 4) | WheelGeometryMap resource data; produced by wheel_geometry |

### src/pipeline/resources/resources/ (26)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| impl/resources/apriltag_c.c | apriltag_c.c | 0 | C adapter to vendored apriltag, used by tag_detectors.cpp |
| impl/resources/apriltag_c.h | apriltag_c.h | 2 (src 2): apriltag_c.c, tag_detectors.cpp | C adapter to vendored apriltag, used by tag_detectors.cpp |
| impl/resources/camera_capture.h | camera_capture.h | 2 (src 1, tests 1): libcamera_camera.cpp, camera_capture_gtest.cpp | capture timing helpers for libcamera_camera.cpp |
| impl/resources/cameras.cpp | cameras.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/cameras.h | cameras.h | 9 (src 3, tests 6) | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/field_map_resource.cpp | field_map_resource.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/field_map_resource.h | field_map_resource.h | 2 (src 2): field_map_resource.cpp, register_resources.cpp | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/libcamera_camera.cpp | libcamera_camera.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/libcamera_camera.h | libcamera_camera.h | 2 (src 2): cameras.cpp, libcamera_camera.cpp | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/pico_telemetry.cpp | pico_telemetry.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/pico_telemetry.h | pico_telemetry.h | 5 (src 4, tests 1) | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/pros_usb_link.cpp | pros_usb_link.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/pros_usb_link.h | pros_usb_link.h | 5 (src 2, tests 3) | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/register_resources.cpp | register_resources.cpp | 0 | registers the <Resource type> plug-ins |
| impl/resources/robot_frame_map.cpp | robot_frame_map.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/robot_frame_map.h | robot_frame_map.h | 2 (src 2): register_resources.cpp, robot_frame_map.cpp | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/serial_links.cpp | serial_links.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/serial_links.h | serial_links.h | 18 (src 4, tests 14) | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/synthetic_rig.cpp | synthetic_rig.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/synthetic_rig.h | synthetic_rig.h | 5 (src 2, tests 3) | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/tag_detectors.cpp | tag_detectors.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/tag_detectors.h | tag_detectors.h | 7 (src 4, tests 3) | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/target_set_resource.cpp | target_set_resource.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/target_set_resource.h | target_set_resource.h | 2 (src 2): register_resources.cpp, target_set_resource.cpp | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/wheel_geometry_resource.cpp | wheel_geometry_resource.cpp | 0 | <Resource type> plug-in (resource_stage.cpp:99) |
| impl/resources/wheel_geometry_resource.h | wheel_geometry_resource.h | 2 (src 2): register_resources.cpp, wheel_geometry_resource.cpp | <Resource type> plug-in (resource_stage.cpp:99) |

### src/pipeline/sensors/ (7)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| payloads/attitude_samples.h | attitude_samples.h | 5 (src 3, tests 2) | sensor.attitude_sample: SensorMap output payload (attitude_channel produces the body-frame sample) |
| payloads/camera_frames.h | camera_frames.h | 7 (src 6, tests 1) | sensor.camera_frame: SensorMap output payload perception binds by sensor id |
| contracts/sensor.h | contract.h | 5 (src 4, tests 1) | sensors stage contract |
| runtime/sensor_catalog.h | sensor_catalog.h | 16 (src 10, tests 6) | SensorCatalog the stage owns (bind-by-id for consumers) |
| payloads/sensor_samples.h | sensor_samples.h | 14 (src 7, tests 7) | sensor.encoder_sample / sensor.imu_sample: SensorMap output payloads |
| runtime/sensor_stage.cpp | stage.cpp | 0 | make_sensors builder |
| runtime/sensor_stage.h | stage.h | 3 (src 2, tests 1): sensor_stage.cpp, system.h, core_gtest.cpp | make_sensors builder |

### src/pipeline/sensors/sensors/ (7)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| impl/sensors/attitude_channel.cpp | attitude_channel.cpp | 0 | <Sensor type> plug-in (sensor_stage.cpp:96) |
| impl/sensors/attitude_channel.h | attitude_channel.h | 3 (src 2, tests 1): attitude_channel.cpp, register_sensors.cpp, stages_gtest.cpp | <Sensor type> plug-in (sensor_stage.cpp:96) |
| impl/sensors/camera_channel.cpp | camera_channel.cpp | 0 | <Sensor type> plug-in (sensor_stage.cpp:96) |
| impl/sensors/camera_channel.h | camera_channel.h | 2 (src 2): camera_channel.cpp, register_sensors.cpp | <Sensor type> plug-in (sensor_stage.cpp:96) |
| impl/sensors/pico_channels.cpp | pico_channels.cpp | 0 | <Sensor type> plug-in (sensor_stage.cpp:96) |
| impl/sensors/pico_channels.h | pico_channels.h | 2 (src 2): pico_channels.cpp, register_sensors.cpp | <Sensor type> plug-in (sensor_stage.cpp:96) |
| impl/sensors/register_sensors.cpp | register_sensors.cpp | 0 | registers the <Sensor type> plug-ins |

### src/pipeline/commands/ (2)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| state/command_state.h | command_state.h | 4 (src 4) | CommandState, produced by the commands stage |
| contracts/commands.h | contract.h | 3 (src 3): brain_link_commands.h, noops.h, system.h | commands stage contract |

### src/pipeline/commands/collectors/ (3)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| impl/commands/brain_link_commands.cpp | brain_link_commands.cpp | 0 | <CommandCollection type> plug-in (system.cpp:276) |
| impl/commands/brain_link_commands.h | brain_link_commands.h | 3 (src 3): brain_link_commands.cpp, register_commands.cpp, brain_link_publisher.cpp | <CommandCollection type> plug-in (system.cpp:276) |
| impl/commands/register_commands.cpp | register_commands.cpp | 0 | registers the <CommandCollection type> plug-ins |

### src/pipeline/localization/ (12)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| state/attitude.h | attitude.h | 4 (src 4) | Attitude, part of the robot estimate localization produces |
| contracts/localization.h | contract.h | 11 (src 11) | localization stage contract |
| state/pose_history.cpp | pose_history.cpp | 0 | pose history localization owns and publishes |
| state/pose_history.h | pose_history.h | 9 (src 7, tests 2) | pose history localization owns and publishes |
| impl/localization/register_localization.cpp | register_localization.cpp | 0 | registers both <Observation> and <Estimator> plug-ins, so it sits above both folders |
| payloads/robot_observations.h | robot_observations.h | 11 (src 7, tests 4) | localization.* payloads its observation functions produce |
| state/robot_state.h | robot_state.h | 10 (src 9, tests 1) | RobotState, localization's output |
| state/robot_state_feed.cpp | robot_state_feed.cpp | 0 | publication side of localization |
| state/robot_state_feed.h | robot_state_feed.h | 6 (src 4, tests 2) | publication side of localization |
| runtime/localization_stage.cpp | stage.cpp | 0 | make_localization builder |
| runtime/localization_stage.h | stage.h | 4 (src 2, tests 2) | make_localization builder |
| state/stillness.h | stillness.h | 3 (src 3): localization.h, stationary_window.h, robot_state_feed.h | StillnessStatus, part of the localization contract |

### src/pipeline/localization/estimators/ (6)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| impl/localization/motion_step.cpp | motion_step.cpp | 0 | step core shared by the two planar estimators only |
| impl/localization/motion_step.h | motion_step.h | 3 (src 3): motion_step.cpp, planar_motion_integrator.h, weighted_planar_fusion.h | step core shared by the two planar estimators only |
| impl/localization/planar_motion_integrator.cpp | planar_motion_integrator.cpp | 0 | <Estimator type> plug-in under <Localization> (localization_stage.cpp:268) |
| impl/localization/planar_motion_integrator.h | planar_motion_integrator.h | 3 (src 2, tests 1): planar_motion_integrator.cpp, register_localization.cpp, localization_gtest.cpp | <Estimator type> plug-in under <Localization> (localization_stage.cpp:268) |
| impl/localization/weighted_planar_fusion.cpp | weighted_planar_fusion.cpp | 0 | <Estimator type> plug-in under <Localization> (localization_stage.cpp:268) |
| impl/localization/weighted_planar_fusion.h | weighted_planar_fusion.h | 3 (src 2, tests 1): register_localization.cpp, weighted_planar_fusion.cpp, localization_fusion_gtest.cpp | <Estimator type> plug-in under <Localization> (localization_stage.cpp:268) |

### src/pipeline/localization/observations/ (10)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| impl/localization/attitude_reference.cpp | attitude_reference.cpp | 0 | <Observation type> plug-in (localization_stage.cpp:223) |
| impl/localization/attitude_reference.h | attitude_reference.h | 3 (src 2, tests 1): attitude_reference.cpp, register_localization.cpp, localization_models_gtest.cpp | <Observation type> plug-in (localization_stage.cpp:223) |
| impl/localization/brain_imu_wheel_bench.cpp | brain_imu_wheel_bench.cpp | 0 | <Observation type> plug-in (localization_stage.cpp:223) |
| impl/localization/brain_imu_wheel_bench.h | brain_imu_wheel_bench.h | 2 (src 2): brain_imu_wheel_bench.cpp, register_localization.cpp | <Observation type> plug-in (localization_stage.cpp:223) |
| impl/localization/imu_heading_increment.cpp | imu_heading_increment.cpp | 0 | <Observation type> plug-in (localization_stage.cpp:223) |
| impl/localization/imu_heading_increment.h | imu_heading_increment.h | 3 (src 2, tests 1): imu_heading_increment.cpp, register_localization.cpp, localization_models_gtest.cpp | <Observation type> plug-in (localization_stage.cpp:223) |
| impl/localization/stationary_window.cpp | stationary_window.cpp | 0 | stillness/bias helper shared by the observation models only |
| impl/localization/stationary_window.h | stationary_window.h | 5 (src 4, tests 1) | stillness/bias helper shared by the observation models only |
| impl/localization/tracking_wheel_motion.cpp | tracking_wheel_motion.cpp | 0 | <Observation type> plug-in (localization_stage.cpp:223) |
| impl/localization/tracking_wheel_motion.h | tracking_wheel_motion.h | 4 (src 2, tests 2) | <Observation type> plug-in (localization_stage.cpp:223) |

### src/pipeline/world_estimation/ (9)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| contracts/association.h | association.h | 2 (src 2): apriltag_world_estimator.h, tag_mount_association.h | generic step contract (SPEC) |
| contracts/field_estimation.h | contract.h | 4 (src 4) | world estimation stage contract (SPEC) |
| payloads/field_object_evidence.h | evidence.h | 10 (src 5, tests 5) | SPEC evidence.h; tag trace types leave it in step 3 |
| state/field_state.h | field_state.h | 7 (src 7) | field state (SPEC) |
| impl/world_estimation/landmark_estimator.cpp | landmark_estimator.cpp | 0 | generic evidence folding, sensor-agnostic (SPEC stage level) |
| impl/world_estimation/landmark_estimator.h | landmark_estimator.h | 3 (src 2, tests 1): apriltag_world_estimator.h, landmark_estimator.cpp, world_estimation_gtest.cpp | generic evidence folding, sensor-agnostic (SPEC stage level) |
| contracts/perception.h | perception.h | 2 (src 2): apriltag_tag_observation.h, apriltag_world_estimator.h | generic step contract (SPEC) |
| runtime/world_estimation_stage.cpp | stage.cpp | 0 | make_world_estimation builder |
| runtime/world_estimation_stage.h | stage.h | 2 (src 2): system.h, world_estimation_stage.cpp | make_world_estimation builder |

### src/pipeline/world_estimation/estimators/ (1)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| impl/world_estimation/register_world_estimation.cpp | register_world_estimation.cpp | 0 | registers apriltag and noop estimators, so it sits above both group folders |

### src/pipeline/world_estimation/estimators/apriltag/ (7)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| impl/world_estimation/apriltag_tag_observation.cpp | apriltag_tag_observation.cpp | 0 | apriltag group: tag observation extraction (SPEC) |
| impl/world_estimation/apriltag_tag_observation.h | apriltag_tag_observation.h | 2 (src 2): apriltag_tag_observation.cpp, apriltag_world_estimator.cpp | apriltag group: tag observation extraction (SPEC) |
| impl/world_estimation/apriltag_world_estimator.cpp | apriltag_world_estimator.cpp | 0 | apriltag group: the world estimator (SPEC) |
| impl/world_estimation/apriltag_world_estimator.h | apriltag_world_estimator.h | 2 (src 2): apriltag_world_estimator.cpp, register_world_estimation.cpp | apriltag group: the world estimator (SPEC) |
| impl/world_estimation/tag_mount_association.cpp | tag_mount_association.cpp | 0 | apriltag group: tag mount association (SPEC) |
| impl/world_estimation/tag_mount_association.h | tag_mount_association.h | 3 (src 2, tests 1): apriltag_world_estimator.cpp, tag_mount_association.cpp, capture_transform_gtest.cpp | apriltag group: tag mount association (SPEC) |
| payloads/tag_observations.h | tag_observations.h | 9 (src 3, tests 6) | apriltag group: tag observation payloads (SPEC) |

### src/pipeline/world_estimation/estimators/noop/ (2)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| impl/noop/noops.cpp | noop.cpp | 0 | git mv target; split into five stage noops (see Noop split) |
| impl/noop/noops.h | noop.h | 7 (src 6, tests 1) | git mv target; split into five stage noops (see Noop split) |

### src/pipeline/targets/ (2)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| contracts/target_resolution.h | contract.h | 3 (src 3): noops.h, configured_targets.h, system.h | targets stage contract (no builder: slot built by System::build) |
| state/target_state.h | target_state.h | 3 (src 3): publishing.h, target_resolution.h, system.h | TargetState, produced by target resolution |

### src/pipeline/targets/resolvers/ (3)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| impl/target_resolution/configured_targets.cpp | configured_targets.cpp | 0 | <TargetResolution type> plug-in (system.cpp:329) |
| impl/target_resolution/configured_targets.h | configured_targets.h | 2 (src 2): configured_targets.cpp, register_target_resolution.cpp | <TargetResolution type> plug-in (system.cpp:329) |
| impl/target_resolution/register_target_resolution.cpp | register_target_resolution.cpp | 0 | registers the <TargetResolution type> plug-ins |

### src/pipeline/publishing/ (1)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| contracts/publishing.h | contract.h | 3 (src 3): noops.h, brain_link_publisher.h, system.h | publishing stage contract (no builder: slot built by System::build) |

### src/pipeline/publishing/publishers/ (5)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| impl/publishing/brain_link_publisher.cpp | brain_link_publisher.cpp | 0 | <Publishing type> plug-in (system.cpp:335) |
| impl/publishing/brain_link_publisher.h | brain_link_publisher.h | 2 (src 2): brain_link_publisher.cpp, register_publishers.cpp | <Publishing type> plug-in (system.cpp:335) |
| impl/publishing/field_documents.cpp | field_documents.cpp | 0 | Brain wire field documents the brain_link publisher sends; also used by tools/field_refs and inspection |
| impl/publishing/field_documents.h | field_documents.h | 6 (src 4, tests 2) | Brain wire field documents the brain_link publisher sends; also used by tools/field_refs and inspection |
| impl/publishing/register_publishers.cpp | register_publishers.cpp | 0 | registers the <Publishing type> plug-ins |

### src/brain_profile/ (7)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| contracts/brain_profile.h | brain_profile.h | 5 (src 4, tests 1) | BrainProfileHost/ProfileBinding: the profile host interface (SPEC brain_profile/) |
| runtime/brain_profile_builder.cpp | brain_profile_builder.cpp | 0 | profile build (SPEC brain_profile/) |
| runtime/brain_profile_builder.h | brain_profile_builder.h | 4 (src 4) | profile build (SPEC brain_profile/) |
| runtime/sensor_loss.cpp | sensor_loss.cpp | 0 | sensor loss and recovery (SPEC brain_profile/) |
| runtime/sensor_loss.h | sensor_loss.h | 2 (src 2): sensor_loss.cpp, system.h | sensor loss and recovery (SPEC brain_profile/) |
| runtime/stationary_precheck.cpp | stationary_precheck.cpp | 0 | stationary precheck (SPEC brain_profile/) |
| runtime/stationary_precheck.h | stationary_precheck.h | 3 (src 2, tests 1): stationary_precheck.cpp, system.h, brain_profile_gtest.cpp | stationary precheck (SPEC brain_profile/) |

### src/config/ (7)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| config/composition.cpp | same | 0 | unchanged (SPEC) |
| config/composition.h | same | 10 (src 2, tests 8) | unchanged (SPEC) |
| config/config_node.cpp | same | 0 | unchanged (SPEC) |
| config/config_node.h | same | 31 (src 28, tests 3) | unchanged (SPEC) |
| config/field_map.cpp | same | 0 | unchanged (SPEC) |
| config/field_map.h | same | 13 (src 9, tests 4) | unchanged (SPEC) |
| config/pose3_config.h | same | 4 (src 4) | unchanged (SPEC) |

### src/io/ (9)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| transport/byte_stream.h | byte_stream.h | 3 (src 3): serial_links.h, file_stream.h, serial_port.h | SPEC io/ (from transport/) |
| transport/file_stream.cpp | file_stream.cpp | 0 | SPEC io/ (from transport/) |
| transport/file_stream.h | file_stream.h | 2 (src 2): serial_links.h, file_stream.cpp | SPEC io/ (from transport/) |
| transport/half_duplex.cpp | half_duplex.cpp | 0 | SPEC io/ (from transport/) |
| transport/half_duplex.h | half_duplex.h | 4 (src 3, tests 1) | SPEC io/ (from transport/) |
| resources/serial_link.h | serial_link.h | 8 (src 8) | byte-link contract (ByteSpan, TransmitWindow); io/half_duplex.h and commands/command_state.h need it |
| transport/serial_port.cpp | serial_port.cpp | 0 | SPEC io/ (from transport/) |
| transport/serial_port.h | serial_port.h | 2 (src 2): serial_links.h, serial_port.cpp | SPEC io/ (from transport/) |
| resources/spi_bus.h | spi_bus.h | 1 (tests 1): resources_gtest.cpp | SPI bus contract beside serial_link.h (includes it); only tests/resources_gtest.cpp uses it |

### src/time/ (3)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| core/clock_sync.h | clock_sync.h | 2 (src 1, tests 1): motion_step.h, localization_gtest.cpp | SPEC time/: clock sync |
| core/host_clock.h | host_clock.h | 17 (src 10, tests 6, tools 1) | SPEC time/: host clock |
| core/time.h | time.h | 26 (src 25, tests 1) | SPEC time/: time types |

### src/math/ (5)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| math/angles.h | same | 45 (src 22, tests 23) | unchanged (SPEC) |
| math/camera_model.h | same | 3 (src 2, tests 1): synthetic_rig.cpp, tag_detectors.cpp, apriltag_gtest.cpp | unchanged (SPEC) |
| math/quaternion.h | same | 9 (src 7, tests 2) | unchanged (SPEC) |
| math/se3.h | same | 12 (src 9, tests 3) | unchanged (SPEC) |
| math/transforms.h | same | 12 (src 11, tests 1) | unchanged (SPEC) |

### src/debug/inspection/ (13)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| inspection/http_server.cpp | http_server.cpp | 0 | SPEC debug/inspection/ moved as a subfolder |
| inspection/http_server.h | http_server.h | 6 (src 4, tests 2) | SPEC debug/inspection/ moved as a subfolder |
| runtime/inspection_config.h | inspection_config.h | 2 (src 2): inspection_service.h, system.h | parses the <Inspection> root element; inspection policy |
| inspection/inspection_document.cpp | inspection_document.cpp | 0 | SPEC debug/inspection/ moved as a subfolder |
| inspection/inspection_document.h | inspection_document.h | 10 (src 2, tests 7, tools 1) | SPEC debug/inspection/ moved as a subfolder |
| inspection/inspection_service.cpp | inspection_service.cpp | 0 | SPEC debug/inspection/ moved as a subfolder |
| inspection/inspection_service.h | inspection_service.h | 5 (src 2, tests 2, tools 1) | SPEC debug/inspection/ moved as a subfolder |
| runtime/inspection_state.h | inspection_state.h | 2 (src 2): inspection_document.h, system.h | snapshots the workers hand to inspection; inspection's data contract |
| inspection/inspection_stats.h | inspection_stats.h | 2 (src 2): inspection_document.h, inspection_service.h | SPEC debug/inspection/ moved as a subfolder |
| inspection/jpeg_encoder.cpp | jpeg_encoder.cpp | 0 | SPEC debug/inspection/ moved as a subfolder |
| inspection/jpeg_encoder.h | jpeg_encoder.h | 2 (src 2): inspection_service.cpp, jpeg_encoder.cpp | SPEC debug/inspection/ moved as a subfolder |
| inspection/json_writer.h | json_writer.h | 10 (src 6, tests 4) | SPEC debug/inspection/ moved as a subfolder |
| inspection/viewer_assets.h | viewer_assets.h | 2 (src 1, tests 1): inspection_service.cpp, inspection_gtest.cpp | SPEC debug/inspection/ moved as a subfolder |

### src/debug/capture/ (10)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| capture/capture_bundle.cpp | capture_bundle.cpp | 0 | SPEC debug/capture/ moved as a subfolder |
| capture/capture_bundle.h | capture_bundle.h | 4 (src 3, tests 1) | SPEC debug/capture/ moved as a subfolder |
| capture/capture_config.h | capture_config.h | 3 (src 2, tests 1): capture_recorder.h, system.h, capture_gtest.cpp | SPEC debug/capture/ moved as a subfolder |
| capture/capture_http.cpp | capture_http.cpp | 0 | SPEC debug/capture/ moved as a subfolder |
| capture/capture_http.h | capture_http.h | 3 (src 2, tests 1): capture_http.cpp, inspection_service.cpp, capture_gtest.cpp | SPEC debug/capture/ moved as a subfolder |
| capture/capture_recorder.h | capture_recorder.h | 4 (src 3, tests 1) | SPEC debug/capture/ moved as a subfolder |
| capture/capture_service.cpp | capture_service.cpp | 0 | SPEC debug/capture/ moved as a subfolder |
| capture/capture_service.h | capture_service.h | 3 (src 3): capture_recorder.h, inspection_document.cpp, inspection_service.cpp | SPEC debug/capture/ moved as a subfolder |
| capture/zip_writer.cpp | zip_writer.cpp | 0 | SPEC debug/capture/ moved as a subfolder |
| capture/zip_writer.h | zip_writer.h | 3 (src 2, tests 1): capture_bundle.cpp, zip_writer.cpp, capture_gtest.cpp | SPEC debug/capture/ moved as a subfolder |

### src/debug/diagnostics/ (7)

| old (src/...) | new file | included by | reason |
|---|---|---|---|
| diagnostics/hub.cpp | hub.cpp | 0 | SPEC debug/diagnostics/ moved as a subfolder |
| diagnostics/hub.h | hub.h | 17 (src 12, tests 5) | SPEC debug/diagnostics/ moved as a subfolder |
| diagnostics/instrumentation.cpp | instrumentation.cpp | 0 | SPEC debug/diagnostics/ moved as a subfolder |
| diagnostics/instrumentation.h | instrumentation.h | 5 (src 3, tests 2) | SPEC debug/diagnostics/ moved as a subfolder |
| diagnostics/link_monitor.cpp | link_monitor.cpp | 0 | SPEC debug/diagnostics/ moved as a subfolder |
| diagnostics/link_monitor.h | link_monitor.h | 2 (src 2): hub.h, link_monitor.cpp | SPEC debug/diagnostics/ moved as a subfolder |
| diagnostics/records.h | records.h | 5 (src 5) | SPEC debug/diagnostics/ moved as a subfolder |

