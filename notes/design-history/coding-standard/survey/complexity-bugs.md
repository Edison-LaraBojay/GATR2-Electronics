# Survey: complexity hot spots and bug classes

Input for the GATR2 coding standard. Read-only survey of the tree at HEAD `ae457ee` (working tree clean) on 2026-09-28. Paths are git paths; on this Windows box the disk folder is still `pi/navigatr` (lowercase). Nothing in the repo was changed.

What this gives the standard:
- section 1: numbers and thresholds for "too complex";
- section 2: duplicated logic, with the one-owner fix for each;
- section 3: subsystems with more mechanisms or layers than they need, and a rule for when a new mechanism is allowed;
- section 4: 17 bug classes that were actually hit, each with a preventive rule and a check;
- section 5: the checks in one table.

## 0. Method and caveats

- **Complexity tool.** lizard 1.24.0 in a scratch venv, over every tracked `.cpp`, `.h`, `.c` and `.js` file outside `third_party/`. That is 5214 functions: 2586 production, 2315 test, 150 sim, 163 tool.
- **Production** means not under `tests/`, not `*_gtest.cpp`, not `sim/`, not `tools/`, not `bench/`. The PROS kernel headers vendored under `brain/locaGATR/include/pros` and `brain/operaGATR/include/pros` are counted; they add only tiny inline functions.
- **CCN** is lizard's cyclomatic complexity (each `if`, loop, `case`, `&&`, `||`, `?:` adds one). **NLOC** is non-blank, non-comment lines in the function.
- **Nesting ("Nest")** is my own heuristic, because lizard's nesting extension crashed on this code. It counts enclosing control and lambda braces inside the function after stripping comments and strings; aggregate initializers are not counted. Treat it as +-1.
- **Duplication.** `lizard -Eduplicate` (token based) over production code, then manual greps for near-duplicates the tool cannot see: the same logic with a different case order or spelling.
- **Compiler probe.** g++ 15.2 (MSYS2 ucrt64), `-fsyntax-only` with extra warnings, on translaGATR, the portable brain sources, and 69 of the 70 naviGATR `.cpp` files (`app/main.cpp` needs a generated header). No objects were written.
- **clang-format 22.1.8** is the pip wheel at `C:/Users/mbes1/AppData/Roaming/Python/Python314/site-packages/clang_format/data/bin/clang-format.exe`. It is not in `/c/msys64/ucrt64/bin`, as the task said. It ran fine on a scratch copy.
- **Bug sources:**
  - `viz-handoff/stage_results/W0..W5_fix.md` and `viz-handoff/final_report.md`;
  - `brain-arch-handoff/` (final_report, phase_c, stage_results, and the FIX sections of progress/wA-wD);
  - `investigatr-handoff/design_review.txt` (a pre-implementation review of the brain link v3 spec);
  - `memory/project_investigatr_brain_link.md` (review wf_1e521a91) and `memory/project_parallel_wheels_bno08x.md`;
  - `git log` and two "bug fixes" diffs, read-only.
- **Scratch outputs** are in `$S/standard/complexity-bugs/`: `lizard_all.csv`, `metrics.txt`, `dup.txt`, `brain_warn.txt`, `nav_warn.txt`, and the scripts `analyze.py` and `tables.py`.
- When I trim a quoted snippet, it says "trimmed".

## 1. Metrics

### 1.1 Distribution

| Scope | Functions | CCN p50 | p90 | p99 | max | CCN>15 | CCN>25 | NLOC p50 | p90 | max | NLOC>60 | ND>3 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| brain/actuGATR | 152 | 2 | 7 | 24 | 38 | 4 | 2 | 6 | 16 | 117 | 3 | 1 |
| brain/communiGATR | 246 | 2 | 10 | 24 | 56 | 11 | 2 | 8 | 26 | 143 | 2 | 1 |
| brain/investiGATR | 74 | 3 | 13 | 32 | 32 | 5 | 2 | 9 | 31 | 107 | 4 | 1 |
| brain/locaGATR | 30 | 2 | 10 | 45 | 45 | 3 | 1 | 9 | 31 | 125 | 2 | 0 |
| brain/operaGATR | 34 | 2 | 9 | 32 | 32 | 1 | 1 | 6 | 34 | 95 | 1 | 1 |
| brain/robot | 7 | 1 | 1 | 4 | 4 | 0 | 0 | 6 | 13 | 26 | 0 | 0 |
| pi/naviGATR/spectaGATR | 496 | 2 | 10 | 35 | 64 | 28 | 10 | 7 | 30 | 242 | 13 | 4 |
| pi/naviGATR/src | 1303 | 2 | 11 | 41 | 105 | 87 | 40 | 8 | 43 | 367 | 82 | 24 |
| pico/aggreGATR | 161 | 2 | 7 | 14 | 18 | 1 | 0 | 7 | 22 | 54 | 0 | 0 |
| translaGATR | 83 | 3 | 14 | 28 | 52 | 7 | 2 | 10 | 48 | 100 | 6 | 0 |
| all C++ prod | 2090 | 2 | 11 | 35 | 105 | 119 | 50 | 8 | 36 | 367 | 100 | 28 |
| all JS prod | 496 | 2 | 10 | 35 | 64 | 28 | 10 | 7 | 30 | 242 | 13 | 4 |
| ALL PROD | 2586 | 2 | 11 | 35 | 105 | 147 | 60 | 8 | 35 | 367 | 113 | 32 |
| tests (all) | 2315 | 1 | 4 | 12 | 28 | 15 | 2 | 12 | 34 | 292 | 55 | 10 |
| sims + tools | 313 | 2 | 10 | 30 | 62 | 20 | 9 | 6 | 38 | 252 | 15 | 9 |

**Buckets** (production, 2586 functions)

| Metric | Buckets (count) |
|---|---|
| CCN | 1-5: 2003; 6-10: 313; 11-15: 123; 16-20: 55; 21-30: 55; 31-50: 27; over 50: 10 |
| NLOC | 1-20: 2094; 21-40: 281; 41-60: 98; 61-80: 43; 81-120: 41; 121-200: 21; over 200: 8 |
| Nesting | 0: 1261; 1: 813; 2: 361; 3: 119; 4: 17; 5: 11; 6: 4 |
| Parameters | 0-3: 2393; 4: 126; 5: 37; 6: 19; 7: 11; 8 or more: 0 |
| File lines (335 production files) | up to 200: 260; 201-400: 36; 401-600: 18; 601-800: 6; 801-1000: 5; 1001-1500: 7; over 1500: 3 |

**Reading:**
- The median function is small and simple everywhere: CCN 2, 8 NLOC.
- The problem is a fat tail, and it sits in naviGATR:
  - 87 of the 147 functions over CCN 15;
  - 82 of the 113 functions over 60 NLOC;
  - 24 of the 32 functions nested deeper than 3.
- pico/aggreGATR is the cleanest component (max CCN 18, max 54 NLOC).
- spectaGATR has the second-worst tail: 28 functions over CCN 15.

### 1.2 Top lists

#### Top 30 by cyclomatic complexity (production C++ and JS)

| # | CCN | NLOC | Nest | Params | Function | Location |
|---|---|---|---|---|---|---|
| 1 | 105 | 310 | 6 | 2 | `TrackingWheelMotion::run` | pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:510 |
| 2 | 79 | 242 | 3 | 3 | `make_synthetic_rig` | pi/naviGATR/src/impl/resources/synthetic_rig.cpp:398 |
| 3 | 67 | 270 | 3 | 3 | `TrackingWheelMotion::create` | pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:105 |
| 4 | 64 | 120 | 1 | 1 | `renderStatus` | pi/naviGATR/spectaGATR/app.js:852 |
| 5 | 61 | 126 | 2 | 5 | `parseBrainProfileConfig` | pi/naviGATR/src/runtime/brain_profile_builder.cpp:242 |
| 6 | 56 | 143 | 3 | 1 | `Client::choose` | brain/communiGATR/src/client.cpp:869 |
| 7 | 53 | 240 | 5 | 1 | `TagMountAssociation::run` | pi/naviGATR/src/impl/world_estimation/tag_mount_association.cpp:148 |
| 8 | 52 | 202 | 4 | 3 | `parseFieldMap` | pi/naviGATR/src/config/field_map.cpp:249 |
| 9 | 52 | 100 | 3 | 3 | `validateRobotProfile` | translaGATR/link_documents.cpp:262 |
| 10 | 51 | 101 | 2 | 3 | `parseInspectionConfig` | pi/naviGATR/src/runtime/inspection_config.h:74 |
| 11 | 48 | 155 | 3 | 4 | `BrainImuWheelBench::create` | pi/naviGATR/src/impl/localization/brain_imu_wheel_bench.cpp:32 |
| 12 | 46 | 79 | 3 | 2 | `readinessOf` | pi/naviGATR/spectaGATR/link_panel.js:29 |
| 13 | 45 | 125 | 3 | 0 | `run` | brain/locaGATR/src/main.cpp:526 |
| 14 | 45 | 110 | 2 | 3 | `TagMountAssociation::create` | pi/naviGATR/src/impl/world_estimation/tag_mount_association.cpp:16 |
| 15 | 44 | 169 | 6 | 1 | `ConfiguredTargetResolution::run` | pi/naviGATR/src/impl/target_resolution/configured_targets.cpp:107 |
| 16 | 44 | 119 | 6 | 0 | `redraw` | pi/naviGATR/spectaGATR/camera_panel.js:363 |
| 17 | 44 | 106 | 2 | 3 | `parseCameraConfig` | pi/naviGATR/src/impl/resources/cameras.cpp:19 |
| 18 | 43 | 163 | 3 | 7 | `prepareStep` | pi/naviGATR/src/impl/localization/motion_step.cpp:367 |
| 19 | 42 | 196 | 3 | 3 | `make_target_set` | pi/naviGATR/src/impl/resources/target_set_resource.cpp:119 |
| 20 | 41 | 367 | 5 | 7 | `writeSnapshotBody` | pi/naviGATR/src/inspection/inspection_document.cpp:1140 |
| 21 | 41 | 154 | 3 | 4 | `draw` | pi/naviGATR/spectaGATR/graphs.js:408 |
| 22 | 41 | 115 | 2 | 2 | `BrainImuWheelBench::run` | pi/naviGATR/src/impl/localization/brain_imu_wheel_bench.cpp:280 |
| 23 | 40 | 91 | 3 | 0 | `render` | pi/naviGATR/spectaGATR/capture_panel.js:251 |
| 24 | 38 | 117 | 3 | 1 | `Motion::update` | brain/actuGATR/src/motion.cpp:363 |
| 25 | 36 | 153 | 6 | 1 | `PicoTelemetry::refresh` | pi/naviGATR/src/impl/resources/pico_telemetry.cpp:245 |
| 26 | 36 | 90 | 3 | 1 | `BrainLinkPublisher::state` | pi/naviGATR/src/impl/publishing/brain_link_publisher.cpp:277 |
| 27 | 35 | 73 | 3 | 1 | `render` | pi/naviGATR/spectaGATR/diagnostics.js:94 |
| 28 | 35 | 70 | 4 | 3 | `SensorLossMonitor::update` | pi/naviGATR/src/runtime/sensor_loss.cpp:65 |
| 29 | 34 | 45 | 2 | 2 | `childSection` | pi/naviGATR/src/config/composition.cpp:62 |
| 30 | 33 | 145 | 4 | 3 | `make_pico_telemetry` | pi/naviGATR/src/impl/resources/pico_telemetry.cpp:763 |

#### Top 30 by length (NLOC, production)

| # | CCN | NLOC | Nest | Params | Function | Location |
|---|---|---|---|---|---|---|
| 1 | 41 | 367 | 5 | 7 | `writeSnapshotBody` | pi/naviGATR/src/inspection/inspection_document.cpp:1140 |
| 2 | 105 | 310 | 6 | 2 | `TrackingWheelMotion::run` | pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:510 |
| 3 | 22 | 271 | 3 | 3 | `helloDocument` | pi/naviGATR/src/inspection/inspection_document.cpp:748 |
| 4 | 67 | 270 | 3 | 3 | `TrackingWheelMotion::create` | pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:105 |
| 5 | 79 | 242 | 3 | 3 | `make_synthetic_rig` | pi/naviGATR/src/impl/resources/synthetic_rig.cpp:398 |
| 6 | 15 | 242 | 4 | 1 | `readZip` | pi/naviGATR/spectaGATR/bundle.js:40 |
| 7 | 53 | 240 | 5 | 1 | `TagMountAssociation::run` | pi/naviGATR/src/impl/world_estimation/tag_mount_association.cpp:148 |
| 8 | 52 | 202 | 4 | 3 | `parseFieldMap` | pi/naviGATR/src/config/field_map.cpp:249 |
| 9 | 42 | 196 | 3 | 3 | `make_target_set` | pi/naviGATR/src/impl/resources/target_set_resource.cpp:119 |
| 10 | 31 | 171 | 2 | 4 | `System::build` | pi/naviGATR/src/runtime/system.cpp:148 |
| 11 | 44 | 169 | 6 | 1 | `ConfiguredTargetResolution::run` | pi/naviGATR/src/impl/target_resolution/configured_targets.cpp:107 |
| 12 | 43 | 163 | 3 | 7 | `prepareStep` | pi/naviGATR/src/impl/localization/motion_step.cpp:367 |
| 13 | 25 | 163 | 2 | 2 | `CaptureRecorder::finish` | pi/naviGATR/src/capture/capture_service.cpp:714 |
| 14 | 17 | 163 | 3 | 4 | `writeProfileSubtrees` | pi/naviGATR/src/runtime/brain_profile_builder.cpp:434 |
| 15 | 48 | 155 | 3 | 4 | `BrainImuWheelBench::create` | pi/naviGATR/src/impl/localization/brain_imu_wheel_bench.cpp:32 |
| 16 | 41 | 154 | 3 | 4 | `draw` | pi/naviGATR/spectaGATR/graphs.js:408 |
| 17 | 36 | 153 | 6 | 1 | `PicoTelemetry::refresh` | pi/naviGATR/src/impl/resources/pico_telemetry.cpp:245 |
| 18 | 19 | 149 | 2 | 1 | `SyntheticRigExecutable::execute` | pi/naviGATR/src/impl/resources/synthetic_rig.cpp:225 |
| 19 | 33 | 145 | 4 | 3 | `make_pico_telemetry` | pi/naviGATR/src/impl/resources/pico_telemetry.cpp:763 |
| 20 | 27 | 144 | 2 | 1 | `LibcameraCamera::Impl::open` | pi/naviGATR/src/impl/resources/libcamera_camera.cpp:200 |
| 21 | 56 | 143 | 3 | 1 | `Client::choose` | brain/communiGATR/src/client.cpp:869 |
| 22 | 33 | 136 | 5 | 6 | `make_localization` | pi/naviGATR/src/runtime/localization_stage.cpp:199 |
| 23 | 16 | 135 | 5 | 4 | `detectionFrame` | pi/naviGATR/src/inspection/inspection_document.cpp:606 |
| 24 | 13 | 134 | 2 | 3 | `metadataJson` | pi/naviGATR/src/capture/capture_bundle.cpp:1033 |
| 25 | 30 | 133 | 3 | 3 | `make_attitude_channel` | pi/naviGATR/src/impl/sensors/attitude_channel.cpp:15 |
| 26 | 16 | 132 | 2 | 1 | `CaptureRecorder::writeStatus` | pi/naviGATR/src/capture/capture_service.cpp:955 |
| 27 | 61 | 126 | 2 | 5 | `parseBrainProfileConfig` | pi/naviGATR/src/runtime/brain_profile_builder.cpp:242 |
| 28 | 45 | 125 | 3 | 0 | `run` | brain/locaGATR/src/main.cpp:526 |
| 29 | 29 | 121 | 3 | 4 | `AprilTagDetector::detect` | pi/naviGATR/src/impl/resources/tag_detectors.cpp:112 |
| 30 | 64 | 120 | 1 | 1 | `renderStatus` | pi/naviGATR/spectaGATR/app.js:852 |

#### Deepest nesting (production, top 20)

| # | CCN | NLOC | Nest | Params | Function | Location |
|---|---|---|---|---|---|---|
| 1 | 105 | 310 | 6 | 2 | `TrackingWheelMotion::run` | pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:510 |
| 2 | 44 | 169 | 6 | 1 | `ConfiguredTargetResolution::run` | pi/naviGATR/src/impl/target_resolution/configured_targets.cpp:107 |
| 3 | 44 | 119 | 6 | 0 | `redraw` | pi/naviGATR/spectaGATR/camera_panel.js:363 |
| 4 | 36 | 153 | 6 | 1 | `PicoTelemetry::refresh` | pi/naviGATR/src/impl/resources/pico_telemetry.cpp:245 |
| 5 | 53 | 240 | 5 | 1 | `TagMountAssociation::run` | pi/naviGATR/src/impl/world_estimation/tag_mount_association.cpp:148 |
| 6 | 41 | 367 | 5 | 7 | `writeSnapshotBody` | pi/naviGATR/src/inspection/inspection_document.cpp:1140 |
| 7 | 33 | 136 | 5 | 6 | `make_localization` | pi/naviGATR/src/runtime/localization_stage.cpp:199 |
| 8 | 32 | 82 | 5 | 5 | `parseCommandLine` | pi/naviGATR/src/app/command_line.cpp:9 |
| 9 | 29 | 79 | 5 | 5 | `composeProfile` | pi/naviGATR/src/config/composition.cpp:332 |
| 10 | 27 | 115 | 5 | 2 | `SyntheticRig::render` | pi/naviGATR/src/impl/resources/synthetic_rig.cpp:57 |
| 11 | 25 | 92 | 5 | 3 | `LocalizationExecutor::operator()` | pi/naviGATR/src/runtime/localization_stage.cpp:30 |
| 12 | 24 | 113 | 5 | 2 | `SegmentFollower::update` | brain/actuGATR/src/follower.cpp:141 |
| 13 | 24 | 103 | 5 | 1 | `BrainLinkCommands::run` | pi/naviGATR/src/impl/commands/brain_link_commands.cpp:270 |
| 14 | 16 | 135 | 5 | 4 | `detectionFrame` | pi/naviGATR/src/inspection/inspection_document.cpp:606 |
| 15 | 12 | 40 | 5 | 4 | `fieldEstimateRecords` | pi/naviGATR/src/impl/publishing/field_documents.cpp:220 |
| 16 | 52 | 202 | 4 | 3 | `parseFieldMap` | pi/naviGATR/src/config/field_map.cpp:249 |
| 17 | 35 | 70 | 4 | 3 | `SensorLossMonitor::update` | pi/naviGATR/src/runtime/sensor_loss.cpp:65 |
| 18 | 33 | 145 | 4 | 3 | `make_pico_telemetry` | pi/naviGATR/src/impl/resources/pico_telemetry.cpp:763 |
| 19 | 32 | 102 | 4 | 5 | `shortestRoute` | brain/investiGATR/src/planner.cpp:319 |
| 20 | 32 | 95 | 4 | 0 | `opcontrol` | brain/operaGATR/src/main.cpp:423 |

#### Longest parameter lists (production, top 20)

| # | CCN | NLOC | Nest | Params | Function | Location |
|---|---|---|---|---|---|---|
| 1 | 43 | 163 | 3 | 7 | `prepareStep` | pi/naviGATR/src/impl/localization/motion_step.cpp:367 |
| 2 | 41 | 367 | 5 | 7 | `writeSnapshotBody` | pi/naviGATR/src/inspection/inspection_document.cpp:1140 |
| 3 | 21 | 59 | 1 | 7 | `resolve` | brain/investiGATR/src/reference.cpp:23 |
| 4 | 12 | 41 | 2 | 7 | `encodeY8Jpeg` | pi/naviGATR/src/inspection/jpeg_encoder.cpp:53 |
| 5 | 10 | 34 | 2 | 7 | `nav_apriltag_detect` | pi/naviGATR/src/impl/resources/apriltag_c.c:108 |
| 6 | 8 | 37 | 3 | 7 | `solveFull` | pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:53 |
| 7 | 6 | 25 | 1 | 7 | `finishStep` | pi/naviGATR/src/impl/localization/motion_step.cpp:549 |
| 8 | 5 | 21 | 4 | 7 | `boxDownscale` | pi/naviGATR/src/inspection/jpeg_encoder.cpp:29 |
| 9 | 4 | 20 | 1 | 7 | `segment` | pi/naviGATR/spectaGATR/series.js:402 |
| 10 | 2 | 26 | 1 | 7 | `frameHeaderDocument` | pi/naviGATR/src/inspection/inspection_document.cpp:2023 |
| 11 | 1 | 6 | 0 | 7 | `transformPoint3` | pi/naviGATR/src/math/se3.h:130 |
| 12 | 33 | 136 | 5 | 6 | `make_localization` | pi/naviGATR/src/runtime/localization_stage.cpp:199 |
| 13 | 27 | 93 | 3 | 6 | `parseVisionCorrection` | pi/naviGATR/src/impl/resources/target_set_resource.cpp:19 |
| 14 | 20 | 64 | 2 | 6 | `shape` | brain/investiGATR/src/planner.cpp:482 |
| 15 | 13 | 67 | 2 | 6 | `make_world_estimation` | pi/naviGATR/src/runtime/world_estimation_stage.cpp:71 |
| 16 | 11 | 48 | 3 | 6 | `make_sensors` | pi/naviGATR/src/runtime/sensor_stage.cpp:77 |
| 17 | 9 | 30 | 2 | 6 | `straightExact` | brain/investiGATR/src/planner.cpp:665 |
| 18 | 9 | 27 | 3 | 6 | `findExits` | brain/investiGATR/src/planner.cpp:630 |
| 19 | 8 | 29 | 2 | 6 | `drawDecimated` | pi/naviGATR/spectaGATR/series.js:365 |
| 20 | 8 | 17 | 1 | 6 | `copyY8Plane` | pi/naviGATR/src/impl/resources/camera_capture.h:96 |

### 1.3 Largest files

Production files over 600 lines (21 files; the table shows the top 20):

| Lines | File |
|---|---|
| 2052 | pi/naviGATR/src/inspection/inspection_document.cpp |
| 1614 | pi/naviGATR/src/inspection/http_server.cpp |
| 1582 | pi/naviGATR/spectaGATR/field_scene.js |
| 1450 | pi/naviGATR/src/runtime/system.cpp |
| 1389 | pi/naviGATR/src/capture/capture_bundle.cpp |
| 1351 | brain/communiGATR/src/client.cpp |
| 1350 | pi/naviGATR/src/inspection/inspection_service.cpp |
| 1157 | pi/naviGATR/spectaGATR/app.js |
| 1104 | pi/naviGATR/src/capture/capture_service.cpp |
| 1025 | translaGATR/frame_codec.cpp |
| 917 | pi/naviGATR/src/impl/resources/pico_telemetry.cpp |
| 899 | brain/investiGATR/src/planner.cpp |
| 863 | pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp |
| 840 | pi/naviGATR/spectaGATR/graphs.js |
| 837 | pi/naviGATR/src/impl/commands/brain_link_commands.cpp |
| 725 | pi/naviGATR/src/impl/resources/libcamera_camera.cpp |
| 684 | brain/locaGATR/src/main.cpp |
| 656 | translaGATR/link_documents.cpp |
| 651 | pi/naviGATR/src/impl/resources/synthetic_rig.cpp |
| 623 | pi/naviGATR/spectaGATR/replay.js |

**Tests:**
- 13 test files are over 1000 lines, 6 over 1500 and 3 over 2000:
  - `translaGATR/tests/frame_codec_gtest.cpp` 3554;
  - `pi/naviGATR/tests/brain_profile_gtest.cpp` 2926;
  - `pi/naviGATR/tests/capture_gtest.cpp` 2119.
- Test functions are simple (CCN p99 12) but long: 55 are over 60 NLOC and 23 over 80. The longest is the TEST at `capture_gtest.cpp:1193`, 292 NLOC.
- Inline XML fixtures dominate the long ones:
  - `vision_targets_gtest.cpp:140` configXml, 188 NLOC;
  - `workers_gtest.cpp:162` rigConfig, 161;
  - `synthetic_rig_gtest.cpp:28` rigConfig, 159;
  - `inspection_gtest.cpp:58` rigXml, 158.

### 1.4 What drives the tail

The pattern matters more than the names. Seven shapes produce almost all of the tail.

**1. Configuration built in one function.** 16 of the top 40 functions by CCN parse XML, validate, bind and construct in one body:
- make_synthetic_rig (79), TrackingWheelMotion::create (67), parseBrainProfileConfig (61), parseFieldMap (52);
- parseInspectionConfig (51, in a header), BrainImuWheelBench::create (48), TagMountAssociation::create (45), parseCameraConfig (44);
- make_target_set (42), composition childSection (34), make_pico_telemetry (33), make_localization (33);
- parseCommandLine (32), System::build (31), make_attitude_channel (30);
- plus translaGATR validateRobotProfile (52).

The CCN comes from chains of `!a || !b || ...` and per-attribute checks, for example `tracking_wheel_motion.cpp:164-170` (trimmed): `if (!w.requireDouble("radius_m", radius_m, err) || !w.requireDouble("position_x_m", x, err) || ...`.

Fix pattern, with each step testable without XML:
1. Parse into a plain struct (data only).
2. Validate the struct in a pure function.
3. Construct from it.

**2. Several phases in one `run()`.** `TrackingWheelMotion::run` (`tracking_wheel_motion.cpp:510`: CCN 105, 310 NLOC, nesting 6) does all of these in one body:
- ingest the wheels, then the gyro;
- the calibration gate and the baseline gate;
- interval collection and the alignment check;
- solve and commit.

TagMountAssociation::run (53), ConfiguredTargetResolution::run (44, nesting 6), PicoTelemetry::refresh (36, nesting 6) and prepareStep (43, 7 parameters) have the same shape. Rule: one function per phase, so that `run()` reads as the list of phases.

**3. Document writers.** They are long but mostly flat, and grow one field at a time:
- writeSnapshotBody (`inspection_document.cpp:1140`: 367 NLOC, 7 parameters);
- helloDocument (271 NLOC), detectionFrame;
- metadataJson (`capture_bundle.cpp:1033`) and CaptureRecorder::writeStatus.

Rule: one writer per sub-object, with a top writer that lists them.

**4. Mode flags inside the main loop.** `System::estimationCycle` checks `if (profiled_)` at `system.cpp:1041`, `1048` and `1055`, and again at `1087` and `1321`. See 3.1.

**5. UI render functions.** Status derivation and DOM writes are mixed in:
- renderStatus (`app.js:852`, CCN 64);
- readinessOf (`link_panel.js:29`, CCN 46);
- redraw (`camera_panel.js:363`, nesting 6);
- graphs draw (`graphs.js:408`).

**6. Protocol schedulers.** `Client::choose` (`client.cpp:869`, CCN 56) is a priority scheduler with starvation guards. W5 fix 1 was a bug in it (see B11).

**7. Brain app loops.** locaGATR `run` (`main.cpp:526`, CCN 45) and operaGATR `opcontrol` (`main.cpp:423`, CCN 32) each do input, link service, screen and recalibration in one loop.

### 1.5 Compiler warning probe

Flags added beyond the current `-Wall -Wextra`: `-Wshadow -Wconversion -Wsign-conversion -Wswitch-enum -Wold-style-cast -Wuseless-cast -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference -Wdouble-promotion -Wimplicit-fallthrough`.

| Scope | -Wswitch-enum | -Wconversion | -Wsign-conversion | -Wuseless-cast | -Wshadow | -Wdouble-promotion | all others |
|---|---|---|---|---|---|---|---|
| translaGATR + brain portable (28 files) | 32 (11 switches) | 0 | 4 | 1 | 1 | 0 | 0 |
| naviGATR src (69 files) | 0 | 84 | 41 | 13 | 5 | 3 | 0 |

- **Free today.** `-Wold-style-cast`, `-Wduplicated-cond`, `-Wduplicated-branches`, `-Wlogical-op`, `-Wnull-dereference` and `-Wimplicit-fallthrough` have zero hits.
- **-Wconversion on naviGATR.**
  - 58 of the 84 hits are in `core/time.h` (int64 ms to double, for example in `secondsBetween`), so one header fixes most of them.
  - The rest cluster in `capture_service.cpp` (15), `handoff.h` (12) and `synthetic_rig.cpp` (10).
  - The brain side has 0 `-Wconversion` hits and 4 sign-conversion hits (`planner.cpp:455-472`).
- **-Wshadow on naviGATR.** All 5 hits are the same pattern: an executor lambda's `const ExecutionContext& context` parameter shadows the factory's `context`.
  - Sites: `attitude_channel.cpp:69`, `pico_channels.cpp:180` and `248`, `pico_telemetry.cpp:847`, `synthetic_rig.cpp:643`.
  - It is harmless today, because the lambdas capture only `state`. It is still exactly where a build-time context could be used at run time by mistake.
- **-Wswitch-enum on the brain.** The hits are switches with `default:` over enum class values:
  - `client.cpp:397`, `765` and `1206`;
  - `vex_imu_recalibration.cpp:52` and `58`;
  - `motion.cpp:252` and `467`;
  - `readiness.cpp:35`.
- **-Wswitch cannot help the wire name tables.** It is already in `-Wall`, but it only works when the switch is on the enum type. The tables in D1 switch on `uint8_t`.

### 1.6 Proposed thresholds

**Two tiers:**
- **Warn:** the reviewer must look, then either split the function or write a one-line reason.
- **Fail:** CI blocks new or changed code over it.

Existing offenders go into a baseline file that can only shrink (a ratchet). That lets the standard land before the refactor.

| Metric (production C++ and JS) | Warn | Fail | Over warn today | Over fail today |
|---|---|---|---|---|
| Cyclomatic complexity per function | > 15 | > 25 | 147 | 60 |
| Function length, NLOC | > 60 | > 100 | 113 | 46 |
| Nesting depth (control and lambda blocks) | > 3 | > 4 | 32 | 15 |
| Parameters | > 5 | > 7 | 30 | 0 |
| File length, physical lines | > 600 | > 1000 | 21 | 10 |
| Test file length | > 1500 | > 2500 | 6 | 2 |
| Test function NLOC, raw-string fixtures excluded | > 80 | none | 23 (fixtures included) | - |

**Why these values:**
- **CCN 15/25.** The codebase p90 is 11 and p99 is 35. 15 is lizard's own default and flags 5.7 percent of functions. 25 blocks 2.3 percent, and almost all of those are the construction and multi-phase functions from 1.4, which is where the user already finds naviGATR hard to read.
- **NLOC 60/100.** That is one to two screens at 100 columns; p90 is 35.
- **Nesting 3/4.** 95 percent of functions are at 2 or less. Past 3, early returns are no longer being used.
- **Parameters 5/7.** Nothing exceeds 7 today, so "fail over 7" costs nothing and holds the line. The 6 and 7 parameter functions (prepareStep, writeSnapshotBody, make_localization, transformPoint3) should take a named struct. That is also the named-type readability the user asked for.
- **File 600/1000.** The 10 production files over 1000 lines are the same files that hold the tail.
- **Readability is king.** The numbers trigger a split; they are not a goal. A flat 20-case name table at CCN 21 is fine as a baseline waiver. A 60 NLOC function nested 5 deep is not.

**Tooling:**
- **lizard** covers C++ and JS with the same numbers in one pip install: `lizard -w -C 25 -Tnloc=100 -a 7` for fail and `lizard -w -C 15 -Tnloc=60 -a 5` for warn.
  - I ran the fail command on `translaGATR/link_documents.cpp`. It prints gcc-style warnings, for example `link_documents.cpp:262: warning: translagatr::validateRobotProfile has 100 NLOC, 52 CCN ...`.
  - `-L` would gate total lines, not NLOC.
  - lizard's nesting extension crashed here, so nesting needs clang-tidy or review.
- **clang-tidy** (`readability-function-size` with NestingThreshold, and `readability-function-cognitive-complexity`) would add nesting and cognitive complexity. It is not installed: a decision.

## 2. Duplication

The exact token duplicate rate over production code is low (1.35 percent). The real duplication is semantic: the same rule written again by hand in another file, usually with a small difference.

Good examples of the opposite already exist:
- one CRC-16 and one CRC-32 in translaGATR (`frame_codec.cpp:271`, `link_documents.cpp:126`), used by the Pi, the Brain, the Pico and the ZIP writer (`zip_writer.cpp:111`);
- one JSON writer (`inspection/json_writer.h`), used by inspection, capture and instrumentation.

### D1. Wire enum name tables: real drift (highest value)

For translaGATR enums, about 40 name functions exist in 10 files. Each is a `switch` on `uint8_t` with a `default:`.

**picoOpName, 4 copies:**
- `diagnostics/instrumentation.cpp:86`: "CONFIGURE", default "unknown";
- `impl/resources/pico_telemetry.cpp:38`: "CONFIGURE", default "command", no case 0;
- `inspection/inspection_document.cpp:294`: "configure", default "unknown";
- `capture/capture_bundle.cpp:101`: "configure", default "".

**Brain result names, 3 copies:**
- `impl/commands/brain_link_commands.cpp:33` brainResultName: "Ok", "UnknownSession", all 12 values, default "unknown result";
- `capture/capture_bundle.cpp:51` brainResultName: "ok", "unknown_session", all 12 values, default "";
- `inspection/inspection_document.cpp:229` resultName: 6 of the 12 values, "other" for the rest.

**brainOpName, 2 copies:**
- `brain_link_commands.cpp:17` (default "unknown op");
- `capture_bundle.cpp:35` (default ""). It sits inside a 1260-line anonymous namespace (`capture_bundle.cpp:18` to `1277`), so it silently coexists with the first one.

**Pico IMU state name, 4 copies:** `instrumentation.cpp:55`, `inspection_document.cpp:263`, `capture_bundle.cpp:78`, `runtime/sensor_loss.cpp:13` (default "in an unknown state").

**picoDetailName, 3 copies:** "wrong target" at `instrumentation.cpp:107`, "wrong_target" at `inspection_document.cpp:315`, and a third at `capture_bundle.cpp:122`.

**Also duplicated:**
- picoCommandStatusName x3, picoImuReasonName x3, firmware name x3;
- pathModeName x2;
- imuSourceName x2 (`inspection_document.cpp:210`, `capture_service.cpp:71`);
- calibrationName x2 (brain `readiness.cpp:85`, `inspection_document.cpp:206`).

**profileReasonName across the link.** The brain copy (`robot_profile.cpp:170`) says "Pi build" and "not accepted by this Pi". The Pi copy (`brain_profile_builder.cpp:221`) says "build" and "not accepted".

**Telemetry enums decoded on the Pi.** `inspection_document.cpp:1696`, `1702`, `1714` and `1722` (motionStateName ... driveFaultName) re-implement actuGATR's `toString` (`motion.cpp:76`, `89`; `drive.cpp:10`). frames.h says these wire values "are actugatr MotionState ... values", so the enum is owned outside translaGATR.

**Effect:**
- The same wire value reaches the viewer JSON, the capture CSV, the instrumentation panel and the event log under different spellings.
- A new enumerator silently becomes "unknown", "other" or "".

**Fix:**
- One `const char* name(EnumType)` per wire enum, in translaGATR next to the enum.
- It switches on the enum type with no default, so `-Wswitch` fires in every component that includes it.
- One spelling (snake_case) for machine output; display text stays in the viewer.

### D2. Request equality and validation: Pi server vs Brain fake Pi

| Function | Pi | Brain fake |
|---|---|---|
| sameRequest | `brain_link_commands.cpp:99` | `communiGATR/sim/fake_pi.cpp:55` |
| sameTelemetry | `:54` | `:33` |
| repeatable | `:86` | `:26` |
| wellFormed | `:137` | `:96` |

- Same cases, different order, so lizard does not flag them.
- sameTelemetry compares 21 fields plus an array one by one. A field added to `BrainTelemetry` has to be added by hand to both copies, and the fake has to keep tracking the real server.
- **Fix:** these are wire facts, so they belong in translaGATR. Or compare the encoded bodies byte for byte, which is what "same request" means on the wire.

### D3. Wire units, pi and the heading rule

**kPi, 6 definitions:**
- `investigatr/geometry.h:20`, `communiGATR/src/attitude.cpp:13`, `translaGATR/link_documents.cpp:13`, naviGATR `math/angles.h:11`;
- literal `3.14159265358979323846` in pico `gyro_window.cpp:13` and naviGATR `stationary_precheck.h:32`.

**kCdegPerRad, 3 definitions:** actuGATR `telemetry.cpp:14`, communiGATR `link_driver.cpp:23`, `robot_profile.cpp:16`.

**wrapAngle, 2 implementations:** naviGATR `angles.h:14` (while loops) and investiGATR `geometry.cpp:10` (`std::remainder`).

**The wire heading rule "(-18000, 18000]" is written 3 times:**
1. naviGATR `math/angles.h:30-33`: llround, then map -18000 to 18000. This was a real bug fix (memory project_investigatr_brain_link.md: "radToCdeg maps -18000 to 18000"), tested at `brain_io_gtest.cpp:1075-1080`.
2. communiGATR `link_driver.cpp:191-196`: round, range check, then the same mapping.
3. actuGATR `telemetry.cpp:41-47`: the comment says `// Angle to centidegrees in (-18000, 18000].`, but the code is `return i16(investigatr::wrapAngle(rad) * kCdegPerRad);` with no -18000 mapping.

The third copy has the bug the Pi fixed, still latent. An angle just above -pi (for example -pi + 1e-5 rad = -17999.94 cdeg) rounds to -18000, outside the stated range. It is display only (telemetry target and heading error), and it was found by this survey, not by a test run.

**Rounding and range helpers, 3 policies:**
- `telemetry.cpp:17` saturate: clamps, and non-finite becomes 0;
- `link_driver.cpp:32` toWire: refuses;
- `robot_profile.cpp:20` toI32: refuses.

**Fix:** translaGATR owns wire units: `toCdeg`, `fromCdeg`, `toMm` and friends, with a named policy (saturate or refuse) and the wrap rule written once.

### D4. Hand-rolled ring buffers and hand-trimmed deques

**Modular-index rings:**
- `diagnostics/hub.cpp:75-91`;
- `state/pose_history.cpp:55-66` (pose ring) and `95-107` (attitude ring): the same code twice in one class. lizard also pairs `pose_history.cpp:131-143` with `174-186`.
- brain `pros_usb_port.cpp:33` FrameQueue;
- in JS, the trail ring in `field_scene.js` and `series.js` Channel.

**Deques trimmed by hand:**
- `system.cpp:974-977`, a vector used as a queue: `events_.erase(events_.begin())`;
- `link_monitor.cpp:139-141`;
- `pico_telemetry.h:267` requests_;
- six deques in `capture_recorder.h:187-217`;
- `field_documents.h:102`.

**Fix:** one `BoundedRing<T>` in naviGATR core. A second use already exists, so this is extraction, not invention.

### D5. Serial open and reopen, twice

- `pros_usb_link.cpp:143-160` repeats the termios setup of `transport/serial_port.cpp` (cfmakeraw, speeds, CLOCAL|CREAD, CRTSCTS).
- It has its own reopen timer with no injectable clock (`pros_usb_link.cpp:145-146`, `if (nowUs() < retry_at_) return false;`).
- Meanwhile `serial_links.cpp:50-75` ReopeningLink already has an injectable clock and counters.
- `pros_usb_link.cpp` also holds 28 of the repo's 87 unbraced one-line ifs.

### D6. The two Brain apps

`locaGATR/src/main.cpp` and `operaGATR/src/main.cpp` both define:

| Helper | locaGATR | operaGATR | Difference |
|---|---|---|---|
| sensorsReady, calibrating | 146-158 | 309-321 | identical (diffed) |
| createLink | 53 | 120 | one line differs |
| attitude | 75 | 142 | |
| sendTelemetry | 83 | 276 | |
| now | 49 | 58 | |
| screen `row` helper | 410 | 227 | |
| VEX recalibration block | 609-617 | 488-496 | the button named in one message |

`docs/communigatr.md` ships that recalibration loop as code to copy (brain-arch progress/wA.md, FIX item 4).

**Fix:** communiGATR provides the readiness predicates and the recalibration service; the apps keep only their screens and buttons.

### D7. naviGATR stage plumbing (lower value)

- `runtime/resource_stage.h:40-79` vs `sensor_stage.h:39-77`: 40 lines of the same executor shape.
- `impl/*/register_*.cpp` boilerplate (5 files).
- `attitude_channel.cpp:8-32` vs `camera_channel.cpp:7-31`.
- `attitude_reference.cpp:3-25` vs `imu_heading_increment.cpp:3-25`.
- Header blocks `tracking_wheel_motion.h:64-100` vs `imu_heading_increment.h:41-77`.

These are candidates only once a PIPELINE.md shows the shared shape.

### D8. Defaults and freshness limits written many times

**The same default in the struct and again in the parser:**
- `tracking_wheel_motion.h:129` `long max_gap_ms = 250;` and `tracking_wheel_motion.cpp:266` `getInt("max_gap_ms", 250, ...)`;
- `imu_heading_increment.h:91` and `.cpp:32`;
- `brain_profile_builder.h:101/108` and `.cpp:38/94`.

**The literal 250 (ms)** appears 36 times in production code, under at least eight names: kFrameFreshMs, kGyroGapMs, kAttitudeFreshMs, kProfileAttitudeMaxAgeMs, stale_after_ms, max_gap_ms, sensor_loss_ms, encoder_stale_after_ms. Some are one policy (sensor freshness); some only coincide.

**Fix:**
- One named constant per meaning.
- Parsers take their default from the struct (`Config{}.max_gap_ms`).

### D9. Clock offset estimators, three copies of one algorithm

- naviGATR `core/clock_sync.h:33` DeviceToHostClock: windowed minimum of host minus device.
- spectaGATR `latency.js:16` ClockSync: minimum RTT of the last 20 pings.
- `tools/perf` pingOffsetEstimates: the same as the viewer, on purpose.

Legitimate across languages, but it is one algorithm with one error bound (+-RTT/2). Document it once and cite it from each copy.

### D10. Readiness derived twice from the same state

brain communiGATR `readiness.cpp` (Readiness, toString at `:55`) and spectaGATR `link_panel.js:29` readinessOf (CCN 46) both turn the state block into ready, waiting or bad. Check before acting: if the Pi published one readiness verdict, both could display it.

### D11. Hand-built JSON beside JsonWriter

`inspection_service.cpp:560-563` (`"{\"ok\":true,\"running\":"` plus concatenation) and `capture_http.cpp:126`. They are small, but they bypass escaping.

### D12. Periodic and retry timers: one idea, many shapes

| Timer | Site |
|---|---|
| inspection state and preview due times | `inspection_service.cpp:904` and `1124` |
| estimate period | `field_documents.cpp:312` |
| publisher period | `brain_link_publisher.cpp:81` |
| capture cooldown (`auto_last_us_ >= 0` as "never") | `capture_service.cpp:619` |
| serial reopen | `serial_links.cpp:61`, `pros_usb_link.cpp:145` |
| Pico diagnostics retry (5 s) | pico_telemetry |
| rig frame period | `synthetic_rig.cpp:340` |
| worker loop | `system.cpp:1414-1430` |
| telemetry drop after 2 periods | brain `client.cpp:961` |
| reopen once a second | `pros_link.cpp:150` |
| serial retries, every 2 ms up to 100 ms | `pros_serial_port` |

Each uses its own time type: chrono `time_point`, int64 us, `MonotonicTime`, `Seconds` double, uint32 ms.

- A shared helper is justified only where the policy is really the same: reopen-with-retry has 3 copies.
- The rule that matters more is one time type per process (see 3.5).

### D13. Parsers outside the XML reader

XML attribute reading goes through one reader (ConfigNode), which is good. Two other kinds of parser are hand-rolled more than once:

| Parser | Copies |
|---|---|
| command line | `app/command_line.cpp:9` parseCommandLine (CCN 32, nesting 5); `tools/perf/inspect_bench.cpp:119` parseOptions (CCN 36); `tools/perf/cdp_bench.cpp:88` parseOptions (CCN 26) |
| JSON reading, in tools and tests | `tools/perf/perf_json.cpp:126` Parser::value (CCN 29); `tests/inspection_document_gtest.cpp:83` JsonCheck::value (CCN 25); `tests/inspection_test_client.h:375` jsonString |

This is low risk, since it sits in tools and tests. It is still where a small vendored library (the third_party rule) beats three hand parsers.

## 3. Systems-level complexity

### 3.1 naviGATR: the main pipeline is not where the folders say it is

**The cycle.** `System::step` (`runtime/system.cpp:1022`) calls `estimationCycle` (`:1033`), `fieldCycle` (`:1106`) and `reportingCycle` (`:1066`). They sit in a 1450-line file whose first 1000 lines are profile hosting, Pico commands, wheel reads, sensor-loss watching, recovery notes and events.

**Construction.**
- `System::build` (`system.cpp:148`, 171 NLOC, CCN 31) parses the XML and calls:
  - `make_resources` (`resource_stage.cpp`);
  - `make_sensors` (`sensor_stage.cpp:77`);
  - `make_localization` (`localization_stage.cpp:199`);
  - `make_world_estimation` (`world_estimation_stage.cpp:71`).
- Implementations are found by XML name through FunctionRegistry (`core/function_registry.h`). The registry is filled by the `register_*.cpp` file in each impl folder plus `runtime/register_all.cpp`.
- So "where the make pipeline is" needs four files and one indirection, and no file is called pipeline.

**runtime/ is a grab bag of 26 files:**
- the coordinator (`system.*`) and the four stage builders;
- Brain profile building (`brain_profile_builder.*`);
- the sensor-loss and stillness policies (`sensor_loss.*`, `stationary_precheck.*`);
- a config parser in a header (`inspection_config.h`);
- catalogs, `handoff.h` and `pico_control_ref.h`.

**The mode flag.** `profiled_` (Brain-profiled vs XML-configured localization) is checked at `system.cpp:293`, `1041`, `1048`, `1055`, `1087` and `1321`.
- The sensor-loss rule exists only on the profiled path. The monitor is configured on profile apply (`system.cpp:866`).
- brain-arch progress/wD.md FIX: "XML-configured localization is unaffected".
- So one pipeline has two safety behaviors, chosen by which config path was used.

**Naming drift.**
- The stage is WorldEstimation (`runtime/world_estimation_stage.*`, `impl/world_estimation/`), but its contract is `contracts/field_estimation.h`.
- `contracts/perception.h` and `association.h` are sub-steps of one estimator, yet they sit at the top level of `contracts/`.

**Docs.** `pi/naviGATR/README.md` is 278 lines (the convention is a short signpost) and `docs/architecture.md` is 381. The stage order is written in `README.md:63-64` but is not visible in the code.

**Proposed rules:**
- **Entry function.** Each pipeline's entry is one short function that reads as the stage list, for example step: resources, sensors, commands, localization, world estimation, target resolution, publishing. It lives in a file named for the pipeline (`pipeline.cpp`) and stays under 100 lines.
- **Construction.** Each stage's builder lives beside the stage. The top-level build is a list of those calls, in the same file as the entry.
- **No mode flags.** Optional behavior is a stage or a policy chosen at build time, not a flag checked inside the loop.
- **PIPELINE.md.** The one at the pipeline root names the entry function and the build function by path.

### 3.2 Reporting paths on the Pi: seven channels

These are the channels for "something happened" and "how is it going":

1. **Diagnostics** (`core/diagnostics.h`): per-slot FunctionStats and per-link LinkStats (bytes, packets, decode errors, request and reply counters). Fed through `in.diagnostics->links[...]` at `brain_link_commands.cpp:278`, `brain_link_publisher.cpp:447` and `pico_telemetry.cpp:254`.
2. **LinkMonitor** (`diagnostics/link_monitor.h`): rx_bytes, tx_attempted, tx_accepted, frames, rejected and reader stats per link. These are byte and frame counts for the same links as channel 1.
3. **DiagnosticsHub** (`diagnostics/hub.h`): a bounded ring of DiagRecords for capture.
4. **System::noteEvent** (`system.cpp:961`): copies the text into a fixed char array for the hub, and appends it to `events_` (32 kept, `system.cpp:68`).
5. **System::noteFault** (`system.cpp:980`): capture triggers. Callers raise both channels 4 and 5 with the same text, at `system.cpp:748-749` ("Brain link quiet for 1 s" twice), `763-765`, `769-771` and `879-882`.
6. **warnings_** (build warnings) and **stderr** in `app/main.cpp` (18 prints).
7. **Per-function status fields** such as ObservationReadiness dropped_intervals and dropped_why (`contracts/localization.h:75`), stats structs (estimationStats, fieldStats, handoff stats), inspection per-client stats and the capture status.

**Naming and matching problems:**
- "diag" names five different things: Diagnostics, DiagnosticsHub, the diag feed document, the Pico diagnostic frame 0x14, and diagDocument.
- Events are free text, and tests match on the text. The W3 fix tests check that events.csv contains "Brain link quiet for 1 s".

**Rules:**
- One event call (`raise(kind, text)`) with a typed kind that fans out to the log, the hub and the capture triggers.
- One counter owner per link.
- Event kinds are an enum with a name table (D1), and tests match the kind.

### 3.3 Configuration paths

**Pi, five ways in:**
- a `<System>` XML;
- a `<Configuration>` composed from files (`config/composition.cpp`; include rules in `composition.h:1-30`);
- a Brain profile turned into generated XML subtrees (`brain_profile_builder.h:1-6`: "the typed profile turned into Sensors and Localization subtrees from fixed Pi templates");
- `.xml.in` templates, which are refused at run time;
- CLI flags (`app/command_line.cpp`).

Inspection and capture configs are parsed in headers (`inspection_config.h:74`, `capture_config.h:89`). There are 34 config XML files.

**Robot geometry, three places:**
- `brain/robot/gatr2_robot.h` for profiled configs;
- `pi/naviGATR/config/shared/robots/*.xml` for XML configs. The Brain switch `kSendProfile` selects between the two (brain-arch final_report section 6).
- each app's `robot_config.h`, for drivetrain, gains and test poses.

**Defaults** are duplicated between struct and parser (D8).

**Rules:**
- One path to a given setting. A new path needs a named consumer that cannot use the existing one.
- Defaults live in one place.
- Non-trivial functions go in `.cpp` files, not headers.

### 3.4 Schedulers, rate limiters and retries

See D12 for the timers. The scheduling on top of them:

**Brain:**
- `Client::choose` is a hand-written priority scheduler. The order, from progress/wA.md: HELLO, placement, control, profile, due poll, READ_WHEELS, map chunk, estimate chunk, path report. It has a starvation guard (4 due polls) and, since W5, an overdue rule.
- ProsLink runs the client in a task; ProsDrive has its own task and mailbox; each Brain app has its own main loop.

**Pi:** the estimation worker, the field worker, the inspection thread, the capture thread and the HTTP server's select loop.

Most of this is necessary. The rule is to make the policy explicit, as a table of what runs when in the PIPELINE.md, rather than to add a framework.

### 3.5 Time representation

**A good core type, with holes.** naviGATR has MonotonicTime tagged with ClockDomain, debug asserts (`core/time.h:17-60`), and one HostClock (`core/host_clock.h`). But:
- `sameDomain` (`time.h:36-38`) accepts kUnset against any domain, so a comparison with an unset time (ms 0) passes the assert;
- `operator==` and `operator!=` (`time.h:46-47`) do not check the domain;
- `operator-` returns a bare int64_t (milliseconds by convention);
- the asserts exist in Debug builds only.

**Other time types in the same process:**
- int64_t microseconds (`HostClock::nowUs`, `host_us` fields, `pi_host_us`);
- std::chrono time points (`inspection_service.cpp:49` `using Clock = std::chrono::steady_clock;`, `http_server.cpp:58`, `system.cpp:1414`);
- plain double milliseconds.

`inspection_service.cpp:896-919` uses three of these in one function.

**Two microsecond clocks with different origins.** Both are int64_t:
- `resources/serial_link.h:33` steadyNowUs() counts from the steady clock's epoch ("arbitrary origin");
- `HostClock::nowUs()` (`core/host_clock.h`) counts from process start.

Nothing in the type stops a value from one being compared with the other.

**Other components:**
- the Brain uses `Seconds` (double) in investiGATR and actuGATR, and uint32 ms from `pros::millis` in PROS code;
- the Pico uses uint32 ms and us;
- JS uses ms numbers, plus `pi_host_us` from the Pi.

**Clock chosen by mode.** `system.cpp:1207`: `d->processedAt = running_.load() ? HostClock::now() : now;`

**Rule:** one clock source and one duration type per process (std::chrono or a tagged type). Raw integers only on the wire and in files, with the unit in the name.

### 3.6 One protocol exchange split across two pipeline slots

- Brain link requests are decoded in the Commands slot and answered in the Publishing slot of the same cycle.
- The two meet through `state/command_state.h:30-58` BrainReplyContext: a 25-field struct of wire codes, where each field applies only to some ops. `command_state.h:25-29`: "result is final except SET_POSE Pending ..., READ_DOC Ok ... and CONTROL Ok ...".
- The design review flagged the pairing risk (design_review.txt: "The protocol is split across two slots that communicate through CommandState.reply, and nothing checks that they are paired").
- **Rule:** a request/reply exchange lives in one module, and per-op data is a `std::variant`, not a struct of all fields.

### 3.7 Mechanisms with hidden per-field contracts

**The identical-diag skip** (W1 fix 3) hashes the diag JSON minus its "varying" spans. Every writer call site must mark its varying values: there are 57 marks in `inspection_document.cpp`.
- Missing one mark stops the skip. That was the W1 bug.
- Marking a content field hides a real change.

**Same shape elsewhere:** field-by-field equality (D2) and the capture CSV column lists.

**Rule:**
- Prefer mechanisms that stay correct when a field is added without an annotation (for example, hash the inputs that matter rather than the output minus exceptions), or put the annotation in the type.
- Otherwise, add a test that fails when a field is added without its annotation.

### 3.8 When a new mechanism or abstraction is allowed

This proposal is consistent with feedback_design_principles.md.

1. **Extract, do not invent.** A helper, base class or template can be introduced only when a second real use exists in the tree, and the same change converts both uses. D1, D3, D4 and D6 show this is needed.
2. **Replace, do not add.** A new event, config, logging, timing or queue mechanism must name the existing one it replaces, or say why none fits. The old one is removed in the same series, or tracked.
3. **One owner per fact.** Every wire constant, name, unit conversion and freshness limit has one definition, and everything else refers to it.
4. **No flags in the loop.** Optional behavior is chosen at build time (a stage or a policy object), not by `if (mode)` inside the main loop.
5. **Docs move with the pipeline.** The affected PIPELINE.md is updated in the same change for a new stage, mechanism or thread.
6. **Hidden-contract test.** If a mechanism is correct only when every future field is annotated, it needs a test that fails when a field is added without the annotation, or a different design.

## 4. Past bug classes

Each class below has what was actually hit (from the reports and code), a preventive rule, and how the rule could be checked. "Report" refers to the handoff files listed in section 0. "Code today" is what is still in the tree.

### B1. Clock-domain and time-base mixing

**Examples:**
- **Design review, "Bus timing / reply deadline".**
  - Commands and Publishing get the same cycle-start `now`, so a reply deadline measured from it always shows 0 elapsed.
  - MonotonicTime counts whole ms, so the 1000 us and 200 us guards could not be measured.
  - Tests drove `step()` with synthetic times while the code compared against `HostClock::now()`: "drops every reply, or none of them".
  - Resolved by moving deadlines into the link, in microseconds (`resources/serial_link.h:39-48` TransmitWindow).
- **W2 fix 9.** Live telemetry was plotted at message build time instead of Pi arrival. It now uses `received_host_ms`, else `host_ms - age_ms`.
- **W2 fix 2.** Ping ids were reused after a double clock reset in the viewer ClockSync.
- **W0 fix 4.** "Publish to receive" included the document build, because the timestamp was taken before the build.
- **W0 fix 5.** A cross-clock ping offset was reported without its +-RTT/2 bound.
- **W4 open risk.** Encoder freshness uses the pipeline clock, while `updated_ms_ago` uses HostClock at write time.
- **viz final_report section 4.** The Brain attitude is timed by Pi arrival, about one telemetry period plus link latency late. Documented, not fixed.
- **Code today:** see 3.5. Three time types in `inspection_service.cpp:896-919`; the clock chosen by mode at `system.cpp:1207`; the sameDomain hole; two int64 microsecond clocks with different origins.

**Rule:**
- Every timestamp has its clock and unit in its type or name: `_host_us`, `_device_ms`, or MonotonicTime with a domain.
- Never compare or subtract across domains.
- The pipeline passes `now` in. Code under the pipeline never reads a clock itself, except to time its own work.
- A time that means "when measured" is taken at the measurement, never at send or build.

**Check:**
- **Types.** The domain check runs in all builds, not only Debug; std::chrono durations replace int64 counts.
- **CI grep gate.** Flag `steady_clock::now` and `HostClock::now` outside `core/`, `app/` and an allowlist.
- **Test pattern.** Every threaded component takes an injectable clock, as ReopeningLink (`serial_links.cpp:52` setClock) and capture (`capture_service.cpp:316` `options_.clock`) already do.
- **Review item:** "which clock, which unit".

### B2. Zero, -1 or null as a sentinel; missing data read as zero

**Examples:**
- **W2 fix 3.** A valid pose with `age_ms` null was drawn as live. motion_step now empties `measuredAtHost` when the source clock is not mapped.
- **viz final_report.** Overlays are drawn only with a resolved target (flag bit 3), because (0,0,0) looked like a valid target. Graphs draw missing data as a gap, "never as zero".
- **W4 fix 3 (declined path).** Withdrawing the Brain attitude through a sentinel on an existing field was refused as "a hidden contract on a shared payload". A typed `available` field was proposed instead.
- **Design review.** request_id "starts at 1, never 0" with the wrap undefined; session 0 means none; with a deterministic nonce, session B could reuse session A's records.

**Code today:**
- `diagnostics/hub.cpp:58-59` forces `host_us` to at least 1 so that 0 can mean "unset": `record.host_us = std::max<int64_t>(1, HostClock::nowUs());`.
- `link_monitor.h:57-59` uses -1 for "never", and `capture_service.cpp:619` tests `auto_last_us_ >= 0`.
- There are about 55 production comments of the form "0 = none", for example tickets at `client.h:71`, `90` and `125`, and ObjectId and MapId at `investigatr/field.h:15-16`.
- The kUnset MonotonicTime compares as ms 0 (3.5).

**JS today:**
- `x || 0` in geometry turns a missing mount coordinate into the origin: `transforms.js:51` and `57-59` (`T.x_m || 0`), `field_scene.js:52-54`.
- `opts.height_m || 0.075` (`field_scene.js:96`) and `rb.height_m || 0.3` (`field_scene.js:772`) replace a real 0 with the default.
- spectaGATR has 45 `|| <number>` defaults against 10 uses of `??`.

**Rule:**
- Absence is typed: `std::optional` in C++, `null` checked with `??` or an explicit test in JS, or a status enum. 0 is a value.
- Where the wire needs a reserved value (ids, sessions), the reserved value is a named constant (for example `kNoSession`) with a helper. Never compare against a bare 0 or -1.
- In JS, use `??` for defaults and `||` only for booleans.

**Check:**
- **Review item.**
- **Grep gate** for `|| 0` and `|| <number>` in spectaGATR (ESLint only if the team accepts the tooling).
- **clang-tidy** `bugprone-unchecked-optional-access`, once optional is in use.
- **Test pattern:** each consumer has a test with the field absent.

### B3. Stale cached state; freshness not carried with the data

**Examples:**
- **W4 fix 1.** Encoder `delta_1s`, direction and note stayed "current" after Pico frames stopped. Fixed with PicoEncoderView::fresh (`pico_telemetry.h:93`) against kFrameFreshMs (`pico_telemetry.h:156`).
- **W4 fix 3.** A withdrawn Brain attitude read as measured, then stale.
- **W2 fix 1.** Replay never marked a stale robot row.
- **phase_c review 1.** ProsDrive's busy reads looked idle; fixed with the DriveRequests mailbox.
- **phase_c 01:55.** An A/X/Y goTo was replaced in the same cycle by `manual(zero)`, decided from a stale snapshot. A fresh status read was added, with tests such as DriveRequests.CenteredSticksNeverCancelANewCommand.
- **wA FIX 2.** An APPLY Ok marked the profile applied before any state had shown it.
- **Design review, "Frame capture race after placement".** The Brain driver's cache still held the old anchor.
- **W3 fix 2.** Host metadata was read only at finalize, so the bundle described the end of the capture, not the window start.

**Rule:**
- A cached copy carries the time it was taken. The reader computes freshness against a named limit.
- A "busy" or "no answer" read returns a distinct status, never the last value as if it were new.
- A decision that drives actuators reads state in the same cycle it acts.

**Check:**
- **Test pattern "source stops":** freeze the producer, advance time past the limit, and assert not fresh, or null.
- **Review item.**

### B4. Missing reset on identity change (session, epoch, boot, connection, profile)

**Examples:**
- **W2 fix 2.** The viewer reset its clock twice (in onOpen and on hello). Now once per connection.
- **W3 fix 5.** `enc*_delta_counts` spanned lost frames. Now a delta is taken only against `seq - 1` of the same boot and epoch, and capture segments split at session, reset, epoch and boot.
- **W4 fix 2.** After a Pi restart, the Pico kept sending the 0x14 frames an earlier Pi had asked for.
- **wB FIX 3.** After calibration, the gyro seeded one sample later than the wheels.
- **wA FIX 2**, and the test ProfileChange.NewProfileNeverShowsTheOldPlacement.
- **Design review.**
  - HELLO left `init_*` untouched, so a pending placement from session A could apply in session B.
  - An in-flight SET_POSE retried across a Pi restart could relocate the new instance.
- **Code today (a manual contract):** `clock_sync.h:20-22` says "the owner must reset the mapper when it detects the discontinuity".

**Rule:**
- State tied to an identity stores that identity and is dropped when the identity changes.
- Prefer keying (by session or boot) or a generation checked at read time over a `reset()` the owner must remember to call.
- `reset()` resets every member, by assigning a value-initialized whole (`*this = State{}` or `state_ = State{}`). Commit 4992248 shows the hazard: adding the member `window_` meant editing `reset()` by hand, from `void reset() { stash_ = Stash{}; }` to also clear `window_` (`motion_step.h`).

**Check:**
- **Test pattern "identity change mid-flow":** change the session, boot or connection in the middle of a flow, for every stateful component.
- **clang-tidy** `cppcoreguidelines-pro-type-member-init`.
- **Review item.**

### B5. Snapshot consistency: data and labels read at different moments

**Examples:**
- **W1 fix 4.** The reset label was read after the snapshot. `underOneReset` (`inspection_document.cpp:1055`) now reads the reset count before and after the build, and rebuilds when they differ.
- **W3 fix 2** and **W0 fix 4**, as in B1 and B3.

**Rule:**
- A document is built from one snapshot, taken under one lock or one generation read.
- Labels (session, epoch, reset count) are read in the same snapshot as the data they label.

**Check:**
- **Stress-test pattern**, as ResetsUnderLoadNeverLabelAnOldPoseWithTheNewSession.
- **Review item.**

### B6. Locks, blocking I/O and task lifecycle

**Examples:**
- **phase_c review 4.** A lock was held across the path sink. It went away with the mailbox, since only the drive task calls Motion now.
- **Memory review wf_1e521a91 (medium).** A PROS `task_delete` of a competition task holding the drive mutex deadlocked the drive task, and the motors kept their last voltage. `~ProsDrive` now quits cooperatively with a bounded wait (`pros_drive.cpp:15-24`).
- **wB review F / memory review (low).** The tcdrain fallback was unbounded while DE was held. It was replaced by non-blocking TIOCOUTQ polling.
- **W1 fix 1.** An immediate closesocket turned a close into a reset. A lingering close was added.
- **wA A3.** ProsLink opens the port outside the mutex (`pros_link.cpp:143-153`). Public calls take the mutex with a 20 ms bound and answer "busy".

**Code today:**
- `ProsLink::run` holds the mutex across `client_.poll(now())` (`pros_link.cpp:156-158`), which performs the serial exchange. That is bounded by design, but it is I/O under a lock.
- ProsDrive uses manual take/give pairs six times (`pros_drive.cpp:28-110`), while ProsLink uses an RAII Lock (`pros_link.cpp:64-80`). Two idioms on one platform.

**Rule:**
- No I/O, sleep, or call into another module while holding a lock: copy under the lock, act outside it.
- Every wait has a bound and a counted timeout.
- RAII guards only; no manual give.
- Tasks stop cooperatively, never by delete while they may hold a lock.

**Check:**
- **clang `-Wthread-safety`** with annotations. It needs clang, which is not in the toolchain today: a decision.
- **Review item.**
- **Test pattern "busy caller":** hold the lock in a test and assert that callers return busy within the bound.

### B7. Unbounded queues, or FIFO where latest-wins was meant

**Examples:**
- **W0 baseline.** The old server queued whole snapshots FIFO. A 100 kB/s viewer saw poses 9 to 12 s old (viz final_report section 2).
- **W1 fix 1.** Kernel and tunnel buffering bypassed latest-wins; fixed with flow control by ping/pong.
- **W1 fix 2.** The reliable_kb floor was below the hello size.
- **W3 fix 4.** max_mb did not bound record memory, because of vector doubling and one growing CSV string.
- **W0 fix 2.** A per-client cap in the perf harness.

**Positive pattern:** DiagnosticsHub (`hub.h:1-11`): "bounded ring ... When the ring is full the new record is dropped and counted per kind, so a stalled consumer can never slow a producer or grow memory".

**Rule:**
- Every queue has a capacity, a drop policy (drop newest, drop oldest, or replace) and a drop counter that someone reads.
- Streaming state is latest-wins; events and documents are bounded FIFO.
- Memory limits are enforced on the bytes actually held.

**Check:**
- **Review item** "capacity, policy, counter".
- **Test pattern "stalled consumer":** stop the reader, push N times the capacity, and assert the memory bound and the counter.

### B8. Units, ranges and rounding

**Examples:**
- **radToCdeg.** It mapped -pi to -18000, outside the (-18000, 18000] contract. Fixed and tested (memory project_investigatr_brain_link.md; `math/angles.h:30-33`; `brain_io_gtest.cpp:1075-1080`).
  - The same defect is still latent in actuGATR `telemetry.cpp:41-47` (D3).
- **Wire rounding direction** was chosen on purpose in wB B2: "box sizes round up and the boundary rounds inward".
- **ms resolution versus us guards:** see B1.
- **wB B4 test fix:** "0.5 rad/s counts as turning under the 5 deg/s limit".

**Positive:**
- Units in names are everywhere: `x_mm`, `heading_cdeg`, `window_ms`, `radius_m`.
- investiGATR has `Seconds` and `Radians` aliases.
- `-Wconversion` is clean on the Brain side.

**Rule:**
- SI units in memory (m, rad, s); wire units only at the codec.
- Every conversion goes through one function per unit pair, owned by translaGATR, with its range and rounding policy in its name or doc.
- A unit suffix on every raw numeric field and parameter.
- `using` aliases (`using Meters = double;`) in public signatures.

**Check:**
- **`-Wconversion -Wsign-conversion`:** cheap on the Brain; about 125 hits on naviGATR, 58 of them in one header.
- **Edge-value tests** for conversions: +-pi, 0, and the type limits.
- **Grep gate** for the literal `3.14159` and for `180.0 /` outside the conversion module.

### B9. Name tables and labels drifting from enums or meaning

**Examples:**
- **D1:** four spellings of picoOpName, and a resultName that covers half the values.
- **A1:** a test was needed to check "that ops 3 and 5 and results 6 and 7 are not reused".
- **Brain apps.** `sensorsReady` is a negative list (`r != Readiness::kConnecting && r != Readiness::kReconnecting && ...`, `locaGATR/src/main.cpp:146-151`), so a new Readiness value is treated as ready.
- **W0 fix 3.** "Snapshots refused" also counted replacements.
- **W2 fix 8.** The seq-gap column overstated replacements.

**Rule:**
- Switch on the enum type, list every enumerator, no `default:`, and return after the switch.
- One name function per enum, next to the enum.
- Membership tests are positive lists or switches.
- A report label states exactly what its counter counts.

**Check:**
- **`-Wswitch`** (already in `-Wall`) works only when switching on the enum type. **`-Wswitch-enum`** covers enum class switches that have a default (32 brain hits today). clang-tidy `bugprone-switch-missing-default-case` must stay off.
- **Test pattern:** for dense enums with a count, iterate every value and assert a non-empty, unique name.
- **Grep gate** for `const char\* .*Name(uint8_t` outside translaGATR.

### B10. Retries, idempotency, dedupe and sequence wrap

**Examples:**
- **Design review.**
  - Dedupe compared only against the last request id, so a GET_STATE between SET_POSE retries made the retry "new" and re-applied a placement.
  - request_id wrap was undefined.
  - Stale bytes in the RX buffer could be parsed as a reply.
- **Memory review (medium).** bench_imu op 5 was polled with a new rid between a timed-out SET_POSE and its retry, so the retry was Stale and the placement rejected.
- **wA FIX 1.** A pending CONTROL ended after 5 misses.
- **wA FIX 3.** READ_WHEELS had no ticket, so its result could not be matched to its request.
- **W2 fix 2** (ping ids), **W3 fix 5** (seq mod 256 gaps), **W4 fix 2** (Pico request resend under one id).

**Rule:**
- Every state-changing request has an id, a documented dedupe rule (per session, with serial-number arithmetic for wrap) and a ticket the caller holds.
- Retries resend the same bytes.
- Readers flush stale input before a request.
- Wrap is tested.

**Check:**
- **Test patterns:** a retry after a lost reply applies once; interleaved requests; the id wrapping at its maximum.
- The FakeBus fault injection in `brain/communiGATR/sim` already supports these.

### B11. Liveness: starvation, deadlines checked on one path only, dead bands

**Examples:**
- **W5 fix 1.** Telemetry starved (0 of 100 reports sent) when round trip plus gap reached the state period (`Client::choose`).
- **Commit 4992248.** The motion_step max_wait deadline was checked only when a heading arrived, so no heading meant waiting forever. It is now checked on every call (`motion_step.h:131` expired, `:156` since). The doc diff: "checked on every invocation whether or not headings keep arriving".
- **W1 fix 1.** A large hello left unconfirmed deadlocked a flow-controlled client.
- **Memory review (low):** "a queued placement with no session never times out".
- **Design review (critical).** Control-law dead bands: zones with "no progress and no transition".
- **wA A2.** A starvation guard after 4 due polls.

**Rule:**
- Every wait has a deadline that is checked on every tick, whatever the input.
- Every scheduler states its fairness bound in its doc, and a test asserts it under the worst link timing.
- "Degrade, do not hang": no state-machine state lacks a timeout exit.

**Check:**
- **Test patterns** "no input at all" and "slow link" (FakeBus delays, as in the SlowTelemetry tests in `client_telemetry_gtest.cpp`).
- **Review item:** list each state's exits.

### B12. Silent data loss, and counters nobody reads

**Examples:**
- **wB FIX 1 (high).** The VEX bench dropped 200 to 250 ms gaps silently.
- **wB FIX 2:** "drops_ was unread before" in TrackingWheelMotion::dropWindow.
- **W3 fix 1.** The metadata said `truncated:false` while the size bound had dropped rows. Now it has `truncated_by` (`capture_bundle.h:100-102`).
- **W0 fix 7.** A throttle that was never applied looked like a slow-network result.

**Rule:**
- Every discard is counted where it happens and surfaced: in status, diag and capture metadata.
- A counter with no reader is a defect.
- Every output states its own completeness: truncated, dropped or stale.

**Check:**
- **Test pattern:** an overflow reports itself.
- **Review item.** Compiler warnings catch only fully unused fields.

### B13. Docs and claims drifting from behavior

**Examples:**
- **wD FIX finding 1.** pico_link.md said the pose holds after a Pico reboot, but the sensor-loss rule unplaces it.
- **W4 fixes 2 and 3:** interfaces.md and brain_profile.md.
- **W5 fix 2.** The docs understated the poll delay and overstated the rate limit.
- **wA FIX 5.** communigatr.md claimed a CMake entry that did not exist.
- **viz final_report section 5.** Leftover "common" wording after the rename.

**Rule:**
- Docs state behavior with a pointer to the test that proves it.
- Numbers in docs come from named constants.
- A behavior change updates the PIPELINE.md or doc in the same change.

**Check:**
- **Review item.**
- **CI:** doc references to test names are checked for existence by grep.

### B14. Timing-dependent and non-failing tests

**Examples:**
- **wB B4.** `SlowFieldWorkNeverStallsEstimation` waited on `cycle()`, which counts at cycle start, but asserted finished cycles: 149 vs 150 under load. It now waits on `estimationStats().cycles` with a 3.5 s wall-clock bound (`workers_gtest.cpp:732`).
- **W1.** A close-echo race failed in 1 of 4 suite repeats.
- **W3 open risk.** `capture_gtest.cpp:2025` sleeps 2.5 s of real time so that a 1 Hz reading happens.
- **W2.** The slow browser scenario failed once while 15 compiler processes were running.
- **W0 fix 1.** The perf harness measured whatever process already held the port.
- **W0 negative check.** CMake split a regex at `;`, so two tests checked only a prefix.
- **W3.** A dangling reference to a temporary in a new test let a mutant survive. `-Wdangling-reference` caught it.

**Where real-time waits are:** they concentrate in the naviGATR threaded tests: `workers_gtest.cpp` has 37 sleep or wait sites, `inspection_feed_gtest.cpp` 18, `capture_gtest.cpp` 9, `inspection_gtest.cpp` 8. The Brain suites use fakes with injected time.

**Positive:** every worker ran mutation checks. W3's 10 mutants were all killed, and C2's 23 of 23.

**Rule:**
- Unit tests use injected time and never sleep.
- Threaded tests wait on a condition with a generous bound, and assert the same observable they waited for.
- Every new test is seen failing once (a mutation or negative check) before it is trusted.
- A test harness verifies that it is talking to the process it started.

**Check:**
- **Grep gate** for `sleep_for` in tests, with an allowlist for the threaded suites, which must use condition waits.
- **`ctest --repeat until-fail:3`** in CI for the threaded suites.
- **`-Wdangling-reference`** stays on.

### B15. Platform, ABI and target-only code paths

**Examples:**
- **Commit 3af0e98 ("bug fixes").** JsonWriter overloads for int, long and int64_t collided where int64_t aliases long versus long long. `json_writer.h` now has one integral template: "Fixed-width integers may alias long or long long, depending on the ABI."
- **Pico serial.** `Serial1.availableForWrite()` returns 0 or 1 on this core, so the old firmware never sent frames (memory project_parallel_wheels_bno08x.md; comment at pico `main.cpp:91`).
- **Memory review (high).** `ProsSerialPort::open` called set_baudrate right after serial_enable and got ENODEV, so the link never came up after power-up. Fixed with bounded retries (progress/wA.md A3).
- **Design review.** The PROS template defaulted to gnu++26, which arm gcc 13.3 cannot build.
- **Linux-only code was never compiled on the dev box:**
  - TCP_NOTSENT_LOWAT and SHUT_WR (W1 open risks);
  - the POSIX serial branch (wB B1: "Not compiled here ... no Linux compiler on this box").

**Rule:**
- Every `#if` branch is compiled in CI: a Linux job for the Pi, and ARM syntax checks for the Brain and the Pico.
- Assumptions about vendor APIs are written next to the call, with the source.
- Overloads on fixed-width integers use one template or exact types.

**Check:** a CI matrix (host MinGW, Linux gcc, arm-none-eabi for PROS and the Pico). The reports already hold manual recipes for each.

### B16. Buffer bounds on wire data

**Examples:**
- **A1.** `encodeFieldMapHeader` and `encodeFieldEstimateHeader` accepted a 24-byte buffer for a whole document.
- **A1 note.** The reply decoder does not check a READ_DOC chunk's offset plus length against the document length. The client assembly has to check it, and does (`doc_assembly.cpp:51-63`).
- **Design review.** Implausible length bytes swallowed the frames that followed.

**Rule:**
- Every decoder checks lengths against both the buffer and the declared total.
- Encoders take the destination size.
- Static buffers are bounded by kMaxFrameLen (already the wire rule).

**Check:**
- **Known-byte vector tests** (`frame_codec_gtest.cpp` has 24) plus randomized decode tests.
- **`-fsanitize=address,undefined`** on a Linux host test job. Not used today: a decision.

### B17. The commit history carries no bug information

- All 55 commit subjects are terse: "bug fixes" x2, "janky fix", "new stuff" x10, "usb fix".
- Fixes are mixed with features: 132f78f "janky fix" touches 188 files, including a vendored library.
- The catalog above had to be rebuilt from agent reports.

**Rule:**
- A fix commit names the symptom and the bug class, for example "fix: telemetry starves when RTT + gap >= state period (liveness)".
- A fix carries its regression test.

**Check:** a PR template, or an optional commit-message lint.

## 5. Checks in one table

| Check | Catches | Cost today | Proposal |
|---|---|---|---|
| clang-format `InsertBraces: true` | unbraced ifs: 87 in 11 files, 39 in `composition.cpp`, 28 in `pros_usb_link.cpp` | verified with 22.1.8 on a scratch copy of `pros_usb_link.cpp`: 28 to 0 | add to `.clang-format` |
| `-Wold-style-cast -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference -Wimplicit-fallthrough` | casts, copy-paste branches, fallthrough | 0 hits | adopt now |
| `-Wswitch-enum`, and name functions that take the enum type | B9 | 32 hits in 11 brain switches | adopt |
| `-Wshadow` | shadowed context and locals | 1 brain, 5 naviGATR | adopt |
| `-Wconversion -Wsign-conversion` | B8 | 5 brain; 125 naviGATR, 58 in `core/time.h` | adopt after one pass |
| `-Wdangling-reference` (in `-Wall` on g++ 13+) | B14 test bugs | already on; caught W3's test bug | keep |
| lizard: fail `-C 25 -Tnloc=100 -a 7`, warn `-C 15 -Tnloc=60 -a 5`, plus file length | section 1 tail, C++ and JS | pip install; baseline file for the 60 current offenders | adopt |
| clang-tidy: readability-function-size (nesting), readability-function-cognitive-complexity, bugprone-branch-clone, bugprone-easily-swappable-parameters, bugprone-narrowing-conversions, bugprone-unchecked-optional-access, cppcoreguidelines-pro-type-member-init, bugprone-use-after-move, bugprone-dangling-handle | tail, B2, B4, B8 | clang-tidy not installed | decision |
| clang `-Wthread-safety` with annotations | B6 | needs clang | decision |
| ASan and UBSan on a Linux host test job | B16, lifetime bugs | new CI job | decision |
| Grep gates: clock reads outside core and app; `sleep_for` in unit tests; `\|\| <number>` in JS; literal pi; `const char* ...Name(uint8_t` outside translaGATR | B1, B14, B2, B8, B9 | small scripts | adopt |
| `ctest --repeat until-fail:3` on the threaded suites | B14 | minutes | adopt |
| Standard test patterns: source stops; identity change mid-flow; no input at all; slow link; stalled consumer; retry after a lost reply; id wrap; enum names complete; overflow reports itself | B3, B4, B11, B7, B10, B9, B12 | per component | test checklist in the standard |
| PR template: which clock and unit; capacity, policy and counter; each state's exits; the doc or PIPELINE.md updated; fix commit names the class | B1, B7, B11, B13, B17 | none | adopt |

**Open decision:** requirements.md flags `-Werror` (the memory says it is on everywhere; this session the user said `-Wall -Wextra` without `-Werror`). The probe shows that translaGATR and the Brain would pass a much larger warning set after a small pass. naviGATR needs one `-Wconversion` pass, mostly in `core/time.h`.
