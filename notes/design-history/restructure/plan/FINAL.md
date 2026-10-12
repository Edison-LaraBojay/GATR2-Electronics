# LEAD CORRECTION (2026-09-28 15:05), overrides everything below

Plug-in folder names are readable plural nouns for what the plug-ins ARE, not a mechanical plural of the XML element. No stutter (resources/resources, sensors/sensors), no awkward plurals (publishings).
Authoritative names:
- resources: family folders directly under pipeline/resources/: cameras/, tag_detectors/, pico/, links/, geometry/, synthetic/; register_resources.cpp at stage level;
- sensors: channels/;
- commands: collectors/;
- localization: observation_models/ and estimators/;
- world_estimation: estimators/{apriltag,noop}/;
- targets: resolvers/;
- publishing: publishers/.
The noop pieces follow the same folders (collectors/noop.*, resolvers/noop.*, publishers/noop.*, localization estimators/noop.*).
move_map_final.tsv has been rewritten to these names by restructure/scripts/rename_plugins.py.
Items C1 and O1 below are superseded. Any leftover old name in this file means the name above.

# naviGATR restructure: final plan (authoritative)

Consolidated from `plan/move_map.md` + `move_map.tsv`, `plan/split_plan.md` and `plan/world_estimation_plan.md`, reviewed against `SPEC.md` and the worktree at ae457ee. Where this file and a draft differ, this file wins. SPEC.md still wins over this file.

- `S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`
- `R` = `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/restructure`
- `WT` = `S/wt-navi` (worktree, detached at ae457ee). Review was read-only: `git -C WT status --short` is empty.
- Move map: `R/plan/move_map_final.tsv` (201 rows, same format as the draft). The draft `move_map.tsv` is superseded.

## 1. Corrections

Every change to the drafts, in one list. Section numbers in brackets point at the draft text being corrected.

**C1. Plug-in folders of the three typed slots use the real XML element, not agent nouns** [move_map.md D2].
- SPEC: plug-in folder = the real XML element in plural form. The only element that selects these plug-ins is the slot element itself (`<CommandCollection type>`, `<TargetResolution type>`, `<Publishing type>`; no `<Collector>`, `<Resolver>` or `<Publisher>` exists in config/** or tests/**).
  - `pipeline/commands/collectors/` becomes `pipeline/commands/collectors/`
  - `pipeline/targets/resolvers/` becomes `pipeline/targets/resolvers/`
  - `pipeline/publishing/publishers/` becomes `pipeline/publishing/publishers/`
- 11 TSV rows change (3 + 3 + 5); nothing else in the TSV changes (`diff move_map.tsv move_map_final.tsv`).
- The noop pieces of C2 follow: `collectors/noop.*`, `resolvers/noop.*`, `publishers/noop.*`.
- Function and file names inside are unchanged (`register_commands`, `register_target_resolution`, `register_publishers`); a move renames no symbol.
- `publishers` is the literal plural of `Publishing`. The user may prefer `publishers/`; that would be a recorded SPEC exception (one sed over the TSV). See open item O1.

**C2. The noop split moves from step 1 to step 2 (as step 2a)** [move_map.md D14, "Noop split"].
- SPEC's behavior rule puts "code moved between files" in the split step. Keeping step 1 a pure rename is what lets C8 prove it identical.
- Step 1: `impl/noop/noops.{h,cpp}` git mv to `pipeline/world_estimation/estimators/noop/noop.{h,cpp}` unchanged (TSV row as drafted). Its 7 includers (5 register files, noops.cpp, `tests/system_build_gtest.cpp:12`) point at that path for the length of step 1.
- Step 2a: split into the 5 pieces of move_map.md's noop table, with C1 paths. `tests/system_build_gtest.cpp:12` is dropped there (it uses no noop symbol; confirmed by the trial build).

**C3. `LandmarkError` lives in `pipeline/world_estimation/field_state.h`, no `landmark_error.h`** [world_estimation_plan.md D3, 2, 3.2, 3.3, 8.2].
- SPEC layout: "field state (FieldState, `LandmarkError`)".
- `struct LandmarkError`, `landmarkError(nominal, estimated)` and `applyLandmarkError(error, landmark_nominal, target_nominal)` go into field_state.h, text as in world_estimation_plan.md 3.2, placed before `struct FieldObjectState`.
- field_state.h includes gain `<cmath>`, `<optional>`, `math/angles.h`, `math/units.h`.
- `configured_targets.cpp` and `tests/landmark_error_gtest.cpp` include `pipeline/world_estimation/field_state.h` instead.
- `landmarkError()` stays public beside the struct: it is the definition of the error (producer side) and the round-trip test needs it. The ONE consumer helper is `applyLandmarkError`.
- New files in step 3 are now 3, not 4: `src/math/units.h`, `src/pipeline/world_estimation/estimators/apriltag/tag_association_trace.h`, `tests/landmark_error_gtest.cpp`.

**C4. world_estimation_plan.md path table (section 2) follows C1 and C3.**
- `configured_targets.{h,cpp}`: `src/pipeline/targets/resolvers/`.
- `field_documents.{h,cpp}`: `src/pipeline/publishing/publishers/`.
- Row `(new) src/pipeline/world_estimation/landmark_error.h` is removed.
- Prototype test includes become post-move paths: `pipeline/world_estimation/field_state.h`, `pipeline/resources/target_set.h`, `pipeline/world_estimation/estimators/apriltag/tag_association_trace.h`.

**C5. Every step uses `plan/move_map_final.tsv`** [split_plan.md intro and 7.2; world_estimation_plan.md header and 2].
- `split_system.py --rename R/plan/move_map_final.tsv`. None of C1's paths appear in system.cpp/system.h includes. Checked: a dry run on the ae457ee text and a full trial both succeed (section 7).

**C6. clang-format path** [SPEC "Code style"; split_plan.md D7; world_estimation_plan.md 8].
- `/c/msys64/ucrt64/bin/clang-format` does not exist on this box (checked).
- Use `CF=/c/Users/mbes1/AppData/Roaming/Python/Python314/site-packages/clang_format/data/bin/clang-format.exe` (`--version`: 22.1.8) and `CFD=/c/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/clang-format-diff.exe`.

**C7. One formatting rule for all steps: format the lines you author, never the lines you move** [move_map.md include-order note; split_plan.md D7; world_estimation_plan.md 8].
- The tree is not clang-format clean (split_plan.md D7: 14 of 198 src files clean), so whole-file formatting of a moved file rewrites unrelated lines and breaks `git diff -M --color-moved` review.
- Step 1: changed lines only (C8 step 1b). In practice this only re-sorts the include blocks 1a touched.
- Step 2: no clang-format on moved bodies. `split_system.py` and the noop split emit their boilerplate (labels, includes, namespace lines) already in clang-format order.
- Step 3: whole-file only for wholly new files (`math/units.h`, `tests/landmark_error_gtest.cpp`). Edited files: changed lines only. `tag_association_trace.h`: only its label, include and namespace lines (the structs are moved text).
- **Bug fixed in world_estimation_plan.md 8.8:** `git diff -U0 HEAD -- <post-move paths>` shows every renamed file as wholly new (HEAD has no such path), so clang-format-diff would reformat whole files. Instead, stage steps 1 and 2 first (`git -C WT add -A pi/naviGATR docs`), make the step 3 edits, then diff worktree against index:
  `git -C WT diff -U0 --no-color -- <edited files> | $CFD -p1 -i -binary $CF`

**C8. Step 1 is done in two parts, with a proof for the first** [move_map.md "Include order"].
- 1a: git mv, then rewrite every include string in place (no re-sort), CMake, embed_assets, comment paths, docs.
  - Proof: every translation unit preprocesses to the same text before and after (`R/scripts/pp_compare.py`; normalizes the tree root, `__DATE__`/`__TIME__`, and the mapped paths in `assert`'s `__FILE__` from `time/time.h`).
  - Trial: 109 of 109 naviGATR translation units identical (`R/logs/plan-check.pp_compare.txt`).
- 1b: `git -C WT diff -U0 --no-color -- pi/naviGATR | $CFD -p1 -i -binary $CF`, before staging the 1a edits. This sorts only the touched include blocks; clang-format sorts a whole block when one of its lines is in range and changes nothing else (tested on main.cpp).
  - 1b is the only step-1 edit that changes what the compiler sees (header order). It is checked by build, ctest and the nm symbol set against 1a.

**C9. Symbol-set expectations are exact now** [split_plan.md 8; move_map.md "Build check"].
- Method, as in split_plan.md: `nm -C --defined-only libnavigatr_core.a`, drop the address, sort unique.
- Step 1 vs ae457ee: identical (72413 lines; see section 7).
- Step 2 vs step 1: identical except exactly 2 added lines, both from the noop split (2a):
  `A .weak.__cxa_pure_virtual._ZN8navigatr10Publishing5resetEv` and `A .weak.__cxa_pure_virtual._ZN8navigatr8Commands5resetEv`.
  These are MinGW COFF weak-alias names for the inline `Commands::reset` / `Publishing::reset`, now emitted by single-class translation units. The code sections (`.text$_ZN8navigatr8Commands5resetEv` etc.) are present before and after. The system split (2b) itself adds nothing (72415 lines before and after 2b).

**C10. A text check covers the code no build here compiles** [move_map.md].
- `libcamera_camera.{h,cpp}` and the `#if NAVIGATR_HAVE_LIBCAMERA` include at `cameras.cpp:13` are built neither on this box nor in CI (option OFF).
- `R/scripts/check_final_map.py R/plan/move_map_final.tsv` resolves every quoted include in `pi/naviGATR/{src,tests,tools}` after the move: 0 unresolved. It also checks coverage, destination uniqueness, folder rules and plug-in names, and finds 0 include cycles (`R/logs/final_map_check.txt`, errors: 0).
- The full trial end state (section 7) also passed a resolver pass over its tree: every quoted include resolves, including the libcamera ones.

**C11. Reference line lists for the final map.** `R/logs/move_map_refs.txt` names the draft folders. Use `R/logs/move_map_refs_final.txt` (same 1087-line list with C1's three renames applied; 29 lines carry the new names).
- Doc links that change because of C1: `pi/naviGATR/docs/architecture.md` 373 (resolvers), 374 (collectors), 375-376 (publishers); `pi/naviGATR/docs/landmarks.md` 114, 165 (publishers). Code comment `src/config/field_map.h:202` names `pipeline/publishing/publishers/field_documents.h`.

**C12. Path-length note in move_map.md is replaced.** It computed 241 characters without building. A real build shows CMake switching long object paths to hashed folders (for example `navigatr_core.dir/5e6431fe3c23aa323685b73731229eca/apriltag/apriltag_tag_observation.cpp.obj`); the longest file in the trial build was 252 characters and everything built. No action. Scripts must not assume object paths.

**Considered and not changed** (listed so nobody re-opens them without new facts):
- `pico_control_ref.h` stays at `pipeline/resources/`. It names the concrete `PicoTelemetry`, but its three consumers (brain_link publisher, inspection document, brain profile host) need the PicoControl accessor. Moving it into the plug-in folder would make all three include a plug-in-folder header.
- Sensor payload headers stay in `pipeline/sensors/` (move_map.md D9). They are the SensorMap output types (`sensor.*` payload names). Camera resources construct them directly and the sensor channel forwards them unchanged, which gives 3 resources-to-sensors include edges (section 5).
- `register_world_estimation.cpp` stays in `estimators/`, not `estimators/apriltag/` (D4). It registers the noop too; SPEC's generic rule puts a register file in the plug-in folder it registers.
- `register_localization.cpp` stays at localization stage level (D5). It registers both `<Observation>` and `<Estimator>`; splitting it would need a new function.

## 2. Final layout

```
pi/naviGATR/src/
  main.cpp
  cli/                   build_config.h.in, command_line.{h,cpp}
  core/                  system.h, handoff.h, system.cpp (step 1) -> build.cpp, run.cpp (step 2)
  pipeline/
    register_all.{h,cpp}
    framework/           ids, records, typed_payload, payload_descriptor, function_registry, function_status,
                         execution_context, diagnostics, slot_init, record_bookkeeping, build_options (all .h)
    resources/           contract.h, stage.{h,cpp}, resource_catalog.h, resource_store.{h,cpp}, resource_instance.h,
                         camera.h, tag_detector.h, pico_control.h, pico_control_ref.h, brain_imu_bench.h,
                         robot_frames.h, target_set.h, wheel_geometry.h, pico_telemetry_samples.h
      register_resources.cpp
      cameras/           cameras, camera_capture.h, libcamera_camera
      tag_detectors/     tag_detectors, apriltag_c
      pico/              pico_telemetry
      links/             pros_usb_link, serial_links
      geometry/          field_map_resource, robot_frame_map, target_set_resource, wheel_geometry_resource
      synthetic/         synthetic_rig
    sensors/             contract.h, stage.{h,cpp}, sensor_catalog.h, sensor_samples.h, attitude_samples.h,
                         camera_frames.h
      channels/          <Sensor> plug-ins (attitude_channel, camera_channel, pico_channels), register_sensors.cpp
    commands/            contract.h, command_state.h
      collectors/  brain_link_commands.{h,cpp}, register_commands.cpp, noop.{h,cpp} (2a)
    localization/        contract.h, stage.{h,cpp}, robot_state.h, robot_state_feed.{h,cpp}, pose_history.{h,cpp},
                         attitude.h, stillness.h, robot_observations.h, register_localization.cpp
      observation_models/ attitude_reference, brain_imu_wheel_bench, imu_heading_increment, tracking_wheel_motion,
                         stationary_window (.h and .cpp each)
      estimators/        planar_motion_integrator, weighted_planar_fusion, motion_step (.h and .cpp each),
                         noop.{h,cpp} (2a)
    world_estimation/    contract.h, stage.{h,cpp}, perception.h, association.h, evidence.h,
                         field_state.h (+ LandmarkError, step 3), landmark_estimator.{h,cpp}
      estimators/        register_world_estimation.cpp
        apriltag/        apriltag_world_estimator, apriltag_tag_observation, tag_mount_association (.h and .cpp),
                         tag_observations.h, tag_association_trace.h (step 3)
        noop/            noop.{h,cpp} (git mv target of noops.*; one factory after 2a)
    targets/             contract.h, target_state.h
      resolvers/  configured_targets.{h,cpp}, register_target_resolution.cpp, noop.{h,cpp} (2a)
    publishing/          contract.h
      publishers/       brain_link_publisher.{h,cpp}, field_documents.{h,cpp}, register_publishers.cpp,
                         noop.{h,cpp} (2a)
  brain_profile/         brain_profile.h, brain_profile_builder.{h,cpp}, sensor_loss.{h,cpp},
                         stationary_precheck.{h,cpp}; step 2 adds profile_host.{h,cpp}, control.cpp,
                         pico_operation.{h,cpp}, recovery.cpp
  config/                unchanged (7 files)
  io/                    byte_stream.h, file_stream.{h,cpp}, half_duplex.{h,cpp}, serial_port.{h,cpp},
                         serial_link.h, spi_bus.h
  time/                  time.h, host_clock.h, clock_sync.h
  math/                  unchanged (5 files); step 3 adds units.h
  debug/
    inspection/          old inspection/ (11) + inspection_config.h, inspection_state.h; step 2 adds inspection_state.cpp
    capture/             old capture/ (10)
    diagnostics/         old diagnostics/ (7); step 2 adds events.{h,cpp}
pi/naviGATR/tools/field_refs/field_refs.cpp
```

Checked against the worktree (`check_final_map.py`, `R/logs/final_map_check.txt`):
- 201 files under `pi/naviGATR/src` (`find` and `git ls-files` agree). All 201 are mapped exactly once: 189 move, 12 stay (`config/` 7, `math/` 5). None is missing, none is listed twice, and no destination is used twice.
- No folder named `impl`. None of `app/ contracts/ payloads/ state/ runtime/ impl/ resources/ transport/ capture/ diagnostics/ inspection/` survives at top level. Old `core/` keeps only engine files.
- Every folder below a stage is a plug-in folder named after its XML element: `<Resource>` resources, `<Sensor>` sensors, `<CommandCollection>` collectors, `<Observation>` observations, `<Estimator>` estimators (localization and world_estimation), `<TargetResolution>` resolvers, `<Publishing>` publishers. `world_estimation/estimators/{apriltag,noop}/` are the SPEC sensor-group folders.

## 3. Draft decisions: adopted or overridden

| Draft decision | Status |
|---|---|
| move_map D1 (`resources/resources/`, `sensors/sensors/`) | adopted (literal SPEC rule) |
| move_map D2 (agent nouns) | overridden by C1 |
| move_map D3 (flat plug-in folders) | adopted |
| move_map D4, D5 (register file placement) | adopted (see "Considered") |
| move_map D6 (`register_all.*` in `pipeline/`) | adopted |
| move_map D7 (`build_options.h` in framework) | adopted |
| move_map D8 (`serial_link.h`, `spi_bus.h` in `io/`) | adopted |
| move_map D9 (sensor payloads in `pipeline/sensors/`) | adopted (see "Considered") |
| move_map D10 (`inspection_state.h`, `inspection_config.h` in `debug/inspection/`) | adopted |
| move_map D11 (shared plug-in helpers in their plug-in folder) | adopted |
| move_map D12 (`field_documents.*` with the brain_link publisher) | adopted, folder per C1 |
| move_map D13 (`navigatr_field_refs` stays a top-level target) | adopted |
| move_map D14 (noop split in step 1) | overridden by C2 (step 2a) |
| move_map D15 (`slot_init.h` in framework) | adopted |
| move_map D16 (src-relative line-1 labels for shared basenames and the 7 payload headers) | adopted |
| split D1-D6 (destinations) | adopted |
| split D7 (formatting) | replaced by C7 |
| WE D1, D2, D4, D6, D7, D8 | adopted |
| WE D3 (`landmark_error.h`) | overridden by C3 |
| WE D5 (wire code unchanged) | adopted; user sign-off O3 |
| WE D9 (policy-none target within 7.11e-15 m) | adopted; user sign-off O2 |

## 4. Execution

Rules from SPEC hold: edits only in `WT`; `git mv`/`git add` only (index), never commit, push, stash, reset or checkout; builds only in `S/wt-build/<key>/` with the SPEC recipe; ucrt64 first on PATH for test executables. Plain ASCII in everything written.

**Step 0. Baseline.** Build `WT` at ae457ee. Save the `ctest -N` names (626) and the nm symbol set (method in C9; 72413 lines). Also extract a baseline tree for C8's proof: `git -C WT archive ae457ee pi/naviGATR translaGATR brain | tar -x -C S/wt-build/<key>/base-tree`, and configure it with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.

**Step 1a. Move (pure rename).**
1. For each TSV row whose target is not `same`: `mkdir -p` the target folder, then `git -C WT mv pi/naviGATR/<old> pi/naviGATR/<new>`.
2. Rewrite include strings in place in `pi/naviGATR/{src,tests,tools}`, with no re-sort: 744 lines in 206 files. Every line is listed in `R/logs/move_map_refs_final.txt`.
3. CMake:
   - `pi/naviGATR/CMakeLists.txt`: `NAVIGATR_SOURCES` 64 paths; line 193 (libcamera); 216 (`src/cli/build_config.h.in`); 217 (`src/main.cpp src/cli/command_line.cpp`); 227 (`tools/field_refs/field_refs.cpp`).
   - `tests/CMakeLists.txt:21` (`../src/cli/command_line.cpp`).
   - `cmake/embed_assets.cmake:36` (`debug/inspection/viewer_assets.h`).
   - `tools/perf/CMakeLists.txt` and `.github/workflows/ci.yaml`: no change.
4. Comment paths: `config/field_map.h:202`, `inspection_service.h:33`, `register_all.h:3`, `stillness.h:5`. Line-1 labels of 24 files per D16: 7 contracts, 8 stage files, 2 noops, 7 payload headers.
5. Docs: the lines in move_map.md "Docs", with C11's changes.
6. Proof: configure `WT` with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` into `S/wt-build/<key>-1a`, then run
   `py -3 R/scripts/pp_compare.py <base build> <1a build> R/plan/move_map_final.tsv <out dir>`.
   Must print `identical: 109, different: 0, unmatched: 0`.
7. Build and ctest. Names must equal the baseline, and nm must be identical to the baseline.
8. `py -3 R/scripts/check_final_map.py R/plan/move_map_final.tsv` must report `errors: 0`. It reads `WT`'s current `src`, so run it before step 1a, or on a fresh archive.

**Step 1b. Sort the touched include blocks.** Do not stage the 1a edits yet (git mv staged only the renames).
- `git -C WT diff -U0 --no-color -- pi/naviGATR | $CFD -p1 -i -binary $CF`
- Build, ctest, nm: all identical to 1a. A header that is not self-contained fails to compile here; fix it by adding its missing include (logic-free).
- Stage: `git -C WT add -A pi/naviGATR docs`.

**Step 2a. Noop split** (move_map.md "Noop split" table, paths per C1).
- Each piece keeps its factory declaration, class and factory definition verbatim.
- CMake: `src/pipeline/world_estimation/estimators/noop/noop.cpp` becomes the 5 `noop.cpp` lines.
- Register files include their own piece; `tests/system_build_gtest.cpp:12` is dropped.

**Step 2b. System split** (split_plan.md 7, with C5 and C7).
- `py -3 R/scripts/split_system.py --src WT/pi/naviGATR/src --rename R/plan/move_map_final.tsv --report R/logs/split_step2_report.txt`
- CMake: 8 `.cpp` lines replace `src/core/system.cpp`.
- Docs: `architecture.md:366` points to `core/build.cpp` and `core/run.cpp`; `third_party/README.md:54` names `core/build.cpp` and `brain_profile/profile_host.cpp`.
- Verify:
  - 0 warnings;
  - ctest names equal step 1;
  - nm vs step 1 per C9 (only the 2 weak-alias lines added);
  - `split_system.py --verify <dir with the step-1 system.h/.cpp>` passes.
- Stage: `git -C WT add -A pi/naviGATR docs`.

**Step 3. World estimation** (world_estimation_plan.md 8, with C3, C4, C7).
- Record `ctest -N` before.
- Edits, in order:
  - add `math/units.h`;
  - `field_state.h`: the C3 block and the two `FieldObjectState` members;
  - `landmark_estimator.cpp`: `setFieldPose` at the 4 sites;
  - `configured_targets.{h,cpp}`: `estimateTargetPose`;
  - `inspection_document.cpp`: section 4.6;
  - the tag trace split (section 6);
  - the 8 tests, and `landmark_error_gtest.cpp` in `TEST_SOURCES` after `world_estimation_gtest.cpp`.
- Format per C7, then `git add` the 3 new files and the edits.
- Expect 634 tests: exactly the 8 names of world_estimation_plan.md 7.2 added, 0 removed, all passing.

**Docs step** (PIPELINE.md files) is separate and not planned here. Its inputs are split_plan.md 5 (the system.h header comment) and world_estimation_plan.md 10.5, plus SPEC "The purpose".

## 5. Layering after the move (known, unchanged by a pure move)

From `check_final_map.py` on the final map (includer -> included). None is a file-level cycle (0 cycles).
- Framework -> stage: `pipeline/framework/slot_init.h` -> `pipeline/localization/contract.h`.
- Earlier stage -> later stage: `resources/cameras/cameras.cpp` -> `sensors/camera_frames.h`; `resources/synthetic/synthetic_rig.cpp` -> `sensors/camera_frames.h`, `sensors/attitude_samples.h`.
- Stage level -> own plug-in folder:
  - `resources/pico_control_ref.h` -> `resources/pico/pico_telemetry.h`;
  - `resources/stage.cpp` -> `resources/links/serial_links.h` (replay link);
  - `localization/register_localization.cpp` -> 6 plug-in headers.
- Plug-in -> other stage's plug-in: `publishers/brain_link_publisher.cpp` -> `collectors/brain_link_commands.h`.
- math/config -> pipeline: `math/transforms.h`, `config/field_map.h` -> `pipeline/framework/ids.h`; `math/camera_model.h` -> `pipeline/resources/camera.h`.
- pipeline -> brain_profile (2: brain_link commands and publisher headers). pipeline -> debug/diagnostics/hub.h (5).
- debug -> core/system.h (5). debug -> plug-in folders (3): `instrumentation.cpp` -> `pico_telemetry.h`, `inspection_document.cpp` -> `tag_detectors.h`, `inspection_state.h` -> `estimators/apriltag/tag_observations.h`.
- Step 2 adds brain_profile -> core (`profile_host.h`) and debug -> core (`events.cpp`, `inspection_state.cpp`), since these files hold `System` members. Folder-level core<->debug and core<->brain_profile loops exist already (`capture_service.cpp` includes `runtime/system.h` at ae457ee). Split follow-ups F1, F5 address them.

## 6. Open items for the user

- **O1.** `publishers/` (C1) is the literal plural of `<Publishing>`. `publishers/` reads better but would be a SPEC exception.
- **O2.** Step 3's only non-bit-identical value is the policy-none latched target: within 7.11e-15 m and 1.94e-15 rad (world_estimation_plan.md 5.3, 9.1).
  - SPEC asks for both "numerically identical" and "use the helper in target resolution"; no formulation gives both bit-exact.
  - Test outcomes are unchanged. A Brain wire target rounds to mm/cdeg, so the result could differ only if the value sat within about 1e-14 of a rounding boundary.
- **O3.** Brain wire code is unchanged (WE D5). "Full poses computed as nominal plus error" holds by definition (error := pose - nominal); recomputing the wire from nominal + error would change test outcomes (FieldEstimate.* in field_documents_gtest) and is not bit-exact.
- **O4.** SPEC's "system.h thin facade" is not reached by step 2: system.h is 489 lines, one class, all members declared. It needs split_plan.md F1 (Brain profile as its own object), which is a follow-up needing approval.
- **O5.** `main.cpp` stays 195 lines (parse, build, inspection start, run loop, stats printing). SPEC calls it a "tiny entry"; no plan splits it. A follow-up could move the printing helpers into `cli/`.
- **O6.** `.clang-format` vs tree style (split_plan.md F9): the config's `class X {` / 1-space trailing comment style differs from the tree. It is the user's call, and whole-tree formatting is a separate commit.

## 7. Trial evidence (scratch copies of ae457ee; `WT` never modified)

Build dirs are under `S/wt-build/plan-check/`. Tools: `R/scripts/apply_move_trial.py` (step 1 + 2a on a scratch tree), `pp_compare.py`, `check_final_map.py`, `we_fold_trial.py` (C3 on top of `we_apply_proto.py`).

| Trial | Result | Logs |
|---|---|---|
| 1a in place, no noop split (`pp/moved-tree`) | 744 include lines rewritten; 109/109 translation units preprocess identically; build exit 0, 0 warnings; 626/626 passed; names identical; nm identical to baseline (72413 lines) | `plan-check.pp_compare.txt`, `plan-check.move1a.*` |
| 1a + 2a (`move-trial`) | build exit 0, 0 warnings; 626/626 passed; names identical to baseline; nm 72415 lines = baseline + the 2 weak-alias lines | `plan-check.move.*`, `plan-check.nm.base.txt`, `plan-check.nm.move_noop.txt` |
| + 2b split with the final map | build exit 0, 0 warnings; 626/626; names identical; nm identical to 1a+2a (72415) | `plan-check.split.*`, `plan-check.nm.split.txt` |
| split on include-sorted system.{h,cpp} (1b state) | `split_system.py` succeeds (anchors hold) | none (dry run) |
| full end state: WE prototype + C3 fold, then 1a + 2a + 2b | build exit 0, 0 warnings; 634/634 passed; names = baseline + exactly the 8 new, 0 removed; every include resolves (libcamera included) | `plan-check.full.*` |
| final map check | 201/201, errors 0, 0 include cycles, 985 quoted includes scanned | `final_map_check.txt` |
