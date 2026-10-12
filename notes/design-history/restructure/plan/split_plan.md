# Step 2 plan: split System (system.cpp 1450 lines, system.h 479 lines)

Authority: `restructure/SPEC.md`. This is the plan for step 2 (the split). Step 2 changes no logic: definitions and declarations move between files. There are no signature changes and `System` stays one class.

- Line numbers below are from ae457ee: `pi/naviGATR/src/runtime/system.cpp` and `system.h`.
- Step 1 moves them to `core/system.cpp` and `core/system.h`. Their text is unchanged apart from include lines, so the anchors still hold.
- **Checked by a trial on a scratch copy** (the worktree was never modified; `git status --short` was empty before and after):
  - builds with 0 warnings;
  - all 626 baseline tests pass, plus the 9 perf tests;
  - the core library's defined-symbol set is identical (72413 entries).
  - Details are in section 8 and `restructure/logs/split_plan_trial.log`.
- **Tool:** `restructure/scripts/split_system.py` does the split and proves that every original line lands in exactly one place.
- **Preview:** the exact generated files are in `restructure/plan/split_preview/`. They are generated on the ae457ee layout, so their includes still say `runtime/...`; step 2 renames them from the step 1 move map.
- **Checked against the draft move map** `restructure/plan/move_map.tsv` (201 lines, as found at the time of writing). Every path this plan assumes is in it:
  - `runtime/system.{h,cpp}` to `core/`;
  - `runtime/inspection_state.h` to `debug/inspection/`;
  - `diagnostics/*` to `debug/diagnostics/`;
  - `core/host_clock.h` and `core/time.h` to `time/`;
  - `resources/pico_control.h` and `runtime/pico_control_ref.h` to `pipeline/resources/`;
  - `contracts/brain_profile.h` to `brain_profile/brain_profile.h`, which does not clash with the new file names.
- A split run on the ae457ee text with `--rename` pointed at that map rewrote every generated include, and system.h's own include block, to existing target paths.

## 1. Result

| File | Lines | Holds |
|---|---|---|
| `core/system.h` | 489 | the class, regrouped into labeled sections (section 5) |
| `core/build.cpp` | 300 | `System()`, `~System()`, `buildFromFile`, `buildFromString`, `build` (calls `make_*` in order) |
| `core/run.cpp` | 370 | `step`, the three cycles, `requestsFrom`, start/stop/reset, the two workers, pause/resume, snapshot readers, binding view |
| `brain_profile/profile_host.h` | 41 | `System::ProfileHost` (the BrainProfileHost adapter), moved verbatim |
| `brain_profile/profile_host.cpp` | 229 | profile host build, waiting localization, prepare, apply, swap, swap to waiting, `profileBinding` |
| `brain_profile/control.cpp` | 141 | CONTROL (`controlProfile`) and READ_WHEELS (`readWheels`) |
| `brain_profile/pico_operation.h` | 23 | `struct PicoOperation` (from system.h) |
| `brain_profile/pico_operation.cpp` | 95 | `submitPico`, `controlProgress`, `updatePicoOperation` |
| `brain_profile/recovery.cpp` | 137 | `watchSensors`, `watchModels`, `losePlacement`, `noteRecovery` |
| `debug/diagnostics/events.h` | 19 | `struct RuntimeEvent` (from system.h) |
| `debug/diagnostics/events.cpp` | 51 | `noteEvent`, `noteFault`, `events` |
| `debug/inspection/inspection_state.cpp` | 184 | `publishSourceHealth`, `publishDetectionFrames`, `sourceHealth`, `detectionFrames` |

- No file exceeds the 600-line warn threshold proposed in `coding-standard/survey/complexity-bugs.md` 1.6.
- `System::build` itself is still 204 lines (see follow-up F3).
- CMake: the one `system.cpp` line in `NAVIGATR_SOURCES` becomes the 8 `.cpp` lines above.

## 2. Every definition in system.cpp and where it goes

system.cpp has 62 segments:
- 48 `System::` member definitions;
- 6 file-local helpers (anonymous namespace);
- 1 nested class;
- 4 section banners;
- 3 structural pieces (includes and namespace lines), regenerated per file.

Ranges run from the first line, including the leading comment, to the closing brace.

| Lines | Definition | Kind | Destination |
|---|---|---|---|
| 1-27 | header label, includes, `namespace navigatr {`, `namespace {` | structural | regenerated per file |
| 29-33 | `Tag<T>` | helper | core/build.cpp (anon ns) |
| 35-51 | `checkChildren` | helper | core/build.cpp (anon ns) |
| 53-61 | `newSessionId` | helper | core/build.cpp (anon ns) |
| 63-66 | `elapsedMs` | helper | core/run.cpp (anon ns) |
| 68 | `kEventsKept` | helper | debug/diagnostics/events.cpp (anon ns) |
| 70-74 | `hexId` | helper | brain_profile/profile_host.cpp (anon ns) |
| 76 | `} // namespace` | structural | regenerated |
| 78-108 | `class System::ProfileHost` | nested class | brain_profile/profile_host.h |
| 110 | `System::System()` | member | core/build.cpp |
| 112-126 | `buildFromFile` | member | core/build.cpp |
| 128-137 | `buildFromString` | member | core/build.cpp |
| 139-146 | `~System` | member | core/build.cpp |
| 148-351 | `build` | member | core/build.cpp |
| 353-395 | `buildProfileHost` | member | brain_profile/profile_host.cpp |
| 397-402 | `buildWaitingLocalization` | member | brain_profile/profile_host.cpp |
| 404-417 | `requestsFrom` | member | core/run.cpp |
| 419 | `// ---- Brain profile ----` | banner | dropped (the folder says it) |
| 421-473 | `prepareProfile` | member | brain_profile/profile_host.cpp |
| 475-552 | `controlProfile` | member | brain_profile/control.cpp |
| 554-574 | `submitPico` | member | brain_profile/pico_operation.cpp |
| 576-584 | `controlProgress` | member | brain_profile/pico_operation.cpp |
| 586-637 | `updatePicoOperation` | member | brain_profile/pico_operation.cpp |
| 639-685 | `readWheels` | member | brain_profile/control.cpp |
| 687-706 | `watchSensors` | member | brain_profile/recovery.cpp |
| 708-715 | `watchModels` | member | brain_profile/recovery.cpp |
| 717-733 | `losePlacement` | member | brain_profile/recovery.cpp |
| 735-814 | `noteRecovery` | member | brain_profile/recovery.cpp |
| 816-825 | `applyPendingProfile` | member | brain_profile/profile_host.cpp |
| 827-883 | `swapProfile` | member | brain_profile/profile_host.cpp |
| 885-908 | `swapToWaiting` | member | brain_profile/profile_host.cpp |
| 910-926 | `pauseWorkers` | member | core/run.cpp |
| 928-936 | `resumeWorkers` | member | core/run.cpp |
| 938-941 | `profileBinding` | member | brain_profile/profile_host.cpp |
| 943-954 | `publishBindingView` | member | core/run.cpp |
| 956-959 | `bindingView` | member | core/run.cpp |
| 961-978 | `noteEvent` | member | debug/diagnostics/events.cpp |
| 980-985 | `noteFault` | member | debug/diagnostics/events.cpp |
| 987-990 | `events` | member | debug/diagnostics/events.cpp |
| 992 | `// ---- snapshot readers ----` | banner | core/run.cpp (kept) |
| 994-997 | `fieldSnapshot` | member | core/run.cpp |
| 999-1002 | `reportingSnapshot` | member | core/run.cpp |
| 1004-1007 | `sourceHealth` | member | debug/inspection/inspection_state.cpp |
| 1009-1012 | `diagnosticsSnapshot` | member | core/run.cpp |
| 1014-1018 | `detectionFrames` | member | debug/inspection/inspection_state.cpp |
| 1020 | `// ---- cycles ----` | banner | core/run.cpp (kept) |
| 1022-1031 | `step` | member | core/run.cpp |
| 1033-1064 | `estimationCycle` | member | core/run.cpp |
| 1066-1104 | `reportingCycle` | member | core/run.cpp |
| 1106-1140 | `fieldCycle` | member | core/run.cpp |
| 1142-1195 | `publishSourceHealth` | member | debug/inspection/inspection_state.cpp |
| 1197-1302 | `publishDetectionFrames` | member | debug/inspection/inspection_state.cpp |
| 1304 | `// ---- lifecycle ----` | banner | core/run.cpp (kept) |
| 1306-1330 | `resetStages` | member | core/run.cpp |
| 1332-1347 | `reset` | member | core/run.cpp |
| 1349-1352 | `start` | member | core/run.cpp |
| 1354-1382 | `startWorkers` | member | core/run.cpp |
| 1384-1391 | `requestStop` | member | core/run.cpp |
| 1393-1409 | `stop` | member | core/run.cpp |
| 1411-1434 | `estimationWorker` | member | core/run.cpp |
| 1436-1448 | `fieldWorker` | member | core/run.cpp |
| 1450 | `} // namespace navigatr` | structural | regenerated |

**Order inside the new files.** Bodies are byte-identical; only their order within a file is chosen.
- `core/build.cpp`: helpers, then `System()`, `buildFromFile`, `buildFromString`, `~System`, `build` (the original order).
- `core/run.cpp`, top down:
  - `step`, `estimationCycle`, `requestsFrom`, `reportingCycle`, `fieldCycle` (under the kept "cycles" banner);
  - `resetStages`, `reset`, `start`, `startWorkers`, `requestStop`, `stop`, `estimationWorker`, `fieldWorker`, `pauseWorkers`, `resumeWorkers` (under "lifecycle");
  - `fieldSnapshot`, `reportingSnapshot`, `diagnosticsSnapshot`, `publishBindingView`, `bindingView` (under "snapshot readers").

  `step()` is the first function after the helper.
- `brain_profile/profile_host.cpp`: `buildProfileHost`, `buildWaitingLocalization`, `prepareProfile`, `applyPendingProfile`, `swapProfile`, `swapToWaiting`, `profileBinding`.
- Every other file keeps the original relative order.

## 3. Shared helpers and the one internal header

**File-local helpers.** No helper is shared. Each is used by functions in exactly one destination file, so each stays in that file's anonymous namespace (internal linkage, as today). No helper header is needed.

| Helper | Used at (ae457ee lines) | Home |
|---|---|---|
| `Tag` | 276, 330, 335 (`build`) | core/build.cpp |
| `checkChildren` | 167, 214 (`build`) | core/build.cpp |
| `newSessionId` | 150 (`build`) | core/build.cpp |
| `elapsedMs` | 1424, 1444 (workers) | core/run.cpp |
| `kEventsKept` | 975 (`noteEvent`) | debug/diagnostics/events.cpp |
| `hexId` | 424, 457, 471 (`prepareProfile`); 840, 879, 882 (`swapProfile`) | brain_profile/profile_host.cpp |

If a later change put `prepareProfile` and `swapProfile` in different files, `hexId` would go into a `brain_profile/profile_internal.h` inside an unnamed namespace. That keeps internal linkage and the same symbol set. This plan does not need it.

**`System::ProfileHost` needs `brain_profile/profile_host.h`.** This is a pure move of lines 78-108 into a header, required by the compiler:
- `profile_host_` is a `std::unique_ptr<ProfileHost>`, and the class is only forward-declared in system.h.
- Probe: `core/build.cpp` without that include fails (`restructure/logs/split_plan_trial.log` item 4):
  - `System::System() = default` instantiates `~unique_ptr<ProfileHost>` for member cleanup: "invalid application of 'sizeof' to incomplete type".
  - `build()` converts `ProfileHost*` to `BrainProfileHost*` at `slot_context.brain_profile = profile_host_.get()`.
  - `~System` needs the complete type for the same reason.
- So `System()`, `~System()` and `build()` share one TU (`core/build.cpp`). It includes the header with the label `// complete ProfileHost: ctor, dtor, build`.
- The only other includer is `brain_profile/profile_host.cpp`, which constructs the object.
- The class's inline virtuals and vtable are weak symbols, so the defined-symbol set is unchanged (verified).

## 4. Includes per new file

Each new file includes its own header first, then `<std>` headers, then project headers sorted (clang-format order). Only headers its moved bodies use are included, all taken from system.cpp's list. The names below are ae457ee paths; step 2 rewrites them with step 1's move map (`--rename`).

| File | Own header | std | project |
|---|---|---|---|
| core/build.cpp | runtime/system.h | chrono, cstdio, random, set | brain_profile/profile_host.h, config/composition.h, config/config_node.h, tinyxml2/tinyxml2.h |
| core/run.cpp | runtime/system.h | array, chrono, stdexcept | core/host_clock.h |
| brain_profile/profile_host.h | (pragma once) | | runtime/system.h |
| brain_profile/profile_host.cpp | brain_profile/profile_host.h | cstdio | config/config_node.h, runtime/pico_control_ref.h, tinyxml2/tinyxml2.h |
| brain_profile/control.cpp | runtime/system.h | algorithm, cmath | payloads/pico_telemetry_samples.h, payloads/sensor_samples.h |
| brain_profile/pico_operation.h | (pragma once) | cstdint | core/time.h |
| brain_profile/pico_operation.cpp | brain_profile/pico_operation.h | | runtime/system.h |
| brain_profile/recovery.cpp | runtime/system.h | | |
| debug/diagnostics/events.h | (pragma once) | cstdint, string | core/time.h |
| debug/diagnostics/events.cpp | debug/diagnostics/events.h | algorithm, cstring | runtime/system.h |
| debug/inspection/inspection_state.cpp | runtime/inspection_state.h | | core/host_clock.h, runtime/system.h |

- system.h keeps all 31 project includes and adds `brain_profile/pico_operation.h` and `debug/diagnostics/events.h`, sorted into the block. None is removed: includers may rely on the transitive includes.
- No includer of system.h changes. There are 31 of them: `src` 6, `tests` 24, `tools/perf` 1.

## 5. system.h

**Namespace-scope types:**
- `FieldSnapshot`, `ReportingSnapshot` and `BindingView` stay. They are the facade's snapshot types, returned by its readers.
- `PicoOperation` moves to `brain_profile/pico_operation.h`. Its producer is the CONTROL 3/4 code, and it carries Brain wire codes.
- `RuntimeEvent` moves to `debug/diagnostics/events.h`, next to `noteEvent`.
- system.h includes both new headers, so every includer still sees the same names. Nothing is renamed; the namespace stays `navigatr`.

**Nested private types stay nested:** `class ProfileHost;`, `ProfileCandidate`, `RecoverySeen`, `FieldWork`. Moving one out would turn `System::X` into `navigatr::X` and drop its private access, which is a declaration change.

**Data members are not moved or reordered** (104 lines, identical). Their order is initialization and destruction order ("declared first, destroyed last", with `capture_` set last and stopped first).

**Member function declarations are regrouped by concern.**
- Their text is verbatim, with their own comments; access is unchanged.
- The labels are new comment lines, each followed by a blank line.
- Three original group comments stay on the groups they describe:
  - "BrainProfileHost, on the estimation worker.";
  - "Pico commands and recovery bookkeeping, on the estimation worker.";
  - "Boundary work: workers stopped or inline." (now over `swapProfile`/`swapToWaiting` only).

```
FieldSnapshot, ReportingSnapshot, BindingView
class System
public:
  // Build              buildFromFile, buildFromString, ~System
  // Run                step, estimationCycle, fieldCycle, reportingCycle, start, stop, running, reset
  // Brain profile      kPicoOperationMs, kWheelFreshMs, applyPendingProfile, profileBinding, picoControl
  // Configuration and identity
                        loopRateHz, cycle, commandsType, configurationId/Name/Digest, sessionId,
                        resetCount, inspection, warnings
  // Snapshots          robotFeed, bindingView, fieldSnapshot, reportingSnapshot, sourceHealth,
                        diagnosticsSnapshot, detectionFrames, estimationStats, fieldStats, events
  // Retained state     resourceMap, sensorMap, robot, field, command, target, diagnostics,
                        fieldDiagnostics, localization, worldEstimation, resources,
                        resourceCatalog, sensorCatalog
  // Debug              diagHub, capture, captureConfig
private:
  class ProfileHost; ProfileCandidate; System();
  // Build              build                                     -> core/build.cpp
  // Run                requestsFrom, resetStages, startWorkers, requestStop, estimationWorker,
                        fieldWorker, pauseWorkers, resumeWorkers, publishBindingView -> core/run.cpp
  // Brain profile      buildProfileHost, buildWaitingLocalization, prepareProfile, controlProfile,
                        controlProgress, readWheels, swapProfile, swapToWaiting, submitPico,
                        updatePicoOperation, watchSensors, watchModels, losePlacement,
                        noteRecovery                              -> brain_profile/*.cpp
  // Events and capture faults    noteEvent, noteFault           -> debug/diagnostics/events.cpp
  // Inspection snapshots         publishSourceHealth, publishDetectionFrames
                                                                  -> debug/inspection/inspection_state.cpp
  data members (unchanged)
```

Changes in the public section:
- `fieldDiagnostics()` moves next to `diagnostics()`; before, it sat inside the capture group.
- `events()` moves into Snapshots.
- The two constants move into Brain profile.

Sizes after the split:
- public section 146 lines;
- private function declarations 66 lines;
- data members 104 lines;
- header total 489 lines (was 479).

Two things stay as they are:
- **The 83-line header comment is untouched.** The docs step turns it into PIPELINE.md text. Its "A Brain-profiled configuration" and "Recovery" paragraphs belong in `brain_profile/PIPELINE.md`.
- **The header gets no thinner than this in a pure move.** Every member must still be declared in the one class. F1 is what makes it thin.

## 6. Decisions for the lead and the user

Each item notes where this plan departs from the task text. All of them are one-line changes in `split_system.py` (a destination constant), so switching costs nothing.

**D1. Pico control goes to `brain_profile/pico_operation.{h,cpp}`, not `pipeline/resources/`.**
- `submitPico`, `controlProgress` and `updatePicoOperation` are `System` members. They touch `pico_op_`, `noteEvent`, `losePlacement` and `execute_localization_.recalibrate()`.
- Put in a stage folder, they would make `pipeline/resources/` include `core/system.h`. A stage would then depend on the engine, the opposite of SPEC's direction: core builds stages, and stages never see System.
- They answer Brain CONTROL requests through BrainProfileHost, and `PicoOperation` holds Brain result codes. SPEC puts the profile host in `brain_profile/`.
- The Pico-resource side does go with the Pico resource in step 1's move map. That is the `PicoControl` contract (`resources/pico_control.h`) and `runtime/pico_control_ref.h` (`requirePicoControl`/`parsePicoReference`, which name the concrete `PicoTelemetry`). The draft `move_map.tsv` sends both to `pipeline/resources/`.
- `pico_operation.{h,cpp}` is kept as its own unit so that F2 can later move it next to the Pico resource with a `git mv`.

**D2. `noteRecovery` goes to `brain_profile/recovery.cpp`.**
- This follows SPEC's "sensor loss and recovery". Its state `RecoverySeen` is also cleared by the profile swap code.
- Caveat: it runs every cycle for every System, not only Brain-profiled ones (Brain link quiet/resumed, Pico identity, IMU calibration events), and it only emits events and faults.
- Alternative: `debug/diagnostics/events.cpp`. Recommendation: keep it in recovery.cpp, since debug/ holds the event channel and the domain decides what to note.

**D3. The inspection publishers go to `debug/inspection/inspection_state.cpp`.**
- `publishSourceHealth`, `publishDetectionFrames` and their readers `sourceHealth()` and `detectionFrames()` fill the types in `inspection_state.h`, so they go next to it.
- The draft move map puts `inspection_state.h` in `debug/inspection/`. If the final map puts it elsewhere, the .cpp goes next to it.
- Alternative: `core/run.cpp`. That would make run.cpp about 540 lines and put AprilTag types (`TagObservationSet`, `TagAssociationTraceSet`) into core, against "the engine and nothing else".

**D4. Events and capture faults go to `debug/diagnostics/events.{h,cpp}`.**
- `noteEvent` posts `DiagKind::kEvent` to the DiagnosticsHub; `noteFault` forwards to the capture recorder.
- Capture plumbing that is inline in other bodies stays where it is, since moving it would change a function:
  - `capture_ = makeCaptureRecorder(...)` in `build`;
  - `capture_->noteReset` in `reset`;
  - `capture_->stop()` in `~System`.

**D5. `publishBindingView` and `bindingView` go to `core/run.cpp`.** They are written at build and at every profile boundary, and read like the other facade snapshots.

**D6. `System()` and `~System()` go to `core/build.cpp`.** They need the complete `ProfileHost` (section 3), and one include serves both of them plus `build()`.

**D7. Formatting (needs a lead decision; SPEC says to format touched files).**
- The SPEC path `/c/msys64/ucrt64/bin/clang-format` does not exist on this box. clang-format 22.1.8 is at `C:/Users/mbes1/AppData/Roaming/Python/Python314/site-packages/clang_format/data/bin/clang-format.exe`.
- The tree is not formatted with the repo `.clang-format`:
  - ae457ee system.cpp would change 216 diff lines, and system.h 139;
  - only 14 of the 198 naviGATR src files are clean;
  - the style rewrites `class X\n{\npublic:` as `class X {\n  public:`, unlike the rest of the tree.
- Formatting the 12 split files would change 365 diff lines inside moved code.
- Recommendation: do not format the moved bodies. The generated lines are already in clang-format order.
- If the lead wants them formatted anyway, `split_system.py --verify` still proves the move: it passed on a fully formatted copy.

## 7. Step 2 procedure (after step 1 is green)

1. **Check that step 1 changed only include lines** in the two files:
   - `git -C <wt> diff -M ae457ee -- pi/naviGATR/src/runtime/system.cpp pi/naviGATR/src/core/system.cpp` (the same for `.h`) must show only `#include` lines.
2. **Split.** `py -3 restructure/scripts/split_system.py --src <wt>/pi/naviGATR/src --rename restructure/plan/move_map.tsv --report restructure/logs/split_step2_report.txt`
   - Use the final map if step 1 renames it (for example `move_map_final.tsv`). The script accepts `src/...` or `pi/naviGATR/src/...` paths and ignores rows outside `src/`.
   - It reads `core/system.{h,cpp}` through the rename map, writes the 11 new files, rewrites `core/system.h` and deletes `core/system.cpp`.
   - It fails if any anchor or line is missing.
   - If step 1 did reformat the files, the anchors fail. In that case, split the ae457ee text instead: `git show ae457ee:pi/naviGATR/src/runtime/system.cpp > <tmp>/system.cpp` (same for `.h`), then run with `--cpp-in <tmp>/system.cpp --h-in <tmp>/system.h --rename ...`. This renames includes everywhere, including system.h's own block. Then format the outputs the same way step 1 formatted.
3. **Check destinations.** The draft map matches them. If the final map puts `inspection_state.h` anywhere other than `debug/inspection/`, or drops `debug/diagnostics/`, edit the `INSPECTION`, `EVENTS` or `EVENTS_H` constants first.
4. **CMake.** In `pi/naviGATR/CMakeLists.txt` `NAVIGATR_SOURCES`, replace `src/core/system.cpp` with:
   - `src/core/build.cpp`, `src/core/run.cpp`;
   - `src/brain_profile/profile_host.cpp`, `src/brain_profile/control.cpp`, `src/brain_profile/pico_operation.cpp`, `src/brain_profile/recovery.cpp`;
   - `src/debug/diagnostics/events.cpp`, `src/debug/inspection/inspection_state.cpp`.
5. **Docs that name system.cpp:**
   - `pi/naviGATR/docs/architecture.md:366`, the "Coordinator and workers" link: point it to `core/build.cpp` and `core/run.cpp`;
   - `pi/naviGATR/third_party/README.md:54`, the tinyxml2 users: `core/build.cpp`, `brain_profile/profile_host.cpp`.
   - Line numbers are from ae457ee; step 1 may already have edited these lines.
6. **Stage.** `git -C <wt> add -A pi/naviGATR/src/core pi/naviGATR/src/brain_profile pi/naviGATR/src/debug pi/naviGATR/CMakeLists.txt <the two docs>` (index only, no commit).
7. **Verify** as in section 8.

## 8. Verification (trial results, and what step 2 must show)

The trial ran on a scratch copy. Commands and paths are in `restructure/logs/split_plan_trial.log`, build dirs under `S/wt-build/split-plan/`.

| Check | Trial result | Step 2 must show |
|---|---|---|
| build, `-Wall -Wextra` | exit 0, 0 warnings, perf tools included | the same |
| ctest | 635/635 passed with perf tools on; 626/626 with them off | the same names as step 1 pass |
| `ctest -N` names | the 626 baseline names all present; +9 perf tests only because perf was on | diff against step 1's list is empty |
| `nm -C --defined-only libnavigatr_core.a`, sorted unique "type name" | identical to baseline, 72413 lines | identical to step 1's library |
| `split_system.py --verify <orig dir>` | every segment found; residue is only labels, includes, namespace lines and the unchanged header preamble | the same |
| coverage (built into the split run) | every non-blank original line used exactly once, except the 1 dropped banner and the regenerated structural lines | the same |
| `git diff -M --color-moved=zebra` | n/a (no git in trial) | only moved blocks, the new boilerplate, labels, and CMake/doc lines |

## 9. Follow-ups (not in step 2; each needs user approval)

- **F1. Brain profile as its own object.**
  - A class in `brain_profile/` that implements `BrainProfileHost` directly and owns the Brain-profile state System carries today:
    - `profiled_`, `profile_config_`, `profile_functions_`, `base_catalog_`;
    - `candidate_`, `applied_`, `profile_pending_`, `profile_applies_`, `profile_mutex_`, `profile_warnings_`;
    - `precheck_`, `pico_`, `pico_op_`;
    - `sensor_loss_`, `loss_warned_`, `sensor_lost_`, `models_dropped_`, `seen_`.
  - It needs a narrow handle on the stages it swaps: the sensor and localization executors, resources, command, robot state and placement floor.
  - The results:
    - the `ProfileHost` adapter and `ProfileCandidate` leave System;
    - about 20 members leave the class;
    - the five `if (profiled_)` hooks at system.cpp 1041, 1048, 1055, 1087 and 1321 become calls on it;
    - system.h becomes the thin facade SPEC describes.
  - Logic-preserving, but not a move.
- **F2. Pico operation tracking as data plus free functions next to the Pico resource.**
  - For example `startPicoOperation(PicoControl&, ...)` and `settlePicoOperation(PicoControl&, PicoOperation&, now)`, returning an outcome that the profile host turns into events and placement loss.
  - Then `pico_operation.*` can live with the Pico resource without including `core/system.h`. That is the task's intent for "Pico control".
- **F3. Readable `build`.**
  - Split the 204-line `System::build` into named steps: top-level sections, stages, slots, capture.
  - Replace the `Tag`/generic `buildSlot` lambda with three explicit slot builds.
  - Replace `checkChildren` with `ConfigNode::onlyChildren` + `atMostOne`, which duplicate it (survey cpp-pi.md).
- **F4. One event call.**
  - An event log in `debug/diagnostics/` owning `events_`, `event_count_` and `event_source_`.
  - A single helper for the paired `noteEvent` + `noteFault` calls with the same text (748-749, 763-765, 769-771, 879-882).
  - A deque instead of vector erase-front.
- **F5. No sensor-specific types in System.**
  - `publishDetectionFrames` binds AprilTag observation and association sets to camera frames.
  - Move that binding behind the estimator, for example a frame-binding hook the apriltag estimator provides, so core and System stay sensor-agnostic.
- **F6. Aliases** (SPEC code style), for example `using DetectionFrameMap = std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>>;`, which is spelled out 4 times. These are declaration edits, so not in step 2.
- **F7. `resumeWorkers` throws when a restart fails** (system.cpp 934, reached from `applyPendingProfile` in main with no try). A degrade policy would be a behavior change.
- **F8. Trim system.h's 31 project includes** after F1, F4 and F5. Includers that relied on transitive includes get direct ones.
- **F9. `.clang-format` vs the tree style.** Either add `AfterClass` and `AccessModifierOffset: -4` so it matches the tree, or run one whole-tree format commit. This is the user's call; it touches every file.
