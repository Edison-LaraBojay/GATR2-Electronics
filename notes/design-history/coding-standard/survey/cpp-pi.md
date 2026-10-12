# Survey: C++ conventions as practiced in naviGATR (Pi)

Scope: `pi/naviGATR/src` (198 C++ files, 37,022 lines, `apriltag_c.c/.h` excluded), `pi/naviGATR/tests` (39 files, 29,379 lines), `pi/naviGATR/tools/perf` (12 C++ files, 4,208 lines). Surveyed 2026-09-28 against the tree at commit ae457ee (working tree clean).

Paths below are relative to `pi/naviGATR/` unless they start with `brain/` or `translaGATR/`. Snippets are quoted verbatim unless marked "(trimmed)".

## Method and caveats

- Counts come from regex scans over comment- and string-stripped source (scripts in the scratchpad `standard/cpp-pi/`). They are heuristic: a count is right to within a few percent and every example cited below was checked by hand.
- Function size and complexity: `lizard` 1.x (NLOC, cyclomatic complexity CCN, parameter count).
- Formatting: `clang-format` 22.1.8 with the repo-root `.clang-format`, run as a dry diff. The task note said a copy lives at `/c/msys64/ucrt64/bin`; it does not (that directory has no clang tools). I used the pip wheel `clang-format==22.1.8` in a scratch venv; the user's roaming copy at `AppData/Roaming/Python/Python314/Scripts/clang-format.exe` also reports 22.1.8 and runs (the memory note that it segfaults on Python 3.14 is out of date).
- Brain comparison: a lighter scan of 135 C++ files under `brain/` (PROS sources and `firmware/` excluded).

## Headline findings

1. The codebase is internally consistent on the big things: PascalCase types, camelCase functions, snake_case fields, `trailing_` private members, `kPascal` constants and enumerators, `#pragma once`, own-header-first includes rooted at `src/`, default member initializers on almost every scalar field, no exceptions, `bool/optional/unique_ptr + std::string& err` for construction failure, `XOutput run(const XInput& in)` + `reset()` for stage contracts, and `std::shared_ptr<const Snapshot>` for anything crossing a thread.
2. The drift is concentrated and nameable: `make_*`/`register_*` snake_case functions; camelCase `*At` timestamp fields; three factory spellings; two microsecond clocks with different origins; one stray `throw`; duplicated enum-to-string tables that disagree; a `kind` tag duplicated next to a `std::variant`; interfaces with one implementation and no fake (and two, `SpiBus`/`SpiDevice`, with zero production implementations).
3. Formatting is not enforced: 235 of 249 naviGATR C++ files differ from the pinned `.clang-format` (14,076 changed diff lines). Brain (102 of 135) and translaGATR (8 of 8) differ too.
4. The user's gripe is structural, not stylistic. The main pipeline lives inside a 1,450-line `runtime/system.cpp` whose class has roughly 47 public and 31 private member functions and about 63 data members; the construction sequence is lines 148-351 of that file, the per-cycle pipeline is lines 1022-1140, and 570 lines of Brain-profile, Pico and recovery logic sit between them. The generic `runtime/` layer also includes two concrete implementations.
5. Comments break the user's terse rule mostly in file headers: 49 of 198 src files open with 15 or more comment lines, 20 with 30 or more (`runtime/system.h` has 83). Inline comments are mostly fine.

---

## 1. Naming

### Dominant convention (evidence)

| Kind | Convention | Evidence |
|---|---|---|
| Types (struct, class, enum, alias) | PascalCase | all 382 struct/class definitions in src |
| Functions and methods | camelCase (or one lowercase word) | 703 camelCase + 502 single-word of 1,264 named src functions (lizard) |
| Struct fields, locals, parameters | snake_case (or one lowercase word) | 776 snake + 908 single-word of 1,733 struct fields |
| Private class members | snake_case with trailing `_` | 586 of 590 class data members (the other 4 are scan artifacts) |
| Constants | `kPascal` (`constexpr`) | 77 `constexpr ... k[A-Z]` definitions; no ALL_CAPS constants in src |
| Enumerators | `kPascal` | all 30 `enum class` in src |
| Namespaces | lowercase component name | `navigatr` in 207 places |
| Files | snake_case, `.h`/`.cpp` pairs | every file under src |
| Registered implementation names (XML `type=`) | snake_case strings | `"pico_encoder_channel"`, `"weighted_planar_fusion"` |

### Inconsistencies

- Function case. 27 src functions are snake_case, all `make_*` (20) or `register_*` (7):
  - `runtime/register_all.h:21` `void register_resources(FunctionRegistry& functions);` next to `runtime/register_all.h:30` `void registerAll(FunctionRegistry& functions);` and `runtime/register_all.h:34` `inline void registerOrDie(...)`.
  - Stage builders `make_resources`, `make_sensors`, `make_localization`, `make_world_estimation` (e.g. `runtime/localization_stage.h:132`) share the `make_` prefix with implementation factories such as `impl/sensors/attitude_channel.cpp:15` `make_attitude_channel`, so the same prefix means both "build a whole stage" and "build one plugin".
- Factory spelling has three forms for the same role (a function stored in `FunctionRegistry`):
  - free snake_case: `impl/sensors/register_sensors.cpp:13` `&make_pico_encoder_channel`
  - static member: `impl/localization/register_localization.cpp:21` `&TrackingWheelMotion::create`
  - free camelCase: `impl/localization/register_localization.cpp:27` `&makeNoopStateEstimator`
- Struct fields that are timestamps are camelCase (38 fields), everything else snake_case:
  - `core/records.h:71` `MonotonicTime measuredAt;   // source clock`, `core/records.h:96` `lastPolledAt`
  - `state/robot_state.h` `measuredAt`, `measuredAtHost`; `resources/camera.h:64` `exposureAt`
  - and a few non-time ones: `contracts/sensor.h:31` `PayloadDescriptor outputPayload;`, `contracts/field_estimation.h:46` `const FieldState& previousField;`, `resources/resource_store.h:76` `FunctionKey implementationType;`
  - while the same concept elsewhere is snake_case: `runtime/handoff.h:128` `int64_t last_cycle_host_ms = -1;`
- pImpl structs drop the trailing underscore even though they are the private state of a class: `inspection/inspection_service.cpp:319` `mutable std::mutex mutex;   // clients, stats, sequence numbers`. Conversely `impl/resources/synthetic_rig.cpp:201` `SyntheticRigExecutable` is a `class` with all-public members that do carry `_`.
- One concept, several names:
  - World estimation: XML `<WorldEstimation>`, folder `impl/world_estimation/`, stage `runtime/world_estimation_stage.h` (`WorldEstimationExecutor`), but the contract is `contracts/field_estimation.h` class `FieldEstimation`, its output `FieldEstimationOutput`, the worker is the "field worker", the snapshot `FieldSnapshot`.
  - Command collection: XML `<CommandCollection>`, contract class `Commands`, register function `register_commands`.
  - Publishing: class `Publishing`, register function `register_publishers` (plural).
  - Diagnostics: `core/diagnostics.h` `struct Diagnostics` (per-function counters) versus folder `diagnostics/` (`DiagnosticsHub`, records, link monitor). A member called `diagnostics` is `Diagnostics*` in `core/execution_context.h:19` and `contracts/commands.h:25`, but `DiagnosticsHub*` in `contracts/slot_init.h:47`, `resources/resource_store.h:44` and `runtime/build_options.h:21`.
- Enum-to-string helpers use a `...Name` suffix consistently (`core/records.h:44` `sourceStateName`), but see section 14 for duplicates.

### Proposed rule

- N1. Types PascalCase; functions and methods camelCase; variables, parameters and struct fields snake_case; private data members snake_case with trailing `_`; constants and enumerators `kPascal`; namespaces, files and folders lowercase snake_case. No exceptions for "pipeline verbs": `make_*` and `register_*` become camelCase (`makeResources`, `registerResources`), or the standard names one explicit exception and says why. (Decision for the lead: the snake form was chosen to mirror the pseudo-code in the stage headers, e.g. `runtime/resource_stage.h:4` `built = make_resources(<Resources>, registry, options)`.)
- N2. Timestamps follow the field rule: `measured_at`, `received_at`, `last_polled_at`.
- N3. One registered-factory spelling. Recommended: a free function `make<Implementation>` declared in the implementation header (functional style, greppable, matches the user's "prefer functions over classes" preference). A class with a private constructor keeps a static `create` only when the factory needs private access; then the free `make<Implementation>` forwards to it.
- N4. One name per concept across XML element, folder, contract, stage, class and log text. A glossary in the pipeline doc lists the canonical noun for each stage (section 27).
- N5. A member named after a type holds that type: no `diagnostics` meaning two different types.

---

## 2. struct vs class, free functions vs members

### Dominant convention (evidence)

- src defines 286 structs and 96 classes (30 `enum class` excluded).
  - 252 structs are pure data: fields with default initializers, no methods. Examples: `core/records.h` `Publication`, `PollResult`, `StoredSample`; every `*Input`/`*Output` of a contract.
  - Classes with behavior keep data private with `_` members. About 21 are abstract contracts (only pure virtual functions plus `virtual void reset() {}`); 33 classes and structs in src derive from one of them.
- Free functions are common and idiomatic here: math (`math/transforms.h` `compose`, `inverse`), parsing (`config/field_map.cpp` `parseFieldMap`), stage builders (`make_*`), step helpers (`impl/localization/motion_step.h` `prepareStep`, `finishStep` operating on a `PlanarStepState` struct). This is already the functional style the user asks for.
- Internal helper classes live in anonymous namespaces inside the `.cpp` (for example `capture/capture_bundle.cpp:264` `Row`, `:347` `Segments`, `:471` `Writer`).

### Inconsistencies

- 34 structs have member functions. Three kinds:
  - Harmless queries or operators: `core/time.h:23` `MonotonicTime` with `isSet()`, `core/ids.h` `TypedId` with comparison operators and a nested `Hash`, `config/field_map.h:159` `FieldMap::find`.
  - Behavior that maintains invariants, which by the user's rule should be a class:
    - `resources/brain_imu_bench.h` `struct BrainImuBench` with `reset()`, `accept()`, `acceptAttitude()` that bump `epoch` and `sequence` (trimmed): `void accept(uint32_t s, uint8_t flags, uint32_t stamp, int32_t rotation, MonotonicTime now) { if (session != s) reset(s); ...`
    - `impl/localization/motion_step.h:63` `struct PlacementEdge` with `bool apply(RobotState& r, const PlacementRequest& placement);`
    - `core/diagnostics.h` `struct Diagnostics` with `note()`
    - `impl/sensors/pico_channels.cpp:28` `ChannelInput` with `bind`, `fresh`, `publicationFor`, `reset`
  - Validation logic on a context struct, including a `private:` section: `contracts/slot_init.h:41` `struct SlotInitializationContext` (the only struct with `private:`), whose `requireObservation` at `contracts/slot_init.h:70` duplicates `contracts/localization.h:178` `StateEstimatorInitializationContext::requireObservation`.
  - pImpl bodies declared `struct` that are full classes with threads and methods: `inspection/http_server.cpp:401` `struct HttpServer::Impl` (`listen`, `run`, `acceptPending`, `readClient`, ...), `inspection/inspection_service.cpp:250` `struct InspectionService::Impl`.
- Static-only class: `core/host_clock.h:16` `class HostClock` with only `static now()`, `static nowUs()`, `static origin()`. A namespace of free functions would do.
- Classes that exist to hold a single call:
  - The four no-op contract implementations in `impl/noop/noops.cpp` are unavoidable while the contracts are virtual.
  - `runtime/resource_stage.h:35` `class ResourceFunctions` and `runtime/sensor_stage.h:34` `class SensorFunctions` are the same class twice with different id types (`add`, `find`, `executionOrder`, `size`, identical bodies).
- Interfaces invented ahead of a second implementation (the "extract, don't invent" principle):

  | Interface | src impls | test fakes |
  |---|---|---|
  | `resources/spi_bus.h:35` `SpiDevice`, `:45` `SpiBus` | 0 | 1 (`tests/resources_gtest.cpp:36` `FakeSpiDevice`) |
  | `capture/capture_service.h:33` `CaptureService` | 1 (`CaptureRecorder`) | 0 |
  | `contracts/perception.h:34` `Perception` | 1 | 0 |
  | `contracts/association.h:39` `Association` | 1 | 0 |
  | `resources/pico_control.h:52` `PicoControl` | 1 | 2 (`tests/brain_profile_gtest.cpp:145` `struct FakePico : PicoControl`) |
  | `contracts/brain_profile.h:40` `BrainProfileHost` | 1 | 1 (`tests/brain_io_gtest.cpp:170` `struct FakeProfileHost : BrainProfileHost`) |

  The last two are justified test seams by the S3 rule below. `Perception` and `Association` are private steps inside `AprilTagWorldEstimator`, never selected by configuration (`impl/world_estimation/apriltag_world_estimator.h:14` "reachable only through this class").
- `class System` (`runtime/system.h:186`) is the opposite problem: one class that owns construction, both workers, the three cycles, Brain profile staging and swapping, Pico CONTROL operations, sensor-loss recovery, snapshot publication, event logging and capture wiring.

### Proposed rule

- S1. `struct` means data only: public fields with default initializers. Allowed extras: comparison operators, a nested `Hash`, and `const` accessors that compute a value from the fields without side effects (`isSet()`, `fieldPose()`). Anything that mutates to keep fields consistent makes it a `class` with private data. (Decision for the lead: the user said "struct for attribute only things" without exceptions. The pure reading would move 34 structs to classes or free functions; the rule above keeps the obviously harmless ones.)
- S2. A `class` exists only when it keeps an invariant or owns a resource (thread, socket, device, mutex). Before adding one, write the free-function version; keep the class only if the function version needs state threaded through many calls. No static-only classes: use a namespace.
- S3. A virtual interface needs two implementations, or one implementation plus a test fake that a test actually uses. Otherwise call the concrete type. Existing single-implementation interfaces are listed and justified in the component pipeline doc or removed.
- S4. Duplicate classes that differ only in a type become one template or one alias (`ResourceFunctions`/`SensorFunctions`).
- S5. A class that needs more than about 25 member functions or has more than one reason to change is split along those reasons (System: build, cycles, workers, Brain profile, recovery).

---

## 3. Namespaces and internal linkage

### Dominant convention (evidence)

- Every src file except `app/main.cpp` and `app/field_refs.cpp` wraps its contents in `namespace navigatr` (207 occurrences). The namespace is flat: there is no per-folder or per-stage namespace.
- The only nested namespaces: `navigatr::payload_names` (7 headers, stable payload name constants, e.g. `payloads/camera_frames.h:20`) and `navigatr::perf` in `tools/perf`.
- File-local helpers go in an anonymous namespace (39 in src, 59 in tests, 7 in perf). No file-scope `static` functions in src.
- Brace style matches the memory rule: `namespace navigatr` then `{` on its own line, contents unindented, `} // namespace navigatr`.
- src never uses `using namespace` at file scope. Tests do: 36 of 38 test `.cpp` files have `using namespace navigatr;` at file scope (for example `tests/workers_gtest.cpp:33`); `tests/command_line_gtest.cpp` qualifies names instead and `tests/brain_link_e2e_gtest.cpp` uses namespace aliases.

### Inconsistencies

- `resources/brain_imu_bench.h:5` `namespace navigatr {` (attached brace), no file-name header line, unbraced single-line ifs. It is the one src file written in a different style.
- `inspection/inspection_document.cpp:1935` `using namespace translagatr;` inside a function body (the only `using namespace` in src).
- `inspection/http_server.cpp:723` `enum ChoiceKind { kChooseControl, kChooseReliable, kChooseSlot };` is a plain enum declared at `navigatr` scope after the anonymous namespace closed (`:351`), so it has external linkage and its values travel as `int& kind`.
- Mutable globals: `inspection/http_server.cpp:99` `std::mutex g_wsa_mutex;` sits at `navigatr` scope, not in the anonymous namespace; `impl/resources/libcamera_camera.cpp:60` `static std::mutex mutex;` and `impl/commands/brain_link_commands.cpp:160` `static std::atomic<uint32_t> instances{0};` are function-local statics. All three wrap process-wide OS or library state.
- Tests use ad hoc aliases for the Brain stack: `tests/brain_link_e2e_gtest.cpp:48` `namespace ag = actugatr;`, `:51` `namespace nv = navigatr;`.

### Proposed rule

- NS1. Every declaration lives in a namespace. The component namespace is `navigatr`. A folder that owns its own pipeline doc (section 27) gets a nested namespace of the same name (`navigatr::localization`, `navigatr::inspection`). Shared vocabulary (ids, time, records) stays in `navigatr`. (Decision for the lead: this is a large mechanical change; the minimum is "everything in `navigatr`", which is already true.)
- NS2. File-local functions, types and constants go in an anonymous namespace inside the component namespace. Never `static` at namespace scope.
- NS3. No `using namespace` in headers or at file scope in src. Allowed at file scope in test and tool mains for the component under test. Namespace aliases for other components are fine and should use the component's name, not two letters.
- NS4. Mutable global state only for process-wide OS or library handles, in the anonymous namespace, named `g_...`, guarded, with a one-line comment naming the handle.

---

## 4. Units and frames in names

### Dominant convention (evidence)

- Units are a name suffix, very consistently: `_m` (756 names), `_rad` (258), `_deg` (138), `_cdeg` (47), `_mm` (68), `_s` (425), `_ms` (702), `_us` (442), `_hz` (147), `_px` (242), rates `_m_s` (32) and `_rad_s` (49), `_dps` (50). Example `math/transforms.h` `struct Pose2D { double x_m = 0.0; double y_m = 0.0; double heading_rad = 0.0; };`.
- XML attributes carry the same suffix (`roll_deg`, `stale_after_ms`, `rate_hz`), so the XML key and the C++ field read the same.
- Frames use `T_a_b` = pose of frame b in frame a, stated once at `math/transforms.h:2-4` ("a value named T_a_b is the pose of frame b expressed in frame a, so compose(T_a_b, T_b_c) yields T_a_c") and repeated in `math/se3.h`. Over 250 uses (`T_field_robot`, `T_robot_camera`, `T_camera_tag`).
- Radians inside, wire units only at the boundary (`math/angles.h:2` "radians internally, wire units at the boundary").

### Inconsistencies

- Unit in a comment instead of the name: `impl/localization/stationary_window.h:64` `double value = 0.0; // travel m, rate rad/s or rotation rad`; `impl/localization/stationary_window.h:126` `double integral = 0.0; // gyro trapezoid, rad`.
- Unit missing where the XML key has it: `impl/localization/tracking_wheel_motion.cpp:262` `double still_travel = 0.005;` read by `:267` `getDouble("max_calibration_travel_m", 0.005, still_travel, err)`.
- Unitless increments: `impl/localization/motion_step.h:78` `void chordOfArc(double dx, double dy, double dtheta, double& lx, double& ly);`; `motion_step.h:163-164` `double dtheta = 0.0; double dt = 0.0;` next to `HeadingAligner::Result::dtheta_rad` and `dt_s` in the same header.
- Unit in the middle: `capture/capture_bundle.h:54` `double pre_s_requested = 0;`.
- Frame named three ways in one struct: `state/robot_state.h:45` `Pose2D odom_pose; // T_odom_robot` and `:46` `Pose2D field_from_odom; // T_field_odom, identity until re-anchored`, while a method on the same struct is `T_odom_robot3()`.

### Proposed rule

- U1. Every physical quantity's name ends in its unit: `_m _mm _um _rad _deg _cdeg _mdeg _s _ms _us _hz _m_s _rad_s _dps _px _kb`. Ratios and scales say so (`_ratio`, `scale`, `gain`). A comment never carries the unit.
- U2. Poses and transforms are named `T_a_b` (pose of b in a). No `a_from_b`, and no bare `odom_pose` with the frame in a comment. Covariances and velocities name their frame when it is not the obvious one.
- U3. SI radians and meters inside the pipeline; wire units (mm, cdeg, us counts) only in the code that encodes or decodes the wire, converted with the named helpers in `math/angles.h`.

---

## 5. Integer and time types

### Dominant convention (evidence)

- Timestamps inside the pipeline are `MonotonicTime` (`core/time.h:23`): `int64_t ms` plus a `ClockDomain` tag (`kUnset`, `kDevice`, `kHost`). Mixed-domain comparisons assert in debug builds. 294 uses in src.
- The single host clock is `HostClock::now()` (ms since process start, host domain) and `HostClock::nowUs()` (same origin, us).
- Device clocks are mapped to host time explicitly through `core/clock_sync.h` `DeviceToHostClock`.
- Durations are plain scalars with a unit suffix: 145 `int64_t *_ms/_us/_s`, 88 `double *_ms/_s`.
- `std::chrono` appears only at the edges (sleeping, worker periods, `core/host_clock.h`). No `std::chrono::duration` in any pipeline struct.
- Fixed-width integers for wire data (`uint8_t` 237, `uint32_t` 158, `uint64_t` 419 uses), `std::size_t` for sizes and indices (276).

### Inconsistencies

- A second microsecond clock with a different origin: `resources/serial_link.h:33` `inline int64_t steadyNowUs()` returns `steady_clock::now().time_since_epoch()` ("arbitrary origin"). `HostClock::nowUs()` counts from process start. Both are `int64_t` named `..._us`, so nothing stops a reader from differencing one against the other. `steadyNowUs` is used by `impl/commands/brain_link_commands.cpp:162`, `impl/resources/pico_telemetry.cpp:63` and the serial links; `HostClock::nowUs` by diagnostics, capture and inspection.
- `core/time.h:46` `inline bool operator==(MonotonicTime a, MonotonicTime b) { return a.ms == b.ms; }` ignores the domain, while `operator<` at `core/time.h:48` asserts `sameDomain`.
- `long` for durations, inherited from `ConfigNode::getInt(const char* key, long def, long& out, ...)` (`config/config_node.h:43`): `impl/localization/attitude_reference.h:57` `long max_age_ms_ = 100;`, `impl/localization/motion_step.h:140` `long max_wait_ms`, `impl/localization/tracking_wheel_motion.h:129` `long max_gap_ms = 250;`. 118 `long` uses in src. `long` is 32-bit on the Windows host build and 64-bit on the Pi (aarch64 Linux).
- Wire enums are carried as bare `uint8_t` with the real type in a comment: `runtime/system.h:147` `uint8_t action = 0; // translagatr::ControlAction`, `:152` `uint8_t result = 0; // translagatr::BrainResult, Pending until settled`.
- Timing classes read `steady_clock` directly instead of an injected clock, which forces real sleeps in tests: `runtime/handoff.h:153` `const auto now = std::chrono::steady_clock::now();` in `WorkerStats::cycleDone`; `tests/workers_gtest.cpp:573` `std::this_thread::sleep_for(Ms(2200));`. By contrast `capture/capture_recorder.h:57` `std::function<int64_t()> clock; // Pi host us; HostClock::nowUs when empty` is injectable.

### Proposed rule

- T1. Pipeline timestamps are `MonotonicTime`. Scalar durations are `int64_t` with `_ms`/`_us` or `double` with `_s`; never `long` or `int`. Config readers return `int64_t`.
- T2. One host clock. `HostClock` is the only source of host time in naviGATR; `steadyNowUs` is removed or becomes `HostClock::nowUs`. Any other clock (Pico, Brain, libcamera `CLOCK_BOOTTIME`) is a named domain and is only converted, never differenced raw.
- T3. `using` aliases name the clock of a raw scalar where a struct would be overkill, for example `using HostUs = int64_t;`. (Decision for the lead: a real `HostUs` struct would make mixing a compile error; the alias only documents it. The user favors aliases for readability.)
- T4. Code with timing logic takes its clock as a parameter or an injected `std::function<int64_t()>` (one alias for that signature, section 18), so tests step time instead of sleeping.
- T5. Wire enums keep their `translagatr` enum type in naviGATR structs, or the struct field name says which enum it holds. No type-in-a-comment.
- T6. `MonotonicTime` equality respects the domain, the same as ordering.

Pi vs Brain: Brain uses `uint32_t *_ms` from the PROS millisecond clock (`brain/communiGATR/include/communigatr/attitude.h:26` `uint32_t stamp_ms = 0; // Brain clock at the read`, `brain/actuGATR/include/actugatr/pros_drive.h:27` `uint32_t period_ms = 10;`) and has no clock-domain type. A standard rule should say: each component has one local clock type; cross-component times cross the wire as the wire type and are mapped explicitly on arrival.

---

## 6. Error handling

### Dominant convention (evidence)

- No exceptions. src has 1 `throw`, 3 `try`/`catch` (around `std::thread` creation in `runtime/system.cpp` and libcamera calls); tests and perf have none.
- Construction and configuration errors: return a failure value and fill `std::string& err` (252 `std::string& err` parameters in src). Error text starts with the XML path: `impl/sensors/attitude_channel.cpp` (trimmed) `err = node.path() + ": needs <Source resource_id=... output_id=.../>"; return std::nullopt;`. Failure values:
  - `bool` plus an out parameter (62 bool-with-err declarations in headers)
  - `std::optional<T>` (stage builders, sensor factories: `contracts/sensor.h:47`)
  - null `std::unique_ptr<T>` (slot and model factories: `contracts/commands.h:43`, `contracts/localization.h:114`)
- Atomic construction: a failed build destroys the whole candidate, `runtime/system.cpp:132-136` (trimmed) `std::unique_ptr<System> system(new System()); if (!system->build(xml, functions, options, err)) { return nullptr; // atomic: the partial candidate dies here }`.
- Degrade instead of fail for hardware that is absent at startup: a device that fails to open becomes a build warning (`impl/resources/serial_links.cpp:183` `context.warnings->push_back(node.path() + ": " + open_err);`) and a runtime `SourceState::kUnavailable`, never a hang.
- Runtime errors never unwind: stages return `FunctionStatus` (`kOk`, `kNoData`, `kFault`, `core/function_status.h:12`) plus a `diagnostic` string, and sources report `SourceState` (`core/records.h:37`).
- Programming errors abort loudly: `runtime/register_all.h:34-39` `registerOrDie` prints and calls `std::abort()` on a duplicate registration.

### Inconsistencies

- The one `throw`: `runtime/system.cpp:934` `throw std::runtime_error(err);` in `resumeWorkers`, reached from `System::reset()` and `System::applyPendingProfile()` while `lifecycle_mutex_` is held. `app/main.cpp:174` calls `system->applyPendingProfile();` with no `try`, so a failed thread start terminates the process instead of degrading.
- `tools/perf/perf_json.h` `bool parseJson(const std::string& text, Json& out, std::string* err = nullptr);` uses an optional pointer where src uses a required reference.
- `[[nodiscard]]` is used nowhere (0 in src, tests, perf, Brain), so an ignored `bool` from a `parse*`/`require*` call compiles silently.
- `impl/resources/pros_usb_link.cpp:79` returns an aggregate of four bools and a C string positionally: `return {false, true, false, false, "USB reply expired"};` (`SerialWriteResult` at `resources/serial_link.h`).
- Mixed shapes for the same job (fallible construction) cost the reader: `bool f(..., T& out, err)` versus `std::optional<T> f(..., err)`.

### Proposed rule

- E1. No exceptions in naviGATR code. Wrap third-party calls that can throw (`std::thread`, libcamera, `std::filesystem` without `error_code`) at the call and convert to the local error form.
- E2. Fallible construction and parsing return `std::optional<T>` (or `std::unique_ptr<T>` for polymorphic results) and take `std::string& err` as the last parameter; the message starts with the XML path. `bool` + out parameter only when filling a field of an existing struct in place (config readers).
- E3. `[[nodiscard]]` on every function whose return value reports success or carries the result.
- E4. Runtime failures are values (`FunctionStatus`, `SourceState`, a `diagnostic` string), never unwinding. Programming errors (registry collisions, broken invariants) abort with a message.
- E5. Absent or failing hardware is a warning plus a status, never a failed build and never a blocking wait (the "degrade, don't hang" principle, already practiced).
- E6. Result structs with several flags are built with named fields, not positional braces.

Pi vs Brain: Brain has zero `std::string& err` parameters and one `std::optional`; it reports with `bool` and small result enums (`brain/communiGATR/include/communigatr/client.h:283` `bool setProfile(const ProfileDocument& doc);`). That fits a PROS target with no XML configuration. The standard should allow both: string errors where there is configuration to point at (Pi), enum results on the Brain and Pico.

---

## 7. Ownership and lifetime

### Dominant convention (evidence)

- `std::unique_ptr` for sole ownership of polymorphic stage objects and services (95 uses in src), for example `runtime/system.h:398-400` `std::unique_ptr<Commands> commands_; std::unique_ptr<TargetResolution> target_resolution_; std::unique_ptr<Publishing> publishing_;`.
- `std::shared_ptr<const T>` for immutable values shared across threads or published as snapshots: `core/typed_payload.h` stores payloads as `std::shared_ptr<const HolderBase>`; camera pixels are `resources/camera.h:74` `std::shared_ptr<const std::vector<uint8_t>> y8;`; `runtime/system.h:452-457` snapshot members. 154 `std::shared_ptr` uses in src; the most repeated are `std::shared_ptr<const ProfileBinding>` (11) and `std::shared_ptr<const FieldMap>` (8).
- Non-const `std::shared_ptr` for shared devices bound through `ResourceStore` (links, cameras, the Pico telemetry resource).
- Raw pointers are non-owning and nullable, mostly in context structs with `= nullptr` defaults: `contracts/sensor.h:40-43` (trimmed) `const ResourceStore* resources = nullptr; const ResourceCatalog* outputs = nullptr; ... std::vector<std::string>* warnings = nullptr;`.
- Configuration views are non-owning and die with the document: `config/config_node.h:5-6` "Views are valid only while the document is loaded, i.e. during initialization; implementations copy what they keep."
- Destruction order is designed and commented: `runtime/system.h:382` `// destruction order: declared first, destroyed last`.
- Naked `new` appears 8 times in src, always handed straight to a smart pointer: private constructors behind static factories (`runtime/system.cpp:132` `std::unique_ptr<System> system(new System());`, `impl/resources/tag_detectors.cpp:73`) and pImpl allocation (`inspection/http_server.cpp:1392` `impl_(new Impl())`). No `delete`.

### Inconsistencies

- Reference-returning getters into worker-owned state, with the thread rule only in a comment: `runtime/system.h:271` `const ResourceMap& resourceMap() const { return execute_resources_.retained(); }` ("Inline mode or stopped workers only"), and a non-const one at `runtime/system.h:278` `Diagnostics& diagnostics() { return diagnostics_; }`.
- Output lists passed as nullable pointer and error as reference in the same signature: `runtime/localization_stage.h:132-137` (trimmed) `make_localization(..., std::vector<std::string>* warnings, std::string& err);`.
- `std::weak_ptr` appears twice, both for the libcamera manager singleton (`impl/resources/libcamera_camera.cpp:64`).

### Proposed rule

- O1. Owning: `std::unique_ptr` by default; `std::shared_ptr<const T>` for immutable shared data and cross-thread snapshots; non-const `std::shared_ptr<T>` only for a device or service that several consumers use for its whole life, and it states its thread rule.
- O2. Non-owning: a reference when never null, a raw pointer when nullable. Raw pointers never own. No naked `new` except inside a factory for a private constructor; no `delete`.
- O3. No reference or pointer into state another thread writes is returned from a public API. Readers get a copy or a `shared_ptr<const Snapshot>`.
- O4. Classes whose member order matters for destruction say so next to the members, as `runtime/system.h:382` does.

Pi vs Brain: Brain has 0 `std::shared_ptr` and 35 `std::unique_ptr` (no shared snapshots, single-threaded PROS tasks plus `pros::Mutex`). The rule O1 holds on both; Brain simply never needs the shared case.

---

## 8. Concurrency

### Dominant convention (evidence)

- Fixed, named threads, all joined: the estimation worker and the field worker (`runtime/system.cpp:1411` `estimationWorker`, `:1436` `fieldWorker`), the HTTP server, the inspection service, the capture recorder, the libcamera completion thread. 12 `std::thread` in src, no `detach()`.
- One writer per mutable state, stated in `runtime/system.h:37-42` ("One writer per mutable state: the estimation worker owns the executors ..."). Readers get `std::shared_ptr<const ...>` snapshots swapped under a mutex (`runtime/system.cpp:1097` `std::lock_guard<std::mutex> lock(snapshot_mutex_);` then `reporting_ = std::move(reporting);`).
- Latest-wins handoff: `runtime/handoff.h:27` `template <typename T> class LatestSlot` replaces a pending entry and counts `replaced_`. Explicitly not used for motion increments (`runtime/handoff.h:8-10`).
- `std::lock_guard` for scopes (166), `std::unique_lock` only for condition waits (5). No `std::scoped_lock`, no recursive or shared mutexes.
- Every condition-variable wait has a predicate and a bound: `runtime/handoff.h:58` `cv_.wait_for(lock, timeout, [&] { return stopped_ || pending_.has_value(); });`, `runtime/system.cpp:1432` `wake_.wait_until(lock, next, [this] { return stop_.load(); });`.
- Atomics for flags and counters only (26 in src), for example `diagnostics/link_monitor.h:111` `std::atomic<uint64_t> rx_bytes_{0};`.
- Lifecycle changes (reset, profile swap) stop and join both workers first, then restart them (`runtime/system.cpp:910` `pauseWorkers`), so no stage is ever reset while running.

### Inconsistencies

- Thread rules live in free-form comments with many spellings (52 annotations): "any thread" (`capture/capture_recorder.h:37`), "Estimation worker only" (`impl/localization/stationary_window.h:29`), "Thread safety: SerialLink implementations are single-threaded unless their own documentation says otherwise" (`resources/serial_link.h:5-6`), "Thread safety: refresh and the channel reads run on the pipeline thread." (`impl/resources/pico_telemetry.h:46`), "Inline mode or stopped workers only" (`runtime/system.h:269-270`).
- Lock order is implicit. `System` has four mutexes (`runtime/system.h:440` `profile_mutex_`, `:451` `snapshot_mutex_`, `:474` `lifecycle_mutex_`, `:475` `wake_mutex_`); the code takes `lifecycle_mutex_` and then the others one at a time (`publishBindingView` at `runtime/system.cpp:943` reads `profileBinding()` under `profile_mutex_`, releases, then takes `snapshot_mutex_`). Nothing documents that no two may nest.
- Helper naming comments like `// lifecycle mutex held` (`runtime/system.h:358`, `:365`) are the only precondition markers.
- `impl/resources/pico_telemetry.cpp:490` `// submit() takes state_mutex_; this runs with it held, so add directly.` shows a caller-holds-lock convention handled by comment.

### Proposed rule

- C1. Every class or struct that is touched by more than one thread has a one-line thread tag on the type, in one fixed form: `// thread: any`, `// thread: estimation worker`, `// thread: field worker`, `// thread: stopped or inline`. Public members that differ from the type's tag carry their own.
- C2. One writer per mutable state. Cross-thread reads go through a `shared_ptr<const Snapshot>` swapped under a mutex, or through `LatestSlot`. Motion data is never dropped by a latest-wins handoff.
- C3. Never hold two mutexes at once. If nesting becomes necessary, the class comment states the order.
- C4. Every blocking wait has a predicate and a timeout. Threads are joined, never detached.
- C5. Atomics only for independent flags and counters, never to publish a multi-field state.
- C6. A function that must be called with a lock held says so in its name suffix (`...Locked`) instead of a comment.

---

## 9. Allocation in hot loops

### Dominant convention (evidence)

- Readability wins and the code allocates freely per cycle at 100 Hz:
  - `runtime/system.cpp:1421-1422` `field_handoff_.put(FieldWork{std::make_shared<const SensorMap>(execute_sensors_.retained()), now});` copies the sensor map (envelopes only; payloads are shared) every estimation cycle.
  - `runtime/system.cpp:1142` `publishSourceHealth` builds a vector of entries with `std::string` fields every cycle, e.g. `:1167` `e.kind = "resource";`.
  - `core/diagnostics.h` `Diagnostics::note(const std::string& label, ...)` does a `std::map<std::string, FunctionStats>` lookup per function per cycle.
- Heavy data is not copied: camera pixels are one `shared_ptr<const std::vector<uint8_t>>` (`resources/camera.h:74`); payloads are shared immutable values (`core/typed_payload.h`).
- `reserve` is rare (15 in src); `push_back` dominates (207).

### Inconsistencies

- None that matter at the current rates. The only measured-performance code is the inspection path, which has its own perf harness (`tools/perf`).

### Proposed rule

- A1. Readable first. Per-cycle allocation of small envelopes, strings and snapshots is allowed. Large buffers (images, point sets) are shared as `shared_ptr<const T>` and never copied per consumer. Optimize only with a measurement from `tools/perf` or a profiler checked in next to the change.

---

## 10. Header layout, include order, forward declarations

### Dominant convention (evidence)

- Every header uses `#pragma once` (128 in src; no include guards).
- File layout, in order:
  1. a comment block whose first line is `// <file name>`
  2. `#pragma once`
  3. the includes
  4. `namespace navigatr` and the body
  185 of 198 src files start with `// <file name>`.
- Include order in `.cpp`: own header first (60 of 70 src `.cpp` files; the other 10 are the two mains, the seven `register_*.cpp` files and `capture/capture_service.cpp`), a blank line, `<standard>` headers, a blank line, project headers. Project includes are rooted at `src/` (`#include "runtime/system.h"`); shared codec includes are rooted at the repo (`#include "translaGATR/frames.h"`).
- Forward declarations to cut header dependencies are common (36 in src headers), for example `contracts/sensor.h:27-28` `class ResourceStore; class ResourceCatalog;` and `config/config_node.h:16-19` forward-declaring `tinyxml2::XMLElement`.
- 68 of 129 src headers are header-only (no matching `.cpp`).

### Inconsistencies

- 13 src files break the first-line rule: the 7 `payloads/*.h` put the path in it (`// payloads/camera_frames.h`), and `app/command_line.h`, `app/command_line.cpp`, `resources/brain_imu_bench.h`, `impl/resources/camera_capture.h`, `impl/resources/pros_usb_link.h/.cpp` have no such line.
- `capture/capture_service.cpp` implements the class declared in `capture/capture_recorder.h`, and `capture/capture_service.h` declares the interface; header and source names do not pair.
- `impl/resources/pros_usb_link.cpp:14-18` has a second `<system>` block (`<cerrno>`, `<fcntl.h>`, `<poll.h>`, ...) after the project includes, inside a platform `#if`.
- Large non-trivial functions defined inline in headers: `runtime/inspection_config.h:74` `parseInspectionConfig` (about 100 lines, CCN 51) and `capture/capture_config.h` `parseCaptureConfig` (about 50 lines).
- Six src files exceed 1,000 lines: `inspection/inspection_document.cpp` (2,052), `inspection/http_server.cpp` (1,614), `runtime/system.cpp` (1,450), `capture/capture_bundle.cpp` (1,389), `inspection/inspection_service.cpp` (1,350), `capture/capture_service.cpp` (1,104).

### Proposed rule

- H1. `#pragma once`; own header first, then standard headers, then third-party, then project headers, each group sorted and separated by a blank line (clang-format `IncludeBlocks: Regroup` with categories can enforce this). Platform headers go in a guarded group at the end.
- H2. A `.cpp` is named after the header it implements.
- H3. Headers declare; definitions longer than a few lines go in the `.cpp` unless they are templates. Parsers never live in headers.
- H4. Forward-declare instead of including when only pointers or references are used.
- H5. A file past about 800 lines is a signal to split by responsibility; the pipeline doc names the split.
- H6. First line `// <file name>`: keep or drop, decide once (section 23).

Pi vs Brain: Brain components use public headers under `include/<component>/` (`brain/communiGATR/include/communigatr/client.h`) with sources in `src/`, `pros/` and `sim/`; naviGATR keeps headers next to sources under one `src/` tree. Include paths differ in case too: `translaGATR/frames.h` (mixed case directory) versus `communigatr/client.h` and `actugatr/drive.h` (lowercase). The standard should pick one layout per kind of component (library with public headers versus executable) and lowercase include paths.

---

## 11. const, constexpr, noexcept, [[nodiscard]]

### Dominant convention (evidence)

- Const-correct accessors and parameters throughout (`const T&` for inputs, `const` member functions for getters, `const auto` locals).
- `constexpr` for named constants (109 in src), both at namespace scope (`math/angles.h:11` `constexpr double kPi = 3.14159265358979323846;`) and as `static constexpr` class members (`runtime/system.h:189` `static constexpr int64_t kPicoOperationMs = 15000; // CONTROL 3/4 bound`, `core/clock_sync.h:72` `static constexpr std::size_t kWindow = 64;`).
- `noexcept` is used nowhere (0). `[[nodiscard]]` is used nowhere (0). `[[maybe_unused]]` once (`impl/resources/pros_usb_link.cpp`).
- `override` marks overrides (135 in src); `final` is rare (3). Base classes mark every overridable function `virtual`.

### Inconsistencies

- The missing `[[nodiscard]]` is the only gap that matters (section 6). Nothing else here needs a fix.

### Proposed rule

- K1. `const` by default for locals, parameters passed by reference, and member functions that do not mutate.
- K2. Named constants are `constexpr`, `kPascal`, and live in the narrowest scope (anonymous namespace or `static constexpr` member).
- K3. `[[nodiscard]]` as in E3.
- K4. `noexcept` is not required (no exceptions are used). Add it only to move operations of types stored in standard containers when a measurement shows it matters. (Decision for the lead.)
- K5. `override` on every override; `virtual` only in the base.

---

## 12. Default member initializers

### Dominant convention (evidence)

- 1,207 scalar data members in src have a default initializer; 10 do not. Example `core/diagnostics.h` `uint32_t runs = 0;`, `core/records.h:78` `SourceState state = SourceState::kNoDataYet;`.
- Enums default to their "unset" or "none" value (`ClockDomain::kUnset`, `SourceState::kNoDataYet`, `EstimateSource::kNone`).
- Config defaults are the struct's field initializers in many parsers: `runtime/system.cpp` `loop.getDouble("rate_hz", loop_rate_hz_, loop_rate_hz_, err)` (42 calls pass the current field as the default).

### Inconsistencies

- The 10 without initializers are static tables and ctor-set members (`capture/capture_bundle.cpp` `PinColumn`, `inspection/viewer_assets.h` `EmbeddedAsset`), plus `impl/resources/serial_links.h` `int64_t retry_us_;` and `impl/resources/libcamera_camera.cpp` `CpuRead::fd_`.
- Default duplicated between the struct and the parse call (86 calls pass a literal default): `impl/sensors/attitude_channel.cpp:33` `long stale_after_ms = 250;` and `:57` `getInt("stale_after_ms", 250, state->stale_after_ms, err)`; also `tracking_wheel_motion.cpp:262/:267` (0.005 twice).

### Proposed rule

- D1. Every scalar, pointer and enum member has a default member initializer. Aggregate constant tables are the only exception.
- D2. A configuration default is written once, as the field initializer, and parsers read `getX(key, out.field, out.field, err)` (or a helper that keeps the current value when the key is absent). No literal default at the call.

---

## 13. explicit

### Dominant convention (evidence)

- Single-argument constructors are `explicit`: 12 of 13 in src (`core/ids.h` `explicit TypedId(std::string v)`, `config/config_node.h:28` `explicit ConfigNode(const tinyxml2::XMLElement* e)`, `runtime/handoff.h:135` `explicit WorkerStats(std::string name)`). Tests follow suit (35 uses).

### Inconsistencies

- `impl/resources/synthetic_rig.cpp:201` `SyntheticRigExecutable(std::shared_ptr<SyntheticRig> rig) : rig_(std::move(rig)) {}`.

### Proposed rule

- X1. Single-argument constructors and conversion operators are `explicit` unless an implicit conversion is the point (and then a comment says so).

---

## 14. enum class and switch exhaustiveness

### Dominant convention (evidence)

- `enum class` with an explicit underlying type (`: uint8_t`) and `kPascal` enumerators, a trailing comment per enumerator: `core/function_status.h:12-16`, `core/records.h:37-42`. 30 in src, 2 plain enums.
- Each enum has a `...Name()` function that switches over every enumerator with no `default`, then returns `"unknown"` after the switch, so `-Wswitch` flags a missing case: `core/function_status.h:18-25` (trimmed) `switch (s) { case FunctionStatus::kOk: return "ok"; ... } return "unknown";`.

### Inconsistencies

- Wire enums arrive as `uint8_t`, so their switches need `default:`, and the same wire enum is stringified in several files with different spellings:
  - `capture/capture_bundle.cpp:158` `case DiagDirection::kTxAccepted: return "tx_accepted";` versus `diagnostics/instrumentation.cpp:49` `case DiagDirection::kTxAccepted: return "tx";`
  - `capture/capture_bundle.cpp:93` `case translagatr::kPicoImuReasonNoResponse: return "no_response";` versus `diagnostics/instrumentation.cpp:70` `... return "no response";` versus `inspection/inspection_document.cpp:278` `... return "no_response";`
  - `picoImuStateName` exists in both `capture/capture_bundle.cpp:78` and `inspection/inspection_document.cpp:263`; `pathModeName` in both `capture_bundle.cpp:69` and `inspection_document.cpp:254`.
- 51 `default:` labels in 75 src switches; most are over `uint8_t` wire codes.
- Plain enum `inspection/http_server.cpp:723` `enum ChoiceKind { ... }` passed as `int& kind`.

### Proposed rule

- EN1. `enum class` with an explicit underlying type. No plain enums.
- EN2. Switches over an `enum class` list every enumerator and have no `default`; the fallback return goes after the switch. Build with `-Wswitch` (on with `-Wall`).
- EN3. One `...Name()` per enum, defined next to the enum. Wire enums get theirs in `translaGATR` so Pi, Brain and Pico print the same text. Machine-readable names (CSV, JSON) and human text are two separate functions if both are needed.

---

## 15. std::variant and std::visit

### Dominant convention (evidence)

- One `std::variant` in src: `diagnostics/records.h:146` `using DiagPayload = std::variant<DiagRobotState, DiagPicoSensor, ... DiagEvent, DiagBytes>;`. No `std::visit` anywhere.
- Open-ended payloads use type erasure instead (`core/typed_payload.h` `TypedPayload::store<T>`/`get<T>`), which is the right tool for a plugin-extensible set.

### Inconsistencies

- `diagnostics/records.h` `struct DiagRecord { DiagKind kind = DiagKind::kEvent; ... DiagPayload payload; };` keeps a `kind` tag next to the variant, so the two can disagree. Consumers switch on `kind` and then `std::get_if` the matching alternative, guarded by a check that exists only because of the duplicate tag: `capture/capture_bundle.cpp:572` `if (!payloadMatches(r)) { ++mismatched_[k]; // a producer bug must never crash the recorder`, then `:585` `case DiagKind::kRobotState: robotState(r, *std::get_if<DiagRobotState>(&p), reset, source); break;`.

### Proposed rule

- V1. A closed set of alternatives is a `std::variant` without a separate tag; dispatch with `std::visit` and an overload set, or `std::get_if` in an if-chain. An open, plugin-extended set uses `TypedPayload`.

---

## 16. std::function and callbacks

### Dominant convention (evidence)

- Registered factories are `std::function` behind `using` aliases in the contract headers: `contracts/sensor.h:47` `using SensorMakeFunction = std::function<std::optional<SensorExecutable>(const ConfigNode&, SensorInitializationContext&, std::string& err)>;`.
- Resources and sensors are delivered as structs of callables rather than virtual classes: `contracts/sensor.h:30-37` `struct SensorExecutable { PayloadDescriptor outputPayload; std::function<PollResult(const ResourceMap&, const ExecutionContext&)> execute; std::function<void()> reset; };`, with state captured in a `shared_ptr` inside the lambda (`impl/sensors/attitude_channel.cpp`).
- Injected clocks and handlers: `std::function<int64_t()>` (7 uses), `inspection/http_server.h` handler callbacks.

### Inconsistencies

- Two mechanisms for "a configured stage implementation": captured `std::function` executables for resources and sensors, virtual classes (`Commands`, `RobotObservationFunction`, `StateEstimator`, `FieldEstimation`, `TargetResolution`, `Publishing`) for everything after. Both are documented and both work, but a reader has to learn both.
- `std::function<int64_t()>` is spelled out 7 times with no alias.

### Proposed rule

- F1. Callbacks and factories are `std::function` behind a named alias in the header that owns the contract.
- F2. One way to write a stage plugin. Recommended: the struct-of-callables form (it is the functional form the user prefers and needs no class), with a class only when the plugin holds invariants worth a type. (Decision for the lead: converting the six virtual contracts is a behavior-neutral but large change; the lighter version is to document both forms in the contracts pipeline doc.)

---

## 17. Templates

### Dominant convention (evidence)

- Few and small (28 `template <` in src): strong ids (`core/ids.h` `TypedId<Tag>`), the registry (`core/function_registry.h`), payload erasure (`core/typed_payload.h`), `runtime/handoff.h` `LatestSlot<T>`, typed bindings (`runtime/resource_catalog.h` `TypedOutputBinding<T>`), `inspection/json_writer.h` `field<T>`.
- No template metaprogramming beyond `std::any` and `std::type_index` in the registry.

### Inconsistencies

- One clever spot: `runtime/system.cpp:31` `template <typename T> struct Tag { using type = T; };` feeds a generic lambda at `runtime/system.cpp:259` `const auto buildSlot = [&](const char* slot_name, auto& target, auto make_tag, auto& context, std::string& label, FunctionKey& type) { using MakeFunction = typename decltype(make_tag)::type; ...`. Three explicit calls would read more plainly than a type-tag trick in the middle of the construction sequence.

### Proposed rule

- TP1. Templates for containers, strong types and type-safe erasure only. No tag-dispatch or generic lambdas where two or three explicit calls would do.

---

## 18. using aliases (user asked for more)

### Dominant convention (evidence)

- 38 `using X = ...` aliases in src: strong id types (`core/ids.h:38-50` `using SensorId = TypedId<struct SensorIdTag>;`), result maps (`core/records.h:102` `using SensorMap = std::unordered_map<SensorId, MeasurementRecord, SensorId::Hash>;`), factory signatures (`using SensorMakeFunction = ...`), `using Transform2D = Pose2D;` (`math/transforms.h`). No `typedef`.

### Where aliases are missing

- `std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>>` is spelled out 4 times (`runtime/system.h:303`, `runtime/system.h:456`, `runtime/system.cpp:1014`, `runtime/system.cpp:1221`).
- `std::function<int64_t()>` 7 times.
- Every `unordered_map` keyed by a typed id repeats the hash: `std::unordered_map<SensorId, uint64_t, SensorId::Hash>` (`runtime/sensor_stage.h:97`), 11 such spellings. A `std::hash` specialization for `TypedId` would drop the third argument everywhere.
- `std::shared_ptr<const ProfileBinding>` 11 times, `std::shared_ptr<const FieldMap>` 8 times.
- Typed ids exist but raw strings still carry ids in places: `contracts/slot_init.h:51` `std::vector<std::string> observation_functions;` (there is an `ObservationFunctionId`), `runtime/build_options.h:17` `std::map<std::string, std::string> replay;` (resource id to path).

### Proposed rule

- AL1. Name every type that is spelled more than twice or is longer than one line: `using FieldSnapshotPtr = std::shared_ptr<const FieldSnapshot>;`, `using HostClockFn = std::function<int64_t()>;`. Aliases live in the header that owns the underlying type.
- AL2. Ids are always their typed id, never a raw `std::string`.
- AL3. `using`, never `typedef`.

---

## 19. XML configuration parsing

### Dominant convention (evidence)

- One wrapper, `config/config_node.h` `ConfigNode`, a non-owning view with strict getters: absent attribute gives the default, malformed text, NaN or infinity is an error with the XML path (`config/config_node.h:8-10`). Required variants for calibration-critical values (`requireDouble` 83 uses, `requireAttr` 49).
- Strict schema checks: `onlyChildren` (27 calls), `onlyAttributes` (23), `atMostOne` (13).
- Ownership of the schema follows the plugin: the generic builder reads only `id` and `type`, and each factory parses its own subtree (`config/config_node.h:2-5`). Factories copy what they keep.
- Only `config/`, `runtime/system.cpp`, `runtime/brain_profile_builder.*` and `impl/publishing/field_documents.cpp` touch `tinyxml2` directly.

### Inconsistencies

- `runtime/system.cpp:36` has its own `checkChildren(parent, allowed, repeatable, err)` that duplicates `ConfigNode::onlyChildren` plus `atMostOne` (`config/config_node.h:68-72`).
- Defaults duplicated at the call site (section 12).
- `long` from `getInt` leaks into durations (section 5).
- Top-level section parsers are spread across `runtime/inspection_config.h`, `capture/capture_config.h`, `runtime/brain_profile_builder.cpp`, `runtime/pico_control_ref.h`, `impl/resources/cameras.cpp`. Per-plugin parsing is intended; the inspection parser in `runtime/` is the odd one out.
- 24 unbraced one-line `if` statements in `config/composition.cpp` alone, e.g. `config/composition.cpp:140` `if (!load(path, document, normalized, err)) return false;` (see section 24).

### Proposed rule

- X-1. All XML access goes through `ConfigNode`; `tinyxml2` appears only in `config/`.
- X-2. Every element a parser accepts is checked with `onlyChildren`/`onlyAttributes`; unknown children and attributes are errors, never ignored.
- X-3. Required means `require*`; optional means `get*` with the struct's own default (D2). Calibration-critical values are always required.
- X-4. A section's parser lives with the code that owns the section.
- X-5. Attribute names are snake_case with a unit suffix and match the C++ field name.

---

## 20. JSON writing

### Dominant convention (evidence)

- One streaming writer, `inspection/json_writer.h` `class JsonWriter`: escapes strings, writes non-finite numbers as `null`, inserts commas. Used by inspection, capture and diagnostics (10 src files besides its own header).

### Inconsistencies

- Hand-concatenated JSON next to the writer: `inspection/inspection_service.cpp:560-564` (trimmed) `std::string body = "{\"ok\":true,\"running\":"; ... body += ",\"contract\":\"" + std::string(kInspectionContract) + "\"}";`, `capture/capture_http.cpp:126` `return json(200, "{\"ok\":true}");`.
- JSON checked by substring: `inspection/inspection_document.cpp:1780` `return w.str().find("\"available\":true") != std::string::npos;`.
- The shared writer lives in `inspection/` but capture and diagnostics include it (`capture/capture_service.cpp` `#include "inspection/json_writer.h"`).
- `tools/perf/perf_json.h` has a second JSON implementation (reader plus `toJson`/`jsonQuote`).

### Proposed rule

- J1. All JSON output goes through `JsonWriter`; no string-built JSON, even for one-line replies. Code never reads its own JSON back as text.
- J2. Shared utilities used by more than one subsystem live in a neutral folder (for example `src/util/` or `core/`), not in one consumer's folder.
- J3. Tools may keep their own reader, but writing uses the same writer.

---

## 21. Logging and events

### Dominant convention (evidence)

- Library code never prints. `printf`/`fprintf` appear only in `app/main.cpp` (18), `app/field_refs.cpp` (4) and the abort path in `runtime/register_all.h:36`. No `std::cout`/`std::cerr`.
- Build-time problems: `std::vector<std::string>* warnings` pushed with the XML path; `app/main.cpp:107-109` prints them.
- Runtime state: `FunctionStatus` plus a `diagnostic` string in every result, `SourceState` per source, counters in `Diagnostics`/`LinkStats`.
- Lifecycle events: `System::noteEvent` (`runtime/system.cpp:961`) posts to the `DiagnosticsHub` and a bounded ring of `RuntimeEvent`; `app/main.cpp:145-152` prints them as `event: ...` lines.
- High-rate diagnostic records: `DiagnosticsHub` posts, drained by the capture recorder on its own thread.

### Inconsistencies

- None significant. The event text is free prose (for example `runtime/system.cpp` `"profile " + hexId(id) + " dropped: superseded before the boundary"`), which is fine for humans but not stable for tools.

### Proposed rule

- L1. Only `app/` writes to stdout or stderr. Libraries report through return values (`err`, warnings, status plus diagnostic), counters, events and hub records.
- L2. Event and warning text is short, lowercase, and names the thing (id or XML path) first.

---

## 22. Magic numbers

### Dominant convention (evidence)

- Behavioral tunables are named constants or configuration: 77 `constexpr k...` in src (`runtime/system.h:189-190` `kPicoOperationMs = 15000`, `kWheelFreshMs = 150`; `core/clock_sync.h:72-73` `kWindow = 64`, `kMinObservations = 8`), plus XML attributes for anything a robot might tune.
- About 320 non-trivial numeric literals remain in src code (after excluding 0, 1, 2, powers of ten, unit factors and default member initializers). Most are protocol constants (`inspection/http_server.cpp` 58, zip format in `capture/zip_writer.cpp` 19) or print precision (`capture/capture_bundle.cpp:692` `row_.f(s.odom_x_m, 6)`).

### Inconsistencies

- Behavior-bearing literals inline:
  - `runtime/handoff.h:159` `0.9 * snapshot_.mean_cycle_ms + 0.1 * work_ms`
  - `runtime/handoff.h:200` `if (since_window_s > 2.0 || (snapshot_.rate_hz == 0.0 && since_window_s > 0.05))`
  - `impl/localization/brain_imu_wheel_bench.cpp:349` `std::fabs(motion.dtheta_rad) > 12.0 * motion.dt_s + 0.1`
  - `impl/resources/pico_telemetry.cpp:172` `now.ms - h.samples.back().first < 50`
  - `app/main.cpp:176` `std::chrono::milliseconds(20)`, `runtime/system.cpp:1439` `std::chrono::milliseconds(100)`
- Integer limits written out twice: `runtime/system.cpp:670` and `impl/publishing/brain_link_publisher.cpp:39` `std::clamp(mm, -2147483648.0, 2147483647.0)`.

### Proposed rule

- M1. Any number that changes behavior (thresholds, gains, periods, timeouts, window sizes) is a named `constexpr` with a unit suffix, or a configuration attribute. Literal numbers are fine for protocol fields with a comment naming the spec, for formatting precision, and for unit conversions through named helpers.
- M2. Type limits come from `std::numeric_limits`.

---

## 23. Comments versus the user's comment rules

User rules (memory `feedback_comment_style.md`): plain ASCII, no emojis, no em dashes, no symbols off a standard keyboard; terse, self-describing labels; not AI-ish; no over-explaining; casual grammar fine.

### What complies

- Plain ASCII: 0 non-ASCII characters in src, tests or perf.
- Inline and trailing comments are mostly short labels: `core/diagnostics.h` `uint32_t stale = 0; // answered kResultStale`, `core/function_status.h:14` `kNoData, // required data absent or stale this cycle, skipped cleanly`, `tests/CMakeLists.txt` `# Test files.`.
- Comment density is modest: 3,793 comment lines to 29,735 code lines in src (0.13); tests 0.06.

### What does not

- File headers are long and explanatory. Median 6.5 lines, but 49 src files have 15 or more and 20 have 30 or more:
  - `runtime/system.h` 83 lines
  - `runtime/brain_profile_builder.h` 65
  - `impl/localization/tracking_wheel_motion.h` 60
  - `impl/commands/brain_link_commands.h` 60
  - `impl/localization/weighted_planar_fusion.h` 58
  - `impl/publishing/brain_link_publisher.h` 48
  They narrate design rationale, for example `core/function_registry.h:11-13` "Holding the registry grants no execution authority: the pipeline coordinator decides which category of factory it retrieves and when the result may run." and `resources/serial_link.h:2` "There is deliberately no universal DataBus abstraction".
- The same text lives in three places: the command-line usage is in the `app/main.cpp:1-17` header, in the `--help` string at `app/main.cpp:80-86`, and in `README.md` "Run".
- 105 `// ---- section ----` banners (for example `runtime/system.cpp:1020` `// ---- cycles ----...`); a banner in a 1,450-line file is a sign the file wants splitting.
- 17 src comments use ASCII arrows (`->`); allowed by "on a standard keyboard", but the memory rule lists "arrows" among symbols to avoid. (Decision for the lead: ASCII `->` is keyboard-typable; the rule most likely meant Unicode arrows.)

### Proposed rule

- CM1. A file header is at most 3 to 5 lines: what the file is, and the one rule a reader must not break. Pipeline narrative, rationale and XML schemas move to the folder's PIPELINE doc (section 27).
- CM2. Comments are noun-phrase labels or one short sentence. Say what, not why, unless the why would surprise a competent reader.
- CM3. Say a thing once. Usage text lives in the `--help` string and is linked from the README.
- CM4. Plain ASCII only. `->` allowed in pseudo-code; no Unicode arrows, dashes or quotes.
- CM5. Decide the `// <file name>` first line: it restates the file name (the user's "restates the obvious" rule argues to drop it), but it is universal today (185 of 198). Recommendation: drop it when files are next touched, never add it to new files.

---

## 24. Formatting and braces

### Dominant convention (evidence)

- The code looks like the intended style: 4-space indent, 100 columns, attached braces except namespaces, left pointer alignment, aligned consecutive declarations and assignments.

### Inconsistencies

- Not machine-enforced. With the pinned repo `.clang-format` and clang-format 22.1.8, 235 of 249 naviGATR C++ files would change (src 184, tests 39, perf 12), 14,076 diff lines. The biggest single difference is trailing-comment spacing: the code mostly uses three spaces before `//`, the config gives one (`core/time.h` `kUnset = 0,   // default constructed` becomes `kUnset = 0, // default constructed`). Adding `SpacesBeforeTrailingComments: 3` alone does not fix it (248 files would still change), so the tree is mixed, not consistently formatted with some other config.
- Brain: 102 of 135 files change; translaGATR: 8 of 8.
- Braces on every `if`: 66 one-line unbraced `if`/`for`/`while` bodies across src, tests and perf, concentrated in `config/composition.cpp` (24), `impl/resources/pros_usb_link.cpp` (21) and `resources/brain_imu_bench.h` (4). Example `impl/resources/pros_usb_link.cpp:79` `if (window.missed(nowUs())) return {false, true, false, false, "USB reply expired"};`.

### Proposed rule

- FM1. clang-format 22.1.8 with the repo `.clang-format` is the only formatting authority; CI fails on any diff. Decide the trailing-comment spacing once in the config (the code's habit is 3), reformat the whole tree in one commit, and add that commit to `.git-blame-ignore-revs`.
- FM2. Braces on every control statement. clang-format 22 can enforce it with `InsertBraces: true`, which makes the rule mechanical.

---

## 25. Function size and complexity (lizard)

- src: 1,294 functions, median length 8 lines. 84 functions over 60 lines, 41 over 100, 86 with CCN over 15, 40 over 25; 22 take 6 or more parameters.
- Largest:
  - `impl/localization/tracking_wheel_motion.cpp:510` `TrackingWheelMotion::run`: 352 lines, CCN 105
  - `inspection/inspection_document.cpp:1140` `writeSnapshotBody`: 398 lines, CCN 41, 7 parameters
  - `impl/localization/tracking_wheel_motion.cpp:105` `TrackingWheelMotion::create`: 286 lines, CCN 67
  - `impl/world_estimation/tag_mount_association.cpp:148` `TagMountAssociation::run`: 284 lines, CCN 53
  - `impl/resources/synthetic_rig.cpp:398` `make_synthetic_rig`: 252 lines, CCN 79
  - `runtime/system.cpp:148` `System::build`: 204 lines, CCN 31
- Factories (`create`/`make_*`) are the long functions because they parse, validate and wire in one body.
- tests: 1,146 functions, median 15 lines; the longest are XML builder helpers (`tests/vision_targets_gtest.cpp:140` `configXml`, 189 lines) and a 315-line `TEST` in `tests/capture_gtest.cpp:1193`.

### Proposed rule

- FN1. Soft limits, checked in CI as warnings: 60 lines and CCN 15 per function, 5 parameters. Over the limit needs a reason in review. A factory splits into parse (XML to config struct), validate (config struct to errors), and build (config struct to object); each is a free function.
- FN2. More than 5 parameters becomes an input struct, the same pattern the stage contracts already use.

---

## 26. Stage contracts: the standard INPUT/OUTPUT the user wants

### Dominant convention (evidence)

- Seven contracts share one shape, `XOutput run(const XInput& in)` plus `virtual void reset() {}`:
  - `contracts/commands.h:38` `virtual CommandsOutput run(const CommandsInput& in) = 0;`
  - `contracts/field_estimation.h:63` `virtual FieldEstimationOutput run(const FieldEstimationInput& in) = 0;`
  - `contracts/localization.h:164` `virtual StateEstimatorOutput run(const StateEstimatorInput& in) = 0;`
  - `contracts/perception.h:39`, `contracts/association.h:44`, `contracts/publishing.h:53`, `contracts/target_resolution.h:51`
- Inputs are structs of `const&` to the standard maps plus an `ExecutionContext`; outputs are value structs with a `FunctionStatus status` and a `std::string diagnostic`.
- Factories share one signature shape: `(const ConfigNode&, <Stage>InitializationContext&, std::string& err)`.
- Results travel in typed maps with standard envelopes (`core/records.h`: `ResourceMap`, `SensorMap`, `ObservationMap`, `AssociationMap`), keyed by typed ids; payloads are `TypedPayload` checked by `PayloadDescriptor` at build.

This is exactly the "existing systems keep their shape and have standard INPUT/OUTPUT so different teams can work inside them" the user asks for.

### Inconsistencies

- `contracts/localization.h:84` `virtual FunctionStatus run(const RobotObservationInput& in, RobotObservationMap& out) = 0;` returns status and writes an out parameter, unlike the other seven.
- Resources and sensors use struct-of-callables executables returning `ResourcePollResult`/`PollResult` rather than `run(Input) -> Output` (section 16).
- Five initialization contexts with overlapping fields (`ResourceInitializationContext`, `SensorInitializationContext`, `RobotObservationInitializationContext`, `StateEstimatorInitializationContext`, `SlotInitializationContext`) and duplicated validation (`requireObservation` twice).
- Stage builder signatures differ in parameter order and content:
  - `make_resources(node, functions, options, warnings, err)`
  - `make_sensors(node, functions, store, resource_outputs, warnings, err)`
  - `make_localization(node, functions, sensors, resources, warnings, err)`
  - `make_world_estimation(node, functions, sensors, resources, warnings, err)`
  Two of them take the store first and the catalog second; the other two take the catalog first.

### Proposed rule

- IO1. Every stage and sub-stage exposes exactly `Output run(const Input& in)` and `void reset()`. `Input` holds `const&` to what the stage reads plus the `ExecutionContext`; `Output` is a value with `status` and `diagnostic`. No out parameters on a contract.
- IO2. Every factory has the shape `std::optional<T>`/`std::unique_ptr<T> make<Impl>(const ConfigNode&, const BuildContext&, std::string& err)`, with one `BuildContext` that grows as stages are built. Stage builders take their arguments in the same order.
- IO3. The Input and Output structs of each stage are the documented seam: they are listed in the stage's pipeline doc, and changing them is a reviewed interface change.

---

## 27. Structure findings behind the naviGATR gripe

The user: "its not clear where the main pipeline is or where the make pipeline shit is". What a new reader has to do today:

1. `app/main.cpp:93-98` registers factories and calls `navigatr::System::buildFromFile(...)`.
2. `runtime/system.h` has the pipeline in an 84-line comment; the class body follows (47 public and 31 private member functions, about 63 data members).
3. Construction is `System::build`, `runtime/system.cpp:148-351`: it calls `make_resources` (`runtime/resource_stage.cpp`), `make_sensors` (`runtime/sensor_stage.cpp`), a generic `buildSlot` lambda for three slots, `make_localization` (`runtime/localization_stage.cpp`), `make_world_estimation` (`runtime/world_estimation_stage.cpp`).
4. The per-cycle pipeline is `System::step` (`:1022`), `estimationCycle` (`:1033`), `reportingCycle` (`:1066`), `fieldCycle` (`:1106`), and the worker loops `estimationWorker` (`:1411`) and `fieldWorker` (`:1436`), after 570 lines of Brain profile, Pico CONTROL and recovery code (`:419-990`).
5. `runtime/` (26 files) mixes the generic pipeline (stage builders, executors, handoff) with specific features (`brain_profile_builder`, `sensor_loss`, `stationary_precheck`, `pico_control_ref.h`, `inspection_config.h`, `inspection_state.h`).

Layering, from include edges between top-level folders:

- The generic layer includes concrete implementations:
  - `runtime/resource_stage.cpp:6` `#include "impl/resources/serial_links.h"` (for the `--replay` override)
  - `runtime/pico_control_ref.h:12` `#include "impl/resources/pico_telemetry.h"`
  - `diagnostics/instrumentation.cpp:17` and `inspection/inspection_document.cpp:17-18` include `impl/...` headers
  That is the opposite of "generic pipeline, specific implementations plugged in".
- Implementations include `runtime/` (register_all.h, sensor_catalog.h, resource_catalog.h), so `runtime` and `impl` depend on each other.
- `capture` and `runtime` include each other (System owns the recorder; the recorder includes `runtime/system.h`). `capture` and `inspection` include each other.
- `math/camera_model.h:15` includes `resources/camera.h`; `transport/half_duplex.h:10` includes `resources/serial_link.h`; `state/robot_state_feed.cpp:5` includes `diagnostics/hub.h`.
- Contracts live in two folders: `contracts/` (stage contracts) and `resources/` (device contracts such as `camera.h`, `serial_link.h`, `spi_bus.h`, plus a mailbox struct `brain_imu_bench.h`).

### Proposed rules (feed the application plan)

- P1. Every component and every sub-pipeline folder has a `PIPELINE.md` of at most one screen:
  - the stages in order
  - each stage's Input and Output types
  - which thread runs it
  - where it is built (file and function)
  - where it runs (file and function)
  The whole-system doc at the repo root links each component's. Deep rationale stays in `docs/`.
- P2. The main pipeline is findable by file name: one file that builds it top to bottom (for example `pipeline/build.cpp`) and one that runs a cycle top to bottom (for example `pipeline/cycle.cpp`). Neither holds feature logic; features (Brain profile, Pico control, recovery, capture) live in their own files and are called from there.
- P3. Dependency direction is one way: `core` and `math` at the bottom; contracts; the generic pipeline (`pipeline/` or `runtime/`); implementations (`impl/`) that register into it; `app/` at the top. Generic code never includes `impl/`. A test in CI can check this with a small include scan.
- P4. One folder for contracts. Device contracts and stage contracts both sit under `contracts/` (or each stage folder owns its contract header); `resources/` is either contracts or implementations, not both.
- P5. Folder names match the stage names in the pipeline doc (section 1, N4).

---

## 28. Tests

### Dominant convention (evidence)

- One test executable per component, test files named `*_gtest.cpp` and listed explicitly: `tests/CMakeLists.txt` `set(TEST_SOURCES command_line_gtest.cpp ... capture_gtest.cpp)`, linked with `GTest::gtest_main`, discovered with `gtest_discover_tests`. System gtest first, pinned FetchContent v1.14.0 fallback.
- `TEST(Suite, BehaviorSentence)`: 625 `TEST`, 2 `TEST_F`, 0 `TEST_P`. Names are PascalCase sentences stating behavior, no underscores: `TEST(BrainImuBench, DuplicateInvalidStaleAndRestartDoNotBridgeTravel)`, `TEST(Workers, StopAndResetWakeAWorkerWaitingForALongPeriod)`.
- Each file: a header comment saying what is covered, `using namespace navigatr;`, helpers and fakes in an anonymous namespace.
- Fakes are hand-written subclasses of the contracts, named `Fake*` or `Scripted*` (`FakePort`, `FakePico`, `ScriptedCamera`, `ScriptedDetector`, `ScriptedLocalization`). No gMock (`BUILD_GMOCK OFF`).
- Tests drive the real System from inline XML (362 raw string literals; 64 `buildFromString`, 14 `buildFromFile` against `NAVIGATR_CONFIG_DIR`).
- Time is injected where possible: 89 `step(hostTime(...))` calls step the pipeline on synthetic time with no sleeping.
- Threaded tests poll against a deadline instead of asserting exact counts, stated at `tests/workers_gtest.cpp:6-9` ("Every timing assertion polls against a deadline or accepts a range"), helper `tests/workers_gtest.cpp:47` `bool waitFor(Ms deadline, const std::function<bool()>& pred)`.
- `EXPECT_*` 4,666, `ASSERT_*` 1,089.

### Inconsistencies

- Code under test mixed into the test list: `tests/CMakeLists.txt:21` `../src/app/command_line.cpp` sits inside `set(TEST_SOURCES ...)` under `# Test files.`; there is no `CODE_UNDER_TEST` list (the memory convention names one).
- A shared helper header in `tests/`: `tests/inspection_test_client.h` (the memory rule says `tests/` holds only test mains). Because the rule forbids shared helpers, helpers are copied instead:
  - `takeReplies` defined in 4 files
  - `rigXml` in 3
  - `rigConfig` in 2 (161 and 163 lines, `tests/workers_gtest.cpp:162`, `tests/synthetic_rig_gtest.cpp:28`)
  - `ScriptedDetector`, `ScriptedCamera` and `FakePico` each defined twice
- Suite and file names do not map one to one (92 suites in 38 files). `SensorLoss` and `StationaryPrecheck` (for `runtime/sensor_loss.cpp`, `runtime/stationary_precheck.cpp`) live in `tests/brain_profile_gtest.cpp`; `DeviceToHostClock` in `tests/localization_gtest.cpp`; `JsonWriter` in `tests/inspection_document_gtest.cpp`.
- Very large test files: `tests/brain_profile_gtest.cpp` (2,926 lines), `tests/capture_gtest.cpp` (2,119), `tests/brain_io_gtest.cpp` (1,818).
- Real sleeps where a clock could be injected: 45 `sleep_for` in tests, up to `tests/workers_gtest.cpp:573` `sleep_for(Ms(2200))`, because `WorkerStats` reads `steady_clock` directly (section 5).
- 4 test files skip the header comment (`tests/brain_imu_bench_gtest.cpp`, `camera_capture_gtest.cpp`, `command_line_gtest.cpp`, `pros_usb_link_gtest.cpp`).
- `tools/perf` tests are not gtest: `tools/perf/perf_selftest.cpp` is a hand-rolled `check()` main, and `tools/perf/CMakeLists.txt` adds 8 `add_test` cases that match console output with `PASS_REGULAR_EXPRESSION`, which break on any wording change.

### Proposed rule

- TS1. Test files are `<unit>_gtest.cpp`, where `<unit>` is the source file or folder under test; a reader finds the tests for `runtime/sensor_loss.cpp` in `sensor_loss_gtest.cpp`. Suite name = the class or function under test.
- TS2. `TEST(Suite, BehaviorSentence)` in PascalCase, no underscores. `TEST_F` only when three or more tests share non-trivial setup.
- TS3. Two explicit lists in CMake: `TEST_SOURCES` (test files only) and `CODE_UNDER_TEST` (sources compiled only for tests). No globbing.
- TS4. Shared test support is allowed in one place, `tests/support/` (headers and sources for fakes, XML builders, polling helpers), listed explicitly. (Decision for the lead: this relaxes the memory rule "tests/ holds only test mains", which currently causes copy-paste of fakes and helpers.)
- TS5. Fakes are hand-written, named `Fake<Contract>` (canned behavior) or `Scripted<Contract>` (replays a list), and live in `tests/support/` once used by two files.
- TS6. No `sleep_for` to wait for a result: step synthetic time or poll with a deadline helper. A test that needs real threads says so in its name suffix (`...Threaded`) so CI can label and time it.
- TS7. Tools test their pure functions with gtest like everything else; output-regex tests only for the final formatted report, one per report section.

Pi vs Brain: Brain components each have their own `tests/` with `*_gtest.cpp` per source file (`brain/actuGATR/tests/pid_gtest.cpp` for `src/pid.cpp`), close to TS1 already. naviGATR groups by feature.

---

## 29. tools/perf C++

- Namespace `navigatr::perf` (the only nested functional namespace in the component). Tool mains use `using namespace navigatr::perf;` at file scope (`tools/perf/cdp_bench.cpp:39`).
- Error form `std::string* err = nullptr` (`tools/perf/perf_json.h`), unlike src's required `std::string& err`.
- Output with `printf` (76 uses), which is correct for a CLI tool.
- Own JSON reader and writer, own test harness (section 28).
- Formatting: all 12 files differ from the pinned clang-format.
- Proposed: tools follow the same rules as src except that they may print and may `using namespace` their own component in mains.

---

## 30. Pi versus Brain: where conventions differ

| Topic | naviGATR (Pi) | Brain (`brain/`) |
|---|---|---|
| Layout | one `src/` with layer folders; headers next to sources; includes rooted at `src/` | per component `include/<component>/`, `src/`, `pros/`, `sim/`, `tests/`; includes rooted at `include/` |
| Namespaces | one flat `navigatr` for 200 files | one per component: `actugatr` (35), `communigatr` (39), `investigatr` (16); `gatr2_robot`, `robot_config` for robot data |
| Errors | `optional`/`unique_ptr`/`bool` + `std::string& err` (252) | `bool` and result enums; 0 `std::string& err` |
| Time | `MonotonicTime` with clock domain, `int64_t` ms/us | `uint32_t *_ms` from PROS millis, no domain type |
| Sharing | `shared_ptr<const T>` snapshots (154 shared_ptr) | 0 `shared_ptr`, 35 `unique_ptr` |
| Threads | `std::thread`, `std::mutex`, `std::condition_variable` | PROS tasks, `pros::Mutex` |
| Naming | same: PascalCase types, camelCase functions, snake_case fields, `trailing_` members, `kPascal` | same |
| Enums | `enum class` (30), 2 plain | `enum class` (44), 2 plain |
| Tests | feature-grouped files, one executable | per-source `*_gtest.cpp` per component |
| Formatting | 235 of 249 files off the pinned config | 102 of 135 files off |
| Language level | C++17, but `target_compile_features(navigatr_core PUBLIC cxx_std_20)` when libcamera is on (`CMakeLists.txt:197`) | C++17 (PROS) |

Standard implications:

- Naming, enums, struct/class, comments and formatting rules can be identical across Pi, Brain and Pico.
- Error, time, threading and allocation rules need a per-target clause: Pi (hosted Linux, strings and heap fine), Brain (PROS, bounded heap, no strings in hot paths), Pico (bare metal).
- Layout: pick one rule per kind of component. A reusable library (translaGATR, communiGATR) exposes `include/<name>/`; an executable (naviGATR) may keep one `src/` tree but must still be readable from folder names (section 27).

---

## 31. Decisions to flag to the lead

1. Warnings: every naviGATR target builds with `-Wall -Wextra` and no `-Werror` (`CMakeLists.txt:201`, `:221`, `:230`; `tests/CMakeLists.txt`; `tools/perf/CMakeLists.txt`). Memory says `-Werror` everywhere; the user said this session to drop it. Needs one explicit rule.
2. C++ level: naviGATR needs C++20 when built with libcamera (`CMakeLists.txt:197`). The standard should say "C++17 language features only, C++20 mode allowed where a dependency's headers demand it".
3. `make_*`/`register_*` snake_case: keep as a named exception or rename (N1).
4. Strict struct rule: allow `const` query helpers on structs or not (S1).
5. Nested namespaces per subsystem (NS1).
6. `// <file name>` first line: keep or drop (CM5).
7. Shared test support folder (TS4) versus the memory rule.
8. Single-implementation interfaces: `SpiBus`/`SpiDevice` have zero production implementations; `CaptureService`, `Perception` and `Association` have one and no test fake. Remove or justify (S3).
9. Stage plugin form: struct-of-callables versus virtual class (F2).
10. Language purity: naviGATR contains a C file (`src/impl/resources/apriltag_c.c`, a C shim over the vendored AprilTag detector) and Python under `tools/` (`tools/calibrate_camera.py`, `tools/viewer/*.py`). The memory rule says "naviGATR is C++ only; exceptions need justification". The C shim is the kind of exception that should be listed by name.
11. Trailing-comment spacing and brace insertion in `.clang-format` (FM1, FM2), then one reformat commit.
