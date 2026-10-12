# Survey: readability, type aliases and named types

Scope: `brain/` (actuGATR, communiGATR, investiGATR, robot, locaGATR,
operaGATR; vendored PROS headers excluded), `pi/naviGATR/src`,
`translaGATR/`, `pico/aggreGATR/src`. 330 first-party source files, 42 test
files. Read-only; nothing in the repo changed.

Driving words from the user: "using aliasing is great", "use struct for
attribute only things", "Readability is king, if something is slightly
slower but more readable, readable wins everyday".

Related surveys: `cpp-brain.md` section 3 (aliases, Brain side) and
`cpp-pi.md` sections 1, 2, 5 (naming, struct/class, integer and time types)
cover overlapping ground; `complexity-bugs.md` D8 covers duplicated
freshness limits; `docs.md` DOC-19 proposes the glossary file. This survey
goes deeper on aliases and named types and proposes the vocabulary itself.

## 0. Method and caveats

- Evidence is `path:line` plus a short quote from the git tree. Quotes are
  exact unless marked "(trimmed)". "After" snippets are proposals, not code
  that exists.
- Paths use git's spelling `pi/naviGATR`. On this checkout the folder shows
  as `pi/navigatr` on disk (the case-only rename in `docs/naming.md`), so
  some tool output prints the lowercase form.
- Counts come from grep over the file list above and lizard 1.24.0
  (scratch venv). Regex counts are approximate; each one used in an argument
  was spot-checked.
- Line numbers are as of 2026-09-28, working tree clean at `ae457ee`.

## 1. Headline findings

1. **Aliases exist but unevenly.** 51 `using X =` alias declarations in
   non-test code: investiGATR (units, ids) and naviGATR `core/` (typed ids,
   record maps, factory signatures) use them well; translaGATR and Pico src
   have none; communiGATR has four.
2. **Wire identities are raw integers everywhere.** `session` is spelled
   `uint32_t` 33 times, `request_id` `uint16_t` 17 times, `pi_instance` 18
   times, `boot_id` 12 times, never with a name for the concept.
3. **Widths already drifted.** `odometry_epoch` is `uint32_t` in 6
   declarations and `uint64_t` in 10; `anchor_revision` 6 and 5. A sensor
   mask is `uint16_t` on the wire and `uint8_t` in a diagnostic record,
   narrowed by a cast, with a comment naming a type that does not exist
   (section 3, N22).
4. **Comments stand in for types.** 17 integer fields carry a comment naming
   the enum they hold (`uint8_t result = 0; // translagatr::BrainResult`).
   The enum types exist and cost nothing to use.
5. **Out-parameters and parallel data hide structure.** The (reason, detail)
   pair is two `uint8_t&` out-params in 6 signatures across 3 components;
   x/y/z and roll/pitch/yaw travel as 3 to 7 loose doubles; `.first` /
   `.second` carry meanings like "(host ms, counts)" known only from a
   comment.
6. **Vocabulary collides.** "session" names two different things in one JSON
   document; "frame" has four meanings; "generation" five; placement is
   also called "init" and "SET_POSE"; `PlacementStatus::state` holds a
   `PlacementResult` next to a field called `result`.
7. **Bool literals at call sites** hide meaning in about 40 call sites that
   are not setters (`applyLinkMonitors(false, false)`).
8. **Long functions**: 100 functions of 60+ NLOC, 31 with cyclomatic
   complexity over 30; the worst is 310 NLOC at CCN 105.
9. **Modern C++17 readability features are unused where they fit**: 0
   structured bindings against 44 `kv.first` / `kv.second` loops; 53
   `std::optional` against 59 `bool have_x` + value pairs.

## 2. Where aliases help: before and after

41 cases, grouped. Each names the proposed alias and where it would live.

### 2.1 Integer identities and counters (wire vocabulary)

These are wire concepts, so the alias belongs in `translaGATR/frames.h`
next to the struct that carries them; every side then uses the same name.

**A1. `SessionId`**
- `translaGATR/frames.h:316` `uint32_t session    = 0; // 0 in HELLO`
- `brain/communiGATR/include/communigatr/client.h:169` `uint32_t          session     = 0;`
- `pi/naviGATR/src/state/command_state.h:87` `uint32_t session = 0;   // current brain session, 0 = none`
- 33 declarations, all `uint32_t`.
- After: `using SessionId = uint32_t; // brain link session, 0 = none`, then
  `SessionId session = 0;`.

**A2. `RequestId`**
- `translaGATR/frames.h:317` `uint16_t request_id = 0; // per brain boot, 1..65535, never 0`
- `translaGATR/frames.h:485` `uint16_t request_id     = 0; // never 0`
- `brain/communiGATR/include/communigatr/client.h:439` `uint16_t            next_request_id_  = 1;`
- 17 declarations. After: `using RequestId = uint16_t; // 0 = none`.

**A3. `PiInstanceId`**
- `translaGATR/frames.h:378` `uint32_t pi_instance = 0; // random nonzero id per Pi process start and reset`
- `pi/naviGATR/src/state/command_state.h:90` `uint32_t      pi_instance = 0;       // this Pi on the link, 0 = no brain link`
- `pi/naviGATR/src/impl/commands/brain_link_commands.h:137` `uint32_t     pi_instance_ = 0;`
- 18 declarations. After: `using PiInstanceId = uint32_t;`.

**A4. `OdometryEpoch`, `AnchorRevision` (and the width split)**
- `translaGATR/frames.h:359-360` `uint32_t odometry_epoch  = 0; // low 32 bits` / `uint32_t anchor_revision = 0; // low 32 bits`
- `pi/naviGATR/src/state/robot_state.h:48-49` `uint64_t odometry_epoch  = 0;` / `uint64_t anchor_revision = 0;`
- `brain/communiGATR/include/communigatr/client.h:86-87` `uint32_t        odometry_epoch  = 0;`
- After: `translagatr::OdometryEpoch = uint32_t` (wire, low bits) and
  `navigatr::OdometryEpoch = uint64_t` (full count). The one place that
  converts is then visible as a type change, not a silent narrowing.

**A5. `BootId`, `AcquisitionEpoch`, `ImuEpoch`**
- `translaGATR/frames.h:78-80` `uint16_t boot_id;` / `uint8_t  acq_epoch;` / `uint8_t  imu_epoch;`
- `pico/aggreGATR/src/imu.h:20` `uint8_t  epoch    = 0; // (re)initializations started this boot`
- `pi/naviGATR/src/runtime/system.h:150` `uint8_t       acq_epoch = 0;   // at submit, RestartAcquisition`
- After: `using BootId = uint16_t; using AcquisitionEpoch = uint8_t; using ImuEpoch = uint8_t;`.

**A6. `DocumentId` (crc32 identities)**
- `translaGATR/frames.h:329` `uint32_t profile_id = 0; // PROFILE_WRITE, PROFILE_APPLY: crc32 of the document`
- `translaGATR/frames.h:365-367` `profile_id`, `map_id`, `estimate_id`, all `uint32_t`
- `brain/communiGATR/include/communigatr/client.h:155` `uint32_t    id       = 0; // crc32 of the configured document`
- `pi/naviGATR/src/contracts/brain_profile.h:29` `uint32_t               id = 0;`
- investiGATR already has `using MapId = uint32_t; // crc32 of the Pi field map document, 0 = none`
  (`brain/investiGATR/include/investigatr/field.h:16`).
- After: `using DocumentId = uint32_t; // crc32 of a link document, 0 = none` in
  `translaGATR/link_documents.h`.

**A7. `CommandId` (one alias exists, the wire and Pi ignore it)**
- `brain/investiGATR/include/investigatr/path.h:15` `using CommandId = uint32_t; // 0 = none`
- `brain/communiGATR/include/communigatr/client.h:293` `bool reportPath(uint32_t command_id, uint8_t path_mode, const translagatr::PathPoint* points,`
- `translaGATR/frames.h:291`, `:343` `uint32_t command_id`
- After: communiGATR takes `investigatr::CommandId` (it already depends on
  investiGATR); translaGATR gets its own `CommandId`.

**A8. `FieldGeneration`**
- `brain/investiGATR/include/investigatr/field.h:60` `uint32_t                 generation = 0; // changes with map or estimates, 0 = none`
- `brain/communiGATR/include/communigatr/client.h:177` `uint32_t             generation = 0; // per published pair, 0 = none yet`
- The sibling concept is already aliased: `using FrameGeneration = uint32_t;` (`geometry.h:18`).
- After: `using FieldGeneration = uint32_t;` beside it.

**A9. `PicoRequestHandle`**
- `pi/naviGATR/src/resources/pico_control.h:65` `virtual uint32_t submit(uint8_t op, uint8_t arg, MonotonicTime now, double timeout_s) = 0;`
- `pi/naviGATR/src/resources/pico_control.h:67` `virtual PicoRequestStatus request(uint32_t handle) const = 0;`
- `pi/naviGATR/src/runtime/system.h:148` `uint32_t      handle = 0;`
- After: `using PicoRequestHandle = uint32_t; // 0 = refused`, the same shape as
  communiGATR's `using PlacementTicket = uint32_t; // 0 = none or refused`
  (`client.h:71`).

**A10. `DiagSourceId` (one concept, four member names)**
- `pi/naviGATR/src/diagnostics/hub.h:47` `uint16_t    sourceId(const std::string& name);`
- `pi/naviGATR/src/runtime/system.h:406` `uint16_t                        event_source_ = 0;   // hub source id of noteEvent`
- `pi/naviGATR/src/state/robot_state_feed.h:125` `uint16_t                        hub_source_ = 0;`
- `pi/naviGATR/src/diagnostics/records.h:152` `uint16_t    source  = 0; // DiagnosticsHub::sourceId of the producer`
- After: `using DiagSourceId = uint16_t; // 0 = none` in `diagnostics/hub.h`,
  and one member name, `source_id_`.

**A11. `SensorMask`**
- `translaGATR/frames.h:72` `uint16_t mask;`
- `pi/naviGATR/src/diagnostics/records.h:79` `uint8_t  mask       = 0; // translagatr::SensorMaskBit`
- `pi/naviGATR/src/impl/resources/pico_telemetry.cpp:140` `p.mask        = static_cast<uint8_t>(s.mask);`
- After: `using SensorMask = uint16_t;` used by both. The narrowing and the
  stale comment (there is no `SensorMaskBit`; the enum is `SensorBit`,
  `frames.h:50`) both disappear. Today it is harmless only because 5 of 16
  bits are defined.

### 2.2 Time values

**A12. `HostMicros` (Pi microsecond stamps)**
- `pi/naviGATR/src/core/host_clock.h:26` `static int64_t nowUs() {`
- `pi/naviGATR/src/capture/capture_bundle.h:37` `int64_t     host_us = 0;`
- `pi/naviGATR/src/capture/capture_recorder.h:99` `int64_t     trigger_us = 0;`
- 132 `int64_t ..._us/_ms/_ns` declarations in non-test code.
- A second microsecond clock with another origin exists:
  `pi/naviGATR/src/resources/serial_link.h:32-33` "Steady clock microseconds,
  arbitrary origin." `inline int64_t steadyNowUs() {`. Both are
  `int64_t`, both reached through functions named `nowUs`.
- After: `using HostMicros = int64_t; // HostClock::nowUs(), process origin`
  in `core/host_clock.h`. Better still, route serial links through
  `HostClock` so only one microsecond origin exists (a simplification, not a
  behavior change for the retry schedule, which only differences its own
  stamps).

**A13. `MicrosClock`, `UnixMillisClock` (clock injection)**
- `pi/naviGATR/src/capture/capture_recorder.h:57` `std::function<int64_t()> clock;   // Pi host us; HostClock::nowUs when empty`
- `pi/naviGATR/src/capture/capture_recorder.h:58` `std::function<int64_t()> wall_ms; // unix ms or -1; the system clock when empty`
- `pi/naviGATR/src/impl/resources/serial_links.h:95`, `:111`, `:131`, `:140` `std::function<int64_t()>` (setClock and clock_, twice)
- Six spellings of the same type carrying two different units.
- After: `using MicrosClock = std::function<HostMicros()>;` and
  `using UnixMillisClock = std::function<int64_t()>;` in `core/host_clock.h`.

**A14. `Millis` (Pi durations have five integer types)**
- `pi/naviGATR/src/config/config_node.h:43` `bool getInt(const char* key, long def, long& out, std::string& err) const;`
- `pi/naviGATR/src/impl/localization/motion_step.h:204` `long max_wait_ms, StateEstimatorOutput& out, PreparedStep& step);`
- `pi/naviGATR/src/runtime/system.h:189` `static constexpr int64_t kPicoOperationMs = 15000;   // CONTROL 3/4 bound`
- `pi/naviGATR/src/resources/pico_control.h:65` `..., MonotonicTime now, double timeout_s) = 0;`
- Millisecond names by type: `long` 26, `int64_t` 46, `double` 24,
  `uint32_t` 3, `int` 2. `long` comes from `getInt` and is then compared
  with the `int64_t` that `MonotonicTime` subtraction returns (`time.h:41`).
- After: `using Millis = int64_t;` in `core/time.h` for every millisecond
  duration, and a `getMillis` config reader. See decision D2 for
  `std::chrono` instead.

**A15. `MicrosU32` (Pico)**
- `pico/aggreGATR/src/pi_link.h:29` `bool ready(uint32_t now_us, uint32_t next_tick_us, bool tx_empty) const;`
- `pico/aggreGATR/src/bno08x_supervisor.cpp:13` `bool elapsed(uint32_t now_us, uint32_t since_us, uint32_t us) { return now_us - since_us > us; }`
- The wrap-safe comparison exists once, file-local; `pico/aggreGATR/src/main.cpp:316`
  and `:325` read `micros()` while the drivers read `time_us_32()`.
- After: `using MicrosU32 = uint32_t; // boot clock, wraps every 71.6 min, compare by subtraction`
  and one shared `elapsedUs(now, since)` next to it. Name matches
  `cpp-brain.md` A-1.

**A16. `Seconds` has two homes on the Brain**
- `brain/communiGATR/include/communigatr/client.h:30` `using Seconds = double;`
- `brain/investiGATR/include/investigatr/geometry.h:11` `using Seconds          = double;`
- After: communiGATR imports it, as its other headers already do
  (`using investigatr::Seconds;`, see `cpp-brain.md` section 3).

### 2.3 Callback types

**A17. `NonceSource`**
- `brain/communiGATR/include/communigatr/client.h:234` `Client(BytePort& port, std::function<uint32_t()> nonce, const ClientConfig& config = {});`
- Same type at `client.h:419` and `brain/communiGATR/src/client.cpp:50`.
- After: `using NonceSource = std::function<uint32_t()>;` in class scope.

**A18. `BenchImuSource`**
- `brain/communiGATR/include/communigatr/client.h:68` `std::function<BenchImuSample()> bench_imu;`
- After: `using BenchImuSource = std::function<BenchImuSample()>;`.

**A19. `ReplyHook`**
- `brain/communiGATR/sim/fake_pi.h:129` `std::function<void(translagatr::BrainReply&)> doc_hook;`
- After: `using ReplyHook = std::function<void(translagatr::BrainReply&)>;`.

**A20. `StopCondition`**
- `brain/communiGATR/sim/link_rig.h:34` `bool runUntil(const std::function<bool()>& done, Seconds limit, Seconds dt = 0.001);`
- After: `using StopCondition = std::function<bool()>;`.

**A21. `ResourcePollFunction`, `SensorPollFunction`**
- `pi/naviGATR/src/contracts/resource.h:47` `std::function<ResourcePollResult(const ExecutionContext&)> execute;`
- `pi/naviGATR/src/contracts/sensor.h:33` `std::function<PollResult(const ResourceMap&, const ExecutionContext&)> execute;`
- These are the stage contracts teams implement against. The factory
  signatures beside them are already aliased (`SensorMakeFunction`,
  `sensor.h:47`), the executed signatures are not.
- After: `using SensorPollFunction = std::function<PollResult(const ResourceMap&, const ExecutionContext&)>;`.
- Not aliased: the `std::function<void()> reset;` beside them
  (`resource.h:51`, `sensor.h:36`). Short and clear already; see NOALIAS-2.

**A22. `JsonSectionWriter`**
- `pi/naviGATR/src/capture/capture_bundle.h:81` `std::function<void(JsonWriter&)> write_host;`
- After: `using JsonSectionWriter = std::function<void(JsonWriter&)>;` in
  `inspection/json_writer.h`, where other writers of sections will find it.

**A23. `ClientCallback`, `ClientTextCallback`**
- `pi/naviGATR/src/inspection/http_server.h:171-173`
  `std::function<void(ClientId)> onOpen;` / `std::function<void(ClientId, const std::string&)> onText;` / `std::function<void(ClientId)> onClose;` (trimmed alignment)
- After: two class-scope aliases in `HttpServer`, next to the existing
  `using ClientId = uint64_t;` (`http_server.h:159`).

### 2.4 Repeated and nested container types

**A24. `DetectionFrameMap`**
- `pi/naviGATR/src/runtime/system.h:303` `std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>> detectionFrames() const;`
- Spelled again at `system.h:456`, `system.cpp:1014`, `system.cpp:1221`.
- After: `using DetectionFrameMap = std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>>;`
  in `runtime/inspection_state.h`. The pointer stays visible inside the alias
  target; the alias names the map, not the pointer (NOALIAS-1).

**A25. `IdMap` (or a `std::hash` specialization)**
- `pi/naviGATR/src/runtime/sensor_stage.h:97` `std::unordered_map<SensorId, uint64_t, SensorId::Hash> retired_epochs_;`
- `pi/naviGATR/src/runtime/resource_stage.h:67` `std::unordered_map<ResourceId, std::size_t, ResourceId::Hash> index_by_id_;`
- `pi/naviGATR/src/state/field_state.h:63` `std::unordered_map<FieldObjectId, FieldObjectState, FieldObjectId::Hash> objects;`
- Also `records.h:102,112,115,127,128`, `localization.h:53`, `sensor_stage.h:65`, `robot_frames.h:22`.
- After, either:
  - `template <typename Id, typename Value> using IdMap = std::unordered_map<Id, Value, typename Id::Hash>;`
    in `core/ids.h`, giving `IdMap<SensorId, uint64_t> retired_epochs_;`; or
  - specialize `std::hash<TypedId<Tag>>` once, so every map is the plain
    `std::unordered_map<SensorId, uint64_t>`. Preferred: standard spelling,
    no project idiom to learn.

**A26. `WarningList`**
- `pi/naviGATR/src/runtime/localization_stage.h:133` `std::vector<std::string>* warnings,`
- The same parameter or member in 11 header declarations
  (`sensor_stage.h:105`, `world_estimation_stage.h:79`, `resource_stage.h:103`,
  the `*InitializationContext` structs in `contracts/`).
- After: `using WarningList = std::vector<std::string>;` in `core/`, and
  `WarningList* warnings`.

**A27. `LinkMonitorList`**
- `pi/naviGATR/src/diagnostics/link_monitor.h:147` `std::vector<std::shared_ptr<LinkMonitor>> all() const;`
- `pi/naviGATR/src/diagnostics/link_monitor.h:157` `std::vector<std::shared_ptr<LinkMonitor>> monitors_;`
- After: `using LinkMonitorList = std::vector<std::shared_ptr<LinkMonitor>>;`.

**A28. `Bytes`, `ByteFrames`**
- `brain/communiGATR/sim/fake_pi.h:45` `std::vector<std::vector<uint8_t>> receive(const uint8_t* data, std::size_t len);`
- `std::vector<uint8_t>` is spelled 235 times in these components and the
  naviGATR tests.
- Three test files already define the alias independently:
  `translaGATR/tests/frame_codec_gtest.cpp:21`, `translaGATR/tests/link_documents_gtest.cpp:21`,
  `pico/aggreGATR/tests/pico_commands_gtest.cpp:18`, each `using Bytes = std::vector<uint8_t>;`.
- After: one `using Bytes = std::vector<uint8_t>;` per component vocabulary
  header, and `std::vector<Bytes>` where a list of frames is meant.

**A29. `HeaderMap`**
- `pi/naviGATR/src/inspection/http_server.h:63` `std::map<std::string, std::string> headers; // lower-case names`
- Same type at `http_server.h:76` and `:225` (`parseQuery`).
- After: `using HeaderMap = std::map<std::string, std::string>;` and
  `using QueryParams = ...` (same type, different role, so two names).

**A30. `ProsMotorGroup`**
- `brain/actuGATR/include/actugatr/pros_motor_output.h:30` `std::vector<std::vector<pros::Motor>> groups_;`
- After: `using ProsMotorGroup = std::vector<pros::Motor>;` then
  `std::vector<ProsMotorGroup> groups_;`. (`MotorGroup` is taken by the
  config struct at `drivetrain.h:29`.)

**A31. `Y8Pixels` (alias the pointee, not the pointer)**
- `pi/naviGATR/src/resources/camera.h:74` `std::shared_ptr<const std::vector<uint8_t>> y8;`
- Same at `runtime/inspection_state.h:45`, `inspection/inspection_service.cpp:297`.
- After: `using Y8Pixels = std::vector<uint8_t>;` and
  `std::shared_ptr<const Y8Pixels> y8;`. Ownership stays readable.

**A32. `ResetTimeline`, `EvictionBuckets`** (the pair becomes a struct, then
the container is aliased; details in N1, N2)
- `pi/naviGATR/src/capture/capture_bundle.h:46` `std::vector<std::pair<int64_t, uint64_t>> resets;`
- `pi/naviGATR/src/capture/capture_recorder.h:190`, `capture_service.cpp:718` same type.
- `pi/naviGATR/src/capture/capture_recorder.h:213` `std::deque<std::pair<int64_t, uint64_t>> evicted_;`

**A33. `TagCellCache`** (see N4)
- `pi/naviGATR/src/impl/resources/synthetic_rig.h:127` `mutable std::map<std::string, std::pair<int, std::vector<uint8_t>>> tag_cache_;`

**A34. `DeclaredIdList`** (see N5)
- `pi/naviGATR/src/config/composition.cpp:313` `bool noteId(std::vector<std::pair<std::string, std::string>>& seen,`
- Same type at `composition.cpp:347` and `:353`.

**A35. `SendResultList`**
- `pi/naviGATR/src/inspection/inspection_service.cpp:925` `std::vector<std::pair<HttpServer::ClientId, HttpServer::Enqueue>> results;`
- Same at `:1021`; a sibling at `:786` pairs two `Enqueue`s.
- After: `struct SendResult { HttpServer::ClientId client; HttpServer::Enqueue outcome; };`
  and `using SendResultList = std::vector<SendResult>;`.

### 2.5 Long public signatures

**A36. Stage builders repeat four inputs** (named struct, see N20)
- `pi/naviGATR/src/runtime/localization_stage.h:129-134` (trimmed)
  `make_localization(const ConfigNode& node, const FunctionRegistry& functions, const SensorCatalog& sensors, const ResourceStore& resources, std::vector<std::string>* warnings, std::string& err);`
- `sensor_stage.h:101-106`, `world_estimation_stage.h:75-80`,
  `resource_stage.h:100-104` repeat the pattern.

**A37. PROS output builder**
- `brain/actuGATR/include/actugatr/pros_motor_output.h:28` `void build(const MotorGroup* const* groups, std::size_t count, Cartridge cartridge);`
- After: `using MotorGroupRefs = std::vector<const MotorGroup*>;` or a
  fixed array alias sized by `kMaxWheelGroups`, so the pointer-to-pointer
  and the separate count go away.

### 2.6 Units

**A38. Acceleration has no alias**
- `brain/investiGATR/include/investigatr/motion_model.h:22` `double           max_accel = 1.0; // m/s^2`
- `brain/investiGATR/include/investigatr/motion_model.h:24` `double           max_alpha = 6.0; // rad/s^2`
- Neighbors use `MetersPerSecond` and `RadiansPerSecond`.
- After: `using MetersPerSecondSquared = double; using RadiansPerSecondSquared = double;` in `geometry.h`.

**A39. Robot config constants skip the unit aliases**
- `brain/robot/gatr2_robot.h:74` `constexpr double   kTrackingRadius = 0.024; // UNMEASURED, provisional 48 mm wheel`
- The field it feeds is `Meters radius` (`robot_profile.h:41`).
- After: `constexpr investigatr::Meters kTrackingRadius = 0.024;`.

### 2.7 Existing aliases that the proposed rules would change

**A40. `HttpServer::Frame` hides a pointer and reuses "frame"**
- `pi/naviGATR/src/inspection/http_server.h:160` `using Frame    = std::shared_ptr<const std::string>;   // one complete WebSocket frame`
- Callers read `const Frame& frame` and cannot see it may be null or shared.
  "Frame" also means a coordinate frame, a camera frame and a wire frame
  (section 4.5).
- After: `using WsMessage = std::string;` and `std::shared_ptr<const WsMessage>`.

**A41. Trivial abbreviation alias**
- `pi/naviGATR/src/inspection/inspection_service.cpp:50` `using Ms    = std::chrono::milliseconds;`
- Saves 20 characters and adds a name to learn. `std::chrono::milliseconds`
  or `using namespace std::chrono_literals;` in the .cpp reads better.

Good aliases to keep and copy (for the standard's examples):
- `pi/naviGATR/src/core/ids.h:38-50` typed string ids (`using SensorId = TypedId<struct SensorIdTag>;`).
- `pi/naviGATR/src/core/records.h:102` `using SensorMap = std::unordered_map<SensorId, MeasurementRecord, SensorId::Hash>;`
- `pi/naviGATR/src/capture/capture_bundle.h:32` `using DiagKindCounts = std::array<uint64_t, kDiagKindCount>;`
- `pi/naviGATR/src/math/transforms.h:28` `using Transform2D = Pose2D;` (a semantic alias: same data, different role at the use site).
- `brain/investiGATR/include/investigatr/geometry.h:11-18` units and `FrameGeneration`.
- `brain/communiGATR/include/communigatr/client.h:71,90,125` one ticket alias per request kind.
- `brain/locaGATR/src/main.cpp:44` `using Link = communigatr::ProsLink;` (one swap point for the whole program).
- `pi/naviGATR/src/impl/resources/serial_links.h:78` class-scope `using Opener = std::function<...>;`.

Iterator types: none are spelled out anywhere; code uses `auto`. Keep it
that way.

## 3. Where a named type beats loose values

**N1. `ResetMark` instead of `std::pair<int64_t, uint64_t>`**
- `pi/naviGATR/src/capture/capture_bundle.h:44-46`
  `// (from host_us, reset count), ascending; ...` then `std::vector<std::pair<int64_t, uint64_t>> resets;` (trimmed)
- Use sites: `capture_service.cpp:761` `if (resets[i].first <= in.window_start_us) {`,
  `capture_bundle.cpp:517-519` `in_.resets.front().second` / `if (r.first <= t) {`.
- After: `struct ResetMark { int64_t host_us = 0; uint64_t reset_count = 0; };`
  then `if (resets[i].host_us <= in.window_start_us)`. The comment explaining
  `.first` and `.second` is no longer needed.

**N2. `EvictionBucket`**
- `pi/naviGATR/src/capture/capture_recorder.h:211-213` "(Pi stamp in ms, count)" `std::deque<std::pair<int64_t, uint64_t>> evicted_;`
- `capture_service.cpp:593-596` `if (!evicted_.empty() && evicted_.back().first >= ms) { ++evicted_.back().second;`
- Same C++ type as N1 but milliseconds instead of microseconds and a count
  instead of a reset count. Two structs make the difference visible.

**N3. `EncoderCountSample`**
- `pi/naviGATR/src/impl/resources/pico_telemetry.h:288` `std::deque<std::pair<int64_t, int32_t>> samples; // (host ms, counts), oldest first`
- After: `struct EncoderCountSample { int64_t host_ms; int32_t counts; };`.

**N4. `RenderedTag`**
- `synthetic_rig.h:127` (A33) and `synthetic_rig.cpp:96` `std::make_pair(total, std::move(cells))`.
- After: `struct RenderedTag { int total = 0; Bytes cells; };` and
  `using TagCellCache = std::map<std::string, RenderedTag>;`.

**N5. `DeclaredId`**
- `pi/naviGATR/src/config/composition.cpp:323-324` `if (previous.first == id) {` / `... " is declared in both " + previous.second + ...` (trimmed)
- After: `struct DeclaredId { std::string id; std::string file; };`.

**N6. `DueClient`**
- `pi/naviGATR/src/inspection/inspection_service.cpp:1066` `std::vector<std::pair<HttpServer::ClientId, bool>> due;   // id, raw`
- After: `struct DueClient { HttpServer::ClientId id; bool raw; };`.

**N7. `ByteRange`**
- `pi/naviGATR/src/inspection/json_writer.h:19-20` `// Byte ranges [begin, end) of a document.` / `using JsonSpans = std::vector<std::pair<std::size_t, std::size_t>>;`
- After: `struct ByteRange { std::size_t begin; std::size_t end; };` and
  `using JsonSpans = std::vector<ByteRange>;`.

**N8. `ProfileVerdict` replaces a two-out-param pattern in six signatures**
- `translaGATR/link_documents.h:152` `bool validateRobotProfile(const RobotProfileDoc& in, uint8_t& reason, uint8_t& detail);`
- `brain/communiGATR/include/communigatr/robot_profile.h:84-85` `bool toProfileDoc(const RobotProfile& in, translagatr::RobotProfileDoc& out, uint8_t& reason, uint8_t& detail);` (trimmed)
- `pi/naviGATR/src/contracts/brain_profile.h:47-48` `virtual bool prepare(..., uint32_t profile_id, uint8_t& reason, uint8_t& detail) = 0;` (trimmed)
- `pi/naviGATR/src/runtime/brain_profile_builder.h:147-148`, `pi/naviGATR/src/runtime/system.h:341-342` same pair.
- Stored the same way in `robot_profile.h:91-92`, `client.h:156-157`, `command_state.h:65-66`.
- After, in `translaGATR/link_documents.h`:
  `struct ProfileVerdict { ProfileReason reason = kProfileReasonNone; uint8_t detail = 0; bool ok() const ...; };`
  returned by value: `ProfileVerdict validateRobotProfile(const RobotProfileDoc& in);`.
  (Or a free `ok(verdict)` if the strict struct rule is adopted, D6.)

**N9. `ControlReply` replaces a raw return plus an out-param**
- `pi/naviGATR/src/contracts/brain_profile.h:53-56`
  `// CONTROL action (translagatr::ControlAction). Returns a translagatr::BrainResult;` /
  `// detail is a translagatr::ControlDetail. ...` /
  `virtual uint8_t control(uint8_t action, uint8_t arg, MonotonicTime now, uint8_t& detail) = 0;` (trimmed)
- `pi/naviGATR/src/runtime/system.h:343-344`, `:348` same shape.
- After: `struct ControlReply { BrainResult result; ControlDetail detail; };`
  and `virtual ControlReply control(ControlAction action, uint8_t arg, MonotonicTime now) = 0;`.
  Two lines of comment become the signature.

**N10. `EulerAngles`**
- `pi/naviGATR/src/math/se3.h:71-72` `inline void eulerFromRotation(const Rotation3& R, double& roll_rad, double& pitch_rad, double& yaw_rad) {` (trimmed)
- `pi/naviGATR/src/math/quaternion.h:99-100`, `pi/naviGATR/src/state/attitude.h:53-54` same out-param triple.
- Callers: `pi/naviGATR/src/state/robot_state.h:91-92`
  `double roll = 0.0, pitch = 0.0, yaw = 0.0;` / `attitudeEuler(attitude, roll, pitch, yaw);`
- After: `struct EulerAngles { double roll_rad, pitch_rad, yaw_rad; };` returned by value:
  `const EulerAngles tilt = attitudeEuler(attitude);`.

**N11. `Point3` and `Pixel`**
- `pi/naviGATR/src/math/se3.h:130-131` `inline void transformPoint3(const Transform3& T_a_b, double x_b, double y_b, double z_b, double& x_a, double& y_a, double& z_a) {` (trimmed; 7 parameters)
- `pi/naviGATR/src/math/camera_model.h:20-21` `inline void opticalFromEngineering(double x_e, double y_e, double z_e, double& x_o, double& y_o, double& z_o) {` (trimmed)
- `camera_model.h:43-45` `inline bool projectOptical(const CameraIntrinsics& K, double x_o, double y_o, double z_o, double& u, double& v) {` (trimmed)
- Six functions in `camera_model.h` take loose coordinates.
- After: `struct Point3 { double x, y, z; };`, `struct Pixel { double u, v; };`,
  `Point3 transformPoint(const Transform3& T_a_b, Point3 p_b);`,
  `std::optional<Pixel> projectOptical(const CameraIntrinsics& K, Point3 p_o);`.
  Pico already has this shape: `struct Vector3` (`pico/aggreGATR/src/gravity_alignment.h:9`).

**N12. Tracking wheel solvers: four parallel vectors and three out-params**
- `pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:53-55`
  `bool solveFull(const std::vector<double>& ux, const std::vector<double>& uy, const std::vector<double>& k, const std::vector<double>& m, double& dx, double& dy, double& dtheta) {` (trimmed)
- `:19-20` `solvePlanar(ux, uy, m, dx, dy)`, `:39` `solveForward(ux, m, dx)`.
- Call site: `:826` `solved = solveFull(ux, uy, k, m, increment.dx_m, increment.dy_m,`
- After: `struct WheelRow { double ux, uy, lever, measured; };` (one per wheel)
  and `std::optional<PlanarDelta> solveFull(const std::vector<WheelRow>& rows);`.
  The row can never be shorter in one column than another.

**N13. `HalfPlane` (parallel arrays, and one letter meaning two things)**
- `brain/investiGATR/src/collision.h:29-32` `struct Polygon { Point p[8]; int   n = 0; };` (trimmed) - here `n` is a count.
- `brain/investiGATR/src/collision.h:36-38` `struct Region { Point  n[8]; double d[8]; int    count = 0;` (trimmed) - here `n` is the normals.
- After: `struct HalfPlane { Point normal; double offset; };` and
  `HalfPlane planes[kMaxSides]; int count;`. `kMaxSides = 8` names the 8.

**N14. `EncoderPrev` (three parallel arrays)**
- `pi/naviGATR/src/capture/capture_bundle.cpp:455-462`
  `bool    has[3] = {};` / `int32_t enc[3] = {};` / `uint8_t enc_seq[3] = {}; // of the frame that carried enc[e]` (trimmed)
- After: `struct EncoderPrev { bool has; int32_t counts; uint8_t seq; };`
  `std::array<EncoderPrev, kEncoderPorts> encoders;`.

**N15. Interleaved DP index**
- `brain/investiGATR/src/planner.cpp:448-474` (trimmed)
  `auto facing = [&](std::size_t leg, int back) { return wrapAngle(travel[leg] + back * kPi); };` /
  `std::vector<double> cost(2 * n);` / `cost[2 * k + d]  = use ? bwd : fwd;`
- `int back` is a bool used as a multiplier; `2 * k + d` packs two columns.
- After: `enum Direction { kForward = 0, kBackward = 1 };` and
  `std::vector<std::array<double, 2>> cost(n);` read as `cost[k][kBackward]`.

**N16. `SlotLabels` (array indexed by convention)**
- `pi/naviGATR/src/runtime/system.h:401` `std::string                       slot_labels_[3];   // commands, targets, publishing`
- Indexed by literal at `system.cpp:277`, `:330`, `:336`, `:1047`, `:1073`, `:1078`
  (`diagnostics_.note(slot_labels_[1], target_out.status);`).
- After: `struct SlotLabels { std::string commands, targets, publishing; };`.

**N17. `PlacementOrigin` and `PlacementIdentity`**
- `pi/naviGATR/src/contracts/localization.h:123` `std::string origin;   // "command", "configuration"`
- `pi/naviGATR/src/state/robot_state.h:62-64` `std::string placement_origin;` / `uint32_t    placement_session  = 0;` / `uint64_t    placement_sequence = 0;`
- Same trio flattened in `diagnostics/records.h:63-64`.
- After: `enum class PlacementOrigin : uint8_t { kNone, kCommand, kConfiguration };`
  and `struct PlacementIdentity { PlacementOrigin origin; SessionId session; uint64_t sequence; };`.
  (Flat CSV records may stay flat; they are a file format.)

**N18. `FrameIdentity` exists privately, the pair is spelled loose five times**
- `pi/naviGATR/src/inspection/inspection_service.cpp:251-258` `struct FrameIdentity { uint64_t epoch = 0; uint32_t sequence = 0; bool set = false; ...` (trimmed)
- Loose copies: `resources/camera.h:62-63`, `payloads/tag_observations.h:55-56`,
  `runtime/inspection_state.h:34-35`, `payloads/field_object_evidence.h:76-77`,
  `inspection_service.cpp:290-291`.
- After: move it to `resources/camera.h` (without `set`; use
  `std::optional<FrameIdentity>` where "not set" matters).

**N19. `WirePose`**
- `brain/communiGATR/include/communigatr/client.h:243` `PlacementTicket submitPlacement(int32_t x_mm, int32_t y_mm, int32_t heading_cdeg);`
- The same triple sits in `BrainRequest` (`frames.h:321-323`) and `BrainState` (`frames.h:355-357`).
- After: `struct WirePose { int32_t x_mm = 0; int32_t y_mm = 0; int32_t heading_cdeg = 0; };`
  in `frames.h`. The codec still writes three fields; the wire layout does not change.

**N20. `StageBuildContext`**
- A36's four repeated inputs become one struct:
  `struct StageBuildContext { const FunctionRegistry& functions; const ResourceStore& resources; WarningList* warnings; };`
  and `make_localization(const ConfigNode& node, const StageBuildContext& build, const SensorCatalog& sensors, std::string& err)`.
  The `*InitializationContext` structs in `contracts/` already do this for
  factories; the stage builders just have not caught up.

**N21. `std::optional` instead of `bool have_x` + `x`**
- 59 `bool have_*` / `has_*` members; 53 `std::optional` uses in headers.
- `pi/naviGATR/src/payloads/tag_observations.h:46-47` `bool   has_reprojection_error  = false;` / `double reprojection_error_px   = 0.0;`
- `pi/naviGATR/src/impl/resources/pico_telemetry.h:248-249` `bool          have_seq_ = false;` / `uint8_t       last_seq_ = 0;`
- `brain/actuGATR/include/actugatr/motion.h:198-199` `bool    have_last_update_ = false;` / `Seconds last_update_      = 0;`
  (six more pairs in the same class: `:212`, `:218-220`, and the
  `started_`, `losing_`, `halting_`, `frame_captured_` pairs).
- After: `std::optional<double> reprojection_error_px;`,
  `std::optional<Seconds> last_update_;`. One member, and the value cannot
  be read while unset without saying so.

**N22. Store wire codes as their enum type, not `uint8_t` plus a comment**
- `pi/naviGATR/src/runtime/system.h:152-153` `uint8_t       result    = 0;   // translagatr::BrainResult, Pending until settled` / `uint8_t       detail    = 0;   // translagatr::ControlDetail`
- `brain/communiGATR/include/communigatr/client.h:107` `uint8_t       action      = 0; // translagatr::ControlAction`
- `pico/aggreGATR/src/imu.h:17-18` `uint8_t  state    = 0; // translagatr::PicoImuState` / `uint8_t  reason   = 0; // translagatr::PicoImuReason`
- `pi/naviGATR/src/state/command_state.h:64-65` `uint8_t  state      = 0;   // translagatr::ProfileState`
- 17 such fields. The translaGATR enums are unscoped with a fixed `uint8_t`
  underlying type, so every byte value, including codes a newer peer sends,
  is a valid value of the enum type. Nothing is lost by declaring
  `translagatr::BrainResult result = translagatr::kResultOk;`.
- See decision D3 for how far down (API layer only, or wire structs too).

**N23. `TurnDirection`**
- `brain/investiGATR/include/investigatr/path.h:29` `int         turn_direction = 0;     // kTurn: +1 CCW, -1 CW`
- After: `enum class TurnDirection : int8_t { kNone = 0, kCounterClockwise = 1, kClockwise = -1 };`.

**N24. Output arrays**
- `brain/investiGATR/include/investigatr/field.h:71` `void boxCorners(const Box& box, const Pose& owner, Point corners[4]);`
- `pi/naviGATR/src/runtime/system.h:345` `uint8_t readWheels(MonotonicTime now, uint8_t& count, translagatr::WheelReading* wheels) const;`
- After: `using BoxCorners = std::array<Point, 4>;` returned by value;
  `readWheels` returns a small struct with the result, the count and a
  `std::array<WheelReading, kWheelReadingsMax>` (the shape
  `BrainReplyContext` already uses at `command_state.h:54-55`).

## 4. Other readability issues

### 4.1 Bool literals at call sites

Setters named for the flag are fine (`setRunning(true)`, `setEnabled(true)`,
`setDriver(false)`), and so is a config default beside its key
(`getBool("invert", false, invert, err)`). About 40 other call sites pass a
bare `true` / `false`; representative ones:

| Call site | Declaration | Proposed |
|---|---|---|
| `pi/naviGATR/src/inspection/inspection_service.cpp:1254` `applyLinkMonitors(false, false);` | `:744` `(bool raw, bool decoded)` | a `LinkMonitorModes { bool raw; bool decoded; }` argument, or two setters |
| `pi/naviGATR/src/impl/commands/brain_link_commands.cpp:428` `monitor_->frame(true, brainOpName(req.op), fields);` | `diagnostics/link_monitor.h:79` `void frame(bool rx, ...)` | `enum class LinkDirection { kRx, kTx }` |
| `brain/investiGATR/src/planner.cpp:848` `findExits(space, request, start, true, push, escapes);` | `:630` `..., const Pose& end, bool leaving, ...` | `enum class PathEnd { kStart, kGoal }` |
| `pi/naviGATR/src/impl/publishing/field_documents.cpp:186` `boundaryMm(b.min_x_m, true, header.min_x_mm)` | `:51` `bool boundaryMm(double m, bool lower, int32_t& out)` | `enum class Side { kLower, kUpper }`, or two functions |
| `pi/naviGATR/src/runtime/localization_stage.cpp:149` `feed_->publish(state_, status, false, ++publication_);` | `state/robot_state_feed.h:98-99` `bool advanced` | `/*advanced=*/false` at minimum; better an enum |
| `pi/naviGATR/src/inspection/inspection_document.cpp:1792` `writeSnapshotBody(w, system, service, now, localization, reset, true);` | `:1140-1142` `bool with_trail` (7th parameter) | options struct |
| `pi/naviGATR/src/runtime/system.cpp:702` `losePlacement(now, "sensor lost: " + r.level, !loss_warned_);` | `system.h:352` `bool note` | `enum class Report { kSilent, kLogEvent }` |
| `brain/actuGATR/src/follower.cpp:78` `heading_(config.heading, true)` | `pid.h:24` `explicit Pid(const PidGains& gains = {}, bool angular = false);` | `Pid::angular(gains)` factory, or `enum class PidKind` |
| `pi/naviGATR/src/capture/capture_service.cpp:468` `finish("shutdown", true);` | `capture_recorder.h:165` `void finish(const char* stop_reason, bool truncated);` | `enum class StopReason` that implies truncation |

### 4.2 Magic numbers

- Column counts that must match code above them:
  `pi/naviGATR/src/capture/capture_bundle.cpp:911` `for (int c = 0; c < 17; ++c) {` and
  `:929` `for (int c = 0; c < 26 + translagatr::kTelemetryWheelsMax; ++c) {`.
  Adding a telemetry column silently shifts every later CSV column.
  Named `kMotionColumns` / `kTelemetryColumns`, or `row_.none(count)` next
  to the column list.
- Unnamed buffer sizes: `brain/communiGATR/include/communigatr/client.h:492`
  `EstimateSeen         estimate_seen_[4];`,
  `brain/communiGATR/include/communigatr/link_driver.h:84` `Numbered                     frames_[4]; // newest first`.
- A wire limit defined twice, outside translaGATR:
  `pico/aggreGATR/src/pico_commands.h:17` `constexpr uint8_t kMaxDiagHz = 5;` and
  `pi/naviGATR/src/impl/resources/pico_telemetry.h:187` `static constexpr uint8_t kMaxDiagHz        = 5;`,
  while `translaGATR/frames.h:435` states it only in a comment
  (`diag_hz u8 (0 off, 1..5)`). It belongs in `frames.h` as one constant.
- The 250 ms freshness limit, six places:
  `pico_control.h:24` (comment), `pico_telemetry.h:155-156`,
  `inspection/inspection_document.h:48`, `runtime/brain_profile_builder.h:97,101,108`.
  See `complexity-bugs.md` D8.
- Iteration counts: `pi/naviGATR/src/math/camera_model.h:77` `for (int i = 0; i < 20; ++i) {`.
- Hand-numbered byte offsets: `translaGATR/frame_codec.cpp:100-134` writes
  and reads the state block with `wr32(p + 26, s.profile_id);` ... `p[39]`,
  mirrored in two functions. About 210 literal offsets in the codec. A
  small cursor (`ByteWriter w{p}; w.u8(s.robot_flags); w.i32(s.x_mm); ...`)
  reads like the wire comment in `frame_codec.h:28-56`, keeps encode and
  decode symmetrical, and the existing 3554-line codec test proves no byte
  moved.

### 4.3 Long functions and deep nesting

lizard over 2240 non-test functions: median 8 NLOC; 188 over 40 NLOC, 63
over 80, 15 over 150; 122 with CCN over 15, 31 over 30.

Worst offenders (NLOC / CCN):

| Function | NLOC | CCN |
|---|---|---|
| `pi/naviGATR/src/inspection/inspection_document.cpp:1140` `writeSnapshotBody` | 367 | 41 |
| `pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:510` `TrackingWheelMotion::run` | 310 | 105 |
| `pi/naviGATR/src/inspection/inspection_document.cpp:748` `helloDocument` | 271 | 22 |
| `pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:105` `TrackingWheelMotion::create` | 270 | 67 |
| `pi/naviGATR/src/impl/resources/synthetic_rig.cpp:398` `make_synthetic_rig` | 242 | 79 |
| `pi/naviGATR/src/impl/world_estimation/tag_mount_association.cpp:148` `TagMountAssociation::run` | 240 | 53 |
| `pi/naviGATR/src/runtime/system.cpp:148` `System::build` | 171 | 31 |
| `brain/communiGATR/src/client.cpp:869` `Client::choose` | 143 | 56 |
| `brain/actuGATR/src/motion.cpp:363` `Motion::update` | 117 | 38 |

Mixing abstraction levels, example: `Client::choose` is the request priority
policy (documented as a list in `client.h:8-14`) but each branch also builds
the request byte by byte, e.g. `client.cpp:941-946` (trimmed):
`request.op = translagatr::kOpProfileWrite; request.offset = offset;`
`request.data_len = static_cast<uint8_t>(std::min<uint16_t>(left, translagatr::kProfileChunkMax));`
`std::copy(config_.profile.bytes + offset, ...)`. The numbered comments
(`// 2. Placement.` at `:891`, `// 3. Control.` at `:910`) are a list the code
could state directly: `choose` reads as the priority list, with
`nextProfileChunk()` and friends doing the building.

Deepest nesting (control statements at depth 6): `pico_telemetry.cpp:293`
(a frame decode inside a do-while inside a for inside a read loop),
`tracking_wheel_motion.cpp:693`, `configured_targets.cpp:208`,
`composition.cpp:377`. Early `continue`, and one `handleFrame(type)` per
frame type, flatten each.

### 4.4 Unclear abbreviations

- `pi/naviGATR/src/inspection/inspection_service.cpp:600-611`: `h_seq`,
  `c_seq`, `r_hello`, `r_hist`, `r_cap`, `h_us`, `t0` in one function
  (`uint64_t            h_seq = 0, c_seq = 0;`).
- `brain/communiGATR/include/communigatr/client.h:486,489` `DocAssembly map_asm_;` /
  `DocAssembly estimate_asm_;`: "asm" is also the ASM330 IMU on the Pico
  (`asm330_supervisor.h`).
- `brain/investiGATR/src/collision.h:22-25` `double c  = 1;` / `double s  = 0;` /
  `Meters hx = 0;` / `Meters hy = 0;` for cosine, sine and half extents.
- `pico/aggreGATR/src/imu_asm330.cpp:54` `asm330::Supervisor g_sup;`,
  `pi/naviGATR/src/inspection/http_server.cpp:458` `rr_last` (round robin),
  `pi/naviGATR/src/capture/capture_bundle.cpp:558` `mk` for a marker.

Keep: established math and domain notation (`T_a_b`, `R`, `q`, `dx`,
`dtheta`, `K` for intrinsics, `ux`/`uy` for unit vectors) and unit
suffixes (`_mm`, `_cdeg`, `_us`). The glossary (4.5) lists the accepted
abbreviations so everything else is spelled out.

### 4.5 Inconsistent vocabulary

**Session means two things in one document.**
- `pi/naviGATR/src/inspection/inspection_document.cpp:760-762`
  `w.key("session");` / `w.beginObject();` / `w.field("id", system.sessionId());`: the Pi process id (a random hex string, `system.cpp:53`).
- `inspection_document.cpp:1286` `w.field("session", c.session);`: the Brain link session (`uint32_t`).
- The wire calls the Pi process identity `pi_instance` (`frames.h:378`);
  capture calls it `pi_session` (`capture_bundle.h:43`).

**Frame has four meanings.** Coordinate frame (`FrameId`, `core/ids.h:49-50`,
whose comment already warns "Not a wire packet."), camera frame
(`CameraFrameData`, `resources/camera.h`), wire frame (`translagatr::FrameReader`,
`FrameType`), WebSocket frame (`HttpServer::Frame`, A40).

**Epoch vs generation in one line.**
- `pi/naviGATR/src/payloads/tag_observations.h:56` `uint64_t      frame_epoch    = 0;   // camera restart generation`
- `pi/naviGATR/src/resources/camera.h:63` `uint64_t      epoch    = 0;   // device restart generation`
- "generation" alone means: field frame identity (`geometry.h:18`), map
  plus estimate pair (`field.h:60`, `client.h:177`), count of profile
  applies (`brain_profile.h:31`), target activations (`target_state.h:46`),
  plans (`motion.h` `plan_generation_`).

**Placement has three names.** Wire op `kOpSetPose` (`frames.h:105`), Brain
`submitPlacement` (`client.h:243`), Pi `init_sequence`, `init_session`,
`init_pose` (`command_state.h:94-96`) next to `placement_session`,
`placement_sequence` for the same thing (`robot_state.h:63-64`,
`command_state.h:38` "SET_POSE: init_sequence it was recorded under").

**State, status and result.**
- `brain/communiGATR/include/communigatr/client.h:82-85`
  `struct PlacementStatus {` / `PlacementTicket ticket = 0;` /
  `PlacementResult state = PlacementResult::kNone;` /
  `uint8_t result = translagatr::kResultOk; // last reply result` (trimmed):
  a `PlacementResult` in a field named `state`, beside a field named `result`.
- The Pi uses `PicoRequestState` (enum) inside `PicoRequestStatus` (struct)
  (`pico_control.h`), the Brain uses `ControlResult` inside `ControlStatus`.
- `ProfileStatus` is a different struct in `communigatr` (`client.h:153`) and
  `navigatr` (`command_state.h:63`); `RobotState` differs between
  `investigatr` (`state_source.h:27`, status + pose + age) and `navigatr`
  (`robot_state.h:44`, full estimate); the wire calls it `BrainState`.

**Pose and velocity spellings.** `investigatr::Pose {x, y, heading}` in
`Meters`/`Radians`; `navigatr::Pose2D {x_m, y_m, heading_rad}`; wire
`x_mm, y_mm, heading_cdeg`. Velocity is `vx, vy, omega`
(`actugatr/chassis.h:24-26`) and `vx_m_s, vy_m_s, yaw_rate_rad_s`
(`robot_state.h:51-53`). The unit spelling differs by design (alias vs
suffix, see ALIAS-4); the words `omega` / `yaw_rate` should not.

**Executor and executable.** `ResourceExecutable` (one configured
resource's poll) vs `ResourceExecutor` (the aggregate stage), likewise
`SensorExecutable` / `SensorExecutor`. One letter apart, different levels
of the pipeline; this feeds the "where is the pipeline" gripe.

**Two host microsecond clocks called `nowUs`** (A12).

### 4.6 Comments that stand in for types, or restate code

- 17 integer fields name their real type in a comment (N22). One is already
  wrong: `records.h:79` names `SensorMaskBit`, which does not exist. A type
  cannot go stale that way; a comment can.
- Function comments doing a signature's job: `brain_profile.h:53-54`
  "Returns a translagatr::BrainResult; detail is a translagatr::ControlDetail." (N9).
- 309 of 330 source files open with a comment that is only the file name
  (`// ids.h`). It restates something every editor shows. Decision D4.
- Section labels in member lists (`// Session.`, `// Placement.` in
  `client.h:442-502`) are the bare noun labels the user asked for; keep them.
- Long file-top comments (83 lines in `runtime/system.h:1-83`, 65 in
  `brain_profile_builder.h`) are architecture docs living in headers; see
  `docs.md` for where they should go.

### 4.7 Clever code that could be plain

- Type carried through a tag object into a generic lambda:
  `pi/naviGATR/src/runtime/system.cpp:30-33` `template <typename T> struct Tag { using type = T; };` and
  `:259-261` `const auto buildSlot = [&](const char* slot_name, auto& target, auto make_tag, ...` /
  `using MakeFunction = typename decltype(make_tag)::type;` (trimmed).
  Plain: a member function template called as
  `buildSlot<CommandsMakeFunction>("CommandCollection", commands_, ...)`.
- `pi/naviGATR/src/impl/world_estimation/tag_mount_association.cpp:55-56`
  `->require<typename std::decay_t<decltype(out)>::element_type>(id, inner);`.
  Plain: a two-line function template taking `std::shared_ptr<const Contract>& out`.
- Nested ternary clamp: `pi/naviGATR/src/math/se3.h:73-75`
  `pitch_rad = std::asin(-R.m[2][0] < -1.0 ? -1.0 : -R.m[2][0] > 1.0 ? 1.0 : -R.m[2][0]);` (trimmed).
  Plain: `std::asin(std::clamp(-R.m[2][0], -1.0, 1.0))`.
- Reverse loop idiom `for (std::size_t k = n; k-- > 0;)` (`planner.cpp:470`)
  and the packed index of N15.
- 44 loops over maps read `kv.first` / `kv.second`
  (`for (auto& kv : clients) { ClientState& c = kv.second; ...`,
  `inspection_service.cpp:1070-1071`); 0 structured bindings in the whole
  codebase. Plain: `for (auto& [id, client] : clients)`.
- Hand-formatted log fields: `pi/naviGATR/src/impl/resources/pico_telemetry.cpp:283-300`
  `char fields[160];` then an `snprintf` with seven `static_cast`s. Fine on
  the Pico; on the Pi a `std::string` builder or the existing `JsonWriter`
  reads better and cannot truncate.

### 4.8 Naming inconsistencies that hurt scanning

(Naming rules proper are in `cpp-pi.md` section 1 and `cpp-brain.md`
section 2; listed here because they block finding things.)
- `pi/naviGATR/src/runtime/register_all.h:21-30` declares
  `void register_resources(...)` ... and then `void registerAll(FunctionRegistry& functions);`.
- Factories: 24 `make_*` snake_case names (`make_localization`) beside
  `makeCaptureService`, `makeCaptureRecorder`, `makeNoopCommands`.
  Searching `make_` misses a third of them.
- `MonotonicTime` members are camelCase (`exposureAt`, `measuredAt`,
  `lastPolledAt`) while every other member is snake_case, e.g.
  `resources/camera.h:64-66` `MonotonicTime exposureAt;` next to
  `int64_t exposure_uncertainty_ms = 0;`.
- Pico enumerators have no `k` prefix: `pico/aggreGATR/src/pico_commands.h:20`
  `None,` vs `kNone` everywhere else.
- About 100 multi-declarator lines, e.g. `runtime/system.h:431`
  `uint64_t reboots = 0, restarts = 0, imu_restarts = 0;`,
  `capture_recorder.h:122` `std::string reason, requester, detail, stop_reason;`.

### 4.9 Struct vs class, as it affects reading

- 429 structs; about 41 declare member functions (approximate parse), for
  example `investigatr::Path` (`length()`, `empty()`), `navigatr::RobotState`
  (`fieldPose()`, `T_odom_robot3()`), `ProfileDocument::configured()`.
- One class exists only to hold static functions: `pi/naviGATR/src/core/host_clock.h:16`
  `class HostClock` with `now()` and `nowUs()`. A namespace with two free
  functions says the same with less.
- Under the user's rule these either become free functions next to the
  struct (`Pose2D fieldPose(const RobotState&)`) or the type becomes a class.
  Decision D6.

## 5. Proposed rules

Numbered for the standard. Each rule is short; the examples above are the
rationale and can be cited from the standard.

### 5.1 When to add an alias

- **ALIAS-1. Name every integer that is an identity, a counter or a code.**
  IDs, epochs, revisions, generations, sequences, tickets and handles, hub
  source ids. One alias per concept, defined once, in the component that
  owns the concept. Wire concepts live in translaGATR. (A1-A11)
- **ALIAS-2. Name every `std::function` type that appears in a header or
  more than once.** Name it by role (ALIAS-NAME-3). (A13, A17-A23)
- **ALIAS-3. Name a container type spelled twice or more, or nested
  (a template argument that is itself a template) in any signature or
  member.** (A24-A35)
- **ALIAS-4. Units.** Floating SI quantities use unit aliases (`Meters`,
  `Seconds`, `Radians`, `MetersPerSecond`, `MetersPerSecondSquared`, ...).
  Integer fixed-point values (wire, firmware, config) keep a unit suffix in
  the name (`x_mm`, `heading_cdeg`, `stamp_ms`) and get no unit alias: the
  unit is in the scaling, and the name carries it at every use.
- **ALIAS-5. One clock alias per clock.** A time value's alias names its
  clock and origin (`HostMicros`, `MicrosU32`); a clock the code does not
  need is removed rather than aliased. (A12-A15)
- **ALIAS-6. A comment that names a type means a missing type.** Use the
  enum type or add the alias, then drop the comment. (N22, 4.6)

### 5.2 When not to alias

- **NOALIAS-1. Never hide ownership, pointer-ness, reference-ness or
  const.** No `FooPtr`, no `using Frame = std::shared_ptr<...>`. Alias the
  pointee and spell the smart pointer at the use (`std::shared_ptr<const Y8Pixels>`).
  An alias whose target contains a pointer is fine when the alias names a
  container of them (`DetectionFrameMap`). (A31, A40)
- **NOALIAS-2. No trivial renames.** No aliases that only abbreviate a
  standard type (`Ms`, `Str`, `Vec`) or rename one that is already clear
  (`using Callback = std::function<void()>;`). (A41, A21 note)
- **NOALIAS-3. No alias over `std::pair` or `std::tuple` whose members
  need names.** If a comment has to say what `.first` is, write a struct.
  (N1-N7)
- **NOALIAS-4. No second alias for a name another namespace already
  owns.** Import it with a using-declaration (`using investigatr::Seconds;`),
  never redefine it. (A16)
- **NOALIAS-5. `using namespace` never in a header.** In production .cpp
  files prefer using-declarations; tests may use `using namespace` for the
  component under test (current practice). The one production case is
  `pico/aggreGATR/src/main.cpp:16`.
- **NOALIAS-6. Not for a private type used once**, unless the alias names
  a concept that the reader needs (compare `function_registry.h:96`,
  private and single use: leave it spelled out).

### 5.3 Alias naming

- **ALIAS-NAME-1.** Aliases are types: `UpperCamelCase`, nouns.
- **ALIAS-NAME-2. Identity and counter suffixes follow the glossary (5.6):**
  `FooId`, `FooEpoch`, `FooRevision`, `FooGeneration`, `FooSequence`,
  `FooTicket` (caller-side token for an async request). Pick `Ticket` for
  the Pi's `handle` too, since the Brain API already uses it publicly.
- **ALIAS-NAME-3. Callables by role:**
  - `FooCallback`: notifies, returns `void` (`ClientCallback`);
  - `FooSource`: takes nothing, returns a value (`NonceSource`,
    `BenchImuSource`); a clock is `FooClock` (`MicrosClock`);
  - `FooHandler`: request in, response out (existing `HttpHandler`);
  - `FooMakeFunction`: a registry factory (existing, keep);
  - `FooHook`: a test or sim injection point (`ReplyHook`).
- **ALIAS-NAME-4. Containers by content:** `FooList` (sequence),
  `FooMap` (keyed; existing `SensorMap`, `ResourceMap`), `FooSet`,
  `FooQueue`. Plural nouns are accepted for fixed arrays of counts
  (`DiagKindCounts`).
- **ALIAS-NAME-5. Units** are plural SI names, compound with `Per`:
  `MetersPerSecondSquared`.
- **ALIAS-NAME-6. Alias templates** read as a noun phrase over their
  parameters: `IdMap<SensorId, uint64_t>`.

### 5.4 Where aliases live

- **ALIAS-HOME-1. Class scope** when only that class's interface uses it
  (`HttpServer::ClientId`, `ReopeningLink::Opener`, `Client::NonceSource`).
- **ALIAS-HOME-2. Namespace scope in the header that owns the concept**
  when several files use it (`DiagSourceId` in `diagnostics/hub.h`).
- **ALIAS-HOME-3. One vocabulary header per component** for units, ids and
  clocks that cut across the component. Today's de facto homes, kept rather
  than adding files:
  - translaGATR: `frames.h` (wire ids, `WirePose`, `SensorMask`) and
    `link_documents.h` (`DocumentId`, `ProfileVerdict`);
  - naviGATR: `core/ids.h`, `core/time.h`, `core/host_clock.h`;
  - Brain: `investigatr/geometry.h` for units (every Brain component already
    depends on it); others import with using-declarations;
  - aggreGATR: `config.h` or a new `time.h` only if `MicrosU32` and its
    helper need a home (the one justified new file).
- **ALIAS-HOME-4. File-local aliases** go in the .cpp's unnamed namespace.
- **ALIAS-HOME-5. Tests use the production alias.** A test that needs
  `Bytes` includes the header that defines it, instead of redefining it
  (three test files do today).

### 5.5 Strong typedefs vs plain aliases

A plain alias names a thing; a strong type also stops the compiler from
accepting the wrong thing. Cost of a strong type here: a wrapper struct,
`.value` at every edge (codec, JSON, CSV, config), operators for `++` and
comparisons, and one more idiom to learn. Recommendation per candidate:

| Candidate | Today | Recommend | Why |
|---|---|---|---|
| String config ids | `TypedId<Tag>`, strong (`core/ids.h:18-50`) | keep; add `std::hash` specialization | already pays: a `ResourceId` cannot be passed as a `SensorId` |
| Wire integer ids and counters | raw `uint16_t` / `uint32_t` | plain alias (ALIAS-1) | wire structs are aggregates encoded field by field on three targets including the RP2040; about 200 codec lines would gain `.value`; no mix-up between two ids has been observed |
| Epoch and revision width | `uint32_t` wire, `uint64_t` Pi | plain alias per side | the aim is to make the conversion point visible, which a plain alias does |
| Pi instants | `MonotonicTime` (ms + runtime domain tag, debug assert) | keep | already the strong option; note `operator==` does not check the domain (`time.h:46`) while `<` does |
| Pi microsecond stamps | raw `int64_t`, two origins | remove the second clock, then a plain alias | one clock makes a strong type unnecessary |
| Pi durations | `long` / `int64_t` / `double` ms and s | one plain `Millis`, or `std::chrono` (D2) | the mix is the problem, not the lack of checking |
| Brain time | `Seconds = double` | plain alias, one definition | matches the planner's SI style |
| Pico time | raw `uint32_t` us | plain `MicrosU32` + one wrap-safe helper | firmware is small; the helper is what prevents wrap bugs |
| SI units | plain aliases | keep plain | a units library is a dependency and noise at this size; revisit only after a unit bug |
| Wire enum codes | `uint8_t` + comment | the enum type itself (N22) | zero cost, all byte values stay representable |

Rule text: **ALIAS-STRONG-1.** Use a plain alias by default. Introduce a
strong type only when a mix-up has happened or would be silent and costly,
and say which in the type's comment. Existing strong types (`TypedId`,
`MonotonicTime`) stay.

### 5.6 Vocabulary glossary (proposal)

One meaning per word, across all components. Where code disagrees today,
the column says what changes. This is the content for `docs/glossary.md`
proposed in `docs.md` DOC-19.

| Term | Means | Today's conflicts |
|---|---|---|
| pose | position and heading only (x, y, heading). No velocity, no validity. | fine |
| transform `T_a_b` | pose of frame b in frame a; `compose(T_a_b, T_b_c)` is `T_a_c` (`transforms.h:2-4`) | fine |
| state | what a thing is now: robot state = pose + velocity + validity + anchors; also a lifecycle enum (`MotionState`) | `BrainState` on the wire is the Pi's reply block; keep the wire name, document it |
| estimate | world estimation's output for field objects | fine |
| observation | an estimator input derived from measurements | fine |
| sample, measurement | one reading from a sensor with its time | fine |
| placement | a field pose given to localization for the robot | retire "init" (`init_sequence`, `init_session`, `init_pose`); `SET_POSE` stays as the wire op name |
| anchor | `T_field_odom`, set by a placement | fine |
| session | the Brain link session opened by HELLO; nothing else | Pi process id called `sessionId()` and JSON key `session` (`inspection_document.cpp:760`); capture `pi_session` |
| instance | one lifetime of a process or object (`pi_instance`) | the Pi process id should use this word |
| boot | one power-on of a device (`boot_id`) | fine |
| epoch | counter that bumps when continuity breaks; values from different epochs are never combined (odometry, acquisition, IMU, camera, accumulator) | comments calling an epoch a "generation" |
| revision | counter of edits to one continuous thing (`anchor_revision`) | `MapIdentity::revision` is an author-declared version: rename `version` |
| generation | identity of one published immutable snapshot; changes whenever its content changes (`Field::generation`, `FrameGeneration`) | "counts applies" (`brain_profile.h:31`), target activations (`target_state.h:46`): these are counts, name them `applies`, `activation` |
| sequence, seq | position in an ordered stream within one epoch; may wrap | `init_sequence` is a placement counter: `placement_sequence` |
| cycle | one pass of a worker loop | fine |
| invocation | one call of a stage function | fine |
| ticket | caller's token for an async request, 0 = refused | Pi `handle` (`pico_control.h:65-67`) |
| id | stable identity; crc-based for documents | fine |
| result | outcome code of a request (`BrainResult`, `PlacementResult`) | `PlacementStatus::state` holds a result (4.5) |
| status | a struct reporting a result or state with details (`ControlStatus`) | fine once fields are named after their types |
| frame (coordinate) | a named coordinate system (`FrameId`) | say "wire frame", "camera frame", "WebSocket message" for the others; rename `HttpServer::Frame` (A40) |
| stage | one aggregate step of the naviGATR pipeline (resources, sensors, localization, world estimation) | fine |
| slot | a single typed pipeline position filled by one registered implementation (commands, target resolution, publishing) | fine |
| executor / executable | stage-level runner / one configured item's runner | rename one side so they differ by more than a suffix, e.g. `ResourceStage` / `ResourcePoll` |
| make / register | `make_*` builds from config; `register_*` adds factories to the registry | one spelling each (4.8) |

Accepted abbreviations (everything else spelled out): `id`, `seq`, `acq`
(acquisition, wire name), `imu`, `enc` (encoder, wire name), `diag`, `doc`,
`rx`/`tx`, `dt`, `prev`, `cfg` only as the Pico namespace, unit suffixes
(`_m`, `_mm`, `_um`, `_rad`, `_deg`, `_cdeg`, `_mdeg`, `_s`, `_ms`, `_us`,
`_hz`, `_mdps`), and math notation (`T_a_b`, `R`, `q`, `K`, `dx`,
`dtheta`, `ux`).

### 5.7 Named types and call-site readability

- **TYPE-1. Return values, not out-parameters,** for results up to a small
  struct. Use `std::optional<T>` for "maybe". Out-params stay only for
  large buffers the caller reuses (`StateSource::field(Field& out)`) and for
  the `std::string& err` convention (252 uses, consistent; keep it).
  (N8-N12, N24)
- **TYPE-2. Two or more values that travel together get a struct**, from
  the second signature that passes them (x/y/z, roll/pitch/yaw, reason and
  detail, epoch and sequence). (N8, N10, N11, N18, N19)
- **TYPE-3. No parallel arrays or packed indices.** One array of structs.
  (N12-N15)
- **TYPE-4. No `bool have_x` beside `x`;** use `std::optional<X>`. (N21)
- **TYPE-5. No bool parameter unless the function name says what it is**
  (`setEnabled(bool)`). Otherwise an enum or an options struct. More than
  one bool parameter is always an options struct. (4.1)
- **TYPE-6. Magic numbers get a named constant next to their meaning.**
  Wire limits live in translaGATR once. Counts that must match a list are
  derived from the list, not typed twice. (4.2)
- **TYPE-7. Structured bindings for pairs and map entries**
  (`for (const auto& [id, client] : clients)`). (4.7)
- **TYPE-8. One declarator per line.** (4.8)

### 5.8 Function shape

- **READ-1. A function works at one level.** A policy function reads as the
  policy (a priority list, a sequence of stages); the building and parsing
  under it are named helpers. (`Client::choose`, 4.3)
- **READ-2. Thresholds.** Reuse `complexity-bugs.md` 1.6 for the numbers;
  from this survey, 60 NLOC and CCN 15 mark a function for review, and
  nesting past 4 control levels inside a function needs early returns or a
  helper.
- **READ-3. Plain over clever.** No type smuggling through generic lambdas
  (`Tag<T>`, `decltype(out)::element_type`); use a named function template.
  Standard helpers over hand-rolled ones (`std::clamp`). (4.7)
- **READ-4. Abbreviations only from the glossary list.** (4.4)

Slower but clearer is accepted under these rules: returning small structs
and `std::optional` by value, `std::array` of structs instead of packed
arrays, and a byte cursor in the codec all cost nothing measurable at the
loop rates here (Pi 100 Hz, Pico 50 Hz, Brain link 50 Hz).

## 6. Decisions for the lead

- **D1. Shared snapshots.** `std::shared_ptr<const ProfileBinding>` is
  spelled 11 times, `std::shared_ptr<const FieldMap>` 8. Options: spell it
  out (NOALIAS-1 strict), or allow one alias template that keeps ownership
  in its name, `template <typename T> using SharedConst = std::shared_ptr<const T>;`.
  Recommendation: allow `SharedConst<T>` and no per-type pointer aliases.
- **D2. Pi durations.** Plain `Millis = int64_t`, or `std::chrono::milliseconds`
  on the Pi host (the standard strong duration; cost is `.count()` at the
  JSON, CSV and config edges). Recommendation: `std::chrono` for new Pi
  host code that is not already built on `MonotonicTime`; `Millis` for the
  rest. Brain and Pico keep their plain aliases.
- **D3. Enum types in wire structs.** Apply N22 at the API layers only
  (communiGATR statuses, naviGATR host interfaces, Pico `imu::Status`), or
  also in `translaGATR/frames.h` structs (the codec gains casts; layout
  unchanged). `cpp-brain.md` A-1 proposes `ControlActionCode = uint8_t`
  aliases instead. Recommendation: the enum type everywhere; it is the
  named type that already exists.
- **D4. File-name first line** (`// ids.h`, 309 of 330 files). Keep as a
  harmless convention, or drop under the user's "skip any comment that just
  restates" rule. Recommendation: drop, keep the description line under it.
- **D5. Renames that cross a contract.** C++ renames (glossary) are free.
  JSON keys (`"session"`), CSV headers and XML attributes are read by
  spectaGATR, captures and configs; renaming them needs a versioned
  contract change. Recommendation: rename C++ now, contract keys only with
  the next schema version.
- **D6. Struct with member functions.** Strict reading of "struct for
  attribute only things": helpers become free functions beside the struct.
  Lenient: allow language-required members (`operator==`, `Hash`) and
  one-line const queries (`empty()`, `valid()`). Recommendation: lenient
  for those two kinds only; anything that computes (`T_odom_robot3()`,
  `Field::find`) becomes a free function or the type becomes a class.

## 7. Applying this without changing behavior

Order that keeps every step compile-checked and test-neutral:
1. Add the aliases to the vocabulary headers (5.4); no use sites yet.
2. Replace spelled-out types with aliases, one component per change. Pure
   type-name substitutions; the compiler checks each.
3. Enum types for wire-code fields (N22, per D3).
4. Named structs for out-params and pairs (section 3), one family per
   change (`ProfileVerdict` across its six signatures together).
5. `std::optional` for `have_x` pairs, structured bindings, one declarator
   per line: mechanical, file by file.
6. Glossary renames (5.6) in C++ only; contract keys per D5.
7. Function splits (4.3) last, each covered by the existing tests.
Each step: build all targets, run all tests, clang-format 22.1.8.
