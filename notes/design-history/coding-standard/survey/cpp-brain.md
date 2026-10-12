# Survey: C++ conventions in brain/, translaGATR/, pico/aggreGATR/

Input for the GATR2 coding standard. Read-only survey of the code as it is on
main at ae457ee (clean tree), 2026-09-28. Every claim cites path:line; snippets
are verbatim unless marked "(trimmed)".

## 0. Scope and method

- 187 tracked C++ files, 39,644 lines: brain/ (all three libraries, sim/, pros/,
  tests/, both apps, robot/), translaGATR/ (src and tests), pico/aggreGATR/
  (src and tests).
- Not surveyed:
  - PROS kernel headers under brain/*/include/pros/ (not in git);
  - vendored C in pico/aggreGATR/third_party/sh2.
  - brain/*/include/main.h and api.h are the PROS template (vendor text); they
    are counted only where noted.
- Buckets used in counts:
  - brain-lib: communiGATR, actuGATR, investiGATR src/, include/, pros/;
  - sim: brain */sim;
  - tests: every tests/ folder;
  - apps: locaGATR, operaGATR, brain/robot;
  - pico: aggreGATR/src;
  - translagatr: translaGATR/*.h, *.cpp.
- Tools (scripts and raw output in the scratchpad `standard/cpp-brain/`):
  - clang-format 22.1.8, `--dry-run` with the repo `.clang-format`. It is not at
    /c/msys64/ucrt64/bin as the brief said; the pinned binary is at
    `C:/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/clang-format.exe`.
  - lizard 1.24.0 (scratch venv): function length (NLOC) and cyclomatic
    complexity (CCN).
  - Python regex scans with comments and strings stripped. They are not a
    parser, so counts are approximate; every rule proposal rests on cited lines,
    not on a count alone.

## 1. Headline findings

1. **Naming is already consistent.** PascalCase types, lowerCamelCase
   functions, snake_case variables, fields and files, a trailing underscore on
   private members, kPascal constants and enumerators.
   - Outliers:
     - the Pico enum class enumerators (29, no k);
     - `PidGains::kP/kI/kD` fields;
     - generated PascalCase `Field::RedGoal2West` constants;
     - platform entry points.
2. **Structs are data.**
   - 90 of 95 brain-lib structs, 18/18 translaGATR structs and 13/13 Pico
     structs have no methods.
   - The 5 exceptions: four small const helpers, and `ProsUsbPort::Io`, a
     task object declared as a struct. `Reference` also carries static
     factories (section 4).
   - 7 of the 35 brain-lib classes are pure interfaces: the seams.
3. **Free functions are normal here.**
   - Counts: 160 free vs 312 member in brain-lib, 72 vs 11 in translaGATR,
     85 vs 76 in the Pico.
   - `readinessOf`, `telemetryOf`, `resolve`, `footprintClearance` and
     `investigatr::collision::*` are pure functional cores.
4. **Almost everything is namespaced.**
   - The exceptions are platform entry points and one generated global
     `struct Field`.
   - The Pico spreads 7 unrelated short namespaces and has no `aggregatr`
     namespace.
5. **Portable code never reads a clock.**
   - 48 Brain declarations take `Seconds now`.
   - 19 Pico declarations take `uint32_t now_us`.
   - Clock reads live only in pros/, the apps and the Pico hardware files.
6. **No exceptions, no assert(), no RTTI anywhere.** Errors come back in 7
   different shapes (section 9).
7. **Units follow two conventions.**
   - The wire and the Pico put units in suffixes (`_ms`, `_us`, `_mm`, `_cdeg`,
     `_um`).
   - Brain SI code uses `using Meters = double` style aliases.
   - Conversion constants are redefined per file: `kPi` 3x, `kCdegPerRad` 3x,
     `kDeg` 2x, and an inline `* 180.0 / investigatr::kPi` 7x.
8. **Formatting is not enforced.**
   - 133 of 187 files differ from clang-format 22.1.8 output with the repo
     config (2264 diagnostics).
   - The Brain writes `public:` at column 0; the config produces `  public:`.
9. **Heap use.**
   - Pico src and translaGATR src never touch the heap.
   - The Brain planner allocates vectors on every plan and runs in the drive
     task.
10. **The two Brain apps duplicate their wiring.**
    - 8 helpers are repeated between `locaGATR/src/main.cpp` and
      `operaGATR/src/main.cpp`.
    - This is the Brain version of the naviGATR "where is the make pipeline"
      gripe (section 18).

---

## 2. Naming

### 2.1 Types

- **Practice:** every struct, class and enum name is PascalCase (165 struct,
  66 class and 86 enum definitions; none lowercase).
  - Names are domain nouns: `ChassisCommand`, `PlanRequest`, `RobotProfile`.
  - Roles take suffixes:
    - `*Config`: `DriveConfig`, `ClientConfig`, `FollowerConfig`;
    - `*Status`: `DriveStatus`, `MotionStatus`;
    - `*Stats`: `ClientStats`, `FrameReaderStats`, `ProsUsbStats`;
    - `*Result`: `PlanResult`.
  - Interfaces are plain nouns: `StateSource`, `PathSink`, `BytePort`,
    `MotorOutput`, `Kinematics`, `Follower`, `PathPlanner`.
- **Inconsistencies:**
  - The name stutters its namespace: `class Bno08xReports`
    (pico/aggreGATR/src/bno08x_reports.h:14) sits in `namespace bno08x`. Its
    neighbours are just `bno08x::Supervisor` (bno08x_supervisor.h:45) and
    `asm330::Supervisor` (asm330_supervisor.h:37).
  - Wire codes and domain enums use different name pairs:
    - `translagatr::LocalizationTopology` (link_documents.h:72, plain enum) vs
      `communigatr::LocalizationTopology` (robot_profile.h:22, enum class):
      same name in two namespaces;
    - `translagatr::ImuSourceCode` (link_documents.h:78) vs
      `communigatr::ImuSource` (robot_profile.h:28): Code suffix;
    - `translagatr::EstimateSourceCode` (frames.h:230) vs
      `investigatr::EstimateSource` (field.h:41): Code suffix.
  - Generated global `struct Field : investigatr::FieldReferences {`
    (operaGATR/include/field_references.h:4) has the same name as
    `investigatr::Field` (investigatr/field.h:59).
- **Proposed rules:**
  - N-1. Types are PascalCase nouns. Do not repeat the namespace in the type
    name (`bno08x::Reports`, not `bno08x::Bno08xReports`).
  - N-2. A wire-code enum and its domain enum never share a name. The wire
    twin always ends in `Code` (`LocalizationTopologyCode`), and the domain
    type keeps the plain name.
  - N-3. Keep the role suffixes: Config, Status, Stats, Result, Request, Reply.
    Interfaces are plain role nouns with no `I` prefix.

### 2.2 Functions and methods

- **Practice (lizard, non-test):** lowerCamelCase or one lowercase word.
  - brain-lib: 443 of 472 function names;
  - Pico: 159/161;
  - translaGATR: 83/83.
  - Every PascalCase hit is a constructor.
  - snake_case appears only for PROS entry points (`competition_initialize`,
    operaGATR/src/main.cpp:412).
- **Verb families already in use:**
  - `encodeX/decodeX` (frame_codec.h:98-131);
  - `toString(enum)`: 13 overloads;
  - `valid(config, why)` (drivetrain.h:64-66, motion_model.h:45) and
    `finite(x)` (geometry.h:45, chassis.h:33);
  - `makeX` (`makeProfileDocument`, robot_profile.h:98; `makeDiag`,
    diag_report.h:51; `makeBootId`, pi_link.h:12);
  - `xOf` (`readinessOf`, readiness.h:69; `snapshotOf`, link_events.h:34;
    `telemetryOf`, telemetry.h:23);
  - `toX/fromX` (`toBody`, chassis.h:31; `toProfileDoc`, robot_profile.h:84;
    `fromReading`, wheel_calibration.h:55).
  - Accessors are nouns with no `get`: `status()`, `config()`, `stats()`.
- **Inconsistencies:**
  - Boolean queries mix forms:
    - `isOpen()` (pros_serial_port.h:23), `isZero` (chassis.h:36) and
      `isTerminal` (motion.h:83);
    - `ready()` (client.h:315), `active()` (motion.h:166) and
      `connected(now)` (client.h:324).
  - Abbreviated private helpers:
    - `wr16/rd16/wr32/rd32` (frame_codec.cpp:12-31, again at
      link_documents.cpp:37-64);
    - `rdi32` (link_documents.cpp:58);
    - `i16/i32/u8` (actuGATR/src/telemetry.cpp:30-40).
- **Proposed rules:**
  - N-4. Functions and methods are lowerCamelCase verbs or verb phrases.
  - N-5. Accessors are bare nouns.
  - N-6. Boolean queries are bare adjectives or past participles (`open()`,
    `ready()`, `terminal()`); drop `is`. Alternatively keep `is` everywhere.
    Pick one: decision D-6.
  - N-7. Keep the existing verb families (encode/decode, toX/fromX, xOf,
    makeX, valid/finite, toString) and use them for new code.
  - N-8. No abbreviations beyond a short domain list: rx, tx, id, imu, crc
    and the unit suffixes. `cfg` is not on it. `writeU16LE`, not `wr16`.
  - Platform callbacks keep the platform's names (section 23).

### 2.3 Variables, parameters, members, globals

- **Practice:**
  - Locals and parameters are snake_case (`next_start`, `last_display`,
    operaGATR/src/main.cpp:324-326).
  - Short names are common in small scopes (`ProsLinkStatus s;`,
    pros_link.cpp:187; `actugatr::TankConfig c;`,
    operaGATR/include/robot_config.h:39).
  - Private members end in `_` in every concrete class (e.g. client.h:418-510,
    73 private data members). No struct field ends in `_`.
  - File-scope globals use a `g_` prefix: 39 in the apps, 38 in the Pico
    (pico main.cpp:72-89, operaGATR/src/main.cpp:41-56).
- **Inconsistencies:**
  - Some classes have public data members: `LinkRig::pi, bus, usb, nonces`
    (communiGATR/sim/link_rig.h:48-52) and `ScriptedPort::reads, writes`
    (communiGATR/tests/client_gtest.cpp:76-77).
  - Function-local statics have no prefix and hide state in functions:
    - `static uint32_t last_ms = 0;` (pico main.cpp:241);
    - `static actugatr::DriveSnapshot drive;` (operaGATR/src/main.cpp:241);
    - `static pros::Task task(background, "testing ui");`
      (operaGATR/src/main.cpp:403).
  - One multi-declaration line: `uint32_t g_boots = 0, g_retries = 0,
    g_rx = 0, g_bad_headers = 0;` (imu_bno08x.cpp:75).
- **Proposed rules:**
  - N-9. Variables, parameters and struct fields are snake_case.
  - N-10. Private and protected class data members end in `_`.
  - N-11. A class has no public data members. If everything is public, it is
    a struct.
  - N-12. File-scope mutable state is `g_` prefixed, in an anonymous
    namespace, and says which task owns it (see C-3).
  - N-13. One declaration per line.

### 2.4 Constants

- **Practice:** every non-test constexpr variable is kPascal (none found
  without the prefix).
  - Static constexpr members match: `kCapacity` (link_events.h:65,
    command_records.h:22, gyro_window.h:20).
  - The one non-constexpr namespace constant matches too:
    `const SPISettings kSpi(3000000, MSBFIRST, SPI_MODE3);`
    (imu_bno08x.cpp:41).
- **Inconsistencies:**
  - `double kP = 0; double kI = 0; double kD = 0;` (actugatr/pid.h:15-17,
    trimmed to one line) are mutable fields named like constants.
  - `static constexpr Reference Origin` and `RobotAtStart`
    (investigatr/reference.h:34-35) are PascalCase constants.
  - Generated `static constexpr investigatr::Reference NeutralGoal4North`
    (field_references.h:7) sits next to `kMapId` (line 5).
- **Proposed rules:**
  - N-14. Compile-time constants, including static constexpr members and
    generated tables, are kPascal: `Field::kRedGoal2West`,
    `FieldReferences::kOrigin`.
  - N-15. The k prefix is only for constants: PID gains become `p, i, d` or
    `proportional, integral, derivative`.

### 2.5 Enums and enumerators

- **Practice:**
  - enum class count: 51.
    - brain-lib, apps and Pico: every domain enum.
    - translaGATR: one (`enum class DocError : uint8_t`,
      link_documents.h:18).
  - Plain enum count: 35.
    - 33 are translaGATR wire codes with explicit values and a fixed
      underlying type (`enum FrameType : uint8_t {`, frames.h:22).
    - 2 are file-local index enums (actuGATR/src/kinematics.cpp:15-16).
  - Every enum states its underlying type.
  - About 348 enumerator lines are kPascal. Wire enums carry a group prefix
    because they are unscoped (`kResultOk`, `kProfileReasonFormat`,
    `kPicoImuReady`).
  - Nested small enums are named `Kind` or `Step`: `Reference::Kind`
    (reference.h:17), `DriveRequest::Kind` (drive_requests.h:24),
    `Client::Kind` (client.h:342), `DocAssembly::Step` (doc_assembly.h:19).
- **Inconsistencies:**
  - Pico enum classes have no k on 29 enumerators:
    - `Off, Wait, Probe, Reset, Configure, Run` (asm330_supervisor.h:20-27);
    - `None, EnableImu, DisableImu, ...` (pico_commands.h:19-26, trimmed);
    - `enum class Link : uint8_t { Held, Booting, Up };` (imu_bno08x.cpp:53).
  - The stored type differs from the enum type: wire structs hold enum values
    in `uint8_t` fields, which is correct for decoding unknown values. The same
    habit continues above the codec (section 3).
- **Proposed rules:**
  - N-16. Domain enums are `enum class Name : <fixed type>` with kPascal
    enumerators.
  - N-17. Wire code enums in translaGATR may stay plain `enum Name : uint8_t`
    with explicit values, a group prefix and a `Code` suffix on the type
    (N-2). Every value is written out; retired values are named in a
    comment, as frames.h:21 and :102 already do.
  - N-18. A field that holds a wire code in a wire struct is typed by an alias
    of the code, not a bare uint8_t (section 3).

### 2.6 Namespace names

- **Practice (Brain libraries):** namespace, include prefix and folder agree.
  - `communigatr` is included as `"communigatr/client.h"` from
    `brain/communiGATR/include/communigatr/`.
  - The same holds for `actugatr` and `investigatr`.
  - docs/naming.md records this mapping.
- **Inconsistencies:**
  - translaGATR:
    - its namespace is `translagatr`;
    - the Brain includes it as `"translaGATR/frames.h"` (folder case,
      client.h:22);
    - the Pico includes it flat as `"frames.h"` (pico_commands.h:11) through
      `-I$PROJECT_DIR/../../translaGATR` (platformio.ini:16).
  - The Pico has 7 top-level namespaces and none is the component's own:
    - `board` (board.h:12) and `cfg` (config.h:17);
    - `imu` (imu.h:11, imu_retry.h:8) and `pilink` (pi_link.h:8,
      pico_commands.h:13, diag_report.h:14);
    - `asm330` (asm330_supervisor.h:12) and `bno08x` (five headers);
    - `encoder` (quadrature.h:8).
  - Apps:
    - `gatr2_robot` (brain/robot/gatr2_robot.h:20) and `robot_config` in both
      apps (operaGATR/include/robot_config.h:24,
      locaGATR/include/robot_config.h:13);
    - everything else sits in an anonymous namespace in main.cpp.
  - Only one nested named namespace exists: `investigatr::collision`
    (investiGATR/src/collision.h:11-14), private to the library.
- **Proposed rules:**
  - N-19. Every component has one root namespace equal to its lowercased
    folder name: `aggregatr`, `translagatr`, `communigatr`.
  - N-20. Sub-namespaces mirror sub-folders when a component has sub-folders
    (`aggregatr::imu`, `aggregatr::board`).
  - N-21. Include paths spell the folder the same way everywhere. Either
    `"translagatr/frames.h"` everywhere (move the headers under an include
    dir) or `"translaGATR/frames.h"` everywhere including the Pico. Decision
    D-4.
  - N-22. Robot and program description namespaces are fine as nouns
    (`gatr2_robot`). Two programs may reuse `robot_config` because they never
    link together.

### 2.7 Files and folders

- **Practice:**
  - All 187 files are snake_case `.h`/`.cpp`. All 53 test files are
    `*_gtest.cpp`, and each is listed in CMake (investiGATR/CMakeLists.txt:24-35).
  - Every `.cpp` with its own header includes it first (59/59).
  - Brain library layout: `include/<ns>/`, `src/`, `pros/`, `sim/`, `tests/`
    (brain/actuGATR/README.md). A private header sits next to its source
    (investiGATR/src/collision.h: "Private to the investigatr library").
- **Inconsistencies:**
  - PROS-only headers live in the portable public include dir
    (`communigatr/pros_link.h`, `actugatr/pros_drive.h`). Their sources live
    in `pros/`. The platform split shows in the source tree but not in the
    header tree.
  - translaGATR:
    - it is flat and has no README;
    - its test project still carries the old name:
      `project(gatr2_common_tests CXX)` (translaGATR/tests/CMakeLists.txt:2)
      and `add_executable(common_tests` (line 37).
  - The Pico `src/` is flat: 29 files for 7 namespaces.
  - pico/aggreGATR/README.md is 211 lines, against "READMEs are short
    signposts".
- **Proposed rules:**
  - N-23. Folder = namespace = include prefix.
  - N-24. Platform-only code (PROS, Arduino) lives under a `pros/` or
    `hardware/` folder, headers included. Portable headers never include
    platform headers.
  - N-25. A component with more than about 12 source files gets sub-folders
    by concern (Pico: `board/`, `imu/`, `link/`, `encoder/`).
  - N-26. Rename leftovers are bugs: target and project names follow the
    folder.

### 2.8 Macros

- **Practice:**
  - No project `#define` in code.
  - Build-variant macros come from the build only: `GATR2_HAT_REV`,
    `GATR2_IMU_BNO08X` and `GATR2_IMU_ASM330` (platformio.ini:20-25). They are
    validated with `#error` (board.h:8-11, config.h:13-15).
  - The PROS template defines `PROS_USE_SIMPLE_NAMES` and `PROS_USE_LITERALS`
    (locaGATR/include/main.h:27, 35).
- **Proposed rules:**
  - N-27. No macros for constants or functions.
  - N-28. Build-variant macros are `GATR2_SCREAMING_CASE`, set only by the
    build, and checked with `#error` where first used.
  - N-29. Vendor template headers are left as shipped and marked vendor (see
    F-3).

---

## 3. `using` aliases

The user asked for aliases liberally; the lead's reading is "alias every
non-trivial type in a signature, member, container, callback, ID or unit;
never hide ownership or pointer-ness".

- **Practice:** only 17 alias declarations exist: 14 in Brain libraries, 1
  in an app, 3 in tests. translaGATR and Pico src have none.
  - Units, in investigatr/geometry.h:11-18:

    ```
    using Seconds          = double;
    using Meters           = double;
    using Radians          = double;
    using MetersPerSecond  = double;
    using RadiansPerSecond = double;
    ```

    `using FrameGeneration = uint32_t;` sits in the same block.
  - IDs:
    - `ObjectId` and `MapId` (field.h:15-16);
    - `CommandId` (path.h:15);
    - `PlacementTicket`, `ControlTicket` and `WheelTicket` (client.h:71, 90,
      125).
  - The app one: `using Link = communigatr::ProsLink;`
    (locaGATR/src/main.cpp:45).
  - 98 using-declarations (55 outside tests) re-export names into another
    namespace, many in public headers: `using investigatr::Meters;` and five more
    (actugatr/chassis.h:12-17); `using investigatr::CommandId;` and five more
    (actugatr/motion.h:30-35).
- **Inconsistencies and gaps:**
  - `communigatr::Seconds` has two homes:
    - `using Seconds = double;` (client.h:30);
    - `using investigatr::Seconds;` (startup_placement.h:16,
      wheel_calibration.h:34).
  - The aliases only document. `Seconds`, `Meters` and `Radians` are all
    `double`, so a `Meters` can be passed where `Seconds` is expected.
  - No alias for acceleration: `double max_accel = 1.0; // m/s^2`
    (motion_model.h:22).
  - Noisy types are spelled out at their use sites:
    - callbacks: `std::function<BenchImuSample()> bench_imu;` (client.h:68),
      `std::function<uint32_t()> nonce` (client.h:234) and
      `Seconds (*now)()` (pros_drive.h:36, again as a member at :67);
    - byte documents as `std::vector<uint8_t>` (client.h:179, 181, 483);
    - `std::vector<std::vector<pros::Motor>> groups_;`
      (pros_motor_output.h:30);
    - out-array params `const MotorGroup* out[kMaxWheelGroups]`
      (drivetrain.h:69).
  - Wire codes are held as bare `uint8_t` with the real type in a comment:
    - `uint8_t action = 0; // translagatr::ControlAction` (client.h:107);
    - `uint8_t health = 0; // translagatr::HealthBit` (link_events.h:27);
    - `uint8_t status = 0; // translagatr::PicoCommandStatus`
      (command_records.h:16);
    - `uint8_t state = 0; // translagatr::PicoImuState` (imu.h:17).
    - About 131 enum-valued `uint8_t` fields exist in non-test code, most in
      wire structs.
  - Pico time is always a raw `uint32_t ..._us`. No alias names the wrapping
    32-bit microsecond clock.
- **Proposed rules:**
  - A-1. Alias every unit, id, ticket, callback, buffer and wire code that
    appears in a signature or member. Examples:
    - `using BenchImuReader = std::function<BenchImuSample()>;`
    - `using Clock = Seconds (*)();`
    - `using DocumentBytes = std::vector<uint8_t>;`
    - `using ControlActionCode = uint8_t;`
    - `using MicrosU32 = uint32_t;` (wrapping clock)
  - A-2. Each alias has exactly one home: the header of the component that
    owns the concept. Other components re-export it with a using-declaration
    in their own namespace, or qualify it. Never re-alias the same name.
  - A-3. Never hide ownership or pointer-ness: no `using LinkPtr =
    std::unique_ptr<Link>`. `std::unique_ptr<Link>` stays visible; an alias
    may name the pointee.
  - A-4. Unit aliases document, they do not check. Where a mix-up already
    happened or would be silent and costly, use a strong type instead.
    Decision D-2; the survey saw no case needing it yet.
  - A-5. `using namespace` is banned in headers and production sources.
    Today it appears once in production: `using namespace translagatr;`
    (pico/aggreGATR/src/main.cpp:16). Tests: see TS-5.

---

## 4. struct vs class, free functions vs member functions

- **Practice (scan of every struct/class body):**

  | bucket | structs | data-only structs | classes | pure-interface classes |
  |---|---|---|---|---|
  | brain-lib | 95 | 90 | 35 | 7 |
  | translaGATR | 18 | 18 | 1 (FrameReader) | 0 |
  | pico | 13 | 13 | 10 | 0 |
  | apps | 3 | 3 | 0 | 0 |
  | sim | 13 | 13 | 10 | 0 |
  | tests | 23 | 12 | 11 | 0 |

  - The interfaces are the seams, each with `virtual ~X() = default` and pure
    virtuals:
    - `StateSource` (state_source.h:38) and `PathSink` (path.h:44);
    - `PathPlanner` (planner.h:46) and `Kinematics` (kinematics.h:22);
    - `MotorOutput` (motor_output.h:14) and `BytePort` (byte_port.h:12);
    - `Follower` (follower.h:59).
  - Concrete classes hold state with invariants:
    - protocol state machines: `Client`, `DocAssembly`, `UsbLineDecoder`,
      `FrameReader`;
    - controllers: `Pid`, `Motion`, `SegmentFollower`;
    - Pico state machines: `asm330::Supervisor`, `bno08x::Supervisor`,
      `imu::RetryPolicy`, `pilink::StatusSchedule`, `GyroWindow`.
  - Free functions carry the pure logic (lizard counts: free vs member):
    - brain-lib 160 vs 312, translaGATR 72 vs 11, Pico 85 vs 76, apps 71 vs 0;
    - `LinkReadiness readinessOf(const Client& client, Seconds now, bool
      accept_configured_anchor);` (readiness.h:69);
    - `translagatr::BrainTelemetry telemetryOf(const DriveSnapshot& snapshot);`
      (telemetry.h:23);
    - `Resolved resolve(...)` (reference.h:68);
    - `footprintClearance` (planner.h:93) and `desaturate` (kinematics.h:68);
    - the whole of `investigatr::collision` (collision.h:45-87).
  - The Pico hardware modules are namespaces of free functions over file-local
    state: `imu::begin/service/setEnabled/status` (imu.h:25-75) and
    `encoder::begin/count/zeroAll` (quadrature.h:13-22).
- **Inconsistencies (against "struct = attributes only"):**
  - Structs with methods:
    - `bool configured() const` (ProfileDocument, robot_profile.h:95);
    - `const FieldObject* find(ObjectId id) const` (Field, field.h:67);
    - `Meters length() const; bool empty() const`, trimmed (Path,
      path.h:39-40);
    - `bool valid() const` (RobotState, state_source.h:35);
    - the static factories `Reference::origin/robotAtStart/object`
      (reference.h:23-27);
    - `struct ProsUsbPort::Io` with `transmit()`, `receive()` and atomics
      (pros_usb_port.cpp:69-161), a full task object declared as a struct.
    - In tests, 11 helper structs have methods (for example the rigs in
      actuGATR/tests).
  - The same concept is written two ways: `valid(const MotionModel&, const
    char**)` is a free function (motion_model.h:45), while
    `RobotState::valid()` is a member (state_source.h:35).
  - Classes with public data: see N-11.
  - Candidates for "class for its own sake" (not verdicts):
    - `GeometricPlanner` holds only its config and has one const method
      (planner.h:72-83). It exists to implement `PathPlanner`, whose other
      implementation is the test planner actugatr/sim/straight_planner.h.
    - `PortMap` (ports.h:15-33) accumulates devices and could be a function
      over a list.
    - `StartupPlacement` (startup_placement.h:43-61) is a 3-method state
      machine, which is justified.
  - `Client` is the one oversized class:
    - 73 private data members (client.h:418-510), 40 private methods, about
      35 public methods;
    - a 1351-line source file;
    - `Client::choose` is 143 NLOC with CCN 56 (client.cpp:869).
- **Proposed rules:**
  - S-1. `struct` = data only: public fields with default initializers, no
    methods, no invariants. Constructors, static factories and member
    functions make it a class.
  - S-2. Behavior on a data struct is a free function in the same namespace
    and header: `bool valid(const RobotState&)`, `Meters length(const Path&)`,
    `const FieldObject* find(const Field&, ObjectId)`. This matches the
    existing `valid(...)`, `finite(...)` and `toString(...)` families.
  - S-3. `class` = state with an invariant that methods protect, or an
    interface. Before writing one, answer in the review: which invariant does
    it guard? If none, write free functions over a struct.
  - S-4. Interfaces are extracted, not invented. An interface exists only for
    a real seam: two implementations, or one implementation plus a host
    fake/sim used by tests (the current practice). It has a virtual default
    destructor and nothing else but pure virtuals.
  - S-5. Leaf implementations are `final`. Only `DifferentialFollower` and
    `HolonomicFollower` are today (follower.h:125, 139); `TankKinematics`,
    `ProsLink` and others are not.
  - S-6. A class that grows past a size threshold (for example 25 data
    members or one function over CCN 25) is split by concern. `Client` is the
    first candidate.

---

## 5. Namespace placement

- **Practice:**
  - Every brain-lib, sim and translaGATR file, and every Pico file except
    main.cpp, wraps its code in a named namespace.
  - Internal helpers sit in anonymous namespaces: 87 blocks, tests included,
    e.g.
    pros_link.cpp:15-54 and frame_codec.cpp:10.
  - No file-scope `static` functions (0 found).
  - Namespace style follows the memory rule: Allman brace, unindented
    contents, and a `} // namespace x` close comment.
- **Outside any namespace (non-test, 13 declarations):**
  - PROS callbacks `initialize`, `disabled`, `competition_initialize`,
    `autonomous` and `opcontrol` in both apps (operaGATR/src/main.cpp:390-423;
    declared `extern "C"` by the template, operaGATR/include/main.h:61-71).
  - Arduino `setup` and `loop` (pico/aggreGATR/src/main.cpp:299, 319).
  - Generated `struct Field` (operaGATR/include/field_references.h:4).
- **Tests:**
  - 9 test files have no namespace at all (e.g.
    actuGATR/tests/chassis_gtest.cpp).
  - Helpers with external linkage sit at global scope after an anonymous
    block closed: `struct BusRun`, `BusRun busRun(...)` and
    `Seconds telemetryExchange(...)`
    (communiGATR/tests/client_telemetry_gtest.cpp:524, 538, 587). All test
    files of a component link into one binary, so a same-named helper in
    another test file is an ODR clash.
- **Inconsistencies:**
  - Namespace brace attached in two Pico headers: `namespace bno08x {`
    (bno08x_reports.h:9, gravity_alignment.h:7).
- **Proposed rules:**
  - NS-1. Everything is in the component namespace. The only exceptions are
    platform callbacks whose names the platform fixes (PROS competition
    functions, Arduino setup/loop). They stay one-line trampolines into
    namespaced code.
  - NS-2. Anything not used outside its `.cpp` goes in an anonymous namespace
    inside the component namespace. No file-scope `static` functions.
  - NS-3. Generated code is namespaced too, e.g. `operagatr::field::kRedGoal2West`.
  - NS-4. Test helpers go in an anonymous namespace; `TEST` bodies stay at
    global scope, as gtest requires.

---

## 6. Units and frames in names

- **Practice, integer and wire units (suffixes):**
  - `_ms` 193 hits, `_us` 189 (all Pico), `_mm` 295, `_cdeg` 140, `_mdeg` 64
    and `_um` 137;
  - rate suffixes `_mm_s` and `_cdeg_s` (frames.h:300-302), `_mdps`
    (imu.h:49), `_radps` and `_mps2` (gravity_alignment.h:29, 36);
  - scaled `_x10` (frames.h:308) and fixed-point `_q9` (gyro_window.h:27);
  - constants take a camel suffix: `kFooMs` 31 and `kFooUs` 99 (e.g.
    `kStopWaitMs`, pros_link.cpp:19; `kBackoffMinUs`, imu_retry.h:12).
- **Practice, Brain SI floating point (aliases, no suffix):**
  - `Meters radius` and `Radians angle` (robot_profile.h:41, 45);
  - `Seconds command_timeout` (drive.h:32);
  - use counts: Seconds 368, Meters 135, Radians 99.
- **Practice, frames:** named only in comments and by one runtime tag.
  - "field frame" appears 22 times, "robot frame" 16 and "body frame" 6.
  - `Pose` is used for both frames: `Pose start; // field frame`
    (path.h:26) and `Meters x = 0; // contact point, robot frame`
    (robot_profile.h:43).
  - The one frame tag in code: `enum class ChassisFrame : uint8_t { kBody,
    kField };` (chassis.h:19). `Drive` refuses the wrong one: `kWrongFrame,
    // field frame command; convert with toBody first` (drive.h:23).
- **Inconsistencies:**
  - Unit spelling varies:
    - `kStartHeadingDegrees` (gatr2_robot.h:170) vs `kVexImuMountYawDeg`
      (gatr2_robot.h:53);
    - `kStartupWaitSeconds` (locaGATR/include/robot_config.h:20) vs
      `kTelemetryPeriodMs` (gatr2_robot.h:61).
  - The unit is only in a comment:
    - `constexpr double kCalibrationDistance = 1.0; // meters`
      (locaGATR/include/robot_config.h:24);
    - `double max_accel = 1.0; // m/s^2` (motion_model.h:22);
    - `l.max_speed = 0.8; // m/s` (operaGATR/include/robot_config.h:85).
  - A raw `double` carries degrees in a SI world: `double roll_deg` in
    `VexAttitude` (attitude.h:27). This is fine because the suffix says so,
    but it is the only degrees-valued struct field.
- **Proposed rules:**
  - U-1. Integer and fixed-point quantities carry the unit suffix
    (`_ms, _us, _mm, _um, _cdeg, _mdeg, _mdps`) in fields and variables, and
    `Ms, Us, Mm` in kConstants. Spell the suffix the same way everywhere:
    `Deg`, never `Degrees`; `S` or `Seconds`, pick one (decision D-5).
  - U-2. SI floating-point quantities use the unit alias type (Meters,
    Radians, Seconds, MetersPerSecond, and new `MetersPerSecondSq`,
    `RadiansPerSecondSq`) and no suffix. A raw `double` with a unit comment
    is not allowed.
  - U-3. Non-SI floating values (degrees in configs, VEX angles) keep an
    explicit suffix (`_deg`) and are converted at the boundary by a named
    helper (R-2).
  - U-4. Frames: the frame is part of the name whenever it is not the
    module's stated default. Either a prefix (`field_pose`, `robot_point`) or
    aliases (`using FieldPose = Pose; using RobotPoint = Point;`). Each module
    header states its default frame in one line, as geometry.h:2-3 does.
    Decision D-3: alias or name.

---

## 7. Integer types

- **Practice:**
  - The wire and the Pico are fixed-width everywhere.
  - Brain SI values are `double`; counts and sizes are `std::size_t` (241
    uses in the Brain).
  - Headers: the Brain includes `<cstdint>` (43) and `<cstddef>` (15);
    translaGATR and the Pico include `<stdint.h>` (22) and `<stddef.h>` (7),
    for C compatibility with the vendored sh2 C code. All code writes
    unqualified `uint8_t` (0 uses of `std::uint8_t`).
- **Inconsistencies:**
  - One byte path uses three length types:
    - `virtual int read(uint8_t* buf, int max) = 0;` (byte_port.h:18);
    - `std::size_t encodeUsbLine(const uint8_t* frame, std::size_t len, ...)`
      (usb_line.h:29, trimmed);
    - `uint16_t crc16(const uint8_t* data, uint16_t len);` (frame_codec.h:68).
  - `int` for counts and attempts sits next to fixed-width fields:
    - `int placement_attempts = 10;` (client.h:52);
    - `int turn_direction = 0; // kTurn: +1 CCW, -1 CW` (path.h:29), which
      is really a sign enum.
  - `long` is used for rounding: `long saturate(double value, long lo, long
    hi)` (telemetry.cpp:17) and `long cdeg` (attitude.cpp:18).
  - Bare `size_t` (Pico src, translaGATR tests: 85) vs `std::size_t` (Brain).
  - The wrap-safe unsigned time compare is written inline at 9 sites:
    - `static_cast<int32_t>(now - g_next_tick) >= 0`
      (pico/aggreGATR/src/main.cpp:326);
    - operaGATR/src/main.cpp:368, 371;
    - locaGATR/src/main.cpp:652, 655;
    - gravity_alignment.cpp:23, 45 and bno08x_reports.cpp:11.
    - Only the Pico names it once: `constexpr bool atOrBefore(uint32_t a,
      uint32_t b)` (gyro_window.h:12).
- **Proposed rules:**
  - I-1. Wire fields, hardware registers, counters that wrap, and
    timestamps are fixed-width.
  - I-2. Sizes, indexes and counts in the Brain are `std::size_t`.
  - I-3. `int` only for small signed arithmetic. Named sign values are enums.
  - I-4. One length type per layer:
    - byte buffers and the codec: `uint16_t`, set by `kMaxFrameLen`;
    - transports: `std::size_t` (`BytePort` moves to it).
  - I-5. Pick one header style per target. `<cstdint>` for C++ everywhere is
    simplest; keep `<stdint.h>` only in headers that C code includes (none
    today).
  - I-6. Wrapping clocks compare only through one named helper per clock
    type (`atOrBefore`, `elapsedSince`), never through an inline cast.
  - I-7. Narrowing to a wire type goes through a named saturating or checked
    helper. These exist per file today (section 18).

---

## 8. Time representation and clock domains

- **Practice:**
  - The Brain has one steady clock: `static Seconds now();` "Poll task clock:
    seconds since PROS started" (pros_link.h:113-115).
    - It is implemented as `static_cast<double>(pros::micros()) * 1e-6`
      (pros_link.cpp:136).
    - It is handed to the drive task as a function pointer
      (`&communigatr::ProsLink::now`, operaGATR/src/main.cpp:175).
  - Portable logic never reads a clock; time is a parameter:
    - 48 Brain declarations take `Seconds now`:
      `void poll(Seconds now);` (client.h:238), `ChassisCommand update(Seconds
      now);` (motion.h:169);
    - 19 Pico declarations take `uint32_t now_us`: `Action step(uint32_t
      now_us);` (asm330_supervisor.h:48).
    - A scan for `pros::millis/micros`, `micros()` and `time_us_*` finds them
      only in pros/, the apps and Pico main.cpp and driver files.
  - Clock domains are named in comments at the boundary:
    - `uint32_t stamp_ms; // Pico acquisition clock; the Pi maps it to host
      time` (frames.h:71);
    - `uint32_t imu_stamp_ms = 0; // Brain acquisition clock, not the Pico
      clock` (frames.h:326);
    - "issued_at: when the command was computed, on the same clock as now"
      (drive.h:54).
  - The Pico has two clocks:
    - the wrapping 32-bit `micros()`/`time_us_32()` for scheduling (main.cpp:325);
    - 64-bit `time_us_64()` for stamps (main.cpp:159) and for ages that must
      not wrap ("64-bit so a long silence cannot wrap the age",
      imu_bno08x.cpp:78).
- **Inconsistencies:**
  - Apps mix `Seconds` from `now()` with raw `pros::millis()` in one loop:
    `const Seconds t = now();` (operaGATR/src/main.cpp:328) and
    `pros::millis() - next_telemetry` (line 368).
  - Durations are stored three ways:
    - `Seconds` in configs (client.h:46-61);
    - `uint32_t ..._ms` in task configs (`uint32_t period_ms = 10;`,
      pros_drive.h:27);
    - `constexpr double kStartupWaitSeconds` (locaGATR/include/robot_config.h:20).
- **Proposed rules:**
  - TM-1. Portable code never reads a clock. Every time-dependent function
    takes `now` (and `dt` where needed) as a parameter. This is the current
    practice; the standard should make it law.
  - TM-2. One steady clock per program, injected where needed: `ProsLink::now`
    on the Brain, `time_us_32`/`time_us_64` on the Pico.
  - TM-3. Durations and instants in Brain logic are `Seconds`. PROS task
    periods and delays stay `uint32_t ..._ms` at the PROS call.
    - Decision D-5: whether program configs use `Seconds` or `_ms`. Today
      the apps use both.
  - TM-4. Every timestamp field names its clock domain in its declaration
    comment, or through an alias such as `PicoMs`, `BrainMs` or `HostSeconds`.
    Values from different domains are never compared without a mapping
    function.
  - TM-5. Wrapping clocks compare through I-6.

---

## 9. Error handling

- **Practice: seven shapes are in use.**
  1. Bool plus out-param: `bool decodeSensorFrame(const uint8_t* buf,
     uint16_t len, SensorSample& out);` (frame_codec.h:104).
  2. Zero sentinel:
     - "Returns bytes written, or 0 if the input is not encodable"
       (frame_codec.h:91);
     - tickets are "0 (refused)" (client.h:240-243).
  3. Status enum return:
     - `DriveFault apply(const ChassisCommand& command, Seconds issued_at,
       Seconds now);` (drive.h:55);
     - `DocError validateFieldMap(...)` (link_documents.h:227);
     - `TrialStatus start(...)` (wheel_calibration.h:107).
  4. Result struct with a status field: `PlanResult { PlanStatus status;
     Path path; ObjectId blocking; }` (planner.h:40-44, trimmed) and
     `Resolved` (reference.h:57-62).
  5. A reason string through an out pointer: `bool valid(const MotionModel&
     model, const char** why = nullptr);` (motion_model.h:45). It is used
     15 times, e.g. drivetrain.h:64-66.
  6. Reason and detail out-params: `bool validateRobotProfile(const
     RobotProfileDoc& in, uint8_t& reason, uint8_t& detail);`
     (link_documents.h:152) and `toProfileDoc` (robot_profile.h:84).
  7. errno on PROS: "False when it could not be created (errno set)"
     (pros_link.h:109-110); `errno = ENOMEM;` (pros_link.cpp:127).
  - Also: an error buffer inside the object (`PortMap::error()`,
    ports.h:17-24), counters instead of errors (section 17), and
    `std::optional` once, for an optional config value
    (`std::optional<RobotProfile> profile;`, pros_link.h:50).
- **Degrade, don't hang, is practiced consistently:**
  - a busy link answers with a safe value: "Busy: an unsettled answer for a
    nonzero ticket" (pros_link.cpp:269-280);
  - `return investigatr::RobotState{}; // kNoLink` (pros_link.cpp:237);
  - the Pico skips a tick rather than block (main.cpp:329-334);
  - `Drive` stops on a stale command (drive.h:2-5);
  - operaGATR keeps the drive off on a config error (main.cpp:397-402).
- **Exceptions:** none. 0 `throw` and 0 `try/catch` in 187 files.
  - Pico builds use `-fno-exceptions` and `-fno-rtti` (section 23).
  - PROS builds pass neither flag (locaGATR/common.mk:39-42).
- **Ignored results:** there is no `[[nodiscard]]` (0 uses) and one
  `(void)` (a test).
  - Best-effort calls drop results silently, with a comment when it is
    deliberate: `g_task->status(drive);` after `// kept when a read is busy`
    (operaGATR/src/main.cpp:241-242).
  - Some drop them with no comment: `serial_->open();` (pros_link.cpp:172)
    and `g_link->reportTelemetry(t);` (operaGATR/src/main.cpp:287).
- **Proposed rules:**
  - E-1. No exceptions and no RTTI in any code that the Pico or translaGATR
    builds. The Brain and host follow the same rule, for one style; decision
    D-7.
  - E-2. Choose the error shape by what the caller needs:
    - (a) pass or fail only: return `bool`, results in an out-param or a
      struct;
    - (b) the caller branches on why: return an `enum class` status, or a
      result struct whose first field is `status`;
    - (c) a human message: an enum plus `toString(enum)`.
    - Retire `const char** why` in favor of (b) or (c), since it has no
      compile-time check.
    - Zero-sentinel tickets and lengths stay where the wire defines 0 as
      "none", named: `constexpr PlacementTicket kNoTicket = 0;`.
  - E-3. Every public function whose return reports failure is
    `[[nodiscard]]`. A deliberate discard is written
    `static_cast<void>(call()); // best effort`.
  - E-4. errno only at the PROS boundary (pros/ files). Portable code never
    reads or sets errno.
  - E-5. Degrade, never hang: every wait is bounded, every consumer has a
    defined answer when its upstream is absent, and the answer is documented
    at the declaration (current practice, pros_link.h:7-11).

---

## 10. Ownership and lifetime

- **Practice:**
  - References are stored as members for required dependencies that outlive
    the holder: `const Kinematics& kinematics_;`, `MotorOutput& output_;`
    (drive.h:67-69) and `Client& client_;` (link_driver.h:80).
  - Pointers are used for optional or rebindable dependencies:
    - `investigatr::StateSource* source_;` (motion.h:184), rebound by
      `setSource`;
    - `investigatr::PathSink* sink_ = nullptr;` (motion.h:189), where
      nullptr means none.
  - Lifetimes are stated in comments: "owner and drive must outlive this
    object" (pros_drive.h:34).
  - The apps own everything through `std::unique_ptr` globals created once in
    `initialize()` (operaGATR/src/main.cpp:41-51).
  - Resource-owning PROS classes delete copying: `ProsLink(const ProsLink&) =
    delete;` (pros_link.h:106-107), plus ProsDrive, ProsUsbPort and
    ProsLink::Lock.
  - Everything else is rule of zero.
- **Inconsistencies:**
  - There is no `std::make_unique` (0 uses). Instead:
    - `g_vex.reset(new communigatr::ProsVexImu(gatr2_robot::kVexImuPort));`
      (operaGATR/src/main.cpp:133), 14 times in operaGATR;
    - `task_.reset(new pros::Task(...))` (pros_drive.cpp:11, trimmed);
    - `? new ProsSerialPort(config.smart_port, config.baud)`
      (pros_link.cpp:87).
  - One raw owning pointer: `Io* io_;` (pros_usb_port.h:56), deleted by hand
    (`delete io_;`, pros_usb_port.cpp:179).
    - It is leaked on purpose when a task is still inside a USB call: "a task
      may own a kernel serial mutex: never pull its state away"
      (pros_usb_port.cpp:174).
    - The reason is good, but the pattern is invisible in the type.
  - Classes holding reference members (`Client`, `Drive`, `Motion`,
    `LinkDriver`) are implicitly copy-constructible, and a copy would alias
    the port or motors. Copy is deleted only on the PROS classes.
- **Proposed rules:**
  - O-1. Ownership is visible in the type:
    - `std::unique_ptr` owns;
    - `T&` is a required non-owning dependency that outlives the holder;
    - `T*` is optional or rebindable and non-owning;
    - a raw owning pointer is never used.
    - The one intentional leak is written as `std::unique_ptr<Io>` plus
      `release()` with a comment at the release.
  - O-2. Create with `std::make_unique`, never `reset(new ...)`.
  - O-3. A class that holds a reference or a hardware handle deletes copy and
    move unless it has a reason to allow them.
  - O-4. The declaration comment of every reference member or constructor
    reference parameter states who must outlive whom, as pros_drive.h:34 does.

---

## 11. Concurrency

- **Practice (Brain):**
  - The core is single-threaded; tasks live only in thin PROS shells.
    - "No clock and no threads: poll(now) does all I/O and never blocks"
      (client.h:5-6).
    - "Not synchronized: ProsDrive guards it with a mutex held only to copy"
      (drive_requests.h:5-6).
  - Tasks:
    - the link poll task (`pros::c::task_create`, pros_link.cpp:122-123);
    - USB tx and rx tasks (pros_usb_port.cpp:205, 214);
    - the drive task (`pros::Task`, pros_drive.cpp:11);
    - one app background task each (operaGATR/src/main.cpp:403,
      locaGATR/src/main.cpp:673).
  - Every mutex take is bounded, and that is written down: "PROS deletes
    competition tasks at mode changes without releasing mutexes they hold.
    Every take is bounded." (pros_drive.h:7-8).
  - Atomics:
    - flags and counters use default `seq_cst` `.load()`/`.store()`
      (pros_drive.h:75-80, pros_link.h:176-183);
    - one SPSC queue uses explicit acquire/release (pros_usb_port.cpp:33-59);
    - counters use relaxed order (pros_usb_port.cpp:61-63).
  - Task ownership of state is noted in comments:
    - `// Background task only.` (operaGATR/src/main.cpp:293);
    - `// VEX IMU recalibration, after the Pi's stillness check. opcontrol
      only.` (line 290);
    - `uint32_t handled_stop_ = 0; // task only` (pros_drive.h:77).
- **Practice (Pico):**
  - Single core; ISRs share counters through `volatile int32_t count`
    (quadrature.cpp:24).
  - Multi-field reads and resets wrap `noInterrupts()`/`interrupts()`
    (quadrature.cpp:78-84, imu_bno08x.cpp:95-101).
  - No `setup1/loop1` (the second core is unused).
- **Inconsistencies:**
  - Two lock styles for the same bounded take:
    - a RAII guard, `class ProsLink::Lock` (pros_link.cpp:64-82);
    - hand-written `mutex_.take(...)` / `mutex_.give()` pairs, 7 sites in
      pros_drive.cpp (e.g. :28-34). An early return added later would leak
      the lock.
  - Two task APIs: `pros::c::task_create` (pros_link.cpp:122) vs `pros::Task`
    (pros_drive.cpp:11), and function-local `static pros::Task` in the apps.
  - Unsynchronized cross-task globals in operaGATR:
    - `const char* g_message` and `double g_speed_scale`
      (operaGATR/src/main.cpp:55-56);
    - both are written in `opcontrol()` (lines 435, 464);
    - `g_message` is also written in `background()` (line 351), and both are
      read by `display()` in the background task (lines 257, 261).
  - Competition-task link calls in operaGATR:
    - docs/communigatr.md:203-207 says "Run loops that call the link in your
      own tasks, as both test programs do";
    - but operaGATR `opcontrol()` calls `g_link->place(...)` (line 471),
      `g_link->recalibrate()` (478, 482) and `g_link->control(...)` (491)
      directly;
    - locaGATR runs everything in its own task (locaGATR/src/main.cpp:673)
      and has an idle `opcontrol()` (line 680).
  - A platform limit that is in the docs only: "`std::shared_ptr` is not
    shared between tasks: the PROS toolchain has no atomic reference counts"
    (docs/communigatr.md:208-209).
- **Proposed rules:**
  - C-1. Portable libraries have no threads, no locks and no atomics. Tasks,
    mutexes and atomics live only in `pros/` shells.
  - C-2. Every PROS mutex take is bounded, through one RAII guard type shared
    by all shells. There is no bare take/give.
  - C-3. Every piece of mutable state that more than one task touches is
    either atomic or guarded, and its declaration comment names the lock.
    State touched by one task says which task. operaGATR's `g_message` and
    `g_speed_scale` violate this today.
  - C-4. Competition callbacks (`autonomous`, `opcontrol`) only post requests
    or run bounded calls. Program loops that talk to the link run in the
    program's own task (the documented rule, made mandatory).
  - C-5. No `std::shared_ptr` across PROS tasks. Hand out value copies.
  - C-6. Pico: ISR-shared data is `volatile`, one word, written by the ISR
    only. Multi-word access or reset happens with interrupts off, in one
    named function.
  - C-7. Task creation uses one API (`pros::Task`), with a name, and the
    priority and stack taken from config constants.

---

## 12. Heap allocation in real-time paths

- **Practice:**
  - Pico src: 0 `new`, 0 `std::vector`, 0 `std::string`. Buffers are fixed
    arrays sized by constants: `Report buf_[kCapacity]` (gyro_window.h:46)
    and `uint8_t g_tx[kBufLen];` (imu_bno08x.cpp:63).
  - translaGATR src: no heap. "Fixed layout records with explicit counts,
    little endian, no heap." (link_documents.h:4). `FrameReader` has a
    static buffer ("Bounded so every parser can use a static buffer",
    frames.h:18).
  - The Brain protocol is mostly allocation-free per cycle:
    - `DocAssembly` is "One document read with READ_DOC chunks into a buffer
      allocated once" (doc_assembly.h:2);
    - `UsbLineDecoder` and `FrameQueue` use fixed arrays (usb_line.h:53-56,
      pros_usb_port.cpp:56).
- **Allocation in control paths (Brain):**
  - The planner allocates on every plan:
    - `std::vector<Region> boxes;` (planner.cpp:41);
    - `std::vector<double> g(n + 1, kInf);` and three more vectors per search
      (planner.cpp:368-371);
    - `push_back` throughout `shape()` (planner.cpp:482-548).
    - It runs in the drive task: "planning, path reports and Pi link reads
      all run in the drive task" (pros_drive.h:5).
    - It is bounded: `max_vertices = 512; // graph bound, kNoPath beyond it;
      work grows as its square` (planner.h:63).
  - Field copies: `std::vector<FieldObject> objects;` (field.h:63) is copied
    out under the link mutex by `ProsLink::field` (pros_link.cpp:242-249).
  - `Client` publishes documents as `std::vector<uint8_t>` (client.h:179-181)
    and calls a `std::function` every state poll (client.h:67-68).
- **Proposed rules:**
  - H-1. Pico and translaGATR: no heap at all. No `new`, no standard
    containers that allocate, no `std::function`, no `std::string`. Capacity
    comes from named constants, with `static_assert`s where sizes interact
    (the Pico main.cpp:21-33 style).
  - H-2. Brain periodic loops (poll task, drive task, USB tasks) do not
    allocate in steady state.
    - Allocation is allowed at startup and on events: a new plan, a new
      field document or a profile change. Each such site is bounded by a
      named limit (`max_vertices`).
    - Containers reused across cycles call `reserve` once at construction.
  - H-3. Where the heap is used in a task, the header says so in one line
    ("allocates per plan, bounded by max_vertices").
  - H-4. Prefer fixed arrays plus a count for small wire-sized collections.
    This is the current pattern: `MotorGroup { count; motors[4] }`
    (drivetrain.h:29-32). Wrapping them in a tiny `FixedList<T, N>` is only
    worth it if it replaces several hand-rolled count/array pairs
    (extraction, not invention).

---

## 13. Header layout and includes

- **Practice:**
  - `#pragma once` in every project header (68/68). Include guards appear
    only in the PROS template (`_PROS_MAIN_H_`).
  - Layout, in 175 of 187 files:
    - line 1 is `// <file name>`;
    - then a short description block;
    - `#pragma once`, then `<system>` includes, a blank line and project
      includes in alphabetical order;
    - then the namespace.
  - In a `.cpp` the own header comes first (59/59).
  - Forward declarations: only two, both private implementation types:
    `class Lock;` (pros_link.h:159) and `struct Io;` (pros_usb_port.h:55).
  - Headers include what they use.
  - Include roots:
    - the Brain uses `"<ns>/<file>.h"` and `"translaGATR/<file>.h"`;
    - sims use `"sim/<file>.h"`;
    - apps use `"gatr2_robot.h"` and `"robot_config.h"`;
    - the Pico uses flat names.
- **Inconsistencies:**
  - 12 files lack the `// <file name>` line:
    - pico/aggreGATR/src/main.cpp, bno08x_reports.h and .cpp,
      gravity_alignment.h and .cpp;
    - two Pico tests;
    - the generated field_references.h;
    - the PROS template files.
  - The include order drifted after the gatr2 -> translagatr rename:
    `"translaGATR/link_documents.h"` sorts before `"investigatr/..."` by hand,
    and clang-format moves it (robot_profile.h:12).
  - A seam leak by include: `actugatr/telemetry.h` includes
    `"translaGATR/frames.h"`. The CMake comment reads "frames.h only: the
    TELEMETRY struct, no codec." (actuGATR/CMakeLists.txt:20).
  - Wire parsing outside the codec: `rd16` and `readHeader` in
    pico/aggreGATR/src/pico_commands.cpp:12-30 re-read a command header that
    the codec refused.
- **Proposed rules:**
  - HD-1. `#pragma once`, then includes in three blocks separated by blank
    lines:
    - own header (in a .cpp);
    - C/C++ standard and platform headers;
    - project headers.
    - Blocks are sorted by clang-format (IncludeBlocks: Regroup with explicit
      categories).
  - HD-2. A header includes what it uses and nothing else. Forward-declare
    only private implementation types.
  - HD-3. Project includes are rooted at a component include dir and spelled
    `"<component>/<file>.h"`. No flat includes of another component's headers.
  - HD-4. Only translaGATR parses wire bytes. A component that needs a partial
    decode asks translaGATR for a function.
  - HD-5. The `// <file name>` first line: keep it, as 175 of 187 files do, or
    drop it everywhere. Decision D-8. The user's comment rules lean to
    dropping it, since the file name is self-evident.

---

## 14. const, constexpr, initializers, explicit, special members

- **Practice:**
  - Default member initializers are nearly universal. Scanning struct and
    class bodies finds 878 initialized builtin members against 43 without.
  - Most of the 43 are class members set in the constructor (`uint8_t port_;`,
    pros_serial_port.h:39).
  - Data-struct exceptions without initializers:
    - `SensorSample` (translaGATR/frames.h:69-81), used as `SensorSample s{};`
      (pico main.cpp:161);
    - `Vector3 { double x; double y; double z; }` (gravity_alignment.h:9-13,
      trimmed);
    - `DiagPin` (diag_report.h:18-21);
    - `GyroWindow::Report` (gyro_window.h:41-44);
    - `double d[8];` in `collision::Region` (collision.h:38).
  - `constexpr`:
    - every constant;
    - constexpr helper functions where compile-time checks use them:
      `distinctPorts` (ports.h:36), `pinsUnique` (config.h:63), `airtimeUs`
      (pi_link.h:15), `shtpRxLen` (shtp_header.h:17), `atOrBefore`
      (gyro_window.h:12).
  - `explicit` on every single-argument constructor found (30 uses; a regex
    for non-explicit one-argument constructors in headers found none).
  - const-correct accessors (`const DriveStatus& status() const`, drive.h:60)
    and `const` locals are common (`const Seconds t = now();`,
    pros_link.cpp:203).
  - Brain configs are built by named field assignment, e.g.
    operaGATR/include/robot_config.h:38-55 and brain/robot/gatr2_robot.h:77-89.
- **Inconsistencies:**
  - Positional aggregate initialization of multi-field structs:
    - `PidGains along{3.0, 0.0, 0.0, 0.0, 100.0};` (follower.h:34), repeated
      in operaGATR/include/robot_config.h:99-102;
    - `p.wheels = {{0, 0.024, 2048, 0.0, 0.12, 0.0}, ...}` (TrackingWheel,
      communiGATR/tests/readiness_gtest.cpp:30, trimmed);
    - `constexpr investigatr::Footprint kFootprint{0.23, 0.23, 0.23, 0.23};`
      (gatr2_robot.h:122).
    - C++17 has no designated initializers, so these are unreadable and
      order-fragile.
  - `mutable` data in test doubles for counting const calls:
    `mutable int plans_ = 0;` (actuGATR/sim/straight_planner.h:34). Fine in
    sims, noted.
- **Proposed rules:**
  - K-1. Every field of a struct and every class data member has a default
    member initializer, unless a constructor always sets it. Wire structs
    included (`SensorSample`).
  - K-2. Positional brace initialization only for types of up to 3 fields
    whose order is obvious (Point{x, y}, Pose{x, y, heading}). Anything else
    is built by named field assignment, or by a named factory function.
    Designated initializers are C++20: not allowed while host builds are
    C++17.
  - K-3. `explicit` on every constructor callable with one argument.
  - K-4. `constexpr` for every constant and every function a
    `static_assert` uses. `const` for every local that is not reassigned.
    const member functions for every accessor.
  - K-5. Rule of zero by default. Deleted copy per O-3. A virtual destructor
    only on interfaces.

---

## 15. switch and enum exhaustiveness

- **Practice:**
  - `toString(enum)` is written as an exhaustive switch with no `default` and
    a fallback return after the switch, so `-Wswitch` catches a new
    enumerator (13 functions; e.g. motion.cpp:89-117 and
    pros_link.cpp:56-62).
  - Switches over wire codes (`uint8_t`) always have a `default`
    (`switch (reply.result)`, client.cpp:503, 586, 652, 695). That is correct,
    because unknown values arrive from newer peers.
- **Inconsistencies:** a `default` label on an enum class switch hides new
  enumerators. About 8 sites:
  - `default: return MotionReason::kInputUnavailable;` over `RobotStatus`
    (motion.cpp:256);
  - `default: status_.state = MotionState::kRunning; break;` over
    `FollowState` (motion.cpp:484);
  - `default: return Readiness::kReady;` over `ImuUse` (readiness.cpp:49);
  - `default:` over `Client::Kind` (client.cpp:430, 807) and `ProfileSync`
    (client.cpp:1211);
  - `default: g_message = "No answer: check the link"; break;` over
    `ControlResult` (locaGATR/src/main.cpp:646).
- **Proposed rules:**
  - W-1. A switch over an enum class lists every enumerator and has no
    `default`. Enumerators that share an outcome are stacked as case labels.
    `-Wswitch` (in -Wall) then flags a new enumerator.
  - W-2. A switch over a raw wire code has a `default` that maps to the
    "unknown" value or rejects the input.
  - W-3. Consider `-Wswitch-enum`. It is noisy with wire-code enums, so it
    applies only to portable Brain libraries; decision D-1 with the warning
    set.

---

## 16. assert and static_assert

- **Practice:**
  - 0 `assert(` in any file.
  - Runtime preconditions are checked and reported instead: `valid(...)`
    functions and startup checks that refuse to drive (operaGATR `checkConfig`,
    main.cpp:84-118).
  - `static_assert`: 16 uses (3 in tests), all build-time configuration
    checks.
    - Pico:
      - `static_assert(pinsUnique(kAllPins, kPinsAssigned), "pin assigned
        twice or not a usable Pico GPIO");` (config.h:77);
      - frame sizes against the TX FIFO (main.cpp:27-33);
      - an SDK constant equals a local constant: `static_assert(SH2_ACCELEROMETER
        == bno08x::kAccelReportId, "SH-2 acceleration report ID");`
        (imu_bno08x.cpp:84).
    - Robot config: smart ports 1..21 (gatr2_robot.h:173-177).
- **Gaps:**
  - The wire sizes are checked only by tests and comments. For example:
    - `constexpr uint8_t kBrainPayloadMax = kMaxFrameLen -
      kLinkEnvelopeLen; // 122` (frame_codec.h:21);
    - `constexpr uint8_t kTelemetryBodyLen = 54;` next to a hand-written
      field list (frames.h:269-310).
  - Neither has a `static_assert`.
- **Proposed rules:**
  - V-1. No `assert()`. It vanishes under NDEBUG, and when active it stops
    the whole program on the robot, the opposite of degrade-don't-hang.
    Preconditions that callers can violate are checked and reported (E-2).
    Impossible states are handled by a documented safe fallback.
  - V-2. Every relation between compile-time constants that code relies on
    is a `static_assert` with a short message: frame sizes, capacities,
    ports, pin budgets and SDK id matches. Comments like `// 122` become
    assertions.

---

## 17. Logging, printing, diagnostics

- **Practice:**
  - Libraries never print. The one `snprintf` in a library formats an error
    string into a member buffer (actuGATR/src/ports.cpp:13, 20).
  - State is exposed as counters and status structs, not logs:
    - `ClientStats` (client.h:199-228);
    - `FrameReaderStats` (frame_codec.h:144-150);
    - `ProsUsbStats` (pros_usb_port.h:20-29);
    - `ProsLinkStats` (pros_link.h:56-61).
  - `LinkEvents` keeps a small event ring for display (link_events.h:63-90).
  - `toString(enum)` gives display text (13 functions).
  - The Brain apps draw status on the Brain screen through one
    `row(index, format, ...)` helper per app (operaGATR/src/main.cpp:227-231).
  - The Pico writes human-readable lines to USB serial only when
    `cfg::kUsbDebug`, and never blocks: "USB is optional: never wait for a
    monitor or for space in its transmit FIFO" (main.cpp:239-240), checked
    with `Serial.availableForWrite()` (main.cpp:251). Machine-readable
    diagnostics go to the Pi as `PicoDiag` frames (frames.h:509-568).
- **Inconsistencies:**
  - Unit tests print timings to stdout:
    `std::printf("128 boxes over the 512 vertex bound: %.0f us\n", ...)`
    (investiGATR/tests/planner_edge_gtest.cpp:500-501, trimmed).
  - Screen coordinates are magic numbers: `pros::screen::erase_rect(0, y,
    479, y + 19);` (operaGATR/src/main.cpp:229), duplicated in locaGATR
    (main.cpp:414).
- **Proposed rules:**
  - L-1. Libraries never print or log. They expose counters (`*Stats`),
    status structs, event rings and `toString`. Programs decide what to show.
  - L-2. Output on a real-time target is non-blocking: check space first and
    drop rather than wait. This is the Pico rule; it applies to the Brain
    screen and console too.
  - L-3. Tests assert and do not print. Timing checks are asserts with a
    bound, or a separate benchmark target.

---

## 18. Magic numbers and repeated low-level code

These are the Brain and Pico versions of the user's "ten million lines for one
command" gripe.

- **Unit constants redefined per file:**
  - `constexpr double kPi = 3.14159265358979323846;`
    (investigatr/geometry.h:20, communiGATR/src/attitude.cpp:13,
    translaGATR/link_documents.cpp:13);
  - `constexpr double kCdegPerRad = 18000.0 / investigatr::kPi;`
    (actuGATR/src/telemetry.cpp:14, communiGATR/src/link_driver.cpp:23,
    communiGATR/src/robot_profile.cpp:16);
  - `constexpr double kDeg = investigatr::kPi / 180.0;`
    (brain/robot/gatr2_robot.h:23, operaGATR/include/robot_config.h:27);
  - the same quantity is also called `kRad` (attitude.cpp:14);
  - inline `* 180.0 / investigatr::kPi` 7 times in the apps
    (operaGATR/src/main.cpp:248, 253, 267, 268;
    locaGATR/src/main.cpp:426, 446, 447).
- **Saturating and rounding conversions, one set per file:**
  - `i16/i32/u8` (telemetry.cpp:30-40);
  - `centidegrees` (attitude.cpp:17);
  - `toI32/toU32/toU16/microns/millidegrees` (robot_profile.cpp:20-50).
- **Little-endian byte helpers written twice in translaGATR and once in the
  Pico:**
  - frame_codec.cpp:12-31;
  - link_documents.cpp:37-64;
  - pico_commands.cpp:12.
- **The same app wiring in both Brain programs:**
  - `now()`: loca :49, opera :58;
  - `createLink()`: loca :53, opera :120;
  - `attitude()`: loca :75, opera :142;
  - `sendTelemetry()`: loca :83, opera :276;
  - `row(...)`: loca :412, opera :227;
  - `placementLostSinceLastCheck()`: loca :192, opera :297;
  - `sensorsReady(...)`: loca :146, opera :309;
  - `calibrating(...)`: loca :154, opera :317;
  - the field-by-field `LinkSnapshot` fill: loca :132-141, opera :337-346.
    `snapshotOf(const Client&, Seconds)` exists (link_events.h:34), but the
    apps hold a `ProsLink`, not a `Client`.
- **The Brain "make pipeline" is hidden in app main.cpp:**
  - `createLink()` and `createDrive()` (operaGATR/src/main.cpp:120-176) build
    the whole chain: link, planner, follower, motion, kinematics, motors,
    drive, owner, drive task.
  - No file or doc names this chain; each app re-derives it.
- **Plain magic numbers:**
  - `for (int i = 0; i < 50 && running_.load(); ++i)` (pros_drive.cpp:17);
  - `pros::delay(20);` (operaGATR/src/main.cpp:217, 380);
  - `if (pros::millis() - last_display >= 100)` (operaGATR/src/main.cpp:376);
  - `waitFor(runDirect(), 25.0)` (operaGATR/src/main.cpp:418);
  - `value / 127.0` (operaGATR/src/main.cpp:385);
  - `EstimateSeen estimate_seen_[4];` (client.h:492) and `Numbered frames_[4];
    // newest first` (link_driver.h:84);
  - `const char* owners_[22] = {};` and `char error_[80] = {};` (ports.h:31-32).
- **Positive examples to keep:**
  - Pico rates and pins are all named in `cfg::` and `board::` with units
    (config.h:20-42, board.h:15-34).
  - locaGATR names its periods (`kLoopPeriodMs`, `kDisplayPeriodMs`,
    locaGATR/include/robot_config.h:41-42).
- **Proposed rules:**
  - R-1. Every number with a unit, a capacity or a tuning role is a named
    constant, with the unit in its name or type. Allowed literals:
    - 0, 1 and -1;
    - 2 in halving;
    - loop bounds over a named array;
    - wire field widths inside translaGATR encode/decode, where the layout
      comment is the name.
  - R-2. One conversion module per side:
    - Brain: an `investigatr` units header, for example `units.h` with
      `kPi`, `radians(deg)`, `degrees(rad)`, `centidegrees(rad)`,
      `millimeters(m)` and saturating narrowing;
    - translaGATR: its own copy, because the Pico cannot include investigatr.
    - This is extraction: there are 4 or more existing users.
  - R-3. One little-endian reader/writer set in translaGATR, shared by
    frame_codec and link_documents.
  - R-4. A helper repeated in two programs moves into a library as a named
    helper ("driver" in the lead's words). The Brain equivalent of
    naviGATR's `core/build.cpp`:
    - one function per library chain (e.g. `communigatr::makeProsLink(robot)`,
      `actugatr::makeDriveStack(config, link)`);
    - with a PIPELINE.md in brain/ that names the chain
      Pi link -> StateSource -> Motion -> Follower -> Drive -> motors.

---

## 19. Formatting (clang-format 22.1.8, repo .clang-format)

- **Measured:** 133 of 187 files differ from the tool's output (2264
  `--dry-run` diagnostics).

  | component | files | differ |
  |---|---|---|
  | translaGATR | 8 | 8 |
  | brain/communiGATR | 57 | 48 |
  | brain/actuGATR | 46 | 36 |
  | brain/investiGATR | 26 | 12 |
  | brain apps + robot | 10 | 10 |
  | pico/aggreGATR/src | 29 | 13 |
  | pico/aggreGATR/tests | 11 | 6 |

- **What differs:**
  - Access modifiers: the config (LLVM base, IndentWidth 4, default
    AccessModifierOffset -2) produces `  public:`.
    - Brain code writes `public:` at column 0 (94 lines);
    - Pico code and two sims have the tool's `  public:` (26 lines, e.g.
      imu_retry.h:23).
  - Short bodies: the tool joins `bool nonNegative(double v) { return
    std::isfinite(v) && v >= 0; }` onto one line and splits
    `case ResolveStatus::kNoField: return MotionReason::kFieldUnavailable;`
    over two (motion.cpp:18-24, 36-39).
  - Alignment drift after renames: `translagatr::FrameReader        reader_;`
    is misaligned after the gatr2 rename (client.h:421).
  - Include order after renames (robot_profile.h:12).
  - Attached namespace braces in two Pico files.
- **Experiment:** adding `AccessModifierOffset: -4`,
  `AllowShortCaseLabelsOnASingleLine: true` and
  `AllowShortFunctionsOnASingleLine: Inline` to match house style still
  leaves 120/187 files differing (1940 diagnostics). The drift is mostly
  alignment, so a one-time format pass is needed whatever the settings.
- **Current CI** (.github/workflows/ci.yaml) has no format job. That is
  consistent with the drift; CI is not a constraint.
- **Proposed rules:**
  - F-1. clang-format 22.1.8 with the repo config is the only authority.
    - Settle the knobs first (decision D-9: access modifier column and short
      case labels).
    - Then one formatting-only commit over everything.
    - Then CI blocks on `--dry-run --Werror`.
  - F-2. Braces on every `if`, `for` and `while` body (the memory rule;
    followed today).
  - F-3. Vendor and generated files are excluded by path (a
    `.clang-format-ignore` or the CI file list): PROS template, sh2, generated
    field references. Generators are fixed to emit formatted, namespaced code.

---

## 20. Comments vs the user's comment rules

- **Compliant today:**
  - 0 non-ASCII characters in 187 files, so no em dashes, smart quotes or
    arrows. 0 TODO/FIXME/XXX.
  - No Doxygen in project code; the only `/** */` blocks are the PROS
    template.
  - Declaration comments are mostly the terse labels the user wants:
    - `// (-pi, pi]` (geometry.h:33);
    - `uint16_t len = 0; // 0 = no document` (robot_profile.h:90);
    - `uint32_t session = 0; // 0 in HELLO` (frames.h:316);
    - `bool reversed = false; // encoder polarity: positive counts travel
      against angle` (robot_profile.h:46).
  - Comment density is moderate: comment/code line ratio 0.11 (brain-lib),
    0.14 (translaGATR), 0.12 (Pico), 0.06 (tests).
- **In tension with the rules (explanation and rationale delivered to the
  reader):**
  - Long header prose, 17 blocks of 12 or more `//` lines:
    - wheel_calibration.h:1-18 (18 lines, including "That scale lumps radius
      error, gearing error, tire compression and slip together; a trial
      cannot tell them apart");
    - client.h:1-14;
    - pros_drive.h:1-13;
    - locaGATR/src/main.cpp:1-16 (a UI manual in a comment).
  - Narrated rationale in declarations:
    - "tx_lifetime_ms: a queued frame not written by then is dropped, so a
      stalled writer never replays expired requests" (pros_usb_port.h:33-34);
    - gravity_alignment.h:17-27 explains the physics.
  - Units and types carried by comments instead of types: see sections 3
    and 6.
  - The `// <file name>` first line restates the file name (175 files).
  - Wire layout diagrams in translaGATR (frames.h:32-49, frame_codec.h:28-56)
    are long but are real contracts, not narration. They are the one place
    long comments earn their space.
- **Proposed rules:**
  - M-1. The comment rules as written: plain ASCII, terse, noun-phrase
    labels, casual grammar, no narration.
  - M-2. What a comment may carry, in order of preference:
    - (a) a unit, range, sentinel or frame the type cannot carry: `// 0 =
      none`, `// (-pi, pi]`;
    - (b) a caller-visible contract that is not obvious from the signature:
      blocking, bounds, which task, what happens when upstream is dead;
    - (c) a wire layout.
    - Rationale, method and procedure go to docs/ or the component
      PIPELINE.md, with at most a one-line pointer in the header.
  - M-3. A header's top comment is at most about 5 lines: what the module is
    and its I/O. Longer text moves to docs.
  - M-4. When a type can carry the information (an alias, an enum, a unit
    type), the comment goes and the type carries it.

---

## 21. Tests

- **Practice:**
  - Layout:
    - one `tests/` per component;
    - files `*_gtest.cpp` with no `main()`, listed explicitly in CMake
      (brain/actuGATR/CMakeLists.txt:36-47);
    - support code in `sim/` libraries, not in `tests/`: `communigatr_fakes`
      and `actugatr_sim` (brain/communiGATR/CMakeLists.txt:31-38).
  - 800 tests: 754 `TEST`, 38 `TEST_P`, 8 `TEST_F`. 3990 `EXPECT_*`, 834
    `ASSERT_*`, 33 `SCOPED_TRACE`, no gmock.
  - Names: suites are PascalCase (unit or behavior: `Client`, `Recovery`,
    `PlannerExit`); cases are PascalCase sentences with no underscores
    (median 28 characters, max 62), e.g.
    `TEST_F(ReadinessSummary, ConnectingUntilTheFirstStateThenReconnectingAfterALoss)`
    (readiness_gtest.cpp:112).
  - Fakes sit at the seams and the real code runs above them:
    - `LinkRig`: "a FakePi behind a FakeBus (RS-485) or a FakeUsb, a Client and
      LinkDriver on the chosen Brain port, and a clock stepped by hand"
      (sim/link_rig.h:1-5);
    - `SimMotorOutput` for `MotorOutput`, `SimulatedState` for `StateSource`,
      `StraightPlanner` for `PathPlanner` (actuGATR/sim).
    - Parameterized over transports with `TEST_P` (client_telemetry_gtest.cpp:195).
  - Determinism: time advances by hand (`void step(Seconds dt = 0.001);`,
    link_rig.h:31), and random tests use a seeded `std::mt19937`
    (planner_random_gtest.cpp:23-32).
  - Pico tests cover only the hardware-free modules, whose headers say "No
    hardware dependencies" (imu_retry.h:3, pi_link.h:3). The drivers
    (imu_bno08x.cpp, imu_asm330.cpp, quadrature.cpp, main.cpp) have no host
    test.
- **Inconsistencies:**
  - `using namespace <component>;` at file scope in test files: 51
    occurrences (e.g. readiness_gtest.cpp:13).
  - Helpers with external linkage outside the anonymous namespace
    (client_telemetry_gtest.cpp:524-587).
  - Test helper classes with public data (N-11).
  - Positional aggregate init of profile wheels (readiness_gtest.cpp:30-34).
  - Timing printouts (planner_edge_gtest.cpp:496-512).
  - Very large test files: frame_codec_gtest.cpp 3554 lines,
    link_documents_gtest.cpp 1683, client_gtest.cpp 1276.
  - A stale test target name (translaGATR/tests/CMakeLists.txt:2, 37).
- **Proposed rules:**
  - TS-1. Keep: `*_gtest.cpp`, explicit CMake lists, one `tests/` per
    component, fakes in `sim/` at seam boundaries, the real code above the
    seam, time injected, seeded randomness.
  - TS-2. Test names: `TEST(Unit, BehaviorSentenceInPascalCase)`. No
    underscores (gtest reserves them).
  - TS-3. Pure logic lives in hardware-free modules with host tests. A
    driver file holds only the hardware calls, and its logic moves out when
    it grows past trivial (the Pico supervisors show how).
  - TS-4. One test file per unit, split by behavior group past about 1000
    lines (`frame_codec_sensor_gtest.cpp`, `frame_codec_brain_gtest.cpp`).
  - TS-5. Tests may write `using namespace <the component under test>;`
    after the includes. Nothing else is imported wholesale.
  - TS-6. Test helpers go in an anonymous namespace. Fixtures that hold state
    and steps are classes with private data, or structs with free helper
    functions (S-1 applies to tests too).
  - TS-7. No output from tests; timing is asserted or moved to a benchmark.

---

## 22. Size and complexity hot spots (lizard, non-test)

- Of 937 non-test functions, 18 are over 60 NLOC and 35 are over CCN 15.

  | function | NLOC | CCN | where |
  |---|---|---|---|
  | `communigatr::Client::choose` | 143 | 56 | client.cpp:869 |
  | `translagatr::validateRobotProfile` | 100 | 52 | link_documents.cpp:262 |
  | `run` (locaGATR main loop) | 125 | 45 | locaGATR/src/main.cpp:526 |
  | `actugatr::Motion::update` | 117 | 38 | motion.cpp:363 |
  | `investigatr::shortestRoute` | 102 | 32 | planner.cpp:319 |
  | `investigatr::GeometricPlanner::plan` | 107 | 32 | planner.cpp:773 |
  | `opcontrol` (operaGATR) | 95 | 32 | operaGATR/src/main.cpp:423 |
  | `actugatr::SegmentFollower::update` | 113 | 24 | follower.cpp:141 |
  | `communigatr::Client::handleReply` | 80 | 24 | client.cpp:351 |

- `opcontrol` repeats the same three-branch block for buttons A, X and Y
  (operaGATR/src/main.cpp:438-458).
- **Proposed rule:**
  - Z-1. Functions read at one level of abstraction (the lead's rule).
  - Soft limits, enforced in review and reported by lizard in CI:
    - about 60 NLOC and CCN 15 per function;
    - about 600 lines per source file;
    - about 25 data members per class.
  - Past a limit, split by named steps, not by line count.

---

## 23. Platform constraints that must shape rules

### V5 Brain / PROS

- **Kernel and toolchain:**
  - PROS kernel 4.2.2: "Run in brain/locaGATR: pros c apply kernel@4.2.2
    --force-apply" (locaGATR/Makefile:55-58).
  - arm-none-eabi gcc 13 for Cortex-A9, hard float with NEON:
    `MFLAGS=-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -Os -g -mthumb`
    (locaGATR/common.mk:4). Double math is cheap on the Brain.
- **Language standard differs between the Brain and the host:**
  - "The kernel template defaults (gnu23, gnu++26) are rejected by arm gcc
    13. Kernel headers need C++20." (locaGATR/Makefile:19-22, trimmed); PROS
    builds use `CXX_STANDARD:=gnu++20`.
  - Host builds are C++17 (brain/CMakeLists.txt:4).
  - The same library sources compile as both, through gatr2_brain.mk. So
    library code must be valid C++17 and warning-free as gnu++20: no C++20
    features, no constructs deprecated in C++20.
- **Warnings:**
  - `GATR2_WARNFLAGS ?= -Wall -Wextra` for the libraries (gatr2_brain.mk:82);
  - apps `WARNFLAGS+=-Wall -Wextra` (Makefile:15);
  - `-Werror` nowhere in scope: host CMake (investiGATR/CMakeLists.txt:41-45),
    Pico (platformio.ini:17), tests.
- **Exceptions and RTTI:** no `-fno-exceptions` or `-fno-rtti` in the PROS
  flags (common.mk:39-42). They are available on the Brain, but shared code
  cannot use them (Pico, below).
- **Tasks:**
  - PROS deletes competition tasks at mode changes without releasing held
    mutexes (pros_drive.h:7-8; docs/communigatr.md:203-207). Hence bounded
    takes, and link loops in the program's own tasks.
  - Priorities are `TASK_PRIORITY_DEFAULT + 1`, stacks
    `TASK_STACK_DEPTH_DEFAULT` (pros_link.cpp:122-123).
  - No atomic reference counts, so no `std::shared_ptr` across tasks
    (docs/communigatr.md:208-209).
- **Blocking calls to keep off control tasks:**
  - smart port open, about 100 ms (pros_serial_port.h:18-19);
  - VEX IMU recalibrate, about 1 s (pros_vex_imu.h:32-34);
  - a USB write blocks while the cable is out (pros_usb_port.h:5-7).
- **Kernel quirks encoded in comments:**
  - "Kernel 4.2.2 ser_read_r appends a NUL after the returned data"
    (pros_usb_port.cpp:129-130);
  - "SERCTL_NOBLKWRITE does nothing in 4.2.2" (pros_usb_port.cpp:196);
  - PROS output COBS must be off for the NG1 line protocol
    (pros_usb_port.h:3-5).
- **Fixed names:**
  - `initialize`, `disabled`, `competition_initialize`, `autonomous` and
    `opcontrol` are global and snake_case, declared `extern "C"` by the
    template (operaGATR/include/main.h:61-71).
  - The PROS template macros `PROS_USE_SIMPLE_NAMES` and `PROS_USE_LITERALS`
    are on (main.h:27, 35).
- **Hardware limits:**
  - Smart Ports 1..21, checked at compile time (ports.h:35-48,
    gatr2_robot.h:173-177) and at runtime (`PortMap`).
- **Clock:** `pros::millis()` is a `uint32_t` (wraps after about 49.7 days),
  and `pros::micros()` is 64-bit (pros_link.cpp:392). Wrap-safe compares are
  used on millis (operaGATR/src/main.cpp:368).
- **Memory budget:** no measured V5 memory budget is recorded in the repo.
  The standard should not invent one: rule H-2 bounds allocation by
  behavior, not by bytes.

### RP2040 Pico / Arduino-pico

- **Core:** Cortex-M0+ with no FPU (`-mcpu=cortex-m0plus`). Floating point is
  software-emulated.
  - The Pico uses `double` in `Vector3` (gravity_alignment.h:9-13) and
    `float` for rates (gyro_window.h:27); the Pico-side math is small and
    per report.
  - Rule: no floating point in ISRs; keep float math out of per-tick paths
    unless measured.
- **Toolchain defaults:** arduino-pico `-std=gnu++17`, `-fno-exceptions`
  unless PIO_FRAMEWORK_ARDUINO_ENABLE_EXCEPTIONS, `-fno-rtti` unless
  PIO_FRAMEWORK_ARDUINO_ENABLE_RTTI, and `-Os`
  (~/.platformio/packages/framework-arduinopico/tools/platformio-build.py:56-78,
  147-161, installed version 1.50401.0).
  - The platform itself is pinned by commit in platformio.ini:10.
  - The installed framework may differ from what that commit resolves to;
    re-check after a clean install.
- **Fixed names:** Arduino `setup()` and `loop()` are global
  (main.cpp:299, 319). Only core 0 is used.
- **UART:**
  - The TX FIFO is 32 bytes and every frame must fit an empty FIFO
    (main.cpp:21-33 static_asserts); a busy FIFO skips the tick.
  - "Serial1.availableForWrite() only reports 0 or 1 on this core"
    (main.cpp:91), so the code reads the hardware flag directly
    (`uart_get_hw(uart0)->fr`, main.cpp:92).
- **Interrupts:** `attachInterrupt(..., CHANGE)` on the encoder pins
  (quadrature.cpp:62-63), `volatile` counters, and `noInterrupts()` sections.
- **Clock:** the 32-bit microsecond clock wraps after about 71.6 minutes,
  so every schedule uses signed-difference compares (main.cpp:324-326,
  gyro_window.h:11-12). 64-bit time is used for stamps and ages.
- **Build variants:**
  - macros from platformio.ini (`GATR2_HAT_REV`, `GATR2_IMU_*`);
  - whole driver files wrapped in `#if defined(GATR2_IMU_BNO08X)`
    (imu_bno08x.cpp:12);
  - one interface header (`imu.h`) with one implementation per build.
- **Vendored C:** the CEVA sh2 driver is included `extern "C"`
  (imu_bno08x.cpp:27-31) and ignored for the ASM330 build (`lib_ignore = sh2`,
  platformio.ini:25). Its known bug is guarded outside it: "sh2 v1.4.0
  indexes its channel table before bounds checking" (shtp_header.h:2-3).
- **Heap:** Pico src allocates nothing; keep it that way (H-1).
- **Host builds:** Pico logic modules are also compiled on the host for
  gtest (pico/aggreGATR/tests/CMakeLists.txt:54-62). So those headers must not
  include Arduino or SDK headers.
  - Today only 4 .cpp files include Arduino or hardware headers: main.cpp,
    imu_bno08x.cpp, imu_asm330.cpp and quadrature.cpp.

### translaGATR (all three targets)

- "No hardware dependencies. Must compile on all three targets."
  (frames.h:2-3).
- The Pico builds only `frame_codec.cpp`
  (`build_src_filter = +<*> +<../../../translaGATR/frame_codec.cpp>`,
  platformio.ini:18); "Brain and Pi only; the Pico does not build this"
  (link_documents.h:5).
- Hence C++17, no heap, no exceptions, no RTTI, no investigatr includes. The
  duplicate `kPi` (link_documents.cpp:13) is the justified exception to R-2.
- It is compiled by three toolchains: arm gcc for the Brain, arm gcc for the
  Pico, and host gcc/clang. Fixed-width types and explicit little-endian
  byte access (frame_codec.cpp:12-31) are mandatory, never `memcpy` of
  structs.

---

## 24. Decisions the standard must put to the user

- D-1. Warnings:
  - Today it is `-Wall -Wextra` everywhere with no `-Werror`, matching this
    session's instruction; the memory says -Werror.
  - Also decide on `-Wswitch-enum`, `-Wshadow` and `-Wconversion` for the
    portable Brain libraries.
- D-2. Units: are aliases enough, or strong unit types for Seconds, Meters and
  Radians? Aliases fit "readability first". This survey did not hunt for
  unit mix-ups systematically and saw none.
- D-3. Frames: frame-named aliases (`FieldPose`) or frame prefixes in
  variable names.
- D-4. The translaGATR include spelling and folder layout (`"translagatr/x.h"`
  under an include dir, or `"translaGATR/x.h"` everywhere, Pico included).
- D-5. Duration units in program configs (`Seconds` vs `_ms`) and the suffix
  spelling (`Deg`/`Degrees`, `S`/`Seconds`).
- D-6. The boolean query prefix: `is` or bare adjective.
- D-7. Exceptions on the Brain and host: forbid everywhere for one style
  (recommended), or allow Brain-only.
- D-8. Keep or drop the `// <file name>` first line.
- D-9. clang-format knobs: access modifiers at column 0
  (`AccessModifierOffset: -4`, today's Brain style) or the LLVM `  public:`
  (today's Pico style); short case labels on one line or not.
- D-10. Pico namespace restructure (`aggregatr::` root with sub-namespaces)
  and folder split; the same open question the lead raised for naviGATR
  (namespaces mirror folders).
