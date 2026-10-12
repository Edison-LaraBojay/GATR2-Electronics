# Industry survey: candidate rules for the GATR2 coding standard

Survey input for the GATR2 coding standard, 2026-09-28. It is not the standard. It collects
industry rules that fit the user's philosophy (requirements.md), backs each one with a primary
source, and shows where it touches real GATR2 code and what enforcing it would cost.
The noise numbers come from runs of the actual tools on the actual tree (section 1).
Nothing in the repository was changed.

Legend for "Applies":
B = Brain (brain/, PROS target and host builds), Pi = naviGATR C++, Pc = Pico firmware
(pico/aggreGATR), C = translaGATR codec, V = spectaGATR viewer (ES modules),
T = tools (pi/naviGATR/tools, bench/, Python, shell, perf C++).

Legend for "Enforcement":
GATE = mechanical and cheap enough to block a PR once the backlog is fixed.
ADVISORY = reported, not blocking (ratchet: no new violations).
REVIEW = human check, listed in the review checklist.
DECISION = the lead has to pick (see section 7).

Citations are `path:line` plus a short quoted snippet (leading spaces trimmed). The noise counts
are unique source locations in GATR2-owned files. Vendored code, googletest and system headers are
left out unless a column says otherwise.

---

## 1. What was measured

### 1.1 Tools and targets

| Tool | Version | What it covered |
|---|---|---|
| GCC host | 15.2.0 (MSYS2 ucrt64) | 210 TUs from four CMake compile databases configured in scratch: naviGATR src+tests, brain libs+sim+tests, translaGATR+tests, pico host-testable src+tests. `-O2 -c` plus 33 extra warnings. |
| GCC Pico | arm-none-eabi 14.3.0 (toolchain-rp2040-earlephilhower 5.140200) | All 14 pico/aggreGATR/src/*.cpp + frame_codec.cpp against the installed arduino-pico core, `-mcpu=cortex-m0plus -Os -std=gnu++17 -fno-exceptions -fno-rtti`. |
| GCC Brain | arm-none-eabi 13.3.1 (PROS toolchain) | locaGATR and operaGATR main.cpp, every brain/*/pros/*.cpp and brain/*/src/*.cpp, with the PROS flags from brain/locaGATR/common.mk (`-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -Os -mthumb -std=gnu++20`). |
| clang-tidy | 22.1.8 (pip wheel in scratch venv) | Same 210 host TUs. First run: every check in bugprone, cppcoreguidelines, modernize, performance, readability, concurrency and misc. Second run: the recommended config in section 4. |
| lizard | 1.24.0 | 438 tracked C/C++ files outside third_party. |
| clang-format | 22.1.8 | Dry run of the repo `.clang-format` over 437 C++ files, plus variants. |
| ESLint | 10.11.0, @eslint/js recommended (64 rules) | 17 spectaGATR modules + 2 tool scripts (copied to scratch). |
| ruff | 0.16.9 | 7 tracked Python files (copied to scratch). |
| Sanitizers | GCC 15.2 ucrt64 | Link test of ASan, UBSan, TSan and UBSan trap mode. |

### 1.2 Headline results

- The code is already clean under `-Wall -Wextra` on all 210 host TUs and all 55 ARM TUs.
- Most of the stricter GCC warnings produce zero or near-zero hits in GATR2 code. This holds for
  -Wold-style-cast, -Wnon-virtual-dtor, -Wsuggest-override, -Wzero-as-null-pointer-constant,
  -Wextra-semi, -Wpedantic, -Wcast-qual, -Wcast-align=strict, -Wformat=2, -Wduplicated-cond,
  -Wduplicated-branches, -Wlogical-op, -Wimplicit-fallthrough=5, -Wvla, -Walloca,
  -Wmismatched-tags and -Wredundant-tags. The noise comes from vendor headers: PROS headers give
  415 warnings, arduino-pico gives 230 (section 5).
- The clang-tidy recommended config (section 4) reports 569 findings in non-test code. About
  three quarters (439) come from four checks with mechanical or batch fixes: naming (225),
  braces (83), cognitive complexity (71) and one-declaration-per-line (60).
- Complexity is concentrated in naviGATR:
  - lizard: 84 naviGATR src functions are over 60 lines and 86 have CCN over 15 (not counting
    tools/perf, which adds 15 and 17). Brain non-test has 13 and 27, Pico 0 and 1, codec 6 and 7.
  - The longest function is `navigatr::writeSnapshotBody` at 398 lines
    (pi/naviGATR/src/inspection/inspection_document.cpp:1140).
  - The most complex is `navigatr::TrackingWheelMotion::run`: 352 lines, CCN 105, cognitive
    complexity 168 (pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:510).
- The repo is not clang-format clean under its own pinned version.
  - 369 of 437 C++ files differ under `clang-format 22.1.8 --dry-run` with the checked-in
    `.clang-format`.
  - Three config fixes (AccessModifierOffset, AllowShortCaseLabelsOnASingleLine, case-insensitive
    include sort) still leave 356 files. Most of the rest is alignment drift.
  - `InsertBraces: true` would add braces at 124 sites in 17 files.
- Sanitizers do not link on this Windows box. `-fsanitize=address`, `undefined` and `thread` all
  fail to link (`cannot find -lasan`, `-lubsan`, `-ltsan`). Only
  `-fsanitize=undefined -fsanitize-trap=undefined` works, and it trapped a signed overflow in a
  test program. Sanitizer runs belong in a Linux CI job.
- JS and Python tools are nearly clean:
  - ESLint recommended: 7 findings in the viewer and tools JS.
  - ruff 0.16.9 default set: 37 findings. The classic E4/E7/E9/F set: 4.

---

## 2. The user's rules mapped to industry sources

| User rule (requirements.md) | Closest industry rules | Agreement | Mechanical enforcement |
|---|---|---|---|
| "using aliasing is great" | C++CG T.43 "Prefer `using` over `typedef` for defining aliases"; AUTOSAR A7-1-6 "The typedef specifier shall not be used."; Google "Aliases" | Full agreement. Google adds a caveat: "Public aliases are for the benefit of an API's user, and should be clearly documented." | `modernize-use-using` (IgnoreExternC). Already 0 hits outside the C boundary. |
| "use struct for attribute only things / class for attribute + methods" | C++CG C.2 "Use `class` if the class has an invariant; use `struct` if the data members can vary independently"; C.8; Google "Use a struct only for passive objects that carry data; everything else is a class."; LLVM "struct should be used when all members are declared public."; AUTOSAR A11-0-2 "A type defined as struct shall: (1) provide only public data members, (2) not provide any special member functions or methods, (3) not be a base of another struct or class, (4) not inherit from another struct or class." | AUTOSAR A11-0-2 matches the user's wording exactly. C.2 and Google are looser: helper methods are allowed on a struct with no invariant. DECISION (section 7). | Half: `misc-non-private-member-variables-in-classes` catches classes with public data. No check forbids methods on a struct, so a script or review is needed. |
| "avoid using classes for the sake of ... can I use a functional approach?" | C++CG C.4 "Make a function a member only if it needs direct access to the representation of a class"; F.8 "Prefer pure functions"; Google "Do not use a class simply to group static members." | Full agreement | `readability-convert-member-functions-to-static` (14 hits, advisory); review |
| "use namespaces appropriately in general everything is namespaced" | MISRA C++:2023 Rule 6.0.3 "The only declarations in the global namespace should be `main`, namespace declarations and `extern "C"` declarations" (title as listed by MathWorks); AUTOSAR M7-3-1 (same text); Google "With few exceptions, place code in a namespace."; C++CG SF.20 "Use `namespace`s to express logical structure" | Full agreement | Script. clang-tidy has no general check for this. |
| "a system should also be almost as clear as possible simply from its folder structure" | Robert C. Martin, Screaming Architecture: "When you look at the top level directory structure, and the source files in the highest level package; do they scream: Health Care System..."; LLVM "Library Layering" | Agreement | Review plus a folder/PIPELINE.md check (DOC-01) |
| "each new pipeline ought to have an explanation that is as concise as possible, as an md" | matklad, ARCHITECTURE.md: "Keep it short: every recurring contributor will have to read it.", "Do name important files, modules, and types. Do not directly link them (links go stale).", "Explicitly call-out architectural invariants" | Agreement. The idea is one doc per pipeline level. | Script checks that the file exists and that the symbols it names exist (DOC-01) |
| "standard INPUT/OUTPUT that way different teams can work inside of them" | Microsoft, Pipes and Filters: "Filters act independently and are unaware of other filters. They're only aware of their input and output schemas."; Cockburn, Hexagonal Architecture intent: "Allow an application to equally be driven by users, programs, automated test or batch scripts, and to be developed and tested in isolation from its eventual run-time devices and databases." | Agreement | Include-dependency rule between stage folders (ARC-03) |
| "do NOT over generalize beyond what is absolutely required" | Google eng-practices: "A particular type of complexity is over-engineering, where developers have made the code more generic than it needs to be, or added functionality that isn't presently needed by the system."; C++CG C.10 "Prefer concrete types over class hierarchies" | Agreement | REVIEW |
| "Readability is king ... slightly slower but more readable, readable wins" | C++CG Per.1 "Don't optimize without reason", Per.2, Per.6 "Don't make claims about performance without measurements"; Google eng-practices: "'Too complex' usually means 'can't be understood quickly by code readers.'" | Agreement | Complexity gates (FN-01..03) |
| The gripe: main pipeline and make_* construction are hard to find | Seemann, Composition Root: "A Composition Root is a (preferably) unique location in an application where modules are composed together." and "As close as possible to the application's entry point." | Direct remedy (ARC-02) | PIPELINE.md names the entry functions; review |

---

## 3. Candidate rules

The IDs are provisional. Each table is one theme. The "Why here" column cites real code.

### 3.1 Types: struct, class, aliases, enums (TYP)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| TYP-01 | `struct` for plain data whose members vary independently. `class` when there is an invariant or any non-public member. Never mix public data members with private state in a `class`. | C++CG C.2, C.8, C.9; Google "Structs vs. Classes"; LLVM "Use of class and struct Keywords"; AUTOSAR A11-0-1 | About 495 `struct` and 186 `class` definitions. Stage inputs are already data-only structs: `struct PublishingInput {` (pi/naviGATR/src/contracts/publishing.h:27). `class TypedPayload` guards an invariant correctly (pi/naviGATR/src/core/typed_payload.h:20). The check reports 65 non-private data members in classes that also have private parts (11 non-test, all Brain; 54 in test fixtures), e.g. protected data `FollowerConfig config_;` (brain/actuGATR/include/actugatr/follower.h:103). | GATE: `misc-non-private-member-variables-in-classes` with IgnoreClassesWithAllMemberVariablesBeingPublic=true. Tests relaxed. | B Pi Pc C |
| TYP-02 | Strict form of the user's rule: a struct has no member functions. Comparison operators and named constructors become free functions in the same namespace. | AUTOSAR A11-0-2; C++CG C.5 "Place helper functions in the same namespace as the class they support", C.168 | A name-matching heuristic (lizard function names joined with struct names) finds about 72 structs with member function bodies, 34 of them in tests/sim. Examples: `struct PayloadDescriptor` with `static PayloadDescriptor of(...)` and `bool matches(const std::type_index& other) const` (pi/naviGATR/src/core/payload_descriptor.h:13, :25); `struct TypedId {` with operators (pi/naviGATR/src/core/ids.h:19). | DECISION between strict (AUTOSAR) and lenient (Google, C.2). No clang-tidy check exists. Enforce with a small AST or grep script, or REVIEW. | B Pi Pc C |
| TYP-03 | No class or struct that only groups static functions or constants. Use a namespace. | Google "Nonmember, Static Member, and Global Functions": "Do not use a class simply to group static members." | `class HostClock` has only static members `now()`, `nowUs()`, `origin()` (pi/naviGATR/src/core/host_clock.h:16). The generated `struct Field : investigatr::FieldReferences {` holds only `static constexpr` references, in the global namespace (brain/operaGATR/include/field_references.h:4). | REVIEW; generator fix | B Pi Pc C |
| TYP-04 | Prefer free functions and pure transforms. A function is a member only if it needs the representation. | C++CG C.4, F.8; LLVM "Turn Predicate Loops into Predicate Functions" | `readability-convert-member-functions-to-static` finds 14 methods that never use `this` (10 in naviGATR src). The math headers (pi/naviGATR/src/math/*.h) already use free functions. | ADVISORY: that check | B Pi Pc C |
| TYP-05 | `using` for every alias. `typedef` only inside `extern "C"` headers. | C++CG T.43; AUTOSAR A7-1-6; Google "Aliases" | The only typedefs are the C boundary `typedef struct nav_apriltag_detector nav_apriltag_detector;` (pi/naviGATR/src/impl/resources/apriltag_c.h:23), inside `extern "C"`, and they must stay C. | GATE: `modernize-use-using` with IgnoreExternC=true (0 hits with that option) | B Pi Pc C |
| TYP-06 | Name domain quantities with aliases or small types. A public alias carries a one-line comment on intent. | C++CG I.4 "Make interfaces precisely and strongly typed", T.42; Google "Aliases" | Good existing use: `using investigatr::Seconds;` (brain/actuGATR/include/actugatr/pid.h:12). Elsewhere units live only in names: `double dx_m = 0.0;` (pi/naviGATR/src/payloads/robot_observations.h:34). `bugprone-easily-swappable-parameters` finds 47 adjacent same-type parameters (9 outside tests). | REVIEW; ADVISORY: swappable-parameters | B Pi Pc C |
| TYP-07 | Stage and module inputs and outputs are named structs, not long parameter lists, pairs or tuples. | Google "Structs vs. Pairs and Tuples"; C++CG F.21, I.23 | contracts/*.h already does this (`struct PublishingInput`, `struct PublishingOutput`, pi/naviGATR/src/contracts/publishing.h:27, :40). 8 naviGATR functions and 1 Brain function take 7 parameters, e.g. `investigatr::resolve` (brain/investiGATR/src/reference.cpp:23). | GATE: parameter count <= 6 (lizard `-a 6`) | B Pi Pc C |
| TYP-08 | Rule of zero. If any of copy, move or destructor is declared, declare or delete all of them. | C++CG C.20, C.21; CERT OOP54-CPP | 14 findings, e.g. `class AprilTagDetector : public TagDetector` declares a destructor only (pi/naviGATR/src/impl/resources/tag_detectors.h:58). | GATE: `cppcoreguidelines-special-member-functions` (AllowSoleDefaultDtor, AllowMissingMoveFunctionsWhenCopyIsDeleted) | B Pi Pc C |
| TYP-09 | A polymorphic base has a public virtual or protected non-virtual destructor. Every override says `override`. | C++CG C.35, C.127, C.128; CERT OOP52-CPP | 0 hits in GATR2 code for -Wnon-virtual-dtor and -Wsuggest-override on host. PROS headers alone produce 361 on the Brain target (section 5.3). | GATE: `modernize-use-override`, `cppcoreguidelines-virtual-class-destructor`; host flags | B Pi Pc |
| TYP-10 | Reference or const data members only in non-copyable view structs such as stage inputs. | C++CG C.12 "Don't make data members `const` or references in a copyable or movable type" | Stage inputs are views on purpose: `const SensorMap& sensors;` (pi/naviGATR/src/contracts/publishing.h:28). `cppcoreguidelines-avoid-const-or-ref-data-members` flags 53 such members, 37 in naviGATR src. | REVIEW; do not enable that check | Pi |
| TYP-11 | `enum class` by default. An unscoped enum with a fixed underlying type is allowed only for wire constants and bit masks inside a namespace. | C++CG Enum.3; AUTOSAR A7-2-3; MISRA C++:2023 10.2.1, 10.2.2 (MathWorks titles) | The codec uses wire enums as integers: `enum FrameType : uint8_t {` (translaGATR/frames.h:22). `cppcoreguidelines-use-enum-class` finds 37, 33 in translaGATR. The file-local `enum ChoiceKind { kChooseControl, kChooseReliable, kChooseSlot };` (pi/naviGATR/src/inspection/http_server.cpp:723) has no such reason. | ADVISORY: `cppcoreguidelines-use-enum-class` outside translaGATR | B Pi Pc C |

### 3.2 Functions, size and complexity (FN)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| FN-01 | A function fits on one screen. Soft limit 60 lines. Anything longer gets split or justified. | JPL Power of Ten rule 4 ("no more than about 60 lines of code per function"); JPL C rule 25; Google "Write Short Functions" (about 40 lines); C++CG F.3 | Functions over 60 lines (lizard): naviGATR src 84 (plus 15 in tools/perf), Brain non-test 13, codec 6, Pico 0. Longest six: `writeSnapshotBody` 398 lines (pi/naviGATR/src/inspection/inspection_document.cpp:1140); `TrackingWheelMotion::run` 352 (tracking_wheel_motion.cpp:510); `TrackingWheelMotion::create` 286; `TagMountAssociation::run` 284; `helloDocument` 284 (inspection_document.cpp:748); `make_synthetic_rig` 252 (pi/naviGATR/src/impl/resources/synthetic_rig.cpp:398). | ADVISORY with ratchet: lizard `-L 60`; GATE on new functions over 120 | all C++; V (ESLint `max-lines-per-function`, 7 hits at 80); T |
| FN-02 | Cognitive complexity at most 25 per function. | C++CG F.2, F.3, F.56; Google eng-practices "Complexity" | Functions at or above each threshold (clang-tidy, non-test): 71 at 25, 34 at 40, 17 at 60, 5 at 100. Top: `TrackingWheelMotion::run` 168, `ConfiguredTargetResolution::run` 112, `TrackingWheelMotion::create` 104, `parseFieldMap` 102, `PicoTelemetry::refresh` 100. Brain top: `Client::choose` 62 (brain/communiGATR/src/client.cpp:869). | ADVISORY with ratchet: `readability-function-cognitive-complexity` Threshold 25 | all C++; V (ESLint `complexity` 15: 34 hits, max `renderStatus` 77 at pi/naviGATR/spectaGATR/app.js:852) |
| FN-03 | At most 6 parameters. | JPL C rule 25 ("no more than 6 parameters"); C++CG I.23 | 9 source functions have 7 (see TYP-07). | GATE: lizard `-a 6` | all |
| FN-04 | Return values, not output parameters, in new interfaces. Keep one systematic error style per layer (ERR-01). | C++CG F.20, F.21, E.27 "If you can't throw exceptions, use error codes systematically" | naviGATR passes `std::string& err` as an out-parameter 217 times in src, e.g. `bool System::build(const char* xml, const FunctionRegistry& functions,` ... `std::string& err)` (pi/naviGATR/src/runtime/system.cpp:148). It is consistent, which E.27 accepts. | DECISION: keep the err-out style or move to a result struct | Pi |
| FN-05 | Unused parameters are unnamed. | C++CG F.9 "Unused parameters should be unnamed" | Already done: `uint8_t System::controlProfile(uint8_t action, uint8_t, MonotonicTime now, uint8_t& detail) {` (pi/naviGATR/src/runtime/system.cpp:475). `readability-named-parameter` contradicts F.9 (58 hits). | REVIEW; keep `readability-named-parameter` off | all C++ |
| FN-06 | Early exit. No `else` after `return`, `break` or `continue`. | LLVM "Use Early Exits and continue to Simplify Code", "Don't use else after a return"; C++CG F.56 | `readability-else-after-return` has 0 hits today, so the gate costs nothing. | GATE | all C++; V |
| FN-07 | A function that reports failure is `[[nodiscard]]`. Callers check the result or cast it to void. | Power of Ten rule 7; JPL C rule 14; MISRA C++:2023 0.1.2 "The value returned by a function shall be used" | `modernize-use-nodiscard` proposes 439 candidates (195 naviGATR src), too many as a blanket rule. The status-returning builders (`bool build(...)`, `std::optional<...> make_*`) are the ones that matter. | REVIEW for new code; `bugprone-unused-return-value` GATE | all C++ |

### 3.3 Namespaces, linkage, headers (NS)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| NS-01 | Everything is in a named namespace. The global namespace holds only `main`, namespaces and `extern "C"`. | MISRA C++:2023 6.0.3; AUTOSAR M7-3-1; Google "Namespaces" | Only 7 non-test files lack a namespace. Two need attention: the generated global `struct Field` (brain/operaGATR/include/field_references.h:4) and pi/naviGATR/src/app/field_refs.cpp. The others are vendor-template PROS main.h/api.h and the C boundary apriltag_c.h. | GATE: script (top-level declaration outside namespace) | B Pi Pc C |
| NS-02 | The namespace mirrors the component folder, lower case. Nested namespaces follow sub-pipeline folders only when that folder is a separately owned unit. | C++CG SF.20; Google: namespaces "should have unique names based on the project name, and possibly its path"; LLVM "Library Layering" | Namespace openings by name: navigatr 197, communigatr 39, actugatr 35, investigatr 16, translagatr 5. naviGATR's 201 src files across 14 top-level folders (plus 8 impl/ sub-folders) all sit in one flat `navigatr`, so the folder layout is invisible in names. Pico has no component namespace (`aggregatr`), only per-driver ones (`encoder`, `imu`, `pilink`, `bno08x`). | DECISION (churn vs clarity); script to check that path and namespace agree | B Pi Pc C |
| NS-03 | File-local helpers go in an anonymous namespace in the .cpp. No anonymous namespaces in headers. | C++CG SF.21; CERT DCL59-CPP; Google "Internal Linkage"; LLVM "Restrict Visibility" | 6 findings, e.g. the file-local `enum ChoiceKind` at global scope in a .cpp (pi/naviGATR/src/inspection/http_server.cpp:723). -Wmissing-declarations: 3, all tests. | GATE: `misc-use-internal-linkage`, `misc-anonymous-namespace-in-header`, -Wmissing-declarations | all C++ |
| NS-04 | No `using namespace` in headers. Allowed in .cpp files and tests. | C++CG SF.6, SF.7; Google; LLVM "Do Not Use using namespace std" | Only tests use it (`using namespace actugatr;` brain/actuGATR/tests/chassis_gtest.cpp:9). The PROS template's `// using namespace pros;` (brain/locaGATR/include/main.h:52) is commented out. | GATE: `google-build-using-namespace` scoped to headers, or grep | all C++ |
| NS-05 | Headers are self-contained, use `#pragma once`, and include what they use. | C++CG SF.8, SF.10, SF.11; Google "Self-contained Headers" | `misc-include-cleaner` reports 4373 findings on the MinGW host run. That is mostly libstdc++ mapping noise and not usable as a gate. | GATE: compile every header alone (a generated header-check target); `misc-include-cleaner` later on Linux, ADVISORY | all C++ |
| NS-06 | No non-inline definitions in headers. One definition per entity. | C++CG SF.2; CERT DCL60-CPP | 0 hits today | GATE: `misc-definitions-in-headers` | all C++ |

### 3.4 Naming (NM)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| NM-01 | One written, machine-checked naming convention. Proposal from the dominant existing style: types `CamelCase`; functions and methods `camelBack`; variables, parameters and struct fields `snake_case`; class members `snake_case_`; constants and enumerators `kCamelCase`; macros `UPPER_CASE`; namespaces `lower_case`. | C++CG NL.8 "Use a consistent naming style", Enum.5; Google "Naming" (same except functions in PascalCase and enumerators as kName) | `readability-identifier-naming` with this proposal: 349 findings (225 non-test). Examples: <br>- A struct mixing styles: `double dx_m` next to `MonotonicTime startAt;` (pi/naviGATR/src/payloads/robot_observations.h:34, :39); `MonotonicTime measuredAt;` (pi/naviGATR/src/core/records.h:71). <br>- Snake-case builders `make_target_set`, `make_resources`, `register_resources` next to camelBack `registerAll` (pi/naviGATR/src/runtime/register_all.cpp:9-10). <br>- Pico enumerators without k: `Off,` (pico/aggreGATR/src/asm330_supervisor.h:21). <br>- `double kP` (brain/actuGATR/include/actugatr/pid.h:15). | GATE after the convention is decided and the backlog is renamed | all C++; V (camelCase already); T (ruff N) |
| NM-02 | One verb family per job: `make*` builds a stage or resource, `register*` fills the registry, `create` is not used. Or the reverse, but only one. | C++CG NL.8; Google eng-practices "Naming" | naviGATR has three build conventions side by side: 16 free `make_*` factories, 18 static `X::create(...)` factories in impl/, and `System::build*`. | DECISION; REVIEW | Pi |
| NM-03 | A physical quantity carries its unit in its type or in a name suffix (`_m`, `_rad`, `_s`, `_us`). | C++CG I.4 (strong types). C++CG NL.5 bans type encoding, but a unit is not a type code. | Already common: `rate_rad_s`, `t_us`, `dx_m`. It prevents unit mixups across the Pico to Pi to Brain seam. | REVIEW | all |
| NM-04 | Name length matches scope. No single letters outside tiny scopes. | C++CG NL.7, ES.7 | `readability-identifier-length` reports 5440 findings. That is noise, so it stays a review rule. | REVIEW | all |

### 3.5 Expressions, conversions, control flow (EX)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| EX-01 | Initialize every object where it is declared. | C++CG ES.20; MISRA C++:2023 11.6.1; CERT EXP53-CPP | `cppcoreguidelines-init-variables`: 7. `cppcoreguidelines-pro-type-member-init`: 9. | GATE: both checks plus -Wall (-Wmaybe-uninitialized at -O2) | all C++ |
| EX-02 | No C-style or functional casts. Use named casts. | C++CG ES.48, ES.49; AUTOSAR A5-2-2; MISRA C++:2023 8.2.2 | -Wold-style-cast: 0 in GATR2 host code. `modernize-avoid-c-style-cast`: 30, 20 in translaGATR tests. The arduino-pico headers produce 47 (section 5.2). | GATE: -Wold-style-cast on host; `cppcoreguidelines-pro-type-cstyle-cast` | all C++ |
| EX-03 | No lossy implicit conversions on wire, fixed-point and counter paths. | C++CG ES.46; AUTOSAR A4-7-1; MISRA C++:2023 7.0.x | translaGATR src and Pico host-built src have 0 -Wconversion and 0 -Wsign-conversion hits. Elsewhere: -Wconversion 47 and -Wsign-conversion 87 on host, mostly benign `int64_t` ms to `double` (e.g. pi/naviGATR/src/core/time.h:66 `return (later - earlier) / 1000.0;`). Brain target: `int` to `int16_t` at brain/locaGATR/src/main.cpp:414. | GATE for translaGATR and Pico (keep at 0); ADVISORY elsewhere | C Pc gate; B Pi advisory |
| EX-04 | Deliberate float vs double on the MCU. | GCC -Wdouble-promotion doc: "CPUs with a 32-bit 'single-precision' floating-point unit implement float in hardware, but emulate double in software." | The RP2040 build is `-mcpu=cortex-m0plus` and arduino-pico defines `PICO_FLOAT_SUPPORT_ROM_V1` and `PICO_DOUBLE_SUPPORT_ROM_V1`. One hit: `sum += buf_[i].rate_q9;` (pico/aggreGATR/src/gyro_window.cpp:42), a float into a double sum, which looks intended. | GATE on the Pico target: -Wdouble-promotion | Pc |
| EX-05 | Signed types for arithmetic. Unsigned only for bits, wire fields and wrap-around clocks. | C++CG ES.100, ES.101, ES.102, ES.106 | The Pico clock compare `static_cast<int32_t>(a - b) <= 0` (pico/aggreGATR/src/gyro_window.h:12) is the intended wrap idiom. | ADVISORY: -Wsign-conversion outside C/Pc | all C++ |
| EX-06 | Floating equality only against exact sentinels such as 0 meaning off, preferably through a named helper. Otherwise use a tolerance. | MISRA C++:2023 Dir 0.3.1 "Floating-point arithmetic should be used appropriately" | -Wfloat-equal: 20 host hits, all intentional sentinel checks such as `if (gains_.kI != 0) {` (brain/actuGATR/src/pid.cpp:22). | REVIEW; do not enable -Wfloat-equal. `bugprone-float-loop-counter` GATE (6, tests only) | all C++ |
| EX-07 | No macros for constants or code. Use `constexpr`. Conditional compilation tests `defined(X)` or a macro that is always defined. | C++CG ES.30, Enum.1; Power of Ten rule 8; JPL C rule 20; AUTOSAR A16-0-1 | `#if NAVIGATR_HAVE_LIBCAMERA` (pi/naviGATR/src/impl/resources/cameras.cpp:12) relies on an undefined macro evaluating to 0. That gives 5 -Wundef hits. | GATE: -Wundef (host); `cppcoreguidelines-macro-to-enum` | all C++ |
| EX-08 | A switch over an enum lists every enumerator and has no `default`, so -Wswitch catches new enumerators. `default` only for non-enum switches. | LLVM "Don't use default labels in fully covered switches over enumerations"; Google "Switch Statements"; C++CG ES.79; MISRA C++:2023 9.4.2 | -Wswitch-enum finds 9 Brain switches over an enum ending in `default:`. Example: `switch (robot.status) {` ... `default: return MotionReason::kInputUnavailable;` (brain/actuGATR/src/motion.cpp:252, :256). A new status value would be silently absorbed. | REVIEW; ADVISORY: -Wswitch-enum (9 hits); GATE: `bugprone-switch-missing-default-case` for non-enum switches (3) | all C++ |
| EX-09 | No implicit fallthrough. | C++CG ES.78; Google | 0 hits at -Wimplicit-fallthrough=5 | GATE | all C++; V (`no-fallthrough` is in ESLint recommended) |
| EX-10 | One declaration per line. | C++CG ES.10, NL.21; JPL C rule 24 | `readability-isolate-declaration`: 148 (60 naviGATR src). Pico: `uint32_t g_boots = 0, g_retries = 0, g_rx = 0, g_bad_headers = 0;` (pico/aggreGATR/src/imu_bno08x.cpp:75). | GATE (auto-fix) | all C++ |
| EX-11 | Braces on every control statement (existing house rule). | House convention. Google and LLVM allow omitting them; LLVM "Don't Use Braces on Simple Single-Statement Bodies". | 124 missing-brace sites in 17 files per `InsertBraces` dry run, 40 in pi/naviGATR/src/config/composition.cpp (e.g. :102 `if (name == "Estimator") return {true, Section::kOpaque};`) and 36 in pi/naviGATR/src/impl/resources/pros_usb_link.cpp (:30 `if (c >= '0' && c <= '9') return c - '0';`). | GATE: clang-format `InsertBraces: true` (fixes it); `readability-braces-around-statements` | all C++; V (`curly`: 0 hits) |
| EX-12 | No shadowing. | C++CG ES.12; MISRA C++:2023 6.4.1 | -Wshadow: 9 host sites. `const CameraMount& c = in.cameras[i];` (brain/communiGATR/src/robot_profile.cpp:134) hides `const ImuCalibrationSettings& c` (:115). A lambda parameter `context` hides the outer `context` (pi/naviGATR/src/impl/resources/pico_telemetry.cpp:847). Both have different types, so -Wshadow=compatible-local would miss both. | GATE: full -Wshadow | all C++; V (`no-shadow`: 1) |
| EX-13 | No magic numbers in production code. Name the constant. | C++CG ES.45 | The checks report 5626. Most are geometry, test data and bit widths, so this stays a review rule. | REVIEW | all |

### 3.6 Memory, resources, real time (MEM)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| MEM-01 | Resources are owned by RAII handles. No naked `new`, `delete`, `malloc`, `free`. | C++CG R.1, R.10, R.11, R.12; AUTOSAR A18-5-1, A18-5-2; MISRA C++:2023 21.6.2; CERT MEM51-CPP | `client_.reset(new Client(port, ...));` (brain/communiGATR/sim/link_rig.cpp:43). `std::fclose(f_);` on a raw `std::FILE*` (pi/naviGATR/src/transport/file_stream.cpp:10). `modernize-make-unique`: 7. | GATE: `cppcoreguidelines-no-malloc`, `modernize-make-unique`, `modernize-make-shared` | all C++ |
| MEM-02 | Pico firmware allocates nothing. Buffers are fixed and sized by `constexpr`. Nothing allocates in an ISR. | Power of Ten rule 3 ("Do not use dynamic memory allocation after initialization."); JPL C rule 5; MISRA C++:2023 21.6.1; arduino-pico docs/hidmaster.rst ("should not do significant work, delay(), or allocate or free memory") | Already true. pico/aggreGATR/src has no `std::vector`, `std::string`, `std::function` or `new`. Example: `Report buf_[kCapacity] = {};` (pico/aggreGATR/src/gyro_window.h:46). The codec compiled into the Pico (translaGATR/frame_codec.cpp) has no containers either. | GATE: link with `-Wl,--wrap=malloc` to a trap in the Pico build, or a grep gate | Pc C |
| MEM-03 | Brain allocates at init, profile load or plan time, never per control cycle. | JPL C rule 5 ("after task initialization"); AUTOSAR A18-5-7 (only "during non-realtime program phases") | Brain types hold vectors, e.g. `std::vector<FieldObject> objects; // sorted by id` (brain/investiGATR/include/investigatr/field.h:63). That is fine at plan time and needs watching in the drive loop. | REVIEW | B |
| MEM-04 | On the Pi, prefer values and `unique_ptr`. `shared_ptr` only for real shared ownership, such as cross-thread snapshots. | C++CG R.5, R.20, R.21 | Snapshots are shared on purpose: `std::shared_ptr<const FieldSnapshot> System::fieldSnapshot() const` (pi/naviGATR/src/runtime/system.cpp:994). | REVIEW | Pi |
| MEM-05 | Per-function stack budget on targets: Pico 1024 bytes, Brain 4096 bytes. | GCC -Wstack-usage doc; JPL C rule 10 (safety margins); PROS rtos.h: TASK_STACK_DEPTH_DEFAULT is "8,192 words, or 32,768 bytes"; arduino-pico docs/multicore.rst: "core 0 has the full 8KB of stack" | Measured: Pico 0 functions over 1024 bytes. Brain 1: 1104 bytes at brain/investiGATR/src/planner.cpp:188 (`sweepFree`), well under 4096. On host builds the warning only fires in gtest bodies (186 of 191 hits), so it belongs on target builds only. | GATE on target builds: `-Wstack-usage=1024` (Pico), `-Wstack-usage=4096` (Brain) | Pc B |
| MEM-06 | No recursion on Brain, Pico and codec. On the Pi, recursion is allowed only with a stated depth bound. | Power of Ten rule 1; JPL C rule 4; AUTOSAR A7-5-2; MISRA C++:2023 8.2.10 | `misc-no-recursion` finds 0 in brain, pico and codec host TUs. naviGATR's XML include expansion recurses: `bool expand(tinyxml2::XMLElement*& node, Section section, const std::string& file,` (pi/naviGATR/src/config/composition.cpp:239). | GATE: `misc-no-recursion` in brain/, pico/, translaGATR/ overlays | B Pc C (Pi review) |
| MEM-07 | Every loop is bounded except one server loop per task or thread. Every busy-wait has a timeout. | Power of Ten rule 2; JPL C rule 3 (exception: "a single non-terminating loop per task or thread") | Good existing example: `while (!intLow()) {` guarded by `if (time_us_32() - t0 > kWakeUs) {` (pico/aggreGATR/src/imu_bno08x.cpp:193-194). | REVIEW; `bugprone-infinite-loop` GATE | B Pc Pi |
| MEM-08 | Raw pointer plus length only at the codec edge, behind length checks. Elsewhere use `std::array` or containers. | C++CG SL.con.1, I.13; CERT CTR50-CPP, CTR53-CPP | The codec is pointer arithmetic by nature: `const uint8_t* at = body + 1 + kWheelReadingLen * i;` (translaGATR/frame_codec.cpp:759). `cppcoreguidelines-pro-bounds-pointer-arithmetic` reports 498 in translaGATR src, so the check is useless there and tests (TST-05) carry the load. | Tests plus sanitizers; `-D_GLIBCXX_ASSERTIONS` in host debug builds | C Pc |

### 3.7 Concurrency and interrupts (CON)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| CON-01 | Each shared mutable datum has one owner and one guard, declared next to it. Locks are RAII and named. | C++CG CP.2, CP.3, CP.20, CP.44, CP.50 | naviGATR src uses `std::mutex` 194 times and `lock_guard` 166 times. `modernize-use-scoped-lock` suggests `std::scoped_lock` at 149 sites (style only). | GATE: `bugprone-unused-raii`; TSan job (TST-09); ADVISORY: scoped_lock | Pi |
| CON-02 | Never use sleeps or delays to synchronize. Wait on a condition in a loop. | JPL C rule 7 ("Task synchronization shall not be performed through the use of task delays."); C++CG CP.42 "Don't `wait` without a condition"; CERT CON54-CPP | Correct pattern already present: `wake_.wait_for(lock, std::chrono::milliseconds(options_.pump_ms),` with a predicate lambda that returns when `stopped_` or `woken_` is set (pi/naviGATR/src/capture/capture_service.cpp:305-306, snippet trimmed). | GATE: `bugprone-spuriously-wake-up-functions`; REVIEW | Pi B |
| CON-03 | Never call unknown code (callbacks, std::function) while holding a lock. | C++CG CP.22 | naviGATR composes stages through `std::function` executables (`executable.execute = [telemetry, outputs](...)`, pi/naviGATR/src/impl/resources/pico_telemetry.cpp:847). | REVIEW | Pi |
| CON-04 | No thread-unsafe libc calls in threaded code. | clang-tidy `concurrency-mt-unsafe` doc ("Checks for some thread-unsafe functions against a black list of known-to-be-unsafe functions") | `std::string socketError() { return std::strerror(errno); }` (pi/naviGATR/src/inspection/http_server.cpp:137) runs in the HTTP server. POSIX does not require `strerror` to be thread-safe. The MinGW run reported 0 hits, so re-measure on Linux. | GATE: `concurrency-mt-unsafe` (Linux CI) | Pi |
| CON-05 | Pico ISR rules. <br>- An ISR reads hardware and writes `volatile` flags, counters and timestamps only. <br>- No allocation, `delay()`, Serial or SPI in an ISR. <br>- Main-loop reads of multi-field ISR state happen inside `noInterrupts()`/`interrupts()`. <br>- ISR functions are named `on*` or `isr*`. | Arduino attachInterrupt reference: "an ISR should be as short and fast as possible", "declare them as `volatile`"; arduino-pico docs/hidmaster.rst | Already correct. <br>- `void onInt() {` stores `time_us_32()` and a flag (pico/aggreGATR/src/imu_bno08x.cpp:87). <br>- `takeEdge()` copies both under `noInterrupts();` (:96) ... `interrupts();` (:100). <br>- The encoder counter is `volatile int32_t count = 0;` (pico/aggreGATR/src/quadrature.cpp:24), zeroed under `noInterrupts();` (:79). | REVIEW checklist | Pc |
| CON-06 | `volatile` is never used for thread synchronization on Pi or Brain. Use `std::atomic`. Exception: `volatile std::sig_atomic_t` signal flags. | C++CG CP.8 "Don't try to use `volatile` for synchronization" | Pi uses `std::atomic` (26 uses in src). The signal flag `volatile std::sig_atomic_t g_stop = 0;` (pi/naviGATR/src/app/main.cpp:36) is the allowed exception, although `cppcoreguidelines-avoid-non-const-global-variables` flags it. | GATE: grep for `volatile` outside pico/ with an allowlist | Pi B |
| CON-07 | PROS tasks delay in every loop. One task owns each motor or device. | PROS multitasking tutorial: "Every tasks' loop should have a `delay()` statement"; it recommends "tasks which will never access the same variables or data" | actuGATR already has a single drive owner (brain/actuGATR/include/actugatr/drive_owner.h). | REVIEW | B |

### 3.8 Error handling and assertions (ERR)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| ERR-01 | One documented error strategy per layer. <br>- Pi: `bool` or `std::optional` plus an error string. <br>- Brain and Pico: status enums. <br>- No exceptions across module boundaries. Exceptions stay off on the Pico. | C++CG E.1 "Develop an error-handling strategy early in a design", E.25, E.26, E.27; AUTOSAR A15-0-1 | Measured: 0 `throw` in brain, pico and translaGATR. naviGATR has 1 `throw` and 3 `try`, all at the libcamera boundary (pi/naviGATR/src/impl/resources/libcamera_camera.cpp:409), and 217 `std::string& err` parameters. arduino-pico builds with `-fno-exceptions` (platform.txt `build.flags.exceptions=-fno-exceptions`). | REVIEW; document in each PIPELINE.md | all |
| ERR-02 | Destructors, moves and swaps never throw. | C++CG C.37 "Make destructors `noexcept`"; CERT DCL57-CPP "Do not let exceptions escape from destructors or deallocation functions" | `bugprone-exception-escape`: `CaptureRecorder::~CaptureRecorder() { stop(); }` (pi/naviGATR/src/capture/capture_service.cpp:313), where `stop()` joins a thread and can throw. | GATE: `bugprone-exception-escape`, `cppcoreguidelines-noexcept-*` | Pi B |
| ERR-03 | Degrade, don't hang: every wait on an upstream has a timeout and a stale flag. This is a project principle, and pipes and filters gives it an industry frame. | Microsoft Pipes and Filters: "Every filter must determine what to do in the case of a breaking error."; Power of Ten rule 2 | Consistent with the Pico WAKE timeout (MEM-07) and the Pi sensor-loss rule (pi/naviGATR/src/runtime/sensor_loss.cpp). | TST-10 fault-injection tests; REVIEW | all |
| ERR-04 | Assert invariants that "cannot happen". Assertions have no side effects. | Power of Ten rule 5; JPL C rule 16; LLVM "Assert Liberally"; C++CG I.6 | Tests are the main guard today. Asserts stay enabled in host test builds. | GATE: `bugprone-assert-side-effect` | all C++ |

### 3.9 Architecture, folders, pipelines (ARC)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| ARC-01 | The folder tree tells the pipeline. <br>- A component folder is one pipeline. <br>- Its sub-folders are stages (sub-pipelines), shared contracts, and plug-in implementations. <br>- Framework and implementation are visibly separate. <br>- A stage folder name matches the stage name in code and in config. | Screaming Architecture (quote in section 2); LLVM "Library Layering" | naviGATR has the parts: `contracts/`, `impl/<stage>/` and `runtime/<stage>_stage.cpp`. But `runtime/` mixes the stage builders with profile swapping and Pico control, and src/ has 14 sibling folders with no top-level map. | REVIEW; DOC-01 codemap | all |
| ARC-02 | One composition root per executable, next to `main`. Reading `main` plus one file shows parse, register, build every stage in order, and the per-cycle call order. | Seemann, Composition Root (quotes in section 2) | Today the chain spans 4 files and 3 naming styles: <br>- `navigatr::registerAll(functions);` (pi/naviGATR/src/app/main.cpp:94) fans out to seven `register_*` calls (pi/naviGATR/src/runtime/register_all.cpp:9-17). <br>- `System::buildFromFile(...)` (main.cpp:97-98) leads to `bool System::build(...)` (pi/naviGATR/src/runtime/system.cpp:148), which calls `make_resources` (:194), `make_sensors` (:202), `make_localization` (:294) and `make_world_estimation` (:317), defined in runtime/*_stage.cpp. <br>- The per-cycle pipeline is `void System::step(MonotonicTime now) {` (system.cpp:1022), calling `estimationCycle`, `fieldCycle` and `reportingCycle`. It sits inside a 1450-line file that also holds profile and Pico control. <br>- pi/naviGATR/docs/architecture.md does describe the chain, but a reader starting from src/ cannot find it. | REVIEW; PIPELINE.md must name these entry functions (DOC-01) | Pi (B, Pc same idea) |
| ARC-03 | Pipes and filters. <br>- Each stage has one declared Input struct and one Output struct. <br>- Stages never include each other's implementation folders. <br>- The runner (pipe) holds no stage logic. | Microsoft Pipes and Filters: "Filters act independently and are unaware of other filters. They're only aware of their input and output schemas." | contracts/*.h already declares the stage I/O structs (`PublishingInput`/`PublishingOutput`, pi/naviGATR/src/contracts/publishing.h:27, :40). This is the user's "standard INPUT/OUTPUT". | GATE: include-graph script (`impl/<a>/` may not include `impl/<b>/`); or one CMake library per stage so the link fails | Pi B |
| ARC-04 | Ports and adapters at the hardware edge. PROS, Arduino, libcamera, sockets and serial code live only in adapter folders or files. Everything else compiles and runs on the host. | Cockburn, Hexagonal Architecture (intent quote in section 2) | Brain separates `pros/` adapters (brain/actuGATR/pros/pros_drive.cpp) from host-built `src/` and `sim/` fakes. The Pico host-builds 10 of 14 .cpp (pico/aggreGATR/tests/CMakeLists.txt). Only `main.cpp`, `imu_bno08x.cpp`, `imu_asm330.cpp` and `quadrature.cpp` need Arduino. | GATE: every non-adapter source is in a host build (CMake lists already explicit) | B Pc Pi |
| ARC-05 | Dependencies point one way (app to runtime to impl to contracts to core) with no cycles. | LLVM "Library Layering"; C++CG SF.9 | `misc-header-include-cycle`: 0 today, cheap to keep. | GATE: `misc-header-include-cycle`; per-layer CMake libraries | all C++ |
| ARC-06 | Interfaces are extracted, not invented: freeze an abstraction only after two concrete uses. | Google eng-practices over-engineering quote; C++CG C.10, C.120 | Project principle, already in memory. The industry sources agree. | REVIEW | all |

### 3.10 Comments and documentation (DOC)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| DOC-01 | Every pipeline and every sub-pipeline folder has one short `PIPELINE.md`: <br>- one paragraph on what problem it answers; <br>- inputs and outputs (the structs); <br>- stages in call order, naming the entry functions; <br>- where construction happens (`make_*`, `register_*`); <br>- invariants and boundaries. <br>It names symbols and does not link to them. | matklad ARCHITECTURE.md ("bird's eye overview", "codemap", "Keep it short", "Do name important files, modules, and types. Do not directly link them", "Explicitly call-out architectural invariants", "Point out boundaries between layers and systems") | The user asked for this. naviGATR has pi/naviGATR/docs/architecture.md with a "Construction" section, but it is not next to the code. Its stage list (`make_resources -> ResourceStore + ResourceExecutor`) has to be matched to runtime/resource_stage.cpp:87 by hand. | GATE: script checks each pipeline folder has PIPELINE.md and every backticked symbol in it exists (possible because it names, not links) | all |
| DOC-02 | A root `PIPELINE.md` for the whole system: Pico to Pi to Brain data flow, the wire contract (translaGATR), and which component owns which question ("where am I", "where are things"). | matklad (same) | docs/interfaces.md and docs/naming.md exist, but nothing gives the one-page system flow. | REVIEW | all |
| DOC-03 | Comments are terse labels that say what a thing is. No narration and no commented-out code. | C++CG NL.1 "Don't say in comments what can be clearly stated in code", NL.3 "Keep comments crisp"; MISRA C++:2023 Dir 5.7.2 "Sections of code should not be 'commented out'". In conflict: LLVM ("write them as English prose, using proper capitalization, punctuation") and AUTOSAR A2-7-3 (document every declaration). | The user's comment rules (feedback_comment_style.md) are stricter than LLVM or Google and win. The existing file heads follow them (`// pid.h` plus one line). | REVIEW | all |
| DOC-04 | Plain ASCII in source, docs and comments. | Google "Non-ASCII Characters" ("should be rare, and must use UTF-8 formatting"). The user's rule is stricter. | Measured: 0 non-ASCII characters in tracked C++, JS, Python, XML and docs, except 7 pcb READMEs with one middle-dot separator line each (e.g. pcb/IMU/IMU_v1/README.md:3). | GATE: `grep -P '[^\x00-\x7F]'` over tracked text files | all |
| DOC-05 | READMEs are short signposts. Deep docs live in `docs/`. A change that alters a pipeline updates its PIPELINE.md in the same PR. | Google eng-practices "Documentation" ("Check to see that it also updates associated documentation") | Existing project convention (feedback_conventions.md) | REVIEW | all |

### 3.11 Testing (TST)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| TST-01 | Tests are deterministic. <br>- No wall-clock sleeps; time comes from an injected clock. <br>- Real-thread integration tests poll with a deadline, never a bare sleep. | Fowler, "Eradicating Non-Determinism in Tests": "Never use bare sleeps to wait for asynchonous responses: use a callback or polling." and "Always wrap the system clock, so it can be easily substituted for testing." | 45 `sleep_for` in naviGATR tests, including `std::this_thread::sleep_for(std::chrono::milliseconds(2500));` (pi/naviGATR/tests/capture_gtest.cpp:2025), there because "the recorder reads the host once a second" (:2023). The seam already exists: `std::function<int64_t()> clock; // Pi host us; HostClock::nowUs when empty` (pi/naviGATR/src/capture/capture_recorder.h:57). `HostClock` itself is static and not injectable (pi/naviGATR/src/core/host_clock.h:16). | GATE: grep for `sleep_for` in tests with an allowlist; ctest per-test timeout | Pi (B, Pc: 0 today) |
| TST-02 | Prefer real implementations, then fakes. Mocks only for interaction that cannot be observed. The owner of the real code maintains the fake, and the fake has its own tests. | Software Engineering at Google ch. 13: "our first choice for tests is to use the real implementations", "A fake must have its own tests to ensure that it conforms to the API of its corresponding real implementation.", "The team that owns the real implementation should write and maintain a fake." | Brain already does this: brain/communiGATR/sim/fake_pi.cpp is tested by brain/communiGATR/tests/fake_pi_gtest.cpp. naviGATR's synthetic rig (pi/naviGATR/src/impl/resources/synthetic_rig.cpp) is a fake resource. | REVIEW | B Pi Pc |
| TST-03 | State testing over interaction testing. | SWE at Google ch. 13 ("Prefer State Testing Over Interaction Testing") | Tests assert on outputs (pose, frames, documents). Keep it that way. | REVIEW | all |
| TST-04 | Randomized tests use fixed seeds and report the seed on failure. | Fowler (non-determinism); RapidCheck and FuzzTest (property-based testing) | Model to copy: "// Randomized fields with fixed seeds" (brain/investiGATR/tests/planner_random_gtest.cpp:2), `for (unsigned seed = 1; seed <= 300; ++seed) {` (:633) with `SCOPED_TRACE("seed " + ...)` (:651). | REVIEW | all |
| TST-05 | Codec property tests: <br>- round-trip `decode(encode(x)) == x` over generated valid frames; <br>- the decoder never crashes or over-reads on random or truncated bytes; <br>- runs under ASan and UBSan. | Google FuzzTest: "property-based tests executed using coverage-guided fuzzing"; RapidCheck ("QuickCheck clone for C++"); CERT CTR50-CPP, STR50-CPP | translaGATR is the narrow seam compiled into three targets. Its tests are example-based: about 174 `TEST` lines in translaGATR/tests/frame_codec_gtest.cpp, including hand-picked `TEST(Sensor, CorruptionRejected) {` (:1010). The pseudo-random tests cover only CRCs and reader noise accounting: `TEST(Crc, MatchesReferenceOnPseudoRandomData) {` (:748) and `TEST(ReaderStats, PseudoRandomNoiseStaysAccounted) {` (:3500). None is an encode/decode round trip. | GATE: seeded random round-trip gtest (no new dependency); optional libFuzzer/FuzzTest job on Linux clang | C (Pc) |
| TST-06 | Test code gets the same warnings and a lighter clang-tidy overlay: long bodies, literals and optional access after an ASSERT are fine. | (practice) | Host -Wstack-usage: 186 of 191 hits are gtest bodies. `bugprone-unchecked-optional-access`: 142 of 151 are tests that check with ASSERT first. | tests/.clang-tidy overlay (section 4.2) | all C++ |
| TST-07 | Every non-adapter source is compiled and tested on the host. The target build only adds the adapters. | Cockburn intent quote | See ARC-04. The explicit CMake lists (feedback_conventions.md) make gaps visible. | GATE (CI host build) | B Pc Pi C |
| TST-08 | Host tests also run under sanitizers in CI. | GCC Instrumentation Options (ASan, UBSan, TSan); C++CG CP.9 "Whenever feasible use tools to validate your concurrent code" | Measured: MinGW GCC 15.2 has no libasan, libubsan or libtsan. Only UBSan trap mode links. naviGATR runs real threads (12 `std::thread` in src). | GATE on Linux CI: ASan+UBSan job; separate TSan job for naviGATR | B Pi Pc C |
| TST-09 | Fault injection: each consumer is tested with its upstream dead, stale, rebooting or sending garbage. | Project principle (degrade, don't hang); Pipes and Filters error-handling note | The Brain link already has recovery tests (brain/communiGATR/tests/recovery_gtest.cpp) and Pico reboot handling appears in capture tests. Make this a rule for every seam. | REVIEW | all |

### 3.12 Build, warnings, analysis, formatting (BLD)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| BLD-01 | Libraries are ISO C++17 without GNU extensions (`CMAKE_CXX_EXTENSIONS OFF`). PROS apps build as gnu++20 because the kernel headers need it. The libcamera backend builds as C++20. Library code must compile in all three modes. | C++CG P.2 "Write in ISO Standard C++"; Google ("Do not use non-standard extensions") | brain/CMakeLists.txt:6 has `set(CMAKE_CXX_EXTENSIONS OFF)`. naviGATR does not (pi/naviGATR/CMakeLists.txt:4-5), so its compile database shows `-std=gnu++17`. Also `CXX_STANDARD:=gnu++20` (brain/locaGATR/Makefile:22, comment "# Kernel headers need C++20.", :20) and `target_compile_features(navigatr_core PUBLIC cxx_std_20)` (pi/naviGATR/CMakeLists.txt:197). | GATE | all C++ |
| BLD-02 | GATR2-owned code has zero warnings under the target's tiered warning set (section 5). | Power of Ten rule 10 ("compiled ... with all compiler warnings enabled ... compile with these setting without any warnings"); JPL C rule 2; LLVM "Treat Compiler Warnings Like Errors" | The baseline `-Wall -Wextra` is already at 0 on all 265 TUs measured. The tier-1 host additions cost 18 sites. | GATE (see 5.5 for -Werror) | all C++ |
| BLD-03 | Vendored code is excluded from every gate: `-w` or SYSTEM includes, plus an exclude filter in clang-tidy. | JPL C rule 1 scope; practice | Already done for apriltag (`target_compile_options(navigatr_apriltag PRIVATE -w)`, pi/naviGATR/CMakeLists.txt:60) and tinyxml2 (`target_include_directories(navigatr_tinyxml2 SYSTEM PUBLIC third_party)`, :13). PROS headers are not excluded: common.mk passes them with `-iquote` (brain/locaGATR/common.mk:150), so they warn as if they were ours. | GATE | all |
| BLD-04 | Static analysis on every PR with the checked-in `.clang-tidy`. | Power of Ten rule 10; C++CG P.12 "Use supporting tools as appropriate" | Section 4 | GATE (Linux CI) | all C++ |
| BLD-05 | clang-format is enforced and matches the house style. `InsertBraces: true`. One mass-reformat commit recorded in `.git-blame-ignore-revs`. | Google eng-practices "Style"; project convention | 369 of 437 files differ today. `.clang-format` does not encode the house style actually used: access specifiers at column 0 in Brain and naviGATR vs indented in Pico, one-line `case X: return Y;`, include order changed by the translaGATR rename, and alignment. | GATE | all C++ |
| BLD-06 | A complexity report (lizard) on every PR, with a ratchet. | JPL C rule 25; Google eng-practices | Section 1.2 | ADVISORY | all |
| BLD-07 | If warnings-as-errors is driven from CMake, `cmake_minimum_required` must be at least 3.24. | CMake COMPILE_WARNING_AS_ERROR (added in 3.24; `cmake --compile-no-warning-as-error` overrides it) | Every CMakeLists.txt says `cmake_minimum_required(VERSION 3.16)`. | DECISION (with 5.5) | all CMake |

### 3.13 Viewer JavaScript (JS)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| JS-01 | ESLint `js.configs.recommended` is a gate. | ESLint recommended set (64 rules in @eslint/js for ESLint 10) | 7 findings: 6 `no-unused-vars` (e.g. unused import `KeyedTable`, pi/naviGATR/spectaGATR/camera_panel.js:19) and 1 `no-useless-assignment` (pi/naviGATR/tools/perf/perf_probe.js:215). | GATE | V T |
| JS-02 | `===`, `const`/`let` only, braces always, no shadowing. | ESLint `eqeqeq`, `no-var`, `prefer-const`, `curly`, `no-shadow` | 0, 0, 0, 0 and 1 findings, so all nearly free. | GATE | V T |
| JS-03 | Same size and complexity limits as C++. | ESLint `complexity`, `max-depth`, `max-lines-per-function` | complexity 15: 34 hits (max 77, `renderStatus`, pi/naviGATR/spectaGATR/app.js:852). max-lines 80: 7. max-depth 4: 2. | ADVISORY with ratchet | V |
| JS-04 | Functions and plain objects by default. `class` only for stateful widgets. This is the user's functional rule applied to JS. | User rule; ESLint `no-restricted-syntax` can flag `ClassDeclaration` | 26 class declarations across 15 modules | DECISION; REVIEW | V |
| JS-05 | No implicit coercion (`!!x`, `+s`). | ESLint `no-implicit-coercion` | 35 findings, e.g. pi/naviGATR/spectaGATR/app.js:493 | ADVISORY | V |

### 3.14 Python and scripts (PY)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| PY-01 | `ruff check` and `ruff format` with line length 100, matching the C++ ColumnLimit. | PEP 8 ("it is okay to increase the line length limit up to 99 characters" for teams that agree); Ruff v0.16 release notes (the default set grew to 413 rules) | ruff 0.16.9 defaults: 37 findings over 7 files (2976 lines), e.g. UP031 6, SIM115 4, BLE001 2. The classic E4/E7/E9/F set: 4. At the default 88 columns, E501 fires 399 times. | GATE: ruff defaults; ADVISORY: C90/PLR | T |
| PY-02 | Python inside naviGATR needs a written justification. This is a project rule. | feedback_conventions.md | 6 tracked .py files in pi/naviGATR/tools (e.g. tools/calibrate_camera.py, tools/viewer/viewer_tests.py), plus shell scripts and JS tools. The rule and the tree disagree. | DECISION | T |

### 3.15 Code review (REV)

| ID | Rule | Source | Why here (evidence) | Enforcement | Applies |
|---|---|---|---|---|---|
| REV-01 | The review checklist follows Google eng-practices headings: design, functionality, complexity (including over-engineering), tests, naming, comments, style, documentation, every line, context. Add one GATR2 item: does this widen a seam or leak hardware knowledge across it? | Google eng-practices "What to look for in a code review" | The design principles make the seam the product (feedback_design_principles.md). | REVIEW (PR template) | all |
| REV-02 | Changes to translaGATR or to contracts/*.h need a reviewer from each side of the seam. | Practice (CODEOWNERS) | The codec is compiled by Brain, Pi and Pico | GATE: CODEOWNERS | C Pi |

---

## 4. Recommended .clang-tidy (clang-tidy 22.1.8)

### 4.1 Root config

Check names come from `clang-tidy --list-checks` of 22.1.8 (606 checks). Option names come from
`--dump-config`. This config parses and was run on all 210 host TUs.

```yaml
---
Checks: >
  -*,
  bugprone-*,
  -bugprone-easily-swappable-parameters,
  -bugprone-narrowing-conversions,
  -bugprone-implicit-widening-of-multiplication-result,
  -bugprone-throwing-static-initialization,
  cppcoreguidelines-avoid-goto,
  cppcoreguidelines-init-variables,
  cppcoreguidelines-interfaces-global-init,
  cppcoreguidelines-missing-std-forward,
  cppcoreguidelines-no-malloc,
  cppcoreguidelines-noexcept-destructor,
  cppcoreguidelines-noexcept-move-operations,
  cppcoreguidelines-noexcept-swap,
  cppcoreguidelines-prefer-member-initializer,
  cppcoreguidelines-pro-type-cstyle-cast,
  cppcoreguidelines-pro-type-member-init,
  cppcoreguidelines-rvalue-reference-param-not-moved,
  cppcoreguidelines-slicing,
  cppcoreguidelines-special-member-functions,
  cppcoreguidelines-virtual-class-destructor,
  misc-*,
  -misc-include-cleaner,
  -misc-const-correctness,
  -misc-no-recursion,
  modernize-avoid-bind,
  modernize-avoid-c-style-cast,
  modernize-deprecated-headers,
  modernize-loop-convert,
  modernize-make-shared,
  modernize-make-unique,
  modernize-redundant-void-arg,
  modernize-use-bool-literals,
  modernize-use-equals-default,
  modernize-use-equals-delete,
  modernize-use-nullptr,
  modernize-use-override,
  modernize-use-using,
  performance-*,
  -performance-enum-size,
  readability-braces-around-statements,
  readability-container-size-empty,
  readability-duplicate-include,
  readability-else-after-return,
  readability-function-cognitive-complexity,
  readability-identifier-naming,
  readability-inconsistent-declaration-parameter-name,
  readability-isolate-declaration,
  readability-misleading-indentation,
  readability-redundant-casting,
  readability-redundant-control-flow,
  readability-redundant-declaration,
  readability-simplify-boolean-expr,
  readability-static-accessed-through-instance,
  readability-use-anyofallof,
  concurrency-*
WarningsAsErrors: ''
HeaderFilterRegex: '.*[/\\](brain|pi|pico|translaGATR)[/\\].*'
ExcludeHeaderFilterRegex: '.*[/\\](third_party|_deps|include[/\\]pros)[/\\].*'
SystemHeaders: false
FormatStyle: none
CheckOptions:
  modernize-use-using.IgnoreExternC: true
  cppcoreguidelines-special-member-functions.AllowSoleDefaultDtor: true
  cppcoreguidelines-special-member-functions.AllowMissingMoveFunctionsWhenCopyIsDeleted: true
  misc-non-private-member-variables-in-classes.IgnoreClassesWithAllMemberVariablesBeingPublic: true
  readability-function-cognitive-complexity.Threshold: 25
  readability-function-cognitive-complexity.IgnoreMacros: true
  readability-braces-around-statements.ShortStatementLines: 0
  readability-identifier-naming.NamespaceCase: lower_case
  readability-identifier-naming.ClassCase: CamelCase
  readability-identifier-naming.StructCase: CamelCase
  readability-identifier-naming.EnumCase: CamelCase
  readability-identifier-naming.TypeAliasCase: CamelCase
  readability-identifier-naming.TemplateParameterCase: CamelCase
  readability-identifier-naming.EnumConstantCase: CamelCase
  readability-identifier-naming.EnumConstantPrefix: k
  readability-identifier-naming.FunctionCase: camelBack
  readability-identifier-naming.MethodCase: camelBack
  readability-identifier-naming.VariableCase: lower_case
  readability-identifier-naming.ParameterCase: lower_case
  readability-identifier-naming.PrivateMemberCase: lower_case
  readability-identifier-naming.PrivateMemberSuffix: _
  readability-identifier-naming.PublicMemberCase: lower_case
  readability-identifier-naming.ConstexprVariableCase: CamelCase
  readability-identifier-naming.ConstexprVariablePrefix: k
  readability-identifier-naming.GlobalConstantCase: CamelCase
  readability-identifier-naming.GlobalConstantPrefix: k
  readability-identifier-naming.MacroDefinitionCase: UPPER_CASE
...
```

Notes:
- In 22.1.8 `HeaderFilterRegex` defaults to `'.*'`, so without `ExcludeHeaderFilterRegex` the
  vendored headers under `third_party/` would be reported.
- The `[/\\]` classes make the same file work on Windows and Linux paths.
- The naming block is only a proposal until NM-01 is decided.

### 4.2 Directory overlays

- `brain/.clang-tidy`, `pico/.clang-tidy`, `translaGATR/.clang-tidy` (MEM-06):
  `InheritParentConfig: true` and `Checks: 'misc-no-recursion'`.
- `pi/naviGATR/.clang-tidy`: nothing extra. The Pi allows bounded recursion (MEM-06).
- Every `tests/.clang-tidy` (TST-06): `InheritParentConfig: true` and
  `Checks: '-bugprone-unchecked-optional-access,-readability-function-cognitive-complexity,-misc-non-private-member-variables-in-classes,-misc-use-internal-linkage'`.
- `translaGATR/.clang-tidy` could add `cppcoreguidelines-pro-bounds-constant-array-index` as
  advisory. Do not add `pro-bounds-pointer-arithmetic` (498 hits, it is the codec's nature).

### 4.3 Measured findings of the recommended config

The recommended config reports 1104 findings, 569 of them outside tests:

| Scope | codec | codec tests | brain | brain tests | pico | pico tests | navi | navi tests |
|---|---|---|---|---|---|---|---|---|
| Findings | 7 | 38 | 52 | 51 | 34 | 13 | 476 | 433 |

Per check. The totals include tests; the notes give the non-test share.

| Check | Findings | Notes |
|---|---|---|
| readability-identifier-naming | 349 | 225 non-test. By kind: 73 public member, 67 variable (mostly lambdas named like functions, e.g. `selectConfig` pi/naviGATR/src/app/command_line.cpp:16), 35 function (`make_*`, `register_*`), 26 enum constant (Pico), 18 parameter. Needs the NM-01 decision. |
| bugprone-unchecked-optional-access | 151 | 9 non-test. Tests are silenced by the overlay. |
| readability-isolate-declaration | 148 | Auto-fix |
| readability-braces-around-statements | 108 | clang-format InsertBraces fixes these |
| readability-function-cognitive-complexity | 78 | 71 non-test. ADVISORY with ratchet. |
| misc-non-private-member-variables-in-classes | 65 | 11 brain src, 9 brain tests, 45 navi tests |
| modernize-avoid-c-style-cast | 30 | 20 in translaGATR tests |
| readability-redundant-casting | 16 | |
| performance-move-const-arg | 16 | |
| readability-use-anyofallof | 15 | |
| cppcoreguidelines-special-member-functions | 14 | |
| cppcoreguidelines-prefer-member-initializer | 14 | |
| performance-inefficient-vector-operation | 12 | |
| everything else | 1 to 9 each | Real bugs worth a look include `bugprone-exception-escape` (1), `bugprone-misplaced-widening-cast` (1, tests), `bugprone-integer-division` (4), `bugprone-too-small-loop-variable` (9, codec tests), `bugprone-branch-clone` (1), `misc-redundant-expression` (1). One false positive: `bugprone-random-generator-seed` at pi/naviGATR/src/impl/commands/brain_link_commands.cpp:158, where the `std::mt19937` member is reseeded from a `std::seed_seq` in the constructor body. `cppcoreguidelines-virtual-class-destructor` (2) fires on the tinyxml2 `XMLElement` forward declaration and needs a NOLINT or the vendored type excluded. |

### 4.4 Checks considered and rejected

These are the counts from the full-group run (unique locations, tests included):

| Check | Findings | Reason |
|---|---|---|
| cppcoreguidelines-avoid-magic-numbers / readability-magic-numbers | 5626 | Geometry, bit widths and test data. EX-13 is a review rule. |
| readability-identifier-length | 5440 | Noise; NM-04 is a review rule |
| misc-include-cleaner | 4373 | libstdc++/MinGW mapping noise; revisit on Linux |
| readability-uppercase-literal-suffix | 1488 | Style with no bug value |
| cppcoreguidelines-pro-bounds-avoid-unchecked-container-access | 1096 | GSL-oriented |
| cppcoreguidelines-pro-bounds-pointer-arithmetic | 720 | Codec nature (MEM-08) |
| readability-math-missing-parentheses | 704 | Noise |
| cppcoreguidelines-pro-bounds-constant-array-index | 525 | GSL-oriented |
| modernize-use-nodiscard | 439 | Too broad; FN-07 targets status functions |
| misc-const-correctness | 380 | Auto-fixable later; not a bug gate |
| cppcoreguidelines-pro-bounds-array-to-pointer-decay | 373 | GSL-oriented |
| cppcoreguidelines-avoid-c-arrays / modernize-avoid-c-arrays | 312 | Fixed arrays are the embedded idiom (MEM-02) |
| modernize-use-scoped-lock | 149 | Style; ADVISORY at most |
| modernize-use-auto | 103 | Conflicts with readability-first |
| cppcoreguidelines-pro-type-vararg | 62 | printf-style logging on Pico and Pi is intended |
| readability-named-parameter | 58 | Contradicts C++CG F.9 |
| cppcoreguidelines-avoid-const-or-ref-data-members | 53 | Stage input views (TYP-10) |
| bugprone-easily-swappable-parameters | 47 | ADVISORY via TYP-06 review |
| readability-implicit-bool-conversion | 43 | Noise vs value |
| cppcoreguidelines-avoid-non-const-global-variables | 39 | Pico ISR state and signal flags are legitimate globals |
| cppcoreguidelines-use-enum-class | 37 | Wire enums (TYP-11) |
| bugprone-narrowing-conversions | 33 | GCC -Wconversion covers it per component |
| cppcoreguidelines-pro-type-reinterpret-cast | 19 | Needed at byte IO edges |
| performance-enum-size | 14 | Wire enums are sized on purpose |
| modernize-concat-nested-namespaces | 1 | Conflicts with the house rule of one brace per namespace |
| modernize-use-trailing-return-type | not run | Style churn |

---

## 5. Recommended compiler warning sets

### 5.1 Measured noise, host GCC 15.2 (210 TUs, `-O2`, GATR2 code only)

| Flag | codec | codec tests | brain | brain tests | pico | pico tests | navi | navi tests | Total |
|---|---|---|---|---|---|---|---|---|---|
| -Wstack-usage=4096 | 0 | 0 | 0 | 100 | 0 | 0 | 5 | 86 | 191 |
| -Wsign-conversion | 0 | 5 | 7 | 7 | 0 | 0 | 41 | 27 | 87 |
| -Wnull-dereference | 0 | 36 | 0 | 0 | 0 | 0 | 7 | 7 | 50 |
| -Wconversion | 0 | 17 | 0 | 1 | 0 | 0 | 17 | 12 | 47 |
| -Wfloat-equal | 0 | 0 | 9 | 5 | 0 | 0 | 6 | 0 | 20 |
| -Wuseless-cast | 0 | 1 | 1 | 0 | 0 | 0 | 14 | 1 | 17 |
| -Wswitch-enum | 0 | 0 | 9 | 0 | 0 | 0 | 0 | 0 | 9 |
| -Wshadow | 0 | 0 | 1 | 0 | 0 | 0 | 5 | 3 | 9 |
| -Wdouble-promotion | 0 | 0 | 0 | 0 | 1 | 0 | 3 | 3 | 7 |
| -Wundef | 0 | 0 | 0 | 0 | 0 | 0 | 2 | 3 | 5 |
| -Wmissing-declarations | 0 | 0 | 0 | 2 | 0 | 0 | 0 | 1 | 3 |
| -Woverloaded-virtual | 0 | 0 | 0 | 0 | 0 | 0 | 1 | 0 | 1 |

These flags gave 0 hits in GATR2 code:

- -Wnon-virtual-dtor, -Wold-style-cast, -Wsuggest-override
- -Wcast-align=strict, -Wcast-qual
- -Wduplicated-cond, -Wduplicated-branches, -Wlogical-op
- -Wformat=2, -Wpedantic
- -Wextra-semi, -Wzero-as-null-pointer-constant, -Wredundant-decls
- -Wvla, -Walloca, -Wimplicit-fallthrough=5
- -Wmismatched-tags, -Wredundant-tags
- -Wcomma-subscript, -Wdeprecated-copy-dtor, -Wctor-dtor-privacy

googletest headers, included as SYSTEM, still reported 255 -Wold-style-cast and 120
-Wdouble-promotion. Those are not counted here.

### 5.2 Measured noise, Pico target (arm-none-eabi 14.3, 15 TUs)

- GATR2 code: 7 warnings.
  - -Wuseless-cast (6), e.g. `static_cast<unsigned long>(g_commands.received()),`
    (pico/aggreGATR/src/main.cpp:265). The cast is needed on x86_64, where `uint32_t` is
    `unsigned int`, and useless on arm-none-eabi, where it is `unsigned long`.
  - -Wdouble-promotion (1).
- arduino-pico framework headers: 230 warnings, mostly from pico-sdk/src/rp2_common and
  cores/rp2040/RP2040Support.h. By flag: -Wundef 74, -Wold-style-cast 47, -Wsign-conversion 28,
  -Wredundant-decls 16, -Wnon-virtual-dtor 12, -Wsuggest-override 12, -Wshadow 9,
  -Wconversion 8, -Wextra-semi 7, -Wuseless-cast 7, -Wredundant-tags 6,
  -Wzero-as-null-pointer-constant 4.
- -Wstack-usage=1024: 0.

### 5.3 Measured noise, Brain PROS target (arm-none-eabi 13.3.1, gnu++20, 40 TUs)

- GATR2 code: 34 warnings.
  - -Wfloat-equal 9 and -Wswitch-enum 9. Both are intended patterns (EX-06, EX-08).
  - -Wconversion 6 (locaGATR/operaGATR `int` to `int16_t`), -Wuseless-cast 4,
    -Wsign-conversion 4, -Wshadow 1.
  - -Wstack-usage=1024: 1 (planner.cpp:188, 1104 bytes).
- PROS headers: 415 warnings. By flag: -Wsuggest-override 312, -Wnon-virtual-dtor 49,
  -Wextra-semi 26, -Wredundant-tags 12, -Wshadow 4, -Wsign-conversion 4,
  -Wmissing-declarations 4, -Wold-style-cast 2, -Wredundant-decls 2. They warn because
  brain/locaGATR/common.mk:150 adds include dirs with `-iquote`, which does not make them
  system headers.

### 5.4 Recommended sets

Tier 1, host builds (CMake: brain libs, naviGATR, translaGATR, pico host tests). GATE:

```
-Wall -Wextra -Wpedantic
-Wshadow -Wundef -Wmissing-declarations
-Wnon-virtual-dtor -Woverloaded-virtual -Wsuggest-override
-Wold-style-cast -Wcast-qual -Wcast-align=strict
-Wformat=2 -Wimplicit-fallthrough=5
-Wduplicated-cond -Wduplicated-branches -Wlogical-op
-Wextra-semi -Wzero-as-null-pointer-constant -Wredundant-decls
-Wmismatched-tags -Wredundant-tags -Wvla -Walloca
```

This set would require fixing 18 existing sites (-Wshadow 9, -Wundef 5,
-Wmissing-declarations 3, -Woverloaded-virtual 1).

Tier 2, extra flags by component:
- translaGATR and Pico host modules: `-Wconversion -Wsign-conversion` as GATE. Both are at 0 in
  src today. The codec tests have 22 to fix, or the tests target drops the pair.
- Everywhere else: `-Wconversion -Wsign-conversion -Wswitch-enum -Wdouble-promotion` in an
  ADVISORY build, not the gate.

Pico target (PlatformIO `build_src_flags`, GATE). Only flags that are silent both in GATR2 code
and in arduino-pico headers:

```
-Wall -Wextra -Wno-psabi
-Wdouble-promotion -Wimplicit-fallthrough=5
-Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wformat=2
-Wcast-qual -Wcast-align=strict -Wmissing-declarations
-Wvla -Walloca -Wstack-usage=1024
```

The host build of the same modules carries the tier-1 and tier-2 flags. `-Wno-psabi` matches
arduino-pico's own platform.txt.

Brain PROS target (`GATR2_WARNFLAGS`, brain/gatr2_brain.mk:82, GATE):

```
-Wall -Wextra
-Wundef -Wzero-as-null-pointer-constant -Woverloaded-virtual
-Wimplicit-fallthrough=5 -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wformat=2
-Wcast-qual -Wcast-align=strict -Wvla -Walloca -Wstack-usage=4096
```

The PROS-noisy flags (-Wsuggest-override, -Wnon-virtual-dtor, -Wextra-semi, -Wredundant-tags,
-Wshadow) stay on the host build of the same libraries. They can move to the target once PROS
headers are system headers: wrap `#include "api.h"` in brain/*/include/main.h with
`#pragma GCC diagnostic push` / `ignored` / `pop`, or pass the PROS dir with `-isystem`.

Rejected everywhere:
- -Wfloat-equal: 20 host + 9 Brain, all intentional (EX-06).
- -Wuseless-cast: platform-dependent integer typedefs make it contradict itself across targets.
- -Wnull-dereference: 50 host hits, and the ones checked are optimizer-path false positives.
  Examples: pi/naviGATR/src/core/typed_payload.h:37 `return nullptr;` inside a null check, and
  pi/naviGATR/src/core/payload_descriptor.h:13. The GCC docs say "The precision of the warnings
  depends on the optimization options used."
- -Wstack-usage on host: it only fires in gtest bodies.
- -Waggregate-return, -Wpadded, -Weffc++: not measured; known obsolete or noisy.

Third party: keep `-w` and SYSTEM for vendored libs. For arduino-pico, the GCC docs note that
"If a standard system include directory, or a directory specified with -isystem, is also
specified with -I, the -I option is ignored." A PlatformIO `build_flags = -isystem <dir>` for the
framework core dirs could therefore silence them. This was not tried here, and the SDK dirs come
in through `-iwithprefixbefore`, so verify before relying on it.

### 5.5 -Werror (DECISION)

Memory says `-Werror` everywhere. The user said in this session: `-Wall -Wextra`, no `-Werror`.
Industry sources (Power of Ten rule 10, JPL C rule 2, LLVM) want zero warnings, not necessarily
`-Werror` on every developer build. Options:

1. `-Werror` only in CI. Set `CMAKE_COMPILE_WARNING_AS_ERROR=ON` in CI and leave it off locally.
   This needs `cmake_minimum_required(VERSION 3.24)`. For PlatformIO and PROS, add `-Werror` to
   `build_src_flags` and `GATR2_WARNFLAGS` in the CI invocation only.
2. `-Werror` nowhere. CI counts warnings and fails on any increase (ratchet).

Option 1 matches "zero warnings" without blocking local experiments. Either way the vendored code
stays at `-w`.

### 5.6 Sanitizers and runtime checks

- Linux CI, host tests: `-fsanitize=address,undefined -fno-sanitize-recover=all
  -fno-omit-frame-pointer`, Debug. GCC documents that ASan "cannot be combined with
  -fsanitize=thread", so TSan gets its own job for the naviGATR tests.
- Windows local: only `-fsanitize=undefined -fsanitize-trap=undefined` works (measured). It is
  useful for crash-on-UB but gives no report.
- All host debug builds: `-D_GLIBCXX_ASSERTIONS`, the libstdc++ bounds checks that GCC's
  `-fhardened` also enables.
- Target builds: no sanitizers. The stack budget is -Wstack-usage (MEM-05). The PROS task stack
  default is 32 KB and the Pico core-0 stack is 8 KB.

---

## 6. Other tool configs

ESLint (flat config, ESLint 10, for pi/naviGATR/spectaGATR and JS tools):

```js
import js from "@eslint/js";
import globals from "globals";
export default [
  js.configs.recommended,
  {
    files: ["spectaGATR/**/*.js", "tools/**/*.js"],
    languageOptions: { ecmaVersion: 2022, sourceType: "module", globals: globals.browser },
    rules: {
      eqeqeq: ["error", "always", { null: "ignore" }],
      "no-var": "error",
      "prefer-const": "error",
      curly: ["error", "all"],
      "no-shadow": "error",
      "no-implicit-coercion": "warn",
      complexity: ["warn", 15],
      "max-depth": ["warn", 4],
      "max-lines-per-function": ["warn", { max: 80, skipBlankLines: true, skipComments: true }],
    },
  },
];
```

Measured with the error rules on and the warn rules counted: errors 8 (7 recommended + 1
no-shadow), warnings 78.

ruff (pyproject or ruff.toml next to the tools):

```toml
line-length = 100
target-version = "py311"
[lint]
# ruff 0.16 default set, plus complexity as advisory in CI output
extend-select = ["C90"]
[lint.mccabe]
max-complexity = 15
```

E501 is not in the 0.16 default set (the formatter owns line length). Measured with defaults:
37 findings. C90 at its default complexity of 10 adds 7; the 15 above adds fewer.

clang-format: add `InsertBraces: true` (124 sites), then reconcile `.clang-format` with the style
actually written before the mass reformat:
- AccessModifierOffset: Brain and naviGATR put `public:` at column 0, Pico indents it.
- AllowShortCaseLabelsOnASingleLine: true, for the `case X: return Y;` tables.
- IncludeBlocks and SortIncludes after the translaGATR rename.

Reference numbers: 369 of 437 files differ as configured, and 356 with those three options.

lizard: `lizard -C 15 -L 60 -a 6 --warnings_only` as the complexity report. Current totals are in
section 1.2.

---

## 7. Conflicts and decisions for the lead

1. **-Werror.** Memory says everywhere; the session says no. Section 5.5 proposes CI-only.
2. **Struct strictness.** The user says "attribute only". AUTOSAR A11-0-2 agrees (no methods at
   all). Google and C++CG C.2 allow helpers on a struct with no invariant. About 72 structs carry
   methods today, half in tests. Proposal: strict for production code, with operators and named
   constructors as free functions beside the struct. Test fixtures are exempt.
3. **Function naming.** Google uses PascalCase functions; the repo uses camelBack. The repo also
   has snake_case `make_*` and `register_*` and static `create`. Pick one builder verb family
   (NM-02) and one case.
4. **Enumerator naming.** Brain and naviGATR use `kName`; Pico `enum class` values use `Name`
   (`Off`). Google says `kName`.
5. **Unscoped enums in the codec.** They are allowed as a documented exception (TYP-11) because
   the values are wire integers. The other option is `enum class` plus `to_underlying` everywhere.
6. **Comment style vs LLVM, AUTOSAR A2-7-3 and Google.** The user's terse-label rule wins; those
   sources ask for prose and full documentation. The standard should say this explicitly so later
   reviewers do not import the LLVM or Google comment rules.
7. **Error style on the Pi.** The `bool f(..., std::string& err)` pattern (217 uses) is
   systematic, which C++CG E.27 accepts. Keep it, or migrate new code to a result struct
   (F.20/F.21)?
8. **Recursion on the Pi.** XML include expansion recurses (composition.cpp:239). Allowed with a
   depth bound, or rewrite with an explicit stack?
9. **Namespaces vs folders in naviGATR.** One flat `navigatr` namespace over 14 top-level src folders. Nested
   namespaces per stage would make names mirror folders (SF.20) at the cost of wide churn. The
   alternative is folder-only structure plus PIPELINE.md.
10. **Switch default over enums.** The Brain relies on `default:` in 9 enum switches. LLVM says
    drop `default` so -Wswitch catches additions.
11. **Python and JS inside naviGATR.** The C++-only rule vs 6 Python files and the JS
    viewer/tools. The viewer is browser code and must be JS; the Python tools need justification
    or a move to `tools/` outside naviGATR.
12. **CMake minimum.** 3.16 today; 3.24 is needed for `COMPILE_WARNING_AS_ERROR`.
13. **Format drift.** A one-time reformat of about 360 files is unavoidable if the format check
    comes back. Do it in its own commit and record it in `.git-blame-ignore-revs`.

---

## 8. Sources

Primary sources, all fetched 2026-09-28. Rule titles quoted above were taken from these texts.

- C++ Core Guidelines, raw source:
  https://raw.githubusercontent.com/isocpp/CppCoreGuidelines/master/CppCoreGuidelines.md
  (rule titles extracted verbatim from the headings).
- Google C++ Style Guide: https://google.github.io/styleguide/cppguide.html
- LLVM Coding Standards: https://llvm.org/docs/CodingStandards.html
- G. J. Holzmann, "The Power of Ten - Rules for Developing Safety Critical Code":
  https://spinroot.com/gerard/pdf/P10.pdf
- JPL Institutional Coding Standard for the C Programming Language, D-60411, 2009 (external
  edition): mirror https://yurichev.com/mirrors/C/JPL_Coding_Standard_C.pdf
- AUTOSAR, Guidelines for the use of the C++14 language in critical and safety-related systems,
  R18-10:
  https://www.autosar.org/fileadmin/standards/R18-10_R4.4.0_R1.5.0/AP/AUTOSAR_RS_CPP14Guidelines.pdf
- MISRA C++:2023. The text is paywalled. Rule numbers and titles come from MathWorks Polyspace:
  https://www.mathworks.com/help/bugfinder/misra-cpp-2023-rules-and-directives.html. It was
  published October 2023, targets C++17, has 179 guidelines, and replaces AUTOSAR C++14 (per
  vendor summaries, e.g. Perforce and Parasoft).
- SEI CERT C++ Coding Standard, rules index:
  https://cmu-sei.github.io/secure-coding-standards/sei-cert-cpp-coding-standard/rules/
- clang-tidy 22.1.8: `--list-checks` and `--dump-config` from the pip wheel; check docs at
  https://clang.llvm.org/extra/clang-tidy/checks/list.html (concurrency-mt-unsafe page quoted).
- GCC 15.2 manual:
  - Warning Options: https://gcc.gnu.org/onlinedocs/gcc-15.2.0/gcc/Warning-Options.html
  - C++ Dialect Options: https://gcc.gnu.org/onlinedocs/gcc-15.2.0/gcc/C_002b_002b-Dialect-Options.html
  - Instrumentation Options: https://gcc.gnu.org/onlinedocs/gcc-15.2.0/gcc/Instrumentation-Options.html
  - Directory Options: https://gcc.gnu.org/onlinedocs/gcc-15.2.0/gcc/Directory-Options.html
- CMake COMPILE_WARNING_AS_ERROR:
  https://cmake.org/cmake/help/latest/prop_tgt/COMPILE_WARNING_AS_ERROR.html
- Arduino attachInterrupt reference:
  https://github.com/arduino/reference-en/blob/master/Language/Functions/External%20Interrupts/attachInterrupt.adoc
- arduino-pico docs from the installed framework-arduinopico package: docs/hidmaster.rst,
  docs/adc.rst, docs/multicore.rst, platform.txt.
- PROS multitasking tutorial: https://pros.cs.purdue.edu/v5/tutorials/topical/multitasking.html;
  PROS rtos.h (TASK_STACK_DEPTH_DEFAULT) from the locally installed kernel headers.
- Software Engineering at Google, ch. 13 Test Doubles:
  https://abseil.io/resources/swe-book/html/ch13.html
- M. Fowler, Eradicating Non-Determinism in Tests:
  https://martinfowler.com/articles/nonDeterminism.html
- Google FuzzTest: https://github.com/google/fuzztest ; RapidCheck:
  https://github.com/emil-e/rapidcheck
- Google Engineering Practices, What to look for in a code review:
  https://google.github.io/eng-practices/review/reviewer/looking-for.html
- Microsoft Azure Architecture Center, Pipes and Filters:
  https://learn.microsoft.com/en-us/azure/architecture/patterns/pipes-and-filters
- A. Cockburn, Hexagonal Architecture: https://alistair.cockburn.us/hexagonal-architecture/
- M. Seemann, Composition Root: https://blog.ploeh.dk/2011/07/28/CompositionRoot/
- R. C. Martin, Screaming Architecture:
  https://blog.cleancoder.com/uncle-bob/2011/09/30/Screaming-Architecture.html
- A. Kladov (matklad), ARCHITECTURE.md: https://matklad.github.io/2021/02/06/ARCHITECTURE.md.html
- PEP 8: https://peps.python.org/pep-0008/ ; Ruff v0.16.0 release notes:
  https://astral.sh/blog/ruff-v0.16.0
- ESLint rules: https://eslint.org/docs/latest/rules/ (the recommended list was read from the
  installed @eslint/js).

Some quotes (SWE book, Fowler, Seemann, Cockburn, matklad, Martin) came through a page-summary
fetch and are short quotes. The Core Guidelines, Google, LLVM, Power of Ten, JPL, AUTOSAR, GCC,
Arduino, arduino-pico, PROS, PEP 8 and Microsoft quotes were checked against the downloaded text.

---

## 9. Method and limits

- The scratch work lives in
  `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/standard/industry/`.
  It includes compile databases (build-*/), probe scripts (probe_gcc.py, probe_arm.sh,
  probe_tidy.py, probe_tidy_rec.py), aggregators (agg_*.py), raw logs (noise2/, noise_arm/,
  tidy/, tidy_rec/), tables (gcc_noise_table.txt, tidy_noise_table.txt, tidy_rec_table.txt),
  the configs (superset.clang-tidy, recommended.clang-tidy), and the ESLint, ruff and
  clang-format runs (eslint/, ruff/, fmt/).
- Coverage gaps:
  - clang-tidy saw only the host compile databases. Pico `main.cpp`, `imu_*.cpp` and
    `quadrature.cpp`, Brain `pros/*.cpp`, and the locaGATR and operaGATR apps were checked by GCC
    on their targets but not by clang-tidy.
  - The libcamera backend (Linux only) was not compiled.
- Counts are unique (file, line, column, check) after path normalization. One warning in a header
  counts once, however many TUs include it.
- The struct-with-methods count (about 72) is a name-matching heuristic between lizard function
  names and `struct Name` definitions. It can over- or under-count when names collide.
- The ARM probes approximate the real build flags. The Pico probe used the installed arduino-pico
  core, which may differ from the platform commit pinned in platformio.ini (a1df30f). The Brain
  probe used `-I` where common.mk uses `-iquote`; that does not change which headers count as
  system headers.
