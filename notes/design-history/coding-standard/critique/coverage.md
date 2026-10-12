# Critique: coverage and correctness

Lens: completeness and correctness against industry practice and against the bug classes this
project actually hit (survey/complexity-bugs.md B1 to B17, section 3.7) and the industry survey
(survey/industry.md). Target: CODING_STANDARD.md draft 1. The draft is not edited here.

Each item: rule id, problem with evidence, and the exact fix. New rule ids take the next free
number in their area (0.2). Code is cited at `ae457ee` with git paths (`pi/naviGATR/...`; on disk
`pi/navigatr`). `(trimmed)` marks a shortened quote.

## How this was checked

- clang-tidy 22.1.8 (the survey's pip wheel, run read-only) on a copy of the draft's 9.2 config:
  `--verify-config` prints "No config errors detected", every check name exists, and the overlays
  inherit as described (`misc-no-recursion` is on under `brain/`; the tests overlay removes what it
  lists). The draft's 9.2 claim holds. What is wrong is what some rules say those checks do.
- Small scratch probes, in
  `scratchpad/standard/critic-coverage/` (`nest/n.cpp`, `nest/u.cpp`, `nest/g.cpp`, `wrap.cpp`,
  `sp.cpp`, `urv.py`): clang-tidy option semantics, a `wrapAngle` probe, a PROS toolchain probe,
  and one check run over the 119 host production TUs.
- Official docs fetched 2026-09-28: clang-tidy check pages (function-size, use-internal-linkage,
  unused-return-value, definitions-in-headers, convert-member-functions-to-static, unused-raii,
  build-using-namespace, macro-usage, macro-to-enum, use-anonymous-namespace,
  capturing-this-in-member-variable, special-member-functions, exception-escape, mt-unsafe,
  header-include-cycle), GCC 15.2 Warning Options and C++ Dialect Options, clang-format style
  options, CMake COMPILE_WARNING_AS_ERROR, Google C++ style guide source, CERT ERR51-CPP.
- Checked and correct, no change needed: the 9.2 check names; overlay inheritance; CMT-10's
  Google quote ("do not be unnecessarily verbose or state the completely obvious" is in the guide);
  RT-5's "no atomic reference counts" (verified, item 22); `-Wdangling-reference` is on in every
  build already (GCC 15.2 doc: "This warning is enabled by -Wextra.").

Priority: H = a gate or rule that is wrong, or a hit bug class with no rule; M = a real gap with
evidence; L = worth adding, low risk if skipped.

---

## A. Tool configuration that does not do what the rule says

1. **CPLX-3, 9.2: `NestingThreshold: 4` enforces the warn tier, not the fail tier (H).**
   - Problem: clang-tidy counts the function body as nesting level 1. In `nest/n.cpp`, a function
     with 4 nested control blocks (CPLX-3 "nest 4", which passes the fail tier `> 4`) is reported:
     `note: nesting level 5 starts here (threshold 4)`. Lambda bodies count as a level too, which
     matches the table's "control and lambda blocks". The table's "over fail today: 15" came from
     the survey's own heuristic (complexity-bugs.md 0: "my own heuristic ... Treat it as +-1"), not
     from the gate tool.
   - Fix: in 9.2 set `readability-function-size.NestingThreshold: 5`. Add under the CPLX table:
     "Nesting is measured by clang-tidy `readability-function-size`, which counts the function
     body as level 1, so `NestingThreshold: N + 1` enforces nesting > N. The warn tier is a report
     run with `NestingThreshold: 4`. Phase 0 recomputes the 'today' columns with the gate tools
     before the baseline is written."

2. **CPLX-2, CPLX-4, 9.2: `LineThreshold: 100` and `ParameterThreshold: 7` duplicate lizard with
   different meanings (M).**
   - Problem: `LineThreshold` counts physical lines; the tool prints "12 lines including whitespace
     and comments (threshold 10)" (`nest/n.cpp`). CPLX-2 is NLOC (lizard `-Tnloc=100`). One rule,
     two tools, two numbers: a 90 NLOC function with 15 comment lines passes lizard and fails
     clang-tidy.
   - Fix: delete `readability-function-size.LineThreshold` and `.ParameterThreshold` from 9.2.
     CPLX-2 and CPLX-4 Enforce lines say "lizard only (it also covers JS)". clang-tidy keeps
     `readability-function-size` for nesting only.

3. **TEST-15, 9.2 tests overlay: the overlay turns off the one check that enforces TEST-15 (H).**
   - Problem: the overlay removes `misc-use-internal-linkage`. TEST-15 names only
     `-Wmissing-declarations`, which GCC documents as functions only: "In C++, no warnings are
     issued for function templates, or for inline functions, or for functions in anonymous
     namespaces." It never sees a type, so it misses the ODR hazard of two test files in one binary
     each defining `struct X`. In 22.1.8 `misc-use-internal-linkage` has `AnalyzeTypes: true` by
     default, and the survey's own tidy logs show it reports exactly 5 test findings, all real:
     - `brain/communiGATR/tests/client_telemetry_gtest.cpp:524` `struct BusRun {`, `:538`
       `BusRun busRun(bool report, const FakeBusConfig& bus = {}) {`, `:587`
       `Seconds telemetryExchange(const FakeBusConfig& bus) {`;
     - `pi/naviGATR/tests/inspection_document_gtest.cpp:31` `struct JsonCheck {` and `:154`
       `bool validJson(const std::string& s) {`, which `pi/naviGATR/tests/brain_profile_gtest.cpp:1501`
       re-declares by hand: `bool validJson(const std::string& s);   // inspection_document_gtest.cpp`.
       That is a shared test helper smuggled across files, exactly what TEST-4 moves to
       `tests/support/`.
   - Fix: drop `-misc-use-internal-linkage` from the tests overlay. TEST-15 Enforce: "clang-tidy
     `misc-use-internal-linkage` (functions, variables and types); compiler `-Wmissing-declarations`."

4. **NS-3: the named check's default fix-it adds the `static` that NS-3 bans (M).**
   - Problem: `misc-use-internal-linkage` has `FixMode` default `UseStatic` ("Adds static keyword
     for internal linkage variables and functions"). Running the phase-1 fix-its would add
     namespace-scope `static`, which `misc-use-anonymous-namespace` (enabled by `misc-*`) then
     flags. Neither the option nor the second check is named in NS-3.
   - Fix: 9.2 CheckOptions add `misc-use-internal-linkage.FixMode: None`. NS-3 Enforce:
     "clang-tidy `misc-use-internal-linkage` (FixMode None), `misc-use-anonymous-namespace` (no
     namespace-scope `static`), `misc-anonymous-namespace-in-header`; compiler
     `-Wmissing-declarations`."

5. **ERR-3: `bugprone-unused-return-value` does not check project functions as configured, and it
   flags the rule's own discard idiom (H).**
   - Problem: the check's defaults are a fixed std/POSIX list (`CheckedFunctions`) and
     `CheckedReturnTypes: ^::std::error_code$;^::std::error_condition$;^::std::errc$;^::std::expected$;...`.
     `AllowCastToVoid` defaults to `false`, so `static_cast<void>(call()); // best effort` is still
     reported. The tree has 0 `[[nodiscard]]` today (grep over the headers of brain, translaGATR,
     naviGATR src and aggreGATR src).
   - Measured: with the options below, scratch `nest/u.cpp` flags a discarded `std::optional<int>`
     and a discarded `navigatr::FunctionStatus`, accepts `static_cast<void>(parse())`, and (as
     expected) does not see `bool open()`. Over the 119 host production TUs it reports 0, so it
     gates for free.
   - Fix, 9.2 CheckOptions:
     ```yaml
     bugprone-unused-return-value.AllowCastToVoid: true
     bugprone-unused-return-value.CheckedReturnTypes: '^::std::error_code$;^::std::error_condition$;^::std::errc$;^::std::optional$;^::navigatr::FunctionStatus$'
     ```
     ERR-3 text, replace the first sentence: "Status and result types carry `[[nodiscard]]` on the
     type (`enum class [[nodiscard]] FunctionStatus`, `struct [[nodiscard]] XyzOutput`), which
     covers every function returning them. A function that reports success as `bool` or
     `std::optional` carries `[[nodiscard]]` on its declaration. Each component's status types are
     listed in `bugprone-unused-return-value.CheckedReturnTypes`." Enforce: "compiler (a discarded
     `[[nodiscard]]` value warns by default); clang-tidy `bugprone-unused-return-value` with the
     9.2 options; review for `bool` returns."

6. **HDR-5, CFG-6: `misc-definitions-in-headers` cannot catch the long header functions the rules
   target (M).**
   - Problem: the check flags "non-extern non-inline function and variable definitions"; its doc
     lists inline functions, in-class member functions, constexpr functions and templates as not
     flagged. The draft's own Bad example is inline:
     `pi/naviGATR/src/runtime/inspection_config.h:73` `inline bool parseInspectionConfig(const ConfigNode& node, InspectionConfig& out,`.
     Same for `pi/naviGATR/src/capture/capture_config.h:89` `inline bool parseCaptureConfig(...`
     (trimmed).
   - Measured: the survey's lizard CSV has 20 production header functions over 15 NLOC (top:
     `parseInspectionConfig` 101, `parseCaptureConfig` 52, `quaternionFromRotation` 30,
     `estimateCaptureExposure` 30, `storePoll` 29, `WorkerStats::cycleDone` 26).
   - Fix: HDR-5 Enforce: "script: lizard over `*.h`; a non-template function body over 15 NLOC in
     a header is reported, baseline ratchet; clang-tidy `misc-definitions-in-headers` for the ODR
     part only." Add the row to 9.8 `check_headers.py`.

7. **TYPE-4: the named check pushes the opposite way (M).**
   - Problem: `readability-convert-member-functions-to-static` "Finds non-static member functions
     that can be made static". Applying it creates more classes that group static members, which
     TYPE-4 bans. No clang-tidy check detects an all-static class.
   - Fix: TYPE-4 Enforce: "script (a class or struct whose every member function is `static` and
     that has no non-static data member); review." Move `readability-convert-member-functions-to-static`
     out of TYPE-4 and into TYPE-3 as a hint: "a method that never uses `this` is a free-function
     candidate (advisory report)".

8. **CON-6: `bugprone-unused-raii` does not detect manual lock/unlock (L).**
   - Problem: it finds "temporaries that look like RAII objects" destroyed at once
     (`scoped_lock(&m);`). The Bad example is manual `take`/`give`
     (`brain/actuGATR/pros/pros_drive.cpp:28-34`), which it never sees.
   - Fix: CON-6 Enforce: "script (grep `\.take(`, `\.give(`, `\.lock()`, `\.unlock()` outside the
     one guard type per platform); clang-tidy `bugprone-unused-raii` for guards that die at once."

9. **OWN-5: `cppcoreguidelines-special-member-functions` does not flag reference-holding classes
   (L).**
   - Problem: it reports classes that define some but not all special members. A class with a
     reference member and none declared passes (check doc; C.21 is satisfied).
   - Fix: OWN-5 Enforce: "review; script (a class with a `&` data member or a device handle and no
     `= delete` copy constructor)". Remove the clang-tidy name from OWN-5.

10. **CONST-7: `cppcoreguidelines-macro-to-enum` rewrites macros into the enum kind ENUM-1
    forbids (M).**
    - Problem: `modernize-macro-to-enum` (the alias target) "replaces macros with an unscoped
      anonymous enum". ENUM-1 allows plain enums only for wire codes; CONST-7 itself asks for
      `constexpr`. The check that matches CONST-7 is `cppcoreguidelines-macro-usage`, which flags
      constant, function-like and variadic macros and has `AllowedRegexp`.
    - Fix: replace `cppcoreguidelines-macro-to-enum` in 9.2 Checks with
      `cppcoreguidelines-macro-usage`, and add
      `cppcoreguidelines-macro-usage.AllowedRegexp: '^(GATR2_|[A-Z]+_HAVE_|[A-Z]+_WITH_|NOMINMAX$|WIN32_LEAN_AND_MEAN$|FD_SETSIZE$|STBI?_|PROS_USE_)'`
      (see item 19 for why the platform names are needed). Measure before gating; the one
      stringify helper (`pi/naviGATR/src/impl/resources/libcamera_camera.cpp:38-39`
      `#define NAVIGATR_STRINGIFY_INNER(x) #x`) cannot be `constexpr` and takes an EXCEPTION tag.

11. **CONST-6: "add to 9.2 if wanted" but `google-explicit-constructor` is already in 9.2 (L).**
    - Fix: CONST-6 Enforce: "clang-tidy `google-explicit-constructor`; review."

12. **NS-4: `google-build-using-namespace` is not header-only (L).**
    - Problem: NS-4 says "on headers"; the check flags every using-directive. That is what NS-4
      wants for production code anyway.
    - Fix: NS-4 Enforce: "clang-tidy `google-build-using-namespace` (all production code; off in
      the tests overlay); script for headers."

13. **ERR-1: `bugprone-exception-escape` only examines some functions (M).**
    - Problem: it checks destructors, moves, `main`, swaps, `noexcept` functions and the functions
      named in `FunctionsThatShouldNotThrow`; it "does not flag every throw". The thread entry
      functions, where an escape calls `std::terminate`, are not covered unless named.
    - Fix: 9.2 CheckOptions
      `bugprone-exception-escape.FunctionsThatShouldNotThrow: 'estimationWorker,fieldWorker,captureLoop'`
      (the option takes plain names, so the HTTP and inspection thread bodies, both `Impl::run`,
      need unique names before they can be listed; see ERR-11 in item 37). ERR-1 Enforce: "script
      (grep `throw`); clang-tidy `bugprone-exception-escape` with the thread entries listed."
      Measure before gating.

14. **D-1, 9.1: "Needs `cmake_minimum_required(VERSION 3.24)`" is not the real requirement (L).**
    - Problem: COMPILE_WARNING_AS_ERROR was "Added in version 3.24"; there is no policy tied to
      `cmake_minimum_required`. The risk is the opposite: on a CMake older than 3.24 the variable is
      silently ignored and CI would pass warnings without failing.
    - Fix, D-1 and 9.1: "Needs CMake 3.24 or newer on every CI runner. Bump
      `cmake_minimum_required` to 3.24 so an older CMake fails loudly instead of ignoring the
      setting."

15. **9.1: two free warnings are missing from the gate (M).**
    - `-Wfloat-conversion`: warns on implicit real-to-integer and double-to-float narrowing (part of
      `-Wconversion`, usable alone). The survey's host logs (industry `noise2/`, built with
      `-Wconversion`) hold 47 unique `[-Wconversion]` sites in GATR2 code, all integer narrowing or
      integer-to-double, and 0 tagged `[-Wfloat-conversion]` (GCC's tag for real-to-integer and
      real-to-real narrowing). It gates B8 (units and rounding) at zero cost on host.
      Fix: add `-Wfloat-conversion` to the host gate; measure on the Pico and PROS targets before
      adding there.
    - `-Wdangling-reference`: already on through `-Wextra` (GCC 15.2) and it caught a W3 test bug
      (complexity-bugs.md B14). Fix: one line under 9.1: "Kept on purpose: `-Wdangling-reference`
      (in `-Wextra`); never add `-Wno-dangling-reference`."

16. **9.3, phase 1 step 2: `InsertBraces` carries an upstream warning the plan does not mention
    (L).**
    - Problem: clang-format's doc: "Setting this option to true could lead to incorrect code
      formatting due to clang-format's lack of complete semantic information. As such, extra care
      should be taken to review code changes made by this option."
    - Fix, 11.2 phase 1 step 2: "The `InsertBraces` change is its own commit per component, built
      and tested before the alignment reformat, and reviewed as code, not as format."

17. **9.2: `concurrency-mt-unsafe` runs with `FunctionSet: any` (L).**
    - Problem: the survey's MinGW run found 0 while `std::strerror` runs on the HTTP thread
      (industry.md CON-04, `pi/naviGATR/src/inspection/http_server.cpp:137`). The CI run is Linux.
    - Fix: 9.2 CheckOptions `concurrency-mt-unsafe.FunctionSet: posix`, and a note that the check
      only gates in the Linux job.

18. **NAM-14: the `g_` prefix is checkable, but the rule says review only (L).**
    - Measured (`nest/g.cpp`): with `readability-identifier-naming.GlobalVariablePrefix: g_` and
      `GlobalVariableCase: lower_case`, a mutable `int count` in an anonymous namespace is reported,
      `g_count` passes, and `constexpr int kLimit` is untouched (ConstexprVariable wins).
    - Fix: add both options to 9.2; NAM-14 Enforce: "clang-tidy naming (GlobalVariablePrefix);
      review for the owner comment."

---

## B. Rules that are wrong, or forbid code that is correct

19. **CONST-7: `#if defined(X)` defeats `-Wundef`; and the rule forbids required platform macros
    (H).**
    - Problem 1: GCC: `-Wundef` "Warn if an undefined identifier is evaluated in an #if directive."
      `defined(X)` does not evaluate X, so a misspelled `#if defined(GATR2_IMU_BN08X)` compiles the
      file empty with no warning. The robust industry form is a macro the build always defines to 0
      or 1, tested with `#if X`, so `-Wundef` catches both a typo and a missing definition. The
      Pico guard `pico/aggreGATR/src/config.h:13-14`
      `#if defined(GATR2_IMU_BNO08X) == defined(GATR2_IMU_ASM330)` / `#error "define exactly one of ..."`
      protects the definition, not each test of it (`imu_bno08x.cpp:12`, `imu_asm330.cpp:6`,
      `main.cpp:39`). naviGATR defines its macro only when on:
      `pi/naviGATR/CMakeLists.txt:195` `target_compile_definitions(navigatr_core PUBLIC NAVIGATR_HAVE_LIBCAMERA=1)`.
    - Problem 2: "No macros for constants or functions" forbids macros a platform or vendored
      header reads: `pi/naviGATR/src/inspection/http_server.cpp:18`
      `#define FD_SETSIZE 128   // listen socket plus up to 64 clients`, `:21`
      `#define WIN32_LEAN_AND_MEAN`, `:24` `#define NOMINMAX`,
      `pi/naviGATR/src/inspection/jpeg_encoder.cpp:11` `#define STB_IMAGE_WRITE_IMPLEMENTATION`,
      `brain/locaGATR/include/main.h:27` `#define PROS_USE_SIMPLE_NAMES`.
    - Fix, CONST-7 text: "No macros for constants or functions; use `constexpr`. Two kinds of macro
      are allowed. (1) Build-variant macros, `GATR2_SCREAMING_CASE` or `<COMPONENT>_HAVE_<DEP>`,
      always defined by the build to 0 or 1 (`NAVIGATR_HAVE_LIBCAMERA=$<BOOL:${NAVIGATR_WITH_LIBCAMERA}>`,
      `-DGATR2_IMU_BNO08X=1 -DGATR2_IMU_ASM330=0`), tested with `#if X`, never `#ifdef` or
      `defined(X)`, so `-Wundef` catches a typo. (2) Configuration macros a platform or vendored
      header reads, defined on the line before that include with a label." Enforce: "compiler
      `-Wundef` (gate on every target); clang-tidy `cppcoreguidelines-macro-usage` (item 10)."
      Bad example stays; the Good becomes the always-defined form.

20. **CON-5: forbids the deliberate lock-free ring the Brain USB port needs (H).**
    - Problem: CON-5 says "Atomics only for independent flags and counters, never to publish
      multi-field state." `brain/communiGATR/pros/pros_usb_port.cpp:32-59` is a single-producer
      single-consumer ring that publishes whole `UsbFrame` structs with acquire/release:
      `slots_[head % Capacity] = frame;` then `head_.store(head + 1, std::memory_order_release);`
      (`:40-41`). The header says why: "read and write only touch lock-free single-producer
      single-consumer queues" so "a blocked USB write (cable out) stalls only the transmit task"
      (`brain/communiGATR/include/communigatr/pros_usb_port.h:5-7`, trimmed). That is the right
      design where a PROS task may be deleted or blocked (RT-7, RT-8), and CON-5 makes it a
      violation. The draft also has no rule on explicit memory orders; this file is the only user.
    - Fix: CON-5, append: "Exception: a single-producer single-consumer ring, one reviewed type per
      platform, with its producer and consumer named in its thread tag and its own tests, where a
      lock cannot be used (an ISR, or a PROS task that can block or be deleted)." New
      **CON-11 (MUST)**: "Atomics use the default `memory_order_seq_cst`, except
      `memory_order_relaxed` for statistics counters and acquire/release inside the CON-5 ring
      type. Any other lock-free code needs an EXCEPTION tag." Why: C++CG CP.100 "Don't use
      lock-free programming unless you absolutely have to". Good:
      `brain/communiGATR/pros/pros_usb_port.cpp:61-63` `void bump(std::atomic<uint32_t>& counter) {`
      `counter.fetch_add(1, std::memory_order_relaxed);`.

21. **TYPE-15: forbids the fix for bug class B15 and two correct templates in the tree (H).**
    - Problem: "Templates only for containers, strong types and type-safe erasure." B15 (commit
      3af0e98) was JsonWriter overloads for `int`, `long` and `int64_t` colliding across ABIs; the
      fix is a constrained template:
      `pi/naviGATR/src/inspection/json_writer.h:85-90`
      `// Fixed-width integers may alias long or long long, depending on the ABI.` then
      `template <typename Integer, std::enable_if_t<std::is_integral_v<Integer> && ...> = 0>`
      `void value(Integer v) {` (trimmed). TYPE-15 as written bans it. It also bans
      `template <typename Build> std::string underOneReset(const System& system, Build build, ...)`
      (`pi/naviGATR/src/inspection/inspection_document.cpp:1054-1055`, trimmed), the B5 fix.
    - Fix, TYPE-15 first sentence: "Templates only for containers, strong types, type-safe erasure,
      a function that takes a callable (`underOneReset(system, build)`), and one constrained
      template that replaces an overload set over integer widths (NUM-5)."

22. **RT-5: the toolchain fact is true, is broader than stated, and is cited from a doc only (M).**
    - Verified: the PROS compiler is "Arm GNU Toolchain 13.3.Rel1 (Build arm-13.24) 13.3.1" and
      `-v` shows `--disable-threads`. Compiling `__gnu_cxx::__default_lock_policy` with the PROS
      flags (`sp.cpp`) gives 0, which is `_S_single`: `std::shared_ptr` reference counts are plain
      increments. The draft states it as fact on the strength of `docs/communigatr.md:208-209` (P-12).
    - Broader: with threads disabled, libstdc++ also does not guard the initialization of
      function-local statics (`__cxa_guard_*` has no gthreads), so a function-local static with a
      dynamic initializer reached from two tasks races. Today there is one candidate:
      `brain/operaGATR/src/main.cpp:241` `static actugatr::DriveSnapshot drive; // kept when a read is busy`,
      reached from `display()`. No race was shown.
    - Fix, RT-5, replace the parenthesis: "(the PROS toolchain is built with `--disable-threads`:
      `shared_ptr` counts are not atomic and function-local static initialization is not guarded;
      checked by compiling `__gnu_cxx::__default_lock_policy`, which is `_S_single`). No
      function-local static with a dynamic initializer is reachable from two tasks." Add the probe
      as a 3-line `static_assert` test in the PROS build so a toolchain change is noticed.

23. **ERR-6: bans `assert` on the Brain and Pico but never defines the "reported check" it
    requires (M).**
    - Problem: Power of Ten rule 5 and LLVM "Assert Liberally" want assertions; the draft rightly
      does not stop the robot (P-7), but "it uses `static_assert` and reported checks" gives no
      shape, so each author invents one.
    - Fix, ERR-6 append: "A reported check is `if (!cond) { ++faults_.<name>; return <status>; }`,
      with the counter surfaced per ERR-9. This is a named deviation from Power of Ten rule 5:
      the check degrades instead of halting."

---

## C. Bug classes that happened here and have no preventing rule

24. **New TYPE-16, B2: in-band sentinels outside the wire (H).**
    - Problem: TYPE-11 covers only `bool have_x` beside `x`; WIRE-6 covers only wire values. B2's
      live cases are neither:
      `pi/naviGATR/src/diagnostics/link_monitor.h:57-59` `int64_t last_rx_us = -1;` (and two more),
      `pi/naviGATR/src/capture/capture_service.cpp:619` `if (auto_last_us_ >= 0 && ...`,
      `pi/naviGATR/src/diagnostics/hub.cpp:58` `record.host_us = std::max<int64_t>(1, HostClock::nowUs());`
      (forces 1 so that 0 can mean unset), and the injected clock
      `pi/naviGATR/src/capture/capture_recorder.h:58` `std::function<int64_t()> wall_ms; // unix ms or -1; ...`.
      CMT-7 even fixes the comment form `// 0 = none`, which reads as permission.
    - Fix, new **TYPE-16 (MUST)**: "Outside the wire, absence is typed: `std::optional<T>`, an
      empty container, or a status enum. No in-band sentinel (0, -1, NaN, empty string, a clamped
      1) in a C++ field, parameter, return or local. On the wire a reserved value is a named
      constant (WIRE-6) and is converted to `std::optional` at the decoder." Why: B2. Scope: All
      C++. Enforce: review; clang-tidy `bugprone-unchecked-optional-access` (already on). Legacy:
      deferred, phase 3. CMT-7: change "sentinels as `// 0 = none`" to "wire sentinels as
      `// 0 = none` (WIRE-6)".

25. **New SYS-24, B5: labels read in a different moment from their data (M).**
    - Problem: W1 fix 4 (the reset label read after the snapshot) is only handled by one helper;
      SYS-15 says readers get a snapshot, but not that a document's labels come from the same one.
    - Fix, new **SYS-24 (MUST)**: "A document or message is built from one snapshot. Its labels
      (session, epoch, reset count, generation) are read from that same snapshot or under the same
      lock; when that is impossible, the builder reads the label before and after and rebuilds on a
      change." Good: `pi/naviGATR/src/inspection/inspection_document.cpp:1055`
      `std::string underOneReset(const System& system, Build build, bool* stable = nullptr) {`.
      Enforce: review; test pattern "reset under load" (item 47), as in
      `pi/naviGATR/tests/inspection_feed_gtest.cpp:1058`
      `TEST(InspectionFeed, ResetsUnderLoadNeverLabelAnOldPoseWithTheNewSession) {`.

26. **New SYS-25, survey 3.7: mechanisms that are correct only if every future field is
    annotated (M).**
    - Problem: the identical-diag skip needs every varying value marked (62 `varying` hits in
      `pi/naviGATR/src/inspection/inspection_document.cpp`; `:49`
      `// Times, ages and stamps are varying: a diag identity check leaves them out.`); a missed mark
      was the W1 fix 3 bug. Field-by-field equality is written twice for one wire struct:
      `pi/naviGATR/src/impl/commands/brain_link_commands.cpp:54` and
      `brain/communiGATR/sim/fake_pi.cpp:33`, both
      `bool sameTelemetry(const translagatr::BrainTelemetry& a, const translagatr::BrainTelemetry& b) {`.
      The survey proposed a rule (3.8 item 6); the draft dropped it.
    - Fix, new **SYS-25 (MUST)**: "No mechanism may depend on a per-field annotation or a
      hand-listed field set that a new field can silently miss. Prefer a design that is correct by
      construction (compare encoded bytes; hash the inputs, not the output minus exceptions). If a
      field list is unavoidable, a test or `static_assert` fails when the struct gains a field
      (for example `static_assert(sizeof(BrainTelemetry) == N, \"update sameTelemetry\")`)."
      Enforce: review; test. WIRE-2, append: "and every wire-level predicate (equality,
      well-formedness, repeatability) lives in translaGATR once; the Pi server and the Brain fake
      call it."

27. **New RT-14, B6: an actuator output with no age limit keeps its last command (H).**
    - Problem: B6 ("the motors kept their last voltage" after a deadlocked drive task). RT-6 and
      RT-7 prevent that one cause; nothing requires the general guard, which the code already has:
      `brain/actuGATR/include/actugatr/drive.h:2-5` "the motors stop at once for anything Drive
      cannot execute: ... or a command older than command_timeout" (trimmed), `:26`
      `kStale,             // older than command_timeout`, `:32`
      `Seconds         command_timeout = 0.1;`; and `pros_drive.h:9-11` "a manual demand goes
      stale and stops" (trimmed).
    - Fix, new **RT-14 (MUST)**: "Every actuator output has an age limit enforced by the task that
      owns the device, not by its callers: a command not refreshed within the limit is replaced by
      the device's safe output (stop, brake) and counted. The limit is a named config value. Test
      pattern 'source stops' applies to every actuator." Scope: Brain (and any future Pi or Pico
      actuator). Enforce: review; test.

28. **New CON-10, B11: logical deadlines checked only when input arrives (M).**
    - Problem: CON-4 covers blocking waits. B11's hit bug was a state waiting for input whose
      deadline was checked only when that input arrived (commit 4992248); W5 fix 1 was scheduler
      starvation. The fixed code states the right rule:
      `pi/naviGATR/src/impl/localization/motion_step.h:134-135` "Every motion window gets a deadline
      from the first call that sees it, checked on every call whether or not a heading is offered"
      (trimmed).
    - Fix, new **CON-10 (MUST)**: "A state that waits for an input or reply checks its deadline on
      every invocation, before it looks at input. Every scheduler or priority policy states its
      fairness bound (the longest any request class can wait) in its PIPELINE.md, and a test
      asserts that bound under the worst link timing." Enforce: review (REV-9); test patterns "no
      input at all", "slow link".

29. **New LOG-5, B9: counter and report labels that say something other than what they count
    (L).**
    - Problem: W0 fix 3 ("Snapshots refused" also counted replacements), W2 fix 8 (the seq-gap
      column overstated replacements). ERR-9 requires counters; nothing requires the name to be
      exact.
    - Fix, new **LOG-5 (MUST)**: "A counter counts one event kind and its name says which
      (`replaced`, `refused`, `expired`); two kinds are two counters. A report column or label
      names its counter exactly." Good: `brain/communiGATR/include/communigatr/pros_usb_port.h:22`
      `uint32_t expired       = 0; // queued frames older than the lifetime, never written`.

30. **New SYS-26, B15: platform branches that no build compiles, and vendor quirks nobody
    records (M).**
    - Problem: B15's hits were ABI integer aliasing, a vendor API that reports 0 or 1, a PROS call
      that fails right after enable, and Linux-only code never compiled on the dev box. 9.9 lists
      CI jobs but no rule says every branch must be built, and the good vendor labels in the tree
      are not a required form.
    - Fix, new **SYS-26 (MUST)**: "Every preprocessor branch and every platform-only file is
      compiled by at least one blocking CI job. A branch no job can build is listed under
      `## Exceptions` in its PIPELINE.md with its manual build recipe. A vendor or platform
      behavior the code relies on is a one-line label at the call." Good:
      `pico/aggreGATR/src/main.cpp:91` `// Serial1.availableForWrite() only reports 0 or 1 on this core.`
      and `brain/communiGATR/pros/pros_serial_port.cpp:28`
      `// Right after serial_enable the port may still refuse its settings.`
      The integer overload part is NUM-5 (item 34).

31. **New PIPE-10, B13: numbers in docs drift from the code (M).**
    - Problem: B13 (W5 fix 2: docs understated the poll delay). PIPE-3 checks names only. The
      draft's own Appendix B writes literal rates ("a sensor frame every 20 ms (50 Hz), UART 115200
      baud", "GET_STATE every 20 ms") and lists their constants only in a trailing "Sources" note.
    - Fix, new **PIPE-10 (MUST)**: "A number in a PIPELINE.md or reference doc names the constant
      or config attribute it comes from, in code font next to the number
      (`50 Hz, kSampleHz`). The doc check verifies the name exists; where the value is a literal
      initializer, it also verifies the value." Apply to Appendix B: move each source from the
      trailing note to its line.

32. **TIME-4 and new TIME-10, B1: the unset-domain hole, the bare subtraction, and clock
    resolution (M).**
    - Problem 1: TIME-4 fixes equality but not `pi/naviGATR/src/core/time.h:36-38`
      `return a.domain == b.domain || a.domain == ClockDomain::kUnset ||` /
      `b.domain == ClockDomain::kUnset;`, so an unset time (ms 0) compares and subtracts against
      either clock without tripping the check (survey 3.5); and `:41` `inline int64_t operator-(MonotonicTime a, MonotonicTime b) {`
      returns a unitless `int64_t`, against TIME-3.
    - Problem 2: B1's design-review finding "MonotonicTime counts whole ms, so the 1000 us and
      200 us guards could not be measured" has no rule; the fix moved deadlines to microseconds
      (`pi/naviGATR/src/resources/serial_link.h:39-48` TransmitWindow, per the survey).
    - Fix, TIME-4 append: "An unset time never compares or subtracts: `sameDomain` is true only for
      two equal set domains, and code tests `isSet()` first. The difference of two
      `MonotonicTime` is a duration type or alias with its unit in the name (`Millis`)."
      New **TIME-10 (MUST)**: "A clock that times an interval has a resolution at least ten times
      finer than the shortest interval it decides on; sub-millisecond deadlines use a microsecond
      clock."

33. **CON-9, B7: latest-wins that the OS buffer undoes (L).**
    - Problem: W1 fix 1: "Kernel and tunnel buffering bypassed latest-wins". CON-9 bounds our
      queues only. The fixed code is a good example:
      `pi/naviGATR/src/inspection/http_server.cpp:632-634` "A small kernel send buffer bounds what
      already-written frames can wait in the kernel; TCP_NOTSENT_LOWAT (Linux) bounds the unsent
      part" (trimmed); `pi/naviGATR/src/transport/serial_port.cpp:242` polls `TIOCOUTQ`.
    - Fix, CON-9 append: "A latest-wins stream over a socket or serial port bounds the OS buffer
      too (a small send buffer, `TCP_NOTSENT_LOWAT`, `TIOCOUTQ`, flow control), or it is not
      latest-wins."

---

## D. Industry rules that are missing

34. **New section 4.19 Numbers (NUM-), and UNIT-6 amended (H).** The draft has no rule on
    non-finite values, floating comparison, float-to-integer conversion or signedness (the words
    NaN and finite appear only in a naming list). Add to Contents: `NUM-`.
    - **NUM-1 (MUST)** "Every floating value that crosses a seam (stage output, wire encode, actuator
      command, config value) is checked finite; a non-finite value is a fault status, never passed
      on." Why: a NaN pose reaches the Brain as a plausible number. Good:
      `brain/actuGATR/include/actugatr/drive.h:4` "non-finite values" stop the motors;
      `brain/investiGATR/include/investigatr/geometry.h:45` `bool finite(const Pose& p);`. Gap: 0
      finite checks in translaGATR (grep).
    - **NUM-2 (MUST)** "A loop never iterates on a floating value; a numeric helper states and
      handles its domain for NaN and +-infinity." Evidence, a latent hang found by this review:
      `pi/naviGATR/src/math/angles.h:14-21` `inline double wrapAngle(double rad) {`
      `while (rad > kPi) {` `rad -= 2.0 * kPi;` ... Probe `wrap.cpp` against the real header:
      `wrapAngle(NaN)` returns nan; `wrapAngle(+inf)` never returns (killed by a 5 s timeout,
      exit 124). It is called on every localization step
      (`pi/naviGATR/src/impl/localization/weighted_planar_fusion.cpp:310`
      `r.odom_pose.heading_rad = wrapAngle(h + dtheta);`), so one infinite heading would stop the
      estimation worker (P-7). Reachability was not shown. The Brain copy uses `std::remainder` per
      the survey and cannot hang. Fix in phase 3 with UNIT-3's single wrap helper plus an edge test.
    - **NUM-3 (MUST)**, replaces UNIT-6's first sentence: "Floating to integer goes through one
      named helper per policy that defines NaN, +-infinity and out-of-range results (saturate or
      refuse, in its name). A raw `static_cast` from floating to integer is not allowed: out of
      range it is undefined behavior (CERT FLP34-C)." Good (refuse):
      `brain/communiGATR/src/link_driver.cpp:32-34` `bool toWire(double value, int32_t& out) {`
      ... `if (!std::isfinite(rounded) || rounded < INT32_MIN || rounded > INT32_MAX) {`
      (trimmed). Good (saturate): `brain/actuGATR/src/telemetry.cpp:17-18`
      `long saturate(double value, long lo, long hi) {` `if (!std::isfinite(value)) {`. Enforce:
      compiler `-Wfloat-conversion` (item 15); review.
    - **NUM-4 (SHOULD)** "Floating equality only against an exact sentinel or an exact assigned
      value (0 meaning off), through a named helper where it repeats; otherwise a named tolerance."
      Why: MISRA C++:2023 Dir 0.3.1; `-Wfloat-equal` is rejected in 9.1 because all 29 hits are
      such sentinels, e.g. `brain/actuGATR/src/pid.cpp:22` `if (gains_.kI != 0) {`, so the rule
      must say what is allowed instead.
    - **NUM-5 (MUST)** "No overload set that distinguishes `int`, `long`, `long long` or
      fixed-width aliases; use one constrained template or distinct names." Why: B15 (commit
      3af0e98). Good: `pi/naviGATR/src/inspection/json_writer.h:85-90` (item 21).
    - **NUM-6 (SHOULD)** "Signed types for arithmetic and differences; unsigned only for bits, wire
      fields, sizes from the standard library and wrapping clocks (TIME-6)." Why: C++CG ES.100 to
      ES.102. Enforce: `-Wsign-conversion` advisory (9.1 already).

35. **New OWN-9 and OWN-10: lifetime of stored callables and views (H).** OWN- covers ownership
    types but not the classic dangling cases the stage design invites (stored `std::function`
    executables, `const T&` view structs).
    - **OWN-9 (MUST)** "A lambda that outlives the current call (stored in a member, an executable,
      a callback, a thread or a task) has an explicit capture list and captures by value: copies or
      `std::shared_ptr`. `[&]` only for a lambda that dies before the call returns. `[this]` only
      when the owner joins or stops the callee before destruction, and the class deletes copy and
      move." Why: Google style "Use default capture by reference ([&]) only when the lifetime of
      the lambda is obviously shorter than any potential captures"; C++CG F.53. Good:
      `pi/naviGATR/src/impl/resources/pico_telemetry.cpp:847`
      `executable.execute = [telemetry, outputs](const ExecutionContext& context) {`;
      `brain/communiGATR/pros/pros_link.cpp:92` `client_(port_, [this] { return nonce(); }, ...)`
      (trimmed) with copy deleted at `pros_link.h:106-107`. Enforce: clang-tidy
      `bugprone-capturing-this-in-member-variable` (on through `bugprone-*`); review. The `-Wshadow`
      hits the survey found are in exactly these lambdas (`context` shadowing the build context,
      complexity-bugs.md 1.5).
    - **OWN-10 (MUST)** "A struct of references (stage inputs, `ConfigNode`) is a call-scoped view:
      built at the call, passed down, never stored in a member, container, lambda capture or return
      value. No object is used after `std::move`; no reference or `string_view` outlives its
      source." Enforce: clang-tidy `bugprone-use-after-move`, `bugprone-dangling-handle`,
      `bugprone-return-const-ref-from-parameter` (all on through `bugprone-*`); compiler
      `-Wdangling-reference`. OWN-7 becomes the config case of OWN-10.

36. **New TIME-11: wall clock (M).**
    - Problem: the Pi has no RTC; `system_clock` jumps when NTP syncs, which breaks any duration or
      ordering built on it. TIME-2 names one steady clock but does not say where wall time may
      appear. Today it is used correctly, for labels only:
      `pi/naviGATR/src/capture/capture_service.cpp:323-324`
      `return std::chrono::duration_cast<std::chrono::milliseconds>(`
      `std::chrono::system_clock::now().time_since_epoch())` behind an injectable `wall_ms`.
    - Fix, new **TIME-11 (MUST)**: "Wall-clock time (`system_clock`, unix ms, `gmtime`) is used
      only for labels and file names, never for durations, timeouts, freshness or ordering; it is
      injected like any clock (TIME-8)." Enforce: TIME-1 script (add `system_clock` to the list).

37. **ERR-1 amended and new ERR-11: exceptions from the standard library and at thread entries
    (M).**
    - Problem: ERR-1 names `std::thread` and `std::filesystem` but not the throwing calls project
      code writes directly (`std::stoi` family, `.at()`, `std::optional::value()`, `std::get`,
      `std::bad_alloc`). One is in production and safe only because of a guard on the line above:
      `pi/naviGATR/src/inspection/http_server.cpp:363`
      `out.push_back(static_cast<char>(std::stoi(s.substr(i + 1, 2), nullptr, 16)));` after
      `std::isxdigit` checks at `:361-362`. Neither the worker loops
      (`pi/naviGATR/src/runtime/system.cpp:1411` `void System::estimationWorker() {`, `:1436`) nor
      the other thread entries catch anything; CERT ERR51-CPP: "If the initial thread function
      exits because an exception is thrown, std::terminate() is called." The service restarts
      (`pi/naviGATR/tools/install_service.sh:87-88` `Restart=always` / `RestartSec=3`), so the cost
      is a 3 s outage and a lost placement, with no record of why.
    - Fix, ERR-1 append: "Project code calls the non-throwing forms: `std::from_chars` instead of
      `std::stoi`/`std::stod`, a checked index instead of `.at()`, `*opt` after a check instead of
      `.value()`, `std::get_if` instead of `std::get`." New **ERR-11 (MUST)**: "`main` and every
      thread or task entry on the Pi end in one catch-all that records a fault event with the thread
      name and exits nonzero, so the service restart is explained in the log." Enforce: clang-tidy
      `bugprone-exception-escape` with the entries listed (item 13).

38. **New SYS-27: explicit registration, no static-initialization side effects (M).**
    - Problem: the gripe is "where is the make pipeline". The one pattern that would make it
      invisible again is a self-registering plug-in (a static object whose constructor registers).
      The tree does not do it today and no rule stops it. Good:
      `pi/naviGATR/src/runtime/register_all.cpp:9-17` `void registerAll(FunctionRegistry& functions) {`
      `register_resources(functions);` ... `register_publishers(functions);` (trimmed).
    - Fix, new **SYS-27 (MUST)**: "Registration is an explicit call chain from `main`
      (`registerAll`, then one `register_<stage>` per stage). No static object with a constructor
      that registers, allocates or does I/O; no work before `main`." Why: C++CG I.22 "Avoid complex
      initialization of global objects". Enforce: review; clang-tidy
      `cppcoreguidelines-interfaces-global-init` (already on).

39. **P-9 amended and new RT-15: bound external input, connections and disk (M).**
    - Problem: P-9 lists "Queues, waits, retries, loops, memory and recursion". The HTTP server and
      the capture recorder already bound inputs, clients and disk, and no rule protects that:
      `pi/naviGATR/src/inspection/http_server.cpp:60-61`
      `constexpr std::size_t kMaxRequestHead   = 16 * 1024;` /
      `constexpr std::size_t kMaxRequestBody   = 64 * 1024;`, `:68`
      `constexpr std::size_t kMaxHttpConnections = 32;`; `pi/naviGATR/src/capture/capture_config.h:54-55`
      `long        max_mb            = 64;` / `long        keep              = 3;`. WIRE-8 covers
      only wire bytes.
    - Fix, P-9: "Queues, waits, retries, loops, memory, recursion, input sizes, open connections
      and files on disk each have a stated bound ...". New **RT-15 (MUST)**: "Every reader of bytes
      from outside the process (socket, serial port, file, XML, capture bundle) caps the size it
      accepts and rejects the rest with a status; every server caps its connections; every writer
      of files caps total size and count and states what it deletes." Enforce: review; test pattern
      "upstream sends garbage" extended to oversize input.

40. **New ALIAS-15: `auto` (M).** The draft rejects `modernize-use-auto` (9.2) but has no rule, and
    "a reader never has to ctrl+click to learn a local's type" is the user's readability goal.
    - Fix, new **ALIAS-15 (MUST)**: "`auto` only where the type is spelled on the same line
      (`std::make_unique<Foo>()`, a cast), for iterators, lambdas and structured bindings, or where
      the type is an unnamed template product. Everywhere else write the type or its alias." Why:
      Google style: "use type deduction only to make the code clearer or safer, and do not use it
      merely to avoid the inconvenience of writing an explicit type." Enforce: review.

41. **New API-8: blocking is part of a function's contract (M).**
    - Problem: CMT-4 lists "blocking and its bound" as something a comment may carry; RT-8 bans long
      blocking on control tasks, but a caller cannot know which calls block unless it is stated.
    - Fix, new **API-8 (MUST)**: "A public function that can block states its worst-case bound in a
      label (`// blocks up to 100 ms`); a class that never blocks says so once in its header. No
      label means it returns without waiting." Good:
      `brain/communiGATR/include/communigatr/client.h:5-6` "No clock and no threads: poll(now) does
      all I/O and never blocks" (trimmed, as cited in TIME-1).

42. **New WIRE-9: version mismatch (M).**
    - Problem: WIRE-3 handles unknown enum values and WIRE-7 bumps versions, but nothing says what a
      reader does with an older or newer peer. Practice exists: `docs/interfaces.md:969`
      "RESTART_ACQUISITION on the Pico (v2 firmware, else Failed with PicoLink)" (trimmed) and the
      test `perf_report_v1_refused` (`pi/naviGATR/tools/perf/CMakeLists.txt:66`).
    - Fix, new **WIRE-9 (MUST)**: "Each versioned contract states its compatibility rule in its
      reference doc: what a reader does with an older and with a newer version (refuse with a named
      status, or read the known part). Both directions are tested."

43. **New REPO-9: toolchains pinned in one place (M).**
    - Problem: B15 includes "The PROS template defaulted to gnu++26, which arm gcc 13.3 cannot
      build". Section 9 pins linters but not compilers. The installed ones: "Arm GNU Toolchain
      13.3.Rel1 (Build arm-13.24) 13.3.1" (PROS), arm-none-eabi 14.3.0 from
      `toolchain-rp2040-earlephilhower 5.140200` (Pico, per industry.md 1.1), MSYS2 GCC 15.2
      (host).
    - Fix, new **REPO-9 (MUST)**: "Every compiler and platform package (host GCC, the PROS
      toolchain and kernel template, the Pico toolchain package and platform commit) is pinned by
      exact version in one file, `docs/toolchains.md`, and CI installs exactly those. A version bump
      is its own commit with the full build and test run."

44. **RT-13 amended: per-function stack limits do not bound a call chain (L).**
    - Problem: `-Wstack-usage` is per function (GCC doc: "Warn if the stack usage of a function
      might exceed byte-size"). The Pico core-0 stack is 8 KB (industry.md MEM-05); eight 1 KB
      frames in one chain pass the rule and overflow.
    - Fix, RT-13 append: "(SHOULD) Target builds also emit `-fstack-usage` and
      `-fcallgraph-info=su`; a report sums the deepest chain from each task or loop entry and
      compares it with that task's stack."

45. **TEST-6 amended: a hung test must fail fast, and order must not matter (M).**
    - Problem: no `TIMEOUT` is set anywhere (every suite uses bare `gtest_discover_tests(<target>)`,
      e.g. `pi/naviGATR/tests/CMakeLists.txt:70` `gtest_discover_tests(navigatr_tests)`), so a
      hang (B11) holds CI for ctest's default 1500 s. Nothing detects order dependence between
      tests that share process state.
    - Fix, TEST-6 append: "Every discovered test has a timeout:
      `gtest_discover_tests(<target> PROPERTIES TIMEOUT 60)` (threaded suites may set more, with a
      label). CI runs each suite once with `--gtest_shuffle`."

46. **New TEST-19, B14: a harness checks the process it talks to (L).**
    - Problem: W0 fix 1, "The perf harness measured whatever process already held the port."
    - Fix, new **TEST-19 (MUST)**: "A test or tool that starts a server binds port 0 or a checked
      free port and confirms the peer's identity (instance id or pid) before it measures or
      asserts."

47. **TEST-10 table: add the patterns for B2, B5, B6, B10 and NUM (M).**
    - Problem: the table misses four patterns that match hit bugs: a field absent (B2, W2 fix 3:
      "A valid pose with age_ms null was drawn as live"), a busy caller (B6, survey check "hold the
      lock in a test and assert that callers return busy within the bound"), interleaved requests
      (B10: "a GET_STATE between SET_POSE retries made the retry new"), reset under load (B5).
    - Fix, add rows:
      | Pattern | Asserts | Bug class it prevents |
      |---|---|---|
      | field absent | each optional input field missing: the consumer shows or sends absent, never 0 or old | in-band sentinel (TYPE-16) |
      | busy caller | hold the owner's lock: every public call returns busy within its bound | blocked caller (CON-3, RT-6) |
      | interleaved requests | another request between a request and its retry: the retry still applies once | idempotency (WIRE-5) |
      | reset under load | resets race the reader: no document carries an old value with a new label | snapshot labels (SYS-24) |
      | non-finite input | NaN and +-inf at every floating input: a fault status, no hang, nothing sent | NUM-1, NUM-2 |

48. **TEST-17 amended: second implementations of a contract (M).**
    - Problem: TEST-17 covers the Python stub server only. `pi/naviGATR/tools/viewer/capture_fixture.py`
      is a second writer of `gatr2.capture/1` with hand-copied column lists; its docstring says it
      "proves nothing about the Pi's capture code" (`:12-13`, trimmed). The user's own CI list
      includes a "cross-language agreement test" (memory feedback_conventions.md, CI).
    - Fix, TEST-17 text: "A second implementation of a versioned contract in any language (a stub
      server, a fixture writer, a reader in the viewer) is checked in CI against output of the real
      producer: the C++ tests write a bundle and a document that the other side's tests load, and
      column and field lists are compared, not copied."

49. **New TEST-20: coverage, from the user's own rule (L).**
    - Problem: memory feedback_conventions.md (CI): "Build/format/tests are blocking; coverage is
      informational, never blocking." The draft does not mention coverage.
    - Fix, new **TEST-20 (SHOULD)**: "The Linux host test job reports line and branch coverage per
      component (gcov/gcovr). Coverage is informational and never blocks." Add to 9.9 as a
      non-blocking job.

50. **TEST-11 amended: fuzzing for the external-input decoders (L).**
    - Problem: TEST-11 uses seeded random inputs; industry practice for decoders of untrusted bytes
      is coverage-guided fuzzing (industry.md TST-05: FuzzTest, libFuzzer). D-13 already brings
      clang to CI.
    - Fix, TEST-11 append: "(SHOULD) A libFuzzer target per external decoder (frame reader, link
      documents, HTTP request head, config composition) runs for a fixed time in the Linux clang
      job; each crash becomes a known-byte test."

51. **0.3 amended: exceptions for compiler warnings and formatting (L).**
    - Problem: 0.3 has forms for code and clang-tidy only; `#pragma GCC diagnostic` and
      `// clang-format off` (9.3 needs it for matrix literals) have no tag form, so CI cannot count
      them.
    - Fix, 0.3 add two bullets: "For a compiler warning: `#pragma GCC diagnostic push` /
      `ignored "-W<flag>"` / `pop` around the smallest block, the push line carrying
      `// EXCEPTION(<RULE-ID>): <reason>`." and "For formatting: `// clang-format off` with the same
      tag, closed by `// clang-format on` in the same function."

52. **RT-3 amended: interrupt masking is per core (L).**
    - Problem: `noInterrupts()` masks only the calling core; the firmware is single-core today (no
      `setup1`/`loop1` in `pico/aggreGATR/src`), so RT-3 is correct now and silently wrong the day
      core 1 is used (SYS-10 already counts a second core).
    - Fix, RT-3 append: "This holds for single-core firmware. State shared with core 1 uses an
      RP2040 spin lock or the multicore FIFO, never `noInterrupts()` alone."

---

## E. Checklists

53. **0.8 and section 8: reference the new rules (M).** Add to the writing checklist: "Every
    floating value that crosses a seam is checked finite; no loop on a float (NUM-1, NUM-2)";
    "Stored lambdas capture by value; `[&]` only for call-local lambdas (OWN-9)"; "Absence is
    `std::optional`, not 0 or -1 (TYPE-16)"; "Every actuator command has an age limit (RT-14)". Add
    review rows:
    | ID | Question | Rules |
    |---|---|---|
    | REV-19 | Does anything depend on a field list a new field can miss? Are labels read with their data? | SYS-24, SYS-25 |
    | REV-20 | Every stored callable: what does it capture, and who outlives whom? | OWN-9, OWN-10 |
    | REV-21 | Every float at a seam: finite? Every float to int: through the named helper? | NUM-1 to NUM-3 |
    | REV-22 | Every platform branch: which CI job builds it? Every vendor quirk: labelled? | SYS-26 |
