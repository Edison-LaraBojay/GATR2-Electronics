---
name: feedback-conventions
description: "GATR2 coding/repo conventions — C++17 warnings-as-errors, no emojis/em-dashes, tests-next-to-code, README/CI rules"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 0693582d-d417-490c-8cb4-1a0c07d0b810
  modified: 2026-07-24T06:54:01.353Z
---

Conventions for GATR2-Electronics.

**Code:**
- C++17. `-Wall -Wextra -Werror` everywhere — warnings are errors from day one.
- clang-format enforced in CI; braces on EVERY if, no single-line unbraced conditionals.

**Namespace / brace style (decided 2026-07-19):** namespace opening brace goes on its OWN line (Allman for namespaces), contents UNINDENTED. Nested namespaces each get their own brace; closing brace labeled `// namespace X` for the namespace IT closes (innermost-first). Everything else (functions, if, enum, struct) uses ATTACHED braces (same line), like the existing frames.h. Closing namespace comments are auto-managed by clang-format `FixNamespaceComments` so they are never hand-maintained (this also auto-fixes label order).

**clang-format setup (2026-07-19):** `.clang-format` at repo root, `BasedOnStyle: LLVM` with overrides: IndentWidth 4, ColumnLimit 100, PointerAlignment Left, NamespaceIndentation None, FixNamespaceComments true, AlignConsecutiveAssignments/Declarations true, `BreakBeforeBraces: Custom` + `BraceWrapping.AfterNamespace: true`. Version PINNED to `clang-format==22.1.8` (local and CI must match; output varies by version). Install locally via `py -m pip install --user clang-format`; the exe is at `C:\Users\mbes1\AppData\Roaming\Python\Python3XX\Scripts\clang-format.exe` (NOTE: the pip wheel segfaults on Python 3.14 as of this date — could not run locally; CI is the verifier). CI job `format` uses actions/setup-python + `pip install clang-format==22.1.8` + `clang-format --dry-run --Werror` over common/brain/pico C++ files.

**Namespace NAME:** stays `gatr2` for the shared wire contract (user decided 2026-07-19 to keep it rather than move to `chomp`, treating a future open-source rename as a cheap mechanical change if Chomp is ever extracted). Brain lib is still `chomp::`.
- No emojis anywhere. No em dashes in documentation.
- Commit messages plain and descriptive. Branch per feature (name/topic), PR to main, review, merge. main protected, requires CI check. Small frequent PRs.
- CLARIFICATION (user, 2026-07-20): "small frequent PRs" applies once the framework/scaffolding is set up, NOT during initial bootstrap. While scaffolding, larger combined commits are fine since everything is interdependent. Do not push for commit splits during setup work.
- pip on Linux boxes needs `--break-system-packages` or a venv; use a venv with requirements.txt for pi/.

**Test file naming + CMake (decided 2026-07-19):** gtest source files are named `*_gtest.cpp` (e.g. `frames_gtest.cpp`), hold `TEST()` cases and no `main()` (link `GTest::gtest_main`). A `tests/` folder holds ONLY test mains — no headers, no shared helpers. Test files are listed EXPLICITLY in CMakeLists (`set(TEST_SOURCES ...)`), NOT globbed — user prefers a new test to be a deliberate line in the PR diff. Code under test is also listed explicitly (`set(CODE_UNDER_TEST ...)`). So adding a test = add its filename to CMakeLists. (Supersedes the context doc's globbed `test_*.cpp`.)

**Repo layout / files:**
- Tests live next to what they test, one `tests/` folder per top-level component, never nested deeper.
- Do NOT create empty placeholder source files. Directories carry a README stating intent; code files appear only when there is code.
- READMEs are short signposts (a few lines). Deep documentation goes in `docs/`. Do not bloat READMEs.

**CI:** GitHub Actions. Grows one job per component as content lands, in order: codec+tests (extend common job), clang-format check, Pico cross-compile (arm-none-eabi via PlatformIO), Python lint + cross-language agreement test, Chomp/PROS build. Build/format/tests are blocking; coverage is informational, never blocking.

**Language constraint (user, 2026-07-24):** navigatr (pi/navigatr) is C++ ONLY — "literally zero other languages"; any exception must be justified to the user. Accepted exceptions so far: CMakeLists (build glue) and XML files (configuration data the C++ parses). Do not add Python tooling/scripts inside navigatr.

**Dependency style (accepted 2026-07-24):** vendor small third-party C++ libs into `<component>/third_party/` (tinyxml2 11.0.0 lives at pi/navigatr/third_party, zlib license, built without -Werror, SYSTEM include). Offline builds beat FetchContent; system packages only for gtest in CI. navigatr tests fall back to FetchContent googletest v1.14 when system gtest is absent (find_package QUIET first).

**Open discrepancy (as of 2026-07-19):** CI git state was mid-relocation — good pipeline is untracked at `.github/workflows/ci.yaml` (cmake/ctest flow), while a stale older `.github/ci.yaml` was still staged (referenced a nonexistent `common/compile_check.cpp`). Note actual file is `ci.yaml` not `.yml`. Needs the git move finalized.

See [[feedback-design-principles]], [[project-wire-format]].
