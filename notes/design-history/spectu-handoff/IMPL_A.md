# spectuGATR implementation, Phase A (approved 2026-09-28: "approve with your recommended defaults")

Phase A covers everything that does not touch naviGATR:
- W0 protocol;
- W3 host platform layer;
- W4 Brain publisher;
- W6 Brain integrations, including the D9 actuGATR fix;
- the receiver core library (decode, combine, sources, capture log), without the HTTP server or main.

Phase B (later, after the naviGATR restructure lands) does:
- W1 viewer move, W2 server extraction, W7 naviGATR adapter;
- receiver main and serving, W8 viewer multi-source, W9 packaging;
- then validation and the final audit.

## Authoritative design
- `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/spectu-handoff/PROPOSAL.md`, approved with every section-8 recommendation and default. Implement what it says:
  - section 5 (API 5.1, units 5.2, publisher internals 5.3, integrations 5.4, wire contract 5.5, receiver rules 5.6, capture 5.8);
  - section 7.1 (W0, W3, W4, W5-core, W6);
  - section 7.2 (validation).
- Where the proposal is a sketch, keep its intent and choose the simplest readable implementation.
- Record every deviation in your log with its reason.
- The discovery findings behind it are in `spectu-handoff/discovery/*.md`.

## Where
- **Worktree:** `S/wt-spectu` (detached HEAD at ae457ee). Edit ONLY there.
  - `S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
  - NEVER modify `C:/GATR2-Electronics` (the user's tree, used by other agents as a baseline) or `S/wt-navi` (the restructure).
- **git:** `git add` (index only) is allowed in the worktree. NEVER commit, push, stash, reset, clean, or check out another ref.
- **Builds:** only in `S/sp-build/<your-key>/`.
- **Logs:** `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/spectu-handoff/impl/<your-key>.md`.

## File ownership in Phase A (a workstream edits only its files)
- **W0 protocol:**
  - `spectuGATR/protocol/**` (C++17, depends on nothing GATR; compiled by the Brain PROS build, the Brain host tests and the host receiver);
  - `spectuGATR/docs/PROTOCOL.md`, `spectuGATR/protocol/VIEW.md`;
  - `spectuGATR/PIPELINE.md` (the whole product, short);
  - `spectuGATR/CMakeLists.txt` (top level, options SPECTUGATR_BUILD_HOST, SPECTUGATR_BUILD_TESTS);
  - `spectuGATR/tests/protocol/**`;
  - `spectuGATR/README.md` (short signpost).
- **W3 platform:** `spectuGATR/host/platform/**` and `spectuGATR/tests/platform/**`. CMake lines only inside `spectuGATR/host/` (a `host/CMakeLists.txt` owned by W3, which later workstreams append to).
- **W4 publisher:**
  - `brain/spectuGATR/{include,src,pros,tests,examples}/**`, `brain/spectuGATR/PIPELINE.md`, `brain/spectuGATR/README.md`;
  - `brain/CMakeLists.txt` (add_subdirectory), `brain/gatr2_brain.mk` (a spectugatr entry compiling brain/spectuGATR/src and spectuGATR/protocol, as it compiles translaGATR today).
- **W5-core receiver:** `spectuGATR/host/{sources,decode,combine,capture}/**`, `spectuGATR/tests/host/**`, appending to `spectuGATR/host/CMakeLists.txt`.
  - Sources: brain_usb (direct user port) and controller (CDC2 USER_READ polling), both over the W3 serial API with fakes for tests.
  - The navigatr WebSocket source is NOT needed (D19 picks c2 by default); skip it and note that.
- **W6 integrations:**
  - `brain/spectuGATR/integrations/**`;
  - neutral hooks in `brain/communiGATR` (ClientObservers, AuxLineSource in ProsUsbPort) and `brain/actuGATR` (the widened PathSink tee, DriveObserver, the D9 path-clear fix with 3 regression tests);
  - `brain/locaGATR/src/main.cpp`, `brain/operaGATR/src/main.cpp`, `brain/robot/gatr2_robot.h` (config knobs);
  - their docs sections in `docs/communigatr.md`, `docs/actugatr.md`, `docs/brain_setup.md` (short additions only).
- **Not in Phase A:** pi/naviGATR/**, the root README.md, docs/README.md, PIPELINE.md at the repo root, .github/**, translaGATR/**.

## Rules
- **Behavior unchanged when spectuGATR is unused.** Hooks default to no observers.
  - Brain host tests: 455 at ae457ee, all must pass, plus new ones.
  - naviGATR tests (built from THIS worktree's pi/naviGATR, old layout, which compiles communiGATR sources): 626, all must pass.
  - Both PROS apps must build.
- **Publisher calls never block.** No I/O in producer calls; bounded memory, fixed after init; latest-wins slots; a bounded durable event FIFO; one producer task per slot, as in the proposal.
- **Code style** (the user's rules; read `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/memory/feedback_system_philosophy.md`, `feedback_comment_style.md`, `feedback_conventions.md`):
  - readability is king;
  - `using` aliases liberally; `struct` for data-only types, `class` for data plus behavior; free functions over classes made for their own sake;
  - everything namespaced (`spectugatr`, `spectugatr::host`, `spectugatr::host::platform`);
  - folder structure says what things are, and each component gets a concise PIPELINE.md;
  - comments are plain ASCII, terse, self-describing labels, not AI-ish;
  - tests are `*_gtest.cpp`, listed explicitly in CMake, never globbed; one tests/ folder per component;
  - clang-format 22.1.8 (`/c/msys64/ucrt64/bin/clang-format`) with the repo `.clang-format`, on files you touch;
  - `-Wall -Wextra`, zero new warnings.
- The draft coding standard at `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/coding-standard/CODING_STANDARD.md` is guidance where it exists and is consistent with the rules above.
- **Host validation only.** Never claim hardware behavior.

## Build recipes (Git Bash)
- **Host CMake:**
  - `export PATH=/c/msys64/ucrt64/bin:$PATH`
  - `cmake -S <src> -B S/sp-build/<key>-<what> -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src`
  - `cmake --build <dir> -j16`
  - `ctest --test-dir <dir> -j16 --output-on-failure`
  - Test exes need ucrt64 first on PATH (else they exit silently with 0xc0000139).
  - `<src>` is one of `S/wt-spectu/brain`, `S/wt-spectu/pi/naviGATR` (the old-layout copy in this worktree), or `S/wt-spectu/spectuGATR`.
- **PROS apps (locaGATR, operaGATR).**
  - First copy the untracked kernel files from the user's tree (read-only there): `cp -r /c/GATR2-Electronics/brain/<app>/firmware S/wt-spectu/brain/<app>/` and `cp -r /c/GATR2-Electronics/brain/<app>/include/pros S/wt-spectu/brain/<app>/include/`.
  - Then build:
    - `TC=/c/Users/mbes1/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr`
    - `cd S/wt-spectu/brain/<app>`
    - `export PROS_TOOLCHAIN="$(cygpath -w $TC)"`
    - `PATH="$TC/bin:$PATH" make -j16`
  - Check the output for warnings. Remove `bin/` and `.d/` afterwards with `make clean`.
