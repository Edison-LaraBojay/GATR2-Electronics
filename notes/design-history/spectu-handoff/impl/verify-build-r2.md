# verify-build-r2: independent build verification, round 2

2026-09-29. Read-only on `S/wt-spectu` (detached at ae457ee), after fix-r1. Nothing edited, staged
or committed there; the only writes were the PROS kernel copies and build outputs, removed again.
`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
All builds fresh, in `S/sp-build/v1-r2-*`. No earlier log trusted; every number below is from a run
made here. The worktree had no file newer than the start of this run (last edit 17:07, run 17:29).

## Verdict

PASS. No blocking issue.

| # | Check | Result |
|---|---|---|
| 1 | Brain host tests | 536/536 pass. All 455 ae457ee names present and passing; 81 new. 0 warnings |
| 2 | naviGATR (wt-spectu/pi/naviGATR) | 626/626 pass, 0 warnings. Compiles wt-spectu communiGATR src |
| 3 | spectuGATR (HOST=ON, TESTS=ON) | 263/263 pass, 0 warnings, -Wall -Wextra on every target |
| 3b | spectuGATR/bench (extra) | 40/40 pass, 0 warnings |
| 4 | PROS locaGATR, operaGATR | rc 0, 0 warnings each; ae457ee baseline also 0, so none new. Cleaned |
| 5 | clang-format 22.1.8 | 179 of 181 added C++ files clean; 2 are PROS template copies. No modified file got worse |
| 6 | Non-ASCII | none in the 210 added files, none in `+` lines of the 27 modified files |
| 7 | Scope | nothing forbidden; 4 files outside the lists plus spectuGATR/bench, all logged deviations |

## Commands and results

Scripts: `S/sp-build/v1-r2-run.sh` (configure + build) and `S/sp-build/v1-r2-pros.sh`.

```
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake -S <src> -B S/sp-build/v1-r2-<name> -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug \
      -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src [extra]
cmake --build S/sp-build/v1-r2-<name> -j8
ctest --test-dir S/sp-build/v1-r2-<name> -j16 --output-on-failure
```

Baseline: `git -C S/wt-spectu archive ae457ee brain translaGATR .clang-format | tar -x -C
S/sp-build/v1-r2-base-src`. The user's tree is past ae457ee (30a2d50 plus edits), so the archive is
the exact 455 baseline.

| Build | Source | cfg/build rc | warnings | ctest |
|---|---|---|---|---|
| v1-r2-brain-base | v1-r2-base-src/brain (ae457ee) | 0/0 | 0 | 455/455 |
| v1-r2-brain | wt-spectu/brain | 0/0 | 0 | 536/536 |
| v1-r2-navi | wt-spectu/pi/naviGATR | 0/0 | 0 | 626/626 |
| v1-r2-spectu | wt-spectu/spectuGATR, `-DSPECTUGATR_BUILD_HOST=ON -DSPECTUGATR_BUILD_TESTS=ON` | 0/0 | 0 | 263/263 |
| v1-r2-bench | wt-spectu/spectuGATR/bench | 0/0 | 0 | 40/40 |

Test names:
- names extracted from ctest output (`Test #N: <name> ... <status>`), including the parameterized
  `Transports/...` names with spaces;
- base list (455) is identical to `S/final-brain.test.log`;
- `comm -23 base new-passed`: empty. Every ae457ee name passes;
- 81 new: Allocation 2, ClientObservers 3, Concurrency 2, DriveFeed 1, EventQueue 7, LinkFeed 6,
  Motion 5 (incl. the 3 D9 regressions CancelDuringReplanHaltClearsThePath,
  NewCommandBeforeItsPlanClearsTheOldPath, RefusedReplanClearsThePath), PathFeed 3,
  PublisherEncode 16, PublisherSchedule 19, Slot 5, UsbLane 3, UsbTransmit 9.

Flags: every non-third-party target's `flags.make` carries `-Wall -Wextra` (brain 15 targets,
spectuGATR 10, naviGATR 5; only naviGATR's vendored tinyxml2 and apriltag lack them, as at base).
naviGATR's `brain_link_e2e_tests` compiles `wt-spectu/brain/communiGATR/src/{client,doc_assembly,
link_driver,readiness,robot_profile,usb_line}.cpp`. `pi/` has no changes in the worktree, so the 626
test sources are ae457ee's.

Every one of the 46 added `*_gtest.cpp` is named in a CMakeLists.txt; no GLOB in any changed
CMakeLists.txt.

### PROS

```
cp -r /c/GATR2-Electronics/brain/<app>/firmware <tree>/brain/<app>/
cp -r /c/GATR2-Electronics/brain/<app>/include/pros <tree>/brain/<app>/include/
TC=/c/Users/mbes1/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr
export PROS_TOOLCHAIN="$(cygpath -w $TC)"; cd <tree>/brain/<app>; PATH="$TC/bin:$PATH" make -j16
```

| App | tree | rc | warnings | errors | hot.package.bin |
|---|---|---|---|---|---|
| locaGATR | ae457ee base | 0 | 0 | 0 | 46584 |
| operaGATR | ae457ee base | 0 | 0 | 0 | 78768 |
| locaGATR | wt-spectu | 0 | 0 | 0 | 48240 |
| operaGATR | wt-spectu | 0 | 0 | 0 | 80640 |

- Warning flags: app `WARNFLAGS+=-Wall -Wextra`, gatr2_brain.mk `GATR2_WARNFLAGS ?= -Wall -Wextra`.
  `grep -ci warn` on each log: 0.
- wt-spectu locaGATR compiled 20 spectuGATR units, operaGATR 22 (publisher, pros, protocol, the
  integrations); both compiled `communiGATR/src/usb_transmit.cpp`.
- Afterwards `make clean` in both wt-spectu apps (rc 0) and `rm -rf firmware include/pros`.
  `git status --porcelain --untracked-files=all --ignored` before and after: identical, 237 entries,
  nothing untracked or ignored.

### clang-format

pip `C:/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/clang-format.exe` 22.1.8, repo
`.clang-format`; `/c/msys64/ucrt64/bin/clang-format` does not exist.

- Added (181 .h/.cpp): `--style=file --dry-run --Werror`. Fails only
  `brain/spectuGATR/examples/standalone/include/{api.h,main.h}`, verbatim PROS template copies (W4
  logged exception).
- Modified (19 .h/.cpp), whole file, `warning:` count at HEAD (stdin, `--assume-filename`) vs now:
  none increased; motion.h 27 -> 26, pros_usb_port.cpp 15 -> 11, link_driver.cpp 18 -> 17, rest equal.
- Modified, hunks: `git diff HEAD -U0 | clang-format-diff -p1` flags hunks in motion.h (one
  alignment line), motion_gtest.cpp (RecordingSink), pros_usb_port.h and pros_link.cpp. In each
  the suggested output is inconsistent with the unformatted file around it (2-space access-block
  indent in a 4-space file, a re-wrapped initializer list), which is W6 note 18. Not blocking.
  Output in `S/sp-build/v1-r2-fmtdiff.txt`.

### Non-ASCII

`perl -ne 'print if /[^\t\n\x20-\x7E]/'` over all 210 added files (every type, .md included) and over
the `+` lines of `git diff HEAD --diff-filter=M -U0`: no hits (also no CR). A probe file with an em
dash is detected, so the scan works.

### Scope (IMPL_A.md ownership)

237 changed paths: W0 38, W3 26, W4 37, W5 52, W6 16, W6 hooks in communiGATR/actuGATR 21. Nothing
under pi/**, translaGATR/**, .github/**, the root README.md, docs/README.md or the root PIPELINE.md.
Outside the explicit lists, all logged:
- `brain/investiGATR/include/investigatr/path.h`: W6 deviation 1 (PathSink widened per 5.4);
- `brain/locaGATR/Makefile`, `brain/operaGATR/Makefile`: `GATR2_BRAIN_LIBS` names spectugatr (W6 log);
- `brain/spectuGATR/CMakeLists.txt`: W4 deviation 14;
- `spectuGATR/bench/**` (43 files): the PROPOSAL 7.1 bench kit, impl/bench-kit.md;
- the fix-r1 `communiGATR/{include/communigatr/usb_transmit.h,src/usb_transmit.cpp,
  tests/usb_transmit_gtest.cpp}` sit in the W6 communiGATR hook area (fix-r1 log item 7).

Classification list: `S/sp-build/v1-r2-scope.txt`.

## Notes (not blocking)

- The modified-file format hunks above stay in file style; a repo-wide format pass is a
  coding-standard question.
- The user's tree has moved past ae457ee in five Phase A files (see verify-build-r1); the apply
  step must re-run these builds on the merged result.
- Host validation only; nothing here says anything about Brain, controller or radio behavior.
