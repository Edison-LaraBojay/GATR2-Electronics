# verify-build-r1: independent build verification, round 1

2026-09-29. Read-only on `S/wt-spectu` (detached at ae457ee). Nothing edited, staged or committed there.
`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
All builds fresh, in `S/sp-build/v1-r1-*`. No earlier log trusted; every number below comes from a run made here.

## Verdict

PASS. No blocking issue. There are three notes: scope deviations that are logged, two vendored PROS headers that are not clang-clean, and overlap with upstream edits.

| # | Check | Result |
|---|---|---|
| 1 | Brain host tests | 527/527 pass. All 455 ae457ee names are present and pass. 72 new tests. 0 warnings (-Wall -Wextra) |
| 2 | naviGATR (wt-spectu/pi/naviGATR) | 626/626 pass. 0 warnings. It compiles the worktree's communiGATR sources |
| 3 | spectuGATR (HOST=ON, TESTS=ON) | 259/259 pass. 0 warnings. -Wall -Wextra on every target |
| 3b | spectuGATR/bench (extra) | 40/40 pass. 0 warnings |
| 4 | PROS locaGATR, operaGATR | both rc=0 with 0 warnings. The ae457ee baseline also has 0 warnings, so nothing new. `make clean` run afterwards; kernel copies removed |
| 5 | clang-format 22.1.8 | 176 of the 178 added files are clean. The 2 exceptions are the PROS template copies. The 19 modified files have no new whole-file violations |
| 6 | Non-ASCII | none in the added files, none in the added lines of modified files |
| 7 | Scope | nothing outside Phase A. Four out-of-list files, all logged deviations (see notes) |

## Commands and results

Helper scripts: `S/sp-build/v1-r1-run.sh` (configure, build, ctest) and `S/sp-build/v1-r1-pros.sh`.

```
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake -S <src> -B S/sp-build/v1-r1-<name> -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug \
      -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src [extra]
cmake --build S/sp-build/v1-r1-<name> -j16
ctest --test-dir S/sp-build/v1-r1-<name> -j16 --output-on-failure
```

Baseline at ae457ee: `git -C S/wt-spectu archive HEAD brain translaGATR .clang-format | tar -x -C S/sp-build/v1-r1-base-src`. This is exactly ae457ee. The user's tree is now at 30a2d50 and has uncommitted edits, so it is no longer the 455 baseline.

| Build | Source | Result |
|---|---|---|
| v1-r1-brain-base | v1-r1-base-src/brain (ae457ee) | 455/455, 0 warnings. Its name list is identical to `S/final-brain.test.log` |
| v1-r1-brain | wt-spectu/brain | 527/527, 0 warnings. `comm` against the base list finds no missing or failing base name |
| v1-r1-navi | wt-spectu/pi/naviGATR | 626/626, 0 warnings. `brain_link_e2e_tests` compiles wt-spectu/brain/communiGATR/src/{client,link_driver,...}.cpp |
| v1-r1-spectu | wt-spectu/spectuGATR, `-DSPECTUGATR_BUILD_HOST=ON -DSPECTUGATR_BUILD_TESTS=ON` | 259/259, 0 warnings |
| v1-r1-bench | wt-spectu/spectuGATR/bench | 40/40, 0 warnings |

The new brain tests include:
- the three D9 regressions: `Motion.CancelDuringReplanHaltClearsThePath`, `Motion.NewCommandBeforeItsPlanClearsTheOldPath` and `Motion.RefusedReplanClearsThePath`;
- `Motion.PathTeeFeedsBothSinks` and `Motion.PathReportsCarryTheirContext`;
- ClientObservers x3, UsbLane x3, LinkFeed x5, PathFeed x3 and DriveFeed x1;
- the publisher suites: Slot, EventQueue, PublisherEncode, PublisherSchedule, Allocation and Concurrency.

PROS. Kernel files were copied from `/c/GATR2-Electronics/brain/<app>/{firmware,include/pros}`, a read-only copy, into both wt-spectu apps and both base apps. The build:

```
TC=/c/Users/mbes1/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr
export PROS_TOOLCHAIN="$(cygpath -w $TC)"; cd <app>; PATH="$TC/bin:$PATH" make -j16
```

| App | rc | warnings | hot.package.bin |
|---|---|---|---|
| new locaGATR | 0 | 0 | 47712 |
| new operaGATR | 0 | 0 | 80120 |
| base locaGATR | 0 | 0 | 46584 |
| base operaGATR | 0 | 0 | 78768 |

- Both new apps compiled brain/spectuGATR/src and pros, spectuGATR/protocol/src, and (operaGATR) all three integrations.
- The warning flags are `-Wall -Wextra`: the app Makefile's WARNFLAGS and gatr2_brain.mk's GATR2_WARNFLAGS.
- The `kSpectuEnabled` gate is a runtime `if` on a constexpr bool, so the enabled branch is still compiled.
- Afterwards, `make clean` was run in all four. The copied firmware/ and include/pros were removed from wt-spectu. `git status` is identical before and after: 234 entries, no untracked or ignored leftovers.

clang-format (pip `clang-format.exe` 22.1.8; `/c/msys64/ucrt64/bin/clang-format` does not exist):
- **Added files:** `--dry-run --Werror` on all 178 added .h/.cpp. Only two fail: `brain/spectuGATR/examples/standalone/include/{api.h,main.h}`, which are verbatim PROS template copies. api.h is byte-identical to operaGATR's, and the apps' own api.h and main.h fail clang-format the same way. W4 logged this exception.
- **Modified files, hunks:** `git diff -U0 HEAD | clang-format-diff -p1` flags hunks in motion.h, motion_gtest.cpp (RecordingSink), pros_usb_port.h and pros_link.cpp. These are the places W6 (deviation 18) left in the file's own style, because a partial format would mix access-specifier indentation.
- **Modified files, whole file:** violation counts at HEAD compared with now: none increased; motion.h went 27 -> 26 and link_driver.cpp 18 -> 17.

Non-ASCII: `perl -ne 'print "$ARGV:$.\n" if /[^\x00-\x7F]/'` over every added file (all types, including .md), and over the `+` lines of `git diff HEAD` for the modified files. Nothing found. A probe file with an em dash was detected, which confirms the scan works.

## Scope (IMPL_A.md ownership)

Every changed path falls under W0, W3, W4, W5-core or W6. Nothing is under pi/**, translaGATR/**, .github/**, the root README, docs/README.md or the root PIPELINE.md. Four paths are outside the explicit lists:

- `brain/investiGATR/include/investigatr/path.h`. PathSink lives there, and PROPOSAL 5.4 widens it. This is W6 deviation 1.
- `brain/locaGATR/Makefile` and `brain/operaGATR/Makefile`. `GATR2_BRAIN_LIBS` has to name spectugatr for the apps to link it. W4 left this to the coordinator; it is W6 deviation 16.
- `brain/communiGATR/CMakeLists.txt`. It lists the new client_observers test (W6 deviation 16).
- `spectuGATR/bench/**` (43 files). This is the bench kit that PROPOSAL 7.1 assigns to Phase A ("W4 and W6 also prepare the B1-B13 programs"). It is logged in impl/bench-kit.md, and it is standalone: no other build compiles it.

`brain/spectuGATR/CMakeLists.txt` and `.../integrations/CMakeLists.txt` are W4's and W6's by implication.

## Notes for integration (not blocking)

- Five Phase A files also changed after ae457ee in the user's tree, either in commits ceeb1aa..30a2d50 or as uncommitted edits: drive_requests.h, operaGATR/src/main.cpp, robot/gatr2_robot.h, docs/actugatr.md and docs/brain_setup.md.
- A trial `git merge-file` merged all five with 0 conflicts. It used the ae457ee version as base, the user's working copy as ours and the wt-spectu version as theirs, all as scratch copies in `S/sp-build/v1-r1-merge`.
- That covers textual merging only. Whether the merged code still compiles and tests together is unchecked; the apply step must re-run the builds.
