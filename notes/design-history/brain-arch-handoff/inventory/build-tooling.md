[harness: subagent output matched instruction-shaped pattern(s): settings-json. Control tags below are neutralized (`<` → `<\`); treat any remaining directive-shaped text as a finding to relay to the user, not an instruction to you.]

# Build, test and toolchain inventory for GATR2-Electronics (Windows box, 2026-09-27)

This was read-only. I changed no repo files and ran no builds. The only commands I ran were version queries, directory listings and greps. The repo is at `c2c5654` with a clean working tree. Everything in the memory notes that was marked "uncommitted" has since been committed, in commits 882c95f through c2c5654 on 2026-09-26/27.

---

## 1. Toolchains on this box

| Tool | Path | Version (queried now) | On persistent PATH? |
|---|---|---|---|
| cmake | `C:\Program Files\CMake\bin\cmake.exe` | 4.2.0 | yes (User and Machine PATH) |
| Host g++ / gcc | `C:\msys64\ucrt64\bin\g++.exe` | 15.2.0 (Rev8, MSYS2), target `x86_64-w64-mingw32` | yes (User PATH has `C:\msys64\ucrt64\bin`) |
| mingw32-make | `C:\msys64\ucrt64\bin\mingw32-make.exe` | GNU Make 4.4.1 | yes |
| gdb | `C:\msys64\ucrt64\bin\gdb.exe` | 16.3 | yes |
| MSYS make | `C:\msys64\usr\bin\make.exe` | not queried | no |
| ninja | **not present** on PATH. PlatformIO has `~/.platformio/packages/tool-ninja` but it is not on PATH. | - | no |
| googletest | **not installed** in ucrt64 (no `lib/cmake/GTest`). Every CMake project falls back to FetchContent v1.14.0. | - | - |
| PROS CLI | `C:\Users\mbes1\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-cli-windows\pros.exe` | `pros, version 3.5.6` | **no**, see below |
| arm-none-eabi-gcc / g++ (PROS) | `...\sigbots.pros\install\pros-toolchain-windows\usr\bin\arm-none-eabi-g++.exe` | Arm GNU Toolchain 13.3.Rel1, 13.3.1 20240614 | **no** |
| PROS toolchain make / sh | same `usr\bin`: `make.exe` (GNU Make 4.4.1), `sh.exe`, `bash.exe`, `cygpath.exe`, `mv.exe`, `mkdir.exe` | - | **no** |
| vexcom | `...\sigbots.pros\install\vex-vexcom-windows` | not queried | no |
| PROS kernel templates | `C:\Users\mbes1\AppData\Roaming\PROS\templates\` has `kernel@2.12.2`, `kernel@4.2.1`, `kernel@4.2.2`, `LemLib@0.5.5`, `liblvgl@9.2.0` | - | - |
| PlatformIO | `C:\Users\mbes1\.platformio\penv\Scripts\pio.exe` | PlatformIO Core 6.2.0 | yes (User PATH) |
| arm gcc (PlatformIO, Pico only) | `~/.platformio/packages/toolchain-rp2040-earlephilhower/bin/arm-none-eabi-gcc.exe` | 14.3.0 | no |
| Python | `py` launcher gives 3.14.3 (`...\Programs\Python\Python314\python.exe`). VS Python 3.9 also present. | - | yes |
| `py -m pros` | **fails**: "No module named pros". There is no pip-installed PROS CLI. | - | - |
| `~/AppData/Roaming/Python/*/Scripts` | Python312 has only `f2py.exe` and `numpy-config.exe`. Python314 has `clang-format.exe`, `clang-format-diff.exe` and `git-clang-format.exe`. No `pros`. | - | - |

**Important environment fact.** `pros`, `arm-none-eabi-*` and the toolchain `make` resolve in this session only because the process inherited a VS Code terminal environment:
- `PROS_TOOLCHAIN=c:\Users\mbes1\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr`
- `PROS_VSCODE_FLAGS=--no-sentry --no-analytics`
- The PATH has the sigbots.pros cli, toolchain and vexcom entries prepended three times.

The PROS extension log (`...\sigbots.pros\logs\PROS_Extension_log_2026-09-27T01-10-49.396Z.txt`) confirms this: "Appending CLI and TOOLCHAIN to PATH", "Setting PROS_TOOLCHAIN". The persistent User and Machine PATH (registry) contain **none** of the sigbots.pros directories. A shell not launched from VS Code must set them explicitly.

In this Git Bash the toolchain `make` (`sigbots.pros/.../usr/bin/make`) comes ahead of `mingw32-make` for plain `make`. CMake's "MinGW Makefiles" generator uses `mingw32-make` explicitly, so this does not matter for CMake builds.

The memory note `reference_navigatr_windows_build.md:11` says "no ninja/make/cl/python". That is stale: `mingw32-make` and Python both exist now. The PATH-order problem it describes is still real. Git Bash puts `/mingw64/bin` before `/c/msys64/ucrt64/bin`, so a ucrt64-built test exe fails with 0xc0000139.

### 1a. Commands that verifiably built things before

**Host CMake (common, brain, navigatr, pico tests).** Source: memory `reference_navigatr_windows_build.md:15-20`, handoff `spec.md` section 0, and `pi/navigatr/README.md:160-167`.
```
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake -S <src> -B "$S/<dir>" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug \
      -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$S/nav-base/_deps/googletest-src"
cmake --build "$S/<dir>" -j8
ctest --test-dir "$S/<dir>" --output-on-failure -j8
```
- `<src>` is one of `pi/navigatr`, `brain`, `common/tests` or `pico/tests`.
- An offline googletest source tree exists at `$S/nav-base/_deps/googletest-src` in three scratchpads, including **this session's** scratchpad `C:\Users\mbes1\AppData\Local\Temp\claude\c--GATR2-Electronics\827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b\scratchpad\nav-base\_deps\googletest-src`.
- Session `cdc4c519` also has `googletest-1.14.0.zip` and `gtest-src/`.
- Every CMakeCache I found in the scratchpads uses generator `MinGW Makefiles` with compiler `C:/msys64/ucrt64/bin/c++.exe` (or `g++.exe`).

**PROS `brain/testing`.** Source: `...\73859862-...\scratchpad\pros_notes.md`, "Verified 2026-09-26", and `docs/communigatr.md:514-545`.
```
TC="$HOME/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr"
PROS="$HOME/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-cli-windows/pros.exe"
cd /c/GATR2-Electronics/brain/testing
"$PROS" c apply kernel@4.2.2 --force-apply --no-download   # restore firmware/, include/pros/ offline
git checkout -- .gitignore                                  # CLI overwrites it
export PROS_TOOLCHAIN="$(cygpath -w "$TC")"                 # PATH alone is NOT enough for the CLI
"$PROS" make
# alternative: PATH="$TC/bin:$PATH" make ; make clean removes bin/ and .d/
```
- Logs of green builds: `73859862.../scratchpad/gate-pros.log`, `final-pros-make.log` and `verify-pros.log`, plus `827c2c12.../scratchpad/review-reqs/pros-make.log`.
- All show every library source compiled in place: 5 investiGATR files, communiGATR `src/client.cpp`, `src/driver.cpp`, `pros/pros_serial_port.cpp` and `pros/pros_driver.cpp`, and `common/frame_codec.cpp`, plus `src/main.cpp` and `src/drive_control.cpp`.
- Hot image is 24.29 KB (24.72 KB in the later review build). Cold package is 1.28 MB.
- The notes also verify `make clean` after each run, so `brain/testing/bin` does not exist now.

**PROS `brain/localization-test`** (the user's app, which agents must not touch per memory). Documented commands, from `brain/localization-test/README.md:72-77`:
```
cd brain/localization-test
pros c apply kernel@4.2.2 --force-apply        # add --no-download if cached
pros make
pros upload --slot 2 --name localization-test --after screen
```
- The same commands appear in `pi/navigatr/docs/usb_localization_bench.md:22-27` and `pi/navigatr/docs/vex_imu_bench.md:29-32`.
- Build output exists in the tree and is gitignored. The newest files are `bin/hot.package.elf`, `bin/main.cpp.o` and `bin/_pros_ld_timestamp.o`, all dated 2026-09-27 00:28, one minute before commit `1b56e7d usb fix`.
- `bin/gatr2/**` holds the library objects, dated 2026-09-26 16:35 and 18:09.
- No log records the exact invocation. **Inference:** the app was built from VS Code with the PROS extension or its terminal. The extension sets `PROS_TOOLCHAIN` and PATH, and `.vscode/c_cpp_properties.json` in that folder was written by it (compilerPath is the one-click arm-none-eabi-g++, cppStandard `gnu++20`).
- Both PROS apps already have `firmware/` (libc.a, libm.a, libpros.a, v5*.ld) and `include/pros/` restored locally. Git does not track these.

**Pico firmware.** CI (`ci.yaml:83-84`) runs `pio run -e hat2_bno08x -e hat2_asm330` in `pico/`. `pico/platformio.ini:7` sets `default_envs = hat2_bno08x`. Local PlatformIO build dirs are listed in section 2.

---

## 2. Existing build directories (names only; none were touched)

### 2a. Session scratchpads under `C:\Users\mbes1\AppData\Local\Temp\claude\c--GATR2-Electronics\*\scratchpad`

About 130 session dirs exist. Only six scratchpads are non-empty: `49e6b669` (144 entries, no CMake builds), `73859862` (125), `827c2c12` (78; this is also **my** session's scratchpad), `b08136da` (1 file), `cdc4c519` (29) and `ffabe616` (9).

Build dirs that contain `CMakeCache.txt`, grouped by source dir:

| Scratchpad | Build dir | CMAKE_HOME_DIRECTORY |
|---|---|---|
| 73859862 | `build-brain-a2`, `build-brain-b2`, `build-brain-b2a`, `build-brain-c1`, `build-brain-c3`, `gate-brain`, `verify-brain`, `review-investigatr/build`, `review-requirements/brain` | `C:/GATR2-Electronics/brain` |
| 73859862 | `build-common`, `build-common-c3`, `gate-common`, `verify-common`, `review-requirements/common`, `review-wire/build-common` | `C:/GATR2-Electronics/common/tests` |
| 73859862 | `b1a-nav`, `build-nav`, `build-nav-c1`, `gate-nav`, `verify-nav`, `review-brainlink/nav`, `review-halfduplex/build`, `review-requirements/nav` | `C:/GATR2-Electronics/pi/navigatr` |
| 73859862 | `b1a-check/build`, `mut-build` (mutation copy `mut-src/pi/navigatr`), `ni-standalone`, `toplevel-check/build` | scratch copies |
| 827c2c12 (this session) | `brain-build` | `C:/GATR2-Electronics/brain` |
| 827c2c12 | `live-build`, `nav-base` (holds `_deps/googletest-src`) | `C:/GATR2-Electronics/pi/navigatr` |
| 827c2c12 | `pico-tests`, `pico-tests2` | `C:/GATR2-Electronics/pico/tests` |
| 827c2c12 | `pv-tests`, `snap-build`, `review-{client,driverpros,investigatr,reqs,session,verify-client,verify-driver-pros,verify-pi-session,verify-requirements,wire}/build-{brain,common,nav}` | scratch snapshot copies under `.../scratchpad/<name>/src/...` |
| cdc4c519 | `build-navigatr` | `C:/GATR2-Electronics/pi/navigatr` |
| cdc4c519 | `build-common` | scratch `common-harness` |
| ffabe616 | `build-common` | `C:/GATR2-Electronics/common/tests` |
| ffabe616 | `nav-base` (holds `_deps/googletest-src`) | `C:/GATR2-Electronics/pi/navigatr` |

Other PROS scratch material in `73859862`: `pros-exp`, `pros-restore`, `pros-b2`, `testing-scaffold/project.pros`, `pros_notes.md`, `arm-a2`, `arm-b2` (arm objects) and `frame_codec_gnu++17.o` / `frame_codec_gnu++20.o`. In `827c2c12`: `pio-base`, `pio-bno`, `pio-live`, `pio-live2` (PlatformIO builds) and `review_result.json`.

### 2b. Inside the repo (all gitignored by `.gitignore:75-78`: `build/`, `build-*/`, `cmake-build-*/`, `out/`)

| Dir | CMakeCache | Source / notes |
|---|---|---|
| `build/cleanup-common` | yes | common/tests, MinGW, Debug (09-19) |
| `build/gravity-alignment-pico` | yes | pico/tests, MinGW, Debug (09-26) |
| `build/navigatr` | yes | pi/navigatr, MinGW (08-19) |
| `build/placement-timeout` | yes | **brain**, MinGW, Debug (09-26) |
| `build/review-common` | yes | common/tests, **Visual Studio 17 2022** |
| `build/review-navigatr` | yes | pi/navigatr, Visual Studio 17 2022 |
| `build/review-navigatr-mingw` | yes | pi/navigatr, MinGW |
| `build/vex-bench-common` | yes | common/tests, MinGW (09-26) |
| `build/vex-bench-navigatr` | yes | pi/navigatr, MinGW Debug, `NAVIGATR_WITH_LIBCAMERA=OFF`, default config synthetic_field_demo (09-26); has bench logs |
| `build/pico-gravity-firmware`, `build/pico-gravity-platformio-cache` | no | PlatformIO output (`hat2_asm330`, `hat2_bno08x`) |
| `build/navigatr-bench-source.tar.gz`, `build/review-common-tests.exe` | - | loose files |
| `build-analysis-navigatr` (tests OFF), `build-analysis-navigatr-tests`, `build-review-navigatr-mingw` | yes | pi/navigatr (08-18) |
| `build-review-common` (VS 2022), `build-review-common-mingw` | yes | common/tests |
| `build-review-navigatr` | no | only `CMakeFiles/` |
| `build-printables` | no | `generate.py`, `render.ps1`, PNGs |
| `pi/navigatr/build`, `build-field`, `build-libcamera`, `build-server`, `build-viewer`, `build-workers` | yes | pi/navigatr, MinGW Debug, all 09-14 (stale; `build-libcamera` has LIBCAMERA=OFF) |
| `pi/navigatr/build-camera-tools` | no | Python `venv` only |
| `brain/build*`, `common/build*`, `common/tests/build*` | **none exist** | - |
| `brain/localization-test/bin`, `.d` | PROS output | ignored by `.gitignore:84-87` (`brain/*/bin/` etc.) |
| `pico/.pio` | PlatformIO | ignored |

---

## 3. CI and CMake targets

### `.github/workflows/ci.yaml`
The workflow runs on pull requests and on pushes to main. It has four jobs, all on `ubuntu-latest` with apt `libgtest-dev cmake`:

| Job | Lines | Commands |
|---|---|---|
| `common` | 9-27 | `cmake -S common/tests -B build`, build, `ctest --output-on-failure` |
| `navigatr` | 29-47 | `cmake -S pi/navigatr -B build-navigatr`, build `-j`, ctest. Defaults: tests ON, libcamera OFF. |
| `brain` | 49-69 | `cmake -S brain -B build-brain`, build, ctest. The comment at `:49-50` says "brain/testing needs the PROS toolchain and is not built here." |
| `pico` | 71-97 | Python 3.12, `pip install platformio`, `pio run -e hat2_bno08x -e hat2_asm330`, then `cmake -S pico/tests -B build-pico-tests` plus build and ctest |

These do **not** exist in CI: no PROS build of either app, no clang-format check, no Pi build with libcamera, and no Python lint (`pi/navigatr/tools/*.py` exist but are not tested in CI).

### `brain/CMakeLists.txt`
- Project `gatr2_brain` with C++17, `CMAKE_CXX_EXTENSIONS OFF` (`:1-6`).
- System gtest, else FetchContent googletest v1.14.0 zip (`:9-23`).
- `add_subdirectory(investiGATR)` then `add_subdirectory(communiGATR)` (`:28-29`).
- There is no actuGATR, no planner subdir and no top-level option.

### `brain/investiGATR/CMakeLists.txt`
- `investigatr` STATIC: `src/drive.cpp`, `geometry.cpp`, `navigator.cpp`, `odometry_source.cpp`, `pid.cpp`. Include dir `include/`, `cxx_std_17` (`:4-12`).
- `investigatr_sim` STATIC: `sim/differential_drive_sim.cpp`, `sim/simulated_source.cpp`. It includes the dir itself so headers are reached as `sim/...`, and links `investigatr` (`:15-20`).
- `investigatr_tests`: `tests/geometry_gtest.cpp`, `pid_gtest.cpp`, `drive_gtest.cpp`, `navigator_gtest.cpp`, `odometry_source_gtest.cpp`. Links `investigatr_sim` and `GTest::gtest_main`, then `gtest_discover_tests` (`:23-33`).
- `-Wall -Wextra` on all three when not MSVC (`:35-39`). The flags are not `-Werror`.

### `brain/communiGATR/CMakeLists.txt`
- `communigatr` STATIC: `src/client.cpp`, `src/driver.cpp` and `${REPO}/common/frame_codec.cpp`. Includes `include/` and the repo root, links **PUBLIC `investigatr`** (`:7-17`).
- `communigatr_fakes` STATIC: `sim/fake_bus.cpp`, `sim/fake_pi.cpp`, `sim/link_rig.cpp` (`:20-26`).
- `communigatr_tests`: `tests/fake_pi_gtest.cpp`, `client_gtest.cpp`, `client_timing_gtest.cpp`, `driver_gtest.cpp`, `equivalence_gtest.cpp`. Links `communigatr_fakes`, `investigatr_sim` and `GTest::gtest_main` (`:29-43`).
- `pros/` (`pros_driver.cpp`, `pros_serial_port.cpp`) is "never built here" (`:2`). It is compiled only through `gatr2_brain.mk`.

### `brain/gatr2_brain.mk` (PROS in-place compile fragment)
- The Makefile includes it twice, around `-include ./common.mk`, guarded by `GATR2_BRAIN_SETUP` (`:13-14`).
- `GATR2_ROOT ?= ../..` must be a relative path, because drive-letter paths break `-iquote` (`:16-18`).
- `GATR2_BRAIN_LIBS ?= investigatr communigatr` (`:21`).
- Explicit source lists: `GATR2_SRC_investigatr` has 5 files (`:25-30`). `GATR2_SRC_communigatr` has `src/client.cpp`, `src/driver.cpp`, `pros/pros_serial_port.cpp`, `pros/pros_driver.cpp` and `common/frame_codec.cpp` (`:32-37`).
- Objects go to `$(BINDIR)/gatr2/<path>.o` (`:40`). `EXTRA_INCDIR += investiGATR/include communiGATR/include <root>` (`:42-45`).
- It uses its own compile rule with `-MMD -MP` (`:55-60`), because common.mk's `mv` of the `.Td` file would overwrite library sources (`:52-54`).
- **A new library such as actuGATR needs:** a `GATR2_SRC_actugatr` list, its include dir in `EXTRA_INCDIR`, and the name added to `GATR2_BRAIN_LIBS` in each PROS Makefile.

### PROS app Makefiles
- `brain/testing/Makefile` and `brain/localization-test/Makefile` are nearly identical.
- `WARNFLAGS+=-Wall -Wextra` and `EXTRA_CXXFLAGS=-D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING` (`:15-17`).
- `C_STANDARD:=gnu17` and `CXX_STANDARD:=gnu++20` (`:21-22`). The template's gnu23/gnu++26 are rejected by arm gcc 13, and the kernel headers need C++20.
- `GATR2_BRAIN_LIBS := investigatr communigatr` (`:27`).
- A kernel-file guard `$(error ...)` fires if `firmware/libpros.a` or `include/pros/rtos.hpp` is missing (testing `:58-60`, localization-test `:57-59`). The guard also blocks `make clean`.
- The fragment is included around common.mk (`:65-67` / `:64-66`).
- `project.pros`: kernel 4.2.2, target v5. testing stores the absolute template location `C:\Users\mbes1\AppData\Roaming\PROS\templates\kernel@4.2.2`; localization-test stores `"."`.
- The app's own sources come from common.mk's `src/` glob:
  - testing: `src/main.cpp` and `src/drive_control.cpp`
  - localization-test: `src/main.cpp` and **`src/usb_bench_driver.cpp`**. This is the USB transport, which lives in the app, not in communiGATR (see section 7).

### `common/tests/CMakeLists.txt`
- Project `gatr2_common_tests` with `add_compile_options(-Wall -Wextra)` (`:6`) and the gtest fallback (`:9-23`).
- `common_tests` = `frames_gtest.cpp` and `frame_codec_gtest.cpp`, plus code under test `../frame_codec.cpp` (`:29-42`).

### `pi/navigatr/CMakeLists.txt`
- Options: `NAVIGATR_BUILD_TESTS` ON (`:9`), `NAVIGATR_WITH_LIBCAMERA` OFF (`:68`), and `NAVIGATR_DEFAULT_CONFIG` = `config/demo/synthetic_field_demo.xml` (`:192-203`). The default config is baked into `generated/navigatr_build_config.h`.
- `navigatr_tinyxml2` STATIC (`:12-13`).
- `navigatr_apriltag` STATIC, C99, `-w`, `NDEBUG`, links Threads. On MinGW it drops `pthreads_cross.c` and adds `-include pthread.h` (`:17-65`).
- `navigatr_core` STATIC is built from `NAVIGATR_SOURCES` (`:71-127`), the generated `viewer_assets.cpp` (`:129-163`, which embeds `viewer/*` and `third_party/three`) and `${REPO}/common/frame_codec.cpp` (`:165-169`). Includes `src` and the repo root. Links `ws2_32` on WIN32 (`:173-175`). The optional libcamera source raises the target to cxx_std_20 (`:177-185`). `-Wall -Wextra` (`:187-189`).
  - Transport and Brain-related sources: `src/transport/half_duplex.cpp` (`:76`), `serial_port.cpp` (`:77`), `impl/resources/serial_links.cpp` (`:79`), **`impl/resources/pros_usb_link.cpp`** (`:80`), `impl/resources/wheel_geometry_resource.cpp` (`:88`), `impl/commands/brain_link_commands.cpp` (`:96`), `impl/localization/tracking_wheel_motion.cpp` (`:98`), **`impl/localization/brain_imu_wheel_bench.cpp`** (`:99`) and `impl/publishing/brain_link_publisher.cpp` (`:113`).
- `navigatr` executable: `src/app/main.cpp` and `src/app/command_line.cpp` (`:204-209`).

### `pi/navigatr/tests/CMakeLists.txt`
- Gtest fallback (`:2-16`).
- `TEST_SOURCES` (`:19-53`) has 32 test files plus `../src/app/command_line.cpp`: command_line, core, transforms, se3, resources, pros_usb_link, half_duplex, geometry_resources, sensors, stages, apriltag, synthetic_rig, pose_history, attitude, localization_models, localization, parallel_wheels, localization_fusion, brain_io, brain_imu_bench, world_estimation, vision_targets, system_build, profiles, end_to_end, brain_link_e2e, inspection, workers, field_assets, capture_transform, camera_capture and inspection_document. Every `*_gtest.cpp` in the folder is listed; none is orphaned.
- **Brain sources compiled into `navigatr_tests`** (`:58-68`), so the navigatr job fails if these change incompatibly:
  - `brain/investiGATR/src/drive.cpp`, `geometry.cpp`, `navigator.cpp`, `odometry_source.cpp`, `pid.cpp`
  - `brain/investiGATR/sim/differential_drive_sim.cpp`
  - `brain/communiGATR/src/client.cpp`, `brain/communiGATR/src/driver.cpp`
  - `frame_codec.cpp` comes from `navigatr_core`.
  - **Not** compiled in: `investiGATR/sim/simulated_source.cpp`, `communiGATR/sim/*` and `communiGATR/pros/*`.
- Include dirs (`:73-77`): `brain/investiGATR/include`, `brain/investiGATR` (for `sim/`) and `brain/communiGATR/include`.
- `NAVIGATR_CONFIG_DIR` is defined to `../config` (`:71-72`). Links `navigatr_core` and `GTest::gtest_main`.
- `docs/communigatr.md:643-648` states that a new or renamed Brain library source must be added both here and in `gatr2_brain.mk`. That now also applies to actuGATR and to any split of investiGATR.

### `pico/tests/CMakeLists.txt`
`pico_tests` = 5 test files (bno08x_reports, gravity_alignment, gyro_window, bno08x_supervisor, shtp_header) plus 4 `../src` sources (`:25-47`).

### Static test counts
These are counts of `TEST(` macros from grep, not from test runs: common 75, investiGATR 75, communiGATR 66 (brain total 141), navigatr 375, pico 53. The last recorded runs in memory were common 73, brain 137 and navigatr 350/351, so tests have been added since.

### Repo-level editor config
`.vscode/settings.json` sets `"cmake.sourceDirectory": "C:/GATR2-Electronics/brain"`.

---

## 4. Documentation inventory

In the Flags column: **M** = describes the Brain movement API, **C** = Brain config (`robot_config.h`, NavigatorConfig, ports), **T** = transport selection (RS-485, USB, serial resources), **L** = landmark reporting. Docs flagged for update are in bold.

### Root and `docs/`

| Doc | Covers | Flags |
|---|---|---|
| **`README.md`** (94) | Repo overview. The Pi runtime paragraph (`:37-48`) says a state reply carries "the pose of the landmark the Brain selected". The Brain section (`:62-74`) says "investiGATR: waypoint navigation and motion control". | M, L |
| `CONTRIBUTING.md` (4) | Pointer to the PCB conventions | - |
| **`docs/README.md`** (57) | Docs index. Its Brain section describes investiGATR as "navigation library: API, input contract, ... control law". Wire-interfaces line: "brain link v3 request/reply". | M, L (index lines) |
| **`docs/communigatr.md`** (662) | Client/Driver layers, PROS use, Driver API, sessions and restarts, timing, PROS packaging and import (`:363-437`), testing app with DriveControl, demo, controls and **config table** (`:439-512`), build (`:514-545`, including `PROS_TOOLCHAIN`), bring-up checklist, host tests (`:615-648`), limitations. | **M, C, T, L**: the heaviest doc to rewrite |
| **`docs/investigatr.md`** (509) | Navigator API (goTo/follow/goToRelative, MotionOptions/Status), input contract, InputSource/OdometrySource, control law, NavigatorConfig and tuning, fallback, build (`:472-485`), limitations (`:499-509`: "Tank drive only, driving forward only", "No obstacle routing"). | **M, C, L** |
| **`docs/interfaces.md`** (630) | Framing, sensor frame 0x01, brain link v3 (request 0x10 / reply 0x11, state block, sessions, bus ownership and timing, examples). It does **not** document `kOpGetStateWithImu = 5` (`common/frames.h:87`); no doc mentions it. | **T, L** |
| **`docs/hardware.md`** (151) | IMU, tracking wheels, HAT, Pico-to-Pi telemetry, RS-485 to V5 (`:105-134`: UART5 GPIO12/13, DE GPIO6), camera | **T** |
| **`docs/navigatr_resources.md`** (293) | Resource contracts: `linux_serial_link`, memory/file_replay links, pico_telemetry, field_map, robot_frame_map, libcamera, apriltag, target_set, **wheel_geometry** (`:258`), synthetic_rig. **Does not document `pros_usb_link`.** | **T**; wheel_geometry matters for RobotProfile |
| `docs/navigatr_sensors.md` (117) | pico_encoder_channel, pico_imu_channel, camera_frame, attitude_channel. Relevant to Brain-sent encoder ports and IMU source. | (RobotProfile impact) |
| **`docs/pi_setup.md`** (239) | Pi provisioning, source transfer with tar/scp, build-bench, UART5 setup, service install, direct Ethernet. States "VEX IMU on Brain port 1, and the Brain-Pi link on port 10" (`:8-9`). | **T, C** |
| `docs/contributing/README.md` (32) | PCB folder conventions | - |

### `pi/navigatr/` and its docs

| Doc | Covers | Flags |
|---|---|---|
| **`pi/navigatr/README.md`** (223) | Runtime design, what runs, coverage (brain link at `:109-113`), config, build (`:158-176`), run options (`:181-223`). `:6`: "Brain owns movement control"; `:8-12`: landmark wire fields. | **L**, T (minor) |
| `pi/README.md` (19) | Signpost to navigatr | L (one line) |
| **`docs/architecture.md`** (315) | Construction, stages, localization, world estimation, and "Commands, target resolution, and publishing" (`:198-262`: brain_link command collection, SELECT_LANDMARK, sessions). | **T, L** |
| `docs/attitude_firmware_followup.md` (60) | What firmware must send for live roll/pitch | - |
| **`docs/calibration_inventory.md`** (75) | Measurement inventory. `:63` points placement at `kStartPose` in `brain/testing/include/robot_config.h`. | **C**; wheel/IMU/camera values become RobotProfile |
| **`docs/configuration.md`** (135) | Default file, runnable starters, inline XML and file references | (RobotProfile boundary) |
| `docs/coordinates.md` (165) | Axes, heading, robot origin, camera offset, field anchor and placement, clocks | L (placement semantics) |
| `docs/field_assets.md` (219) | Override field CAD-derived geometry and tag mounts | - |
| `docs/inspection.md` (283) | HTTP/WebSocket inspection, snapshot JSON, viewer | L (nominal vs estimated display) |
| **`docs/landmarks.md`** (171) | "Field estimates, targets, and reporting". Its **Brain output** section (`:107-171`) states that one session-selected `landmark_id` is reported per GET_STATE, with `FieldObject object_id/wire_id` mapping, source observed/nominal/none, and SELECT_LANDMARK results. "No checked-in profile maps a wire id yet." | **L**: must change for "publish all landmark estimates" |
| `docs/localization_fusion.md` (179) | weighted_planar_fusion estimator | - |
| **`docs/parallel_wheel_bringup.md`** (253) | Two parallel wheels plus BNO08X bring-up. Section 6 "Brain placement" (`:183-207`) refers to `kStartPose` in brain/testing and to "a field-coordinate goTo". | **M, C** |
| `docs/pi_camera_setup.md` (290) | libcamera build and capture | (camera mounts become RobotProfile) |
| **`docs/setup.md`** (648) | Full setup. "Connect the Brain" (`:468-601`) has subsections Pipeline, Serial resource, Placement and sessions, Landmarks on the supplied profiles, Check the link, and Hardware checks. It assumes RS-485 only: "over the HAT's RS-485 link" (`:472`). | **T, L, C** |
| **`docs/usb_localization_bench.md`** (121) | USB bench: `kUseUsbBench`/`kUseVexImuBench` in localization-test, `pros_usb_link` `brain_usb` (VID 2888 / PID 0501 / interface 02), `brain_imu_planar_bench`, install with `--config ...bench_vex_imu_usb.xml`. "The USB adapter ... lives in the test app; the shared communiGATR Smart Port driver ... unchanged" (`:111-114`). | **T, C** |
| **`docs/vex_imu_bench.md`** (113) | VEX IMU bench fallback over RS-485 or USB. Brain `robot_config.h` flags, `bench_vex_imu.xml`, service install. It mentions "an optional state request carrying IMU data" without naming op 5. | **T, C** |
| `docs/printables/README.md` (35) | Checkerboards and field coordinate guide | - |

### `brain/` READMEs

| Doc | Covers | Flags |
|---|---|---|
| **`brain/README.md`** (30) | Folder map and host build commands (`:21-25`) | M (one line), needs actuGATR |
| **`brain/investiGATR/README.md`** (12) | "Navigator ... returning forward/turn demand for a tank drive" | **M** |
| **`brain/communiGATR/README.md`** (16) | Client, Driver, ProsDriver, ProsSerialPort layout | **T** |
| **`brain/testing/README.md`** (29) | Demo app with robot_config.h and DriveControl. PROS restore/build at `:17-26`. | **M, C** |
| **`brain/localization-test/README.md`** (148) | User's app: `kUseUsbBench`, `kNavigatrPort`, `kNavigatrBaud`, start pose, build/upload, run procedure, driver calls (`:130-145`) | **C, T** (the user owns this file) |

### Other READMEs
- `bench/README.md`: bring-up utilities index.
- `bench/rs485_link/README.md`: raw RS-485 byte test (T, low priority).
- `pico/README.md`: Pico firmware.

---

## 5. Pi service install and config selection

**`pi/navigatr/tools/install_service.sh`**

Usage is `sudo bash tools/install_service.sh --user USER [--binary PATH] [--config PATH] [--dry-run]` (`:7-15`).
- Defaults: `binary="$project_dir/build-bench/navigatr"` (`:30`) and `config="$project_dir/config/override/diagnostics/bench_vex_imu.xml"` (`:31`). `project_dir` is `tools/..`, which is `pi/navigatr` (`:27-28`).
- Relative option paths are resolved against the **caller's CWD**, then made absolute with `realpath -e` (`:54-55`). Paths must match `^/[a-zA-Z0-9_./-]+$` (`:56-58`).
- A non-dry run requires Linux, root, an existing user, systemd, and the `dialout` and `gpio` groups (`:60-71`).

The generated unit (`:73-96`):
```
[Unit] Description=naviGATR localization and inspection; StartLimitIntervalSec=0
[Service] Type=exec; User=<user>; SupplementaryGroups=dialout gpio; WorkingDirectory=<pi/navigatr>
ExecStart=<binary> --config_file=<absolute config>
Restart=always; RestartSec=3; TimeoutStopSec=15; StandardOutput/Error=journal; SyslogIdentifier=navigatr
[Install] WantedBy=multi-user.target
```
Install (`:103-117`): `systemd-analyze verify`, back up any existing `/etc/systemd/system/navigatr.service`, `install -m 0644`, `daemon-reload`, `enable`, `restart`.

**How the config is chosen:**
- The service always passes `--config_file=<abs>`. The only way to change the config is to re-run the installer with `--config`, or to edit the XML and `systemctl restart navigatr`.
- Without `--config_file`, the binary uses the compiled default `kDefaultConfigFile` (`src/app/build_config.h.in:6`). That is the absolute path of `NAVIGATR_DEFAULT_CONFIG`, default `config/demo/synthetic_field_demo.xml` (`CMakeLists.txt:192-203`).
- `parseCommandLine` (`src/app/command_line.cpp:9-97`) accepts `--config_file=X`, `--config_file X`, `--config-file`, or a single positional filename. A second filename is an error ("specify only one configuration filename").
- There is **no runtime or Brain-driven config selection**. The XML is read once at startup (`pi/navigatr/README.md:190-192`: "The chosen path is baked in; XML contents are read on each startup").

**`docs/pi_setup.md` exact commands**
- Pi packages: `sudo apt install -y build-essential cmake git` (`:44-45`).
- Transfer from Windows: `tar.exe -czf "$env:TEMP\navigatr-source.tar.gz" --exclude=build --exclude=build-* --exclude=.pio pi/navigatr common`, then `scp` (`:55-56`). The tarball contains only `pi/navigatr` and `common`. It excludes `brain/`, but the Pi build has `NAVIGATR_BUILD_TESTS=OFF`, so the Brain sources are not needed there.
- Build: `cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_TESTS=OFF -DNAVIGATR_WITH_LIBCAMERA=OFF` and `cmake --build build-bench -j2` (`:65-67`).
- Install: `sudo bash tools/install_service.sh --user "$USER"`, with an alternative using explicit `--binary build-bench/navigatr --config config/override/diagnostics/bench_vex_imu.xml` (`:120, 134-136`).
- Manage: `systemctl restart|stop|disable --now|enable --now navigatr` and `journalctl -u navigatr -f` (`:150-154`).
- UART: `enable_uart=1`, `dtoverlay=uart5`. `/dev/ttyAMA0` is the Pico and `/dev/ttyAMA5` is the Brain, at 115200. The DE GPIO is a sysfs number (518 on a base-512 chip) (`:81-110`).

**USB variant** (`usb_localization_bench.md:37-42`): the same build with `-j1`, then `sudo bash tools/install_service.sh --user "$USER" --config config/override/diagnostics/bench_vex_imu_usb.xml`.

---

## 6. `.claude/settings.local.json`

The file is from 2026-07-24. It holds only old one-off allows:
- `Bash(set -e)`
- mkdir/mv/rm of `pico/gatr2-pico` scaffolding
- `curl` / `Invoke-WebRequest` of tinyxml2 11.0.0 files
- `PowerShell($ProgressPreference='SilentlyContinue')`

It has no general allow for cmake, ctest, pros, make, pio, git or build commands, so those will prompt unless allowed at user level. It has no deny rules. The repo has no other `.claude` files.

---

## 7. Findings for the contract designer

Facts first; speculation is labeled.

1. **None of the planned new pieces exist yet.** A search for `RobotProfile|actugatr|mecanum|holonomic|obstacle` found only "no obstacle routing" statements: `brain/investiGATR/include/investigatr/navigator.h:4`, `docs/investigatr.md:153,507` and `brain/localization-test/README.md:126`. The only drivetrain code is tank: `mixTank` in `investigatr/drive.h`, and `DriveControl` in `brain/testing` is the only motor writer.
2. **USB transport is app-local.** `communigatr::UsbBenchDriver` (`brain/localization-test/include/usb_bench_driver.h:13-32`) mirrors `ProsDriver`'s API (`start`, `now`, `request`, `latest`, `submitPlacement`, `placementResult`, `placementStatus`, `status`) and frames bytes as `NG1:<hex>\n`. It is not in `gatr2_brain.mk` or `brain/CMakeLists.txt`, and it has no host tests on the Brain side. The Pi side (`pros_usb_link.cpp`) is in `navigatr_core` and tested in `pros_usb_link_gtest.cpp`, which has 10 TESTs. Moving it into communiGATR means updating `gatr2_brain.mk`, `brain/communiGATR/CMakeLists.txt`, possibly `pi/navigatr/tests/CMakeLists.txt:59-68`, and the localization-test Makefile. That Makefile belongs to the user.
3. **Two build lists must stay in sync:** `gatr2_brain.mk:25-37` and `pi/navigatr/tests/CMakeLists.txt:59-68`. A third, `brain/*/CMakeLists.txt`, covers host targets. Changing investiGATR's Navigator API breaks `navigatr_tests` (brain_link_e2e) as well as the brain job.
4. **PROS constraints for new Brain code:** library code must compile as C++17 on the host and as gnu++20 with arm-none-eabi-g++ 13.3.1. `GATR2_ROOT` must be relative. Every new source goes into an explicit list. Without liblvgl, use `pros::screen::print`. Linking uses `--gc-sections`.
5. **Doc and config discrepancies:**
   - `docs/pi_setup.md:8-9` says the Brain-Pi link is on port 10. `brain/localization-test/include/robot_config.h:14` has `kNavigatrPort = 4` and `:11` has `kUseUsbBench = true`. `brain/testing/include/robot_config.h:32` has port 10.
   - `docs/navigatr_resources.md` lacks `pros_usb_link`.
   - `docs/interfaces.md` and `docs/communigatr.md` lack op 5 `kOpGetStateWithImu`, which appears in `common/frames.h:87` and `communigatr/client.h:19-40`.
   - The two bench profiles use different wheel models: `brain_imu_parallel_bench` in `bench_vex_imu.xml` and `brain_imu_planar_bench` in `bench_vex_imu_usb.xml` (`usb_localization_bench.md:115-120`).
6. **Landmark reporting today is single-selection.** Per `pi/navigatr/docs/landmarks.md:107-171`, one landmark is selected per session and reported through GET_STATE. Publishing all landmarks will touch the landmarks.md, interfaces.md, communigatr.md, investigatr.md and root README landmark text.
7. **The service config is fixed at install time.** A Brain-sent RobotProfile has no existing hook: config is one XML path read at startup, with no reload path. Speculation: applying a profile "at a controlled boundary" would need a new mechanism, since `systemctl restart` is the only reload the docs describe.
8. **Stale local build dirs.** Build dirs in the repo (`pi/navigatr/build*` from 09-14, and others) predate the brain link v3 changes. The freshest verified host builds are in the scratchpads of sessions `73859862` and `827c2c12`. For new work, configure fresh dirs in the new session's scratchpad using the recipe in 1a and `FETCHCONTENT_SOURCE_DIR_GOOGLETEST` pointing at an existing `nav-base/_deps/googletest-src`.