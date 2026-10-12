# W4 stage 1 result: wire codec, Pico firmware, Pi link instrumentation, attitude, camera preview

Date 2026-09-28. Host builds and host tests only; nothing here was run on the Pico, the Pi, the Brain or the camera. Details, API notes and contract issues: progress/W4.md.

## Built

### translaGATR (codec, done first)
- `frame_codec.{h,cpp}`: TELEMETRY op 12 encode/decode (fixed 54-byte body, clear groups zero on the wire and ignored on decode, unknown flag bits kept, wheels over 6 refused), header-only replies, length tables; Pico `kPicoOpDiagnostics` (diag_hz body); `encodePicoDiag`/`decodePicoDiag` (0x14, 26-byte payload, 32-byte frame); FrameReader accepts 0x14 and fills `FrameReaderStats` (bytes, frames, sync_dropped, length_errors, check_errors; documented byte accounting); `decodeTelemetryBody` and `decodePicoDiagPayload` for raw hub records (used by W1 and W3).
- `frames.h`: comment fix only (`link_rx_bad` meaning).
- Tests: `tests/frame_codec_gtest.cpp` (+32: known-byte vectors from an independent Python packer `$S/w4-vectors/pack.py`, field offsets, lengths, corruption, capacity, reader stats cases), `tests/frames_gtest.cpp` (+1, updated registries).

### Pico firmware (pico/aggreGATR)
- `pico_commands.{h,cpp}`: DIAGNOSTICS (0..5 Hz, 6..255 BadBody), recorded and deduplicated like the other commands, `Effect::SetDiagnostics`.
- `pi_link.{h,cpp}`: `DiagSchedule`, the same idle windows as status frames (FIFO empty, airtime + 1 ms before the next tick), status first, never forced.
- `diag_report.{h,cpp}` (new): pin packing, counter wrap, error saturation, report age (0xFFFF none).
- `imu.h` + both drivers: `imu::counters()` from state the drivers already keep (no extra SPI traffic).
- `main.cpp`: pins read with `gpio_get` only (static_assert: every diag pin is in the pin budget), `link_rx_bad` from the command FrameReader stats, `ticks_skipped`, diag frames after status, USB link line extended.
- Tests: `diag_report_gtest.cpp` (new, 7), `pico_commands_gtest.cpp` (+5), `pi_link_gtest.cpp` (+7, including a simulation that status + 5 Hz diagnostics never delay a sensor frame). README updated.

### Pi (pi/naviGATR)
- `impl/resources/pico_telemetry.{h,cpp}`: decodes 0x14 (newest kept); `<Diagnostics hz="0..5"/>`; DIAGNOSTICS per Pico boot, again after a reboot, `firmware` on UnknownOp, `refused`, 2 s attempts retried every 5 s when unanswered; hub posts (status and diag always, sensor only when wanted); LinkMonitor fed from the existing reads and writes; `instrumentation()` view (diag state and frame, frame counts, encoder counts and change over about 1 s with rebase on restarts, serial reopen counters).
- `impl/resources/pros_usb_link.{h,cpp}`: dropped NG1 lines counted as Brain link rejections with reasons; console lines counted apart.
- `impl/commands/brain_link_commands.{h,cpp}`: TELEMETRY Ok header-only, resend of the newest id answered again, attitude to the BenchImu mailbox only, raw body posted; kVexImu, kPath, kBrainRequest posts; LinkMonitor at the decoded frame layer; op and result names.
- `impl/publishing/brain_link_publisher.{h,cpp}`: replies to the monitor as attempted and accepted bytes; completes the request record with the result and length actually sent (0 when refused or expired).
- `diagnostics/link_monitor.{h,cpp}`: staged request slot; quiet links' rates decay instead of freezing. `diagnostics/instrumentation.cpp`: the full section 7 object with honest pin and encoder wording.
- Attitude: `resources/brain_imu_bench.h` (attitude mailbox), `impl/localization/brain_imu_wheel_bench.{h,cpp}` (optional attitude output: gravity reference, no yaw, Pi arrival time), `runtime/brain_profile_builder.{h,cpp}` (VEX profiles fold `profile_attitude`, max_age_ms 250), `impl/localization/motion_step.cpp` (a configured AttitudeFold with nothing retained names its source). `RobotState.attitude` and the synthetic rig path are unchanged in structure; Pico IMU profiles carry no attitude (documented).
- Configs: `<Diagnostics hz="1"/>` in `brain_profile_usb.xml` and `brain_profile_rs485.xml`; new `config/override/brain_profile_usb_camera.xml` (the USB config plus a `libcamera_camera` 640x480 Y8 15 Hz and a `camera_frame` sensor, world estimation noop, preview budget 5 Hz / 640 px).
- Tests: brain_io_gtest (+9), pico_link_gtest (+11), brain_imu_bench_gtest (+3), brain_profile_gtest (+7, including a dead-camera and a live-camera fixture on the camera config: localization, placement and the Brain state block unchanged, no field correction, raw preview frames only), pros_usb_link_gtest (+1).
- Docs: `docs/interfaces.md` (0x14, DIAGNOSTICS, TELEMETRY, reader stats, vectors, compatibility), `pi/naviGATR/docs/pico_link.md` (Diagnostics, Instrumentation), `brain_profile.md` (attitude, camera config, Brain link instrumentation), `camera_preview.md` (new: packages, NAVIGATR_WITH_LIBCAMERA build, config selection, service commands, restart and reupload table, later calibrated-estimation steps, no CSI hot-plug promise).

## Test commands and results (host only)
Build dirs under `$S` (MinGW Makefiles, Debug, -Wall -Wextra, googletest from `$S/nav-base`), `/c/msys64/ucrt64/bin` first on PATH.
- translaGATR: `cmake -S translaGATR/tests -B $S/w4-transla ...`; `common_tests.exe`: 254/254 passed (221 before), 0 warnings.
- Pico host tests: `cmake -S pico/aggreGATR/tests -B $S/w4-pico ...`; `pico_tests.exe`: 129/129 passed (110 before), 0 warnings.
- PlatformIO: `pio run -e hat2_bno08x -e hat2_asm330`: both SUCCESS, 0 warnings (bno08x RAM 16420 B / flash 95560 B; asm330 RAM 10988 B / flash 87528 B).
- naviGATR: `cmake -S pi/naviGATR -B $S/w4-nav ...`; `ctest --test-dir $S/w4-nav -j16`: 604/604 passed, 0 build warnings (shared tree with other workers' in-progress code at the time of the run).
- Cross-check only (W5's area): `cmake -S brain -B $S/w4-brain ...`; 444/444 passed against the new codec.

## Contract issues (full text in progress/W4.md)
1. W1: `attitudeStatus()` should read `stale` when an invalid attitude still carries `measuredAt`, and `unavailable` when it names a `source` without a measurement; today both read `assumed_level`. Suggested three-line change given.
2. W1: add Pico op 4 `DIAGNOSTICS` to the inspection Pico op names.
3. W5's proposed `kTelemetryTarget` flag: W4 agrees; waiting for the lead's go-ahead (no codec change needed).
4. No lead CMake change needed from W4.

## Open risks
- No hardware validation: DIAGNOSTICS and 0x14 on the real UART, `gpio_get` on pins in UART/SPI function, TELEMETRY over USB and RS-485, the camera mode on the physical camera.
- Brain attitude is timed by Pi arrival (about one telemetry period plus link latency late); 250 ms freshness leaves about two missed reports of margin. VEX sign conventions are W5's and UNVERIFIED.
- DIAGNOSTICS shares the Pico command queue and 100 ms gap with CONTROL 3/4 (a CONTROL right after a Pico boot can go out up to 100 ms later). A deaf Pico gets a small DIAGNOSTICS attempt every ~7 s.
- The camera config refuses to start on a binary without libcamera (documented); a missing camera at runtime is only a warning.
