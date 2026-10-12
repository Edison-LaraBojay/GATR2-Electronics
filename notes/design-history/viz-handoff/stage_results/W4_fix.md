# W4 fix round: review findings on the wire codec, Pico and Pi instrumentation, and attitude work

Date: 2026-09-28. These are host builds and host tests only; nothing ran on the Pico, the Pi, the Brain or a camera. Full notes are in progress/W4.md (FIX section and CONTRACT ISSUES).

## Findings

### 1. MEDIUM: encoder `delta_1s`, `direction` and `note` stayed "current" after Pico frames stopped. FIXED.
- `pi/naviGATR/src/impl/resources/pico_telemetry.h` and `.cpp`:
  - New `PicoEncoderView::fresh`: the port was updated within 250 ms of the refresh (`kFrameFreshMs`, the pipeline clock, the same bound as `frames_fresh`).
  - `delta_known` now requires `fresh`.
  - A per-port `updated_ms` is kept. A silence longer than the 1 s window clears the history, so a resumed port never reports travel across the gap as a one-second change.
- `pi/naviGATR/src/diagnostics/instrumentation.cpp`:
  - Writes `fresh`.
  - `delta_1s`, `span_ms` and `direction` are null while `fresh` is false.
  - The note reads `no recent counts: Pico frames for this port stopped`.
  - `counts` keeps the newest value, with `updated_ms_ago`.
- New tests:
  - `PicoInstrumentation.EncoderChangeIsUnknownOnceFramesStop` (view level). It covers the case right after frames stop, still current at 200 ms, unknown at 300 ms, still unknown after 10 s, and a resume after the silence that does not count the 5000 counts moved while the link was down.
  - `PicoInstrumentation.SystemWritesNoEncoderChangeOnceFramesStop` (a full System plus `writeInstrumentation`). While moving it reads `fwd` and "edges are arriving". After 10 s without frames it reads `"fresh":false,"delta_1s":null,"span_ms":null,"direction":null` with the no-recent-counts note.
- Docs: `pi/naviGATR/docs/pico_link.md` (encoder fields and test list).

### 2. LOW: unrequested 0x14 frames after a Pi restart, and docs claiming this never happens. FIXED in code and docs.
- `pico_telemetry.cpp`: `requestDiagnostics` now wraps `driveDiagnostics` (the old body).
  - With no `<Diagnostics>` rate, a 0x14 frame of the *current* Pico boot means an earlier Pi process asked for it. That boot is asked once for rate 0, through the same request machinery (resend under one id, retry after 5 s if unanswered).
  - The state stays `not requested`.
  - Frames from an earlier boot never trigger this, and a rebooted Pico is sent nothing.
  - The firmware already accepts rate 0 (`PicoCommands.DiagnosticsSetsTheRate`), so it needs no Pico change and no reflash.
- Corrected statements:
  - `docs/interfaces.md`: the overview line, the 0x14 section and the compatibility bullet. An older Pi build sees 0x14 only when a new Pi asked earlier in the same Pico boot, and its reader discards those frames like line noise until the Pico reboots.
  - `pi/naviGATR/docs/pico_link.md`: the Diagnostics section.
  - The `pico_telemetry.h` file comment.
- New test: `PicoDiagnostics.APiWithoutARateTurnsOffFramesAnEarlierPiAskedFor`.
  - Setup: a first PicoTelemetry sets rate 1. A second one, with no rate, is attached to the same Pico, which has not rebooted.
  - After the second one runs, the Pico rate is 0, DIAGNOSTICS ran twice, and the last command is hz 0 to that boot. The state is `not requested`.
  - For 5 s after that, no more frames and no more commands arrive. After a Pico reboot, nothing is sent.

### 3. LOW: a withdrawn Brain attitude group read as measured, then stale, while the docs said unavailable. FIXED by correcting the docs; the explicit-withdrawal code path is DECLINED for now.
- Current behavior:
  - The estimator keeps the last tilt: measured until 250 ms after the last attitude report, then stale (level, old measurement time).
  - This is truthful under the documented freshness rule: "measured" means a tilt measured at most 250 ms ago.
  - For such reports the instrumentation already shows `brain.telemetry.attitude: null`.
- Why the explicit path is declined:
  - An explicit "no attitude now" needs a typed signal on `AttitudeObservation`, in `src/payloads/robot_observations.h`, which is not W4's file.
  - Using only W4 files would mean a sentinel on an existing field (for example an empty `reference`): a hidden contract on a shared payload.
  - The typed alternative (about 20 lines plus a test) is proposed to the lead under CONTRACT ISSUES.
- Corrected:
  - `pi/naviGATR/docs/brain_profile.md`: the attitude table gains a row for dropped groups, and "no attitude report yet" is now the only unavailable row.
  - `docs/interfaces.md`: the TELEMETRY attitude paragraph.
  - The `BrainImuBench::acceptAttitude` comment in `src/resources/brain_imu_bench.h`.
  - The `src/impl/localization/brain_imu_wheel_bench.h` file comment.
- New test: `BrainImuBench.TelemetryDroppingTheAttitudeGroupAgesTheLastTiltOut`.
  - Attitude-less reports at 120 and 240 ms: still measured, with the original time.
  - At 280 ms: stale, level, source `brain_vex_imu`, old time.
  - When the group returns: measured again.

## Test commands and exact results
Build dirs are under `$S`: MinGW Makefiles, Debug, -Wall -Wextra, with `/c/msys64/ucrt64/bin` first on PATH.

| Suite | Commands | Result |
|---|---|---|
| naviGATR | `cmake --build $S/w4-nav -j16`, then `ctest --test-dir $S/w4-nav -j16` | **613/613 passed** (604 at stage 1: +4 W4 tests, the rest are other workers' new tests). 0 warnings in W4 files. The only build warning is W3's `tests/capture_gtest.cpp:594` (-Wdangling-reference). |
| translaGATR | `$S/w4-transla` | 254/254 passed, 0 warnings. No source change this round. |
| Pico host tests | `$S/w4-pico` | 129/129 passed, 0 warnings. No source change this round. |

PlatformIO was not rerun, because no firmware source changed.

## Contract issues (details in progress/W4.md)
- **W1, still open:** `attitudeStatus()` (`src/inspection/inspection_document.cpp:1683`) still maps the estimator's stale attitude to `assumed_level`. Until it adopts the mapping W4 suggested earlier, the viewer cannot show `stale` or the configured-but-unmeasured `unavailable` for the fold's output.
- **W2, optional:** show the new encoder `fresh` field (or `updated_ms_ago`) next to `counts`. The panel is already honest without it: nulls render as n/a, and the note is shown.
- **Lead, optional:** a typed `available` field on `AttitudeObservation`, for an immediate `unavailable` when the Brain withdraws its attitude.

## Open risks
- None of this is validated on hardware. That includes DIAGNOSTICS 0 on the real UART, and the Pi-restart scenario with a Pico that stays powered.
- Encoder freshness uses the pipeline clock. If the pipeline itself stalls, the published view freezes with it; `updated_ms_ago` (HostClock at write time) still shows the true age.
- The turn-off in finding 2 costs one small command, once per Pico boot, and only when unrequested frames appear. An older Pi build cannot turn frames off; they stop at the next Pico reboot.
