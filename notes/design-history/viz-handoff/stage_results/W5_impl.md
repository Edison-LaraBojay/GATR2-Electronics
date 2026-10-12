# W5 stage 1 result: Brain telemetry and VEX attitude

Date 2026-09-28. Host builds and PROS builds only. Nothing was committed, flashed, uploaded or run on hardware.

## Built

**communiGATR** (`brain/communiGATR`)
- New `include/communigatr/attitude.h`, `src/attitude.cpp`:
  - `VexAttitude`: raw PROS roll and pitch, with a stamp.
  - `RobotAttitude{valid, roll_rad, pitch_rad}`.
  - `robotAttitudeFromVex(roll_deg, pitch_deg, mount_yaw_deg)`: builds the up vector from the IMU angles, rotates it exactly by the mount yaw about +z, and reads robot roll in (-pi, pi] and pitch in [-pi/2, pi/2] back out. Uses the translaGATR convention: roll about +x, left side up positive; pitch about +y, nose down positive.
  - `setAttitude(BrainTelemetry&, RobotAttitude)`: centidegrees and the flag. An invalid attitude clears the group.
- `client.{h,cpp}`:
  - `Client::reportTelemetry(const translagatr::BrainTelemetry&)` and `telemetryUnsupported()`.
  - `ClientConfig::telemetry_period` (0.1 s).
  - `ClientStats`: `telemetry_reports`, `telemetry_replaced`, `telemetry_dropped`, `telemetry_refused`.
  - Scheduling: telemetry has the lowest priority, after path reports. It is not a waiting transfer, so it never takes a due poll's turn and never uses the four-poll starvation rule. It goes only in the first slot right after a state reply. Sends sit on a 0.1 s grid and are never closer together than 50 ms.
  - Delivery: latest wins, one attempt, never resent. A report is dropped with its session, and also when it is still unsent two periods later (a link outage), so it cannot reach the Pi looking fresh.
  - UnsupportedOp (older Pi) or InvalidArgument: telemetry is marked unsupported for this session and counted, and the session is kept. From another `pi_instance` the reply is treated as a Pi restart. The next session tries once more.
- `pros_link.{h,cpp}`: `ProsLink::reportTelemetry` (bounded mutex; busy returns false) and `ProsLinkStatus::telemetry_unsupported`.
- `pros_vex_imu.{h,cpp}`: `ProsVexImu::attitude()` reads `get_roll`/`get_pitch`. It is invalid on error, while calibrating, and on `PROS_ERR_F`.
- `sim/fake_pi.{h,cpp}`:
  - TELEMETRY is kept, answered header only, and repeatable (same id and body).
  - `setTelemetryResult()` makes the fake a refusing Pi.
  - The old-Pi mode is `setUnsupportedOp(kOpTelemetry)`.
- Tests:
  - New `tests/attitude_gtest.cpp` (9).
  - New `tests/client_telemetry_gtest.cpp` (19; the client tests run on both RS-485 and USB).
  - `client_timing_gtest.cpp` now includes TELEMETRY in the timeout budget check.
- Build lists: `CMakeLists.txt` and `gatr2_brain.mk` (`src/attitude.cpp`).

**actuGATR** (`brain/actuGATR`)
- `DriveStatus::motor_rpm[]`: the motor velocity targets `Drive` sent. They are 0 while stopped, and `stop()` keeps the group count, so a stop reads as a commanded 0 rather than as missing data.
- New `include/actugatr/telemetry.h`, `src/telemetry.cpp`: `telemetryOf(const DriveSnapshot&)` builds the motion and wheels groups (mm, mm/s, cdeg, cdeg/s, rpm x10). Values are rounded, saturated and wrapped; non-finite values become 0. The target is 0 when there is no resolved destination.
- New `tests/telemetry_gtest.cpp` (6). It also pins every MotionState, MotionReason, PlanMode and DriveFault value that the docs tables list.
- `CMakeLists.txt`: the new source and test, plus the repo root include (only `translaGATR/frames.h` is used). `gatr2_brain.mk`: `src/telemetry.cpp`.

**Programs**
- `brain/robot/gatr2_robot.h`: `kSendTelemetry` (true), `kTelemetryPeriodMs` (100), `kVexImuMountYawDeg` (0, PLACEHOLDER).
- `brain/operaGATR/src/main.cpp`: the background task sends telemetry every 100 ms on a grid. Motion and wheels come from `ProsDrive::status()`, a copy made under the drive mutex; no lock is held while the telemetry is built or sent. The VEX attitude is added. Screen row 9 shows the tilt and whether telemetry is on, off or refused by the Pi.
- `brain/locaGATR/src/main.cpp`: sends the attitude every 100 ms (VEX IMU profile only). Status row 6 shows the robot-frame roll and pitch, plus "(no TM)" when the Pi refused telemetry.

**Docs**
- `docs/communigatr.md`: new Telemetry section (API, rules, cost, attitude) and updated layer, table, config, stats, priority list, recovery table, fakes and tests.
- `docs/actugatr.md`: new Telemetry section with the field and unit table and the enum tables for MotionState, MotionReason, PlanMode and DriveFault as sent.
- `docs/brain_setup.md`: new section 10 (telemetry and attitude, settings, Pi version behavior, cost, sign conventions marked UNVERIFIED, and a bench check with a fix table), plus notes in sections 1, 3, 7 and 9.
- Short README updates: brain, communiGATR, actuGATR, robot.

## Test commands and results
Recipe (Git Bash, `/c/msys64/ucrt64/bin` first on PATH):
```
cmake -S brain -B $S/w5-brain-final -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$S/nav-base/_deps/googletest-src
cmake --build $S/w5-brain-final -j16
ctest --test-dir $S/w5-brain-final -j16
```
- Before the changes: 410/410 passed, 0 warnings.
- After (fresh build): 0 warnings, 444/444 passed (+34). The telemetry, attitude and timing subset passed 34/34 on each of 3 repeats.
- PROS (spec section 0 recipe, `make.exe -j16` in each program folder): operaGATR and locaGATR both link with 0 warnings and 0 errors.
- ARM check (`arm-none-eabi-g++ -std=c++17 -fsyntax-only -Wall -Wextra`) of attitude.cpp, client.cpp, telemetry.cpp, drive.cpp and fake_pi.cpp: clean.

Measured telemetry cost. These are host fakes with default timing, a 3 s window, and telemetry at 10 Hz (30 sent), taken with a throwaway probe against the final libraries:

| Transport | State polls without | State polls with | Longest poll gap without | Longest poll gap with |
|---|---|---|---|---|
| RS-485 bus | 145 | 130 (133 in the test's run) | 21 ms | 32 ms |
| USB console | 145 | 133 (137 in the test's run) | 21 ms | 30 ms |

## Contract issues (also in progress/W5.md)
1. The spec says telemetry "must not delay state polls". That cannot be met literally at the defaults. Telemetry never takes a due poll's turn, but one TELEMETRY exchange (about 9.8 ms plus the 5 ms gap on RS-485) is longer than the roughly 5 ms idle window after a state exchange. The next poll therefore moves back, costing 6 to 10% of polls in the host fakes, and the VEX IMU samples carried by GET_STATE slow by the same fraction. A strict fit rule would never send telemetry. Available switches: `kSendTelemetry = false`, or `kTelemetryPeriodMs = 200`. This needs a lead decision.
2. The motion group has no validity bit for its target. With no resolved destination the target is (0, 0, 0), which looks like a real target at the field origin. Proposal: `kTelemetryTarget = 1u << 3` in `TelemetryFlagBit`, owned by lead, W4 and W1/W2. W4's decoder already keeps unknown flag bits, so an older Pi would ignore it. Until then, W1/W2 should treat (0, 0, 0) as missing, or draw the target only while `motion_state` is 2 (running) or 3 (settling).
3. For the lead: the Pi e2e test compiles the changed `client.cpp` and `drive.cpp`. Neither has new source dependencies. W5 did not build it.

## Open risks
- The VEX roll and pitch sign conventions and axis assignment are UNVERIFIED on hardware, and the mount yaw is a PLACEHOLDER. The bench check is in brain_setup.md section 10. An IMU mounted on its side or upside down is not supported.
- The telemetry cost on the real V5 USB link has not been measured.
- `stamp_ms` is the Brain millis at assembly, not a measurement time. The drive snapshot can be up to 10 ms older.
