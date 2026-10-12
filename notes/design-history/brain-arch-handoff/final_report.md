# GATR2 Brain/Pi architecture: handoff

Nothing here has run on hardware. Every result below is a host test, an ARM
compile, a PROS link or a PlatformIO build. I committed, pushed, flashed and
deployed nothing (you made two commits yourself during the run, d4bc875 and
f657cdf; the rest is uncommitted in the working tree). No PCB files were
touched.

The full user guide is [docs/brain_setup.md](../../../../GATR2-Electronics/docs/brain_setup.md)
(repo path `docs/brain_setup.md`).

## 1. What is implemented

**Brain libraries** (`brain/`, portable C++ with host tests, compiled into PROS by `brain/gatr2_brain.mk`)
- **communiGATR**: Pi link v4 over USB or RS-485.
  - Sessions, reconnects, and the robot profile upload and apply.
  - Field map and estimate transfer.
  - Placement, control commands with request ids, wheel readings.
  - Readiness summary, recovery history, and the startup placement policy.
  - Per-wheel travel calibration.
  - A VEX IMU recalibration gated by the Pi's stillness check.
- **investiGATR**: planner only.
  - Direct and obstacle-avoiding plans over the field's collision boxes.
  - Start escape and goal approach near walls and landmarks.
  - Shared field, reference and path types.
- **actuGATR**: `goToDirect` / `goToAvoiding(destination, relativeTo = Field::Origin, options)`.
  - Segment follower, differential and holonomic, with no corner cutting.
  - Tank and mecanum kinematics with desaturation.
  - Drive owner (manual, navigate, disabled).
  - PROS drive task with a request mailbox and bounded locks.
- **brain/robot/gatr2_robot.h**: the one robot description both programs share.

**Brain programs** (PROS kernel 4.2.2)
- `brain/localization-test`: motor free. Status, wheel calibration and recovery pages; placement; IMU recalibration and reinit.
- `brain/testing`: drive program.
  - Direct, avoiding and landmark-relative tests; manual driving.
  - Tank, with a mecanum example.
  - Config checks at start.

**Pi (naviGATR)**
- Brain-profiled localization (`<BrainProfile>`): the Pi builds its models from the Brain's typed profile.
  - Two wheels + VEX IMU, two wheels + Pico IMU, two forward wheels, three wheels ± Pico IMU.
  - Each correction applied once.
- Field documents for Brain planning.
- Stationary window: gyro bias calibration and stillness.
- Wheel readings; Pico commands through the Pico link.
- **Sensor-loss rule**: a used sensor stale over 250 ms or restarted invalidates the pose.
- Pico link v2: boot identity, restart detection, commands, serial reopen.
- Viewer: boundary and boxes, profile footprint, Brain path, and a link/readiness panel.

**Pico firmware**
- v2 sensor frames with boot identity and epochs.
- Pi commands (IMU reinit, counter restart; CONFIGURE exists but is unused).
- Non-blocking IMU bring-up with retries, BNO08X and ASM330.

## 2. Runnable configurations

| Platform | Config | Use |
|---|---|---|
| Brain | `brain/localization-test` | bench: pose, calibration, recovery; no motors |
| Brain | `brain/testing` | driving tests and tuning |
| Pi | `pi/navigatr/config/override/brain_profile_usb.xml` | **your setup**: USB Brain link, Pico encoders, VEX or Pico IMU from the profile |
| Pi | `pi/navigatr/config/override/brain_profile_rs485.xml` | same over RS-485 |
| Pi | older XML configs (bench_vex_imu*, parallel_wheels*, three_wheel templates, demos) | still runnable; with the Brain programs set `kSendProfile = false` |
| Pico | `hat2_bno08x`, `hat2_asm330` | same encoders; the IMU chip differs |

Current robot setup in `gatr2_robot.h`:
- two perpendicular wheels, forward on port 0 and sideways on port 1;
- the VEX IMU on Smart Port 1;
- USB.

**Update order:** Pi first, then the Pico. The new Pico firmware sends only v2 frames, which the old Pi drops.

## 3. Build, upload and service commands

**Brain (Windows, the VS Code PROS extension terminal).**
- Once per program folder: `pros c apply kernel@4.2.2 --force-apply --no-download`, then `git checkout -- .gitignore`.
- localization-test: `cd brain/localization-test`, `pros make`, `pros upload --slot 2 --name localization-test --after screen`
- testing: `cd brain/testing`, `pros make`, `pros upload --slot 1 --name testing --after screen`

**Pi (Raspberry Pi OS), after copying the source over** (see `docs/pi_setup.md`):
```sh
cd ~/navigatr/pi/navigatr
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_TESTS=OFF -DNAVIGATR_WITH_LIBCAMERA=OFF
cmake --build build-bench -j2
sudo bash tools/install_service.sh --user "$USER" --binary build-bench/navigatr --config config/override/brain_profile_usb.xml
sudo systemctl restart navigatr            # after a rebuild or an XML edit
sudo journalctl -u navigatr -f             # live log
```
Viewer: from your computer run `ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>`, then open `http://127.0.0.1:8765/`.

**Pico (PlatformIO, Windows or Linux): Pi first, then the Pico.**
```sh
cd pico
pio run -e hat2_bno08x                  # or -e hat2_asm330; build only
pio run -e hat2_bno08x -t upload        # flash
```

## 4. Where to edit

| What | File |
|---|---|
| Tracking wheels (port, CPR, polarity, gearing, radius, position, angle, travel scale), setup (`kSetup`), IMU source and VEX port, footprint, start pose, link (`kUseUsb`, `kLinkPort`, `kSendProfile`) | `brain/robot/gatr2_robot.h` |
| Drivetrain (tank or mecanum), motor ports and reversal, speed limits, clearance, follower gains, manual speeds, test destinations, startup placement | `brain/testing/include/robot_config.h` |
| Bench settings: startup placement, wheel-calibration reference and limits | `brain/localization-test/include/robot_config.h` |
| Pi devices, wired encoder ports, sensor loss limit (`<Timing sensor_loss_ms>`), fusion noise | `pi/navigatr/config/override/brain_profile_usb.xml` (or `_rs485`) |

No robot geometry lives in the Pi XML for these configs. Values marked PLACEHOLDER or UNMEASURED are guesses, including the 0.024 m wheel radius, the footprint, the start pose and the gains.

## 5. Direct and avoiding tests

In `testing`, with the robot at the start pose (placeholder (1.2, 1.8, 0), open floor west of the center goal):
- A: direct test, `goToDirect(kDirectGoal)`, 0.5 m north.
- X: avoiding test, `goToAvoiding(kAvoidGoal)`, around the center goal.
- Y: landmark test, `goToAvoiding(kLandmarkOffset, Field::RedGoal2West)`, nominal map pose.
- B: cancel. UP: place at the start pose. LEFT/RIGHT: speed scale. The sticks take over.
- `autonomous()` runs direct, then avoiding.

The screen shows the command state, reason, errors, segment and plan count for tuning. Planner limit: with a 0.46 m footprint and 0.06 m clearance, avoiding moves cannot pass between diagonal goals or leave the corner pockets; drive out of those with a direct move.

## 6. Switching IMU and wheel profiles

All in `gatr2_robot.h`, then rebuild and upload; the Pi rebuilds its models from the new profile.
- `kSetup = kTwoWheelVexImu` (current), `kTwoWheelPicoImu`, or `kThreeWheelPicoImu` (fill in `leftWheel()`/`rightWheel()`).
- `kUseUsb = false` for RS-485, with the Pi on `brain_profile_rs485.xml`. That is the only change on the Pi side.
- Mecanum: `kDrivetrain = kMecanum` in the testing `robot_config.h`.
- `kSendProfile = false` for an older XML-configured Pi config.

## 7. Recovery, placement and recalibration

**Rule:** a sensor the active profile uses that stops delivering for more than 250 ms, or restarts, makes the pose invalid. Both programs then show "POSE INVALID: put the robot at the start pose, press A/UP". Unused sensors never matter; a dead Pico IMU with the VEX profile does nothing.

With the VEX IMU, the samples ride on the Brain link. So a USB drop over 250 ms, a Brain program restart and a VEX recalibration all need a new placement. Startup placement does it at program start.

The same applies when a localization model has to drop measured motion while the robot is placed ("motion lost"). An example is moving a Pico IMU robot before its gyro bias calibration finishes: keep it still until readiness shows Ready.

**Automatic:**
- link reconnect and the session;
- profile re-upload after a Pi restart (a Brain restart re-uploads nothing);
- map and estimate transfers resumed or restarted;
- serial ports reopened (Brain RS-485, Pi, Pico);
- Pico IMU retries;
- Pico commands resent until acknowledged, never run twice.

**Needs you:**
- a placement after any sensor loss, a Pi restart, a profile change, a recalibration or a reinitialization;
- restarting a movement (none resume on their own);
- making wheel calibration values permanent;
- fixing a rejected profile.

## 8. Calibration procedure

**Wheels** (localization-test, Wheel calibration page; full steps in `docs/brain_setup.md` section 4):
1. Pick the wheel and set the reference distance you will push (0.5 m minimum).
2. UP, push exactly that distance straight without turning (0.5 degree limit), then DOWN.
3. Repeat until 3 trials agree within 1 percent.
4. A applies it as a new profile. Place again, then copy the value into that wheel's `travel_scale`.

A trial is rejected if it is stale, restarted, rotated, not along the wheel, in the wrong direction, too short, or out of range (over 10 percent: check radius, CPR, gearing). The wheel's own share of any small rotation is removed.

**IMU:**
- **VEX:** X checks with the Pi that the robot is still, then starts the VEX calibration (about 2 s), then place again.
- **Pico IMU:**
  - X runs the Pi's stationary-window bias calibration; it fails after 60 s without a still window.
  - Y reinitializes the Pico IMU, then place again.
  - Stationary handling reports zero velocity while still; it is not a ZUPT filter.

## 9. Tests actually run

Final run at 02:55 on 2026-09-28, clean builds in fresh lead directories, MinGW g++ 15.2, -Wall -Wextra, no -Werror.

| Target | Result |
|---|---|
| Brain libraries: investiGATR, communiGATR, actuGATR, trajectory integration (`brain/CMakeLists.txt`) | 410/410, 0 warnings |
| Shared protocol (`common/tests`) | 221/221, 0 warnings |
| Pi naviGATR (`pi/navigatr`), including `brain_link_e2e_tests` (9) | 532/532, 0 warnings |
| Pico host tests (`pico/tests`) | 110/110, 0 warnings |
| Pico firmware `pio run -e hat2_bno08x -e hat2_asm330` | both SUCCESS, 0 warnings (build only) |
| PROS `brain/localization-test` and `brain/testing` (kernel 4.2.2, arm-none-eabi 13.3.1) | both link, 0 warnings (not uploaded) |

Also run during the work:
- Mutation checks by each worker: tests caught every injected fault except ones shown to have no effect.
- Repeated runs of the threaded Pi tests.
- The viewer in headless Chrome against a simulated pipeline.
- Independent adversarial reviews of every worker's code and of the lead's code, with all findings fixed.

Not run here:
- The Linux-only serial reopen test (runs in CI).
- `tools/viewer_smoke.sh`, which does not complete on this Windows machine.

## 10. Not verified on hardware

- The USB and RS-485 links on real hardware: NG1 over `/ser/ngtr`, serial settle and reopen, the PROS USB task behavior while unplugged.
- The Pi-to-Pico command wire and status frames on the HAT, and both new Pico firmware builds on the board.
- IMU bring-up retries on real BNO08X and ASM330 parts.
- The tracking wheel radius (0.024 m provisional), positions and polarities.
- The 250 ms sensor-loss limit against real poll gaps and UART jitter.
- VEX IMU recalibration timing, and the bench arrival-time pairing of VEX samples.
- Motor ports, directions, gains and speed limits; the follower on a real drivetrain.
- Collision boxes against a real field; planner timing on the V5.
- The viewer against a live robot. It was checked in headless Chrome against a simulated pipeline only.
- The Linux-only serial reopen test runs in CI, not here.

## 11. Decisions made during the run

- Sensor gap:
  - a used sensor dropping (> 250 ms or restart) invalidates the pose, with a warning; place again at the start pose;
  - no bridging;
  - an unused Pico IMU never matters.
- No Pico CONFIGURE on profile apply; the Pico IMU stays on and retries.
- Viewer additions kept.
- The placeholder start pose moved from (0.6, 0.6) to (1.2, 1.8). The old spot is a closed corner pocket for avoiding moves with this footprint and clearance.
