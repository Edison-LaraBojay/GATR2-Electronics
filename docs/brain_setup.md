# Brain setup, calibration and tests

This guide covers how to configure, build, calibrate and test the robot from
the Brain:

- The Brain describes the robot: tracking wheels, encoders, IMU source, footprint, start pose, drivetrain and gains. It sends that description to the Pi as the robot profile each time it connects.
- The Pi owns its devices (serial ports, cameras, the field definition).
- The Pico owns pins and sensor drivers.

Changing the robot's geometry or tuning means editing Brain C++ and uploading;
no Pi XML changes.

Nothing in this guide has been validated on the robot yet. The host test suites
check the software, not the hardware.

## 1. Where settings live

| What | File |
|---|---|
| Tracking wheels: encoder port, counts per revolution, polarity, gearing, radius, mounting position, measuring direction, travel scale | [brain/robot/gatr2_robot.h](../brain/robot/gatr2_robot.h) |
| Localization setup (two wheels + VEX IMU, two wheels + Pico IMU, three wheels + Pico IMU) | `gatr2_robot.h`, `kSetup` |
| IMU source, VEX Smart Port, IMU calibration settings | `gatr2_robot.h` |
| Robot footprint, start pose | `gatr2_robot.h` |
| Pi link: USB or RS-485 (Smart Port, baud) | `gatr2_robot.h`, `kUseUsb` |
| Drivetrain (tank or mecanum): motor ports, reversal, cartridge, gearing, wheel size, track and wheelbase | [brain/testing/include/robot_config.h](../brain/testing/include/robot_config.h) |
| Speed limits, clearance, follower gains, manual speeds, test destinations, startup placement | `brain/testing/include/robot_config.h` |
| Bench settings: startup placement policy, calibration reference distance and limits | [brain/localization-test/include/robot_config.h](../brain/localization-test/include/robot_config.h) |
| Pi devices, field, cameras, service | the Pi config, `pi/navigatr/config/override/brain_profile_usb.xml` or `brain_profile_rs485.xml` |
| Pico pins and IMU driver | `pico/src/board.h`, the PlatformIO environment |

Values marked UNMEASURED or PLACEHOLDER are guesses; measure them before
trusting the results. The tracking wheel radius, 0.024 m, is provisional.

**Current bench setup**
- Encoder port 0 (J2) measures forward travel and port 1 (J3) measures sideways travel.
- The VEX IMU is on Smart Port 1.
- The Brain talks to the Pi over USB.

The program checks port conflicts: drive motors, the VEX IMU and the RS-485 port must all be different. With a conflict the drive program refuses to create its motors and says why on the screen.

### Switching setups

- **External Pico IMU:** set `kSetup = Setup::kTwoWheelPicoImu`. The Pico firmware decides which chip (BNO08X or ASM330); the profile only selects the Pico IMU port.
- **Three tracking wheels:** set `kSetup = Setup::kThreeWheelPicoImu`, then fill in `leftWheel()` and `rightWheel()`. The Pi fuses the three wheels with the Pico IMU. A three-wheel setup with the VEX IMU is refused, because the VEX IMU runs on the Brain's clock.
- **RS-485:** set `kUseUsb = false` and `kLinkPort`, and run the Pi's `brain_profile_rs485.xml` config. The Pi must run the matching transport config; this is the one setting that is not Brain only.
- **Mecanum:** set `kDrivetrain = Drivetrain::kMecanum` in the testing `robot_config.h`, and fill in `mecanum()`.

## 2. Build and upload

**Brain (Windows, PROS CLI from the VS Code PROS extension terminal).** The first time, in each program folder:

```sh
cd brain/localization-test
pros c apply kernel@4.2.2 --force-apply --no-download
git checkout -- .gitignore
```

Then build and upload:

```sh
pros make
pros upload --slot 2 --name localization-test --after screen
```

The drive program goes in its own slot:

```sh
cd brain/testing
pros make
pros upload --slot 1 --name testing --after screen
```

**Pi (Raspberry Pi OS).** Copy the source from Windows, build and install the service. See [Pi setup](pi_setup.md) for access and the copy step. On the Pi:

```sh
cd ~/navigatr/pi/navigatr
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_TESTS=OFF -DNAVIGATR_WITH_LIBCAMERA=OFF
cmake --build build-bench -j2
sudo bash tools/install_service.sh --user "$USER" --binary build-bench/navigatr --config config/override/brain_profile_usb.xml
```

**Pico (PlatformIO, Windows or Linux).** Build, then flash when the Pico is connected over USB:

```sh
cd pico
pio run -e hat2_bno08x                 # or -e hat2_asm330
pio run -e hat2_bno08x -t upload       # flashes the Pico
```

The Pi and the Brain work with the older Pico firmware too, but only the new firmware reports its boot identity and takes Pi commands (IMU reinitialize, acquisition restart). Without it, the Pi falls back to detecting reboots from the clock and those commands fail with "Pico link".

## 3. Localization test program

`localization-test` never drives motors. Move the robot by hand.

**Startup**
1. Start the Pi service and the program, with the robot still.
2. The program connects over the configured link and sends the robot profile. The Pi checks and applies it; the screen shows the profile id, or the reason for a rejection.
3. **Hold still while the sensors start.**
   - With the VEX IMU, the Brain calibrates it (about 2 s).
   - With the Pico IMU, the Pico aligns to gravity (about 2 s level and still), then the Pi measures the gyro bias over a still window (2 s by default).
4. Once the link, profile and sensors are ready, the program places the robot at `kStartPose`, **once per program start**. If that takes longer than `kStartupWaitSeconds` (90 s, enough for a cold Pi boot), it gives up and you place explicitly. It never places the robot on its own again.

**Pages** (B, or the right screen button, cycles)

| Page | Controls |
|---|---|
| Status | pose, readiness, sensors, calibration. A places at the start pose. X recalibrates the IMU. Y reinitializes the Pico IMU. Hold L1+R1 and press A to reinitialize localization. |
| Wheel calibration | see section 4 |
| Recovery | the last link, session, Pi, Pico and calibration events, with times and outage lengths |

**Readiness line**
- Connecting, reconnecting.
- Profile pending, profile rejected (with the reason).
- Sensors unavailable, sensors initializing.
- Waiting for stillness, calibrating.
- Needs placement.
- Ready.

A pose is **LIVE** when it is connected, placed and no older than 0.25 s.

**Viewer:** the browser viewer on the Pi shows the same pose with the field, its obstacle boxes, the robot footprint from the profile, and a calibration and recovery panel, all without a camera. From your computer run `ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>`, then open `http://127.0.0.1:8765/`.

## 4. Per-wheel calibration

**What it measures and what it does not**

The wheel travel the Pi reports already includes the encoder counts per revolution, polarity, gearing and wheel radius. A calibration push compares that raw travel with a distance you measure yourself (tape measure or field tiles), which gives one number per wheel: `travel_scale`.

That number lumps together:
- radius error;
- gearing error;
- tire compression;
- slip.

A single push cannot tell them apart. A constant scale does not fix slip, missed encoder counts, compression that changes with load, or magnet eccentricity that varies with shaft angle.

Before calibrating:
- Get the radius, counts per revolution, gearing and polarity physically right first.
- Never use the travel scale to hide a wrong one. Corrections beyond 10 percent are refused for that reason.
- Never calibrate against naviGATR's own pose.

**Procedure (forward wheel, then sideways wheel)**
1. Open the Wheel calibration page. LEFT/RIGHT selects the wheel (port 0 forward, port 1 sideways).
2. Set the reference distance you will push: L1/R1 change it by 1 cm, L2/R2 by 10 cm. The starting value is `kCalibrationDistance` (1.0 m). Use at least 0.5 m.
3. Put the robot against a straight edge with room to push along the wheel's measuring direction (forward for port 0, left for port 1).
4. Press UP. The program reads the wheels.
5. Push the robot exactly the reference distance, straight, without turning it. Measure the distance at a point near the robot origin (the point the profile positions are measured from): the program removes the wheel's own share of any small rotation, but not the tape point's.
6. Press DOWN. The trial is accepted or rejected with a reason:
   - reading stale;
   - encoder restarted;
   - robot rotated (more than 0.5 degree);
   - push not along the wheel (the cross wheel moved more than 5 percent);
   - wrong direction or polarity;
   - wheel barely moved;
   - scale out of range: check the radius, CPR and gearing;
   - profile changed.
7. Repeat until at least 3 trials agree within 1 percent. The page then shows READY and the proposed scale.
8. Press A to apply it now:
   - the Brain sends a new profile, and the Pi rebuilds its models with a new baseline, so the change cannot look like motion;
   - this ends the current placement;
   - A again places the robot at the pose it had before the change.
9. Make it permanent: copy the shown value into that wheel's `w.travel_scale` in `gatr2_robot.h`, then rebuild and upload.

The perpendicular wheels calibrate independently: a forward push must leave the sideways wheel still, and the reverse.

**Periodic (angle dependent) correction is not implemented.** The Pico counts quadrature edges from an arbitrary zero at power-up. A repeatable per-angle table would need an index pulse, absolute angle, or homing, which the current encoders and firmware do not provide.

## 5. IMU calibration and reinitialization

**VEX IMU (bench source)**
- The VEX firmware owns its calibration. The Pi applies no bias of its own, so the calibration is never applied twice.
- X on the Status page runs `pros::Imu::reset()`. Hold still for about 2 s. The Pi sees invalid samples meanwhile and holds the pose.
- The VEX IMU gives heading only, so stationary detection on the Pi uses the wheels and the heading change. There is no raw gyro rate or acceleration.

**Pico IMU (BNO08X or ASM330)**

| Step | What it does |
|---|---|
| Gravity alignment | On the BNO08X, at every IMU start. It finds the up axis over 2 s still and level, which makes yaw correct for any fixed mounting. It does not track later rocking or tilt, and gives neither field heading nor mounting yaw. |
| Gyro bias | Measured by the Pi over a still window: wheels still, gyro steady, samples fresh and continuous. Movement restarts the window. Without a still window in 60 s the calibration fails and X retries. The bias is re-measured after any Pico or IMU restart. |
| Bias upkeep | While the robot stays still after calibration, the bias is nudged by bounded steps. Any movement or doubtful data stops it at once. |
| Field heading | Comes only from placement. |

- X starts a recalibration; the Pi refuses it while the robot moves.
- Y asks the Pico to reinitialize its IMU, then recalibrates. The pose holds.

**Stationary handling.** While a still window qualifies, the Pi reports zero velocity and the "stationary" flag. This is gated stationary handling, not a ZUPT filter:
- none of the estimators here models velocity with uncertainty;
- it does not undo drift from before the stop.

**Reinitialize localization** (L1+R1+A) starts a new odometry epoch and needs a new placement. Use it after moving the robot with the Pi off, or when the pose is known to be wrong.

## 6. Placement

- **Program start:** the robot is placed at `kStartPose` once, after the link, profile and sensors are ready and the Pi IMU calibration has settled, within `kStartupWaitSeconds` of program start. A failed IMU calibration holds it until X recalibrates. `kStartupPolicy` chooses:
  - `kAlways` (default): place at program start.
  - `kIfUnplaced`: keep a placement the Pi still holds, for example after a Brain restart mid-match.
  - `kNever`: you place explicitly.
- **Explicit placement:** A (localization-test) or UP (testing) places at the start pose. Put the robot there first.
- **The Brain never re-places the robot on its own** after:
  - a reconnect;
  - a Pi restart;
  - a profile change;
  - a reinitialization.

  The screen shows "Needs placement" instead.

## 7. Drive test program

`testing` drives the robot with actuGATR.

**Controls**
- Left stick Y drives forward and right stick X turns; left stick X strafes on mecanum.
- A runs the direct test, X the avoiding test, Y the landmark test.
- B cancels. Moving a stick also takes over from a running test.
- UP places at the start pose.
- DOWN recalibrates the IMU, only while no movement runs.
- LEFT/RIGHT changes the speed scale.

**Tests**

| Button | Test |
|---|---|
| A | `goToDirect(kDirectGoal)`: straight moves from the field origin reference, no obstacle checks |
| X | `goToAvoiding(kAvoidGoal)`: routes around the field's obstacle boxes |
| Y | `goToAvoiding(kLandmarkOffset, landmark())`: relative to a named landmark from the generated `Field` table. Without a camera it uses the landmark's nominal map pose (`kRequireObservedLandmark = false`). |

- autonomous() runs the direct test, then the avoiding test.
- disabled() stops the drive.
- The screen shows the command state and reason, errors, cross-track, segment and plan count, which is what you tune by.

**Tuning:** see [actuGATR tuning](actugatr.md#tuning). Start at speed scale 0.3.

**Planner limit:**
- The planner treats the robot as a circle around its origin: the footprint's farthest corner plus the clearance.
- On the Override field that circle cannot pass between diagonal goals for robots larger than about 0.4 m square. An 18 inch robot is refused there and must use direct moves in those areas.
- See [investiGATR](investigatr.md).

## 8. What recovers on its own

| Event | What happens |
|---|---|
| USB cable out and back in | The Brain keeps its session and resumes when replies return. The Pi reopens the USB device. The pose continues if the Pi kept running. A movement that was running when the link dropped has failed and does not resume. |
| Brain program restart | The Brain opens a new session and resends the same profile. The Pi recognizes it and keeps localization and the placement. Startup placement follows `kStartupPolicy`. Old movements are gone. |
| Pi restart | The Brain sees a new Pi instance, uploads the profile again, and reads the field map again. The Pi starts unplaced: "Needs placement". |
| Pico restart (new firmware) | The Pi sees the new boot identity, rebases the encoders (counts back at zero never look like movement) and recalibrates the IMU bias. The pose holds, although motion during the outage is lost. |
| Pico IMU failure | The Pico retries it (5 quick attempts, then every 30 s) while the encoders keep flowing. The Pi marks the IMU initializing or failed. A VEX profile is not affected. |
| Interrupted profile or map transfer | Resumed or restarted. Partial documents are never used. |
| Serial device missing at Pi start | Retried once a second. |

**What needs you:**
- a placement after a Pi restart, a profile change or a reinitialization;
- IMU recalibration after moving the robot during calibration;
- wheel calibration values made permanent in `gatr2_robot.h`;
- fixing a rejected profile;
- restarting a movement.

## 9. Tests run and what is not validated

The host test suites cover:
- the wire protocol;
- the Brain link client;
- the planner;
- motion and drives in simulation;
- the Pi pipeline over memory links;
- the Pico logic.

The final report lists the exact counts. Not validated until tried on the robot:
- the USB and RS-485 links on real hardware;
- the Pi to Pico command wire on the HAT;
- the tracking wheel geometry and radius;
- IMU behavior;
- motor directions and gains;
- the collision boxes against a real field;
- planning times on the V5.
