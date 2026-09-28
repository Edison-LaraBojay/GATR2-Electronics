# VEX IMU bench source

The Brain reads the VEX IMU and sends its continuous rotation in every state
poll; the Pi pairs it with the Pico wheels and returns the ordinary robot pose.
This is the current bench IMU while the external BNO08X is unavailable: a
missing or failed Pico IMU never blocks it.

It pairs the latest readings by their arrival on the Pi. It does not
synchronize the Brain and Pico clocks, so delay and jitter become position
error during fast motion. Use slow translation and turns. It supplies heading
only: no rate, acceleration or tilt.

## Selecting it

On the Brain, in [brain/robot/gatr2_robot.h](../../../brain/robot/gatr2_robot.h):
`kSetup = Setup::kTwoWheelVexImu` and `kVexImuPort` (Smart Port 1 on the
bench). The Brain sends a profile with IMU source Brain VEX IMU; the Pi
selects the model from its topology:

| Profile topology | Pi model |
|---|---|
| two wheels in independent directions (the bench: forward on port 0, sideways on port 1) | `brain_imu_planar_bench`: both translation axes measured |
| two forward wheels (0 or 180 degrees) | `brain_imu_parallel_bench`: sideways motion assumed zero |
| three wheels | refused (IMU combination): the VEX IMU runs on the Brain clock and cannot be fused with independent wheel rotation |

On the Pi, the Brain-profile configs already accept it: `<BrainImu
resource_id="brain_imu"/>` in the profile element and `<BenchImu
resource_id="brain_imu"/>` on the brain_link CommandCollection. See
[USB bench](usb_localization_bench.md) for the full procedure and
[Brain robot profiles](brain_profile.md) for the rules.

## Calibration ownership

- The VEX firmware calibrates the VEX IMU (`pros::Imu::reset()`, started from
  the Brain program). The Pi applies no bias to its rotation, so calibration
  is never applied twice, and the state block reports calibration none.
- While the VEX IMU calibrates its samples are invalid. The Pi counts an
  invalid sample as missing, so a recalibration ends the placement: place the
  robot again afterwards.
- Stationary detection uses the wheels and the heading change only (reduced
  evidence): the rotation must stay within 1 deg/s times the window and every
  wheel within 1 mm.

## Freshness and recovery

The samples ride on the Brain link. With a VEX profile the Pi ends pose
continuity when no valid sample arrives for 250 ms (`sensor_loss_ms`), and when
the mailbox restarts (a new Brain session). So a link outage over 250 ms, a
Brain program restart or a VEX recalibration each need a new placement; the
Brain programs place at their start pose at program start and otherwise show
Needs placement. Stale or missing IMU or wheel data never integrates: the model
rebaselines and replays nothing from the gap.

## XML-configured bench profiles

[bench_vex_imu.xml](../config/override/diagnostics/bench_vex_imu.xml) (RS-485,
two parallel wheels, `brain_imu_parallel_bench`) and
[bench_vex_imu_usb.xml](../config/override/diagnostics/bench_vex_imu_usb.xml)
(USB, the perpendicular bench, `brain_imu_planar_bench`) describe the same
bench in Pi XML, with UNMEASURED wheel defaults (radius 0.024 m, 4000 counts,
offsets 0.15 m). They still build and run, but they refuse Brain profiles, and
the current Brain programs always send one and then refuse to place. Use them
only with a Brain client built without a profile, or for host tests and
replays. Their wheel `direction` attribute is geometric; with a Brain profile
the equivalent is the wheel's encoder polarity.

## Viewer

From the viewing computer:

```sh
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
```

Open `http://127.0.0.1:8765/`. The field and robot pose use the normal viewer.
Landmarks stay nominal; there is no camera correction and no measured tilt on
this path.
