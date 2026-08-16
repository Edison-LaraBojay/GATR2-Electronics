# navigatr

`navigatr` is the Raspberry Pi estimation runtime. It turns timestamped sensor
data and Brain commands into a smooth robot pose, an optional resolved target
pose, health information, and diagnostics. XML selects the hardware and the
implementation used at each fixed semantic position; it does not define an
arbitrary execution graph.

This README is the operational contract. The deeper framework rules are in
[`docs/navigatr.md`](../../docs/navigatr.md); registered configuration schemas
are catalogued in [`docs/navigatr_resources.md`](../../docs/navigatr_resources.md)
and [`docs/navigatr_sensors.md`](../../docs/navigatr_sensors.md).

## Fixed pipeline

Resources are initialized before the runtime loop and passed to the factories
that explicitly reference them. Every runtime cycle then executes the same
semantic sequence:

```text
Sensor Collection -> Command Collection -> Preprocessing
  -> Localization Prediction -> Perception -> Association
  -> Pose Correction -> World Prediction / Target Resolution -> Publishing
```

| Position | Standard input | Standard output | Responsibility |
|---|---|---|---|
| Sensor Collection | cycle time and each initialized sensor executable | `SensorResultsMap` | Poll every configured sensor and retain its latest typed sample, state, timestamps, and diagnostic. |
| Command Collection | previous `CommandState`, time, and the configured command transport | `CommandState` | Apply newly received command edges and otherwise carry the previous command forward. |
| Preprocessing | sensor results and time | `ArtifactMap` | Convert raw samples into implementation-defined typed artifacts, such as a wheel/IMU motion increment. |
| Localization Prediction | sensor results, artifacts, previous robot state, and command state | `RobotState` | Advance smooth odometry and maintain the pose history needed to evaluate delayed observations at exposure time. |
| Perception | sensor results and preprocessing artifacts | `ObservationMap` | Turn camera frames or other perceptual inputs into timestamped observations in standard engineering frames. |
| Association | observations, predicted robot state, world state, command state, and active target state | `AssociationMap` | Decide which configured physical landmark instance could have produced target-relevant evidence. |
| Pose Correction | predicted robot state plus artifacts, observations, and associations | `RobotState` | Optionally correct robot localization. The AprilTag target profiles deliberately select `noop` here so vision does not jump wheel/IMU odometry. |
| World Prediction / Target Resolution | previous world and target state, associations, robot state, commands, and time | `WorldState` and `TargetState` | Maintain configured world estimates and resolve or latch the selected target. |
| Publishing | all standard results and states | status/side effects | Publish the configured robot/target data and health without changing estimation state. |

Every position has one explicitly selected `type`, including intentional
absence such as `<Perception type="noop"/>`. Missing types, missing references,
duplicate ids, incompatible payloads, and malformed calibration are startup
errors rather than implicit behavior.

## Profiles are not targets

A **profile** describes one complete deployed Pi topology: serial links,
sensors, two- or three-wheel geometry, camera devices, algorithms, maps, and
publishers. It is selected at process startup and remains fixed for that run.
Different robots or genuinely different hardware stacks should have different
profiles.

A **target** is a configured navigation intent inside a profile. The Brain
selects it by a stable wire id at runtime:

- `robot_relative` snapshots a configured translation and heading in the
  robot-at-activation axes exactly once.
- `landmark_relative` combines a configured landmark, approach frame,
  controlled robot frame, and desired offset. Its vision policy is explicit:
  `none`, `acquire_once`, or a future separately implemented policy.

Selecting a target must not select a file, infer a wheel layout, or rebuild the
pipeline. The intended startup contract is an allowlisted profile id resolving
to a complete XML file, with the active profile identity/hash made visible to
the Brain and diagnostics. Passing an arbitrary pathname is a bring-up
mechanism, not the final deployment-selection contract.

## Odometry and landmark evidence

Tracking-wheel measurements and the IMU produce the continuously evolving
odometry pose. The field-to-odometry transform and retained host-clock pose
history preserve the distinction between smooth local motion and field
interpretation.

An AprilTag observation is transformed through the calibrated chain

```text
detector-native tag -> camera engineering frame -> robot body
  -> odometry at exposure time -> configured physical tag mount -> landmark
```

Only evidence associated with the active target may participate in its
acquisition. `acquire_once` gathers a configured number of mutually consistent,
fresh observations, latches one target pose in the odometry frame, and then
closes the correction gate. Late evidence from an older target generation or
odometry epoch is rejected. Loss of camera visibility after a successful latch
does not move the target.

## Configuration and calibration policy

- `type` selects a registered implementation; `id` identifies one configured
  instance; `*_id` attributes reference an existing producer or resource.
- The generic builder requires only framework-owned structure. Each selected
  implementation validates its own child attributes and nested XML.
- Runnable `.xml` profiles contain confirmed physical and protocol values.
- `.xml.in` files retain descriptive `@...@` tokens for every unknown value.
  Do not replace an unknown measurement with zero merely to make parsing pass.
- `calibration_status="UNCONFIGURED"` always fails. `provisional` is accepted
  only with `--allow-provisional` for deliberate bench work; `verified` is the
  deployment state.
- Camera intrinsics belong to the exact camera, lens/focus state, sensor mode,
  crop, resolution, and pixel format used at runtime. Camera extrinsics and
  robot contact frames are separate measured geometry.

## Configuration set

- `config/three_wheel_imu_no_landmark_correction.xml` - internal three-wheel +
  IMU odometry bring-up rig; all downstream behavior is explicitly disabled.
- `config/two_wheel_imu_no_landmark_correction.xml` - internal two-wheel + IMU
  odometry bring-up rig; the IMU heading constraint is mandatory.
- `config/three_wheel_imu_apriltag_landmark_correction.xml.in` - non-runnable
  three-wheel camera/target calibration template.
- `config/two_wheel_imu_apriltag_landmark_correction.xml.in` - non-runnable
  two-wheel camera/target calibration template.
- `config/navigatr.xml` - transitional legacy profile; it is not the canonical
  profile-selection mechanism.

The odometry bring-up files do not publish a pose to the Brain. The AprilTag
templates are not runnable merely because their placeholders have been filled:
deployment additionally requires the real libcamera capture backend, AprilTag
detector adapter, and a physical Brain-to-Pi command return path.

## Build and host-side checks

```text
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

The runtime form during bring-up is:

```text
./build/navigatr <complete-config.xml> [--cycles <n>]
./build/navigatr <complete-config.xml> --replay <resource_id>=<capture.bin>
./build/navigatr <complete-config.xml> --allow-provisional
```

Replay must feed the same decoder and semantic pipeline as live hardware. It is
for repeatable regression and timing tests, not a second implementation of the
estimator.
