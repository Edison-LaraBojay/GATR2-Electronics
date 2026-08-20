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
  -> Localization -> World Estimation -> Target Resolution -> Publishing
```

Each position holds one selected implementation behind one contract. An
implementation may be a leaf, an explicit `noop`, or a composite that
privately owns a nested pipeline (for example `landmark_world` internally
runs observation extraction, association, and a landmark estimator). The
coordinator never learns how many internal children exist, children are
reachable only through their parent, and a child fault never partially
commits the parent output.

| Position | Standard input | Standard output | Responsibility |
|---|---|---|---|
| Sensor Collection | cycle time and each initialized sensor executable | `SensorResultsMap` | Poll every configured sensor and retain its latest typed sample, state, timestamps, and diagnostic. |
| Command Collection | previous `CommandState`, time, and the configured command transport | `CommandState` | Apply newly received command edges and otherwise carry the previous command forward. |
| Preprocessing | sensor results and time | `ArtifactMap` | Convert raw samples into implementation-defined typed artifacts, such as a wheel/IMU motion increment. |
| Localization | sensor results, artifacts, previous robot state, and command state | `RobotState` | Advance smooth odometry, maintain the exposure-time pose history, and (in a future composite) own any robot pose correction internally so vision never jumps wheel/IMU odometry from outside. |
| World Estimation | sensor results, artifacts, robot state, previous world, commands, and previous target state | `WorldState` plus published observation/association evidence | Estimate external state. The `landmark_world` composite privately runs observation extraction, association, and a landmark estimator with an explicit commit policy (`always`, `never`, `on_target_lock`). |
| Target Resolution | commands, robot state, world state, and the published evidence | `TargetState` | Focused domain logic driven by the configured target set: activation edges, robot-relative snapshots, acquire-once latching, timeouts, epoch cancellation. Not an open plugin point. |
| Publishing | all standard results and states | status/side effects | Publish the configured robot/target data and health without changing estimation state. Focused boundary logic driven by the configured transports. |

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
pipeline. A profile is a `<Configuration>` document composing a Robot
description, optional Field data, and a Pipeline fragment by file, resolved
relative to the referencing file with strict fragment roots, no repeated or
template includes, and no cross-file duplicate ids. The resolved profile
carries its declared id and a content digest over every contributing file,
printed at startup and visible to diagnostics. Passing a plain `<System>`
pathname remains a bring-up mechanism, not the deployment-selection contract.

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

```text
config/
  shared/
    robots/gatr2_as5047_bno08x.xml.in     measured robot description template
    pipelines/
      two_wheel_bno08x_no_correction.xml
      three_wheel_bno08x_no_correction.xml
      two_wheel_bno08x_camera_diagnostic.xml
      three_wheel_bno08x_camera_diagnostic.xml
  override/
    field.xml                             nine landmarks, 36 tag mounts
    diagnostics/*.xml.in                  composed diagnostic profiles
    blue/routes/  red/routes/             deliberately empty
```

The robot template owns every physical fact (transports, encoder channels,
wheel geometry, robot frames, camera); the pipeline fragments restate no
measurement and differ only through which declared wheels they reference.
The diagnostic profiles are templates on purpose: they reference the
measured `gatr2_as5047_bno08x.xml`, which exists only after every `@...@`
token is replaced with a measured value. The former root-level XMLs with
guessed geometry have been removed; nothing runnable carries an unmeasured
number. Even a fully measured camera profile additionally requires the real
libcamera capture backend, the AprilTag detector adapter, and a physical
Brain-to-Pi command return path before the vision stack is live.

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
