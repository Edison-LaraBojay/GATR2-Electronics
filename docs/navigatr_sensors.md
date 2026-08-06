# navigatr sensors

Registered sensor types. Every configured measurement producer has its own
id and type; the generic builder owns only id, type, duplicates, and factory
lookup, and the selected factory owns everything else in its subtree. The
framework polls sensors and does the record bookkeeping itself: receivedAt
and sequence are assigned by Sensor Collection, a new publication replaces
the stored latest sample, no publication and fault states never erase it.
The registry and code are the source of truth; this file catalogs them.

## pico_encoder_channel

- Factory: `make_pico_encoder_channel`.
- Output payload: `sensor.encoder_sample` (`EncoderSample`), an accumulated
  unwrapped shaft angle in radians with counts-per-revolution and electrical
  sign already applied.
- Schema:

```xml
<Sensor id="tracking_encoder_a" type="pico_encoder_channel">
    <Source resource_id="pico_telemetry" channel="0"/>
    <Calibration counts_per_revolution="4000" invert="false"/>
    <Freshness stale_after_ms="250"/>   <!-- optional; 0 disables -->
</Sensor>
```

- Required resources: a `PicoTelemetry` resource.
- Timestamp source: the Pico clock stamp of the decoded packet (device
  domain).
- Update behavior: publishes when its channel advanced in the shared
  snapshot; a quiet cycle is Valid without a publication. Count wraparound is
  handled with modular arithmetic.
- Failure behavior: link death is Fault after any already-decoded data has
  been published; an open link that goes silent turns Unavailable after
  `stale_after_ms` (default 250) instead of staying Valid forever. History is
  retained in both cases.
- Calibration ownership: counts per revolution and electrical inversion live
  here. Wheel radius, mounting position, and measurement direction are
  preprocessing configuration, never sensor configuration.

## pico_imu_channel

- Factory: `make_pico_imu_channel`.
- Output payload: `sensor.imu_sample` (`ImuSample`), yaw rate in radians per
  second, sign normalized, bias not removed.
- Schema:

```xml
<Sensor id="robot_imu" type="pico_imu_channel">
    <Source resource_id="pico_telemetry" channel="imu"/>
    <Calibration invert="false"/>
</Sensor>
```

- Required resources: a `PicoTelemetry` resource.
- Timestamp source: Pico clock (device domain).
- Update behavior and failure behavior: as the encoder channel.
- Calibration ownership: electrical sign here; bias estimation belongs to
  `imu_normalization` or a HeadingConstraint, which own their
  own `bias_samples` windows.

## camera_frame

- Factory: `make_camera_frame`.
- Output payload: `sensor.camera_frame` (`CameraFramePayload`): the frame
  data, the camera's engineering frame id (resolved in the robot frame
  map by consumers), and the intrinsics the frame was captured under.
- Schema:

```xml
<Sensor id="front_camera" type="camera_frame">
    <Source resource_id="front_camera_device"/>
</Sensor>
```

- Required resources: a `CameraDevice` resource.
- Timestamp source: the exposure timestamp, host clock.
- Update behavior: a new device frame publishes; a live camera between
  frames is a healthy quiet cycle (a slow camera does not disappear).
- Failure behavior: a dead camera device is Unavailable with a
  diagnostic, never a silent absence; history is retained.
- Calibration ownership: intrinsics and the extrinsic frame id live on
  the camera device resource; the mounting transform lives in the robot
  frame map under that frame id.

## Adding a sensor type

Write a factory that returns a `SensorExecutable` (its `PayloadDescriptor`
plus the `execute` and `reset` callables with resources captured in the
closure), register it in `impl/sensors/register_sensors.cpp`, document it
here, and list its tests. Downstream algorithms bind to the payload contract
by sensor id and never learn the hardware: replay, simulation, and live
devices are indistinguishable behind the same payload.
