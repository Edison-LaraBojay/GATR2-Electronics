# navigatr sensors

Registered sensor types. Every configured measurement processor has its own
id and type; the generic builder owns only id, type, duplicates, and factory
lookup, and the selected factory owns everything else in its subtree. A
sensor names its input as `<Source resource_id="..." output_id="..."/>`,
binds it at build against the resource's declared outputs with the payload
type it expects (a wrong resource, output, or payload fails the build), and
at runtime reads that record out of the read-only `ResourceMap`. The sensor
stage polls sensors and does the record bookkeeping itself: sequence and epoch
are assigned by the stage, upstream receipt time is preserved, a publication replaces
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
    <Source resource_id="pico_telemetry" output_id="encoder_a"/>
    <Calibration counts_per_revolution="4000" invert="false"/>
    <Freshness stale_after_ms="250"/>   <!-- optional; 0 disables -->
</Sensor>
```

- Input: a `pico_telemetry` output publishing `pico.encoder_counts`.
- Timestamp source: the Pico clock stamp of the decoded packet (device
  domain), carried over from the resource output.
- Update behavior: publishes when the bound output has a new sequence; a
  retained record read again is a healthy quiet cycle, never a second
  measurement. Count wraparound is handled with modular arithmetic.
- Failure behavior: a faulted output (link death) is Fault after any
  already-decoded data has been published; an output that stays healthy but
  silent turns the sensor Unavailable after `stale_after_ms` (default 250)
  instead of staying Valid forever. History is retained in both cases.
- Calibration ownership: counts per revolution and electrical inversion live
  here. Wheel radius, mounting position, and measurement direction are
  robot-observation configuration, never channel-sensor configuration.

## pico_imu_channel

- Factory: `make_pico_imu_channel`.
- Output payload: `sensor.imu_sample` (`ImuSample`), yaw rate in radians per
  second, sign normalized, bias not removed.
- Schema:

```xml
<Sensor id="robot_imu" type="pico_imu_channel">
    <Source resource_id="pico_telemetry" output_id="imu"/>
    <Calibration invert="false"/>
</Sensor>
```

- Input: a `pico_telemetry` output publishing `pico.gyro_rate`.
- Timestamp source: Pico clock (device domain).
- Update behavior and failure behavior: as the encoder channel. The
  resource's packet-by-packet accumulated angle is converted and forwarded
  as `accumulated_angle_rad` with its epoch, so batching loses no rotation.
- Calibration ownership: electrical sign and wire-unit conversion here;
  bias estimation belongs to `imu_heading_increment` or the
  `tracking_wheel_motion` HeadingConstraint, which own their `bias_samples`
  windows. Nothing is integrated twice.

## camera_frame

- Factory: `make_camera_frame`.
- Output payload: `sensor.camera_frame` (`CameraFramePayload`): the frame
  data, the camera's engineering frame id (resolved in the robot frame
  map by consumers), and the intrinsics the frame was captured under.
- Schema:

```xml
<Sensor id="front_camera" type="camera_frame">
    <Source resource_id="front_camera_device" output_id="frame"/>
</Sensor>
```

- Input: a camera resource output publishing `sensor.camera_frame`; this is
  a forwarding sensor, the payload passes through unchanged.
- Timestamp source: the exposure timestamp, host clock.
- Update behavior: a new frame on the output publishes once; a live camera
  between frames is a healthy quiet cycle (a slow camera does not
  disappear).
- Failure behavior: a dead camera device is Unavailable with a
  diagnostic, never a silent absence; history is retained.
- Calibration ownership: intrinsics and the extrinsic frame id live on
  the camera device resource; the mounting transform lives in the robot
  frame map under that frame id.

## attitude_channel

- Input/output: `AttitudeSample`, carrying an orientation quaternion, reference,
  quality, and epoch. A Source binds a resource and named output as above.
- Optional `<Mounting roll_deg="..." pitch_deg="..." yaw_deg="..."/>` describes
  the sensor mounting; the processor converts orientation to the robot body.
- Rejects nonfinite or zero-length quaternions, normalizes valid ones, and
  preserves measurement and upstream receipt times. Optional Freshness uses
  `stale_after_ms` (default 250; zero disables the timeout).
- `attitude_reference` is the downstream localization observation. The synthetic
  rig provides this payload; current Pico firmware has no attitude report.

## Adding a sensor type

Write a factory that binds its inputs through the `ResourceCatalog`
(`TypedOutputBinding<Payload>` by resource id and output id), captures its
parsed configuration and state, and returns a `SensorExecutable` (its
`PayloadDescriptor` plus `execute(resource_map, context)` and `reset`);
register it in `impl/sensors/register_sensors.cpp`, document it here, and
list its tests. Never capture a `ConfigNode`; the document dies after build.
Downstream algorithms bind to the payload contract by sensor id and never
learn the hardware: replay, simulation, and live devices are
indistinguishable behind the same payload.
