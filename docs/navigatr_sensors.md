# navigatr sensors

Registered sensor types. Every configured measurement producer has its own
id and type; the generic builder owns only id, type, duplicates, and factory
lookup, and the selected factory owns everything else in its subtree. The
framework polls sensors and does the record bookkeeping itself: receivedAt
and sequence are assigned by Sensor Collection, a new publication replaces
the stored latest sample, no publication and fault states never erase it.
The registry and code are the source of truth; this file catalogs them.

## sensor/pico_encoder_channel

- Factory: `PicoEncoderChannelSensor::create`.
- Output payload: `sensor.encoder_sample` (`EncoderSample`), an accumulated
  unwrapped shaft angle in radians with counts-per-revolution and electrical
  sign already applied.
- Schema:

```xml
<Sensor id="tracking_encoder_a" type="sensor/pico_encoder_channel">
    <Source resource_id="pico_telemetry" channel="0"/>
    <Calibration counts_per_revolution="4000" invert="false"/>
</Sensor>
```

- Required resources: a `PicoTelemetry` resource.
- Timestamp source: the Pico clock stamp of the decoded packet (device
  domain).
- Update behavior: publishes when its channel advanced in the shared
  snapshot; a quiet cycle is Valid without a publication. Count wraparound is
  handled with modular arithmetic.
- Failure behavior: link death is Fault after any already-decoded data has
  been published; history is retained.
- Calibration ownership: counts per revolution and electrical inversion live
  here. Wheel radius, mounting position, and measurement direction are
  preprocessing configuration, never sensor configuration.

## sensor/pico_imu_channel

- Factory: `PicoImuChannelSensor::create`.
- Output payload: `sensor.imu_sample` (`ImuSample`), yaw rate in radians per
  second, sign normalized, bias not removed.
- Schema:

```xml
<Sensor id="robot_imu" type="sensor/pico_imu_channel">
    <Source resource_id="pico_telemetry" channel="imu"/>
    <Calibration invert="false"/>
</Sensor>
```

- Required resources: a `PicoTelemetry` resource.
- Timestamp source: Pico clock (device domain).
- Update behavior and failure behavior: as the encoder channel.
- Calibration ownership: electrical sign here; bias estimation belongs to
  `preprocessor/imu_normalization` or a HeadingConstraint, which own their
  own `bias_samples` windows.

## Adding a sensor type

Implement `Sensor` (poll returning `SensorPollResult`, declare
`outputPayload()`), capture resources in the factory, register the factory in
`register_sensors`, document it here, and list its tests. Downstream
algorithms bind to the payload contract by sensor id and never learn the
hardware: replay, simulation, and live devices are indistinguishable behind
the same payload.
