# navigatr resources

Registered resource types. A resource is an initialized live object owned by
the ResourceStore: buses, links, shared decoders, shared data. Consumers hold
shared handles captured at initialization; the store's id mapping is frozen
after startup and resources are destroyed after everything that captured
them. A shared_ptr does not make hardware thread safe; each contract states
its own guarantee. The registry and code are the source of truth; this file
catalogs them.

Stable PCB wiring is documented in `hardware.md`; the XML remains the
executable configuration and there is no monolithic robot config header.

## resource/linux_serial_link

- Contract: `SerialLink` (readAvailable, write).
- Schema:

```xml
<Resource id="pico_uart" type="resource/linux_serial_link">
    <Device path="/dev/ttyAMA0"/>
    <Baud value="921600"/>
</Resource>
```

- Dependencies: none.
- Ownership: opens the device at initialization; open failure is a build
  warning and a dead link at runtime (an unplugged cable must not stop the
  robot). Closed on destruction.
- Thread safety: single threaded.
- Failure behavior: reads report the link closed; consumers surface fault
  states.

## resource/memory_link

- Contract: `SerialLink`.
- Schema: `<Resource id="x" type="resource/memory_link"/>`.
- In-memory link for tests and loopback rigs. Input and output streams are
  separate, so a bidirectional link never reads its own writes.
- Thread safety: single threaded.

## resource/file_replay_link

- Contract: `SerialLink` (read only; writes fail).
- Schema:

```xml
<Resource id="pico_uart" type="resource/file_replay_link">
    <File path="capture.bin"/>
</Resource>
```

- A capture that cannot open is a build error (not hot-pluggable). The app's
  `--replay <resource_id>=<capture.bin>` swaps any declared resource for this
  implementation; naming an undeclared id is a build error.

## resource/pico_telemetry

- Contract: `PicoTelemetry`.
- Schema:

```xml
<Resource id="pico_telemetry" type="resource/pico_telemetry">
    <Serial resource_id="pico_uart"/>
</Resource>
```

- Dependencies: a `SerialLink` resource.
- Decodes Pico packets at most once per runtime cycle and keeps a coherent
  latest snapshot per channel (encoder channels 0..2, gyro, accel), so every
  logical channel sensor reads the same decoded packet and nobody re-drains
  the UART. All Pico wire knowledge lives here and in `common/`.
- Thread safety: cycle-snapshot based, single threaded.
- Failure behavior: link death is reported per channel consumer; decoded
  history is retained.

## resource/field_map

- Contract: `const FieldMap`.
- Schema:

```xml
<Resource id="override_field" type="resource/field_map">
    <Landmark id="center_goal">
        <NominalPose x_m="1.8" y_m="1.8" heading_deg="0"/>
        <Tag instance="goal_front" family="tag36h11" observed_id="7"
             x_m="0.15" y_m="0" heading_deg="180"/>
    </Landmark>
</Resource>
```

- Immutable shared field data: landmarks with nominal field poses and
  physical tag instances (several instances may share one printed
  observed_id; which instance produced an observation is association's
  decision). Meters and degrees in, meters and radians in memory. No brain
  wire ids here.
- Thread safety: immutable after construction.

## SpiBus contract

`SpiBus`/`SpiDevice` are typed contracts ready for direct-wired SPI sensors:
bus wiring belongs to the bus resource, per-device chip select, frequency,
mode, and word size belong to the device configuration, and every transfer is
one atomic transaction (lock, apply settings, assert chip select, move bytes,
release, unlock), so devices with different settings share one controller.
A fake bus exercises the contract in tests; the Linux spidev implementation
lands with the first SPI-wired sensor.
