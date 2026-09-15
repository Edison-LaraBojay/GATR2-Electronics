# navigatr resources

Registered resource types. A resource is an initialized live object owned by
the ResourceStore: buses, links, shared decoders, shared data. Consumers hold
shared handles captured at initialization; the store's id mapping is frozen
after startup and resources are destroyed after everything that captured
them. A resource that produces measurements also attaches a
`ResourceExecutable`: declared named outputs plus `execute` and `reset`.
The resource stage polls every executable once per cycle and publishes the
outputs into the `ResourceMap` (`ResourceMap[id].outputs[output_id]`), each
output with its own timing, sequence, and health. Configuration-only
resources attach nothing, never appear in the `ResourceMap`, and are bound
directly by their consumers. A shared_ptr does not make hardware thread
safe; each contract states its own guarantee. The registry and code are the
source of truth; this file catalogs them.

Stable PCB wiring is documented in `hardware.md`; the XML remains the
executable configuration and there is no monolithic robot config header.

## linux_serial_link

- Contract: `SerialLink` (readAvailable, write).
- Schema:

```xml
<Resource id="pico_uart" type="linux_serial_link">
    <Device path="/dev/ttyAMA0"/>
    <Baud value="115200"/>
    <DriverEnable gpio="6"/>   <!-- optional, e.g. RS-485 DE//RE -->
</Resource>
```

- Dependencies: none.
- Ownership: opens the device at initialization; open failure is a build
  warning and a dead link at runtime (an unplugged cable must not stop the
  robot). Closed on destruction. `DriverEnable` drives the named GPIO high
  (sysfs) while the link exists and low on destruction; on the HAT this is
  the RS-485 transceiver enable on GPIO6, whose pulldown idles the bus when
  the Pi is dead. GPIO failure is a build warning.
- Thread safety: single threaded.
- Failure behavior: reads report the link closed; consumers surface fault
  states.
- Hardware alignment: the Pico transmits at 115200 (`pico/src/config.h`), the
  brain link is UART5 at `/dev/ttyAMA5` (`dtoverlay=uart5`), and that link is
  one way, Pi to brain, so command collection stays `noop` until a return
  path exists.

## memory_link

- Contract: `SerialLink`.
- Schema: `<Resource id="x" type="memory_link"/>`.
- In-memory link for tests and loopback rigs. Input and output streams are
  separate, so a bidirectional link never reads its own writes.
- Thread safety: single threaded.

## file_replay_link

- Contract: `SerialLink` (read only; writes fail).
- Schema:

```xml
<Resource id="pico_uart" type="file_replay_link">
    <File path="capture.bin"/>
</Resource>
```

- A capture that cannot open is a build error (not hot-pluggable). The app's
  `--replay <resource_id>=<capture.bin>` swaps any declared resource for this
  implementation; naming an undeclared id is a build error.

## pico_telemetry

- Contract: `PicoTelemetry`.
- Schema:

```xml
<Resource id="pico_telemetry" type="pico_telemetry">
    <Serial resource_id="pico_uart"/>
    <Output id="encoder_a" channel="0"/>
    <Output id="encoder_b" channel="1"/>
    <Output id="encoder_c" channel="2"/>
    <Output id="imu" channel="imu"/>
</Resource>
```

- Dependencies: a `SerialLink` resource.
- Outputs: one per `<Output>`, named by `id`. `channel` is `0..2` for an
  encoder counter, published as `pico.encoder_counts` (`PicoEncoderCounts`,
  raw absolute counts), or `imu` for the yaw gyro, published as
  `pico.gyro_rate` (`PicoGyroRate`: latest rate in millidegrees per second
  plus the angle integrated over every decoded packet in millidegrees and
  its discontinuity epoch). Output ids are configuration; a duplicate id or
  a channel out of range fails the build.
- Drains and decodes the link once per resource-stage invocation and
  publishes each configured channel that advanced, stamped with the Pico
  clock of its packet; a channel with nothing new keeps its retained
  record. Several packets drained in one cycle publish once per output with
  the latest value, and the gyro accumulator keeps the rotation of every
  packet. Nobody else touches the UART; channel sensors read the
  `ResourceMap`. All Pico wire knowledge lives here and in `common/`.
- Thread safety: cycle-snapshot based, single threaded.
- Failure behavior: a closed link is Fault on the resource and on every
  output with nothing new; decoded history is retained. Reset clears the
  decoder once and every output restarts its sequence in the next epoch.

## field_map

- Contract: `const FieldMap`.
- Schema:

```xml
<Resource id="game_field" type="field_map">
    <Landmark id="center_goal">
        <NominalPose calibration_status="verified"
                     x_m="1.7832" y_m="1.7832" heading_deg="0"/>
        <ApproachFrame id="center_goal_west_face" calibration_status="verified">
            <PoseOfApproachFrameInLandmark x_m="-0.14" y_m="0" z_m="0"
                roll_deg="0" pitch_deg="0" yaw_deg="180"/>
        </ApproachFrame>
        <TagMount instance_id="center_goal_west_tag"
                  calibration_status="verified" family="tag36h11"
                  observed_id="0" detection_size_m="0.06">
            <PoseOfTagSurfaceInLandmark x_m="-0.14" y_m="0" z_m="0.25"
                roll_deg="0" pitch_deg="0" yaw_deg="180"/>
        </TagMount>
    </Landmark>
</Resource>
```

- Immutable shared field data: a landmark has one semantic origin, one
  nominal field pose, any number of named approach frames (+x outward from
  the landmark, +y left looking outward), and any number of physical tag
  mounts in full SE(3) (several mounts may share one printed observed_id;
  which mount produced an observation is association's decision, by full
  pose, never forced). `detection_size_m` is the edge of the detector's
  pose-estimation corners, not the sticker. Every geometry attribute is
  required and calibration gated. Meters and degrees in, meters and
  radians in memory. No brain wire ids here.
- Thread safety: immutable after construction.

## robot_frame_map

- Contract: `const RobotFrameMap`.
- Schema:

```xml
<Resource id="robot_geometry" type="robot_frame_map">
    <Frame id="front_camera_engineering" parent_frame_id="robot_body"
           calibration_status="verified">
        <PoseOfChildInParent x_m="0.2" y_m="0" z_m="0.3"
            roll_deg="0" pitch_deg="12" yaw_deg="0"/>
    </Frame>
</Resource>
```

- Named robot interaction frames (contact points, camera optical centers)
  measured relative to the robot pose origin, full SE(3). Chains through
  `parent_frame_id` resolve to `robot_body` at build; a missing parent or
  a cycle is an error. A mechanism whose position changes during operation
  is not a fixed frame; define per-state frames or do not use it.
- Thread safety: immutable after construction.

## libcamera_camera

- Contract: `CameraDevice`.
- Schema: `Device index`, `Capture` (width, height, `pixel_format="Y8"`,
  rate), `Calibration` (`calibration_status`, `calibration_id`,
  `Intrinsics model="brown_conrady"` with the full distortion set tied to
  the exact camera/lens/resolution, `Extrinsic frame_id` naming the
  engineering frame in the robot frame map). See
  `config/*_apriltag_landmark_correction.xml.in`.
- Output: one `sensor.camera_frame` (`CameraFramePayload`) under the id of
  the optional `<Output id="frame"/>` child (default `frame`), published
  per new device frame and stamped at exposure (host clock). Any
  `CameraDevice`, including test doubles, becomes an executable resource
  through `cameraResource(device, output_id)`.
- Startup fails when calibration resolution and capture resolution
  disagree. On builds without a capture backend the device is fully
  validated but dead, with a build warning; the output is Unavailable, the
  camera_frame sensor forwards that, and the robot keeps running. The
  libcamera capture backend lands with camera bring-up on the Pi.

## apriltag_detector

- Contract: `TagDetector` (frame in, detector-native detections out).
- Schema: one or more `<Family name="tag36h11" detection_size_m="0.06"/>`.
- The adapter around the upstream AprilRobotics library is not built into
  the binary yet; the type registers and validates so configurations hold,
  and selecting it fails loudly at build instead of detecting nothing.
  Perception owns the single fixed conversion from detector-native axes to
  engineering axes; no native axis escapes it.

## target_set

- Contract: `const TargetSet`.
- Navigation targets the brain selects by wire id. Landmark-relative
  targets name a landmark, one of its approach frames, a controlled robot
  frame, the desired controlled-frame pose, and an explicit
  `VisionCorrection` policy (`none`, `acquire_once` with timeout fallback,
  consistency window, age and angular-speed limits, optional
  `PreferredCamera` and `AllowedTagMount` preferences that never force
  association). Robot-relative targets carry a delta snapshotted once per
  new command sequence. Validated at build against the field map and the
  robot frame map. See `config/*_apriltag_landmark_correction.xml.in` and
  `docs/navigatr.md` for the runtime semantics in `target_tracker`.

## wheel_geometry

- Contract: `const WheelGeometryMap`.
- Schema:

```xml
<Resource id="wheel_geometry" type="wheel_geometry">
    <Wheel id="left_wheel" sensor_id="tracking_encoder_a" label="left"
           calibration_status="verified"
           radius_m="0.0254" position_x_m="0.000" position_y_m="0.130"
           measurement_angle_deg="0" direction="positive"/>
</Resource>
```

- Measured tracking-wheel geometry owned by the robot description. Every
  attribute is required and calibration gated; radius is the loaded
  effective rolling radius. `tracking_wheel_odometry` consumes it through
  `<Wheels resource_id=...><Use wheel_id=.../></Wheels>`, so two- and
  three-wheel pipelines differ only through wheel references. The inline
  `<TrackingWheel>` form remains for self-contained bring-up documents;
  the two forms are mutually exclusive within one preprocessor.
- Thread safety: immutable after construction.

## SpiBus contract

`SpiBus`/`SpiDevice` are typed contracts ready for direct-wired SPI sensors:
bus wiring belongs to the bus resource, per-device chip select, frequency,
mode, and word size belong to the device configuration, and every transfer is
one atomic transaction (lock, apply settings, assert chip select, move bytes,
release, unlock), so devices with different settings share one controller.
A fake bus exercises the contract in tests; the Linux spidev implementation
lands with the first SPI-wired sensor.
