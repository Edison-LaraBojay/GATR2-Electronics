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

- Contract: `SerialLink` (readAvailable, write, windowed write, inputPending,
  nowUs).
- Schema:

```xml
<Resource id="brain_uart" type="linux_serial_link">
    <Device path="/dev/ttyAMA5"/>
    <Baud value="115200"/>
    <DriverEnable gpio="6"/>   <!-- optional, RS-485 DE and /RE: half duplex -->
</Resource>
```

- `DriverEnable` optional attributes: `post_guard_us` (driver held after the
  transmitter is empty, default 2 character times, 174 us at 115200) and
  `tx_margin_us` (added to the frame airtime for the transmit deadline,
  default 2000).
- Dependencies: none.
- `Device` optionally accepts `required="true"` (default false). When true,
  inability to open the serial device or its configured DriverEnable GPIO is
  a build error. The bench startup service uses this to retry until its UARTs
  and GPIO are accessible. This checks local hardware access, not whether the
  Brain or Pico is connected or responding.
- Ownership: opens the device at initialization, with no automatic reopen.
  With `required="false"`, open failure is a build warning and a dead link at
  runtime. Closed on destruction; a half-duplex link drives DE low first.
- Clock: `nowUs()` is the steady clock in microseconds. Transmit windows and
  read timestamps use it, never the pipeline cycle time.
- Windowed write: `write(bytes, TransmitWindow{not_before_us, deadline_us})`
  waits until `not_before_us` and sends nothing once `deadline_us` has
  passed (`expired`). Plain `write(bytes)` uses an unbounded window.
- Half duplex (`DriverEnable` present). The GPIO is exported through sysfs,
  driven low at open so the transceiver listens, and its value file stays
  open. Each write:
  1. checks the window (`expired`, nothing sent);
  2. refuses while received bytes are waiting (`FIONREAD`; `input_pending`,
     nothing sent);
  3. drives DE high and writes every byte across partial writes, waiting for
     output space with poll();
  4. waits for the transmitter to be empty (`TIOCSERGETLSR`/`TEMT` polled
     once per character time; `tcdrain` if the driver lacks that ioctl);
  5. holds DE for the post guard, then drives it low.

  Steps 3 and 4 share one deadline: bytes * 10 / baud + `tx_margin_us` from
  DE high. Every error or timeout discards unsent output (`tcflush`
  `TCOFLUSH`) and drives DE low. `late_release` marks a release later than
  deadline + post guard (for example the thread was preempted). The write
  blocks its caller for about the frame airtime: 5.1 ms for 59 bytes at
  115200.
- Without `DriverEnable` the link is full duplex: windows apply, input
  pending does not.
- Thread safety: single threaded.
- Failure behavior: reads report the link closed; consumers surface fault
  states. A write reports `ok = false` with `error` text; `expired` and
  `input_pending` mean nothing was sent. If the DriverEnable GPIO cannot be
  reached, initialization fails for a required device; otherwise it is a build
  warning and every write fails. The link never reports success with nothing
  on the bus.
- Hardware checks: the `gpio` attribute is the sysfs number. Newer kernels
  can offset it (for example 512 + n, so GPIO6 is 518); compare
  `/sys/class/gpio/gpiochip*/base` on the installed kernel. If the runtime is
  killed (SIGKILL, crash) while transmitting, DE stays high until the next
  start drives it low. Implemented and host tested with a fake port; not
  validated on hardware (TIOCSERGETLSR on the Pi UART, turnaround timing, DE
  behavior).
- Hardware alignment: the Pico transmits at 115200 (`pico/src/config.h`); the
  brain link is UART5 at `/dev/ttyAMA5` (`dtoverlay=uart5`), RS-485 half
  duplex with DE on GPIO6, where the Pi only answers Brain requests.

## memory_link

- Contract: `SerialLink`.
- Schema: `<Resource id="x" type="memory_link"/>`.
- In-memory link for tests and loopback rigs. Input and output streams are
  separate, so a bidirectional link never reads its own writes.
- Clock: steady microseconds unless a test injects one with `setClock`.
- Windowed write never waits: before or inside the window it writes, after
  the deadline it is `expired`, and while input is waiting it is
  `input_pending`; nothing is written in either case.
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
  parsed and numerically validated. `calibration_status` is a reader annotation
  with no runtime effect. Meters and degrees in, meters and radians in memory.
  No brain wire ids here. Optional Dimensions, Feature, and Visual metadata
  supplies viewer geometry only.
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
  rate), optional `Calibration` (`calibration_id`,
  `Intrinsics model="brown_conrady"` with the full distortion set tied to
  the exact camera/lens/resolution, `Extrinsic frame_id` naming the
  engineering frame in the robot frame map). See the
  [camera template](../pi/navigatr/config/shared/robots/gatr2_front_camera.xml.in).
  Omitting Calibration supports 2D detection and inspection without metric pose.
- Output: one `sensor.camera_frame` (`CameraFramePayload`) under the id of
  the optional `<Output id="frame"/>` child (default `frame`), published
  per new device frame and stamped at exposure (host clock). Any
  `CameraDevice`, including test doubles, becomes an executable resource
  through `cameraResource(device, output_id)`.
- Startup fails when calibration resolution and capture resolution
  disagree. Selecting this resource without compiling the libcamera backend
  fails configuration. With the backend built, a camera open failure produces
  a warning and an unavailable device so the rest of the runtime can run.
  Capture receives frames asynchronously and resource polling forwards the
  newest completed frame. See [Pi camera setup](../pi/navigatr/docs/pi_camera_setup.md).

## apriltag_detector

- Contract: `TagDetector` (frame in, detector-native detections out).
- Schema: one or more `<Family name="tag36h11" detection_size_m="0.06"/>`.
- The bundled AprilRobotics adapter decodes configured families, including
  `tagCircle21h7`, and solves metric poses when calibration and corner size are
  available. Detector settings are supplied in a `Detector` child (including
  `quad_decimate`, `nthreads`, and `refine_edges`). Without intrinsics it still
  publishes IDs/corners for inspection. Perception converts detector-native
  axes to engineering camera/tag axes before association.

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
  robot frame map. The `configured_targets` implementation consumes this
  resource; see [target lifecycle and output](../pi/navigatr/docs/landmarks.md).

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
  numeric geometry attribute is required and validated; radius is the loaded
  effective rolling radius. `calibration_status` is an optional reader note.
  `tracking_wheel_motion` consumes it through
  `<Wheels resource_id=...><Use wheel_id=.../></Wheels>`, so wheel pipelines
  differ in wheel references plus, for forward-only parallel wheels, the
  explicit `<LateralMotion assume="zero"/>`. The inline
  `<TrackingWheel>` form supports self-contained configurations;
  the two forms are mutually exclusive within one observation function.
- Thread safety: immutable after construction.

## synthetic_rig

- A hardware-free resource producing raw encoder counts, accumulated gyro data,
  optional attitude, and rendered AprilTag frames through named outputs.
- Binds a field map and wheel geometry; configuration supplies the trajectory,
  camera, gyro bias, timing, and optional landmark displacement.
- The demo drives the same channel sensors, observation models, detector, and
  association code used by other profiles. Its geometry and measurements are
  synthetic, not calibration for a physical robot.
- See the [demo robot fragment](../pi/navigatr/config/shared/robots/synthetic_rig.xml)
  and [demo profiles](../pi/navigatr/config/demo/).
