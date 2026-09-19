# navigatr

`navigatr` is the Raspberry Pi sensing runtime for the GATR2 VEX robot. It
combines measurements into a robot pose and field-object estimates. Configured
target resolution can turn a requested target into a desired robot pose; the
Brain owns movement control and motor commands.

Robot position and reported landmark position use the configured field
coordinates. Robot heading is its orientation in that field. The landmark's
inspection `heading_error` is its estimated rotation away from nominal, wrapped
to `(-180, 180]` degrees. The existing Brain publisher uses full field heading
and gives a matching latched target precedence over a mapped field-object pose.

## Runtime design

These documents describe the implemented runtime and data contracts. Hardware
integration still needs the checks recorded in the deployment documents.

| Document | Responsibility |
|---|---|
| [Configuration](docs/configuration.md) | Run commands, the compiled default file, inline XML, and nested file references. |
| [Architecture](docs/architecture.md) | Captured resource/sensor functions, ResourceMap and SensorMap result contracts, all runtime paths, nested stage I/O, workers, and pose history. |
| [Coordinates](docs/coordinates.md) | Field and robot axes, heading, camera mounting, attitude, and measurement-time transforms. |
| [Landmarks](docs/landmarks.md) | Nominal and observed field state, association, target resolution, and Brain output. |
| [Inspection](docs/inspection.md) | The versioned inspection contract, the service, and the browser viewer. |
| [Field assets](docs/field_assets.md) | Official Override CAD source, revision, units, axis conversion, and what the field file was checked against. |
| [Calibration inventory](docs/calibration_inventory.md) | Every remaining measurement, where it goes, and what it gates. |
| [Pi camera setup](docs/pi_camera_setup.md) | libcamera stack, build flag, capture mode, exposure timing convention, hardware checks still to run. |
| [Attitude follow-up](docs/attitude_firmware_followup.md) | What the Pico firmware would have to send for live tilt. |

## What runs

One executable, built once from a configuration, with two workers and an
optional inspection service:

```text
construction   parse -> make_resources -> make_sensors -> make_localization
               -> field estimation, target resolution, publishing slots
               -> freeze (any failure destroys the candidate)

estimation     resources -> sensors -> commands -> localization
worker         -> target resolution + publishing against the newest field snapshot
(loop rate)    -> hands the sensor snapshot to the field worker (latest wins)

field worker   perception (AprilTag) -> association -> landmark estimate
(event driven) -> publishes an immutable FieldSnapshot and the detection frame
               bound to its exact image identity

inspection     reads snapshots only; JSON + JPEG over loopback HTTP/WebSocket
service        at its own rate; a slow browser is skipped, never waited for
```

Localization is a self-contained component: configured robot-observation
functions grouped by measurement model (`tracking_wheel_motion`,
`imu_heading_increment`, `attitude_reference`), one state estimator
(`planar_motion_integrator`), and a history ring of recent poses that only
localization writes. Readers get copied snapshots or synchronized timestamped
lookups (`RobotStateFeed`). Every mutable state has one writer; `reset()` stops
both workers, resets every stage once, and restarts them.

## Implementation coverage

Implemented and covered by host tests:

- Aggregate stages `make_resources`, `make_sensors`, `make_localization` with
  captured executors; typed result maps with receipt, provenance, sequence and
  epoch preserved through derived records.
- Tracking-wheel odometry with gyro heading, per-source interval alignment,
  encoder rebase on a source restart, rejection of nonpositive intervals, no
  endpoint bridging of invalid spans; optional attitude (quaternion) reconciled
  with the planar heading, aged separately, level fallback labeled assumed.
- Pose history ring (binary search, shortest-arc yaw interpolation, attitude
  slerp, gap and epoch gates, explicit lookup statuses).
- Real AprilTag detection (vendored AprilRobotics detector, `tagCircle21h7` and
  the other supported families) with 2D decoding without calibration, metric
  poses only with intrinsics and a configured corner size, distortion undone
  before the solve, every decoded tag visible with its association decision.
- Capture-time transform chain with the camera offset, mounting rotation and
  measured tilt; association by full pose against every mount sharing an id.
- Two-worker scheduling with a bounded latest-frame handoff, per-worker rate
  and drop counters, orderly shutdown.
- Inspection contract `navigatr.inspect/1`, the service, and the bundled
  three.js viewer (field, robot, trail, nominal vs estimated landmarks, camera
  image with identity-bound overlays, status badges, diagnostics).
- A synthetic rig resource that drives the real pipeline without hardware:
  encoder counts, gyro with bias, attitude (measured or unavailable), rendered
  camera frames of the configured field, a displaced landmark.

Implemented but not run on hardware (see the Pi camera setup document for the
exact checks): the libcamera capture backend (`libcamera_camera`, built only
with `-DNAVIGATR_WITH_LIBCAMERA=ON` on the Pi) including exposure timestamp
mapping and buffer ownership.

Deferred, deliberately:

- Live tilt: the Pico firmware sends gyro Z alongside encoders; the protocol has
  optional accel XY fields, but the firmware does not populate them. No
  attitude report exists, so live runs show attitude unavailable and the
  association uses the assumed-level policy. The attitude path is exercised by
  the synthetic rig.
- Manual exposure and gain control for the camera (auto exposure is used).
- Half-duplex RS-485 turnaround: `DriverEnable` is held high, so the shared HAT
  link cannot yet receive Brain commands using that configuration.
- Measured calibration values for the GATR2 robot: the robot templates stay
  `.xml.in` until measured (see the calibration inventory).

Synthetic tests and desktop browser checks do not establish Pi camera
performance or physical alignment accuracy.

## Configuration and calibration

XML selects registered implementations and supplies their configuration. IDs are
opaque references. Startup checks reject missing producers, incompatible payloads,
duplicate outputs, and invalid calibration.

- [`config/shared/robots/`](config/shared/robots/): the GATR2 robot templates
  (`.xml.in`, measured values required), the uncalibrated camera fragment for
  live inspection, and the synthetic rig fragments.
- [`config/shared/pipelines/`](config/shared/pipelines/): two- and three-wheel
  diagnostic pipelines, the synthetic demo pipeline, and the camera-only
  inspection pipeline.
- [`config/override/field.xml`](config/override/field.xml): nominal Override
  geometry for nine goals and their tag mounts, plus display dimensions and
  static features.
- [`config/override/diagnostics/`](config/override/diagnostics/): composed
  profile templates and the runnable live camera inspection profile.
- [`config/demo/`](config/demo/): hardware-free demo profiles.
- [`config/examples/modular/main.xml`](config/examples/modular/main.xml): a minimal
  runnable scaffold with separate Resources, Sensors, Pipeline, and Localization files.

Files ending in `.xml.in` contain unmeasured or unresolved values and cannot be
loaded. `calibration_status` is an optional reader annotation that runtime
does not interpret; actual parameter values and geometry are validated.
Display metadata (`Dimensions`, `Feature`,
`Visual`) never feeds an estimate.

## Build

Host (Windows, Git Bash, from `pi/navigatr`):

```text
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j8
ctest --test-dir build --output-on-failure
```

Pi (Raspberry Pi OS, from `pi/navigatr`; packages in the Pi camera setup
document):

```text
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_WITH_LIBCAMERA=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

Without `NAVIGATR_WITH_LIBCAMERA`, selecting `libcamera_camera` is a
configuration error naming the missing backend; nothing pretends to capture.

## Run

```text
./build/navigatr [--config_file="profile.xml"] [--inspect-port <n>]
                 [--cycles <n>] [--inline] [--replay <resource_id>=<capture.bin>]
```

- With no filename, the initial compiled default is
  `config/demo/synthetic_field_demo.xml`. Select another at build time with
  `cmake -S . -B build "-DNAVIGATR_DEFAULT_CONFIG=config/my_robot.xml"`, then rebuild.
  The chosen path is baked in; XML contents are read on each startup.
- `--config_file` overrides that choice for one run; `--config-file` and the
  positional filename also work. `--help` prints the compiled default path.
- Default execution: workers plus the inspection service when the profile enables it.
  Ctrl-C stops the service, then the workers, then the system.
- `--inline` runs every stage on one thread at the loop rate (replay, bench).
- `--inspect-port` enables the inspection service on loopback without editing
  the profile.

Hardware-free demo with the viewer:

```text
./build/navigatr
./build/navigatr --config_file="config/demo/synthetic_field_demo_no_attitude.xml"
```

Live camera inspection before metric calibration (Pi, libcamera build):

```text
./build/navigatr --config_file="config/override/diagnostics/live_camera_inspection.xml"
```

From the viewing computer, forward the port and open the page:

```text
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
http://127.0.0.1:8765/
```

The browser smoke test (`tools/viewer_smoke.sh`) starts the demo, loads the
page headless, and checks that snapshots and a detection frame arrived.
