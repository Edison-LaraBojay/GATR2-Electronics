# Inspection service and viewer

The inspection service is an optional, read-only window into the running
runtime. It publishes what the workers already produced, on its own thread,
over one loopback HTTP/WebSocket port, and serves the browser viewer that
renders it. Nothing a browser does changes an estimate; disconnecting every
browser changes nothing in estimation.

## Enabling it

Add to a `<System>` document or a `<Configuration>` profile:

```xml
<Inspection enabled="true" bind="127.0.0.1" port="8765"
            snapshot_hz="20" preview_hz="5" preview_quality="70"
            preview_max_width="640" max_clients="4" client_buffer_kb="1024">
    <RobotBody length_m="0.45" width_m="0.45" height_m="0.30"
               origin_x_m="0" origin_y_m="0"/>
</Inspection>
```

`--inspect-port <n>` on the command line enables the service on loopback at
that port without editing the profile. `static_root="<dir>"` serves the viewer
from a directory instead of the copy compiled into the binary (development
only).

The bind defaults to loopback: reach it through SSH from the viewing computer,
then open the URL in a browser on that computer.

```text
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
http://127.0.0.1:8765/
```

The Pi captures, estimates, encodes and serves bounded data; the browser
renders WebGL. No Pi desktop, X11, CDN, or internet access is involved: the
viewer and the pinned three.js files are compiled into the binary.

## Routes

| Route | Content |
|---|---|
| `GET /` | viewer `index.html` |
| `GET /app.js`, `/style.css`, other `viewer/` files | viewer files |
| `GET /vendor/three.module.min.js`, `/vendor/OrbitControls.js`, `/vendor/LICENSE` | pinned three.js (see `third_party/README.md`) |
| `GET /api/hello` | the `hello` document |
| `GET /api/snapshot` | the `snapshot` document |
| `GET /api/frame.jpg[?camera=<sensor_id>]` | newest JPEG preview for a camera (first camera when omitted); 404 when none |
| `GET /api/health` | `{"ok":true,"running":...,"clients":n}` |
| `GET /ws` | WebSocket upgrade for the live feed |

## WebSocket feed

Every message from the server is either a JSON text message with a `type`
field or a binary message carrying one JPEG preview.

- On connect the server sends `hello`, then `snapshot` at `snapshot_hz`.
- Previews are sent per client at that client's preview budget when a new
  detection frame identity appears. A binary message is
  `uint32 little-endian header length` + `header JSON` + `JPEG bytes`. The
  header is the `frame` document below.
- The client may send `{"type":"preview","hz":5,"quality":70,"max_width":640}`
  to change its own budget. `hz` 0 stops previews for that client. Values are
  clamped to `[0, 30]` Hz, `[1, 100]` quality, `[64, 1920]` px.
- A client whose outgoing buffer is full has the next snapshot or frame skipped
  (counted in `workers.inspection.snapshots_skipped` / `frames_skipped`); a
  client that makes no progress for ten seconds is closed. The service never
  waits for a browser.

## Time and identity

Every time in a document is `host_ms`: the Pi host monotonic clock in
milliseconds since the process started. Each document carries the `host_ms` it
was produced at. The base age of a measurement at host timestamp `x` is
`document.host_ms - x`. Between received updates, displayed pose, attitude,
field-snapshot, and image ages advance by elapsed browser receipt time measured
with `performance.now()`. The browser never subtracts its absolute clock from a
Pi timestamp. Elapsed receipt time also determines when the whole feed is stale
or disconnected; it does not measure network transit delay.

`session.id` is a random identity of the process instance; `session.reset_count`
counts in-process `reset()` calls. A change in either (or in
`robot.odometry_epoch`) means trails, cached frames and selections are
incompatible and must be cleared. Detection frames are identified by
`(camera, epoch, sequence)`; overlays are drawn only on the image with exactly
that identity.

The inspection snapshot obtains robot state, localization status, publication
number, and bounded trail through one `RobotStateFeed::snapshot()` call. They
therefore describe the same localization publication even if estimation or a
reset runs concurrently. Field-worker publications have their own invocation
and timestamps; they need not be contemporaneous with the latest robot pose.

## `hello`

```text
type, contract ("navigatr.inspect/1"), host_ms
session         {id, reset_count}
configuration   {id, name, digest (hex), loop_rate_hz, commands_type,
                brain_profile}      brain_profile: localization comes from a Brain profile
inspection      {snapshot_hz, preview_hz, preview_quality, preview_max_width}
robot_body      {length_m, width_m, height_m, origin_x_m, origin_y_m}
fields[]        one per configured field_map resource
  resource_id, name
  revision      human-bumped field revision, 0 when undeclared
  map_id        null or the map document id a Brain reads (8 hex digits)
  map_error     null, or why this field cannot be served as a map
  boundary      null or {min_x_m, min_y_m, max_x_m, max_y_m, note}   planning, field frame
  obstacles[]   {id, wire_id|null, pose {x_m, y_m, heading_deg}, collision_box|null, note}
  dimensions    null or {inside_x_m, inside_y_m, wall_height_m, wall_thickness_m,
                tile_m, source, revision, units_note}
  features[]    {id, kind (box|tape), x_m, y_m, z_m, size_x_m, size_y_m, size_z_m,
                yaw_deg, color, note}      static display geometry, never observed
  landmarks[]   {id, wire_id|null, nominal {x_m, y_m, heading_deg},
                collision_box|null (landmark frame),
                visual null or {shape (octagonal_prism|box), height_m,
                  base_across_flats_m, top_across_flats_m, size_x_m, size_y_m,
                  size_z_m, tag_plate_width_m, tag_plate_height_m,
                  tag_plate_thickness_m, color, note},
                mounts[] {instance_id, family, observed_id, detection_size_m,
                  T_landmark_tag {x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, R[9]}},
                approaches[] {id, T_landmark_approach {...}}}
tag_families    {family: {width_at_border, total_width}}   cell geometry
cameras[]       {resource_id, implementation, frame_id, alive, diagnostic,
                intrinsics|null, mounted, T_robot_camera|null}
camera_sensors[] sensor ids publishing camera frames
localization    {estimator_type, functions[] {id, type},
                history {retention_ms, capacity, max_interpolation_gap_ms, attitude_gap_ms}}
warnings[]      build warnings
```

A `collision_box` is `{x_m, y_m, yaw_deg, size_x_m, size_y_m, note}`: center
and yaw in its owner's frame, `size_x_m` along its own x. These are the
declared values; the map document the Brain reads rounds box sizes up and the
boundary inward to whole mm ([field assets](field_assets.md#planning-data)). An element
with no box is not an obstacle.

Transforms are `{x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, R}` with
`R = Rz(yaw) Ry(pitch) Rx(roll)` row-major, the convention in
[coordinates](coordinates.md). Field and robot frames are +x forward/right,
+y left/up-the-drawing, +z up; the viewer maps them to its own scene axes
explicitly (see `viewer/app.js`).

## `snapshot`

```text
type, contract, host_ms, session {id, reset_count}, cycle, running
robot           valid, initialized, placement_origin ("command", "configuration"
                or "" before any placement), placement_session,
                placement_sequence, odometry_epoch, anchor_revision,
                odom {x_m, y_m, heading_deg}, field {...}, field_from_odom {...},
                vx_m_s, vy_m_s, yaw_rate_deg_s, confidence, has_covariance,
                odom_covariance {xx, xy, xh, yy, yh, hh}?   odometry frame, m and rad
                measured_at {clock, ms}, measured_at_host_ms|null, age_ms|null,
                attitude {valid, assumed_level, reference, source, epoch, quality,
                  measured_at_host_ms|null, measured_at_source {clock, ms}, age_ms|null,
                  roll_deg, pitch_deg, yaw_deg, q_wxyz[4]}
localization    estimator_type, updates, history_size, clock_mapped, publication,
                all_ready, stationary, continuity_breaks, last_break,
                functions[] {id, type, ready, note, dropped_intervals, dropped_why,
                  stillness {monitored, stationary, calibration (none|collecting|done|
                    waiting for stillness|waiting for data|failed), reason, progress_ms,
                    window_ms, windows, restarts, movements, attempts, steps,
                    bias_dps|null}}
trail[]         oldest first, bounded: {host_ms, x_m, y_m, heading_deg, epoch,
                attitude_valid, roll_deg?, pitch_deg?}   odometry frame
field_snapshot  {invocation, at_host_ms, age_ms, status, diagnostic}
field_objects[] {id, valid, observed, source (none|field_map|observed), confidence,
                frame, pose {x_m, y_m, heading_deg}, nominal|null,
                heading_error_deg|null, displacement_m|null, anchor_revision,
                odometry_epoch, last_observed_host_ms|null, age_ms|null,
                last_source, last_feature, last_source_sequence}
detection_frames[] one per camera, the newest processed frame:
                camera, frame_id, epoch, sequence, exposure_host_ms,
                received_host_ms, processed_host_ms, exposure_age_ms,
                exposure_uncertainty_ms, exposure_time_reliable, width_px,
                height_px, has_image, has_observations, has_trace,
                intrinsics|null,
                mounted, T_robot_camera|null, field_invocation,
                pose_at_exposure {status, exact, odometry_epoch, odom {...}, field {...}},
                attitude_at_exposure {status, attitude {...}},
                field_from_odom {...}, anchor_revision,
                detector_processing_ms, frame_note,
                tags[] {index, family, id, hamming, decision_margin,
                  corners_px[4][2], center_px[2], has_pose, T_camera_tag|null,
                  reprojection_error_px|null, alternate_pose_ambiguity|null,
                  association null or {accepted, rejection, object, mount,
                    candidates[] {object, mount, implied_odom {...},
                      implied_field {...}, translation_error_m,
                      heading_error_deg, score}}}
target          null or {active, target_id, wire_id, generation, status, latched,
                T_odom_robot_target {...}, odometry_epoch, activated_host_ms}
command         null or {session, init_sequence, init_session, init_pose {...},
                object_requested, object_wire_id, object_sequence}
brain_link      null without a brain_link CommandCollection, else
                takes_profile, link_open, session, pi_instance,
                last_request_host_ms|null, last_request_age_ms|null,
                state null or {flags {pose_valid, localized, age_known, anchor_command,
                  anchor_configured}, health {encoders_fresh, gyro_fresh, vision_alive,
                  bias_calibrated, pico_link, imu_initializing, imu_failed, stationary},
                  calibration, profile_state, profile_id|null, map_id|null, estimate_id,
                  odometry_epoch, anchor_revision}
                profile {state (none|applying|applied|rejected), id|null, applied_id|null,
                  reason, detail,
                  running null or {id, generation, topology, summary,
                    wheels[] {port, sensor_id, counts_per_rev, gear, reversed, radius_m,
                      x_m, y_m, angle_deg, travel_scale},
                    imu {source (none|pico|brain_vex), port, vex_smart_port, inverted,
                      bias_function},
                    footprint {front_m, back_m, left_m, right_m},
                    calibration {window_ms, still_rate_dps, still_travel_m}}}
                calibration null or {function, ready, stillness {...}}
                wheels[] {port, valid, fresh, counts, travel_m, discontinuity, age_ms,
                  counts_per_rev, gear, reversed, radius_m, travel_scale}
                operation null or {action, result, detail, started_host_ms, age_ms}
                path null or {session, command_id, mode (direct|avoiding),
                  received_host_ms, age_ms, points[] {x_m, y_m}}
pico            null or {resource_id, frames_fresh, identity, boot_id, acq_epoch,
                imu_epoch, reboots, restarts, imu_restarts, last_frame_age_ms|null,
                status_known, last_status_age_ms|null, firmware|null, uptime_ms|null,
                imu null or {enabled, state, reason, attempts},
                last_command null or {request_id, op, status, detail}}
events[]        oldest first, the newest 32: {sequence, host_ms|null, text}
sources[]       {kind (resource|sensor), id, state (no_data_yet|valid|unavailable|fault),
                diagnostic, payload, has_sample, measured_at {clock, ms},
                received_host_ms|null, receipt_age_ms|null, last_polled_host_ms|null,
                sequence, epoch, upstream_source, upstream_clock,
                upstream_sequence, upstream_epoch}
workers         estimation {running, cycles, rate_hz, last_cycle_ms, mean_cycle_ms,
                  max_cycle_ms, period_target_ms, overruns, dropped, pending,
                  last_cycle_host_ms}
                field {same}
                inspection {running, port, clients, clients_total,
                  client_disconnects, snapshots_sent, snapshots_skipped,
                  frames_sent, frames_skipped, bytes_sent, encodes,
                  last_encode_ms, mean_encode_ms, snapshot_rate_hz, frame_rate_hz}
diagnostics     null or {estimation {cycles, functions[] {label, runs, ok, no_data,
                  fault, last}, links[] {id, bytes, packets, decode_errors, seq_gaps,
                  requests, duplicates, superseded, stale, unknown_session,
                  unanswered, replies, expired, input_pending, late_release,
                  tx_errors}},
                field {same}}
```

Meaning of a few fields:

- `robot.measured_at` is the source clock of the newest folded measurement;
  `measured_at_host_ms` is null until the estimator has a device-to-host
  mapping, and then `age_ms` says how old the pose is.
- `robot.attitude.valid` false with `assumed_level` true is the level
  fallback: the viewer draws it level and labels it assumed. Attitude has its
  own measurement timestamp and age. Its history is separate from pose history,
  so a new wheel pose cannot make retained tilt fresh.
- `field_objects[].source` `field_map` is the nominal placement, never an
  observation; `observed` is only true in the cycle that folded evidence.
  `heading_error_deg` is the estimate's rotation away from nominal.
- `detection_frames[].tags[].has_pose` false means a 2D decode: no metric
  axes are drawn. `reprojection_error_px` and `alternate_pose_ambiguity` are
  null when the detector did not compute them.
- `pose_at_exposure.status` other than `ok` explains why the frame could not
  be placed in the field (`pending`, `gap`, `expired`, `epoch_mismatch`, ...).
- When association produced a trace, the detection frame retains the pose,
  attitude, and field anchor/revision actually used for that association. The
  inspector does not repeat history lookup later and relabel an earlier decision
  with a newer pose or anchor. Pose, yaw rate, and attitude are obtained together
  through `sampleAt(exposure_time)`. Without an association exposure context,
  inspection uses `sampleSnapshotAt(exposure_time)` to capture the available
  history answer and current anchor atomically for display.
- `workers.field.dropped` counts sensor snapshots replaced before the field
  worker reached them (latest-frame policy). Motion increments are never in
  that queue.
- `command.session` is the Brain application session the brain link opened
  (0 when none). It is unrelated to the top-level `session.id`, which
  identifies this Pi process. `init_session`/`init_sequence` identify the
  newest Brain placement; it is requested from localization only while
  `init_session == session`. `robot.placement_*` is what localization actually
  applied. `object_wire_id` is the Brain's landmark wire id and
  `object_sequence` counts selections, releases and new sessions.
- `localization.stationary` is true while some function's stationary window
  qualified and nothing moved since; velocity is then reported as zero and the
  pose is left alone (gated stationary handling, not a ZUPT filter).
  `continuity_breaks` counts poses ended by a lost sensor or lost motion,
  `last_break` names the newest. A function's `dropped_intervals` counts the
  times it discarded measured motion (a gap past its limit, a restart,
  movement while its bias calibrated), `dropped_why` the newest reason; with
  a Brain profile each rise while placed ends the placement.
  `stillness.movements` counts the window restarts caused by movement.
  `stillness.calibration` is the gyro bias calibration of that
  function; `bias_dps` is null until a bias exists. See
  [Brain robot profiles](brain_profile.md#calibration-and-stationary-handling).
- `brain_link.state` is exactly the GET_STATE block the Pi would answer now,
  rebuilt every cycle, so it matches what the Brain reads. `profile.id` is the
  newest id reported (applying or rejected); `applied_id` is the running one.
  Profile and map ids are 8 lowercase hex digits (the generated Brain header's
  `Field::kMapId` is the same map id).
- `brain_link.calibration` is the function that owns the IMU bias for the
  running profile; null when nothing calibrates on the Pi (Brain VEX IMU).
- `brain_link.wheels` is READ_WHEELS as the Brain would read it, with the
  running profile's corrections beside each reading. `travel_m` applies counts
  per revolution, gearing, polarity and radius, never the travel scale.
- `brain_link.operation` is a CONTROL 3 or 4 the Pi is running on the Pico.
- `brain_link.path` is the Brain's newest PATH_REPORT, for display only; the
  Pi does nothing else with it. A report with mode none clears it.
- `pico` is the Pico link the brain_link CommandCollection names, else the
  first Pico telemetry resource. `identity` false with fresh frames is Pico
  firmware without boot identity (v1 frames): no status and no commands.
- `events` is the bounded lifecycle log: Brain session and link changes,
  profile applies and refusals, Pico reboots, epochs and commands, calibration
  results, and sensor loss ("place again").
- `links[]` is keyed by the serial resource id. A frame failing its checksum
  or CRC is skipped by the frame reader and not counted anywhere. `packets`
  counts decoded sensor frames on a Pico link and every CRC-valid frame on
  the brain link; `decode_errors` counts valid frames whose body does not
  decode (brain link: a request whose body length does not match its op).
  `seq_gaps` applies to Pico telemetry only.
- The brain link counters, zero on other links: `requests` (decoded
  requests), `duplicates` (answered from the session's record, not applied
  again), `superseded` (completed in the same drain as a newer request, never
  applied), `stale` and `unknown_session` (answered with that result),
  `unanswered` (processed but not answered: completed in the first drain after
  start or reset, or more bytes followed it), `replies` (written), `expired`
  (reply window missed, nothing sent), `input_pending` (Brain bytes waiting
  before the driver enable, nothing sent), `late_release` (driver enable
  released after the frame deadline plus post guard; can accompany a sent
  reply), `tx_errors` (any other write failure).

## `frame`

Header preceding one JPEG in a binary WebSocket message:

```text
type ("frame"), contract, host_ms, camera, epoch, sequence, exposure_host_ms,
width_px, height_px, preview_width_px, preview_height_px, quality, encode_ms,
format ("image/jpeg")
```

`preview_width_px / width_px` is the overlay scale: corners in
`detection_frames[].tags[].corners_px` are in captured-image pixels, drawn
only when the header's `(camera, epoch, sequence)` equals the snapshot's.

## Viewer behavior

`viewer/` is a static ES-module app over the pinned three.js: an orbit /
top-down / robot-follow 3D field built from `hello.fields` (dimensions,
features, landmark visuals, tag plates), the robot outline at `robot.field`
with measured tilt when `robot.attitude.valid`, a bounded odometry trail
re-expressed under the current anchor, nominal landmark ghosts beside the
estimated poses, the camera mount and calibrated frustum, the camera panel
with tag outlines bound to image identity, and a diagnostics panel from
`sources`, `workers`, `localization` (with each function's stillness and
calibration) and `diagnostics`. Status badges: connecting, live, stale
(newest snapshot older than 1 s of browser time), disconnected, no image, no
tags, localization unavailable, attitude unavailable/assumed, missing metric
calibration data, metric unavailable. The page never estimates anything; it
draws what the documents say and ages it.

Planning and the Brain link:

- The field boundary (green), fixed obstacle boxes (red) and landmark boxes
  (orange) are drawn as outlines. A landmark box follows the landmark: at its
  estimate when observed, else at nominal. The `planning` button hides and
  shows them together with the path.
- The robot outline is the running profile's footprint once a profile is
  applied, else the display-only `RobotBody`.
- The Brain's reported path is a polyline, cyan for direct and magenta for
  avoiding.
- The Brain link panel shows a readiness line (link, profile, sensors, map,
  calibration, placement), the running profile, the bias calibration, the
  wheel readings with their corrections, the Pico link and the event log,
  newest first. The first item that is not ready is also a status badge, and
  `#status` carries it as `data-readiness` (`ready` when everything is).

All of this works with no camera and noop world estimation: the Brain-profile
configs run exactly that way.

Camera scene objects use stable camera identity, not image sequence or epoch.
Detection metadata supplies the mounted camera once available; a matching camera
resource or mounting-frame declaration from `hello` serves as its fallback,
rather than adding a second camera model. New images update the existing preview
and its exact-identity overlay without creating a camera object for every frame.
