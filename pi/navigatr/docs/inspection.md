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
configuration   {id, name, digest (hex), loop_rate_hz}
inspection      {snapshot_hz, preview_hz, preview_quality, preview_max_width}
robot_body      {length_m, width_m, height_m, origin_x_m, origin_y_m}
fields[]        one per configured field_map resource
  resource_id, name
  dimensions    null or {inside_x_m, inside_y_m, wall_height_m, wall_thickness_m,
                tile_m, source, revision, units_note}
  features[]    {id, kind (box|tape), x_m, y_m, z_m, size_x_m, size_y_m, size_z_m,
                yaw_deg, color, note}      static display geometry, never observed
  landmarks[]   {id, nominal {x_m, y_m, heading_deg},
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

Transforms are `{x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, R}` with
`R = Rz(yaw) Ry(pitch) Rx(roll)` row-major, the convention in
[coordinates](coordinates.md). Field and robot frames are +x forward/right,
+y left/up-the-drawing, +z up; the viewer maps them to its own scene axes
explicitly (see `viewer/app.js`).

## `snapshot`

```text
type, contract, host_ms, session {id, reset_count}, cycle, running
robot           valid, initialized, odometry_epoch, anchor_revision,
                odom {x_m, y_m, heading_deg}, field {...}, field_from_odom {...},
                vx_m_s, vy_m_s, yaw_rate_deg_s, confidence, has_covariance,
                odom_covariance {xx, xy, xh, yy, yh, hh}?   odometry frame, m and rad
                measured_at {clock, ms}, measured_at_host_ms|null, age_ms|null,
                attitude {valid, assumed_level, reference, source, epoch, quality,
                  measured_at_host_ms|null, measured_at_source {clock, ms}, age_ms|null,
                  roll_deg, pitch_deg, yaw_deg, q_wxyz[4]}
localization    estimator_type, updates, history_size, clock_mapped, publication,
                all_ready, functions[] {id, type, ready, note}
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
                height_px, has_image, intrinsics|null,
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
command         null or {stream_on, init_sequence, init_pose {...}, mode,
                object_requested, object_wire_id, object_sequence}
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
                  fault, last}, links[] {id, bytes, packets, decode_errors, seq_gaps}},
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
features, landmark visuals, tag plates), the robot body from
`hello.robot_body` at `robot.field` with measured tilt when
`robot.attitude.valid`, a bounded odometry trail re-expressed under the current
anchor, nominal landmark ghosts beside the estimated poses, the camera mount
and calibrated frustum, the camera panel with tag outlines bound to image
identity, and a diagnostics panel from `sources`, `workers` and
`diagnostics`. Status badges: connecting, live, stale (newest snapshot older
than 1 s of browser time), disconnected, no image, no tags, localization
unavailable, attitude unavailable/assumed, missing metric calibration data, metric
unavailable. The page never estimates anything; it draws what the documents
say and ages it.

Camera scene objects use stable camera identity, not image sequence or epoch.
Detection metadata supplies the mounted camera once available; a matching camera
resource or mounting-frame declaration from `hello` serves as its fallback,
rather than adding a second camera model. New images update the existing preview
and its exact-identity overlay without creating a camera object for every frame.
