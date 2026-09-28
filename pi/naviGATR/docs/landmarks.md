# Field estimates, targets, and reporting

Field estimation tracks configured physical objects in the same field coordinate
system used for robot localization. Target resolution separately turns configured
navigation intent into a desired robot pose. The browser exposes both; the Brain
link reports only physical landmark poses, never a resolved target.

## Definitions and retained state

`field_map` holds immutable nominal landmark poses, physical tag mounts, named
approach frames, planning data, and optional display geometry. The Override
definition contains nine goals and eight fixed obstacles. Visual metadata such as
boxes, colors, and wall dimensions is used by the viewer and does not enter
localization, association or planning. The planning data (revision, Boundary,
wire ids, collision boxes, Obstacles) is described in
[field assets](field_assets.md#planning-data).

The `apriltag` world estimator maintains `FieldState.objects`, keyed by
configured object ID.
It seeds every mapped object from its nominal pose with source `field_map` and
confidence 0.5. These entries are available before any camera observation, so
`valid` alone does not mean an object has been seen.

Accepted evidence can change an entry to source `observed`. The entry retains:

- Full object pose in odometry coordinates, plus its current field representation.
- Odometry epoch and field-anchor revision.
- Confidence, latest observation time, camera source, frame sequence, and mount ID.
- `observed`, meaning evidence updated the entry during this field invocation.

On a later invocation without new evidence, the observed pose remains retained,
its observation timestamp stays unchanged, and `observed` becomes false. There
is no automatic age-based expiry of a retained FieldObjectState. Inspection shows
observation age so consumers can distinguish old evidence from a new sighting.

Re-anchoring derives a new field representation from the retained odometry pose;
it is not a new observation. An odometry epoch change resets observed entries to
map nominal because their previous coordinate context no longer exists.

## The apriltag estimator subpipeline

```text
camera sensor
  -> ObservationExtraction (AprilTagObservationPerception)
  -> Association (TagMountAssociation, optional)
  -> LandmarkEstimation (LandmarkEstimator)
  -> FieldSnapshot
```

The steps are fixed and private to the `apriltag` estimator; the coordinator
sees only the `WorldEstimation` contract. Omitting `Association` publishes
decodes without evidence, for camera inspection before calibration.

The detector decodes tag family, ID, image corners, and quality. Decoding can run
without calibration for camera inspection. Metric poses require camera intrinsics
and the configured detected-corner size. Perception publishes detections once per
new frame and preserves that frame's image identity and exposure timestamp.

Association evaluates every configured mount matching the observed family and ID.
It retrieves robot pose and attitude at exposure time, applies camera and tag
mount transforms in 3D, and evaluates the implied object pose against the nominal
or retained estimate. Gates include range, expected view, facing, projected pixel
size, detection quality, pose translation/heading error, and optional reprojection
or alternate-pose ambiguity limits.

Candidates are ranked by translation error plus weighted heading error. The best
must pass its gates and beat the runner-up by the configured ambiguity margin.
A request or target preference does not override physical association. Close
competing mounts remain unassociated; no square-symmetry equivalence grouping is
implemented. Optional trace output records candidate and rejection reasons for
the viewer.

`LandmarkEstimation` folds the one declared `FieldObjectPoseEvidenceSet` output.
Evidence from another odometry epoch or containing invalid numeric values is
ignored. Accepted measurements of the same object in one invocation combine by
confidence-weighted position and circular heading mean. `blend` in `[0, 1]`
controls interpolation from the retained estimate; `commit="always"` applies
updates and `commit="never"` publishes evidence without updating the objects.
Confidence is an algorithm score, not a covariance or calibrated probability.
A step fault leaves the previous field state intact.

The field worker runs regardless of target selection and can update several
objects. It publishes FieldState plus observation and association maps. Seeing a
displaced goal changes that goal's estimate, not the robot pose used to measure it.

## Configured targets

`target_set` declares targets selected by a target `wire_id` (its own id space,
not a field object wire id): a selection command activates the target with that
id, and a release deactivates it. Brain link v4 has no selection op, so only
in-process callers (the tests) select targets today; the Brain plans its own
movement from the field documents. `configured_targets` resolves and retains one
active TargetState:

- `robot_relative` snapshots a configured movement relative to the robot when a
  new request activates.
- `landmark_relative` combines a landmark pose, named approach frame, controlled
  robot frame, and desired controlled-frame relationship to produce a robot pose.
- `VisionCorrection type="none"` uses the current committed landmark estimate
  or nominal map pose at activation.
- `VisionCorrection type="acquire_once"` collects qualified evidence after
  activation, checks its timing and consistency, and latches a target. Its
  configured timeout can fall back to the immutable nominal field pose.

Acquisition preferences such as camera and allowed mount filter accepted evidence
for the requested target; they do not decide association. Request sequence and
odometry epoch control the target lifecycle. A latched target is a desired robot
body pose in odometry coordinates, re-expressed in the field for inspection. It
is not the landmark's physical pose and is never sent to the Brain; the Brain
applies its own offset to the landmark pose and performs movement control.

## Brain output

The [`brain_link` publisher](../src/impl/publishing/brain_link_publisher.cpp)
answers Brain requests only; the wire layout is in
[interfaces](../../../docs/interfaces.md). Everything is taken from the cycle
that processed the request: the robot state, the newest completed field
snapshot, and sensor health.

GET_STATE robot fields:

- x/y/heading are the robot origin in the field frame
  (`T_field_odom * T_odom_robot`), millimeters and centidegrees.
- `PoseValid` is `RobotState.valid`; `Localized` is `RobotState.initialized`
  (a placement set the field anchor). `AnchorCommand` or `AnchorConfigured`
  follows `placement_origin`: a Brain SET_POSE or the profile's
  `InitialPlacement`.
- `AgeKnown` and `robot_age_ms` (cycle time minus the pose's host
  measurement time, clamped to 0..65535) are set only when the estimator maps
  the pose onto the host clock.
- `odometry_epoch` and `anchor_revision` are the low 32 bits of the robot
  state's counters.
- Health bits are information only, never acknowledgements. With a Brain
  profile they follow the running profile (its encoders, its IMU source, the
  model that owns the bias; see
  [Brain robot profiles](brain_profile.md#state-block-health)). In an
  XML-configured profile they come only from configured `Health` references:
  encoders fresh when every listed `Encoder` sensor is valid and was received
  within `fresh_ms`; gyro fresh the same for `Gyro`; bias calibrated once the
  `BiasCal` observation function has reported ready (latched until reset).
  Vision alive is set when the newest field snapshot published observations,
  which only a field cycle that processed a new camera frame does, so the bit
  can clear between frames. The Pico link bits (4 to 6) need `<Pico>` on the
  Publishing; the stationary bit (7) needs a model with a stationary window.
- `calibration` is the bias calibration state; `profile_state`,
  `profile_reason`, `profile_detail` and `profile_id` the robot profile status.
- `map_id` and `estimate_id` name the current field documents, 0 when the
  publisher has no `Field`.

The state block carries no landmark. Every configured object travels in the
field documents.

### Field documents

```xml
<Publishing type="brain_link">
    <Serial resource_id="brain_uart"/>
    <Field resource_id="override_field" estimate_period_ms="200"/>
</Publishing>
```

`Field` names a `field_map` resource; `estimate_period_ms` defaults to 200 and
must be positive. Without a `Field`, `map_id` and `estimate_id` are 0 and
READ_DOC answers `Unavailable`. The layouts are in `translaGATR/link_documents.h`;
the builder is [field_documents](../src/impl/publishing/field_documents.h).

**Map document** (READ_DOC kind 1). It is built once, when the publisher is
built, from the planning data (see
[field assets](field_assets.md#planning-data)):

- every Landmark: kind landmark; flags estimated and reference, plus obstacle
  with a CollisionBox;
- every Obstacle: kind fixed; flag obstacle with a CollisionBox;
- records sorted by wire id, with the nominal pose and the box;
- the Boundary and the revision in the header.

`map_id` is its CRC-32. The build fails with a clear error when the field has
no `revision`, no `Boundary`, a landmark without `wire_id`, more than 128
objects, or a value outside its wire range. Inspection's `hello` shows the same
planning data per field (boundary, boxes, wire ids, revision and `map_id`), and
the viewer draws it; see [inspection](inspection.md).

**Estimate document** (READ_DOC kind 2). The reporting cycle builds it from the
newest field snapshot and the robot state. It holds one record per map object,
in map order:

- **Observed.** A valid observed entry whose evidence came from the robot's
  current odometry epoch.
  - The pose is `T_field_odom * T_odom_object` under the robot's current
    anchor.
  - `age_ms` is cycle time minus the last observation time, clamped, as of
    the snapshot.
- **Nominal** (map pose, valid, age 0) for everything else:
  - fixed obstacles;
  - landmarks with no entry, as with `noop` world estimation;
  - entries still at the field-map seed;
  - invalid entries;
  - observations from another odometry epoch.

The header carries `map_id`, `estimate_id`, `odometry_epoch` and
`anchor_revision`: the robot frame the poses are expressed in, the same pair the
state block reports.

**Taking snapshots.**
- A new snapshot (`estimate_id` + 1) is taken when a record's source,
  validity or wire pose (mm, cdeg) changes, or when the epoch or anchor
  changes.
- At most one is taken per `estimate_period_ms`; the first is taken at once.
- A growing age alone takes none, so the Brain adds the time since it
  received the snapshot.
- The last three snapshots are retained. Ids count up per publisher and are
  never reused, even across a reset.

**READ_DOC.**
- `doc_id` 0 means the current document.
- The reply carries:
  - the resolved `doc_id`;
  - `total_len`;
  - the CRC-32 of the whole document;
  - the offset;
  - `min(max_len, 96, remaining)` bytes.
- `Stale` when `doc_id` is no longer retained. The Brain restarts from the
  newest.
- `InvalidArgument` for an unknown kind, `max_len` 0, or an offset at or past
  the end.

Transfers use the ordinary request/reply slot, so the Pi never transmits
unasked. A reply write can take up to about 13 ms of the estimation worker's
cycle.

Heading is the landmark's full field orientation. The Brain composes its own
offset onto this pose exactly once. Configured targets stay on the Pi.

Ages are Pi host clock differences taken at the Pi cycle start. The Brain adds
its measured round trip, so its total is an upper bound only to within the Pi
cycle processing time (a few milliseconds).

Inspection's `heading_error_deg` is the wrapped difference between estimated and
nominal object heading, in `(-180, 180]` degrees. It is not sent to the Brain,
and there is no configurable 90-degree symmetry period. See
[coordinates](coordinates.md).

Replies go out through the publisher's `Serial` link: the half-duplex RS-485
link or `pros_usb_link` over USB. See
[setup](setup.md#connect-the-brain) for the RS-485 configuration and hardware
checks.
