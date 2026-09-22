# Field estimates, targets, and reporting

Field estimation tracks configured physical objects in the same field coordinate
system used for robot localization. Target resolution separately turns configured
navigation intent into a desired robot pose. The browser exposes both; the Brain
publisher's object fields depend on which of these outputs is available.

## Definitions and retained state

`field_map` holds immutable nominal landmark poses, physical tag mounts, named
approach frames, and optional display geometry. The Override definition contains
nine goals. Visual metadata such as boxes, colors, and wall dimensions is used by
the viewer and does not enter localization or association.

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

`target_set` declares targets selected by command wire ID. `configured_targets`
resolves and retains one active TargetState:

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
body pose in odometry coordinates, re-expressed in the field for output. It is not
the landmark's physical pose. The Brain performs movement control.

## Brain output

The [`vex_brain` publisher](../src/impl/publishing/vex_brain.cpp) uses the shared
[pose-frame codec](../../../common/frames.h). Robot x/y/heading always describe the
robot origin in the field frame. When the command requests an object:

1. A matching active, latched, non-cancelled target supplies the desired robot
   pose. A vision-locked target sets `kStatusObjObserved`.
2. Otherwise, a matching configured `FieldObject` mapping supplies that object's
   retained absolute pose. The entry's `observed` flag controls
   `kStatusObjObserved` for this path.
3. If neither is available, `kStatusObjValid` remains clear.

Object heading is a full field orientation on both paths. Consumers must use the
profile's target/object mapping to interpret the object fields. The separate
optional landmark list contains relative dx/dy, bearing, and quality for a
compatible configured `LandmarkAssociationSet`; tag-mount pose evidence is a
different payload and cannot be bound to that list.

Inspection's `heading_error_deg` is the wrapped difference between estimated and
nominal object heading, in `(-180, 180]` degrees. It is not sent as
`obj_heading_cdeg`, and there is no configurable 90-degree symmetry period. See
[coordinates](coordinates.md).

The command parser supports stream, placement, and object requests, but the
Linux serial resource currently holds its optional RS-485 DriverEnable high
rather than switching direction around writes. The checked-in physical HAT path
therefore needs receive/turnaround work before bidirectional command traffic can
be used on that shared link.
