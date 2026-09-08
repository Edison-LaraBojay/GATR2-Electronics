# Landmark estimation and reporting

The Brain requests one physical landmark or named scoring face. Navigatr reports
its estimated position in the same field coordinates as the robot, together
with its rotation away from nominal field orientation. The Brain uses that
geometry for alignment and constructs its own desired robot/contact pose.

The recommended design keeps the nominal field definition and a separate cache
of accepted measured object estimates. Observations update that cache; reporting
selects the one requested entry. Selection also prioritizes processing work.

The [nested estimation pipeline](architecture.md#landmark-estimation-pipeline)
defines preparation, extraction, candidate generation, geometry estimation,
association resolution, and state estimation as separate contracts. Each may
contain a series of interchangeable child implementations. Its final output is
a LandmarkStateMap snapshot; reporting selects and formats a reference from that
snapshot independently of sensor processing.

## Static field definition and measured state

Keep these concepts separate:

| Information | Contents | Updated by observations? |
|---|---|---|
| Field definition | Coordinate convention, identifiable objects, nominal poses, physical tag mounts, named reference geometry, and declared orientation symmetries. | No. Configuration supplies nominal facts and calibration. |
| Robot state | Estimated robot pose, field anchor, motion history, timing and reference identities. | By the selected localization implementation. |
| Measured landmark cache | Accepted geometric evidence by object identity, retained pose representative, provenance, and observation time. | Yes, when associated evidence is accepted. |
| Selection | The one reference requested for reporting. | No. Brain commands change it. |

Nominal placement supports search, association, and the heading-error baseline.
It does not initialize a measured estimate or acquire a fresh observation time.
An unobserved reference is unavailable as a measured report. The Brain may choose
to use nominal geometry separately for its behavior.

The selected identity must specify whether its position is a goal origin or a
particular face. A side selector must distinguish a material face identity from
a geometric side defined by the approach. Indistinguishable symmetric faces
cannot receive asserted physical identities without supporting evidence; see
[symmetry and side selection](coordinates.md#symmetric-objects-and-side-selection).
Different mounts on one sufficiently rigid goal may contribute
to a single object estimate. Derive face poses from that estimate and their fixed
geometry. Independently moving or flexible references require an appropriate
model; neither rigidity nor stationarity is guaranteed by a reference ID.

## Report contract

The following are semantic fields, not a binary layout:

| Information | Meaning |
|---|---|
| Robot `x_F, y_F, heading` | Estimated field position of the fixed robot origin and full field orientation of robot-forward. |
| Selected reference ID and request generation | Which physical reference the report answers and which request it belongs to. |
| Landmark `x_F, y_F` | Estimated field position of the selected reference point. |
| Landmark `heading_error` | Rotation from nominal under the reference's declared orientation period: ordinarily 360 degrees, or 90 degrees for a fully interchangeable square. |
| Estimate availability | Whether accepted evidence and the required coordinate/time support exist. |
| Estimate-effective time | The time the estimate describes, including any stated stationary carry-forward. |
| Last accepted observation time and source identity | Actual measurement provenance; republishing or selecting a cached estimate does not refresh it. |
| Field definition, anchor revision, and odometry epoch | The coordinate context needed to interpret the estimate and detect incompatible snapshots. |

See [Coordinates](coordinates.md) for the angle convention and reconstruction of
a full orientation representative. The nominal definition, symmetry period, and
any side-selection rule must be available to a consumer using the residual angle.
Never pass `(x_F, y_F, heading_error)` directly to a generic pose-composition
function without restoring nominal yaw and a consistent reference interpretation.

One report can contain both robot and selected-landmark state even when their
pipelines run at different rates. Preserve their effective times or reconcile
them at a supported time; packet transmission time is not measurement time.
Fixed validity/status bits describe the report. An object ID identifies the
requested reference; objects do not need dynamically assigned mask bits.

## Retention policy

Use a sparse cache of confidently measured objects, indexed by physical object
identity, and report at most one selected reference. Cache a pose when the
processing pipeline has accepted evidence for it, including incidental objects
when the configured processing budget permits. Selection does not erase other
accepted entries or oblige the detector to cover the entire field.

Each entry needs observation time, source provenance, coordinate identities,
validity/uncertainty, and a reuse policy. A previous observation can be useful on
selection, but it does not count as a current sighting. Selecting an unusable or
unobserved entry reports unavailable until valid evidence arrives. Clearing the
request stops the landmark report while retained entries continue to age.

The configured field has nine goals. Nine triples of 64-bit x, y, and yaw values
occupy 216 bytes before metadata and container overhead. Retaining those poses
is not a substantial memory concern on the Pi. This calculation is not a measured
allocation size or a camera-performance result.

For stationary objects, a cache does not need a propagation loop over every
entry on every robot update. Retain poses in a common local frame, update entries
when evidence arrives, and derive the selected entry's field or robot-relative
representation when needed. Each entry's age still grows while unseen.

The cache supports reuse without requiring a whole-field prediction pass. Keeping
only the selected estimate would reduce lifecycle scope but lose prior sightings;
its primary benefit is simplicity, not substantial RAM savings. Do not create
live valid entries merely because an object is listed in the static definition.

## Processing scope and association

Processing scope is independent of retention. Keeping a pose after it has already
been confidently estimated is inexpensive; obtaining and validating additional
poses may be the meaningful work. Measure those stages before choosing policy.

An AprilTag detector searches the image region supplied to it. Selecting one
landmark does not inherently avoid that search. Decoded IDs and image corners
can precede metric pose solving, so some irrelevant candidates can be rejected
before that solve. See the
[AprilTag detection and pose interfaces](https://github.com/AprilRobotics/apriltag#pose-estimation).

Repeated printed IDs still require object and mount association. Static alternatives
may need to be considered even when processing prioritizes the selected object.
A request is not proof of identity. Equivalent mount assignments on a declared
symmetric object describe one pose class; group those before applying ambiguity
gates. Competing physical objects or inequivalent poses remain ambiguous until
the evidence resolves them. Two visible sides improve geometric consistency but
cannot identify interchangeable physical labels under exact symmetry.

Multiple associated tags constrain one common object pose. Account for their
different mounting orientations before combining them, and align equivalent pose
branches before averaging. An apparently square layout is insufficient to declare
symmetry if the sensed features or scoring function distinguish its sides.

Known geometry and calibrated projection can predict useful image regions. Use
the camera pose at capture time and permit uncertainty in robot pose, nominal
placement, and calibration. A search margin must include the displacement the
measurement is meant to discover. Widen acquisition/reacquisition after misses.
Camera dimensions alone do not establish a safe image strip.

An unscaled software crop shifts the principal point by its crop origin, or its
detected corners can be mapped back to original image coordinates. Resizing also
scales the relevant intrinsics. These follow from the
[OpenCV camera model](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html).
Reducing detector input pixels, narrowing optics, filtering decoded IDs, and
skipping metric pose solves affect different stages. Compare latency and
acquisition reliability on the actual camera settings.

## Age, updates, and alignment

Accept fresh, correctly associated evidence to initialize or update an estimate.
Preserve capture time and source/frame identity. Duplicate data and out-of-order
results require explicit handling; an older completion cannot silently replace
a newer accepted observation or reset its age.

For the stationary-reference model, robot motion changes the relative geometry
without moving the retained landmark pose. Continue through a blind turn using
motion history and a consistent coordinate anchor. New visual evidence can update
the estimate on reacquisition. This is a maintained estimate, not an acquire-once
destination latch.

Age limits alone are insufficient to establish blind-alignment accuracy: travel,
turn angle, speed, sensor outages, and possible object motion also matter.
Selecting a cached estimate must apply its reuse/validity policy and retain the
original observation time. Clearing selection stops its report immediately.
Coordinate resets and field-definition changes invalidate or explicitly convert
retained state as described in [Coordinates](coordinates.md#initial-pose-and-coordinate-continuity).

The derived field landmark is correlated with the localization used to construct
it. Feeding that result back as an independent robot-position correction would
reuse that information. Fresh relative observations still constrain the
robot-landmark relationship; an independently trusted fixed reference could
support a separate localization implementation. The alignment pipeline does not
assume the selected goal is an independent absolute reference.

Validate delayed-image transforms, camera-offset turns, nonzero nominal headings,
equivalent square rotations, branch changes near 45 degrees, competing object
identities, selection changes, and cache age. Physical testing must
measure final lateral and angular alignment errors against the mechanism's
tolerance, including observation loss and reacquisition.
