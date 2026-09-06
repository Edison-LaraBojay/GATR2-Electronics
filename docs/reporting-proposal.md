Reporting proposal, revised 2026-09-06.

The user supports Pi-side temporary tracking and reporting of one selected
landmark alongside robot localization, without propagating a live field of other
objects. They are now considering how physical goals, their scoring faces, and
AprilTag observations should be modeled and associated, and whether restricted
image searches or fewer pose solves justify their cost and accuracy tradeoffs.
This document records that direction and remaining recommendations. It is not an
implemented API or wire format.

The product is one VEX sensing interface combining multiple physical sensors.
Robot localization is the baseline capability. Landmark sensing primarily supports
final physical alignment, including seeing a face with a forward camera and then
turning away to score with the rear. It is not intended to correct field localization
or take over autonomous behavior.

For that use case, the recommendation is for Navigatr to maintain a temporary
estimate of only the selected stationary landmark or face:

1. The Brain selects the physical reference it wants information about.
2. No measured landmark estimate exists until an observation is accepted and
   associated with that reference.
3. Accepted observations initialize or update that estimate.
4. Between observations, robot motion updates where that reference lies relative
   to the robot, assuming the landmark remains stationary.
5. Clearing or changing the selection discards the old selection's estimate.
   Motion-reference discontinuities require explicit invalidation or conversion.

One active selection is the initial scope, while each estimate remains
self-contained and explicitly identified. Tracking state
belongs to that landmark; the estimate's meaning does not depend on being the only
one in existence. One active selection is an initial capability limit, not a
permanent assumption to spread throughout unrelated contracts.

Simultaneous request sets, per-landmark cancellation, scheduling and capacity
policies can wait for a concrete behavior needing multiple tracks. Multiple tags
or cameras contributing evidence about the same selected landmark are a different
capability and do not imply multiple independent landmark tracks. Adding multiple
selections later can reuse the estimate definition, but still requires explicit
selection, workload, association, and reporting decisions then.

For a rigid goal, the assistant recommends distinguishing three definitions:

| Definition | Meaning |
|---|---|
| Physical landmark/object | The goal that moves as one rigid object, with a convenient local origin and optional nominal field placement. |
| Named face/reference frame | A scoring face's defined point and direction fixed to that object. These names stay attached to the physical faces as the object rotates. |
| Physical tag mount | A particular marker instance with its printed family/ID and measured position/orientation relative to the object. Printed ID alone may not uniquely identify the mount. |

The Brain would select, for example, Goal A / south scoring face. Correctly
associated tags on both its west and south sides could support the same estimate.
There is one live rigid pose, from which the selected face pose is derived. Other
faces are fixed geometric references, not independently propagated tracks.
The object origin may be its center for convenience, but estimating a separate
center first is not required; the selected face can itself be the common pose
reference used by the implementation.

Known mounting geometry converts a detected tag pose into a measurement of the
same object/reference pose. Multiple visible tags must be brought into that common
reference before combining them. Their raw headings differ due to mounting and
must not simply be averaged. Another implementation is to fit one object pose
jointly to the image corners from all correctly associated tags, using their known
3D layout. This approach is described in
[OpenCV's marker-board pose documentation](https://docs.opencv.org/4.13.0/db/da9/tutorial_aruco_board_detection.html).
Repeated printed IDs still need physical-mount association before assigning those
corners to the model; a generic ID lookup does not solve that ambiguity.

This grouping is justified only if tag mounts and faces maintain their relative
geometry to the accuracy needed for scoring. Independently moving or sufficiently
flexible faces may need independent definitions or estimates. An indirectly
estimated south face inherits the calibration and rigidity errors of the west
tag used to infer it. Last accepted observation time means the last evidence
contributing to the estimate, which need not directly depict the selected face.

One selected track is primarily a scope/state decision, not a claim that camera
computation is halved. AprilTag detection exposes the decoded ID and image corners
before a separate metric pose solve; see the
[detector interface](https://raw.githubusercontent.com/AprilRobotics/apriltag/master/apriltag.h)
and [pose-estimation usage](https://github.com/AprilRobotics/apriltag#pose-estimation).
Irrelevant IDs can sometimes be rejected before solving metric poses. Repeated
IDs may need projected-image expectations, pose hypotheses, or a joint geometric
fit to settle association. Image pixel position is not robot-relative metric
position, and image-plane tag rotation is not by itself the goal's planar yaw.

The detector still searches the image/region it is given. Predicted search regions
are a possible later optimization with acquisition/reacquisition coverage tradeoffs.
Detection, pose estimation, association, and tracking must be profiled separately
on the actual capture settings before asserting where computation is spent.
Maintaining other objects' live poses is unnecessary, but considering their static
mount definitions as alternative associations may be necessary to reject a wrong
match. Selecting a goal must not force every matching printed ID onto that goal.

The user's proposed camera optimizations remain implementation options:

| Option | Potential benefit | Constraint |
|---|---|---|
| Fixed image crop before detection | Fewer searched pixels, preserving resolution within the retained image. | Validate the full working distance, tag height, mounting angle, and robot pitch/roll envelope, including complete tag boundaries and margins. Camera height alone does not establish a safe strip. |
| Predicted region before detection | Search near the selected object's projected tag geometry. | Include uncertainty in robot pose, goal placement, timing, and calibration; expand after misses and retain wider acquisition/reacquisition. A predicted region is not proof of identity. |
| Narrower optical field of view | More pixels on relevant geometry at the same image dimensions. | Does not inherently reduce pixel-processing work; sacrifices coverage. A lower-resolution capture mode is a separate choice. |
| Pose from one associated tag | Avoid additional independent metric pose solves. | Detection, decoding, and corner extraction have already occurred; may lose useful constraints or consistency checks. Measure whether the saved stage matters. |

Before acquisition, a nominal layout and approximate robot pose can supply a broad
search prior. After acquisition, the retained selected estimate can predict image
locations through robot motion. Margins must allow the discrepancies vision is
intended to measure, and should cover useful mounts on other faces as appropriate.
Masking a full-size image or resizing a crop back to full size does not provide
the same input-pixel reduction as passing a smaller image to the detector.

An unscaled software crop changes pixel coordinates, not physical camera geometry.
Under the standard camera projection model, subtract the crop origin from the
principal point, or transform detections back into the original image coordinates
before pose estimation. Resizing additionally scales the relevant intrinsics.
See [OpenCV's camera model](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html).
Different physical optics or capture modes need their calibration relationship
established rather than assuming the same transformation applies.

Choosing a single pose source should consider association reliability, corner
quality, view geometry, and mounting calibration, not merely decoded ID confidence.
Using several associated tags in one rigid pose fit remains an alternative to
independent per-tag solves. The recommendation is to profile detection and geometry
separately, evaluate a conservative fixed crop, and add prediction-based cropping
if measured savings justify it. Compare acquisition reliability, false matches,
capture-to-estimate latency, and scoring alignment error rather than FPS alone.
These options do not require a live field model or a different Brain interface.

The Brain owns desired standoff, contact offsets, front/rear approach, destination
construction, motor control, and whether the estimate is suitable for the action.
Navigatr's retained value is a physical landmark estimate, not a robot destination.
This does not require the existing target acquisition/locking implementation.

| Information exposed | Meaning |
|---|---|
| Robot `x, y, heading` | Current estimated pose of the robot reference point in its initialized navigation coordinates. |
| Selected landmark/reference identity | Which identified physical object or face the retained estimate describes. |
| Landmark `delta_x, delta_y, delta_heading`, when an estimate exists | Estimated current geometry of that reference relative to the robot origin and axes. |
| Estimate-effective time | The instant those reported coordinates describe. Robot and landmark relative estimates should refer to a coherent instant. |
| Last accepted observation time | When actual sensor evidence last updated the landmark estimate. Publishing another propagated estimate does not advance this time. |
| Coordinate reference identity | Allows retained data to be invalidated or explicitly transformed across a restart, reset, or redefinition of navigation coordinates. |

The core needs explicit absence before acquisition or when propagation is no
longer supportable. It does not need a large set of searching/locked/lost modes.
Last observation time lets the Brain distinguish recent evidence from information
carried through a blind maneuver without requiring an observed/propagated enum.
A new-observation identity may also be useful to consumers, with delivery and
deduplication details left to the library/transport.

Observation-only events were an earlier alternative and could remain an optional
diagnostic view. The currently supported direction keeps short-term propagation
on the Pi. Executing the same propagation math on the Pi instead of the Brain
does not inherently reduce physical drift; the reason to prefer the Pi is
ownership of synchronized measurements and history.

The physical coordinate definitions remain:

- Robot pose refers to a configured physical origin in initialized navigation
  coordinates. This is an estimate, not a guarantee of true field coordinates.
- Landmark delta_x is forward and delta_y is left from that robot origin.
  Sensor mounting geometry has already been accounted for.
- Delta_heading is the landmark's defined direction relative to robot-forward.
  The deltas are a relative pose, not differences between consecutive samples.
- A landmark or face has a fixed definition of its reference point and axes.
  No desired scoring distance or mechanism offset is embedded in that definition.

The proposed scoring-face convention remains +x inward through the face, robot
+x forward and +y left, and counterclockwise-positive yaw. Front-first square
alignment then reports relative yaw zero; the Brain chooses other approach
orientations. This has not been approved as a replacement for existing outward
approach frames.

A local propagation implementation can retain one pose:

```text
landmark_in_local =
    robot_in_local_at_observation * landmark_relative_at_observation

landmark_relative_now =
    inverse(robot_in_local_now) * landmark_in_local
```

Here multiplication means pose composition, including rotation of translations.
The observation uses robot pose at its actual measurement time, not at receipt.
No measured global landmark field position is needed. For an ideal in-place
180-degree turn, a landmark originally one meter ahead becomes one meter behind.
Its physical position has not changed; the robot-relative coordinates have.

Three distinct kinds of reference information should stay separate:

| Reference information | Purpose |
|---|---|
| Landmark catalog | What can be recognized, physical identities, marker-to-face geometry, and named reference frames. |
| Optional nominal seasonal layout or request hint | Expected placement or direction to help aim, constrain association, or distinguish repeated identities. |
| Temporary selected-landmark estimate | The one live measured estimate carried through the current interaction. |

Recognizing a uniquely identified marker does not inherently require a field map.
Expected placement may be necessary for a particular association strategy when
objects/faces are otherwise ambiguous. Nominal placement is a prior, not visual
evidence; it must not silently initialize a supposedly observed track. If the
requested face was never acquired, propagation cannot recover it from nothing.

Blind-approach accuracy is not established by the current repository. A common
rigid offset in navigation coordinates cancels when using the same coordinates
for robot and landmark. Given correct association, the remaining error comes
from the initial observation/calibration, motion-estimation error after that
observation, timing, and possible landmark movement. Pre-existing drift can still
affect association if nominal field expectations are used to identify the face.

As geometric sensitivity only, one degree of angular error at a one-meter range
corresponds to about 17.45 mm of sideways discrepancy; two degrees corresponds to
about 34.90 mm. These are calculated from range times sine of angular error, not
predictions of this robot's drift or complete scoring error.

Retention limits must be tied to measured motion accuracy. Time alone does not
describe a blind interval: turn angle, travel distance, speed, surface interaction,
and sensor interruptions matter. The general role of local odometry and the need
to limit retained data as error accumulates are described in
[REP 105](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst).

The relevant physical validation is the actual maneuver: acquire at intended
distances/angles, stop visual updates at the intended last sighting, turn through
the planned angle, and back to the intended contact relationship. Measure final
lateral and angular error independently of the estimator over repeated runs,
including both turn directions and intended operating conditions. Compare that
error with the mechanism's tolerance. No acceptable tolerance, blind interval,
or measured acquisition success rate has been established in this discussion.

If that maneuver misses the required accuracy, moving propagation between
processors will not fix it. Calibration, acquiring closer to the turn, a shorter
blind approach, another useful viewing direction, or a mechanical alignment aid
are possible responses to evaluate against measured failures.

No firmware, runtime, or binary-protocol changes accompany this recommendation.
