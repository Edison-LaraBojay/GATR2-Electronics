# Coordinates and heading

Robot pose and reported landmark position use the same initialized field
coordinates. A landmark is an estimated physical reference, such as a goal origin
or a named scoring face. Its identity specifies exactly which point and axes
the report describes.

## Axes and units

| Frame | Origin | Positive axes |
|---|---|---|
| Field `F` | Inside bottom-left corner in the configured audience-view field drawing. | +x right on that drawing, +y toward its top, +z vertically up. |
| Robot `R` | One measured, fixed point on the chassis, conveniently near the nominal turning center. | +x robot-forward, +y robot-left, +z up. |
| Odometry `O` | Local origin established by localization. | Fixed local axes; its relationship to the field is an explicit transform. |
| Camera engineering `C` | Camera optical center. | +x looking forward through the lens, +y camera-left, +z camera-up. |
| Landmark/reference `L` | Configured physical reference point. | Fixed to that reference; nominal field orientation and mount transforms define the directions. |

This field convention is recorded in
[`config/override/field.xml`](../config/override/field.xml). Field directions do
not rotate with the robot or automatically flip with alliance. The field drawing
and definition identify the physical walls; screen orientation alone is not a
calibration. Goal centers in that definition are projected onto the field floor,
and their nominal frames align with the field axes.

Use meters and radians internally. Configuration and diagnostics may use degrees
when the unit is explicit in the field name. Heading is counterclockwise-positive
when viewed from above: zero points along +x, +90 degrees along +y, 180 degrees
along -x, and -90 degrees along -y. Wrap angles to `(-pi, pi]`, equivalently
`(-180 degrees, 180 degrees]`, including ordinary angular differences. A declared
object symmetry can give its orientation error a shorter period, as defined below.

The body-axis and rotation conventions follow
[REP 103](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst).
The field origin and wall directions are project configuration.

## Robot heading and landmark heading error

Robot `heading` is the full yaw of robot-forward in the field frame. Zero means
the robot points along field +x; initialization may specify any heading.

Internally, a landmark pose contains a full field yaw `yaw_F`. For a symmetric
object this is one representative of its equivalent orientations. Its reported
`heading_error` uses the orientation period declared by the reference definition:

```text
heading_error = wrap_period(estimated_yaw_F - nominal_yaw_F, period)
representative_yaw_F = wrap(nominal_yaw_F + heading_error)
```

`wrap_period` returns an equivalent angle in `(-period/2, period/2]`. Use a full
360-degree period for a uniquely oriented reference. A fully indistinguishable,
functionally interchangeable square can use 90 degrees. The period is part of the
static reference contract, not a property guessed from the current image.

The nominal orientation comes from that same definition. Positive error means
the representative is counterclockwise from nominal; negative means clockwise.
An ideally placed reference has zero error regardless of robot heading.

| Nominal reference yaw | Estimated reference yaw | Reported heading error |
|---|---|---|
| 0 degrees | +4 degrees | +4 degrees |
| +90 degrees | +94 degrees | +4 degrees |
| +179 degrees | -179 degrees | +2 degrees |

These examples use the full 360-degree period. All nine goal frames in the
configured field have nominal yaw zero, so their representative yaw and error
coincide near zero. Their nominal headings and repeated tag IDs do not establish
that every actual goal has four interchangeable sides.

The report is `(x_F, y_F, heading_error)`: x and y are estimated field positions,
not offsets from nominal placement. Because its third value is a residual, this
tuple is not itself a rigid pose transform. Reconstruct a full representative
yaw and honor the reference's symmetry and side-selection convention before
applying pose composition. Use the explicit name `heading_error` at the report
boundary and `yaw` or a fully framed pose internally.

This error is also distinct from bearing to a reference and from the robot's
alignment error. For example, an ideally placed goal can report zero error while
the robot faces away from it. The Brain derives a desired robot heading using
the selected face orientation and the mechanism's approach relationship, then
computes its control error against robot heading.

A reference estimate transformed using robot localization inherits localization
error. Its heading error is an estimated discrepancy from nominal, not an
independent measurement proving how far the physical goal rotated. Camera
calibration, visual error, and robot heading error may contribute.

## Symmetric objects and side selection

Consider a square with identical sensing features and interchangeable scoring
geometry on all four sides. Orientations `yaw`, `yaw + 90 degrees`,
`yaw + 180 degrees`, and `yaw + 270 degrees` describe the same observable and
usable arrangement. Report its smallest orientation error from nominal modulo
90 degrees, in `(-45 degrees, 45 degrees]`.

| Rotation from nominal | Equivalent reported error |
|---|---|
| +10 degrees | +10 degrees |
| -10 degrees | -10 degrees |
| +50 degrees | -40 degrees |
| -50 degrees | +40 degrees |
| +90 degrees | 0 degrees |
| -45 or +45 degrees | +45 degrees by the endpoint convention |

Forty-five degrees is a representation boundary, not a physical limit. At that
boundary, two nominal-side assignments are equally close. A small-rotation prior
can choose a convenient representative in normal operation; it cannot prove an
unobservable physical label. Two visible sides improve geometric constraints but
do not remove exact fourfold symmetry. Pose estimation commonly treats such
indistinguishable poses as equivalent; see the
[BOP symmetry-aware evaluation](https://bop.felk.cvut.cz/challenges/bop-challenge-2019/).

Apply this symmetry only when both the sensing geometry and the task permit it.
A distinguishable marker orientation, asymmetric mount, different mechanism, or
non-interchangeable scoring side can require a longer period or a separate
identity. Equal printed IDs alone are insufficient evidence of full symmetry.

For symmetric goals, prefer storing/reporting the object center and an orientation
representative. Derive candidate face points and normals from that geometry.
Distinguish two ways of requesting a side:

- A physical face identity follows one material face. Recovering it requires an
  observable distinguishing cue or adequately constrained tracking history.
- A geometric side is selected by a defined relation, such as the face oriented
  toward a particular approach. This can be useful without identifying which
  material face occupies that side.

Do not silently treat those meanings as interchangeable. The Brain can choose a
candidate face using its approach and keep that geometric choice through a
maneuver. A sensor-side face report likewise needs an explicit selector and
continuity policy. A concrete selected face has its own position and directed
normal; the object's 90-degree equivalence does not make every face point the
same point. Return ambiguity when the requested side cannot be chosen reliably.

The canonical error jumps from about +45 to -45 degrees at the boundary even
for smooth motion. Estimation must align equivalent orientation branches before
combining measurements, and control must preserve the selected face. Whenever
the object representative changes by 90 degrees, permute its local face/mount
labels consistently so an unchanged physical surface does not jump elsewhere.
Do not average or differentiate canonical errors across that discontinuity as
ordinary scalars. For example, +44 and -44 degrees are close modulo 90 degrees;
their arithmetic mean of zero is wrong.

For two visible tags, normalize their camera poses, evaluate consistent mount
assignments, and bring both measurements into one object-pose hypothesis. A joint
fit to their corners can also use the known mount geometry. Group hypotheses
that differ only by the declared object symmetry before deciding whether identity
is ambiguous. Competing physical goals remain distinct, as do pose solutions
that are not related by a valid symmetry. Expected field orientation is a prior;
it must allow the displacement being measured.

## Physical references and camera mounting

Define goal origins, scoring-face frames, and physical marker mounts separately.
A face's direction must be explicitly configured; a raw tag's normal is not
automatically the scoring approach direction. Several tags on one rigid object
can support the same object pose, from which its selected face is derived.
Do not average raw headings of differently oriented tag mounts.

Measure the camera's translation and full rotation relative to the robot origin.
The robot origin remains fixed on the chassis even when the instantaneous center
of rotation changes. A camera offset by `a` forward and `b` left has planar position:

```text
camera_x_F = robot_x_F + a*cos(robot_heading) - b*sin(robot_heading)
camera_y_F = robot_y_F + a*sin(robot_heading) + b*cos(robot_heading)
```

For example, with the robot origin at field `(1, 1)` and a camera 0.2 m forward,
the camera is at `(1.2, 1)` at robot heading zero and `(1, 1.2)` at +90 degrees.
The robot's reported position remains `(1, 1)` throughout that ideal turn.

For a planar mounting rotation, camera yaw also includes its mounting yaw. Use
full 3D transforms for actual mounting height, pitch, roll, and tag geometry
before projecting a valid reference estimate to the planar report.
If a camera actually articulates relative to the chassis, its robot-to-camera
transform must include that articulation at exposure time; a fixed calibration
alone is sufficient only for a rigidly mounted camera.

AprilTag optical coordinates have +x image-right, +y image-down, and +z forward.
Normalize those axes and tag conventions in the detector adapter before using
engineering transforms. See the
[AprilTag coordinate documentation](https://github.com/AprilRobotics/apriltag#coordinate-system).

## Initial pose and coordinate continuity

Use `T_A_B` to mean the pose of frame B expressed in frame A. Then
`T_A_B * T_B_C = T_A_C`; multiplication rotates translations as well as adding
orientation.

The initial pose command specifies `T_F_R`, the field pose of the robot origin.
Maintain smooth local odometry `T_O_R` and an explicit field anchor:

```text
T_F_O = initial_T_F_R * inverse(T_O_R_at_initialization)
T_F_R = T_F_O * T_O_R
```

The command is an approximate initial placement, not an independent proof of
true robot position. The field definition provides the nominal coordinate system
and geometry; localization estimates how the robot moves within it. Transform
observations into that same system using the estimated robot pose at measurement
time. Both translation and heading error in the initial anchor then contribute
to the inferred field poses of observed landmarks.

For example, suppose a robot is truly at x = 1.0 m but is initialized at x = 1.1 m,
with heading correct. A reference measured one meter forward is estimated at
x = 2.1 m although its true position is x = 2.0 m. Its relative distance remains
`2.1 - 1.1 = 1.0 m`. Association against a nominal reference at x = 2.0 m needs
enough tolerance for that 0.1 m discrepancy without confusing another object.
Initial heading error similarly affects the field representation through the
full rigid transform, rather than through a constant translation alone.

Association priors must account for approximate initial placement, subsequent
motion error, sensor/calibration uncertainty, and physical landmark displacement.
Reject or preserve ambiguity when competing objects cannot be distinguished.
After a reference is acquired, its measured cache entry can also supply a prior.
Correct relative alignment depends on correct association and consistent frames;
the shared-coordinate cancellation does not establish which nominal object was
seen or remove additional error accumulated after its observation.

An implementation may begin with coincident field and odometry coordinates, but
their initialization and reset meaning must remain explicit. Without a known
anchor, local motion can run while field reports remain unavailable. Nominal
field priors cannot be compared to arbitrary local coordinates.

Changing only the field anchor preserves local motion and increments an anchor
revision. Re-express robot and retained landmarks consistently under that anchor;
do not mix snapshots from different revisions. Nominal-error reports are then
recomputed against the field definition. A change in reported coordinates after
re-anchoring is not a new landmark observation. A discontinuity in local odometry
changes the odometry epoch and requires invalidation or a known conversion of
history and retained estimates. A changed nominal definition also requires an
explicit field-definition identity.

The distinction between continuous odometry and an externally defined reference
is also described in
[REP 105](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst).

## Measurement-time transforms

For a tag measured at capture time `t`, let `T_L_tag` be its associated mount in
the chosen landmark representative L. After detector-axis normalization:

```text
T_O_L = T_O_R(t) * T_R_C(t) * T_C_tag(t) * inverse(T_L_tag)
T_F_L = T_F_O * T_O_L
```

Retrieve `T_O_R(t)` from timestamped history, not from the latest state at
processing completion. If reporting a face S on the object, first derive
`T_O_S = T_O_L * T_L_S` and use that face's nominal orientation for its error.
`T_R_C(t)` is constant for a rigid camera mount. This composition includes the
camera's orbit around the robot origin and returns geometry referenced to that
robot origin. It does not require the robot origin to coincide with the camera.

For a stationary landmark, retain `T_O_L` and derive its field report using a
consistent anchor. Robot motion alone does not move its stored local or field
position. The robot-relative relationship at a later time is:

```text
T_R_L(now) = inverse(T_O_R(now)) * T_O_L
```

This derived relationship supports alignment even though the primary landmark
report uses field coordinates. In an ideal turn with no new observations, the
landmark's field position and heading error remain fixed, while its relative
position changes with the robot.

For example, a landmark at field `(2, 1)` is one meter ahead of a robot at
`(1, 1, 0 degrees)`. If the robot turns to 180 degrees at the same origin, that
landmark remains at field `(2, 1)` and is now one meter behind in robot axes.

A common rigid coordinate error cancels in a relative transform constructed
consistently from robot and landmark poses. Observation error and motion error
since acquisition remain. Expressing a landmark in the field frame does not make
it an independent field-localization reference.

For a heading example on the correct face branch, suppose true robot heading is
30 degrees but localization reports 40 degrees. A measured reference direction
30 degrees clockwise from robot-forward produces estimated field yaw 10 degrees.
Subtracting the reported robot heading recovers `10 - 40 = -30 degrees`, the
correct relative direction, even though the inferred field orientation is off
by 10 degrees. This cancellation requires consistent frames and times; it does
not cancel sensor mounting errors or additional drift after the observation.
