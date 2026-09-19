# Coordinates and measurement time

Robot localization and field-object estimates share the configured field frame.
Camera measurements are transformed through the robot body at exposure time;
processing completion time is not the pose time of the image.

## Axes and units

| Frame | Origin and axes |
|---|---|
| Field `F` | For the Override definition, the inside bottom-left corner in the audience-view drawing; +x right, +y toward the top, +z up. |
| Robot `R` | A fixed chassis reference point near the nominal turning center; +x forward, +y left, +z up. |
| Odometry `O` | Continuous local localization frame, related to the field by an explicit anchor. |
| Camera engineering `C` | Optical center; +x through the lens, +y camera-left, +z camera-up. |
| Landmark `L` | Configured object origin, with tag mounts and approach frames expressed relative to it. |
| Canonical tag | Surface origin; +x outward toward the viewer, +z printed top, +y completing the right-handed frame. |

Field axes are fixed by [field.xml](../config/override/field.xml); they do not
rotate with the robot or flip automatically with alliance. The Override landmark
origins lie on the field floor beneath the goals.

Internal distances are meters and angles are radians. XML attributes explicitly
ending in `_deg` use degrees. Heading is positive counterclockwise from +x when
viewed from above: +90 degrees points along +y. `wrapAngle` returns `(-pi, pi]`.
The Brain codec converts positions to millimeters and headings to centidegrees.

Use `T_A_B` for the pose of frame B expressed in frame A:

```text
T_A_C = T_A_B * T_B_C
```

Composition rotates translation as well as adding orientation. Full camera/tag
chains use 3D rigid transforms; configuration Euler angles form
`Rz(yaw) * Ry(pitch) * Rx(roll)`. Project to planar pose only after composing the
chain. [se3.h](../src/math/se3.h) and [transforms.h](../src/math/transforms.h)
implement these conventions.

## Heading values

Robot heading and `FieldObjectState.pose.heading_rad` are full field orientations.
The existing Brain publisher also emits full field heading, whether its object
fields contain a target robot pose or a mapped field-object pose.

Inspection additionally calculates:

```text
heading_error = wrapAngle(estimated_object_heading - nominal_object_heading)
```

This is rotation away from the configured nominal orientation. It is not a full
pose orientation and must not be substituted into a transform. A square's
geometric symmetry does not change the implemented wrap period: there is no
configured 90-degree symmetry reduction. Association evaluates explicit tag
mount candidates and rejects close competing assignments. Printed IDs alone do
not determine which mount was observed.

## Robot origin and camera offset

All wheel positions and camera mounts are measured from the same robot origin.
That point remains fixed on the chassis even if the instantaneous turning center
changes. For a camera mounted `a` meters forward and `b` meters left, a level
robot has:

```text
camera_x_F = robot_x_F + a*cos(heading) - b*sin(heading)
camera_y_F = robot_y_F + a*sin(heading) + b*cos(heading)
```

If the robot origin stays at `(1, 1)` and the camera is 0.2 m forward, rotating
from zero to +90 degrees moves the camera from `(1.2, 1)` to `(1, 1.2)`. The
reported robot origin stays at `(1, 1)`.

`robot_frame_map` stores rigid robot-to-camera translation and rotation. The
AprilTag adapter converts optical camera axes (+x image-right, +y image-down,
+z forward) to engineering camera/tag axes at the perception boundary. Mounting
height, pitch, roll, and robot tilt are included before projection to 2D. There
is no runtime articulated-camera model; configured mounts are rigid.

## Field anchor and placement

Localization maintains `T_O_R` and `T_F_O` separately:

```text
T_F_R = T_F_O * T_O_R
T_F_O = requested_T_F_R * inverse(T_O_R_at_placement)
```

InitialPlacement and placement commands specify the approximate field pose of
the robot origin. Re-anchoring preserves local odometry and increments an anchor
revision. Retained observed objects are stored in odometry coordinates and
re-expressed under the new anchor. A hard odometry reset changes the epoch and
invalidates measurements tied to the previous local frame.

An initial placement error also appears in inferred field-object coordinates.
For example, if the robot's initialized x is 0.1 m too large, a correctly measured
object one meter ahead is also represented 0.1 m too far along x. Their relative
displacement still cancels that shared translation. Correct association and
consistent timestamps remain necessary; later odometry error does not cancel
merely because the coordinates share a frame. The association gates must allow
real placement error without accepting another object's mount.

## Exposure-time transforms

Given capture time `t`, camera mount `T_R_C`, observed tag pose `T_C_tag`, and
configured tag mount `T_L_tag`:

```text
T_O_L = T_O_R(t) * T_R_C * T_C_tag(t) * inverse(T_L_tag)
T_F_L = T_F_O * T_O_L
```

The actual chain lifts the robot pose into 3D using the exposure-time attitude.
The final object pose is planar. A retained stationary object needs no per-tick
robot-relative update. Its relative pose at a later time can be derived as:

```text
T_R_L(now) = inverse(T_O_R(now)) * T_O_L
```

A landmark computed with localization as an input is not an independent robot
position measurement. The current field estimator never feeds it back into
localization.

## Clocks, history, and attitude

Resource samples distinguish source measurement time from host receipt time.
Pico samples carry a device clock; localization maps accepted motion endpoints to
the host monotonic clock using upstream receipt information. A polling timestamp
cannot substitute for when the data arrived. Camera frames carry host exposure
time from the capture backend; see [camera setup](pi_camera_setup.md) for its
mapping and exposure-midpoint convention.

`RobotStateFeed` supplies synchronized current snapshots and timestamped lookups.
Pose history uses binary search and interpolation within configured gap and age
limits. It returns explicit failure statuses for unavailable history instead of
silently substituting the latest pose. Camera detections preserve frame sequence,
epoch, and exposure time throughout association and inspection.

Attitude samples have independent timestamps and history. The estimator combines
measured tilt with planar heading; attitude-only updates do not advance planar
pose history. Association's `Attitude` policy is either `require`, which rejects
missing valid tilt, or `assume_level`, which uses planar yaw and labels the
assumption. The current Pico firmware supplies gyro Z, not an attitude quaternion;
live tilt therefore needs the [firmware follow-up](attitude_firmware_followup.md).
The synthetic rig exercises both measured-attitude and assumed-level behavior.
