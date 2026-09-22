# Localization fusion

`weighted_planar_fusion` is the state estimator that blends the supported
robot observations by explicit uncertainty and carries a pose covariance.
`planar_motion_integrator` remains available; it applies the same rules
without a noise model. Both fill the single `Estimator` slot of
`Localization` and produce the authoritative `RobotState`; there is no second
fusion layer after them.

```text
SensorMap
  -> tracking_wheel_motion   BodyMotionIncrement (d_w, dtheta_w, coupling J)
  -> imu_heading_increment   HeadingIncrement    (dtheta_g)
  -> attitude_reference      AttitudeObservation (tilt)
  -> RobotObservationMap
  -> weighted_planar_fusion  RobotState + odom_covariance + history
```

## Configuration

```xml
<Estimator type="weighted_planar_fusion">
    <Motion observation_id="tracking_motion">
        <Noise translation_floor_m="0.0005" translation_per_m="0.02"
               rotation_floor_rad="0.001" rotation_per_rad="0.02"
               rotation_per_m="0.01"/>
    </Motion>
    <Heading observation_id="imu_heading" interval_tolerance_ms="20">       optional
        <Noise angle_random_walk_rad_per_sqrt_s="0.002" bias_rad_per_s="0.0005"/>
    </Heading>
    <Attitude observation_id="attitude" max_age_ms="250"/>                  optional
</Estimator>
```

Exactly one `Motion`, at most one `Heading`, at most one `Attitude`. Every
reference must be a declared localization output of the matching payload;
anything else, including landmark evidence, fails the build. `Noise` is
required where present and every attribute is required: an absent value is
an error, never a default. Floors must be positive. All values are one sigma.

| Attribute | Units | Meaning |
|---|---|---|
| `translation_floor_m` | m | translation noise per step, per body axis, at zero travel |
| `translation_per_m` | m/m | translation noise proportional to the distance of the step |
| `rotation_floor_rad` | rad | wheel-solved rotation noise per step at zero motion |
| `rotation_per_rad` | rad/rad | rotation noise proportional to the solved rotation |
| `rotation_per_m` | rad/m | rotation noise proportional to the distance of the step |
| `angle_random_walk_rad_per_sqrt_s` | rad/sqrt(s) | gyro angle random walk, must be positive |
| `bias_rad_per_s` | rad/s | residual gyro bias after the model's calibration |

The checked-in values in `config/shared/pipelines/synthetic_fusion_demo.xml`
are placeholders. Nothing on the robot has been measured; the numbers are
assumptions in the stated units. Replace them from stationary and known-path
recordings before trusting the covariance.

## Algorithm

Per accepted step over an interval `dt` with body-frame wheel translation
`d_w`, wheel rotation `dtheta_w`, and, when present, gyro rotation `dtheta_g`
over the same interval:

```text
s_d^2 = floor_m^2 + (per_m |d_w|)^2                          per body axis
s_w^2 = floor_rad^2 + (per_rad |dtheta_w|)^2 + (per_m |d_w|)^2
s_g^2 = ARW^2 dt + (bias dt)^2

w       = s_w^2 / (s_w^2 + s_g^2)
dtheta  = dtheta_w + w (dtheta_g - dtheta_w)                 precision weighted
var(dtheta) = s_w^2 s_g^2 / (s_w^2 + s_g^2)

d       = d_w + J (dtheta - dtheta_w)                        J = d(d)/d(dtheta)
var(d)  = s_d^2 I + J J' w^2 (s_w^2 + s_g^2)
```

`J` is the geometric coupling the wheel model publishes with every
increment: for wheels with unit directions `U` and lever arms `k`,
`d = (U'U)^-1 U' (m - k dtheta)`, so `J = -(U'U)^-1 U' k`. Revising the
rotation with independent evidence re-solves the translation through the
same measurements; a rear wheel's travel that the wheel-only solve blamed on
rotation becomes translation when the gyro says the robot turned less.
Without a coupling the published translation stands.

The body increment `(d, dtheta)` becomes the chord of the constant-curvature
arc `l = C(dtheta) d`, with the Jacobian of `C` applied to the covariance,
and enters the odometry pose as

```text
x' = x + R(h) l,  h' = h + dtheta
P' = F P F' + G Q G'      F = [[1, 0, -g_y], [0, 1, g_x], [0, 0, 1]],  G = [[R(h), 0], [0, 1]]
```

`F` carries the coupling of heading uncertainty into position: after a
straight run, lateral uncertainty grows with the heading variance times the
distance, which is what dead reckoning actually does. The covariance starts
at zero at the odometry origin, which is exact by definition, and restarts
there on reset. A placement re-anchors the field frame and leaves the
odometry covariance alone.

Cases:

- Motion with rotation, no usable heading: wheel rotation with `s_w^2`.
- Motion with rotation and an aligned, independent heading: the weighted
  step above. The diagnostic reports the gyro weight.
- Motion without observed rotation (`has_rotation` false) and an aligned
  heading: the heading supplies the rotation, and through `J` the
  translation `d = d_w + J dtheta_g` with the cross term `J s_g^2`. Without
  a heading the step is refused; nothing is fabricated.
- No motion: hold, effective time unchanged, attitude still folded.

## Rules preserved from the integrator

- One source clock per motion; device stamps map to the host clock through
  the receipt-based `DeviceToHostClock`; an attitude on another device never
  borrows the wheel clock offset.
- A heading is fused only when its `[startAt, endAt]` matches the motion's
  within `interval_tolerance_ms`. A mismatched heading that ends at or before
  the motion is rejected; one that ends later stays pending for a later
  motion, so a gyro increment that spans two wheel intervals is never
  applied to the first.
- A heading that shares a source with the motion is rejected with a
  diagnostic and never counted twice. A wheel profile with a
  `HeadingConstraint` already folds that gyro; configuring the same gyro as
  a `Heading` on top does nothing but log the rejection, and the two-wheel
  profiles keep working unchanged.
- Source time regression bumps the odometry epoch and integrates nothing.
  A repeated effective time is rejected as already consumed. Every offered
  observation is accepted or rejected exactly once through `settle`.
- The attitude is tilt only. Its yaw is discarded; the planar heading comes
  from wheels and gyro. It ages on its own host time and falls back to an
  explicitly assumed level attitude.
- `confidence` stays a producer score. The covariance is `odom_covariance`
  with `has_covariance`; the inspection snapshot carries it.

## Not modeled

- Correlation between wheel translation and wheel rotation inside one solve
  (the real `(A'A)^-1` structure); the two are modeled independent in the
  body frame with the coupling `J` re-solving translation.
- Correlation between consecutive steps beyond what the pose covariance
  carries (no bias state; gyro bias is a per-step term).
- Any correction of the pose from landmarks. Landmark evidence is derived
  through this pose and must not be fed back as a robot observation; the
  build refuses its payload.
- More than one motion or heading contributor.

## Verification

`tests/localization_fusion_gtest.cpp` covers the expected weighted result,
uncertainty-dependent influence, the coupling re-solve, partial
observations, the covariance recurrence against an independent restatement,
shared sources, mismatched intervals, repeated samples, dropouts, holds,
time regression, reset, placement, tilt-only attitude, the real wheel and
gyro models through `make_localization` (three wheels plus an independent
gyro, and two wheels with the constraint plus the same gyro), and the
checked-in `config/demo/synthetic_fusion_demo.xml` against the rig's truth.
None of it establishes the noise values; those are placeholders.
