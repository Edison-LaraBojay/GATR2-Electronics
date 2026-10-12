# World estimation change (SPEC step 3): plan

Status: planned and prototyped. The worktree `S/wt-navi` was not modified.
The prototype ran on a scratch copy of ae457ee: 634/634 tests pass, which is
the 626 baseline names unchanged plus 8 new tests. The numbers are in
section 9.

- `S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`
- `R` = `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/restructure`
- Line numbers are at ae457ee. Post-move paths come from `R/plan/move_map.tsv`
  (13:55) and `R/scripts/split_system.py`.

## 0. Decisions in one screen

| # | Decision | Why |
|---|---|---|
| D1 | `FieldObjectState` keeps `pose` as the stored source value. It gains `std::optional<Pose2D> nominal` and `LandmarkError error`. | Keeps every existing pose bit-identical. The error is derived from the pose, never the other way round (SPEC: "error is derived from that derived field pose and the map nominal"). |
| D2 | The error is written only through `setFieldPose(obj, pose)` in landmark_estimator.cpp, at the 4 existing pose sites. | Pose and error can never disagree. No site that derives the pose today is missed. |
| D3 | One helper, `applyLandmarkError(error, landmark_nominal, target_nominal)`, in `pipeline/world_estimation/landmark_error.h`. It is written as target plus correction. | Zero error returns the target bit for bit (measured: 1,000,000 of 1,000,000 cases). The textbook form `N + d + R(o)` does not (844,683 of 1,000,000). |
| D4 | Target resolution: only `estimateTargetPose` (policy none) switches to the helper. `mapNominalTargetPose` and the acquire_once path stay as they are. | Only policy none reads the field estimate. acquire_once reads raw per-frame odometry evidence, and the timeout fallback reads the map nominal only. |
| D5 | Publishing (Brain wire) code is unchanged. | See 4.7. The wire derives its own pose from `T_odom_object` under the publish-time anchor (field_documents.cpp:245). Rebuilding it as nominal + error would change test outcomes and is not bit-exact. |
| D6 | Inspection reads `nominal`, `heading_error_deg` and `displacement_m` from the field state and drops its resource-store scan. | The output is bit-identical in every System in the repo, since each has exactly one field_map (proved by the new test and by grep). |
| D7 | The tag trace types move to `estimators/apriltag/tag_association_trace.h`. This is a pure move. | SPEC layout. `R/plan/move_map.md` ("Tag trace split") assigns it to this step. |
| D8 | `math/units.h` gets `using Meters = double; using Radians = double;`. | SPEC asks for plain double aliases. No such aliases exist today (grep of src: none). |
| D9 | Bit-exactness: pose, inspection values and wire are bit-identical. The policy-none target is equal within 7.1e-15 m and 1.9e-15 rad (measured), and all its tests use tolerance 1e-9 or wider. | Proved in section 5, measured in section 9. |

## 1. Inventory: where the estimated field-object pose lives today

### 1.1 Produced (all in `src/impl/world_estimation/landmark_estimator.cpp`, `LandmarkEstimator::run` 42-166)
| Site | Line | Today | Change |
|---|---|---|---|
| seed every mapped landmark | 49-61 (55) | `obj.pose.pose = decl.nominal` | `obj.nominal = decl.nominal; setFieldPose(obj, decl.nominal);` |
| odometry epoch changed: back to nominal | 70-79 (76) | `obj.pose.pose = decl->nominal` | `setFieldPose(obj, decl->nominal);` |
| anchor changed: re-express | 81-85 (83) | `obj.pose.pose = compose(robot.field_from_odom, obj.T_odom_object)` | `setFieldPose(obj, compose(...))` |
| fold accepted evidence | 136-164 (152) | same compose | `setFieldPose(obj, compose(...))` |
| first-evidence blend reads the pose | 143 | `compose(inverse(F), obj.pose.pose)` | unchanged |

### 1.2 Stored and passed through
- `src/state/field_state.h:40-60` `FieldObjectState`: `pose` (field frame, derived under `anchor_revision`), plus `T_odom_object`, `odometry_epoch` and `anchor_revision`. All kept.
- `src/impl/world_estimation/apriltag_world_estimator.cpp:166` (`out.field = in.previousField` on a fault) and `:201` (`landmarks_->run`).
- `src/impl/noop/noops.cpp:42` (noop passes the previous field through).
- `src/runtime/system.cpp:1106-1133` `fieldCycle` builds the next `FieldSnapshot` (`system.h:134`). `System::field()` is at `system.h:275`.
- `src/runtime/system.cpp:1066-1078` `reportingCycle` gives `field->field` to target resolution and publishing, together with `robot_`. `robot_` can carry a newer anchor than the snapshot.

### 1.3 Consumed
| Consumer | Where (ae457ee) | Reads | Change |
|---|---|---|---|
| association prior | impl/world_estimation/tag_mount_association.cpp:313-320 | `pose.pose` when valid, else nominal | none (pose unchanged) |
| target resolution, policy none | impl/target_resolution/configured_targets.cpp:76-91 `estimateTargetPose`, called at :137 | `pose.pose`, then `resolveFromLandmark(compose(inverse(F), P))` | helper (4.5) |
| target resolution, timeout fallback | configured_targets.cpp:93-105 `mapNominalTargetPose` | map nominal only | none |
| target resolution, acquire_once | configured_targets.cpp:244, :284 | `FieldObjectPoseEvidence::T_frame_object` (odometry frame), not the field state | none |
| Brain wire field documents | impl/publishing/field_documents.cpp:220-260, pose at :245 | `T_odom_object`, composed with the publish-time `robot.field_from_odom` | none (4.7) |
| brain link publisher | impl/publishing/brain_link_publisher.cpp:380-381 | `documents_->update(in.field, in.robot, in.now)` | none |
| inspection field_objects | inspection/inspection_document.cpp:1198-1209 (nominal scan), 1210-1252 (objects), 1230-1242 (error fields) | `pose.pose` minus a scanned map nominal | read from the state (4.6) |
| capture | src/capture/* | no field-object CSV. grep for FieldState, field_objects and fieldSnapshot in src/capture finds nothing. Its CSV streams are robot pose, Brain telemetry targets and path points. | none |
| viewer | spectaGATR/app.js:815-816, field_scene.js:1294, 1578 | JSON `heading_error_deg`, `displacement_m` | none (JSON unchanged) |
| docs | pi/naviGATR/docs/inspection.md:540, 637; landmarks.md:238-241; setup.md:686-687 | describe these fields | still true, no change needed |

### 1.4 Tests that assert on these today
- world_estimation_gtest.cpp. There are 16 tests; the LandmarkEstimator ones read `pose.pose` with a 1e-12 or 1e-9 tolerance (lines 341-472). The AprilTagWorldEstimator ones are at 566 and 618-628.
- vision_targets_gtest.cpp (20). Target x and y use 1e-9 (468-469, 730-731, 753, 876-878). Heading uses `fabs(h)` against pi (471), and latches compare `EXPECT_EQ` to themselves (484, 678-679). Field `pose.pose` uses 1e-9.
- field_documents_gtest.cpp (18). FieldEstimate.* builds `FieldObjectState` by hand with only `T_odom_object` set (`observedAt`, lines 80-88). The expected wire values come from `T_odom_object` (637-643).
- synthetic_rig_gtest.cpp:301-313 (`pose.pose`, loose tolerances) and :325-331 (trace).
- capture_transform_gtest.cpp:174-177 and :307 (trace).
- inspection_document_gtest.cpp:512 and parallel_wheels_gtest.cpp:703, 837 check only `"field_objects":[]`. No test asserts `heading_error_deg` or `displacement_m` values today. The new test 7.2(8) adds one.

## 2. Path map (current to post-move)

| ae457ee | post-move (move_map.tsv / split_system.py) |
|---|---|
| (new) | `src/math/units.h` (math/ stays in place) |
| (new) | `src/pipeline/world_estimation/landmark_error.h` |
| src/state/field_state.h | src/pipeline/world_estimation/field_state.h |
| src/impl/world_estimation/landmark_estimator.{h,cpp} | src/pipeline/world_estimation/landmark_estimator.{h,cpp} |
| src/payloads/field_object_evidence.h | src/pipeline/world_estimation/evidence.h |
| (new, split out of the above) | src/pipeline/world_estimation/estimators/apriltag/tag_association_trace.h |
| src/impl/world_estimation/tag_mount_association.{h,cpp} | src/pipeline/world_estimation/estimators/apriltag/tag_mount_association.{h,cpp} |
| src/impl/world_estimation/apriltag_world_estimator.{h,cpp} | src/pipeline/world_estimation/estimators/apriltag/apriltag_world_estimator.{h,cpp} |
| src/contracts/field_estimation.h | src/pipeline/world_estimation/contract.h |
| src/impl/target_resolution/configured_targets.{h,cpp} | src/pipeline/targets/resolvers/configured_targets.{h,cpp} |
| src/resources/target_set.h | src/pipeline/resources/target_set.h |
| src/impl/publishing/field_documents.{h,cpp} | src/pipeline/publishing/publishers/field_documents.{h,cpp} |
| src/inspection/inspection_document.cpp | src/debug/inspection/inspection_document.cpp |
| src/runtime/inspection_state.h | src/debug/inspection/inspection_state.h |
| src/runtime/system.cpp `publishDetectionFrames` (trace use at 1241) | src/debug/inspection/inspection_state.cpp (split_system.py `INSPECTION`) |
| src/core/ids.h, src/core/time.h | src/pipeline/framework/ids.h, src/time/time.h |
| src/state/pose_history.h | src/pipeline/localization/pose_history.h |
| tests/* | unchanged (tests/ does not move) |

If step 1 lands with different names, use the step 1 TSV. The edits themselves do not change.

## 3. New and changed types

### 3.1 `src/math/units.h` (new)
```cpp
// units.h
// Unit aliases.

#pragma once

namespace navigatr
{

using Meters  = double;
using Radians = double;

} // namespace navigatr
```

### 3.2 `src/pipeline/world_estimation/landmark_error.h` (new)
```cpp
// landmark_error.h
// A landmark's error from its map nominal pose, in field axes. (0, 0, 0) is
// the field as built; the map's heading zero is only an authoring convention.

#pragma once
#include <cmath>

#include "math/angles.h"
#include "math/transforms.h"
#include "math/units.h"

namespace navigatr
{

struct LandmarkError {
    Meters  dx     = 0.0;
    Meters  dy     = 0.0;
    Radians dtheta = 0.0;
};

// estimated minus nominal
inline LandmarkError landmarkError(const Pose2D& nominal, const Pose2D& estimated) {
    return LandmarkError{estimated.x_m - nominal.x_m, estimated.y_m - nominal.y_m,
                         wrapAngle(estimated.heading_rad - nominal.heading_rad)};
}

// A nominal target relative to a landmark, moved with the landmark's error:
// its offset from the nominal landmark rotates by dtheta, then shifts by
// (dx, dy). Written as target plus correction so zero error is exact.
inline Pose2D applyLandmarkError(const LandmarkError& error, const Pose2D& landmark_nominal,
                                 const Pose2D& target_nominal) {
    const Meters ox = target_nominal.x_m - landmark_nominal.x_m;
    const Meters oy = target_nominal.y_m - landmark_nominal.y_m;
    const double c  = std::cos(error.dtheta);
    const double s  = std::sin(error.dtheta);
    return Pose2D{target_nominal.x_m + error.dx + (c - 1.0) * ox - s * oy,
                  target_nominal.y_m + error.dy + s * ox + (c - 1.0) * oy,
                  wrapAngle(target_nominal.heading_rad + error.dtheta)};
}

} // namespace navigatr
```
- Nominal plus error is `applyLandmarkError(error, nominal, nominal)`: the offset is 0, so the result is `(N.x + dx, N.y + dy, wrap(N.h + dtheta))`. There is no separate function for it. SPEC asks for ONE helper, and nominal plus error is that helper with the landmark as its own target.
- Location: it is the world estimation output type, so it sits at stage level ("A type used by several stages lives with its producer"). Targets include it directly.

### 3.3 `FieldObjectState` (`pipeline/world_estimation/field_state.h`)
- Add includes `<optional>` and `"pipeline/world_estimation/landmark_error.h"`.
- After `FramedPose2D pose;` add:
  ```cpp
      std::optional<Pose2D> nominal;   // estimator's map pose, empty when not in its map
      LandmarkError         error;     // pose minus nominal, field axes; zero without one
  ```
- Append to the header comment:
  ```
  //
  // A mapped entry also carries its map nominal and the LandmarkError of
  // its field pose from it, written together with the pose.
  ```
- Unchanged: `T_odom_object`, `odometry_epoch`, `anchor_revision`, and the re-anchor and epoch rules. `pose` is still derived under the current anchor exactly as today. `error` is derived from that `pose`, at the same moment. No aggregate is brace-initialized by position anywhere (grep `FieldObjectState{` in src and tests: none), so adding members is safe.
- Why `nominal` is stored and not looked up by consumers: the estimator's `<FieldMap>`, the association's `<FieldMap>`, target resolution's `<FieldMap>` and inspection's resource scan are four separate lookups today. Storing the nominal the error was measured from makes the error self-contained. It also keeps the policy-none algebra exact even when the maps differ (5.2, case b).

### 3.4 `setFieldPose` (landmark_estimator.cpp, anonymous namespace)
```cpp
// field pose and its error from nominal, always written together
void setFieldPose(FieldObjectState& obj, const Pose2D& pose) {
    obj.pose.pose = pose;
    obj.error     = obj.nominal ? landmarkError(*obj.nominal, pose) : LandmarkError{};
}
```
Used at the 4 sites in 1.1. Each call receives the exact expression that was assigned before, so `pose` is bit-identical. An object folded from evidence but absent from the estimator's map (possible only if the association's `<FieldMap>` differs; see 10) keeps `nominal` empty and `error` zero.

## 4. Consumers

### 4.5 Target resolution (`pipeline/targets/resolvers/configured_targets.cpp:76-91`)
```cpp
// Policy none: the nominal target moved with the landmark's committed
// error (zero while unobserved), never in-flight evidence.
Pose2D ConfiguredTargetResolution::estimateTargetPose(
    const TargetDecl& decl, const TargetResolutionInput& in) const {
    Pose2D        landmark_nominal;
    LandmarkError error;
    if (const auto* lm = field_->find(decl.landmark)) {
        landmark_nominal = lm->nominal;
    }
    const auto it = in.field.objects.find(decl.landmark);
    if (it != in.field.objects.end() && it->second.valid) {
        // an estimate without a map nominal is its own nominal
        landmark_nominal = it->second.nominal.value_or(it->second.pose.pose);
        error            = it->second.error;
    }
    const Pose2D T_field_target =
        applyLandmarkError(error, landmark_nominal, decl.resolveFromLandmark(landmark_nominal));
    return compose(inverse(in.robot.field_from_odom), T_field_target);
}
```
- Add `#include "pipeline/world_estimation/landmark_error.h"`.
- Header comment `configured_targets.h:11-13`: replace "latch from the current committed landmark estimate (world state when valid, else the nominal map pose); zero visual correction, not a zero pose" with "latch the nominal target moved by the landmark's committed LandmarkError (zero while unobserved); zero visual correction, not a zero pose".
- Unchanged: `mapNominalTargetPose` (93-105), the acquire_once candidate (244), the lock (284), and the latch, epoch and generation logic.

### 4.6 Inspection (`debug/inspection/inspection_document.cpp`)
- Delete 1198-1209: the `// nominal poses come from the configured maps` scan into `std::map<std::string, Pose2D> nominal`.
- Replace 1230-1237 with:
  ```cpp
              if (o.nominal) {
                  pose2(w, "nominal", *o.nominal);
                  w.field("heading_error_deg", radToDeg(o.error.dtheta));
                  w.field("displacement_m", std::hypot(o.error.dx, o.error.dy));
  ```
  The `else` block with its three `fieldNull` calls stays.
- Remove `#include <map>` (line 8). The scan was its only user: `grep -c std::map` gives 1 at ae457ee.
- Bit-identity: `o.error.dtheta` is `wrapAngle(pose.h - nominal.h)`. That is today's expression on the same two doubles: the pose is the value stored with it, and the nominal is the same `LandmarkDecl::nominal` of the same shared `FieldMap` instance. Likewise `hypot(dx, dy)` is `hypot(pose.x - nominal.x, pose.y - nominal.y)`. The JSON number text (`%.6g`, json_writer.h:101) is therefore identical. Test 7.2(8) proves it on the synthetic rig by comparing strings.

### 4.7 Publishing: unchanged, and why this departs from the SPEC's wording
SPEC says the field documents carry full poses "computed as nominal plus error". The plan keeps the wire code as it is:
1. The wire derives its pose separately from the field state: `compose(robot.field_from_odom, s.T_odom_object)` (field_documents.cpp:245), using the publish-time robot. `reportingCycle` passes `robot_`, which can hold a newer anchor than the field snapshot. The test is named FieldEstimate.ObservedOnlyInTheRobotsEpochComposedWithTheCurrentAnchor.
2. field_documents_gtest builds `FieldObjectState` with only `T_odom_object` (`observedAt`, 80-88). A wire read from `nominal`/`error` would send nominal for those entries and fail 637-643. That is a test outcome change, so a bug by SPEC rule.
3. `FieldMapDocument` holds mm/cdeg records, not float nominals.
4. Nominal + error is not bit-exact. On 1,000,000 cases, 79,065 differ by 1 ulp in heading (max 4.44e-16 rad).

"Full pose = nominal + error" still holds by definition, since `error := pose - nominal`, and test 7.2(4) checks it. The wire stays bit-identical. The lead should confirm this reading.

### 4.8 Association prior
`tag_mount_association.cpp:313-320` keeps reading `pose.pose`. No change.

## 5. Proofs

Notation: `a*b` is `compose(a, b)` on planar poses, `a^-1` is `inverse`, and F is `field_from_odom`. N is the landmark's map nominal and P its derived field pose. `e = landmarkError(N, P) = (P.xy - N.xy, wrap(P.h - N.h))`, and `N+e` is `(N.xy + e.dxy, wrap(N.h + e.dtheta))`. Algebraically `N+e = P`, with headings equal mod 2pi.

### 5.1 `resolveFromLandmark(L) = L * c` for a fixed planar c
`resolveFromLandmark(L) = planar(L3 * C)` with `C = T_landmark_approach * D3 * T_robot_controlled^-1`, a fixed SE(3) transform (target_set.h:81-87). `L3 = transform3FromPlanar(L)` is `Rz(L.h)` plus translation `(L.x, L.y, 0)`. So:
- Translation: `(L.x, L.y) + Rz(L.h)(C.x, C.y)`, with z not used.
- Rotation: `Rz(L.h) * C.R`. `planarFromTransform3` takes yaw as `atan2(R[1][0], R[0][0])` (se3.h:139-145). That is the angle of the first column projected on xy. `Rz(L.h)` rotates that column by exactly L.h, so the yaw is `L.h + yaw(C.R)` mod 2pi.

Hence `planar(L3 * C) = L * c` with `c = (C.x, C.y, yaw(C.R))`. The one exception is when C's x axis is vertical (pitch of +-90 deg), where yaw is undefined. A robot target pitched 90 degrees is meaningless and degenerates the same way in the old and new code.

### 5.2 The helper equals the old composition
- The helper is left multiplication by `D = (N+e) * N^-1`. For any T, `D * T` rotates by `(N.h + dtheta) - N.h = dtheta`. Its translation is `(N+e).xy + R(N.h + dtheta) R(-N.h)(T.xy - N.xy) = N.xy + e.dxy + R(dtheta)(T.xy - N.xy)`. Its heading is `T.h + dtheta`. That is `applyLandmarkError(e, N, T)`; the target-plus-correction form is the same sum regrouped.
- Old policy none: `Old = resolveFromLandmark(F^-1 * P) = F^-1 * P * c` (5.1).
- New: `New = F^-1 * applyLandmarkError(e, N, N * c) = F^-1 * D * N * c = F^-1 * (N+e) * c = F^-1 * P * c = Old`.

The three branches of `estimateTargetPose`:
- (a) Valid and mapped: shown above.
- (b) Valid, `nominal` empty (unmapped): N := P and e = 0. So `D = I` and `New = F^-1 * P * c = Old`. Because zero error is exact (5.3), the helper step contributes no rounding here.
- (c) Invalid or absent: N = the resolver's map nominal (or `Pose2D{}`, exactly as today) and e = 0. `New = F^-1 * N * c = resolveFromLandmark(F^-1 * N) = Old`.

Mixed anchors (the field snapshot derived under an older F than `robot_`) enter only through `P`, the same stored double in both versions. So the identity holds with any anchor lag.

### 5.3 Bit-exact or tolerance, per quantity
| Quantity | Relation to today | Why |
|---|---|---|
| `FieldObjectState::pose` | bit-identical | Same expressions, same inputs (setFieldPose receives today's right-hand side). |
| `error` for seeded or epoch-reset entries | exactly (0, 0, 0) | `x - x == 0` and `wrapAngle(h - h) == 0` in IEEE. |
| inspection `nominal`, `heading_error_deg`, `displacement_m` | bit-identical | Same expression on the same doubles (4.6). |
| Brain wire records | bit-identical | Code unchanged (4.7). |
| association priors | bit-identical | Read `pose` (4.8). |
| policy-none `T_odom_robot_target` | within 7.11e-15 m and 1.94e-15 rad (measured over 1e6 cases) | Different operation order. The old code does inverse(F), then the SE(3) chain. The new code does the chain in the field frame, then the correction, then inverse(F). No formulation can be bit-exact: even at e = 0 the chain runs in another frame (measured: 62,772 of 1e6 equal bits). |
| `applyLandmarkError(e, N, N)` vs P (round trip) | position bit-exact in 1e6 realistic cases; heading within 4.44e-16 | `a + (b - a)` is not always b. Counterexample: `0.1 + (1e-17 - 0.1) = 1.39e-17`. So it is tested with tolerance, and P stays the stored value. |
| `applyLandmarkError(0, N, T)` vs T | bit-exact (1e6 of 1e6) | `(cos 0 - 1) = 0` and `sin 0 = 0` exactly, so the correction is +-0. `T.h` is already in (-pi, pi]. |

Headings are compared as `wrapAngle(a - b)`, never raw. A value next to +-pi may come out on either side of the wrap. The probe saw 0 such flips in 1e6 cases, but it is possible.

## 6. Tag trace split (pure move, no logic)

### 6.1 `pipeline/world_estimation/evidence.h` keeps
- The header comment's first two paragraphs (generic evidence, provenance).
- Includes `<cstdint>`, `<string>`, `<vector>`, `"pipeline/framework/ids.h"`, `"time/time.h"` and `"math/transforms.h"`. Drop the pose_history include: its only user is `TagAssociationTraceSet::exposure_context`.
- `payload_names::kFieldObjectPoseEvidenceSet` (string unchanged), `FieldObjectPoseEvidence` and `FieldObjectPoseEvidenceSet`.

### 6.2 `pipeline/world_estimation/estimators/apriltag/tag_association_trace.h` (new) takes
- The third header paragraph (ae457ee `field_object_evidence.h:13-15`), reworded as the file's own label: "The tag mount association's decision made visible: every decoded tag with its candidates, the accepted mount or the reason it was rejected, for inspection. It never feeds estimation."
- Includes `<cstddef>`, `<cstdint>`, `<string>`, `<vector>`, `"pipeline/framework/ids.h"`, `"time/time.h"`, `"math/transforms.h"` and `"pipeline/localization/pose_history.h"`.
- `namespace payload_names { constexpr const char* kTagAssociationTraceSet = "association.tag_association_trace_set"; }`. The string is unchanged; it is the payload descriptor name.
- `TagAssociationCandidate`, `TagAssociationTrace` and `TagAssociationTraceSet`, verbatim from ae457ee lines 54-87.

### 6.3 Include changes (post-move paths)
| File | Change |
|---|---|
| pipeline/world_estimation/estimators/apriltag/tag_mount_association.cpp | add the trace include (it uses both) |
| debug/inspection/inspection_state.h (ae457ee runtime/inspection_state.h:19) | replace the evidence include with the trace include (only `DetectionFrameSnapshot::trace`, line 53) |
| debug/inspection/inspection_state.cpp (publishDetectionFrames) and debug/inspection/inspection_document.cpp | nothing: both get the trace through inspection_state.h, as today through system.h |
| tests/capture_transform_gtest.cpp | add the trace include (uses both) |
| tests/synthetic_rig_gtest.cpp, tests/vision_targets_gtest.cpp | replace the evidence include with the trace include (trace only) |
| configured_targets.cpp, apriltag_world_estimator.cpp, landmark_estimator.h, tests/localization_fusion_gtest.cpp, tests/world_estimation_gtest.cpp | keep evidence.h only |

Dependency note: `debug/inspection` depends on `estimators/apriltag` for the trace and tag observation types. It already depends on apriltag payloads today (`inspection_state.h:20` includes tag_observations.h, which moves to `estimators/apriltag/`). This is not new coupling, only now visible in the path.

## 7. Tests

### 7.1 Existing tests: only include lines change; every outcome is unchanged
- Include edits: capture_transform_gtest.cpp, synthetic_rig_gtest.cpp and vision_targets_gtest.cpp (6.3).
- Tests that run changed code and must pass as they are: world_estimation (16), vision_targets (20; policy none: PolicyNoneLatchesNominalImmediately, OdometryEpochResetCancelsLatchedTarget, SelectingOrClearingATargetDoesNotResetFieldState), field_documents (18), inspection_document (14), inspection (10), inspection_feed (21), synthetic_rig (5), capture_transform (4), workers (12), brain_link_e2e (10). Plus the rest of the 626.
- Prototype result: all 626 baseline names pass, and `ctest -N` shows no name removed or renamed (9.2).

### 7.2 New tests (8), all passing in the prototype
| # | File | Test | Checks |
|---|---|---|---|
| 1 | tests/landmark_error_gtest.cpp (new; add to `TEST_SOURCES` after `world_estimation_gtest.cpp`) | LandmarkError.ZeroIsTheIdentity | 10,000 random N and T, a quarter with headings within 1e-9 of +-pi. `applyLandmarkError({}, N, T) == T` and `landmarkError(N, N) == {0,0,0}`, both EXPECT_EQ (bit-exact). |
| 2 | same | LandmarkError.RotatesTheOffsetAboutTheNominalLandmarkInFieldAxes | Hand case in field axes: N (1, 2, 90 deg), T (1.5, 2, 90 deg), e (0.1, -0.2, 90 deg) gives (1.1, 2.3, 180 deg). |
| 3 | same | LandmarkError.HelperEqualsTheFullPoseComposition | 100,000 random cases (seed 20260928), a quarter near +-pi, random F. Half use the vision_targets chain and half a random tilted SE(3) chain. `F^-1 * apply(landmarkError(N, P), N, resolve(N))` vs `resolve(F^-1 * P)` within 1e-12, heading wrapped. |
| 4 | same | LandmarkError.RoundTripsWithTheFieldPose | 100,000 random cases. `apply(e, N, N)` vs P within 1e-12, and `landmarkError(N, apply(e, N, N))` vs e within 1e-12. |
| 5 | tests/world_estimation_gtest.cpp | LandmarkEstimator.SeededLandmarksCarryTheirNominalAndZeroError | After a seed: `nominal ==` the map decl and `error == {0,0,0}`, both EXPECT_EQ. The fixture's nominal heading is 90 deg, so it is not trivially zero. |
| 6 | same | LandmarkEstimator.ErrorIsDerivedWithTheFieldPoseUnderEveryAnchor | F = (0.3, -0.2, 0.25). Fold evidence, then `error == landmarkError(nominal, pose)` EXPECT_EQ, with `|dxy| > 0.1`. Re-anchor (revision + 1, F = (-0.4, 0.1, -1.0)): still EXPECT_EQ. Odometry epoch + 1: error exactly zero. |
| 7 | tests/vision_targets_gtest.cpp | VisionTargets.PolicyNoneMovesTheNominalTargetWithTheLandmarkError | A displaced, turned goal (+0.05, -0.03, 0.08 rad) committed under F = (0.1, -0.05, 0.1), then select(3). The latched target equals `decl->resolveFromLandmark(compose(inverse(F), goal pose))` within 1e-9 and is more than 0.03 m from the nominal target. |
| 8 | tests/inspection_document_gtest.cpp (add includes `config/field_map.h`, `math/angles.h`; place after the `balancedObject` helpers, before StateHistoryAndDiagSplitTheSnapshot) | InspectionDocuments.FieldObjectErrorsAreThePoseMinusNominal | kRig, 180 steps, goal observed. The goal's field_objects JSON contains `"heading_error_deg":X,"displacement_m":Y`, where X and Y are written by JsonWriter from today's formulas (exact string match). |

Mutation check (prototype): with the target error forced to zero, test 7 fails (distance to nominal 4.4e-16). With the inspection heading error scaled by 1.001, test 8 fails. After reverting both, 634/634 pass.

The code for all 8 tests is in `R/scripts/we_proto/tests/landmark_error_gtest.cpp`, `world_estimation_add.cpp`, `vision_targets_add.cpp` and `inspection_document_add.cpp`. It uses ae457ee include paths; switch them to post-move paths (for example `pipeline/world_estimation/landmark_error.h`, `pipeline/resources/target_set.h`, `pipeline/world_estimation/estimators/apriltag/tag_association_trace.h`).

## 8. Execution (step 3, in `S/wt-navi` after steps 1 and 2)
1. Record `ctest -N` before: build per SPEC into `S/wt-build/<key>/`, then run `ctest --test-dir <dir> -N > R/logs/<key>.before.ctestN.txt`.
2. Add `src/math/units.h` and `src/pipeline/world_estimation/landmark_error.h` (3.1, 3.2).
3. Edit `field_state.h` (3.3) and `landmark_estimator.cpp` (3.4, 4 sites).
4. Edit `configured_targets.cpp` and `.h` (4.5) and `inspection_document.cpp` (4.6).
5. Tag trace split (6).
6. Add the tests (7.2) and the `tests/CMakeLists.txt` line.
7. `git add` the 3 new headers and the new test file. Index only; no commit.
8. Format. New files: `clang-format -i` on the whole file. Existing files: changed lines only:
   `git -C S/wt-navi diff -U0 --no-color HEAD -- <touched files> | clang-format-diff -p1 -i -binary <clang-format>`
   - SPEC's `/c/msys64/ucrt64/bin/clang-format` does not exist on this box. Version 22.1.8 is at `/c/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/clang-format.exe`, and clang-format-diff is next to it.
   - Whole-file formatting of a touched ae457ee file rewrites unrelated lines. The repo writes `x;   // c` (3 spaces), and the config yields 1 space. It also splits `case X: return` and realigns blocks, as seen in field_state.h, landmark_estimator.cpp, configured_targets.cpp and field_object_evidence.h. So format changed lines only.
   - Changed-line formatting output is in `R/logs/we-plan.cfd_changed_lines.diff`: include order and alignment of the new lines only.
9. Build, run `ctest -N` after, and diff. Expect exactly the 8 names in 7.2 to be added and 0 removed. Full ctest: 634/634.

## 9. Verification done for this plan

### 9.1 Numeric probe
`R/scripts/we_probe.cpp`, compiled to `S/wt-build/we-plan/we_probe.exe` with g++ 15.2.0 -O0 against the worktree headers. 1,000,000 cases, 250,000 of them near +-pi:
```
nominal+error (form A, target=landmark) vs P n=1000000 bit_equal=920935 max_xy=0 max_h=4.44e-16
nominal+error (form B, target=landmark) vs P n=1000000 bit_equal=920935 max_xy=0 max_h=4.44e-16
identity form A vs T                         n=1000000 bit_equal=844683 max_xy=8.88e-16 max_h=0
identity form B vs T                         n=1000000 bit_equal=1000000 max_xy=0 max_h=0
helper form A vs old composition             n=1000000 bit_equal=53577 max_xy=5.33e-15 max_h=1.33e-15
helper form B vs old composition             n=1000000 bit_equal=53244 max_xy=5.33e-15 max_h=1.33e-15
helper form B vs old, tilted SE(3) chain     n=1000000 bit_equal=25203 max_xy=7.11e-15 max_h=1.94e-15
zero error helper vs old nominal (P == N)    n=1000000 bit_equal=62772 max_xy=5.33e-15 max_h=9.44e-16
0.1 + (1e-17 - 0.1) == 1e-17: no (got 1.3877787807814457e-17)
```
Form A is `N + d + R(o)`; form B is the chosen `T + d + (R - I)o`.

### 9.2 Prototype build and tests (ae457ee layout, scratch copy; the worktree was not touched)
- Source: `git -C S/wt-navi archive ae457ee pi/naviGATR translaGATR brain` extracted into `S/wt-build/we-plan/proto`. Build dir: `S/wt-build/we-plan/build` (SPEC recipe, Debug, MinGW Makefiles).
- Baseline: 626 tests (`R/logs/we-plan.base.ctestN.txt`), "100% tests passed, 0 tests failed out of 626" (`R/logs/we-plan.base.test.log`).
- Change applied with `py -3 R/scripts/we_apply_proto.py <proto>/pi/naviGATR`. That script makes exact-once replacements and copies files from `R/scripts/we_proto/`. Build: 0 errors, 0 warnings with -Wall -Wextra (`R/logs/we-plan.proto.build.log`).
- After: 634 tests (`R/logs/we-plan.proto.ctestN.txt`), "100% tests passed, 0 tests failed out of 634" (`R/logs/we-plan.proto.test.log`).
- Sorted-name diff of `we-plan.base.names.txt` against `we-plan.proto.names.txt`: exactly the 8 additions in 7.2 and no removals.
- Reference patch on the ae457ee layout, unformatted and without reformat noise: `R/plan/world_estimation_proto.diff` (17 files).

## 10. Edge cases and points for the lead
1. **Multiple field maps.** Every System in `config/` and `tests/` declares one `field_map` resource, and every `<FieldMap resource_id>` in it names that resource (grep). With two maps, behavior changes in only two places:
   - Inspection prints the estimator's nominal instead of the first matching map in resource order.
   - Inspection prints null for an observed object missing from the estimator's map. Today it prints numbers against another map.
   Policy none is unaffected: it uses the state's own nominal, algebraically equal to today (5.2 a and b).
2. **Wire reading (4.7).** The wire code is unchanged, not recomputed from nominal + error. Please confirm this reading of "computed as nominal plus error".
3. **clang-format.** The SPEC path is wrong on this box, and whole-file formatting churns ae457ee files (8). The repo-wide trailing-comment spacing (3 spaces in the code, 1 from the config) is a separate question.
4. **Scope held.** No wire switch to the error form (open decision, SPEC). No fusion and no second estimator. acquire_once is not converted, since it composes per-frame odometry evidence, not the committed field error.
5. **Docs step facts** for world_estimation/PIPELINE.md:
   - `FieldState` exposes per object `pose`, the map `nominal` and `LandmarkError{dx, dy, dtheta}` (field axes, pose minus nominal, (0, 0, 0) = as built).
   - Target resolution applies the error with `applyLandmarkError`.
   - The Brain wire still sends full poses.
   - The error budget lines are already in SPEC ("The purpose").
