# Claude prompt: Field Estimation refactor

Copy everything inside the block below into Claude from the repository root.

```text
You are working in the GATR2-Electronics repository. Implement the Field
Estimation architecture described below, starting from the repository's
current state. Do not treat current behavior as authoritative when it
conflicts with this prompt, but preserve unrelated behavior and user changes.

Before editing:

1. Read README.md, pi/navigatr/README.md, docs/navigatr.md, and
   docs/ARCHITECTURE_CONTRACT.md.
2. Inspect the current implementations and tests named below.
3. Run `git status --short`. Preserve the existing untracked `.claude/`
   directory and every unrelated modification.
4. Run the existing NaviGATR tests to establish the baseline.
5. Make a short implementation plan, then carry it through completely. Do not
   stop after renaming types or writing a design document.

Current implementation facts
----------------------------

The current top-level slot is `WorldEstimation`:

- Contract: `pi/navigatr/src/contracts/world_estimation.h`
- State: `pi/navigatr/src/state/world_state.h`
- Concrete composite:
  `pi/navigatr/src/impl/world_estimation/landmark_world.{h,cpp}`
- Registration:
  `pi/navigatr/src/impl/world_estimation/register_world_estimation.cpp`
- Generic association contract: `pi/navigatr/src/contracts/association.h`
- Generic observation-extraction contract is currently named `Perception`:
  `pi/navigatr/src/contracts/perception.h`
- AprilTag extractor:
  `pi/navigatr/src/impl/perception/apriltag_tag_observation.{h,cpp}`
- AprilTag mount association:
  `pi/navigatr/src/impl/association/tag_mount_association.{h,cpp}`
- Current landmark pose evidence:
  `pi/navigatr/src/payloads/landmark_pose_observations.h`
- Field definitions: `pi/navigatr/src/config/field_map.{h,cpp}`
- Targets: `pi/navigatr/src/resources/target_set.h` and
  `pi/navigatr/src/impl/target_resolution/configured_targets.{h,cpp}`
- Runtime order: `pi/navigatr/src/runtime/system.{h,cpp}`
- Main focused tests:
  `pi/navigatr/tests/world_estimation_gtest.cpp`,
  `vision_targets_gtest.cpp`, `end_to_end_gtest.cpp`,
  `system_build_gtest.cpp`, and `profiles_gtest.cpp`.

Today `landmark_world` privately runs one observation extractor, one
associator, and a hard-coded landmark estimator. The AprilTag extractor can be
target-gated (`detect="on_demand"`). `TagMountAssociation` can also be gated
through a `<Targets>` resource. The estimator consumes the AprilTag-specific
`LandmarkPoseObservationSet`, seeds nominal map poses, and supports
`always`, `never`, and `on_target_lock` commit policies. This target coupling
is exactly what this refactor must remove.

Required model and terminology
------------------------------

Rename the architectural concepts consistently:

- `WorldEstimation` -> `FieldEstimation`
- `WorldState` -> `FieldState`
- `WorldObject` -> `FieldObjectState`
- Prefer `FieldObjectId` over `WorldObjectId` where the identifier refers to
  an object defined by the field map.

Update filenames, C++ APIs, factory registration, diagnostics, XML slot names,
tests, README files, and the normative architecture contract consistently. Do
not leave a mixture of world/field names except where "world" is ordinary
English rather than an API concept. If backward-compatible XML aliases are
kept temporarily, make them explicit, tested, and documented; do not create
silent aliases through loose parsing.

The concepts have these exact responsibilities:

`FieldMap`

- Immutable configuration constructed from XML.
- Describes physical field objects, their opaque ids, nominal canonical poses,
  fixed geometry, observable features, and fixed transforms such as the four
  tag mounts on a goal.
- A physical goal has one canonical pose, normally its center plus rotation.
  The four side/tag poses are derived from that pose and their configured
  transforms; they are not four independently estimated goals.
- The estimator must never mutate `FieldMap`.
- Keep the existing field XML working unless a deliberate schema migration is
  implemented and every repository profile/test is migrated. Do not invent a
  closed enum of all possible season objects.

`FieldState`

- Mutable current estimates keyed by field-object id.
- Each object state initially needs: a field-framed pose, validity,
  confidence, observed-this-cycle, last-observed time, and estimate source.
- One current state is sufficient. Do not add a whole-state history buffer in
  this pass. State before an update is the prior/current input; the function
  returns the next state atomically.
- Poses are expressed in the named canonical field frame, but estimates are
  unbounded. Never clamp them to the nominal 144-by-144-inch boundary.
- An unobserved object is still processed conceptually: preserve or predict it
  according to its estimator policy, set observed-this-cycle false, and retain
  meaningful timestamps/confidence. Do not manufacture a zero observation.

`TargetSet`

- Configuration for imaginary desired robot poses, not physical objects.
- A point five inches outside a goal while facing the goal belongs here.
- A tag mount, line pair, or physical side geometry belongs in `FieldMap`.
- Target resolution composes the selected target definition with the latest
  `FieldState` and `RobotState`; target selection and reporting never control
  Field Estimation.

Required pipeline boundaries
----------------------------

Use this logical pipeline:

    SensorResultsMap + ArtifactMap
        -> Observation Extraction
        -> ObservationMap
        -> Association using FieldMap + prior FieldState + RobotState
        -> associated field evidence
        -> Field Estimation/fusion
        -> FieldState
        -> Target Resolution

The three questions are:

- Extraction: "What did the sensor observe?"
- Association: "Which configured physical field object and feature caused
  that observation?"
- Estimation: "How should that evidence update FieldState?"

Enforce these ownership rules:

1. Extractors understand sensors and implementation-specific detection.
   They consume the standard retained sensor results/artifacts and output
   immutable typed observations. Process a camera frame once, not once per
   candidate field object.
2. Associators do not fetch raw sensors and do not mutate FieldMap or
   FieldState. They consume typed observations and emit immutable associated
   evidence. Abstaining on ambiguity is normal.
3. Estimators consume prior FieldState plus associated evidence and return a
   new FieldState. A child fault must leave the prior state untouched; no
   partial commit.
4. Missing required configured sensors/resources are startup errors with
   contextual messages. A correctly configured sensor with no fresh runtime
   sample produces no observation/evidence that cycle, not a startup fault and
   not a fake zero.
5. Generic runtime code transports typed record envelopes and must not inspect
   cameras, AprilTags, line segments, goal classes, or match loaders.

Evidence contract
-----------------

Generalize the current `LandmarkPoseObservationSet` into field-object pose
evidence. The stable meaning should be approximately:

    FieldObjectPoseEvidence
      - object id
      - configured feature instance id (for example a physical tag mount)
      - implied object pose in a named frame, currently odometry
      - measurement/exposure time on the host monotonic timeline
      - source sensor id
      - source sequence/frame identity
      - confidence/quality

The AprilTag association implementation should emit this generic pose evidence
after matching a detection to a specific configured tag mount. It may know the
full tag geometry and SE(3) chain; the Field Estimation contract must not.

Keep record payloads open and typed. Do not introduce a central variant or an
enum containing every future observation/evidence type. A future line
extractor may emit line segments, and a future match-loader associator may
match a pair of lines against configured geometry and emit the same
field-object pose evidence. Do not implement the match-loader detector now and
do not invent its final XML schema in this pass; make sure it can be added as a
new extractor/associator without editing generic orchestration.

Continuous operation and target decoupling
------------------------------------------

Field Estimation runs every system cycle regardless of whether the Brain has
requested a target or requested no target report.

- Remove `CommandState` and `TargetState` from generic Field Estimation,
  observation-extraction, and association inputs when they exist only for
  target gating.
- AprilTag extraction must not implicitly stop because no navigation target is
  pending. If an explicit compute/scheduling policy is retained, it must be an
  implementation-owned source policy independent of navigation target state.
- `TagMountAssociation` must associate decisive observations against compatible
  configured field objects, not only the selected target.
- Move preferred-camera, allowed-mount, target-generation, and acquisition
  consistency filtering into Target Resolution, where navigation intent
  belongs. Evidence older than target activation must not acquire the new
  target.
- Field Estimation must not use `commit="on_target_lock"`. Field-state update
  policy is independent of whether target resolution accepted enough evidence
  to latch a navigation target. Keep an explicit `never`/trace mode if useful;
  the normal estimator commits accepted field evidence.
- Preserve the explicit Brain request flag for "no target report" rather than
  making target id zero secretly control estimation. Clearing a target request
  must not clear FieldState or disable extraction/association.

Frame and heading rules
-----------------------

- Preserve the existing internal project math convention in
  `math/transforms.h`: field +x right/east, +y up/north, heading zero along +x,
  positive counterclockwise. Do not rewrite transform math to compass angles.
- Every pose crossing a boundary must identify or document its frame.
- Persistent FieldState is field-relative, not robot-relative. Robot-relative
  observations are transient; target resolution may produce robot-relative or
  field-relative control output as explicitly required.
- Shared global frame error is allowed to cancel during relative targeting:

      T_robot_object = inverse(T_field_robot) * T_field_object

  Therefore field estimates are never rejected merely for lying outside the
  physical field boundary.
- A feature/tag surface has an outward heading. A desired relative heading of
  180 degrees means the robot faces back toward that surface. In a consistent
  frame:

      desired_field_heading = feature_field_heading + relative_heading

- If the Brain-facing API later exposes compass bearings (`North=0`,
  `East=90`, clockwise), perform that conversion only at the command/publishing
  boundary and test it. Do not mix it into internal SE(2)/SE(3) math.

Estimator behavior for this pass
--------------------------------

Do not introduce an EKF or speculative motion model. Keep the first estimator
small and deterministic:

- Seed configured objects from nominal FieldMap poses.
- Carry the prior FieldState forward atomically.
- Mark all objects unobserved at the start of a cycle.
- Apply accepted pose evidence only to its named configured object.
- Update measurement/last-observed time and confidence when evidence commits.
- Preserve the existing configurable blend behavior if it remains useful.
- If multiple accepted measurements update the same object in one cycle, do
  not let unordered container or detector iteration order silently determine
  the result. Fuse them deterministically (for example weighted x/y plus a
  circular heading mean) or explicitly select the best-confidence measurement,
  and test the chosen policy.
- Reject non-finite evidence before it reaches state.

Robot correction remains separate. Field Estimation updates physical external
objects, not `RobotState`. A future trusted fixed-anchor correction belongs in
an explicit localization/robot-correction implementation. Do not silently make
AprilTag goal observations rewrite smooth wheel/IMU odometry.

Implementation quality rules
----------------------------

- Prefer a staged, compiling migration over blind global replacement.
- Preserve strong typed ids and typed payload declaration checks.
- Opaque ids must never be parsed for meaning.
- Do not make field objects own sensor pointers. The same camera can observe
  many objects; sensor dependencies belong to extractors.
- Do not classify capability solely with strings such as
  `goal_with_apriltag`. Observable capabilities/features should be
  implementation-owned data so one object can expose several kinds of
  geometry.
- Do not mutate an observation in place to "tag" it. Association emits a new
  associated evidence record with provenance.
- Preserve timestamp-at-exposure handling and robot pose lookup at exposure.
- Preserve strict configuration validation and explicit no-op selections.
- Do not edit or remove `.claude/settings.local.json`.
- Do not modify Brain/Pico/PCB code unless a compile-time shared API rename
  genuinely requires it.

Required tests
--------------

Update existing tests and add focused coverage proving at least:

1. FieldMap remains immutable after observations.
2. FieldState seeds from the map and is updated independently.
3. FieldState accepts unbounded coordinates outside nominal field limits.
4. No target requested still allows extraction, association, and FieldState
   updates.
5. Selecting or clearing a target does not reset FieldState.
6. Missing required extractor sensor/resource fails at construction; no fresh
   sample produces no evidence without corrupting state.
7. AprilTag association identifies the physical mount using full geometry and
   abstains when ambiguous, including repeated printed ids on several mounts.
8. Target Resolution filters generic evidence by requested field object,
   configured feature/mount, preferred source, freshness, and activation time.
9. Multiple evidence records for one object have deterministic fusion or
   selection independent of iteration order.
10. A fault in extraction/association leaves the prior FieldState untouched.
11. Relative target resolution is invariant under the same rigid field-frame
    transform applied to robot and object estimates.
12. XML profiles require the explicit `FieldEstimation` slot and its explicit
    noop when disabled.

Update all architecture documentation so it matches the implemented behavior;
`docs/ARCHITECTURE_CONTRACT.md` is normative and must not retain the old
target-gated World Estimation rules.

Verification and handoff
------------------------

Run formatting appropriate to the repository, build NaviGATR, and run the full
test suite with failures shown. Inspect `git diff --check` and `git status`.
Do not claim completion if old API names remain accidentally, tests are merely
disabled, or docs contradict code. In the final report, summarize the contract
changes, migration compatibility choices, tests run/results, and any genuine
remaining hardware/backend limitation (for example an unavailable physical
camera or AprilTag backend).
```
