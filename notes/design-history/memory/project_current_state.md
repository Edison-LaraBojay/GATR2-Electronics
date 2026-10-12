---
name: project-current-state
description: "2026-09-22: WorldEstimation stage (apriltag/noop estimators) + weighted_planar_fusion localization estimator landed in pi/navigatr working tree, 267/267 tests green, ALL uncommitted; four unclaimed untracked files (incl. a Python tool) need the user's decision"
metadata:
  node_type: memory
  type: project
  originSessionId: b08136da-04b0-495c-bd43-e435c223477d
  modified: 2026-09-22T23:20:29.905Z
---

As of 2026-09-22 evening (verify against repo; NOTHING committed, tree dirty on main; deletions were git rm so they show staged "D ").

Two Claude sessions worked the same tree concurrently and split by files (peer session mbes1-ae did world estimation, this one did localization fusion).

**World estimation (peer):** FieldEstimation slot replaced by an aggregate stage `<WorldEstimation><Estimator id=".." type="apriltag|noop">` built by make_world_estimation (runtime/world_estimation_stage.*), executor label "WorldEstimation/<id>". impl/world_estimation/ holds apriltag_world_estimator (owns ObservationExtraction -> Association (optional) -> LandmarkEstimation subtree, fixed step types), landmark_estimator, moved apriltag_tag_observation + tag_mount_association. Perception/Association registries and noops deleted; landmark_field deleted. field_estimation_gtest -> world_estimation_gtest. All test XML and config pipelines converted.

**Localization fusion (this session):** new StateEstimator `weighted_planar_fusion` (impl/localization/weighted_planar_fusion.*): required `<Noise>` per Motion/Heading in explicit one-sigma units (translation_floor_m, translation_per_m, rotation_floor_rad, rotation_per_rad, rotation_per_m; angle_random_walk_rad_per_sqrt_s, bias_rad_per_s), precision-weighted rotation, translation re-solved via geometric coupling J = -(U'U)^-1 U'k that tracking_wheel_motion now publishes (BodyMotionIncrement.has_rotation_coupling/dx,dy_per_dtheta_m_rad), chord Jacobian, P' = FPF' + GQG'. RobotState gained PoseCovariance odom_covariance + has_covariance (inspection JSON exports it). Shared helpers motion_step.{h,cpp} (PlacementEdge, chordOfArc, AttitudeFold) used by both estimators; planar_motion_integrator kept, behavior unchanged. Demo: config/demo/synthetic_fusion_demo.xml + shared/pipelines/synthetic_fusion_demo.xml (three wheels WITHOUT HeadingConstraint + imu_heading_increment, noise values are PLACEHOLDERS). Doc: docs/localization_fusion.md; calibration_inventory has a noise row. Tests: tests/localization_fusion_gtest.cpp (21 tests incl. demo-vs-rig-truth).

**2026-09-22 late round (this session):** localization correctness fixes shared by both estimators via impl/localization/motion_step.* (prepareStep/finishStep, HeadingAligner, PlanarStepState): heading support must equal the motion window exactly on one named clock (partial contiguous support accumulates in the estimator while the motion waits, bounded by Heading max_wait_ms, default 100; interval_tolerance_ms is refused at build); clock identity compared by Provenance.clock name (device domain alone is nothing; clock change = epoch bump like a reboot); lineage via new Provenance.measurement = acquisition output ("pico_telemetry.imu") carried through channel sensors and models, used for independence instead of sensor ids. Deadline (2026-09-24 review fix): every held motion window gets max_wait_ms from its first sighting, checked on every call (HeadingAligner::align wraps alignWindow; window_ tracking cleared only on fused/released/expired), so a stream of mismatched headings or a no-rotation motion without any stash cannot block. Tests: 282/282 (lineage tests incl. real pico_telemetry memory-link alias path are now really in localization_fusion_gtest.cpp; the first append had silently failed).

**Verified:** cmake MinGW build clean (-Wall -Wextra, no warnings), ctest 267/267, `navigatr --inline --cycles 400` on the fusion demo runs.

**Unclaimed untracked files created 19:17 on 2026-09-22 by neither session's stated footprint** (user must decide; not deleted): config/override/diagnostics/{two,three}_wheel_imu_camera_pipeline.xml.in, config/shared/robots/gatr2_two_wheel_as5047_imu.xml.in, tools/calibrate_camera.py (Python inside navigatr violates the C++-only rule unless the user accepts it).

**Known limitations:** fusion models wheel translation/rotation noise independent (no (A'A)^-1 structure), single Motion + single Heading only, no bias state, landmarks never correct the pose (build refuses their payload). Noise values unmeasured.

See [[project-navigatr-architecture]], [[feedback-conventions]], [[feedback-design-principles]], [[reference-navigatr-windows-build]].
