---
name: project-navigatr-architecture
description: "navigatr v4 (hierarchical build, 2026-08-18): 6-slot pipeline, WorldEstimation/TargetResolution split, landmark_world composite, composed Configuration profiles, wheel_geometry resource"
metadata: 
  node_type: memory
  type: project
  originSessionId: cdc4c519-2766-4356-ae37-f4b594d849ab
  modified: 2026-08-18T04:06:48.357Z
---

navigatr RESTRUCTURED 2026-08-17/18 per the user's round-5 "hierarchical but controlled" contract (supersedes the 9-slot v3). Deep docs: docs/navigatr.md + navigatr_resources.md + navigatr_sensors.md + pi/navigatr/README.md (user-authored, corrected).

**Six-slot pipeline:** Sensor Collection -> Command Collection -> Preprocessing -> Localization -> WorldEstimation -> TargetResolution -> Publishing. Perception/Association survive as COMPOSITE-CHILD contracts only; PoseCorrection/WorldPrediction contracts DELETED (correction belongs inside future Localization composites). Implementations may be leaf, noop, or composite owning a private nested pipeline; children built from the same registry categories, reachable only through the parent, child fault = no partial commit. Selective genericity: TargetResolution ("configured_targets") and Publishing are focused config-driven logic, NOT open plugin points (user addendum).

**landmark_world (WorldEstimation composite):** children ObservationExtraction (Perception contract, apriltag_tag_observation w/ detect=always|on_demand), Association (tag_mount_association; <Targets> now OPTIONAL -> diagnostic mode associates whole field, generation 0), Estimator (composite-owned landmark_estimator, commit="always|never|on_target_lock" + blend). on_target_lock buffers the acquiring generation privately and folds its mean one cycle AFTER kLockedVision (tests step once more); nominal fallback/cancel discards buffer. WorldEstimationOutput carries world + published observations/associations; System feeds those to TargetResolution + Publishing and fills slot decls from producesObservations/Associations().

**configured_targets (TargetResolution):** port of target_tracker minus world mutation: activation edge on command.object_sequence (deferred while robot invalid), robot-relative snapshot-once, acquire_once private buffer with (camera,frame_sequence) dedupe + exposure-time yaw gate (robot.yawRateAt), latch = resolveFromLandmark(mean of consistent window), timeout fallback = immutable field nominal, epoch change cancels. Config children: Targets, FieldMap, Evidence association_id.

**Composed profiles (config/composition.h/.cpp):** <Configuration id> + ConfigurationName/Loop/Robot file=/Field file= (repeatable)/Pipeline file=; paths relative to referencing file; fragment roots validated (Robot={Resources,Sensors}, Field=Resource|Field, Pipeline); repeated includes, .xml.in, any @TOKEN@ attr, cross-file duplicate resource/sensor ids fail with include chain + file:line; FNV-1a digest + id exposed via System::configurationId()/configurationDigest(), printed by main. buildFromFile auto-detects Configuration vs plain System root.

**wheel_geometry resource:** robot-description-owned measured geometry (per-Wheel calibration_status REQUIRED); tracking_wheel_odometry accepts EITHER inline <TrackingWheel> OR <Wheels resource_id><Use wheel_id/></Wheels> (mutually exclusive). PreprocessorInitializationContext gained resources ptr. ResourceInstance gained setResetHook/resetOnce + ResourceMap::resetAll (pico_telemetry resets once, channel closures no longer reset shared state).

**Config tree:** config/shared/robots/gatr2_as5047_bno08x.xml.in (all physical facts, tokens, tagCircle21h7 detector, camera); shared/pipelines/{two,three}_wheel_bno08x_{no_correction,camera_diagnostic}.xml (value-free, vex_brain publishing, camera ones use landmark_world commit=always diagnostic association); override/field.xml (9 landmarks/36 tagCircle21h7 mounts, provisional -> needs --allow-provisional); override/diagnostics/*.xml.in (Configuration wrappers referencing the not-yet-existing measured robot file); blue/red/skills routes EMPTY on purpose (user: never invent routes/targets/wire ids). OLD root configs with guessed geometry DELETED (navigatr.xml, *_no_landmark_correction.xml, old .in templates).

**Build gate (user-mandated):** builds + gtests ONLY. -Werror promotion removed everywhere (-Wall -Wextra kept); CI format job deleted; .gitignore ignores build*/ dirs. Verified: common 35/35, navigatr 116/116 (13 test files; world_prediction_gtest -> world_estimation_gtest, new profiles_gtest for composition/wheel-geometry/checked-in-tree honesty), pico `pio run` builds.

**Preserved (do not rewrite, per user):** signature-scoped FunctionRegistry w/ plain names, strong opaque IDs, ResourceMap/SensorMap/SensorResultsMap, factory-captured SensorExecutable, TypedPayload, explicit noops, lazy resource deps + cycle detection, wheel math, transactional acquisition/exposure-time fusion/repeated-tag gates/invalid-pose protections/target-gated detection (rounds 3-4).

**Still pending (round-5 scope not implementable on Windows dev box or needing user values):** real libcamera capture + AprilRobotics tagCircle21h7 adapter + shared pi/vision lib + camera_calibrate/camera_inspect tools; Pico BNO08x SPI driver + versioned wire protocol (CRC-16, length-delimited) + acquisition handshake (§2.9/2.10/§3); AS5047P acquisition hardening (§4); fusion interval-alignment/pose-age/residual health (§5, partial); richer record envelopes + checked writers + structured diagnostics + facade (2.4/2.5/2.7/2.8 partial); paced replay. User values questionnaire (§9) unanswered: BNO module/wiring, robot origin, wheel geometry/CPR, camera choice/mounting, two-wheel pair confirmation.

See [[project-wire-format]], [[feedback-design-principles]], [[project-current-state]].
