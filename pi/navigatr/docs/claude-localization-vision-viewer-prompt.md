# Localization, live vision, and field inspection implementation prompt

Implement this combined task in the existing repository. This is an implementation request, not a request for another proposal. Complete the localization restructure, integrate working live camera/AprilTag support, and build a usable real-time browser inspector with a 3D field view and camera overlays.

The previous localization-only prompt has NOT been implemented. This prompt replaces it in full. Camera bring-up, independent scheduling, and inspection tools are now in scope; they must not remain deferred merely because an earlier proposal deferred them.

## 1. Starting point and scope

The uncommitted Resources/Sensors restructure is the starting point. Its reported baseline is 136 passing tests; verify the actual current baseline rather than assuming that report still describes the working tree. Preserve unrelated changes. Do not reset, stage, or commit work.

Keep implementation, configuration, tests, tools, and documentation under pi/, with the root README updated if needed. Read common/ and firmware interfaces where necessary to understand real inputs. Do not change PCB files, Brain code, Pico firmware, or shared wire protocols in this pass. In particular, do not quietly invent new firmware telemetry to make an attitude demonstration appear live.

Read pi/navigatr/docs/architecture.md, coordinates.md, landmarks.md, the current README, and the actual implementations. docs/restructure-context.md provides background; older alternatives do not override this prompt.

Preserve intended functionality, supported hardware paths, algorithms, and calibration behavior through native implementations of the new contracts. Delete superseded infrastructure once its useful behavior has moved. Do not keep legacy wrappers, alternate old pipelines, or XML aliases solely to preserve obsolete architecture. Correct demonstrated bugs rather than reproducing them.

The final product must run headlessly on a Raspberry Pi 4. I will connect over SSH and open the inspector in my computer's browser. The main deliverable is a useful live view of the actual program: current localization, configured field geometry, reported landmark estimates, camera images, and the AprilTags it detects.

## 2. Preserve readable construction and execution

Each category uses registered factories selected by XML type. Factories initialize configured instances once; aggregate builders capture their children, bindings, and persistent state. Multiple instances may use the same type. Generic executors own iteration, validation, and map assembly.

Preserve the logical chain:

    ResourceXml -> make_resources -> execute_resources
    SensorXml   -> make_sensors   -> execute_sensors
    LocalizationXml -> make_localization -> execute_localization

    ResourceMap = execute_resources(context)
    SensorMap = execute_sensors(ResourceMap, context)
    RobotState = execute_localization(SensorMap, requests, context)

The scheduler may invoke acquisition and consumers independently; this dataflow does not require them to share one blocking loop. Retain the readable aggregate calls and single acquisition ownership when introducing scheduling.

ResourceStore owns initialized dependencies; ResourceMap carries runtime resource results. Keep named output IDs even for single-output resources. Sensors consume the read-only ResourceMap through configured, typed bindings. Avoid arbitrary field-query languages and hardware-name branches in generic stages.

A callable C++ wrapper exposing lifecycle operations and output declarations is fine; literal lambdas are not mandatory. Capture owned configuration, not references to temporary XML nodes. Keep runtime requests explicit instead of hiding mutable System access in implementations.

## 3. Make localization a self-contained component

make_localization constructs and captures a configurable collection of robot-observation functions, one selected state estimator, persistent model/estimator state, the field anchor, and history/publication ownership.

Internally:

    observations = execute_robot_observations(sensor_map, context)
    update = execute_state_estimation(observations, previous_state,
                                      requests, context)
    // Finalize the accepted update and publish coherent state/history.
    return current_robot_state

Use RobotObservationMap consistently. Observation construction and state estimation are configurable responsibilities. Finalization is bookkeeping, not a third XML-selected "commit localization" plugin. System must not own model baselines or append localization history.

Observation factories bind configured sensor IDs, compatible payloads, model geometry, and named outputs. Validate missing/duplicate IDs, payload mismatches, output declarations, geometry, and estimator compatibility at build time.

Group observation functions by measurement model, not by hardware brand or one-function-per-sensor. A model may consume one sensor, several, or a configured list. Do not require every source to produce a complete pose. Insufficient geometry must fail or remain explicitly partial, never acquire fabricated components.

Initially, migrate the tracking-wheel model to produce a body-motion increment and the existing integrator to consume it. Keep typed observations extensible to supported angular, velocity, or pose constraints without implementing an unrelated new EKF or general execution graph.

Observations preserve measured components, frame/origin, time or interval, contributing source identities/sequences/epochs, and available quality/uncertainty. Preserve lineage: a motion observation already using an IMU is not independent of another observation of that same IMU. Unsupported observations must not be silently ignored or counted twice.

## 4. Port algorithms and fix timing/lifecycle behavior

Inspect and migrate useful behavior from impl/preprocessing/tracking_wheel_odometry.*, impl/preprocessing/imu_normalization.*, and impl/localization/wheel_imu_prediction.*, including their configuration and tests.

Preserve geometry-based wheel solving and mounting-offset compensation; supported two-wheel-plus-heading, three-wheel, and additional-wheel configurations; units/signs and encoder wrap handling; stationary IMU bias calibration and restart-on-motion behavior; wheel rebasing during calibration; packet-batched gyro integration; gap/outage handling; straight/arc pose integration; intended optional orientation inputs through compatible time intervals; edge-triggered initialization; and clock mapping.

Put sensor-only conditioning in Sensors where their inputs support it. Joint wheel/IMU calibration logic may remain with the measurement model. Give each correction one owner. Preserve the Pico resource's lossless packet-level gyro accumulation or an equally lossless implementation; integrating just the last rate in each packet batch is a regression.

Inventory actual sensor adapters and robot profiles. Preserve every currently supported device path natively. Distinguish implemented support from planned backends.

Correct these known upstream gaps:

- Publication currently carries measuredAt and payload while successive stages assign receivedAt again. Preserve actual upstream host receipt time; processing/publication time must not become the clock-synchronization input.
- Carry source clock identity/domain and discontinuities through derived records. Stage-local sequence numbers do not replace source provenance.
- pico_channels recognizes changed input epochs as new samples without rebasing the encoder's previous-count baseline. Rebase affected state on detected restart/discontinuity while preserving normal supported counter wrap.
- ChannelInput::fresh can treat unseen retained data as healthy before considering its current status, and does not consistently propagate unavailable status. Define per-output health semantics. Preserve a good sample decoded before link closure without relabeling invalid retained data as fresh evidence.

Reject or explicitly handle nonpositive intervals, unsupported gaps, and accumulator discontinuities. Do not bridge invalid spans using endpoint integration. Combining unrelated intervals by choosing the largest dt is not synchronization.

Track retained samples by source identity, sequence, and epoch. Models own bounded pending inputs and time-matching policies for their particular requirements; never wait for every configured sensor before any localization can run. Accepted, pending, and rejected evidence have explicit disposition so failure/retry neither loses accumulated motion nor integrates it twice. Straightforward ownership is sufficient; no generic transaction framework is required.

Without evidence or an explicit prediction model, keep the estimate's previous effective time. Loop iterations do not refresh measurements.

## 5. Own history and coordinate publication inside localization

Localization is the sole writer. System, landmark estimation, and instrumentation receive coherent latest-state snapshots and timestamped lookup access. A factory may capture a publication writer while System retains the reader. Use synchronization or immutable snapshots; const references into concurrently mutated containers are insufficient.

Keep history separate from lightweight RobotState. Implement a time-ordered ring with a configurable retention target initially five seconds and explicit capacity behavior appropriate to publication rate. Binary-search logical order across physical ring wrap. Support exact timestamps and bracketed position/shortest-yaw interpolation, with configurable gap gates. Define same-time revisions; reject expired, unsupported future, out-of-order, and incompatible-epoch data. Apply equivalent validity rules to yaw-rate lookups. A consumer may wait boundedly for a future bracket; never silently clamp unsupported queries.

Do not substitute loop time when host measurement time is unavailable, or append a retained pose as a new estimate. Readers must see matching timestamps, anchor revisions, and coordinate epochs.

Preserve:

    T_field_robot = T_field_odom * T_odom_robot

Field re-anchoring and discontinuous odometry reset are different events. Invalidate or consistently transform affected history/landmark data. Robot pose always identifies the configured fixed chassis origin, not the optical center or a changing instantaneous center of rotation.

Localization must work without a camera, field-object catalog, landmark selection, or connected Brain. Provide a configuration/command path for initial placement using the existing request semantics.

## 6. Implement real acquisition and AprilTag processing

The current camera factory in impl/resources/cameras.cpp constructs DeadCamera. The factory in tag_detectors.cpp explicitly fails because the AprilTag backend is absent. Replace these production stubs with real backends; synthetic input alone does not satisfy this task.

Implement Raspberry Pi camera capture through the existing resource/sensor boundaries, using the supported Raspberry Pi/libcamera stack. Preserve exact capture mode, stride/pixel-format handling, frame ownership, calibration association, sequence/epoch, and exposure metadata. Camera buffer reuse must not corrupt a frame being processed or displayed. Document the exposure timestamp convention and mapping to the runtime monotonic clock; receipt is not exposure. Report timestamp uncertainty when the backend cannot provide reliable exposure timing.

Integrate the actual AprilTag library. The configured season uses tagCircle21h7; do not implement only a hardcoded tag36h11 demo. Preserve configurable families and physical detector-corner size semantics. Handle detector optical axes, canonical tag axes, and pixel-corner ordering correctly.

Allow image preview and 2D tag decoding without metric calibration. Metric tag poses and field updates require usable numeric intrinsics, distortion treatment, sizes, and extrinsics. Calibration labels in XML are human annotations only; runtime must not read them. Split or extend contracts so unavailable metric pose is explicit, never a zero/identity pose mistaken for a valid solve.

Use consistent pixel coordinates across detection, rectification/crop, pose solving, and overlays. Do not overlay undistorted or cropped corners on an unrelated raw image. Preserve optional quality measurements as optional; unavailable reprojection or ambiguity metrics are not zero error.

Expose all decoded detections before association filtering, including unknown or rejected tags, through the normal processing path. Camera/tag inspection must work without an active landmark selection. Do not open the camera or run a duplicate detector exclusively for the viewer.

Preserve and connect the existing field-estimation/association behavior needed to produce real live estimates. Repeated observed tag IDs are not unique physical mount identities. Keep ambiguous associations and rejected evidence visible. Seeing a landmark updates its estimate; a landmark estimate derived using localization is not an independent robot-localization correction.

Do not rewrite unrelated Brain reporting semantics or introduce a whole-field tracking algorithm just to populate the display.

## 7. Schedule acquisition, localization, perception, and inspection independently

Implement the decoupling needed for this live system now. Camera reads, tag detection, image compression, disk/network operations, and slow browser clients must not synchronously stall localization.

Prefer one estimation runtime with explicit worker ownership and a small optional inspection service; an in-process server or a separate server using bounded local IPC is acceptable. Choose a concrete design appropriate to the repository and implement the actual handoffs, lifecycle, and error handling. Do not duplicate sensor acquisition or estimation in another program.

Give each mutable device/model/estimate one writer. Readers use immutable snapshots or synchronized bounded handoffs. Do not share the retained mutable ResourceMap/SensorMap across threads by reference. Different workers must not race to execute/reset the same configured resource.

Bound queues, memory, client buffers, and work. Latest-frame replacement is appropriate for pending camera work; silently dropping motion increments or unaccounted gyro samples is not. Match detections to their original frame even if capture has advanced. No unbounded backlog, detached-thread lifetime hazards, or global lock held during detection/network transmission. Stop and join workers before releasing dependencies. Viewer disconnect/reconnect and disabled instrumentation must not stop estimation.

Do not assume localization is inherently faster than perception. Scheduling and rate configuration are independent of the selected sensor inventory. Instrument actual rates, latency, drops, and queue occupancy rather than claiming a Pi 4 performance result without measurement.

## 8. Add optional attitude support correctly

I want the robot visualization to show rocking/tilt when measured, and camera transforms to use that attitude at exposure time. Extend internal robot-state/observation/history contracts to support body attitude with an explicit frame/reference, timestamp, validity, quality, and source provenance. Prefer normalized quaternions or an equally sound rotation representation internally; expose roll/pitch/yaw for inspection with documented conventions.

Preserve existing planar localization behavior. Reconcile tilt and localization heading into one consistent orientation; do not apply IMU yaw or the field anchor twice. Calibrate IMU-to-body orientation. Attitude has its own availability and age; repeating an old attitude beside a newer planar pose does not make it fresh. Interpolate valid attitude samples using proper rotation interpolation and respect gaps/epochs.

Use the capture-time transform chain before planar projection:

    T_field_tag(t) = T_field_odom * T_odom_robot(t)
                     * T_robot_camera * T_camera_tag(t)

Account for the camera's translation and full mounting rotation. An offset camera moves when the robot rotates in place, while the fixed robot origin can remain stationary. Dynamic pitch/roll changes this transform as well. Missing vertical-position evidence must remain an explicit ground-plane/height assumption, not invented full 6-DoF localization.

The current shared telemetry exposes gyro_z and accel_xy, not a full attitude report. Do not claim robust roll/pitch from that yaw-only path, or infer rocking merely from acceleration without an appropriate model. Implement the optional Pi-side attitude path, connect any genuinely available compatible orientation source, and exercise it through replay/synthetic inputs. Otherwise show attitude unavailable; a level display fallback must be labeled assumed. Make the camera's degraded-mode policy explicit when attitude at exposure is missing or stale.

If live tilt requires a new Pico report/protocol field, document precisely what is missing as a follow-up outside this pass. Do not block the working planar viewer or falsely mark live attitude complete. Full attitude is an internal sensing capability, not a cosmetic UI animation.

## 9. Publish instrumentation from actual runtime data

Create a documented, versioned inspection contract independent of the Brain wire protocol. Publish bounded snapshots/events from actual runtime boundaries. Image encoding, serialization, and client delivery run outside estimation work.

Include runtime/session identity, configuration/calibration revision, clocks/epochs, robot state and validity, effective times, attitude availability, field/landmark estimates and provenance, selected reference when applicable, source health, and relevant performance counters.

For each inspected detection frame, provide camera identity, source epoch, sequence, exposure time, matching image, detected family/ID/corners, optional pose/quality, association candidates/result, and rejection reason when available. Publish detector errors through diagnostics rather than silently converting failure into an empty detection list.

Bind overlays to the exact image identity, not merely the latest available JPEG. Include enough timing information to distinguish current robot state, capture-time pose, and delayed landmark observations. Browser wall time is not the Pi's monotonic clock. On restart/reconnect, clear incompatible frames, trails, and selection state using session/epoch metadata.

Instrumentation is optional and read-only with respect to estimates. Browsers render server-produced data; they must not run a second estimator or mutate state to make the display look correct.

## 10. Build the browser inspector

Serve a complete, locally bundled UI from the Pi, defaulting to loopback. Supply an exact SSH port-forward command and browser URL, for example an implemented service on port 8765 reached through:

    ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>

No Pi desktop, X11 forwarding, cloud account, CDN, or internet connection should be required at runtime. Render WebGL/3D on the viewing computer; the Pi captures, estimates, and serves bounded data. Pin dependencies and provide repeatable asset/build setup without requiring a development server in production.

Make the 3D field the main view, with camera inspection alongside it. Provide:

- Orbit/pan/zoom, reset view, top-down view, and robot-follow mode.
- Recognizable field geometry, a simple configurable robot body, fixed origin and forward direction, optional measured tilt, and a bounded pose trail.
- Camera mounting position/orientation and an optional calibrated frustum.
- Clearly different nominal geometry and live estimated landmark poses, with object identity, age, source, and validity on inspection. A nominal ghost/outline and measured pose should make displacement/rotation easy to see.
- Camera image, correctly registered tag outlines/IDs, selected detection details, and accepted/rejected/unassociated status. Metric axes only when their calibration and pose are available.
- Clear status for connecting, stale, disconnected, no image, no tags, unavailable localization, unavailable attitude, and missing metric calibration data. XML calibration labels must not control the UI. Old imagery/estimates must visibly age instead of appearing live indefinitely.
- A concise diagnostics panel for source health, measurement age, acquisition/detection/localization/viewer rates, latency, and drops, with configurable camera preview quality/rate to manage bandwidth.

Use readable units, coordinate labels, and a clean layout. Clearly label object orientation versus heading error from nominal; preserve configured symmetry/association semantics. Do not infer which physical face is meant solely by nearest yaw when repeated tags or geometry leave it ambiguous.

Viewing the entire nominal field does not require maintaining observed tracks for every element. Display whichever estimates the runtime actually publishes, whether one or several. Never label CAD placement, a stale cache, or client interpolation as a fresh observation.

## 11. Use the actual field configuration and CAD

Start from config/override/field.xml. It identifies V5RC Override 2026-2027 and states that its provisional geometry came from the official 2026-04-26 STEP. Retrieve the matching official VEX field CAD/drawings, verify their relationship to that configuration, and record source URL, revision, units, axis/origin conversion, and extraction method. Do not silently replace configured coordinates with a different CAD revision.

Provide recognizable lightweight models, using offline conversion/simplification to browser assets or geometry derived from verified dimensions. Keep expensive STEP parsing/tessellation off the Pi and out of runtime startup. Avoid shipping a huge detailed assembly directly to the browser. Preserve required attribution/licenses and provide a reproducible asset-generation/download process where appropriate.

The runtime's resolved field definition is the source of nominal placement. Add explicit dimensions/visual metadata if currently missing; do not maintain a second hardcoded field in frontend JavaScript. A companion asset manifest may bind meshes/local visual offsets to stable configured object IDs. Keep mesh origins distinct from landmark/tag origins and apply their transforms explicitly. Verify units, handedness, and placement against known field points so a plausible-looking mirror or scale error cannot pass unnoticed.

Render unknown shapes with labeled simple geometry rather than preventing the rest of the field from loading. Keep CAD-derived values provisional where appropriate. Competition placement imperfections are what estimation must handle; do not ask me to measure those in advance or clamp observed objects back onto nominal placements.

## 12. Calibration and usable run modes

I authorize clearly identified placeholders for information that requires my hardware: camera model/mode, intrinsics/distortion, camera/IMU extrinsics, robot dimensions, encoder calibration, tag detector-corner sizes where not verified, and initial robot placement. Reuse supported CAD-derived nominal values rather than replacing them with arbitrary guesses.

Provide one clear inventory of remaining measurements and where each goes. calibration_status="provisional", "verified", "UNCONFIGURED", and any similar labels are purely human-readable XML annotations: runtime must not read or interpret them, require them, expose them as runtime state, gate behavior on them, or offer an --allow-provisional option. Validate actual required parameters, unresolved placeholder tokens, finite numbers, dimensions, and compatible geometry. Missing metric calibration remains explicit through absent intrinsics or unavailable transforms, independently of labels. Unknown values must not masquerade as measured defaults.

Deliver usable configurations for:

- Hardware-free demo/replay: the complete UI and normal runtime contracts work immediately with clearly labeled synthetic evidence, including moving localization, detections, a displaced landmark, and measured-versus-unavailable attitude cases.
- Live camera inspection: real images and 2D tag detections work before metric calibration is complete, with metric output explicitly unavailable as necessary.
- Live localization/field inspection: existing supported wheel/IMU inputs and calibrated camera inputs feed the same runtime and viewer. Cameras, landmarks, or browser clients may be absent without disabling basic localization.

Demo/replay should feed the real processing boundaries, not animate frontend-only fake telemetry. Include at least one real detector test using a correctly generated configured-family tag image. Document which demo paths use synthetic observations versus actual image detection.

Provide Pi setup/build/run instructions, feature/dependency checks, SSH access instructions, and reproducible startup/shutdown. Unsupported build features should produce explicit errors when requested, not a production backend that silently behaves like DeadCamera.

## 13. Verification and completion

Complete the work in dependency order, keeping the final system integrated. Remove superseded localization/preprocessing slots, factories, files, and build entries after migrating their behavior. Adapt field consumers to safe localization access. Update configurations and Pi documentation to describe the resulting system directly, with honest implemented/deferred coverage.

Port meaningful existing tests to native contracts instead of retaining obsolete classes. Add focused tests for:

- Binding failures, repeated implementation types, opaque IDs, supported geometry, calibration, integration, batch preservation, and no duplicate evidence consumption.
- Receipt preservation, source restart/epoch propagation, status retention, invalid intervals, and failed/retried updates.
- History ring/angle wrap, interpolation, expiry, unavailable host time, resets/re-anchors, and attitude timing/rotation interpolation.
- Delayed frames, nonzero camera mounting offsets, in-place turns, and pitch/roll, demonstrating correct fixed-robot-origin transforms.
- Actual configured-family detection, distortion/corner mapping, repeated-ID association ambiguity, and exact image/overlay identity.
- Slow camera/detector/client behavior, bounded buffers, independent localization progress, reconnect, device failure, reset, and orderly shutdown.
- End-to-end telemetry and a browser smoke test: load the field, move the robot, inspect a detection, displace/rotate an estimated landmark relative to nominal, demonstrate stale/disconnected states, and reconnect without mixed epochs.

Run the full host suite and relevant backend/viewer checks. The previous Windows host commands, from pi/navigatr in Git Bash, were:

    export PATH=/c/msys64/ucrt64/bin:$PATH
    cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
    cmake --build build -j8
    ctest --test-dir build --output-on-failure

Also validate the Linux/Pi build path as far as the available environment permits. Synthetic tests, cross-builds, and desktop browser checks do not establish Pi 4 camera performance or physical alignment accuracy. If hardware is available, measure the live path; otherwise provide the precise remaining hardware checks without claiming they passed. Do not reduce behavioral coverage to obtain a passing test count.

Finish with the implemented construction/runtime/worker flow, preserved algorithms/device paths, removed infrastructure, concrete correctness changes, exact launch/SSH/browser commands, actual test results, calibration inventory, and remaining hardware-dependent limitations. Do not stop at a plan, a disconnected frontend mockup, a telemetry schema without producers, or a synthetic-only replacement for live camera support.

## Primary references

- [Official VEX Override field reference and STEP description](https://www.vexrobotics.com/override-manual): locate the matching field CAD and verify dimensions/revision against the repository configuration.
- [AprilRobotics AprilTag implementation](https://github.com/AprilRobotics/apriltag): verify supported families, detection conventions, physical tag-size meaning, and pose-estimation interfaces.
- [Raspberry Pi camera software documentation](https://www.raspberrypi.com/documentation/computers/camera_software.html): verify supported capture/backend setup for the selected Pi OS and camera.

Consult current primary documentation for the exact dependencies you select. These references are implementation inputs, not evidence that hardware in this repository has been tested.
