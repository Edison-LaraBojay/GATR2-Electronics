# User request 2, verbatim (2026-09-27)

This is an ADDITIONAL implementation task to incorporate into your current workflow. Continue the existing architecture work; integrate these requirements into it.

The two additions are:
1. Proper sensor calibration and per-wheel tuning.
2. Robust initialization, reconnection, and recovery across the Brain, Pi, Pico, and sensors.

Run independent work in parallel where practical. Coordinate ownership of shared interfaces, protocol definitions, configuration, and application files before editing them. If something depends on unfinished contracts, sequence that work rather than creating a competing implementation.

Inspect the current tree first. Some issues identified below may already have changed during your work.

1. CURRENT HARDWARE AND REQUIRED APPLICATION

The immediate hardware configuration is:

- Two PERPENDICULAR tracking wheels.
- naviGATR/Pico encoder port 0: forward/backward.
- naviGATR/Pico encoder port 1: sideways.
- VEX IMU: Brain Smart Port 1.
- Brain–Pi communication: USB.
- Pico–Pi communication: existing UART.
- Wheel radius: provisionally 0.024 m, editable and clearly labeled unmeasured.
- Other unknown geometry remains explicitly configurable.
- No camera or AprilTag requirement for this test.

brain/localization-test is the actual PROS application I build and upload to the Brain. Integrate these additions into that application using the shared driver and normal naviGATR pipeline.

Keep localization-test motor-free. It must let me configure the robot, calibrate, set placement, read pose, and observe recovery. The broader navigation application can continue integrating planning and motor control separately.

The Pi’s existing 3D inspection interface must continue showing the robot and nominal field without camera/world correction. Add useful calibration and recovery status to the existing inspection system.

An absent or broken external BNO08X must not prevent this VEX-IMU profile from becoming ready.

Preserve the intended external Pico IMU path, supported three-wheel configurations, and explicit USB/RS-485 selection from the main task.

2. CONFIGURATION AND PIPELINE OWNERSHIP

Use the Brain-owned typed RobotProfile/configuration mechanism already being developed. Do not create another configuration protocol or hardcoded bench localization implementation.

Robot-specific tuning should primarily be editable in Brain C++:

- Logical encoder port.
- Wheel radius.
- Wheel measuring direction.
- Wheel mounting position relative to the robot origin.
- Encoder polarity.
- Actual encoder counts per revolution.
- Gearing, if applicable.
- Per-wheel measured travel correction.
- IMU source and supported calibration settings.

Keep physical GPIO mappings, hardware driver details, and Pi resource configuration in their existing appropriate layers.

Apply corrections at a clearly defined point:
resource acquisition -> sensor conversion/calibration -> robot observations -> state estimation.

Do not apply polarity, scale, mounting transforms, or bias corrections twice.

Configuration changes must rebuild or update the affected captured stages correctly. Editing a backing geometry object is insufficient if an estimator copied its values during construction.

Changing calibration or geometry must establish appropriate new baselines/generations so the change cannot appear as robot movement.

3. PER-WHEEL CALIBRATION AND TUNING

Support independent calibration of each wheel assembly while separating:

A. Physical geometry:
   radius, mounting position, measuring direction.

B. Encoder configuration:
   counts per revolution, polarity, gearing.

C. Empirical correction:
   an optional per-wheel travel scale, default 1.0.

Do not silently change the mounting offset or falsify encoder CPR to compensate for unrelated errors.

Be honest about identifiability: a measured travel test observes a combined distance scale. It cannot independently determine wheel radius, gearing error, slip, and encoder error.

Provide a small usable calibration workflow associated with localization-test:
- Capture fresh start/end wheel measurements.
- Supply an independently measured travel distance.
- Compute/report a proposed correction.
- Allow applying it through the normal Brain configuration.
- Show the active values and calibration outcome.

Use whichever small interface best fits the current application, such as controller/screen actions plus editable C++ inputs or an existing diagnostics interface. Document the exact procedure.

For the current perpendicular setup, forward and sideways wheel calibration must be independently usable.

Reject invalid runs: insufficient travel, stale/discontinuous data, inappropriate rotation, invalid reference distance, or source restart. Explain the rejection. Prefer repeatable trials over accepting one arbitrary movement.

Do not use naviGATR’s own fused displacement as the independent ground truth for calibrating the sensors that produced it.

Do not pretend a constant scale fixes:
- Slip.
- Missed encoder transitions.
- Changing wheel compression.
- Angle-dependent magnet eccentricity.

Inspect the current A/B acquisition before proposing periodic correction. Its incremental count starts at arbitrary zero; a repeatable angle-error table requires a repeatable shaft-phase reference, such as index, absolute angle, or homing.

Implement useful per-wheel linear calibration now. Do not ship a fake periodic correction based on arbitrary startup count modulo CPR.

4. IMU CALIBRATION AND STATIONARY DETECTION

Review calibration across all active paths, including:
- Pico BNO08X startup alignment.
- Two-wheel heading constraint.
- Independent IMU heading observation used with fusion.
- VEX IMU bridge.

Previously, the two-wheel model checked wheel travel during bias collection, while the independent heading observation simply averaged initial samples. Eliminate inconsistent assumptions through shared policy/helpers where appropriate.

Use a continuous window of fresh, suitable evidence before accepting stationary calibration. Depending on capabilities, evidence can include:
- Wheel movement.
- Gyro magnitude and variation.
- Acceleration magnitude and variation.
- Sample continuity and freshness.

Requirements:
- Missing or repeated data must not count as new stationary evidence.
- Movement invalidates/restarts the candidate window.
- Zero motor commands alone do not establish stationarity.
- Unchanging wheel counts alone do not establish stationarity.
- Calibration must have bounded attempts and visible waiting/failure reasons.
- Retry without blocking unrelated acquisition or communication.
- Use elapsed time/sample support appropriately; avoid treating an arbitrary number of irregularly spaced samples as an equivalent calibration window.

For the external IMU:
- Preserve XYZ handling and arbitrary rigid mounting support.
- Distinguish startup gravity alignment, gyro bias calibration, mounting orientation, and field placement.
- Gravity alone does not determine mounting yaw or field heading.
- A startup gravity projection is not dynamic roll/pitch compensation.
- Handle the supported attitude path honestly; do not claim that later rocking is corrected when only a frozen axis is used.

For VEX:
- Inspect the actual available measurements.
- The existing bridge has primarily forwarded heading and validity.
- Do not invent XYZ acceleration, attitude, or raw gyro bias from heading-only telemetry.
- If additional PROS measurements are useful and available, extend the normal typed bridge/protocol with units, timestamps, validity, and discontinuity semantics.
- Otherwise expose the limitations of a reduced-evidence mode.
- Define ownership of VEX firmware calibration versus host correction so calibration is not accidentally applied twice.

Calibration/recalibration must be accessible from localization-test without restarting the program.

5. CONSERVATIVE STATIONARY UPDATES / ZUPT

Add useful stationary drift handling through the normal localization architecture.

For the existing wheel-based estimator, prioritize:
- Qualified stationary detection.
- Correct stationary velocity reporting.
- Carefully gated gyro bias maintenance where supported.
- No integration of known-invalid intervals.

A formal zero-velocity filter update only belongs in an estimator that actually models the relevant velocity state and uncertainty. Do not describe a simple clamp as a complete ZUPT filter.

Stationary handling must:
- Preserve field placement.
- Avoid hiding genuine slow movement.
- Immediately stop learning from evidence that becomes unsuitable.
- Respect source epochs, calibration changes, and freshness.
- Avoid presenting sensor-derived stationary evidence as independent information that can be double-counted.
- Avoid claiming to recover position error accumulated before the stop.

If this needs different algorithms for different IMU capabilities, share the policy and status contract without forcing identical mathematics onto incompatible measurements.

6. INITIALIZATION AND RECOVERY MODEL

Implement explicit lifecycle behavior instead of relying on happy-path startup order.

A communication timeout means communication was lost; it does not prove a device rebooted.

Use identities/generations sufficient to distinguish:
- Brain program instance/session.
- Pi program instance/session.
- Pico acquisition instance.
- Individual sensor reinitialization.
- Calibration/configuration discontinuities.

Do not rely solely on timestamp regression to detect every Pico restart.

Required behavior:

Cable interruption, same instances:
- Reconnect automatically.
- Reject stale/replayed data.
- Determine whether measurement continuity was preserved.
- Avoid unnecessary resets.

Brain restart:
- Establish a fresh session.
- Reapply/validate the robot profile idempotently.
- Cancel old movement commands.
- Do not reset all Pico acquisition simply because a Brain session changed.

Pi restart:
- Reconnect to Brain and Pico regardless of startup order.
- Recover configuration through the existing initialization contract.
- Rebuild valid measurement baselines.
- Expose whether field placement is still valid or must be supplied again.

Pico restart:
- Announce a new acquisition identity.
- Pi discards old baselines and invalid intervals.
- Counts returning to zero must never look like large robot movement.

Individual sensor failure:
- Retry the affected sensor automatically while it is enabled.
- Keep unrelated sensors and communication running.
- Reinitialize/calibrate the affected sensor when necessary.
- Mark discontinuity before accepting its new measurements.

Do not silently restore the original starting pose after an arbitrary reconnect. Initial application placement and recovery placement are different situations.

Communication recovery must not automatically resume an interrupted movement command.

Inspect and fix relevant existing weaknesses, including:
- Serial resources that only attempt to open during construction.
- BNO startup failures that permanently prevent later initialization attempts.
- Source restarts that rebaseline but incorrectly retain invalid calibration.
- Application initialization paths that only try once.

Use bounded attempts and retry delays. “Keep retrying” must not mean a blocking loop or a busy loop.

Recoverable hardware failures should not require manually restarting either program. Permanently absent/broken hardware should remain explicitly unavailable while the rest of the system continues operating.

7. PI -> PICO CONTROL

Add a small control mechanism on the existing link if the main implementation does not already provide one.

Support clearly distinguished operations, conceptually:
- Configure supported acquisition/sensor settings.
- Reinitialize a particular sensor.
- Explicitly restart acquisition.

An ordinary reconnect should not automatically zero every counter.

An explicit full acquisition restart may reset counts, but it must also change acquisition identity/epoch and invalidate old Pi baselines.

A sensor-only restart should preserve unrelated acquisition where possible.

Commands need:
- Protocol/version validation.
- Session/instance targeting.
- Request IDs.
- Defined duplicate handling.
- Accepted/running/completed/failed status, as appropriate.
- Bounded execution and clear failure reasons.
- Recovery/resynchronization after disconnect.

An acknowledgement lost in transit must not cause repeated command retries to repeatedly reset counts or restart calibration.

Do not infer completion merely from command receipt. Report when the sensor actually becomes ready, or when initialization/calibration is still pending.

Keep normal telemetry flowing during recovery. Preserve existing framing, scheduling, and shared protocol ownership rather than adding ad hoc console commands.

The Pico should also retry enabled sensors locally without requiring the Pi to issue a command every time something fails.

8. INTEGRATION AND ACCEPTANCE

The primary acceptance path is:

Brain localization-test
-> shared communiGATR USB transport
-> normal Pi configuration and localization pipeline
-> Pico encoders + Brain VEX IMU
-> robot pose back to Brain and existing 3D visualizer.

It must work without external BNO, camera, or AprilTags.

Expose enough status to distinguish:
- Connecting/reconnecting.
- Configuration pending/rejected.
- Sensor unavailable/initializing.
- Waiting for stillness.
- Calibrating.
- Ready.
- Needs placement.

Include meaningful tests for:
- Independent perpendicular-wheel calibration.
- Invalid and discontinuous calibration runs.
- Calibration while moving, stopping, and moving again.
- Freshness and duplicate-sample handling.
- Sensor-specific capabilities.
- No double application of corrections.
- Brain/Pi/Pico startup in different orders.
- Initial failure followed by successful retry.
- Cable disconnect/reconnect.
- Individual device/sensor restart.
- Duplicate and interrupted control commands.
- Counter/heading resets without false displacement.
- Missing optional IMU while the VEX profile operates.
- Recovery without automatic movement resumption.
- Existing three-wheel fusion and supported external IMU behavior.

Build the affected firmware, PROS applications, and host targets where the toolchains are available. Distinguish software tests from hardware validation.

Coordinate this as an addition to the current workflow:
- Freeze shared recovery/calibration contracts with the integration lead.
- Parallelize independent calibration, recovery, and test work when file ownership permits.
- Serialize shared protocol/profile/application integration.
- Avoid simultaneous builds into the same build directory.
- Preserve unrelated changes.
- Do not edit PCB files.
- Do not commit, push, flash, or deploy.

Update current usage documentation and give me:
- Exactly where Brain tuning lives.
- How to perform per-wheel calibration.
- How to request IMU calibration/reinitialization and placement.
- What recovery happens automatically.
- What requires explicit placement or calibration input.
- Exact Brain/Pi/Pico build and update instructions, labeled by platform.
- Tests actually run and remaining hardware limitations.

Keep the immediate USB + perpendicular two-wheel + VEX-IMU localization-test working throughout integration. Do not leave these additions as a separate experimental path that the uploaded application never uses.
