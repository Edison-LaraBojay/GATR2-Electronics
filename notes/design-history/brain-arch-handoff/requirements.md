# User request, verbatim (2026-09-27)

You are working in GATR2-Electronics. Implement the following integrated Brain/Pi architecture. Inspect the current tree first; this prompt describes the intended result, not a guarantee that every existing implementation matches it.

Complete the implementation, examples, documentation, and meaningful verification. Do not stop after scaffolding interfaces.

1. Priorities and current hardware

The immediate priority is a working, configurable test setup:

- Tank drivetrain.
- Two PERPENDICULAR tracking wheels:
  - naviGATR encoder port 0: forward/backward.
  - naviGATR encoder port 1: sideways.
- Tracking-wheel radius: provisionally 0.024 m.
- VEX IMU on Brain Smart Port 1.
- Brain ↔ Pi communication over USB for the current test.
- Pico supplies encoder readings to the Pi through the existing UART.
- Pi localization and the existing browser/3D visualizer must continue working.

The intended permanent IMU is an external BNO08X acquired through the Pico. Its physical bring-up is unresolved. RS-485 hardware communication is also unresolved. Preserve both implementations, but do not claim these hardware issues are fixed merely because software tests pass.

The full implementation must support:

- Tank and mecanum drivetrains.
- Holonomic and non-holonomic path planning.
- Direct movement and obstacle-avoiding movement.
- Two perpendicular tracking wheels plus IMU.
- Three tracking wheels plus IMU, including appropriate fusion.
- Explicit USB or RS-485 selection.
- Brain-side robot configuration.
- Field definition and field-estimate transfer.
- Reconnection/reinitialization without manually restarting both programs.

Swerve should have sensible extension points, but implementing coaxial/differential swerve is NOT required in this task.

Do not modify PCB projects. Preserve unrelated working-tree changes and coordinate with any other agents working in the repository. Do not commit, push, flash hardware, or deploy services unless separately requested.

2. Inspect and preserve the existing work

Relevant areas include:

- brain/communiGATR
- brain/investiGATR
- brain/testing
- brain/localization-test
- brain/gatr2_brain.mk
- common/frames.h and the common codec
- pi/navigatr
- pico/src/board.h and existing Pico sensor implementations

The working USB transport currently lives partly in brain/localization-test. It wraps the existing communiGATR Client/Driver rather than implementing an unrelated protocol.

Current investiGATR includes waypoint following and PID control. brain/testing contains a PROS motor/task wrapper. Refactor these deliberately; do not duplicate the existing control logic.

The Pi already retains estimates for all configured landmarks, but ordinary Brain reporting currently exposes one selected landmark. Visualization geometry currently has display-only semantics.

Resources/Sensors/localization currently bind configuration at construction. Moving configuration to the Brain requires a real configuration/application mechanism, not merely adding setters to a geometry object that an estimator already copied.

Preserve the corrected perpendicular-wheel bench behavior and the established localization timing, lineage, and reset rules.

3. Establish a clean component split

communiGATR:
- Communication, sessions, transport, configuration exchange, pose/map reception.
- Implements a source-independent state/field interface.
- Does not plan routes or control motors.

investiGATR:
- Destination resolution and path planning.
- Accepts robot state, a destination, obstacle information, and robot motion constraints.
- Produces a complete path with explicit success/failure.
- Does not own PROS motors or path-following PID loops.
- Does not force every follower to use a “next point to chase” algorithm.

actuGATR:
- New reusable library under brain/actuGATR.
- Owns path following, pose-feedback control, settling/completion, and drive execution.
- Contains drivetrain adapters and hardware adapters.
- Move existing applicable PID/following logic here rather than duplicating it.
- Keep portable logic separate from PROS-specific code.

Use small, clear interfaces for:
- State/field source.
- Robot motion model and limits.
- Path planner.
- Path follower/controller.
- Drivetrain kinematics/execution.
- Motor/module I/O where necessary.

Use descriptive aliases for complicated types. Avoid an unnecessarily large inheritance hierarchy.

The common chassis command should have explicit physical units and a declared frame, such as body-frame:
- vx: forward m/s
- vy: leftward m/s
- omega: CCW rad/s

Existing normalized forward/turn demands cannot simply be renamed to m/s. Implement the conversion and motor velocity control properly, using appropriate feedback or an explicitly documented supported control method.

4. Motion models and supported drivetrains

The planner must receive motion capabilities, footprint, and relevant limits. Do not hardcode individual drivetrain names throughout planning code.

Holonomic:
- Support independent translation and heading.
- Mecanum must actually support lateral movement and heading control.

Non-holonomic:
- Tank drive cannot move sideways.
- Tank drive can turn in place.
- Plan executable movements instead of generating lateral motion and dropping vy later.

Different drivetrains may share geometric planning algorithms, while their controllers and kinematics differ.

Implement real tank and mecanum adapters, including:
- Motor ordering and reversal.
- Drive-wheel dimensions and gearing where needed.
- Coupled wheel-speed/output limits.
- Output/braking behavior.
- Manual/navigation/disabled ownership.
- One authoritative motor writer.
- Immediate stopping for invalid/stale commands and required stop conditions.

Keep tracking-wheel dimensions separate from driven-wheel dimensions.

Make room for future steering/module constraints without claiming unsupported swerve implementations exist.

5. Unified movement API

Provide two clear public movement commands, approximately:

motion.goToDirect(destination, relativeTo = Field::Origin, options);
motion.goToAvoiding(destination, relativeTo = Field::Origin, options);

Exact naming can be refined for consistency, but preserve these semantics:

- One command for direct movement.
- One command for obstacle-avoiding movement.
- Both use the SAME reference argument.
- Omitting the reference means Field::Origin.
- Do not require separate absolute and landmark-relative versions of each command.

Examples:

motion.goToDirect({1.0, 0.5, 0.0});
motion.goToAvoiding({-0.4, 0.0, 0.0}, Field::SomeLandmark);

Positions and heading are relative to the supplied reference. Apply the reference translation and rotation exactly once.

Field::Origin is a virtual reference, not a physical obstacle.

Keep season-specific named references in generated/application configuration, preferably derived from the canonical field definition. The generic libraries must not hardcode this season’s landmark names or count. Validate map identity/version so a named reference cannot silently identify the wrong object.

If preserving robot-relative destinations, express them through the same reference mechanism, with clear “robot pose when command begins” semantics.

Direct mode:
- Plans direct translation, with heading/turn phases appropriate to the motion model.
- Field-origin commands work without a landmark map or AprilTags.
- Does not secretly perform obstacle avoidance.

Avoiding mode:
- Requires a usable obstacle map.
- Reports blocked/invalid/no-path conditions explicitly.
- Never silently falls back to direct driving.

Preserve cancellation, command identity, timeouts, completion/settling, source changes, and stale-state handling.

6. Obstacle representation and planning

Use conservative 2D rectangular footprints.

A landmark’s box covers its maximum horizontal protrusion. For example, a 5×5 base with a 20×20 top is represented by a box covering the 20×20 extent. Do not implement height-dependent collision modeling or CAD-mesh collision in this task.

Include:
- Landmark pose and box dimensions.
- Box rotation with landmark heading.
- Box offset relative to the landmark origin where needed to enclose asymmetric geometry.
- Field boundaries.
- Explicitly configured fixed obstacles.
- Robot footprint and clearance/uncertainty margin.

Add explicit collision semantics to field configuration. Do not automatically turn every visual feature into an obstacle: tape and decorative elements are not obstacles. Reuse trustworthy dimensions where appropriate, but do not assume display geometry is a verified collision model.

Use a straightforward, bounded planning algorithm appropriate for this small field. Explain the choice. A conservative enclosing robot footprint is acceptable initially if its limitations are documented.

Check:
- Initial pose.
- Exact destination.
- Translation segments.
- In-place turns and final heading changes.
- Footprint about the actual reported robot origin.
- Execution/tracking clearance.

The current follower can advance early at waypoints and cut corners. Fix the planner/follower integration so collision-free waypoint segments do not become a colliding executed trajectory.

When accepted field corrections affect the route or its destination, revalidate and replan as needed. Preserve command identity and the original timeout; avoid repeatedly resetting progress because of tiny estimate changes.

Scope is known field elements and their placement corrections. No ball detection, opponent tracking, LiDAR, or general 3D obstacle perception.

7. Field transfer and protocol

The number of landmarks comes from configuration and may change between seasons.

Use:
- An explicit count.
- Fixed-layout records.
- Explicit versions/revisions and validity/source information.
- Existing framing/CRC concepts.

Do not use a zero-byte terminator. Binary records naturally contain zero bytes.

At initialization, transfer the complete nominal field needed by the Brain:
- Map identity/version.
- Stable semantic object IDs.
- Nominal poses.
- Collision boxes.
- Field bounds and other required planning metadata.

Physical landmark identity is not necessarily AprilTag printed ID; printed IDs may repeat.

During operation, provide complete field-estimate snapshots through counted records. Include all configured objects, with nominal/observed source and measurement age/status. Do not make obstacle knowledge depend on the one currently selected alignment target.

The public state source should expose a coherent complete cached map/snapshot. It should not expose half-assembled packet fragments to planning.

The current bounded frames cannot hold an arbitrarily large field:
- Implement paging/chunking where needed.
- Validate counts, offsets, lengths, duplicates, and memory bounds.
- Include sufficient snapshot/map/frame identity to reject mixed generations.
- Publish an assembled snapshot atomically.
- Recover from interrupted transfers and reconnects.
- Test fields with different counts and more records than fit one packet.

Keep pose polling responsive. Map transfer/world updates may use a separate configured cadence. Do not resend immutable box geometry with every pose reply.

Maintain request/reply bus ownership and RS-485 timing. The Pi should not start unsolicited transmissions. Do not add automatic transport-priority/failover behavior.

Version protocol changes deliberately and update codecs, fake peers, tests, and documentation together. Unsupported peers/configurations should fail clearly.

8. Brain-owned robot configuration

Programmers should normally configure the robot in Brain C++ and rebuild/upload the Brain, rather than edit the same robot settings in multiple Pi XML files.

Brain-owned settings:
- Logical encoder ports.
- Wheel radius, counts/revolution, direction reversal.
- Wheel mounting offsets and measurement directions.
- Two-wheel/three-wheel localization profile selection.
- IMU source and logical IMU port.
- VEX Smart Port when using the temporary Brain IMU source.
- Robot footprint/reference geometry.
- Robot-specific camera mounting transforms where applicable.
- Motor ports, reversal, gearing, controller gains, and movement limits.
- Initial field pose.

Pi-owned settings:
- Linux devices and acquisition interfaces.
- Available sensor/resource implementations.
- Field definition.
- Camera intrinsics/distortion and detector configuration.
- Runtime/inspection settings.

Pico-owned settings:
- Board revision and physical GPIO mapping.
- Sensor bring-up and hardware driver selection.

The current Pi HAT v2 mapping already provides:
- Encoder port 0: A0/B0, J2.
- Encoder port 1: A1/B1, J3.
- Encoder port 2: A2/B2, J4.
- One current external IMU connection.

Expose logical ports, not GPIO numbers, in Brain configuration. Name/index the IMU port so additional IMU ports can be supported later, even though only port 0 currently exists.

Selecting an IMU source in Brain configuration must not pretend to change the Pico’s compiled hardware driver.

Implement a typed, versioned RobotProfile exchange:
Connect → submit profile → validate/apply → acknowledge accepted profile
→ sensor/calibration readiness → placement → normal operation.

Do not send arbitrary XML or filesystem paths from the Brain.

Apply configuration at a controlled boundary:
- Same profile retry is idempotent.
- Unsupported channels/topologies fail clearly.
- No silent fallback geometry.
- Changed sensor geometry/source/origin resets affected baselines/history and advances the appropriate generation.
- Do not mutate an existing copied wheel map and assume the solver updates.
- Keep acquisition and inspection available while waiting for a profile when practical.

After the supporting software is installed, changing supported robot geometry/topology should require only a Brain configuration change and upload.

Explicit USB/RS-485 selection can still require matching Pi connection configuration. Document that exception honestly; do not turn this into an auto-transport project.

9. Localization and IMU requirements

Primary test:
- Encoder 0 forward.
- Encoder 1 sideways.
- VEX IMU on Smart Port 1.
- USB transport.
- Independent forward and sideways displacement.

External IMU:
- Preserve BNO08X acquisition and existing calibration/alignment behavior.
- Support selecting the normal Pico IMU output through the Brain robot profile.
- Preserve freshness, validity, source epochs and reset behavior.

VEX IMU:
- Keep the existing optional measurement-forwarding capability as a bench source.
- It must work in the updated testing applications through shared components.
- Do not make the generic planner/controller depend on VEX IMU details.
- Retaining the existing arrival-time approximation is acceptable if explicit; do not claim it synchronizes the Brain and Pico.

Three-wheel mode:
- Preserve and expose three-wheel localization and IMU fusion.
- Switching to supported three-wheel geometry should be a Brain profile change once the necessary Pi/Pico software is installed.
- Provide an actual tested three-wheel example, not a placeholder enum.

Fusion must respect the existing architecture:
- Two perpendicular wheels may use IMU rotation as a heading constraint.
- Do not then count that same gyro a second time as independent fusion evidence.
- Three-wheel fusion can combine unconstrained wheel rotation with an independent gyro observation.
- Preserve physical measurement lineage, named clock identity, exact interval support, bounded waiting, and exactly-once consumption.
- Do not weaken these checks to make mixed Brain/Pico clock data pass.
- Any unsupported combination must be explicit rather than silently presented as fused.

10. Reconnect, reset, and recalibration

Support, without manual program restarts:
- USB cable removal/reconnection.
- Serial transport reopening where applicable.
- Brain restart while Pi remains running.
- Pi restart while Brain remains running.
- Interrupted profile/map exchange.
- Pico/sensor restart.
- Explicit recalibration/reinitialization.

On recovery:
- Re-establish session/configuration/map synchronization.
- Reject stale replies and old-generation data.
- Resume valid state reporting when readiness is restored.
- Keep motor output stopped when required inputs are unavailable.
- Do not automatically resume an interrupted driving command.

Do not teleport the robot back to the configured starting pose after every transient disconnect. Preserve a valid anchor when continuity is established; expose a clear placement/reinitialization requirement when continuity is lost.

Initial program startup may deliberately apply the application’s configured starting pose. Distinguish that from an unexpected reconnect after the robot has moved.

Provide callable configuration/calibration/placement recovery operations and usable test-app controls. Recalibration must require the appropriate stationary condition; it must not silently begin while driving.

11. Shared transport and test applications

Move reusable USB support out of localization-test and into communiGATR.

Support explicit:
- USB.
- RS-485/Smart Port.

Keep their public driver/state-source behavior consistent. Preserve the working USB protocol envelope and account for PROS USB console behavior. Do not regress battery-only operation or block on a disconnected debug terminal.

Update both:

brain/localization-test:
- Shared driver/configuration.
- Current two-perpendicular-wheel + VEX IMU + USB setup.
- Pose placement, calibration/recovery controls, status and visualization.
- Remains motor-free.

brain/testing:
- Uses communiGATR + investiGATR + actuGATR.
- Current tank/two-perpendicular/VEX/USB profile.
- Easy direct-movement and obstacle-avoidance tests.
- Easy Brain-side tuning.
- Manual/disabled/cancel behavior and clear status.
- Example landmark-relative and Field::Origin commands.
- Optional external IMU and three-wheel configurations.
- An actual mecanum example/configuration and controller implementation.

Use 0.024 m as a clearly provisional tracking-wheel radius. Unknown offsets, drivetrain dimensions, motor ports and gains may have clearly labeled editable placeholders.

Do not accidentally retain a motor on Smart Port 1 while the VEX IMU uses it. Existing testing defaults currently include that conflict. Validate active-device port conflicts.

Do not change localization geometry merely because USB versus RS-485 was selected.

Preserve the browser/3D viewer for localization-only operation with no camera or AprilTags. Expose useful profile/map/calibration/connection readiness. Add a way to inspect planned paths and obstacle footprints if practical within the existing inspection architecture; avoid creating a second visualization stack.

12. Parallel/series agent workflow

Use one integration lead and up to three workers.

Phase A — inventory and contracts, SERIAL ownership:
- Inspect current code, instructions, tests, and working-tree changes.
- Establish an ownership list.
- Agree on coordinates, units, logical ports, shared state/map/profile contracts.
- Define protocol and planner/follower/drive boundaries before parallel implementation.
- Workers may inspect independently, but must not invent incompatible interfaces.

Phase B — PARALLEL implementation after contracts stabilize:

Worker A:
- common protocol/codecs and related tests.
- communiGATR.
- Shared USB/RS-485 transports.
- Profile/map transfer state machines and client recovery.

Worker B:
- Pi profile application.
- Resource/sensor/localization integration.
- Two-/three-wheel and IMU configuration.
- Field collision definitions and publication.
- Associated Pi tests.

Worker C:
- investiGATR planner-only implementation.
- Motion-model-aware direct and obstacle planning.
- Geometry/collision checking.
- Planner tests.

Integration lead:
- actuGATR following/controllers and tank/mecanum adapters.
- Shared-contract coordination.
- Test applications and cross-component integration.
- Top-level build files and final documentation.

Assign every shared registration/build/test file to ONE owner. Hand protocol-header ownership to Worker A after initial agreement; subsequent contract changes must go through the lead and be communicated to affected workers.

Do not run concurrent builds against the same build directory. Coordinate test/build ownership or use separate build directories. Do not let agents edit each other’s files without an explicit handoff.

Phase C — SERIAL integration:
1. Codec/transport behavior and bounded transfers.
2. Brain profile → Pi localization → Brain pose/map.
3. Reconnect, profile replay, reset, calibration and interrupted synchronization.
4. Planner → follower → drivetrain → testing application.
5. Localization-only bench and 3D viewer regression.

Use independent reviewers for finished areas where useful. Resolve findings rather than merely listing them.

Phase D — final verification and documentation:
- Lead coordinates complete relevant host tests and PROS builds.
- No flashing or deployment.
- Report actual results and remaining hardware-only checks.

13. Verification and completion criteria

Include meaningful tests for:

- Tank and mecanum kinematics, signs, units, coupled limits and stop behavior.
- Holonomic/non-holonomic route feasibility.
- Direct versus avoiding modes with default Origin and landmark references.
- Relative translation/heading composition.
- Rotated obstacle boxes, robot clearance, boundaries, blocked starts/goals, no route.
- Executed simulated trajectory clearance, including turns and corners.
- Field corrections causing route revalidation/replanning.
- Different landmark counts, chunked transfer, malformed counts, interrupted snapshots.
- Map/profile mismatch and idempotent retries.
- Brain/Pi restart, cable reconnect, stale data, frame changes and explicit reinitialization.
- Two perpendicular wheels: each axis independently, signs and turn-offset compensation.
- Three-wheel plus independent IMU fusion through the real configuration/protocol path.
- Existing VEX bench and external-IMU source semantics.
- Real shared USB and RS-485 codec paths with fake transports.
- Both PROS applications compiling against the supported kernel.
- Viewer/localization functioning without camera/world-estimation updates.

Update usage documentation and examples to the final design. Remove genuinely obsolete code/configuration/documentation rather than leaving contradictory old APIs. Preserve still-supported configurations and capabilities.

Provisional/calibration annotations are reader information, not a new runtime approval system.

The final handoff must state:
- What is implemented.
- Which configurations are runnable.
- Exact build/upload/Pi service commands.
- Where to edit robot geometry, gains, ports, initial pose and transport.
- How to run direct and obstacle-avoiding tests.
- How to switch VEX/external IMU and two-/three-wheel profiles.
- How recovery and recalibration work.
- Tests actually run and their results.
- What remains unverified on real hardware.

The practical end result should be that I can configure the robot mainly from the Brain, run localization-only tests, run direct movement and obstacle-avoiding movement, and tune those behaviors without repeatedly editing Pi robot XML.
