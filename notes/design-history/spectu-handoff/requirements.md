# spectuGATR request (2026-09-28, verbatim; replaces the earlier correction)

Context from the user's paste: "Claude's concerns are valid, especially the extra round trip and competition with localization traffic. ... The distinction to preserve is Brain-owned visualization data versus Pi-owned diagnostics, while allowing the transport layer to forward/cache data efficiently. That doesn't require putting path handling inside localization."

```text
This is the complete replacement specification. Read it instead of treating the earlier correction as a standalone task.

First produce a concrete architecture and transport/bandwidth proposal for review. Do not begin implementation until that proposal is approved. The implementation requirements and subsequent audit below define the intended scope.

Your concerns about bandwidth, latency, transport coexistence, and existing functionality are valid. Investigate them explicitly rather than satisfying the requested separation through an inefficient round trip.

Do not modify PCB files, flash, deploy, commit, or push. Preserve other agents' work.

PRODUCT NAME AND PURPOSE

The canonical name is spectuGATR.

It names the overall visualization/diagnostics product:
- Shared browser viewer.
- Portable receiver/server.
- Shared telemetry contracts.
- Brain-side publishing library.

The existing spectaGATR viewer should become part of this product. It is not a separate competing viewer. Preserve and migrate its functionality; do not rewrite working features unnecessarily.

The intended organization is approximately:

spectuGATR/
    protocol/
    host/
        sources/
    viewer/
    tests/
    docs/

brain/spectuGATR/
    include/
    src/
    pros/
    integrations/
    tests/

naviGATR retains its source-specific publishing adapter.

Adjust details based on the current repository, but preserve these dependency boundaries and one shared viewer implementation.

AUTHORITATIVE DATA OWNERSHIP

The Brain owns the core visualization feed in both standalone and integrated use:
- Robot pose.
- Field geometry and landmark state known to the Brain.
- Paths and targets.
- Execution/control information.
- Attitude when available.
- Other Brain-owned telemetry.

The Brain may obtain its localization/field data from naviGATR, another library, or the user's own code. That must not require different spectuGATR APIs or separate "with Pi" and "without Pi" Brain libraries.

naviGATR contributes Pi-specific instrumentation:
- Acquisition/UART data.
- Tracking-wheel and other raw sensor readings.
- Available digital pin states.
- Sensor initialization, calibration, freshness, and failures.
- Estimator internals and timing.
- Camera frames and detection diagnostics.
- Other information beyond the Brain's normal functional view.

The functional naviGATR -> Brain localization/field connection continues. We are separating visualization ownership, not removing information the robot needs to operate.

Receiver modes:
1. Brain only: field, robot movement, paths, and available Brain telemetry.
2. Brain + naviGATR: the same core scene plus Pi instrumentation.
3. naviGATR only: Pi diagnostics/camera, without substituting a Pi-owned live core robot feed.

If the Brain disconnects, do not silently switch the displayed robot to another authoritative pose source. A last-known state may be retained if clearly marked stale.

The core scene should represent what the Brain actually knows and uses. Do not replace the Brain's accepted field revision with a newer Pi revision it has not yet received.

PUBLISHER API

Initialize one publisher within the Brain process and give relevant components references to that instance.

Each component calls descriptive methods for data it owns, for example:

viz.updatePose(...)
viz.updateField(...)
viz.updatePath(...)
viz.sample(...)

Do not introduce a central coordinator that polls and understands every subsystem.

The Pi has its own publisher instance. These are separate processes; the receiver combines their streams.

Standalone users call the same methods directly. They must not need naviGATR, communiGATR, investiGATR, or actuGATR just to publish their own field and pose.

Provide optional integration for our libraries:
- The robot-state/field provider publishes its information.
- The component accepting/replacing a plan publishes the active path.
- Control components publish execution/actuation information.

Trace actual ownership. investiGATR alone does not necessarily own current pose, field acquisition, or execution state.

Preserve meaningful path semantics, including turn-only segments, replacements, clears, and cancellation.

Visualization methods must never secretly issue SET_POSE or motor commands.

Use descriptive aliases liberally where they improve readability, especially nested containers, callback types, and long public signatures. Prefer named structs for semantic fields. Do not perform an unrelated repository-wide alias refactor.

Pose should support:
    updatePose(x, y, heading)
    updatePose(x, y, Attitude{roll, pitch, yaw})

Use explicit documented units and existing coordinate conventions.
Attitude supplies heading through yaw.
Heading-only input means tilt was not supplied, not measured zero tilt.

STANDARD MESSAGES AND DYNAMIC MEASUREMENTS

Keep a small, versioned contract for:
- Source/session identity and capabilities.
- Field geometry and revisions.
- Pose, attitude, frames, validity, and timing.
- Paths/targets and lifecycle.
- Events.
- Named measurement definitions and samples.

Allow arbitrary named numeric channels with type/unit metadata so adding a measurement does not require editing the viewer.

Do not force arbitrary values into pose fields.
Do not build an unrestricted schema framework.

Unavailable optional capabilities should not appear as broken required components.

TRANSPORT: ANSWER THESE QUESTIONS IN THE DESIGN PROPOSAL

1. Standalone wireless Brain access

The desired route is:

Brain -> VEX radio -> controller connected by USB to laptop
-> spectuGATR receiver -> browser

The receiver is a separate program bundled with spectuGATR. Browser Web Serial/WebUSB is not required.

Direct Brain USB may be an additional useful transport, but wireless robot operation through the controller is the intended standalone workflow.

Verify current PROS/controller protocol behavior and device discovery using primary documentation/source. Do not assume the existing direct-Brain USB adapter supports controllers unchanged.

2. Combined Brain + naviGATR access

Do not assume either:
- All Brain visualization must travel back through naviGATR; or
- Controller telemetry and the existing Brain-Pi USB connection can operate independently without interference.

Investigate the actual stream ownership.

The current communiGATR USB path reportedly reads stdin and changes COBS behavior globally. Determine whether controller terminal access shares or consumes the same underlying buffers.

Do not add competing serial readers, multiple independent stdin consumers, or uncoordinated global serial configuration.

A single transport owner with explicit multiplexing/fan-out may be appropriate.

Forwarding through a Pi-hosted transport service is acceptable if necessary, but:
- It must be explicit transport behavior.
- Core localization/world-estimation must not interpret visualization paths.
- The Brain remains the producer of its visualization data.
- The standalone Brain publisher remains usable without naviGATR.

Present the simplest supported topology for:
- Standalone Brain/controller.
- Our current Brain-Pi USB setup plus visualization.
- Brain-Pi RS-485 plus visualization.

Distinguish demonstrated behavior from untested assumptions.

3. Pose rate and link budget

There is NO requirement to publish Brain pose at a fixed 30 Hz.

Publish actual new states at a sustainable configured rate. Avoid duplicate samples and unnecessary additional request/reply transactions.

Investigate:
- Piggybacking or batching where appropriate.
- Separate visualization transport where genuinely supported.
- Compact messages.
- Prioritized scheduling.
- Bounded coalescing of obsolete live state.
- Visualization rate reduction under pressure.

Localization polling, required sensor traffic, and control communication take priority.

Do not trade away their deadlines merely to meet an arbitrary visualizer frequency. Publishing callbacks must not perform blocking I/O.

Use the existing measurements of TELEMETRY's impact as evidence to investigate. Reproduce relevant tests against the proposed design.

4. Latency

The reported approximately 25 ms measurement-to-browser loopback result is a baseline for that specific tested configuration, not a wireless guarantee.

Our priority remains low latency and smooth display.

Brain-authoritative pose necessarily arrives after the Brain has received it. Quantify that inherent delay separately from avoidable queueing, serialization, radio, and browser delay.

Do not hide additional latency with smoothing or fresh receipt timestamps.
Preserve original measurement timestamps, validity, and source identity.

There is not yet an approved fixed extra-latency allowance. Present measured/estimated tradeoffs and a recommended budget before implementation.

If preserving Brain authority materially conflicts with the current latency target, make that conflict explicit. Do not silently restore Pi-authoritative pose or claim both goals were met.

5. Field geometry

The Brain logically publishes the field it actually uses.

Standalone mode must be able to provide complete field definitions without a Pi.

Do not resend the entire field every pose update. Use initial definitions and meaningful revisions/updates.

For integrated mode, evaluate an optional cache/reference optimization:
- The Brain selects/publishes a specific field identity and revision.
- A receiver may resolve it from matching cached content.
- It must validate the match and obtain missing content reliably.
- It must not substitute whatever field the Pi currently happens to have.
- Standalone behavior and the public Brain API remain unchanged.

Recommend whether full initial transfer or verified references are appropriate. Explain the complexity/bandwidth tradeoff.

MULTIPLE SOURCES

The receiver must support Brain and naviGATR simultaneously, not just mutually exclusive source selection.

Use explicit identities for:
- Robot.
- Source/session.
- Field/map and revision.
- Coordinate frame/placement generation.
- Sequence and clock domain.

Handle one source arriving first, missing sources, restarts, stale paths, field changes, and reconnection.

Do not merge data solely by arrival time.
Do not compare timestamps from unrelated clocks without a defined mapping.
Do not combine paths and poses from incompatible placements.

An example launch experience might be:

spectugatr --source brain
spectugatr --config combined.json
spectugatr --replay capture.zip

Choose and document the final commands. One command should start the receiver/server and optionally open the browser.

CROSS-PLATFORM REQUIREMENTS

Use CMake for the desktop C++ receiver and portable host tests.

Support Windows, macOS, and Linux through shared code and a small platform-specific device/transport layer where necessary.

Account for:
- Serial discovery and stable identity.
- Permissions.
- Disconnect/reconnect.
- Filesystem paths.
- Browser launch.
- Packaging.

Users should be able to use prebuilt packages without a compiler.

Keep the Brain publisher compatible with the existing PROS build workflow.

Maintain one shared browser implementation and offline assets. Pi-hosted and laptop-hosted viewing must not become separate forks.

State which platforms were actually built/tested. Do not claim physical controller validation based on mocks.

PRESERVE THE VISUALIZER WORK

Inventory the current implementation before changing it.

Retain completed improvements and finish agreed gaps:
- Low-latency delivery and bounded backpressure.
- Browser smoothness and performance instrumentation.
- Field/robot/path/trail visualization.
- Sensor and transport diagnostics.
- Dynamic graphs.
- Attitude display with honest validity/staleness.
- Optional camera preview without calibrated field correction.
- Capture, export, replay, and lifecycle events.

Keep generic behavior in spectuGATR and source-specific interpretation in adapters.

Camera changes should remain minimal. A missing camera must not invalidate unrelated localization. Preview must not require field correction or invented calibration.

PERFORMANCE REQUIREMENTS

Measure:
- Source/sample age.
- Generation and serialization time.
- Message size and rate.
- Queue depth/age, drops, and replacements.
- Round-trip time.
- Browser processing/frame time.

Use the existing low-latency architecture where it works.

Replace obsolete wholly unsent live state without corrupting partial frames or required message ordering.
Keep durable events/recordings separate from replaceable live state.

Avoid repeated full histories/configuration and unnecessary DOM rebuilding.

Any smoothing must be presentation-only, bounded, and reset on discontinuity/staleness. Exact values remain available in readouts, graphs, and exports.

RECORDING AND REPLAY

Preserve bounded rolling history and short detailed capture:
- Configurable pre-trigger/post-trigger periods.
- Actual available updates, not artificial duplicate samples.
- Optional fault triggers with limits.
- Explicit overflow/loss records.

Standalone receiver capture only contains data it actually received. Do not claim it preserves every native-rate Brain sample across a constrained radio link.

Retain source-side Pi capture where useful through an adapter.

Export portable bundles containing:
- Separate CSV streams.
- Configuration and schema metadata.
- Units and frame conventions.
- Source/session/clock identities.
- Events and loss information.

Replay should synchronize graphs and the 3D scene, including field revisions and discontinuities.

Replay must not control the live robot.

INTEGRATION AND VALIDATION AFTER DESIGN APPROVAL

Use coordinated parallel work where useful:
- Shared contract and receiver/transports.
- Brain publisher and optional library integration.
- Viewer extraction/source-independent UI.
- naviGATR adapter and capture/replay preservation.

Agree on shared contracts and file ownership first. Integrate and validate serially.

Update the current actual test applications, currently expected to be:
- brain/locaGATR: motor-free localization.
- brain/operaGATR: planning/driving.

Verify names rather than restoring obsolete folders.

Provide minimal standalone examples with no other GATR library dependency.

Test:
- Standalone basic field/pose.
- Optional paths and channels.
- Combined-source authority/frame handling.
- Missing capabilities.
- Source restart/reconnection.
- Transport coexistence.
- Backpressure and partial frames.
- Capture/export/replay consistency.
- Browser responsiveness.
- Preservation of current localization/control functionality.

FINAL AUDIT - AFTER IMPLEMENTATION

Once implementation and validation are complete, perform a substantial independent read-only audit.

The audit is report-only. Do not implement its recommendations until reviewed.

Review the entire pipeline:
- Acquisition/resources/sensors.
- Localization timing, calibration, lineage, and fusion.
- Profiles, placement, sessions, and recovery.
- Capture-time world estimation and transforms.
- Communication and transport ownership.
- Planning/collision checking/following/control.
- spectuGATR source combination, buffering, capture, replay, and isolation from critical work.

Recheck previously reported fixes against actual code.

For each substantive finding provide:
- Severity and practical impact.
- Exact code references.
- Concrete trigger/interleaving.
- Why current checks/tests miss it.
- Recommended minimal correction and tradeoffs.
- Proposed regression test.
- Confidence and remaining hardware uncertainty.

Separate actual defects, intentional limitations, and optional improvements.

IMMEDIATE DELIVERABLE

For now, return the design proposal only:
- Current implementation inventory.
- Dependency/source ownership diagram.
- Supported connection topologies and unresolved hardware facts.
- Answers to the five transport/rate/latency/field/name questions.
- Recommended API and field-transfer strategy.
- Measured evidence or clearly labeled estimates.
- Implementation breakdown and validation plan.
- Any decisions that genuinely require user input.

No implementation yet. Once approved, execute the implementation,
validation, and final read-only audit in that order.
```

## APPROVAL (user, 2026-09-28 ~15:45)

"approve with your recommended defaults"

The design proposal (spectu-handoff/PROPOSAL.md) is approved with every recommendation and default in its section 8:
- D1 a; D2 a (b later); D3 no ack; D4 b; D5 b; D6 b; D7; D9 fix in actuGATR; D10 a; D11 a (c later); D12 radio placeholders; D13 unsigned; D14; D15; D17 a; D18 MIT; D19 c2 if the cable is acceptable, else c.
- D8 and D16 are deferred.

Order from the spec: implementation, then validation, then the final whole-system read-only audit (report only).
