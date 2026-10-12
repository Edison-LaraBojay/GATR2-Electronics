# User request 2026-10-09 (verbatim)

Implement the replacement spectaGATR described below. This is an implementation task, not just a proposal.

First inspect the current repository and any local instructions. Other agents may work here, so check the working tree, establish file ownership, and preserve unrelated changes. Do not commit or flash hardware unless explicitly asked.

IMPORTANT CHANGE IN DIRECTION

We are replacing the old spectaGATR with a standalone robot-pose visualizer.

This is the main replacement application, not a reduced mode of the old Pi debugger. It should be inexpensive to run, but it is not a temporary, disposable demo.

However, KEEP the previous Pi-based system under deprecated for now. Tomorrow's demonstration needs a fallback if the microcontroller/RS-485 route is not ready. The previous system communicated between the Brain and Pi over USB, which was demonstrated working.

Do not delete that fallback, scatter its dependencies, or assume that moving its files into deprecated left it buildable.

Codex briefly started deleting the old viewer, then reverted that work. The old files should be restored. Verify the actual tree rather than relying on this description.

The replacement's name is spectaGATR, exactly that spelling.

1. THE PRODUCT WE WANT

The laptop owns the field definition and renders the scene.

The Brain supplies robot position and orientation through an easy C++ API. A laptop receiver transports those updates into the browser.

The intended wireless connection is:

Brain -> VEX controller over radio -> USB -> laptop receiver -> browser

Also support direct Brain USB -> laptop for development and fallback where feasible through the same receiver architecture.

The browser does not need Web Serial. We already selected a local receiver program because browser serial support is inconsistent across platforms.

The intended initial functionality is:

- Render a configured field.
- Draw a simple robot at its supplied position and orientation.
- Display heading or full attitude.
- Show a bounded movement trail.
- Show the latest numerical pose.
- Indicate disconnected/stale data clearly.
- Provide useful camera controls, including an overhead view.
- Work without naviGATR, investiGATR, custom sensor electronics, cameras, or a Pi.

The application does not calculate localization from an IMU. The caller supplies an already calculated pose.

Do not bring across the old electronics debugger: no raw UART inspection, Pico sensor panels, AprilTag/camera interfaces, Pi calibration controls, runtime inspection schema, capture-bundle machinery, or general diagnostic framework.

Do not add path planning, motor control, obstacle avoidance, or automatic motion to this task. Existing planning/control libraries remain independent. Future visualization features may be added later.

Data displays derived purely from timestamped pose, such as estimated speed or distance traveled, are optional after the main application works. Do not let those delay the demo.

2. KEEP A REAL PI/USB FALLBACK

The long-term direction is microcontroller localization with Brain-owned planning and field knowledge. That replacement localization is not implemented yet, and the MCU-to-Brain connection depends on RS-485 hardware that may not be ready.

Therefore preserve a usable, documented fallback using the former arrangement:

Pico acquisition -> Pi naviGATR -> Brain over USB

The original Pi-hosted viewer can remain part of that fallback. It does not need to be converted into the new viewer architecture to serve tomorrow's demonstration.

Inspect these locations:

- deprecated/pi/naviGATR
- deprecated/pi/naviGATR/spectaGATR
- deprecated/pico/aggreGATR
- deprecated/brain/operaGATR
- deprecated/brain/locaGATR
- deprecated/brain/robot
- deprecated/brain/actuGATR

Also inspect the retained active libraries:

- brain/communiGATR
- brain/investiGATR
- brain/actuGATR
- pico/aggreGATR
- translaGATR

Do not assume an archived directory is self-contained. Check:

- CMake repository-root calculations after the move.
- Relative include paths and shared protocol dependencies.
- Configuration-file references.
- Viewer asset embedding and vendored dependencies.
- Brain application Makefiles and PROS dependencies.
- Service-installation paths and default configuration.
- Compatibility with the active libraries, which may have changed.
- Which Pico firmware actually emits the old Pi telemetry.

In particular, active aggreGATR was reduced to local acquisition/debugging during retirement. Do not claim it still supplies the Pi simply because the sensor drivers remain.

Make the fallback recoverable through an explicit documented procedure, preferably buildable in its deprecated location. If a small compatibility repair or isolated legacy source is needed, keep it clearly inside the fallback. Do not revert active library interfaces just to support an old test application.

Keep the old and new applications clearly separated. The new spectaGATR must not import code from deprecated at build time or depend on running naviGATR.

The fallback is temporary. Avoid spending this task redesigning it.

3. BRAIN PUBLISHER API

Provide a small, usable publisher library with overloads along these lines:

viz.updatePose(x, y, heading);
viz.updatePose(x, y, Attitude{roll, pitch, yaw});

Requirements:

- Attitude is an explicit named type with named components.
- Heading-only publication sets roll and pitch to zero, including after a previous full-attitude update.
- Both overloads normalize into one well-defined message shape.
- Specify the angle order, units, coordinate convention, and rotation composition.
- Do not infer units from the magnitude of a value.
- Prefer the repo's established SI conventions internally; provide clearly named conversions/helpers if needed for inches or degrees in examples.
- Use descriptive aliases liberally where they improve readability.
- Do not expose deeply nested template types throughout user-facing code.
- Use an appropriate fixed-size type where needed; std::vector<double, 3> is not a valid fixed-size vector.
- The publisher must not require an investiGATR StateSource or a communiGATR client.
- A normal PROS application should be able to supply poses from its own odometry.
- Initialization and shutdown should be straightforward.

Attitude changes orientation. It does not provide robot height. Do not invent vertical position from pitch or roll.

Prefer a clean separation between the portable publisher/message code and its PROS transport adapter.

4. REPOSITORY STRUCTURE

Use clear ownership. A reasonable structure is:

spectaGATR/
    laptop receiver/server
    browser application
    field configurations
    tests and documentation

brain/spectaGATR/
    publisher headers and implementation
    PROS adapter
    integration documentation

brain/spectaGATR-demo/
    separate PROS example application

You may adjust names/subfolders if existing conventions suggest something better, but retain the separation:

- Laptop application.
- Importable Brain library.
- Actual example PROS project.
- Deprecated fallback.

The publisher library is not itself a competition application. It does not own initialize(), autonomous(), or opcontrol().

The example application does own the ordinary PROS lifecycle and shows how to use the library.

Use CMake for portable C++ host components, consistent with the repository. Browser rendering can remain JavaScript. Prefer reusing sound existing rendering code and appropriately licensed assets over recreating them, but remove their Pi-specific assumptions from the active implementation.

Preserve third-party license notices.

5. FIELD AND ROBOT GEOMETRY

The laptop should load the field locally. The Brain should not have to resend field geometry for this initial scope.

Inspect the existing Override field definition and rendering code. Reuse the existing nominal field geometry and colors where sensible rather than replacing the field with an unrelated empty square.

Make the field selectable/configurable on the laptop. It must work without network access at runtime once installed.

Keep visual robot dimensions configurable on the laptop as well. Do not require adding a robot-profile protocol merely to draw a box.

Define and display the coordinate convention clearly:

- Origin.
- Positive X.
- Positive Y.
- Zero heading.
- Positive heading direction.
- Robot forward/left/up axes.
- Roll, pitch, yaw convention.

Follow the established repo convention unless there is a concrete reason to change it. Verify the implementation matches the documentation; do not silently mirror or swap axes.

Do not interpret zero heading as automatically meaning "up the screen."

An example application should make unit conversions obvious.

6. TRANSPORT: VERIFY THE REAL VEX/PROS PATH

Do not assume opening an arbitrary serial port exposes plain pose messages.

Inspect the existing PROS USB transport and the actual PROS/controller console framing. The VEX system/upload interface and user-console interface are different.

Useful existing references include:

- brain/communiGATR/pros/pros_usb_port.cpp
- The archived Pi pros_usb_link implementation.
- Existing USB framing helpers and transport tests.

The existing communiGATR USB setup may enable a custom stream and change PROS serial/COBS settings. Do not copy those calls into the new publisher without understanding their process-wide effects.

The new publisher must not accidentally break an existing Brain-to-Pi USB connection, normal console output, or uploading.

Check whether controller-console telemetry and the old Brain-to-Pi USB link can coexist in practice. If they cannot with the existing firmware setup:

- State the exact limitation.
- Provide explicit operating modes.
- Keep the old Pi-hosted viewer usable for the fallback.
- Do not claim coexistence without evidence.
- Do not redesign communiGATR just to avoid documenting a temporary limitation.

Use current primary documentation or the relevant PROS implementation when transport details are uncertain.

Choose a modest, configurable publication rate and validate it. Do not promise a particular wireless throughput based on direct USB behavior.

The robot control loop must not block waiting for a viewer or a slow connection:

- Bounded buffering.
- Prefer the newest pose over accumulating old poses.
- Background I/O where necessary.
- Bounded work per iteration.
- Clean handling of disconnects and retries.
- No unbounded allocation growth.

The laptop receiver should support reconnecting without restarting the whole application.

The browser should reconnect independently to its local receiver.

7. MESSAGE CONTRACT

Keep the protocol small, explicit, documented, and independently testable.

Include enough information to distinguish:

- Protocol version.
- A new publisher run/session.
- Pose sequence.
- Source timestamp.
- Position.
- Orientation.

Choose a format appropriate for the actual transport. A modest, clearly framed text message is acceptable if it reliably survives the PROS/controller console path. A huge generic telemetry schema is unnecessary.

Handle:

- Partial reads.
- Multiple messages in one read.
- Other console text interleaved with pose output.
- Malformed or oversized messages.
- Nonfinite coordinates/angles.
- Repeated or out-of-order updates.
- Publisher restarts.
- Timestamp wrap if using a wrapping Brain clock.
- Truncated data during disconnect.
- Stale poses.

Do not compare unrelated Brain and laptop clock values as though they share an epoch.

Document the actual freshness/latency measurements available. Distinguish local receive age from true end-to-end measurement age.

8. SMOOTHNESS AND RESPONSIVENESS

Smooth visualization matters, but correctness comes first.

Separate pose receipt from browser rendering:

- Render using requestAnimationFrame.
- Keep only a bounded amount of recent data.
- Avoid rebuilding field meshes on each pose.
- Avoid rebuilding the entire interface on every message.
- Prevent a slow client from creating a growing queue.
- Use efficient, bounded trail storage.
- Avoid expensive work in the Brain control task.

If presentation interpolation is useful, keep its delay small and bounded. Handle heading wrap correctly. Full-attitude interpolation should use an appropriate rotation representation rather than interpolating through the wrong angle discontinuity.

Numerical readouts should show actual received values. Any smoothed presentation should not alter the underlying pose.

Do not continue imaginary motion indefinitely after telemetry stops. Mark the pose stale and freeze or otherwise clearly display its last known state.

A restart or explicit pose reset must not draw a misleading trail across unrelated coordinate frames.

Do not implement an elaborate smoothing framework before the basic live display works.

9. TOMORROW'S DEMO

Provide a concrete demonstration path that does not depend on finishing MCU localization.

The standalone viewer needs a deterministic simulated-pose mode so we can validate rendering and transport-independent behavior without hardware.

The Brain example should demonstrate:

- Publisher initialization.
- Heading-only publication.
- Full-attitude publication.
- Supplying a pose from caller-owned code.
- Disconnect/reconnect behavior.

Be explicit about where x/y come from.

An IMU does not produce x/y position. If the example uses synthetic motion, label it synthetic. If it uses an existing working pose provider, identify it and keep that dependency confined to the example.

Do not add unsolicited motor movement. A visualization demonstration should not start driving the robot.

If you can reuse the existing Pi/USB pose source in an optional example without compromising the new library's independence, that is useful. It is not a requirement to route the replacement through the Pi.

The fallback should retain the known two-perpendicular-tracking-wheel plus VEX IMU configuration where available. Inspect archived configuration rather than guessing ports, signs, geometry, scale factors, or defaults.

Historical hardware context that may help, but must be checked against the current files:

- Two perpendicular tracking wheels.
- Sideways wheel was most recently assigned encoder port 0.
- Forward wheel was most recently assigned encoder port 1.
- VEX IMU was moved to Smart Port 20.
- Brain-to-Pi USB was used after RS-485 problems.
- The previous system displayed live pose successfully.

These are historical values, not permission to silently overwrite newer configuration.

10. IMPLEMENTATION WORKFLOW

(Coordinator handles: A reconnaissance and contract; B parallel agents with separate ownership; C serial integration; D focused independent review.)

11. VALIDATION

Run meaningful checks appropriate to what changed.

At minimum:

- Build the portable publisher/receiver.
- Build the PROS example with the available toolchain.
- Test both updatePose overloads, including switching from attitude to heading-only.
- Test framing through fragmentation, garbage, truncation, and reconnect.
- Test invalid/nonfinite values and bounded buffers.
- Test session restart and stale-state behavior.
- Verify X/Y directions, heading sign, heading wrap, and roll/pitch axes.
- Exercise the browser with known deterministic trajectories.
- Verify it runs offline with local assets.
- Confirm new spectaGATR builds and runs without naviGATR.
- Verify the fallback can be built or provide the precise remaining blocker, rather than merely confirming that its source files exist.

Target Windows first because that is the current laptop environment. Structure the receiver for Linux/macOS too. CMake alone does not make serial handling portable; isolate OS-specific code and use appropriate implementations.

Do not claim Linux, macOS, controller-radio performance, or physical hardware success unless actually tested. Distinguish compile checks, simulated tests, and real hardware tests.

If physical hardware is unavailable, provide a short, exact validation procedure for the user, including what successful connection and stale/disconnected states should look like.

12. DOCUMENTATION AND HANDOFF

Primary README (spectaGATR): what it does, dependencies, build commands, importing the Brain publisher, minimal real C++ usage example, building/uploading the example project, selecting controller or Brain connection, starting the receiver and opening the viewer, selecting the field, units and conventions, reconnection behavior, known console/USB coexistence limitations.

Separate short fallback procedure: exact Pi source/build/configuration paths, exact Brain project, exact Pico firmware variant if required, what must be rebuilt or reuploaded, how the Pi service is configured/restarted after path changes, how to open the old Pi viewer, which steps have and have not been verified.

Do not perform remote service changes, hardware flashing, destructive Git operations, or commits without an explicit request.

Keep archived Pi HAT designs and unrelated PCB work untouched.
