Implement investiGATR and its Navigatr integration in this repository.

Read the current code and repository instructions first. Preserve unrelated
changes, do not touch PCB files, and do not commit anything. Changes may extend
beyond the new Brain folders wherever the shared protocol, Pi runtime, build
configuration, tests, or documentation require it.

Keep the implementation small and understandable.

DIRECTORY STRUCTURE

brain/investiGATR/
    Portable C++ waypoint navigation and motion-control library.

brain/navigatr-interface/
    Navigatr communication driver implementing investiGATR's input interface.
    Isolate PROS-specific serial/task code from portable protocol handling.

brain/testing/
    A complete, minimal PROS application using both libraries.

Only the testing application owns initialize(), autonomous(), and opcontrol().
The libraries have public headers and implementation files, with host-testable
builds and a documented PROS integration/packaging route.

INPUT ABSTRACTION

investiGATR accepts a reference to a standard abstract InputSource interface.

NavigatrDriver implements that interface. Another implementation can supply
Brain odometry, simulation, or another localization system without changing
investiGATR.

Define small, descriptive input/output types:
- Robot pose, validity, and measurement freshness.
- Landmark estimates addressed by stable IDs.
- Source/session and coordinate-frame generation information.
- Input request describing the information needed for the active command.
- Input snapshot.
- Drive command and motion status.

InputSource returns the latest available snapshot without blocking the control
loop. The Navigatr implementation handles communication separately and can
request the currently needed landmark. Missing data is explicitly unavailable.

Use meters and radians, heading counterclockwise from +x, and robot axes
+x forward/+y left. Estimates used together must share a coordinate frame.

The core must not depend on PROS, serial hardware, Navigatr-specific packets,
or motor APIs. Prefer simple structs, descriptive aliases, and small functions.

NAVIGATION API

Provide nonblocking operations equivalent to:

goTo(fieldPose)
goToRelative(landmarkId, landmarkLocalOffset)
follow(path)
cancel()
status()
update(now)

Commands establish the objective; update() samples the InputSource and produces
drive demand/status. Define command replacement behavior explicitly.

A path is an ordered collection of absolute or landmark-relative waypoints.
goTo is the single-waypoint case. Keep routes caller-defined.

Resolve relative destinations as:

T_field_destination = T_field_landmark * T_landmark_destination

Rotate the offset translation and compose heading correctly. The destination
is the desired ROBOT pose. Landmark headings are full field orientations.
Do not apply the camera mounting offset again.

An active relative destination follows modest changes in its landmark estimate.
Handle those changes without restarting the whole command or its timeout.
Bound unreasonable jumps and output changes. Use position/heading tolerances
and a continuous settling interval. Completed commands remain completed until
a new command starts.

BASIC CONTROL

Assume differential/tank drive for the first implementation unless existing
robot configuration clearly requires otherwise; document the assumption.

Implement straightforward PID-based drive-to-pose behavior:
- Turn toward the destination.
- Drive toward it while correcting heading.
- Reduce forward demand when substantially misaligned.
- Finish at the requested heading.
- Reconsider position when an active relative destination moves.

Use actual elapsed time, shortest-angle errors, bounded integral/output,
configurable slew limits, and appropriate PID resets on command/phase changes.
Start example integral gains at zero.

Return documented, platform-independent forward/turn demands. The PROS adapter
mixes and limits them into left/right motor commands.

Provide bounded waiting for initial pose/landmark availability, motion timeout,
cancellation, and explicit failure statuses. Invalid or stale input produces
zero motor demand. Loss of the connection or a coordinate-frame discontinuity
must not silently continue an old movement.

This version follows supplied waypoints. Automatic obstacle routing is outside
its scope.

BRAIN REBOOT BEHAVIOR - ESSENTIAL REQUIREMENT

The Brain may be powered off and restarted repeatedly while the Pi keeps running.

After restarting, the Brain runs initialize(), establishes communication, sends
its initialization commands, and starts new autonomous/testing commands.
This must work without restarting the Pi, clearing files, or manually resetting
sequence numbers.

It must start a NEW application session, not resume the previous movement.

Implement a proper Brain-request/Pi-response protocol:
- The Brain owns polling and initiates transactions.
- The Pi responds to requests; it does not rely on unsolicited pose streaming.
- Pi localization, world estimation, and inspection continue independently.
- A fresh Brain initialization establishes a distinguishable new session.
- Pi restarts must also be identifiable.
- Correlate replies with session and request IDs.
- Ignore delayed packets belonging to previous sessions.
- Scope duplicate-command handling to the session.
- Retries must not apply initialization or other state-changing commands twice.
- Return explicit command results and use bounded retries/timeouts.

Existing pose-valid, localization-initialized, and object-valid status bits
are NOT acknowledgements and do not establish session identity.

On a new Brain session, clear prior client requests/selections and any
session-owned target latch. Do not inherit the previous active movement.

Preserve the Pi's continuing localization unless the new Brain explicitly
requests pose initialization. A new session alone must not relocate the robot.

Crucially, carry session identity through application-level deduplication and
placement handling too. A genuine initialization command after reboot must
apply even if its numeric sequence or pose matches a command from the old boot.
An acknowledgement of applied placement must mean localization actually applied
it, not merely that the command packet decoded.

The Brain driver starts with empty/invalid cached input. It becomes ready only
after the new session has valid correlated responses. Link recovery must never
automatically resume a cancelled movement command.

PROTOCOL AND PHYSICAL LINK

Inspect common/frame_codec.*, common/frames.h, Pi command collection, publishing,
target handling, and serial resources. Make coordinated changes wherever needed.

Resolve these existing protocol limitations:
- obj_* currently can represent either a landmark pose or an already resolved
  robot destination. Define unambiguous payload semantics. goToRelative must
  consume an actual landmark pose and apply its offset exactly once.
- A newly received packet can contain an old measurement. Preserve robot
  measurement age and landmark observation age separately from link freshness.
- Brain/Pi/Pico clocks have different origins. Normalize freshness explicitly;
  do not subtract unrelated clock values.
- Carry enough session, epoch, and anchor information to reject incompatible
  snapshots after restarts or re-anchoring.
- Clearly distinguish nominal landmark data from observed estimates.

Keep frames bounded and explicitly version the revised contract. Update shared
codecs, Pi producers/consumers, configurations, tests, and documentation together.

The current HAT serial driver holds DriverEnable high. Implement real
half-duplex turnaround for this request/response design:
- Pi listens while idle.
- Enable transmission only for its response.
- Complete transmission before releasing the bus.
- Handle partial writes, errors, and timeouts.
- Serialize Brain requests and define bus ownership/turnaround so retries and
  delayed replies do not create simultaneous transmitters.

Answer data requests from available estimates; do not block the bus waiting
for camera processing.

Do not present encoded command packets alone as working bidirectional support.

OTHER SOURCES AND FALLBACK

Demonstrate the same investiGATR controller using an independent InputSource.

Document how existing Brain odometry can implement the interface. Independence
does not create backup localization automatically: a fallback must supply a real
valid pose in a known frame.

Source switching is explicit, cancels incompatible active motion, and addresses
coordinate alignment. Never substitute invented poses when Navigatr disappears.

PROS TESTING APPLICATION

Keep brain/testing straightforward:
- One robot-specific configuration for motors, directions, limits, gains,
  serial port, and starting pose.
- initialize() performs a bounded connection/setup sequence and sends explicit
  initialization commands, checking their results.
- Autonomous demonstrates an absolute destination, a waypoint path, and
  landmark-relative alignment.
- Acquisition/control updates do not block competition callbacks indefinitely.
- One owner writes drivetrain motors; disable/cancel stops outputs.
- Hardware-dependent values are clearly marked and explicitly configured.

Use the actual libraries without duplicating their implementations.

VERIFICATION

Add meaningful host tests and a small drivetrain simulation covering:
- Fixed-pose convergence, heading wrap, final orientation, and rotated offsets.
- Modest moving-landmark updates, waypoint progression, and settling.
- Replacement, cancellation, timeout, missing/stale input, and frame changes.
- Equivalent behavior from Navigatr and independent input sources.
- Fragmented/corrupt packets, duplicates, dropped replies, and stale measurements.
- Half-duplex direction changes, transmit completion, and error recovery.

Include an end-to-end simulated reboot scenario:
1. Brain session A initializes and requests a landmark.
2. Brain disconnects while the Pi continues running.
3. Brain session B starts with reset request counters and sends fresh commands.
4. Its initialization and selection apply correctly.
5. Delayed session-A commands/replies cannot affect session B.
6. No old movement resumes and no manual Pi reset is required.

Also test Pi restart while the Brain stays on, and initialization-command retries.

Run relevant existing suites when changing the common protocol or Pi code.
Build the PROS example if the toolchain is available; identify anything that
could not be validated. Do not claim hardware validation without hardware.

Document the public API, input contract, source-adapter implementation,
build/import steps, tuning, testing setup, restart behavior, and limitations.
Summarize what changed and what was verified.
