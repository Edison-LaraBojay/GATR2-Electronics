Restructure context, captured 2026-09-06 from repository commit `74d2432`.

This is a descriptive working brief for the upcoming redesign. It records what
the repository expresses and what the user has actually said. It does not make
the existing architecture, documentation, tests, or inferred preferences binding.

The user's stated position is the starting point:

- A substantial restructure is under consideration, including discarding concepts.
- The existing code was AI-generated and has not been reviewed by the user.
- Preserve understanding and useful principles; retain code when it earns its place.
- Avoid making the user repeatedly explain the underlying problem and intent.
- The initial investigation did not establish which specific principles to
  retain. The follow-up below records the user's subsequent direction; the
  repository-derived candidates later in this brief remain unapproved.

In the next discussion on 2026-09-06, the user clarified the intended scope and
the boundary they are considering:

- The system should survive VEX seasons. General usefulness outside VEX is not
  a goal, and the infrastructure should not depend on particular sensors.
- Precise alignment with a particular side of a goal is a primary use case,
  for both autonomous behavior and assistance during driver control.
- Understanding the problem is necessary for the user to work effectively;
  settling infrastructure should proceed through understandable responsibilities.
- They are considering localization plus optional, requested landmark estimation:
  the Brain asks about landmark X, and the Pi reports information about X
  relative to the robot, primarily position and heading.
- Under this proposed division, the Brain would own destination construction,
  desired standoff, contact offsets, and how to use an observation for control.
  The Pi would report estimated environmental geometry rather than calculate a
  behavior-specific target pose.
- The user questions whether a persistent global field model is necessary.
  This is an open design decision, not a request to remove it from code yet.
- A fixed pipeline is acceptable in principle; its exact stages are unsettled.
- Heading meaning and identification of a particular goal/face need to become
  explicit. Small physical goal rotations are an expected operating condition,
  but the identity/face ambiguity and the angle convention are separate issues.
- The user subsequently clarified that alignment evidence measures the
  robot-landmark relationship. A discrepancy from the relationship predicted
  using robot heading and nominal field geometry can include both actual
  landmark displacement/rotation and robot odometry drift. Calling that
  discrepancy a measured field correction would overstate what is known.
  The intended benefit is constraining relative alignment at the interaction.
- The user next asked to define what is reported to the consumer, explicitly
  setting binary encoding aside. The concrete assistant proposal is in
  [reporting-proposal.md](reporting-proposal.md); it is not yet an approved API.
- They then simplified the proposed interface to ongoing current robot
  `(x, y, heading)` and landmark-relative `(delta_x, delta_y, delta_heading)`
  emitted only on observation. A landmark observation is an event, not a held
  landmark status with an `observed` flag. The Brain may use events immediately,
  retain a destination, or propagate landmark information using localization.
  This supersedes the assistant's earlier landmark status table and pending
  question about Pi-side retention. Exact API and timing delivery remain open.
- The user then returned to the underlying product problem rather than treating
  the two proposed outputs as a settled architecture. The immediate value is
  combining three tracking wheels and one IMU, fused on the Pi, into one clean
  sensing interface through ONE VEX Brain port. Camera processing is another
  capability on that same interface, with further sensors possible later.
  Port consolidation and removing sensor integration work from Brain code are
  central motivations. Localization has value independently of vision or targets.
- The user then explicitly reopened where landmark propagation belongs. AprilTag
  and landmark sensing are primarily for final scoring alignment, not visual field
  localization or full autonomous navigation. With one forward camera, a rear
  approach may require observing the relevant face, turning away, and carrying
  that information through the blind interval. The user is considering temporary
  tracking of only the requested landmark inside Navigatr, while retaining at most
  a nominal seasonal reference for recognition/association rather than a live field.
  At that point, propagation ownership and acceptable blind-maneuver error were open.
- The user then expressed support for the Pi-side temporary tracking approach,
  and asked whether simultaneous selection of multiple landmarks should be possible.
  They could not identify a current need and questioned building that capability
  now. The assistant recommends one active selection initially, with an explicitly
  identified, self-contained estimate per landmark so single selection is not a
  permanent assumption throughout the architecture. Simultaneous selection is
  deferred pending a concrete use case; this scope recommendation is not yet a
  separate user decision. Several cameras/tags observing one landmark are distinct
  from tracking several landmarks. Blind-maneuver accuracy remains unmeasured.
- The user next confirmed the initial scope: report and propagate only the selected
  entity, with no live propagated field. They questioned whether this saves much
  camera computation and how to associate many visible AprilTags against nominal
  field geometry. They are considering independent face definitions versus a goal
  with multiple faces, including combining observations of west/south sides to
  infer one goal position and rotation. That geometry representation is not yet
  a user decision.
- The user then proposed reducing camera work with a predicted image search region,
  a permanent reduction in the useful field of view, and/or solving pose from only
  one of several tags associated with the selected goal. They asked whether these
  save enough computation to justify their complexity. These remain options to
  evaluate, not approved implementation requirements or measured speedups.

For rigid goals, the assistant recommends a physical object with named face frames
and separately identified tag mounts. Select the object and face to report; accept
appropriate evidence from any associated mount on that object; retain one rigid
pose and derive the selected face. An explicit center estimate and separate face
trackers are unnecessary. Full 3D mounting geometry is needed internally before
planar reporting; rigidity and calibration determine whether indirect face estimates
are accurate enough. Face names remain attached to physical geometry as it rotates.

Selecting one track does not imply proportional detector savings. Finding/decoding
image tags and estimating metric pose are separable; IDs/corners may allow some
filtering before pose solving. Repeated IDs can require geometric hypotheses and
nominal-field/odometry context. The static catalog may need to supply competing
associations even when only the selected object receives live updates. Ambiguous
evidence must not be forced onto the requested object. No camera performance
measurement has been made in this investigation.

The assistant recommends distinguishing actual image cropping from narrowing the
optical field of view at unchanged pixel dimensions. Cropping can reduce detector
input while retaining resolution on included tags; a narrower lens alone does not
reduce the pixel count. A fixed crop needs validation across working distances,
tag heights, camera mounting angles, and robot pitch/roll. A predicted crop needs
uncertainty margins and wider acquisition/reacquisition so pose drift or goal
displacement cannot hide useful evidence. After acquisition, the selected estimate
can provide the search prior instead of relying only on nominal field placement.
Crop coordinates must remain consistent with camera calibration. Decoding a tag
and locating its corners already precede the optional metric pose solve, so using
one tag saves a different stage of work. Multiple associated tags can also feed
one common object-pose fit. Actual latency, association reliability, and alignment
error should determine these choices; they need not change the reporting contract.

For that clarified use case, the user supports Pi-side temporary tracking of the
selected stationary landmark. Navigatr already owns measurement
timing, calibration, and robot motion history. The Brain still owns the desired
contact relationship, approach, and whether a given estimate is suitable for its
behavior. This revisits the event-only proposal; it does not require preserving
the existing target tracker or adding a persistent estimate of every field object.
Only the selected feature needs a retained estimate. A static catalog of identities,
geometry, and optional nominal locations can be independent of that live state.

The recommendation is architectural, not evidence that a blind turn is accurate
enough. With a correctly associated fresh relative observation, a common rigid
offset in the odometry coordinates cancels when re-expressing the landmark relative
to the robot. Error in motion after that observation, initial visual/calibration
error, timing error, and possible landmark movement remain. Processing on the Pi
does not inherently reduce physical odometry error compared with the same math and
inputs on the Brain. The actual observe-turn-back-up maneuver needs measurement
against scoring tolerances. A question about allowable lateral and heading error
is pending; no acceptable error bound or blind interval has been established.

The assistant's resulting recommendation is to model the product as one integrated
VEX sensing subsystem with multiple capabilities. Navigatr owns acquisition,
calibration, timing, sensor fusion, and interpretation into useful geometry. The
Brain chooses goals and behavior. Continuous robot pose and occasional landmark
observations remain a plausible first interface, rather than the definition of
everything the subsystem may ever provide. A new sensor may improve an existing
estimate or support another measurement capability. A fixed processing structure
is compatible with replaceable sensor and algorithm implementations.

This recommendation also revisits the processing burden at the interface: making
the Brain reconstruct sensor timing is not inherently simpler. The Pi already has
the sensor context needed to time-align observations. Pairing an observation with
the matching robot pose or compensating it to an explicitly stated instant belongs
to measurement processing and is distinct from ongoing propagation after visibility
loss. The latest discussion considers moving that limited propagation into
Navigatr too. The timing-delivery mechanism has not been selected.

Assistant proposal for discussion, not yet user-approved:

1. Separate a robot coordinate reference, a catalog of recognizable landmarks
   and their local geometry, and a persistent estimate of landmark positions
   across the field. The first two can exist without the third. Robot position
   may be relative to an initialized odometry origin without storing field objects.
2. Let a requested landmark be a named physical object or a named face with a
   well-defined origin and axes. Pi-side sensor/mount geometry produces a pose
   relative to the robot. Brain-side behavior decides where its mechanism should
   go relative to that reported geometry. Defining a face does not choose a
   standoff or tell the robot which end to score with.
3. The event-only proposal remains an option: ongoing robot pose samples and new
   landmark observation events, with no event when no accepted observation occurs.
   For the later rear-scoring use case, the assistant recommends a continuously
   updated selected-landmark relative estimate when available, alongside its last
   accepted visual observation time. A new estimate timestamp must not masquerade
   as new visual evidence. A large landmark status enum is not required.
4. A possible alignment-friendly face convention is origin at a defined point
   on the face, +x inward through the face, +z up, and +y completing the
   right-handed frame. Robot +x is forward, +y left, +z up. Relative face yaw is
   counterclockwise from robot-forward to face +x. Front-first square alignment
   then reports yaw zero; rear-first square alignment reports 180 degrees modulo
   wrapping. The Brain owns the desired front/rear/other approach offset. This
   proposed inward convention differs from existing outward approach frames and
   must not silently replace them or reinterpret old configuration values.
5. Relative yaw is not deviation from a nominal field orientation and is not
   bearing to the landmark origin. Rotating both robot and landmark equally
   leaves relative yaw unchanged. Converting it into an odometry-frame heading
   requires the robot heading at the same effective time as the report; a delayed
   exposure-relative measurement cannot be added to the latest heading casually.
   Expressing the observation in field coordinates also carries robot-pose error
   into the inferred landmark pose. A single relative observation cannot separate
   unknown landmark field offset from unknown robot field-pose error. Alignment
   can nevertheless use the relative measurement directly. Its achievable error
   depends on sensing, calibration, timing, and control; this is not a guaranteed
   global error bound. After the last observation, reliance on odometry allows
   relative error to accumulate again. Correcting field localization would require
   an additional trusted reference or assumption about landmark field pose.
6. Association matches observations to the requested identity. A request is a
   filter, not evidence: indistinguishable goals/faces still require another cue
   or an ambiguous/unavailable result. A uniquely identified face may need only
   simple lookup; association need not imply a maintained global field.
7. Continuing after visibility loss is now under discussion as a Pi-side sensing
   responsibility. Retain only the selected landmark in local odometry and
   re-express it in current robot coordinates, assuming stationarity or an explicit
   landmark motion model. The Brain may separately retain a desired destination.
   No measured track exists until an observation is accepted; nominal placement
   must not silently substitute for acquisition. Clearing/switching selection or
   a discontinuous odometry reference requires explicit invalidation or conversion.

The current implementation described below predates that direction. Its target
tracker and world model are context for reconsideration, not the proposed design.

The broader problem clarified by the user is integrating multiple external
sensors into one useful VEX sensing device and Brain connection. Reliable robot
localization is the first capability; observation of physical features can support
precise alignment in autonomous and driver control. The existing repository framed
this more narrowly around robot motion and navigation targets. GATR2 has a V5
brain responsible for behavior and motor control, with custom electronics
supplying measurements and a Pi supplying estimates. A goal's center, its face,
the robot's tracked origin, and its actual contact point can all be distinct;
their geometry remains relevant even when the Brain owns destination construction.

A concrete configured example is backing the rear contact point toward a
selected face of a center goal, then executing a robot-relative move toward a
matchloader. This comes from an unfinished configuration template, not evidence
of an approved or tested autonomous route. See the
[two-wheel target template](../pi/navigatr/config/two_wheel_imu_apriltag_landmark_correction.xml.in)
and [target geometry](../pi/navigatr/src/resources/target_set.h).

The current design addresses that problem by integrating tracking wheels and
gyro for continuous local odometry, using camera evidence to resolve a selected
landmark-derived target, and latching the resulting destination. The intended
benefit is that the robot can turn or lose sight of the landmark while retaining
a stable destination. Vision's role in correcting targets, its acquire-once
policy, and whether it should also correct robot localization are open redesign
decisions, even though the existing docs describe a definite policy.

The current infrastructure is:

| Area | Role and evidence | Present boundary |
|---|---|---|
| `pcb/` | KiCad schematics, layouts, libraries, and board revisions for encoders, IMU, and Pi HAT. | Revision names and files do not establish fabrication or bring-up status. |
| `pico/` | RP2040 firmware, Arduino framework through PlatformIO. Acquires quadrature counts and SPI gyro; sends device-stamped sensor frames at a configured 50 Hz. | Acquisition and transport; wheel geometry and pose estimation live on the Pi. |
| `common/` | Dependency-light C++ framing and codecs for sensor, pose, and command messages. | Explicit integer units, masks, sequence numbers, framing and checksum; bytes can be implemented independently on another device. |
| `pi/navigatr/` | C++17 executable and library built with CMake; vendored TinyXML2; registered implementations selected by XML. | One synchronous loop with a fixed sequence: sensors, commands, preprocessing, localization prediction, perception, association, pose correction, world/target prediction, publishing. |
| `brain/` | Described as the Chomp V5 integration library. | Only a README is present here; the claimed library headers and production controller integration are absent. |
| `bench/rs485_link/` | Python Pi transmitter and a standalone V5 receiver example. | A link bring-up utility, not the complete controller. |
| `.github/workflows/ci.yaml` | Defines common/Pi host tests and a Pico firmware build; formatting is advisory. | Test definitions and workflow configuration are not evidence of on-robot performance or current CI results. |

On the Pi, resources own shared devices and immutable geometry/configuration;
logical sensors expose typed measurements; implementations consume those
contracts. Startup resolves references and payload compatibility before the
loop runs. Registration is compiled into the executable, not a dynamic plugin
system. A profile selects the full topology at startup; a target is a runtime
navigation request inside that topology. File replay replaces a serial resource
and uses the same decoder and downstream processing. See
[runtime construction and execution](../pi/navigatr/src/runtime/system.cpp),
[registration](../pi/navigatr/src/runtime/register_all.cpp), and
[build definition](../pi/navigatr/CMakeLists.txt).

That flexibility operates within existing assumptions: shared state describes
a planar robot and one active target, and target state carries Brain wire IDs.
Changing those concepts will require revisiting the common contracts as well
as swapping implementations.

These are candidate principles worth discussing independently of their current
implementation:

| Underlying concern | Current expression | What remains a choice |
|---|---|---|
| Keep hardware acquisition predictable while doing more expensive estimation elsewhere. | Pico/Pi/Brain responsibility split; backed-up telemetry is dropped rather than blocking acquisition. | Exact processor allocation, transport, sampling rates, and scheduling. |
| Describe interchangeable measurements and capabilities without teaching all algorithms each device's details. | Shared resources, logical sensors, typed payloads, opaque instance IDs and explicit references. | XML, registry design, class structure, number of abstractions, and plugin support. |
| Preserve clear control coordinates while changing field interpretation. | Smooth odometry `O` and field anchor `F`, with `T_field_robot = T_field_odom * T_odom_robot`; target poses latch in `O`. | Estimator, correction policy, and which coordinates the controller consumes. |
| Express the physical interaction the robot should achieve. | Landmark, approach face, desired contact-frame pose, and robot contact geometry resolve to a desired robot-body pose. | Pi versus Brain ownership of target construction; static configured targets versus richer runtime requests. |
| Interpret evidence at the time and in the coordinates where it was measured. | Device/host clock distinctions, pose history, camera exposure timestamps, explicit transform chains. | Clock synchronization method, buffering, interpolation, threading, and history limits. |
| Distinguish missing, old, invalid, and newly measured data. | Sensor health separate from retained latest sample, publication sequence, and freshness policy; raw gyro packets accumulate before consumers read the latest state. | Status vocabulary, queues, data retention, and degradation policy. |
| Keep old commands and observations from corrupting new intent. | Command deduplication, target generations, odometry epochs, reset invalidation. | Message protocol and whether invalidation cancels, reacquires, or recovers another way. |
| Treat uncertain visual evidence as uncertain. | Physical tag mounts separate from printed IDs, ambiguity gates, consistent observations from distinct frames, atomic acquisition, explicit timeout fallback. | AprilTags, active-target-only processing, acquire-once behavior, thresholds, and probabilistic estimation. |
| Expose missing calibration and broken configuration. | Explicit selections, startup checks, non-runnable templates and provisional calibration mode. | File format, schema, calibration workflow, and how verification evidence is stored. |
| Reproduce failures without the complete robot. | Shared live/replay decoder, fake devices, geometry and lifecycle tests. | Existing tests and APIs; test expected physical behavior rather than preserving incidental implementation. |

Useful source anchors for those concerns are
[robot frames and history](../pi/navigatr/src/state/robot_state.h),
[target acquisition](../pi/navigatr/src/impl/world_prediction/target_tracker.cpp),
[tag association](../pi/navigatr/src/impl/association/tag_mount_association.cpp),
[shared telemetry decoding](../pi/navigatr/src/impl/resources/pico_telemetry.cpp),
and [vision/target scenarios](../pi/navigatr/tests/vision_targets_gtest.cpp).

There is no user endorsement here of preserving the fixed nine-stage pipeline,
mandatory noops, a single function registry, XML, startup-only profiles, planar
robot state, current wire format, acquire-once targeting, AprilTags, or current
directory layout. Even hardware choices need to be distinguished from hardware
already built or otherwise committed. These choices should be assessed against
the desired behavior and actual constraints.

The implementation is less complete than the architectural prose can suggest:

- [Camera construction](../pi/navigatr/src/impl/resources/cameras.cpp) creates a
  `DeadCamera`; the [AprilTag factory](../pi/navigatr/src/impl/resources/tag_detectors.cpp)
  explicitly fails because the real detector backend is absent.
- The documented RS-485 operating mode holds transmit enable high. The camera
  templates explicitly reserve a separate, unspecified command return device.
  Brain-to-Pi target selection is a known unfinished integration boundary.
- The two/three-wheel odometry profiles disable publishing. The camera/target
  profiles are `.xml.in` templates. The legacy `navigatr.xml` has publishing but
  disables commands and vision. There is no complete deployable camera-target
  system demonstrated by these files.
- The CLI currently accepts an XML pathname. The documented allowlisted profile
  identity/hash deployment mechanism remains intended behavior.
- The present odometry path solves wheel geometry with a gyro heading
  constraint and integrates planar motion. It is not the EKF mentioned in older
  hardware prose; the predictor assigns confidence `1.0` rather than maintaining
  an uncertainty model. See [prediction](../pi/navigatr/src/impl/localization/wheel_imu_prediction.cpp).
- Documentation can disagree with both code and other documentation. For example,
  `hardware.md` describes wheel/gyro disagreement checking, but the current
  [publisher](../pi/navigatr/src/impl/publishing/vex_brain.cpp) derives gyro health
  from sensor validity and freshness. Older wire prose describes a requested
  object's pose, while the publisher can send the desired robot target pose in
  those same fields. Clarify the meaning before carrying that interface forward.
- Numeric examples, `verified` attributes, and detailed component rationales are
  repository claims. This investigation did not establish measurement provenance,
  validate datasheets, inspect live devices, or prove assembled hardware behavior.

Future discussion should resolve which of these concepts the user wants to
discard, the robot tasks and accuracy/timing outcomes that matter, which physical
hardware is already committed, and what the Brain should ask for and receive.
Those decisions will determine how much infrastructure is justified. Until then,
use this brief as context and a source map, not as a design to reproduce.

Investigation scope: read the architecture and hardware/interface documents,
configuration profiles, firmware/protocol and Pi runtime/algorithm code, test
scenarios, build workflow, PCB metadata, and recent repository history. No code
was changed and no builds, tests, deployment, or hardware sessions were run.
