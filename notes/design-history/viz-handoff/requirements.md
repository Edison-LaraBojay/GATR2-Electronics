# User request (2026-09-28, verbatim)

Implement a substantial improvement to the naviGATR visualizer and diagnostic tools, then perform an independent, in-depth audit of the resulting system.

ORDER MATTERS:
1. Implement and validate the visualizer/diagnostic improvements.
2. Establish the final implementation state.
3. Perform the read-only audit.
4. Report audit findings and recommended fixes. DO NOT implement those audit recommendations until we review them.

The highest priority is reducing visualizer latency and improving smoothness WITHOUT compromising localization, communication, control, or the accuracy of the information shown.

This is substantial work. Break it into coordinated tasks and use parallel agents where appropriate.

CURRENT CONTEXT

Our primary physical setup is:
- Two perpendicular tracking wheels:
  - Pico encoder port 0 measures forward/backward movement.
  - Pico encoder port 1 measures sideways movement.
- VEX IMU on Brain Smart Port 1.
- Brain–Pi communication over USB.
- Pico supplies encoder telemetry to the Pi.
- brain/localization-test is the motor-free integration application.
- brain/testing exercises driving/planning.
- External Pico IMU and RS-485 support must remain intact.
- Three-wheel localization/fusion and other supported drive configurations must remain intact.
- Robot dimensions/calibration values may still be placeholders.

The current visualizer already has substantial functionality, including robot/field visualization, paths, trails, obstacles, camera UI, and diagnostic panels. Inspect and extend what exists.

Do not assume previous reports accurately describe the current implementation. Verify the code.

Other agents may have modified the working tree. Preserve their work, establish file ownership before parallel edits, and avoid shared build-directory collisions.

Do not modify PCB files, commit, push, flash hardware, or deploy to the physical robot.

PHASE 1 — VISUALIZER AND DIAGNOSTIC IMPLEMENTATION

A. Establish a performance baseline

The user reports BOTH:
- Robot movement appearing late.
- Stuttering when interacting with the 3D view.

Separate these problems.

Confirm where work occurs:
- Pi: acquisition, estimation, inspection serialization, transport, and camera encoding.
- Browser: message processing, interface updates, and 3D rendering.

Measure what is possible locally:
- Snapshot generation duration and size.
- Actual publication/receipt rates.
- Outgoing queue size, replacements, drops, and backlog.
- Browser frame rate/frame time and update-processing cost.
- Round-trip time.
- Behavior under a slow consumer or burst of updates.

Distinguish source measurement age, publication delay, network delay, and browser delay. Do not subtract timestamps from unrelated clocks and call the result latency.

Record the test conditions. Do not invent hardware measurements or claim that a suspected bottleneck is proven.

B. Reduce latency and browser work

Inspect these known candidates:
- Full snapshots repeatedly include diagnostics and significant history.
- Some UI elements are rebuilt frequently.
- Outgoing snapshots can accumulate in FIFO order, leaving old positions queued while newer snapshots are refused.

Implement an appropriate solution, favoring simplicity and existing contracts.

Requirements:
- Favor the newest useful visualization state.
- Bound queue memory and stale-data accumulation.
- Coalesce replaceable, wholly unsent state updates where appropriate.
- Never corrupt a partially transmitted WebSocket frame.
- Preserve session/configuration/control ordering and necessary events.
- Treat recordings and events differently from replaceable live pose updates.
- Do not merely increase buffer sizes.

Separate frequent lightweight state updates from slower diagnostics/configuration/history updates where beneficial. Avoid resending unchanged data unnecessarily.

Maintain correct behavior for:
- Initial connection.
- Late-joining viewers.
- Reconnection.
- Session, epoch, profile, and placement changes.
- Dropped/coalesced updates.
- Full-state recovery after incremental updates.

Reduce unnecessary DOM rebuilding and allocations. Avoid expensive work for hidden panels. Retain readable diagnostics and the existing useful overlays.

Browser rendering should run independently of telemetry arrival.

If using display interpolation/smoothing:
- Keep it bounded and explicitly presentation-only.
- Minimize added latency.
- Preserve exact underlying measurements for readouts, graphs, export, and estimation.
- Do not hide genuine jumps, stale data, or discontinuities.
- Reset appropriately on invalid pose, placement, session/epoch changes, and reconnection.
- Do not extrapolate indefinitely during lost updates.

Measure improvements under repeatable conditions. Report before/after results where available, plus remaining limits.

C. Add useful hardware and transport instrumentation

Extend the existing diagnostic UI rather than creating a separate application.

Include, where supported:
- Recent RX/TX bytes in hex.
- Decoded packet/message names and useful fields.
- Decode rejection reasons and framing/error counters.
- Byte and packet rates.
- Last valid message times.
- Source freshness, retries, initialization state, and calibration state.

Observe existing transport reads/writes. Do not create a second reader that consumes bytes from an active serial port.

Raw capture must be optional, bounded, and inexpensive when disabled. Distinguish attempted transmissions from bytes actually accepted/sent if that affects interpretation.

Digital pin instrumentation:
- Label readings as HIGH/LOW or digital pin state.
- Show observed activity when available.
- Do not claim these are measured voltages or signal-quality measurements.
- UART normally idles HIGH; do not interpret a static HIGH as proof of disconnection.
- Software sampling can miss fast transitions; do not present it as a reliable oscilloscope.
- Do not change pin modes or interfere with UART/SPI operation to inspect pins.

Pico instrumentation:
- Consider forwarding the useful existing USB-console diagnostics to the Pi through an optional diagnostic message.
- Preserve normal sensor telemetry and acquisition priority.
- Useful examples include IMU initialization/retries, INT/RST/WAKE states, receive/error counts, and available report information.
- Keep unavailable diagnostics explicitly unavailable.

Encoder status must be honest:
- Counts, changes, direction, and available A/B observations are useful.
- Zero counts or no motion alone cannot distinguish a stationary encoder from a disconnected one.
- Fresh Pico packets do not prove every attached sensor is functioning.

Do not introduce a major protocol redesign just for debugging. Use clean, minimal extensions and document compatibility/rebuild requirements.

D. Live graphs and tuning overlays

Provide selectable time-series plots for available data, including:
- Localization x/y/heading and velocity.
- Roll/pitch when measured.
- Each wheel’s counts, changes, and derived travel.
- IMU rotation/rate and calibration bias.
- Sensor age, packet rate, errors, and recovery events.
- Commanded versus actual movement, tracking errors, and motor commands where reported.

Reuse the existing planned path and actual trail.
Add destination/desired heading/follower target overlays where useful and supported.

If additional Brain telemetry is needed, keep it optional and separate from the planner/controller’s functional behavior.

Make units, axes, sources, and validity clear.
Handle heading wrap sensibly.
Represent missing data as missing rather than silently filling it with zero.

Graphs must remain bounded. Decimate large recordings appropriately for display while retaining original recorded samples for export. Preserve spikes and gaps where possible.

E. Short diagnostic recording

Implement bounded detailed capture, preferably on the Pi so browser/network performance does not determine which samples are retained.

The intention is to record more of the data already available—not automatically change sensor rates, localization behavior, or control timing.

For example:
- Normal mode: lightweight visualization and a small rolling buffer.
- Trigger capture: retain a configurable short pre-trigger interval.
- Record a configurable additional interval, such as 10–30 seconds.
- Finish automatically and make the recording downloadable.

Capture actual available updates, not repeated samples generated by faster polling.

Allow selection of useful streams:
- Robot state.
- Encoder/IMU readings.
- Commands and reported actuation.
- Transport diagnostics/raw bytes.
- Lifecycle and calibration events.
- Paths/targets.

Record:
- Source measurement timestamps and clock identity.
- Pi arrival/publication timestamps where available.
- Sequence/session/epoch identifiers.
- Validity and freshness information.
- Dropped-record counts and reasons.
- Configuration/profile identity and relevant calibration values.

Keep capture work off critical acquisition/estimation/control paths. Bound memory, storage, duration, and raw-byte volume. Report overflow explicitly.

Define behavior for:
- Browser disconnecting during capture.
- Multiple viewers requesting capture.
- A source/session restarting during capture.
- Capture cancellation and normal completion.
- Resource limits or failed file writes.

Do not silently join incompatible sessions into a continuous trajectory.

Support optional automatic triggers for useful faults, such as link loss or localization continuity loss, if this can be implemented cleanly. Use cooldowns/limits so persistent faults cannot generate endless captures.

F. Replay and export

Provide a shared replay timeline:
- Play/pause.
- Scrubbing.
- Playback speed.
- Event markers.
- Synchronized graphs and 3D state.

Selecting an event or graph point should show the corresponding robot state and available path/target information.

Clearly distinguish replay from live operation. Replaying data must not issue robot commands or alter live state.

Export a portable bundle, ideally ZIP, containing:
- Separate CSVs for streams with different sampling times.
- Configuration/metadata in JSON or another documented format.
- Events.
- Units, coordinate conventions, clock/session information, and schema version.
- Data-loss/overflow information.

Do not force asynchronous sensors into one artificial sample grid for export.
Use stable, documented column names and represent missing values clearly.

The bundle should be usable outside the viewer and suitable for uploading to another tool or AI for analysis.

G. Attitude visualization

The existing viewer already appears to support roll/pitch when valid attitude is provided. Verify the full path.

Current VEX IMU telemetry may only carry heading; current Pico telemetry may only carry projected yaw information. Do not confuse startup mounting/gravity calibration with live attitude.

Make measured attitude available where feasible through clean, optional telemetry/sensor integration. Apply the correct IMU-to-robot mounting transform and coordinate convention.

Show measured, assumed-level, unavailable, and stale attitude distinctly.

Tilt visualization is the goal. Do not invent vertical position or imply reliable height estimation from the IMU.

If one hardware path cannot supply usable attitude yet, document that explicitly while preserving the working path.

H. Camera preview without field correction

Provide an easy configuration for our current Brain-profile USB localization setup with optional live camera preview.

Requirements:
- Localization still works without field correction.
- Camera preview does not require calibrated intrinsics, measured mounting geometry, tag association, or landmark correction.
- Field geometry, robot pose, and paths remain visible.
- Missing/unavailable camera must not invalidate unrelated localization.
- Do not silently enable field correction because the camera is enabled.
- Clearly distinguish preview-only behavior from calibrated estimation.
- Reuse existing camera resources/pipelines where appropriate.

Keep camera changes minimal. We have not yet physically tested the camera.

Document:
- Required Pi packages/build options.
- Configuration selection.
- How to enable preview.
- What needs restarting/rebuilding.
- Whether any Brain/Pico reupload is actually necessary.
- The later steps required to enable calibrated field estimation.

Do not promise physical hot-plug support for a CSI ribbon camera. Software optionality and safe physical connection are different matters.

Keep camera preview rate/size bounded so it cannot create unbounded visualization backlog.

I. Validation and documentation

Run appropriate host checks in an isolated build directory.
Add meaningful tests for:
- Backpressure and latest-state delivery.
- Partial-frame integrity and message ordering.
- Viewer reconnect/reset/state merging.
- Smoothing reset/staleness behavior.
- Capture bounds, dropped data, and session boundaries.
- Export/replay consistency.
- Preview-only configuration without field correction.

Exercise the actual browser where practical, including a slow-client scenario and a recording/replay workflow.

Keep all existing core functionality intact.
Distinguish host validation from hardware validation.
Do not claim physical tests that were not performed.

Provide concise operating instructions for the user, with Windows/Brain/Pico/Pi commands clearly separated where relevant.

Suggested implementation workflow:
1. Agree on minimal snapshot/capture/export contracts and file ownership.
2. Parallelize:
   - Inspection transport and server performance.
   - Browser rendering, UI, graphs, and replay.
   - Capture/export backend.
   - Camera/attitude/instrumentation integration.
3. Integrate serially.
4. Validate and document.
5. Freeze implementation for the audit below.

Adapt this breakdown to the available agents and actual dependencies. Avoid several agents editing the same file concurrently.

PHASE 2 — IN-DEPTH READ-ONLY AUDIT

Only begin this phase after the above implementation and validation are complete.

Spend substantial time examining both the recent changes and the entire pipeline. Parallelize independent reviews where useful. Prefer independent reviewers over having each agent only review its own work.

This phase is REPORT ONLY:
- No fixes, formatting, refactoring, configuration edits, deployment, or hardware operations.
- Record the commit and working-tree state inspected.
- Recheck findings against the final code.
- Existing validation results are evidence only for the exact version and scenarios tested.
- Recommendations return to us for review before implementation.

Audit these areas:

1. Actual integration
Trace the configured primary setup end to end:
Pico → Resources → Sensors → localization → robot state/history
→ publishing → communiGATR → Brain consumers.

Also trace world estimation, planning, control, inspection, and profile/placement flows.

Identify disconnected implementations, stale configuration formats, unsupported combinations, and documentation/code contradictions.

2. Recent fixes
Recheck previous findings rather than accepting “fixed” claims:
- Navigation request acceptance/consumption/publication races.
- Neutral manual input accidentally cancelling navigation.
- Lost or discarded sensor motion and continuity invalidation.
- Calibration timing and movement rejection.
- Profile/placement/reconnect ordering.

Classify each as fixed, still present, uncertain, or inapplicable.

3. Measurement correctness
Review:
- Clock identity, timestamp origin, arrival time, wraparound, and reset epochs.
- Duplicate consumption and measurement lineage.
- Wheel/IMU interval alignment, deadlines, and lost increments.
- All supported wheel layouts.
- Geometry, polarity, CPR, gearing, and per-wheel scale.
- IMU mounting, bias, gravity alignment, stationary detection, and ZUPT assumptions.
- Acceleration being mistaken for gravity.
- Units, transforms, heading wrapping, covariance, and misleading confidence/readiness.

Distinguish deliberate model limitations from defects.

4. Recovery
Trace:
- Brain restart.
- Pi restart.
- Pico restart.
- Cable disconnect/reconnect without process restart.
- IMU failure, return, and recalibration.
- Profile/placement commands during recovery.
- Old replies/partial packets crossing session boundaries.

Explain what is retained, invalidated, retried, or acknowledged. Find cases requiring an unnecessary manual restart.

Review thread ownership, lifetime safety, locking, bounded queues, shutdown, and optional work delaying critical work.

5. World estimation
Review:
- Capture-time pose/attitude lookup.
- Camera mounting translation and rotation.
- Robot-center versus camera-center coordinates.
- Association and landmark symmetry conventions.
- Nominal versus observed states.
- Stale estimates and placement changes.
- Whether localization-derived information is incorrectly fed back as independent evidence.

6. Planning and actuation
Review:
- Origin-relative and landmark-relative goals.
- Field transfer/update consistency.
- Robot footprint and obstacle inflation.
- Full-path collision checks and field boundaries.
- Holonomic/nonholonomic feasibility.
- Tank/mecanum conventions.
- PID timing, saturation, integral behavior, stopping, cancellation, and settling.
- Stale pose, invalid placement, lost links, and obsolete commands.
- Landmark changes during execution.

7. New visualizer/diagnostic work
Review whether:
- Backpressure still creates hidden stale state.
- Smoothing conceals faults.
- Reconnect/session handling mixes incompatible data.
- Capture/export loses meaning, timestamps, or evidence.
- Raw instrumentation interferes with communication.
- Recording or preview harms localization/control timing.
- UI health indicators overstate what is actually known.
- Replay can accidentally affect the live robot.
- Resource usage stays bounded.

8. Tests and recommendations
Identify missing asynchronous, failure, recovery, malformed-input, and integration coverage.

For each substantive finding include:
- Severity and practical impact.
- Exact file/line references.
- Trigger or concrete interleaving.
- Why existing checks/tests do not prevent it.
- Recommended minimal correction.
- Tradeoffs and plausible counterarguments.
- Proposed regression test.
- Confidence and remaining hardware uncertainty.

Consolidate duplicate findings. Challenge assumptions before reporting.
Do not manufacture findings to fill the report.

FINAL DELIVERABLE

Provide:
1. Implemented visualizer/diagnostic changes.
2. How to build/run/use them on our current setup.
3. Measured performance results and test evidence.
4. Remaining implementation limitations.
5. The separate read-only audit report.
6. Prioritized recommendations:
   - Current localization-test blockers.
   - Driving-test blockers.
   - Release blockers.
   - Optional improvements.
7. A dependency-aware follow-up plan.

Do not implement the audit recommendations. We will review that report separately.

# Second user request in the same message (verbatim)

also another stupid task, where possible and I really mean it whever possible make the names end with GATR however only really folders/main infrastructure yk? not like a file name unless its an abstract thing right, but like investigatr, navigatr, actugatr right and document those name changes and change those folder names and justify why the name made sense right this again you can implement the folder names alreeady maybe file name changes if possible and obviously namespace I figure if applicable names right but it has to make sense dont do it for the sake of just it ending in gatr it has to make sense, like navigatr right and shit and make it tuff wherever possible and shit

# Lead notes
- The renames are done (see docs/naming.md): brain/localization-test is now brain/locaGATR, brain/testing is brain/operaGATR, pico/ firmware is pico/aggreGATR, common/ is translaGATR (namespace translagatr), pi/navigatr is pi/naviGATR, and the viewer is pi/naviGATR/spectaGATR.
- The case-only folder rename pi/navigatr to pi/naviGATR is still pending on disk and in git, because a user process holds the folder. On Windows the paths are case-insensitive, so use pi/naviGATR everywhere.
