# Worker B progress (Pi)

## Stage B1: Pi build restored under v4, v4 server basics (DONE)

### Files changed (all under pi/navigatr)
- `CMakeLists.txt`: `common/link_documents.cpp` added to navigatr_core.
- `tests/CMakeLists.txt`: `brain_link_e2e_gtest.cpp`, the `BRAIN_SOURCES` list and the Brain include dirs removed. The test file itself is untouched and not compiled (it is v3). Lead writes the v4 e2e test in Phase C; B re-adds the CMake entries on request.
- `src/contracts/brain_profile.h` (new): `BrainProfileHost::prepare(doc, id, reason, detail)`, the seam the commands slot calls after decode + `validateRobotProfile`. System implements it in B2.
- `src/contracts/slot_init.h`: `BrainProfileHost* brain_profile` (null = config takes no profile). Removed `world_estimation_noop` (only the v3 SELECT refinement used it).
- `src/runtime/system.cpp`: dropped the `world_estimation_noop` fill. Nothing else yet (B2 sets `brain_profile` and the boundary).
- `src/state/command_state.h`: v4 `BrainReplyContext` (profile_id, received, profile_state/reason/detail, doc_kind/doc_id/doc_offset/doc_max_len, action); new `ProfileStatus {state, reason, detail, id, applied_id}` and `PathReport {session, command_id, mode, count, points[13] (m), received}` in `CommandState`. `object_*` target selection kept (see risks).
- `src/impl/commands/brain_link_commands.{h,cpp}`: v4 server.
- `src/impl/publishing/brain_link_publisher.{h,cpp}`: v4 replies; FieldObject mapping removed.
- `src/transport/serial_port.{h,cpp}`: review finding F.
- Tests: `brain_io_gtest.cpp` (rewritten for v4), `brain_imu_bench_gtest.cpp`, `pros_usb_link_gtest.cpp`, `half_duplex_gtest.cpp`, `end_to_end_gtest.cpp`, `parallel_wheels_gtest.cpp`.
- Config comments that described FieldObject/SELECT: `config/shared/pipelines/parallel_wheels_bno08x_camera.xml`, `config/override/diagnostics/three_wheel_imu_camera_pipeline.xml.in`.

### Behavior implemented (commands slot)
- Order: version -> unknown op (UnsupportedOp; ops 3, 5 retired) -> `request_id == 0` or malformed body (InvalidArgument) -> HELLO -> session -> dedupe -> execute.
- Malformed body checks (no Pi state needed; they run before session/dedupe and never consume the request id): GET_STATE unknown imu flag bits; READ_DOC kind not 1/2 or max_len 0; PROFILE_WRITE total_len outside [24, kProfileMaxLen] or offset+len > total_len; PROFILE_APPLY total_len out of range; CONTROL action not 1/2; PATH_REPORT mode > 2.
- Dedupe (spec 3): SET_POSE and CONTROL records answer duplicates with the recorded result (never re-executed; counted in `duplicates`); the newest id with GET_STATE/READ_DOC/PROFILE_WRITE/PROFILE_APPLY/PATH_REPORT and an identical body is executed again (GET_STATE does not re-accept its IMU sample); same id with another op or body is InvalidArgument; older is Stale.
- GET_STATE: sample goes to the BenchImu mailbox when configured (v3 `accept` semantics, including invalid = reset), ignored otherwise; never UnsupportedOp.
- HELLO: new session clears dedupe records, the bench mailbox and the path report; keeps localization, anchor, profile status and staging. Unapplied SET_POSE withdrawn by the session change as before.
- SET_POSE: NotReady (recorded, header-only reply) when a `BrainProfileHost` exists and `profile.applied_id == 0`; otherwise v3.
- PROFILE_WRITE: single staging slot keyed by (profile_id, total_len), survives sessions, cleared by `reset()`. Resend below `received` with identical bytes Ok; different bytes or offset above `received` InvalidArgument. A new id restarts staging only when the write is acceptable (offset 0); an invalid write leaves staging intact. Ok body: profile_id, received.
- PROFILE_APPLY: InvalidArgument unless the staged document is complete, same id and length, crc32 == id. Then: id == applied_id -> Ok/Applied, no reset; id applying -> Pending; id reported rejected or in the remembered ring (4) -> ProfileRejected with the stored reason, host not called again; no host -> ProfileRejected NotAccepted; decode fail -> Format; `validateRobotProfile` fail -> its reason/detail; `host->prepare` fail -> its reason/detail; success -> Pending, status Applying. Every rejection sets the reported status (state Rejected, that id) while `applied_id` keeps running.
- READ_DOC: shape-checked, request recorded in the reply context; publisher answers Unavailable (no field documents until B2).
- CONTROL: NotReady for now, recorded for dedupe (B3).
- PATH_REPORT: stored in `CommandState.path` (meters, session, command_id, mode, points, host receipt); mode none clears. Ok header only.

### Behavior implemented (publisher)
- State block: v3 robot/health fields plus profile_state/reason/detail/id from `CommandState.profile`; map_id, estimate_id 0; calibration none.
- SET_POSE Pending upgraded to Ok when applied (v3). READ_DOC Ok -> Unavailable. Other bodies from the reply context.
- `<Publishing>` children limited to Serial and Health; `<FieldObject>` fails with "FieldObject was removed with brain link v4 ...".

### Review finding F
- `SerialPort::open`: `c_cflag &= ~CRTSCTS` after cfmakeraw.
- `SerialPort::transmitterEmpty`: TIOCSERGETLSR when available, else TIOCOUTQ (queue empty -> 1, else 0, ioctl error -> -1). No tcdrain, so the half-duplex loop is bounded by its deadline while DE is held. Limitation (commented): an empty kernel queue can leave bytes in the UART FIFO; the loop polls only after the frame airtime and then holds the post guard.
- Not compiled here: this is the POSIX branch; no Linux compiler on this box (no WSL). TIOCOUTQ is a standard Linux tty ioctl.

### Test results (exact)
- Configure: `cmake -S C:/GATR2-Electronics/pi/navigatr -B $S/wB-nav -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$S/nav-base/_deps/googletest-src` (PATH with /c/msys64/ucrt64/bin first).
- Build: `cmake --build $S/wB-nav -j8`: exit 0, no warnings (-Wall -Wextra).
- Tests: `ctest --test-dir $S/wB-nav -j8`: **377/377 passed** (0 failed).
- Mutation check (reverted, file byte-identical afterwards): disabling the SET_POSE NotReady gate, the same-body check for newest resends, and the resend byte compare each made the expected BrainLink tests fail (4 failures).

### CONTRACT ISSUES
- (note, not blocking) Remembered rejections: the Pi answers a re-APPLY of a rejected id from memory until a Pi restart. communiGATR's `resubmitProfile()` with the same document therefore gets the same rejection; only a changed document (new crc) retries. This matches spec 5.1 ("remembered for that id"). Worker A should not expect `resubmitProfile()` to force a Pi rebuild.
- (note) Staging is one slot. Two Brains, or interleaved writes of two ids, thrash it. Not expected with one Brain.

### Risks / notes for the lead
- `CommandState.object_*` (target selection for `configured_targets`) has no v4 Brain op, so target resolution has no production trigger. Kept because `vision_targets_gtest` drives it in-process and the spec retires only the FieldObject mapping. Decide in Phase C whether to retire target resolution.
- `brain_link_e2e_gtest.cpp` is on disk but not built (v3). Lead replaces it.
- Pi docs still describe v3 (SELECT, op 5, FieldObject): docs stage (5.6).
- B2 must: set `slot_context.brain_profile` before building CommandCollection for `<BrainProfile>` configs, implement the boundary, and write `command_.profile` (Applied / Rejected Build) at the swap; health GyroFresh for the bench mailbox follows the profile then.
- (correction) the profile boundary items above are stage B3, not B2; B2 was the field documents.

## Stage B2: field collision semantics and field documents (DONE)

### Files (all mine)
- `pi/navigatr/src/config/config_node.{h,cpp}`: `onlyChildren`, `onlyAttributes`, `atMostOne` (strict schema helpers).
- `pi/navigatr/src/config/field_map.{h,cpp}`: `FieldMap::revision`, `boundary` (FieldBoundaryDecl), `obstacles` (ObstacleDecl), `findObstacle`; `LandmarkDecl::wire_id`, `LandmarkDecl::box` (CollisionBoxDecl). Parser rules:
  - root children {Dimensions, Boundary, Feature, Obstacle, Landmark}; root attributes {id, type, name, revision, calibration_status};
  - Landmark attributes {id, wire_id, calibration_status}; children {NominalPose, ApproachFrame, TagMount, Visual, CollisionBox};
  - Boundary, CollisionBox and Obstacle strict;
  - revision 1..65535; wire_id 1..65535, unique over landmarks and obstacles (optional on Landmark at parse, required on Obstacle);
  - at most one Boundary or CollisionBox; box sizes > 0; Boundary min < max;
  - Obstacle id unique and never a Landmark id.
- `pi/navigatr/src/impl/publishing/field_documents.{h,cpp}` (new): `loadFieldMapFile`, `buildFieldMapDocument`, `fieldEstimateRecords`, `fieldReferencesHeader`, class `FieldDocuments` (update/read/reset, 3 retained).
- `pi/navigatr/src/impl/publishing/brain_link_publisher.{h,cpp}`:
  - `<Field resource_id estimate_period_ms="200"/>`: strict, at most one, period > 0;
  - map built at create; a build error names the reason;
  - `update()` every reporting cycle, before the reply;
  - state block map_id and estimate_id; READ_DOC served;
  - `reset()` drops the estimates; `fieldDocuments()` accessor.
- `pi/navigatr/src/app/field_refs.cpp` (new): tool `navigatr_field_refs <field.xml> <out.h> [label]`.
- `pi/navigatr/CMakeLists.txt`: field_documents.cpp in navigatr_core; `navigatr_field_refs` executable; custom target `field_references` (regenerates the checked-in header, label `pi/navigatr/config/override/field.xml`).
- `pi/navigatr/tests/CMakeLists.txt`: `field_documents_gtest.cpp`; define `NAVIGATR_FIELD_REFERENCES_H`.
- `brain/testing/include/field_references.h` (generated): map_id 0xF6B3F2C9, revision 1, 9 references.
- `pi/navigatr/config/override/field.xml`:
  - revision="1"; Boundary 0..3.5664;
  - wire_id 1-9 on the goals (file order); CollisionBox 0.1543 m square on every goal;
  - 8 Obstacles (loaders 101-104, toggles 111-114), each box the Feature footprint;
  - tape not an obstacle; notes and provisional labels.
- Checked-in brain_link pipelines now serve the field with `<Field resource_id="override_field" estimate_period_ms="200"/>`:
  - bench_vex_imu.xml, bench_vex_imu_usb.xml;
  - parallel_wheels_bno08x_camera.xml, parallel_wheels_bno08x_no_camera.xml;
  - three_wheel_imu_camera_diagnostic.xml, three_wheel_imu_no_correction.xml, three_wheel_imu_camera_pipeline.xml.in;
  - `three_wheel_imu.xml.in` gained `<Field file="../field.xml"/>`.
- Tests:
  - new `field_documents_gtest.cpp` (19 tests);
  - `field_assets_gtest.cpp` (+7 tests);
  - `parallel_wheels_gtest.cpp`: BrainPlacesTheRobotAndReadsTheField reads the served map and estimate;
  - `brain_io_gtest.cpp`: see the contract section.
- Docs:
  - `pi/navigatr/docs/field_assets.md`: Planning data section, checks, provisional list;
  - `pi/navigatr/docs/landmarks.md`: Brain output rewritten for v4, new Field documents section;
  - `docs/navigatr_resources.md`: field_map schema;
  - targeted v3 fixes in `pi/navigatr/docs/{architecture,coordinates,setup,parallel_wheel_bringup,calibration_inventory}.md` and `pi/navigatr/README.md`.

### Behavior
- **Map document.**
  - Landmarks are kind landmark with flags estimated and reference, plus obstacle with a box. Obstacles are kind fixed, flag obstacle with a box; no box means not an obstacle.
  - Sorted by wire id.
  - Rounding: positions to the nearest mm, headings to the cdeg; box sizes UP to whole mm (goal 155, loader 95 x 103, toggle 661 x 52); the boundary INWARD (0..3566 mm).
  - map_id = crc32, checked with validateFieldMap; a map_id of 0 is refused.
  - Build errors: no revision, no Boundary, a landmark without wire_id, more than 128 objects, a value outside the wire range.
- **Estimate records.**
  - Observed only when the FieldObjectState is valid, kObserved, from the robot's odometry_epoch and host-stamped. Pose = compose(robot.field_from_odom, T_odom_object); age = now - lastObservedAt, clamped.
  - Everything else is nominal (map pose, valid, age 0): fixed objects, missing entries (noop), map-seeded entries, invalid entries, other epochs.
  - Header: map_id, estimate_id, u32 epoch, u32 anchor.
- **Snapshot rule.**
  - The first update takes one at once.
  - Afterwards a new id needs both: a change (the records' source, flags, x, y or heading at wire resolution, or the epoch, or the anchor), and at least estimate_period_ms since the last one taken.
  - Age alone never takes one; the document keeps the age at snapshot time.
  - 3 retained. Ids count up per publisher and are never reused, also across reset(); reset drops the retained documents.
- **READ_DOC.**
  - doc_id 0 = current.
  - Kind 1 with another id: Stale. Kind 2 not retained: Stale; none yet: Unavailable.
  - Unknown kind, max_len 0, or offset >= total: InvalidArgument.
  - data = min(max_len, 96, total - offset). The reply carries the resolved id, total_len, and the crc32 of the whole document (for the map, equal to map_id).
  - No Field: Unavailable, and map_id and estimate_id are 0.
- **Generated header.**
  - Format exactly as spec 5.3.
  - References are the objects with the reference flag (the landmarks), in map order, named by the CamelCase of the id.
  - Refuses ids that are not identifiers, that collide (for example goal_a and goal__a), or that shadow Origin, RobotAtStart, kMapId or kRevision.

### Test results (exact)
- Build dir `$S/wB-nav` (MinGW Makefiles, Debug, ucrt64 g++ 15.2, -Wall -Wextra): `cmake --build $S/wB-nav -j8 --clean-first` exit 0, **0 warnings**.
- `ctest --test-dir $S/wB-nav -j8`: **403/403 passed**.
  - 377 before B2; +19 field_documents, +7 field_assets, +1 brain_io, 1 renamed.
- `cmake --build $S/wB-nav --target field_references` regenerated the header; `FieldAssets.CheckedInFieldReferencesMatchTheField` passes byte for byte.
- The generated header is syntax-checked, including investigatr/reference.h and static_asserts on the ids and kMapId: clean with host g++ -std=c++17 and with PROS arm-none-eabi-g++ 13.3.1 gnu++20 (-Wall -Wextra).
- Mutation check in a scratch copy ($S/wB-mut, not the repo): 21 mutants over field_documents.cpp/.h, brain_link_publisher.cpp and field_map.cpp. All 21 are killed, after I added 3 tests for the first-round survivors: heading-only change, publisher reset, and an out-of-order declaration that needs the sort.

### CONTRACT ISSUES / deviations (none blocking)
- **Goal box 0.1543 m, not the spec's 0.1542.** The base octagon's circumscribed circle is 0.1425/cos(22.5 deg) = 0.15424 m. At 0.1542 the required "goal box contains the Visual base" test would fail by 0.04 mm. On the wire both are 155 mm (sizes round up), so nothing downstream changes.
- **Wire rounding.** Box sizes round up and the boundary rounds inward (a Pi choice inside the existing layout). Brain consumers see boxes >= declared and a boundary <= declared.
- **Estimate age.** age_ms is the age when the snapshot was taken, and an age change alone does not produce a new estimate_id. Worker A (LinkDriver field age) and the lead: add (now - received_at) when judging observation age.
- **Estimate ids.** estimate_id restarts at 1 after a Pi process restart; within a process it is monotonic, also across System::reset. Key estimates by (pi_instance, estimate_id). map_id is content based and stable across Pi restarts, so a Brain map cache keyed by map_id survives them.
- **Lead amendment 8.2** changed common/ during B2: profile header and wheel are 32 bytes, op 11 READ_WHEELS exists, CONTROL actions 3/4 exist, and CONTROL NotReady is header only. I adjusted just enough to keep the Pi green; B3/B4 implement the rest:
  - brain_io_gtest profile lengths now derive from kProfileHeaderLen and kProfileWheelLen (they were literal 72/96);
  - the retired-op test uses op 12 as the undefined op, since 11 is now defined;
  - brain_link_commands: READ_WHEELS is Unavailable without a profile host and NotReady with one, and is answered again as a read-only op. CONTROL accepts actions 1..4 (all still NotReady); action 5+ is InvalidArgument. New test `BrainLinkSession.WheelReadingsAndPicoControlWaitForTheProfileBoundary`.

### Notes for other workers / lead
- **Brain references.** Brain code can `#include "field_references.h"` (brain/testing/include) and use `Field::NeutralGoal0Center` etc. with investigatr::Reference. `Field::kMapId` = 0xF6B3F2C9 for field.xml revision 1. After any planning edit of field.xml:
  1. bump revision;
  2. run `cmake --build <dir> --target field_references`;
  3. rebuild the Brain apps.
- **Served everywhere.** Every checked-in brain_link profile, including the current USB bench (bench_vex_imu_usb.xml), now serves the Override map: 17 objects, map 500 bytes = 6 READ_DOC chunks, estimate 364 bytes = 4 chunks.
- **Inspection** (spec 5.5: boundary, boxes and map_id in hello) is not done in B2. `BrainLinkPublisher::fieldDocuments()` and `buildFieldMapDocument` are ready for it.
- **Worker D:** I edited pi/navigatr/CMakeLists.txt and tests/CMakeLists.txt with the Edit tool (context-matched insertions), no reformatting.

### Risks
- Nothing here ran on hardware. The collision boxes are derived from the manual and the CAD, not measured.
- **Stale reads with large fields.** With 128 objects an estimate is 2584 bytes (27 chunks). At about 20 ms per chunk, with 3 retained and a 200 ms period, a slow reader can see Stale and must restart. This is documented; the Brain restarts from the newest.

## Stage B3: Brain robot profile application (DONE, 2026-09-27 night)

Note: HEAD is now d4bc875 ("new stuff"; the user committed the whole tree at 21:41). Most B3 code is inside that commit; `git diff` shows only what changed after it.

### Files (all mine, under pi/navigatr)
- **8.2 correction mapping (one application point each)**
  - `src/impl/sensors/pico_channels.{h,cpp}`: `<Calibration counts_per_revolution gear invert>`; wheel angle = counts * 2pi / (cpr * gear), negated by invert (= kWheelReversed, encoder polarity).
  - `src/resources/wheel_geometry.h`, `src/impl/resources/wheel_geometry_resource.cpp`: optional `travel_scale` (default 1, > 0).
  - `src/impl/localization/tracking_wheel_motion.{h,cpp}`: travel = d(angle) * radius * travel_scale * sign, once; HeadingConstraint `window_ms` (the bias window needs bias_samples AND window_ms of sample time); `recalibrate()`; the calibration travel gate also counts right after a dropped window (calibration branch now before the baseline branch).
  - `src/impl/localization/brain_imu_wheel_bench.{h,cpp}`: travel_scale; inline `TrackingWheel` form besides the Wheels reference; parallel mode accepts 0 or 180 deg wheels (projection by ux, equivalent to normalizing 180 to 0 with the sign reversed).
  - `src/impl/localization/imu_heading_increment.{h,cpp}`: Calibration `window_ms`; `recalibrate()`.
  - `src/contracts/localization.h`: `RobotObservationFunction::recalibrate()` (default false).
  - Profile-built configs: the geometric `direction` is always positive; the Pico IMU sign is the sensor invert (kImuInvert).
- **Profile builder** `src/runtime/brain_profile_builder.{h,cpp}` (new):
  - strict `<BrainProfile>` schema (Encoders/Port, Imu, BrainImu, Calibration, Timing, Fusion, History);
  - `checkProfileCapabilities`: wired ports, IMU source and port, BrainImu mailbox, cameras refused, the supported topology x IMU table, three wheels + Pico IMU needs `<Fusion>`;
  - `writeProfileSubtrees`: a tinyxml2 Sensors + Localization tree from the typed profile (ids `profile_encoder_<port>`, `profile_imu_<port>`, `profile_motion`, `profile_heading`), built by the ordinary factories;
  - the profile's calibration_window_ms and still_travel_um replace the Pi defaults when nonzero; still_rate_cdps is not consumed yet (B4 StationaryWindow).
- **Stationary precheck** `src/runtime/stationary_precheck.{h,cpp}` (new): over the last 300 ms every profile wheel < 1 mm, Pico gyro |rate| < 2 deg/s, each source fresh (150 ms), the window covered, no gap > 100 ms, no discontinuity; a repeated record is not evidence. VEX profiles check the wheels only (B4 adds the VEX rotation).
- `src/contracts/brain_profile.h`: `ProfileBinding`; `BrainProfileHost::prepare/applied/control`.
- `src/runtime/system.{h,cpp}`:
  - waiting noop localization;
  - candidate built in `prepareProfile` on the estimation worker (capability check, generated tree, make_sensors + make_localization; nothing running is touched);
  - boundary `swapProfile`: top of `step()` inline, `applyPendingProfile()` from the main thread in worker mode (reset() stop/swap/restart pattern);
  - **new this session: a candidate superseded by a later APPLY (the running profile again, or a refused one) is dropped at the boundary.** Before, it would have swapped in while the Brain had been told the other id was applied;
  - placement floor in `requestsFrom`; persistent feed; `BindingView` for race-free inspection reads; `RuntimeEvent` log (32 kept);
  - `controlProfile`: CONTROL 1/2 with the precheck; 3/4 NotReady until B4;
  - `reset()` returns to waiting.
- `src/runtime/sensor_stage.{h,cpp}`: `SensorExecutor::replaceProfile` (profile sensors swapped, configured ones kept with their records; a reused id continues under a new record epoch).
- `src/runtime/localization_stage.{h,cpp}`: `continueFrom` (same feed and counters, epoch + 1, anchor revision continued, unplaced), `recalibrate`.
- `src/impl/commands/brain_link_commands.{h,cpp}`, `src/state/command_state.h`: CONTROL routed to the host; detail kept in the dedupe record.
- `src/impl/publishing/brain_link_publisher.{h,cpp}`: with a profile host `<Health>` takes only fresh_ms; encoders, gyro (Pico channel or the Brain bench mailbox), BiasCal and `calibration` follow the binding; the CONTROL reply carries calibration and detail.
- `src/inspection/inspection_document.cpp`: hello reads `bindingView()` (sensors, estimator type, warnings).
- `src/app/main.cpp`: the worker loop calls `applyPendingProfile()` every 20 ms and prints events.
- Configs: `config/override/brain_profile_usb.xml`, `brain_profile_rs485.xml` (header comment rewrapped this session).
- CMake: navigatr_core += brain_profile_builder.cpp, stationary_precheck.cpp; tests += brain_profile_gtest.cpp.
- Tests: `tests/brain_profile_gtest.cpp`, 36 tests (34 BrainProfile, 2 StationaryPrecheck). New this session: TheLatestApplyWinsOverACandidateWaitingForTheBoundary, PicoImuInversionAppliesOnce, ThreeWheelRecalibrationRestartsTheFusedGyroBias, AllCorrectionsApplyOnceInTheTrackingWheelModel, ForwardAndSidewaysScalesArePerWheelWith{TheVexImu,ThePicoGyro}, Rs485ConfigDiffersFromUsbOnlyInTheBrainLink; NewProfileResetsAndWithdrawsPlacement now also checks the anchor continuation and the sensor record epoch.

### Behavior summary
- Waiting: noop localization, no pose, profile_state none; SET_POSE, CONTROL and READ_WHEELS NotReady. Pico telemetry, the Brain link, field documents, inspection and raw encoder source health keep running.
- PROFILE_APPLY: the commands slot decodes and runs `validateRobotProfile`; the host runs the capability checks and builds the candidate at once. Failures are ProfileRejected with reason/detail, remembered per id. Success is Pending until the boundary. Re-APPLY of the running id is Ok with no reset.
- Boundary, new profile: odometry epoch + 1 (never regresses), history cleared, unplaced, anchor revision continued, every earlier SET_POSE withdrawn (placement floor), profile sensors restart under new record epochs. The commands slot (session, pi_instance, staging, dedupe), resources, world estimation, publishing and inspection survive.
- Models per the spec 5.1 table. Everything else is refused (ImuCombination, Observability, EncoderPort, ImuSource, ImuPort, Camera).
- CONTROL 1: precheck (NotStationary), then the active model's bias calibration restarts; the pose holds. brain_vex: nothing to calibrate on the Pi (Ok, calibration none).
- CONTROL 2: precheck, then a localization reset (epoch + 1, unplaced, SET_POSE withdrawn, bias recalibration).
- CONTROL duplicates answer from the record with the current calibration state.

### Test results (exact)
- `cmake --build $S/wB-nav -j8 --clean-first`: exit 0, **0 warnings** (-Wall -Wextra, ucrt64 g++ 15.2).
- `ctest --test-dir $S/wB-nav -j8`: **439/439 passed** (403 after B2, +36 in brain_profile_gtest).
- The two worker-mode tests repeated 15 times: 15/15.
- **Mutation check** in the scratch copy `$S/wB-mut` (script `b3_mutants2.sh`, log `b3_mutants2.log`): **45 mutants, all killed.** This includes the round-1 survivor M5 (tracking model ignores travel_scale) and new mutants for the supersede check, anchor, sensor epoch, feed, binding view, both recalibrate hooks, both calibration windows, IMU invert and the capability checks.
- Lead e2e harness copied to `$S/wB-e2e` (build `$S/wB-e2e-build`, 0 warnings): **6/8 pass**. Both failures are B4 items of spec 8.8: `WheelReadingsReportRawTravel` (READ_WHEELS still NotReady) and `PulledCableStopsTheMoveAndTheLinkResumes` (the bench model drops travel over a VEX IMU gap).

### CONTRACT ISSUES / deviations (none blocking)
- Profiles with camera records are refused (kProfileReasonCamera): no camera mount path from the profile exists on the Pi (spec 5.6: a camera variant only if implemented and tested). `<CameraSlot>` is not in the schema.
- The candidate is built on APPLY, before the boundary, so a build failure is reported at once as ProfileRejected/Build. The boundary itself only moves objects and cannot fail.
- A SET_POSE withdrawn by a boundary stays Pending on the wire; the Brain's placement ticket times out and a new SET_POSE is needed. Same as the v3 session-change withdrawal.
- Worker mode: the boundary is the main thread's 20 ms poll. A Brain request that arrives while the workers are paused can miss its 40 ms reply window; the Brain retries.

### Notes for B4 / lead
- B4: StationaryWindow (8.3) and its thresholds, still_rate_cdps, READ_WHEELS, CONTROL 3/4 through PicoControl, Pico configure on apply, health bits 4..7, the 8.8 gap bridging.
- brain_link_e2e CMake entries not touched (8.8: the lead adds them).

### Risks
- Nothing here ran on hardware. The profile path is tested with memory links, the real NG1 envelope and codec, and synthetic Pico frames only.
- The worker-mode supersede test runs the workers on the real host clock for 200 ms per request. It passed 15/15 repeats but assumes the machine never stalls a worker that long.

## Stage B4: calibration and recovery (spec 8.3, 8.7, 8.8) (DONE; final section at the end)

Started fresh: no B4 edits were on disk (git diff on pi/navigatr showed only B3 work).

### Plan
- impl/localization/stationary_window.{h,cpp}: StationaryWindow (evidence, gaps, identity, time-based qualification) and GyroBiasCalibration (bounded attempt, gated maintenance, invalidation).
- tracking_wheel_motion, imu_heading_increment, brain_imu_wheel_bench use it; Stillness status in ObservationReadiness; executor zero velocity while stationary.
- Travel loss (8.8): models count unbridgeable travel; LocalizationExecutor breaks continuity (epoch + 1, unplaced); System withdraws SET_POSE and logs.
- Bench model bridges gaps up to bridge_max_ms with unchanged identities.
- READ_WHEELS through BrainProfileHost::readWheels; CONTROL 3/4 through PicoControl; CONFIGURE on apply; health bits 4..7.

## LEAD NOTE to Worker B (2026-09-27 23:05): answer to your question, read this first
- The other agent editing tracking_wheel_motion.{h,cpp} at 22:59-23:00 was an accidental duplicate of you. My message resumed a copy of your agent id instead of reaching you. **It is stopped now. You own all B4 files again; nobody else will edit them.**
- It acted on a scope change the user made at 22:58. **Spec 8.8 is rewritten: detect only, bench model only.**
  - No bridging, no bridge_max_ms, no Continuity element, and no travel-loss changes in tracking_wheel_motion or the other Pico/three-wheel models.
  - Its removal of your continuity code from tracking_wheel_motion therefore matches the new scope; do not restore it.
  - Keep your StationaryWindow work. Check that tracking_wheel_motion.{h,cpp} compiles; the duplicate stopped mid-edit and may have left the bench .cpp half done.
- Still required in B4:
  - 8.3 StationaryWindow;
  - READ_WHEELS Ok;
  - CONTROL 3/4 through PicoControl;
  - the bench-model travel-loss detection (8.8 item 1: clear localized, new epoch, withdraw SET_POSE, log);
  - health bits.
- Probably dropped (user deciding, lean scope): Pico CONFIGURE on apply. Do it last, only if time allows.

## LEAD DECISION for Worker B (2026-09-27 23:08): answer (b), with details
- **The removal was intended.** The user narrowed spec 8.8 at 22:58 to detect-only, bench model only; the lead relayed it. The editing agent was an accidental duplicate of your id, created when I tried to message you, and it is stopped. **You own all four files; no other agent will touch them.**
- **tracking_wheel_motion.{h,cpp}.** Final state: no travel-loss, bridge, Continuity or anchor_angle code.
  - Keep your StationaryWindow and recalibration work.
  - Use the other agent's header, and remove the travel-loss parts from your restored .cpp so the pair compiles.
- **brain_imu_wheel_bench.{h,cpp}.** Start from your own versions and remove only the bridging: bridge_max_ms and gap integration. **Keep the detection:**
  - When the model rebaselines after a gap or an identity change and any profile wheel moved more than still_travel, report the travel loss. The executor and System then break continuity: epoch + 1, unplaced, pending SET_POSE withdrawn, RuntimeEvent / inspection note.
  - If nothing moved, keep the quiet rebaseline.
  - Keep the executor and System continuity plumbing.
- **Keep the consequence one switch.** The user may still choose "warn only": keep the pose, log the loss, no unplace. Put the unplace behind one flag in the bench model or executor, default unplace. I will note here if the user flips it.
- **Permissions.** If the classifier still blocks an edit to your own files, stop and say so in your stage result. Do not work around it; I will take it to the user.
- **Lean scope:** do Pico CONFIGURE on apply last, only if time allows.

## USER DECISION on 8.8 (2026-09-27 23:12)
Travel lost during a sensor gap -> warn, and the pose becomes invalid (unplaced, epoch + 1, pending SET_POSE withdrawn); the operator places again at the start position. That is the default of the switch from the LEAD DECISION above; keep it. Warning = RuntimeEvent / inspection note on the Pi. The Brain apps show their own on-screen warning from the placement loss.

## USER SCOPE CUT (2026-09-27 23:15): Pico CONFIGURE on apply is dropped (spec 8.9). Do not implement it.

## USER RULE (2026-09-27 23:20): spec 8.10 REPLACES the 8.8 travel-loss detection. Read 8.10.
Any used sensor that goes stale for more than sensor_loss_ms (250 ms) or changes identity -> the pose is invalid: unplaced, epoch + 1, SET_POSE withdrawn, note. This applies to every profile model, with no moved-wheels condition. An unused Pico IMU never matters. Generalize your bench-only detection into this; it is simpler, living at the executor/System level over the profile's bound sources.

## Stage B4 status after the lead decisions (Worker B, 2026-09-27 ~23:45; superseded by the final section below)
- Read the LEAD NOTE, LEAD DECISION, USER DECISION, the scope cut and USER RULE 8.10. Applied:
  - tracking_wheel_motion.{h,cpp}: the other agent's header, and my .cpp without travel-loss/bridge/Continuity/anchor code (StationaryWindow work kept).
  - brain_imu_wheel_bench: no bridging, no model-level loss; stale or restart rebaselines as before B4; StationaryWindow stillness kept.
  - Travel-loss plumbing replaced by spec 8.10 at System level: runtime/sensor_loss.{h,cpp} SensorLossMonitor over the profile's used sources; System::watchSensors -> LocalizationExecutor::loseContinuity (epoch + 1, unplaced, pose shown held), placement floor, RuntimeEvent "sensor lost: <source> ...: place again".
  - Switch: <BrainProfile><Timing sensor_loss_ms="250" on_sensor_loss="unplace|warn"/>, default unplace.
  - Pico CONFIGURE removed (8.9).
- Build $S/wB-nav: 0 warnings; ctest 499/499. Lead e2e harness ($S/wB-e2e -> $S/wB-e2e-build, current repo e2e test): 9/9.

## Stage B4: calibration and recovery, final (Worker B, 2026-09-28)

Scope as decided by the lead and the user: spec 8.3 (StationaryWindow, READ_WHEELS, CONTROL 3/4, health bits), 8.8 narrowed, 8.9 (no Pico CONFIGURE) and 8.10 (a used sensor that drops invalidates the pose). Nothing here ran on hardware.

### Files (all pi/navigatr, mine)
- New:
  - `src/impl/localization/stationary_window.{h,cpp}`: `StationaryWindow`, `GyroBiasCalibration`, `readStillness`, `stillnessOf`.
  - `src/state/stillness.h`: `BiasCalibration` (values equal gatr2::CalibrationState; static_assert in the publisher) and `StillnessStatus`.
  - `src/runtime/sensor_loss.{h,cpp}`: `SensorLossMonitor` (spec 8.10).
  - `src/runtime/pico_control_ref.h`: `requirePicoControl` / `parsePicoReference` for `<Pico resource_id>`.
  - `tests/stationary_window_gtest.cpp` (14 tests).
- Changed:
  - `src/contracts/localization.h`: `ObservationReadiness::stillness`.
  - `src/state/robot_state_feed.h`: `ObservationFunctionStatus::stillness`; `LocalizationStatus::stationary()`, `continuity_breaks`, `last_break`.
  - `src/contracts/brain_profile.h`: `BrainProfileHost::controlProgress`, `readWheels`.
  - `src/state/command_state.h`: `BrainReplyContext::wheel_count`, `wheels`.
  - `src/impl/localization/tracking_wheel_motion.{h,cpp}`: HeadingConstraint bias from the StationaryWindow (the other agent's header kept per the lead; no travel-loss/bridge code).
  - `src/impl/localization/imu_heading_increment.{h,cpp}`: same window, optional `<Wheel sensor_id radius_m/>` evidence under `<Calibration>`.
  - `src/impl/localization/brain_imu_wheel_bench.{h,cpp}`: `<Stillness>` window over wheels + VEX rotation; no bridging; rebaseline behaviour as before B4.
  - `src/runtime/localization_stage.{h,cpp}`: stillness in function status; zero reported velocity while stationary; `loseContinuity(why)`.
  - `src/runtime/stationary_precheck.{h,cpp}`: Brain VEX rotation source.
  - `src/runtime/system.{h,cpp}`: `<Pico>` on the CommandCollection, CONTROL 3/4, controlProgress, readWheels, watchSensors/loseSensor, recovery events.
  - `src/runtime/brain_profile_builder.{h,cpp}`: Calibration still_rate_dps, max_rate_dps, evidence_gap_ms, attempt_s; Timing sensor_loss_ms, on_sensor_loss; generated Stillness/Calibration attributes and wheel evidence.
  - `src/impl/commands/brain_link_commands.{h,cpp}`: READ_WHEELS through the host; a CONTROL duplicate with a Pending record asks the host for progress.
  - `src/impl/publishing/brain_link_publisher.{h,cpp}`: `<Pico>`, health bits 4..7, calibration states, READ_WHEELS body.
  - `config/override/brain_profile_{usb,rs485}.xml`: `<Pico resource_id="pico_telemetry"/>` on CommandCollection and Publishing, new Calibration and Timing attributes.
  - `CMakeLists.txt` (+stationary_window.cpp, sensor_loss.cpp), `tests/CMakeLists.txt` (+stationary_window_gtest.cpp).
  - Tests: brain_profile_gtest.cpp (FakePico, `mask_drop`, `pico_zero` in the rig; BrainRecovery and SensorLoss suites; SameProfileReapplies now uses the Pico IMU profile, a VEX Brain restart is its own SensorLoss test; VexRecalibrate expects Failed/ImuUnused for CONTROL 3), brain_io_gtest.cpp (FakeProfileHost), localization_models_gtest.cpp (calibration tests feed every source; 4 new), localization_gtest.cpp (no aggregate init).

### Behaviour
- **StationaryWindow** (every IMU bias path and the stationary status):
  - A sample counts only when its (sequence, record epoch) is new.
  - These restart it as WaitingData:
    - an identity change: record epoch, encoder discontinuity, IMU source epoch or Brain IMU epoch;
    - a gap over evidence_gap_ms (100) or an out-of-order sample;
    - a gyro accumulator drop;
    - a source quiet for evidence_gap_ms of host time.
  - These count as movement and restart it as WaitingStill:
    - a wheel range over still_travel (1 mm);
    - a gyro over max_rate (5 deg/s), or further than still_rate (1 deg/s) from the window mean;
    - a VEX rotation range over still_rate x window.
  - It qualifies when every source spans window_ms (profile, or 2000) of its own sample time with bias_samples (20) samples. The next window starts from the newest samples.
  - WaitingStill returns to collecting after a quarter window of still samples.
- **GyroBiasCalibration**:
  - The first qualified window gives the bias: accumulated angle over elapsed time, or the trapezoid when there is no accumulator.
  - No qualified window within attempt_s (60) gives Failed, which stays until recalibrate.
  - Each later window moves the bias by 0.2 of the difference, at most 0.2 deg/s.
  - A gyro record or source epoch change invalidates the bias and starts a new attempt, in tracking_wheel_motion and imu_heading_increment.
  - The sample that completes the window is calibration data; the next one seeds the integrator. This is the behaviour from before B4.
- **Status.** ObservationReadiness.stillness carries the calibration state (wire values), reason, progress, windows, attempts, steps, bias and the stationary flag.
  - The publisher maps the bias function's state to `calibration`.
  - Health bit 7 comes from LocalizationStatus::stationary().
  - While stationary, the executor reports zero velocity and leaves the pose alone. This is gated stationary handling, not a ZUPT filter.
- **VEX profiles.** The bench window runs over the wheels and the VEX rotation. Calibration is None. A failed or absent Pico IMU never matters.
- **READ_WHEELS** returns one record per profile wheel, in profile order:
  - port;
  - fresh (150 ms) and valid flags;
  - discontinuity: low 16 bits of the encoder discontinuity plus the sensor record epoch;
  - raw counts from the pico_telemetry output;
  - travel_um: wheel angle x radius, with CPR, gear and polarity applied and no travel scale;
  - age.
  - NotReady without an applied profile; Unavailable without `<BrainProfile>`.
- **CONTROL 3 (ReinitImu):**
  - Failed/ImuUnused unless the profile uses the Pico IMU.
  - Failed/PicoLink without a Pico link, identity or fresh frames, or when submit returns 0.
  - Otherwise it sends PicoOpReinitImu with the IMU port as argument and answers Pending until the request completes. On completion it recalibrates and answers Ok.
  - Failure cases:
    - a Pico failure gives Failed with the Pico detail (PicoRefused when it has none);
    - a lost request gives Failed/PicoLink;
    - 15 s gives Failed/TimedOut.
- **CONTROL 4 (RestartAcquisition):** stationary precheck first. Then Pending until the request completes and the link shows a new acq_epoch or restart count; then Ok.
- **CONTROL duplicates.** A duplicate of a Pending CONTROL asks controlProgress and updates the record; it never resubmits.
- **Completion of CONTROL 3 or 4** ends pose continuity while placed (8.10: a used source restarted).
- **Health bits 4..6** are reported whatever the profile uses:
  - bit 4 PicoLink = frames fresh;
  - bit 5 ImuInitializing = status known and Initializing, Aligning or Retrying;
  - bit 6 ImuFailed = status known and Failed.
- **Sensor loss (8.10).** SensorLossMonitor watches only the sources the profile uses: the encoders, the Pico IMU when imu_source is pico, and the Brain VEX mailbox when brain_vex.
  - Edges (a single event):
    - an identity change;
    - a Pico reboot or acquisition restart (link counters);
    - a restart of the used Pico IMU.
  - Levels (a lasting condition):
    - stale beyond sensor_loss_ms, by host receipt. An invalid VEX sample counts as missing and moves its epoch.
    - a used Pico IMU whose status is not Ready.
  - System::watchSensors runs after commands and before localization. With the robot placed it calls loseContinuity:
    - epoch + 1;
    - unplaced, with the pose shown held;
    - placement floor = init_sequence, so a SET_POSE in the same cycle is withdrawn too;
    - an event "sensor lost: <source> ...: place again (odometry epoch N)".
  - on_sensor_loss="warn" only logs, once per edge or per stale stretch.
  - Placing while a source is still lost does not hold: the next cycle unplaces again.
- **Events** (RuntimeEvent, printed by main, for B5 inspection):
  - sensor lost;
  - Brain requests resumed, or quiet for 1 s;
  - Brain session opened;
  - Pico frames lost or restored;
  - Pico reboot, acquisition epoch, IMU restart;
  - per function: calibration done, failed or restarted;
  - the Pico command lifecycle.

### Test results (exact)
- **Build** `$S/wB-nav` (MinGW Makefiles, Debug, g++ 15.2, -Wall -Wextra):
  - `cmake --build $S/wB-nav -j8 --clean-first`: exit 0, **0 warnings**;
  - after the last test edits, `cmake --build $S/wB-nav -j8`: exit 0, 0 warnings.
- **ctest** `ctest --test-dir $S/wB-nav -j8`: **505/505 passed**. That is 439 after B3, plus Worker D's pico_link tests, plus mine:
  - 14 in stationary_window_gtest;
  - 4 new in localization_models;
  - 1 new precheck test;
  - 19 BrainRecovery and SensorLoss tests;
  - the edits to existing tests listed below.
- **Whole suite in one process**, `tests/navigatr_tests.exe --gtest_repeat=2`: 505 + 505 passed.
- **Worker tests:**
  - `--gtest_filter=Workers.* --gtest_repeat=15`: 15/15;
  - `BrainProfile.WorkerMode*:BrainProfile.TheLatestApply*` x15: 30/30.
- **Lead e2e harness** (`$S/wB-e2e` -> `$S/wB-e2e-build`, current repo brain_link_e2e_gtest.cpp, 0 warnings): **9/9 passed**. This includes WheelReadingsReportRawTravel, PulledCableStopsTheMoveAndTheLinkResumes, UsedSensorDropInvalidatesThePoseEvenWhenStill and BrainRestartKeepsTheProfileButNeedsAPlacement.
- **Mutation check** in the scratch copy `$S/wB-b4-mut` (mutants.sh, survivors.sh, mutants.log): 51 mutants.
  - The first round killed 45 of them.
  - I added tests for the survivors: W9 accumulator, H2 imu_heading restart, B1 VEX rotation evidence, Y9 same-cycle SET_POSE, S2 encoder restart without Pico identity. All five are now killed.
  - Result: **50/51 killed**.
  - The survivor, S6, is equivalent: an invalid VEX sample updates the identity one cycle early and produces the same loss.

### Changes to existing tests (intended behaviour changes)
- **localization_models:**
  - The calibration tests now feed every source, since retained records are no evidence.
  - CalibratesThenIntegratesWithProvenance uses stationary-sized rates; 0.5 rad/s counts as turning under the 5 deg/s limit.
  - CalibrationRebases checks WaitingStill.
- **brain_profile:**
  - SameProfileReappliesWithoutResetting uses the Pico IMU. The VEX Brain-restart case moved to SensorLoss.ABrainRestartRestartsTheVexImu, which expects unplaced (8.10).
  - VexRecalibrate expects CONTROL 3 to fail with ImuUnused.
  - TheLatestApplyWins runs with on_sensor_loss=warn, because its worker phases have no sensor traffic.
- **workers_gtest SlowFieldWorkNeverStallsEstimation:**
  - The loop waited on cycle(), which counts at cycle start, while the assertion reads finished cycles. Under load it failed 149 vs 150.
  - Older builds failed the same way under load.
  - The loop now waits on estimationStats().cycles.
- **brain_io_gtest:** FakeProfileHost implements the new host calls and no longer uses aggregate init.
- **localization_gtest:** no aggregate init of ObservationReadiness; the new field would warn.

### CONTRACT ISSUES / deviations (none blocking)
- `<Pico resource_id>` resolves require<PicoTelemetry> first and falls back to require<PicoControl> (runtime/pico_control_ref.h). The fallback lets tests register a FakePico under the PicoControl contract without depending on Worker D's PicoTelemetry constructor.
- Completing CONTROL 3 or 4 unplaces the robot. 8.3 said "pose holds" and 8.10 says Pico IMU reinit needs a placement; this follows 8.10.
- Pico CONFIGURE is not implemented (8.9). The builder writes nothing for it, and the Pico IMU stays enabled.
- A pure stale gap unplaces even when the robot stood still (8.10). Examples:
  - a USB outage over 250 ms with a VEX profile;
  - a Brain program restart, because the new session moves the mailbox epoch.
- A new SET_POSE placed while a used source is still stale is withdrawn on the next cycle. The Brain must place again once the sources are fresh.
- Not detectable, to document in B5: a physically disconnected quadrature encoder keeps its last count and reads as standing still.

### For B5 and the lead
- **B5 docs:**
  - Pages: pi/navigatr/docs architecture, calibration_inventory, localization_fusion, parallel_wheel_bringup, setup, vex_imu_bench and usb_localization_bench, the new brain_profile.md, and inspection.
  - Topics:
    - the StationaryWindow and the calibration states, and that this is not a ZUPT filter;
    - sensor loss and its switch;
    - READ_WHEELS, CONTROL 3/4 and the health bits;
    - the `<Pico>` element.
- **B5 inspection** sources:
  - per-function stillness (functionStatus().stillness);
  - System::events();
  - System::picoControl()->link();
  - wheel readings. System::readWheels is private; B5 exposes a reader.
- **Lead:**
  - the e2e CMake target is still yours (8.8);
  - the docs/communigatr.md lines noted in phase_c are still yours.

### Risks
- Nothing ran on hardware. Untested against real conditions:
  - the stillness thresholds against real sensor noise (BNO08X, ASM330, VEX IMU);
  - Pico status timing;
  - real USB outages.
- sensor_loss_ms of 250 ms is untested against real Brain poll gaps (lost replies, map transfers) and Pico UART jitter. A value that is too tight unplaces spuriously. It is configurable, and warn mode exists.

## Stage B5: configs, inspection and viewer, docs (DONE, 2026-09-28 ~01:30)

Started fresh: no B5 edits were on disk. Nothing ran on hardware.

### Code (all pi/navigatr, mine)
- `src/state/command_state.h`: CommandState `pi_instance`, `link_open`, `last_request`; the brain_link commands slot sets them every run (`brain_link_commands.{h,cpp}`; header comments also lose the stale "Pico configure").
- `src/contracts/publishing.h`: `PublishingOutput::brain_state` (optional BrainState). The brain_link publisher fills it every cycle with the block it would answer; the GET_STATE reply uses the same value.
- `src/runtime/system.{h,cpp}`: `PicoOperation` is a public struct; `ReportingSnapshot` gains `brain_state`, `wheels` (READ_WHEELS now, profiled configs) and `pico_operation`, filled in `reportingCycle`.
- `src/runtime/brain_profile_builder.{h,cpp}`: public `profileTopologyName`, `profileReasonName` (moved from system.cpp and the builder's anonymous namespace).
- `src/inspection/inspection_document.{h,cpp}`:
  - hello: `configuration.commands_type`, `configuration.brain_profile`; `fields[]` `revision`, `map_id` (8 hex digits, from buildFieldMapDocument) or `map_error`, `boundary`, `obstacles[]` {id, wire_id, pose, collision_box, note}, `landmarks[].wire_id` and `collision_box`.
  - snapshot: `localization.stationary`, `continuity_breaks`, `last_break`, `functions[].stillness`; `brain_link` (null without brain_link commands): takes_profile, link_open, session, pi_instance, last request time/age, `state` (the GET_STATE block decoded: flags, health bits, calibration, profile state/id, map_id, estimate_id, epoch, anchor), `profile` {state, id, applied_id, reason, detail, running {id, generation, topology, summary, wheels with CPR/gear/reversed/radius/x/y/angle/travel scale, imu {source, port, vex_smart_port, inverted, bias_function}, footprint, calibration settings}}, `calibration` (the bias function's stillness), `wheels` (readings plus that wheel's corrections, matched by port), `operation` (CONTROL 3/4 on the Pico), `path`; `pico` (the CommandCollection's Pico link, else the first Pico telemetry resource: identity, epochs, restarts, status, IMU state/reason/attempts, last command); `events` (bounded log).
- Viewer (`viewer/`): new `link_panel.js` (readiness line link/profile/sensors/map/calibration/placement, profile, calibration, wheel table, Pico line, event log newest first); `field_scene.js` draws the boundary, fixed obstacle boxes, landmark boxes (at the estimate when observed, else nominal), the profile footprint as the robot outline (else RobotBody), the Brain path polyline; `planning` toggle; hidden objects are not hover-picked; `app.js` readiness badge and `#status data-readiness`; `diagnostics.js` stillness columns and brain link/path rows; `index.html`, `style.css`.
- `tools/install_service.sh`: default config is now `config/override/brain_profile_usb.xml` (was the XML RS-485 bench); usage text updated; dry run checked.
- Configs: comments only. `brain_profile_{usb,rs485}.xml` lose "the IMU enable" (CONFIGURE was dropped); the four runnable XML-configured profiles (`bench_vex_imu*.xml`, `parallel_wheels_bno08x*.xml`) say they refuse Brain profiles and name the primary configs.

### Tests
- `brain_profile_gtest.cpp`:
  - The Rig runs either checked-in config: `brain_profile_rs485.xml` has its `brain_uart` replaced by raw frames over the memory link (USB keeps NG1). The Pi end is a `PullableLink` a test can close like a pulled USB cable.
  - New: `BrainProfileConfigs.UsbRunsTheCurrentBrainProfile`, `BrainProfileConfigs.Rs485RunsTheCurrentBrainProfile` (waiting state, SET_POSE NotReady, map served, the current Brain profile of gatr2_robot.h applied over the wire, readiness bits, placement, each wheel on its axis, inspection agrees); `BrainProfileInspection.HelloCarriesThePlanningFieldTheBrainReads`, `.SnapshotShowsProfileCalibrationWheelsPicoEventsAndPath`, `.SensorLossAndStillnessShowInTheSnapshot` (cable pull: link_open false, continuity break named, request age, reopen).
- `brain_io_gtest.cpp`: new `BrainLinkSession.LinkLivenessGoesToTheCommandStateEveryCycle` (closable link: pi_instance before any request, last request time, link_open false while closed and true again).
- `inspection_document_gtest.cpp`: null planning data and null brain_link/pico for a display-only config, stillness present; `validJson` moved to namespace `inspection_test` so brain_profile_gtest shares it.

### Results (exact)
- `cmake --build $S/wB-nav -j8 --clean-first`: exit 0, **0 warnings** (-Wall -Wextra, ucrt64 g++ 15.2).
- `ctest --test-dir $S/wB-nav -j8`: **511/511 passed** (505 after B4 + 6 new).
- `tests/navigatr_tests.exe --gtest_repeat=2`: 511 + 511 passed.
- New tests x15: all passed. `Workers.*:BrainProfile.WorkerMode*:BrainProfile.TheLatestApply*:Inspection*` x15: 15/15 (27 tests each).
- Lead e2e harness (`$S/wB-e2e` -> `$S/wB-e2e-build`, repo `brain_link_e2e_gtest.cpp`): **9/9 passed**, no compiler warnings.
- Mutation check in `$S/wB-b5-mut` (`mutants.sh`, `survivors.sh`, `mutants.log`): **28/28 killed** (26 in round 1; the two survivors, link_open and pi_instance written as constants in the snapshot, are killed after the PullableLink and pi_instance checks were added).
- Viewer, headless Chrome over the DevTools protocol (`$S/wB-b5/cdp_check.py`) against a scratch rig (`$S/wB-b5/viewer_rig.cpp`: brain_profile_usb.xml over memory links, scripted Brain and Pico, the inspection service):
  - VEX profile: readiness "ready", profile, map f6b3f2c9 rev 1, wheels table, path, boxes, footprint drawn (`cdp1.png`); no page errors.
  - 1.5 s cable pull: "link quiet", "stale: VEX IMU", "needs placement", event "sensor lost: brain_vex_imu stale 263 ms: place again" (`cdp2.png`).
  - Pico IMU profile while moving: "calibration waits for stillness", reason "wheel port 1 moved" (`cdp4.png`).
  - Synthetic demo (camera, no Brain link): unchanged, panel says no Brain link (`cdp_demo.png`).
  - `tools/viewer_smoke.sh` could not be used: headless Edge and Chrome never finish `--virtual-time-budget` with the live WebSocket on this box (watchdog kills them); the CDP script replaces it here.

### Docs
- New `pi/navigatr/docs/brain_profile.md`: ownership (Brain, Pi, Pico), configs and schema, lifecycle (waiting, apply, continuity table), supported setups and every refusal reason, corrections once (pipeline), READ_WHEELS, calibration and stationary handling (StationaryWindow, bias, VEX ownership, BNO08X alignment, not a ZUPT filter, the undetectable pulled encoder), CONTROL 1-4 with the precheck numbers, sensor loss and recovery tables, health bits, inspection, transport exception.
- Owned docs are plain ASCII now (the old m/s squared, multiplication and en dash symbols in setup.md, parallel_wheel_bringup.md and calibration_inventory.md replaced).
- Updated: `pi/navigatr/docs/{architecture, configuration, inspection, landmarks, setup, usb_localization_bench (rewritten), vex_imu_bench (rewritten), calibration_inventory, parallel_wheel_bringup, localization_fusion}.md`, `pi/navigatr/README.md`, `docs/navigatr_resources.md` (pros_usb_link, brain_imu_bench, pico_telemetry as PicoControl, serial reopen and TIOCOUTQ, wheel_geometry travel_scale, Brain profile consumers), `docs/navigatr_sensors.md` (gear, restarts, generated profile sensors), `docs/pi_setup.md` (Brain-profile configs, transport table, service install commands), `docs/hardware.md` (two-way Pico link, USB section, logical ports).

### Notes for the lead and other workers
- **The current Brain programs cannot place against an XML-configured Pi config.** They always send a profile; those configs answer ProfileRejected/not accepted, and the communiGATR client refuses placement while its profile is not applied. By design (spec 5.2); documented in every affected doc and config header. Use the Brain-profile configs with the current apps.
- `tools/install_service.sh` now defaults to `brain_profile_usb.xml`. The documented commands pass `--config` explicitly anyway.
- `docs/brain_setup.md` (lead's) section 5 says Pico IMU reinit (Y) keeps the pose ("The pose holds"); section 8 and the Pi say it needs a placement (8.10). `common/frames.h` (A's) comments on ControlAction 3 and 4 also say "pose holds". Worth aligning.
- The inspection contract stays `navigatr.inspect/1`: everything is additive, and the viewer ignores what it does not know.
- The e2e CMake target is still the lead's (8.8).

### Risks
- Nothing ran on hardware. The viewer was checked in headless Chrome with software WebGL against a scripted rig, not a Pi.
- `readWheels` now also runs every reporting cycle for inspection (two or three wheels, cheap); worker-mode tests stay green.

## Stage FIX: review findings for B1-B5 (Worker B, 2026-09-28)

HEAD is now f657cdf (the user committed the tree); `git diff` on pi/navigatr shows only this stage. Nothing ran on hardware.

### Per-finding disposition
1. **high, bench drops 200-250 ms gaps silently: FIXED.**
   - Every observation function now counts the times it discarded measured motion after it had a baseline: `ObservationReadiness::dropped_intervals` + `dropped_why` (also in `ObservationFunctionStatus` and the inspection snapshot).
   - `LocalizationExecutor` sums the rises per run (`droppedIntervals()`, `lastDrop()`). `System::watchModels` runs right after localization (profiled configs): a rise while placed ends continuity like a sensor loss (`losePlacement`, event "motion lost: <function>: <why>: place again", placement floor so a SET_POSE from that cycle is withdrawn too). Same `on_sensor_loss` switch; in warn mode the note is skipped when watchSensors already logged a loss that cycle.
   - `loseContinuity` now publishes to the feed at once (inspection never shows localized after a post-localization loss).
   - Bench: new `<Freshness max_age_ms="200"/>` (default keeps XML bench configs unchanged); the builder writes `max_age_ms = sensor_loss_ms`, so at defaults a VEX or wheel gap up to 250 ms is one measured step (no travel lost) and a longer one is dropped and unplaces. Counted drops: missing/stale/invalid after a baseline, IMU epoch change, receipt not advancing, step > max_age ("no VEX IMU and wheel pair for N ms"), encoder restart, rotation jump.
   - SensorLossMonitor: an invalid VEX sample or a new Brain session (mailbox epoch move) is now a restart edge in that same cycle, so the event names the sensor before the bench's drop.
   - Docs corrected: usb_localization_bench.md, brain_profile.md (new "Motion lost" part, table row), vex_imu_bench.md, calibration_inventory.md (raising sensor_loss_ms alone never lets a gap pass unmeasured), setup.md, architecture.md, pi_setup.md.
2. **medium, Pico gyro gap just over max_gap_ms: FIXED** by the same counter: `TrackingWheelMotion::dropWindow` counts every dropped window (drops_ was unread before). imu_heading_increment drops are NOT counted: in the supported pairing (three wheels, weighted_planar_fusion) the wheels carry rotation and the estimator uses it alone for an interval without a heading, so no travel is lost (header comment says so). No receipt-gap edge added to SensorLossMonitor: host receipt gaps also appear on Pi stalls, where device-clock models lose nothing.
3. **medium, motion while the gyro bias calibrates: FIXED.**
   - `StationaryWindow::movements()` (restarts for movement; `StillnessStatus::movements`, in inspection).
   - Tracking model, while not calibrated: a movement restart of its window counts a dropped interval ("moved while the gyro bias calibrated (<reason>)"); wheel travel rebased away adds up per wheel and counts past `max_calibration_travel_m` (catches a push across a sample gap, where the window restarts for the gap and never compares across it). A sample the window already counted is not counted again.
   - Covers CONTROL 1 on a placed robot, placement during the first calibration, and a Failed calibration (still not calibrated: any movement unplaces).
   - Found while tracing: after calibration the gyro seeded one sample later than the wheels (pre-existing since before B4). With 50 Hz Pico frames (20 ms) and a 20 ms interval tolerance, jitter would misalign the first solve and drop it after max_pending_ms, which would now unplace a robot placed during calibration. Fixed: the completing sample seeds the gyro, the same sample the wheels rebase on.
4. **low, count-only bias windows: FIXED.** Both factories default `window_ms` to 2000 (the StillnessConfig default); `readStillness` and `<BrainProfile><Calibration window_ms>` now require window_ms > 0. Checked-in synthetic demos set `window_ms="1000"` (the rig holds still 2 s at 50 Hz); tests with short still lead-ins set explicit windows (end_to_end 990, parallel_wheels 990, synthetic_rig 700, inspection_document 150, model unit tests 5/10). Hardware pipelines (bias_samples 200 at 50 Hz = 4 s) are unaffected by the 2 s default.

### Files (all mine)
- src: contracts/localization.h, state/stillness.h, state/robot_state_feed.h, impl/localization/{stationary_window, tracking_wheel_motion, imu_heading_increment, brain_imu_wheel_bench}.{h,cpp}, runtime/{localization_stage, system, sensor_loss, brain_profile_builder}.{h,cpp}, inspection/inspection_document.cpp.
- config: shared/pipelines/synthetic_field_demo.xml, synthetic_fusion_demo.xml.
- tests: brain_profile_gtest.cpp, localization_models_gtest.cpp, stationary_window_gtest.cpp, end_to_end_gtest.cpp, parallel_wheels_gtest.cpp, synthetic_rig_gtest.cpp, inspection_document_gtest.cpp.
- docs: pi/navigatr/docs/{brain_profile, usb_localization_bench, vex_imu_bench, calibration_inventory, setup, architecture, localization_fusion, inspection}.md, docs/pi_setup.md.

### New tests (12)
- SensorLoss: AnInvalidVexSampleIsOneRestart (warn mode: one restart event), GapsUnderTheLimitLoseNoTravelWhileRolling (VEX link 220 ms, VEX Pico 220 ms, Pico IMU Pico 220 ms: placed, same epoch, pose = truth), AGapPastTheLimitBetweenChecksEndsContinuity (260 ms gaps the cycle-start check never sees: VEX link, VEX Pico, Pico gyro; unplaced, new epoch, placement recovers), APlacementInTheCycleOfADroppedIntervalIsWithdrawn (and the feed is published at once), TheVexStepLimitIsTheProfileLossLimit (sensor_loss_ms 400: 340 ms measured, 420 ms unplaces; encoder stale_after still ends a 300 ms Pico gap), ADroppedIntervalAfterANewProfileStillCounts, AVexRotationJumpEndsContinuity.
- BrainRecovery.MovingWhileTheGyroBiasCalibratesEndsContinuity (CONTROL 1 then roll; placed during the first calibration then roll; placed during calibration and still: holds, then rolling is measured).
- TrackingWheelMotion: TurningWhileCalibratingCounts, TravelRebasedAwayWhileCalibratingCounts, TheCalibrationTravelSumStartsAtEachCalibration, BiasWindowsAreSampleTimeByDefault (also imu_heading_increment; 0 and -5 refused).
- Extended: CalibrationRebasesWheelsAndRestartsOnMotion (first solve 1010..1015, dropped exactly 1), StationaryWindow gap/movement counters, ConfigurationSchemaIsStrict (Calibration window_ms 0), RecalibrateNeedsStillnessAndHoldsThePose (still localized), SensorLossAndStillnessShowInTheSnapshot (dropped_intervals/why, movements).

### Test results (exact)
- `cmake --build $S/wB-nav -j8 --clean-first`: exit 0, **0 warnings** (MinGW Makefiles, Debug, ucrt64 g++ 15.2, -Wall -Wextra).
- `ctest --test-dir $S/wB-nav -j8`: **523/523 passed** (511 + 12 new).
- `tests/navigatr_tests.exe --gtest_repeat=2`: 523 + 523 passed.
- `BrainProfile*:SensorLoss*:BrainRecovery*:Workers*:StationaryWindow*:GyroBias*:BrainLink*:TrackingWheelMotion*:ImuHeadingIncrement*` x5: 155/155 each time.
- Lead e2e harness (`$S/wB-e2e` -> `$S/wB-e2e-build`, repo brain_link_e2e_gtest.cpp + current Brain sources): **9/9 passed**, 0 warnings.
- Mutation check in `$S/wB-fix-mut` (`mutants.sh`, `survivors.sh`, `mutants.log`, `survivors.log`): **34 mutants, 32 killed, 2 equivalent.**
  - Round 1 killed 28; survivors F24/F26 (travel sum not restarted by recalibrate or a dropped window), F30 (movements not in the status) and F32 (a second restart edge after an invalid VEX sample) are killed by the tests added after it.
  - F3 (no drop baseline at the swap) is equivalent: watchModels resyncs every cycle, and a drop in the first cycle after a swap finds the robot unplaced by the new profile anyway.
  - F15 (bench encoder restart not counted) is equivalent: the same record epoch or discontinuity change is a SensorLossMonitor edge before localization in the same cycle.

### CONTRACT ISSUES / deviations (none blocking)
- The bench's step limit follows `sensor_loss_ms` (one number). Raising sensor_loss_ms therefore lets the VEX bench integrate longer steps (e.g. 400 ms); the user's "no bridging" is read as "never integrate across a gap the model drops", not as "no step longer than 200 ms". Say so if the lead wants a separate cap.
- CONTROL 1 on a two-wheel Pico IMU profile: the pose holds only while the robot stays still until calibration is done (spec 5.4 said "pose holds"; 8.10 decides).
- imu_heading_increment gap drops are not counted (no travel lost with three wheels).

### For the lead
- `docs/brain_setup.md` (yours): with a two-wheel Pico IMU profile, placing during calibration or recalibrating while placed is fine only if the robot stays still until calibration is done; moving earlier gives NeedsPlacement with the event "motion lost: profile_motion: moved while the gyro bias calibrated". The Brain apps should not drive until readiness is Ready. New event prefix "motion lost:" beside "sensor lost:".
- Inspection additions (additive, contract unchanged): `functions[].dropped_intervals`, `dropped_why`, `stillness.movements`.

### Risks
- Nothing ran on hardware. The movement-while-calibrating rule uses the stationary thresholds (1 mm, 1 deg/s about the mean, 5 deg/s): a noisy gyro that restarts the window would unplace a robot placed during calibration. Untested against the real BNO08X/ASM330 noise.
- A Pi estimation stall longer than max_gap_ms makes the two-wheel Pico gyro model drop the stretch (it sees only the newest sample), which now unplaces. Conservative, not silent.
