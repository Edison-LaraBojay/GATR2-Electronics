# Integrated Brain/Pi architecture: contract spec (Phase A)

Lead session 827c2c12 (name gatr2-electronics-14). Written 2026-09-27.

- The verbatim user request is `requirements.md` in this folder. Read it; it is the acceptance test.
- The inventory reports are in `inventory/`. Read the ones for your area before you start.

`$REPO` = `C:/GATR2-Electronics` (Git Bash: `/c/GATR2-Electronics`).
`$S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.

---

## 0. Ground rules (all workers)

**Repository rules**
- Do not commit, push, flash, deploy, install services, or touch `pcb/`.
- Git history is the backup. Everything as of HEAD `c2c5654` is committed. Recover old code with `git show HEAD:<path>`; never `git checkout`, `git reset` or `git stash`.
- Edit only files you own (section 1). If you need something from another owner:
  - write it under `CONTRACT ISSUES` in your progress file `progress/<you>.md` in this folder;
  - choose the least invasive local workaround and continue.
  - Never edit another owner's file.

**Conventions**
- C++17 for host and library code. PROS app code is gnu++20.
- Warnings: `-Wall -Wextra`, and no `-Werror` anywhere.
- Braces on every `if`, `for` and `while`.
- Namespaces: opening brace on its own line and contents unindented; attached braces for everything else. Match the surrounding files and `.clang-format` (LLVM base, indent 4, column limit 100).
- Comments (the user's rules): terse, plain ASCII, no em dashes or smart quotes, noun-phrase labels, not AI-ish, no narration of rationale the code already shows.
  - Docs follow the same rules.
  - No emojis anywhere.
- Tests:
  - files are `*_gtest.cpp`, listed explicitly in CMake (never globbed);
  - a `tests/` folder holds only test sources;
  - no empty placeholder files.
- READMEs are short signposts; deep documentation goes in `docs/`.
- Never invent physical measurements. Unknown robot values are editable placeholders labeled PLACEHOLDER or PROVISIONAL. The tracking wheel radius is 0.024 m, PROVISIONAL.
- Do not claim hardware validation from software tests.

**PROS facts**
- Kernel 4.2.2, arm-none-eabi-g++ 13.3.1, gnu++20.
- `_GLIBCXX_HAS_GTHREADS` is undefined, so `std::shared_ptr` reference counts are **not atomic** across PROS tasks, and `std::mutex`/`std::thread` do not exist.
  - Never share a `shared_ptr` between PROS tasks after start-up.
  - Use `pros::Mutex` with **bounded** `take(timeout)` in background tasks, `std::atomic` for flags, and value copies.
- PROS deletes competition tasks (autonomous, opcontrol) at mode changes without releasing mutexes they hold.
  - Background tasks must never block forever on a lock a competition task can hold.
  - A failed bounded take means a safe action: motors brake, the link skips a cycle, and a counter is incremented.
- `SERCTL_NOBLKWRITE` is ineffective in 4.2.2. USB writes can block while the cable is unplugged, so USB I/O stays in dedicated tasks.

**Builds**
- Each worker uses its own build dirs under `$S`, prefixed with its name (`wA-`, `wB-`, `wC-`, `lead-`). Never build in a directory another worker uses. Never build inside the repo.
- Host recipe (Git Bash):

  ```
  export PATH=/c/msys64/ucrt64/bin:$PATH
  cmake -S <src> -B "$S/<dir>" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug \
        -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$S/nav-base/_deps/googletest-src"
  cmake --build "$S/<dir>" -j8 [--target <yours>]
  ctest --test-dir "$S/<dir>" --output-on-failure -j8 [-R <yours>]
  ```

- `brain/` is one CMake project with three subdirectories. Configure must never fail because of partial work: list a source in CMake in the same step you create it, and never list a missing file. Build only your own targets.
- ARM syntax check for PROS-only sources, from `$REPO`:

  ```
  TC="$HOME/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr"
  "$TC/bin/arm-none-eabi-g++" -std=gnu++20 -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard \
    -mthumb -Os -Wall -Wextra -Wno-psabi -D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING -fsyntax-only \
    -iquote brain/testing/include -Ibrain/testing/include -Ibrain/investiGATR/include \
    -Ibrain/communiGATR/include -Ibrain/actuGATR/include -I. <file.cpp>
  ```

- The PROS app builds (`pros make`) belong to the lead in Phase C.

**Progress file**
- Keep `progress/<you>.md` in this folder current with: done, in progress, CONTRACT ISSUES, test results with exact counts, and open risks.
- It is how the lead integrates your work, so write it as you go, not only at the end.

---

## 1. Ownership

| Owner | Files |
|---|---|
| **Worker A** (protocol + communiGATR) | `common/**` (frames.h, frame_codec.*, link_documents.*, common/tests/**); `brain/communiGATR/**` including its CMakeLists; `docs/communigatr.md`; `docs/interfaces.md` |
| **Worker B** (Pi) | `pi/navigatr/**` (src, tests, config, docs, viewer, tools, both CMakeLists); `docs/navigatr_resources.md`, `docs/navigatr_sensors.md`, `docs/pi_setup.md`, `docs/hardware.md`; the generated `brain/testing/include/field_references.h` (generator output only) |
| **Worker C** (planner) | `brain/investiGATR/**` including its CMakeLists; `docs/investigatr.md` |
| **Lead** | `brain/actuGATR/**`; `brain/CMakeLists.txt`; `brain/gatr2_brain.mk`; `brain/README.md`; `brain/testing/**` (except the generated file); `brain/localization-test/**`; `docs/actugatr.md` (new); `docs/README.md`; root `README.md`; `.github/workflows/ci.yaml`; this handoff folder |

**Contract files and handover**
- The lead wrote the contract files in Phase A: `common/frames.h`, `frame_codec.*`, `link_documents.*` and the `brain/investiGATR` public headers and type sources.
- Ownership of `common/**` passes to Worker A, and of `brain/investiGATR/**` to Worker C.
- Owners may fix bugs and add helpers. They must not change a wire layout, a public name, or a signature other workers use. Such a change goes into CONTRACT ISSUES, and the lead decides.

**Removed and broken code at hand-off**
- The lead has already removed the old investiGATR `Navigator`, `InputSource`/`input.h`, `pid`, `drive`, `OdometrySource` and `sim/`. The lead rebuilds the parts worth keeping in actuGATR.
- Consequences, each fixed by its owner as its first step:
  - `brain/communiGATR` no longer compiles: `Driver` implements the removed `InputSource`, and the client speaks v3. Worker A.
  - `pi/navigatr` no longer compiles:
    - `brain_link_commands`/`brain_link_publisher` use v3 names;
    - `tests/CMakeLists.txt` lists removed Brain sources for `brain_link_e2e_gtest.cpp`.
    - Worker B: drop that test and the `BRAIN_SOURCES` list for now. The lead writes a v4 end-to-end test in Phase C and asks B to add it back.
  - Both PROS apps. Lead, Phase C.
  - `common/tests` expect v3. Worker A.

---

## 2. Coordinates, units, ports

- **Frames (unchanged).**
  - Field frame: origin at the inside bottom-left corner in the audience view, +x right, +y toward the 0-degree wall, heading CCW from +x, wrapped to (-pi, pi].
  - Robot frame: +x forward, +y left, origin the reported robot point.
- **Units.**
  - Internal: SI (m, rad, s).
  - Wire pose: mm, centidegrees, ms.
  - Wire profile: micrometers and millidegrees.
  - Chassis commands: vx forward m/s, vy left m/s, omega CCW rad/s, body frame unless declared otherwise.
- **Encoder port k** (0..2) = Pico encoder channel k = sensor frame bit k. HAT v2 mapping: port 0 is A0/B0 on J2, port 1 is A1/B1 on J3, port 2 is A2/B2 on J4.
- **Pico IMU port 0** = the single external IMU on SPI1/J7. The chip (BNO08X or ASM330) is fixed by the Pico firmware build and cannot be chosen from the Brain. Its yaw rate is sensor frame bit 3. Ports other than 0 do not exist on HAT v2.
- **VEX Smart Port** 1..21 uses V5 numbering. It is used only for the Brain bench IMU source, which the Brain reads itself. The Pi receives samples in GET_STATE.

---

## 3. Brain link v4

The wire contract is written in `common/frames.h`, `frame_codec.h` and `link_documents.h`; read those first.

**Framing and constants**
- The envelope, CRC-16 and frozen header offsets are unchanged. `kBrainLinkVersion = 4`.
- Payload max 122 bytes (frame max 128).
- Chunk sizes:
  - `kProfileChunkMax` = 106 (PROFILE_WRITE data);
  - `kDocChunkMax` = 96 (READ_DOC data);
  - `kPathReportMaxPoints` = 13.
- Retired numbers are never reused:
  - ops 3 (SELECT_LANDMARK) and 5 (GET_STATE_WITH_IMU);
  - results 6 and 7.

**Ops (all requests use the frozen header)**

| op | body | Pi behavior |
|---|---|---|
| 1 HELLO | nonce | as v3 (retry, stale nonce ring of 4, new session). A new session: clears dedupe records and bench IMU mailbox, withdraws an unapplied SET_POSE, keeps localization, anchor, profile and staging |
| 2 SET_POSE | x, y, heading | as v3, except `kResultNotReady` while the Pi config uses a Brain profile and none is applied |
| 4 GET_STATE | bench IMU block (flags, stamp, rotation) | Ok + state block. A sample with `kBenchImuValid` goes to the BenchImu mailbox when one is configured, else it is ignored. Unknown flag bits: InvalidArgument. The same rid as newest: fresh state, sample not re-accepted |
| 6 PROFILE_WRITE | profile_id, total_len, offset, data | stages bytes for profile_id (see 5.1). Ok body: profile_id, received (contiguous bytes held) |
| 7 PROFILE_APPLY | profile_id, total_len | see 5.1. Ok (applied), Pending (applying), ProfileRejected (reason, detail), InvalidArgument (staging incomplete or crc32 != profile_id), NotReady never |
| 8 READ_DOC | doc_kind, doc_id (0 = current), offset, max_len | Ok body: kind, doc_id, total_len, crc32 of the whole document, offset, data (min(max_len, 96, total-offset) bytes). Stale when doc_id is no longer retained. Unavailable when the Pi has no such document (no `<Field>` on the publisher, or no estimate yet). InvalidArgument for unknown kind, max_len 0, or offset >= total_len |
| 9 CONTROL | action, arg | see 5.4. Ok body: action, calibration |
| 10 PATH_REPORT | command_id, mode, count, points (mm) | stores the latest path for inspection only. Ok, header only. mode kPathNone clears |

**Results.** Ok 0, Pending 1, UnknownSession 2, UnsupportedVersion 3, UnsupportedOp 4, InvalidArgument 5, Stale 8, NotReady 9, ProfileRejected 10, Unavailable 11, NotStationary 12.

**Session and dedupe (Pi)**
- `request_id` 0 is InvalidArgument. A newer id is new.
- A not-newer id is never applied again:
  - equal to the last SET_POSE or CONTROL record with the same body: answered as a duplicate of that record, never re-executed;
  - equal to the newest id with a read-only or idempotent op (GET_STATE, READ_DOC, PROFILE_WRITE, PROFILE_APPLY, PATH_REPORT): answered again (writes and applies are idempotent by content);
  - same id with another op or body: InvalidArgument;
  - else Stale.
- Newest request wins per drain. The reply window, the no reply on the first drain, and turnaround are unchanged.

**State block (40 bytes, GET_STATE Ok)**

| Offset | Field |
|---|---|
| 0 | robot_flags |
| 1 | x_mm |
| 5 | y_mm |
| 9 | heading_cdeg |
| 13 | robot_age_ms |
| 15 | odometry_epoch |
| 19 | anchor_revision |
| 23 | health |
| 24 | profile_state |
| 25 | profile_reason |
| 26 | profile_id |
| 30 | map_id |
| 34 | estimate_id |
| 38 | calibration |
| 39 | profile_detail |

- The landmark fields are gone. All field estimates travel as documents.
- health bit 1 `kHealthGyroFresh` means the profile's IMU source (Pico gyro or Brain bench mailbox) is fresh. Encoders fresh covers the profile encoders.

**Bus timing**
- Frames can now reach 128 bytes (11.1 ms at 115200) in either direction. The Brain's per-request response timeout is `response_timeout + byte_time * max(0, request_frame + max_reply_frame - 85)`, where 85 bytes is the v3 budget pair.
- Defaults: response_timeout 60 ms and `byte_time = 10/baud`. The same formula is used for USB, where it only adds margin.
- `docs/interfaces.md` must restate the budget inequality with the new sizes.
- The Pi reply write blocks the estimation worker for up to about 13 ms. That is accepted and documented, since reads happen only when the Brain asks.
- The Brain keeps pose polling responsive with this priority:
  1. HELLO without a session.
  2. Placement.
  3. Control.
  4. Profile sync.
  5. Due state poll.
  6. Map chunk.
  7. Estimate chunk.
  8. Path report.
- Transfers (6-8) are sent only when the state poll is not due.

---

## 4. Documents (`common/link_documents.h`)

**Common rules**
- Layouts are in the header. Integers are little endian.
- Counts are explicit, records are fixed length, no terminators, and there is a format byte.
- `crc32` = CRC-32/ISO-HDLC.
- `validateRobotProfile` is the shared semantic check. The Brain runs it before upload; the Pi runs it before its capability checks. Reason codes are `ProfileReason`.

**Profile**
- profile_id = crc32(document).
- Topologies:

| Topology | Wheels | IMU source | Rules |
|---|---|---|---|
| `kTopologyTwoWheelImu` | 2 | pico or brain_vex | directions independent: \|det\| >= 1e-3 |
| `kTopologyTwoForwardWheelImu` | 2 | pico or brain_vex | angles 0 or +-180 deg only; zero sideways travel is assumed |
| `kTopologyThreeWheel` | 3 | none or pico | the 3x3 [ux uy k] determinant >= 3.2e-5; brain_vex is rejected with ImuCombination because it runs on the Brain clock |

**Field map**
- map_id = crc32(document).
- Objects are sorted by object_id (1..65535).
- kind is landmark or fixed. Flags: obstacle, estimated (landmarks only), reference.
- The box is in the object frame; obstacle ⇔ box size > 0.
- The boundary is a rectangle.

**Field estimate**
- One record per map object, in map order. Records carry source, valid, pose and age_ms.
- The header carries map_id, estimate_id, odometry_epoch and anchor_revision.
- Fixed objects are always nominal and valid. For estimated objects, source none ⇔ not valid.
- age_ms is observation age at the snapshot for observed records and 0 for nominal ones.

---

## 5. Pi behavior (Worker B)

### 5.1 Brain-profiled configurations

A Pi configuration opts in with a `<BrainProfile>` element under `<Localization>`:

```xml
<Localization>
  <BrainProfile>
    <Encoders resource_id="pico_telemetry">
      <Port index="0" output_id="encoder_0"/>
      <Port index="1" output_id="encoder_1"/>
      <Port index="2" output_id="encoder_2"/>
    </Encoders>
    <Imu port="0" resource_id="pico_telemetry" output_id="imu_0"/>
    <BrainImu resource_id="brain_imu"/>
    <CameraSlot index="0" frame_id="front_camera_mount"/>
    <Defaults ... heading constraint bias_samples, history retention, fusion noise, timing .../>
  </BrainProfile>
</Localization>
```

B may refine element names, but it must keep these properties:
- The Pi owns devices, available ports and model tuning.
- The Brain profile owns geometry, topology, IMU source and footprint.
- No robot geometry in the Pi XML for these configs.
- No XML or paths from the Brain.

**Waiting state**
- Until a profile is applied, localization is noop: no pose, `profile_state` none, SET_POSE NotReady.
- Resources, commands, publishing, world estimation (if it has no camera-mount dependency) and inspection keep running. Raw encoder counts stay visible in source health.

**PROFILE_WRITE**
- Staging is keyed by profile_id and bounded to kProfileMaxLen. It survives sessions.
- `total_len` must be in [kProfileHeaderLen, kProfileMaxLen].
- A write at an offset below `received` with identical bytes is Ok (a resend). Different bytes, or an offset above `received`, is InvalidArgument.
- A different profile_id restarts staging.

**PROFILE_APPLY**
- InvalidArgument when staging is incomplete or crc32 != id.
- If id == the applied id: Ok at once, with **no reset** (the idempotent path; this is what keeps the pose across Brain restarts).
- Otherwise:
  1. `decodeRobotProfile`.
  2. `validateRobotProfile`.
  3. Pi capability checks: ports wired in `<Encoders>`, IMU port present, BrainImu configured for brain_vex, camera slots present.
  4. Build the candidate at once, on the command thread, from typed data. Reuse the existing factories by building an internal tinyxml2 tree (ids generated on the Pi; the Brain never sends XML) or typed configs, whichever B finds cleaner. It must not mutate copied models.
- A failure is ProfileRejected with reason and detail, remembered for that id.
- On success the reply is Pending. The swap happens at the next controlled boundary (5.2), after which the state reports applied with profile_id and the same APPLY answers Ok.

**Models per topology and IMU source** (reject anything else explicitly)

| Topology | IMU | Model |
|---|---|---|
| two wheel | pico | `tracking_wheel_motion` + HeadingConstraint + `planar_motion_integrator` (gyro counted once) |
| two wheel | brain_vex | `brain_imu_planar_bench` + `planar_motion_integrator` |
| two forward wheel | pico | `tracking_wheel_motion` + HeadingConstraint + `<LateralMotion assume="zero"/>` + `planar_motion_integrator` |
| two forward wheel | brain_vex | `brain_imu_parallel_bench` (normalize 180-degree wheels to 0 degrees with reversed direction) + `planar_motion_integrator` |
| three wheel | none | `tracking_wheel_motion` + `planar_motion_integrator` |
| three wheel | pico | `tracking_wheel_motion` (no constraint) + `imu_heading_increment` + `weighted_planar_fusion` (independent gyro fused once) |

- Wheel reversal maps to geometric `direction="negative"`. The sensor `invert` stays false.
- CPR maps to the encoder sensor. `kImuInvert` maps to the Pico IMU sensor invert.
- The Health lists and BiasCal follow the profile.

### 5.2 Controlled boundary and continuity

**Boundary**
- `System` gains an explicit apply step, called by the main thread in worker mode and at the top of `step()` in inline mode. It follows the `reset()` pattern: stop workers, move in the candidate executors and slots, restart.
- The brain link Commands slot, its session and pi_instance, the resources (serial links, pico telemetry, cameras) and inspection survive.
- Inspection accessors touched by the swap must be race free (mutex or an immutable binding snapshot).

**Continuity rules**
- A new profile (a different id) means continuity is lost:
  - `odometry_epoch` advances past the previous value (never regresses on the wire);
  - history clears and the robot is unplaced (`kRobotLocalized` clear);
  - any SET_POSE request from before the apply is withdrawn and never re-applied to the new odometry origin. This is the `requestsFrom` hazard in inventory/pi-localization.md 6.2.4.
  - A new SET_POSE is required.
- The same profile is idempotent: nothing resets.
- A Pi restart starts unplaced with no profile, and pi_instance changes.
- A Pico restart keeps today's source-epoch and rebase behavior.

**Rejection**
- A profile that fails at the boundary is reported with `kProfileReasonBuild`, and the previous profile keeps running.
- Pi configs without `<BrainProfile>` answer PROFILE_APPLY with ProfileRejected / `kProfileReasonNotAccepted` and keep their XML localization.

### 5.3 Field documents

**Field XML** (in `config/override/field.xml` and the `FieldMap` parser)
- Add explicit collision semantics. Visual data never becomes behavior implicitly.
- Root `revision="N"` (a human-bumped integer).
- `<Boundary min_x_m min_y_m max_x_m max_y_m/>` (required for publishing a map).
- `wire_id="N"` on every `Landmark` and every new `<Obstacle>` (unique, 1..65535; the stable semantic object id, not the AprilTag id).
- At most one `<CollisionBox x_m y_m size_x_m size_y_m yaw_deg note/>` per Landmark (landmark frame) and per `<Obstacle id wire_id x_m y_m heading_deg>` (fixed element frame). An element with no box is not an obstacle.
- `parseFieldMap` rejects unknown children and attributes where new elements are added.
- The toggles and loaders become fixed obstacles; tape is not.
- The goal box is the maximum horizontal protrusion: the base octagon's circumscribed square, 0.1542 m, orientation independent, with a note.
- Every value stays provisional with a note of its source.
- Tests: boundary equals Dimensions inside; boxes inside the boundary; each goal box contains the Visual base; unique wire ids.

**Map document**
- Built once from the parsed FieldMap: landmarks are kind landmark, flags estimated|reference|obstacle-if-box; obstacles are kind fixed, flags obstacle.
- map_id = crc32. Served by READ_DOC kind 1.
- Configured by `<Publishing type="brain_link"><Field resource_id="..." estimate_period_ms="200"/>`.

**Estimate document**
- Built in the reporting cycle from the newest FieldSnapshot, the robot state and the map.
  - Observed only when its odometry_epoch matches, composed with the current anchor, as the v3 single-landmark code did.
  - Otherwise nominal.
  - Landmarks absent from world estimation (noop) are nominal.
- A new document (estimate_id + 1) is taken when content changes (source, validity, pose at mm/cdeg resolution, epoch or anchor) and at least `estimate_period_ms` has passed.
- The last 3 documents are retained for chunked reads.

**Generator**
- A C++ tool target `navigatr_field_refs` (pi/navigatr/tools or src/app) reads a field XML with the real parser, builds the map document, and writes a C++ header.
- Run it on `config/override/field.xml` to produce `brain/testing/include/field_references.h`:

  ```cpp
  // Generated by navigatr_field_refs from pi/navigatr/config/override/field.xml. Do not edit.
  #pragma once
  #include "investigatr/reference.h"
  struct Field : investigatr::FieldReferences {
      static constexpr investigatr::MapId kMapId    = 0x........;
      static constexpr uint16_t           kRevision = N;
      static constexpr investigatr::Reference NeutralGoal0Center = investigatr::Reference::object(W, kMapId);
      ... one per reference object, CamelCase of the id ...
  };
  ```

- A navigatr test regenerates the header in memory and requires it to equal the checked-in file byte for byte, so the map id cannot drift from the field.

### 5.4 Control, calibration, stationarity

**Recalibrate**
- NotReady without an applied profile.
- The Pi requires the robot stationary: every profile encoder moved less than 1 mm of wheel travel and, with a Pico IMU, |yaw rate| < 2 deg/s over the last 300 ms. Otherwise NotStationary and nothing starts.
- Then it restarts IMU bias calibration in the active model (HeadingConstraint or imu_heading_increment). This needs a small public hook on the observation functions.
- The pose holds while calibrating, and calibration restarts on motion (existing behavior).
- With brain_vex there is no Pi calibration: Ok with calibration none. The Brain resets its VEX IMU itself.

**Reinitialize**
- The same stationary check.
- Then a localization reset: epoch + 1, unplaced, SET_POSE requests withdrawn, bias recalibration.

**Reporting**
- The CONTROL record is deduplicated like SET_POSE.
- `calibration` in the state block is none, running or done.

### 5.5 Inspection and viewer
- snapshot:
  - a brain link section with session, pi_instance, profile state/id/reason/detail and a human summary (topology, wheels with ports, IMU source and port, VEX port, footprint);
  - map_id, estimate_id and calibration state;
  - the last path report (command, mode, points).
- hello: field boundary, collision boxes (landmark and fixed, with wire ids), map_id and revision.
- viewer:
  - draw the boundary and boxes as wireframes;
  - draw landmark boxes at the estimate when observed, else nominal;
  - use the profile footprint for the robot when one is applied, else the configured RobotBody;
  - draw the path polyline;
  - show a readiness line: link, profile, map, calibration, placement.
- Localization-only operation with no camera and no world estimation must keep working.
- Keep `docs/inspection.md` in step with the document.

### 5.6 Configs and docs
- New runnable configs:
  - `config/override/brain_profile_usb.xml`: `pros_usb_link`, Pico telemetry with all 3 encoder outputs plus IMU port 0, BenchImu, noop world estimation.
  - `brain_profile_rs485.xml`: `linux_serial_link /dev/ttyAMA5`, DE GPIO as in the current RS-485 bench.
  - A camera variant only if a camera mount path is implemented and tested.
- Keep the existing XML-configured profiles runnable (the bench_vex_imu*, parallel_wheels*, three_wheel templates, synthetic demos). Their docs must say the Brain-profile configs are the primary path. Remove only what is genuinely obsolete (the `<FieldObject>` landmark selection mapping).
- Update `pi/navigatr/docs/*` (architecture, configuration, landmarks, inspection, setup, usb_localization_bench, vex_imu_bench, calibration_inventory, and a new `brain_profile.md`), `docs/navigatr_resources.md` (add `pros_usb_link`), and `docs/pi_setup.md` (service install for the new configs).
  - Transport exception: USB vs RS-485 still needs the matching Pi config. Say so plainly.
- Review finding F: `serial_port.cpp`. Clear CRTSCTS after cfmakeraw, and bound the tcdrain fallback while DE is held (TIOCOUTQ polling to the deadline).

---

## 6. Brain libraries

### 6.1 investiGATR (Worker C): planner only, plus the shared types

**Shared types**
- The types exist and are implemented in `investigatr_types`: geometry, field, state_source, reference (with `resolve`), motion_model, path.
- Do not change their meaning. Adding tests for them is welcome.

**Worker C implements**
- `GeometricPlanner`, `toString(PlanStatus)` and `footprintClearance`, in a new `investigatr` library target (linking `investigatr_types`), with tests.

**Method**
- Configuration space:
  - obstacles = boxes of `obstacle` objects, at their current estimate pose (or nominal when not valid), grown by R = `enclosingRadius(model)` on every side (a conservative superset of the true Minkowski sum);
  - free region = boundary shrunk by R.
- A circle of radius R about the origin covers the footprint at every heading, so turns in place and heading changes along translations are collision free wherever the origin is free.
- Graph:
  - vertices: start, goal and the inflated-box corners pushed out by `vertex_margin`, dropping corners that are outside the free region or strictly inside another inflated box;
  - edges: segments not crossing the interior of any inflated box.
  - Dijkstra or A* on length. Bounded by `max_vertices`.
- Explain in docs why this suits a small field with tens of boxes: exact, bounded, deterministic, few waypoints, and a good fit for turn-drive-turn execution. State its limit (the circle is conservative for long thin robots; narrow gaps may be refused).

**Checks and results**
- Initial pose: out of bounds or inside an inflated box gives kStartOutOfBounds or kStartBlocked, with `blocking` set.
- Exact goal: same, with kGoalOutOfBounds or kGoalBlocked.
- Every segment, and the turns at the start, the vertices and the goal heading.
- No path: kNoPath. Never fall back to direct.

**Direct mode**
- No field needed and no checks.
- Non-holonomic:
  - turn to face the goal (or away from it when reverse is allowed and shorter), translate, turn to the goal heading;
  - drop turns below `min_turn` and translations below `min_segment`.
- Holonomic: one translate segment with the heading moving to the goal heading.

**Avoiding mode, the same shaping per segment**
- Non-holonomic: turn, translate, turn, ..., final turn.
- Holonomic: translate segments; the heading moves linearly over the whole path length to the goal heading.

**Model rules**
- `model.holonomic == false && !model.turn_in_place`, or `min_turn_radius > 0`: kUnsupportedModel.
- Planning never names drivetrains.

**`clear(path, field, model)`**
- The same inflated-box test over every segment. The follower (lead) calls it with the remaining path from the current pose whenever a new field generation arrives.

**`footprintClearance`**
- Exact rotated rectangle versus rotated rectangle (separating axis) plus bounds. Report the signed distance and the nearest object.
- The lead's executed-trajectory tests use it.

**Tests (at least)**
- Holonomic and non-holonomic feasibility, direct vs avoiding, reverse choice.
- Rotated boxes, box offsets, boundaries, blocked start and goal (with `blocking`), no route.
- A narrow gap refused for a wide model and passed for a narrow one.
- Every planned vertex and segment clear by at least R; randomized fields (fixed seeds) where every returned path passes `clear` and every sampled pose along it passes `footprintClearance`.
- `max_vertices` bound respected.
- Field corrections: moving a box onto the path makes `clear` false.

**Docs and README**
- Rewrite `docs/investigatr.md` for the planner and the shared types. Motion and following go to `docs/actugatr.md`, owned by the lead, so link there.
- Update `brain/investiGATR/README.md`.

### 6.2 communiGATR (Worker A)

**Portable core (host tested)**

*Client (v4)*
- Keeps the v3 session, correlation, request id and nonce rules. SELECT and op 5 are removed.
- Every GET_STATE carries the bench IMU block from `ClientConfig::bench_imu` when set, else flags 0.
- The per-request timeout formula is in section 3.
- **Profile sync.**
  - With a profile document configured, after the first state of a session:
    - if state.profile_id != mine, or its state is not applied, then write chunks from offset 0 (resume at `received` within a session), then APPLY;
    - on Pending, re-send APPLY every `pending_retry`.
  - Rejected is settled until the session changes or `resubmitProfile()`, with no retry storm.
  - Status: profile state, id, reason, detail.
- **Map sync.**
  - When state.map_id != 0 and not equal to the cached complete map id, read kind 1 chunks.
  - Assemble into a bounded buffer: total_len <= kFieldMapMaxLen, offsets contiguous, doc_id constant, and total_len and crc constant across chunks.
  - Verify crc32 == map_id and `validateFieldMap`.
  - The cache survives sessions and Pi restarts, keyed by map_id.
- **Estimate sync.**
  - When the map is complete, state.estimate_id != the assembled id, and `field_period` (default 0.5 s) has passed since the last assembly start (or a read is in progress), read kind 2.
  - Stale restarts from the newest id, with a backoff after 3 in a row.
  - Validate with `validateFieldEstimate` against the cached map.
  - Publish map plus estimate as one generation only when both are complete and consistent. **Never expose a partial document.**
- **Placement.**
  - `submitPlacement` returns 0 (refused) unless the session is ready and, with a profile configured, the profile is applied.
  - NotReady results settle as kRejected with result NotReady.
  - Everything else as v3, including the fix for review finding B (no state poll between a timed-out SET_POSE and its same-rid retry).
- **Control.** `recalibrate()` and `reinitialize()` return tickets, with statuses pending, ok, not stationary, not ready, rejected, timed out or session lost.
- **Path report.** `reportPath(command, mode, points)` is best effort: latest wins, one attempt, and it is dropped without a session. Points beyond 13 are decimated to 13, keeping the first and last.

*Robot profile builder*
- A `communigatr::RobotProfile` in SI units (wheels with port, radius m, counts/rev, x/y m, angle rad, reversed; topology; IMU source, port and VEX port; footprint; cameras) converts to `gatr2::RobotProfileDoc`.
- Run `validateRobotProfile` before upload and report a local rejection with the same reason codes.

*LinkDriver : investigatr::StateSource*
- `robot(now)`:
  - status derivation: kNoLink when not connected; kNoProfile when a profile is configured and not applied; kCalibrating when calibration is running; kUnplaced when not localized, or when the anchor is only configured and `accept_configured_anchor` is false; kNoPose when the pose is not valid or the age is unknown; else kValid.
  - age as v3: `robot_age_ms/1000 + round_trip + (now - received_at)`.
  - frame generation from (pi_instance, session, odometry_epoch, anchor_revision).
- `field(out)`:
  - a complete `investigatr::Field` in SI units (mm to m, cdeg to rad wrapped);
  - generation increments per published map-plus-estimate;
  - `frame` is the generation of the estimate's (pi_instance, session, epoch, anchor);
  - `received_at` is when the estimate completed.
- Also implements `investigatr::PathSink`, forwarding to `reportPath`.

**Transports**
- `BytePort` stays the seam.
- The NG1 USB line codec is portable (`src/`): encode a frame as `NG1:` + uppercase hex + `\n`; decode lines with the marker anywhere, even length, at most 256 digits, uppercase only, and whole-line drops. It must match the Pi's `pros_usb_link` parser.
- Host tests run the real Client over the USB line codec and over the half-duplex FakeBus against the v4 FakePi.

**PROS layer (`pros/`)**
- `ProsSerialPort`: review finding A. Retry `serial_set_baudrate` and `serial_flush` with `pros::delay(2)` for up to about 100 ms after `serial_enable`, and make `open()` retryable.
- `ProsUsbPort`: moved from `brain/localization-test/src/usb_bench_driver.cpp`. It keeps its tasks, its `/ser/ngtr` + `SERCTL_DISABLE_COBS` + stdin handling, its queues and tx lifetime. Nothing in it may block the poll task.
- `ProsLink`: one class for both transports, selected by `LinkConfig::transport` (`kUsb` or `kSmartPort` with port and baud). It owns the port, Client, LinkDriver and poll task.
  - Every public call uses a bounded `take`.
  - It implements StateSource and PathSink.
  - API: `start()`, `static now()`, `status()` (started, transport, connected, ready, session, pi_instance, error, peer version, profile, map id and complete flag, estimate id and age, calibration, health, localized, stats), `place(pose)`, `placement(ticket)`, `recalibrate()`, `reinitialize()`, `control(ticket)`, `resubmitProfile()`.
- `ProsVexImu`: the bench IMU source from `readBenchImu` in localization-test (sign conversion to CCW millidegrees, invalid while calibrating or erroring), plus `recalibrate()` (`reset(false)`) and `calibrating()`.

**Fakes and tests**
- A v4 FakePi (profile staging and apply with rejection, NotReady, documents with a configurable object count including more than one chunk, estimate replacement and Stale, control with NotStationary, restart, faults) and a LinkRig.
- The tests the request lists for communiGATR:
  - different landmark counts, chunked transfer, malformed counts, interrupted snapshots;
  - map/profile mismatch and idempotent retries;
  - Brain and Pi restarts, cable reconnect (the port closed then reopened), stale data, frame changes, explicit reinitialize;
  - real USB and RS-485 codec paths with fake transports.

**Docs**
- Rewrite `docs/communigatr.md`.
- Update `docs/interfaces.md` for v4: ops, bodies, state block, documents, sessions, the timing budget with 128-byte frames, and the USB NG1 envelope.
- Update `brain/communiGATR/README.md`.

### 6.3 actuGATR (lead)

actuGATR is new. The chassis command is `vx` forward m/s, `vy` left m/s and `omega` CCW rad/s, body frame. It contains:
- tank and mecanum kinematics with coupled wheel-speed desaturation;
- a drive stage that refuses invalid or stale commands and any vy on a tank;
- motor I/O with V5 internal velocity control (`move_velocity` rpm through gearing);
- differential and holonomic segment followers;
- the unified `Motion` API (`goToDirect`, `goToAvoiding`, `Reference`);
- the drive owner (manual/navigate/disabled);
- port conflict checks and PROS adapters;
- tank/mecanum sims and executed-trajectory clearance tests.

`Motion` consumes `investigatr::StateSource`, `investigatr::PathPlanner` and `investigatr::PathSink` only.

---

## 7. Phase C integration (lead, serial)

1. Codec and transport behavior, and bounded transfers: A's tests, reviewed.
2. Brain profile, then Pi localization, then Brain pose and map. Lead writes a v4 `brain_link_e2e_gtest.cpp` in pi/navigatr/tests linking the real communiGATR portable core against the real Pi System over memory links (B adds the CMake entries).
3. Reconnect, profile replay, reset, calibration and interrupted sync, end to end.
4. Planner, then follower, then drivetrain sim, then the testing app logic.
5. Localization-only bench and 3D viewer regression.

Then the PROS builds of both apps, the full host gate, independent reviews, and docs.

---

## 8. Lead amendments (binding; read even if your stage started earlier)

### 8.1 (Worker C, stage C2) Start escape and goal approach near walls and landmarks

**Problem.** VEX robots start touching the field wall. Under the circle model the origin is then within R of the boundary, so avoiding mode would refuse the first command of every match (kStartOutOfBounds). Goals next to a landmark or wall hit the same limit.

**Required behavior in `GeometricPlanner`**
- **Start escape.** When the start fails the circle test but the exact footprint at the start pose is clear:
  - try straight escapes along the start heading: forward, then backward when reverse is allowed and the model allows it (a holonomic model may also try its left and right body axes);
  - use the shortest escape of length <= 2R that reaches a point passing the circle test;
  - the whole swept exact rectangle (footprint grown by `model.clearance`) must be clear of every box and inside the bounds;
  - the escape becomes the first translate segment (no turn before it), and planning continues from its end.
- **Goal approach.** Mirrored at the goal: when the goal fails the circle test but the exact footprint at the goal pose is clear, find a circle-free point along the goal heading (behind or in front of it) whose swept exact approach to the goal is clear. Plan to that point, then add the straight approach segment (and, for non-holonomic models, the approach needs no turn at the goal because it runs along the goal heading).
- Report kStartBlocked/kStartOutOfBounds or kGoalBlocked/kGoalOutOfBounds only when no escape or approach exists, or when the exact footprint itself collides. Keep `blocking` meaningful.
- **Segment flag.** Mark these segments with a new additive field in `investigatr::PathSegment`:

  ```cpp
  bool exact = false; // swept exact footprint checked, not the circle (escape, approach)
  ```

  `clear()` must check `exact` segments with the exact swept rectangle, and the rest with the circle. Otherwise the lead's Motion revalidation, which calls `clear()` on the remaining path from the current pose, would replan forever next to a wall.
  - This is the only change allowed to the shared type headers. The lead's actuGATR needs no change for it.
- **Tests:**
  - start against each of the four walls, facing away and facing along the wall;
  - the goal approach beside a landmark box;
  - an escape blocked by a second box (kStartBlocked);
  - `clear()` true on a path that begins with an escape segment;
  - randomized tests still pass with exact-footprint checks along escape segments.
- **Docs:** describe the escape and the approach and their limits in docs/investigatr.md.

### 8.2 Addition: calibration, per-wheel tuning, and recovery (user request 2, verbatim in requirements_2.md)

The lead froze these contract changes; they are already on disk. Owners implement against them.

**common (Worker A owns again)**
- frames.h:
  - sensor frame v2 (type 0x04: boot_id, acq_epoch, imu_epoch); v1 (0x01) is still decoded;
  - Pico link command 0x12 and status 0x13 frames, with PicoOp/PicoCommandStatus/PicoCommandDetail/PicoImuState/PicoImuReason/PicoFirmware;
  - brain op READ_WHEELS 11 with WheelReading;
  - kResultFailed 13;
  - health bits 4..7 (PicoLink, ImuInitializing, ImuFailed, Stationary);
  - CalibrationState 3..5 (WaitingStill, WaitingData, Failed);
  - ControlAction 3 ReinitImu and 4 RestartAcquisition, and ControlDetail;
  - ProfileReason 14 Calibration.
- frame_codec:
  - CONTROL replies Ok, Pending and Failed carry action, calibration and detail (3 bytes);
  - READ_WHEELS Ok is count plus 16-byte records;
  - encode/decode for sensor v2 and the Pico command/status frames;
  - FrameReader accepts the new types.
- link_documents: profile header 32 bytes and wheel record 32 bytes.
  - Header adds calibration_window_ms, still_rate_cdps and still_travel_um.
  - Wheel record adds gear_micro and travel_scale_ppm.
  - kWheelReversed now means **encoder polarity**.
  - Validation: gear 0.1..10, travel scale 0.9..1.1, calibration settings (0 = Pi default) in range.
- Worker A must update common/tests for all of this (known-byte vectors, lengths, FrameReader, profile layout, new ranges) and keep them exhaustive as in A1.

**One application point per correction (Pi)**
- Acquisition (`pico_telemetry`): raw counts, identity.
- Encoder sensor conversion: wheel angle = counts * 2pi / (counts_per_rev * gear), sign by polarity (sensor invert = kWheelReversed).
- Observation model: travel = delta wheel angle * radius * travel_scale, along the measuring angle. The geometric `direction` is always positive for profile-built configs, which **replaces the spec 5.1 mapping** of reversal to direction.
- `wheel_geometry` gains an optional `travel_scale` (default 1), consumed by tracking_wheel_motion and both bench models.
- IMU: sign once in the sensor, bias once in the observation function. The VEX rotation arrives CCW and already calibrated by VEX firmware; the Pi applies no bias to it.

### 8.3 Pi (Worker B, stages B3/B4)

**StationaryWindow**
- A single shared helper, `StationaryWindow` (for example in impl/localization), used by every IMU bias path: HeadingConstraint in tracking_wheel_motion, imu_heading_increment, and the bench models for the stationary status.
- Evidence counts only when a sample's sequence advanced; repeated or retained records never count.
- Every required source must keep fresh, continuous samples: the profile encoders, plus the gyro or the VEX rotation. A gap longer than max_gap (default 100 ms) or any epoch/discontinuity change restarts the window with state WaitingData.
- Thresholds over the window:
  - per-wheel travel <= still_travel (default 1000 um);
  - gyro |rate - window mean| <= still_rate (default 1.0 deg/s) and |rate| <= 5 deg/s;
  - VEX rotation change <= still_rate * window.
- Movement restarts the window (WaitingStill).
- The window qualifies by elapsed sample time >= calibration_window (default 2 s) with at least 20 samples per source. This is time, not an arbitrary sample count.
- Calibration is bounded: no qualified window within 60 s gives Failed, and CONTROL recalibrate retries. Nothing blocks acquisition or communication.

**Bias**
- Bias comes from a qualified window, as accumulated angle over elapsed time.
- Gated maintenance: every later qualified window moves the bias by a bounded step (weight 0.2, at most 0.2 deg/s per step), and stops the moment evidence is unsuitable.
- An IMU restart or epoch change, a Pico reboot, or a sensor restart **invalidates the calibration** and restarts it (this fixes calibration being kept after a source restart). The pose holds.

**Stationary reporting**
- While qualified stationary, the estimator reports zero velocity and `kHealthStationary`. Placement is kept and no position change is fabricated.
- Document that this is gated stationary handling, not a ZUPT filter: no estimator here models velocity with uncertainty.

**VEX profiles**
- Reduced-evidence stationary detection from the wheels plus the VEX rotation.
- Calibration state None; the Brain owns the VEX calibration (`pros::Imu::reset`).
- An absent or broken BNO08X never blocks a VEX profile from becoming ready.

**READ_WHEELS**
- One record per profile wheel in profile order: port, flags (fresh, valid), discontinuity (the encoder discontinuity epoch, low 16 bits), counts (raw), travel_um (the sensor's continuous wheel angle times radius, no travel scale), age.
- NotReady without an applied profile; Unavailable for configs without `<BrainProfile>`.

**CONTROL (record deduplicated like SET_POSE)**

| Action | Behavior |
|---|---|
| 1 Recalibrate | stationary precheck (NotStationary if moving now), then restart the windowed calibration; Ok |
| 2 Reinitialize | as before |
| 3 ReinitImu | needs a Pico IMU profile and v2 Pico firmware, else Failed with detail ImuUnused or PicoLink. Submit PicoOpReinitImu through `PicoControl`; Pending while the Pico reports it running; Ok once completed and recalibration restarted; Failed with detail on Pico failure or the 15 s bound |
| 4 RestartAcquisition | stationary precheck; submit PicoOpRestartAcquisition; Pending until the new acq_epoch appears in sensor frames; Ok. Localization rebases and the pose holds |

- A duplicate of the Brain's CONTROL request reports current progress and never resubmits to the Pico.

**Pico configuration and health**
- On profile apply, send PicoOpConfigure with imu_enabled = (IMU source is pico) when a Pico v2 link exists.
- Health bits PicoLink, ImuInitializing and ImuFailed come from `PicoControl::link()`.
- The resource id is given by `<Pico resource_id="pico_telemetry"/>` on the brain_link CommandCollection and Publishing, optional. The contract header is `src/resources/pico_control.h`, written by the lead and implemented by Worker D.

**Inspection**
- Calibration state, reason, window progress, bias, attempts and the stationary flag.
- Per-wheel readings with the active CPR, gear, radius and travel scale.
- Pico link status: boot, epochs, IMU state, reason, attempts, last command.
- A bounded recovery event log: link lost/restored, Pico reboot, acquisition restart, IMU reinit, profile applied, placement withdrawn, calibration restarts.

**Tests**
- Stationary window: freshness, duplicate samples, gaps, discontinuity, moving then stopping then moving, time-based qualification, and bounded failure.
- Bias invalidated by a restart; gated maintenance.
- Independent perpendicular-wheel travel with scale and gear applied once, with no double application (polarity, CPR, gear and scale each checked).
- READ_WHEELS and CONTROL 3/4 against a fake PicoControl.
- VEX profile ready with no Pico IMU frames at all.
- The existing three-wheel fusion and external IMU tests stay green.

### 8.4 Worker D (new, Pico and Pi Pico link)

**Owns**
- `pico/**`.
- `pi/navigatr/src/impl/resources/pico_telemetry.{h,cpp}`, `serial_links.{h,cpp}`, `src/payloads/pico_telemetry_samples.h`, `src/resources/pico_control.h` (after the lead), new `tests/pico_link_gtest.cpp`, new `pi/navigatr/docs/pico_link.md`.
- A single-line Edit insertion (never Write) in `pi/navigatr/tests/CMakeLists.txt`, and in `pi/navigatr/CMakeLists.txt` if a source is added.

**Pico firmware**
- Identity and status:
  - v2 sensor frames: random nonzero boot_id per boot, acq_epoch, imu_epoch, frame <= 32 bytes;
  - status frames every 200 ms and on every change, only while the TX FIFO is empty and never delaying a due sensor frame.
- Commands:
  - non-blocking UART RX command parsing;
  - record duplicates (the last 4 request ids with op and body), check the target boot;
  - CONFIGURE (imu_enabled), REINIT_IMU (imu_epoch + 1, Running until ready, Completed/Failed), RESTART_ACQUISITION (zero the counters atomically against the encoder ISRs, acq_epoch + 1, Completed).
- IMU retry and blocking:
  - local IMU retry while enabled: 5 quick attempts with 0.5 s backoff doubling to 8 s, then Failed with slow retries every 30 s;
  - fix the permanent failures: BNO08X `sh2_open` failure, and ASM330 WHO_AM_I failure;
  - never block the loop beyond the existing bounded bring-up sections.
- Host tests: portable command/state and retry logic, split into testable units, with tests.
- Build `pio run -e hat2_bno08x -e hat2_asm330` (build only, no upload).

**Pi**
- `pico_telemetry` decoding and identity:
  - decode v2 sensor frames and status frames, and keep decoding v1 (identity unknown, stamp-regression fallback, flagged in diagnostics);
  - a boot_id or acq_epoch change bumps the device epoch, so encoders rebase with no displacement;
  - an imu_epoch change affects only the IMU output (gyro accumulator epoch plus the per-output upstream epoch).
- Implement `PicoControl`: request ids from a random base per Pi process, resend every 200 ms until the status reports the id, bounded by timeout, completion only from reported status; `link()` is thread safe.
- `linux_serial_link`: when the device fails to open or closes, retry the open at a bounded rate (1 s) without blocking the loop, and keep `required` for startup semantics.
- `tests/pico_link_gtest.cpp`:
  - identity changes produce no false displacement through the real telemetry, sensor and localization path;
  - an IMU-only restart keeps the encoders continuous;
  - v1 fallback;
  - command resend and duplicate handling against a fake Pico byte peer;
  - the serial reopen.

### 8.5 Brain (Worker A, stages A3/A4)

**Client**
- `requestWheels()` gives a ticket for a READ_WHEELS read, scheduled after control. The latest result carries its receive time.
- CONTROL tickets for all four actions. A Pending reply resends the **same** request id every `pending_retry` until a final result, bounded by `control_deadline` (default 20 s), then kTimedOut. Never a new id for the same operation.

**RobotProfile builder**
- Per wheel: counts per revolution, polarity, gear, radius, position, measuring angle, travel scale.
- IMU calibration settings.

**Readiness**
- A central readiness summary in the portable core, used by both apps: Connecting, Reconnecting, ProfilePending, ProfileRejected, SensorsUnavailable, SensorsInitializing, WaitingStill, Calibrating, NeedsPlacement, Ready.
- Also the decoded health, calibration and Pico bits.
- Update the fakes and tests: READ_WHEELS, CONTROL Pending to Ok/Failed with lost replies (never re-executed), and readiness transitions.

### 8.6 Lead
- WheelCalibration helper (portable, host tested): trials from READ_WHEELS snapshots, rejection rules, proposal and spread. Added to communiGATR after Worker A's pipeline.
- localization-test calibration and recovery UI, the shared robot config, docs.
- Runtime profile change (8.5 addition): `ProsLink::setProfile(const RobotProfile&)` (and the matching Client/LinkDriver call) replaces the configured profile, validates it with the same shared rules, and starts profile sync with the new id; used when localization-test applies a wheel calibration. The Pi then applies it as a new profile (new odometry epoch, unplaced); the app offers an explicit placement at the last pose, never an automatic one. `profile()` returns the active configured profile for display.

### 8.7 Lead decisions after the session restart (binding)

**Contract state**
- The common contract for the calibration and recovery addition is **frozen**. Worker A updates common/tests for it (A's CONTRACT ISSUE 3).
- `PicoTelemetry` now publicly implements `navigatr::PicoControl`. A resource holds one contract, so consumers `require<PicoTelemetry>(id)` and use it only through the `PicoControl` interface.
  - The lead added a minimal implementation: `link()` gives freshness only, `submit()` returns 0, and `request()` returns kUnknown.
  - Worker D replaces it with the real one. Worker B codes against the interface and tests with a fake PicoControl.

**Accepted as implemented**
- A's transfer starvation guard (a waiting transfer goes after 4 due polls).
- A's estimate age bound (`snapshot_after`; infinite age for an estimate that predates the session). The Pi keeps taking new estimate documents only on content change.
- C's swept clearance near the path's start and goal poses.
- B's goal box of 0.1543 m and its wire rounding (boxes round up, the boundary rounds inward).
- B's estimate_id restarting per Pi process; the Brain keys by (pi_instance, estimate_id).

**Already fixed**
- `Motion::remaining()` now checks the remaining planned segments unmodified. C's note is out of date.

**Lead-owned files inside communiGATR**
- `include/communigatr/wheel_calibration.h`, `src/wheel_calibration.cpp`, `tests/wheel_calibration_gtest.cpp`.
- `include/communigatr/startup_placement.h`, `src/startup_placement.cpp`, `tests/startup_placement_gtest.cpp`.
- Worker A lists them in communiGATR's CMakeLists (library and test target) and does not edit them.

**Stage restructuring after the restart**
- Worker B stages:
  - B3: profile application, including the 8.2 correction mapping.
  - B4: calibration and recovery (8.3).
  - B5: configs, inspection and viewer, docs.
- Worker D starts after Worker C's review and fix, so at most three workers run at once.


### 8.8 Lead: sensor gaps must not lose wheel travel silently (Worker B, stage B4; binding, NARROWED by the user 2026-09-27 23:00)

**Finding** (lead e2e test `pi/navigatr/tests/brain_link_e2e_gtest.cpp`, `PulledCableStopsTheMoveAndTheLinkResumes`):
- Setup: brain_profile_usb.xml, two perpendicular wheels, Brain VEX IMU.
- A 1 s USB outage during a 0.6 m/s move lets the robot roll about 0.18 m.
- `brain_imu_wheel_bench` sees the VEX IMU stale, rebaselines, and drops that travel.
- The state still says localized and pose valid, so the Brain shows Ready with a pose 0.18 m off.

**Worker B: the agent that edited tracking_wheel_motion at 22:59-23:00 was an accidental duplicate of you and is stopped. See LEAD NOTE at the end of progress/wB.md.**

**User decision: detect, do not compensate.** A sensor that fails mid-run is a wiring or hardware fault; no bridging or gap integration.

**Required (bench model only)**
1. When `brain_imu_wheel_bench` rebaselines after a gap or an identity change, compare the profile wheels' raw travel across the gap. The gap is the last used sample to the first fresh one; use the wheel angles it already keeps. If any wheel moved more than the profile `still_travel` (Pi default when 0), pose continuity is lost:
   - clear `kRobotLocalized` (the Brain sees NeedsPlacement);
   - advance `odometry_epoch` per the 5.2 continuity rules, and withdraw pending SET_POSE;
   - record a RuntimeEvent / inspection note ("travel lost during a sensor gap: place again").
   - If nothing moved, today's quiet rebaseline stays.
2. No change to the Pico IMU and three-wheel models, and no `bridge_max_ms`.
3. **READ_WHEELS** must answer Ok with the profile wheels once a profile is applied (8.3). The e2e test `WheelReadingsReportRawTravel` checks raw travel (CPR, gear, polarity, radius applied; travel_scale not applied) to 1e-4 m against the truth. Today it answers NotReady (9).

**Tests**
- Worker B: model tests for gap-with-travel (unplaced, new epoch, SET_POSE withdrawn) and gap-still (quiet rebaseline, still placed).
- The lead's e2e tests `PulledCableStopsTheMoveAndTheLinkResumes` (now expects NeedsPlacement after the reconnect) and `WheelReadingsReportRawTravel` must pass.
  - To build them, copy the lead harness `$S/lead-e2e/CMakeLists.txt` (the test plus the brain sources, linked to navigatr_core) into your own dir, e.g. `$S/wB-e2e`, and configure into `$S/wB-e2e-build`.
- The lead adds the e2e CMake target (`brain_link_e2e_tests`) after B's pipeline ends. **B: do not add it yourself.**

### 8.9 User scope cuts (2026-09-27 23:15; binding for all workers and reviewers)
- **No Pico CONFIGURE on profile apply.** The Pico IMU stays enabled from boot and keeps retrying when absent or failed, which is harmless.
  - A Brain VEX IMU or no-IMU profile simply does not read it. Readiness and health for such a profile ignore the Pico IMU, as A implemented.
  - The CONFIGURE command stays in the Pico firmware and the wire contract, unused by the Pi.
  - Reviewers: not a finding.
- **The Brain VEX IMU path is the user's working setup.** Their external IMU is faulty. It must work with no Pico IMU present or with a failed one:
  - encoder frames keep flowing;
  - a VEX profile's readiness, calibration and pose never depend on Pico IMU state.
- **Travel lost in a sensor gap (8.8):** warn, and the pose becomes invalid (unplaced); the operator places again at the start pose. No bridging.

### 8.10 User rule: a used sensor that drops invalidates the pose (2026-09-27 23:20; binding, REPLACES 8.8 item 1)
The user's rule: if any sensor the active profile uses for localization stops delivering, the pose is invalid until the operator places the robot again. Sensors the profile does not use never matter.

**Rule (Pi, all profile models, not only the bench model)**
- **Used sources:**
  - the profile's encoder ports;
  - the Pico IMU when imu_source is pico;
  - the Brain VEX IMU (bench mailbox) when imu_source is brain_vex.
- **Pose continuity is lost** when the robot is localized and any used source:
  - is stale longer than `sensor_loss_ms`. Pi default 250 ms, configurable in `<BrainProfile><Timing>`. For the Brain VEX IMU, "stale" means no valid sample. A sample flagged invalid, for example during a VEX recalibration, counts as missing.
  - changes identity: a Pico boot or acquisition restart, an encoder discontinuity or epoch change, a Pico IMU epoch change or reinit, or a Brain IMU epoch change.
  - is missing from Pico frames: a used encoder channel absent from the mask, or a used Pico IMU reporting failed or initializing.
- **Consequence:** the same as a new profile per 5.2:
  - clear `kRobotLocalized`;
  - `odometry_epoch` + 1;
  - withdraw pending SET_POSE;
  - RuntimeEvent / inspection note naming the source, for example "sensor lost: brain_vex_imu stale 612 ms: place again".
  - A later placement restores normal operation once the sources are fresh again.
- **No wheel-movement condition and no bridging.** A pure stale gap counts even when the robot stood still.
- **Unused sources:** a failed, absent or initializing Pico IMU with a brain_vex or none profile changes nothing.
- **Superseded:**
  - 5.2 "A Pico restart keeps today's source-epoch and rebase behavior" (a Pico restart now unplaces);
  - 8.8 item 1's "moved more than still_travel" condition.
  - 8.8's READ_WHEELS item stays.
- **Undetectable (document it):** a quadrature encoder that is physically disconnected keeps reporting its last count, which reads the same as standing still.

**Consequences to document** (brain_setup.md is the lead's; the Pi docs are B's):
- A Brain program restart or a USB drop longer than 250 ms with a VEX profile needs a new placement. Startup placement (kAlways) does it at program start.
- VEX IMU recalibration and Pico IMU reinit both need a placement afterwards.

**Tests**
- Worker B: unit or system tests per used source (encoder stale, encoder identity change, Pico IMU failed while used, VEX stale, VEX invalid), the unused Pico IMU failing with no effect, and placement recovering.
- The lead updates the e2e tests: Brain restart and a short still cable pull now expect NeedsPlacement; new Pico-link-drop case.
- (23:30) **Viewer additions (5.5) stay in scope** (user decision): boundary and boxes, profile footprint, path polyline, readiness line.
