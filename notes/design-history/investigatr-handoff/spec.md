# investiGATR + Navigatr brain link: design spec (v2, after design review)

Requirements: `requirements.md` next to this file (the user's request, verbatim).
This spec is the shared contract for every implementer. If this spec and the
requirements disagree, the requirements win; report the conflict in your
result. Numbers marked "default" are starting points, not measured facts.

Repository root: `C:\GATR2-Electronics` (Git Bash `/c/GATR2-Electronics`).
Scratchpad `$S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/73859862-8cc3-4c8a-a408-13cc873c10fd/scratchpad`.
Every build directory goes under `$S`, never in the repo.

## 0. Repository rules (must follow)

- Do not touch `pcb/` (a user-modified .kicad_sch there must be preserved). Do not
  commit, stage, checkout, restore, stash, or otherwise change git state.
- OTHER CLAUDE SESSIONS may edit this same tree (the original owner "gatr2-electronics-d1" has ended; the rule still applies).
  Do NOT edit these files it owns: `pico/**`,
  `pi/navigatr/src/impl/localization/tracking_wheel_motion.*`,
  `pi/navigatr/src/impl/localization/register_localization.cpp`, any NEW config
  files it adds (two-parallel-wheel / BNO08X profiles under config/shared/robots,
  config/shared/pipelines, config/override/diagnostics), its new bring-up doc
  and its new tests. Shared files where only small localized edits are allowed,
  re-reading right before editing: `pi/navigatr/docs/calibration_inventory.md`,
  `docs/hardware.md`, `pi/navigatr/README.md`, `pi/navigatr/tests/profiles_gtest.cpp`.
  Exception: when converting `vex_brain`/`vex_brain_serial` slot types to
  `brain_link` you may change ONLY those type lines in its config files. If a
  build/test failure originates in its files, report it; do not fix it.
- C++17 for host code, `-Wall -Wextra` clean. Brain library code must ALSO
  compile in the PROS build as gnu++20 with arm-none-eabi-g++ 13.3.
- Style (clang-format LLVM base): IndentWidth 4, ColumnLimit 100,
  PointerAlignment Left, braces on EVERY if/for/while body. Namespace opening
  brace on its own line, contents NOT indented, `} // namespace name`.
  Functions, structs, if: attached braces. Align consecutive assignments and
  declarations like existing code. Quoted includes for project headers
  (`#include "investigatr/navigator.h"`, `#include "common/frame_codec.h"`).
- Comments: terse, plain ASCII only (no em/en dashes, no smart quotes, no
  arrows, no emojis), self-describing labels, never AI-ish narration or long
  rationale. File header = file name line plus a few lines of what it is, like
  existing files. Docs: no emojis, no em dashes, plain and factual.
- Tests: `*_gtest.cpp`, `TEST()` cases, no `main()`, link `GTest::gtest_main`.
  A `tests/` folder holds ONLY test source files (no headers, no helpers).
  Test files and code under test are listed EXPLICITLY in CMake (no globs).
  Shared host-only helpers live in a `sim/` folder.
- READMEs are short signposts. Deep docs go in `docs/` (root) or
  `pi/navigatr/docs/` (Pi runtime). navigatr is C++ only (plus CMake, XML).
- Never invent robot measurements, routes or wire ids for real hardware. The
  PROS app's hardware values are clearly marked placeholders.
- Windows build (Git Bash):
  ```
  export PATH=/c/msys64/ucrt64/bin:$PATH
  cmake -S <src> -B "$S/<dir>" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug \
        -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$S/nav-base/_deps/googletest-src"
  cmake --build "$S/<dir>" -j8
  ctest --test-dir "$S/<dir>" --output-on-failure -j8
  ```
  ucrt64 must be first on PATH. googletest source is already at
  `$S/nav-base/_deps/googletest-src` (no network needed). Baseline: navigatr
  282/282 pass. Every gtest fallback must use FetchContent name `googletest`.
- PROS: CLI `~/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-cli-windows/pros.exe`
  (3.5.6); toolchain `.../install/pros-toolchain-windows/usr/bin` (arm gcc
  13.3.1, make, sh; put it on PATH for `pros make`). kernel@4.2.2 is cached at
  `~/AppData/Roaming/PROS/templates/kernel@4.2.2`. The template default
  `-std=gnu++26`/`gnu23` is REJECTED by gcc 13; kernel headers need C++20.

### 0.1 Peer session gatr2-electronics-61 (resumed d1), added 2026-09-26
It is now active in this tree, implementing BNO08X, parallel wheels, A/B
profiles, viewer and preview work. In addition to the files above it OWNS (never
edit): `pi/navigatr/viewer/**`, `src/inspection/inspection_service.*`,
`src/runtime/inspection_state.h`, `tests/parallel_wheels_gtest.cpp`, its new
configs (parallel-wheel robot, shared localization, the A/B profiles, a synthetic
demo). It is DELETING the obsolete templates `gatr2_two_wheel_as5047_imu.xml.in`,
`two_wheel_imu*.xml.in`, `two_wheel_imu_*pipeline*`,
`two_wheel_imu_no_correction.xml`, `two_wheel_imu_camera_diagnostic.xml`: skip
any that are gone; never recreate them.
SHARED, localized edits only, re-read right before every edit, Edit never Write:
- `src/runtime/system.cpp`: the peer changes ONLY `System::publishDetectionFrames`
  and the field-worker camera feed. Everything else there is ours.
- `src/inspection/inspection_document.cpp`: ours = the `command` and `links`
  sections. The peer's = robot/field/landmark/camera sections. Do not add fields
  to the robot section; report in open_issues if placement identity should be
  exported.
- `tests/CMakeLists.txt` (the peer adds one line), `tests/profiles_gtest.cpp`,
  `field_assets_gtest.cpp`, `inspection_gtest.cpp`, `inspection_document_gtest.cpp`,
  `workers_gtest.cpp`.
- `RobotState.initialized` keeps its name and meaning (the field anchor was
  applied by localization); the peer relies on it. The placement identity fields
  are added beside it.
The peer's new A/B profiles may still say vex_brain or noop; convert ONLY their
slot type lines if they break the build, and report it.

## 1. Overview

```
Brain (PROS)                                   Pi (navigatr)
brain/testing app (control task owns motors)   CommandCollection "brain_link"
  -> investigatr::Navigator                       read + decode request, session,
       -> InputSource&                            dedupe, apply to CommandState
            navigatr_interface::NavigatrDriver  Localization applies placement
              NavigatrClient (protocol)         TargetResolution (unchanged; latch
              BytePort <- PROS serial             cleared by the session edge)
                  RS-485 half duplex            Publishing "brain_link": answer this
                                                  cycle's request, half-duplex TX
```
Namespaces: `gatr2` (wire contract), `navigatr` (Pi), `investigatr`,
`navigatr_interface`.

## 2. Wire contract: brain link v3 (common/)

The Pico sensor frame (0x01, XOR) is UNCHANGED (the Pico compiles
common/frame_codec.cpp and uses encodeSensorFrame/kSensor*). The old pose
frame (0x02) and command frame (0x03) are REMOVED from frames.h, the codec and
their tests (keep a one-line comment that 0x02/0x03 are retired).
`constexpr uint8_t kBrainLinkVersion = 3;`

### 2.1 Envelope (types 0x10, 0x11)
```
0xAA 0x55 | type u8 | len u8 | payload[len] | crc u16 little endian
```
- crc = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0)
  over type, len, payload (not the sync bytes). crc16("123456789") == 0x29B1.
- `kFrameBrainRequest = 0x10` (payload len 8..32), `kFrameBrainReply = 0x11`
  (payload len 13..64). A len outside its type's range rejects immediately.
- FrameReader: learns these types (length from byte 3, CRC validation);
  sensor frames keep XOR. On ANY rejection (bad len, bad checksum/CRC, unknown
  type) the reader rescans the bytes it had buffered after the rejected sync
  byte for the next sync pair instead of discarding them all (small change,
  applies to all types; existing reader tests must still pass).

### 2.2 Request payload (Brain -> Pi)
Header (offsets from payload start, frozen for all future versions):
```
0 version u8 | 1 op u8 | 2 session u32 (0 in HELLO) | 6 request_id u16 | 8 body
```
request_id: per Brain boot counter, starts at 1, wraps 65535 -> 1, never 0.
Each NEW transaction takes the next id; a retry resends the byte-identical
frame (same session, id, op, body). GET_STATE is never retried; each poll is a
new id.

| op | name | body | payload len |
|---|---|---|---|
| 1 | kOpHello | nonce u32 | 12 |
| 2 | kOpSetPose | x_mm i32, y_mm i32, heading_cdeg i32 | 20 |
| 3 | kOpSelectLandmark | landmark_id u8, flags u8 (bit0 selected; clear = release) | 10 |
| 4 | kOpGetState | none | 8 |

Decode: envelope/CRC valid and payload >= 8 else false. version != 3: header
decoded, body not, returns true. Current version: known op with wrong length ->
false; unknown op -> true, body ignored.

### 2.3 Reply payload (Pi -> Brain)
```
0 version u8 | 1 op u8 (echo) | 2 session u32 | 6 request_id u16 (echo)
8 result u8  | 9 pi_instance u32 | 13 body
```
session = echo of the request's session, except a HELLO Ok carries the new (or
retried) session. pi_instance = random nonzero id of this Pi runtime, new on
every process start and every System::reset().
decodeBrainReply decodes the 13-byte header for ANY version (body only for v3).

| value | result | meaning |
|---|---|---|
| 0 | kResultOk | done; for SET_POSE: localization APPLIED it |
| 1 | kResultPending | SET_POSE accepted in this session, not applied yet |
| 2 | kResultUnknownSession | session is not the Pi's current session |
| 3 | kResultUnsupportedVersion | version mismatch; header carries the Pi version |
| 4 | kResultUnsupportedOp | op unknown to this Pi |
| 5 | kResultInvalidArgument | request_id 0, duplicate id with different op/body |
| 6 | kResultUnknownLandmark | SELECT of an id with no configured mapping |
| 7 | kResultLandmarkUnsupported | SELECT while world estimation is noop (no estimate can exist) |
| 8 | kResultStale | older request id not matching a recorded request, or HELLO reusing a recent nonce |

Bodies (no body for any other pair):
| op | result | body | payload |
|---|---|---|---|
| HELLO | every result | nonce u32 (echo) | 17 |
| SET_POSE | Ok, Pending | odometry_epoch u32, anchor_revision u32 (current) | 21 |
| SELECT | Ok | landmark_id u8, flags u8 (echo) | 15 |
| GET_STATE | Ok | state block, 40 bytes | 53 |

State block (offsets within the block):
```
0  robot_flags u8  bit0 kRobotPoseValid     estimator produced a pose
                   bit1 kRobotLocalized     field anchor set by a placement
                   bit2 kRobotAgeKnown      robot_age_ms meaningful
                   bit3 kRobotAnchorCommand anchor came from a Brain SET_POSE
                                            (any session of this pi_instance)
                   bit4 kRobotAnchorConfigured anchor came from <InitialPlacement>
1  x_mm i32 | 5 y_mm i32 | 9 heading_cdeg i32   robot origin, field frame,
                   heading CCW from +x, normalized to (-18000, 18000]
13 robot_age_ms u16    Pi cycle host now minus RobotState::measuredAtHost,
                       clamped to [0, 65535]
15 odometry_epoch u32  low 32 bits
19 anchor_revision u32 low 32 bits
23 health u8     bit0 encoders fresh, bit1 gyro fresh, bit2 vision alive,
                 bit3 bias calibrated (information only, never an ack)
24 landmark_id u8      the session's selected landmark, 0 when none
25 landmark_source u8  0 none, 1 nominal, 2 observed
26 lm_x_mm i32 | 30 lm_y_mm i32 | 34 lm_heading_cdeg i32
                       full physical landmark field pose, same anchor as robot
38 landmark_age_ms u16 observed only: cycle now minus lastObservedAt, clamped;
                       0 otherwise
```
- Landmark fields are ALWAYS the physical landmark pose, never a resolved robot
  destination; the Brain applies its offset exactly once. Configured targets
  are Pi internal (inspection only) and never go on the wire.
- source none whenever no usable estimate exists (world estimation noop, no
  selection, entry missing/invalid, observed entry from another odometry
  epoch). Nominal is labeled nominal, never observed.
- Ages use Pi host clock values only; the Brain adds its measured round trip.
  Pi ages are taken at the Pi cycle start, so the Brain's sum is an upper bound
  only to within the Pi cycle processing time (a few ms); document that.
- Frame identity for the Brain = (pi_instance, session, odometry_epoch,
  anchor_revision). Any change is a coordinate discontinuity.
- Status/health/validity bits are never acknowledgements.

### 2.4 Codec API (common/frame_codec.h)
`crc16`, `kLinkEnvelopeLen = 6`, structs `BrainRequest`, `BrainState`,
`BrainReply` in frames.h, `encodeBrainRequest/decodeBrainRequest/
encodeBrainReply/decodeBrainReply`. Names may be adjusted; bytes may not.

### 2.5 common tests and build
Rewrite `common/tests/frame_codec_gtest.cpp`: delete pose/command cases; add
crc16 check value; known-byte vectors for each request op and each reply
(op, result) body; wrong body length rejected for known ops; version mismatch
and unknown op decode true; reply header decode for a foreign version; len
bounds per type; FrameReader with sensor and link frames interleaved, split
byte by byte, CRC error then relock, rescan after a corrupted len.
`common/tests/CMakeLists.txt`: find_package(GTest QUIET) then FetchContent
googletest v1.14 fallback, mirroring pi/navigatr/tests/CMakeLists.txt.

## 3. Bus ownership and timing

- The Brain is the only initiator, with at most ONE outstanding request.
- Brain: before sending each request it drains its RX and resets its
  FrameReader (anything received earlier is stale by definition). Its response
  timeout (default 60 ms) starts when its write call returns. After a reply
  or a timeout it waits request_gap (default 5 ms) before the next request.
- Pi reads the link once per estimation cycle (drain until empty). The
  brain_link Commands slot timestamps every readAvailable call with the link's
  own steady microsecond clock (`SerialLink::nowUs()`, injectable in tests;
  never the pipeline `now`, which is the cycle start and millisecond based).
- Reply window: a request completed in drain k was provably incomplete at the
  last read of drain k-1. The Pi may start its reply only while
  `now_us <= last_read_us(k-1) + reply_window_us` and not before
  `completion_read_us + turnaround_guard_us`. Defaults: reply_window 40 ms,
  guard 1000 us. If drain k is the first since open/start/reset (no k-1), the
  request gets no reply (the Brain retries). The window check happens INSIDE
  the link immediately before asserting DE:
  `SerialWriteResult write(ByteSpan, const TransmitWindow&)` with
  `TransmitWindow{not_before_us, deadline_us}` and `SerialWriteResult.expired`.
  The plain `write(ByteSpan)` stays for other users.
- Also no reply when: bytes followed the processed request in the same drain
  (the Brain may be transmitting), or the link reports input pending right
  before DE (`FIONREAD` on Linux; counted, reply dropped).
- Newest request wins: if one drain completes several requests, only the
  newest is processed; the others were abandoned and are counted, never
  applied. A buffered partial frame is discarded when a whole drain returns
  zero new bytes (requests are sent in one burst).
- Budget (document with the formula, not just numbers): Brain timeout 60 ms >=
  reply_window 40 ms + max reply airtime (59 B at 115200 = 5.1 ms) + tty and V5
  latency + margin. Because the Pi never starts a reply after the Brain's
  request end + reply_window, and finishes it within airtime + DE release, a
  retry never overlaps a late reply. Residual risks to document: Pi preempted
  between the deadline check and DE assert, and DE release delayed by
  preemption after transmit completion (counted as late_release).
- Build check: a profile with brain_link Commands must have
  `1000 / loop_rate_hz <= reply_window_ms / 2`, else build error (so one full
  loop period plus processing fits the window).
- brain_link requires threaded mode: `--inline`/step() runs world estimation
  between Commands and Publishing. main prints a warning when --inline is used
  with brain_link. Do not reorder step() (tests rely on it).
- Pi TX timeout is derived from the frame: bytes * 10 / baud + 2 ms. The Pi
  write blocks the estimation worker for about the airtime (5 ms for 59 B at
  115200); document the cost. Brain GET_STATE period default 20 ms.
- V5 smart port handles its own RS-485 direction: hardware assumption, document.

## 4. Session semantics

### 4.1 Pi (brain_link Commands)
State: current session (u32, 0 none), its opening (nonce, rid), whether any
non-HELLO request was accepted in it, a ring of the last 4 opening nonces
(including current), newest processed rid, last SET_POSE record (rid, body,
placement sequence), last SELECT record (rid, body), pi_instance, poll timing.

- HELLO:
  - nonce and rid equal the current session's opening AND nothing else was
    accepted in it: retry, answer Ok with the same session, change nothing.
  - else nonce in the recent-nonce ring: kResultStale, change nothing.
  - else open a NEW session: random nonzero u32 != previous; push nonce;
    clear client state: object_requested = false, object_wire_id = 0,
    object_sequence += 1 (this edge deactivates any configured target latch),
    dedupe records cleared. HELLO NEVER touches init_pose, init_sequence,
    init_session or localization. Answer Ok with the new session.
- Other ops with session != current (or current 0): kResultUnknownSession.
- rid == 0: kResultInvalidArgument.
- Newer rid (16-bit serial arithmetic: (rid - newest) mod 65536 in 1..32767,
  or no newest yet): new request; apply; record; newest = rid.
- Not newer: if it equals the last SET_POSE or last SELECT record's rid with
  the same op and body, answer as a duplicate (never re-apply); if it equals
  newest with op GET_STATE, answer fresh data; if same rid but different
  op/body, kResultInvalidArgument; otherwise kResultStale. Nothing applied.
- SET_POSE (new): init_pose, init_session = session, init_sequence += 1;
  record the sequence. Publishing answers Ok iff RobotState's applied
  placement identity == ("command", session, that sequence), else Pending.
  Duplicates are answered the same way. GET_STATE may be sent between SET_POSE
  retries; the record makes that safe.
- SELECT (new): select bit set: object_requested = true, object_wire_id = id;
  clear: object_requested = false; object_sequence += 1 either way.
  Publishing answers (new and duplicate alike): release -> Ok; world estimation
  noop -> kResultLandmarkUnsupported; id not mapped -> kResultUnknownLandmark;
  else Ok.
- GET_STATE: no state change.
- Version mismatch: kResultUnsupportedVersion. Unknown op: kResultUnsupportedOp.
- `System::requestsFrom` emits the command placement only while
  `init_session == command.session` (a new session withdraws an unapplied
  placement; PlacementEdge keeps the applied identity, so nothing re-applies).
- init_sequence restarts at 0 after System::reset (CommandState is cleared);
  uniqueness comes from (origin, session, sequence) plus the new pi_instance.

### 4.2 Brain (NavigatrClient)
State: nonce (fresh per HELLO transaction, mixed from several entropy sources
on the Brain; kResultStale on HELLO -> new nonce and retry), rid counter,
session (0 until HELLO Ok), pi_instance recorded at HELLO Ok, latest state,
receive time and round trip of the matched attempt.
- Correlation first: accept a reply only if op and request_id match the
  outstanding request and (HELLO) nonce matches or (other ops) session matches
  the session the request was sent in. Everything else is dropped and counted.
- Only a correlated reply may trigger the pi_instance rule: a different
  pi_instance than recorded = Pi restarted -> session change.
- kResultUnknownSession for our session -> session change.
- ANY session change (UnknownSession, pi_instance change, new HELLO): clear
  cached input (invalid), fail every in-flight state-changing ticket with
  kSessionLost before the next transmission (never resend it), clear the
  acknowledged selection, then HELLO with a fresh nonce.
- Ready = session open and at least one GET_STATE Ok in this session.
- connected = ready and last correlated reply received within link_timeout
  (default 0.25 s); link_age = now - that receive time.
- Scheduling, one outstanding request: placement > select > GET_STATE poll
  (default every 20 ms). Unsupported version/op are terminal errors reported in
  status, not retry storms (retry HELLO slowly, e.g. every 1 s).
- Placement: sent only when the application submits it; same-rid retries on
  timeout or Pending, bounded by attempts and a deadline. Result states:
  kPending, kApplied, kRejected (error result), kTimedOut ("outcome unknown",
  the app must not assume not applied), kSessionLost. When a placement comes
  back Ok, record its (odometry_epoch, anchor_revision); cached input stays
  unusable and the ticket stays kPending until a GET_STATE reply reports that
  anchor (then kApplied). So the Navigator never sees a pre-placement pose after
  kApplied.
- Selection: the wanted landmark (from InputRequest) is reconciled with the
  session's acknowledged selection by SELECT with bounded retries. Unknown/
  Unsupported results are settled outcomes for that wanted id; do not resend
  until the wanted id or the session changes.

A new session never relocates the robot. A genuine SET_POSE after a Brain
reboot applies even with the same rid and pose, because dedupe is per session
and placement identity is (origin, session, sequence).

## 5. Pi changes (pi/navigatr)

### 5.1 Half-duplex serial
- `LinuxSerialLink` with `<DriverEnable gpio=".."/>` is half duplex: drive DE
  low at open (listen), keep the sysfs value fd open (pwrite at offset 0).
  write: check window (expired -> no TX), check input pending (-> no TX), assert
  DE, write all bytes across partial writes/EAGAIN with poll() and a deadline
  derived from the frame, wait for transmitter empty (TIOCSERGETLSR/TEMT with
  deadline, tcdrain fallback), post guard (default 2 character times), release
  DE. Every error or timeout path releases DE and discards unsent output
  (tcflush TCOFLUSH). If DE is configured but its GPIO was unavailable, write
  returns failure (counted); never "succeed" with nothing on the bus.
- Put the sequence in a small portable piece, e.g.
  `src/transport/half_duplex.{h,cpp}`, over a tiny port interface (set driver,
  write some, wait writable, transmitter empty, input pending, discard output,
  clock). The Linux link implements it; tests use a fake.
- `SerialPort::write` must not busy spin: poll() with a timeout.
- `SerialLink` gains `nowUs()` and `write(ByteSpan, const TransmitWindow&)`
  (default implementation ignores the window) and optional `inputPending()`.
  MemoryLink: injectable clock, default real steady clock, windows respected.
- Non-POSIX builds keep compiling.
- The sysfs GPIO number base can differ on newer kernels (e.g. 512 + n);
  document as a hardware check; DE stuck high after SIGKILL until next start
  (start-up drives it low) documented.

### 5.2 Commands, state, placement
- New `CommandCollection type="brain_link"` with `<Serial resource_id="..."/>`
  (replaces `vex_brain_serial`, deleted). Implements 4.1 and 3.
- `CommandState` gains `uint32_t session`, `uint32_t init_session`, and a
  per-cycle `BrainReplyContext reply` (pending flag, op, session, rid,
  pi_instance, precomputed result, nonce, placement sequence, select echo,
  TransmitWindow). The Commands slot clears `reply` first on every run. Remove
  `stream_on` and `mode`.
- `PlacementRequest` gains `uint32_t session`. `PlacementEdge` compares
  (origin, session, sequence). `RobotState` gains `placement_origin`,
  `placement_session`, `placement_sequence` set by `PlacementEdge::apply`.
- No TargetState change: the HELLO object_sequence edge already clears a latch.
  `object_wire_id` now means the landmark wire id.
- `SlotInitializationContext` gains the CommandCollection type and its Serial
  resource id (filled after Commands is built) and whether world estimation is
  noop. brain_link Publishing refuses to build unless Commands is brain_link on
  the same resource; System::build refuses brain_link Commands with a
  non-brain_link Publishing.
- `PublishingInput` gains `Diagnostics* diagnostics = nullptr` (last member),
  passed from System::reportingCycle.

### 5.3 Publishing
```xml
<Publishing type="brain_link">
    <Serial resource_id="brain_uart"/>
    <Reply window_ms="40" turnaround_guard_us="1000"/>            optional
    <Health fresh_ms="150"> Encoder/Gyro/BiasCal as today </Health>  optional
    <FieldObject object_id="center_goal" wire_id="1"/>   repeatable, wire_id 1..255
</Publishing>
```
(the Reply element may live on Commands instead if that is simpler; pick one
and document it). Sends only when `command.reply.pending`, one reply built per
2.3/4.1 from this cycle's RobotState, the newest FieldSnapshot and health.
Landmark: FieldObjectState of the selected wire id; observed with matching
odometry_epoch -> compose(robot.field_from_odom, T_odom_object), source
observed; map seeded -> nominal pose, source nominal; otherwise none. World
estimation noop -> none. robot_flags anchor bits from placement_origin.
Writes once through `write(bytes, window)`. No unsolicited output ever.
`payloads/landmark_associations.h` is deleted if nothing else uses it.
`LinkStats` gains counters (requests, duplicates, superseded, stale,
unknown_session, replies, expired, input_pending, late_release, tx_errors) and
they appear in the inspection `links` output (update docs/inspection.md; keep
contract id navigatr.inspect/1).

### 5.4 Files, tests, docs (Pi)
Must update (build or tests break otherwise): CMakeLists source list;
tests/CMakeLists.txt; register_commands.cpp; register_publishers.cpp;
tests/end_to_end_gtest.cpp (uses vex_brain*, CommandFrame, PoseFrame, and a test
expecting unsolicited output: rewrite for brain_link, no output without a
request); tests/system_build_gtest.cpp (stream_on assertion, 'vex_brain'
wrong-category key); tests/world_estimation_gtest.cpp ('vex_brain' key);
inspection_document.cpp; every config using vex_brain/vex_brain_serial
(use brain_link for BOTH slots; keep DriverEnable gpio="6"; update comments;
if a converted profile fails the loop-rate check, raise its rate or document).
Tests (Pi):
- tests/brain_io_gtest.cpp rewritten: session open, HELLO retry idempotent,
  HELLO with a recent nonce -> Stale, per-session dedupe, SET_POSE Ok vs
  Pending and never re-applied, SET_POSE then GET_STATE then SET_POSE retry
  applies once, stale older rid, rid wrap, new-session placement with same rid
  and pose applies (anchor_revision + 1, placement_session changes), HELLO
  leaves anchor unchanged, unknown session, select/release/unknown landmark/
  world-estimation-noop Unsupported with GET_STATE source none, target latch
  cleared on new session, newest-request-wins, trailing bytes suppress reply,
  first drain after start gets no reply, window expiry drops reply, request
  split across two cycles, corrupt CRC -> no reply and no change, no output
  without a request, version/op errors, landmark nominal vs observed with the
  robot's anchor, robot age known/unknown and clamped, anchor origin bits,
  pi_instance differs across System instances and after reset(), pairing
  build errors.
- tests/half_duplex_gtest.cpp: DE before first byte, partial writes, DE
  released only after transmitter empty, release on write error, on timeout,
  on never-empty transmitter, expired window -> no DE, input pending -> no DE,
  after an error the next write succeeds with correct sequencing.
- tests/localization_gtest.cpp: same origin and sequence, different session
  -> re-anchors.
Docs: pi/navigatr/README.md (lines about targets on the wire, deferred
half-duplex item), docs/architecture.md, docs/landmarks.md (Brain output),
docs/coordinates.md (object fields), docs/inspection.md (command and links
schema), docs/setup.md (DriverEnable, pairing, threaded mode, profiles have no
FieldObject mappings so SELECT answers UnknownLandmark, *_no_correction
profiles use noop world estimation so Unsupported), docs/configuration.md if it
lists slot types, docs/calibration_inventory.md (brain link row), root
docs/hardware.md (RS-485 section only), docs/navigatr_resources.md,
docs/pi_setup.md. Honest: implemented and host tested, not hardware validated.

## 6. investiGATR (brain/investiGATR)

Portable C++17 (and gnu++20 on PROS). No PROS, serial, packets, motors, threads.
Layout: `include/investigatr/*.h`, `src/*.cpp`, `sim/` (host-only sim),
`tests/*_gtest.cpp`, `CMakeLists.txt`, `README.md`.

### 6.1 Types
```cpp
using Seconds = double; using Meters = double; using Radians = double;
struct Pose { Meters x = 0; Meters y = 0; Radians heading = 0; };   // field frame, CCW from +x
Pose compose(const Pose& a, const Pose& b);   // a * b: rotate b's translation by a.heading, headings add, wrapped
Radians wrapAngle(Radians);                   // (-pi, pi]
using LandmarkId = uint16_t;                  // 0 is never a valid landmark
using FrameGeneration = uint32_t;             // 0 = no frame; unique per source instance

struct RobotEstimate { bool valid = false; Pose pose; Seconds age = 0; };
enum class LandmarkSource : uint8_t { kNone, kNominal, kObserved };
enum class LandmarkStatus : uint8_t { kNotRequested, kPending, kAvailable,
    kUnknownLandmark, kUnsupported, kUnavailable, kStale };
struct LandmarkEstimate { LandmarkId id = 0; LandmarkStatus status = kNotRequested;
    LandmarkSource source = kNone; Pose pose; bool age_known = false; Seconds age = 0; };
struct InputRequest { bool landmark = false; LandmarkId landmark_id = 0; };
struct InputSnapshot { FrameGeneration frame = 0; bool connected = false;
    Seconds link_age = 0; RobotEstimate robot; LandmarkEstimate landmark; };
class InputSource { public: virtual ~InputSource() = default;
    virtual void request(const InputRequest&) = 0;     // nonblocking, idempotent
    virtual InputSnapshot latest(Seconds now) = 0; };  // nonblocking; ages relative to now
struct DriveCommand { double forward = 0; double turn = 0; };   // [-1,1]; +forward robot +x,
                                                                // +turn counterclockwise
struct TankOutput { double left = 0; double right = 0; };
TankOutput mixTank(const DriveCommand&);   // left = f - t, right = f + t, scaled so max |.| <= 1
```
Landmark usable iff status kAvailable, id matches, source != kNone, source
allowed by require_observed_landmark, and (observed with age known) age <=
max_landmark_age. A robot with frame 0 is unusable.

### 6.2 Navigator API
```cpp
struct Destination { static Destination field(Pose); static Destination relative(LandmarkId, Pose offset);
                     static Destination robotRelative(Pose offset); ... };
struct Waypoint { Destination destination; bool stop = false; };   // the last one always stops
using Path = std::vector<Waypoint>;
using CommandId = uint32_t;
struct MotionOptions { Seconds timeout = 0; bool require_observed_landmark = true; };  // timeout 0 = config default
class Navigator {
public:
    Navigator(InputSource& source, const NavigatorConfig& config = {});
    CommandId goTo(const Pose& field_pose, const MotionOptions& = {});
    CommandId goToRelative(LandmarkId, const Pose& landmark_local_offset, const MotionOptions& = {});
    CommandId goToRobotRelative(const Pose& offset, const MotionOptions& = {});
    CommandId follow(const Path&, const MotionOptions& = {});
    void cancel();
    void setSource(InputSource& source);
    const MotionStatus& status() const;
    DriveCommand update(Seconds now);
    static bool valid(const NavigatorConfig&, const char** why = nullptr);
};
```
goTo/goToRelative/goToRobotRelative are one-stop-waypoint follow().
Relative: T_field_dest = compose(T_field_landmark, offset). Robot relative:
compose(first usable robot pose after that waypoint activates, offset),
resolved once.

MotionStatus: command_id, state {kIdle, kWaiting, kTurning, kDriving,
kAligning, kCompleted, kCancelled, kFailed}, reason {kNone, kInvalidCommand,
kInputUnavailable, kInputLost, kFrameChanged, kLandmarkUnknown,
kLandmarkUnsupported, kLandmarkUnavailable, kLandmarkJump, kTimedOut,
kSourceChanged, kCancelledByCaller}, waypoint index and count, destination
(+ has_destination), distance and heading error, elapsed. Waiters compare
status().command_id with their own id (a mismatch means replaced).

### 6.3 Semantics
- Replacement: a new command replaces any command immediately (new id, PIDs
  reset, timers restart). Slew memory carries over.
- cancel(): active -> kCancelled/kCancelledByCaller, zero output. No effect on a
  terminal command. Terminal states persist until a new command; nothing resumes.
- Invalid config or invalid command (empty path, NaN, landmark id 0) ->
  kFailed/kInvalidCommand at once.
- Timing starts at the command's first update(). The motion timeout covers the
  whole command including waiting. Landmark/robot-relative resolution waits are
  measured from that waypoint's activation.
- Per update order: dt; terminal -> zero; timeout; frame check; input
  usability; landmark resolution; phase/control; completion.
- dt = now - previous update, clamped to [0, max_dt]; a gap > max_dt also
  resets PIDs and the settle timer.
- source.request(current_need) every update (idempotent). setSource: old
  source gets request({}), active command -> kCancelled/kSourceChanged.
- Frame: captured at the first usable robot input. A later snapshot with a
  nonzero frame different from the captured one -> kFailed/kFrameChanged.
- Input usable iff robot.valid, frame != 0, connected, robot.age <=
  max_pose_age, link_age <= max_link_age.
  - Never had usable input: kWaiting, zero; > input_wait_timeout -> kFailed/
    kInputUnavailable.
  - Had usable input and connected == false -> kFailed/kInputLost at once.
  - Had usable input, connected, pose invalid/stale -> kWaiting, zero; resume
    only within input_loss_timeout in the same frame, else kFailed/kInputLost.
- Landmark (relative waypoint), status handling: kUnknownLandmark ->
  kFailed/kLandmarkUnknown at once; kUnsupported -> kFailed/kLandmarkUnsupported
  at once; kPending/kUnavailable/kStale/not usable -> kWaiting zero until
  resolved or input_wait_timeout from waypoint activation ->
  kFailed/kLandmarkUnavailable. After resolution: missing updates keep the last
  destination (same frame).
- Landmark following: jump = change between consecutive accepted candidates
  of the same source. jump > max_landmark_jump (translation or heading) ->
  kFailed/kLandmarkJump. A source change nominal -> observed (only possible when
  require_observed_landmark is false) is accepted if within
  max_nominal_correction, else kLandmarkJump. The destination follows the latest
  accepted candidate at most landmark_follow_speed * dt and
  landmark_follow_turn_rate * dt (shortest angle). Same command id and timeout.
- Control (tank drive, forward only):
  d = distance to destination point, eb = wrap(bearing - heading),
  ef = wrap(dest.heading - heading), along = d * cos(eb).
  Phase selection (command start, waypoint advance, leaving kWaiting, leaving
  Aligning): stop waypoint and d <= arrive_distance -> Aligning; else
  |eb| > turn_exit -> Turning; else Driving.
  Turning: forward 0, turn = turnPid(eb); |eb| <= turn_exit -> Driving; stop
  waypoint and d <= arrive_distance -> Aligning. When |eb| > pi - 5 deg keep the
  previous turn sign.
  Driving: forward = clamp(drivePid(along_error) * max(0, cos eb), 0,
  max_forward) where along_error = along (+ remaining resolved path length to
  the next stop waypoint for pass-through waypoints); turn = headingPid(eb) when
  d > near_distance else 0. |eb| > turn_in_place_threshold and d >
  near_distance -> Turning. Stop waypoint: d <= arrive_distance -> Aligning;
  along <= 0 (reached or passed the perpendicular) -> Aligning if d <=
  position_tolerance else phase selection. Pass-through waypoint: advance when
  d <= waypoint_pass_radius, or along becomes <= 0 after having been > 0 while
  Driving toward it. Pass-through headings are ignored.
  Aligning: forward 0, turn = turnPid(ef); d > position_tolerance -> phase
  selection.
  Completion: in Aligning, d <= position_tolerance and |ef| <= heading_tolerance
  held continuously for settle_time -> kCompleted. Settle timer resets on
  leaving the condition, any unusable cycle, phase change, dt gap.
  min_forward/min_turn (default 0): nonzero demand below the minimum is raised
  to it (static friction), only while outside tolerance.
- PID: kP, kI, kD, integral_limit, output_limit; dt == 0 is P only; derivative
  skipped on the first sample after reset; angular PIDs use wrap(e - e_prev)/dt;
  reset on command start, waypoint change, phase change. Example kI = 0.
- Output: clamp to max_forward/max_turn, then slew limit on INCREASES of |value|
  only (per second); decreases are immediate; a sign reversal drops to 0 first.
  Terminal, idle, waiting and unusable input return exactly {0,0} and reset
  slew memory.

### 6.4 NavigatorConfig (defaults are tuning starting points)
| name | unit | default |
|---|---|---|
| position_tolerance | m | 0.03 |
| arrive_distance | m | 0.02 (< position_tolerance) |
| heading_tolerance | rad | 0.035 |
| settle_time | s | 0.25 |
| near_distance | m | 0.10 |
| waypoint_pass_radius | m | 0.15 (>= near_distance) |
| turn_in_place_threshold | rad | 0.5 |
| turn_exit | rad | 0.1 (< turn_in_place_threshold) |
| drive / heading / turn PID | per m / per rad | e.g. kP 2.0 / 1.5 / 1.2, kI 0, kD small |
| max_forward, max_turn | fraction | 0.8, 0.7 |
| forward_slew, turn_slew | fraction per s | 2.0, 3.0 |
| min_forward, min_turn | fraction | 0, 0 |
| max_pose_age | s | 0.25 |
| max_link_age | s | 0.25 |
| input_wait_timeout | s | 2.0 |
| input_loss_timeout | s | 0.3 |
| default_timeout | s | 10.0 |
| max_landmark_age | s | 1.5 (observed estimates) |
| max_landmark_jump | m, rad | 0.15, 0.25 |
| max_nominal_correction | m, rad | 0.5, 0.5 |
| landmark_follow_speed | m/s | 0.3 |
| landmark_follow_turn_rate | rad/s | 0.6 |
| max_dt | s | 0.1 |
Validated: constraints above, all values >= 0, max_dt > 0. Document that
position_tolerance must exceed the origin-to-turning-center offset plus coast,
and max_pose_age >= poll period + response time + typical Pi pose age. No
latency compensation.

### 6.5 OdometrySource (independent InputSource)
Portable dead reckoning from left/right travel (m) and optional absolute
heading. `align(pose, now)` puts it in a known frame (new generation, unique per
instance); `invalidate()`; `update(now, left_m, right_m, optional heading)`.
No pose before align(). Landmarks: status kUnsupported. connected = aligned.
Fallback procedure (document and test): while Navigatr is usable, the app may
re-align odometry from fresh (age <= max_pose_age, preferably stationary)
Navigatr snapshots; on Navigatr loss the app explicitly calls
setSource(odometry) (active command cancelled, kSourceChanged) and issues new
commands. Never align from stale or invented data.

### 6.6 Simulation and tests
sim/: differential drive (pose, track width, max wheel speed, first-order wheel
lag, optional origin offset from turning center; input TankOutput) and
SimulatedSource (truth with configurable age/latency, landmark table with
status/source, frame generation control, connected flag).
Tests (names indicative): geometry_gtest (compose/wrap across +-pi, rotated
offsets), pid_gtest (reset, integral clamp kI 0 and > 0, wrapped derivative,
dt 0), drive_gtest (mixTank, slew increase-only), navigator_gtest on the sim:
convergence from several start headings including one across +-pi; final
heading; pure heading change at the current point; overshoot and lateral
offset inside near_distance still complete; relative offset under 90 deg
landmark rotation equals the hand-computed pose; slow landmark drift (below
jump, above follow speed) completes with the same command_id and timeout; a
3 cm landmark shift during Aligning completes; a large jump fails kLandmarkJump;
nominal not accepted by default; nominal->observed within
max_nominal_correction completes when nominal allowed; 3-waypoint path advances
pass-through points (including a missed radius) and settles only at the last;
settle interrupted restarts; replacement gets a new id and restarts timeout;
cancel gives exactly zero next update and persists after input returns;
timeout; missing input -> kInputUnavailable; stale pose mid-command -> zero then
kInputLost; connected false -> kInputLost at once; frame change -> kFrameChanged;
kUnknownLandmark/kUnsupported fail at once; robot-relative resolves once;
setSource -> kSourceChanged; completed stays completed; invalid config/command.
odometry_source_gtest: no pose before align, new generation after align,
integration, fallback switch after aligned-from-Navigatr.

### 6.7 Build
`brain/CMakeLists.txt` is the only top-level host project: resolves gtest once
(find_package QUIET, else FetchContent `googletest` v1.14), then
add_subdirectory(investiGATR) and add_subdirectory(navigatr-interface). Targets:
`investigatr` (src), `investigatr_sim` (sim, host only), test executables.

## 7. navigatr-interface (brain/navigatr-interface)

Layout: `include/navigatr_interface/*.h`, `src/*.cpp` (portable), `pros/*.cpp`
(+ header under include if needed; PROS only, never listed in CMake), `sim/`
(host-only fake Pi built on the real common codec), `tests/`.
- `BytePort`: nonblocking `int read(uint8_t*, int)`, `bool write(const uint8_t*, int)`.
- `NavigatrClient`: section 4.2 state machine, time injected via poll(now).
- `NavigatrDriver`: investigatr::InputSource over the client. mm/cdeg to
  m/rad. robot.valid iff PoseValid && Localized && AgeKnown && anchor accepted
  (command anchors always; configured anchors only with
  `accept_configured_anchor`, default false) && not waiting on a placement
  anchor. robot.age = reported + round trip + (now - receive time). Frame
  generation increments whenever (pi_instance, session, epoch, anchor) changes.
  Landmark: status from selection/results (kPending until the SELECT is acked
  and a state reply carries the id; kUnknownLandmark/kUnsupported from SELECT
  results; kUnavailable when source none; kStale when not connected),
  otherwise kAvailable with source/pose/age (age = reported + round trip +
  now - receive). Placement API: `submitPlacement(pose) -> ticket`,
  `placementResult(ticket)`. Starts empty/invalid; connected per 4.2.
- PROS: `ProsSerialPort` (BytePort over serial_enable/serial_set_baudrate/
  serial_read/serial_write/serial_get_read_avail), and a wrapper class that owns
  a pros::Task polling every ~2 ms and a pros::Mutex around every public call
  (implements InputSource by delegation). Nonce: mix pros::micros() at several
  events, battery voltage/current, and an address; document it.
- CMake: target `navigatr_interface` (src + ../../common/frame_codec.cpp,
  links investigatr), `navigatr_interface_fakes` (sim), tests.
- Tests: fragmented/corrupt bytes, truncated reply then valid reply, duplicate
  replies, dropped replies (placement retried with the same bytes, GET_STATE
  gets a new id), stale replies (old rid, old session, old nonce, before-send
  bytes drained), stale measurement (large reported age -> Navigator not
  usable), Pi restart (pi_instance change via a correlated reply, UnknownSession)
  -> new session, invalid input, new generation, in-flight placement kSessionLost
  never resent, placement Ok then kApplied only after a state with that anchor,
  ready only after correlated GET_STATE, rid wrap, HELLO Stale -> new nonce,
  unsupported version terminal, timing: a fake bus/clock shows the Brain never
  transmits while a reply can still start and that the default timeout meets
  the section 3 budget, landmark statuses. Equivalence: the same Navigator
  scenario on the drivetrain sim with SimulatedSource vs NavigatrDriver fed by
  the fake Pi reporting the sim truth; both complete at the same pose within
  tolerance.

## 8. brain/testing (PROS application)

- Create with `pros c new-project brain/testing v5 4.2.2 --no-default-libs`
  (or equivalent verified command; no liblvgl). Commit-worthy files:
  project.pros, Makefile, common.mk, include/api.h, include/main.h,
  include/robot_config.h, src/*.cpp, .gitignore, README.md. Gitignored:
  firmware/ and include/pros/ (kernel binaries and headers) plus bin/ etc.
  Restore command, verified offline on a scratch copy (delete ignored files,
  restore, pros make): `pros c apply kernel@4.2.2 --force-apply --no-download`
  (adjust if the real command differs; document what works). Makefile guard:
  missing firmware/libpros.a -> $(error ...) naming the restore command.
- Makefile: `CXX_STANDARD:=gnu++20`, `C_STANDARD:=gnu17` above the include;
  `EXTRA_CXXFLAGS += -D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING`. Never edit common.mk.
- Library sources compiled in place through a reusable fragment
  `brain/gatr2_brain.mk` parameterized by `GATR2_ROOT ?= ../..` (the packaging
  route for other PROS projects). NEVER reuse common.mk's DEPFLAGS,
  MAKEDEPFOLDER or RENAMEDEPENDENCYFILE for library sources (its mv would
  overwrite the .cpp files). Verified pattern:
  ```make
  LIB_SRC := <explicit list: $(GATR2_ROOT)/brain/investiGATR/src/*.cpp, .../navigatr-interface/src/*.cpp,
              .../navigatr-interface/pros/*.cpp, $(GATR2_ROOT)/common/frame_codec.cpp>
  EXTRA_INCDIR := $(GATR2_ROOT)/brain/investiGATR/include $(GATR2_ROOT)/brain/navigatr-interface/include $(GATR2_ROOT)
  lib_obj = $(BINDIR)/lib/$(subst ../,,$(1)).o
  LIB_OBJ := $(foreach s,$(LIB_SRC),$(call lib_obj,$(s)))
  ELF_DEPS += $(LIB_OBJ)      # before -include ./common.mk
  -include ./common.mk
  define lib_src_rule
  $(call lib_obj,$1): $1
  	$(VV)mkdir -p $$(dir $$@)
  	$$(call test_output_2,Compiled $$< ,$(CXX) -c $(INCLUDE) $(CXXFLAGS) $(EXTRA_CXXFLAGS) -MMD -MP -MF $$(@:.o=.d) -o $$@ $$<,$(OK_STRING))
  endef
  $(foreach s,$(LIB_SRC),$(eval $(call lib_src_rule,$(s))))
  -include $(LIB_OBJ:.o=.d)
  ```
  Split the fragment so the ELF_DEPS/INCDIR part is included before
  common.mk and the rules after. Test the Makefile in a scratch copy of brain/
  FIRST, and back up nothing into the repo. PROS templates (IS_LIBRARY) are not
  used; document why (they need sources under the project's own src/).
- `include/robot_config.h`: ALL hardware values clearly marked (motor ports,
  reversed flags, gearset, voltage limit, serial port, baud, starting pose,
  NavigatorConfig overrides, demo destinations/path, landmark wire id and
  offset: placeholders).
- A control task is the ONLY motor writer: owns the Navigator and drivetrain,
  runs at a fixed period (10 ms), modes {disabled, autonomous navigation,
  manual}; mixes with mixTank, applies directions and the voltage limit;
  cancel/disable stop outputs immediately. Public calls are mutex protected.
- initialize(): start the Navigatr driver; bounded wait for connected; submit
  starting pose; bounded wait for kApplied; report via pros::screen::print
  (kernel only; no pros::lcd); never hang.
- autonomous(): goTo (absolute), follow (path), goToRelative (landmark), each
  waited with a bounded loop comparing command_id; stop on failure.
- opcontrol(): manual tank through the same owner; a button runs the demo, a
  button cancels. disabled(): cancel and stop. competition_initialize(): nothing heavy.
- Build with `pros make` and report. Document hardware bring-up checks.

## 9. Docs

- docs/interfaces.md: brain link v3 (envelope, CRC, ops, results, state block,
  sessions, bus ownership and timing with the budget formula); remove pose and
  command frame sections; update docs/README.md line about frames.
- docs/investigatr.md: public API, input contract, InputSource adapters
  (Navigatr, odometry, simulation) and how to write one, control law, config
  table and tuning, fallback and source switching, host build and tests,
  limitations (tank, forward only, no obstacle routing, no latency compensation).
- docs/navigatr_interface.md: driver and client, session and restart behavior
  (Brain reboot, Pi restart, link loss, placement acks), PROS integration and
  packaging (gatr2_brain.mk import steps), testing app setup and hardware
  bring-up checklist.
- brain/README.md and brain/*/README.md: short signposts.
- root README.md (Pi runtime paragraph about targets/objects on the wire,
  repository map adds brain/), docs/README.md index,
  `.github/workflows/ci.yaml` job `brain`:
  `cmake -S brain -B build-brain && cmake --build build-brain -j && ctest --test-dir build-brain --output-on-failure`
  (CI does not build brain/testing: no PROS toolchain there).

## 10. Verification gate

Run and report: common ctest; navigatr ctest (baseline 282, plus new tests);
brain ctest; `pros make` in brain/testing; Pico build only if PlatformIO works
offline and the other session's pico/ work is not mid-edit (otherwise report
not run). Unvalidated on hardware (must be listed in docs and the final
report): V5 smart port RS-485 direction handling, real turnaround timing and
TIOCSERGETLSR behavior on the Pi UART, sysfs GPIO numbering, DE behavior,
motor directions, gains, tolerances, and everything needing the robot.
