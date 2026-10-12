# Pi-side Brain link and USB/RS-485 transport: read-only inventory

Paths are relative to `C:/GATR2-Electronics`. `PN` means `pi/navigatr`. No files were changed.

## 0. Where the pieces sit and how one cycle runs

- **Wire contract (shared).** `common/frames.h` (Brain link v3: lines 66-185) and `common/frame_codec.{h,cpp}`.
- **Commands slot.** `PN/src/impl/commands/brain_link_commands.{h,cpp}`, registered as `"brain_link"` at `PN/src/impl/commands/register_commands.cpp:12`.
- **Publishing slot.** `PN/src/impl/publishing/brain_link_publisher.{h,cpp}`, registered as `"brain_link"` at `PN/src/impl/publishing/register_publishers.cpp:13`.
- **Byte-link contract.** `PN/src/resources/serial_link.h`. Implementations:
  - `linux_serial_link`, `memory_link` and `file_replay_link` in `PN/src/impl/resources/serial_links.{h,cpp}`.
  - `pros_usb_link` in `PN/src/impl/resources/pros_usb_link.{h,cpp}`.
  - All registered at `PN/src/impl/resources/register_resources.cpp:21-30`.
- **Transport.** `PN/src/transport/half_duplex.{h,cpp}`, `serial_port.{h,cpp}`, `byte_stream.h`.
- **Bench IMU mailbox.** `PN/src/resources/brain_imu_bench.h`, registered as `"brain_imu_bench"` at `register_resources.cpp:21-24`. It is read by `PN/src/impl/localization/brain_imu_wheel_bench.{h,cpp}`.

**Cycle order in threaded mode** (`PN/src/runtime/system.cpp:687-710`). Everything below runs on the estimation worker thread:

1. `estimationCycle(now)`, at lines 327-342, runs resources, then sensors, then **`commands_->run({command_, now, &diagnostics_})`** (line 335), then **localization** with `requestsFrom(command_)` (line 339).
2. Sensors are handed off to the field worker.
3. `reportingCycle(now)`, at lines 344-373, runs target resolution and then **`publishing_->run(...)`** (line 353) with the **latest** `FieldSnapshot` from the field worker.

`now` is `HostClock::now()`, taken once at cycle start with millisecond resolution (`PN/src/core/host_clock.h:19-22`).

Consequences:
- The reply is written in the same cycle as the request that caused it.
- Landmark data can lag by one field-worker cycle.

**Build-time pairing rules:**
- `brain_link` commands require `brain_link` publishing (`system.cpp:265-269`).
- The publisher requires the commands slot to be `brain_link` on the **same** `Serial` resource (`brain_link_publisher.cpp:44-48`).
- `--inline` mode with brain_link only prints a warning (`PN/src/app/main.cpp:109-110`).

---

## 1. Ops, payloads, result codes, sessions, dedupe, placement, reported state

### 1.1 Envelope and size constants
- **Link frame layout:** `AA 55 | type u8 | len u8 | payload[len] | crc u16 LE`. The CRC is CRC-16/CCITT-FALSE over type, len and payload (`frames.h:66-78`, `frame_codec.cpp:66-74`).
- **Type codes:** `kFrameBrainRequest=0x10` and `kFrameBrainReply=0x11` (`frames.h:24-25`).

| Constant | Value | Where |
|---|---|---|
| `kSync0/kSync1` | 0xAA/0x55 | frames.h:15-16 |
| `kMaxFrameLen` | 128 (whole frame, sync..crc) | frames.h:19 |
| `kLinkEnvelopeLen` | 6 | frame_codec.h:17 |
| `kBrainRequestHeaderLen` | 8 | frame_codec.h:20 |
| `kBrainRequestMaxLen` | 32 (payload) | frame_codec.h:21 |
| `kBrainReplyHeaderLen` | 13 | frame_codec.h:22 |
| `kBrainReplyMaxLen` | 64 (payload) | frame_codec.h:23 |
| `kBrainStateLen` | 40 | frame_codec.h:24 |
| `kBrainLinkVersion` | 3 | frames.h:80 |

There is **no `kMaxPayload` constant** anywhere in the repo; I grepped for it. A `len` outside [8,32] for requests or [13,64] for replies is rejected as soon as the header is read (`frame_codec.cpp:56-64`, `438-443`).

### 1.2 Requests (Brain to Pi)
Header, frozen for all versions: `version u8 | op u8 | session u32 | request_id u16` (`frames.h:79-81`; encoder at `frame_codec.cpp:259-292`). All fields are little-endian.

| op | const | body (offset from payload 8) | payload len | frame bytes |
|---|---|---|---|---|
| 1 | `kOpHello` | nonce u32 | 12 | 18 |
| 2 | `kOpSetPose` | x_mm i32, y_mm i32, heading_cdeg i32 (field frame, CCW) | 20 | 26 |
| 3 | `kOpSelectLandmark` | landmark_id u8, flags u8 (bit0 `kSelectFlagSelected`; clear = release) | 10 | 16 |
| 4 | `kOpGetState` | none | 8 | 14 |
| 5 | `kOpGetStateWithImu` | imu_flags u8 (bit0 `kBenchImuValid`), imu_stamp_ms u32, imu_rotation_mdeg i32 | 17 | 23 |

- Lengths come from `brainRequestLen()` (`frame_codec.cpp:164-173`).
- Struct: `struct BrainRequest` (`frames.h:134-149`). Comments there: `imu_stamp_ms` is the "Brain acquisition clock, not the Pico clock", and `imu_rotation_mdeg` is "continuous rotation, CCW positive".
- Decode (`frame_codec.cpp:294-337`):
  - A non-v3 version decodes the header only and returns true.
  - An unknown op decodes the header only and returns true.
  - A known op with the wrong len returns false.

### 1.3 Replies (Pi to Brain)
Header, frozen: `version | op (echo) | session u32 | request_id u16 | result u8 | pi_instance u32` (`frame_codec.cpp:346-352`). `struct BrainReply` is at `frames.h:170-185`. `brainReplyLen(op,result)` is at `frame_codec.cpp:175-191`.

| op | result | body | payload | frame |
|---|---|---|---|---|
| HELLO | **every** result | nonce u32 (echo) | 17 | 23 |
| SET_POSE | Ok or Pending | odometry_epoch u32, anchor_revision u32 (current values) | 21 | 27 |
| SELECT | Ok | landmark_id u8, flags u8 (echo) | 15 | 21 |
| GET_STATE / GET_STATE_WITH_IMU | Ok | 40-byte state block | 53 | 59 |
| anything else, or result > 8 | — | header only | 13 | 19 |

**State block layout** (`wrState`, `frame_codec.cpp:94-109`):

| Offset | Field |
|---|---|
| 0 | robot_flags u8 |
| 1 | x_mm i32 |
| 5 | y_mm i32 |
| 9 | heading_cdeg i32, in (-18000,18000] (`math/angles.h:30-33`) |
| 13 | robot_age_ms u16 |
| 15 | odometry_epoch u32 (low 32 bits) |
| 19 | anchor_revision u32 (low 32 bits) |
| 23 | health u8 |
| 24 | landmark_id u8 |
| 25 | landmark_source u8 |
| 26 | lm_x_mm i32 |
| 30 | lm_y_mm i32 |
| 34 | lm_heading_cdeg i32 |
| 38 | landmark_age_ms u16 |

**Result codes** (`frames.h:94-104`):

| Value | Name | Meaning |
|---|---|---|
| 0 | Ok | Done; for SET_POSE, localization applied it |
| 1 | Pending | SET_POSE accepted in this session, not applied yet |
| 2 | UnknownSession | |
| 3 | UnsupportedVersion | Reply carries the Pi's version |
| 4 | UnsupportedOp | |
| 5 | InvalidArgument | rid 0, or a reused rid with a different op or body |
| 6 | UnknownLandmark | No mapping for the id |
| 7 | LandmarkUnsupported | World estimation is noop |
| 8 | Stale | |

**Flag bits:**
- robot_flags (`frames.h:111-117`): PoseValid=1, Localized=2, AgeKnown=4, AnchorCommand=8, AnchorConfigured=16.
- health (`frames.h:120-125`): EncodersFresh=1, GyroFresh=2, VisionAlive=4, BiasCalibrated=8.
- landmark source (`frames.h:127-131`): None=0, Nominal=1, Observed=2.

### 1.4 Processing order in `BrainLinkCommands::process` (`brain_link_commands.cpp:213-294`)
The reply context is first filled with an echo of op, session and rid, plus `pi_instance`, the nonce, and `result=Ok` (215-222). The checks then run in this order:

1. `version != 3` gives UnsupportedVersion (224-227).
2. `brainRequestLen(op)==0` (unknown op) gives UnsupportedOp (228-231).
3. Op 5 without a `<BenchImu>` configured gives **UnsupportedOp** (232-235).
4. Op 5 with any imu_flags bit other than bit 0 gives InvalidArgument (236-239).
5. `request_id==0` gives InvalidArgument (240-243).
6. HELLO goes to `hello()` (244-247).
7. `session_==0 || req.session!=session_` gives UnknownSession, counted in `stats->unknown_session` (248-254).
8. A request id that is not newer (`have_newest_ && !newerId(rid, newest_rid_)`) goes to `repeat()` (255-258).
9. Otherwise it is a new request: record `newest_rid_` and set `accepted_=true` (260-262). Then by op:
   - **Op 5.** `bench_imu_->accept(session_, imu_flags, imu_stamp_ms, imu_rotation_mdeg, now)` (264-267). The reply is a normal state reply.
   - **SET_POSE** (268-278):
     - `c.init_pose` = (mm/1000, mm/1000, cdegToRad), `c.init_session=session_`, `++c.init_sequence`.
     - Records `SetPoseRecord{rid, x, y, h, sequence}`.
     - `result=Pending` and `placement_sequence=c.init_sequence`.
   - **SELECT** (279-290):
     - Flag set: `object_requested=true` and `object_wire_id=landmark_id`. Flag clear: `object_requested=false`.
     - `++c.object_sequence`.
     - Records a `SelectRecord` and echoes the id and flags.
   - **GET_STATE** changes nothing (291-292).

`newerId` uses 16-bit serial arithmetic: `(a-b) mod 65536` in 1..32767 (`brain_link_commands.cpp:21-24`).

### 1.5 Session and `pi_instance` logic
**`pi_instance`:**
- It is a random nonzero u32, created once per `BrainLinkCommands` construction (32-42).
- The seed mixes two `random_device` draws, the steady clock split into two halves, and a static per-process instance counter.
- `reset()` picks a new value that differs from the previous one (108-123). `System::reset()` calls it through `resetStages()` (`system.cpp:575-595`), which also sets `command_ = CommandState{}`.

**`hello()`** (296-332):
- **Retry.** If `session_!=0`, the nonce and rid equal the opening pair, and `!accepted_`, the reply is Ok with the same session and nothing changes (299-303).
- **Stale.** If the nonce is in `recent_nonces_`, the reply is Stale and counted (304-311). The ring is the last 4 opening nonces (`kRecentNonces=4`, line 18) and includes the current one. The Stale reply's session field echoes the request's session, which is 0.
- **New session:**
  - `session_=randomNonzero(session_)`.
  - `bench_imu_->reset(session_)`, which sets `valid=false` and `++epoch`.
  - Records the opening nonce and rid, sets `accepted_=false`, pushes the nonce onto the ring (max 4).
  - Clears `have_newest_` and both dedupe records.
  - Sets `c.session`, `object_requested=false`, `object_wire_id=0`, and `++object_sequence` (313-331).
- HELLO **does not touch** localization, `init_sequence`, `init_pose` or `init_session` (comment at 326).

### 1.6 Dedupe (`repeat()`, 334-372)
Only three things are remembered: the **last SET_POSE**, the **last SELECT**, and `newest_rid_`.

- **rid equals the SET_POSE record's rid.**
  - Same op and body: Pending with the recorded `placement_sequence`, counted as a duplicate. The publisher upgrades it to Ok if that placement has been applied.
  - Otherwise: InvalidArgument.
- **rid equals the SELECT record's rid.**
  - Same op and body: echo with Ok. The publisher then refines the result (see 1.8).
  - Otherwise: InvalidArgument.
- **rid equals `newest_rid_`.**
  - GET_STATE or op 5: Ok with fresh state. **The IMU sample is not re-accepted** because the switch is never reached.
  - Other ops: InvalidArgument.
- **Anything else:** Stale.

The last SET_POSE record can be matched after newer GET_STATE ids have been used. The test at `PN/tests/brain_imu_bench_gtest.cpp:156-159` relies on this ("A pending SET_POSE may be retried after newer IMU/state polling").

### 1.7 Placement identity
- **Identity is a (origin, session, sequence) triple.** `RobotState` has `placement_origin` (`"command"` or `"configuration"`), `placement_session` and `placement_sequence` (`PN/src/state/robot_state.h:62-64`).
- **Request.** `System::requestsFrom` asks for a placement only when `init_sequence!=0 && init_session==command.session` (`system.cpp:275-287`). A new session therefore withdraws an unapplied placement.
- **Apply.** `PlacementEdge::apply` (`PN/src/impl/localization/motion_step.cpp:70-86`) applies only when the triple differs from the last applied one.
  - It sets `field_from_odom = placement * inverse(odom_pose)`, `++anchor_revision`, `valid=initialized=true`, and stores the triple.
  - The odometry epoch is not changed.
- **Ack.** The publisher replies Ok only if `robot.placement_origin=="command" && placement_session==ctx.session && placement_sequence==ctx.placement_sequence`; otherwise Pending (`brain_link_publisher.cpp:280-287`). Both answers carry the current `odometry_epoch` and `anchor_revision`.
- **Sequence scope.** `init_sequence` increases for the whole Pi instance and is not reset per session.
- **Configured placement** has origin `"configuration"`, sequence 1 (`PN/src/runtime/localization_stage.cpp:228-234`). It applies only when no command placement is requested (lines 65-68).

### 1.8 What GET_STATE reports (`BrainLinkPublisher::state`, `brain_link_publisher.cpp:181-257`)
**Robot fields:**
- PoseValid means `robot.valid`. Localized means `robot.initialized`.
- AgeKnown plus `robot_age_ms = now - measuredAtHost`, clamped to 0..65535. It is set only if both times are in the host clock domain.
- The anchor bits come from `placement_origin`.
- The pose is `robot.fieldPose()`. Epoch and revision are truncated to u32.

**Health bits** (all Pi-side information):
- Encoders: every configured `<Encoder>` sensor is `kValid` and `now - latest.receivedAt <= fresh_ms` (default 150).
- Gyro: the same test on the configured `<Gyro>` sensor.
- Vision: the field snapshot's `observations` map is non-empty.
- BiasCal: latched once the named observation function reports ready (`run()` 261-266; reset in `reset()` 147).

**Landmark: only one is reported, the selected one.**
- `landmark_id = command.object_requested ? object_wire_id : 0` (226).
- If that id is 0, or world estimation is noop, or the wire id has no mapping, or the object is missing or invalid, the source stays none (227-238).
- **Observed.** It is reported only if `o.odometry_epoch == robot.odometry_epoch` and `lastObservedAt` is host time. The pose is `compose(robot.field_from_odom, o.T_odom_object)` and the age is `now - lastObservedAt` (240-246).
- **Field map.** Reported as `o.pose.pose` with source Nominal and age 0 (247-249).
- Anything else gives source none.

**SELECT result refinement** (`selectResult`, 168-179):
- A release is always Ok.
- If world estimation is noop: LandmarkUnsupported.
- If the id has no `<FieldObject>` mapping: UnknownLandmark.
- Otherwise Ok.

The selection is **recorded in either case**. Tests: `brain_io_gtest.cpp:650-684`.

---

## 2. USB envelope (`pros_usb_link`)

**Structure.** `ProsUsbLink` (`pros_usb_link.h:22-41`) wraps an inner `SerialLink`. In production the inner link is the file-local `ReconnectingUsbPort` (`pros_usb_link.cpp:44-176`). The factory `make_pros_usb_link` (280-293) requires `<Device path="auto"|explicit>` and rejects `DriverEnable`. Its header comment reads: "Optional USB is retried once per second; absence never prevents startup" (`pros_usb_link.h:43-44`).

**Framing:**
- Each unchanged v3 link frame becomes one ASCII line: `"NG1:" + UPPERCASE hex of each byte + "\n"` (`pros_usb_link.cpp:264-276`).
- There is no escaping, no COBS and no checksum beyond the inner CRC. Hex encoding is itself the escaping: it keeps the binary PROS stdin control sequence `pR` out of the stream (test `pros_usb_link_gtest.cpp:69-78`).
- The header comment says: "The Brain app must disable its output COBS mode" (`pros_usb_link.h:1-3`). The Brain side does this with `pros::c::serctl(SERCTL_DISABLE_COBS, ...)` at `brain/localization-test/src/usb_bench_driver.cpp:103`.
- Constants:
  - `kPrefixLength=4` (24).
  - `kLineLimit = 4 + 2*128 + 256 = 516` characters (25).
  - Maximum outgoing line = 4 + 256 + 1 = 261 bytes. A write with `source.size==0 || >128` fails with "USB frame size invalid" (265-266).

**Receive parser** (`acceptByte` 225-238, `finishLine` 208-223). This is how the link coexists with PROS stdout and console text:
- Bytes build up until `\n`, and one trailing `\r` is stripped.
- The parser uses the **last** `"NG1:"` in the line (`rfind`), so a diagnostic prefix before the marker is tolerated.
- The digits after the marker must be non-empty, even in number, at most 256, and all in `0-9A-F`. Lowercase is rejected. Any violation drops the **whole line** silently.
- Lines without the marker are ignored. A line longer than 516 characters switches to discard mode until the next `\n`.
- Decoded bytes are queued in `decoded_` (a `std::deque`) and handed to the caller in pieces as needed.
- Test: `pros_usb_link_gtest.cpp:94-107`.

**Read budget.**
- `readAvailable` performs at most **4 inner reads of 256 bytes (1024 raw bytes)**, and only while `decoded_` is empty ("Never drain an unbounded diagnostic stream in one localization step", 242-253).
- If those 1024 bytes are all noise it returns 0. The commands drain then ends for this cycle even if the tty still holds data.
- LinkStats byte counters count **decoded inner bytes**, not console bytes (`usb_localization_bench.md:103-105`).

**Device discovery** (`findProsUsbUserPort`, 179-206):
- It scans `/sys/class/tty/ttyACM*/device` and walks up to 12 parent levels for `bInterfaceNumber`, `idVendor` and `idProduct`.
- It accepts only `02 / 2888 / 0501`. More than one match returns empty ("multiple Brains: require explicit device").
- It never guesses the ACM number and never opens the system/upload interface.

**Open.** `connect()` (142-170):
- `O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC`, raw mode, B115200, `CLOCAL|CREAD`, no CRTSCTS, `VMIN=0`/`VTIME=0`.
- **`tcflush(TCIOFLUSH)`**, so bytes waiting from before the connect are dropped.

**Reconnect and unplug:**
- **Read path.** `readAvailable` calls `connect()` first; if that fails the link reports closed. A zero-timeout `poll` that returns `POLLHUP|POLLERR|POLLNVAL`, or a `read` that returns 0 or a hard error, calls `disconnect()` (close the fd) and reports closed (50-70).
- **Write path.** Any write error, a poll failure, a hangup, or backpressure past the bound also disconnects (91-114).
- **Retry rate.** `connect()` is rate-limited by `retry_at_ = now + kRetryUs` (1,000,000 µs) at each attempt (144-145).
  - After a disconnect, the first retry is immediate if more than 1 s has passed since the last attempt. After that it retries at most once per second.
  - Discovery runs again each time, so a changed ACM number is picked up.
- **Thread cost.** Discovery, which reads sysfs files, and `open` both run on the **estimation worker thread**, inside `commands_->run`.
- **Pi-side effect of "closed":**
  - `ProsUsbLink` clears `line_` and the discard flag (246-250).
  - `BrainLinkCommands::run` sets `have_last_read_=false` and returns `kNoData`, keeping all command and session state (`brain_link_commands.cpp:144-148`). A request decoded earlier in that same drain is **dropped without processing**, because the local `request` is discarded.
  - The first drain after reconnecting has no previous drain, so a request finished in it is applied but **not answered**. The Brain's retry is then answered: for HELLO, the retry path returns the same session (test `brain_io_gtest.cpp:546-556`).
- **Write timing.**
  - `ReconnectingUsbPort::write` checks `window.missed`, **sleeps** until `not_before_us`, and checks again.
  - Backpressure is bounded to `min(deadline, now+5 ms)` (78-88).
  - It **does not refuse on input pending**; only `MemoryLink` and the half-duplex path do. On USB the link is effectively full duplex at the transport level. The "trailing bytes" and "first drain" rules still apply because they live in `BrainLinkCommands`.

**Brain restart:**
- The Pi has **no USB-specific restart handling**. The new Brain boot sends HELLO with a fresh nonce and request_id restarts at 1. The Pi opens a new session, the dedupe records reset, and the bench IMU epoch increases.
- The robot pose and anchor survive. An unapplied placement from the old session is withdrawn.
- The doc says: "Disconnects make poses stale; reconnecting does not automatically reapply a previous placement" (`usb_localization_bench.md:113-114`).
- *Speculation:* I could not confirm from code whether a V5 program restart, as opposed to a power cycle, re-enumerates the USB CDC device. If it does not, the Pi sees an uninterrupted link and the first HELLO is answered at once.

---

## 3. RS-485 half-duplex timing

**Resource.** `linux_serial_link` with `<DriverEnable gpio=.. post_guard_us=.. tx_margin_us=..>` (`serial_links.h:4-19`; factory `serial_links.cpp:94-166`).
- Default baud is 115200. Allowed bauds are 9600, 19200, 38400, 57600, 115200 and 230400, plus 460800 and 921600 if the platform defines them (`serial_port.cpp:41-57`).
- `required="true"` makes a failed open or GPIO a build error. Otherwise it is a warning plus a dead link (105, 112-121, 153-163).
- **There is no reopen or reconnect for `linux_serial_link`.** A closed fd means `read` returns -1, i.e. closed, on every read from then on (`serial_links.cpp:33-39`, `serial_port.cpp:129-138`).

**DE handling** (`serial_port.cpp:149-186`):
- Uses sysfs `/sys/class/gpio/gpioN`. Setup: export, then `direction` set to `"low"` (retrying up to 20 times at 10 ms on EACCES), then `value` kept open.
- `setDriver()` writes `"1"` or `"0"` with `pwrite`. `close()` drives it low.
- The header comment says the GPIO drives "RS-485 DE and /RE" (`serial_links.h:7`), so the Pi receiver is off while it transmits.
- If the GPIO is missing on a half-duplex link, every write fails (`serial_links.cpp:46-48`; test `half_duplex_gtest.cpp:366-381`).

**Transmit sequence** (`transmitHalfDuplex`, `half_duplex.cpp:86-133`):
1. `waitForWindow`: fail as expired if missed; otherwise sleep until `not_before` and re-check the deadline (52-62).
2. If `inputPending()` (via `FIONREAD`, `serial_port.cpp:188-191`), refuse with `input_pending`.
3. Record `on_us`. The deadline is `on_us + transmitBudgetUs(bytes, baud, margin)`.
4. DE on.
5. `writeAll` to the deadline.
6. `waitTransmitterEmpty`: sleep until `on_us + airtime`, then poll `TIOCSERGETLSR & TIOCSER_TEMT` every character time until the deadline. The fallback is `tcdrain`, which has **no deadline** (`serial_port.cpp:219-237`).
7. Post-guard sleep.
8. DE off.
- On any error, `tcflush(TCOFLUSH)` runs and DE is still released.
- `late_release` is reported if release happens after `deadline + post_guard`.

**Timing constants:**

| Name | Value | Where |
|---|---|---|
| `characterTimeUs(baud)` | ceil(10e6/baud) = **87 µs** @115200 | half_duplex.cpp:37-42; test half_duplex_gtest.cpp:146 |
| `transmitBudgetUs` | ceil(bytes·10e6/baud) + margin; 59 B → **7122 µs** | half_duplex.cpp:44-50; test :149 |
| `post_guard_us` default | 2 × char = **174 µs** | half_duplex.h:49; serial_links.cpp:139 |
| `tx_margin_us` default | **2000 µs** | half_duplex.h:50 |
| `kWriteMarginUs` (non-windowed writes) | 2000 µs | serial_port.cpp:39 |
| Reply `window_ms` default | **40** | brain_link_commands.cpp:63 |
| `turnaround_guard_us` default | **1000** | brain_link_commands.cpp:64 |
| Validation | `window_ms>0`, `0 <= guard < window_ms*1000` | :70-74 |
| Loop constraint | `1000/loop_rate_hz <= window_ms/2`, else build error | :76-84 |

**Reply window** (`run`, `brain_link_commands.cpp:125-211`):
- Each drain reads in a loop until a read returns 0. Each read is stamped with `link_->nowUs()` taken **before** the read, into a 128-byte stack buffer.
- **Newest request wins.** Earlier requests in the same drain are counted as `superseded` and never applied.
- `not_before_us = completed_us + guard` and `deadline_us = previous drain's final empty read + window` (208-209).
- **No reply** in two cases: the first drain since construction, reset or close; or more bytes arrived after the request in the same drain (`completed_at < drained`). In both cases the request is **still applied** and counted as `unanswered` (201-206).
- A drain with zero bytes resets the `FrameReader`, discarding any partial frame (192-194).

**The 50 Hz / 40 ms numbers mean two different things:**
- On the **Pi**, a 40 ms window needs a loop period of at most 20 ms, so a **loop rate of at least 50 Hz**. The test shows 40 Hz fails and 50 Hz builds (`brain_io_gtest.cpp:958-966`). Both bench configs use 100 Hz.
- On the **Brain**, the default GET_STATE poll is 20 ms (50 Hz) (`brain/communiGATR/include/communigatr/client.h:30`).
- Other Brain defaults from `docs/interfaces.md:504-512`: response timeout 60 ms, request gap 5 ms, link timeout 0.25 s.
- The budget inequality is `T >= A_req + W + A_rep + R + L`, i.e. 60 ≥ 2.3 + 40 + 5.1 + 0.2 + L (`interfaces.md:516-533`).
- The reply write **blocks the estimation worker** for about the airtime (`interfaces.md:547-548`).
- `interfaces.md:37-42` states that none of these timings has been validated on hardware.

---

## 4. VEX IMU bench forwarding, end to end

**What the Brain sends** (Brain side, for context only):
- `readBenchImu` (`brain/localization-test/src/main.cpp:23-38`):
  - `stamp_ms = pros::millis()`.
  - The sample is invalid while calibrating or in an error state.
  - `rotation_mdeg = lround(-imu.get_rotation()*1000)`. PROS reports clockwise-positive degrees and this converts to CCW millidegrees. Non-finite or out-of-range values are invalid.
- When `bench_imu` is configured, communiGATR sends **op 5 in place of every GET_STATE** (`brain/communiGATR/src/client.cpp:404-418`). The rate is therefore the state poll period, default 20 ms. One op-5 poll is also forced between SET_POSE retries (419-423).
- The rotation is continuous and unwrapped. There is no heading-rate field and no tilt.
- The IMU Smart Port (port 1) is a Brain constant in `robot_config.h` (`vex_imu_bench.md:17-23`). **The Pi has no concept of a Smart Port or an IMU port**; a grep for `smart.?port|imu_port` in `PN/src` and `PN/config` found nothing.

**Pi intake:**
- Op 5 is accepted only if `<BenchImu resource_id=..>` is configured on the CommandCollection (`brain_link_commands.cpp:91-95`). Otherwise the result is UnsupportedOp (test `brain_imu_bench_gtest.cpp:216-221`).
- On a **new** request id, `BrainImuBench::accept(session, flags, stamp, rotation, now)` is called with **`now` = the loop's host time at cycle start**. The link read time is not used, so the resolution is 1 ms and the value is quantized to the loop period.

**Mailbox** `struct BrainImuBench` (`PN/src/resources/brain_imu_bench.h:9-37`). It has no lock and is documented as same-worker only (lines 5-7). Fields: `valid`, `session`, `stamp_ms`, `rotation_mdeg`, `sequence`, `epoch`, `received`. Behaviour:
- `reset(next_session)`: `valid=false`, `received={}`, `++epoch`.
- A session change resets it.
- `flags&1==0`: resets if it was valid, then returns.
- **Same stamp**: ignored ("retained sample cannot refresh health").
- **Stamp lower than the previous one** (Brain restart): reset, then the new sample is accepted into the new epoch. This is a plain u32 compare with no wrap handling.
- Otherwise: store, `received=now`, `++sequence`.
- HELLO (`brain_link_commands.cpp:314`) and `reset()` (109) also reset it.

**Localization model.** The observation types are `brain_imu_parallel_bench` and `brain_imu_planar_bench`, registered at `PN/src/impl/localization/register_localization.cpp:16-18`; the code is `brain_imu_wheel_bench.cpp`.
- **Config:** `<Imu resource_id>`, `<Wheels resource_id=wheel_geometry><Use wheel_id/>×2`, `<Output observation_id>` (25-35).
  - Exactly 2 wheels with distinct encoder sensors (61-64).
  - Parallel mode needs both wheels at a 0° measurement angle.
  - Planar mode needs `|det| >= 1e-3` (46-50, 67-72).
- **Freshness:** IMU `received` and each encoder's `receivedAt` must be host time within **200 ms** of `context.now` (85-97). Otherwise it drops its baseline and returns NoData.
- **Rebaseline** (no output that step) on any of:
  - a new IMU epoch;
  - a gap of more than 200 ms between IMU receipts, or time not moving forward;
  - a change in a wheel's stored epoch or discontinuity epoch;
  - `|dθ| > 12 rad/s · dt + 0.1` (106-112, 124).
- It emits only when the IMU sequence advanced **and** both wheels have new samples (104-116).
- **Output:** a `BodyMotionIncrement` with `startAt` = the previous IMU receipt, `endAt` = the current IMU receipt (host), `dtheta` from the rotation difference, and translation from the wheel travel after removing the lever-arm term `k·dθ`.
  - Parallel mode: `dx` is the mean of the two wheels and `dy=0`.
  - Planar mode: a 2×2 solve.
  - `measuredAt = receivedAt = endAt`; provenance includes `{"brain_imu_bench","brain.vex_imu","brain",seq,epoch}` (118-166).
  - The estimator in both bench configs is `planar_motion_integrator`.
- **Timing caveat.** Because commands run before localization in the same `estimationCycle`, the sample is used in the cycle that received it. Wheel and IMU data are paired **by Pi arrival time only**; the build prints a "BENCH ONLY" warning (73-76).

**Configs:**
- `bench_vex_imu.xml`: RS-485 `/dev/ttyAMA5`, 115200, DE GPIO 6, `required`; parallel model; loop 100 Hz; world estimation noop; no `<FieldObject>`.
- `bench_vex_imu_usb.xml`: `pros_usb_link path="auto"`, planar model, one sideways wheel at 90°, otherwise the same.
- Wheel geometry is XML-only in both: radius 0.024, CPR 4000, offsets ±0.15, labelled `UNMEASURED_BENCH_DEFAULT`.

---

## 5. Landmark tracking and SELECT

- **Holder.** `FieldState { unordered_map<FieldObjectId, FieldObjectState> objects; }` (`PN/src/state/field_state.h:62-64`). It sits inside `FieldSnapshot` (`PN/src/runtime/system.h:88-96`). The field worker produces a new snapshot each cycle and it is read under `snapshot_mutex_` (`system.cpp:291-294`, `375-409`).
- **Are all configured landmarks kept? Yes, when the AprilTag world estimator is used.** `LandmarkEstimator::run` seeds **every** `field_->landmarks` entry at its nominal pose: `kFieldMap`, confidence 0.5, valid (`PN/src/impl/world_estimation/landmark_estimator.cpp:47-61`).
  - Observed entries are kept in the odometry frame and tagged with their epoch.
  - An epoch change reverts an entry to nominal.
  - A re-anchor re-derives the field pose (62-86).
  - Evidence folding is at 88-165.
- **Noop world estimation** passes the previous field through unchanged (`PN/src/impl/noop/noops.cpp:37-44`). In practice the map stays empty. Publishing then reports none, and SELECT returns LandmarkUnsupported.
- **SELECT effect:**
  - It only sets `CommandState.object_requested/object_wire_id/object_sequence` (`command_state.h:52-54`).
  - The publisher maps the wire id to a `FieldObjectId` through its `<FieldObject object_id wire_id>` list (wire ids 1..255, unique both ways; `brain_link_publisher.cpp:112-143`) and reports **only that one** object.
  - `configured_targets` also acts on the `object_sequence` edge to latch a target (`PN/src/impl/target_resolution/configured_targets.cpp:115-122`). Targets never go on the wire.
- **No production pipeline has a `<FieldObject>` mapping.** `config/shared/pipelines/parallel_wheels_bno08x_camera.xml:9-12` says "None is configured yet, so such requests are refused as unknown." Mappings exist only in tests.
- **Publishing more than the selected landmark does not exist.** There is no way to send other landmark estimates, the list of landmarks, or the field map to the Brain.

---

## 6. What blocks a larger multi-frame transfer

1. **Frame caps.**
   - Request payload is at most **32**, so at most 24 body bytes per request (`frame_codec.cpp:56-59`).
   - Reply payload is at most **64**, so at most 51 body bytes. Each landmark entry in the current format is 16 bytes (id, source, x, y, heading, age), so at most 3 fit in one reply with no robot state.
   - Any frame is at most **128** bytes (`kMaxFrameLen`, which also sizes `FrameReader::buf_` at `frame_codec.h:94` and the publisher's `buf` at `brain_link_publisher.cpp:299`).
   - `ProsUsbLink::write` rejects anything over 128 (265). `finishLine` rejects more than 256 hex digits (214).
   - The `len` field is a u8.
2. **One request, one reply, per estimation cycle.**
   - `BrainReplyContext` holds one reply and is cleared on every run (`brain_link_commands.cpp:127-128`).
   - The publisher writes it once (`brain_link_publisher.cpp:268-305`). The Pi never sends unprompted.
   - The Brain keeps **at most one request outstanding** (`frames.h:70-71`, `interfaces.md:479`).
3. **Newest request wins per drain.** Earlier requests are counted as `superseded` and **never applied**. Trailing bytes suppress the reply. So a Brain cannot pipeline chunks.
4. **Dedupe memory is a single slot per op.** Only the last SET_POSE, the last SELECT and `newest_rid_` are remembered. A chunked write from Brain to Pi, such as a RobotProfile, would need its own per-chunk records, or chunks that are idempotent by (transfer id, index). A re-read of a GET-type chunk is only answered for `rid==newest_rid_`.
5. **Reply time budget.**
   - A 128-byte frame at 115200 baud takes about 11.1 ms of airtime; its budget is 13.1 ms including the 2 ms margin.
   - That is longer than the 10 ms period at 100 Hz and it blocks the estimation worker.
   - *(Calculation)* With the Brain's T=60 ms, a 128-byte reply leaves L ≤ about 6.4 ms, against 12.4 ms today.
   - On USB, the write backpressure bound is 5 ms and exceeding it **disconnects**.
6. **Rough throughput** *(estimate, labeled)*. One round trip is the request airtime, plus waiting up to one Pi loop period, plus the 1 ms guard, plus the reply airtime, plus the 5 ms Brain request gap. That is roughly 15-25 ms per chunk, or about 40-60 chunks/s, and it shares the slot with the 50 Hz state poll.
7. **The reply context is hard-coded.** `BrainReplyContext` (`command_state.h:27-39`) has fields only for today's five ops. `brainRequestLen`/`brainReplyLen` return 0 for unknown ops or results, which means header-only decode and an UnsupportedOp reply.
8. **USB read budget.** At most 1024 raw bytes per `readAvailable` call when nothing has been decoded yet, and a returned 0 ends the drain.
9. **The commands drain buffer** is 128 bytes, but it loops until empty, so it does not limit size.

---

## 7. Explicitly absent (verified by grep or reading)

- **No RobotProfile** type, op or struct exists anywhere under `PN/src`.
- **No runtime reconfiguration.**
  - `System` has only `buildFromFile` and `buildFromString` (`system.h:109-116`) plus `reset()`, which rebuilds nothing.
  - Wheel radius, CPR, inversion, offsets, directions, sensor ports, IMU source and camera mounts are all fixed in XML at build time.
  - `reset()` changes `pi_instance`, so the Brain session is lost.
- **No existing boundary for handing a profile to other stages.** The only data that Commands passes to another stage is `BrainImuBench`, a mailbox shared on the same worker thread.
- **The Pi does not transfer the field map or any other landmark** over the link.
- **The Pi has no Smart Port or IMU-port concept.**
- **`linux_serial_link` never reconnects.**
- **No `kMaxPayload` constant.**
- **No unprompted Pi output and no multi-frame replies.**
- **No USB input-pending refusal.**
- **No CRC on the NG1 line itself** (the inner frame CRC is the only check).

## 8. Doc drift and risks noticed
- `docs/interfaces.md` is out of date:
  - It still says "Four operations" (250). Its op table (274-279) and reply-body table (324-329) leave out op 5.
  - It describes only the RS-485 transport and does not document the NG1 USB envelope; `usb_localization_bench.md:109-114` describes it only in prose.
  - "largest v3 request is 26 bytes" is still correct.
- The `tcdrain` fallback in `transmitterEmpty` has no deadline and can overrun the reply budget, which would show up as `late_release`.
- Op 5 stamp-regression detection uses a raw u32 compare, so it would misfire when the counter wraps after about 49.7 days.
- A `closed` read part-way through a drain discards a request that had already been decoded in that drain.