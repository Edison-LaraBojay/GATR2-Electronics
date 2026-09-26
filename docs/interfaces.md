# Wire interface specification

This is the datasheet for the byte-level contract between the three devices.
It describes the framing, the frame types, the brain link protocol, and the
rules every encoder and parser must obey. The single source of truth in code is
`common/frames.h` with the codec in `common/frame_codec.h`. When this document
and the header disagree, the header wins and this document is the bug.

There are two links:

- Sensor link, Pico to Pi: one way, handshake-free. Every frame stands alone.
- Brain link v3, Brain and Pi: request/reply. The Brain sends a request, the Pi
  answers it with one reply or stays silent, and the Brain retries on timeout.
  Sessions and request ids make retries, reboots and delayed frames safe.

## Pipeline

```
                                                  request 0x10
sensors --> Pico ---------------------> Pi <------------------------ Brain --> motors
                  sensor frame 0x01        ------------------------>
                  one way, no reply               reply 0x11
                                           RS-485 half duplex, Brain initiates,
                                           one request outstanding
```

Sensor frames travel Pico to Pi on their own UART. Nothing is acknowledged or
retransmitted; a lost sensor frame is a gap in `seq`.

Brain link frames share one RS-485 half-duplex bus between the Pi HAT and a V5
smart port. The Brain is the only initiator. The Pi never sends unsolicited
output: it transmits only a reply to a request, inside a bounded window, and
listens otherwise. Every request gets an explicit result code or no reply at
all; there are no status-bit acknowledgements. See
[bus ownership and timing](#bus-ownership-and-timing).

Validation status: the codec, the Pi runtime and the Brain library are
implemented and host tested (`common/tests`, Navigatr and Brain host tests with
fake links and clocks). Nothing on the brain link is validated on hardware yet:
V5 smart port RS-485 direction handling, real turnaround timing and transmitter
empty detection on the Pi UART, the DE GPIO numbering and behavior, and every
latency figure below are unmeasured.

## Global rules

| Rule | Value |
|------|-------|
| Byte order (multi-byte fields) | Little-endian, least significant byte first |
| Numeric representation | Fixed-point integers only, no floats on the wire |
| Maximum frame length | 128 bytes (`kMaxFrameLen`), sync through checksum or CRC |
| Absence signaling | Structural (mask bit cleared, flag or source field), never a sentinel value |

Every multi-byte field is little-endian. Endianness is a per-field rule: the
bytes within one field are ordered least significant first, but the fields
themselves appear in the order listed. Single-byte fields have no byte order.

No value on the wire is a float. All physical quantities are fixed-point
integers in the units below. A parser reconstructs a field by reading its bytes
low to high, then interprets the integer in the field's unit.

### Fixed-point units

| Quantity | Unit on the wire | Meaning |
|----------|------------------|---------|
| Position (x, y) | mm | millimeters |
| Heading | cdeg | centidegrees, 1/100 of a degree, counterclockwise from field +x |
| Angular rate (gyro) | mdeg/s | millidegrees per second |
| Acceleration | mg | milli-g, 1/1000 of standard gravity |
| Encoder count | counts | raw quadrature counts; counts per revolution is sensor calibration |
| Age | ms | milliseconds, a difference of two Pi host clock values |

## Framing

Every frame starts with the sync pair and a type byte. The sensor frame and the
brain link frames then use different envelopes.

Sensor envelope (type 0x01), XOR checksum:

```
+--------+--------+------+------------------+-----+
| 0xAA   | 0x55   | type | type-specific    | xor |
| sync0  | sync1  |      | body             |     |
+--------+--------+------+------------------+-----+
```

Link envelope (types 0x10 and 0x11), explicit length and CRC:

```
+--------+--------+------+-----+-------------------+----------+
| 0xAA   | 0x55   | type | len | payload           | crc      |
| sync0  | sync1  | u8   | u8  | len bytes         | u16      |
+--------+--------+------+-----+-------------------+----------+
  0        1        2      3     4 .. 3+len          4+len
```

| Field | Size | Description |
|-------|------|-------------|
| sync0 | 1 | Constant `0xAA` (`kSync0`), marks a possible frame start |
| sync1 | 1 | Constant `0x55` (`kSync1`), confirms the frame start |
| type | 1 | Frame type, see the type registry |
| len | 1 | Link frames only: payload bytes, bounded per type |
| body or payload | varies | Layout determined by type |
| xor | 1 | Sensor frames: XOR of every preceding byte in the frame |
| crc | 2 | Link frames: CRC-16/CCITT-FALSE over type, len and payload |

A link frame is `len + 6` bytes (`kLinkEnvelopeLen`).

### Frame type registry

| type | Name | Direction | Constant | Payload len |
|------|------|-----------|----------|-------------|
| 0x01 | Sensor frame | Pico to Pi | `kFrameSensor` | from the mask |
| 0x10 | Brain request | Brain to Pi | `kFrameBrainRequest` | 8 to 32 |
| 0x11 | Brain reply | Pi to Brain | `kFrameBrainReply` | 13 to 64 |

Types 0x02 and 0x03 were the one-way pose and command frames. They are retired
and every current reader rejects them.

### Checksum (sensor frames)

The checksum is a single byte: the XOR of every byte from `sync0` up to but not
including the checksum itself.

```
xor = sync0 ^ sync1 ^ type ^ body[0] ^ body[1] ^ ... ^ body[n-1]
```

A receiver validates by XOR-ing the whole frame including the checksum byte. A
valid frame yields zero, because a value XOR-ed with itself cancels.

### CRC (link frames)

CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no input or output
reflection, final XOR 0. It covers `type`, `len` and the payload, not the sync
bytes, and is stored little-endian after the payload. The standard check value
is `crc16("123456789") = 0x29B1`. `crc16()` in the codec implements it.

A `len` outside its type's range rejects the frame as soon as the len byte is
read, before any payload arrives. A decoder also rejects a frame whose total
length is not `len + 6`.

### Loss of sync and recovery

A receiver that is not locked to a frame scans the incoming bytes for the pair
`0xAA 0x55`, then reads a candidate frame and checks its type, length, and
checksum or CRC. On any rejection (unknown type, invalid mask, len out of range,
bad checksum or CRC) the sync pair was coincidental data or the frame was
damaged. The receiver drops only that sync byte and rescans the bytes it has
already buffered for the next sync pair, so a damaged length cannot swallow the
frames behind it.

One rescan can find several complete frames at once. `FrameReader::push()`
reports the first; `FrameReader::next()` reports each further buffered frame
without a new byte.

Sensor frames carry no cross-frame state, so the Pico or Pi may reboot
mid-frame and the stream recovers on the next clean sync pair. Brain link
recovery after reboots is handled by sessions, below.

## Sensor frame (type 0x01)

Pico to Pi. Carries raw, timestamped sensor samples. Only the sensors present
this cycle appear in the payload.

```
+------+------+------+-----+-----------+--------+----------------+-----+
| 0xAA | 0x55 | 0x01 | seq | stamp_ms  | mask   | payload        | xor |
|      |      |      | u8  | u32       | u16    | present sensors|     |
+------+------+------+-----+-----------+--------+----------------+-----+
  0      1      2      3     4..7        8..9     10..              last
```

| Offset | Field | Type | Size | Description |
|--------|-------|------|------|-------------|
| 0 | sync0 | u8 | 1 | `0xAA` |
| 1 | sync1 | u8 | 1 | `0x55` |
| 2 | type | u8 | 1 | `0x01` |
| 3 | seq | u8 | 1 | Sequence counter, increments per frame, wraps 255 to 0 |
| 4 | stamp_ms | u32 | 4 | Pico clock in milliseconds at sample time |
| 8 | mask | u16 | 2 | Present-sensor bitfield, see the sensor bit table |
| 10 | payload | varies | varies | Present sensors only, packed in ascending bit order |
| last | xor | u8 | 1 | Frame checksum |

`seq` lets the receiver detect dropped frames (a gap in the count) and duplicates.
It counts frames, not time.

`stamp_ms` is the Pico sample clock. The Pi separately records host receipt time
and maps accepted localization intervals into the host clock for camera/history
lookup. These time domains must not be compared directly. The u32 field wraps
after about 49.7 days; device restarts also require source-epoch handling.

### Sensor mask and payload

`mask` declares which sensors are present. Each bit maps to one sensor. A set bit
means that sensor's bytes are in the payload; a cleared bit means its bytes are
absent, not zero and not a sentinel. The payload contains the present sensors
back to back, in ascending bit order. A parser walks the mask from bit 0 upward,
and for each set bit copies that sensor's width from the width table.

| Bit | Constant | Sensor | Width (bytes) | Type | Unit |
|-----|----------|--------|---------------|------|------|
| 0 | `kSensorEnc0` | Encoder channel 0 | 4 | i32 | counts |
| 1 | `kSensorEnc1` | Encoder channel 1 | 4 | i32 | counts |
| 2 | `kSensorEnc2` | Encoder channel 2 | 4 | i32 | counts |
| 3 | `kSensorGyroZ` | Yaw rate | 4 | i32 | mdeg/s |
| 4 | `kSensorAccelXY` | Acceleration x and y (reserved) | 8 | 2 x i32 | mg |

The width column is `kSensorWidth` in the header. It is the only table a parser
needs; the parser walks the payload by width and never interprets sensor meaning.
`kSensorBitCount` (currently 5) equals the number of width entries, and a test
guards that they stay in step.

Channel orientation and placement belong to the Pi's configured wheel geometry.
Gyro rate has bias left in it; the Pi's localization observation model removes
that bias. With BNO08X firmware, `gyro_z` is the three-axis uncalibrated gyro
projected onto the up axis learned during a stationary, level startup. It is
counterclockwise-positive yaw in the same mdeg/s wire field. The gyro bit stays
clear until alignment completes and both acceleration and gyro reports are
fresh. The startup axis remains fixed until an IMU reset. ASM330 firmware sends
the physical Z-axis rate. Neither path sends live pitch/roll; BNO acceleration
is used internally for alignment and does not populate the reserved accel bit.

Adding a sensor bit requires updating the shared width table, encoder/decoder,
firmware producer, and consumers together. The codec rejects unknown mask bits:
an old parser cannot skip a payload whose width it does not know.

### Sensor frame example

Encoders 0 and 1 and the gyro present. Encoder 2 and accel absent.

| Field | Value | Bytes on the wire |
|-------|-------|-------------------|
| seq | 7 | `07` |
| stamp_ms | 1000 | `E8 03 00 00` |
| mask | 0x000B (bits 0, 1, 3) | `0B 00` |
| enc0 | 1000 counts | `E8 03 00 00` |
| enc1 | -500 counts | `0C FE FF FF` |
| gyro_z | 2500 mdeg/s | `C4 09 00 00` |

Full frame (23 bytes):

```
AA 55 01 07 E8 03 00 00 0B 00 E8 03 00 00 0C FE FF FF C4 09 00 00 CD
```

The trailing `CD` is the XOR of the preceding 22 bytes.

## Brain link v3

`kBrainLinkVersion = 3`. The Brain asks, the Pi answers. Four operations:
open a session (HELLO), place the robot (SET_POSE), select a landmark
(SELECT_LANDMARK), and read the current state (GET_STATE).

### Request (type 0x10, Brain to Pi)

Payload offsets below count from the first payload byte (frame offset 4).

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | version | u8 | 3 |
| 1 | op | u8 | Operation, see the op table |
| 2 | session | u32 | Session from the HELLO reply; 0 in HELLO |
| 6 | request_id | u16 | Brain transaction id, never 0 |
| 8 | body | varies | Layout determined by op |

The header offsets (version through request_id) are frozen for every future
version.

`request_id` is a per-boot counter on the Brain: it starts at 1 and wraps from
65535 to 1, never 0. Each new transaction takes the next id. A retry resends the
byte-identical frame (same session, id, op and body). GET_STATE is never
retried; each poll is a new id.

| op | Constant | Body | Payload len |
|----|----------|------|-------------|
| 1 | `kOpHello` | nonce u32 | 12 |
| 2 | `kOpSetPose` | x_mm i32, y_mm i32, heading_cdeg i32 | 20 |
| 3 | `kOpSelectLandmark` | landmark_id u8, flags u8 | 10 |
| 4 | `kOpGetState` | none | 8 |

- HELLO `nonce`: fresh for every HELLO transaction (not per retry), mixed from
  several entropy sources on the Brain. It identifies the opening attempt.
- SET_POSE: the robot origin's field pose, heading counterclockwise from +x.
- SELECT_LANDMARK `flags` bit 0 (`kSelectFlagSelected`): set selects
  `landmark_id`, clear releases the selection.

Decoding (`decodeBrainRequest`): the frame must pass the envelope, len and CRC
checks and carry at least the 8-byte header. A version other than 3 decodes the
header only and succeeds, so the Pi can answer it. For version 3, a known op
with the wrong payload length fails; an unknown op decodes the header only and
succeeds, so the Pi can answer `kResultUnsupportedOp`.

### Reply (type 0x11, Pi to Brain)

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | version | u8 | The Pi's version, 3 |
| 1 | op | u8 | Echo of the request |
| 2 | session | u32 | Echo of the request; for HELLO Ok the opened (or retried) session |
| 6 | request_id | u16 | Echo of the request |
| 8 | result | u8 | Result code |
| 9 | pi_instance | u32 | Random nonzero id of this Pi runtime |
| 13 | body | varies | Layout determined by op and result |

These 13 header bytes are frozen for every future version, and
`decodeBrainReply` decodes them for any version (the body only for version 3).
`pi_instance` changes on every Navigatr process start and every
`System::reset()`, so a Brain can tell that the Pi restarted.

| Value | Constant | Meaning |
|-------|----------|---------|
| 0 | `kResultOk` | Done. For SET_POSE: localization has applied the placement |
| 1 | `kResultPending` | SET_POSE accepted in this session, not applied yet |
| 2 | `kResultUnknownSession` | The session is not the Pi's current session |
| 3 | `kResultUnsupportedVersion` | Version mismatch; the reply header carries the Pi's version |
| 4 | `kResultUnsupportedOp` | Op unknown to this Pi |
| 5 | `kResultInvalidArgument` | request_id 0, or a reused id with a different op or body |
| 6 | `kResultUnknownLandmark` | SELECT of an id with no configured mapping |
| 7 | `kResultLandmarkUnsupported` | SELECT while world estimation is noop, so no estimate can exist |
| 8 | `kResultStale` | Older request id that matches no recorded request, or HELLO reusing a recent nonce |

Reply bodies. Every (op, result) pair not listed has no body.

| op | result | Body | Payload len |
|----|--------|------|-------------|
| HELLO | every result | nonce u32, echo | 17 |
| SET_POSE | Ok, Pending | odometry_epoch u32, anchor_revision u32, current values | 21 |
| SELECT_LANDMARK | Ok | landmark_id u8, flags u8, echo | 15 |
| GET_STATE | Ok | state block, 40 bytes | 53 |

A version 3 reply for a known (op, result) with the wrong payload length fails
to decode. An unknown op or result decodes the header only.

### State block (GET_STATE Ok)

Offsets count from the start of the block (payload offset 13).

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | robot_flags | u8 | See the robot flag table |
| 1 | x_mm | i32 | Robot origin x, field frame |
| 5 | y_mm | i32 | Robot origin y, field frame |
| 9 | heading_cdeg | i32 | Robot heading, counterclockwise from field +x, normalized to (-18000, 18000] |
| 13 | robot_age_ms | u16 | Pi cycle time minus the pose's measurement host time, clamped to 0..65535 |
| 15 | odometry_epoch | u32 | Low 32 bits of the Pi's odometry epoch |
| 19 | anchor_revision | u32 | Low 32 bits of the Pi's field anchor revision |
| 23 | health | u8 | See the health table |
| 24 | landmark_id | u8 | The session's selected landmark, 0 when none |
| 25 | landmark_source | u8 | 0 none, 1 nominal, 2 observed |
| 26 | lm_x_mm | i32 | Landmark x, field frame |
| 30 | lm_y_mm | i32 | Landmark y, field frame |
| 34 | lm_heading_cdeg | i32 | Landmark full field heading |
| 38 | landmark_age_ms | u16 | Observed only: Pi cycle time minus the last observation time, clamped; 0 otherwise |

| Bit | Constant | Set when |
|-----|----------|----------|
| 0 | `kRobotPoseValid` | An estimator has produced a pose |
| 1 | `kRobotLocalized` | The field anchor was set by a placement |
| 2 | `kRobotAgeKnown` | `robot_age_ms` is meaningful (the pose has a host measurement time) |
| 3 | `kRobotAnchorCommand` | The anchor came from a Brain SET_POSE (any session of this `pi_instance`) |
| 4 | `kRobotAnchorConfigured` | The anchor came from the Pi's configured `<InitialPlacement>` |

| Bit | Constant | Set when |
|-----|----------|----------|
| 0 | `kHealthEncodersFresh` | All configured encoder health sources are valid and fresh |
| 1 | `kHealthGyroFresh` | The configured gyro health source is valid and fresh |
| 2 | `kHealthVisionAlive` | The current field snapshot's observation map is nonempty |
| 3 | `kHealthBiasCalibrated` | The configured bias-calibration observation function has reported ready |

| Value | Constant | Meaning |
|-------|----------|---------|
| 0 | `kLandmarkSourceNone` | No usable estimate; landmark pose fields carry no meaning |
| 1 | `kLandmarkSourceNominal` | Configured map pose, never labeled observed |
| 2 | `kLandmarkSourceObserved` | Vision estimate from the current odometry epoch |

Semantics:

- Robot and landmark poses are expressed under the same field anchor. The
  landmark fields are always the physical landmark pose, never a resolved robot
  destination. The Brain applies its own offset exactly once
  (`T_field_destination = T_field_landmark * T_landmark_destination`).
  Configured targets are internal to the Pi (inspection only) and never go on
  the wire.
- `landmark_source` is none whenever no usable estimate exists: world
  estimation is noop, nothing is selected, the entry is missing or invalid, or
  an observed entry belongs to another odometry epoch.
- `odometry_epoch` increments when the Pi's local odometry frame becomes
  discontinuous. `anchor_revision` increments on every field re-anchor. Frame
  identity for the Brain is (`pi_instance`, `session`, `odometry_epoch`,
  `anchor_revision`); any change is a coordinate discontinuity.
- Ages are differences of Pi host clock values only. No Brain, Pi or Pico clock
  value is ever subtracted from another device's clock. The Brain adds its own
  measured round trip and its time since receipt. The Pi takes ages at its cycle
  start, before it reads the request, so the Brain's sum is an upper bound only
  to within the Pi cycle processing time (a few ms).
- Status, health and validity bits are information, never acknowledgements,
  and they do not identify a session. Results are the only acknowledgements.

### Sessions

A session is one Brain application run, opened by HELLO. Duplicate detection,
landmark selection and placement acknowledgement are all scoped to the session.

Pi behavior:

- HELLO
  - Same nonce and request_id as the current session's opening, and nothing
    else accepted in that session yet: a retry. Answer Ok with the same
    session; change nothing.
  - Otherwise a nonce among the last 4 opening nonces (including the current
    one): answer `kResultStale`; change nothing.
  - Otherwise open a new session: a random nonzero u32 different from the
    previous one. The previous session's client state is cleared: the landmark
    selection is released, any session-owned target latch is cleared, and the
    duplicate records are dropped. HELLO never touches localization or a
    placement already applied. Answer Ok with the new session.
- Any other op whose session is not the current one (or when no session is
  open): `kResultUnknownSession`.
- `request_id` 0: `kResultInvalidArgument`.
- A newer request_id (no newest yet, or `(rid - newest) mod 65536` in
  1..32767) is a new request: apply it, record it, and make it the newest.
- A request_id that is not newer is never applied again:
  - equal to the last SET_POSE or SELECT_LANDMARK record with the same op and
    body: answered as a duplicate of that record;
  - equal to the newest with op GET_STATE: answered with fresh state;
  - same id with a different op or body: `kResultInvalidArgument`;
  - anything else: `kResultStale`.
- SET_POSE (new): the Pi requests the placement tagged with (command, session,
  sequence). The reply is Ok once localization reports that exact placement
  applied, otherwise Pending; a duplicate is answered the same way. Placement
  identity includes the session, so a genuine SET_POSE after a Brain reboot
  applies even with the same request_id and pose. A new session withdraws a
  placement that was not applied yet.
- SELECT_LANDMARK (new): records the selection (or the release). The reply is
  Ok for a release, `kResultLandmarkUnsupported` when world estimation is noop,
  `kResultUnknownLandmark` when the id has no mapping, otherwise Ok. The
  selection is recorded either way; the state block then reports source none.
- GET_STATE: no state change. Ok with the state block.
- Another version: `kResultUnsupportedVersion`. Unknown op:
  `kResultUnsupportedOp`.

Brain behavior:

- Correlate first: accept a reply only if its op and request_id match the
  outstanding request and, for HELLO, the nonce matches, or, for other ops, the
  session matches the one the request was sent in. Everything else is dropped
  and counted.
- Only a correlated reply can signal a Pi restart: a `pi_instance` different
  from the one recorded at HELLO Ok.
- On any session change (`kResultUnknownSession` for the current session, a
  `pi_instance` change, or a new HELLO): invalidate cached input, fail every
  in-flight state-changing request as session lost without resending it, clear
  the acknowledged selection, then HELLO with a fresh nonce. `kResultStale` on
  HELLO also means a fresh nonce.
- Ready means a session is open and at least one GET_STATE in it returned Ok.
- A SET_POSE Ok records the reply's (`odometry_epoch`, `anchor_revision`). The
  placement counts as applied only when a later GET_STATE reports that anchor,
  so no pre-placement pose is used after it.
- Placement and selection are retried with the same bytes, bounded by attempts
  and a deadline. `kResultUnsupportedVersion` and `kResultUnsupportedOp` are
  terminal errors, not retry storms.

Consequences:

- Brain reboot: the new boot opens a new session. Delayed requests from the old
  session get `kResultUnknownSession` and change nothing. Delayed replies to the
  old session fail correlation. The robot pose continues; a new session alone
  never relocates the robot, and no old selection or movement carries over.
- Pi restart or reset: `pi_instance` changes and the old session is unknown.
  The Brain opens a new session and never resends an in-flight placement.

The Brain-side implementation is `brain/communiGATR`, described in
[communiGATR](communigatr.md).

### Bus ownership and timing

Rules:

- The Brain is the only initiator and has at most one request outstanding.
- Before each request the Brain drains its receive buffer and resets its frame
  reader; anything received earlier is stale. Its response timeout starts when
  its write call returns. After a reply or a timeout it waits `request_gap`
  before the next request.
- The Pi reads the link once per estimation cycle, draining until empty, and
  timestamps every read with the link's own steady microsecond clock.
- Reply window. A request completed in drain k was provably incomplete at the
  last read of drain k-1, `r(k-1)`. The Pi may start its reply only while
  `now <= r(k-1) + reply_window` and not before `completion_read +
  turnaround_guard`. The check runs inside the serial link immediately before
  the driver enable (DE) is asserted. A request completed in the first drain
  since open, start or reset has no `r(k-1)` and gets no reply.
- No reply either when more bytes followed the processed request in the same
  drain (the Brain may be transmitting), or when the link reports input pending
  just before DE (counted).
- Newest request wins: if one drain completes several requests, only the newest
  is processed; the others are counted and never applied. A buffered partial
  frame is discarded when a whole drain returns no new bytes, because requests
  are sent in one burst.
- The Pi asserts DE only for its reply, waits until the transmitter is empty,
  holds a short post guard (default 2 character times), then releases DE. Every
  error or timeout path releases DE and discards unsent output. The Pi transmit
  timeout is `bytes * 10 / baud + 2 ms`.

| Name | Side | Default |
|------|------|---------|
| response timeout `T` | Brain | 60 ms |
| request gap | Brain | 5 ms |
| GET_STATE poll period | Brain | 20 ms |
| link timeout (connected) | Brain | 0.25 s |
| reply window `W` | Pi | 40 ms |
| turnaround guard | Pi | 1000 us |
| baud | both | 115200 |

The defaults are starting points, not measurements.

Budget. Let

- `A_req` = request airtime after the Brain's write returns; the largest v3
  request is 26 bytes, 2.3 ms at 115200 baud
- `A_rep` = reply airtime; the largest v3 reply is 59 bytes, 5.1 ms
- `R` = DE release after the last stop bit (post guard 0.17 ms plus the
  transmitter-empty poll)
- `L` = V5 transmit latency plus Pi UART and tty receive latency plus margin

The Pi's `r(k-1)` precedes the arrival of the request's last byte, so the Pi
never starts a reply later than `W + L` after the request ends on the wire, and
the reply is off the bus `A_rep + R` after that. A retry can therefore never
overlap a late reply when

```
T >= A_req + W + A_rep + R + L
60 ms >= 2.3 + 40 + 5.1 + 0.2 + L    (L up to 12.4 ms)
```

The request gap adds 5 ms of further margin. Residual risks: the Pi can be
preempted between the window check and asserting DE, and DE release can be
delayed by preemption after transmit completion (counted as `late_release`).

Pi configuration constraints:

- A profile with brain_link commands must satisfy
  `1000 / loop_rate_hz <= reply_window_ms / 2`, otherwise the build fails, so
  one full loop period plus processing fits the window.
- The brain link requires Navigatr's threaded mode. `--inline` (and `step()`)
  runs world estimation between the command read and the reply; the executable
  warns when `--inline` is used with the brain link.
- The reply write blocks the Pi estimation worker for about the reply airtime
  (5 ms for 59 bytes at 115200).
- The V5 smart port is assumed to switch its own RS-485 direction. This is a
  hardware assumption and is not validated.

Pi configuration and wiring: [Navigatr setup](../pi/navigatr/docs/setup.md) and
[hardware](hardware.md).

### Brain link examples

Values used: session `0xA1B2C3D4`, pi_instance `0x0BADF00D`, nonce
`0x12345678`. These frames are the known-byte vectors in
`common/tests/frame_codec_gtest.cpp`, checked against an independent CRC.

GET_STATE request, request_id 65535:

| Field | Value | Bytes on the wire |
|-------|-------|-------------------|
| sync, type, len | request, 8 | `AA 55 10 08` |
| version, op | 3, GET_STATE | `03 04` |
| session | 0xA1B2C3D4 | `D4 C3 B2 A1` |
| request_id | 65535 | `FF FF` |
| crc | 0xA186 over `10 08 .. FF FF` | `86 A1` |

GET_STATE Ok reply, state block: flags PoseValid, Localized, AgeKnown,
AnchorCommand (0x0F); robot (1500, -250, 18000); robot age 35 ms; epoch 2;
anchor revision 7; health 0x0B; landmark 3 observed at (2000, 1000, -4500);
landmark age 120 ms.

```
Requests
HELLO rid 1                AA 55 10 0C 03 01 00 00 00 00 01 00 78 56 34 12 E6 29
SET_POSE rid 2             AA 55 10 14 03 02 D4 C3 B2 A1 02 00 62 02 00 00 37 FE FF FF
  (610, -457, -9000)       D8 DC FF FF C5 85
SELECT rid 3, id 5 on      AA 55 10 0A 03 03 D4 C3 B2 A1 03 00 05 01 D1 3E
GET_STATE rid 65535        AA 55 10 08 03 04 D4 C3 B2 A1 FF FF 86 A1

Replies
HELLO Ok                   AA 55 11 11 03 01 D4 C3 B2 A1 01 00 00 0D F0 AD 0B 78 56 34
                           12 BB 6D
HELLO Stale                AA 55 11 11 03 01 00 00 00 00 01 00 08 0D F0 AD 0B 78 56 34
                           12 CB DD
SET_POSE Ok (2, 7)         AA 55 11 15 03 02 D4 C3 B2 A1 02 00 00 0D F0 AD 0B 02 00 00
                           00 07 00 00 00 F0 26
SET_POSE Pending (2, 6)    AA 55 11 15 03 02 D4 C3 B2 A1 02 00 01 0D F0 AD 0B 02 00 00
                           00 06 00 00 00 27 15
SET_POSE UnknownSession    AA 55 11 0D 03 02 D4 C3 B2 A1 02 00 02 0D F0 AD 0B A1 8C
SELECT Ok                  AA 55 11 0F 03 03 D4 C3 B2 A1 03 00 00 0D F0 AD 0B 05 01 22
                           A4
GET_STATE Ok               AA 55 11 35 03 04 D4 C3 B2 A1 FF FF 00 0D F0 AD 0B 0F DC 05
                           00 00 06 FF FF FF 50 46 00 00 23 00 02 00 00 00 07 00 00 00
                           0B 03 02 D0 07 00 00 E8 03 00 00 6C EE FF FF 78 00 FE 2C
```

## Structs are not the wire

The structs in `frames.h` (`SensorSample`, `BrainRequest`, `BrainReply`,
`BrainState`) are in-memory conveniences. Their byte layout differs from the
wire because of padding, and the request and reply structs hold the body fields
of every op. Only the codec maps between structs and bytes. Never copy a struct
onto the wire, and never overlay a struct on received bytes.

## Versioning

Brain link frames carry their version in the first payload byte. The request
header (8 bytes) and reply header (13 bytes) keep their offsets in every future
version, so any version can decode enough to correlate a reply and report a
mismatch. A Pi answers a request of another version with
`kResultUnsupportedVersion` in its own version; a Brain treats that result as a
terminal error. The type bytes 0x10 and 0x11 and the length bounds are shared by
all versions.

The sensor frame has no version byte. A frame log or capture is tied to the
format it was recorded under. When a field width or layout changes, older
captures may no longer decode. Record the format version alongside any stored
capture so a decoder can reject or adapt to mismatches.

## Language parity

The Pico firmware, the Pi runtime and the Brain library
(`brain/communiGATR`) share `common/frames.h` and the C++ codec
`common/frame_codec.cpp`; it builds as C++17 on the host and as gnu++20 in the
PROS build. The RS-485 bench programs exercise raw text transfer and are not
implementations of these frames.
