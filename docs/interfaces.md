# Wire interface specification

This is the datasheet for the byte-level contract between the Pico, the Pi and
the Brain: framing, frame types, the Pico control frames, brain link v4 and the
documents it moves in chunks. The source of truth in code is `translaGATR/frames.h`,
with the codec in `translaGATR/frame_codec.h` and the documents in
`translaGATR/link_documents.h`. When this document and the headers disagree, the
headers win and this document is the bug.

There are three exchanges:

- Sensor link, Pico to Pi: one way, handshake free. Every sensor frame stands
  alone. v2 frames carry the Pico's acquisition identity.
- Pico control, Pi to Pico on the same UART pair: the Pi sends commands, the
  Pico reports their progress only in periodic status frames. Optional
  diagnostic frames (0x14) flow only after a Pi asked the current Pico boot
  for them.
- Brain link v4, Brain and Pi: request/reply over RS-485 or the V5 USB console.
  The Brain sends a request, the Pi answers it with one reply or stays silent,
  and the Brain retries on timeout. Sessions and request ids make retries,
  reboots and delayed frames safe.

## Pipeline

```
        sensor 0x01/0x04, status 0x13, diag 0x14           request 0x10
sensors --> Pico ----------------------------> Pi <--------------------------- Brain --> motors
                 <----------------------------    --------------------------->
                         command 0x12                      reply 0x11
               Pico UART, 115200                RS-485 half duplex, or V5 USB NG1 lines;
                                                Brain initiates, one request outstanding
```

Sensor frames travel Pico to Pi on their own UART. Nothing is acknowledged or
retransmitted; a lost sensor frame is a gap in `seq`. Pico commands and status
frames share that UART pair.

The Brain is the only initiator on the brain link. The Pi never sends
unsolicited output: it transmits only a reply to a request, inside a bounded
window, and listens otherwise. Every request gets an explicit result code or no
reply at all; status bits are never acknowledgements. See
[bus ownership and timing](#bus-ownership-and-timing).

Validation status: host tests only. The codec is tested in `translaGATR/tests`
with independently packed known-byte vectors, the Brain library against a fake
Pi in `brain/communiGATR/tests`, and the Pi and Pico sides in their own host
suites. Nothing here is validated on hardware: V5 smart port RS-485 direction
handling, the V5 USB console under load and across unplugging, turnaround and
transmitter empty timing on the Pi UART, the DE GPIO, the Pico command wire on
the HAT, and every latency figure below are unmeasured.

## Global rules

| Rule | Value |
|------|-------|
| Byte order (multi-byte fields) | Little endian, least significant byte first |
| Numeric representation | Fixed-point integers only, no floats on the wire |
| Maximum frame length | 128 bytes (`kMaxFrameLen`), sync through checksum or CRC |
| Absence signaling | Structural (mask bit cleared, flag, count or source field), never a sentinel value |
| Reserved bytes and bits | Written 0; a document with a nonzero reserved byte is rejected |

Every multi-byte field is little endian. Endianness is a per-field rule: the
bytes within one field are ordered least significant first, but the fields
appear in the order listed. Single-byte fields have no byte order.

### Fixed-point units

| Quantity | Unit on the wire | Meaning |
|----------|------------------|---------|
| Position (pose, field map) | mm | millimeters |
| Position (robot profile) | um | micrometers |
| Heading | cdeg | centidegrees, CCW from field +x |
| Angle (robot profile) | mdeg | millidegrees, CCW |
| Rotation (Brain bench IMU) | mdeg | continuous, unwrapped, CCW |
| Angular rate (gyro) | mdeg/s | millidegrees per second |
| Angular rate (calibration setting) | cdeg/s | centidegrees per second |
| Acceleration | mg | milli-g |
| Encoder count | counts | raw quadrature counts |
| Ratio (gear, travel scale) | x 1e6 | `gear_micro`, `travel_scale_ppm`; 1000000 is 1.0 |
| Age | ms | difference of two Pi host clock values |
| Stamp | ms | one device's own clock; never compared across devices |

Frames: field frame origin at the inside bottom-left corner in the audience
view, +x right, +y toward the 0 degree wall, heading CCW from +x. Robot frame:
+x forward, +y left, origin the reported robot point.

## Framing

Every frame starts with the sync pair and a type byte. Sensor frames and link
frames then use different envelopes.

Sensor envelope (types 0x01 and 0x04), XOR checksum:

```
+--------+--------+------+------------------+-----+
| 0xAA   | 0x55   | type | type-specific    | xor |
| sync0  | sync1  |      | body             |     |
+--------+--------+------+------------------+-----+
```

Link envelope (types 0x10 to 0x14), explicit length and CRC:

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
| type | 1 | Frame type, see the registry |
| len | 1 | Link frames only: payload bytes, bounded per type |
| body or payload | varies | Layout determined by type |
| xor | 1 | Sensor frames: XOR of every preceding byte in the frame |
| crc | 2 | Link frames: CRC-16/CCITT-FALSE over type, len and payload |

A link frame is `len + 6` bytes (`kLinkEnvelopeLen`).

### Frame type registry

| type | Name | Direction | Constant | Payload len |
|------|------|-----------|----------|-------------|
| 0x01 | Sensor frame v1 | Pico to Pi | `kFrameSensor` | from the mask |
| 0x04 | Sensor frame v2 | Pico to Pi | `kFrameSensorV2` | from the mask |
| 0x10 | Brain request | Brain to Pi | `kFrameBrainRequest` | 8 to 122 |
| 0x11 | Brain reply | Pi to Brain | `kFrameBrainReply` | 13 to 122 |
| 0x12 | Pico command | Pi to Pico | `kFramePicoCommand` | 6 to 8 |
| 0x13 | Pico status | Pico to Pi | `kFramePicoStatus` | 20 |
| 0x14 | Pico diagnostic | Pico to Pi, only after DIAGNOSTICS | `kFramePicoDiag` | 26 |

Types 0x02 and 0x03 were the one-way pose and command frames. They are retired
and every current reader rejects them.

### Checksum (sensor frames)

The XOR of every byte from `sync0` up to but not including the checksum
itself. A receiver validates by XOR-ing the whole frame including the checksum
byte; a valid frame yields zero.

### CRC-16 (link frames)

CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection,
final XOR 0. It covers `type`, `len` and the payload, not the sync bytes, and
is stored little endian after the payload. Check value:
`crc16("123456789") = 0x29B1`.

A `len` outside its type's range rejects the frame as soon as the len byte is
read, before any payload arrives. A decoder also rejects a frame whose total
length is not `len + 6`.

### CRC-32 (documents)

CRC-32/ISO-HDLC (zlib): reflected polynomial 0xEDB88320, initial value and
final XOR 0xFFFFFFFF. Check value: `crc32("123456789") = 0xCBF43926`. It
identifies whole documents: `profile_id` and `map_id` are the CRC-32 of the
document bytes, and every READ_DOC chunk carries the CRC-32 of its whole
document.

### Loss of sync and recovery

A receiver that is not locked to a frame scans for `0xAA 0x55`, then reads a
candidate frame and checks its type, length, and checksum or CRC. On any
rejection (unknown type, invalid mask, len out of range, bad checksum or CRC)
it drops only that sync byte and rescans the bytes it has already buffered, so
a damaged length cannot swallow the frames behind it.

One rescan can find several complete frames at once. `FrameReader::push()`
reports the first; `FrameReader::next()` reports each further buffered frame
without a new byte. `FrameReader` accepts all seven types; each consumer then
ignores the types that are not meant for it.

`FrameReader::stats()` counts what the reader saw (`FrameReaderStats`, never
cleared by `reset()`):

| Counter | Counts |
|---------|--------|
| `bytes` | every byte pushed |
| `frames` | every valid frame reported, by `push()` or `next()` |
| `sync_dropped` | every discarded byte, including the sync byte of a rejected candidate; the rest of a rejected candidate is rescanned, not counted |
| `length_errors` | rejected candidates with an impossible length, an unknown type or an invalid sensor mask |
| `check_errors` | rejected candidates failing their checksum or CRC |

Without a `reset()`, `bytes` = bytes of reported frames + `sync_dropped` +
bytes still buffered. The error counters count candidates, not bytes; line
noise that looks like a sync pair can therefore raise them.

Sensor frames carry no cross-frame state, so the Pico or Pi may reboot
mid-frame and the stream recovers on the next clean sync pair. Brain link
recovery after reboots is handled by sessions.

## Sensor frames (types 0x01 and 0x04)

Pico to Pi. Raw, timestamped sensor samples; only the sensors present this
cycle appear in the payload. v2 adds the acquisition identity before the mask.
Current Pico firmware sends v2; the Pi still decodes v1 from older firmware.

v1:

```
+------+------+------+-----+-----------+--------+----------------+-----+
| 0xAA | 0x55 | 0x01 | seq | stamp_ms  | mask   | payload        | xor |
|      |      |      | u8  | u32       | u16    | present sensors|     |
+------+------+------+-----+-----------+--------+----------------+-----+
  0      1      2      3     4..7        8..9     10..              last
```

v2:

```
+------+------+------+-----+----------+---------+-----------+-----------+--------+---------+-----+
| 0xAA | 0x55 | 0x04 | seq | stamp_ms | boot_id | acq_epoch | imu_epoch | mask   | payload | xor |
|      |      |      | u8  | u32      | u16     | u8        | u8        | u16    |         |     |
+------+------+------+-----+----------+---------+-----------+-----------+--------+---------+-----+
  0      1      2      3     4..7       8..9      10          11          12..13   14..      last
```

| Field | Type | Description |
|-------|------|-------------|
| seq | u8 | Frame counter, wraps 255 to 0; a gap is a lost frame |
| stamp_ms | u32 | Pico clock in ms at sample time; wraps after about 49.7 days |
| boot_id | u16 | v2: random, nonzero, new at every Pico boot |
| acq_epoch | u8 | v2: explicit acquisition restarts in this boot (counters zeroed) |
| imu_epoch | u8 | v2: IMU initializations and reinitializations in this boot |
| mask | u16 | Present-sensor bitfield |
| payload | varies | Present sensors, packed in ascending bit order |
| xor | u8 | Frame checksum |

`stamp_ms` is the Pico sample clock. The Pi records host receipt time
separately and maps accepted localization intervals into the host clock.
These time domains are never compared directly.

### Acquisition identity (v2)

A change of any identity field means the matching measurements are not
continuous with earlier ones. Epochs are compared for inequality only; they
wrap.

| Change | Meaning | Pi response |
|--------|---------|-------------|
| `boot_id` | Pico rebooted; counters restarted | New device epoch: encoders start a new baseline, so counts back at zero never read as motion; IMU restarts |
| `acq_epoch` | RESTART_ACQUISITION zeroed the counters | Same as a reboot for the encoders |
| `imu_epoch` | IMU (re)initialized | IMU output only: new gyro epoch and bias recalibration; encoders stay continuous |

v1 frames carry no identity. The Pi then detects a Pico restart from stamp
regression only, and flags that fallback in its diagnostics. Stamp regression
cannot see every restart, which is why v2 exists.

### Sensor mask and payload

A set bit means that sensor's bytes are in the payload; a cleared bit means
they are absent, not zero and not a sentinel. A parser walks the mask from bit
0 upward and, for each set bit, copies that sensor's width.

| Bit | Constant | Sensor | Width | Type | Unit |
|-----|----------|--------|-------|------|------|
| 0 | `kSensorEnc0` | Encoder port 0 (HAT v2 A0/B0, J2) | 4 | i32 | counts |
| 1 | `kSensorEnc1` | Encoder port 1 (A1/B1, J3) | 4 | i32 | counts |
| 2 | `kSensorEnc2` | Encoder port 2 (A2/B2, J4) | 4 | i32 | counts |
| 3 | `kSensorGyroZ` | Yaw rate of Pico IMU port 0 | 4 | i32 | mdeg/s |
| 4 | `kSensorAccelXY` | Acceleration x and y (reserved) | 8 | 2 x i32 | mg |

The width column is `kSensorWidth`, the only table a parser needs.
`kSensorBitCount` (5) equals its length. The codec rejects unknown mask bits:
an old parser cannot skip a payload whose width it does not know. Adding a bit
means updating the width table, codec, firmware and consumers together.

What an encoder port measures (radius, direction, counts per revolution,
polarity) is set by the Brain robot profile on the Pi, not here. Gyro rate has
its bias left in; the Pi removes it once, in the observation model. With
BNO08X firmware, `gyro_z` is the three-axis uncalibrated gyro projected onto
the up axis learned during a stationary, level startup; the bit stays clear
until alignment completes. The startup axis stays fixed until an IMU
reinitialization, so later rocking is not compensated. ASM330 firmware sends
the physical Z rate. Neither sends live pitch or roll.

### Sensor frame examples

v1, encoders 0 and 1 and the gyro present:

| Field | Value | Bytes |
|-------|-------|-------|
| seq | 7 | `07` |
| stamp_ms | 1000 | `E8 03 00 00` |
| mask | 0x000B (bits 0, 1, 3) | `0B 00` |
| enc0 | 1000 | `E8 03 00 00` |
| enc1 | -500 | `0C FE FF FF` |
| gyro_z | 2500 mdeg/s | `C4 09 00 00` |

```
v1 (23 bytes)  AA 55 01 07 E8 03 00 00 0B 00 E8 03 00 00 0C FE FF FF C4 09 00 00 CD
v2 (27 bytes)  AA 55 04 09 04 03 02 01 EF BE 02 05 0B 00 E8 03 00 00 0C FE FF FF C4 09
               00 00 7F
```

The v2 frame is seq 9, stamp 0x01020304, boot_id 0xBEEF, acq_epoch 2,
imu_epoch 5, with the same three sensors.

## Pico control (types 0x12, 0x13 and 0x14)

The Pi controls the Pico's acquisition over the same UART pair: configure the
IMU, reinitialize the IMU, restart acquisition, and ask for optional
diagnostic frames. The Pico never answers a command directly; it reports
progress in status frames. Same envelope and CRC as the brain link.
`kPicoLinkVersion = 1`; decoders refuse any other version.

### Command (type 0x12, Pi to Pico)

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | version | u8 | 1 |
| 1 | op | u8 | See the op table |
| 2 | request_id | u16 | Never 0; the Pi starts from a random base per process |
| 4 | target_boot_id | u16 | The boot this command is for |
| 6 | body | varies | Layout determined by op |

| op | Constant | Body | Payload len |
|----|----------|------|-------------|
| 1 | `kPicoOpConfigure` | imu_enabled u8 (0 or 1) | 7 |
| 2 | `kPicoOpReinitImu` | imu_port u8 (0 on HAT v2) | 7 |
| 3 | `kPicoOpRestartAcquisition` | none | 6 |
| 4 | `kPicoOpDiagnostics` | diag_hz u8 (0 off, 1..5; 6..255 fail with BadBody) | 7 |

A command with an unknown op decodes its header, so the Pico can report it
failed with `kPicoDetailUnknownOp`. Firmware older than DIAGNOSTICS answers
op 4 that way.

### Status (type 0x13, Pico to Pi)

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | version | u8 | 1 |
| 1 | boot_id | u16 | As in sensor frames |
| 3 | acq_epoch | u8 | As in sensor frames |
| 4 | imu_epoch | u8 | As in sensor frames |
| 5 | uptime_ms | u32 | Pico clock |
| 9 | imu_state | u8 | `PicoImuState` |
| 10 | imu_reason | u8 | `PicoImuReason`, why the last attempt failed |
| 11 | imu_attempts | u16 | Attempts in the current episode |
| 13 | flags | u8 | bit 0 `kPicoImuEnabled` |
| 14 | last_request_id | u16 | Newest command, 0 = none yet |
| 16 | last_op | u8 | Its op |
| 17 | last_status | u8 | `PicoCommandStatus` |
| 18 | last_detail | u8 | `PicoCommandDetail` |
| 19 | firmware | u8 | IMU driver built in: 0 unknown, 1 BNO08X, 2 ASM330 |

| Value | `PicoImuState` | `PicoCommandStatus` | `PicoCommandDetail` | `PicoImuReason` |
|-------|----------------|---------------------|---------------------|-----------------|
| 0 | Disabled | None | None | None |
| 1 | Initializing | Running | WrongTarget | NoResponse |
| 2 | Aligning (BNO08X gravity alignment) | Completed | UnknownOp | Boot (timeout) |
| 3 | Ready | Failed | BadBody | Features (reports not acknowledged) |
| 4 | Retrying (backoff) | | ImuAbsent | Stream (reports stopped) |
| 5 | Failed (slow retries continue) | | ImuDisabled | |
| 6 | | | NoSuchPort | |

### Pico control rules

- A command runs only when `target_boot_id` is the current `boot_id`;
  otherwise it fails with `WrongTarget`. A command meant for a previous boot
  never runs after a Pico reboot.
- The Pico records the last 4 request ids with their op and body. A repeated
  id with the same op and body is never run again; its recorded status is
  reported. A lost status therefore never makes the Pi's resend zero the
  counters or restart the IMU twice.
- Status frames go every 200 ms and after every command state change, only
  while the UART transmit FIFO is empty, and never delay a due sensor frame.
- The Pi resends a command every 200 ms until a status names its request id,
  within a bound. Completion is taken only from a reported `Completed` or
  `Failed`, never from receipt.
- CONFIGURE sets whether the IMU is enabled. The Pi sends it when it applies a
  Brain profile (enabled exactly when the profile's IMU source is the Pico).
- REINIT_IMU advances `imu_epoch`, reports `Running` until the IMU is ready,
  then `Completed`; `Failed` with `ImuAbsent` when it does not initialize
  within its attempts, `ImuDisabled` while disabled, `NoSuchPort` for a port
  other than 0.
- RESTART_ACQUISITION zeroes the encoder counters atomically against the
  encoder interrupts, advances `acq_epoch`, and completes at once.
- While enabled, the Pico retries a failing IMU on its own: 5 quick attempts
  with a backoff from 0.5 s doubling to 8 s, then `Failed` with a slow retry
  every 30 s. Encoders keep streaming throughout.
- An ordinary reconnect changes nothing on the Pico; only
  RESTART_ACQUISITION or a reboot zeroes counters.
- DIAGNOSTICS sets the diagnostic frame rate and completes at once; it is
  recorded and deduplicated like every other command. The rate is 0 after
  every boot, so the Pi asks again after a reboot.

### Diagnostic frame (type 0x14, Pico to Pi)

Optional instrumentation for the Pi's viewer and captures; nothing on the Pi
localizes from it. A Pico sends it only while a DIAGNOSTICS command of its
current boot set a nonzero rate. The rate lasts until the Pico reboots or
receives DIAGNOSTICS 0, so a Pi process that never asked can still receive
frames an earlier Pi process asked for (navigatr restarted with another
configuration or build while the Pico stayed powered):

- a current Pi without `<Diagnostics>` sends DIAGNOSTICS 0 once to that boot
  and the frames stop;
- an older Pi build does not know type 0x14; its reader discards those
  frames like line noise until the Pico reboots (its sync and length reject
  counters rise). Sensor frames between them still decode.

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | version | u8 | 1 |
| 1 | boot_id | u16 | As in sensor frames |
| 3 | seq | u8 | Counts diagnostic frames, wraps |
| 4 | firmware | u8 | As in the status frame |
| 5 | pins | u16 | Pin levels read back, bit set = HIGH (`PicoDiagPin`) |
| 7 | pins_known | u16 | Bit set = that pin was sampled |
| 9 | imu_rx | u16 | IMU packets or reads completed |
| 11 | imu_bad | u16 | Bad headers or failed reads |
| 13 | imu_resets | u8 | Hub resets or chip restarts seen |
| 14 | imu_error | i8 | Last driver error code, 0 none |
| 15 | reports_ok | u16 | Sensor reports accepted |
| 17 | reports_rejected | u16 | Sensor reports rejected |
| 19 | report_age_ms | u16 | Since the newest accepted report; 0xFFFE means at least that, 0xFFFF none |
| 21 | link_rx_bad | u16 | Pi to Pico candidate frames rejected (length or CRC), from the Pico's command FrameReader |
| 23 | ticks_skipped | u16 | Sensor ticks skipped because the TX FIFO was busy |
| 25 | flags | u8 | bit 0 `kPicoDiagImuPresent`: the build drives an IMU |

Counters are free running and keep their low bits (they wrap). The IMU
counters mean what each driver can report: BNO08X counts SHTP packets, bad
headers, hub resets (SH2_RESET events) and the last sh2 result (negative
`SH2_ERR_*`); ASM330 counts gyro samples read, failed probes, configurations
and health checks, software resets issued and the last failure reason (a
`PicoImuReason`), and has no report rejection (always 0).

`PicoDiagPin` bits: 0 IMU INT, 1 IMU RST, 2 IMU WAKE/PS0, 3 IMU CS, 4..9
encoder 0 A/B, 1 A/B, 2 A/B, 10 the UART RX from the Pi. The levels are logic
levels read back from the pads when the frame was built, with `gpio_get`,
which never changes a pin's function, direction or pull. They are not
voltages and not a signal-quality measurement; one sample per frame misses
fast transitions. The UART RX idles HIGH, so HIGH there does not show a
connection or its absence. RST and WAKE exist on the BNO08X build only.

Timing: a diagnostic frame is 32 bytes, exactly the RP2040 TX FIFO. It goes
out only in an idle window, like a status frame (FIFO empty, and its airtime
plus a margin before the next sensor tick), and after any due status frame.
It never delays a sensor frame; a busy link delays or thins it instead.

### Pico control examples

```
CONFIGURE rid 0x0102, boot 0xBEEF, enabled   AA 55 12 07 01 01 02 01 EF BE 01 26 24
REINIT_IMU rid 0xFFFF, boot 0x0001, port 0   AA 55 12 07 01 02 FF FF 01 00 00 EE 5F
RESTART_ACQUISITION rid 7, boot 0xBEEF       AA 55 12 06 01 03 07 00 EF BE CF 9E
DIAGNOSTICS rid 0x0304, boot 0xBEEF, 1 Hz    AA 55 12 07 01 04 04 03 EF BE 01 CA 47
DIAGNOSTICS rid 0x0305, boot 0xBEEF, off     AA 55 12 07 01 04 05 03 EF BE 00 BA FD
Status                                       AA 55 13 14 01 EF BE 02 05 56 34 12 00 04 03 02
                                             01 01 02 01 02 01 04 01 09 BF
Diagnostic                                   AA 55 14 1A 01 EF BE 7F 01 05 04 FF 07 34 12 56
                                             00 03 FA FE FF 02 01 FF FF A0 00 0C 0B 01 F0 D6
```

The diagnostic frame is boot 0xBEEF, seq 127, BNO08X, pins INT, WAKE and Pi
RX HIGH of 0x07FF sampled, imu_rx 0x1234, imu_bad 0x56, 3 resets, error -6,
65534 reports accepted, 258 rejected, no report yet (0xFFFF), 160 link
rejections, 0x0B0C ticks skipped, IMU present.

The status is boot 0xBEEF, acq_epoch 2, imu_epoch 5, uptime 0x00123456 ms, IMU
retrying after a features failure, 0x0102 attempts, enabled, last command
0x0102 REINIT_IMU running with detail ImuAbsent, BNO08X firmware.

## Brain link v4

`kBrainLinkVersion = 4`. The Brain asks, the Pi answers. Operations: open a
session (HELLO), place the robot (SET_POSE), read the state (GET_STATE, which
also carries the Brain bench IMU sample), upload and apply the robot profile
(PROFILE_WRITE, PROFILE_APPLY), read field documents in chunks (READ_DOC),
control calibration and acquisition (CONTROL), report the planned path for
inspection (PATH_REPORT), and read raw wheel travel for calibration
(READ_WHEELS).

### Transports

The same frames travel over either transport. The selection is explicit on
both sides: the Brain's `LinkConfig::transport` and the Pi configuration's
serial resource (`linux_serial_link` or `pros_usb_link`) must match. There is
no automatic transport choice or failover.

| | RS-485 | USB |
|---|---|---|
| Brain | V5 smart port in generic serial mode | V5 USB user console |
| Pi | `linux_serial_link`, HAT RS-485 transceiver, DE on a GPIO | `pros_usb_link`, the Brain's user CDC interface |
| Duplex | Half duplex, Pi drives DE for its reply | Full duplex at the transport |
| Bytes | Link frames as is | One NG1 line per link frame |
| Baud | 115200 default, equal on both sides | Not applicable |

#### USB NG1 envelope

The PROS console consumes some binary sequences and mixes program output into
the same stream, so on USB each link frame travels as one text line:

```
"NG1:" + uppercase hex of every frame byte (sync0 through crc) + "\n"
```

Example, the 23-byte GET_STATE request of the examples below: 50 characters,
then the newline.

```
NG1:AA5510110404D4C3B2A1FFFF000000000000000000280F
```

Receiver rules, the same on both sides (`communigatr/usb_line.h` and the Pi's
`pros_usb_link`):

- A line ends at `\n`; one trailing `\r` is stripped.
- The last `NG1:` in the line counts; text before it (a console prefix) is
  ignored. Lines without the marker are console text and are ignored.
- The digits after the marker must be uppercase `0-9A-F`, even in number, and
  2 to 256 long (1 to 128 bytes). Any fault drops the whole line.
- A line longer than 516 characters (4 + 256 + 256 of prefix room) is dropped
  whole, up to its newline.
- There is no checksum on the line itself; the frame CRC inside it is the
  check.

Brain side: output goes through the PROS named stream `/ser/ngtr` with PROS
output COBS disabled for the whole program, so any console text also arrives
plain and the Pi skips it. Input comes from stdin. Pi side: the device is
found by USB identity (interface 02, VID 2888, PID 0501); with more than one
Brain connected an explicit device is required. After an unplug the Pi retries
the open at most once a second and flushes stale input on open. The first
request completed after the Pi opens the device is applied but not answered
(first-drain rule); the Brain's retry gets the answer.

### Request (type 0x10, Brain to Pi)

Payload offsets count from the first payload byte (frame offset 4).

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | version | u8 | 4 |
| 1 | op | u8 | Operation |
| 2 | session | u32 | Session from the HELLO reply; 0 in HELLO |
| 6 | request_id | u16 | Brain transaction id, never 0 |
| 8 | body | varies | Layout determined by op |

The header offsets are frozen for every version.

`request_id` is a per-boot counter on the Brain: it starts at 1 and wraps from
65535 to 1, never 0. Each new transaction takes the next id. Only HELLO,
SET_POSE and CONTROL are ever resent with the same id and bytes; every other
request is idempotent by content and gets a new id each time.

| op | Constant | Body | Payload | Frame |
|----|----------|------|---------|-------|
| 1 | `kOpHello` | nonce u32 | 12 | 18 |
| 2 | `kOpSetPose` | x_mm i32, y_mm i32, heading_cdeg i32 | 20 | 26 |
| 4 | `kOpGetState` | imu_flags u8, imu_stamp_ms u32, imu_rotation_mdeg i32 | 17 | 23 |
| 6 | `kOpProfileWrite` | profile_id u32, total_len u16, offset u16, data 1..106 | 17..122 | 23..128 |
| 7 | `kOpProfileApply` | profile_id u32, total_len u16 | 14 | 20 |
| 8 | `kOpReadDoc` | doc_kind u8, doc_id u32, offset u16, max_len u8 | 16 | 22 |
| 9 | `kOpControl` | action u8, arg u8 | 10 | 16 |
| 10 | `kOpPathReport` | command_id u32, path_mode u8, count u8, count x (x_mm i32, y_mm i32) | 14..118 | 20..124 |
| 11 | `kOpReadWheels` | none | 8 | 14 |
| 12 | `kOpTelemetry` | `BrainTelemetry`, fixed 54 bytes | 62 | 68 |

Ops 3 (SELECT_LANDMARK) and 5 (GET_STATE_WITH_IMU) are retired v3 ops and are
never reused.

Request bodies (offsets from the body start, payload offset 8):

- HELLO: `nonce` at 0, fresh for every HELLO transaction (not per retry).
- SET_POSE: `x_mm` at 0, `y_mm` at 4, `heading_cdeg` at 8. The robot origin's
  field pose, heading CCW from +x.
- GET_STATE: `imu_flags` at 0 (bit 0 `kBenchImuValid`, other bits must be 0),
  `imu_stamp_ms` at 1 (the Brain's clock when it read the sample),
  `imu_rotation_mdeg` at 5 (continuous rotation, CCW). Flags 0 when the
  Brain has no bench IMU sample; the other fields are then 0. The Pi puts a
  valid sample in its Brain IMU mailbox when its configuration has one and
  ignores it otherwise.
- PROFILE_WRITE: `profile_id` at 0, `total_len` at 4, `offset` at 6, data at 8
  (the rest of the payload, `kProfileChunkMax` = 106 bytes at most).
- PROFILE_APPLY: `profile_id` at 0, `total_len` at 4.
- READ_DOC: `doc_kind` at 0 (1 field map, 2 field estimate), `doc_id` at 1 (0
  = current document), `offset` at 5, `max_len` at 7 (1..255; the Pi sends at
  most 96).
- CONTROL: `action` at 0 (`ControlAction`), `arg` at 1 (0, reserved).
- PATH_REPORT: `command_id` at 0, `path_mode` at 4 (0 none, 1 direct, 2
  avoiding), `count` at 5 (0..13), points at 6, field frame mm. The length
  must equal `14 + 8 * count`.
- READ_WHEELS: no body.
- TELEMETRY: see [TELEMETRY](#telemetry).

The Pi answers `kResultInvalidArgument` for a body it cannot use: unknown
GET_STATE flag bits; a PROFILE_WRITE or PROFILE_APPLY `total_len` outside
[32, 240], or a chunk that ends past it; an unknown READ_DOC kind or
`max_len` 0; a CONTROL action outside 1..4; a PATH_REPORT mode above 2.

Decoding (`decodeBrainRequest`): the frame must pass the envelope, len and CRC
checks and carry at least the 8-byte header. A version other than 4 decodes
the header only and succeeds, so the Pi can answer it. For version 4, a known
op with a payload length outside its range fails; an unknown op decodes the
header only and succeeds, so the Pi can answer `kResultUnsupportedOp`.

### Reply (type 0x11, Pi to Brain)

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | version | u8 | The Pi's version, 4 |
| 1 | op | u8 | Echo of the request |
| 2 | session | u32 | Echo; for HELLO Ok the opened (or retried) session |
| 6 | request_id | u16 | Echo |
| 8 | result | u8 | Result code |
| 9 | pi_instance | u32 | Random nonzero id of this Pi runtime |
| 13 | body | varies | Layout determined by op and result |

These 13 header bytes are frozen for every version, and `decodeBrainReply`
decodes them for any version (the body only for version 4). `pi_instance`
changes on every Navigatr process start and every `System::reset()`, so the
Brain can tell that the Pi restarted.

| Value | Constant | Meaning |
|-------|----------|---------|
| 0 | `kResultOk` | Done. SET_POSE: localization applied the placement. PROFILE_APPLY: the profile runs. CONTROL: the action finished |
| 1 | `kResultPending` | SET_POSE accepted, not applied yet. PROFILE_APPLY accepted, swapped in at the next boundary. CONTROL waiting on the Pico or the calibration |
| 2 | `kResultUnknownSession` | The session is not the Pi's current session |
| 3 | `kResultUnsupportedVersion` | Version mismatch; the reply header carries the Pi's version |
| 4 | `kResultUnsupportedOp` | Op unknown to this Pi |
| 5 | `kResultInvalidArgument` | request_id 0, a malformed body, or a reused id with another op or body; see the per-op rules |
| 8 | `kResultStale` | Older request id matching no record, HELLO reusing a recent nonce, or READ_DOC of a document no longer retained |
| 9 | `kResultNotReady` | Needs an applied Brain profile first (SET_POSE, CONTROL, READ_WHEELS) |
| 10 | `kResultProfileRejected` | PROFILE_APPLY: this Pi cannot run the profile; the body names why |
| 11 | `kResultUnavailable` | READ_DOC: no such document on this Pi; READ_WHEELS: this Pi configuration takes no Brain profile |
| 12 | `kResultNotStationary` | CONTROL: the robot moved in the stationary check; nothing started |
| 13 | `kResultFailed` | CONTROL ran and failed; `detail` says why |

Results 6 and 7 are the retired v3 landmark selection results and are never
reused.

Reply bodies. Every (op, result) pair not listed has no body (13-byte payload,
19-byte frame).

| op | result | Body | Payload | Frame |
|----|--------|------|---------|-------|
| HELLO | every result | nonce u32, echo | 17 | 23 |
| SET_POSE | Ok, Pending | odometry_epoch u32, anchor_revision u32, current values | 21 | 27 |
| GET_STATE | Ok | state block, 40 bytes | 53 | 59 |
| PROFILE_WRITE | Ok | profile_id u32, received u16 | 19 | 25 |
| PROFILE_APPLY | Ok, Pending, ProfileRejected | profile_id u32, profile_state u8, reason u8, detail u8 | 20 | 26 |
| READ_DOC | Ok | doc_kind u8, doc_id u32, total_len u16, crc32 u32, offset u16, data 1..96 | 27..122 | 33..128 |
| CONTROL | Ok, Pending, Failed | action u8, calibration u8, detail u8 | 16 | 22 |
| READ_WHEELS | Ok | count u8 (0..4), count x 16-byte wheel record | 14..78 | 20..84 |

- PROFILE_WRITE Ok: `received` is the number of contiguous bytes the Pi holds
  of that profile.
- READ_DOC Ok: `doc_id` is the actual document id (never 0), `total_len` and
  `crc32` describe the whole document, `offset` echoes the request, and the
  data is `min(max_len, 96, total_len - offset)` bytes.
- CONTROL: `action` echoes the request, `calibration` is the
  `CalibrationState` after the action, `detail` is a `ControlDetail`.

READ_WHEELS wheel record (16 bytes, one per profile wheel, in profile order):

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | port | u8 | Encoder port |
| 1 | flags | u8 | bit 0 `kWheelFresh` (received within the Pi freshness window), bit 1 `kWheelValid` (the sensor has a value) |
| 2 | discontinuity | u16 | Low 16 bits of the encoder discontinuity epoch; changes whenever motion may have been lost |
| 4 | counts | i32 | Raw counts as last received from the Pico |
| 8 | travel_um | i32 | Continuous wheel travel: counts per revolution, gearing, polarity and radius applied, travel scale not applied |
| 12 | age_ms | u16 | Pi cycle time minus receipt, clamped |
| 14 | reserved | u16 | 0 |

`travel_um` stays continuous across source restarts (it is rebased, never
reset); `discontinuity` tells a reader that a restart happened in between.

A version 4 reply for a known (op, result) with a length outside its range, or
a READ_WHEELS count that does not match the length, fails to decode. An unknown
op or result decodes the header only.

### TELEMETRY

Optional, best effort, Brain to Pi: what the Brain measured and commanded,
for the Pi's viewer and captures only. The Pi answers `Ok` with no body; the
report never changes localization, placement, the profile, the path or any
reply. The Brain sends it at most every `telemetry_period`, at the lowest
priority, one request outstanding as always.

Body (offsets from the body start, payload offset 8), 54 bytes:

| Offset | Field | Type | Group | Description |
|--------|-------|------|-------|-------------|
| 0 | flags | u8 | | bit 0 attitude, bit 1 motion, bit 2 wheels (`TelemetryFlagBit`); other bits are kept and ignored |
| 1 | stamp_ms | u32 | | Brain clock when the values were taken |
| 5 | roll_cdeg | i16 | attitude | robot frame, about +x (forward), positive left side up |
| 7 | pitch_cdeg | i16 | attitude | robot frame, about +y (left), positive nose down |
| 9 | command_id | u32 | motion | The command being executed |
| 13 | motion_state | u8 | motion | actugatr `MotionState` |
| 14 | motion_reason | u8 | motion | actugatr `MotionReason` |
| 15 | plan_mode | u8 | motion | investigatr `PlanMode` |
| 16 | segment | u8 | motion | Current path segment |
| 17 | segment_count | u8 | motion | Path segments |
| 18 | target_x_mm | i32 | motion | Field-frame destination |
| 22 | target_y_mm | i32 | motion | |
| 26 | target_heading_cdeg | i16 | motion | |
| 28 | cmd_vx_mm_s | i16 | motion | Commanded body-frame velocity, +x forward |
| 30 | cmd_vy_mm_s | i16 | motion | +y left |
| 32 | cmd_omega_cdeg_s | i16 | motion | CCW positive |
| 34 | cross_track_mm | i16 | motion | Tracking error |
| 36 | distance_error_mm | i16 | motion | |
| 38 | heading_error_cdeg | i16 | motion | |
| 40 | drive_fault | u8 | motion | actugatr `DriveFault` |
| 41 | wheel_count | u8 | wheels | 0..6 (`kTelemetryWheelsMax`) |
| 42 | wheel_rpm_x10 | 6 x i16 | wheels | Motor velocity targets, rpm x 10; entries past wheel_count are 0 |

A group whose flag bit is clear is written as zeros and ignored by the
decoder whatever the wire holds. A wheels group with `wheel_count` above 6
fails to decode (the encoder refuses it). Distances saturate at +-32767 mm.
The enum tables are in docs/actugatr.md.

Compatibility: a Pi without op 12 answers `UnsupportedOp`, header only; the
Brain then stops sending TELEMETRY for that session and keeps the session.
A resend of the newest request id with the same body is answered `Ok` again
and recorded once.

On the Pi, a Brain VEX profile folds the attitude group as the robot tilt:
measured while the newest report carrying the group is at most 250 ms old by
Pi arrival time, then stale (level assumed, the old measurement time kept).
Reports without the group (the VEX IMU calibrating or failing) do not end
that early; before the first attitude the tilt is unavailable. The Brain applies
its IMU mounting before sending; see docs/brain_setup.md for the sign
conventions, which are not yet verified on hardware.

### State block (GET_STATE Ok)

Offsets count from the start of the block (payload offset 13).

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | robot_flags | u8 | See the robot flag table |
| 1 | x_mm | i32 | Robot origin x, field frame |
| 5 | y_mm | i32 | Robot origin y, field frame |
| 9 | heading_cdeg | i32 | Robot heading, CCW from +x, in (-18000, 18000] |
| 13 | robot_age_ms | u16 | Pi cycle time minus the pose's measurement host time, clamped to 0..65535 |
| 15 | odometry_epoch | u32 | Low 32 bits of the Pi's odometry epoch |
| 19 | anchor_revision | u32 | Low 32 bits of the Pi's field anchor revision |
| 23 | health | u8 | See the health table |
| 24 | profile_state | u8 | `ProfileState` |
| 25 | profile_reason | u8 | `ProfileReason` when rejected |
| 26 | profile_id | u32 | Applied, applying or rejected profile, 0 = none |
| 30 | map_id | u32 | Field map document, 0 = no field |
| 34 | estimate_id | u32 | Newest field estimate document, 0 = none yet |
| 38 | calibration | u8 | `CalibrationState` of the Pi IMU bias calibration |
| 39 | profile_detail | u8 | Wheel or camera index the reason is about |

There are no landmark fields: all field estimates travel as documents.

| Bit | robot_flags | Set when |
|-----|-------------|----------|
| 0 | `kRobotPoseValid` | An estimator has produced a pose |
| 1 | `kRobotLocalized` | The field anchor was set by a placement |
| 2 | `kRobotAgeKnown` | `robot_age_ms` is meaningful |
| 3 | `kRobotAnchorCommand` | The anchor came from a Brain SET_POSE (any session of this `pi_instance`) |
| 4 | `kRobotAnchorConfigured` | The anchor came from the Pi's configured initial placement |

| Bit | health | Set when |
|-----|--------|----------|
| 0 | `kHealthEncodersFresh` | Every profile encoder is valid and fresh |
| 1 | `kHealthGyroFresh` | The profile's IMU source (Pico gyro or Brain bench samples) is fresh |
| 2 | `kHealthVisionAlive` | The current field snapshot has observations |
| 3 | `kHealthBiasCalibrated` | The IMU bias calibration has reported ready |
| 4 | `kHealthPicoLink` | Pico frames are arriving |
| 5 | `kHealthImuInitializing` | The Pico IMU is initializing or aligning |
| 6 | `kHealthImuFailed` | The Pico IMU failed; the Pico keeps retrying while enabled |
| 7 | `kHealthStationary` | A qualified stationary window holds right now |

| Value | `ProfileState` | `CalibrationState` |
|-------|----------------|--------------------|
| 0 | None: nothing applied, localization waits | None: nothing to calibrate on the Pi for this profile |
| 1 | Applying: accepted, applied at the next boundary | Running: collecting a stationary window |
| 2 | Applied: `profile_id` runs | Done |
| 3 | Rejected: `profile_id` was refused | WaitingStill: movement seen, the window restarts when still |
| 4 | | WaitingData: IMU or wheel samples missing, stale or discontinuous |
| 5 | | Failed: no qualified window within the bound; recalibrate retries |

A rejected profile never stops a running one: the state then names the
refused id, and the running profile keeps localizing.

Semantics:

- The robot pose and the field estimate share the anchor named by
  (`odometry_epoch`, `anchor_revision`). Frame identity for the Brain is
  (`pi_instance`, `session`, `odometry_epoch`, `anchor_revision`); any change
  is a coordinate discontinuity.
- `odometry_epoch` increments when the Pi's odometry frame becomes
  discontinuous: a new profile, a reinitialize, a localization reset. It never
  goes back on the wire. `anchor_revision` increments on every placement.
- Ages are differences of Pi host clock values only. The Brain adds its own
  measured round trip and its time since receipt; no clock value of one
  device is ever subtracted from another's.
- Status, health and validity bits are information, never acknowledgements,
  and do not identify a session. Results are the only acknowledgements.

### Sessions and dedupe

A session is one Brain application run, opened by HELLO. Duplicate detection
and placement acknowledgement are scoped to the session. The profile, its
staging, the field documents and localization are not.

Pi behavior:

- HELLO
  - Same nonce and request_id as the current session's opening, and nothing
    else accepted in that session yet: a retry. Ok with the same session;
    nothing changes.
  - Otherwise a nonce among the last 4 opening nonces: `kResultStale`;
    nothing changes.
  - Otherwise a new session, a random nonzero u32 different from the previous
    one. The dedupe records, the bench IMU mailbox and the reported path are
    cleared, and a SET_POSE not yet applied is withdrawn. Localization, an
    applied placement, the profile and its staging are kept.
- Any other op whose session is not current, or with no session open:
  `kResultUnknownSession`.
- `request_id` 0 or a malformed body: `kResultInvalidArgument`.
- A newer request_id (none yet, or `(rid - newest) mod 65536` in 1..32767) is
  a new request: execute it, record it, make it the newest.
- A request_id that is not newer is never executed again:
  - equal to the last SET_POSE or CONTROL record with the same body: answered
    as a duplicate of that record, reporting its current progress;
  - equal to the newest id, with the same op and body, for GET_STATE,
    PROFILE_WRITE, PROFILE_APPLY, READ_DOC, READ_WHEELS or PATH_REPORT:
    answered again (these are read-only or idempotent by content; a repeated
    GET_STATE does not accept its bench IMU sample again);
  - the same id with another op or body: `kResultInvalidArgument`;
  - anything else: `kResultStale`.
- Newest request wins per drain: when one drain completes several requests,
  only the newest is processed.

Brain behavior:

- Correlate first: accept a reply only if its op and request_id match the
  outstanding request and, for HELLO, the nonce matches, or, for other ops,
  the session matches the one the request was sent in. Everything else is
  dropped and counted.
- Only a correlated reply can signal a Pi restart: a `pi_instance` different
  from the one recorded at HELLO Ok.
- On a session change (`kResultUnknownSession`, a `pi_instance` change, or an
  incompatible peer): fail every in-flight state-changing request as session
  lost without resending it, restart profile sync, drop partial documents
  (complete ones are kept), then HELLO with a fresh nonce. `kResultStale` on
  HELLO also means a fresh nonce.
- Pi silence alone never ends a session; it only makes the link not
  connected. When replies resume in the same session nothing restarts.
- Ready means a session is open and at least one GET_STATE in it returned Ok.
- `kResultUnsupportedVersion` and `kResultUnsupportedOp` are terminal errors:
  the session is dropped and HELLO repeats every `hello_backoff`, never a retry
  storm.

Consequences:

- Brain restart: a new session. Delayed requests of the old session get
  `kResultUnknownSession`; delayed replies fail correlation. The pose and
  placement continue; if the Pi already runs the Brain's profile, nothing is
  uploaded and nothing resets. No movement carries over.
- Pi restart or reset: `pi_instance` changes and the old session is unknown.
  The Brain opens a new session, never resends an in-flight placement or
  control, uploads its profile again, and must place the robot again.

### Placement (SET_POSE)

- With a Pi configuration that takes a Brain profile, SET_POSE answers
  `kResultNotReady` until a profile is applied.
- A new SET_POSE requests a placement tagged (command, session, sequence) and
  answers Pending. The reply is Ok once localization reports that exact
  placement applied; a duplicate is answered the same way. Both carry the
  current `odometry_epoch` and `anchor_revision`.
- The Brain counts a placement as applied only when a later GET_STATE reports
  the Ok's anchor revision, so no pre-placement pose is used after it.
- A new session or a new profile withdraws a placement that was not applied
  yet; it is never applied to a later odometry origin.

### Robot profile exchange

The Brain owns the robot's localization geometry and sends it as a typed
document (see [robot profile document](#robot-profile-document)). The Pi never
receives XML or file paths. `profile_id` is the CRC-32 of the document.

```
Connect -> first GET_STATE shows profile_id and profile_state
  -> PROFILE_WRITE chunks (skipped when the Pi already runs this id)
  -> PROFILE_APPLY -> Pending ... -> Ok (applied)
  -> a GET_STATE that shows it applied (the Brain's confirmation)
  -> sensors and calibration ready -> SET_POSE -> normal operation
```

A state reply written before the swap still carries the old profile's
epoch and placement, so the Brain counts a new profile as applied only from a
GET_STATE that reports it, never from the APPLY Ok alone.

PROFILE_WRITE (Pi):

- Staging holds one document, keyed by `profile_id`, at most
  `kProfileMaxLen` (240) bytes. It survives sessions.
- `total_len` must be in [32, 240] and `offset + data_len <= total_len`,
  else InvalidArgument.
- A write at an offset above `received` (a gap) is InvalidArgument. A write
  that overlaps held bytes must repeat them exactly (a resend), else
  InvalidArgument. A different `profile_id` or `total_len` restarts staging.
- Ok carries `received`.

PROFILE_APPLY (Pi):

1. Staging incomplete, another id or length, or CRC-32 of the staged bytes not
   equal to `profile_id`: InvalidArgument.
2. `profile_id` equal to the applied id: Ok at once, and nothing resets. This
   is what keeps the pose across Brain restarts.
3. Already applying this id: Pending. Refused before: ProfileRejected with the
   remembered reason.
4. Otherwise: decode, the shared `validateRobotProfile`, then the Pi's
   capability checks (encoder ports wired, IMU port present, the Brain IMU
   mailbox configured for a VEX source, camera slots present). A failure is
   ProfileRejected with reason and detail. A Pi configuration without a Brain
   profile section answers ProfileRejected with `kProfileReasonNotAccepted`
   and keeps its own localization.
5. On success: Pending. The swap happens at the next controlled boundary;
   after it the state reports Applied and the same APPLY answers Ok. A build
   failure at the boundary reports `kProfileReasonBuild` and the previous
   profile keeps running.

Applying a different profile loses continuity: `odometry_epoch` advances,
history clears, the robot is unplaced, and any unapplied SET_POSE is
withdrawn. A new placement is required. The same profile again resets nothing.
A Pi restart starts with no profile, unplaced.

### Field documents (READ_DOC)

The Pi publishes the field map (kind 1, `doc_id` = `map_id`) and field
estimate snapshots (kind 2, `doc_id` = `estimate_id`). GET_STATE names the
current ids; the Brain reads them in chunks of at most 96 bytes.

Pi rules:

- `doc_id` 0 means the current document; the reply names the real id.
- Unknown kind, `max_len` 0, or `offset >= total_len`: InvalidArgument.
- No such document (no field configured, or no estimate yet): Unavailable.
- A `doc_id` that is no longer retained: Stale. The map is fixed for a Pi
  process; the last 3 estimates are retained.
- An estimate is taken (`estimate_id` + 1) when its content changes (source,
  validity, pose at mm and cdeg resolution, epoch or anchor) and at least
  `estimate_period_ms` (default 200) has passed. `estimate_id` restarts in
  each Pi process, so the Brain keys estimates by (`pi_instance`,
  `estimate_id`).

Brain rules for assembling one document:

- Every chunk names the same kind and doc_id, repeats the `total_len` and
  `crc32` of the first, starts where the previous one ended and stays inside
  `total_len` (at most the kind's maximum length).
- A complete document must match its `crc32`. A map must also have CRC-32
  equal to `map_id` and pass `validateFieldMap`; an estimate must name the
  held map and pass `validateFieldEstimate` against it.
- Anything else discards the assembly. A partial document is never used.
- A map and an estimate are published together, as one generation, only when
  both are complete and consistent.

### Control

| Action | Constant | Effect on the Pi |
|--------|----------|------------------|
| 1 | `kControlRecalibrate` | Stationary check, then restart the IMU bias calibration in the active model. Pose holds. With a Brain VEX IMU profile: Ok with calibration None: only the stationary check, after which the Brain resets its own IMU |
| 2 | `kControlReinitialize` | Stationary check, then a localization reset: `odometry_epoch` + 1, unplaced, unapplied SET_POSE withdrawn, bias recalibration |
| 3 | `kControlReinitImu` | Needs a Pico IMU profile and v2 Pico firmware, else Failed with ImuUnused or PicoLink. Sends REINIT_IMU to the Pico; Pending while it runs; Ok when it completed and the recalibration restarted; Failed with the Pico's reason or after 15 s. Pose holds |
| 4 | `kControlRestartAcquisition` | Stationary check, then RESTART_ACQUISITION on the Pico (v2 firmware, else Failed with PicoLink); Pending until the new `acq_epoch` appears in sensor frames; Ok. The used encoders restarted, so the pose is invalid (unplaced, new odometry epoch): place again |

- Without an applied profile: NotReady.
- Stationary check: over the last 300 ms every profile encoder moved less than
  1 mm of wheel travel and, with a Pico IMU, |yaw rate| < 2 deg/s. Otherwise
  NotStationary and nothing starts.
- The CONTROL record is deduplicated like SET_POSE. The Brain asks again with
  the same request id while the answer is Pending, and after a lost reply
  (also across a cable pull in the same session); the duplicate reports
  current progress and never resubmits to the Pico.

| Value | `ControlDetail` | Meaning |
|-------|-----------------|---------|
| 0 | None | |
| 1 | PicoLink | No Pico frames, or its firmware takes no commands (v1) |
| 2 | ImuAbsent | The Pico IMU did not initialize within its attempts |
| 3 | ImuUnused | The profile does not use the Pico IMU |
| 4 | PicoRefused | The Pico rejected the command |
| 5 | TimedOut | No completion within the bound (15 s for ReinitImu) |
| 6 | Calibration | No qualified stationary window within the bound |

### Path report and wheel readings

- PATH_REPORT stores the session's latest planned path for Pi inspection only;
  it never changes localization. `path_mode` 0 clears it. A new session
  clears it. The Brain sends the path's vertices (segment starts and
  translation ends, repeats removed), at most 13, thinned evenly with the
  first and last kept.
- READ_WHEELS answers one record per profile wheel. It is meant for
  calibration: two readings taken before and after a measured push give each
  wheel's raw travel. NotReady without an applied profile; Unavailable on a
  Pi configuration without a Brain profile section.

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
  DE is asserted. A request completed in the first drain since open, start or
  reset has no `r(k-1)` and gets no reply (it is still applied).
- No reply either when more bytes followed the processed request in the same
  drain (the Brain may be transmitting), or when the link reports input
  pending just before DE (counted).
- Newest request wins per drain. A buffered partial frame is discarded when a
  whole drain returns no new bytes, because requests are sent in one burst.
- The Pi asserts DE only for its reply, waits until the transmitter is empty,
  holds a short post guard (default 2 character times), then releases DE.
  Every error or timeout path releases DE and discards unsent output. The Pi
  transmit timeout is `bytes * 10 / baud + 2 ms`, 13.1 ms for a 128-byte
  frame.

| Name | Side | Default |
|------|------|---------|
| base response timeout `T0` (`response_timeout`) | Brain | 60 ms |
| byte time `b` (`byte_time`) | Brain | 10 / baud = 86.8 us |
| request gap | Brain | 5 ms |
| GET_STATE poll period | Brain | 20 ms |
| link timeout (connected) | Brain | 0.25 s |
| reply window `W` | Pi | 40 ms |
| turnaround guard | Pi | 1000 us |
| baud | both | 115200 |

The defaults are starting points, not measurements.

Frames reach 128 bytes in either direction (11.1 ms at 115200 baud), so the
Brain's response timeout depends on the request:

```
T(request) = T0 + b * max(0, request_frame + max_reply_frame(op) - 85)
```

`request_frame` is the request's own frame length, `max_reply_frame(op)` the
largest reply frame its op can get, and 85 bytes is the v3 budget pair (a
26-byte request and a 59-byte reply). The same formula is used on USB, where
it only adds margin.

Budget. Let

- `A_req` = request airtime after the Brain's write returns
- `A_rep` = reply airtime
- `R` = DE release after the last stop bit (post guard 0.17 ms plus the
  transmitter-empty poll, 0.2 ms)
- `L` = V5 transmit latency plus Pi UART and tty receive latency plus margin

The Pi's `r(k-1)` precedes the arrival of the request's last byte, so the Pi
never starts a reply later than `W + L` after the request ends on the wire,
and the reply is off the bus `A_rep + R` after that. A retry can therefore
never overlap a late reply when

```
T(request) >= A_req + W + A_rep + R + L
```

Because `A_req + A_rep = b * (request_frame + reply_frame)`, the formula keeps
the latency allowance at the v3 value for every pair of 85 bytes or more:
`L <= T0 - 85 b - W - R = 60 - 7.38 - 40 - 0.2 = 12.4 ms`. Smaller pairs leave
more.

| Request | Request frame | Largest reply frame | `T` | `A_req + A_rep` | `L` allowed |
|---------|---------------|---------------------|-----|-----------------|-------------|
| HELLO | 18 | 23 | 60.0 ms | 3.6 ms | 16.2 ms |
| SET_POSE | 26 | 27 | 60.0 ms | 4.6 ms | 15.2 ms |
| GET_STATE | 23 | 59 | 60.0 ms | 7.1 ms | 12.7 ms |
| PROFILE_WRITE, full chunk | 128 | 25 | 65.9 ms | 13.3 ms | 12.4 ms |
| PROFILE_APPLY | 20 | 26 | 60.0 ms | 4.0 ms | 15.8 ms |
| READ_DOC | 22 | 128 | 65.6 ms | 13.0 ms | 12.4 ms |
| CONTROL | 16 | 22 | 60.0 ms | 3.3 ms | 16.5 ms |
| PATH_REPORT, 13 points | 124 | 19 | 65.0 ms | 12.4 ms | 12.4 ms |
| READ_WHEELS | 14 | 84 | 61.1 ms | 8.5 ms | 12.4 ms |

The request gap adds 5 ms of further margin. Residual risks: the Pi can be
preempted between the window check and asserting DE, and DE release can be
delayed by preemption after transmit completion (counted as `late_release`).

Pose polling priority on the Brain (one request at a time): HELLO without a
session, placement, control, profile sync, the due state poll, READ_WHEELS,
map chunk, estimate chunk, path report. Reads and transfers go only while the
state poll is not due, except that a waiting transfer goes after 4 due polls
in a row (exchanges that outlast the poll period would otherwise starve it).
In host tests on the default bus, state polls stay at most 43 ms apart (one
period, one 128-byte READ_DOC exchange and the gap) while a 128-object map
moves.

Pi configuration constraints:

- A configuration with brain_link commands must satisfy
  `1000 / loop_rate_hz <= reply_window_ms / 2`, otherwise the build fails, so
  one loop period plus processing fits the window.
- The brain link requires Navigatr's threaded mode; the executable warns when
  `--inline` is used with it.
- The reply write blocks the Pi estimation worker for up to about 13 ms (a
  128-byte reply). That is accepted: replies happen only when the Brain asks,
  and large replies only during transfers.
- The V5 smart port is assumed to switch its own RS-485 direction. This is a
  hardware assumption and is not validated.
- On USB there is no DE and no input-pending refusal; the Pi bounds write
  backpressure to 5 ms and treats a stalled device as unplugged. The reply
  window, first-drain and trailing-bytes rules still apply.

Pi configuration and wiring: [Navigatr setup](../pi/naviGATR/docs/setup.md#connect-the-brain)
and [hardware](hardware.md).

### Brain link examples

Values used: session `0xA1B2C3D4`, pi_instance `0x0BADF00D`, nonce
`0x12345678`, profile_id `0xCAFEBABE`, map_id `0x89ABCDEF`. These frames are
the known-byte vectors in `translaGATR/tests/frame_codec_gtest.cpp`, packed by an
independent generator and checked against an independent CRC.

GET_STATE request, request_id 65535, no bench IMU sample:

| Field | Value | Bytes |
|-------|-------|-------|
| sync, type, len | request, 17 | `AA 55 10 11` |
| version, op | 4, GET_STATE | `04 04` |
| session | 0xA1B2C3D4 | `D4 C3 B2 A1` |
| request_id | 65535 | `FF FF` |
| imu_flags, stamp, rotation | 0, 0, 0 | `00 00 00 00 00 00 00 00 00` |
| crc | 0x0F28 | `28 0F` |

GET_STATE Ok state block: flags PoseValid, Localized, AgeKnown, AnchorCommand
(0x0F); robot (1500, -250, 18000); age 35 ms; epoch 2; anchor revision 7;
health 0x0B; profile rejected (3), reason observability (6), detail 1, id
0xCAFEBABE; map 0x89ABCDEF; estimate 0x102; calibration running (1).

```
Requests
HELLO rid 1                    AA 55 10 0C 04 01 00 00 00 00 01 00 78 56 34 12 AD 21
SET_POSE rid 2                 AA 55 10 14 04 02 D4 C3 B2 A1 02 00 62 02 00 00 37 FE FF FF
  (610, -457, -9000)           D8 DC FF FF DC 0D
GET_STATE rid 42, bench        AA 55 10 11 04 04 D4 C3 B2 A1 2A 00 01 78 56 34 12 B5 21 F9
  stamp 0x12345678, -450123    FF 18 5F
PROFILE_WRITE rid 5, total     AA 55 10 11 04 06 D4 C3 B2 A1 05 00 BE BA FE CA D0 00 6A 00
  208, offset 106, byte 5A     5A 73 B5
PROFILE_APPLY rid 6            AA 55 10 0E 04 07 D4 C3 B2 A1 06 00 BE BA FE CA D0 00 79 49
READ_DOC rid 7, map, offset    AA 55 10 10 04 08 D4 C3 B2 A1 07 00 01 EF CD AB 89 C0 00 60
  192, max 96                  73 FC
CONTROL rid 8, recalibrate     AA 55 10 0A 04 09 D4 C3 B2 A1 08 00 01 00 49 0F
PATH_REPORT rid 10, cmd 77,    AA 55 10 1E 04 0A D4 C3 B2 A1 0A 00 4D 00 00 00 02 02 E8 03
  avoiding, (1000, -2000),     00 00 30 F8 FF FF FF FF FF FF FF 7F 00 00 9F CB
  (-1, 32767)
READ_WHEELS rid 11             AA 55 10 08 04 0B D4 C3 B2 A1 0B 00 82 2D

Replies
HELLO Ok                       AA 55 11 11 04 01 D4 C3 B2 A1 01 00 00 0D F0 AD 0B 78 56 34
                               12 15 1F
HELLO Stale                    AA 55 11 11 04 01 00 00 00 00 01 00 08 0D F0 AD 0B 78 56 34
                               12 65 AF
SET_POSE Ok (2, 7)             AA 55 11 15 04 02 D4 C3 B2 A1 02 00 00 0D F0 AD 0B 02 00 00
                               00 07 00 00 00 70 2F
SET_POSE NotReady              AA 55 11 0D 04 02 D4 C3 B2 A1 02 00 09 0D F0 AD 0B 56 AA
GET_STATE Ok                   AA 55 11 35 04 04 D4 C3 B2 A1 FF FF 00 0D F0 AD 0B 0F DC 05
                               00 00 06 FF FF FF 50 46 00 00 23 00 02 00 00 00 07 00 00 00
                               0B 03 06 BE BA FE CA EF CD AB 89 02 01 00 00 01 01 71 B2
PROFILE_WRITE Ok, 107 held     AA 55 11 13 04 06 D4 C3 B2 A1 05 00 00 0D F0 AD 0B BE BA FE
                               CA 6B 00 DB 8C
PROFILE_APPLY Pending          AA 55 11 14 04 07 D4 C3 B2 A1 06 00 01 0D F0 AD 0B BE BA FE
                               CA 01 00 00 E7 8E
PROFILE_APPLY Rejected,        AA 55 11 14 04 07 D4 C3 B2 A1 06 00 0A 0D F0 AD 0B BE BA FE
  observability, wheel 1       CA 03 06 01 37 44
READ_DOC Ok, estimate 5,       AA 55 11 1B 04 08 D4 C3 B2 A1 07 00 00 0D F0 AD 0B 02 05 00
  total 44, crc 0x11223344,    00 00 2C 00 44 33 22 11 2B 00 EE 1A 33
  offset 43, byte EE
READ_DOC Unavailable           AA 55 11 0D 04 08 D4 C3 B2 A1 07 00 0B 0D F0 AD 0B 30 8A
CONTROL Ok, reinitialize,      AA 55 11 10 04 09 D4 C3 B2 A1 08 00 00 0D F0 AD 0B 02 01 00
  calibration running          C6 40
CONTROL Failed, restart acq,   AA 55 11 10 04 09 D4 C3 B2 A1 0D 00 0D 0D F0 AD 0B 04 05 05
  calibration failed, timeout  13 FA
CONTROL NotStationary          AA 55 11 0D 04 09 D4 C3 B2 A1 08 00 0C 0D F0 AD 0B 78 64
READ_WHEELS Ok, 2 wheels       AA 55 11 2E 04 0B D4 C3 B2 A1 0B 00 00 0D F0 AD 0B 02 00 03
                               34 12 C0 1D FE FF 06 12 0F 00 0C 00 00 00 01 01 FF FF FF FF
                               FF 7F 00 00 00 80 FF FF 00 00 5E 0E
```

The READ_WHEELS records are port 0, fresh and valid, discontinuity 0x1234,
counts -123456, travel 987654 um, age 12 ms; and port 1, fresh only,
discontinuity 0xFFFF, counts INT32_MAX, travel INT32_MIN, age 65535 ms.

TELEMETRY, as packed by the independent generator:

```
TELEMETRY rid 0x0102, every    AA 55 10 3E 04 0C D4 C3 B2 A1 02 01 07 EF CD AB 00 2E FB 37
  group                        02 04 03 02 01 02 03 01 04 09 DC 05 00 00 36 F7 FF FF D8 DC
                               20 03 88 FF 94 11 DD FF B0 04 96 00 05 04 B0 04 50 FB FF 7F
                               00 80 00 00 00 00 3E 3E
TELEMETRY rid 13, attitude     AA 55 10 3E 04 0C D4 C3 B2 A1 0D 00 01 E8 03 00 00 FA 00 0C
  only (2.50, -5.00 deg)       FE, then 45 zero bytes, E0 69
TELEMETRY Ok                   AA 55 11 0D 04 0C D4 C3 B2 A1 0D 00 00 0D F0 AD 0B 55 98
TELEMETRY UnsupportedOp        AA 55 11 0D 04 0C D4 C3 B2 A1 0D 00 04 0D F0 AD 0B 53 11
  (an older Pi)
```

The full report is stamp 0x00ABCDEF, roll -12.34 deg, pitch 5.67 deg, command
0x01020304, state 2, reason 3, mode 1, segment 4 of 9, target (1500, -2250)
mm at -90 deg, command (800, -120) mm/s and 45 deg/s, errors -35 mm, 1200 mm
and 1.5 deg, fault 5, 4 wheels at 120.0, -120.0, 3276.7 and -3276.8 rpm.

## Documents

Documents are moved in chunks: the robot profile Brain to Pi with
PROFILE_WRITE, the field map and field estimate Pi to Brain with READ_DOC.
Layouts are in `translaGATR/link_documents.h`.

- Integers are little endian. Counts are explicit, records are fixed length,
  and there are no terminators: binary records contain zero bytes.
- Every document starts with a format byte; map and estimate headers also
  carry their record length. An unknown format or record length is rejected.
- Reserved bytes are 0 and are checked.
- `profile_id` and `map_id` are the CRC-32 of the document bytes, so equal ids
  mean equal content.

| Document | Header | Record | Count limit | Largest |
|----------|--------|--------|-------------|---------|
| Robot profile | 32 | wheel 32, camera 28 | 3 wheels, 4 cameras | 240 bytes, 3 chunks |
| Field map | 24 | object 28 | 128 objects | 3608 bytes, 38 chunks |
| Field estimate | 24 | object 20 | the map's count | 2584 bytes, 27 chunks |

### Robot profile document

Robot frame: +x forward, +y left, origin the reported robot point. The
footprint is the distance from the origin to each side of the enclosing
rectangle.

Header (32 bytes):

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | format | u8 | 1 |
| 1 | topology | u8 | 1 two wheels + IMU, 2 three wheels, 3 two forward wheels + IMU |
| 2 | wheel_count | u8 | 2 or 3, per topology |
| 3 | camera_count | u8 | 0..4 |
| 4 | imu_source | u8 | 0 none, 1 Pico, 2 Brain VEX |
| 5 | imu_port | u8 | Pico IMU port (0 on HAT v2); 0 otherwise |
| 6 | vex_smart_port | u8 | 1..21 with the Brain VEX source; 0 otherwise |
| 7 | imu_flags | u8 | bit 0 `kImuInvert` (Pico yaw sign flipped); Pico source only |
| 8 | footprint_front_um | i32 | 0..2000000 |
| 12 | footprint_back_um | i32 | 0..2000000 |
| 16 | footprint_left_um | i32 | 0..2000000 |
| 20 | footprint_right_um | i32 | 0..2000000 |
| 24 | calibration_window_ms | u16 | Stationary window for IMU bias; 0 = Pi default, else 500..20000 |
| 26 | still_rate_cdps | u16 | Gyro rate still counted as still; 0 = Pi default, else 10..2000 |
| 28 | still_travel_um | u16 | Per-wheel travel still counted as still over the window; 0 = Pi default, else 20..5000 |
| 30 | reserved | u16 | 0 |

Wheel record (32 bytes, `wheel_count` of them):

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | encoder_port | u8 | naviGATR encoder port 0..2; distinct per wheel |
| 1 | flags | u8 | bit 0 `kWheelReversed`: encoder polarity, positive counts mean travel against the measuring direction |
| 2 | reserved | u16 | 0 |
| 4 | counts_per_rev | u32 | Encoder counts per encoder shaft revolution, 1..1000000 |
| 8 | radius_um | u32 | Tracking wheel radius, 1000..200000 |
| 12 | x_um | i32 | Contact point, robot frame, within +-1000000 |
| 16 | y_um | i32 | within +-1000000 |
| 20 | angle_mdeg | i32 | Measuring direction, CCW from +x, within +-360000 |
| 24 | gear_micro | u32 | Encoder revolutions per wheel revolution x 1e6, 100000..10000000 (0.1..10) |
| 28 | travel_scale_ppm | u32 | Measured distance correction x 1e6, 900000..1100000 (0.9..1.1); 1000000 uncalibrated |

Camera record (28 bytes, `camera_count` of them): `slot` u8, 3 reserved
bytes, then `x_um`, `y_um`, `z_um` (within +-2000000) and `roll_mdeg`,
`pitch_mdeg`, `yaw_mdeg` (within +-360000), all i32. Slots are distinct.

Each wheel keeps three kinds of value apart, and the Pi applies each exactly
once:

| Kind | Fields | Applied in |
|------|--------|------------|
| Encoder | counts_per_rev, gear_micro, `kWheelReversed` | The encoder sensor conversion: wheel angle = counts x 2 pi / (counts_per_rev x gear), sign by polarity |
| Geometry | radius, x, y, angle | The observation model: travel = wheel angle x radius, along the measuring direction |
| Empirical | travel_scale_ppm | The observation model, as a factor on travel |

A measured travel test observes only the product of radius, gearing and travel
scale. Keep radius and gearing physical and put the measured correction in the
travel scale; its range is narrow on purpose.

Topologies and IMU sources:

| Topology | Wheels | IMU source | Observability rule |
|----------|--------|------------|--------------------|
| 1 two wheels + IMU | 2 | Pico or Brain VEX | Directions independent: `abs(ux0 uy1 - uy0 ux1) >= 1e-3` |
| 3 two forward wheels + IMU | 2 | Pico or Brain VEX | Angles exactly 0 or +-180000 mdeg; zero sideways travel is assumed |
| 2 three wheels | 3 | none or Pico | `abs(det [ux uy k]) >= 3.2e-5`, with k = x uy - y ux in m |

A Brain VEX source with three wheels is refused (`ImuCombination`): its samples
run on the Brain clock and cannot be fused as an independent observation. No
IMU source requires three wheels.

`validateRobotProfile` is the shared semantic check. The Brain runs it before
upload; the Pi runs it before its capability checks. It checks, in order, and
reports the first failure:

| Value | `ProfileReason` | Detail | Failed check |
|-------|-----------------|--------|--------------|
| 1 | Format | | Not a decodable format 1 document |
| 2 | Topology | | Unknown topology |
| 3 | WheelCount | | Wrong wheel count for the topology |
| 10 | Camera | | More than 4 cameras |
| 4 | EncoderPort | wheel | Port used twice (the Pi also: not wired on this Pi) |
| 5 | WheelGeometry | wheel | Flags, counts, radius, offsets, angle, gear or scale out of range |
| 7 | ImuSource | | Unknown source, or flags it cannot take |
| 8 | ImuPort | | Port fields that do not fit the source (the Pi also: no such Pico IMU port) |
| 9 | ImuCombination | | Topology and source cannot be combined |
| 6 | Observability | | Wheel directions cannot resolve the motion |
| 11 | Footprint | | A side out of range, or zero length or width |
| 14 | Calibration | | Calibration settings out of range |
| 10 | Camera | camera | Slot used twice, or mount out of range |
| 12 | Build | | Pi only: building the localization failed |
| 13 | NotAccepted | | Pi only: this configuration takes no Brain profile |

Example (96 bytes, CRC-32 0x6482E4C9): two wheels + IMU, Brain VEX IMU on
smart port 1, footprint front 0.2 m, back 0.15 m, sides 0.18 m, calibration
window 2 s, still rate 1 deg/s, still travel 1 mm; wheel 0 on port 0, 4000
counts, radius 24 mm at (0, 0.1 m) measuring forward; wheel 1 on port 1,
reversed, 8192 counts, radius 24 mm at (-0.05 m, 0) measuring left, gear 2.5,
travel scale 1.012345.

```
01 01 02 00 02 00 01 00 40 0D 03 00 F0 49 02 00    header
20 BF 02 00 20 BF 02 00 D0 07 64 00 E8 03 00 00
00 00 00 00 A0 0F 00 00 C0 5D 00 00 00 00 00 00    wheel 0
A0 86 01 00 00 00 00 00 40 42 0F 00 40 42 0F 00
01 01 00 00 00 20 00 00 C0 5D 00 00 B0 3C FF FF    wheel 1
00 00 00 00 90 5F 01 00 A0 25 26 00 79 72 0F 00
```

### Field map document

`map_id` is the CRC-32 of the document. It is fixed for a Pi process and built
from the Pi's field definition.

Header (24 bytes):

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | format | u8 | 1 |
| 1 | record_len | u8 | 28 |
| 2 | revision | u16 | Declared in the field definition |
| 4 | object_count | u16 | 0..128 |
| 6 | reserved | u16 | 0 |
| 8 | min_x_mm | i32 | Boundary: the region the robot footprint must stay inside |
| 12 | min_y_mm | i32 | |
| 16 | max_x_mm | i32 | min < max on both axes |
| 20 | max_y_mm | i32 | |

Object record (28 bytes):

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | object_id | u16 | Stable semantic id 1..65535, strictly increasing; not an AprilTag id |
| 2 | kind | u8 | 1 landmark (may be estimated), 2 fixed element (never estimated) |
| 3 | flags | u8 | bit 0 obstacle, bit 1 estimated (landmarks only), bit 2 reference |
| 4 | x_mm | i32 | Nominal pose, field frame |
| 8 | y_mm | i32 | |
| 12 | heading_cdeg | i32 | In (-18000, 18000] |
| 16 | box_x_mm | i16 | Collision box center, object frame |
| 18 | box_y_mm | i16 | |
| 20 | box_heading_cdeg | i16 | Box rotation in the object frame, in (-18000, 18000] |
| 22 | box_length_mm | u16 | Along the box's own x |
| 24 | box_width_mm | u16 | |
| 26 | reserved | u16 | 0 |

An object is a planning obstacle exactly when the obstacle flag is set, and
then its box has nonzero length and width. An object without the flag has an
all-zero box. The box covers the element's largest horizontal extent; visual
geometry never becomes an obstacle implicitly. `reference` marks objects a
movement command may be relative to.

### Field estimate document

A complete snapshot: one record per map object, in map order.

Header (24 bytes):

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | format | u8 | 1 |
| 1 | record_len | u8 | 20 |
| 2 | object_count | u16 | Equal to the map's |
| 4 | map_id | u32 | The map this estimate belongs to |
| 8 | estimate_id | u32 | Snapshot counter of this Pi process |
| 12 | odometry_epoch | u32 | Anchor of the poses |
| 16 | anchor_revision | u32 | |
| 20 | reserved | u32 | 0 |

Object record (20 bytes):

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | object_id | u16 | The map's id at this index |
| 2 | source | u8 | 0 none, 1 nominal, 2 observed |
| 3 | flags | u8 | bit 0 `kEstimateValid`, set exactly when source is not none |
| 4 | x_mm | i32 | Field frame under the header's anchor |
| 8 | y_mm | i32 | |
| 12 | heading_cdeg | i32 | In (-18000, 18000] |
| 16 | age_ms | u16 | Observation age when the snapshot was taken, clamped; 0 unless observed |
| 18 | reserved | u16 | 0 |

Objects without the map's estimated flag are always nominal. An observed
record is reported only when its observation belongs to the current odometry
epoch, composed with the current anchor; otherwise the record is nominal.
Landmarks the Pi does not estimate (no camera, no world estimation) are
nominal.

## Structs are not the wire

The structs in `frames.h` and `link_documents.h` (`SensorSample`,
`BrainRequest`, `BrainReply`, `BrainState`, `BrainTelemetry`, `WheelReading`,
`PicoCommand`, `PicoStatus`, `PicoDiag`, `RobotProfileDoc`, the document
records) are in-memory
conveniences. Their layout differs from the wire because of padding, and the
request and reply structs hold the body fields of every op. Only the codec
maps between structs and bytes. Never copy a struct onto the wire, and never
overlay a struct on received bytes.

## Versioning

- Brain link frames carry their version in the first payload byte. The
  request header (8 bytes) and reply header (13 bytes) keep their offsets in
  every version, so any version can decode enough to correlate a reply and
  report a mismatch. A Pi answers a request of another version with
  `kResultUnsupportedVersion` in its own version; a Brain treats that as a
  terminal error and repeats HELLO every `hello_backoff`. v3 and v4 peers do
  not interoperate: update the Brain programs and the Pi together.
- Op and result numbers are never reused once retired (ops 3 and 5, results
  6 and 7).
- Documents carry a format byte (profile, map and estimate are format 1) and,
  for the map and estimate, their record length. A new layout gets a new
  format number; a reader rejects formats it does not know.
- The sensor frame has no version byte; the type byte distinguishes v1 (0x01)
  from v2 (0x04). A capture is tied to the format it was recorded under, so
  record the format alongside stored captures.
- The Pico link carries `kPicoLinkVersion` (1) in every payload; frames of
  another version are dropped.
- Optional additions stay inside version 4 and Pico link version 1, and each
  side degrades on its own. TELEMETRY (op 12): an older Pi answers
  `UnsupportedOp` and the Brain stops sending it for that session; an older
  Brain never sends it. DIAGNOSTICS (Pico op 4) and the diagnostic frame
  (0x14): older firmware answers `UnknownOp` and the Pi shows Pico
  diagnostics unavailable; a new Pico sends 0x14 only after a Pi asked its
  current boot. An older Pi therefore sees 0x14 only when a new Pi asked
  earlier in the same Pico boot (Pi builds switched without power-cycling
  the Pico); it discards those frames like line noise until the Pico reboots,
  and a current Pi that did not ask turns them off (see the diagnostic
  frame). Rebuilding one side alone is therefore safe; the new information
  appears only once every side involved runs the new build.
- Bodies kept as raw bytes (the Pi hub records) decode with
  `decodeTelemetryBody` and `decodePicoDiagPayload`, the same rules as the
  frame decoders.
- A protocol change updates the headers, the codec, the fakes
  (`brain/communiGATR/sim`), the tests and this document together.

## Language parity

The Pico firmware, the Pi runtime and the Brain library (`brain/communiGATR`)
share `translaGATR/frames.h` and the C++ codec `translaGATR/frame_codec.cpp`; it builds
as C++17 on the host, gnu++17 for the Pico, and gnu++20 in the PROS build.
`translaGATR/link_documents.cpp` is built by the Brain and the Pi only. The RS-485
bench programs exercise raw text transfer and are not implementations of these
frames.
