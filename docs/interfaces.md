# Wire interface specification

This is the datasheet for the byte-level contract between the three devices.
It describes the framing, the two frame types, and the rules every encoder and
parser must obey. The single source of truth in code is `common/frames.h`. When
this document and the header disagree, the header wins and this document is the
bug.

The wire contract is the portable product of the project. Any device that
speaks these bytes correctly interoperates, with no shared code and no
handshake.

## Pipeline

```
sensors --> Pico --> Pi --> brain --> motors
              |        |  ^
        sensor frame   |  | command frame (type 0x03)
         (type 0x01)   pose frame
                       (type 0x02)
```

Sensor frames travel Pico to Pi. Pose frames travel Pi to brain. Command frames
travel brain to Pi. Every link is one way. A receiver never acknowledges, never
requests retransmission, and never blocks waiting on its upstream. A command
that is lost is simply resent by the brain on its own schedule.

## Global rules

| Rule | Value |
|------|-------|
| Byte order (multi-byte fields) | Little-endian, least significant byte first |
| Numeric representation | Fixed-point integers only, no floats on the wire |
| Maximum frame length | 128 bytes (`kMaxFrameLen`), sync through checksum |
| Absence signaling | Structural (mask bit cleared), never a sentinel value |

Every multi-byte field is little-endian. Endianness is a per-field rule: the
bytes within one field are ordered least significant first, but the fields
themselves appear in the order listed. Single-byte fields have no byte order.

No value on the wire is a float. All physical quantities are fixed-point
integers in the units below. A parser reconstructs a field by reading its bytes
low to high, then interprets the integer in the field's unit.

### Fixed-point units

| Quantity | Unit on the wire | Meaning |
|----------|------------------|---------|
| Position (x, y, dx, dy) | mm | millimeters |
| Heading, bearing | cdeg | centidegrees, 1/100 of a degree |
| Angular rate (gyro) | mdeg/s | millidegrees per second |
| Acceleration | mg | milli-g, 1/1000 of standard gravity |
| Encoder count | counts | raw quadrature counts, 4000 per revolution |

## Framing

Every frame has the same envelope:

```
+--------+--------+------+------------------+-----+
| 0xAA   | 0x55   | type | type-specific    | xor |
| sync0  | sync1  |      | body             |     |
+--------+--------+------+------------------+-----+
```

| Field | Size | Description |
|-------|------|-------------|
| sync0 | 1 | Constant `0xAA` (`kSync0`), marks a possible frame start |
| sync1 | 1 | Constant `0x55` (`kSync1`), confirms the frame start |
| type | 1 | Frame type, see the type registry |
| body | varies | Layout determined by type |
| xor | 1 | XOR of every preceding byte in the frame |

### Frame type registry

| type | Name | Direction | Constant |
|------|------|-----------|----------|
| 0x01 | Sensor frame | Pico to Pi | `kFrameSensor` |
| 0x02 | Pose frame | Pi to brain | `kFramePose` |
| 0x03 | Command frame | Brain to Pi | `kFrameCommand` |

### Checksum

The checksum is a single byte: the XOR of every byte from `sync0` up to but not
including the checksum itself.

```
xor = sync0 ^ sync1 ^ type ^ body[0] ^ body[1] ^ ... ^ body[n-1]
```

A receiver validates by XOR-ing the whole frame including the checksum byte. A
valid frame yields zero, because a value XOR-ed with itself cancels.

### Loss of sync and recovery

A receiver that is not currently locked to a frame scans the incoming bytes for
the pair `0xAA 0x55`, then reads a candidate frame and checks its length and
checksum. A bad checksum or an impossible length means the sync pair was
coincidental data, so the receiver resumes scanning from the next byte.

Because frames carry no cross-frame state, any device may reboot mid-match and
recover on the next clean sync pair. This is what makes the link handshake-free.

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

`stamp_ms` is the Pico clock, and the Pico clock is the single time reference for
all fusion. The sample is timestamped when it is read, never on arrival at the Pi.
Only the difference between two stamps matters, so the zero point (Pico boot) is
irrelevant. The u32 field wraps after about 49.7 days, far beyond any match.

### Sensor mask and payload

`mask` declares which sensors are present. Each bit maps to one sensor. A set bit
means that sensor's bytes are in the payload; a cleared bit means its bytes are
absent, not zero and not a sentinel. The payload contains the present sensors
back to back, in ascending bit order. A parser walks the mask from bit 0 upward,
and for each set bit copies that sensor's width from the width table.

| Bit | Constant | Sensor | Width (bytes) | Type | Unit |
|-----|----------|--------|---------------|------|------|
| 0 | `kSensorEnc0` | Parallel tracking wheel | 4 | i32 | counts |
| 1 | `kSensorEnc1` | Perpendicular tracking wheel | 4 | i32 | counts |
| 2 | `kSensorEnc2` | Spare or second parallel wheel | 4 | i32 | counts |
| 3 | `kSensorGyroZ` | Yaw rate | 4 | i32 | mdeg/s |
| 4 | `kSensorAccelXY` | Acceleration x and y (reserved) | 8 | 2 x i32 | mg |

The width column is `kSensorWidth` in the header. It is the only table a parser
needs; the parser walks the payload by width and never interprets sensor meaning.
`kSensorBitCount` (currently 5) equals the number of width entries, and a test
guards that they stay in step.

Gyro rate is raw, with bias not removed. Bias removal happens in fusion on the Pi,
never on the Pico.

Adding a sensor is additive: define a new bit, add its width entry, and write the
firmware that sets the bit and appends the bytes. Existing parsers keep working
because they walk the mask by width and ignore bits they do not know.

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

## Pose frame (type 0x02)

Pi to brain. Carries the fused pose, the absolute pose of the one world object
the brain requested, and any live landmark observations. Pose is relative to
the zero point. There is no z.

```
+------+------+------+-----+----------+-------+-------+--------------+--------+
| 0xAA | 0x55 | 0x02 | seq | stamp_ms | x_mm  | y_mm  | heading_cdeg | status |
|      |      |      | u8  | u32      | i32   | i32   | i32          | u16    |
+------+------+------+-----+----------+-------+-------+--------------+--------+
+-----------+----------+----------+------------------+-------------+-----------+-----+
| object_id | obj_x_mm | obj_y_mm | obj_heading_cdeg | n_landmarks | landmarks | xor |
| u8        | i32      | i32      | i32              | u8          | n entries |     |
+-----------+----------+----------+------------------+-------------+-----------+-----+
```

| Offset | Field | Type | Size | Description |
|--------|-------|------|------|-------------|
| 0 | sync0 | u8 | 1 | `0xAA` |
| 1 | sync1 | u8 | 1 | `0x55` |
| 2 | type | u8 | 1 | `0x02` |
| 3 | seq | u8 | 1 | Sequence counter, increments per frame, wraps 255 to 0 |
| 4 | stamp_ms | u32 | 4 | Timestamp of the fused estimate |
| 8 | x_mm | i32 | 4 | Position x, relative to the zero point |
| 12 | y_mm | i32 | 4 | Position y, relative to the zero point |
| 16 | heading_cdeg | i32 | 4 | Heading in centidegrees |
| 20 | status | u16 | 2 | Health and validity bits, see the status table |
| 22 | object_id | u8 | 1 | Id of the requested world object |
| 23 | obj_x_mm | i32 | 4 | Object absolute x |
| 27 | obj_y_mm | i32 | 4 | Object absolute y |
| 31 | obj_heading_cdeg | i32 | 4 | Object absolute heading in centidegrees |
| 35 | n_landmarks | u8 | 1 | Number of landmark entries that follow, 0 to 8 |
| 36 | landmarks | varies | 8 each | `n_landmarks` entries, see the entry layout |
| last | xor | u8 | 1 | Frame checksum |

The object fields are always present so the frame layout is stable. The brain
reads them only when `kStatusObjValid` is set. An object can be valid without
being currently observed: a map or last-seen estimate keeps `kStatusObjValid`
set while `kStatusObjObserved` is clear.

### Status word

The brain reads this word for `isValid` and for health gating. Each bit is
independent.

| Bit | Constant | Set when |
|-----|----------|----------|
| 0 | `kStatusPoseValid` | The pose estimate is valid |
| 1 | `kStatusEncHealthy` | Encoders are reporting and sane |
| 2 | `kStatusGyroHealthy` | Gyro is reporting and sane |
| 3 | `kStatusVisionAlive` | Vision is producing detections |
| 4 | `kStatusBiasCal` | Init gyro bias calibration completed cleanly |
| 5 | `kStatusLocInit` | Localization was initialized from a commanded pose |
| 6 | `kStatusObjRequested` | The brain has an active object request |
| 7 | `kStatusObjValid` | Object fields hold a usable estimate |
| 8 | `kStatusObjObserved` | The object was seen this cycle, not just mapped |

### Landmark entry

Each entry is a live relative transform from the robot to one landmark, for
servoing directly onto a target. `n_landmarks` is capped at `kMaxLandmarks`
(8), which bounds the frame.

| Offset | Field | Type | Size | Description |
|--------|-------|------|------|-------------|
| 0 | id | u8 | 1 | Landmark identifier |
| 1 | dx_mm | i16 | 2 | Relative x to the landmark |
| 3 | dy_mm | i16 | 2 | Relative y to the landmark |
| 5 | bearing_cdeg | i16 | 2 | Bearing to the landmark center |
| 7 | quality | u8 | 1 | 0 to 255, derived from range and viewing angle |

Total entry size is 8 bytes. Bearing to the landmark center is more reliable than
landmark orientation, because a planar tag pose is ambiguous. Consumers prefer
bearing. `quality` maps to measurement noise inside fusion; it is not a hard
threshold on the wire.

### Pose frame example

A valid pose, all health bits set, object 3 held as a valid map estimate, no
landmarks in view.

| Field | Value | Bytes on the wire |
|-------|-------|-------------------|
| seq | 42 | `2A` |
| stamp_ms | 5000 | `88 13 00 00` |
| x_mm | 1500 | `DC 05 00 00` |
| y_mm | -250 | `06 FF FF FF` |
| heading_cdeg | 9000 (90.00 deg) | `28 23 00 00` |
| status | 0x00FF (bits 0 to 7) | `FF 00` |
| object_id | 3 | `03` |
| obj_x_mm | 2000 | `D0 07 00 00` |
| obj_y_mm | 1000 | `E8 03 00 00` |
| obj_heading_cdeg | 0 | `00 00 00 00` |
| n_landmarks | 0 | `00` |

Full frame (37 bytes):

```
AA 55 02 2A 88 13 00 00 DC 05 00 00 06 FF FF FF 28 23 00 00 FF 00 03 D0
07 00 00 E8 03 00 00 00 00 00 00 00 A7
```

The trailing `A7` is the XOR of the preceding 36 bytes.

## Command frame (type 0x03)

Brain to Pi. Carries autonomous intent: initialize localization, select the
world object to track, control streaming. Fixed length, 21 bytes. Fields a
command does not use are zero on the wire.

```
+------+------+------+-----+---------+-------+-------+--------------+------+-----------+-------+-----+
| 0xAA | 0x55 | 0x03 | seq | command | x_mm  | y_mm  | heading_cdeg | mode | object_id | flags | xor |
|      |      |      | u8  | u8      | i32   | i32   | i32          | u8   | u8        | u8    |     |
+------+------+------+-----+---------+-------+-------+--------------+------+-----------+-------+-----+
```

| Offset | Field | Type | Size | Description |
|--------|-------|------|------|-------------|
| 0 | sync0 | u8 | 1 | `0xAA` |
| 1 | sync1 | u8 | 1 | `0x55` |
| 2 | type | u8 | 1 | `0x03` |
| 3 | seq | u8 | 1 | Sequence counter, increments per frame, wraps 255 to 0 |
| 4 | command | u8 | 1 | Command code, see the command table |
| 5 | x_mm | i32 | 4 | Expected pose x, `kCmdInitPose` only |
| 9 | y_mm | i32 | 4 | Expected pose y, `kCmdInitPose` only |
| 13 | heading_cdeg | i32 | 4 | Expected heading, `kCmdInitPose` only |
| 17 | mode | u8 | 1 | Localization configuration selector |
| 18 | object_id | u8 | 1 | Requested world object, `kCmdSelectObject` only |
| 19 | flags | u8 | 1 | Command flag bits |
| 20 | xor | u8 | 1 | Frame checksum |

### Command codes

| Code | Constant | Meaning |
|------|----------|---------|
| 0x01 | `kCmdInitPose` | Reset localization to the given field pose |
| 0x02 | `kCmdSelectObject` | Request `object_id`, or clear the request when the flag is off |
| 0x03 | `kCmdSetStream` | Turn pose streaming on or off |

An unknown command code decodes fine and is ignored by the Pi, so a newer brain
can talk to an older Pi without breaking the link.

### Command flags

| Bit | Constant | Meaning |
|-----|----------|---------|
| 0 | `kCmdFlagObjectRequested` | With `kCmdSelectObject`: set requests the object, clear cancels |
| 1 | `kCmdFlagStreamOn` | With `kCmdSetStream`: set starts streaming, clear stops |

### Command frame example

Initialize localization at (610 mm, 457 mm, 90.00 deg).

| Field | Value | Bytes on the wire |
|-------|-------|-------------------|
| seq | 9 | `09` |
| command | 0x01 init pose | `01` |
| x_mm | 610 | `62 02 00 00` |
| y_mm | 457 | `C9 01 00 00` |
| heading_cdeg | 9000 | `28 23 00 00` |
| mode | 0 | `00` |
| object_id | 0 | `00` |
| flags | 0 | `00` |

Full frame (21 bytes):

```
AA 55 03 09 01 62 02 00 00 C9 01 00 00 28 23 00 00 00 00 00 57
```

The trailing `57` is the XOR of the preceding 20 bytes.

## Structs are not the wire

The structs in `frames.h` (`SensorSample`, `PoseFrame`, `LandmarkObs`,
`CommandFrame`) are in-memory conveniences. Their byte layout differs from the wire because of
padding and fixed-size arrays. Only the codec maps between structs and bytes.
Never copy a struct onto the wire, and never overlay a struct on received bytes.

## Versioning

A frame log or capture is tied to the format version it was recorded under. When
a field width or layout changes, older captures may no longer decode. Record the
format version alongside any stored capture so a decoder can reject or adapt to
mismatches.

## Language parity

All three devices are C++ and share `common/frames.h` and the codec directly,
so there is one implementation of the wire format in production code. Python
appears only in host-side tooling (bench scripts, log analysis); any Python
that speaks frames is checked against the C++ codec with shared byte vectors,
never trusted on its own.
