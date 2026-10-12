---
name: project-wire-format
description: "GATR2 wire contract — frame format, framing, fixed-point, mask-walking; the portable product of the project"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0693582d-d417-490c-8cb4-1a0c07d0b810
  modified: 2026-07-24T06:55:10.636Z
---

The wire format is THE contract, defined once in `translaGATR/frames.h` (was `common/frames.h`; namespace `translagatr`, was `gatr2`). It is the portable artifact of the project.

**Rules:** All multi-byte fields little-endian. All values fixed-point integers (mm, centidegrees, millidegrees/sec, milli-g). No floats on the wire. Frames bounded by `kMaxFrameLen = 128` so every parser uses static buffers.

**Framing:** `sync 0xAA 0x55 | type byte | type-specific body | XOR checksum of all preceding bytes`. Receivers that lose sync scan forward for the next sync pair — this is what makes mid-match reboots recoverable with no handshake.

**Sensor frame (Pico -> Pi, type 0x01):** `sync | type | seq u8 | stamp_ms u32 | mask u16 | payload | xor`. Mask declares which sensors present; payload contains only present sensors packed in ascending bit order. Widths come from `kSensorWidth` table (parsers walk it, need no sensor semantics). Dead sensor clears its bit, bytes absent. NO sentinel values anywhere. Adding a sensor = new bit + width entry + firmware driver; existing parsers must not break.

**Pose frame v2 (Pi -> brain, type 0x02, changed 2026-07-24):** `sync | type | seq u8 | stamp_ms u32 | x_mm i32 | y_mm i32 | heading_cdeg i32 | status u16 | object_id u8 | obj_x_mm i32 | obj_y_mm i32 | obj_heading_cdeg i32 | n_landmarks u8 | n x {id u8, dx_mm i16, dy_mm i16, bearing_cdeg i16, quality u8} | xor`. Header 36 bytes. Status u16 bits 0-8: pose valid, enc healthy, gyro healthy, vision alive, bias cal, loc init (pose set from command), obj requested, obj valid, obj observed. Object fields always present, brain reads them only when ObjValid; ObjValid without ObjObserved = map/last-seen estimate. quality 0-255 maps to measurement noise internally.

**Command frame (brain -> Pi, type 0x03, added 2026-07-24):** `sync | type | seq u8 | command u8 | x_mm i32 | y_mm i32 | heading_cdeg i32 | mode u8 | object_id u8 | flags u8 | xor`, fixed 21 bytes. Commands: 0x01 kCmdInitPose, 0x02 kCmdSelectObject (flag bit0 = requested), 0x03 kCmdSetStream (flag bit1 = on). Unknown command values decode fine and are ignored (newer brain, older Pi). Receivers dedupe by seq: resend of same seq applied once, new command = new seq.

**Critical:** The structs in frames.h are NOT the wire layout (padding, fixed arrays). Only the codec maps structs<->bytes. NEVER memcpy a struct to the wire.

**Cross-language (updated 2026-07-24):** all three devices are C++ sharing frames.h + codec directly; no generated frames.py in production. Python only in host bench tooling, and any frame-speaking Python must be checked against the C++ codec with shared byte vectors.

See [[project-overview]], [[feedback-conventions]].
