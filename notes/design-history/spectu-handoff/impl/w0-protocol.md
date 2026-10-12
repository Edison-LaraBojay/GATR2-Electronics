# W0 protocol: implementation log

2026-09-28. Workstream key `w0-protocol`. Worktree `S/wt-spectu` (detached at ae457ee), where
`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
Only `spectuGATR/**` was touched (all new files). Nothing committed; the new files are marked
intent-to-add (`git add --intent-to-add spectuGATR`, index only, content unstaged).

## What was built

Library `spectugatr_protocol` (C++17, std only, namespace `spectugatr::protocol`), layout per
STRUCT-9:

| File | Holds |
|---|---|
| `protocol/include/spectugatr/protocol/wire.h` | ids (aliases), limits, reserved values, wire codes (record kinds, flags, statuses, def kinds, severity, doc kinds, path states, segment flags, object kinds/flags, estimate sources), `WireText`, `seqDistance`, one name function per code enum |
| `crc.h` | `crc16` (CCITT-FALSE), `crc32` (ISO-HDLC), own code |
| `units.h` | m/rad/s to mm/cdeg/ms and back (`toMillimeters`, `toCentidegrees`, `toWireMs`, `toDurationMs`, inverses) |
| `records.h` | the six records as structs in wire units, `Record` variant, `recordKind`, `recordBytes` |
| `frame.h` | `FrameHeader`, `FrameBytes`, `FrameBuilder` (allocation-free packing, durable flag set automatically by an EVENT), `decodeFrame` -> `FrameResult` with `FrameError` |
| `line.h` | `encodeLine`, `lineFrame` (one pre-split line), `lineIsDurable` (fixed hex offset), `LineScanner` (byte stream, bounded, `ScanStats`) |
| `documents.h` | definition, estimate, path, names: sizes, encoders into caller buffers, structural decoders, `documentId`, `docChunk` (chunk that fits a frame's remaining room) |
| `doc_assembler.h` | `DocAssembler`: chunks -> documents, crc32-checked, abandon rules, `AssemblyStats` (receiver side; allocates) |
| `protocol/src/byte_cursor.*`, `record_codec.*` | private: LE byte cursor, record bodies |

Build: `spectuGATR/CMakeLists.txt` (options `SPECTUGATR_BUILD_HOST`, `SPECTUGATR_BUILD_TESTS`,
gtest provider, `add_subdirectory(protocol)`, `tests/protocol`, and `host` when HOST is on),
`protocol/CMakeLists.txt` (self-contained target), `tests/protocol/CMakeLists.txt` (explicit
list, `gtest_discover_tests(... PROPERTIES TIMEOUT 60)`).

Docs: `spectuGATR/docs/PROTOCOL.md` (byte tables, identities, versioning rule, version history),
`spectuGATR/protocol/VIEW.md` (view/1), `spectuGATR/PIPELINE.md` (the product), `spectuGATR/README.md`.

Tests (`spectugatr_protocol_tests`, 106 tests, 11 files): crc, units, wire, records, frame, line,
documents, doc_assembler, golden_vectors, robustness (seeded round trips of random frames,
COBS-wrapped lines between `sout` packets, NUL padding anywhere, garbage between lines, lines cut
by non-blocking writes, every truncation and every changed character of a line fails to decode,
the 123 B COBS limit, random bytes, random record bodies with a good crc, random documents),
allocation (counting `operator new`: encoding frames, lines, documents and chunks allocates 0).

Golden vectors: an independent Python packer written from PROTOCOL.md with the standard library
only (`struct`, `zlib.crc32`, CRC-16 written from the spec, a port of PROS `cobs_encode`) at
`S/sp-build/w0-protocol/vectors/pack.py`, output `vectors.txt`. Its C++ literals are pasted into
`tests/protocol/golden_vectors_gtest.cpp` (spliced by script, not by hand). The C++ codec produces
every line and document byte for byte and decodes them to the expected fields: liveness, pose
with tilt, held unplaced pose, SESSION with and without the placement ref, channel and event
DEFs, 4 samples, EVENT (durable), a mixed frame, definition/estimate/path/names documents with
their doc_ids, both chunks of the definition (the first a full 122 B frame, 249 characters), and a
COBS stream with a `sout` text packet in between. Packer sizes match PROPOSAL 5.5 (33, 89, 93,
125, 95, 61, 143, 249 characters).

## Decisions and deviations (with reasons)

1. **POSE flag bit 6, measured known.** The API has `measured_at = kNotSupplied` ("publish age
   shown") but 5.5 had no way to say so on the wire. Clear bit 6 means `measured_ms` is when the
   publisher took the pose. Bit 7 stays reserved.
2. **Reserved values are named and decode to absent** (TYPE-16, WIRE-6): `kNoSession` (a frame
   with session 0 is refused, `kNoSession` error), `kNoDocument` (field, estimate, path ids),
   `kNoPeriod` (SESSION pose_period_ms), `kFieldFrame` (path reference). SESSION field id and
   revision travel as `std::optional<FieldRef>`.
3. **Decode order: version first.** Byte 0 is frozen, so a newer major is named (`kNewerVersion`,
   `header.version`) even if its size or crc layout changed. Then size 14..122, crc, session,
   records. crc16 is stored little endian, as translaGATR does.
4. **Record rules.** Unknown kinds skipped by length; a known kind with a short body, a SAMPLES
   count over 16 or a text over 32 is skipped and counted malformed; a record running into the crc
   fails the frame. Bytes after the known fields are ignored (appended fields). DOC_CHUNK takes no
   appended fields since its data runs to the end of the record (stated in the versioning rule).
5. **Limits not given by the proposal:** DEF unit <= 8 characters; SAMPLES <= 16 entries per
   record (what fits one frame); POSE status 3 bits (0..4 used). Robot name 16, DEF name 32,
   note 24, 128 objects, 32 segments are from the proposal.
6. **Wire codes chosen:** record kinds 1..6 (proposal); doc kinds 1 definition, 2 estimate,
   3 path, 4 names; def kinds 0 real, 1 integer, 2 bool, 3 event; severity 0..2; pose status
   0 valid .. 4 calibrating (API order); path state 0 active .. 5 failed (API order); object kinds
   and flags and estimate sources as in the brain link (so the layouts stay byte identical).
7. **Path document details:** segment flags bit 0 turn (else translate), bit 1 reverse, bit 2
   exact, bit 3 clockwise (turns; else CCW); `has_target` must be 0 or 1; `field_id` 0 = none.
   Layout and the 38 + note + 12n size are exactly 5.5.
8. **Names document** (5.5 said only "id u16 plus a name per object"): `field_id u32 | count u16 |
   count x (object_id u16 | name_len u8 | name <= 32)`, tying names to one definition. Nothing
   produces it yet; the codec is there so receivers can read it.
9. **Definition and estimate decoders** accept a record_len larger than they know (read the known
   part, forward compatible) and refuse a smaller one; reserved bytes are written 0 and ignored on
   read (the link validator is stricter; spectu readers stay lenient for compatibility). Estimate
   header `map_id` is named `field_id`.
10. **kMaxLineChars = 249.** The 5.1 transport sketch says 250 for "SG1:" + 2 x 122 + "\n", which
    is 249. W4 can size its buffer with `protocol::kMaxLineChars`.
11. **DocAssembler lives in the protocol library.** 7.2 lists "an abandoned document" among the
    protocol tests. It is receiver side and allocates; it compiles for PROS (checked) but the
    Brain has no reason to link it (see W4 notes).
12. **Unit conversions in the protocol** (`units.h`): one conversion module for the wire units
    (UNIT-3, WIRE-2) so the publisher and the Pi adapter round the same way. Round to nearest mm
    and cdeg, cdeg wrapped to (-18000, 18000]; clock ms floored and wrapped.
13. **Byte identity with translaGATR.** The proposal chose own code with no GATR dependency, so
    the definition/estimate layouts are written twice (a WIRE-1/WIRE-2 exception by design).
    Checked by a scratch program (not committed): 2000 random maps and estimates, 0..128 objects,
    spectu bytes and crc32 equal translaGATR's; a one-bit negative control is caught.
14. **view/1 `samples` are reliable, not replaceable per (src, kind).** SAMPLES records carry
    different channel sets; one slot per (src, samples) would drop other channels' latest values
    for a slow client. The publisher caps the samples rate, so the reliable volume is small.
15. **view/1 `relay` envelope has a `type`.** The proposal's `{"src","rx_us","msg"}` gains
    `"type":"relay"` so every server message has a type. The receiver reads only the Pi's pong
    and does not relay it.
16. **view/1 additions** needed to make 5.9 implementable: `src` keys (`brain_usb`, `controller`,
    `navigatr_brain`, `navigatr`), `source.refused` (second Brain, newer major), the `stats`
    field list, late-joiner replay of session/defs/docs/pose, DEF and document dedupe (rule 12),
    enum codes as names (unknown codes as numbers), flag bytes as booleans, NaN samples as `null`,
    decoded document shapes, `brain_frames` reserved for the later navigatr source.
17. **Top-level CMake adds `host/` plainly** under `SPECTUGATR_BUILD_HOST` (default ON). W3 owns
    `host/CMakeLists.txt`; until it exists, configure with `-DSPECTUGATR_BUILD_HOST=OFF` (as done
    here). The gtest block is skipped when a parent already provides `GTest::gtest_main`. No
    `URL_HASH` on the fetch, as in `brain/CMakeLists.txt` (a hash could not be verified here).
    `protocol/CMakeLists.txt` sets its own `-Wall -Wextra`, so `brain/CMakeLists.txt` can
    `add_subdirectory` it without the top level.
18. **clang-format:** `/c/msys64/ucrt64/bin/clang-format` does not exist on this box; used the pip
    22.1.8 at `C:/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/clang-format.exe` with the
    repo `.clang-format` (it runs; the memory note about a segfault no longer holds). Existing
    files (for example `brain/communiGATR/include/communigatr/doc_assembly.h`,
    `translaGATR/frame_codec.cpp`) are not clean under that config, so the new files look
    different: access specifiers indented 2, short functions on one line, case labels on their
    own lines. Not changed; a repo-wide question for the coding standard.
19. **The golden packer is in scratch** (task instruction), not in the repo; CMAKE-9 would want a
    committed generator if the vectors are regenerated in CI.
20. **PIPELINE.md names future pieces** (`Publisher`, `nextLine`, `StdioTransport`,
    `pros_usb_link`, `brain_viz_lines`, `host/`, `viewer/`) that W4, W5 and Phase B create. A
    PIPE-3 name check passes only once they land.

## Notes for the other workstreams

- **W4 publisher.** Pack with `FrameBuilder` (`add` returns false when a record does not fit or
  is over a limit; `roomBytes()` for scheduling; `recordBytes()` to size before packing), then
  `finish(header)` and `encodeLine(frame, buf, kMaxLineChars)`. Documents: encode into a fixed
  buffer (`definitionBytes(128)` = 3608, `estimateBytes(128)` = 2584, path <= 446), `documentId`,
  then `docChunk(doc, offset, builder.roomBytes())` until nullopt. Texts through `makeText(s,
  kMax...Chars)`. Units through `units.h`. POSE: set `measured_known`, `tilt` with `age_ms` at
  sent_ms, `held` for repeats. `gatr2_brain.mk` needs include dir `spectuGATR/protocol/include`
  and the encode-side sources `src/{byte_cursor,crc,documents,frame,line,record_codec,records,units,wire}.cpp`;
  `doc_assembler.cpp` is not needed on the Brain. All 10 sources compile with the PROS toolchain
  (arm-none-eabi-g++ 13.3.1, gnu++20, PROS flags, -Wall -Wextra): 0 errors, 0 warnings.
- **W5 receiver.** One `LineScanner` per source (NUL, COBS, console text handled; `reset()` on
  reconnect), `decodeFrame`, one `DocAssembler` per source (`reset()` on a new session,
  `keepOnly(kind, id)` on each SESSION). `ScanStats`, `FrameError`, `AssemblyStats` map to the
  view/1 `source.stats` names. Enum names for JSON come from `wire.h`.
- **W7 naviGATR adapter.** `lineIsDurable` picks the durable queue without decoding; `lineFrame`
  decodes lines `pros_usb_link` already split. A Pi computes the same field_id for its FieldMap
  with `encodeDefinition` + `documentId` (byte identical to the link map doc).
- **W3 platform.** The top level adds `host/`; guard host tests with `SPECTUGATR_BUILD_TESTS`;
  `GTest::gtest_main` is available when tests are on.

## Commands and results (Git Bash, `export PATH=/c/msys64/ucrt64/bin:$PATH`)

```
cmake -S S/wt-spectu/spectuGATR -B S/sp-build/w0-protocol-host -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Debug -DSPECTUGATR_BUILD_HOST=OFF \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src
cmake --build S/sp-build/w0-protocol-host -j16          -> exit 0, 0 warnings (-Wall -Wextra)
ctest --test-dir S/sp-build/w0-protocol-host -j16 --output-on-failure
                                                        -> 100% tests passed, 0 failed out of 106
spectugatr_protocol_tests.exe --gtest_shuffle --gtest_repeat=3
                                                        -> 106 passed, 3 times
```

- UBSan trap + libstdc++ assertions: same configure plus
  `-DCMAKE_CXX_FLAGS="-fsanitize=undefined -fsanitize-trap=undefined -D_GLIBCXX_ASSERTIONS"` in
  `S/sp-build/w0-protocol-ubsan` -> 106/106 passed. (ASan does not link on MinGW; TEST-18 leaves
  it to Linux CI.)
- Golden packer: `python S/sp-build/w0-protocol/vectors/pack.py > vectors.txt` (Python 3.12.11,
  ucrt64) -> exit 0.
- translaGATR cross-check: `g++ -std=c++17 -Wall -Wextra -O1` of `xcheck.cpp`,
  `translaGATR/{link_documents,frame_codec}.cpp` and the protocol sources in
  `S/sp-build/w0-protocol-xcheck` -> "2000 random maps and estimates (0..128 objects): 0
  mismatches"; negative control -> 1 mismatch, exit 1.
- PROS toolchain compile of each protocol source (`-c`, PROS MFLAGS/CPPFLAGS/GCCFLAGS,
  `--std=gnu++20 -Wall -Wextra`) into `S/sp-build/w0-protocol-arm` -> 0 failures, 0 warnings.
- Formatting: `clang-format --dry-run --Werror --style=file` over every new .h/.cpp -> exit 0.
  All new text is plain ASCII with LF line endings.
- Mutation check (TEST-12) on a scratch copy in `S/sp-build/w0-protocol-mut`, one mutation at a
  time, each caught by at least one test: crc ignored, held bit wrong, NUL kept, NUL uncounted,
  document crc ignored, first marker wins, cut lines uncounted, newer version accepted, durable
  never set, longer records misread, no abandon on a new id of the same size (test added for it),
  -180 deg not wrapped, no marker restart, session 0 accepted, CR not stripped, has_target
  unchecked, an allocation in `FrameBuilder::add`. The SAMPLES count guard (count > 16) only
  prevents an out-of-bounds write: caught under `_GLIBCXX_ASSERTIONS`, not in a plain build.

Nothing here ran on a Brain, controller, radio or Pi. The COBS behavior is the PROS 4.2.2
`cobs_encode` port, not a device.
