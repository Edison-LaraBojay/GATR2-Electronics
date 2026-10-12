# W0 review (adversarial): implementation log

2026-09-29. Workstream key `w0-review`. Read-only on the worktree `S/wt-spectu`; nothing there was
edited, staged or committed. All builds and scripts live in `S/sp-build/w0-review-*`, where
`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.

Scope: `spectuGATR/protocol/**`, `docs/PROTOCOL.md`, `protocol/VIEW.md`, `PIPELINE.md`, `README.md`,
the CMake files and `tests/protocol/**`, checked against PROPOSAL 5.1, 5.2, 5.5, 5.6, 5.9, 7.1 and 7.2.

Verdict: no blocking issues. The code matches PROTOCOL.md byte for byte, and SG1 parsing agrees with
the documented rules on every fuzz case. There are 5 should-fix items to settle before W0 is frozen,
plus minor items (listed at the end).

## What was built (scratch only)

| File (`S/sp-build/w0-review-py/`) | What |
|---|---|
| `sg1.py` | independent decoder written from PROTOCOL.md alone, before reading the C++: scanner, frame, the six records, four documents, assembler. Stdlib only (`struct`, `zlib`) |
| `harness.cpp` / `harness.exe` | links the protocol sources. `encode SEED N` writes random C++-encoded frames and documents with their source values; `scan FILE` runs a raw byte stream through `LineScanner` -> `decodeFrame` -> `DocAssembler` -> `decode*` and prints the results; `line HEX` runs `lineFrame` and `lineIsDurable` |
| `check_encoded.py` | decodes the C++ vectors with `sg1.py` and compares every field |
| `fuzz.py` | own packer (from PROTOCOL.md) plus transport damage, differential against `harness scan`: every frame's bytes, error, header, records, unknown/malformed counts, assembled documents, and the scan counters |

Where PROTOCOL.md is silent, the reference follows the implementation's leniency: texts up to 32 on
read, `other_lines` counts only non-empty lines, one `cut` per extra marker, and the assembler
validates a chunk before touching its partial. Every such choice is listed under the findings.

## Results

1. **C++ encode -> independent decode.** Seeds 1-5, 5000 frames each: 25,000 frames, 66,692 records
   (all six kinds) and 2,505 documents (all four kinds, up to 128 objects and 32 segments, chunked,
   some chunks sharing a frame with a POSE). 0 mismatches in any header field, record field, durable
   flag, `lineIsDurable`, document byte, doc_id or decoded document field.
   - The first run flagged 50 SAMPLES values whose bit patterns were signaling NaNs. The cause was
     Python's float round trip quieting the NaN, not the wire. The comparison now uses the raw u32
     bits, and the C++ side carries sNaN bits unchanged.
2. **Independent pack + damage -> C++ decode (differential).** 1,900 stream cases, 39,380 scanned
   frames and 2,438 completed documents: 0 disagreements.
   - Streams mixed good frames, NG1 lines, console text, lowercase hex, `\r\n` and `\r\r\n`, overlong
     prefixes (200-3000 chars) before a marker, 240-800 digit marker lines, versions 0/2/9/255, bad
     crc, session 0, truncated frames, records running into the crc, unknown and malformed records.
   - They also had lines cut by the next SG1 line, SG1 lines cut by an NG1 line, `NG1:ABCD` before a
     marker, shuffled and repeated chunks, incomplete documents and random bytes.
   - Half of the streams went through PROS-style COBS (`sout` stream id, writes split at 1..300
     bytes). Half were then damaged: NULs, CRs, lost bytes, flipped bytes, inserted garbage, stray
     markers and early newlines.
   - Every C++ frame outcome was exercised: none, bad_crc, bad_version, newer_version, no_session,
     record_overrun and too_short. about 5,300 unknown and 5,100 malformed records.
3. **Targeted line probes** (`harness line`): the trailing `\r` is stripped; the last marker wins;
   lowercase digits give bad_digit; `lineFrame` with a NUL gives odd_digits or no_marker (finding m1);
   `lineIsDurable` reads byte 1 bit 0.
4. **Single-digit corruption.** All 1,260 single-hex-digit substitutions of a 42 B POSE line: 1,230
   bad_crc, 29 newer_version, 1 bad_version (finding S1).
5. **Existing tests.** Debug build 106/106 passed; Release build 106/106 passed, with 1 warning
   (finding S3).
6. **PROS toolchain** (arm-none-eabi-g++ 13.3.1, PROS MFLAGS, `-Os`, `--std=gnu++20 -Wall -Wextra`):
   10/10 sources compile with 0 warnings, 13,175 B of text.
7. **Coding-standard flags** (GCC 15.2, `-O2`, the host gate set plus the CMAKE-13 `-Werror=` list):
   `doc_assembler.cpp` fails (finding S3). There are 2 `-Wshadow` warnings (m4).
   `-Wconversion -Wsign-conversion`: 0.
8. **Other checks.**
   - No GATR includes: only std headers and the library's own.
   - All files are plain ASCII with LF endings.
   - clang-format 22.1.8 `--dry-run --Werror` over every .h and .cpp: clean.
   - Definition and estimate layouts match `translaGATR/link_documents.h:157-247` field by field.

## Findings

### Should-fix (before W0 is frozen)

- **S1, versioning: a corrupted line can read as "newer version".**
  - `decodeFrame` checks byte 0 before the crc (frame.cpp:111), and PROTOCOL.md tells readers to
    refuse the frame and show "publisher speaks vN".
  - A single flipped hex digit in the version byte yields kNewerVersion (29 of 1,260 substitutions
    above). The same happens when any cut or garbage line whose first byte is over 1.
  - VIEW `source.refused` then allows a false refusal or a false version message.
  - Fix: freeze the trailing CRC-16/CCITT-FALSE (over all bytes but the last two) with byte 0 for
    every version, and check it before naming a newer version. Alternatively, have W5 refuse only
    after repeated newer-version frames with no good v1 frame, and write that rule down in
    PROTOCOL.md.
- **S2, versioning: the durable bit is not frozen.**
  - The Pi relay reads flags bit 0 at a fixed hex offset without decoding (`lineIsDurable`, 5.5), yet
    the rule freezes only the line format and byte 0.
  - A v1 Pi relaying for a newer receiver could send durable frames down the latest-wins lane.
  - Fix: freeze byte 1 bit 0 as the durable bit in every version (one line in PROTOCOL.md).
- **S3, build: a new warning in optimized builds.**
  - GCC 15.2 at `-O2` and `-O3` warns `-Wmaybe-uninitialized` in `doc_assembler.cpp:46`. It is a
    false positive, through `std::optional<Partial>` holding vectors.
  - A CMake Release build shows 1 warning, against the "zero new warnings" rule. Under CMAKE-13
    `-Werror=uninitialized` the file fails at `-O2` and `-O3`. W9 packages and CI build optimized.
  - The Debug and PROS `-Os` builds are clean.
  - Tried in scratch: `slot.emplace()` clears `-O2` but not `-O3`. A `Partial` array without
    `std::optional` (`total == 0` means empty) gives 0 warnings at both levels. The
    `doc_assembler` and `robustness` tests pass on that variant (22/22 at `-O3`,
    `S/sp-build/w0-review-warn/da_fix3.cpp`).
- **S4, VIEW.md is missing part of 5.9.** It has nothing for the Pi channel table.
  - 5.9 says InspectionService "gains a small channel table that emits the same `def` and `samples`
    messages, so adding one needs no viewer code".
  - VIEW.md defines `def` and `samples` only as Brain records (with session, frame_seq and sent_ms),
    and `relay` only as untouched inspect/2.
  - Add the Pi form: the envelope, which fields are absent, and which clock they use.
- **S5, identity: field_id equality across Brain and Pi cannot hold with the 5.1 API as written.**
  - PROTOCOL.md says the definition is the link map byte for byte "so a Pi can compute the same
    field_id", and 7.2 tests a round trip from "Pi map doc -> Brain Field -> definition".
  - The 5.1 `FieldObject` has no box center offset or box heading, and no estimated or reference
    flags, so the Brain cannot re-encode the link map's bytes.
  - This is not a W0 code bug; it is a contract gap for W4 and W6. Either extend `Box` and
    `FieldObject`, pass the link-map bytes through, or reword PROTOCOL.md and drop the 7.2 check.
    The receiver rules (5.6) do not depend on it.

### Minor

- **m1.** PROTOCOL.md lists `lineFrame` among the receivers that discard 0x00 bytes. Only
  `LineScanner` does; `lineFrame` with a NUL gives odd_digits or no_marker. The Pi link runs with
  COBS off, so this is low risk. Fix the doc, or skip NULs in `lineFrame`.
- **m2.** Where PROTOCOL.md is loose:
  - "hex characters 6 and 7 after the marker" counts from the marker's first character. After the
    marker, the flags are characters 2-3.
  - "other lines" means non-empty lines only.
  - A text over its limit but up to 32 is accepted on read (for example a 20-character robot name);
    a text over 32 is malformed. The doc does not say either.
- **m3.** Versioning, DEF kinds: byte 3 is unit_len for kinds 0-2 and severity for kind 3, so a new
  DEF kind is ambiguous to a v1 reader. State that new DEF kinds use the channel layout, and that
  new flag bits never move existing fields.
- **m4.** `FrameError::kNoSession` shadows `protocol::kNoSession` (`-Wshadow`, host gate set, 2
  hits).
  - NAM-5 nit: `RecordKindCode` and `DocKindCode` carry the `Code` suffix but have no domain twin.
- **m5.** With the default `SPECTUGATR_BUILD_HOST=ON`, configure fails (`host/` is missing) until W3
  lands, and the README's plain `cmake -S spectuGATR` hits this too. This is already recorded as
  W0 deviation 17.
- **m6.** VIEW.md `samples` is reliable, where 5.9 made it replaceable per (src, kind). The reason in
  W0 deviation 14 is sound. W2 and W5 must bound the reliable queue for a stalled client and count
  what they drop.
  - 5.3 lists radio link quality under view/1 source stats; `stats` has no field for it yet. Adding
    fields keeps `/1`.
- **m7.** Nothing in PROTOCOL.md tells writers that each line must be one write call starting a
  packet. The 122 B limit assumes each line starts its own COBS packet. W4 should know this.

### Checked and fine

- Every record kind, identity and unit is present: 5.5 frame and record layouts, SESSION identities
  and the placement ref, POSE scales (mm, cdeg, ms, saturations), DEF, SAMPLES, EVENT, DOC_CHUNK,
  and the path document (38 + note + 12n). Every size in 5.5 is right.
- Encoding does not allocate: pointer and count APIs, fixed arrays, no heap in any encode path.
  W0's counting `operator new` test passes. Encoding is bounded: every loop has a fixed limit.
- `LineScanner` holds 245 digits at most, resyncs on a marker, and handles NUL, COBS, CR/LF,
  lowercase, overlong lines and NG1 lines as documented.
- The receiver-side `DocAssembler` allocates at most `total` bytes per kind.
- Versioning otherwise holds: unknown kinds are skipped by length, appended fields are ignored, and
  longer object records are read up to the known part. Both directions are tested (WIRE-9).
- No dependency on other GATR code. Code is namespaced `spectugatr::protocol`, uses aliases,
  structs for data and classes only for builder, scanner and assembler, and terse ASCII comments.
  PIPELINE.md and README.md are short.

## Commands (Git Bash; `export PATH=/c/msys64/ucrt64/bin:$PATH`)

```
cmake -S S/wt-spectu/spectuGATR -B S/sp-build/w0-review-build -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Debug -DSPECTUGATR_BUILD_HOST=OFF \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src      -> exit 0
cmake --build S/sp-build/w0-review-build -j16                              -> exit 0, 0 warnings
ctest --test-dir S/sp-build/w0-review-build -j16 --output-on-failure       -> 106/106 passed
same with -DCMAKE_BUILD_TYPE=Release into S/sp-build/w0-review-release     -> 106/106 passed,
    1 warning (-Wmaybe-uninitialized, stl_iterator.h via doc_assembler.cpp:46)
cmake -S ... -B S/sp-build/w0-review-default2 (HOST default ON)            -> configure error: host/ missing
g++ -std=c++17 -O1 -Wall -Wextra -I.../protocol/include harness.cpp .../protocol/src/*.cpp -o harness.exe
                                                                          -> 0 warnings
./harness.exe encode {1..5} 5000 > encN.jsonl; python check_encoded.py encN.jsonl
                                                                          -> errors 0 (each seed)
python fuzz.py 1 300; python fuzz.py {2..5} 400                           -> failing cases 0 (each)
arm-none-eabi-g++ (PROS flags, -Os, gnu++20, -Wall -Wextra) -c each src   -> 0 fails, 0 warnings
clang-format.exe (pip 22.1.8) --dry-run --Werror --style=file (all .h/.cpp) -> exit 0
g++ -O2 <CMAKE-13 -Werror= list> <host gate set> -c each src              -> doc_assembler.cpp fails
    (-Werror=maybe-uninitialized); 2 -Wshadow warnings; -Wconversion -Wsign-conversion: 0
da_fix3 variant (no std::optional<Partial>): -O2/-O3 0 warnings; doc_assembler + robustness
    tests at -O3: 22/22 passed
```

Nothing ran on a Brain, controller, radio or Pi. The COBS model is a standard COBS encoder with a
4-byte stream id, not the PROS kernel.
