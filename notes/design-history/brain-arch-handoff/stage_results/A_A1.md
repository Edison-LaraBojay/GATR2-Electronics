Stage A1 is finished: the v4 protocol files had no layout or naming bugs, I made two small behaviour fixes, and all 168 common tests pass with no compiler warnings. No wire layout, public name or signature changed.

**Review result.** I checked `common/frames.h`, `frame_codec.{h,cpp}` and `link_documents.{h,cpp}` line by line against spec sections 3 and 4. Everything matches, including the 40-byte state block offsets, the chunk sizes and the header-only handling of unknown ops, results and versions. The frame reader is unchanged from v3. The validator's wheel limits agree with the Pi's wheel models in `tracking_wheel_motion.cpp` and `brain_imu_wheel_bench.cpp`.

**Changes in `common/`:**
- **`link_documents.cpp`, IMU flags:** with IMU source none, a nonzero `imu_flags` is now rejected as `kProfileReasonImuSource` instead of `ImuPort`. This matches the Pico and VEX branches; B's existing expectation in `brain_io_gtest.cpp:1109` is not affected.
- **`link_documents.cpp`, header writers:** `encodeFieldMapHeader` and `encodeFieldEstimateHeader` now refuse a buffer smaller than the whole document (`fieldMapLen` / `fieldEstimateLen` of the count). Before, a 24-byte header buffer was enough, so a too-small buffer failed later or not at all.
- **Cosmetic:** constant alignment, one line wrapped to 100 columns, and more exact comments on the `ProfileReason` values (WheelGeometry, ImuSource, Footprint) and on `validateFieldEstimate`.
- **`common/tests/CMakeLists.txt`:** added `link_documents_gtest.cpp` and `../link_documents.cpp`, listed explicitly.

**Tests:**
- **`frames_gtest.cpp` (14 tests, rewritten):** all v4 constants and struct defaults, and a check that ops 3 and 5 and results 6 and 7 are not reused.
- **`frame_codec_gtest.cpp` (96 tests, rewritten):**
  - **Known-byte vectors:** 24 literal vectors covering every op and every reply body. They came from a separate Python packer, which reproduces the old v3 vectors exactly, and each is checked in the test with a separately written CRC-16.
  - **Size limits:** the largest bodies fill the 128-byte frame (PROFILE_WRITE 106 bytes, READ_DOC 96 bytes); PATH_REPORT with 13 points is 124 bytes.
  - **Length rules:** lengths are checked for every op and every (op, result) pair, and outside-range lengths are rejected. Unknown or retired ops and results, and other versions, decode as header only.
  - **Frame reader:** covers 128-byte frames, and all the old rescan cases are kept.
- **`link_documents_gtest.cpp` (58 tests, new):**
  - **CRC-32:** returns the check value 0xCBF43926 and agrees with a table-driven reference.
  - **Profile validation:** all 9 topology × IMU source combinations. Every shared `ProfileReason` is covered, including the observability boundaries (57 vs 58 mdeg for two wheels, 31 vs 33 µm for three wheels). Build and NotAccepted come only from the Pi.
  - **Map and estimate:** maps and estimates with 0, 1, 2, 7, 37, 127 and 128 objects. Every `DocError` is triggered, including duplicate and swapped ids, reserved bytes and an estimate checked against the wrong map.

**Commands and results:**
- Build: `cmake -S common/tests -B $S/wA-common -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$S/nav-base/_deps/googletest-src`, then `cmake --build $S/wA-common -j8`.
- Run: `ctest --test-dir $S/wA-common --output-on-failure -j8` gave **168/168 passed**.
- Mutation check in a scratch copy (`$S/wA-mut`): I planted 20 deliberate bugs and the tests caught 19. The one they missed (raising the PATH_REPORT point limit to 14) cannot change behaviour, because the length check already caps the count at 13.
- Pico context, compile check only (no `pio`): host g++ `-std=c++17 -Wpedantic` on `frame_codec.cpp` passed, and the PlatformIO arm-none-eabi-g++ 14.3 `-std=gnu++17 -mcpu=cortex-m0plus` also passed. The Pico only uses the sensor-frame part of the codec, which v4 did not change.
- Brain context: PROS arm-none-eabi-g++ 13.3.1 gnu++20 check of `frame_codec.cpp` and `link_documents.cpp` passed.

**For the lead and other workers:**
- The v3 functions `brainRequestLen` and `brainReplyLen` are gone; v4 uses `brainRequestMinLen/MaxLen(op)` and `brainReplyMinLen/MaxLen(op, result)`. Two places still use the old names: `pi/navigatr/src/impl/commands/brain_link_commands.cpp:228` (B) and `brain/communiGATR/sim/fake_pi.cpp:64` (mine, next stage).
- For the per-request timeout formula, the largest reply for any op is `brainReplyMaxLen(op, kResultOk)`.
- B's code already includes `link_documents.h`, and `link_documents.cpp` is already listed in `gatr2_brain.mk` and `pi/navigatr/CMakeLists.txt`.

**Contract notes (not blocking, also in my progress file):**
- For two forward wheels, only angles of exactly 0 and ±180000 mdeg are accepted; 360000 is refused, which is stricter than the Pi model but matches the spec wording.
- The shared check does not range-check `encoder_port` or `imu_port`; which ports exist is the Pi's check, so the Brain cannot pre-check HAT v2 ports with it.
- The reply decoder does not check that a READ_DOC chunk's offset plus length stays within the document's total length; the client's document assembly must check it in my next stage.

**Still to do:** `docs/interfaces.md` still describes v3; I rewrite it in the communiGATR stage.

Progress file: `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/brain-arch-handoff/progress/wA.md`