Stage B1 is done: the Pi build compiles again and speaks v4. Full build with `-Wall -Wextra` has no warnings, and all 377 navigatr tests pass.

**Build and test commands** (Git Bash, `/c/msys64/ucrt64/bin` first on PATH, `$S` = scratchpad):
- Configure: `cmake -S C:/GATR2-Electronics/pi/navigatr -B "$S/wB-nav" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$S/nav-base/_deps/googletest-src"`
- Build: `cmake --build "$S/wB-nav" -j8` exited 0.
- Tests: `ctest --test-dir "$S/wB-nav" -j8` gave 377/377 passed.
- Mutation check: I disabled, one at a time, the SET_POSE NotReady gate, the same-body check on newest-id resends, and the resend byte compare. Each made the expected BrainLink tests fail. The source is restored byte for byte, rebuilt, and 377/377 again.

**Files changed** (all under `C:/GATR2-Electronics/pi/navigatr`):
- **Build:**
  - `CMakeLists.txt` adds `common/link_documents.cpp` to navigatr_core.
  - `tests/CMakeLists.txt` drops `brain_link_e2e_gtest.cpp`, `BRAIN_SOURCES` and the Brain include dirs. The v3 test file is left on disk, uncompiled, for your Phase C rewrite.
- **New seam, `src/contracts/brain_profile.h`:** `BrainProfileHost::prepare(const gatr2::RobotProfileDoc&, uint32_t id, uint8_t& reason, uint8_t& detail)`. A pointer `BrainProfileHost* brain_profile` sits on `SlotInitializationContext`; null means the config takes no profile. Nothing sets it yet; the System does that in B2.
- **Removed:** `world_estimation_noop` from `slot_init.h` and `runtime/system.cpp`. Only the v3 SELECT reply used it.
- **`src/state/command_state.h`:**
  - `BrainReplyContext` carries the v4 fields.
  - `CommandState` gains `ProfileStatus profile` (state, reason, detail, id, applied_id) and `PathReport path` (session, command id, mode, up to 13 points in metres, host receipt time).
- **`brain_link_commands.{h,cpp}`:**
  - Ops 3 and 5 are UnsupportedOp.
  - Malformed bodies get InvalidArgument before the session and dedupe checks, and never use up the request id.
  - Dedupe follows spec section 3. SET_POSE and CONTROL repeats are answered from their records. A resend of the newest request with the same body is executed again; a resent GET_STATE does not re-accept its IMU sample.
  - GET_STATE samples go to the bench IMU mailbox when one is configured and are ignored otherwise.
  - HELLO keeps the profile and staging and clears the stored path.
  - SET_POSE is NotReady while a profile host exists and no profile is applied.
  - PROFILE_WRITE staging and PROFILE_APPLY follow spec 5.1. Rejections are remembered for the last 4 ids, and without a host APPLY answers ProfileRejected / NotAccepted.
  - CONTROL is NotReady (B3). PATH_REPORT stores the path; mode none clears it.
- **`brain_link_publisher.{h,cpp}`:**
  - The state block reports the profile status; map_id, estimate_id and calibration are 0/none.
  - READ_DOC answers Unavailable until B2.
  - A `<FieldObject>` element in a config now fails with a clear "removed with brain link v4" error, and unknown children are rejected.
- **Review finding F, `src/transport/serial_port.{h,cpp}`:** CRTSCTS is cleared after `cfmakeraw`. The `tcdrain` fallback is replaced by non-blocking TIOCOUTQ polling, so the wait while DE is held is bounded by the deadline. This is the Linux-only code path and was not compiled: there is no Linux compiler on this box (no WSL).
- **Tests updated:**
  - `brain_io_gtest.cpp` rewritten for v4; it adds tests for profile staging and apply, and for path reports.
  - `brain_imu_bench_gtest.cpp`: the perpendicular-wheel tests are unchanged and still pass.
  - `pros_usb_link_gtest.cpp` adds a round trip of the largest 128-byte request and reply through the NG1 envelope.
  - `half_duplex_gtest.cpp` adds the 128-byte frame budget.
  - `end_to_end_gtest.cpp` and `parallel_wheels_gtest.cpp`: their SELECT steps are replaced with PATH_REPORT and READ_DOC.
- **Config comments** that described FieldObject/SELECT: `config/shared/pipelines/parallel_wheels_bno08x_camera.xml` and `config/override/diagnostics/three_wheel_imu_camera_pipeline.xml.in`.

**For Worker A and you:**
- A re-APPLY of a rejected id gets the remembered rejection until the Pi restarts, as spec 5.1 says. So `resubmitProfile()` with an unchanged document will not force the Pi to rebuild; only a changed document retries.
- On the Pi, a PROFILE_WRITE for a new profile id must start at offset 0. Staging is a single slot.
- In B2 the System must set `slot_context.brain_profile` before building CommandCollection. At the swap it must write `command_.profile` (Applied, or Rejected with kProfileReasonBuild).
- The target-selection fields in `CommandState` (`object_*`) are kept, but no v4 op sets them. `configured_targets` therefore has no production trigger; decide in Phase C whether to retire it.

**Not done yet:**
- Pi docs still describe v3; that is the docs stage.
- Inspection output for the profile and path waits for spec 5.5.
- No new code has been checked on hardware.

The progress file with full details is `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/brain-arch-handoff/progress/wB.md`.