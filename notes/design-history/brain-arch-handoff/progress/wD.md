# Worker D progress (Pico firmware and Pi Pico link)

## Stage D1: Pico firmware (spec 8.4 Pico part)

Status: DONE 2026-09-27. No earlier wD work was on disk (pico/ was clean at HEAD d4bc875),
so D1 started fresh. Only pico/** was edited. Nothing flashed.

### Files
New portable units (no Arduino includes), all host tested:
- pico/src/imu_retry.{h,cpp}: `imu::RetryPolicy` (episode, attempts, reason, exhausted),
  `imu::DeviceView`, `wireState()`, `wireReason()`; constants kQuickAttempts 5,
  kBackoffMinUs 0.5 s, kBackoffMaxUs 8 s, kSlowRetryUs 30 s, kStableUs 1 s.
- pico/src/asm330_supervisor.{h,cpp}: `asm330::Supervisor`, non-blocking
  Wait/Probe/Reset/Configure/Run with actions Probe, PollReset (1 ms rate, 100 ms bound),
  Configure (readback), Check (every 250 ms, 2 consecutive failures restart the chip).
- pico/src/command_records.{h,cpp}: `pilink::CommandRecords`, last 4 (request_id, op, body)
  with status/detail; same id replaces its record; else empty slot, else oldest.
- pico/src/pico_commands.{h,cpp}: `pilink::Commands` (receive -> Outcome{answered, Effect},
  imuProgress(state), fill(PicoStatus) for last_*).
- pico/src/pi_link.{h,cpp}: `pilink::makeBootId(entropy, time_us)` (murmur3 finalizer, 0 maps
  to 0xA5A5), `airtimeUs()`, `pilink::StatusSchedule(period, min_gap, guard)`.
Changed:
- pico/src/bno08x_supervisor.{h,cpp}: states Off/Reset/Boot/WaitAck/Run; holds from
  RetryPolicy; start(), restart() (10 ms reset pulse, new episode), stop(); Events.open_failed;
  epoch(); holdUs() replaces backoffUs(). Boot 500 ms, ack 200 ms, stale 100 ms kept.
- pico/src/imu_bno08x.cpp: sh2_open is retried (openOrRelease on every ReleaseReset until it
  succeeds), setEnabled/reinit/status/firmware, diagnostics add attempts and epoch,
  DISABLED/OPEN_ERROR states; bounded blocking documented in the header comment.
- pico/src/imu_asm330.cpp: driven by asm330::Supervisor; the 100 x delay(1) reset loop is gone;
  WHO_AM_I failure retried; power down on disable.
- pico/src/imu.h: `imu::Status`, setEnabled(), reinit(), status(), firmware().
- pico/src/quadrature.{h,cpp}: `encoder::zeroAll()` (noInterrupts around all three).
- pico/src/config.h: kStatusPeriodUs 200 ms, kStatusMinGapUs = kTickUs, kStatusMarginUs 1 ms,
  kRxFifoLen 128, kRxBytesPerPass 64.
- pico/src/main.cpp: v2 sensor frames (identity), boot id from rp2040.hwrand32() (pico_rand:
  ROSC bits and timer) mixed with time_us_64, RX command parsing via FrameReader, effects,
  watchImu, status frames in the idle window, USB `link` diagnostics line.
- pico/tests/CMakeLists.txt: 5 new test files, 5 new sources + common/frame_codec.cpp,
  include dir ../../common.
- pico/tests/bno08x_supervisor_gtest.cpp: rewritten for the retry policy (13 tests).
- New tests: imu_retry_gtest.cpp (10), asm330_supervisor_gtest.cpp (8),
  command_records_gtest.cpp (6), pico_commands_gtest.cpp (17), pi_link_gtest.cpp (12).
- pico/README.md: Pi link, IMU retry, bounded blocking, monitor lines, build; ASCII only.

### Design decisions
- Frames: v2 max 31 bytes (static_assert <= 32), status 26 bytes (static_assert). TXFE rule
  kept: a busy FIFO skips the tick. Status only when TXFE and next_tick - now >= airtime(26)
  + 1 ms (3257 us), every 200 ms or >= 20 ms after the previous one when requested (command
  answered, IMU state/reason/attempts/epoch/enabled changed, REINIT record settled).
- Commands:
  - Ignored (no status change): link version != 1, request_id 0, bad CRC/envelope.
  - Failed, not recorded: WrongTarget (target != boot_id, target 0 never matches),
    UnknownOp, BadBody for a known op with the wrong length (header parsed locally because
    decodePicoCommand refuses such frames).
  - Recorded (duplicate-proof): every other command for this boot, including content
    failures (CONFIGURE value > 1: BadBody; REINIT port != 0: NoSuchPort; REINIT while
    disabled: ImuDisabled).
  - A duplicate never runs again; last_* then shows its record. last_* = newest command frame
    received (new or duplicate), so the Pi can query an older request by resending it.
  - Same id with another op or body is a new command and replaces the old record.
  - CONFIGURE with the current value: Completed, no IMU restart. IMU enabled at boot.
  - REINIT_IMU: Running; Completed when the state reaches Ready (BNO08X: after gravity
    alignment, so it needs stillness); Failed ImuAbsent when the retry is exhausted; Failed
    ImuDisabled if disabled meanwhile. Several running REINITs settle together.
  - RESTART_ACQUISITION: zeroAll with interrupts off, acq_epoch + 1, Completed, applied before
    the next sample.
- Retry: episode starts at boot, enable, REINIT, or loss of a device that ran 1 s. Waits after
  failed attempt n: 0.5, 1, 2, 4 s (doubling, 8 s cap unreachable with 5 attempts), then Failed
  and 30 s. Loss of a stable device waits 0.5 s and opens a new episode. Failed persists
  through slow attempts until one comes up. Reason reported only while not up.
- imu_epoch: BNO08X +1 at start, restart, every failure (reset hold) and every hub reset after
  boot (Enable from WaitAck/Run); ASM330 +1 per probe. Both wrap (u8), compare for change only.
- Wire IMU state: Disabled / Ready / Aligning (BNO08X Run, not aligned) / Failed (exhausted) /
  Retrying (waiting after a failure) / Initializing.
- Bounded blocking left: BNO08X sh2_open (10 ms pulse + up to 200 ms boot poll) until the first
  success (normally once in setup); failed open returns after the pulse. WAKE handshake <= 2 ms.
  sh2_setSensorConfig completes at send.

### CONTRACT ISSUES
- None blocking.
- Note for Worker A (no change requested): decodePicoCommand returns false for a known op with
  the wrong body length, so the Pico parses the header itself (pico_commands.cpp readHeader)
  to report BadBody. Duplicates the 6-byte header offsets.
- Interpretation for the lead: "5 quick attempts, 0.5 s doubling to 8 s" gives waits 0.5, 1, 2,
  4 s; the 8 s cap is kept as a constant but not reached. docs/interfaces.md wording matches.

### Test results (host, Git Bash, ucrt64 first on PATH)
```
cmake -S /c/GATR2-Electronics/pico/tests -B "$S/wD-pico-tests" -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$S/nav-base/_deps/googletest-src"
cmake --build "$S/wD-pico-tests" -j8
ctest --test-dir "$S/wD-pico-tests" --output-on-failure -j8
```
- Baseline before changes: 53/53.
- After D1: 110/110 passed (bno08x_reports 9, gravity_alignment 15, gyro_window 13,
  shtp_header 7, bno08x_supervisor 13, imu_retry 10, asm330_supervisor 8, command_records 6,
  pico_commands 17, pi_link 12). 0 compiler warnings.
- Firmware (build only), from pico/:
  `PLATFORMIO_WORKSPACE_DIR="$S/wD-pio" pio run -e hat2_bno08x -e hat2_asm330`
  clean build: both SUCCESS, 0 warnings. hat2_bno08x RAM 16368 B, flash 94328 B;
  hat2_asm330 RAM 10920 B, flash 86272 B. Nothing written to pico/.pio.

### Risks / for the lead and D2
- DO NOT FLASH this firmware before the Pi side decodes v2: current
  pi/navigatr/src/impl/resources/pico_telemetry.cpp skips any frame type other than 0x01
  (line ~76), so a v2-only Pico gives the Pi no encoder data. Planned in D2 (spec 8.4 Pi).
- boot_id is 16 bits: 1 in 65535 reboots repeats the previous id; the Pi's stamp-regression
  fallback still catches most of those.
- PicoControl (D2): the status after a burst of commands names only the newest; resend each
  outstanding request (every 200 ms) until its id appears. One outstanding command at a time
  is simplest.
- Nothing is hardware validated: RX parsing, status timing on the real UART, sh2_open retry,
  ASM330 health checks and REINIT on a real BNO08X are software-tested only.
- ASM330 health check restarts the chip after 2 bad checks (500 ms); each restart changes
  imu_epoch, so the Pi recalibrates bias (needs stillness).

## Stage D2: Pi side of the Pico link (spec 8.4 Pi part, 8.7)

Status: DONE 2026-09-27 23:35. No D2 edits were on disk at the start:
pico_telemetry.{h,cpp} and serial_links.{h,cpp} are as committed in d4bc875 (lead's minimal
PicoControl). Build dir $S/wD-nav.

### Done so far (D2)
- pico_telemetry.{h,cpp}: v1 + v2 sensor frames and status frames decoded; encoder epoch
  (reboot or acq change) and IMU epoch (reboot or imu change) are the per-output upstream
  epochs; reboot = new boot_id, frame version change, or device clock regression (v1
  fallback, repeated boot id). Real PicoControl (see design). refresh(ExecutionContext) now.
  deviceEpoch() replaced by encoderEpoch()/imuEpoch() (no other users).
- serial_links.{h,cpp}: ReopeningLink (opener, 1 s retry from reads, never waits);
  linux_serial_link wraps LinuxSerialLink in it; LinuxSerialLink reports closed on a hung-up
  tty (read 0 + poll error/hangup). required keeps its startup meaning.
- pico_control.h: comments only (any-thread calls, failure details); no signature change.
- pico_telemetry_samples.h: comment only.
- tests/pico_link_gtest.cpp (27 tests, 26 on Windows; the pty test is POSIX only) + one line in
  tests/CMakeLists.txt.
- docs/pico_link.md.

### Design decisions (D2)
- Identity (pico_telemetry): reboot = new boot_id, frame version change (v1 <-> v2), or device
  clock regression (the v1 fallback; also a repeated 16-bit boot id). Reboot: encoder epoch +1,
  IMU epoch +1, gyro reseeded, reboots +1, unsettled commands of the old boot fail PicoLink.
  acq_epoch change: encoder epoch +1 only (IMU untouched, no needless bias recal). imu_epoch
  change: IMU epoch +1 and gyro reseeded only. Encoder outputs publish upstream.epoch =
  encoderEpoch(), the IMU output imuEpoch(). Link close/reopen with the same identity: nothing.
- v1 frames: identity false, resource diagnostic "v1 sensor frames: no Pico identity, reboots
  found by clock regression, commands unavailable", submit() returns 0.
- PicoControl: request ids from a random 16-bit base per PicoTelemetry (random_device + clock),
  skip 0; target = boot_id at submit. All I/O in refresh(); submit()/request()/link() lock a
  mutex (safe from any thread, incl. the main-thread profile boundary). One command frame per
  100 ms at most; each unsettled command resent every 200 ms under its id, oldest first,
  except a Running one the newest status already names. Completion only from status frames
  (boot, request id and op must match). Timeout: Failed PicoLink if never reported, TimedOut
  if reported. Pico detail map: ImuAbsent -> ImuAbsent, WrongTarget -> PicoLink, everything
  else -> PicoRefused. At most 16 requests kept (settled evicted first). reset() keeps
  identity, epochs and commands (they describe the Pico), clears decoder and channels.
- frames_fresh: a sensor frame within 250 ms (host clock) and the serial link not closed.
  status_known: newest status frame is of the current boot; status zeroed otherwise.
- linux_serial_link: resource is a ReopeningLink(opener, first device). Retry from reads only,
  at most once per 1 s since the last attempt (the build-time open counts), never waits;
  writes fail while closed. LinuxSerialLink reports closed on a read error or when an idle read
  finds the tty hung up (poll). DE: set up on the first link even when the tty failed (pin
  driven low), and again on each reopen. required=true keeps the startup hard error.

### Test results (D2, host, Git Bash, ucrt64 first on PATH)
- Own files: 0 warnings (-Wall -Wextra). pico_link_gtest: 29 tests, 28 run on Windows (the pty
  reopen test is POSIX only; its code was syntax-checked here with a shim, not run).
- Consistent B snapshot ($S/wD-ctl/src, B files copied ~22:50, my files current):
  `cmake --build $S/wD-ctl/build -j8; ctest --test-dir $S/wD-ctl/build -j8` -> 465/465 passed
  (before the last 3 pico_link tests were added; those 28/28 pass in the same build).
- Control: same snapshot with my 6 files at HEAD: the 3 B tests that failed in my first
  full-tree run (TrackingWheelMotion.CalibrationRebasesWheelsAndRestartsOnMotion,
  ImuHeadingIncrement.CalibratesThenIntegratesWithProvenance,
  BrainProfile.VexRecalibrateHasNothingToCalibrateOnThePi) pass with and without my files, so
  those failures came from an older, inconsistent B4 state, not from D2.
- Mutation check (script $S/wD-mutate.sh, log $S/wD-mutants.log + wD-mutants2.log): 24 mutants
  of pico_telemetry.cpp / serial_links.cpp, 23 killed. Survivor M22 (no FrameReader reset on
  close) is not observable: the reader resyncs on the bad checksum anyway.
- Full current tree ($S/wD-nav): does NOT build since ~23:00 because B4 is mid-rework after the
  user's 8.8 scope change (tracking_wheel_motion.cpp refers to bridge_max_ms_ etc. that its
  header no longer declares). Outside my files; retrying.
- Final (after the 3 extra tests and the timeout clamp): snapshot build $S/wD-ctl/build,
  467/467 passed (439 others + 28 pico_link), 0 warnings in rebuilt TUs; pico_link filter
  30/30 repeats. Mutants rerun on the added tests: M05, M20, M23 now killed (23/24 total).

### CONTRACT ISSUES (D2)
- None blocking. pico_control.h: comments only (any-thread calls, failure detail set, timeout
  clamp). No signature or struct change, so B's fake PicoControl is unaffected.

### For Worker B / the lead
- submit() returns 0 with v1 frames, stale frames (> 250 ms), a closed link, an unknown op or a
  nonpositive/NaN timeout: map 0 to Failed PicoLink. Timeouts above 1 h are clamped.
- request(): kSending -> kRunning -> kCompleted / kFailed(detail). Details: PicoLink (never
  reported before the timeout, or Pico rebooted, or WrongTarget), TimedOut (reported but not
  settled), ImuAbsent, PicoRefused (UnknownOp, BadBody, NoSuchPort, ImuDisabled).
- RESTART_ACQUISITION can be reported Completed before the first sensor frame with the new
  acq_epoch; for CONTROL 4 wait for link().acq_epoch (or link().restarts) to change.
- link().reboots / restarts / imu_restarts and frames_fresh transitions are the inputs for the
  recovery event log (Pico reboot, acquisition restart, IMU reinit, link lost/restored).
- Health bits: PicoLink = link().frames_fresh; ImuInitializing / ImuFailed from
  link().status (status_known first; status is zeroed when not known).
- Docs owned by B that are now stale: docs/navigatr_resources.md (linux_serial_link "dead link"
  wording: it now reopens every 1 s; pico_telemetry now also decodes v2 and status frames and
  sends commands), docs/hardware.md "Pico-to-Pi telemetry" (the Pi now transmits commands to
  Pico GP17 RX), parallel_wheel_bringup.md:179 and calibration_inventory.md:65 (IMU reset now
  restarts bias calibration). pi/navigatr/docs/pico_link.md is the reference.
- pico/README.md now links pi/navigatr/docs/pico_link.md (one line).
- Flash order: Pi software with D2 first, then the v2 Pico firmware.

### Risks (D2)
- Not on hardware: Pi TX -> Pico RX (GP17) wire on HAT v2, command RX and status timing on the
  real UART, reopen of a real /dev/ttyAMA0 or RS-485 device.
- The pty reopen test only runs on Linux (CI); here it was syntax-checked with a shim only.
- LinuxSerialLink hangup detection adds one poll() per idle read; a UART that reported
  POLLERR/POLLHUP while healthy would be closed and reopened (Linux n_tty does not).
- Reopen with DriverEnable re-runs the sysfs setup; on a freshly exported pin that can sleep up
  to 200 ms (only when the pin was not already exported).
- Current tree, after B4 settled (23:29): `cmake --build $S/wD-nav -j8` exit 0, 0 warnings;
  `ctest --test-dir $S/wD-nav -j8` 499/499 passed (includes the 28 pico_link tests). Between
  ~23:00 and 23:29 the tree did not build because of B's mid-edit tracking_wheel_motion and
  brain_imu_wheel_bench (18 errors, none in D files); recorded, waited, retried.
- B's current code already reads PicoControl::link() (brain_link_publisher, system.cpp via
  runtime/pico_control_ref.h); submit() is not called yet.

## Stage FIX (reviewer finding on D1/D2)

Status: DONE (doc only). One finding, medium, pi/navigatr/docs/pico_link.md.

### Finding 1: pico_link.md said the pose holds after a Pico reboot / restart / IMU reinit
- Verified real. runtime/sensor_loss.cpp raises "Pico rebooted", "Pico acquisition restarted"
  and (IMU source pico only) "Pico IMU restarted" edges, plus encoder discontinuity edges;
  System::watchSensors -> loseSensor -> loseContinuity (unplaced, epoch + 1, SET_POSE
  withdrawn) with unplace_on_sensor_loss default true (brain_profile_builder.h:112).
  system.cpp:618/625 also call loseSensor after CONTROL 3/4 complete. The monitor is
  configured only on profile apply (system.cpp:848), so XML-configured localization is
  unaffected; CONTROL needs an applied profile (NotReady), CONTROL 3 needs imu_source pico.
- Fixed in pico_link.md:
  - Identity bullets: bias recalibration kept; new bullet that a restart of a used source
    invalidates the pose with a Brain profile (link to the table).
  - Serial reopen: a gap longer than sensor_loss_ms still invalidates the pose.
  - Reboot/restart table split into "Pose, Brain profile" and "Pose, XML config" with the
    8.10 rule (used sources, stale > sensor_loss_ms, used Pico IMU not ready, identity
    change), `<BrainProfile><Timing on_sensor_loss="warn"/>` keeps the pose, pointers to
    docs/brain_setup.md sections 8 and 6.
  - Tests line: the restart tests run an XML-configured three-wheel pipeline.
- No code change: pico_link_gtest restart tests use XML localization, where the pose holds.

### Test results (Stage FIX)
- Pico host tests ($S/wD-pico-tests): build exit 0, 0 warnings; 110/110 passed.
- pio run -e hat2_bno08x -e hat2_asm330 ($S/wD-pio, build only): both SUCCESS, 0 warnings,
  RAM/flash unchanged (16368/94328, 10920/86272).
- pi/navigatr current tree ($S/wD-nav, incremental; B's files had changed and rebuilt):
  `cmake --build $S/wD-nav -j8` exit 0, 0 warnings; `ctest --test-dir $S/wD-nav -j8`
  505/505 passed, including the 28 pico_link tests (PicoLinkIdentity, PicoLinkRestarts,
  PicoControl, SerialReopen). The Linux-only pty reopen test is not run on Windows.
- Doc is plain ASCII (0 non-ASCII bytes).
