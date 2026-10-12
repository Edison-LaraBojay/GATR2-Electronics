# W3 fix pass: review findings on capture, export, triggers, state tap

Date: 2026-09-28. Host: Windows 11, MSYS2 UCRT64 GCC 15.2, MinGW Makefiles, Debug, -Wall -Wextra. All results are host tests or host probes. Nothing ran on the Pi, the Pico or the Brain.

## Summary

All six findings were accepted and fixed; none was declined. Only W3 files were edited:
- `pi/naviGATR/src/capture/{capture_bundle.h,capture_bundle.cpp,capture_recorder.h,capture_service.cpp,zip_writer.h,zip_writer.cpp}`
- `pi/naviGATR/tests/capture_gtest.cpp`
- `pi/naviGATR/docs/capture.md`

`system.cpp` and `robot_state_feed` are unchanged in this pass.

## Per finding

### 1. (medium) metadata.json said `truncated:false` when the CSV size bound dropped rows: FIXED

- `buildCaptureBundle` computes one verdict and returns it in the output:
  - `out.truncated_by` holds the stop reason when the recorder truncated, plus `bundle_size_limit` when any rows were omitted;
  - `out.truncated` is true when that list is not empty.
- `metadata.json` `trigger` and the status entry both write `truncated` and `truncated_by`. `stop_reason` still says only why recording stopped, so a capture can be `post_window` and truncated by `bundle_size_limit`.
- Hub-ring drops stay in `drops`, not in `truncated`. The docs and README say so.
- Test: `CaptureRecorder.BundleSizeLimitMarksTheBundleTruncated` (the reviewer's probe scenario). It asserts:
  - the same `"truncated":true,"truncated_by":["bundle_size_limit"],"stop_reason":"post_window"` in the metadata and the status;
  - rows + omitted = 6000.

  `RecordLimitTruncatesEarly` and `PostWindowStopsAtTheEnd` also check `truncated_by`.

### 2. (medium) Host metadata was read only at finalize: FIXED

- **Sampling.** While a rolling window runs, the recorder reads `CaptureHost::writeMetadata` once a second on the capture thread, using the same thread-safe getters as before. It keeps readings back to one at or before `now - rolling_s`.
- **Choice at the trigger.** `begin()` keeps the newest reading at or before the window start. When there is none, it uses the oldest reading; when there are no readings at all (no rolling window), it reads now.
- **metadata.json.**
  - The top-level `configuration`, `profile`, `localization`, `pico_link` and `software` are read at the end, as before.
  - New `host_sampled_pi_host_us` says when that was.
  - New `host_at_start` has `{sampled_pi_host_us, the same keys}`.

  The README and docs explain both readings.
- **Why not read in `begin()`, as suggested.** For `profile_changed` and `pico_reboot`, System notes the fault after the change. A read in `begin()` would therefore record the new profile or boot in both places. A mutation test confirms this: reading at `begin()` fails both new tests.
- **Tests.**
  - `CaptureRecorder.HostMetadataAtTheWindowStartAndAtTheEnd` covers three cases:
    - a change during the post window;
    - a change just before an automatic `profile_changed` trigger;
    - no rolling window.
  - `CaptureSystem.PicoRebootFiresTheAutomaticCaptureWithBothBoots` runs the real System host: `host_at_start.pico_link.boot_id` is 4369 (the old boot) and the top level is 8738 (the new boot).
- **Cost.** One small JSON string per second, kept for `rolling_s`.

### 3. (low) `rolling_evicted` was a lifetime counter: FIXED

- Evictions are counted per millisecond of the evicted record's own Pi stamp, in a deque of `(ms, count)` entries trimmed to `rolling_s`.
- A capture sums only the entries at or after its window start. The window's first millisecond counts whole, and the docs say so.
- Test: `CaptureRecorder.RollingEvictedCountsOnlyThisCapturesWindow`.
  - The reviewer's scenario now gives 0 with `pre_s_actual` 10.
  - A later burst gives exactly the window's posted records minus the rows.
  - A pre-window shorter than the rolling one counts 150 of the 400 evicted.

### 4. (low) `max_mb` did not bound record memory (vector doubling): FIXED

- `Active::records` is now a `std::deque`.
- Measuring after that change showed that the build peak came from the single growing CSV string. The fix pass therefore also:
  - writes CSV text in chunks of at most 1 MiB. `ZipEntry` gained `chunks`, the ZIP writer appends them into its reserved output, and the CRC is `translagatr::crc32` over the written region, patched into the header;
  - releases the records once the rows are written (`CaptureBundleInput::rows_written`).

Measured with the scratch probe `$S/w3-probe/mem.cpp` on the same Windows host, Debug -O0, UCRT heap; not the Pi. Defaults: `max_mb` 64, cap 399457 x 168 B = 64.0 MiB. Only the active capture holds records.

| Build | At the cap | Peak through the bundle build |
|---|---|---|
| Before: `std::vector` of records (reviewer probe) | 87.9 MiB | not measured |
| Deque, one growing CSV string | 72.9 MiB | 242 MiB above start |
| Deque, chunked CSV text, records released before the ZIP (final) | 72.9 MiB | 132 MiB above start |

- The docs memory table was rewritten with these numbers and their conditions. The glibc overhead on the Pi was not measured.
- Test: `CaptureBundle.ZipRoundTrip` now includes a chunked entry, checked by the independent reader and CRC table. There is no memory assertion in the repo tests.

### 5. (low) `enc*_delta_counts` spanned lost frames: FIXED

- A delta is now filled only against the same port in the frame with `seq - 1` (mod 256) of the same boot and acquisition epoch. It is empty after a gap, and when the port was missing from the frame before.
- New last column `seq_gap_frames`: the frames missing between the previous recorded frame of that acquisition and this one (mod 256). It is empty for the first frame of an acquisition.
- The raw counts stay. The README and docs explain how to span a gap, and that the count is modulo 256.
- The viewer reads only `enc*_counts` (checked `spectaGATR/replay.js`).
- Test: `CaptureBundle.EncoderDeltasOnlyBetweenConsecutiveFrames` sends frames with seq 1, 2, 5, 6 (port 1 missing), 7, 255 and 0 (wrap), and checks the deltas and gaps per row.
- `SegmentsAtSessionEpochBootAndResetChanges` now sets seq.

### 6. (low) System wiring had no tests: FIXED, with one remaining gap

- **`CaptureSystem.BrainLinkQuietFiresTheAutomaticCapture`.** A plain `<System>` with the brain_link over a memory link:
  - HELLO and GET_STATE requests, then 1.25 s of host time with no requests;
  - the status shows `fired 1` and `last_reason link_lost`;
  - metadata has `detail` "Brain link quiet for 1 s";
  - `events.csv` has "Brain link quiet for 1 s" and "Brain session opened" with kind runtime and source system. That is the `System::noteEvent` to hub path;
  - a second quiet edge inside the cooldown gives `seen 2`, suppressed as cooldown 1, and no second bundle.
- **`CaptureSystem.PicoRebootFiresTheAutomaticCaptureWithBothBoots`.** The shipped `brain_profile_usb.xml` is resolved by composition. Both devices become memory links, and `<Capture>` is inserted into the resolved System document.
  - A new `boot_id` fires `pico_reboot`, and both boots are in the metadata, as in finding 2.
  - "Pico frames lost" during the recording is suppressed as busy.
  - Both events are in `events.csv`.
- **Reset test.** `ResetIsASegmentBoundaryAndEventsAreRecorded` now checks the trigger marker and the "Pi reset: reset count 1" marker in `events.csv`.
- **Remaining gap.** The `continuity_lost` and `profile_changed` edges in System are still not driven by a test. The recorder side is tested, and the docs say so.

## Test commands and results

```
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake --build $S/w3-nav -j16
$S/w3-nav/tests/navigatr_tests.exe --gtest_filter='Capture*' --gtest_repeat=3
ctest --test-dir $S/w3-nav -j16
```

**Live tree** (`$S/w3-nav`)
- 0 warnings.
- `Capture*`: 32/32 in each of 3 repeats: 28 W3 tests (22 before, 6 new) and 4 CaptureTransform.
- Full `ctest -j16` at about 06:00: 612/614. The two failures are W1's `InspectionFeed.IdenticalDiagIsSkippedUntilTheKeepalive` and `Inspection.ConfigRejectsOutOfRangePortAndStallClose`. W1 was editing its files at 05:56-05:57 (for example, `reliable_kb must be at least 128`), and they are not related to capture.
- Earlier interruptions from other workers:
  - At 05:47, W4's `BrainImuBench.TelemetryDroppingTheAttitudeGroupAgesTheLastTiltOut` failed while W4 was editing it; it passed at 06:00.
  - At about 05:55, one build failed on W1's `inspection_document.cpp` while W1 was editing it. I waited and retried; the next build was clean.

**Snapshot tree** (`$S/w3-mut`: the tree at about 05:35 plus my final files; build `$S/w3-mut-build`)
- A fresh build had 0 warnings.
- Full `ctest -j16 --repeat until-fail:3`: 610/610.

**Mutation checks** (`$S/w3-mutate.sh`, rebuilding after each). Every mutant was killed:
- `truncated_by` without `bundle_size_limit`;
- the host read at `begin()`, checked by both host tests;
- no `host_at_start`;
- evictions counted over the whole rolling window;
- evictions counted over the lifetime;
- the delta without the seq check;
- no `link_lost` `noteFault` in System;
- no hub post in `System::noteEvent`;
- no `pico_reboot` `noteFault`;
- the ZIP CRC not patched.

One mutant (evictions counted over the whole rolling window) first survived. The reason was a dangling reference to a temporary `ZipRead` in the new test, and `-Wdangling-reference` flagged it. After fixing the test, the mutant is killed.

## Contract issues

None new. The earlier ones stand:
- `composition.cpp` does not pass `<Capture>` through `<Configuration>`.
- The `DiagRobotState` advanced flag is optional.

The new test works around the first one by inserting `<Capture>` into the resolved System document.

## Open risks

- **Host metadata age.** `host_at_start` is at most one reading period (1 s) older than the window start. A change between two readings shows only in `events.csv`.
- **Eviction counting.** `rolling_evicted` counts the window's first millisecond whole.
- **Sequence wrap.** `seq_gap_frames` cannot see a loss of a multiple of 256 frames.
- **Memory.** The numbers are from the Windows host heap in Debug. The Pi's glibc overhead and the build time of a large bundle on the Pi are not measured.
- **Timing in one test.** `PicoRebootFiresTheAutomaticCaptureWithBothBoots` waits 2.5 s of real time, so that the 1 Hz host reading sees the old boot before the trigger. It passed 3x in a -j16 full-suite repeat, but it depends on timing.
- **Not hardware validated.** No automatic trigger has been seen on hardware.
