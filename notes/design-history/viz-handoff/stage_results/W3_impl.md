# W3 stage 1 result: capture, export, triggers, state tap

Date: 2026-09-28. Host: Windows 11, MSYS2 UCRT64 GCC 15.2, MinGW Makefiles, Debug, -Wall -Wextra. Build dir `$S/w3-nav`. All results below are host tests and a host run of the synthetic demo; nothing was run on the Pi, Pico or Brain.

## What was built (spec 2.3-2.4, 5)

**New files** (all in `pi/naviGATR`)
- `src/capture/capture_config.h`: the `<Capture>` struct and a strict parser. Defaults apply when the element is absent.
- `src/capture/capture_recorder.h` and `src/capture/capture_service.cpp` replace the stub. The recorder:
  - drains `System::diagHub()` on its own thread every 20 ms;
  - keeps a rolling window of the default streams (every stream except `bytes`);
  - on a trigger, takes `pre_s` from that window, records the post window, then finalizes on the same thread;
  - bounds records and memory with one shared cap for the rolling window and the active capture; reaching it truncates with `record_limit` or `memory_limit`;
  - keeps the newest `keep` bundles; supports cancel; refuses a second start as busy;
  - optionally writes each bundle to a directory (`.part` then rename). A failed write is reported and the bundle stays in memory; only this run's own files beyond `keep` are deleted;
  - runs automatic triggers (`link_lost`, `continuity_lost`, `pico_reboot`, `profile_changed`) only when idle, with a cooldown and an hourly cap, and counts suppressed ones;
  - takes Pi reset boundaries, counts hub drops per stream from the window start, and closes a recording capture as truncated (`shutdown`) when stopped.
  - `SystemCaptureHost` writes the metadata sections: configuration, Brain profile (wheels with CPR, gear, radius, position, angle, polarity and travel scale; footprint; calibration settings), localization and calibration state, Pico link identity, software identity.
- `src/capture/capture_bundle.{h,cpp}`: the `gatr2.capture/1` bundle, one CSV per stream plus `metadata.json` and a generated `README.txt`.
  - Columns follow spec 5.3.
  - Segments cover four domains: robot (reset_count, odometry_epoch, anchor_revision), pico (identity, boot and epochs), brain (session) and pi (reset_count).
  - The unwrapped heading restarts in each robot segment, and encoder deltas only span one boot and acquisition epoch.
  - robot rows carry a `new_measurement` flag.
  - A record whose payload does not match its stream is skipped and counted; it never throws.
  - The CSV text is bounded by `max_mb`.
  - Pico diag and Brain telemetry are decoded with W4's `decodePicoDiagPayload` and `decodeTelemetryBody` (whole frames are also accepted); `payload_hex` is always kept.
- `src/capture/zip_writer.{h,cpp}`: store-only ZIP. CRC is `translagatr::crc32`; the DOS time comes from the Pi system clock.
- `src/capture/capture_http.{h,cpp}`: `captureRoute(System&|CaptureService*, req, resp)`.
  - Routes: start, cancel, status and `<id>.zip`. The ZIP uses W1's `shared_body`, so it is not copied per request.
  - Errors: 405, 400, 409 busy, 404 not ready, 503 no service. Ids are validated.
  - W1 already routes it.
- `tests/capture_gtest.cpp`: 22 tests.
- `docs/capture.md`: configuration, behavior, bounds, HTTP, the status schema, the bundle schema and conventions, and a Python analysis snippet.

**Edited files**
- `src/runtime/system.{h,cpp}`:
  - `<Capture>` is an allowed child and is parsed.
  - The feed gets the hub.
  - `noteEvent` posts `kEvent`, and `noteFault` feeds the automatic triggers.
  - `reset()` notes the boundary before the stages republish.
  - The recorder is built last in `build()` and stopped first in `~System`.
  - `capture_` is now a `unique_ptr<CaptureRecorder>`; `capture()` still returns `CaptureService*`. `captureConfig()` is added.
- `src/state/robot_state_feed.{h,cpp}`: `setDiagnostics()`. After releasing the feed lock, `publish` posts one `DiagRobotState`, only while the hub wants robot states.
- `src/app/main.cpp`: prints one capture line at startup. Shutdown order is inspection, then the recorder, then the workers.
- `CMakeLists.txt` and `tests/CMakeLists.txt`: single-line insertions of `capture_bundle.cpp`, `capture_http.cpp`, `zip_writer.cpp` and `capture_gtest.cpp`.

## Test commands and results

```
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake -S pi/naviGATR -B $S/w3-nav -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$S/nav-base/_deps/googletest-src
cmake --build $S/w3-nav -j16
ctest --test-dir $S/w3-nav -j16 -R "^Capture"
ctest --test-dir $S/w3-nav -j16
```

**Capture tests**
- `ctest -R ^Capture`: 26/26 passed. That is the 22 new W3 tests plus 4 pre-existing CaptureTransform tests. They also pass with `--gtest_repeat=3` in one process.
- The new tests cover:
  - config parse, and the System accepting `<Capture>`;
  - idle wanted mask, rolling window bounds (time and records), pre-window trimming, post-window stop, record-limit truncation;
  - cancel, busy refusal and clamping;
  - auto trigger cooldown, hourly cap and busy suppression;
  - segments at epoch, anchor, Pico boot, Brain session and Pi reset changes;
  - drop accounting (hub ring full, not selected, malformed);
  - CSV escaping and missing values;
  - a ZIP round trip with a test-side reader and an independent table CRC;
  - replay consistency: every stream, record values in and CSV values out, Pico diag and telemetry through W4's codec;
  - HTTP routes, directory keep and write failure, and stop closing a capture as truncated;
  - the threaded recorder, the feed tap, a System reset boundary, and a synthetic-demo System run in worker mode.

**Full suite** (shared tree at about 04:50)
- 603/604 passed.
- The one failure is `BrainProfileCamera.PreviewConfigIsTheUsbConfigPlusOnlyTheCamera`, which belongs to W4 (brain_profile_gtest and config, in progress). It is not related to capture.
- An earlier run also failed W4's `BrainLinkSession.VersionAndRetiredOpErrors` (op 12 is now TELEMETRY); it passed in later runs.
- Warnings: 0 in W3 files.

**Host end-to-end run**
- Setup: `navigatr.exe config/demo/synthetic_field_demo.xml --inspect-port 18733`, with W1's current server, driven by curl.
- `POST /api/capture/start?pre_s=2&post_s=3` returned the id; a second start returned 409 busy.
- The capture went `ready` with `stop_reason` `post_window`: 450 robot_state rows over 5 s, and a 109734-byte ZIP.
- `unzip -t` reported no errors, and every file was listed.
- About half the rows had `new_measurement=1`: the loop publishes at about 2x the measurement rate.

**Timing probe** (scratch program, not a repo test)
- Conditions: the same host, the Debug library at -O0, 100000 records (half robot states, half Pico frames).
- `buildCaptureBundle` took 1419 ms for a 14.3 MB ZIP; 10000 records took 130 ms.
- In memory, one record (`sizeof(DiagRecord)`) is 168 bytes, so the default `max_mb=64` binds at 399457 records, slightly below `max_records`.
- No Pi measurement exists.

## Contract issues (also in progress/W3.md)
1. Lead: `src/config/composition.cpp` does not pass `<Capture>` through `<Configuration>` profiles, and every shipped config is one. The fix is to add `name == "Capture"` next to `"Inspection"`:
   - line 68 and line 74 (`childSection`);
   - lines 382-383 (the `composeProfile` singular list).

   Until then those profiles run with the defaults (enabled, memory only, no auto triggers). `docs/capture.md` says so.
2. Lead (optional): add `bool advanced` to `DiagRobotState` (records.h). It would make `new_measurement` exact; today the flag is derived from `source_ms` changes.

## Open risks
- Finalizing runs on the capture thread, which does not drain the hub meanwhile.
  - Measured on the host: about 1.4 s per 100k records in Debug.
  - At typical rates (a few hundred records per second) the 16384-record ring absorbs that. Any drop is counted in `hub_ring_full` and in the status.
- Memory bounds:
  - worst case while one bundle is built: about 3 x `max_mb` (records, CSV text, ZIP);
  - plus `keep` finished bundles;
  - with the defaults, up to about 400 MB in a flood scenario. Typical captures are a few MB. Documented.
- Event text is truncated to 119 bytes by the `DiagEvent` contract.
- The synthetic System test does not reach measured (non-held) poses within its 0.8 s window. Measured rows and `source_age_ms` were checked by hand in the host run above, not by an assertion.
- Nothing is hardware validated. Auto triggers were exercised with synthetic fault notes, not real link loss.
