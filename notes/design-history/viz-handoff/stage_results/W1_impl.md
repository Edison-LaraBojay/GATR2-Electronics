# W1 stage 1 result: inspection transport and server (navigatr.inspect/2)

Host validation only (Windows 11, MinGW ucrt64 GCC 15.2, Debug, -Wall -Wextra, loopback). Nothing here was run on the Pi or with robot hardware.

## What was built (all under pi/naviGATR)

**Transport: `src/inspection/http_server.{h,cpp}`**
- Per WebSocket client:
  - one in-flight frame, never modified or dropped once started;
  - a reliable FIFO bounded by `reliable_bytes`. Overflow drops the unsent messages, queues close 1013 and closes the socket within 1 s. It is counted and recorded in `recent_closes`;
  - one replaceable slot per channel name, with a priority. Lower priority numbers go first, with round robin among equal priorities. A newer message replaces the unsent one. A message over `client_buffer_bytes` is refused, and the older unsent one of its channel is dropped too.
- Send order after the in-flight frame: reliable, `state`(0), `telemetry`(1), `diag`(2), `instrumentation`(3), `preview:<camera>`(4).
- Enqueue never blocks. It tries one non-blocking flush of at most 256 KiB; the server thread writes the rest.
- Stall close is unchanged.
- Per-client and per-channel stats: queued, sent, replaced, refused, dropped, bytes, and enqueue-to-written ms.
- Close reasons and counters: stalled, reliable_overflow, protocol, peer.
- Other server changes:
  - POST bodies up to 64 KiB (413 above); GET and POST reach the handler, other methods get 405 `Allow: GET, POST`;
  - `HttpResponse::shared_body` sends large bundles without a copy;
  - `send_buffer_bytes` sets SO_SNDBUF.
- Behavior change found with the new many-module viewer: `max_clients` now bounds WebSocket feed clients only.
  - Plain HTTP requests have their own cap of 32 concurrent connections.
  - Before this change, a page load's parallel module requests got 503 and the viewer never started (see Tests).

**Feed: `src/inspection/inspection_service.{h,cpp}`**
- **On open** (server thread, before the client is registered): hello (with `client_id`), history and capture status. Nothing can precede the hello.
- **state:**
  - sent when the feed publication advanced, at most the client's `state_hz` (default 30, max 60), plus a 1 s keepalive;
  - built once per tick and shared between clients;
  - the publication is polled every 5 ms once a client's rate allows the next state.
- **diag:** one global `diag_hz` schedule, so it is built at most once per period for all clients. It is skipped per client when the content hash matches the last diag that client got. The hash leaves out `seq`, `host_ms`, `workers.inspection` and `inspection`. A 2 s keepalive still sends it.
- **event:** pushed from `System::events()` in order. `seq` is the RuntimeEvent sequence, so clients dedupe against `diag.events`.
- **capture:** pushed when `CaptureService::version()` changes.
- **telemetry:** from `diagHub().latest(kBrainTelemetry)`, decoded with `translagatr::decodeTelemetryBody`. At most 20 Hz, subscribers only. Enum values are sent as numbers plus snake_case names.
- **instrumentation:** at most 4 Hz on one global schedule, to subscribers only, through `writeInstrumentation`. At most two variants are built, with and without raw bytes. `diagHub().links().setRaw/setDecoded` is switched edge-triggered while any instrumentation subscriber wants raw or decoded, and switched off at stop.
- **Previews:** through per-camera slots. The header now carries `detection`, the entry of exactly that frame (W2 request).
- **Client messages:** subscribe, ping (answered at once on the server thread, `id` and `client_ms` echoed verbatim), history, preview.
- **Session reset:** clear the unsent replaceable messages, then hello and history of the new session before any newer state.
- **Capture routes** go to W3's `captureRoute(System&, ...)`. POST elsewhere gets 405 `Allow: GET`.

**Documents: `src/inspection/inspection_document.{h,cpp}`**
- New documents: `stateDocument`, `historyDocument`, `diagDocument` (with content hash), `eventDocument`, `pongDocument`, `telemetryDocument` / `decodeTelemetryRecord`, `instrumentationDocument`, `captureDocument`.
- The robot writer is shared by snapshot and state.
- `robot.attitude.status`: `measured` (valid, host age <= 250 ms), `stale` (valid, older or age unknown), `assumed_level` or `unavailable`.
- hello: contract `navigatr.inspect/2`, `features`, `client_id`.
- `GET /api/snapshot` is unchanged in shape: contract `navigatr.inspect/1`, with trail, plus the additive `attitude.status`.
- Other files:
  - `src/inspection/inspection_stats.h`: feed and build stats;
  - `json_writer.h`: `raw()`;
  - `src/runtime/inspection_config.h`: `state_hz`, `diag_hz`, `reliable_kb`, `send_buffer_kb`. `snapshot_hz` still parses and is unused.

**Tests**
- `tests/inspection_feed_gtest.cpp` (new, 14 tests), `tests/inspection_test_client.h` (new, a shared test-side HTTP/WebSocket client).
- `tests/inspection_gtest.cpp` and `tests/inspection_document_gtest.cpp` updated: 5 new document tests, the new config checks, the admission and replacement bounds.
- `tests/CMakeLists.txt`: a one-line insertion.

**Docs and tools**
- `docs/inspection.md`: the full inspect/2 schema, delivery and ordering rules, latency terms, and the config table.
- `tools/viewer_smoke.sh`:
  - HTTP document checks (hello features, inspect/1 snapshot with trail, `attitude.status`, capture status);
  - inspect/2 page attributes (`data-contract`, `data-mode`, `data-diags`, `data-attitude-status`);
  - `--attitude-badge stale`;
  - the DOM dump and the screenshot now run as two browser runs, because with both flags this Chrome dumps the page before the feed script ran.

## Test commands and results
Build: `cmake -S $S/w1-wrap -B $S/w1-build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$S/nav-base/_deps/googletest-src && cmake --build $S/w1-build -j16`. `$S/w1-wrap` only does `add_subdirectory` of the repo tree; its stub is inactive now that W3's `capture_http.cpp` exists.
- Forced recompile of every W1 source and test: 0 warnings.
- `navigatr_tests --gtest_filter=InspectionFeed.*:Inspection.*:InspectionDocuments.*:JsonWriter.*`: 34/34 passed, and 4 x 34 passed with `--gtest_repeat=4`.
- `ctest -j16 -R "Inspection|JsonWriter"`: 46/46 passed (in parallel, under load).
- Full `ctest --test-dir $S/w1-build/navigatr -j16` at the end, on the working tree with the other workers' changes as of then: 604/604 passed.
- `tools/viewer_smoke.sh <w1 navigatr.exe> config/demo/synthetic_field_demo.xml --attitude-badge valid`: PASS. The page reported state live, 57 states, 10 frames, contract inspect/2, 9 diags, attitude measured.
- The same with `synthetic_field_demo_no_attitude.xml --attitude-badge assumed`: PASS.
- Before the `max_clients` fix, the smoke run FAILED. CDP showed `latency_panel.js` and `capture_panel.js` answered 503.

**Host measurement.** Conditions: Windows 11, 32 logical CPUs, Debug build, `synthetic_field_demo.xml`, one local Python client, 10 s, other workers building on the same machine. Probe script: `$S/w1_feedprobe.py`.
- `GET /api/snapshot` (the old per-push document, with trail): 39,384 bytes.
- `state`: 1,080 B mean at 29.6/s, about 32 KB/s. Build mean 284 us, max 828 us.
- `diag`: 19.6 KB mean at 4.1/s, about 80 KB/s. Build mean 2.5 ms, max 4.6 ms.
- previews: 7.7 KB at 4.8/s. hello 23 KB and history 19-24 KB, once each.
- Enqueue-to-written on loopback: 0.03-0.3 ms.

These are host numbers from a Debug build, not Pi numbers. W0 owns the before/after comparison.

## Contract issues
- None blocking.
- W4's configs still set `snapshot_hz="20"`. It parses and is ignored by inspect/2.
- Suggestion for W3: a publish notification from RobotStateFeed would remove the 5 ms publication poll.
- W2's requests handled: frame `detection`, `instrumentation.host_ms` (present), hello `client_id`, smoke attributes.
- Note for W2: in the separate screenshot run the 3D canvas renders black, while the DOM run reports `webgl=1`. It may be render-on-demand under headless virtual time. Only the screenshot is affected.

## Open risks
- Strict priority means a client too slow for its state rate can go without diag, instrumentation and previews while that lasts. They stay bounded, because they are replaceable.
- Kernel buffers on both ends can still hold already-written frames behind the latest-wins queue. `send_buffer_kb` bounds the Pi side. The default stays at the OS value, pending W0's measurements; a small value limits preview throughput.
- The capture POST routes have no CSRF guard. The server binds loopback by default and no robot command is reachable. Still, another page in the same browser could start or cancel a capture.
- A state or diag built across a reset can arrive just before the new hello carrying the new session. The docs tell clients to drop messages whose session differs from their newest hello.
- The telemetry enum names are a copy of the actuGATR and investiGATR enums and must be kept in step. An unknown value is sent as its number with the name `unknown`.
