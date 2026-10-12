# Visualizer, diagnostics, capture and replay: contract spec (Phase 1)

The user's request is in `requirements.md`, next to this file; read it first. This spec freezes the contracts and assigns file ownership. The lead may add numbered amendments in section 12. Read section 12 at the start of every stage.

## 0. Ground rules (every worker)

**Repository and setup**
- The repo is `C:/GATR2-Electronics`. Git Bash, with `/c/msys64/ucrt64/bin` first on PATH.
- `$S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
- The Pi runtime lives at `pi/naviGATR`. On disk the folder is still `pi/navigatr` until a case-only rename; Windows paths are case-insensitive, so use `pi/naviGATR`.
- Component names are in `docs/naming.md`:
  - `translaGATR/` is the shared wire codec (namespace `translagatr`, formerly `common/` and `gatr2`);
  - `pico/aggreGATR` is the Pico firmware;
  - `brain/locaGATR` and `brain/operaGATR` are the PROS programs;
  - `pi/naviGATR/spectaGATR` is the viewer.

**Hard limits**
- Do not commit, push, flash, deploy, or touch `pcb/`.
- Do not edit files you do not own (section 1). If you need a change in someone else's file, write it under CONTRACT ISSUES in your progress file and work around it.
- The lead reads progress files between stages.

**Builds**
- Every worker builds in its own directories, `$S/<wN>-*`. Never build inside the repo; never share a build directory.
- Host recipe: `cmake -S <src> -B $S/<wN>-x -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$S/nav-base/_deps/googletest-src`, then `cmake --build ... -j16`, then `ctest --test-dir ... -j16`.
  - Run test binaries with `/c/msys64/ucrt64/bin` on PATH; without it they fail silently with 0xc0000139.
- PROS (Brain programs):
  ```
  P="$HOME/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install"
  cd brain/<app>
  PROS_TOOLCHAIN="$P/pros-toolchain-windows/usr" PATH="$P/pros-toolchain-windows/usr/bin:$PATH" make.exe -j16
  ```
  - `pros make` does not work from Git Bash.
  - Only the Brain worker (W5) builds the PROS programs, and only W5 edits `brain/**`.
- Pico: from `pico/aggreGATR`, run `/c/Users/mbes1/.platformio/penv/Scripts/pio.exe run -e hat2_bno08x -e hat2_asm330`. Only W4 runs it; it builds into the repo's `.pio`.
- Tests are listed explicitly in each CMakeLists (no globs); add your own test files to the list you own.

**Code style**
- C++17; `-Wall -Wextra` with no `-Werror`, and aim for 0 warnings.
- Plain ASCII in code, comments and docs. No em dashes.
- Terse comments that say why. Braces on every `if`/`for`/`while`.
- Match the surrounding style.

**Honesty rules (from the user)**
- Never claim hardware validation. Label host tests as host tests.
- Do not invent measurements; record test conditions.
- Do not present pin states as voltages, or zero counts as proof of a working sensor.

**Progress file:** keep `progress/<wN>.md` in this folder current. It holds what is done, what is in progress, CONTRACT ISSUES, test results with exact counts, and risks. Stage results go to `stage_results/`.

## 1. Workers and file ownership

| Worker | Area | Owns (edit freely) |
|---|---|---|
| W0 perf | Baseline and after measurements, harness | `pi/naviGATR/tools/perf/**` (new): C++ harness and CDP scripts, with its own CMake behind `NAVIGATR_BUILD_PERF_TOOLS` (default OFF). The one line that adds that option and `add_subdirectory` to `pi/naviGATR/CMakeLists.txt` goes through the lead (CONTRACT ISSUES). |
| W1 feed | Inspection transport and server | `pi/naviGATR/src/inspection/**`, `src/runtime/inspection_config.*`, `tests/inspection_gtest.cpp`, `tests/inspection_document_gtest.cpp`, new `tests/inspection_feed_gtest.cpp`, `tools/viewer_smoke.sh`, `docs/inspection.md` (pi/naviGATR/docs) |
| W2 viewer | Browser rendering, UI, graphs, replay, capture UI, instrumentation panels | `pi/naviGATR/spectaGATR/**` (flat folder: the CMake glob embeds only top-level files; add new `.js`/`.css` files at top level). Browser test scripts under `pi/naviGATR/tools/viewer/**` (new). |
| W3 capture | Recorder, ZIP export, triggers, state tap | `pi/naviGATR/src/capture/**` (the lead's stub `capture_service.cpp` is yours to replace), `src/runtime/system.{h,cpp}`, `src/state/robot_state_feed.{h,cpp}`, `src/app/main.cpp`, `tests/capture_gtest.cpp` (new), `docs/capture.md` (new, pi/naviGATR/docs) |
| W4 instrumentation | Wire codec additions, Pico firmware, Pi Pico/Brain link instrumentation, attitude on the Pi, camera preview config | `translaGATR/**` (codec + tests), `pico/aggreGATR/**`, `src/diagnostics/**` (the lead's `link_monitor.*`, `instrumentation.cpp`), `src/impl/resources/{pico_telemetry,serial_links,pros_usb_link}.*`, `src/resources/{pico_control.h,brain_imu_bench.h,serial_link.h}`, `src/impl/commands/brain_link_commands.*`, `src/impl/publishing/brain_link_publisher.*`, `src/impl/localization/brain_imu_wheel_bench.*` and other attitude files you need under `src/impl/localization/` and `src/impl/sensors/`, `src/runtime/brain_profile_builder.*`, `config/**`, `tests/{pico_link,brain_profile,brain_io,brain_imu_bench,sensors,pros_usb_link,half_duplex}_gtest.cpp`, `docs/interfaces.md`, and in pi/naviGATR/docs: `pico_link.md`, `brain_profile.md`, `camera_preview.md` (new) |
| W5 brain | Brain telemetry and VEX attitude | `brain/**` (all Brain libraries and both programs), `docs/{communigatr,actugatr,brain_setup}.md` |

**Lead only**
- `pi/naviGATR/CMakeLists.txt` and `pi/naviGATR/tests/CMakeLists.txt`. Ask under CONTRACT ISSUES with the exact lines: new sources, new test files.
  - Exception: W0 adds nothing there itself.
  - W1, W3 and W4 may each add their own new test files and sources to those two lists with a single-line insertion, as long as they do not reformat.
- `src/diagnostics/{records.h,hub.h,hub.cpp}` and `src/capture/capture_service.h`: the contracts. Changes go through CONTRACT ISSUES.
- `docs/naming.md`, root `README.md`, `docs/README.md`.

**Shared read-only:** everything else in `pi/naviGATR/src`. For example `core/`, `state/`, `runtime/` files not listed above, and `impl/` files not listed. Need a change there? Ask.

## 2. Architecture decisions (frozen)

1. **Live feed split** (W1 server, W2 client). The WebSocket moves to contract `navigatr.inspect/2`:
   - small frequent `state` messages;
   - slower `diag` messages;
   - a one-off `history`;
   - reliable `event` / `capture` / `pong` messages;
   - replaceable `telemetry`, `instrumentation` and preview messages.

   `GET /api/snapshot` stays as the full inspect/1-style document, including `trail`, for tools and tests.
2. **Latest-wins per client** (W1). Replaceable channels hold at most one unsent message each; a newer one replaces it. A partially sent frame is never touched. Reliable messages go through a bounded FIFO; overflowing it closes that client, and reconnecting recovers full state. Memory is bounded per client.
3. **One DiagnosticsHub per System** (lead, done: `src/diagnostics/hub.h`, `records.h`).
   - Producers post small POD records. The capture recorder (W3) is the only draining consumer.
   - Live views use `hub.latest(kind)` and the `LinkMonitor` rings (`src/diagnostics/link_monitor.h`), which hold raw bytes, decoded frames and counters.
   - The hub reaches resources through `ResourceInitializationContext::diagnostics` and slots through `SlotInitializationContext::diagnostics`. Both are null outside a System; check before use.
   - `System::diagHub()` and `System::capture()` exist.
4. **Capture is on the Pi** (W3). It drains the hub on its own thread, keeps a rolling pre-trigger window, records post-trigger, and builds a store-only ZIP (CRC-32 = `translagatr::crc32`). It never changes rates or timing.
5. **New wire pieces** (structs frozen in `translaGATR/frames.h` by the lead; codec and tests by W4):
   - Brain `TELEMETRY` op 12 (`BrainTelemetry`, 54-byte body, reply Ok header-only);
   - Pico diagnostic frame 0x14 (`PicoDiag`, 26-byte payload, 32-byte frame);
   - `kPicoOpDiagnostics` = 4 (`diag_hz` u8);
   - `FrameReaderStats` counters inside `FrameReader`.

   **Compatibility:**
   - An older Pi answers TELEMETRY with UnsupportedOp; the Brain must stop sending it for that session and never drop the session over it.
   - An older Pico answers DIAGNOSTICS with UnknownOp; the Pi marks Pico diagnostics unavailable.
   - A new Pico sends 0x14 only after being asked, so an older Pi never sees it.
6. **Attitude.**
   - **VEX IMU (the working setup):** the Brain reads `pros::Imu` roll and pitch, converts them to the robot frame using `gatr2_robot::kVexImuMountYawDeg` (0/90/180/270, default 0, UNVERIFIED sign conventions) and sends them in TELEMETRY. The Pi shows them as measured attitude while fresh (at most 250 ms old) and as stale after.
   - **Pico IMU:** attitude is documented as not available yet. It needs an IMU-to-robot mounting rotation that no profile carries, and the user's external IMU is faulty. Keep the synthetic rig attitude path working.
   - **States:** the viewer shows measured, assumed level, unavailable and stale distinctly.
   - **No invented height:** the robot stays on the floor plane.
7. **Camera preview without field correction** (W4 config and doc, W1/W2 already show previews). Add a config that runs the Brain-profile USB localization plus a camera and a world estimator that publishes raw frames without correcting anything:
   - no intrinsics, mounting, tags or landmark correction needed;
   - a missing camera never breaks localization;
   - preview rate and size stay bounded by the existing per-client budget.

## 3. WebSocket contract `navigatr.inspect/2` (W1 server, W2 client, W0 harness)

JSON text unless noted. Unknown fields must be ignored by clients. Every server message has `type`.

### 3.1 Server to client

| type | Channel | When | Content |
|---|---|---|---|
| `hello` | reliable | first message on connect; again after a session reset (`session.reset_count` changed) | today's hello document with `contract:"navigatr.inspect/2"`, plus `features:{state_hz, diag_hz, capture:bool, instrumentation:bool, telemetry:bool, history_max}` |
| `history` | reliable | right after `hello`, after a reset, and on client request | `{session, publication, trail:[...]}`: the bounded trail, same entry fields as today (oldest first) |
| `state` | replaceable | when the feed publication advanced, at most `state_hz` (default 30, max 60), plus a 1 Hz keepalive | `{seq, host_ms, host_us, session, publication, robot:{...}, localization:{all_ready, stationary, continuity_breaks, last_break}}`. `robot` has exactly today's snapshot `robot` fields, and `robot.attitude` gains `status`: `measured`, `assumed_level`, `unavailable` or `stale` |
| `diag` | replaceable | at most `diag_hz` (default 4); skipped for a client when byte-identical to the last diag sent to it | today's snapshot document without `trail`, with `type:"diag"`, plus `inspection.clients:[...]` (per-client queue stats, see 3.3) and `hub:{posted, dropped, queued}` |
| `event` | reliable | each new System event, in order | `{seq, host_ms, text}`. New clients get the recent list inside `diag.events`, as today |
| `telemetry` | replaceable | each new Brain TELEMETRY record, at most 20 Hz | `{seq, host_ms, session, stamp_ms, flags, attitude:{roll_deg,pitch_deg}\|null, motion:{command_id, state, reason, mode, segment, segment_count, target:{x_m,y_m,heading_deg}, cmd:{vx_m_s,vy_m_s,omega_deg_s}, cross_track_m, distance_error_m, heading_error_deg, drive_fault}\|null, wheels:{rpm:[...]}\|null}`. The names of `state`/`reason`/`mode`/`drive_fault` are added by W1 from the enum values (W5 documents the tables in docs/actugatr.md; W1 may send numbers plus names) |
| `instrumentation` | replaceable | at most 4 Hz, only to clients subscribed | the `writeInstrumentation` object (section 7) |
| `capture` | reliable | on connect, and whenever `CaptureService::version()` changes | the `writeStatus` object |
| `pong` | reliable | for each ping | `{id, client_ms, host_ms, host_us}` |
| binary | replaceable per camera | preview budget, as today | unchanged: `[u32 LE header len][JSON header][JPEG]` |

`host_ms` is Pi `HostClock` milliseconds and `host_us` is the same clock in microseconds (`HostClock::nowUs()`), both taken when the message was built. The robot's `measured_at_host_ms` and `age_ms` are as today. `seq` counts per channel per server.

### 3.2 Client to server

- `{"type":"preview", ...}`: unchanged.
- `{"type":"ping","id":n,"client_ms":t}`: the server answers `pong` at once.
- `{"type":"subscribe","state_hz":30,"diag":true,"instrumentation":false,"raw":false,"decoded":false,"telemetry":true}`:
  - Missing fields keep their value.
  - `raw`/`decoded` switch the LinkMonitor rings on while any subscribed client wants them, and off when none does.
  - Hidden panels must unsubscribe.
- `{"type":"history"}`: the server replies with a `history` message.

Capture control is HTTP (section 5.4). The viewer never sends robot commands, and nothing in inspect/2 can.

### 3.3 Server queue and stats (W1)

**Queue**
- Per client:
  - one in-flight frame, which is never modified;
  - a reliable FIFO bounded by `reliable_kb` (default 256; overflowing it closes the client with the counter `closed_reliable_overflow`);
  - one slot for each replaceable channel (per camera for previews).
- When the socket can take more and nothing is in flight, the order is: reliable, then `state`, `telemetry`, `diag`, `instrumentation`, preview (round robin across cameras).
- Enqueueing never blocks on a socket. Socket writes happen on the server thread, or are non-blocking with bounded work under the lock.
- The stall close from today stays.

**Stats in `diag.inspection.clients[]`** (per client):
- id, queued bytes, in-flight bytes;
- per channel: sent, replaced, bytes;
- reliable backlog;
- last enqueue-to-written ms per channel;
- dropped or closed reasons.

**Build timing:** the state and diag build time (us) and size (bytes), for the last message and as a mean, in `diag.inspection`.

### 3.4 Latency terms (W1, W2, W0)

| Term | How | Clock |
|---|---|---|
| Source age at publish | `state.host_ms - robot.measured_at_host_ms` | Pi host clock only |
| Publish to receive | estimated from the ping/pong offset: take the minimum-RTT sample of the last 20 pings; offset = `host_ms - (t_send + t_recv)/2`; report ±RTT/2 | estimate |
| Receive to render | `performance.now()` when the message arrived, to the rAF that drew it | browser only |

Never subtract a Pi timestamp from a browser timestamp directly.

## 4. Browser (W2)

**Rendering and message handling**
- The rendering loop is `requestAnimationFrame`, independent of message arrival.
- Message handlers parse and store only; no DOM or scene work in `onmessage`.
- Scene updates from the newest stored state, once per animation frame.
- DOM panels update at no more than 10 Hz, and only when visible (collapsed or hidden panels unsubscribe or skip).
- Update rows by key. No `innerHTML` rebuild of whole tables per update.

**Trail and scene objects**
- The trail is an incremental ring. Upload the GPU buffer only for new points (update ranges); no per-point array allocation.
- Label sprites re-rasterize only when their text changes. Age text is updated at 1 Hz at most, or moved to the DOM.
- The path uses instanced or `Points` geometry, not one mesh per point.
- Follow mode must not allocate per frame.

**Smoothing** (presentation only, UI toggle, default on)
- The rendered pose may lead the newest received pose by at most 100 ms of extrapolation along the reported velocity, or interpolate between the last two states with at most one state period of delay. W2 picks one and documents the latency it adds.
- Snap with no blending on any of:
  - session, reset, `odometry_epoch`, `anchor_revision`, placement or validity change;
  - a jump over 0.2 m or 15 degrees;
  - a reconnect.
- When stale (no state for 250 ms, or the robot age over 250 ms): no extrapolation, and a visible stale state.
- Readouts, graphs, recordings and export always use exact received values.

**Live graphs** (canvas, no new libraries)
- Selectable series: x, y, heading, vx, vy, yaw rate; roll/pitch when measured; wheel counts/deltas/travel from instrumentation or READ-style data where available; IMU rotation/rate and bias from diag; source ages, packet rates and errors; telemetry commanded vs actual; tracking errors; wheel rpm targets.
- Bounded ring buffers, 60 s by default.
- Min/max decimation per pixel column, so spikes survive.
- Gaps: missing samples draw as gaps (the gap threshold is 3x the nominal period), never as zeros.
- Heading: a wrap-aware plot (unwrap within a segment, break at wraps) or an unwrap toggle.
- Units and sources in the legend; a validity mark for invalid spans.
- Event markers.

**Overlays:** the destination, desired heading and follower target from `telemetry`, drawn next to the existing planned path and trail.

**Replay**
- Load a capture ZIP, either from the Pi list (`/api/capture/<id>.zip`) or a local file. The ZIP is store-only, so a tiny reader is enough.
- Parse the CSVs.
- Timeline: play/pause, scrub, speed (0.25x to 4x), event markers.
- 3D state and graphs stay in sync with the timeline.
- Clicking an event or graph point seeks there.
- Trajectories split at session, epoch and boot changes.
- A prominent REPLAY banner. Live-only controls are disabled while in replay (preview budget, capture start).
- The live connection keeps running but is not rendered.
- Replay sends nothing to the robot. There is no such path, so keep it that way.

**Capture UI:** start (pre, post, streams), cancel, status, list, download.

**Instrumentation panel**
- Per link: counters, rates, last-valid ages, decode rejections by reason, and FrameReader stats.
- Raw hex view: toggle, bounded, direction marked, attempted vs accepted marked.
- Decoded message list.
- Pico diagnostics, with pins labeled as logic levels read back (not voltages). UART RX idling HIGH is not a disconnection sign.
- Encoder status wording that does not overclaim.

**Latency panel:** the 3.4 terms, frame time p50/p95/max, messages per second per channel, and replaced/skipped counts.

**Preserve** everything in `progress/inventory_viewer.md` section 4, plus the smoke-test hooks (`#status` data attributes, `window.__navigatr`).

## 5. Capture and export (W3)

### 5.1 Configuration
An optional element under `<System>`, which W3 adds to the allowed children:

```xml
<Capture enabled="true" rolling_s="10" max_pre_s="30" max_post_s="60" default_pre_s="10" default_post_s="15" max_records="400000" max_mb="64" keep="3" directory="" auto="" auto_cooldown_s="120" auto_max_per_hour="6"/>
```
- **Defaults when absent:** enabled, as above.
- **`directory`:** empty keeps captures in memory only (the newest `keep`). Otherwise also write `<directory>/<id>.zip`; a write failure is reported, and the bundle stays available from memory.
- **`auto`:** a comma list of `link_lost`, `continuity_lost`, `pico_reboot`, `profile_changed`. Empty means off.

### 5.2 Behavior
- **One capture at a time.** A start while one is recording is refused as `busy`; every viewer sees the same status.
- **Streams:** the default is every kind except raw `bytes`. `bytes` needs an explicit choice, and while that capture records, the LinkMonitors feed raw bytes to the hub.
- **Rolling window:** kept for the default kinds only. Selected-only kinds start at the trigger.
- **Timeline:** trigger, then `pre_s` from the rolling window, then record `post_s`, then finalize on the capture thread (build the ZIP), then `ready`.
  - Cancel discards.
  - Limits reached (records, bytes) stop recording early with `truncated:true`.
- **Viewer independence:** a browser disconnecting changes nothing.
- **Restarts:** a source, session, profile or Pi reset during a capture keeps recording. Every row carries its identifiers, `metadata.segments` lists the boundaries, and nothing is joined across them.
- **Drop accounting:**
  - hub drops per kind (ring full) since the capture started;
  - capture drops (limits);
  - for records not wanted, the fact that they were not recorded.

  All of it goes in `metadata.json` and `status`.
- **Auto triggers:** fire only when idle, respect the cooldown and the hourly cap, and record their reason.

### 5.3 Bundle (`gatr2.capture/1`)
**Files:** a store-only ZIP containing:
- `metadata.json`, `README.txt` (human schema);
- `events.csv`, `robot_state.csv`;
- `pico_sensor.csv`, `pico_status.csv`, `pico_diag.csv`;
- `vex_imu.csv`, `brain_requests.csv`, `brain_telemetry.csv`, `paths.csv`;
- `transport_bytes.csv` (only when selected).

Omit files for streams that were not selected; list present and absent streams in metadata.

**CSV conventions**
- Header row; units in the column names (`x_m`, `heading_deg`, `stamp_ms`, `pi_host_us`).
- An empty field means missing; booleans are 0/1; decimal points; `\n` line ends.
- Every row has `pi_host_us`, plus `source_clock` and `source_ms` when the source has its own clock.
- Also the identifiers that apply: `pi_session`, `reset_count`, `odometry_epoch`, `anchor_revision`, `boot_id`, `acq_epoch`, `imu_epoch`, `brain_session`, `request_id`, `seq`, `publication`.
- Headings in degrees wrapped to (-180, 180] plus a `heading_unwrapped_deg` column where useful.
- Transport bytes are hex strings.

**`metadata.json` must contain**
- schema, created `pi_host_us`, and wall clock if known;
- Pi session id, reset count, configuration name/digest;
- running profile summary (topology, IMU source, wheels with CPR/gear/radius/position/angle/polarity/travel scale, footprint, calibration settings), and the calibration state;
- trigger (reason, requester), pre/post requested and actual, start/trigger/end times;
- streams (present, row counts, first/last times);
- drops;
- segments;
- units and conventions: frames (field and odometry), +x forward, +y left, CCW headings, clocks and what each clock is;
- software identity (build date, contract versions).

### 5.4 HTTP (W3 implements handlers; W1 routes them)
- **`POST /api/capture/start`**, query `pre_s`, `post_s`, `streams` (comma list of kind names, `default`, or `all`): returns `{ok, id}` or `{ok:false, error}`.
- **`POST /api/capture/cancel?id=`**
- **`GET /api/capture/status`**: the `writeStatus` object.
- **`GET /api/capture/<id>.zip`**: `application/zip`. 404 while not ready.
- **Interface:** W3 exposes free functions in `src/capture/capture_http.h`, for example `HttpResponse captureRoute(System&, const HttpRequest&)`, returning false when the path is not a capture route. W1 calls them from the route table. Declare them in your first hours and note it in your progress file.

### 5.5 Tests (W3)
- Ring bounds.
- Pre-window trimming.
- Post-window stop.
- Truncation.
- Cancel.
- Busy refusal.
- Auto trigger cooldown and cap.
- Session boundary segments.
- Drop accounting.
- CSV escaping and missing values.
- **ZIP round trip:** a test-side minimal ZIP reader parses the bundle back, and the CRCs match.
- **Replay consistency:** records in, exported CSV rows out, the same values.

## 6. Wire codec additions (W4, first; W5 depends)

**Codec (translaGATR)**
- **TELEMETRY:** `encodeBrainRequest`/`decodeBrainRequest` handle op 12 with the fixed 54-byte body. The Ok reply is header-only. Length tables are updated.
- **Pico command:** handles `kPicoOpDiagnostics` with a 1-byte `diag_hz` body; values 6..255 are BadBody on the Pico.
- **`encodePicoDiag`/`decodePicoDiag`:** new functions for frame 0x14.
- **FrameReader:** accepts 0x14 and fills `FrameReaderStats`.
- **Tests:** independent known-byte vectors (a Python packer in `$S`, as before) for all of these, plus the stats counting cases.
- **Docs:** `docs/interfaces.md` updated.

**Pico (aggreGATR)**
- Handles DIAGNOSTICS (rate 0..5). Record it like the other commands, and dedupe it.
- Builds diag frames on the same schedule as status frames. They never delay sensor ticks; a diag may be skipped.
- Pins are read with `gpio_get`, never changing the function or direction.
- IMU driver counters feed `imu_*`/`reports_*`.
- `link_rx_bad` comes from the command FrameReader stats; `ticks_skipped` counts sensor ticks skipped for a busy FIFO.
- Host tests; both envs must build.

**Pi**
- **pico_telemetry:**
  - decodes 0x14 and keeps the latest;
  - sends DIAGNOSTICS when `<Diagnostics hz="1"/>` is present in its resource config, sending it again after a Pico reboot;
  - marks diagnostics unavailable with the reason `firmware` on UnknownOp;
  - posts `kPicoSensor`/`kPicoStatus`/`kPicoDiag` to the hub (check `wants()` for sensor);
  - feeds its LinkMonitor from the reads and writes it already does.
- **brain_link_commands/publisher:**
  - TELEMETRY: decode, answer Ok, post `kBrainTelemetry`, and keep the attitude for the robot attitude (section 2.6). It never touches localization state.
  - Post `kBrainRequest`, `kVexImu` and `kPath`.
  - Feed the Brain link's LinkMonitor at the decoded frame layer. For USB, also count dropped NG1 lines as rejections.
- **`writeInstrumentation`:** full schema, section 7.
- **Brain-profile configs:** add `<Diagnostics hz="1"/>` to the Pico telemetry resource. Tests in the files you own.

## 7. Instrumentation JSON (W4 writes, W2 renders)

```
{ "links": [ { "id", "kind",
               "rx_bytes", "tx_attempted", "tx_accepted", "rx_frames", "tx_frames", "rejected",
               "reader": {"bytes","frames","sync_dropped","length_errors","check_errors"},
               "rates": {"rx_bytes_s","tx_bytes_s","rx_frames_s"},
               "last_rx_ms_ago", "last_tx_ms_ago", "last_valid_rx_ms_ago",   // null when never
               "raw_on", "raw": [ {"ms_ago","dir":"rx|tx|tx_attempted","hex":"AA55..."} ],
               "decoded": [ {"ms_ago","dir","name","fields"} ],
               "errors": [ {"ms_ago","reason"} ] } ],
  "pico": { "available": bool, "reason": "...",           // e.g. "firmware" or "not requested"
            "status": {...PicoStatus fields, ages...},
            "diag": { "age_ms", "seq", "firmware",
                      "pins": [ {"name","level":"HIGH|LOW","known":bool,"driven":bool} ],
                      "imu": {...}, "link_rx_bad", "ticks_skipped" } | null,
            "encoders": [ {"port","counts","delta_1s","direction":"fwd|rev|still",
                           "note":"no change: stationary or disconnected, cannot tell"} ] },
  "brain": { "telemetry_age_ms", "telemetry_supported": bool|null, "vex_imu": {...} },
  "hub": { "posted":{kind:n}, "dropped":{kind:n}, "queued" } }
```
Ages are computed on the Pi (host clock) at write time.

## 8. Brain (W5)

**communiGATR**
- `Client::reportTelemetry(const translagatr::BrainTelemetry&)`: latest wins, best effort, at most `telemetry_period` (default 0.1 s). It is the lowest priority, below path reports, and must not delay state polls.
- If the Pi answers UnsupportedOp or InvalidArgument to TELEMETRY: set `telemetryUnsupported` for this session, count it, and keep the session.
- `ProsLink::reportTelemetry` with the bounded mutex.
- FakePi supports TELEMETRY and an "old Pi" mode answering UnsupportedOp.
- Tests cover scheduling (never starving state polls), latest-wins, and old-Pi behavior on both transports.

**Attitude** (portable)
- `attitude.h/.cpp`: `robotAttitudeFromVex(roll_deg, pitch_deg, mount_yaw_deg) -> {roll_rad, pitch_rad}`. It rotates the gravity direction; exact, not a small-angle approximation.
- Tests: 0/90/180/270 degrees and combined tilts.
- `ProsVexImu` gains `attitude()`, which reads `get_roll`/`get_pitch`. It is invalid while calibrating.
- Document the VEX sign conventions as UNVERIFIED, with a bench check procedure (tilt the nose up, check the viewer).

**actuGATR**
- A portable `telemetryOf(...)` builds the motion and wheels groups from `DriveSnapshot`/`MotionStatus`/`DriveStatus`: commanded chassis, wheel rpm targets and destination, with unit conversion and saturation. Add the fields to `DriveStatus` if they are missing.
- Tests.

**Programs**
- `gatr2_robot.h` gains `kSendTelemetry` (default true) and `kVexImuMountYawDeg` (0, PLACEHOLDER).
- operaGATR sends motion, wheels and attitude every 100 ms from the drive task snapshot, without holding any lock the drive task needs.
- locaGATR sends attitude.
- Both PROS builds must link with 0 warnings; ARM-check the portable sources.

**Docs:** the telemetry, attitude and enum tables.

## 9. Performance measurement (W0)

**Baseline source:** a frozen copy of the pre-change Pi runtime and viewer is at `$S/viz-baseline-src` (`pi/naviGATR`, `translaGATR`). Build the baseline from it in `$S/w0-base-build`. Never modify the repo copy for the baseline.

**Harness:** `pi/naviGATR/tools/perf`.
- **C++ `inspect_bench`:** runs a System (synthetic rig or demo config) plus an InspectionService in-process. It opens N WebSocket clients, including a slow client (reads B bytes/s, or pauses) and a burst scenario. Measure:
  - snapshot/state/diag build time and size;
  - publication and receipt rates;
  - server queue bytes, replacements, refusals and closes;
  - received-pose staleness at receipt, meaning now minus the message's `host_ms` and minus `measured_at_host_ms`. Same process, same clock, exact;
  - RTT.
- **Protocol support:** the harness must speak inspect/1 (baseline, snapshot-only) and inspect/2.
- **Browser:** a headless Chrome/Edge via CDP (Python `websockets`, or raw CDP over HTTP/WS; install nothing globally, and a venv in `$S` is fine). Load the viewer against a running demo config. Measure:
  - frame time distribution idle and during a scripted orbit drag (`Input.dispatchMouseEvent`);
  - long tasks;
  - message handling cost (add `performance.mark` hooks through a query flag in W2's code, or measure `onmessage` from outside);
  - heap;
  - a slow-client scenario (CPU throttling via `Emulation.setCPUThrottlingRate`, network throttling).

**Recording:** conditions (machine, OS, browser version, config, rates, client count), with medians/p95/max.

**Deliverables**
- `stage_results/W0_baseline.md` early (stage 1).
- `stage_results/W0_after.md` after integration (stage 2), with the same scenarios on the final tree.
- Scripts must run on both.

## 10. Stages

1. **Stage 1 (parallel): W0 through W5 implement.**
   - W4 does the translaGATR codec first and notes it in its progress file; W5 needs it for its encode tests. W5 starts with the attitude math, the actuGATR helper and client scheduling against the frozen structs.
   - Each worker ends with its own tests green in its own build dir and a stage result.
2. **Integration (lead):**
   - CMake entries;
   - full builds and tests (brain, translaGATR, naviGATR, aggreGATR host tests, pio, PROS);
   - browser workflows (W2 scripts): slow client, recording then replay;
   - W0's after-measurements.
3. **Cross reviews:** independent reviewers review the finished areas. The owners fix; the lead integrates.
4. **Freeze.** Record the commit and working-tree state.
5. **Phase 2 audit:** read-only, separate reviewers, report only.

## 11. Inventories
`progress/inventory_pi.md` and `progress/inventory_viewer.md` are the explorer reports on the current code. They are facts with file:line references; verify before relying on them.

## 12. Amendments

**12.1 (lead, 2026-09-28): TELEMETRY and state-poll timing.** The Brain link carries one request at a time, so any request delays the next one. TELEMETRY therefore meets the "never delays state polls" line of section 8 when:
- it never takes the turn of a poll that is due;
- it is sent only in the free slot after a state reply;
- at most one telemetry exchange (about 10 to 15 ms) sits between two polls.

W5 implemented it this way. Reviewers: an occasional poll gap of one extra exchange is not a finding. Starvation or a telemetry burst would be.

**12.2 (lead, 2026-09-28): slow-link staleness, handled at integration.** W2 measured several seconds of pose staleness on slow links even though the queue no longer backs up:
- 40 KB/s: 1.5 to 3.6 s at p50;
- 400 KB/s: 110 to 140 ms.

Causes: diag messages of about 20 KB at 4 Hz, JPEG previews, and a large OS send buffer. The lead integration step does the following, with W1 as owner of the files:
1. A small default `send_buffer_kb` (16).
2. `TCP_NOTSENT_LOWAT` on Linux, where available.
3. An adaptive diag and preview rate per client: back off while that client's last diag or preview took longer to drain than its period, and while a `state` is waiting behind an in-flight large frame.
4. The `detection` entry in each preview frame header (W2's request).

Reviewers: this is known and scheduled.

**12.3 (user, 2026-09-28 06:4x): the Phase 2 read-only audit is deferred.** Finish, validate and document the visualizer and diagnostics work (Phase 1) only. Do not start the audit until the user asks.
