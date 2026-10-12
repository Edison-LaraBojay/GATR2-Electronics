# Discovery: spectaGATR viewer inventory and migration to one multi-source viewer

Scope: pi/naviGATR/spectaGATR (the browser viewer), its serving, its tests,
its replay model, and what must change so ONE viewer shows Brain-only,
Brain + naviGATR, and naviGATR-only. Read-only; nothing in the repo changed
(`git status --short` empty before and after). Tree: HEAD ae457ee.

Basis tags: [code] file:line verified, [measured-host] ran here (command
given), [inference], [unknown]. Paths are relative to C:/GATR2-Electronics
unless absolute. `SV` = pi/naviGATR/spectaGATR.

## 1. File inventory (SV, 19 files, 8797 lines) [code]

| File | Lines | Responsibility | Key exports | Class |
|---|---|---|---|---|
| index.html | 258 | DOM skeleton, importmap `three -> ./vendor/three.module.min.js` (:10), `#status` data-* hooks (:14), panels: Brain link (:59-68), Camera (:70-84), Landmarks (:86-89), dock tabs diag/graphs/inst/latency/capture (:95-231), module loader with 5x reload retry (:234-256) | - | mixed (naviGATR panel markup) |
| app.js | 1157 | orchestrator: Feed wiring (:111-144), per-state numbers (:151-177), piNow (:181-188), subscriptions (:278-287), preview budget (:313-345), applyFeed per rAF (:379-444), onHello/onState/liveRobot (:456-526), replay mode (:539-735), panels at 10 Hz (:754-800), landmarks table (:802-835), status/badges/readout (:852-977), rAF loop + render budget (:1040-1092), `window.__navigatr` (:76, metrics :1095-1122, replayState/seek :1123-1132), load-event hold (:1143-1157) | none (module) | mixed, core of migration |
| feed.js | 449 | the one WebSocket: same-origin `/ws` (:94-97), reconnect, ping timer (:130-138), parse-and-store dispatch (:255-286), session model (:288-311), inspect/1 compat split (:412-427), binary preview frames (:429-448) | `Feed`, `CONTRACT_2` | transport generic; dispatch inspect/2-specific |
| latency.js | 173 | ClockSync (min-RTT of last 20 pings, offset = host - (send+recv)/2, +-RTT/2) (:16-84), Stat ring (:87-135), Rates (:138-173) | `ClockSync`, `Stat`, `Rates`, `PING_KEEP` | generic |
| smoothing.js | 193 | presentation-only pose blend (<=100 ms, snap on identity/jump/turn/stale) (:53-193), `poseFreshness` 250 ms rule (:26-34), `measuredOnSourceClock` reads robot.measured_at (:40-43) | `PoseSmoother`, `poseFreshness`, `measuredOnSourceClock`, `SMOOTH_MAX_MS`, `SNAP_DIST_M`, `SNAP_HEADING_DEG`, `STALE_MS` | generic (identity fields naviGATR-shaped :59, :80-96) |
| series.js | 446 | bounded time rings per channel, NaN-for-missing, break/invalid flags, min/max decimation, wrap/unwrap drawing | `Channel`, `Series`, `SeriesStore`, `lowerBound`, `valueAt`, `Decimator`, `drawDecimated`, `FLAG_*` | generic (comment :10-12 says one clock per store) |
| graphs.js | 840 | `PLOTS` presets (:26-46), `META`/metaFor fixed ids + prefixes (:49-98), `LiveCollector` maps inspect/2 docs to series (:106-311), canvas `Plot` (:314-578), `GraphPanel` picker/hold/columns (:626-820), localStorage `spectagatr.graphs` (:822-840) | `LiveCollector`, `GraphPanel`, `PLOTS`, `GRAPH_HZ` | Plot/GraphPanel generic; LiveCollector/PLOTS/META naviGATR-specific |
| field_scene.js | 1582 | three.js scene: field from hello.fields (:492-564), planning boxes (:567-614), robot (:724-892), trail ring in odometry frame + anchor (:902-1099), applyDiag: footprint, camera mounts, landmark estimates, path (:1103-1122), path (:1126-1184), camera mounts on robot (:1186-1245), estimates (:1247-1340), telemetry overlays (:1344-1456), clearSession/setMode (:1461-1485), picking (:1510-1567) | `FieldScene`, `TRAIL_MAX`, re-exports `attitudeStatus`, `enumName` | scene core generic; its inputs are naviGATR document shapes |
| camera_panel.js | 598 | JPEG preview canvas, overlays bound to (camera, epoch, sequence), tag list/details, decode only when shown | `CameraPanel` | naviGATR-specific |
| diagnostics.js | 216 | sources/workers/localization functions/diagnostics links/session tables + preview budget form | `Diagnostics` | naviGATR-specific |
| link_panel.js | 275 | Brain-link readiness (link, profile, sensors, map, calibration, placement) from diag.brain_link + state.robot (:29-114), profile, wheels, Pico, event log | `LinkPanel`, `readinessOf` | naviGATR-specific (the Pi's view of the Brain link) |
| instrumentation_panel.js | 237 | link counters/rates/rejections, raw hex/decoded, Pico pins/encoders, Brain telemetry support, hub | `InstrumentationPanel` | naviGATR-specific |
| latency_panel.js | 124 | latency terms, per-channel rates/gaps/handler cost, server per-client queue stats from diag.inspection (:80-118) | `LatencyPanel`, `replacedFor` | terms generic; client/build tables read naviGATR server stats |
| capture_panel.js | 343 | Pi capture start/cancel/status/list/download/replay; `CAPTURE_KINDS` = Pi hub streams (:18-29); same-origin `/api/capture/*` (:168, :207, :226, :238, :329) | `CapturePanel`, `CAPTURE_KINDS` | naviGATR-specific |
| bundle.js | 281 | store-only ZIP reader with CRC check, RFC 4180 CSV, `unitOf` column-suffix units | `crc32`, `readZip`, `parseCsvRows`, `CsvTable`, `unitOf` | generic |
| replay.js | 623 | gatr2.capture/1 model on `pi_host_us` (:17-45, :82-94, :135-154), robot/trail/path/telemetry at playhead, graph store of every numeric column (:306-349) + live-id aliases (:352-424); `ReplayClock` (:583-623) | `ReplayModel`, `ReplayClock` | ReplayModel Pi-bundle-specific; ReplayClock generic |
| transforms.js | 131 | planar/3D transform helpers, camera projection mirroring src/math/camera_model.h, fmt/fmtMs, attitudeStatus, enumName | `DEG`, `wrapDeg`, `composePlanar` (unused), `rotationRpy`, `rotationOf`, `transformPoint`, `projectEngineering`, `fmt`, `fmtMs`, `attitudeStatus`, `enumName` | generic |
| dom.js | 216 | keyed DOM updates (write on change only) | `el`, `setText`, `setClass`, `setTitle`, `setHidden`, `KeyedTable`, `BadgeSet`, `Lines` | generic |
| style.css | 655 | dark theme, panels, `body.replaying` dimming (:620-634) | - | generic styling, naviGATR panel ids |

Small dead/duplicate code [code]: `composePlanar` (transforms.js:20) unused;
`FieldScene.trailFromState` (field_scene.js:1041) used only by unit_tests.js:300;
camera_panel.js:59-68 duplicates dom.js `el`.

### Message types consumed (inspect/2 plus inspect/1 compat) [code]

| Message | Delivery (server) | Stored in | Consumers |
|---|---|---|---|
| `hello` | reliable | feed.js:313-328 (hello, features, contract2, sub.state_hz) | app.js:456-471 -> scene.buildField, cameraPanel.setHello, diagnostics.setHello; link_panel readiness map rev (:76-86); renderLandmarks (:825); applyPreview camera_sensors (:319) |
| `state` | replaceable, <=30 Hz + 1 s keepalive | feed.js:330-341 | app.js storedState (:151-177: trail queue, sourceAge, publishToReceive), onState (:473-512), liveRobot (:514-526), renderStatus (:852-977); graphs.js onState (:159-206) |
| `history` | reliable | feed.js:359-372 (batch queue) | app.js:408-414 -> scene.trailFromHistory |
| `diag` | replaceable, <=4 Hz | feed.js:343-355 (+ diag.events) | scene.applyDiag (field_scene.js:1103-1122), cameraPanel.updateDiag, graphs onDiag (:235-285), diagnostics, link_panel, latency_panel, renderLandmarks, badges (app.js:946-955) |
| `event` | reliable | feed.js:375-409 (dedupe by seq, host_ms order, keep 200) | link_panel renderEvents (:249-274), graph event markers (app.js:743-752) |
| `telemetry` | replaceable, <=20 Hz | feed.js:262-267 | scene.applyTelemetry (app.js:437-443, field_scene.js:1395-1438), graphs onTelemetry (:208-233), badge (app.js:956-959) |
| `instrumentation` | replaceable, <=4 Hz, opt-in | feed.js:268-273 | instrumentation_panel render (:95-152), graphs onInstrumentation (:287-310) |
| `capture` | reliable | feed.js:274-277 | capture_panel setStatus via app.js:759-764; badge (app.js:960-963) |
| `pong` | reliable | latency.js ClockSync.pong (:46-73) | m.rtt (app.js:139-142), piNow, publishToReceive |
| binary preview | replaceable per camera | feed.js:429-448 `[u32 LE len][header JSON][JPEG]` | camera_panel onFrame (:218-232) |
| `snapshot` (inspect/1) | - | feed.js:412-427 splits into state/diag/history | same as above |

Client -> server: `ping`, `subscribe`, `history`, `preview` only (feed.js:153-190,
app.js:308-345); replay adds nothing (app.js:10-12; verified by
viewer_tests.py:764-768). Contract doc matches code on these shapes
(pi/naviGATR/docs/inspection.md:90-122) [code].

## 2. Single-source assumptions and where the core scene is read

### Single-source assumptions [code]

| Assumption | Where |
|---|---|
| one WebSocket, same origin, fixed `/ws` | feed.js:94-97, 99-126 |
| one browser<->server clock estimate, reset per connection | feed.js:81, 130-138, 292-302; latency.js:16-84 |
| one session `{id, reset_count}`; any change clears events, history, state for everything | feed.js:48-50, 288-311 |
| one hello, one contract flag, one feature set | feed.js:53-56, 313-328 |
| connection stale = no `state` for 1 s (only state counts) | feed.js:200-205 |
| one "Pi clock now" for graphs, event ages, camera exposure age | app.js:181-188, 202, 1078; link_panel.js:268; camera_panel.js:495-497 |
| one graph store, x axis labeled "Pi host clock" | app.js:104-105; graphs.js:10-15, 817-818; series.js:10-12 |
| one robot, one trail, one path, one smoother, one telemetry overlay set | field_scene.js:318-326, 724-762, 902-920, 315-317; app.js:102 |
| one field scene; several hello.fields overlay each other, center/span from the last with dimensions | field_scene.js:518-561 |
| identity tuple = Pi session/reset/odometry_epoch/anchor_revision/placement | app.js:362, 487-495; smoothing.js:59, 80-96; graphs.js:170-182 |
| one client id for the latency panel's "this page" row | latency_panel.js:81-83 |
| capture service at same origin `/api/capture/*` | capture_panel.js:168, 207, 226, 238, 329 |
| one global `window.__navigatr`, one `#status` row of data-* | app.js:76; index.html:14 |
| load event held until 10 `state` + 1 frame | app.js:1143-1157 |
| replay timeline = `pi_host_us`; bundle without it is refused | replay.js:17-18, 82-94, 135-154 |

### Where the core scene comes from today [code]

Everything the 3D scene draws is naviGATR-sourced; the Brain's own data
arrives only as Pi relays (PATH_REPORT inside diag, TELEMETRY op 12).

| Scene element | Source message and field | Read at |
|---|---|---|
| robot pose (drawn, smoothed) | Pi `state.robot.field` (Pi localization estimate) | app.js:473-512 (applyRobotState :482, smoother.push :496), app.js:514-526 (placeRobot :525); built by inspection_document.cpp:1798-1835 from `robotFeed()->snapshot` |
| readout x/y/heading, pose age, badges | Pi `state.robot` (+ diag.localization) | app.js:881-977 |
| attitude (tilt, label, badge, graphs) | Pi `state.robot.attitude` (estimator attitude; for Brain VEX profiles it is fed by TELEMETRY) | field_scene.js:823-831; app.js:929-945; graphs.js:201-204 |
| trail | Pi `history.trail` + `state.robot.odom` at `measured_at_host_ms`, per `odometry_epoch`, drawn under `field_from_odom` | app.js:169-176, 405-425, 501-510; field_scene.js:902-1099 |
| field geometry | Pi `hello.fields[]` (configured FieldMap resources: dimensions, features, landmark visuals/mounts, boundary, obstacles) | app.js:456-466 -> field_scene.js:492-564; inspection_document.cpp:806-830 |
| landmark estimates + table | Pi `diag.field_objects` (Pi world estimation) + hello.fields nominal | field_scene.js:1247-1320; app.js:802-835 |
| robot outline | Pi `diag.brain_link.profile.running.footprint`, else `hello.robot_body` | field_scene.js:765-778, 1107-1116 |
| path | Brain PATH_REPORT relayed as Pi `diag.brain_link.path` | field_scene.js:1120, 1126-1169; inspection_document.cpp:1406-1425 |
| destination/desired heading/follower target, execution | Brain TELEMETRY relayed as Pi `telemetry.motion` | app.js:437-443; field_scene.js:1395-1438; Pi nulls target without flag bit 3 (inspection_document.cpp:1967-1970) |
| camera mounts/frustum on the robot | Pi `diag.detection_frames`, `hello.cameras` | field_scene.js:1186-1245 (added to `this.robot`, :1233) |

Consequence [inference]: with the new ownership rule, every "robot pose,
attitude, trail" read above becomes a Pi-instrumentation read, and the core
scene needs a Brain-sourced input that does not exist in the viewer today.

### What the Brain actually knows (constrains the core-scene inputs) [code]

- Pose: `investigatr::RobotState {status, pose (field frame), age, frame
  (FrameGeneration), connected, link_age}` (brain/investiGATR/include/investigatr/state_source.h:27-37).
  No odometry frame, odometry epoch or anchor -> a Brain trail is a
  field-frame trail broken per frame generation, not the Pi's
  odometry-frame trail re-expressed under anchor corrections (behavior change).
- Field: `investigatr::Field {generation, map {id = crc32 of the Pi map
  document, revision}, bounds, objects[] {numeric id, kind, obstacle, box,
  nominal, source none/nominal/observed, valid, pose, age}, frame,
  received_at}` (field.h:17-68). No walls, tiles, features, landmark
  visuals, tag mounts or string ids: a Brain-only scene can draw boundary,
  boxes and object markers; the rich display geometry needs a separately
  resolved, identity-verified definition.
- Path: typed segments `kTurn` (in place, direction) / `kTranslate`
  (reverse, exact) with start/end headings (path.h:17-40). The wire
  PATH_REPORT keeps at most 13 xy points, drops headings and segment kinds
  (frames.h:111, 247; link_driver.cpp:164-182 dedups a turn's point;
  client.cpp:229-240 decimates >13). The viewer's follower-target check
  `segment_count + 1 === points.length` (field_scene.js:1426-1427) is then
  false for any path with a turn or >13 points, so the target is hidden and
  turns are invisible [code + inference].

## 3. Generic vs naviGATR-specific (evidence)

| Module | Verdict | Evidence [code] |
|---|---|---|
| dom.js, series.js, latency.js (ClockSync/Stat/Rates), transforms.js, bundle.js, smoothing.js (PoseSmoother) | generic, keep as-is | no document field names except `attitudeStatus` (transforms.js:109-120, reads `status`/`valid`/`assumed_level`) and `measuredOnSourceClock` (smoothing.js:40-43, reads `robot.measured_at`) |
| graphs.js Plot, GraphPanel, Decimator use | generic | consume only `SeriesStore` + meta {label, unit, source, wrap} (graphs.js:314-578, 626-820) |
| graphs.js LiveCollector, PLOTS, META | naviGATR adapter | reads state.robot, telemetry.motion, diag.brain_link.wheels, localization.functions stillness, sources, diagnostics links, workers, instrumentation pico/brain/links (graphs.js:159-310) |
| field_scene.js core (floor/boxes/robot/trail/path/overlays/picking) | generic renderer | draws from plain objects; but `buildField` takes hello.fields shape (:492-564), `robotBodyFor`/`applyDiag`/`updateEstimates`/`updateCameraMounts` read diag.brain_link/field_objects/detection_frames (:765-778, 1103-1320) -> move these to adapters |
| feed.js | split | socket/reconnect/ping/counters generic (:94-253); `dispatch`, `noteSession`, `onSnapshot` inspect/2-specific (:255-427) |
| latency_panel.js | split | terms generic (:60-78); clients/builds tables read diag.inspection (naviGATR HttpServer stats, :80-120) - a receiver reusing HttpServer would have the same stats |
| replay.js | split | ReplayClock generic (:583-623); ReplayModel hard-codes gatr2.capture/1 columns and pi_host_us (:17-45) |
| diagnostics.js, link_panel.js, instrumentation_panel.js, camera_panel.js, capture_panel.js | naviGATR-specific | read diag.sources/workers/localization/diagnostics, diag.brain_link/pico, instrumentation links/pico/hub, detection_frames + JPEG header identity, Pi capture HTTP API and hub stream kinds |
| app.js | orchestration, mixed | wires all of the above; badges read Pi-only fields (field_snapshot, workers, clock_mapped) (:917-955) |

Capability-gating precedent exists [code]: link panel hides without
brain_link/pico (link_panel.js:131-133); instrumentation says "no Pico in
this configuration" (instrumentation_panel.js:160), "no Brain link"
(:210); camera says "no camera configured" (camera_panel.js:443-445).

## 4. Minimal migration to one multi-source viewer

### 4.1 Wire shape to the browser (proposal) [inference]

Keep ONE WebSocket per page (to whichever receiver serves the page: Pi or
laptop). Add a source envelope instead of a new transport:

- Receiver `hello`: `contract "spectugatr.view/1"`, `client_id`, receiver
  id/version, `viewer_build` (asset hash), robot `{id, name}`, `sources[]`.
- Reliable `source` message per source and on every change: `src` (stable
  key), `kind` (brain | navigatr | other), `robot_id`, `session` (source
  instance id + restart count), `state` (connecting | live | stale | lost |
  absent), `capabilities[]`, `clock {domain, mapping {offset_us,
  uncertainty_us, method, valid}}`, transport stats.
- Every forwarded message gains `src` and `rx_us` (receiver clock at
  arrival) and keeps its own `seq` and source-clock times. Binary preview
  headers gain `src` (header is already JSON, feed.js:439-441).
- Brain core messages (standard contract, not inspect/2): `pose`,
  `field` (definition + revision), `path` (lifecycle), `exec`, `channels`
  (definitions), `samples`, `event`.
- naviGATR messages pass through in inspect/2 shape with `src` added, so
  the existing naviGATR panels keep their parsers.
- Compat: a server that answers `navigatr.inspect/2` (today's Pi) is one
  implicit source `{src:"navigatr", kind:"navigatr"}` on the Pi clock.
- Client -> receiver: `subscribe`/`history`/`preview` carry `src`; the
  receiver forwards to that source adapter (keeps "hidden panel
  unsubscribes" economy end to end). `ping` stays receiver-level.

The existing HttpServer already supports this per client: replaceable
slots are keyed by any channel name with a priority (http_server.h:96-99,
183-188), so `brain:pose` can outrank `navigatr:diag` and
`navigatr:preview:<cam>` [code]. HttpServer depends only on sockets and
core/host_clock.h (http_server.cpp:14-50) [code].

### 4.2 Viewer changes, smallest first

| Step | Change | Files | Behavior preserved |
|---|---|---|---|
| M0 | Move SV to spectuGATR/viewer, three.js to spectuGATR/viewer/vendor/ (physical `vendor/` so any static server works), tools/viewer to spectuGATR/tests/viewer; rename hook to `window.__spectugatr` keeping `window.__navigatr` alias | all SV files unchanged except paths; CMake embed; test path constants | 100%; rerun existing 9 scenarios |
| M1 | Split feed.js: `Connection` (socket, reconnect, ClockSync, counters, send) + `SourceState` per `src` (hello/identity, session, versions, arrivals, events, history, staleness). Legacy inspect/2 = one SourceState. Session change clears only that source | feed.js; app.js reads `hub.source(src)` | inspect/2 dispatch code reused as the navigatr SourceState handler |
| M2 | Core scene model owned by the Brain source: `CoreScene {pose {x, y, heading, valid, status, frame_gen, t_src, age}, attitude {roll, pitch, yaw|null, supplied, status, t_src}, field {map_id, revision, planning, display?}, path {command_id, generation, state, segments[]}, exec}`. app.js feeds scene/smoother/trail/readout only from CoreScene. naviGATR `state.robot` becomes Pi instrumentation (graphs, a "Pi estimate" readout) | app.js (:151-177, 379-526, 852-977), field_scene.js (buildField input, trail in field frame per generation, path segments incl. turns) | smoothing rules, stale/unmeasured verdicts, render loop, trail ring, labels |
| M3 | Generic channels: `channels` defs `{id, label, unit, type, group, wrap, valid_rule}` -> `store.get(src + '/' + id, channel(src, group), meta)`; `samples` appended by source time mapped to receiver time. Picker lists groups dynamically (reuse replay's `setColumns`/`addColumnPlot`, graphs.js:713-748); PLOTS become optional presets | graphs.js, app.js | Plot/Decimator/series untouched; naviGATR LiveCollector stays as the navigatr adapter |
| M4 | Capability gating: panels mount only when a source declares the capability; absent = hidden or one "not present" line, never an error. naviGATR panels (Brain link, Diagnostics, Instrumentation, Camera, Pi capture) move under viewer/sources/navigatr/ | index.html, app.js, panel modules (paths only) | panel internals unchanged |
| M5 | Per-source status: badges/readout name the core source; Brain `lost` keeps the last pose drawn faded "Brain disconnected, last pose Ns ago" and NEVER falls back to Pi pose; per-source data-* hooks | app.js renderStatus | existing stale look (field_scene.js:780-785) |
| M6 | Replay multi-source (section 7) | replay.js, bundle.js | gatr2.capture/1 still replays as single navigatr source |

### 4.3 Per-mode behavior (target) [inference]

| Element | Brain only | Brain + naviGATR | naviGATR only |
|---|---|---|---|
| robot pose/attitude/trail | Brain `pose` | Brain `pose` | "core scene unavailable: no Brain source" (no robot drawn); Pi estimate only in graphs/Pi readout |
| field | Brain field (planning layer; display layer only if identity+revision verified) | same; a newer Pi revision is not substituted; Pi panel may show "Pi rev N vs Brain rev M" | Pi field: open decision (see Q1) |
| path, targets, exec overlays | Brain | Brain | unavailable |
| landmark estimates | Brain `Field.objects` (source/valid/age) | Brain; Pi `field_objects` only as labeled Pi overlay if the user wants (Q2) | Pi `field_objects` in Pi panel/table |
| graphs | Brain channels | Brain + naviGATR channels on the receiver timeline, each with source + mapping uncertainty | naviGATR channels |
| camera, link, instrumentation, Pi diagnostics, Pi capture | hidden (absent capability) | naviGATR | naviGATR |
| receiver capture/replay | yes | yes | yes |

Composition rules the viewer must enforce [inference]:
- Draw path/targets only when their frame generation equals the drawn
  pose's; otherwise dim and label "from an earlier placement".
- Camera mounts (T_robot_camera, robot frame) may ride on the Brain robot
  only when the Pi's running profile id equals the Brain's profile id;
  otherwise draw nothing on the robot and say why.
- Events from several sources are listed per source; a merged list is
  ordered by mapped receiver time and labeled, never by bare arrival.
- Smoother identity becomes {src, session, frame_gen, valid, connection}
  (smoothing.js:80-96 shape kept).

### 4.4 Clocks [code + inference]

Today: one ClockSync browser<->Pi (latency.js:16-84); graphs on the Pi
clock (graphs.js:10-15). Proposal: keep ClockSync unchanged for
browser<->receiver; the receiver owns one mapping per source (source clock
-> receiver clock) and publishes it in `source.clock`. The viewer plots on
the receiver clock only samples whose source mapping is `valid`, carries
`uncertainty_us` into the legend/tooltip, and never subtracts a
source-clock time from another domain. For a Pi source reached over
WebSocket the receiver can reuse the inspect/2 ping/pong (inspection.md:118-119,
500-508) with the same min-RTT method; for a Brain over serial the mapping
is one-way (arrival-bounded) unless the Brain protocol adds an echo; that
choice belongs to the transport design [inference].

## 5. How assets are served today, and a single copy for the receiver

[code]
- naviGATR embeds the viewer: CMakeLists.txt:141-180 globs
  `spectaGATR/*.{html,js,css,json,svg}` (flat, CONFIGURE_DEPENDS), maps
  each to its bare file name (:150-154, `get_filename_component NAME`),
  adds `vendor/three.module.min.js`, `vendor/OrbitControls.js`,
  `vendor/LICENSE` from third_party/three (:155-159), and runs
  cmake/embed_assets.cmake (hex byte arrays, content type by extension)
  into generated/viewer_assets.cpp compiled into navigatr_core (:176-181).
- Serving: inspection_service.cpp:511-533 `serveStatic`: `/` ->
  index.html; with `static_root` set, a regular file under it wins
  (Cache-Control no-store), else the embedded copy; so a static_root
  without vendor/ still gets three.js from the binary (viewer_tests.py:30-35
  relies on this). `static_root` must be a readable directory at create
  (inspection_service.cpp:1272-1278); attribute parsed in
  runtime/inspection_config.h:64, 100-101.
- Pinned three.js: pi/naviGATR/third_party/three/three.module.min.js
  (691648 B, r170), OrbitControls.js (32134 B, imports from 'three'),
  LICENSE (MIT); documented in pi/naviGATR/third_party/README.md ("three").
  Page resolves it via importmap (index.html:10) and
  `./vendor/OrbitControls.js` (field_scene.js:31-32).
- Two test harnesses re-map `vendor/` to third_party/three by hand:
  viewer_tests.py:434-440 (unit server) and stub_server.py:47-48, 707-717;
  unit.html:6 importmap points at `../../third_party/three/...`.

Single-copy proposal [inference]:
- spectuGATR/viewer/ holds the app and `vendor/three/...` physically at the
  served path, so every static server (receiver, Pi, Python stub, unit
  harness) serves the tree as-is; the two re-map hacks disappear.
- One CMake function in spectuGATR (e.g. `spectugatr_embed_viewer(<target>)`)
  wraps embed_assets.cmake; naviGATR and the desktop receiver both call
  it, so both binaries carry byte-identical assets. Must switch to
  `GLOB_RECURSE ... RELATIVE` and keep subpaths (the current NAME
  flattening would collide once the viewer has subfolders).
- Embed a viewer build hash; `hello.viewer_build` lets the page detect a
  receiver/page mismatch after an update.
- Desktop receiver: embedded by default (prebuilt package, no loose files,
  works offline); `--viewer-dir <dir>` for development with the same
  per-file fallback as static_root.
- HttpServer (pi/naviGATR/src/inspection/http_server.{h,cpp}, 227+1614
  lines) is already Winsock/POSIX portable and is the natural shared
  server. Portability gaps for macOS [code + unknown]: send uses
  MSG_NOSIGNAL only when defined (http_server.cpp:124-128) and nothing
  ignores SIGPIPE (only SIGINT/SIGTERM handlers, src/app/main.cpp:140-141);
  if the macOS SDK lacks MSG_NOSIGNAL a peer reset during send could raise
  SIGPIPE. TCP_NOTSENT_LOWAT is guarded by `#ifdef` (:642) so it would
  compile in wherever the SDK defines it. Needs a macOS build to confirm.

## 6. Tests and hooks

### Inventory [code]

| File | Lines | What |
|---|---|---|
| tools/viewer/viewer_tests.py | 1131 | headless Chrome/Edge over CDP; scenarios unit, live, nowebgl, slow (ThrottleProxy :206-291), reconnect (KillProxy :294-341), replay, record, capfail, paths (:1-27, :1087); targets `--stub` or `--binary` (static_root rewritten to this tree, :143-160) |
| tools/viewer/stub_server.py | 742 | Python inspect/2 test double: per-client replaceable slots, reliable FIFO, capture recorder writing gatr2.capture/1, controls /stub/{kill,reset,restart,epoch,jump,pause,unmeasured,ready,clock,capfail,stats} (:12-21, :605-648); Brain path embedded in diag.brain_link (:386-416), telemetry (:429-440) |
| tools/viewer/capture_fixture.py | 240 | deterministic gatr2.capture/1 bundles with closed-form pose; variants hole/frozen |
| tools/viewer/unit.html + unit_tests.js | 18 + 686 | 40 browser unit tests of smoothing, series, scene trail/labels/overlays, bundle, latency, feed, graphs, replay (unit_tests.js:49-639) |
| tools/viewer/cdp.py | 306 | minimal CDP client; temp browser profile via tempfile (:139) |
| tools/viewer_smoke.sh | 330 | --dump-dom smoke on a binary, reads `#status` data-* (:8-11, :247-303) |
| tools/perf/perf_probe.js | 250 | injected before the page; wraps WebSocket (:126-157) and rAF; reads `#status` dataset and `window.__navigatr` (:208-217, :243) |
| tools/perf/cdp_bench.cpp | 791 | idle/orbit (+_cpuN/_netK) scenarios; waits on `#status` data-live-seen (:693-694) |

Hooks the tools depend on [code]: `#status` data-state, -snapshots,
-frames, -session, -cycle, -errors, -attitude, -attitude-status, -webgl,
-live-seen, -readiness, -contract, -mode, -smoothing, -diags, -trail,
-pose, -replay-pose, -placement (index.html:14; app.js:852-977, 994);
`window.__navigatr` {feed, hello, errors, lastSnapshot, connection, mode,
metrics(), replayState(), seek()} (app.js:62-76, 1095-1132); element ids
used by tests (cap-*, replay-*, preview-form, lat-*, graph legend
classes).

### Measured here [measured-host]

- `TMP=<scratch>/spectu/viewer/tmp py -3 -B tools/viewer/viewer_tests.py unit live reconnect replay paths --stub --out <scratch>/spectu/viewer/stub-run`
  (cwd pi/naviGATR): PASS unit 41/41 (40 unit tests + no-exception check),
  live 14/14, reconnect 9/9, replay 20/20, paths 15/15; 61 s wall.
- `... viewer_tests.py live --binary <scratch>/final-navi/navigatr.exe --out <scratch>/spectu/viewer/bin-run`
  with ucrt64 on PATH: PASS 16/16 including camera preview hz 0 when the
  panel is hidden and back on when shown.
- Timing numbers from these runs (e.g. binary live: frame interval p50
  66.7 ms, publish-to-receive p50 137 ms, RTT p50 109 ms) were taken while
  other agents were loading the host; they are NOT a baseline and contradict
  nothing in the final report (loopback 0.39 ms was measured on a quiet
  host in Release).
- `git status --short` empty after the runs.

### What changes to test multiple sources [inference]

- stub_server.py: add a receiver mode emitting `spectugatr.view/1` with two
  synthetic sources: `brain` (field-frame pose on its own clock with a
  configurable offset/drift, field def + revision, typed path with turns,
  exec, channels) and `navigatr` (current inspect/2 generator, tagged).
  Per-source controls: `/stub/src/<src>/{kill,restart,pause,lost,absent,clockstep,fieldrev,placement}`.
- New scenarios: `brainonly` (no naviGATR panels, no errors, graphs from
  channels), `combined` (core scene from brain only: move the Pi pose and
  check the drawn robot does not follow), `navionly` (robot not drawn,
  core-unavailable badge, camera/diag/instrumentation live),
  `sourceloss` (brain lost -> last pose faded + labeled, no switch to Pi;
  recovery snaps), `fieldrev` (Pi advertises rev N+1 while Brain keeps rev
  N: scene stays at N; Brain switches: scene rebuilds, trail/path break),
  `placement` (path from old frame generation dimmed), `clocks` (offset
  source clocks: aligned plots within the declared uncertainty; unmapped
  source not co-plotted), `turnpath` (turn-only segments drawn and follower
  target resolved), `replaymulti`.
- Keep the old 9 scenarios running against the compat path (implicit
  navigatr source) until the Pi server speaks the new contract.
- Hooks: keep `data-live-seen` meaning "some source live" (cdp_bench
  waits on it, cdp_bench.cpp:693-694); add `data-core` (brain src key or
  `unavailable`), `data-core-pose` (fresh | stale | lost | none),
  `data-sources` (`brain:live,navigatr:stale`), per-source
  `[data-source=...]` on panels; `metrics()` gains `sources{}` with
  per-source counts, seq gaps, handler cost, mapping uncertainty.
  perf_probe.js wraps browser APIs so it keeps working unchanged.

## 7. Replay

### Today [code]
- Bundle gatr2.capture/1 (docs/capture.md:222-356): store-only ZIP,
  one CSV per Pi hub stream, one shared clock `pi_host_us`; other clocks
  only as `source_clock`/`source_ms` columns; metadata segments per domain
  (robot, pico, brain, pi).
- ReplayModel: timeline = min/max pi_host_us over all tables
  (replay.js:135-154); robot from robot_state.csv, segment = change of
  (session, reset, epoch) (:185-199); trail in odometry frame re-expressed
  by a per-row anchor (:472-481, :485-509); path groups by (pi_host_us,
  command_id) (:232-278); telemetry at or before t (:543-572); every numeric
  column plottable as `csv.<stream>.<col>` plus aliases to live plot ids
  (:306-424); segment starts become event markers (:292-300).
- ReplayClock plays 0.25x-4x, seek never plays through (:583-623).
- Replay only reads; the live socket keeps running unrendered and nothing
  but ping/subscribe is sent (app.js:539-568; viewer_tests.py:764-768).

Gaps found [code]:
- `new_measurement` (capture_bundle.cpp:186-191) is ignored by replay.js;
  repeated held rows become graph samples (duplicate samples in replay).
- capture_fixture.py headers lag the runtime: no `new_measurement` in
  ROBOT_HEADER (capture_fixture.py:24-29), no `target_valid` in
  TELEMETRY_HEADER (:31-36) vs capture_bundle.cpp:186-191, 226-228.
- replay.js:554 treats a telemetry row without a target as "no motion",
  so idle/waiting rows lose cmd/errors in overlays/tooltip (graphs still
  plot them via CSV aliases).
- Fields are not in the bundle: replay draws today's hello field, not the
  field (or revision) in force when recorded (app.js:456-466 is live-only;
  ReplayModel has no field).

### Multi-source bundle proposal [inference]
- New schema (e.g. `spectugatr.capture/1`), receiver-written; one shared
  timeline = receiver clock `rx_us`; every row also carries `src`,
  `src_session`, `src_clock`, `src_t_us`, `seq`, and the identities that
  apply (`frame_gen`, `field_id`, `field_rev`, `placement`).
- `metadata.json`: `sources[]` {src, kind, robot_id, sessions[],
  capabilities, clock domain, mapping per segment {offset, uncertainty,
  method}}, `fields[]` {map_id, revision, content hash, first rx_us,
  accepted rx_us}, loss per source (receiver drops, transport gaps from
  seq), units/frames/conventions as today.
- Field definitions stored as files (`fields/<map_id>@<rev>.json`) plus
  `field_revisions.csv` (rx_us, src, map_id, revision, event:
  offered | accepted | cleared), so replay rebuilds the scene with the
  revision the Brain had accepted at the playhead.
- `paths.csv` keeps segment kind, headings, direction and lifecycle
  (new, replaced, cleared, cancelled) instead of xy points only.
- Replay core = Brain streams only; Pi `robot_state` replays as Pi
  instrumentation. Segments split per source session and frame generation;
  trails and graph lines never join across them (existing rule, generalized).
- gatr2.capture/1 stays readable: adapter maps it to one `navigatr` source
  with timeline `pi_host_us` and no core scene (or, per Q3, a labeled Pi
  estimate).
- Combining a Pi-side bundle with a receiver bundle needs a mapping that
  exists only if the receiver recorded that Pi session; otherwise refuse
  to co-plot and say so.

## 8. Doc/code contradictions noticed [code]

| Doc/comment | Code |
|---|---|
| field_scene.js:1388-1394 "the target carries no validity bit yet ... a future target-valid flag, bit 3" | Pi nulls `motion.target` without bit 3 (inspection_document.cpp:1967-1970); docs/inspection.md:447-452 and capture.md `target_valid` describe bit 3 as live. Comment stale |
| inspection.md:313-315 "the viewer maps them to its own scene axes explicitly (see spectaGATR/app.js)" | the mapping is in field_scene.js:11-17, 38 (DEFAULT_UP +z, 1:1 axes) |
| series.js:10-12, graphs.js:10-15, graphs.js:817-818 "Pi host clock" | true only for single-source naviGATR; label will be wrong for any other source |
| viz final_report.md "overlays drawn only with a resolved target (flag bit 3)" | field_scene.js:1404 also accepts `active` without bit 3, but live target is already null without it, so effectively consistent; replay rows have no flags and need a target value (replay.js:554) |

## Open questions / needs bench test

1. naviGATR-only mode: may the 3D view show the Pi's configured field (and
   camera/landmark diagnostics in it) labeled "Pi configuration, no Brain
   source", or must the 3D view be blank? Requirement forbids a Pi-owned
   live robot feed, not necessarily Pi field context. User decision.
2. Combined mode: show the Pi pose estimate and Pi landmark estimates as a
   distinct, labeled overlay (useful for localization debugging) or keep
   them strictly in graphs/tables? User decision.
3. Trail semantics change: Brain field-frame trail broken per frame
   generation replaces the Pi odometry trail re-expressed under anchor
   corrections. Acceptable, or keep an optional Pi odometry trail overlay?
4. Brain-only display geometry: accept a plain boundary/box scene when no
   verified display definition is available, or require the Brain
   publisher to carry display geometry? (ties to field-transfer design.)
5. Clock mapping for a Brain on serial (controller or USB): one-way
   arrival-bounded, or add an echo op? Determines the uncertainty the
   graphs can show. Needs the transport agent's answer.
6. Path wire: typed segments with headings/turns vs the current 13-point
   xy PATH_REPORT; the viewer can only honor turn-only segments if the
   Brain publisher sends them.
7. macOS: HttpServer SIGPIPE/MSG_NOSIGNAL behavior; build and run the
   HttpServer tests on macOS before claiming support.
8. Browser cost with two sources at full rate plus previews over one
   socket at 4x CPU throttle: rerun tools/perf cdp_bench idle/orbit on a
   quiet host once a multi-source stub exists (the numbers measured here
   were under load and are not a baseline).
9. Wireless: none of the latency/smoothing numbers apply to the VEX
   radio + controller path; bench test needed for Brain pose rate and age
   at the browser.
