# W2 stage 1 result: spectaGATR viewer (inspect/2 client) and browser tests

Scope: spec sections 3 (client side) and 4. Only W2-owned files were edited:
`pi/naviGATR/spectaGATR/**` (flat) and `pi/naviGATR/tools/viewer/**` (new). Nothing committed.

## What was built

**New modules (pi/naviGATR/spectaGATR)**

| File | Purpose |
|---|---|
| feed.js | The one WebSocket. Handlers parse and store (latest per channel, queued history/events, versions, receipt times). It speaks inspect/2, reads inspect/1 `snapshot` by splitting it, sends ping/subscribe/history/preview only, and counts seq gaps and handler cost per channel. |
| smoothing.js | Presentation-only interpolation (details below). |
| series.js | Bounded time rings per channel, with a NaN-for-missing, invalid and break flag per slot. Min/max per-pixel decimation keeps spikes and gaps (3x median interval); heading unwraps within a segment. |
| graphs.js | Live collector (per message, typed arrays, publication-deduped) and the canvas plot panel. Plots are selectable; legends carry units and sources. Invalid spans show as red ticks, events as dashed markers, hold/unwrap/window controls sit beside the plots, and a click seeks in replay. |
| bundle.js | Store-only ZIP reader with a CRC-32 check (deflate through DecompressionStream if present), RFC 4180 CSV, and units read from column names. |
| replay.js | gatr2.capture/1 model per W3's docs/capture.md headers. It answers robot/trail/path/telemetry at a playhead; segments split at session/reset/epoch, and the trail is drawn in odometry with the row's anchor. Every numeric CSV column is plottable. ReplayClock runs at 0.25x to 4x. |
| latency.js, latency_panel.js | The spec 3.4 terms (min-RTT ping offset, +-RTT/2), frame interval/work, panel cost, messages/s per channel, seq gaps, the server per-client queue stats, and the document build stats. |
| capture_panel.js | Start (pre/post/streams, bytes off by default), cancel, status, list, download, and "replay" from the Pi list or a local ZIP. |
| instrumentation_panel.js | The section 7 object: link counters/rates/ages/rejections/reader stats, a raw hex view (rx / tx ok / tx tried) and a decoded list (both opt-in), and the Pico diag. Pins are shown as logic levels read back; encoders show counts/change/A-B levels. Honest notes come from both the page and the Pi. |
| dom.js | Keyed tables, badges and lines (update in place, never innerHTML rebuilds). |

**Reworked:** app.js, field_scene.js, camera_panel.js, diagnostics.js, link_panel.js, index.html, style.css, transforms.js.

- **One rAF loop.** It applies the newest stored state, diag and telemetry, then renders only when something changed, with at least one redraw per second.
- **Panels.** Panels refresh at <= 10 Hz and only when visible (IntersectionObserver for the side panels; dock tabs). The instrumentation stream is subscribed only while its tab (or the live graphs) is visible; raw/decoded only on request.
- **Trail.** An odometry-frame ring of 4000 points, mirror-written so the draw range stays contiguous. A new point uploads 24 bytes twice (addUpdateRange). The line object carries field_from_odom; a new epoch clears it and requests history.
- **Path.** Points plus one line (no mesh per vertex).
- **Labels.** Re-rasterized only on text change, at most once per second; landmark age moved to the DOM table.
- **Picking.** At most once per frame, never while dragging; cached rect; tooltip measured once per text.
- **Follow.** No per-frame allocation.
- **Overlays.** Telemetry destination ring and heading arrow, the desired heading arrow at the robot, and the follower target (only when the path is the command's own and segment_count + 1 == vertices).
  - Per W5's note, a target draws only while the command is running or settling (or target flag bit 3); (0,0,0) is never drawn as a guess.
- **Velocity frames.** Commanded velocity (body frame) is plotted against the actual velocity rotated into the body frame by the same state's odometry heading, and labeled so. Odometry-frame velocity has its own plot.
- **Replay banner.** A REPLAY banner. Live-only controls (preview budget, capture start/cancel) are disabled, and live side panels are tagged LIVE and dimmed. The live socket keeps running unrendered, and replay has no send path.
- **Render budget.** Frames costing more than 12 ms are spaced so drawing takes about a third of the main thread; full rate while dragging. `?nobudget` disables it.
- **Load retry.** index.html awaits import('./app.js') and reloads up to 5 times if a module fetch failed.
- **Preserved.** Every feature in inventory_viewer.md section 4 is kept, as are the smoke hooks (`#status` data-* and `window.__navigatr`).
  - Added: data-attitude-status, data-contract, data-mode, data-trail, data-diags, data-smoothing, `__navigatr.metrics()`, `replayState()` and `seek()`.
  - `?perf` adds performance.mark/measure per message and per frame (for W0).

**Smoothing (the latency it adds)**
- It interpolates from the drawn pose to the newest received pose over one measured state period, capped at 100 ms.
- The drawn robot trails the newest pose by at most that period: about 33 ms at 30 Hz, about 16 ms on average. It never extrapolates.
- It snaps on any of:
  - session, reset, epoch, anchor, placement or validity change;
  - reconnect;
  - a jump over 0.2 m or 15 deg;
  - stale data (no state for 250 ms, or pose age over 250 ms). The robot is then drawn exact, dimmed and labeled "pose stale", with a status badge.
- Readouts, graphs, trail and exports never read it.

**Browser tests (pi/naviGATR/tools/viewer, Python stdlib only)**
- cdp.py: a minimal CDP client (WebSocket, eval, throttling, file input, mouse, screenshot).
- stub_server.py: an inspect/2 test double with latest-wins per client, a capture recorder and /stub/ controls.
- capture_fixture.py: writes gatr2.capture/1 ZIPs with W3's headers and a closed-form trajectory.
- unit.html and unit_tests.js: browser unit tests.
- viewer_tests.py: runs the unit, live, nowebgl, slow, reconnect, replay and record scenarios against `--stub` or `--binary`.

## Test commands and results (host tests; not hardware validation)

**Conditions**
- Windows 11, i9-14900HX (32 threads), 16 GB.
- Chrome 153.0.8010.53 headless, WebGL in software (swiftshader, no GPU).
- Target builds:
  - current tree: `$S/w2-nav/navigatr.exe`, Release, navigatr target only, 0 warnings;
  - frozen baseline: `$S/w2-base/navigatr.exe` from `$S/viz-baseline-src`.
- Config: config/demo/synthetic_field_demo.xml, rewritten with static_root = the working-tree spectaGATR.
- Other workers were building in parallel, so timings are noisy.

Final run on the final code (2026-09-28):
```
py pi/naviGATR/tools/viewer/viewer_tests.py unit live nowebgl reconnect replay record --stub
   -> unit 32/32 (31 unit tests + no page exceptions), live 14/14, nowebgl 5/5, reconnect 7/7, replay 16/16, record 15/15
py pi/naviGATR/tools/viewer/viewer_tests.py live nowebgl reconnect replay record --binary $S/w2-nav/navigatr.exe
   -> live 14/14, nowebgl 5/5, reconnect 3/3, replay 16/16, record 15/15 (record: W3's real capture API; start from the
      page, second start refused 409, the finished Pi bundle replayed)
py pi/naviGATR/tools/viewer/viewer_tests.py slow --binary $S/w2-nav/navigatr.exe --rate 50000000 --cpu 4
   -> PASS 4/4: staleness p50 247 ms, p95 523 ms, max 542 ms (27 samples, Pi clock only)
bash pi/naviGATR/tools/viewer_smoke.sh $S/w2-nav/navigatr.exe <config with static_root> --attitude-badge valid   (W1's script)
   -> PASS (contract inspect/2, attitude-status measured, trail 86)
```
Earlier the same day, inspect/1 compatibility against $S/w2-base/navigatr.exe (frozen baseline, --config its own demo):
live 13/13, nowebgl 5/5, reconnect 3/3, replay 16/16.

**Mutation check.** Disabling gap detection or the stale rule makes 2 unit tests fail; restoring the code passes.

**Slow-link measurements**
- The proxy paces bytes on a schedule, with a small upstream window. Staleness is measured on the Pi clock only: GET /api/snapshot host_ms minus the page's newest state host_ms. Each run was 15-20 s after 5 s of warm-up.
- 400 KB/s: p50 110-140 ms.
- 40 KB/s, previews off:

  | Send buffer | diag on | diag off |
  |---|---|---|
  | send_buffer_kb=8 | p50 1.5 s | 112 ms |
  | OS default | 3.6 s | 2.3 s |

- 100 KB/s, previews at the server default, CPU 1x: p50 1.5 s, p95 1.8 s.
- No run showed growing backlog; the server's own stats show latest-wins replacing (for example state 45 sent / 432 replaced).
- The residual staleness comes from about 20 KB diag frames at 4 Hz, JPEG previews and kernel send buffers (see issue 4).

**CPU 4x, unlimited link (3 runs each)**

| Render budget | p95 | max |
|---|---|---|
| with budget | 0.5-1.0 s | about 1 s |
| without (`?nobudget`) | up to 4.4 s | 5.9 s |

Profile at 4x: about 32% three.js, 49% native renderer time; message handlers a few percent.

## Contract issues and findings (details in progress/W2.md)
1. **W1, preview header.** Please add `detection` (the detection_frames entry of that exact image) to the preview header. With diag at 4 Hz, the overlay identity match is otherwise less frequent. The viewer uses the field if present.
2. **W1, slow links.** Consider a small default `send_buffer_kb` (8-16) and/or TCP_NOTSENT_LOWAT on Linux, plus a leaner or rate-adapted diag for slow clients. Numbers are above.
3. **W1 (resolved).** max_clients counts only feed clients now. The viewer's reload-on-failed-module stays as defense in depth.
4. **W3.** Replay reads columns by name, so the new `new_measurement` column is harmless and is plottable like any column.
5. **W5.** The telemetry target has no validity bit; the viewer applies the running/settling rule and honors flag bit 3 if it is adopted.

## Open risks
- Frame numbers from this machine are software-GL numbers. A GPU laptop should hit the 12 ms budget far less often; not measured here.
- The stub is a test double; only the --binary runs exercise the naviGATR server.
- Heading unwrap toggle, the graph hold, and event-marker hover are exercised by hand only (screenshots), not asserted.
- Replay of very large bundles (the 64 MB limit) was not tested; parsing is slice-based, and the rows sit in memory as strings, about 2-3x the CSV size.
