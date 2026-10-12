# W2 fix pass: review findings on the spectaGATR viewer

Scope: the 11 confirmed review findings on W2's stage-1 work. Only W2-owned files were edited:
`pi/naviGATR/spectaGATR/*` (flat) and `pi/naviGATR/tools/viewer/*`. No new files, so the CMake embed glob is unchanged.
Nothing committed, flashed or deployed.

## Files changed

- **spectaGATR:** app.js, feed.js, latency.js, latency_panel.js, capture_panel.js, camera_panel.js, diagnostics.js,
  field_scene.js, graphs.js, replay.js, smoothing.js, index.html (one element: `#cap-last`).
- **tools/viewer:** viewer_tests.py, stub_server.py, capture_fixture.py, unit_tests.js.

## Per finding

| # | Finding | Result | What changed |
|---|---|---|---|
| 1 | Replay never marks an old/stale robot row (medium) | Fixed | `replayFrame` applies the live rule on the recording's clock: `poseFreshness(row age, playhead - row time)`. A hole or a repeated pose with growing `source_age_ms` draws faded as "robot (pose stale in recording)", with a badge. The readout shows the row time, the pose age at the row, and the time from the row to the playhead (it used to print the playhead as the recording time). |
| 2 | Ping id reuse after a double clock reset (medium) | Fixed | `ClockSync` ids never restart. A pong uses its echoed `client_ms` (the server echoes it verbatim), and a pending time that disagrees is dropped. `Feed.onOpen` resets the clock once per connection; the first hello no longer resets it again. |
| 3 | A valid pose with `age_ms` null is drawn as live (medium) | Fixed | `poseFreshness` returns `'unmeasured'` when the age is unknown, and the robot gets the faded look and a badge. The runtime (motion_step.cpp) also empties `measuredAtHost` when the source clock is not mapped yet, so there are two wordings. `measured_at.ms` null: "pose not measured yet (configuration placement, no sensor reading)". `measured_at.ms` set: "pose age unknown: measured on the pico clock, not mapped to the Pi clock". Also added: a "localization not ready: <function (note)>" badge when `all_ready` is false, and `data-pose`. |
| 4 | "clock unmapped" badge can never appear on inspect/2 (medium) | Fixed | `clock_mapped` is read from `diag.localization`, or from `state.localization` when that carries it. |
| 5 | Capture panel hides `file_error` and `status.last` (medium) | Fixed | A new "file" column shows the saved path, "memory only", or "WRITE FAILED: <file_error>; in memory only" (red). A `#cap-last` line shows the last outcome: ready, cancelled, FAILED: error, or ready but the file write failed. Each change is also announced once in `#cap-msg`. A failed bundle, which leaves no list row, now shows. |
| 6 | 'capture' messages never reach the panel (low) | Fixed | `panels()` passes each new `feed.capture` to the panel, whether or not its tab is shown. HTTP polling is only a fallback, after 2.5 s without a push or on inspect/1. An older HTTP answer (by `pi_host_us`) never overwrites a newer push. The header comment is corrected. |
| 7 | No-WebGL replay says "no robot state" (low) | Fixed | `replayFrame` always runs; scene calls are no-ops without WebGL, and the trail work is skipped. The readout, `replayState()` and the scrub bar all follow the playhead. |
| 8 | Seq-gap column overstates replacements with several viewers (low) | Fixed | The column is relabelled "seq gaps (replaced, skipped or built for other clients)". A new "replaced for this page (server)" column comes from this page's own `diag.inspection.clients` entry (previews summed per camera). |
| 9 | Live telemetry plotted at message build time (low) | Fixed | Samples sit at `received_host_ms`, else at `host_ms - age_ms`. This is the same instant as the capture's `pi_host_us`, and the legend says "at Pi arrival". |
| 10 | Previews decoded while hidden (low) | Fixed for live; partly declined for replay | The page asks for `hz 0` while the camera panel is out of view or the page is hidden. Back in view, it re-sends the form budget (or nothing, if the server defaults are in force). This is re-applied per connection, and a budget the user set survives a new hello. Frames for a hidden panel or a camera that is not selected are kept undecoded (the newest per camera) and decoded when shown. Declined: pausing previews during replay, because the live camera panel stays on screen (marked LIVE). |
| 11 | Coverage gaps in the browser suite (low) | Fixed | New scenarios `capfail` and `paths`. `reconnect` now also drops the socket of the same process on `--binary`, through a relay. `nowebgl` replays a bundle, `replay` checks the hole and frozen-age spans, and `record` checks that a capture started elsewhere shows by push. The stub now matches the runtime more closely: see below. |

**Stub changes (finding 11)**
- The keepalive recomputes `age_ms`, and is sent only after 1 s without a state.
- Capture status is pushed on an HTTP start and on 1 Hz progress.
- Per-client stats have W1's shape, and hello carries `client_id`.
- The capture status has `last`, `file` and `file_error`.
- New controls: `unmeasured` (with `&source=1`), `ready`, `clock` and `capfail`.
- Ping ids are logged, and the reconnect scenario checks that none repeat on a connection.

## Test commands and results (host tests; not hardware validation)

**Conditions**
- Windows 11, i9-14900HX, headless Chrome 153 with swiftshader WebGL.
- Current tree: `$S/w2-nav/navigatr.exe`, rebuilt from the current tree (Release, navigatr target, 0 warnings).
- Baseline (inspect/1): `$S/w2-base/navigatr.exe`.
- Config: `config/demo/synthetic_field_demo.xml` (the harness points `static_root` at the working-tree viewer).

**Unit tests and stub scenarios**
```
py pi/naviGATR/tools/viewer/viewer_tests.py unit live nowebgl reconnect replay record capfail paths --stub
```
- unit: 41/41 (40 tests plus no page exceptions; it was 32/32).
- live 14/14, nowebgl 8/8, reconnect 9/9, replay 20/20, record 17/17, capfail 7/7, paths 15/15.

**Current-tree runtime**
```
py pi/naviGATR/tools/viewer/viewer_tests.py live nowebgl reconnect replay record capfail slow --rate 50000000 --cpu 4 --binary $S/w2-nav/navigatr.exe
```
- live 16/16, nowebgl 8/8, reconnect 7/7, replay 20/20, record 17/17, capfail 5/5, slow 4/4.
- The final slow run: p50 172 ms, p95 372 ms, max 511 ms.

**Baseline runtime**
```
py pi/naviGATR/tools/viewer/viewer_tests.py live nowebgl replay reconnect --binary $S/w2-base/navigatr.exe --config <baseline demo>
```
- live 15/15, nowebgl 8/8, replay 20/20, reconnect 7/7.

**Other checks**
- `bash pi/naviGATR/tools/viewer_smoke.sh $S/w2-nav/navigatr.exe <config with static_root> --attitude-badge valid` (W1's script): PASS.
- **Mutation check** (`$S/w2-fix/mutate.py`): putting back each finding's old behavior fails its tests.
  - Unit: 4 tests (findings 2, 3 and 9).
  - Stub: nowebgl 3 checks (7), replay 2 (1), record 1 (6), capfail 1 (5), paths 1 (4).
  - Current-tree runtime: live, 2 preview checks (10).
  - Files were restored byte-identical (md5 checked).
- **Slow scenario** (50 MB/s link, CPU 4x): 7 runs during the fix pass.
  - Ranges: p50 110 to 497 ms, p95 263 to 1486 ms, max 505 to 2432 ms.
  - The worst run failed its sample-count check (3/4), with one 4.8 s sample. 15 compiler processes from other workers were running during it. The other 6 runs passed.
  - Stage 1 measured p50 247, p95 523 and max 542 ms in one run.
  - These runs are not controlled, so no change is claimed either way.

## Contract issues

None blocking. An optional one for W1: `clock_mapped` in `state.localization` would let the badge follow the 30 Hz state instead of the 4 Hz diag. The page already uses it when present.

## Open risks

- The baseline server counts every TCP connection toward `max_clients` (stage-1 issue 6). Against it, running `reconnect` before `replay` can get the next page's modules a 503. The current server does not do this.
- The pushed-capture check could pass by luck if the fallback poll fired right after the external start. In these runs the start button disabled in 0.1 s each time.
- Pausing previews relies on the IntersectionObserver rooted at `#side`. If a layout change makes it report "visible" always, previews simply keep flowing, as before this fix.
- All results are host and browser tests on this machine: no hardware, and WebGL runs in software.
