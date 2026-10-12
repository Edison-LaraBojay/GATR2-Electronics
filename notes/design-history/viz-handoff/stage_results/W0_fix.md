# W0 fix stage: review findings on the perf harness

The review of W0's harness (`pi/naviGATR/tools/perf`) and baseline doc found 7 problems. All 7 were real; all are fixed, none declined. Only `tools/perf/**` and W0's own stage docs were changed; no product code. Everything here is host tooling on the dev laptop, not hardware validation.

`$S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.

## Findings

### 1. run_perf.sh measured whatever already held the port (medium): fixed
**Before.** `run_perf.sh` polled `/api/health` without checking that its own `navigatr` had bound the port. A squatter on `--port` (or a stale Chrome on `--cdp-port`) was measured and labeled as the requested build, and the run was logged as exit 0.

**Fix, in layers.**
- `run_perf.sh` refuses a browser run when `127.0.0.1:--port` or `--cdp-port` already accepts connections. It uses curl exit 7 as "free", with a 5 s limit, because Windows takes about 2 s to refuse a loopback connect.
- After the health check it requires `kill -0` on its `navigatr`, and reads the session id `navigatr` printed (`profile ... session <id>`).
- It waits for the browser's own `DevTools listening on ws://...` line.
- It passes both to `navigatr_cdp_bench` as `--expect-session` and `--expect-browser-ws`.
  - `cdp_bench` exits 3 when `/json/version` or `/api/hello` `session.id` does not match.
  - It records `server_hello_start` and `server_hello_end` (contract, session, configuration, inspection rates) and `browser_ws` in its conditions.
  - It sets `server_changed` and exits 3 if the session changed during the run.
- Each repeat deletes its own old `server_rN.json`/`browser_rN.json` first, so a failed repeat cannot leave stale results behind.
- Every failure is a `FAILED:` line in `conditions.txt`, and the script exits 1.
- The report prints each set's measured contract (server runs, and the browser runs' server) and a "server session changed" row.
- Small extras:
  - the leftover-process cleanup no longer matches its own PowerShell;
  - the DevTools wait loop no longer reads a log that the background launch has not created yet.

**Verified** (`$S/w0-results/fix-check/`):
- **Reviewer's case.** An inspect/1 `navigatr` on 18796, then `run_perf.sh --only browser --port 18796`. Result: "FAILED: browser run 1: 127.0.0.1:18796 already answers; not measuring whatever holds it", with `run_perf exit=1`.
- **`cdp_bench` against a live Chrome and that squatter.**
  - Wrong `--expect-session`: "the server on 127.0.0.1:18796 is not the one started (session 'd060848e9a784d5a', expected 'deadbeef00000000')", exit 3.
  - Wrong `--expect-browser-ws`: exit 3.
  - Matching both: exit 0, with identity recorded.
- **Normal path.** `run_perf.sh --only browser` on the baseline build and on the live build: exit 0, "browser run 1 exit 0 (server session ...)", and no leftover listeners.

### 2. The harness's own parsing delayed burst receive times (medium): fixed, baseline rerun
**Before.** Each client JSON-parsed every message between socket reads. In a burst, later messages were stamped late by the harness's parse cost (12 to 28 ms p50 per 51 KB snapshot).

**Fix.** The receive loop now only reads and keeps raw frames with their read time. It parses only the hello inline, because it needs the contract. Everything else is parsed after the run: `finish()`, then a stable sort by receive time. The diag-per-second timeline is rebuilt afterwards.
- A per-client cap (`--defer-mb`, default 256 MB) falls back to inline parsing and counts it (`harness_parse.inline_over_defer_cap`).
- Parse cost is still reported, as `harness_parse.parse_ms`, and is now off the receive path.

**Rerun of the baseline server scenarios** with the fixed harness: `$S/w0-results/baseline-fix`, 3 repeats, 05:55 to 06:01, all exit 0. The host was quieter than in the first run: 3 to 34% busy between runs.

| Stall scenario (3 s pause) | First run (inline parse) | Rerun (deferred parse) |
|---|---|---|
| time to first fresh snapshot after resume, ms | 159.6 (65.3-430.4) | 28.0 (19.4-41.9) |
| stale snapshots delivered after resume | 25 (24-31) | 24 (24-25) |
| first message after resume, lag ms | 2988.9 | 2998.1 |

The reviewer's interleaved A/B on the same tree measured 74.9 / 184.2 / 50.5 ms inline against 26.3 / 35.7 / 32.5 ms deferred. That comparison isolates the parse effect from load. W0_baseline.md sections 2, 4.1, 5 and 6 now use the rerun, and say that the first run's 160 ms was mostly harness cost.

### 3. "snapshots refused" was the wrong label for inspect/2 (medium): fixed
**Before.** On inspect/2, `snapshots_skipped` equals `states_replaced`, which counts replaced-unsent states (latest-wins working as designed) as well as refusals.

**Fix, report.** The rows now read "server: state messages not delivered, whole scenario (inspect/1: refused; inspect/2: replaced unsent or refused)", and the same for the browser. Where the fields exist there are new rows:
- `messages_replaced` (all clients and channels);
- `messages_refused`;
- `closed_stalled` / `closed_reliable_overflow` deltas.

**Fix, per-client server view** (spec 9: queue bytes, replacements, refusals, closes).
- The harness takes each client's `hello.client_id`.
- It finds that client in every `diag.inspection.clients[]` any client of the scenario received, deduped by diag `seq`.
- It emits `server_view`: `queued_bytes`/`in_flight_bytes`/`reliable_backlog` summaries, the newest per-channel `sent`/`replaced`/`refused`/`dropped`/`bytes`, and preview and reliable totals.
- The report shows "server view: `state` / `diag` / previews sent / replaced unsent / refused", reliable sent, and queued bytes. `inspect_bench` prints the same per client.

**Verified** on the live tree (`fix-check/after_quick.json`), slow client: state 33 sent / 198 replaced / 0 refused. Whole server: replaced unsent +223, refused +0. The old label would have read those replacements as refusals.

### 4. "Publish to receive" included the document build (low): fixed
**Before.** inspect/1 takes `host_ms` before building the snapshot (`inspection_service.cpp:438` into `inspection_document.cpp:991`), so the figure is document build + queue + socket + read.

**Fix.**
- The report row and the `inspect_bench` printout now say so.
- The report adds "p50 minus p50 build time of that document (queue + socket + read, estimate)". It uses `state_build_us` on inspect/2 and `snapshot_build_us` on inspect/1.
- W0_baseline.md sections 2, 4.1 and 5 are corrected, and W0_impl.md has a Corrections section.

**Corrected baseline.** Publish to receive is 2.36 / 3.58 / 14.1 ms, with a build of 1.9 ms p50. Queue + socket + read is about 0.3 ms at p50; per run it ranges from -0.7 to 2.9 ms, a noisy difference of medians. The first run's "queue plus loopback is 4.5 / 5.5 / 22 ms" statement was wrong.

### 5. The ping-offset estimate had no +-RTT/2 bound (low): fixed
**Fix.**
- New `perf_analysis.{h,cpp}` with `pingOffsetEstimates()`: the minimum-RTT pong of the last 20, per receipt. Each estimate carries its `half_width_ms` (that pong's RTT/2).
- Estimates wider than `--offset-max-half-ms` (default 10 ms) are counted (`too_wide`) and left out of `publish_to_receive_ms`. The full set stays in `publish_to_receive_all_ms`.
- On inspect/1, where pongs carry no host stamp, the estimate is `null`.
- `printClient` prints "+-x ms (half-width p50); N of M estimates wider than +-10 ms not used", or "n/a: all M estimates wider ...".
- The report has rows for the estimate, its half-width and the too-wide counts.

**Verified.**
- **Live tree, in-process, 05:51.** All 29 of the slow client's estimates were too wide, so none was reported. Before the fix it printed about 454 ms against an exact 837 ms, as the reviewer showed. That run used the pre-tidy print format; the final binary prints "n/a: all N estimates wider than +-10 ms".
- **Live tree, in-process, final binary, 06:16.** The tree had changed by then, and the slow client's lag was about 230 ms p50.
  - 31 of 123 estimates were used, with a half-width of 7.54 ms. Their true error was -7.32 ms, inside the bound.
  - Over all 123 estimates the error reached -207.6 ms, with half-widths up to 207.7 ms. Every estimate stayed inside its own bound, and the wide ones are the ones left out.
  - `offset_error_ms` (used) and `offset_error_all_ms` are both emitted in-process.
- **Caveat.** The used estimates summarize only the messages that had a narrow pong in their 20-pong window. That can be a biased subset: here, early messages before the backlog (p50 161 ms against an exact p50 of 231 ms over all messages). The printout gives "N of M estimates ... not used", so the subset is visible.
- **`--connect` against a separate live `navigatr`.** Fast client 0.42 ms +-0.13. Slow client: 45 of 94 estimates used (+-0.54 ms p50), 49 too wide.

### 6. Conditions left out the inspect/2 settings (low): fixed
**Fix.**
- SFINAE getters record `state_hz`, `diag_hz`, `reliable_kb` and `send_buffer_kb` on every run: null on a tree without them, values on inspect/2 (live: 30, 4, 256, 0).
- `snapshot_hz_applies_to: "inspect/1 only"` marks what `snapshot_hz` means.
- New `--send-buffer-kb` override.
- The server's `/api/hello` session, configuration and inspection objects are recorded in both modes, so `--connect` runs say which server and rates they measured.

### 7. idle_net100 looked like a slow-network result (low): fixed
**Fix.**
- Dropped from the default lists in `run_perf.sh` and `cdp_bench`.
- When a `_netK` scenario is requested:
  - `cdp_bench` computes the page's WebSocket receive rate (the sum of the probe's `bytes_s`) into `ws_received_bytes_s`;
  - it sets `network_throttle` and prints a warning when the page received more than 1.1 x K;
  - the report prints a note above that scenario's table. For the first baseline run it reads: "100 kB/s requested; the page received 1128 kB/s over its WebSockets (median). The throttle was NOT applied to the WebSocket: this is not a slow-network result."
- The report also has a "page WebSocket receive rate" row for every scenario.

## Tests (host only)
**Build.** Release, g++ 15.2.0 (MSYS2 ucrt64), `-Wall -Wextra`; 0 warnings in `tools/perf` on both trees.
```
cmake -S pi/naviGATR/tools/perf -B $S/w0-base-build  -DNAVIGATR_SOURCE_DIR=$S/viz-baseline-src/pi/naviGATR
cmake -S pi/naviGATR/tools/perf -B $S/w0-after-build -DNAVIGATR_SOURCE_DIR=C:/GATR2-Electronics/pi/naviGATR
cmake --build <dir> -j16
```
The live-tree build failed twice while W1 was mid-edit in `inspection_document.cpp` and `http_server.cpp` (not W0 files). After waiting it built at 06:05.

**ctest.**
- `ctest --test-dir $S/w0-base-build`: 100% passed, 9/9.
- `ctest --test-dir $S/w0-after-build`: 100% passed, 9/9.

| Test | Checks |
|---|---|
| `perf_selftest` | The estimator on synthetic pongs: exact with a symmetric pong; error within +-RTT/2 with an asymmetric one; picks the minimum RTT; the 20-pong window forgets old pongs; unstamped pongs and receipts are ignored. Summaries drop NaN/inf. |
| `perf_report_not_delivered_label`, `_refused_row`, `_server_view`, `_minus_build`, `_wide_offset`, `_net_throttle_flag` | The report's labels and values on `testdata/report_v2` (inspect/2-shaped). The label test fails if the old "snapshots refused" text appears. |
| `perf_report_v1_refused`, `_v1_minus_build` | The same on `testdata/report_v1` (inspect/1-shaped, old JSON). The ping-offset row must show "-", not numbers. |

**Negative check** (`$S/w0-fix-negtest`, 8 tests expecting wrong values): 8/8 failed as intended. That check found a real problem in the first version of the regex tests. CMake splits a regex list at `;`, so the tests whose text contains "refused; inspect/2" and "requested; the page" only checked a prefix. They now use `.` for the semicolon.

**Functional checks** (`$S/w0-results/fix-check/`): see findings 1, 3 and 5. All test ports were free again afterwards.
- **Deferred-parse fallback.** `--defer-mb 0.3` on the baseline gave 44 messages parsed inline and 28 after the run. Receive intervals stayed ordered (p50 47 ms, no negatives), so the post-run sort works.
- **Final rebuild** after the last edit (06:2x), both trees: 0 warnings, ctest 9/9 and 9/9. A 3 s baseline smoke run exited 0, with `ping_offset_estimate: null` on inspect/1.

**Baseline rerun.**
```
bash tools/perf/run_perf.sh --build $S/w0-base-build --src $S/viz-baseline-src/pi/naviGATR \
     --out $S/w0-results/baseline-fix --label "baseline (inspect/1)" --only server --repeats 3
```
Result: 3/3 server runs exit 0; the summary is in `$S/w0-results/baseline-fix/summary.md`.

## Changed files
- `pi/naviGATR/tools/perf/`:
  - `inspect_bench.cpp`, `cdp_bench.cpp`, `perf_report.cpp`, `perf_json.{h,cpp}` (summaries skip NaN/inf), `run_perf.sh`, `CMakeLists.txt`, `README.md`;
  - new: `perf_analysis.{h,cpp}`, `perf_selftest.cpp`, `testdata/report_v1/server_r1.json`, `testdata/report_v2/{server_r1,browser_r1}.json`.
- `viz-handoff/stage_results/W0_baseline.md`: revised (fix-stage note at the top, sections 1, 2, 4.1, 4.3, 5, 6 regenerated, 7, 8, 9, 10).
- `W0_impl.md`: Corrections section.
- `progress/W0.md`: FIX section.

## Contract issues
- **Lead (unchanged).** Add the `NAVIGATR_BUILD_PERF_TOOLS` option and `add_subdirectory(tools/perf)` at the end of `pi/naviGATR/CMakeLists.txt`; the exact lines are in `progress/W0.md`. Once added, the perf CTest checks run with naviGATR's tests when both are ON.
- **W1.** The harness now also reads:
  - `hello.client_id`;
  - `workers.inspection.{messages_replaced, messages_refused, closed_stalled, closed_reliable_overflow}`;
  - `diag.inspection.clients[].{id, queued_bytes, in_flight_bytes, reliable_backlog, channels}`;
  - diag `seq`.

  The list is in `progress/W0.md`. A rename breaks only the harness's extra rows, not the product.

## Open risks
- The first run and the rerun differ in host load as well as harness. The before/after comparison (stage 2) must use the fixed harness on both trees, interleaved (W0_baseline.md section 9).
- Deferred parsing adds several seconds after each client scenario on inspect/1 (400 snapshots at up to about 30 ms each). It is off the measured path, and runs take longer.
- **Preliminary observation, live tree 05:51, `send_buffer_kb` 0.** The inspect/2 stall client still got 35 stale states after a 3 s pause. They all arrived within about 15 ms, so this is kernel-buffered data, which amendment 12.2 addresses. The after run should show the effect of the new default.
- Still loopback on one Windows host. There is no Pi, Wi-Fi or SSH tunnel measurement, and CDP cannot throttle the WebSocket.

## Repository note
At 06:17 the user committed the working tree (`2d11c67 rename of folders`). It includes the tools/perf files as fixed here; `git status` was clean afterwards. W0 did not commit, push or touch `pcb/`.
