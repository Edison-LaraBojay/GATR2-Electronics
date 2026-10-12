# W0 baseline: inspection feed and viewer performance before the change (inspect/1)

**What this is.** Host measurements on the development laptop, taken with the new harness in `pi/naviGATR/tools/perf`, against the frozen pre-change copy at `$S/viz-baseline-src`. This is not the Pi and not hardware validation. No robot, Pico, Brain, camera or Wi-Fi link was involved. The data came from the synthetic demo config. The same commands will run against the final tree (section 9).

`$S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`. Raw results: `$S/w0-results/baseline/` (3 repeats: `server_rN.json`, `browser_rN.json`, `conditions.txt`, `summary.md`) and `$S/w0-results/baseline-extra/`.

**Revised in the fix stage (06:05).** A review found harness and wording problems; `W0_fix.md` has the details. The server scenarios were rerun with the fixed harness (it no longer parses between socket reads) into `$S/w0-results/baseline-fix/` (3 repeats). The server numbers in sections 4.1, 5 and 6 now come from that rerun, unless marked "first run". The browser numbers are still from the first run: the fixes do not change what the browser harness measures. Also corrected:
- "Publish to receive" includes the document build (section 2).
- `idle_net100` is not a slow-network result.

## 1. Conditions

| Item | Value |
|---|---|
| Date | 2026-09-28, 04:26 to 04:39 local (3 repeats), extra runs 04:40 to 04:45. Server rerun with the fixed harness: 05:55 to 06:01 (3 repeats). |
| Machine | Intel i9-14900HX (32 logical CPUs), 15.7 GB RAM, RTX 4060 Laptop GPU, display 165 Hz, Windows 11 Home 10.0.26200, Acer power plan |
| Other load | Not controlled. Other workers were compiling, and the user's own applications were open (VS Code, Discord, Chrome, a game client). Per-scenario host CPU is in every table: 23% to 100% of all cores in the first run. A second `navigatr` (W2's baseline demo) was running on another port. The rerun host was quieter: 3% to 34% busy between runs, and 8% to 79% during scenarios. |
| Tree | `$S/viz-baseline-src/pi/naviGATR` + `translaGATR`: inspect/1, snapshot only |
| Build | Release (`-O3`, NDEBUG), g++ 15.2.0 (MSYS2 ucrt64), `$S/w0-base-build`, standalone `tools/perf` build pointed at the baseline tree |
| Config | `config/demo/synthetic_field_demo.xml`: synthetic rig, loop 100 Hz, `snapshot_hz` 20, previews 5 Hz, quality 70, max width 640, `client_buffer_kb` 1024, `stall_close_ms` 10000, `max_clients` 4 |
| Server scenarios | In-process (System + InspectionService + loopback clients in one process); 6 s warm-up so the trail is full; 20 s per scenario; ping every 250 ms per client |
| Slow client | Token-bucket reads at 100 kB/s (plus one extra run at 500 kB/s), `SO_RCVBUF` 8 KB |
| Stall client | Reads normally, stops reading for 3 s at t = 3 s, then drains. It keeps sending pings while stalled (a network-level stall, not a frozen browser). |
| Burst client | Raises its own preview budget to 30 Hz, 1280 px, quality 90 for 5 s from t = 3 s |
| Browser | Chrome 153.0.8010.53, `--headless=new`, real GPU (WebGL renderer "ANGLE (NVIDIA GeForce RTX 4060 Laptop GPU, Direct3D11)"), viewport 1600x900, device pixel ratio 1. The rAF cadence in headless mode was about 120 Hz (median interval 7.9 ms), not the panel's 165 Hz. |
| Browser server | The baseline `navigatr` binary, same config, `--inspect-port 18765`. The browser page is the viewer embedded in that binary. |
| Browser scenarios | 8 s warm-up after the viewer reports live; 15 s each: `idle`, `orbit` (left-drag on the 3D canvas at about 60 moves per s on an ellipse), `idle_cpu4` and `orbit_cpu4` (CPU throttling 4x through `Emulation.setCPUThrottlingRate`), `idle_net100` (CDP network emulation at 100 kB/s; it did not throttle the WebSocket, see 4.3, and it is no longer a default) |
| Repeats | 3. Cells show p50 / p95 / max: the median over repeats of each run's p50 and p95, and the largest max of any run. |

## 2. Clocks: what each number is

- **Source age at publish** = document `host_ms` minus `robot.measured_at_host_ms`. Both are Pi host clock; exact to the 1 ms resolution of `host_ms` in inspect/1.
- **Publish to receive** = the harness's `HostClock::nowUs()` when the read that completed the frame returned, minus the document's `host_ms`. The harness runs in the same process, so this is the same clock: exact (plus up to 1 ms truncation of `host_ms`).
  - The server takes `host_ms` **before** it builds the document: `inspection_service.cpp:438` passes `HostClock::now()` into `snapshotDocument`, which writes it at `inspection_document.cpp:991`.
  - So this covers **document build + queue + socket + harness read**, not the queue alone. On inspect/1 the build is most of it (section 4.1).
  - The report adds "p50 minus the p50 build time" as an estimate of queue + socket + read.
  - The first version of this document said "queue, socket and harness read". That was wrong.
- **Harness parsing** (fixed in the fix stage).
  - The first harness parsed each message between socket reads. That delayed the receive times of later messages in a burst by the harness's own parse cost: 12 to 28 ms (p50) per 51 KB snapshot.
  - Clients now only read during a run and parse afterwards.
  - The server numbers here come from the rerun with the fixed harness.
- **Pose age at receipt** = the same receipt time minus `robot.measured_at_host_ms`. Exact, same clock.
- **RTT** = WebSocket control ping to pong (inspect/1 has no JSON ping). The server queues the pong in the same per-client FIFO as the snapshots, so RTT includes that queue.
- **Browser "receive to next frame"** = `performance.now()` at the start of the `onmessage` of a snapshot, to the start of the next animation frame. Browser clock only.
- **Browser "arrival lag, relative to best"** = (`performance.now()` at arrival minus the document's `host_ms`) minus the smallest such value in the window. These are different clocks, so only the relative value means anything: it is the extra delay compared with the best-delivered message in the same 15 s, an estimate. No Pi time is ever subtracted from a browser time as a latency.
- **Cross-process (`--connect`)**: RTT and relative lag only for inspect/1. For inspect/2 it also reports the section 3.4 ping-offset estimate.
  - Each estimate carries a +-RTT/2 half-width: half the RTT of the pong it used.
  - Estimates wider than +-10 ms (`--offset-max-half-ms`) are counted and left out, not reported as a number. That happens, for example, when every pong waited behind a backlog.
  - In-process runs also report the estimate's error against the true offset of 0.

## 3. Where the work happens (verified in the baseline code)

**Pi, inspection thread, every 50 ms:**
- `snapshotDocument` builds the whole document: robot, localization, the trail (241 entries here; up to 300), field objects, detection frames, sources, workers, diagnostics.
- `broadcastText` appends it to every client's `std::string` FIFO and calls `flush()` right away, on the inspection thread and under the server mutex (`http_server.cpp` `queueBytes`).
- A full FIFO refuses the new message and keeps the old ones queued (`sendFrame`). A client that falls behind is therefore always served the oldest snapshots first.
- JPEG preview encoding is on the same thread (mean encode 2 to 6 ms here).

**Pi, other threads:** the estimation worker at 100 Hz runs apart from inspection; the HTTP server thread polls every 20 ms and answers WebSocket pings into the same FIFO.

**Browser (`spectaGATR/app.js`, `field_scene.js`):**
- Every snapshot runs synchronously inside `onmessage`: `JSON.parse` of about 51 KB, then `onSnapshot`, which does the scene update, a camera panel redraw, the landmark table rebuild and the status badges (diagnostics and link panel are throttled to 250 ms).
- Rendering is `renderer.setAnimationLoop`: every display frame, whether anything changed or not.
- Every `pointermove` over the 3D view calls `getBoundingClientRect` and a raycast over the pickables.

## 4. Findings: the two complaints, separated

These are measurements on this host, with causes named only where the code shows the mechanism. Nothing here proves what happens on the user's Pi, network or laptop.

**4.1 "Robot movement appears late": a backlog problem, not a steady-state cost.**
- With a consumer that keeps up, the Pi side adds little.
  - Pose age at receipt is 26.5 / 37.4 / 49.6 ms (p50 / p95 / max; first run: 26 / 40 / 133).
  - Of that, the source age at publish is 23 / 35 / 47 ms.
  - Publish to receive is 2.4 / 3.6 / 14 ms (first run, on a busier host: 4.5 / 5.5 / 22). That figure includes building the 51 KB snapshot, which is 1.9 ms at p50 (first run: 3.1 ms).
  - Queue + loopback socket + read is therefore small. Publish to receive minus the build is about 0.3 ms at p50 for the fast client alone (first run: 1.5 ms).
  - This estimate subtracts medians taken in different scenarios. Single runs of the clients that keep up range from -0.7 to 2.9 ms (rerun) and from -0.6 to 1.9 ms (first run). It is within the noise of the method, not a precise figure.
  - The first version of this line called 4.5 / 5.5 / 22 ms "queue plus loopback socket". That was wrong: most of it was the build.
  - The browser then adds 3.4 / 8.4 / 77 ms from arrival to the next frame when idle.
- The feed is heavy: about 1.05 MB/s for one client (20 snapshots of about 51 KB). The trail is 34 KB of each 51 KB snapshot (67%), resent 20 times a second.
- A consumer slower than 1.05 MB/s gets old poses, in a fixed order. The server keeps the oldest queued data and refuses the newest: 347 refusals (346 to 347) in 20 s at 100 kB/s in the rerun, and 346 in the first run.
  - The lag climbs by about 0.95 s every second until the queue is full, then stays at queue size over read rate:

    | Read rate | Lag plateau | Reached after |
    |---|---|---|
    | 100 kB/s | about 11.8 s (rerun: 11.5 to 11.7 s) | about 13 s |
    | 500 kB/s | about 2.35 s | about 5 s |

    The queue works out to about 1.17 MB: the 1 MB per-client buffer plus socket buffers.
  - RTT on that client is 11.4 s, because the pong waits in the same FIFO.
  - The fast client next to it is barely affected (4.4 / 8.3 / 29 ms, against 4.5 / 5.5 / 22 alone).
- A consumer that pauses for 3 s receives 24 (24 to 25) stale snapshots in a burst when it resumes. The first is 2998 ms old (2984 to 3010). A browser would parse and apply every one of those stale snapshots.
- The first fresh snapshot arrives 28 ms after the resume (19 to 42).
  - The first run reported 160 ms (65 to 430). That was mostly the harness: it JSON-parsed each 51 KB snapshot (12 to 28 ms p50) between socket reads, so every later arrival in the burst was stamped late.
  - The reviewer ran an interleaved A/B on this tree: 75 / 184 / 51 ms with parsing between reads, against 26 / 36 / 33 ms with parsing deferred. The number of stale snapshots was the same either way.
  - The rerun above uses the fixed harness. The host was also quieter during the rerun, so part of the difference between the runs is load.
- A browser that cannot keep up does the same thing. At 4x CPU throttling the viewer handled 6.2 of 20 snapshots per s. Arrivals lagged the best by 4.1 s (p50), 6.7 s (p95) and up to 19 s. The server refused 208 snapshots in 15 s. The page showed poses seconds old while "live".
- Not measured: the user's real link (Pi to laptop over Wi-Fi and an SSH tunnel) and the real laptop. Whether they sustain 1.05 MB/s is the open question. `navigatr_inspect_bench --connect` through the SSH forward can measure that link from the laptop (section 9).

**4.2 "Stutter when interacting with the 3D view": main-thread saturation in the browser.**
- **Idle**, on this fast desktop GPU:
  - The main thread is 82% busy (CDP TaskDuration), and JS alone is about 62% of wall time.
  - The page's rAF callbacks cost 5.2 / 10.9 / 22 ms per frame, at about 120 frames per s, even with nothing changing.
  - Snapshot handling costs 60 ms per s, in synchronous chunks of 2.4 / 5.5 / 11 ms.
  - The DOM takes 960 mutation records per s, and CDP counts 7830 DOM nodes (524 live), so detached nodes pile up for the GC.
  - 16 frames per 15 s take longer than 33 ms.
- **Orbit drag:**
  - Layouts rise from 26 to 163 per s, consistent with `getBoundingClientRect` plus a raycast on every pointer move.
  - Frames over 33 ms go from 16 to 40 per 15 s, and the frame interval p95 from 23 to 31.5 ms.
  - Input-event acknowledgement is 11 / 34 / 105 ms.
  - The rAF callbacks peak at 41 ms, and occasional long tasks of 51 to 58 ms appear.
- **At 4x CPU throttling** (a slower laptop, roughly): the viewer collapses to 1 to 3 frames per s. Long tasks run 121 / 558 / 1616 ms, the snapshot handler takes 24 / 91 / 295 ms, and orbit input acknowledgement takes 395 / 1346 / 1573 ms.
- The per-function split (JSON.parse vs DOM rebuild vs render) was not profiled. The hotspot list in `inventory_viewer.md` section 8 matches these symptoms but is attributed from the code, not from a profile.

**4.3 Other observations.**
- **Build cost.** Building the snapshot costs 1.9 / 3.5 / 38 ms in the rerun (the first run, under heavier load: 3.1 / 4.2 / 16.8 ms). Without the trail it costs 0.31 / 0.70 / 3.1 ms (first run: 0.54 / 0.96 / 1.4 ms), so the trail is about 80 to 85% of the build time.
- **Publication rate.** Localization publishes at about 92 Hz (target 100 under this machine load). A 20 Hz feed delivers about 22% of the publications, which is fine for a display.
- **Windows timer granularity.** The 20 Hz snapshot cadence shows as alternating 47 / 62 ms intervals on this host (Windows timers run at 15.6 ms). The Pi's Linux timers should give 50 ms; that was not measured.
- **Preview burst.** No measurable effect here. The synthetic camera frames are small (about 8 KB at 1280 px and quality 90) and publish at about 10 Hz, so 5 s at a 30 Hz budget added little. A real camera's JPEGs would be larger; not measured.
- **Estimation worker.** It ran at 92 to 96 Hz against a 100 Hz target. Its overrun counters (3 to 69 since process start) track the machine load. These runs cannot attribute any of that to the inspection feed.
- **Viewer page load fails intermittently on the baseline server.** In a clean repeat of 6 fresh headless page loads (04:57), 2 never went live. In both, one ES module (`transforms.js`) got HTTP 503 "too many clients", and the viewer stayed at "connecting" with no WebSocket and no error shown on the page. Earlier ad-hoc loads (04:48 to 04:52) failed 3 of 7 with the same symptom, before statuses were recorded.
  - The mechanism: the inspect/1 server counts plain HTTP connections against `max_clients` (4), and Chrome fetches the module graph over more parallel connections than that.
  - The 3 measured baseline runs all loaded on the first try (`viewer_live: true`).
  - `navigatr_cdp_bench` now reloads a page that is not live within 15 s, up to 3 times. It records `load_attempts` and each failed load's resource statuses, and the report shows the attempts.
  - W1's in-progress server counts feed clients only against `max_clients`. The after run will show whether that fixed it.
- **CDP network throttling.** `idle_net100` did not throttle the WebSocket, so it is **not a slow-network result**.
  - 21.7 snapshots per s, about 1.1 MB/s, still arrived.
  - That scenario also ran right after the CPU-throttled ones, with the host 57 to 99% busy, so its frame numbers are confounded and are not used.
  - Slow networks are covered by the C++ slow client instead.
  - Since the fix stage, `idle_net100` is not in the default scenario lists. When a `_netK` scenario is requested, `navigatr_cdp_bench` and the report compare the page's received rate with K and flag a throttle that did not apply (see its table in section 6).

## 5. Headline table

p50 / p95 / max; single values are the median (min to max) over 3 repeats. Server rows: the rerun with the fixed harness (`baseline-fix`, 05:55 to 06:01, quieter host), with the first run (04:26 to 04:39) in brackets where it differs. Browser rows: first run only.

| Quantity | Baseline (inspect/1) |
|---|---|
| Feed bytes per client | 1.05 MB/s (20 snapshots/s of 51.6 KB) |
| Snapshot build, us (with / without trail) | 1932 / 3456 / 37980; 311 / 703 / 3073 (first run: 3086 / 4189 / 16803; 536 / 959 / 1407) |
| Source age at publish, ms | 23 / 35 / 47 (first run: 21 / 35 / 129) |
| Publish to receive, fast client, ms (exact; includes the snapshot build) | 2.36 / 3.58 / 14.1 (first run: 4.45 / 5.50 / 22.1) |
| Of which queue + socket + read, p50 minus p50 build (estimate, noisy) | 0.33 ms (first run: 1.49) |
| Pose age at receipt, fast client, ms (exact) | 26.5 / 37.4 / 49.6 (first run: 26.2 / 39.6 / 133) |
| RTT, fast client, ms | 0.60 / 0.78 / 0.88 (first run: 0.76 / 0.95 / 2.30) |
| Publish to receive, slow client 100 kB/s, ms | 8834 / 11662 / 11984, plateau about 11.5 to 11.7 s (first run: 8675 / 11860 / 11979) |
| Publish to receive, slow client 500 kB/s, ms (1 run, first harness) | 2296 / 2359 / 2377 (plateau about 2.35 s) |
| Snapshots refused by the server, slow scenario (20 s) | 347 (first run: 346) |
| After a 3 s pause: stale snapshots delivered / first one's age / first fresh one after | 24 / 2998 ms / 28 ms (19 to 42). The first run's "160 ms" to the first fresh snapshot was mostly harness parsing. |
| Browser idle: frames per s, frame interval ms | 96.9; 7.9 / 23.0 / 79.0 |
| Browser idle: main thread busy, onmessage ms per s | 82%; 59.9 |
| Browser orbit: frames over 33 ms per 15 s, input ack ms | 40; 11.3 / 33.9 / 104.5 |
| Browser 4x CPU: frames per s, arrival lag vs best, ms | 2.9; 4126 / 6741 / 19242 |

## 6. Full generated tables

Generated by the fixed `navigatr_perf_report` (fix stage). Server tables: `$S/w0-results/baseline-fix` (the rerun with the fixed harness). Browser tables: `$S/w0-results/baseline` (the first run). Rows marked inspect/2 or "server view" show "-" on this inspect/1 tree.

Cells: p50 / p95 / max, where p50 and p95 are medians over repeats and max is the largest seen in any repeat. Single values: median (min-max) over repeats.

#### Server: snapshot/state build (in-process, direct calls)

| metric (p50 / p95 / max) | baseline (inspect/1) |
|---|---|
| full snapshot build, us | 1932 / 3456 / 37980 |
| full snapshot size, bytes | 51337 / 52777 / 54148 |
| snapshot without trail, build us | 311 / 703 / 3073 |
| snapshot without trail, bytes | 17084 / 18233 / 19666 |
| inspect/2 state build, us | - |
| inspect/2 state size, bytes | - |
| inspect/2 history build, us | - |
| inspect/2 history size, bytes | - |
| hello size, bytes | 23159 (23159-23159) |
| trail entries in snapshot | 243 (232-244) |
|   section `trail`, bytes | 34346 (32802-34498) |
|   section `robot`, bytes | 830 (830-832) |
|   section `localization`, bytes | 845 (845-845) |
|   section `field_objects`, bytes | 4109 (4103-4114) |
|   section `detection_frames`, bytes | 3845 (3199-3860) |
|   section `brain_link`, bytes | 4 (4-4) |
|   section `pico`, bytes | 4 (4-4) |
|   section `events`, bytes | 83 (83-83) |
|   section `sources`, bytes | 4508 (4508-4508) |
|   section `workers`, bytes | 712 (711-714) |
|   section `diagnostics`, bytes | 1400 (1400-1400) |

#### Server feed: fast client alone (`fast` scenario)

| metric | baseline (inspect/1) |
|---|---|
| state/snapshot messages per s | 20.0 (20.0-20.0) |
| bytes read per s | 1050080 (1033030-1050640) |
| distinct publications received | 400 (399-400) |
| localization publication rate, Hz | 95.5 (91.7-95.9) |
| message size, bytes (p50 / p95 / max) | 51613 / 53217 / 54390 |
| receive interval, ms | 46.9 / 62.9 / 80.2 |
| source age at publish, ms | 23.0 / 35.0 / 47.0 |
| publish to receive: build + queue + socket + read, ms (exact clock) | 2.36 / 3.58 / 14.09 |
|   p50 minus p50 build time of that document, ms (queue + socket + read, estimate) | 0.33 (-0.25-0.89) |
| pose age at receipt, ms (exact) | 26.5 / 37.4 / 49.6 |
| messages older than fresh_ms at receipt | 0 (0-0) |
| RTT, ms | 0.60 / 0.78 / 0.88 |
| ping-offset estimate of publish to receive, ms (+- below) | - |
|   its +- half-width (RTT/2 of the pong used), ms | - |
|   estimates too wide to use / all | - |
| closed by server (1 = yes) | 0 (0-0) |
| server view: `state` sent / replaced unsent / refused | - |
| server view: `diag` sent / replaced unsent / refused | - |
| server view: previews sent / replaced unsent / refused | - |
| server view: reliable messages sent | - |
| server view: queued bytes, per diag | - |
| publish to receive, max per 1 s of the run, ms | 4.2, 3.6, 3.2, 3.8, 3.5, 3.2, 3.2, 4.0, 3.4, 3.2, 3.5, 3.5, 3.5, 4.5, 3.5, 3.7, 3.8, 4.3, 3.5, 4.1, - |
| host CPU busy during scenario, % of all cores | 21 (8-48) |
|   of which other processes, % | 20 (7-47) |
| estimation worker rate after scenario, Hz | 94.4 (85.1-97.8) |
| estimation worker max cycle since start, ms | 7.4 (6.1-15.9) |
| estimation worker overruns since start | 0 (0-9) |
| server: state messages not delivered, whole scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 0 (0-0) |
| server (inspect/2): messages replaced unsent, all clients and channels | - |
| server (inspect/2): messages refused, all clients and channels | - |
| server (inspect/2): clients closed stalled / reliable overflow | - |

#### Server feed: fast client, next to a slow one (`slow` scenario)

| metric | baseline (inspect/1) |
|---|---|
| state/snapshot messages per s | 20.0 (20.0-20.0) |
| bytes read per s | 1053010 (1047420-1055240) |
| distinct publications received | 400 (400-400) |
| localization publication rate, Hz | 93.3 (91.8-94.5) |
| message size, bytes (p50 / p95 / max) | 51704 / 53024 / 54669 |
| receive interval, ms | 46.9 / 62.4 / 72.2 |
| source age at publish, ms | 23.0 / 35.0 / 49.0 |
| publish to receive: build + queue + socket + read, ms (exact clock) | 2.94 / 4.60 / 6.64 |
|   p50 minus p50 build time of that document, ms (queue + socket + read, estimate) | 0.32 (-0.74-2.28) |
| pose age at receipt, ms (exact) | 26.3 / 38.0 / 52.7 |
| messages older than fresh_ms at receipt | 0 (0-0) |
| RTT, ms | 0.60 / 0.80 / 1.18 |
| ping-offset estimate of publish to receive, ms (+- below) | - |
|   its +- half-width (RTT/2 of the pong used), ms | - |
|   estimates too wide to use / all | - |
| closed by server (1 = yes) | 0 (0-0) |
| server view: `state` sent / replaced unsent / refused | - |
| server view: `diag` sent / replaced unsent / refused | - |
| server view: previews sent / replaced unsent / refused | - |
| server view: reliable messages sent | - |
| server view: queued bytes, per diag | - |
| publish to receive, max per 1 s of the run, ms | 3.7, 3.9, 4.2, 4.1, 4.5, 4.2, 4.3, 4.5, 4.0, 4.3, 4.2, 3.4, 3.4, 5.0, 4.2, 5.0, 4.0, 3.7, 3.4, 3.5, - |
| host CPU busy during scenario, % of all cores | 33 (10-44) |
|   of which other processes, % | 32 (9-43) |
| estimation worker rate after scenario, Hz | 97.5 (97.4-97.5) |
| estimation worker max cycle since start, ms | 7.9 (7.4-15.9) |
| estimation worker overruns since start | 0 (0-9) |

#### Server feed: slow client (throttled reads) (`slow` scenario)

| metric | baseline (inspect/1) |
|---|---|
| state/snapshot messages per s | 1.7 (1.7-1.7) |
| bytes read per s | 99923 (99846-99924) |
| distinct publications received | 35 (35-35) |
| localization publication rate, Hz | 94.4 (91.4-95.6) |
| message size, bytes (p50 / p95 / max) | 51824 / 52459 / 53064 |
| receive interval, ms | 527.1 / 644.8 / 677.8 |
| source age at publish, ms | 24.0 / 35.0 / 45.0 |
| publish to receive: build + queue + socket + read, ms (exact clock) | 8834.23 / 11661.80 / 11984.20 |
|   p50 minus p50 build time of that document, ms (queue + socket + read, estimate) | 8832.30 (8804.56-8846.56) |
| pose age at receipt, ms (exact) | 8851.2 / 11680.8 / 11991.2 |
| messages older than fresh_ms at receipt | 35 (35-35) |
| RTT, ms | 11255.40 / 11602.40 / 11952.60 |
| ping-offset estimate of publish to receive, ms (+- below) | - |
|   its +- half-width (RTT/2 of the pong used), ms | - |
|   estimates too wide to use / all | - |
| closed by server (1 = yes) | 0 (0-0) |
| server view: `state` sent / replaced unsent / refused | - |
| server view: `diag` sent / replaced unsent / refused | - |
| server view: previews sent / replaced unsent / refused | - |
| server view: reliable messages sent | - |
| server view: queued bytes, per diag | - |
| publish to receive, max per 1 s of the run, ms | 741, 1689, 2635, 3599, 4554, 5052, 5985, 6949, 7887, 8834, 9750, 10712, 11616, 11650, 11626, 11546, 11603, 11526, 11496, 11480, - |
| server: state messages not delivered, whole scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 347 (346-347) |
| server (inspect/2): messages replaced unsent, all clients and channels | - |
| server (inspect/2): messages refused, all clients and channels | - |
| server (inspect/2): clients closed stalled / reliable overflow | - |

#### Server feed: observer, next to a stalling client (`stall` scenario)

| metric | baseline (inspect/1) |
|---|---|
| state/snapshot messages per s | 20.0 (20.0-20.0) |
| bytes read per s | 1049030 (1047610-1055520) |
| distinct publications received | 400 (400-401) |
| localization publication rate, Hz | 94.5 (92.1-96.9) |
| message size, bytes (p50 / p95 / max) | 51624 / 53069 / 54458 |
| receive interval, ms | 46.9 / 62.5 / 78.4 |
| source age at publish, ms | 23.0 / 34.0 / 48.0 |
| publish to receive: build + queue + socket + read, ms (exact clock) | 2.73 / 5.22 / 12.62 |
|   p50 minus p50 build time of that document, ms (queue + socket + read, estimate) | 0.63 (0.38-1.25) |
| pose age at receipt, ms (exact) | 26.6 / 37.7 / 52.2 |
| messages older than fresh_ms at receipt | 0 (0-0) |
| RTT, ms | 0.58 / 0.91 / 1.28 |
| ping-offset estimate of publish to receive, ms (+- below) | - |
|   its +- half-width (RTT/2 of the pong used), ms | - |
|   estimates too wide to use / all | - |
| closed by server (1 = yes) | 0 (0-0) |
| server view: `state` sent / replaced unsent / refused | - |
| server view: `diag` sent / replaced unsent / refused | - |
| server view: previews sent / replaced unsent / refused | - |
| server view: reliable messages sent | - |
| server view: queued bytes, per diag | - |
| publish to receive, max per 1 s of the run, ms | 3.4, 3.5, 3.4, 3.7, 3.7, 4.3, 5.0, 4.1, 6.2, 4.4, 4.6, 4.0, 2.9, 6.1, 3.6, 5.3, 7.3, 5.7, 6.1, 5.4, 3.2 |
| host CPU busy during scenario, % of all cores | 31 (10-79) |
|   of which other processes, % | 29 (9-75) |
| estimation worker rate after scenario, Hz | 94.2 (89.3-97.7) |
| estimation worker max cycle since start, ms | 8.4 (7.4-16.4) |
| estimation worker overruns since start | 0 (0-13) |
| server: state messages not delivered, whole scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 36 (36-36) |
| server (inspect/2): messages replaced unsent, all clients and channels | - |
| server (inspect/2): messages refused, all clients and channels | - |
| server (inspect/2): clients closed stalled / reliable overflow | - |

#### Server feed: stalling client (`stall` scenario)

| metric | baseline (inspect/1) |
|---|---|
| state/snapshot messages per s | 18.2 (18.2-18.2) |
| bytes read per s | 957334 (953919-960462) |
| distinct publications received | 364 (364-365) |
| localization publication rate, Hz | 94.5 (92.1-96.9) |
| message size, bytes (p50 / p95 / max) | 51654 / 53024 / 54458 |
| receive interval, ms | 46.8 / 62.5 / 3067.4 |
| source age at publish, ms | 23.0 / 34.0 / 48.0 |
| publish to receive: build + queue + socket + read, ms (exact clock) | 2.99 / 2105.69 / 3009.52 |
|   p50 minus p50 build time of that document, ms (queue + socket + read, estimate) | 0.99 (0.52-1.52) |
| pose age at receipt, ms (exact) | 27.6 / 2129.7 / 3033.5 |
| messages older than fresh_ms at receipt | 24 (24-25) |
| RTT, ms | 0.64 / 2145.84 / 2953.19 |
| ping-offset estimate of publish to receive, ms (+- below) | - |
|   its +- half-width (RTT/2 of the pong used), ms | - |
|   estimates too wide to use / all | - |
| closed by server (1 = yes) | 0 (0-0) |
| server view: `state` sent / replaced unsent / refused | - |
| server view: `diag` sent / replaced unsent / refused | - |
| server view: previews sent / replaced unsent / refused | - |
| server view: reliable messages sent | - |
| server view: queued bytes, per diag | - |
| first message after resume, lag ms | 2998.1 (2984.0-3009.5) |
| stale messages delivered after resume | 24 (24-25) |
| time to first fresh message after resume, ms | 28.0 (19.4-41.9) |
| publish to receive, max per 1 s of the run, ms | 3.7, 3.3, 3.3, -, -, -, 2998, 4.0, 5.4, 4.2, 4.4, 3.9, 2.9, 6.3, 3.4, 5.4, 7.5, 5.5, 6.3, 5.2, 3.1 |
| server: state messages not delivered, whole scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 36 (36-36) |
| server (inspect/2): messages replaced unsent, all clients and channels | - |
| server (inspect/2): messages refused, all clients and channels | - |
| server (inspect/2): clients closed stalled / reliable overflow | - |

#### Server feed: observer, next to a preview burst (`burst` scenario)

| metric | baseline (inspect/1) |
|---|---|
| state/snapshot messages per s | 20.0 (20.0-20.0) |
| bytes read per s | 1056580 (1041020-1057790) |
| distinct publications received | 400 (400-400) |
| localization publication rate, Hz | 92.3 (90.0-94.7) |
| message size, bytes (p50 / p95 / max) | 51915 / 53240 / 54376 |
| receive interval, ms | 47.1 / 62.8 / 77.2 |
| source age at publish, ms | 24.0 / 35.0 / 50.0 |
| publish to receive: build + queue + socket + read, ms (exact clock) | 3.97 / 5.12 / 20.62 |
|   p50 minus p50 build time of that document, ms (queue + socket + read, estimate) | 0.81 (0.29-2.66) |
| pose age at receipt, ms (exact) | 27.6 / 38.8 / 54.0 |
| messages older than fresh_ms at receipt | 0 (0-0) |
| RTT, ms | 0.66 / 0.89 / 3.69 |
| ping-offset estimate of publish to receive, ms (+- below) | - |
|   its +- half-width (RTT/2 of the pong used), ms | - |
|   estimates too wide to use / all | - |
| closed by server (1 = yes) | 0 (0-0) |
| server view: `state` sent / replaced unsent / refused | - |
| server view: `diag` sent / replaced unsent / refused | - |
| server view: previews sent / replaced unsent / refused | - |
| server view: reliable messages sent | - |
| server view: queued bytes, per diag | - |
| publish to receive, max per 1 s of the run, ms | 3.6, 3.3, 3.3, 4.3, 4.7, 5.3, 5.7, 4.7, 5.1, 5.0, 5.7, 5.4, 5.0, 5.2, 5.5, 5.3, 5.6, 4.6, 4.0, 4.2, 2.7 |
| host CPU busy during scenario, % of all cores | 56 (23-58) |
|   of which other processes, % | 53 (22-56) |
| estimation worker rate after scenario, Hz | 96.7 (93.7-97.6) |
| estimation worker max cycle since start, ms | 15.6 (7.9-16.4) |
| estimation worker overruns since start | 3 (0-13) |
| server: state messages not delivered, whole scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 0 (0-0) |
| server (inspect/2): messages replaced unsent, all clients and channels | - |
| server (inspect/2): messages refused, all clients and channels | - |
| server (inspect/2): clients closed stalled / reliable overflow | - |

#### Server feed: preview burst client (`burst` scenario)

| metric | baseline (inspect/1) |
|---|---|
| state/snapshot messages per s | 20.0 (20.0-20.0) |
| bytes read per s | 1080850 (1066350-1081500) |
| distinct publications received | 400 (400-400) |
| localization publication rate, Hz | 92.3 (90.0-94.7) |
| message size, bytes (p50 / p95 / max) | 51915 / 53240 / 54376 |
| receive interval, ms | 47.1 / 62.8 / 77.1 |
| source age at publish, ms | 24.0 / 35.0 / 50.0 |
| publish to receive: build + queue + socket + read, ms (exact clock) | 4.19 / 5.36 / 21.00 |
|   p50 minus p50 build time of that document, ms (queue + socket + read, estimate) | 0.74 (0.51-2.93) |
| pose age at receipt, ms (exact) | 27.6 / 39.2 / 54.2 |
| messages older than fresh_ms at receipt | 0 (0-0) |
| RTT, ms | 0.64 / 0.85 / 3.16 |
| ping-offset estimate of publish to receive, ms (+- below) | - |
|   its +- half-width (RTT/2 of the pong used), ms | - |
|   estimates too wide to use / all | - |
| closed by server (1 = yes) | 0 (0-0) |
| server view: `state` sent / replaced unsent / refused | - |
| server view: `diag` sent / replaced unsent / refused | - |
| server view: previews sent / replaced unsent / refused | - |
| server view: reliable messages sent | - |
| server view: queued bytes, per diag | - |
| publish to receive before burst, ms | 2.30 / 3.19 / 8.03 |
| publish to receive during burst, ms | 3.93 / 5.88 / 16.71 |
| preview messages per s | 5.9 (5.9-5.9) |
| publish to receive, max per 1 s of the run, ms | 3.6, 3.3, 3.4, 7.1, 5.9, 5.5, 5.9, 7.7, 5.3, 5.0, 5.9, 5.6, 5.3, 5.4, 5.3, 5.0, 5.4, 4.7, 4.3, 4.4, 2.9 |
| server: state messages not delivered, whole scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 0 (0-0) |
| server (inspect/2): messages replaced unsent, all clients and channels | - |
| server (inspect/2): messages refused, all clients and channels | - |
| server (inspect/2): clients closed stalled / reliable overflow | - |

#### Browser: page load

| metric | baseline (inspect/1) |
|---|---|
| page loads until the viewer was live (1 = first try) | - |
| viewer live (1 = yes) | 1 (1-1) |
| server session changed during the run (1 = yes, invalid) | - |

#### Browser: `idle`

| metric | baseline (inspect/1) |
|---|---|
| host CPU busy during scenario, % of all cores | 23 (23-23) |
| frames per s (rAF) | 96.9 (96.2-97.8) |
| frame interval, ms (p50 / p95 / max) | 7.9 / 23.0 / 79.0 |
| frames over 33 ms | 16 (8-22) |
| page rAF callback time per frame, ms | 5.2 / 10.9 / 22.2 |
| long tasks (> 50 ms) | 0 (0-1) |
| long task duration, ms | 51.0 / 51.0 / 51.0 |
| main thread busy, % (CDP TaskDuration) | 82 (82-83) |
| WebSocket onmessage time, ms per s | 59.9 (59.1-61.2) |
| `snapshot` messages per s | 20.0 (20.0-20.1) |
| `snapshot` onmessage handler, ms | 2.40 / 5.50 / 11.20 |
| `state` messages per s | - |
| `state` onmessage handler, ms | - |
| `diag` messages per s | - |
| `diag` onmessage handler, ms | - |
| `binary` messages per s | 4.8 (4.7-4.9) |
| `binary` onmessage handler, ms | 0.80 / 1.50 / 4.50 |
| receive to next frame, ms | 3.4 / 8.4 / 76.5 |
| arrival lag vs host_ms, relative to best, ms (estimate) | 4.2 / 19.6 / 50.2 |
| input event ack, ms (orbit only) | - |
| layouts per s | 26.2 (26.1-26.6) |
| DOM mutation records per s | 960.1 (960.0-960.6) |
| DOM nodes incl. detached (CDP Nodes, end) | 7830 (3239-8246) |
| JS heap used, bytes (sampled 2 Hz) | 18000899 / 24565095 / 27626817 |
| page WebSocket receive rate, kB/s | 1042.3 (1041.5-1043.1) |
| server: state messages not delivered during scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 0 (0-0) |
| server (inspect/2): messages refused during scenario | - |

#### Browser: `orbit`

| metric | baseline (inspect/1) |
|---|---|
| host CPU busy during scenario, % of all cores | 26 (24-30) |
| frames per s (rAF) | 71.3 (66.9-86.3) |
| frame interval, ms (p50 / p95 / max) | 8.1 / 31.5 / 78.8 |
| frames over 33 ms | 40 (32-51) |
| page rAF callback time per frame, ms | 5.6 / 11.7 / 40.6 |
| long tasks (> 50 ms) | 1 (0-3) |
| long task duration, ms | 51.5 / 55.0 / 58.0 |
| main thread busy, % (CDP TaskDuration) | 85 (84-90) |
| WebSocket onmessage time, ms per s | 70.6 (64.1-74.6) |
| `snapshot` messages per s | 20.0 (20.0-20.0) |
| `snapshot` onmessage handler, ms | 2.80 / 6.70 / 12.90 |
| `state` messages per s | - |
| `state` onmessage handler, ms | - |
| `diag` messages per s | - |
| `diag` onmessage handler, ms | - |
| `binary` messages per s | 4.8 (4.8-4.9) |
| `binary` onmessage handler, ms | 0.90 / 1.50 / 7.70 |
| receive to next frame, ms | 7.7 / 26.4 / 71.2 |
| arrival lag vs host_ms, relative to best, ms (estimate) | 5.4 / 30.6 / 68.8 |
| input event ack, ms (orbit only) | 11.3 / 33.9 / 104.5 |
| layouts per s | 162.7 (160.6-173.9) |
| DOM mutation records per s | 1103.7 (1102.7-1115.9) |
| DOM nodes incl. detached (CDP Nodes, end) | 19323 (5354-20050) |
| JS heap used, bytes (sampled 2 Hz) | 23516992 / 40111410 / 41839317 |
| page WebSocket receive rate, kB/s | 1054.3 (1053.4-1060.2) |
| server: state messages not delivered during scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 0 (0-0) |
| server (inspect/2): messages refused during scenario | - |

#### Browser: `idle_cpu4`

| metric | baseline (inspect/1) |
|---|---|
| host CPU busy during scenario, % of all cores | 30 (24-48) |
| frames per s (rAF) | 2.9 (1.1-3.3) |
| frame interval, ms (p50 / p95 / max) | 283.3 / 758.1 / 1762.6 |
| frames over 33 ms | 45 (25-51) |
| page rAF callback time per frame, ms | 86.5 / 364.3 / 834.7 |
| long tasks (> 50 ms) | 66 (40-70) |
| long task duration, ms | 121.0 / 558.0 / 1616.0 |
| main thread busy, % (CDP TaskDuration) | 92 (89-93) |
| WebSocket onmessage time, ms per s | 212.9 (137.4-219.1) |
| `snapshot` messages per s | 6.2 (1.2-7.2) |
| `snapshot` onmessage handler, ms | 24.40 / 91.00 / 294.90 |
| `state` messages per s | - |
| `state` onmessage handler, ms | - |
| `diag` messages per s | - |
| `diag` onmessage handler, ms | - |
| `binary` messages per s | 2.5 (0.3-2.8) |
| `binary` onmessage handler, ms | 3.00 / 12.20 / 17.20 |
| receive to next frame, ms | 108.6 / 336.1 / 801.8 |
| arrival lag vs host_ms, relative to best, ms (estimate) | 4126.2 / 6741.1 / 19242.0 |
| input event ack, ms (orbit only) | - |
| layouts per s | 2.8 (1.0-3.2) |
| DOM mutation records per s | 386.6 (111.9-426.2) |
| DOM nodes incl. detached (CDP Nodes, end) | 12881 (12116-28125) |
| JS heap used, bytes (sampled 2 Hz) | 18415797 / 41777042 / 45534540 |
| page WebSocket receive rate, kB/s | 330.4 (64.0-378.2) |
| server: state messages not delivered during scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 208 (176-490) |
| server (inspect/2): messages refused during scenario | - |

#### Browser: `orbit_cpu4`

| metric | baseline (inspect/1) |
|---|---|
| host CPU busy during scenario, % of all cores | 54 (37-81) |
| frames per s (rAF) | 1.2 (0.9-2.1) |
| frame interval, ms (p50 / p95 / max) | 738.9 / 1314.1 / 1665.4 |
| frames over 33 ms | 22 (17-43) |
| page rAF callback time per frame, ms | 386.6 / 624.8 / 784.7 |
| long tasks (> 50 ms) | 36 (36-67) |
| long task duration, ms | 241.0 / 1080.0 / 1402.0 |
| main thread busy, % (CDP TaskDuration) | 89 (89-113) |
| WebSocket onmessage time, ms per s | 125.9 (111.3-178.0) |
| `snapshot` messages per s | 1.2 (0.6-3.2) |
| `snapshot` onmessage handler, ms | 81.40 / 204.90 / 324.00 |
| `state` messages per s | - |
| `state` onmessage handler, ms | - |
| `diag` messages per s | - |
| `diag` onmessage handler, ms | - |
| `binary` messages per s | 0.3 (0.2-1.3) |
| `binary` onmessage handler, ms | 7.40 / 13.90 / 16.60 |
| receive to next frame, ms | 231.6 / 622.8 / 962.6 |
| arrival lag vs host_ms, relative to best, ms (estimate) | 6969.4 / 15100.2 / 17981.3 |
| input event ack, ms (orbit only) | 394.5 / 1345.5 / 1572.8 |
| layouts per s | 2.9 (2.3-6.9) |
| DOM mutation records per s | 105.7 (61.1-235.8) |
| DOM nodes incl. detached (CDP Nodes, end) | 13733 (12026-17209) |
| JS heap used, bytes (sampled 2 Hz) | 21699960 / 28853061 / 44887515 |
| page WebSocket receive rate, kB/s | 61.4 (31.8-169.7) |
| server: state messages not delivered during scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 470 (346-563) |
| server (inspect/2): messages refused during scenario | - |

#### Browser: `idle_net100`

Network emulation (baseline (inspect/1)): 100 kB/s requested; the page received 1128 kB/s over its WebSockets (median). The throttle was NOT applied to the WebSocket: this is not a slow-network result.

| metric | baseline (inspect/1) |
|---|---|
| host CPU busy during scenario, % of all cores | 75 (57-99) |
| frames per s (rAF) | 20.9 (11.9-23.7) |
| frame interval, ms (p50 / p95 / max) | 43.4 / 125.3 / 218.3 |
| frames over 33 ms | 174 (167-178) |
| page rAF callback time per frame, ms | 15.2 / 19.9 / 68.6 |
| long tasks (> 50 ms) | 17 (14-38) |
| long task duration, ms | 65.0 / 113.0 / 120.0 |
| main thread busy, % (CDP TaskDuration) | 69 (63-73) |
| WebSocket onmessage time, ms per s | 141.2 (139.2-174.9) |
| `snapshot` messages per s | 21.7 (19.9-22.0) |
| `snapshot` onmessage handler, ms | 6.70 / 9.80 / 42.10 |
| `state` messages per s | - |
| `state` onmessage handler, ms | - |
| `diag` messages per s | - |
| `diag` onmessage handler, ms | - |
| `binary` messages per s | 5.2 (4.8-5.3) |
| `binary` onmessage handler, ms | 1.60 / 2.10 / 7.40 |
| receive to next frame, ms | 29.2 / 122.7 / 179.6 |
| arrival lag vs host_ms, relative to best, ms (estimate) | 56.1 / 1053.4 / 1692.4 |
| input event ack, ms (orbit only) | - |
| layouts per s | 13.9 (11.9-17.0) |
| DOM mutation records per s | 1021.3 (954.4-1028.2) |
| DOM nodes incl. detached (CDP Nodes, end) | 10197 (5050-11858) |
| JS heap used, bytes (sampled 2 Hz) | 19833247 / 34353643 / 39762584 |
| page WebSocket receive rate, kB/s | 1127.6 (1054.3-1146.8) |
| server: state messages not delivered during scenario (inspect/1: refused; inspect/2: replaced unsent or refused) | 0 (0-1) |
| server (inspect/2): messages refused during scenario | - |


## 7. Extra single runs

**Slow client at 500 kB/s** (half the feed rate; `$S/w0-results/baseline-extra/slow500k.json`; host 21% busy):
- The slow client's publish-to-receive, per second of the run: 544, 1057, 1601, 2090, 2354, then flat at 2336 to 2377 ms.
- Overall: 2296 / 2359 / 2377 ms, with 9.3 snapshots per s received and 191 refused.
- The fast neighbour stayed at 2.6 / 7.5 / 18 ms.

**Cross-process check of `--connect`** (a separate baseline `navigatr` on port 18766; `$S/w0-results/baseline-extra/connect.json`):
- Fast client: relative lag 2.0 / 3.6 / 9.3 ms, RTT 0.63 / 0.83 / 0.92 ms.
- Stalling client: relative lag up to 2954 ms, and RTT up to 2970 ms, because the pong is queued behind the backlog.

This shows the mode works for a server this harness does not own, such as the Pi through an SSH forward. It is not a Pi measurement.

Both extra runs used the first harness, which parsed between reads. The lags here are seconds long, far above that parse cost, so the conclusions stand, but the millisecond-level figures carry that bias.

## 8. Limits of these numbers

- **Load.** The host was shared and busy. Absolute CPU figures, the build microseconds and the frame times move with that load. The per-scenario host CPU is in the tables. The before/after comparison should use the interleaved procedure in section 9, so both trees see the same conditions.
- **Browser.** Headless Chrome on a desktop-class GPU at about 120 Hz. A 60 Hz laptop does half the render calls per second, and a weaker GPU or CPU does each one slower. CPU throttling at 4x only approximates a slower machine.
- **Network.** Loopback only. There is no Wi-Fi, SSH tunnel or Pi network stack. On the Pi, Linux socket buffers and Wi-Fi rates will differ, and TCP send buffers autotune larger on Linux.
- **Synthetic data.** The config is synthetic: no Brain link or Pico sections (4 bytes each here), small synthetic camera frames, and a synthetic robot. On the real Brain-profile setup, `brain_link`, `pico` and `diagnostics` add bytes to every inspect/1 snapshot.
- **Attribution.** Causes are attributed from code reading plus these symptoms. There was no function-level profile.
- **Two harness behaviours to keep in mind.** The stall client keeps sending pings while stalled. The slow client's `SO_RCVBUF` is 8 KB, but Windows may still buffer more on loopback.
- **Harness revisions.**
  - The first run's server numbers were taken by a harness that parsed between reads. Burst timings (the stall resume) carried its parse cost.
  - The fix-stage harness parses after the run. Compare before and after only with the fixed harness on both trees; section 9 rebuilds both.
  - The first run and the rerun also differ in host load (section 1).

## Note: preliminary inspect/2 check (not the after-measurement)

At 04:45 the harness built unchanged against the live tree, where W1's inspect/2 work was still in progress, and ran for 6 s per scenario. Those numbers are only a compile-and-protocol smoke test; the real after-measurement comes after integration:
- The contract was detected as `navigatr.inspect/2`.
- `state` was about 1.08 KB, built in about 24 us p50, at 30 per s.
- The fast client's publish to receive was 0.46 / 0.67 / 2.6 ms.
- The slow client at 100 kB/s stayed bounded at about 0.85 to 0.93 s instead of growing to 11.8 s.

The residual 0.9 s is most likely bytes already written ahead of the newest state (the in-flight frame and the kernel socket buffers). It is noted for W1 in `progress/W0.md`.

## 9. How to rerun every scenario


All commands are for Git Bash on this Windows machine. `S` is the scratchpad folder:
```
S=C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad
export PATH=/c/msys64/ucrt64/bin:$PATH
```

**1. Build the harness against each tree** (Release, as on the Pi). The harness source is the same for both; it detects inspect/2 at compile time.
```
# baseline: the frozen inspect/1 copy
cmake -S /c/GATR2-Electronics/pi/naviGATR/tools/perf -B $S/w0-base-build -G "MinGW Makefiles" \
      -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_SOURCE_DIR=$S/viz-baseline-src/pi/naviGATR
cmake --build $S/w0-base-build -j16

# final tree: inspect/2
cmake -S /c/GATR2-Electronics/pi/naviGATR/tools/perf -B $S/w0-after-build -G "MinGW Makefiles" \
      -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_SOURCE_DIR=C:/GATR2-Electronics/pi/naviGATR
cmake --build $S/w0-after-build -j16

# the tools' own checks (estimator, report labels), either build
ctest --test-dir $S/w0-after-build
```
Once the lead adds the `NAVIGATR_BUILD_PERF_TOOLS` lines, an in-tree build also works: `cmake -S pi/naviGATR -B <dir> -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_PERF_TOOLS=ON`. `run_perf.sh` finds the tools under `<dir>/tools/perf`.

**2. Run everything.** Each call runs the server scenarios in-process, then the browser scenarios against the `navigatr` binary of the same build.
- Use a new `--out` folder: a repeat deletes its own old `server_rN.json`/`browser_rN.json` before it runs.
- The default browser scenarios are `idle,orbit,idle_cpu4,orbit_cpu4`.
- A browser run is refused when `--port` or `--cdp-port` already answers. `navigatr_cdp_bench` also checks that the server session and the DevTools URL are the ones the run started.
- The script exits 1 when any run failed. Each failure is a `FAILED` line in `conditions.txt`.
```
cd /c/GATR2-Electronics/pi/naviGATR
bash tools/perf/run_perf.sh --build $S/w0-base-build --src $S/viz-baseline-src/pi/naviGATR \
     --out $S/w0-results/baseline-2 --label baseline --repeats 3
bash tools/perf/run_perf.sh --build $S/w0-after-build --src /c/GATR2-Electronics/pi/naviGATR \
     --out $S/w0-results/after --label after --repeats 3
```
Machine load changes over time, so a before/after comparison is fairest when the two are interleaved:
```
for i in 1 2 3; do
  bash tools/perf/run_perf.sh --build $S/w0-base-build --src $S/viz-baseline-src/pi/naviGATR \
       --out $S/w0-results/ab-base --label baseline --first $i --repeats 1
  bash tools/perf/run_perf.sh --build $S/w0-after-build --src /c/GATR2-Electronics/pi/naviGATR \
       --out $S/w0-results/ab-after --label after --first $i --repeats 1
done
$S/w0-after-build/navigatr_perf_report.exe $S/w0-results/ab-base=baseline $S/w0-results/ab-after=after
```

**3. Single scenarios by hand.**
- **Server, in-process (exact ages):** `navigatr_inspect_bench --config <tree>/config/demo/synthetic_field_demo.xml --scenario build,fast,slow,stall,burst --duration 20 --warmup 6 --out server.json`. `--help` lists the knobs:
  - scenario knobs: `--slow-bytes-s`, `--stall-ms`, `--burst-hz`;
  - in-process overrides: `--snapshot-hz`, `--client-buffer-kb`;
  - inspect/2 overrides, applied only when the tree has the fields: `--state-hz`, `--diag-hz`, `--reliable-kb`, `--send-buffer-kb`, `--subscribe '<json>'`;
  - `--offset-max-half-ms`, `--defer-mb`.

  The conditions record the `state_hz`, `diag_hz`, `reliable_kb` and `send_buffer_kb` in effect (null on a tree without them). `snapshot_hz` is marked as the inspect/1 rate.
- **Server in another process** (the Pi through an SSH forward, or a running `navigatr`): `navigatr_inspect_bench --connect 127.0.0.1:8765 --scenario fast,slow,stall`.
  - Ages are then estimates only: RTT, relative lag, and for inspect/2 the section 3.4 ping-offset estimate, printed with its +-half-width. Estimates wider than +-10 ms are counted and not used.
  - The server's hello (session, configuration, inspection rates) is recorded in the conditions.
- **Browser:**
  ```
  (cd <tree> && <build>/navigatr config/demo/synthetic_field_demo.xml --inspect-port 18765) > nav.log 2>&1 &
  "/c/Program Files/Google/Chrome/Application/chrome.exe" --headless=new --remote-debugging-port=19333 \
      --user-data-dir=$S/w0-chrome --no-first-run --window-size=1600,900 about:blank 2> chrome.log &
  <build>/navigatr_cdp_bench --cdp 127.0.0.1:19333 --url http://127.0.0.1:18765/ --server 127.0.0.1:18765 \
      --expect-session <the "session" navigatr printed in nav.log> \
      --expect-browser-ws <the "DevTools listening on" URL in chrome.log> \
      --scenarios idle,orbit,idle_cpu4,orbit_cpu4 --duration 15 --warmup 8 --close-browser 1 --out browser.json
  ```
  `run_perf.sh` does exactly this. Exit 3 means the port was serving something else.
- Tables: `navigatr_perf_report <results dir>=label [<results dir>=label ...]`.

## 10. Harness files (all new, `pi/naviGATR/tools/perf`)

| File | What |
|---|---|
| `CMakeLists.txt` | Standalone (`-DNAVIGATR_SOURCE_DIR=<tree>`) or in-tree under `NAVIGATR_BUILD_PERF_TOOLS` (the lead adds that option; see `progress/W0.md`) |
| `inspect_bench.cpp` | Server scenarios: build, fast, slow, stall, burst; in-process or `--connect`; inspect/1 and inspect/2 (optional config fields and `stateDocument`/`historyDocument` timing are detected at compile time) |
| `cdp_bench.cpp`, `perf_probe.js` | Headless Chrome/Edge over raw CDP; the probe wraps `requestAnimationFrame` and `WebSocket` from outside the app |
| `perf_report.cpp` | Markdown tables, one column per results folder |
| `perf_net.*`, `perf_json.*`, `perf_sys.*` | Loopback HTTP/WebSocket client, small JSON reader and summaries, host/process CPU sampling |
| `perf_analysis.*` | The section 3.4 ping-offset estimator with its +-RTT/2 half-width (fix stage) |
| `perf_selftest.cpp`, `testdata/` | Checks run by `ctest`: the estimator, and the report's labels on inspect/1- and inspect/2-shaped fixture folders (fix stage) |
| `run_perf.sh` | Runs everything against one build, records conditions and host load, refuses busy ports, exits 1 on any failed run |
| `README.md` | Short signpost |

No product code was changed. The harness is C++; the page probe is JavaScript, the viewer's own language, injected by CDP; the runner is bash, like `tools/viewer_smoke.sh`. No Python.
