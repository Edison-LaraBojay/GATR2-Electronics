# W0 (performance measurement): stage 1 result

## Built
All files are new, in `pi/naviGATR/tools/perf`. No product code was touched.
- `CMakeLists.txt`: standalone (`-DNAVIGATR_SOURCE_DIR=<tree>`), or in-tree behind `NAVIGATR_BUILD_PERF_TOOLS`, which the lead still has to add (below).
- `inspect_bench.cpp`: the server-side harness.
  - Modes: in-process (System + InspectionService + loopback WebSocket clients; exact host-clock ages) or `--connect host:port` (RTT, relative lag, and the section 3.4 ping-offset estimate).
  - Scenarios: `build`, `fast`, `slow`, `stall`, `burst`.
  - Speaks inspect/1 and inspect/2. On a tree that has them, it also times `stateDocument`/`historyDocument` and sets `state_hz`/`diag_hz`/`reliable_kb` (detected at compile time).
- `cdp_bench.cpp` + `perf_probe.js`: the browser harness.
  - Headless Chrome/Edge over raw CDP; nothing installed, no Python.
  - The probe wraps rAF and WebSocket from outside the app, so any viewer version can be measured.
  - Scenarios: idle, orbit drag, `_cpuN`, `_netK`.
  - Also samples server counters and host CPU per scenario, and reloads a page that never goes live, recording why.
- `perf_report.cpp`: markdown tables, one column per results folder.
- `perf_net.*`, `perf_json.*`, `perf_sys.*`: shared helpers.
- `run_perf.sh`: runs everything and records conditions. `--first` allows interleaved A/B runs.
- `README.md`.

## Commands and results (host only; not hardware validation)
- `cmake -S pi/naviGATR/tools/perf -B $S/w0-base-build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_SOURCE_DIR=$S/viz-baseline-src/pi/naviGATR` then `cmake --build $S/w0-base-build -j16`: OK, 0 warnings in tools/perf.
- The same against the live tree, in `$S/w0-after-build` (`-DNAVIGATR_SOURCE_DIR=C:/GATR2-Electronics/pi/naviGATR`): OK, 0 warnings in tools/perf (built 04:45 and 05:00 while other workers were still editing).
- `bash tools/perf/run_perf.sh --build $S/w0-base-build --src $S/viz-baseline-src/pi/naviGATR --out $S/w0-results/baseline --label baseline --repeats 3`: 3 of 3 server runs and 3 of 3 browser runs exited 0, and the viewer was live in 3 of 3.
- Extra runs:
  - Slow client at 500 kB/s: lag plateau about 2.35 s.
  - `--connect` against a separate baseline process: works, estimates only.
  - Preliminary 6 s run against the live inspect/2 tree: contract detected; state 1.08 KB built in about 24 us; fast client publish to receive 0.46 ms p50; slow client bounded at about 0.9 s.
- No gtest was added. The harness is a tool, and it was exercised end to end on both trees.

## Baseline in one paragraph
Full details are in `W0_baseline.md`. With a consumer that keeps up, the Pi side adds little. Pose age at receipt is 26 / 40 / 133 ms (p50 / p95 / max), of which queue plus loopback is 4.5 / 5.5 / 22 ms. The feed, though, is 1.05 MB/s per client: 51 KB snapshots, 67% of each being the trail, sent 20 times a second. A consumer that falls behind is served the oldest data first from a queue of about 1.17 MB, while newer snapshots are refused:
- a 100 kB/s reader reaches 11.8 s of lag, and a 500 kB/s reader 2.35 s;
- a 3 s pause delivers 25 stale snapshots, the first 3 s old;
- a 4x CPU-throttled browser lags 4 to 19 s.

That is the measured mechanism behind "late". "Stutter" is main-thread saturation in the browser. Idle, the main thread is 82% busy, from continuous rendering at about 120 Hz and 60 ms per s of synchronous snapshot handling. An orbit drag raises layouts from 26 to 163 per s, and frames over 33 ms from 16 to 40 per 15 s; input acknowledgement runs 11 / 34 / 105 ms. The baseline server also sometimes fails the page load with a 503 on an ES module (max_clients counts HTTP connections).

## Contract issues
- **Lead:** add the `NAVIGATR_BUILD_PERF_TOOLS` option and `add_subdirectory(tools/perf)` at the end of `pi/naviGATR/CMakeLists.txt`. The exact lines are in `progress/W0.md`.
- **W1:** the harness depends on `snapshotDocument`'s signature and on the `workers.inspection` counters in `/api/snapshot`; both are unchanged in the live tree.
- **W1 observation:** the residual 0.9 s slow-client lag in inspect/2 is most likely in-flight plus kernel-buffered bytes. Details and possible levers are in `progress/W0.md`.

## Open risks
- The host was shared and busy (23% to 100% CPU per scenario, recorded). Compare before and after with the interleaved A/B loop in `W0_baseline.md` section 9.
- Headless Chrome ran on a desktop GPU at about 120 Hz. CDP network throttling does not affect WebSockets. Everything was loopback: no Wi-Fi, SSH tunnel or Pi.
- Causes are attributed from code plus symptoms. No function-level profile was taken.

## Corrections (fix stage, see W0_fix.md)
- "queue plus loopback is 4.5 / 5.5 / 22 ms" above is wrong: inspect/1 stamps `host_ms` before building the 51 KB snapshot, so that figure is document build + queue + socket + read, and the build is most of it. W0_baseline.md sections 2, 4.1 and 5 now say so and give the build-subtracted estimate.
- The stall-resume figure (first fresh snapshot 160 ms after resume) was mostly the harness's own JSON parsing between reads. The harness now parses after the run; the rerun figure is in W0_baseline.md section 4.1.
- `idle_net100` is not a slow-network result (CDP did not throttle the WebSocket); it is out of the default lists and flagged when requested.
