# perf

Inspection feed and viewer measurements on a host. These are host numbers, not hardware validation.

- `navigatr_inspect_bench`: builds a System from a config, serves it in-process and connects loopback WebSocket clients: fast, slow (throttled reads), stalling, and preview burst. Every age uses the one Pi host clock, so publish-to-receive and pose age at receipt are exact. Publish to receive starts at the document's `host_ms`/`host_us`, which the server takes before building the document, so it includes the build. The clients only read during a run and parse afterwards, so the harness's own parse cost never delays a receive time. On inspect/2 it also records each client's server-side queue view (`diag.inspection.clients`). `--connect host:port` measures a server in another process instead: RTT, relative lag, and the spec 3.4 ping-offset estimate with its +-RTT/2 bound (estimates wider than `--offset-max-half-ms` are counted, not used). It speaks inspect/1 and inspect/2.
- `navigatr_cdp_bench` with `perf_probe.js`: drives headless Chrome/Edge over CDP. It measures frame timing, long tasks, onmessage cost per message type, receive-to-next-frame, DOM churn and heap, idle and during an orbit drag, with optional CPU throttling. `_netK` scenarios are not in the defaults: CDP network emulation did not slow the WebSocket in Chrome 153, and the tool flags it when that happens. `--expect-session` and `--expect-browser-ws` refuse to measure a server or browser other than the one the caller started.
- `navigatr_perf_report`: markdown tables from run folders, one column per folder.
- `run_perf.sh`: runs every scenario against one build and records the machine conditions. A browser run is refused when its ports are already in use, and the script exits 1 when any run failed.
- `navigatr_perf_selftest` and the `perf_report_*` CTest checks (fixture in `testdata/`): the estimator and the report labels.

Build against any naviGATR tree (Release, like the Pi):

```
cmake -S tools/perf -B <dir> -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_SOURCE_DIR=<naviGATR tree>
cmake --build <dir> -j16
ctest --test-dir <dir>
```

Or build in-tree with `-DNAVIGATR_BUILD_PERF_TOOLS=ON`.

Run and compare:

```
tools/perf/run_perf.sh --build <dir> --src <naviGATR tree> --out <results> --label after
<dir>/navigatr_perf_report <baseline results>=baseline <results>=after
```
