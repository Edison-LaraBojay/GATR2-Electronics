# W1 review fixes: inspection transport and server

Host validation only (Windows 11, MinGW ucrt64 GCC 15.2, Debug, -Wall -Wextra, loopback, headless Chrome). Nothing ran on the Pi, over WiFi, or with robot hardware. Build dir `$S/w1-build` (wrapper `$S/w1-wrap`, which also builds the scratch measurement probe `$S/w1-wrap/w1_probe.cpp`, not part of the repo).

## Findings

### 1. (medium) Latest-wins bypassed by kernel buffering: FIXED

Kernel buffers were only part of the problem: the documented access path is an SSH tunnel, whose buffers no socket option on the Pi can bound. So the fix is end-to-end, plus the kernel items from spec 12.2.

**Flow control by WebSocket ping/pong** (`src/inspection/http_server.{h,cpp}`)
- After each frame the server sends a ping whose 8-byte payload is the number of bytes written before it. Browsers answer pings in their network stack without page code; a pong proves everything before that position was read.
- One probe ping goes after the handshake. A client that answers it is under flow control from then on:
  - nothing new starts while `ack_window_kb` (64) of written data is unconfirmed; a close frame still goes;
  - a replaceable frame starts only while at most 16 KiB is unconfirmed, and at most 2 frames of one channel are unconfirmed, so the slot keeps coalescing instead of the network queueing old states;
  - large replaceable frames (4 KiB or more: diag, previews) go one at a time, only with at most 8 KiB unconfirmed, and not before a gap of 3x the previous one's delivery time (write start to its pong), capped at 4 s and at half the stall close. This is the spec 12.2 back-off: large frames take about a quarter of a slow link at most.
- Only pongs matching an outstanding ping position count (at most 64 outstanding; a "ping owed" flag covers a full ring). Pongs also count as progress for the stall close. When the probe's pong arrives, a ping follows at once for the data sent meanwhile; without it, a large hello and history left unconfirmed deadlocked the client (caught by the new minimum-budget test).
- `select()` waits for writability only when flow control lets something go, so held-back data never busy-loops.
- A client that never answers keeps the kernel bound only, with large frames paced by write completion.

**Kernel bound (spec 12.2 items 1 and 2)**
- `send_buffer_kb` now defaults to 16. It sets `SO_SNDBUF`, and `TCP_NOTSENT_LOWAT` where the OS defines it (Linux; not on Windows, so that line is untested here).
- It applies to WebSocket feed sockets at upgrade only; plain HTTP (capture ZIP downloads) keeps the OS default.

**Stats**
- `diag.inspection.clients[]` adds `flow_control`, `unacked_bytes`, `ack_rtt_ms`, `ack_rtt_min_ms`, `pace_gap_ms`.
- Channels add `last_acked_ms` (enqueue to the pong covering it, on the Pi clock).
- `workers.inspection.flow_control_clients` (also in `GET /api/snapshot`).
- New channel name `ws_ping`.

**Found while fixing: close echo lost.** A client answering pings can send a pong after its close. An immediate `closesocket` then turned the close into a reset, and the client could lose the close echo (`Inspection.ReconnectingClientGetsHelloWithTheSameSession` failed once in 4 suite repeats). Fixed with a lingering close: shutdown of the send side, then wait for the peer's EOF, bounded by the 1 s grace. Input on a closing connection is discarded. Afterwards: 60/60 repeats of that test, plus 4x41 suite repeats.

**Tests at the shipped defaults, no grace period** (`tests/inspection_feed_gtest.cpp`)
- `RateLimitedReaderGetsFreshStatesAtTheDefaults`: 60 KB/s reader, every state from the first. Asserts flow control on, at least 2 diags, and state age at receipt p50 < 100 ms, p95 < 400 ms, max < 1000 ms.
- `PausedClientResumesWithFreshStatesNotABacklog`: 3 s pause with the OS receive buffer. At most 2 stale states and at most window plus one frame of bytes before the first fresh state; every later state is under 250 ms old.
- `FlowControlKeepsOldStatesOutOfTheNetwork` (server level, default buffers): 2000 states to a non-reading client.
  - Unconfirmed bytes stay at 2 states or fewer, and the reader gets at most 2 older states before the newest.
  - A second client with `answer_pings` off still gets the newest state and sees exactly one probe ping.
- Replaced `SlowClientReceivesRecentStatesNotABacklog`, which used `send_buffer_kb=8`, a 4 KB receive buffer and a 700 ms grace.
- The test client (`tests/inspection_test_client.h`) answers pings when the reader reaches them, as a browser does. `answer_pings = false` gives a client without flow control.

### 2. (low) reliable_kb floor below hello/history size: FIXED
- `reliable()` admits a message when the reliable FIFO is empty, whatever its size; the budget bounds the backlog.
- The floor is raised to 128 (`kMinReliableKb`) in `src/runtime/inspection_config.h` and `docs/inspection.md`.
- Tests:
  - `ReliableBudgetBoundsTheBacklogNotOneMessage` (server level): a 2 MB message and a 40 KB one are admitted against a 16 KB budget; the next one overflows and closes the client;
  - `MinimumReliableBudgetOpensEveryClientWithAFullTrail`: `reliable_kb="128"`, 400 cycles of trail, 3 connects, each gets hello, history (>10 KB), capture, then states, and nothing closes;
  - the config test now rejects `reliable_kb="127"`, `ack_window_kb="8"` and `send_buffer_kb="-1"`, and checks the new defaults.

### 3. (low) Identical-diag skip never fired while running: FIXED
- `JsonWriter::recordVarying` / `varying()` / `varyingField()` record byte spans of varying values; `contentHash` sorts the spans (they nest) and skips them.
- Varying values:
  - every host time, age and source stamp;
  - `cycle` and the `robot` block (state carries it);
  - `localization.updates`, `history_size` and `publication`;
  - `dropped_intervals` (it rose every cycle while the rig moved during calibration: the case that kept the skip from ever firing once the process clock had passed the rig's hold);
  - stillness progress counters;
  - field snapshot invocation; object and source sequences;
  - detection frame sequence, invocation, processing time, and the pose and tilt at exposure;
  - `estimate_id`, the wheel counts, travel and age, `uptime_ms`;
  - worker timing, dropped and pending;
  - run and traffic counts in `diagnostics`;
  - hub posted, queued and drained.
- Kept as content: readiness, notes, reasons, calibration states, epochs, events, error and fault counts, overruns, detections, profiles.
- Unchanged content now refreshes every 1 s (was a 2 s keepalive), so panels stay current; content changes still go at `diag_hz`.
- Measured with the workers running (scratch probe): 1 diag/s sent, 3 of 4 skipped. With the demo's camera, tag detections change every frame, so the diag goes at `diag_hz` there, as it should.
- Tests:
  - `UnchangedDiagIsSkippedWhileTheWorkersRun`: skips > 0 and 2 to 8 diags in 3 s;
  - `DiagHashIgnoresTheMovingRobotButNotContent`: calibration and reset change the hash, 3 cycles of steady motion do not;
  - `IdenticalDiagIsSkippedUntilTheRefresh` updated to the 1 s refresh;
  - doc test `DiagHashIgnoresVaryingValuesButNotContent`.

### 4. (low) Reset label read after the snapshot: FIXED
- `inspection_document.cpp` `underOneReset` reads `resetCount()` before the data, builds, and checks it after; it rebuilds when a reset landed between, 3 attempts.
- It covers state, history, diag and snapshot. When still unstable, state and diag are skipped for that tick (the clients stay due), and history and snapshot use the last build.
- Why this is enough: `System::reset` publishes the power-on state and increments the count with the workers paused between. So a stable label never carries an older session's pose. The reverse, the old label on the invalid power-on state of the next epoch, can happen once per reset and is documented.
- `docs/inspection.md` now says clients adopt the session from any message, which is what `spectaGATR/feed.js` does; it no longer says clients drop such messages.
- Test: `ResetsUnderLoadNeverLabelAnOldPoseWithTheNewSession`. Workers run, `reset()` every 25 ms for 3 s, states at 60 Hz and diags at 20 Hz. For labels r < r2, every valid state labeled r has an older odometry epoch than any state labeled r2, and state labels never go back.
  - This is a guard under load, not a reproduction: the original window is a few instructions wide.

## Test commands and exact results
- Build: `cmake --build $S/w1-build -j16` (all targets). The forced recompile of `http_server.cpp`, `inspection_document.cpp`, `inspection_service.cpp` and the three inspection test files gave 0 warnings.
- `navigatr_tests --gtest_filter=InspectionFeed.*:Inspection.*:InspectionDocuments.*:JsonWriter.*`:
  - one run: 41/41 passed (34 before: 8 new tests, 1 replaced);
  - `--gtest_repeat=4`: 4 x 41 passed, after the lingering close. Before it, one repeat run had one failure: the close-echo race above.
- `--gtest_filter=Inspection.ReconnectingClientGetsHelloWithTheSameSession:Inspection.StopIsPrompt*:Inspection.HttpRoutes*`, run 60 times: 60/60 passed.
- Full `ctest --test-dir $S/w1-build/navigatr -j16` on the working tree as of the run: 622/622 passed (13.6 s).
- `tools/viewer_smoke.sh <w1 navigatr.exe> config/demo/synthetic_field_demo.xml --attitude-badge valid`: PASS.
  - New check: `flow_control_clients` max=1 while the page was loaded, so headless Chrome answers the flow-control pings.
  - Page: live, 20 states, 4 frames, 3 diags, attitude measured.
- The same with `synthetic_field_demo_no_attitude.xml --attitude-badge assumed`: PASS (flow_control_clients max=1).
- W2's `tools/viewer/viewer_tests.py` run against my binary (not edited; artifacts in `$S/w1fix-vt-*`). The `slow` scenario uses real Chrome, CPU throttle x4 and a throttled TCP proxy; staleness is Pi now minus the page's newest state, on the Pi clock.

| Setup | p50 | p95 | max | Result |
|---|---|---|---|---|
| 40 KB/s, flow control on, run 1 | 91 ms | 462 ms | 629 ms | PASS |
| 40 KB/s, flow control on, run 2 | 93 ms | 509 ms | 599 ms | PASS |
| 40 KB/s, `ack_window_kb="0"` | 1786 ms | 2472 ms | 2853 ms | FAIL (its 2000 ms bound) |
| 100 KB/s, flow control on | 92 ms | 265 ms | 294 ms | PASS |

  - Spec 12.2 recorded 1.5 to 3.6 s p50 at 40 KB/s before.
  - `live` 14/14, `reconnect` 7/7, `record` 17/17 passed.
- Scratch probe (`$S/w1-build/w1_probe.exe`, synthetic rig, workers running, `preview_hz=0`). The reader drains 60 KB/s, and ages count from the first message:

| Setup | p50 | p95 | max |
|---|---|---|---|
| Defaults (flow control on) | 1 to 16 ms | 183 to 189 ms | 262 to 277 ms |
| `ack_window_kb="0" send_buffer_kb="0"` | 341 to 399 ms | 590 to 648 ms | not recorded |
| `ack_window_kb="0"`, 16 KiB buffer | 326 to 402 ms | 491 to 705 ms | not recorded |

  - Reader at 20 KB/s, flow control on: second-half p50 77 ms, p95 95 to 108 ms. Whole-run p95 614 ms, because the first states wait behind hello and history.
  - 3 s pause: 2 stale states (14 KB) before the first fresh one with flow control, 84 stale states (106 to 117 KB) without.
  - All in one process on one clock (HostClock), so the ages are exact.

## Contract issues
- **W0:** `tools/perf/inspect_bench.cpp` does not answer server pings, so its clients take the legacy path (kernel bound only); it counts the one probe ping as `ws_other`. To measure what a browser gets, answer each ping (opcode 0x9) with a masked pong (0xA) carrying the same payload, when the reader reaches it. W0's before/after slow-client numbers without that measure the fallback, not the viewer.
- **W2** (informational, no change needed):
  - diag ages and counters refresh at 1 Hz when nothing else changed, so extrapolate ages from the diag's arrival as already done for `field_snapshot`;
  - new per-client flow control fields for the latency panel if wanted.
- **Lead:** spec 12.2 items 1 to 3 are now done in W1's files. Item 3 uses delivery time measured by pongs rather than write completion, which the SSH tunnel would hide.

## Open risks
- Flow control needs clients that answer pings. RFC 6455 requires it and Chrome does, as the smoke check shows. A pong proves the browser's network stack read the bytes, not that the page handled them.
- On a link slower than the feed, a state can wait behind one large frame already in flight (a 20 KB diag at 60 KB/s is 330 ms). On a link slower than the states alone (about 30 KB/s), a state can be up to 16 KiB of transmit time old. The diag is not made smaller.
- Paced or replaced previews are still encoded at the requested rate, which costs Pi CPU for a slow client.
- `TCP_NOTSENT_LOWAT` and `SHUT_WR` compile on Linux only; not built or run here.
- A half-closed connection counts toward `max_clients` or the HTTP bound for up to 1 s if the peer never closes.
- Pings add about 10 bytes per frame downstream and 14 upstream.
- The reset-label stress test cannot force the original interleaving; the fix rests on the ordering argument above.

## Tree state
Commit `2d11c67` ("rename of folders", 06:17, not made by this worker) landed during the fix. It contains most of these edits as they stood then. The rest is uncommitted in the working tree: the lingering close in `http_server.cpp`, the final `docs/inspection.md` text, and the flow-control check in `tools/viewer_smoke.sh`. All results above are for the working tree.
