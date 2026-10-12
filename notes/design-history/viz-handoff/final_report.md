# Phase 1 final report: visualizer and diagnostics

Date 2026-09-28. Everything below is host validation on the Windows laptop (i9-14900HX, 32 threads, Windows 11, MinGW ucrt64 GCC 15.2, Chrome 153 headless). Nothing ran on the Pi, the Pico, the Brain or the camera. Nothing was committed, pushed, flashed or deployed. Phase 2 (the whole-system read-only audit) has not started; it waits for the user.

Tree: HEAD `2d11c67` ("rename of folders", committed by the user at 06:17) plus 46 modified files in the working tree (fix stage, integration, docs).

## 1. What changed

### Latency and smoothness (naviGATR inspection + spectaGATR)
- Feed contract `navigatr.inspect/2` replaces the 20 Hz full snapshot push:
  - `state`: about 1.1 KB, up to 30 Hz, latest-wins;
  - `diag`: up to 4 Hz, skipped when unchanged;
  - `history`: the trail, on connect, epoch change or request;
  - `event`, `telemetry`, `instrumentation`, `capture`, `pong`;
  - binary previews.
  - `GET /api/snapshot` still returns the full inspect/1 document.
- Per client, the server keeps:
  - one in-flight frame, never modified;
  - a bounded reliable FIFO (hello, history, events, capture status);
  - one replaceable slot per channel, so a newer unsent state replaces the older one.
- Flow control uses WebSocket ping/pong with a 64 KiB unconfirmed-bytes window, 16 KiB socket send buffers, and TCP_NOTSENT_LOWAT on Linux. Old positions no longer queue up behind a slow link.
- Per-client and per-channel stats: sent, replaced, refused, bytes and queue age. There are also three latency terms:
  - source age at publish, on the Pi clock;
  - publish to receive, a ping-offset estimate with a +-RTT/2 bound;
  - receive to render, on the browser clock.
- Browser:
  - handlers only parse and store; one requestAnimationFrame loop draws;
  - panels update in place at most 10 Hz, and only while visible;
  - the trail is a ring buffer with range uploads; labels are rasterized only when their text changes;
  - picking is throttled, and a render budget applies under heavy load.
- Smoothing is presentation only:
  - it lags the newest pose by at most one state period (about 33 ms at 30 Hz, never over 100 ms) and never extrapolates;
  - it snaps on session, epoch, placement or validity changes, on reconnect, and on jumps over 0.2 m or 15 deg;
  - after 250 ms without data the robot is drawn exact and dimmed, labeled "pose stale";
  - readouts, graphs and exports never use the smoothed pose.

### Instrumentation
- `DiagnosticsHub` (Pi): producers post small fixed records. Posting costs one short lock and a copy into a bounded ring. Drops are counted per stream. A stream nobody wants costs one atomic load.
- `LinkMonitor` per link, fed only from the reads and writes the links already do (no second reader). It keeps:
  - always-on counters: RX bytes, TX attempted and TX accepted, frames, rejections with reasons, reader stats (sync drops, length and check errors);
  - rates and last-valid ages;
  - optional bounded raw-hex and decoded rings, on only while a viewer asks.
- Pico diagnostic frame 0x14 (optional, 0 to 5 Hz, sent only in the idle windows after a status frame, never delaying a sensor frame). It carries:
  - pin logic levels (labeled HIGH/LOW read back, not voltages; a static UART HIGH proves nothing);
  - IMU init and retry state;
  - link RX errors and skipped ticks.
- An older Pico answers "unknown op" and the viewer says so.
- Encoder status shows counts, change over 1 s, direction and A/B levels. It says plainly that zero change cannot tell stationary from disconnected.

### Graphs, overlays, capture, replay, export
- Live graphs: selectable series with units and sources. Missing data is drawn as a gap, never as zero. Min/max decimation keeps spikes, headings are unwrapped, and invalid spans and events are marked. Commanded vs actual velocity are in the same body frame.
- Overlays: telemetry destination and heading, desired heading at the robot, follower target. They are drawn only with a resolved target (flag bit 3); (0,0,0) is never drawn as a guess.
- Pi-side capture:
  - a rolling pre-trigger window, then a post window;
  - streams selectable, raw bytes off by default;
  - bounded by records, memory, duration and bundle size, with truncation reasons recorded;
  - one capture at a time (a second start gets 409);
  - it survives a browser disconnect, and cancel works;
  - automatic triggers (link lost, continuity lost, Pico reboot, profile changed) with a cooldown and an hourly cap;
  - optional directory write, where a failure keeps the bundle in memory and says so loudly.
- Bundle `gatr2.capture/1` (store-only ZIP):
  - one CSV per stream at its own sampling times;
  - `metadata.json` with configuration, profile, calibration, identities, clocks, units, conventions, drops and segments;
  - `README.txt`;
  - segments split at session, reset, epoch and boot, so incompatible data is never joined.
- Replay: a ZIP from the Pi or a local file. Play/pause, scrub, 0.25x to 4x, event markers, graphs and the 3D view share the same playhead. A REPLAY banner is shown, live controls are disabled, and there is no send path.

### Attitude
- Brain TELEMETRY op 12 (optional, 100 ms, lowest priority, latest-wins, never resent).
  - It carries VEX IMU roll and pitch, converted to the robot frame with `kVexImuMountYawDeg`, plus the movement state, target, commanded speeds, errors and motor targets.
  - An older Pi answers UnsupportedOp and the Brain stops sending for that session.
- Viewer status is one of `measured` (at most 250 ms old), `stale`, `unavailable` or `assumed_level`, each drawn distinctly. Tilt only, never height.
- Pico IMU attitude is unavailable: the firmware does not send accel. This is documented, and the old path is preserved.

### Camera preview
- `config/override/brain_profile_usb_camera.xml` is the current USB setup plus a 640x480 Y8 15 Hz camera.
- World estimation is `noop`: no detection, association or correction.
- Previews are bounded at 5 Hz and 640 px.
- A missing camera is a warning only. Tests cover a dead camera and a streaming one.

### Names (docs/naming.md)
naviGATR, spectaGATR, aggreGATR, translaGATR (was `common/`, namespace `translagatr`), locaGATR, operaGATR. investiGATR, communiGATR and actuGATR are unchanged.

## 2. Before and after (host, synthetic demo, loopback; not the Pi)

Interleaved A/B, 3 rounds, same machine, same config (`config/demo/synthetic_field_demo.xml`), 07:00 to 07:30. Host CPU was 3 to 9% busy before runs. Baseline = the frozen pre-change tree (inspect/1), after = this tree, both Release. Raw: `$S/w0-results/final-ab-base`, `final-ab-after`, report `final-ab-report.md`. Cells are p50 / p95 / max.

| | baseline | after |
|---|---|---|
| bytes per viewer per s | 1054 kB/s | 151 kB/s |
| state message size | 51.9 kB | 1.1 kB |
| state rate to a fast viewer | 20 /s | 29.7 /s |
| publish to receive, fast viewer, ms | 2.04 / 2.69 / 7.58 | 0.39 / 0.51 / 1.19 |
| pose age at receipt, fast viewer, ms | 26.3 / 36.8 / 52.6 | 24.9 / 35.5 / 51.3 |
| **slow viewer (100 kB/s): pose age at receipt, ms** | **8852 / 11694 / 11822** | **250 / 502 / 626** |
| slow viewer: distinct positions received in 20 s | 35 | 334 |
| stalled viewer (3 s): time to first fresh state after resume | 29 ms | 4.6 ms |
| browser idle: main thread busy | 84% | 69% |
| browser idle: frame interval ms | 8.0 / 16.2 / 47.9 | 8.0 / 8.3 / 24.1 |
| browser idle: frames over 33 ms per 15 s | 2 | 0 |
| browser idle: WebSocket handler time | 56 ms/s | 7.4 ms/s |
| orbit drag: frames per s | 95.6 | 123.6 |
| orbit drag: frame interval p95 | 17.1 ms | 8.3 ms |
| orbit drag: input ack ms | 9.2 / 18.5 / 44.8 | 6.5 / 9.7 / 20.4 |
| orbit drag: layouts per s | 185 | 12 |
| orbit drag: DOM nodes at end | 19867 | 2945 |
| CPU 4x idle: frames per s | 4.0 | 60.8 |
| CPU 4x idle: arrival lag vs best, ms | 2548 / 3388 / 4469 | 28 / 137 / 616 |
| CPU 4x orbit: frames per s | 4.4 | 10.2 |
| CPU 4x orbit: arrival lag vs best, ms | 2469 / 2827 / 3471 | 137 / 239 / 506 |
| CPU 4x orbit: input ack ms | 217 / 326 / 397 | 98 / 153 / 251 |
| estimation worker rate and overruns | ~96 Hz, 0 | ~96 Hz, 0 (unchanged) |

Reading:
- "Robot appears late" was mostly backlog. On a link slower than the feed, the old server queued whole snapshots in FIFO order, so a 100 kB/s viewer saw positions 9 to 12 s old; now 0.25 to 0.6 s. A saturated browser main thread gave 2.5 s lag at 4x CPU; now 28 to 137 ms.
- On a fast link the Pi inspection path now adds under 1 ms. Pose age is still about 25 ms, which is the source age at publish (measurement to localization publication in this synthetic demo), upstream of inspection. This run does not show what it is on the robot.
- The stall case still delivers about 29 small stale states after a 3 s network stall (kernel buffers). They arrive within 5 ms of resuming, so the viewer catches up almost at once.
- "Stutter when orbiting" is gone at normal CPU: p95 frame 17 to 8 ms, no frames over 33 ms. At 4x CPU the orbit is still render-bound (three.js/WebGL, 99% main thread): 10 fps, better than 4.4 but not smooth. Frames and long tasks rise with the frame rate.
- Smoothing adds at most one state period (about 16 ms average, 33 ms max at 30 Hz). The harness numbers are before smoothing.

## 3. Test evidence (fresh builds, 07:00 to 07:45)

| Suite | Result |
|---|---|
| brain (communiGATR, actuGATR, investiGATR, sims) | 455/455, 0 warnings |
| translaGATR codec | 254/254, 0 warnings |
| Pico host tests (aggreGATR) | 129/129, 0 warnings |
| naviGATR | 626/626, 0 warnings (includes `BrainLinkEndToEnd.TelemetryKeepsFlowingOnASlowerLine`) |
| PlatformIO `hat2_bno08x`, `hat2_asm330` | build OK, 0 warnings |
| PROS locaGATR, operaGATR (clean) | build OK, 0 warnings |
| naviGATR Release with perf tools | build OK, 0 warnings; perf selftests pass |
| Browser suite vs the final binary | 9/9 scenarios, 118 checks (unit, live, nowebgl, slow, reconnect, replay, record, capfail; paths needs the stub) |
| Browser suite vs the stub server | 9/9 scenarios, 145 checks (paths 15/15) |
| Slow client, real binary, 100 kB/s + CPU 4x | staleness on the Pi clock p50 142 ms, p95 295 ms, max 387 ms, no growing backlog |
| `tools/viewer_smoke.sh` on the final binary | PASS (inspect/2, attitude measured, flow control active) |

Counts before this work: translaGATR 221, Pico 110.

## 4. Limits and open items

- Host only. Not run on hardware:
  - the Pi build and viewer over the real SSH tunnel;
  - DIAGNOSTICS and 0x14 on the real Pico UART;
  - `gpio_get` on pins in UART/SPI function;
  - TELEMETRY over V5 USB and RS-485;
  - the camera config on the physical camera;
  - TCP_NOTSENT_LOWAT (Linux only; not compiled here);
  - automatic capture triggers.
- Attitude sign convention and `kVexImuMountYawDeg` (0, a placeholder) are UNVERIFIED; the bench check is in docs/brain_setup.md section 10.
- The Brain attitude is timed by Pi arrival (about one telemetry period plus link latency late).
- Telemetry costs the Brain link some state polls: host sims show 6 to 11% fewer at default timing and up to 20% fewer on a slow line. `kSendTelemetry = false` turns it off.
- Pico IMU profiles show attitude `assumed_level` (the firmware sends no accel).
- A 20 KB diag already in flight can delay a state on a very slow link (about 330 ms at 60 KB/s). Previews still get encoded at the requested rate for a slow client.
- Browser frame numbers above use the laptop GPU; the W2 slow tests used software GL. The Pi's CPU cost of capture bundle building is unmeasured.
- Case-only rename: staged in the git index (423 pure renames, `pi/navigatr/` to `pi/naviGATR/`); the user commits it. The disk folder stays lowercase until this session closes (harmless on Windows).
  To fix the disk name afterwards, from Git Bash at the repo root: `mv pi/navigatr pi/naviGATR_tmp && mv pi/naviGATR_tmp pi/naviGATR` (git sees no change). The empty `brain/localization-test` shell was removed.

## 5. Docs
- pi/naviGATR/docs: inspection.md (inspect/2, delivery, latency terms, viewer), capture.md, camera_preview.md, pico_link.md (Diagnostics, Instrumentation), brain_profile.md.
- docs: interfaces.md (0x14, DIAGNOSTICS, TELEMETRY, reader stats), communigatr.md and actugatr.md (Telemetry sections), brain_setup.md section 10 (telemetry, attitude, bench check), naming.md.
- pi/naviGATR/tools/perf/README.md (baseline, rerun, compare).
- Indexes: docs/README.md and pi/naviGATR/README.md.
- Final-pass fixes: the Pi copy command packed the renamed `common` folder (now `translaGATR`), mismatched Pi paths, leftover "common codec" wording, and capture.md claiming `<Capture>` was not yet accepted in Configuration profiles.
