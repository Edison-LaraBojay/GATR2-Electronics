# Discovery: latency chain, Brain-authoritative vs Pi-authoritative pose

Date 2026-09-28. Read-only on the repo (HEAD ae457ee, clean tree; pi folder on disk is lowercase
`pi/navigatr`). No hardware here: nothing below is a robot measurement.

Basis tags: [code] file:line verified in this tree, [source] primary external source,
[measured-host] run here (command given), [estimate] model or arithmetic from stated assumptions,
[inference], [unknown]. Scratch: `$S/spectu/latency-chain/` where
`$S = C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.

## 0. Answer in five lines

1. Only the Pi -> Brain reply leg is inherent to Brain authority: ~4 ms mean on V5 USB (assumed USB
   timing) and ~8 ms on RS-485 (5.12 ms of it is reply airtime). The request leg costs link time, not
   pose age.
2. Pose age when the Brain first has it. These are true ages, with the real communiGATR Client and
   the real naviGATR System in the loop and a modeled transport, not Pi-reported ages:
   - USB (2 ms line): 15.4 ms mean, p95 25.4;
   - RS-485: 20.4 mean, p95 28.7 [measured-host latprobe];
   - my independent model gives 16.5 and 20.2 [estimate].

   A Pi-authoritative 30 Hz viewer of the same pose gets 17.1 mean and 26.8 p95 true source age in
   the same probe.
3. The transports add the real cost:
   - Relaying through the Pi on the current link: ~32-33 ms mean and ~41-42 p95 already at the Pi
     drain [measured-host]. That is ~35-37 mean and ~45-49 p95 at receipt [estimate]. With 10 Hz
     TELEMETRY on, the p95 at the Pi drain is 54-60.
   - Direct Brain USB: ~20-23 mean, but only standalone, because the Brain has one USB port and the
     Pi uses it.
   - Controller radio: unknown. With an assumed 10-50 ms hop it gives ~48-51 mean and ~70 p95.
4. Conflict with the ~25 ms baseline: yes, for the combined topology. The relay costs about +11-12 ms
   at p50 and about +11-13 ms at p95 against the measured host baseline (24.9/35.5 ms). Roughly 14 ms
   of that is avoidable scheduling (details in section 3). Standalone wireless cannot be judged until
   the radio is bench-measured.
5. Keep time honestly:
   - Pi-reported ages are already too low: ~7.7 ms mean in the probe, because the min-offset mapping
     absorbs constant delay. robot_age + RTT on the Brain can therefore be too young by up to ~9 ms;
   - carry the measurement stamp as an interval on each source clock;
   - map clocks explicitly (ping-offset where a return path exists, else a one-way minimum offset,
     labeled as a lower bound);
   - never use the receipt time as the measurement time.

## 1. Stage table: sensor measurement -> pixels

Mean / range, ms. INH = inherent to Brain authority. COM = common to both authorities (upstream
of the fork). AVD = avoidable (queueing, polling, serialization, radio, browser).

| # | Stage | Value | Basis | Class |
|---|---|---|---|---|
| 1 | Pico sample hold (50 Hz; the newest sample is 0-20 ms old at a random instant) | mean 10, 0-20 | [code] pico/aggreGATR/src/config.h:24-25; tick loop main.cpp:318-339 | COM (Pico rate) |
| 2 | Pico -> Pi UART, v2 sensor frame 31 B (14 hdr + 16 payload + 1 xor) at 115200 8N1 | 2.69 | [code] config.h:21; translaGATR/frame_codec.h:14-15; frames.h:34-36, 58-64 | COM |
| 3 | Pi tty/driver after the last byte | <1? | [unknown] | COM |
| 4 | Pi drains the Pico UART once per estimation cycle (100 Hz) | 0-10, mean 5 | [code] naviGATR src/impl/resources/pico_telemetry.h:5-7; runtime/system.cpp:1411-1432; config/override/brain_profile_usb.xml:18 | COM, AVD-in-principle |
| 4a | Device->host mapping uses the minimum (host receipt - device stamp) offset over 64 samples, so it absorbs every constant part of the delay: airtime, plus the Pi batching phase while the 50 Hz Pico and 100 Hz Pi stay phase-locked. Pi-reported ages are therefore too low. In the probe, a Pi 30 Hz state reported 9.45 mean against a true 17.1 | bias -2.7 to about -12.7 | [code] src/core/clock_sync.h:1-20, 32-49; [measured-host] latprobe | COM (honesty caveat) |
| 4b | Age of the newest pose at a Pi cycle, synthetic demo (no UART) | p50 10, p95 25, mean 10.9 | [measured-host] snap_probe.py against final-rel navigatr.exe (sec. 6) | COM |
| 5 | Pi-authoritative viewer: inspect/2 state (30 Hz default, latest-wins, 5 ms feed poll). "Source age at publish" | p50 24, p95 34-35, max 50 | [measured-host] A/B report and rerun (sec. 6); [code] inspection_service.cpp:65, 896-946, 1178-1188; inspection_config.h:47, 69 | Pi path only |
| 5a | Same demo, pose age at random-time /api/snapshot builds | mean 16.5, p50 16, p95 32 | [measured-host] snap2.json | Pi path |
| 6 | GET_STATE poll period; one request outstanding | 20 (50 Hz) | [code] brain/communiGATR/include/communigatr/client.h:49; client.cpp:837-839 | link design |
| 7 | Brain link task (client.poll) grid | 0-2 | [code] pros_link.h:44; pros_link.cpp:145-163 | AVD (small) |
| 8 | Request leg, USB: tx task idle sleep 2 ms; PROS stdout flushed by the system daemon every 2 ms; then VEXos USB | 0-2 + 0-2 + [unknown] | [code] pros/pros_usb_port.cpp:22, 98-124; [source] PROS system_daemon.c (sec. 7) | RTT only, not pose age |
| 8r | Request leg, RS-485: GET_STATE 23 B = 6 envelope + 8 header + 9 body | 2.00 airtime + smart port [unknown] | [code] frame_codec.h:18-26, 31-35 | RTT only |
| 9 | Pi reads the Brain port only in the commands stage of each 100 Hz cycle (no reader thread) | 0-10, mean 5 | [code] brain_link_commands.cpp:280-346; system.cpp:1045; pros_usb_link.cpp:43-66 (nonblocking poll) | RTT only; AVD |
| 10 | Pi builds the reply from the SAME cycle's localization: resources -> sensors -> commands -> localization -> publishing | pose is the newest at that cycle | [code] system.cpp:1039-1076 | - |
| 10a | robot_age_ms = cycle start (in.now) - measuredAtHost. ms-quantized u16; no absolute time; no measurement id | - | [code] brain_link_publisher.cpp:287-290; frames.h:358 | timestamp caveat |
| 11 | Reply write: at least 1 ms guard after the request's completing read, plus cycle work (host 0.13-0.18 ms, max 4.5-5.8 ms; Pi unknown) | ~1-2 | [code] brain_link_commands.cpp:189-190, 373-374; pros_usb_link.cpp:84; [measured-host] last_cycle_ms in snapshots, A/B max cycle | INH |
| 12u | Reply leg, USB: 59 B frame sent as a 123-char NG1 line; VEXos USB; PROS serial daemon reads every 1 ms (priority MIN+1); rx task reads per line | [unknown] + 0-1 | [code] usb_line.h:18-25; pros_usb_port.cpp:126-160; [source] ser_daemon.c, ser_driver.c | INH |
| 12r | Reply leg, RS-485: 59 B = 6 + 13 + 40 | 5.12 airtime + smart port [unknown] | [code] frame_codec.h:24-26, 44 | INH |
| 12h | RS-485 only: the Pi estimation worker blocks until TX is empty (DriverEnable gpio 6 in the RS-485 config) | ~5.3 per reply | [code] transport/half_duplex.cpp:86-125; config/override/brain_profile_rs485.xml:40-43 | Pi loop cost |
| 13 | Brain client.poll picks up the reply | 0-2 | [code] pros_link.cpp:145-163 | INH (small) |
| = | **Reply leg total (Pi cycle -> Brain has it)** | USB mean 3.9, p95 5.2; RS-485 mean 7.7, p95 8.8 | [estimate] chain_model.py | **INH** |
| = | **Pose age when the Brain first has it (true)** | USB (2 ms line) 15.4 mean, p95 25.4, RTT 11.6; RS-485 20.4 / 28.7, RTT 12.3 | [measured-host] latprobe (real Client + System, modeled transport); model 16.5 / 20.2 | COM + INH |
| = | robot_age_ms as the Brain receives it | mean 4.5-6.5, values 0 or 10 (ms-quantized Pi cycle ages) | [measured-host] latprobe | honesty caveat |
| 14 | Brain consumers. LinkDriver age = robot_age + round_trip + since. This is NOT a guaranteed upper bound, because robot_age inherits the 4a bias. Estimate minus true: USB +0.85 mean (min -8.7); RS-485 -3.56 mean (min -9.1). The drive loop runs every 10 ms, locaGATR every 20 ms, operaGATR background every 20 ms | +0-10 or +0-20 if the pose is published from an app loop. Probe: a pose held and republished on a free 30 Hz Brain grid is 26.1 mean against 15.4 at receipt (USB) | [code] link_driver.cpp:89; actuGATR pros_drive.h:27; locaGATR robot_config.h:41; operaGATR main.cpp:380, 521; [measured-host] latprobe | AVD (publish on receipt instead) |
| 15 | Brain publish scheduling | see sec. 2 | - | AVD |
| 16 | Transport to receiver: (a) Pi relay, (b) controller radio, (c) direct Brain USB | see sec. 2 | - | AVD / unknown |
| 17 | Receiver -> browser publish->receive, loopback | p50 0.18-0.39, p95 0.28-0.51 | [measured-host] A/B, rerun | AVD (network) |
| 17a | Pi -> laptop over Wi-Fi / SSH tunnel | - | [unknown] | AVD |
| 18 | Browser receive -> next rAF frame (idle / orbit / 4x CPU idle) | 2.1/5.1/9.3; 2.0/4.2/9.5; 3.7/13.5/55.8 | [measured-host] A/B report | AVD (browser) |
| 19 | Presentation smoothing: blends over one measured period, cap 100 ms, never extrapolates. Worst-case lag 33 ms at 30 Hz, 20 at 50 Hz, 100 at 10 Hz | mean about half a period | [code] spectaGATR/smoothing.js:1-18 | AVD (presentation) |
| 20 | Compositor / scanout to photons | - | [unknown]; no harness measures it | AVD |

## 2. Brain publish scheduling and transport options (after the Brain has the pose)

| Option | Added delay | Link cost | Basis / notes |
|---|---|---|---|
| Publish on state receipt, in the link poll task, as a non-blocking enqueue to a writer | 0-2 ms | none | [estimate]. This is the only way to avoid +0-20 ms of app-loop phase (row 14). |
| Next TELEMETRY slot (op 12): 100 ms grid, first slot after a state reply | 0-100 ms, overdue up to +50; longest wait measured 114-180 ms (host fakes) | 1 exchange per report; costs 6-20% of polls (host), 7-13% in the sibling probe | [code] client.h:12-14, 61; client.cpp:955-1044; [measured-host] viz-handoff W5_fix.md:36-52; discovery/brain-link.md probe B. Unfit for pose. |
| Piggyback the pose (or an ack) on the next GET_STATE | wait ~ state_period - RTT (model ~8 ms mean) + request leg + Pi cycle 0-10 | +16 B per poll (pose echo) = +1.39 ms RS-485 airtime per 20 ms; +6 B for an ack | [code] client.cpp:837-839, 1058-1071 (GET_STATE body is fixed today); [estimate]. Needs a wire change. |
| Separate request right after each state reply | gap 5 ms + request leg + Pi cycle | a whole extra exchange per pose; an exchange plus the 5 ms gap does not fit in 20 ms, so polls roughly halve at pose rate | [code] client.h:48; [measured-host] W5 / sibling (TM at 20-30 Hz costs 20-50% of polls). Not viable at pose rate. |
| (a) Pi relays to its viewer | Pi read cycle 0-10 + inspection 5 ms feed poll (0-5) + WS 0.2-0.4 | as the row above | [code] inspection_service.cpp:65, 1188. Today TELEMETRY is capped at 20 Hz to the viewer (kTelemetryPeriod 50 ms, :67); a pose stream needs its own channel. |
| (b) Controller radio -> controller USB -> laptop | [unknown] | none on the Brain-Pi link | pros-cli: one ext 0x27 read returns at most 0x40 = 64 B, and the reader loop has no sleep. vexide: 100 ms handshake timeout; cargo-v5 sleeps 10 ms per loop. PROS: controller screen updates closer than 50 ms apart over VEXnet are not applied. [source] sec. 7. The radio hop is the dominant unknown. |
| (c) Direct Brain USB -> laptop | PROS flush 0-2 + USB ~1-2 [assumed] + receiver read | none | [source] system_daemon.c; [inference] the Brain has one USB port, so this is standalone or wired only. The current USB topology already uses that port for the Pi (gatr2_robot.h:33 kUseUsb = true). |

Totals, measurement -> receiver receipt, before browser frame and smoothing [estimate, chain_model.py,
300 runs x 2 s at random phases; ms mean / p50 / p95]:

| Path | USB Brain link | RS-485 Brain link |
|---|---|---|
| Pi-authoritative viewer, same model (Linux-like timers) | 18.3 / 18.3 / 28.8 | same |
| Measured host baseline, Pi-authoritative (Windows, A/B) | p50 24.9, p95 35.5 | - |
| Brain has it | 16.5 / 16.6 / 25.8 | 20.2 / 20.0 / 29.6 |
| (a) piggyback, Pi relays, as built (100 Hz read, 5 ms feed poll) | 36.3 / 36.4 / 45.9 | 37.2 / 36.9 / 48.6 |
| (a) piggyback, Pi reads the Brain port on arrival + event-driven relay | 33.6 / 33.4 / 43.3 | 35.0 / 34.9 / 44.4 |
| (a) separate request + arrival read + event relay (costs polls) | 27.6 / 27.7 / 36.8 | 34.0 / 34.1 / 42.7 |
| (b) radio, hop ASSUMED 10-50 ms | 47.8 / 47.8 / 68.9 | 51.4 / 51.4 / 72.8 |
| (c) direct Brain USB (only meaningful standalone) | 19.5 / 19.6 / 28.9 | 23.2 / 23.0 / 32.7 |

Model GET_STATE RTT: USB 12.0 mean, RS-485 12.7 mean. The sibling's real-Pi-runtime harness
measured 4.7-6.7 ms mean with 2 ms per line and 9.4-14 ms with 3 ms per line
(discovery/brain-link.md probe B), consistent in magnitude.

Real-code probe [measured-host]: `$L/build/latprobe.exe <seeds> <run_s> <telemetry> <line_us>`,
with LATPROBE_RS485=1 for the bus. Source is `$L/probe/latprobe.cpp`, found in this scratch
folder from an earlier pass; I reviewed its measurement definitions and reran it. It runs the real
communiGATR Client and the real naviGATR System (brain_profile_usb.xml) on one simulated clock. The
modeled transport includes PROS 2 ms tx, system-daemon grids, the 1 ms serial daemon and the 2 ms
poll task. Ground truth: the robot spins at 90 deg/s, so heading encodes the Pico sample time.
20 seeds x 4 s.

| Metric (true age unless noted) | USB 2 ms, TM off | USB 2 ms, TM 10 Hz | RS-485, TM off | RS-485, TM 10 Hz |
|---|---|---|---|---|
| GET_STATE sent /s | 49.7 | 43.4 | 49.5 | 43.2 |
| Brain new-state interval p95 | 20 | 40 | 20 | 40 |
| Pi 30 Hz state, true source age, mean / p95 | 17.1 / 26.8 | same | 19.0 / 29.0 | same |
| same instant, Pi-reported age, mean | 9.45 | same | 10.07 | same |
| Brain receipt, true age, mean / p95 | 15.4 / 25.4 | 15.2 / 25.1 | 20.4 / 28.7 | 22.1 / 28.7 |
| round trip, mean / p95 | 11.6 / 14 | 13.9 / 14 | 12.3 / 14 | 14.5 / 18 |
| LinkDriver age estimate minus true, mean (min) | +0.85 (-8.7) | +3.2 (-8.7) | -3.56 (-9.1) | -1.31 (-7.1) |
| pose relayed on the next GET_STATE, at the Pi drain, mean / p95 | 32.0 / 41.3 | 34.7 / 54.5 | 33.3 / 42.3 | 37.9 / 59.9 |
| pose relayed on TELEMETRY, at the Pi drain, mean / p95 (6.6-6.7 /s only) | - | 31.7 / 40.7 | - | 35.4 / 42.3 |

Add ~0-5 ms (inspection feed poll) and 0.2-0.4 ms (WS) for receiver receipt. Outputs:
`$L/rerun_{usb,rs485}_tel{0,1}.txt`, with earlier sweeps in `$L/final_*.txt` and `$L/sweep_*.txt`.
Line delays are stand-ins for the unknown VEXos and USB time.

Model assumptions (unmeasured):
- USB one way (VEXos + CDC, excluding PROS tasks): 0.5-2 ms;
- smart port latency: 0-1 ms;
- Pi cycle work: 0.2-1.5 ms;
- tty: 0-0.5 ms;
- radio hop: 10-50 ms (placeholder only).

## 3. Conflict with the ~25 ms baseline, with numbers

- The baseline is Pi-authoritative, synthetic, loopback, Windows host. Pose age at receipt was
  p50 24.9 / p95 35.5 / max 51.3 [measured-host, final-ab-report.md]. It is before the browser
  frame (+2.1 p50) and before smoothing (+up to 33 at 30 Hz).
- The inherent Brain-authority cost is small: the reply leg, ~4 ms (USB, assumed) or ~8 ms
  (RS-485) mean.
- Combined topology, current link, Pi relay: ~36-37 ms mean and ~46-49 ms p95 at receipt
  [estimate]. The real-code probe gives 32-33 mean and 41-42 p95 at the Pi drain, before the
  ~3 ms relay [measured-host]. With the default 10 Hz TELEMETRY on, the p95 at the Pi drain rises
  to 54-60 ms because polls get displaced.
  - Against the measured baseline: about +11-12 ms at p50 and +11-13 ms at p95 (TELEMETRY off).
  - Against the probe's Pi 30 Hz true source age (17.1 mean, 26.8 p95): about +18 mean and +18 p95.
    This is the like-for-like comparison, because both are true ages.
- Where the extra ~18 ms mean over the same-model Pi path goes (model means):

  | Cause | ms | Class |
  |---|---|---|
  | Reply leg | ~4-8 | inherent |
  | Waiting for the next poll slot to piggyback | ~8 | avoidable only with an extra transaction, which costs polls |
  | Pi 100 Hz batching of the Brain port | ~5 | avoidable with a Pi reader on arrival |
  | Inspection 5 ms feed poll | ~2.5 | avoidable with an event-driven relay |
  | Request leg | ~3 | - |

  These sum to more than 18, because the Pi path also has its own ~0-33 ms state-phase term.
- Even with arrival reads and an event-driven relay, the piggyback path stays ~34 mean / ~43 p95
  (USB). Brain authority over the Pi relay therefore does not meet the ~25 ms baseline. It meets it
  approximately only on direct Brain USB (standalone).
- Standalone wireless: unknown. PROS's own 50 ms VEXnet limit on controller screen updates suggests
  a radio hop of tens of ms [inference]. The bench must measure it before a budget.
- Smoothing lag grows with the source period (up to 100 ms at 10 Hz). With a low-rate radio stream,
  smoothing would hide latency unless its lag is reported separately (app.js:54, 497 already keep a
  smoothLag stat) or it is capped near 33 ms [inference].

Recommended budget to propose for review [estimate, to be replaced by bench numbers]:

| Topology | Mean, ms | p95, ms | Measured from -> to |
|---|---|---|---|
| Combined, Pi relay | <= 40 | <= 55 | measurement -> receiver receipt |
| Combined, Pi relay | <= 22 | <= 30 | Brain receipt -> receiver receipt |
| Standalone USB | <= 5 | <= 10 | Brain publish -> receiver receipt |
| Standalone radio | no budget | no budget | report measured age as a lower bound until benched |

## 4. Timestamps that must survive end to end

| Hop | Must carry | Today | Basis |
|---|---|---|---|
| Pico -> Pi | stamp_ms (Pico clock), boot_id / acq_epoch / imu_epoch | yes | [code] frames.h:32-47, 69-81 |
| Pi internal | measuredAt (device) and measuredAtHost (Pi steady ms since process start) + session | yes | [code] core/time.h:1-8, 17-27; core/host_clock.h; inspection_document.cpp:1106-1108 |
| Pi -> Brain | robot_age_ms (age at the reply cycle start) + pi_instance, session, odometry_epoch, anchor_revision, flags | age only; no absolute stamp, no measurement id | [code] frames.h:353-369; brain_link_publisher.cpp:287-290 |
| Brain internal | measurement interval on the Brain clock: [sent_at - age - floor, received_at - age], where floor is the 4a bias (>= 2.7, up to ~12.7 ms, unknown per run). The Pi cycle start lies between the request write and the reply receipt | not computed. LinkDriver uses age + RTT, which the probe shows can be up to ~9 ms too young | [code] client.h:165-172 (received_at, round_trip); link_driver.cpp:89; [inference] bound; [measured-host] latprobe |
| Brain -> receiver | Brain clock stamp of measurement (interval or +-RTT/2) and of publish; Brain program/session id (millis restarts per program run); per-stream sequence; pose source identity (Pi instance/session/epoch/anchor, or "user"); validity | TELEMETRY has stamp_ms only (assembly time, not measurement) | [code] frames.h:288; brain_imu_bench.h:13-18 |
| Pi relay | Pi arrival host_ms, labeled as relay time and never as measurement | received_host_ms exists for TELEMETRY | [code] docs/inspection.md:434-437 |
| Receiver | receipt time on the receiver clock, for mapping and transport stats only | - | - |

Dedupe gap: "avoid duplicate samples" needs a measurement identity on the wire. Without it, a
stationary robot's genuinely new samples look identical, and a value-based dedupe would freeze the
age. Two options: add a pose sequence or the low bits of measuredAtHost to the GET_STATE state
block (a localization wire change, the user's call), or let the Brain dedupe by overlapping
measurement intervals plus identical bytes [inference; fragile].

The 50 Hz poll against the 50 Hz Pico produces a phase that wanders slowly:
- within a run the age at poll is nearly constant, anywhere in ~3-23 ms;
- a poll delayed by TELEMETRY or a transfer re-phases the grid (client.cpp:837-839);
- the model shows <1% repeats with no telemetry;
- the real-code probe saw 0 same-measurement repeats in 3,400-4,000 states per case. With
  TELEMETRY on, the new-state interval reaches 40-50 ms at p95/max, so some Pico samples are
  skipped.

Bench: record robot_age_ms over several runs [inference].

## 5. Presenting age honestly across clocks

Clock domains:

| Clock | Counts from | Basis |
|---|---|---|
| Pico ms | Pico boot | - |
| Pi host ms | Pi process start (HostClock) | [code] host_clock.h |
| Brain | program start: pros::millis, or pros::micros in ProsLink::now | [code] pros_link.cpp:136 |
| Receiver | receiver process steady clock | - |
| Browser | performance.now() | - |

Mappings available or proposed:

| Pair | Method | Error | Status |
|---|---|---|---|
| Pico -> Pi | min offset over a 64-sample window, 8-sample warm-up | ages biased low by the transfer floor | [code] clock_sync.h |
| Pi -> browser | ping/pong; min-RTT of the last 20; offset = pong.host_ms - (t_send + t_recv)/2 | +-RTT/2; the browser never subtracts its clock from a Pi stamp | [code] spectaGATR/latency.js:10-84; docs/inspection.md:237-245 |
| Brain -> Pi | none today | Brain stamps only detect duplicates | [code] brain_imu_bench.h:13-18 |
| Brain -> Pi, proposed | each GET_STATE exchange is NTP-like: Brain sent_at / received_at vs Pi cycle time T1. The Brain reports (request_id, sent_at, received_at) in a later request; the Pi pairs it with its T1; min-RTT filter | +-RTT/2 (~5-7 ms) | [inference] |
| Brain -> Pi, exact alternative | if the Brain echoes the request_id of the state it accepted and the Pi keeps a small ring of sent replies, the Pi gives an exact Pi-clock age for unmodified states; a value mismatch is flagged | exact (Pi clock) | [inference] |
| Brain -> receiver, push only (USB direct, radio) | one-way min offset min(recv - brain_ms), as clock_sync.h does | age is a LOWER bound (true age >= shown). Label "age >= X ms" or "excl. link floor" | [inference] |
| Brain -> receiver, with a return path | ping as above | +-RTT/2; over the radio, RTT is likely tens of ms, so the bound is wide | vexide writes via the controller (channel 2, 224 B chunks); pros-cli user_fifo_write returns without sending [source]. The Brain's stdin is already consumed by the communiGATR USB rx task, so a second stdin reader conflicts [code] pros_usb_port.cpp:126-160 (see sibling pros-serial) |

Presentation rules:
- The displayed age comes from the source clock via its mapping, with the mapping's bound. It is
  never the receiver's receipt time.
- Brain and Pi stamps are never compared without a mapping.
- Unknown mapping means "unmeasured", never fresh. That is the current poseFreshness rule
  [code] smoothing.js:21-33.

## 6. Commands run here (host evidence)

```
export PATH=/c/msys64/ucrt64/bin:$PATH
# A/B reproduction, in-process, exact Pi-clock ages
$S/final-rel/tools/perf/navigatr_inspect_bench.exe \
    --config C:/GATR2-Electronics/pi/navigatr/config/demo/synthetic_field_demo.xml \
    --scenario fast --duration 12 --warmup 4 --port 18772 --out $L/bench_fast_30hz.json
#   source age @pub p50 24.00 p95 34.00 max 50.00; publish->recv p50 0.18; age at receipt p50 24.43
# same with --state-hz 60 --port 18773: source age p50 23, p95 34 (the state rate is not the cause)

# standalone binary and read-only probes (L = $S/spectu/latency-chain)
$S/final-rel/navigatr.exe --config-file .../synthetic_field_demo.xml --inspect-port 18774
py -3 $L/ws_probe.py 18774 15 $L/ws1.json    # state msgs: age p50 24, p95 35, mean 24.2; interval p50 31.0 ms
py -3 $L/snap_probe.py 18774 10 $L/snap2.json # random-time age: mean 16.5, p50 16, p95 32
# earlier run, port 18771: age at the last estimation cycle p50 10, p95 25, mean 10.9; cycle work 0.13-0.18 ms
py -3 $L/chain_model.py [as-built|separate|fastread+event|separate+fastread+event]  # -> chain_model*.out
```

The processes were stopped by PID. latprobe.exe was run from `$L/build`, which already existed:
an earlier pass of this topic built it into this scratch folder. I did not rebuild it. It links
final-navi libs and the repo config at NAVIGATR_CONFIG_DIR.

```
PATH=/c/msys64/ucrt64/bin:$PATH $L/build/latprobe.exe 20 4 0 2000   # -> rerun_usb_tel0.txt
PATH=... $L/build/latprobe.exe 20 4 1 2000                           # -> rerun_usb_tel1.txt
LATPROBE_RS485=1 $L/build/latprobe.exe 20 4 {0,1} 0                  # -> rerun_rs485_tel{0,1}.txt
```

Finding on the baseline:
- viz-handoff/final_report.md calls the ~25 ms "measurement to localization publication". The code
  and data disagree: the pose at the localization cycle is ~11 ms old (mean).
- State messages are built ~13 ms later on average than their pose's cycle. Random-time documents
  are ~16.5 ms old against 24.2 for state messages.
- The state interval p50 is 31.0 ms instead of 33.3, which suggests wakeups quantized to a timer
  tick. Likely cause: Windows timer quantization with the inspection thread and the estimation
  worker waking on shared ticks [inference]. Verify on the Pi, where the baseline may be closer to
  ~16-18 ms.

## 7. External sources (quoted)

- pros-cli, V5WirelessPort.read: `self.buffer.extend(self.device.user_fifo_read())`, no sleep
  (https://raw.githubusercontent.com/purduesigbots/pros-cli/develop/pros/serial/ports/v5_wireless_port.py).
- pros-cli, user_fifo_read: `tx_payload = struct.pack("<2B", self.channel_map['download'], 0x40)`
  and `self._txrx_ext_packet(0x27, tx_payload, 0, check_length=False)`. user_fifo_write:
  `# Not currently implemented` / `return`. DownloadChannel: `ft_transfer_channel('download')` with
  `time.sleep(.25)` polls, and back to `'pit'` on exit
  (https://raw.githubusercontent.com/purduesigbots/pros-cli/develop/pros/serial/devices/vex/v5_device.py).
- pros-cli terminal: `elif is_v5_user_joystick: ... ser = V5WirelessPort(port)`; the device is
  `V5UserDevice` (COBS, topics sout/serr) unless `--raw`. The reader loop is
  `data = self.device.read()` with no sleep
  (https://raw.githubusercontent.com/purduesigbots/pros-cli/develop/pros/cli/terminal.py,
  .../pros/serial/terminal/terminal.py, .../pros/serial/devices/vex/v5_user_device.py).
- vex-v5-serial: the controller read is `UserDataPacket { channel: 1, write: None },
  Duration::from_millis(100)`. The write is `split_at(min(224, len))` on `channel: 2`
  (https://raw.githubusercontent.com/vexide/vex-v5-serial/main/packages/vex-v5-serial/src/serial.rs).
- cargo-v5 terminal: `sleep(Duration::from_millis(10)).await;` after each read/write select
  (https://github.com/vexide/cargo-v5/blob/main/src/commands/terminal.rs).
- vexide serial deep dive: over a controller, "stdio output has to be received with a
  UserFifoPacket" (https://vexide.dev/blog/posts/serial-deep-dive/).
- PROS system daemon: `task_delay_until(&time, 2)`, `vexBackgroundProcessing()`,
  `ser_output_flush()`, priority `TASK_PRIORITY_MAX - 2`
  (https://raw.githubusercontent.com/purduesigbots/pros/develop/src/system/system_daemon.c).
- PROS serial driver: stdout goes through a stream buffer; `ser_output_flush` writes
  `vexSerialWriteFree(1)` bytes with `vexSerialWriteBuffer(1, ...)`; `VEX_SERIAL_BUFFER_SIZE 2047`.
  ser_read_r loops on `inp_buffer_read(0)` until at least one character arrives
  (https://raw.githubusercontent.com/purduesigbots/pros/develop/src/system/dev/ser_driver.c).
- PROS serial daemon: `vexSerialReadChar(1)` with `task_delay(1)` between attempts, priority
  `TASK_PRIORITY_MIN + 1`, `INP_BUFFER_SIZE 0x1000`
  (https://raw.githubusercontent.com/purduesigbots/pros/develop/src/system/dev/ser_daemon.c).
  The source is the develop branch; the repo targets kernel 4.2.2 (pros_usb_port.cpp:129), so line
  details may differ.
- PROS Controller API: "updates faster than 10ms when on a wired connection or 50ms over Vexnet will
  not be applied to the controller" (set_text, print, clear_line, rumble)
  (https://pros.cs.purdue.edu/v5/api/cpp/misc.html).
- VEX: the Download channel "allows for higher data rates"; Pit is slower
  (https://kb.vex.com/hc/en-us/articles/21410932256660-Understanding-the-User-Settings-for-VEX-VS-Code-Extension).
  No official latency or bandwidth figures were found.

## 8. Doc/code notes

- final_report.md section 2 wording: see section 6. "Source age" includes inspection timing, not
  just localization.
- pros_usb_port.cpp:139 sleeps 5 ms only after a read returns <= 0. PROS ser_read_r spins on
  inp_buffer_read(0) (1-tick waits) until a character arrives, so the rx task normally blocks in
  read and the 5 ms path is rare [source + inference].
- Brain attitude over TELEMETRY is timed by Pi arrival, as the final report says
  [code] brain_imu_bench.h:13-16.

## Open questions / needs bench test

1. Real GET_STATE RTT and reply-leg time on V5 USB (VEXos USB and PROS daemons) and on RS-485
   (smart port TX/RX latency). Log client round_trip plus Pi read/write stamps together.
2. The robot_age_ms distribution on hardware over several runs (phase lock between the 50 Hz poll
   and the 50 Hz Pico), and how often polls repeat or skip a Pico sample.
3. Controller radio: latency and throughput of Brain stdout -> controller -> ext 0x27 / UserFifo
   reads. Pit vs download channel. Whether switching to the download channel affects driving or
   competition use. Whether the controller buffers Brain output or relays each read.
4. Whether any usable Brain <-> receiver return path exists over the radio for ping-offset without
   a second stdin consumer.
5. Pi-side baseline on Linux: does the ~24 ms state-message age drop toward ~16-18 ms without the
   Windows timer effect?
6. Pi estimation-cycle time on the Pi 4/5 (the reply guard assumes 1 ms, the model 0.2-1.5), and
   the estimation-worker stall from RS-485 reply airtime.
7. Wi-Fi / SSH tunnel delay from the Pi to the laptop browser, which adds to every Pi-relayed
   stream.
8. Display latency after rAF (compositor, scanout), which no current harness measures.
9. User decision: accept roughly +10-15 ms (combined) for Brain authority, or approve link changes:
   - a Pi reader on arrival;
   - an event-driven relay;
   - a measurement id plus a piggyback field in GET_STATE.

   Each is a localization-link wire or timing change.
