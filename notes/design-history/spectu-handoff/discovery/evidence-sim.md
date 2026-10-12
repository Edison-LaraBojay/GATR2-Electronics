# Discovery: host-simulation evidence of what TELEMETRY / viz traffic costs the Brain link

Date 2026-09-28. Repo C:/GATR2-Electronics at ae457ee (clean), read-only. No hardware.
Every number below is [measured-host] = host SIMULATION (virtual clock), not the V5, not the Pi.
Labels: [code] file:line verified, [measured-host] command given, [inference], [unknown].

Scratch dir S = C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/spectu/evidence-sim
- My files: CMakeLists.txt, e2e_sweep.cpp, fake_numbers.cpp, aggregate.py, build-v2/, logs/v2-*, logs/step1_*.
- NOTE: S already held files from an earlier, unfinished run of this task (src/, runs/, patch/,
  linkrig_sweep.cpp, patch2.py, build/, logs/*.txt, 12:12-12:58). My CMakeLists.txt replaced the one
  that was there; nothing else of it was touched or used as evidence.

## 0. Headline numbers (real Pi runtime harness, Pi loop 100 Hz, Pi phase not locked to the Brain)

| TM period | polls/s (off = 50.0) | TM delivered/s (reported/s) | link time per TM | held-state age mean/p95/max ms (off: 12.9/22.5/33.2, USB 2 ms) | report -> Pi mean/max ms |
|---|---|---|---|---|---|
| 200 ms | 45.9-46.4 (-7 to -8%) | 3.6-4.2 (5) | 17-20 ms (54 B); 30 ms (>=80 B RS-485) | 14.4/30.2/44.2 | 93-114 / 204-213 |
| 100 ms | 43.1-43.9 (-12 to -14%); RS-485 >=80 B: 39.7 (-20.5%) | 6.7-7.0 (10) | same | 15.4/35.3/44.3 | 57-66 / 104-114 |
| 50 ms | 37.0-38.2 (-24 to -26%); RS-485 >=80 B: 32.9 (-34%) | 11.3-13.3 (20) | same | 17.6/39.3/44.3 | 33-43 / 55-64 |

- Pi measurement -> Brain receipt (state age at receipt) is NOT changed by telemetry: 3.0 ms (USB 2 ms
  line), 4.0 (3 ms), 6.0 (5 ms), 7.2 (RS-485 115200). Telemetry costs the Brain freshness only through
  fewer polls: longer holds (poll gap p95 20 -> 40 ms, max 22-30 -> 42-58 ms).
- 0 timeouts, 0 drops, 0 refused in every sweep cell; at most 1 non-poll request between two polls.
- The "no cost at the default 2 ms line" result in the repo docs is a phase-lock artifact of the test
  rig (section 4.3).

## 1. Step 1: existing tests, run as built

| Command (PATH=/c/msys64/ucrt64/bin first) | Result | Log |
|---|---|---|
| final-brain/communiGATR/communigatr_tests.exe --gtest_filter='*Telemetry*:ClientTiming.*' | 35/35 PASSED | S/logs/step1_communigatr.log |
| final-navi/tests/brain_link_e2e_tests.exe | 10/10 PASSED (TelemetryKeepsFlowingOnASlowerLine 504 ms) | S/logs/step1_e2e.log |

The tests assert thresholds only and print no numbers [code client_telemetry_gtest.cpp:259-268,
494-501, 597-636; brain_link_e2e_gtest.cpp:796-838]. Numbers were extracted with a copy of the test
harness code plus printf (S/fake_numbers.cpp; harness functions copied from
client_telemetry_gtest.cpp:118-131, 395-464, 524-595) [measured-host: S/build-v2/fake_numbers.exe ->
S/logs/v2-fake_numbers.txt]:

Host fakes (communiGATR/sim: FakePi answers at once; FakeUsb 2+4+2 ms fixed, no byte time
fake_usb.h:148-152; FakeBus 115200 byte-timed, reply 2 ms fake_bus.h:19-26):

| Test (what it asserts) | Rs485 | Usb |
|---|---|---|
| Telemetry.StatePollsKeepTheirRate (with >= 85% of without), 3 s | 145 -> 133 polls (-8.3%), 30 TM | 145 -> 137 (-5.5%), 30 TM |
| same harness, 5 s, report every 100 ms (programs' cadence) | 243 -> 220 (-9.5%), 50 TM on time, worst wait 116 ms | 243 -> 227 (-6.6%), 50 TM, 114 ms |
| OnePerPeriodOnAGrid: report every 1 ms, 3 s | 30 sent, 2970 replaced | 30 sent, 2970 replaced |
| SlowTelemetry (bus reply 8 ms / USB turnaround 11 ms) LateButNeverStarved, 5 s | 32 sent (6.4/s), 32 overdue, worst wait 171 ms, best gap 154 ms, 0 drop/timeout | 32 (6.4/s), 32 overdue, 166 ms, 154 ms |
| SlowTelemetry.StatePollsKeepMostOfTheirRate (>= 80%), 3 s | 139 -> 121 (-12.9%) | 141 -> 127 (-9.9%) |
| SlowTelemetry.SharesTheLinkWithAWaitingTransfer, 5 s | 30 TM, 41 paths, 165 polls, worst wait 206 ms | 30, 41, 166, 200 ms |
| TelemetryBus default, 3 s | 145 -> 130 (-10.3%); worst poll gap 21 -> 32 ms; TM exchange 9.75 ms; never with poll due; 0 overdue | - |
| TelemetryBus slow (reply 8 ms), 3 s | 139 -> 120 (-13.7%); worst poll gap 22 -> 44 ms; 19 TM all overdue; TM exchange 15.75 ms | - |

Real Pi runtime, the e2e test's own setup (1 ms tick, Pi exactly on 10 ms multiples, 10 s after
ready) [measured-host: S/build-v2/e2e_sweep.exe test -> S/logs/v2-e2e-test.csv]:

| line each way | TM | polls/s | TM sent/s | overdue | replaced | poll gap max | RTT mean/max | report -> Pi mean/p95/max |
|---|---|---|---|---|---|---|---|---|
| 2 ms | off | 49.7 | - | - | - | 28 ms | 6.7/12 ms | - |
| 2 ms | 100 ms | 49.0 | 10.0 | 3 | 0 | 30 | 4.7/12 | 75.8/87/97 ms |
| 3 ms (= TelemetryKeepsFlowingOnASlowerLine) | off | 48.9 | - | - | - | 40 | 10.8/16 | - |
| 3 ms (same) | 100 ms | 42.5 (-13.1%) | 6.5 (65/10 s; test needs >= 50) | 58 | 35 | 40 | 14.0/14 | 54.7/95/95; max gap at Pi 180 ms (test <= 250) |

Agrees with W5_fix.md table ("3 or 4 ms: 63/100, 489 to 437") and sibling brain-link.md Probe B
(42.6 polls/s, 6.4 TM/s).

## 2. The scratch experiment (S/e2e_sweep.cpp)

Real code: communiGATR Client/LinkDriver/NG1 codec, actuGATR Motion/Drive, investiGATR planner
(compiled from brain/ by path) against navigatr_core built FRESH from pi/naviGATR at ae457ee into
S/build-v2/navi (add_subdirectory, NAVIGATR_BUILD_TESTS OFF). Rig copied from
brain_link_e2e_gtest.cpp:53-486: config override/brain_profile_usb.xml or brain_profile_rs485.xml,
Brain link and Pico UART replaced by memory links, Pico frames fed at each Pi step, Brain poll task
2 ms (as pros_link.h:44), control 10 ms, bench VEX IMU on every GET_STATE, robot placed and ready,
3 s settle (field map transfer), then a 20 s window.

Changes vs the test rig: tick 100 us (test: 1 ms); Pi loop phase 0/0.4/0.8/1.2/1.6 ms vs the Brain
2 ms grid, plus Pi step jitter uniform 0-0.5 ms [inference: stands in for Linux sleep_until jitter,
app/main.cpp:164]; every cell = 5 phases, mean (min-max). A jitter-0 sweep was also run
(S/logs/v2-sweep-j0.*); conclusions are the same except where noted.

Line models:
| name | model | basis |
|---|---|---|
| usb2 | NG1 lines, 2 ms each way, no byte time | = test UsbWire default kLineUs 2000 (brain_link_e2e_gtest.cpp:57,140-228) |
| usb3 | same, 3 ms | = TelemetryKeepsFlowingOnASlowerLine (:797) |
| usb5 | same, 5 ms | sensitivity [inference] |
| rs485 | raw frames, 10 bits/byte at 115200 (86.8 us/B) half duplex; Pi reply starts 1 ms after the Pi step that read the request; 0.2 ms release | turnaround_guard_us 1000 and window 40 ms (brain_link_commands.cpp:189-190,373-374); MemoryLink never waits (serial_links.h:123-126), so the guard is modeled in the wire |

Payload size: TELEMETRY is fixed 54 B (frames.h:268-310, kTelemetryBodyLen 54). Without codec changes a
larger body was modeled as extra AIRTIME on each TELEMETRY request (bytes delivered to the Pi are the
real 68-byte frame, timed as 68+N bytes). Bodies: 54 (+0), 80 (+26), 114 (+60, the largest single frame:
kMaxFrameLen 128, frames.h:19; payload max 122, request header 8, frame_codec.h:21-22), 128 (+74,
HYPOTHETICAL: a 142-byte frame exceeds kMaxFrameLen and the FrameReader buffer, frame_codec.h:181; it
would need a codec change or two frames). USB payload was NOT swept: both the repo's UsbWire and FakeUsb
deliver a line after a fixed delay regardless of length, so the model predicts no size effect by
construction; the real V5 USB character rate is [unknown].

Metrics: polls/s = GET_STATE writes; TM sent/s = ClientStats::telemetry_reports; TM acked/s = TELEMETRY
Ok replies read by the Brain (= delivered to and processed by the Pi; lossless wires); overdue/replaced/
dropped/timeouts = ClientStats deltas (client.h:223-227) per 20 s; state age at receipt = Brain receipt
(state().received_at) - Pi measurement, where Pi measurement = sim time of the Pi step that built the
reply minus its robot_age_ms (brain_link_publisher.cpp:287-290); held-state age = time-average over
every tick of (now - measurement time of the state the Brain currently holds); report -> Pi = Pi step
that processed the TELEMETRY minus the app's report time.

Build/run [measured-host]:
```
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake -S $S -B $S/build-v2 -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=<scratchpad>/nav-base/_deps/googletest-src
cmake --build $S/build-v2 -j16            # 0 warnings (-Wall -Wextra)
cd $S/build-v2
./fake_numbers.exe                       > ../logs/v2-fake_numbers.txt
TRACE_TM=1 ./e2e_sweep.exe test          > ../logs/v2-e2e-test.csv 2> ../logs/v2-e2e-test-trace.txt
./e2e_sweep.exe sweep 500                > ../logs/v2-sweep-j500.csv   # 125 runs, 68 s wall
./e2e_sweep.exe sweep 0                  > ../logs/v2-sweep-j0.csv
./e2e_sweep.exe piggy 500                > ../logs/v2-piggy-j500.csv
./e2e_sweep.exe pirate                   > ../logs/v2-pirate.csv
./e2e_sweep.exe one usb2 0 <0|100> <phase_us> 0   # phase scan -> ../logs/v2-phase-scan-usb2.csv
py -3 ../aggregate.py ../logs/v2-sweep-j500.csv > ../logs/v2-sweep-j500.md
```

## 3. Sweep results [measured-host, S/logs/v2-sweep-j500.md; 5 phases x 20 s per row]

| line | body B | TM | polls/s mean (min-max) | vs off | TM sent/s | TM acked/s | overdue/20s | replaced/20s | drop | tmo | poll gap p95/max ms | age at receipt mean/p95/max ms | held age mean/p95/max ms | RTT mean/max ms | report->Pi mean/p95/max ms | bus busy % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| usb2 | 54 | off | 50.0 (50.0-50.0) |  | 0 | 0 | 0 | 0 | 0 | 0 | 20/22 | 3.0/3.9/3.9 | 12.9/22.5/33.2 | 10.7/16 | - | - |
| usb2 | 54 | 200 | 46.4 (46.2-47.2) | -7.2% | 4.2 | 4.2 | 50 | 17 | 0 | 0 | 40/40 | 3.0/3.9/3.9 | 14.4/30.2/44.2 | 13.3/16 | 109/208/208 | - |
| usb2 | 54 | 100 | 43.9 (43.8-44.6) | -12.1% | 7.0 | 7.0 | 94 | 59 | 0 | 0 | 40/42 | 3.0/3.9/3.9 | 15.4/35.3/44.3 | 13.3/16 | 63/108/108 | - |
| usb2 | 54 | 50 | 38.2 (37.5-39.2) | -23.6% | 13.3 | 13.3 | 202 | 133 | 0 | 0 | 40/42 | 3.0/3.9/3.9 | 17.6/39.3/44.3 | 13.2/16 | 35/58/58 | - |
| usb3 | 54 | off | 50.0 |  | 0 | 0 | 0 | 0 | 0 | 0 | 20/22 | 4.0/4.9/4.9 | 13.9/23.5/32.8 | 9.6/16 | - | - |
| usb3 | 54 | 200 | 46.0 (45.0-46.2) | -8.1% | 4.0 | 4.0 | 60 | 18 | 0 | 0 | 40/40 | 3.9/4.9/4.9 | 15.5/33.9/45.1 | 13.8/16 | 93/196/205 | - |
| usb3 | 54 | 100 | 43.3 (41.8-43.7) | -13.4% | 6.7 | 6.7 | 106 | 66 | 0 | 0 | 40/42 | 3.9/4.9/4.9 | 16.6/37.9/45.1 | 13.9/16 | 59/105/105 | - |
| usb3 | 54 | 50 | 37.1 (35.9-37.5) | -25.7% | 12.9 | 12.9 | 222 | 143 | 0 | 0 | 40/42 | 3.9/4.9/4.9 | 19.0/40.5/45.1 | 13.9/16 | 36/57/57 | - |
| usb5 | 54 | off | 50.0 (49.9-50.0) |  | 0 | 0 | 0 | 0 | 0 | 0 | 20/26 | 5.9/6.9/6.9 | 15.9/25.5/36.7 | 13.3/20 | - | - |
| usb5 | 54 | 100 | 43.2 (41.5-43.7) | -13.4% | 6.7 | 6.7 | 105 | 65 | 0 | 0 | 40/42 | 6.0/6.9/6.9 | 18.6/40.0/47.1 | 13.9/20 | 57/105/105 | - |
| usb5 | 54 | 50 | 37.2 (35.9-37.5) | -25.6% | 12.8 | 12.8 | 222 | 143 | 0 | 0 | 40/42 | 5.9/6.9/6.9 | 21.0/42.5/47.1 | 13.9/16 | 34/55/55 | - |
| rs485 | 54 | off | 50.0 |  | 0 | 0 | 0 | 0 | 0 | 0 | 20/26 | 7.2/8.1/8.1 | 17.1/26.7/37.9 | 12.2/20 | - | 35.6 |
| rs485 | 54 | 200 | 45.9 (45.1-46.1) | -8.0% | 4.0 | 4.0 | 60 | 19 | 0 | 0 | 38/46 | 7.2/8.1/8.1 | 18.7/36.8/58.0 | 14.2/18 | 96/203/204 | 35.8 |
| rs485 | 54 | 100 | 43.1 (40.9-43.7) | -13.7% | 6.8 | 6.8 | 104 | 63 | 0 | 0 | 38/48 | 7.2/8.1/8.1 | 19.9/42.3/58.0 | 14.5/18 | 62/104/104 | 35.9 |
| rs485 | 54 | 50 | 37.0 (35.1-37.5) | -25.9% | 12.6 | 12.6 | 228 | 147 | 0 | 0 | 46/48 | 7.2/8.1/8.1 | 22.4/46.3/58.0 | 15.2/18 | 33/62/62 | 35.8 |
| rs485 | 80 | 200 | 44.6 (42.6-45.0) | -10.8% | 3.6 | 3.6 | 50 | 27 | 0 | 0 | 46/48 | 7.2/8.1/8.1 | 19.8/47.1/58.3 | 14.2/18 | 104/204/213 | 35.3 |
| rs485 | 80 | 100 | 39.7 (37.1-40.4) | -20.5% | 6.8 | 6.8 | 108 | 63 | 0 | 0 | 48/48 | 7.2/8.1/8.1 | 22.2/51.5/58.3 | 14.5/18 | 66/113/114 | 35.0 |
| rs485 | 80 | 50 | 32.9 (31.5-33.3) | -34.1% | 11.4 | 11.3 | 204 | 172 | 0 | 0 | 48/48 | 7.1/8.1/8.1 | 25.6/53.2/58.3 | 15.2/18 | 43/64/64 | 34.7 |
| rs485 | 114 | 200/100/50 | identical to 80 B to 0.1 | | | | | | | | | | | | | 36.4/37.0/38.0 |
| rs485 | 128* | 200/100/50 | identical to 80 B to 0.1 | | | | | | | | | | | | | 36.8/37.8/39.4 |
(usb5 200 ms row: 46.0, -8.0%, 4.0 TM/s; see the .md.) *128 B is airtime-only, not a real frame.

Derived link time consumed per TELEMETRY exchange, (1000 ms - polls*20 ms)/TM sent [measured-host,
derived]: usb2 17.2-17.7 ms; usb3/usb5 20.0 ms; RS-485 54 B 20.0-20.6 ms; RS-485 80/114/128 B 30.0 ms.
Reports replaced before sending (latest wins): ~0.8-1.3/s of 5, ~3.0-3.3/s of 10, ~6.7-8.6/s of 20.

### 3.1 Readings
- One TELEMETRY exchange costs about one whole poll period (17-20 ms), not "10 to 15 ms": the Pi answers
  only once per 10 ms loop (brain_link_commands.cpp:270-376 run once per cycle; Loop rate_hz 100,
  brain_profile_usb.xml:18, brain_profile_rs485.xml:19), so GET_STATE RTT is 10-13 ms mean, 16-20 max,
  and RTT + 5 ms request_gap (client.h:48) leaves no idle window inside the 20 ms state_period
  (client.h:49). Telemetry therefore rides the overdue path (client.cpp:966-975) at most once per 1.5
  periods: 6.7-7.0/s delivered at a 100 ms period, not 10.
- Cost scales with the TM rate almost linearly and is nearly line-independent between 2 and 5 ms USB
  and RS-485 54 B: 7-8% at 200 ms, 12-14% at 100 ms, 24-26% at 50 ms.
- Payload on RS-485 is a STEP, not a slope: 80, 114 and (hypothetical) 128 B give identical results,
  because the longer request misses the next Pi step and waits one more 10 ms cycle (30 ms per exchange).
  RS-485 bytes are not the bottleneck: bus 35-39% busy. [measured-host; step explanation = inference from
  airtime arithmetic: 68 B = 5.9 ms, 94 B = 8.2 ms, 128 B = 11.1 ms, 142 B = 12.3 ms at 86.8 us/B]
- State age at receipt does not change with telemetry (the reply path is untouched); only the hold
  between updates grows (poll gap p95 20 -> 38-48 ms).
- VEX IMU samples ride GET_STATE (brain_link_e2e_gtest.cpp:659-660 comment; bench_imu in statePoll,
  client.cpp:1058-1067), so the Brain-IMU sample stream to the Pi loses the same fraction and gets the
  same 40-58 ms holes [code + measured-host]. Effect on pose accuracy: not measured (robot stationary).

### 3.2 Telemetry latency to the Pi (report -> Pi processing)
Mean 57-66 ms at 100 ms period, p95/max 104-114 ms; 93-114 ms mean at 200 ms (max 213); 33-43 ms at 50 ms.
Trace [measured-host: TRACE_TM=1 e2e_sweep.exe one usb2 0 100 400 500; S/logs/v2-e2e-test-trace.txt]:
reports at 100 ms steps go out at 16-96 ms after being made. Cause [code + trace]: the client keeps its
own send grid (next_telemetry_, client.cpp:61,840-846) independent of when the app reports; an on-time
send needs the poll not due (client.cpp:1029-1035), which rarely holds here, so sends take the overdue
path at grid + period/2 (:971-975), and a send more than half a period late re-bases the grid to
now + period (:845). Reports made just after a send wait most of a period; some are replaced unsent.
In the phase-locked rig at usb2 phase 0 the same config gives 18 ms mean (S/logs/v2-phase-scan-usb2.csv).
The Pi passes the Brain stamp_ms and its own `now` to acceptAttitude (brain_link_commands.cpp:532-540);
final_report.md:150 says the attitude is timed by Pi arrival, which would make it 57-66 ms (mean) older
than shown at a 100 ms period [inference from the measured lag; Pi display path not examined here].

## 4. Other experiments

### 4.1 Piggyback cost: extra bytes on every GET_STATE request, TM off, RS-485 [measured-host: e2e_sweep.exe piggy 500]
| GET_STATE request | polls/s mean (min-max) | poll gap p95/max ms | RTT mean/max ms | age at receipt ms | held age mean/p95/max ms | bus busy % |
|---|---|---|---|---|---|---|
| 23 B (now) | 50.0 | 20/26 | 12.2/20 | 7.2 | 17.1/26.7/37.9 | 35.6 |
| 31 B (+8) | 50.0 | 20/26 | 12.3/20 | 7.2 | 17.1/26.7/37.9 | 39.0 |
| 39 B (+16) | 50.0 | 20/26 | 12.8/20 | 7.2 | 17.1/26.7/37.9 | 42.5 |
| 47 B (+24) | 50.0 (49.9-50.0) | 20/28 | 13.4/22 | 7.2 | 17.1/26.7/38.3 | 46.0 |
| 55 B (+32) | 49.9 | 20/28 | 13.5/22 | 7.2 | 17.1/26.7/38.3 | 49.4 |
| 71 B (+48) | 46.5 (39.9-49.9) | 30/30 | 15.7/24 | 7.1 | 18.1/34.5/38.3 | 52.5 |
| 87 B (+64) | 35.4 (33.3-39.9) | 30/30 | 22.4/24 | 7.1 | 21.5/36.2/38.3 | 44.9 |
| 103 B (+80) | 33.3 | 32/32 | 24.0/26 | 7.2 | 22.1/36.2/38.3 | 46.9 |
Up to about +32 B per poll costs no polls on the modeled RS-485 with a 100 Hz Pi (airtime fits inside the
Pi-cycle quantization); from about +48 B the exchange crosses a Pi cycle and polls drop to 33-47/s. On the
USB model any size is free by construction [unknown on hardware]. A compact Brain pose record (e.g.
x,y i32, heading i16, stamp u32, flags u8 = 15 B) is inside the free range [inference].

### 4.2 Pi loop rate sensitivity (config Loop rate_hz overridden in the sim XML) [measured-host: e2e_sweep.exe pirate; S/logs/v2-pirate.md]
| line | Pi loop | TM | polls/s | TM delivered/s | RTT mean/max | poll gap p95/max | report->Pi mean/max |
|---|---|---|---|---|---|---|---|
| usb2 | 100 Hz | off / 100 ms | 50.0 / 43.9 (-12.1%) | 7.0 | 10.7/16 / 13.3/16 | 20/22 / 40/42 | 63/108 |
| usb2 | 200 Hz | off / 100 ms | 50.0 / 45.4 (-9.2%) | 10.0 | 7.8/10 / 8.6/10 | 20/22 / 30/32 | 28/42 |
| usb3 | 200 Hz | off / 100 ms | 50.0 / 45.0 (-9.9%) | 10.0 | 10.0/12 / 9.7/12 | 20/22 / 30/32 | 34/46 |
| rs485 | 200 Hz | off / 100 ms | 50.0 / 43.2 (-13.6%) | 9.0 | 11.5/14 / 12.6/14 | 20/22 / 36/40 | 52/108 |
The Pi's answer timing (loop period) is the largest single lever on telemetry cost and lag on USB.

### 4.3 Phase lock of the repo rig [measured-host: S/logs/v2-phase-scan-usb2.csv, jitter 0]
usb2, TM 100 ms, Pi phase vs Brain 2 ms grid: 0 us -> 49.9 polls/s, 10.0 TM/s, 0 overdue, report->Pi 18 ms;
100, 200, 500, 1000, 1500, 1900 us -> 43.75 polls/s, 6.3 TM/s, 123 overdue/20 s, report->Pi 66-68 ms.
The repo e2e rig steps the Pi exactly on 10 ms multiples and polls the Brain on 2 ms multiples
(brain_link_e2e_gtest.cpp:381,389), which is the only phase where the 2 ms line shows no cost.

## 5. Step 3: inherent delay Pi measurement -> Brain, vs avoidable queueing (TM off, jitter 500) [measured-host]

| component (ms) | usb2 | usb3 | usb5 | rs485 | notes |
|---|---|---|---|---|---|
| a. Pi measurement -> Pi reply build (robot_age_ms) | 0 | 0 | 0 | 0 | sim artifact: Pico frame fed right before the Pi step (brain_link_e2e_gtest.cpp:381-386) |
| b. request write -> Pi build (uplink + wait for Pi loop) | 7.8 mean / 12.1 max | 5.6/11.1 | 7.4/15.0 | 5.0/12.0 | the loop wait lengthens RTT but does not age the data: the Pi answers with that cycle's state |
| c. Pi build -> Brain receipt (downlink + Brain 2 ms poll grid) | 2.95 / 3.9 | 3.95 / 4.9 | 5.94 / 6.9 | 7.15 / 8.1 | rs485 = 1 ms guard + 59 B reply airtime 5.1 ms + grid |
| d. = state age at Brain receipt (a + c) | 3.0 mean, p95 3.9 | 4.0 / 4.9 | 5.9 / 6.9 | 7.2 / 8.1 | inherent for Brain-authoritative pose |
| e. GET_STATE RTT (b + c) | 10.7 / 16 | 9.6 / 16 | 13.3 / 20 | 12.2 / 20 | |
| f. receipt interval (poll cadence) | 20.0 mean, p95 22, max 30 | 20/22/28 | 20/22/30 | 20/22/30 | state_period 20 ms (client.h:49); Brain sees every 2nd Pi cycle |
| g. held-state age (d + time since receipt), time-avg | 12.9 / p95 22.5 / max 33.2 | 13.9/23.5/32.8 | 15.9/25.5/36.7 | 17.1/26.7/37.9 | ~ d + f/2 |

Avoidable queueing added by TELEMETRY (same line, same phases): held-state age mean +1.5/+2.5/+4.7 ms,
p95 +7.7/+12.8/+16.8 ms, max +11 ms at 200/100/50 ms on usb2; on RS-485 with >= 80 B bodies at 50 ms,
mean +8.5 ms, p95 +26.5 ms, max +20 ms. d is unchanged in every cell. Brain-owned data sent to the Pi as
TELEMETRY waits a further 57-66 ms mean (100 ms period) before the Pi sees it (section 3.2), most of it
the client's send-grid wait (avoidable), about 10-14 ms of it the exchange itself.

On hardware a is not 0 [code + inference]: robot_age_ms = Pi cycle time - measuredAtHost
(brain_link_publisher.cpp:287-290), where measuredAtHost maps the Pico stamp through the MINIMUM observed
(receipt - device) offset over 64 samples (clock_sync.h:1-12, 38-63; motion_step.cpp:557-572) and Pico
receipt is stamped at the Pi cycle (pico_telemetry.cpp:290 applyPacket(s, context.now)). So robot_age
includes the loop pickup delay beyond the best case but EXCLUDES the minimum Pico->Pi UART/batching latency;
the true Pi-measurement -> Brain-receipt age is that floor + robot_age + c. The Brain-side PROS task
quantization (tx task idle 2 ms, pros_usb_port.cpp:22; System Daemon 2 ms flush and Serial Daemon 1 ms,
per sibling brain-link.md 1.2/1.4) is not modeled; it adds to b and c like a longer line (usb3/usb5 rows).

## 6. Doc/claim vs measurement

| Claim | Where | This evidence |
|---|---|---|
| real Pi runtime, 2 ms each way: 100 of 100 sent, "497 and 500 (no change)" | docs/communigatr.md:780 (table row), W5_fix.md | phase-lock artifact: any Pi phase offset >= 0.1 ms gives 63-70 of 100 and -12% polls (4.3, 3) |
| each exchange pushes the next poll back "about 10 to 15 ms" | brain/robot/gatr2_robot.h:56-58; docs/brain_setup.md:325-326; docs/communigatr.md:764-767 | true on fakes (worst poll gap 21 -> 32 ms); with the 100 Hz Pi loop 17-20 ms per 54 B exchange, 30 ms for >= 80 B on RS-485; poll gap p95 20 -> 40 ms |
| "6 to 11% fewer at the default timing" | docs/brain_setup.md:327-328; docs/communigatr.md:1116-1117; final_report.md:151 | fakes 5.5-10.3% (reproduced); real-Pi-loop harness 12-14% at 100 ms on every line incl. 2 ms |
| "about 6 per second instead of 10" only on slow links | docs/brain_setup.md:333-338; communigatr.md:1117-1118 | 6.7-7.0/s already at the default 2 ms line once the phase lock is broken |
| attitude "about one telemetry period plus link latency late" | viz final_report.md:150 | report -> Pi 57-66 ms mean, 104-114 ms max at 100 ms (0.6 period mean, ~1.1 max) |

## 7. Open questions / needs bench test
1. Real GET_STATE RTT and its spread on V5 USB and on RS-485 with the Pi at 100 Hz; the whole cost
   picture hinges on RTT + 5 ms gap vs the 20 ms state_period.
2. Real Pi loop jitter and phase drift vs the Brain 2 ms task (the sim shows a knife-edge at exact lock).
3. V5 USB character rate / per-line latency vs length (payload effect on USB is unmodeled here).
4. Pico -> Pi minimum latency (excluded from robot_age_ms) and real robot_age_ms on the robot.
5. Whether the client's independent telemetry grid (client.cpp:840-846) is intended; its 50-110 ms added
   lag matters for any Brain-published viz using the TELEMETRY rules.
6. Effect of the thinner VEX IMU sample stream (40-58 ms holes with telemetry) on pose accuracy while moving.
7. Whether a 200 Hz Pi loop (4.2) is affordable on the Pi CPU with the real pipeline.
