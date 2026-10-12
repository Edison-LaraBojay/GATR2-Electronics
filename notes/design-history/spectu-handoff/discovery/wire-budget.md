# spectuGATR discovery: wire sizes, field documents, link budget

Date 2026-09-28. Read-only on the repo (HEAD ae457ee, `git status` clean). Host only: nothing ran on a
Brain, Pi, controller or radio. Basis tags: [code] file:line, [source] URL, [measured-host] command
given, [inference], [unknown].

Scratch: $W = C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/spectu/wire-budget
- `wire_sizes.cpp` -> `wire_sizes.out`: encodes every brain link frame with the repo codec, NG1 line
  lengths with communiGATR `encodeUsbLine`, and builds the real override field documents with naviGATR
  `loadFieldMapFile` + `buildFieldMapDocument`. Compiled by g++ straight from repo sources.
- `budget_probe.cpp` -> `budget_probe.out`: communiGATR host rig (FakeUsb / FakeBus + FakePi, real
  Client) 10 s simulated per cell; TELEMETRY at other periods stands in for a separate viz exchange.
- `path_probe3.cpp` -> `path_probe3.out`: real override map doc -> FakePi -> Client/LinkDriver ->
  investigatr::Field -> GeometricPlanner, 3000 random start/goal pairs per model.
- `demo.xml` (copy of config/demo/synthetic_field_demo.xml, absolute paths, port 28763) run with
  final-rel/navigatr.exe; `hello.json`, `ws_sizes.py`, `json_estimate.out`, `json_pose.out`.
  The navigatr.exe instance was stopped afterwards (only one instance was running, PID 16972).
- External sources fetched to $W/ext (PROS kernel, vex-v5-serial).

Build/run recipe: `export PATH=/c/msys64/ucrt64/bin:$PATH; g++ -std=c++20 -O2 -I<repo> -I<repo>/brain/communiGATR/include -I<repo>/brain/communiGATR -I<repo>/brain/investiGATR/include -I<repo>/pi/navigatr/src -I<repo>/pi/navigatr/third_party <probe>.cpp <needed .cpp files>`

--------------------------------------------------------------------------------------------------
## 1. Brain link frame sizes and framing

Envelope: `AA 55 | type u8 | len u8 | payload | crc16` = 6 bytes; CRC-16/CCITT-FALSE over type, len
and payload; frame max 128, payload max 122 [code] translaGATR/frames.h:19,84-97;
translaGATR/frame_codec.h:17-26. Request header 8 (version, op, session u32, request_id u16), reply
header 13 (+ result u8, pi_instance u32) [code] frame_codec.h:22-24. One request outstanding, Brain is
the only initiator [code] frames.h:88-89.

USB: each frame is one line `"NG1:" + uppercase hex + "\n"` = 2n + 5 chars [code]
brain/communiGATR/src/usb_line.cpp:29; pi/naviGATR/src/impl/resources/pros_usb_link.cpp:279-291.
Line limit 516 chars [code] usb_line.h:25. No COBS: communiGATR disables PROS output COBS for the whole
program [code] brain/communiGATR/pros/pros_usb_port.cpp:195-199.

All rows [measured-host] `$W/wire_sizes.exe` (encoder output), cross-checked with the length tables
[code] frame_codec.cpp:319-382. RS-485 airtime = bytes x 86.8 us (115200 8N1) [inference].

| Op | Request B | Reply B (Ok) | NG1 req/rep chars | Exchange B | RS-485 airtime |
|---|---|---|---|---|---|
| HELLO | 18 | 23 | 41 / 51 | 41 | 3.6 ms |
| SET_POSE | 26 | 27 | 57 / 59 | 53 | 4.6 ms |
| GET_STATE | 23 | 59 (40-byte state) | 51 / 123 | 82 | 7.1 ms |
| GET_STATE error | 23 | 19 | 51 / 43 | 42 | 3.6 ms |
| PROFILE_WRITE | 23..128 (106 B data max) | 25 | 51..261 / 55 | 48..153 | 4.2..13.3 ms |
| PROFILE_APPLY | 20 | 26 | 45 / 57 | 46 | 4.0 ms |
| READ_DOC | 22 | 33..128 (96 B data max) | 49 / 71..261 | 55..150 | 4.8..13.0 ms |
| CONTROL | 16 | 22 | 37 / 49 | 38 | 3.3 ms |
| PATH_REPORT | 20 + 8n, n<=13 (max 124) | 19 | 45..253 / 43 | 39..143 | 3.4..12.4 ms |
| READ_WHEELS | 14 | 20 + 16n (2 wheels: 52) | 33 / 45..173 | 34..98 | 3.0..8.5 ms |
| TELEMETRY | 68 (54-byte fixed body) | 19 (header only) | 141 / 43 | 87 | 7.6 ms |

Robot profile document for this robot (2 wheels, 0 cameras) is 96 B = 1 PROFILE_WRITE chunk; max
240 B = 3 chunks [measured-host] wire_sizes.out.

Overhead: 6 B/frame binary (GET_STATE request 17 B payload -> 23 B frame). NG1 more than doubles it:
23 B -> 51 chars (+122%) [measured-host].

Body headroom: TELEMETRY request 68 B leaves 60 B below 128. But TELEMETRY and GET_STATE have fixed
lengths (min == max) and a v4 decoder rejects any other length [code] frame_codec.cpp:319-341,555-560;
frames.h:310. Any piggyback needs a new op or link version plus a HELLO-time capability check.

### Timing constants that set the real budget

| Item | Value | Where |
|---|---|---|
| GET_STATE poll period | 20 ms (50 Hz); apps do not override it | [code] client.h:49; locaGATR/src/main.cpp:53-72, operaGATR/src/main.cpp:120-139 set only transport, profile, bench_imu, telemetry_period |
| Gap after reply/timeout | 5 ms | [code] client.h:48 |
| Response timeout | 60 ms + byte_time beyond 85 B pair | [code] client.h:44-47 |
| TELEMETRY period | 100 ms, on | [code] brain/robot/gatr2_robot.h:60-61 |
| Field estimate re-read | at most every 0.5 s, only when estimate_id changes | [code] client.h:59; client.cpp:1260-1276 |
| Pi estimate snapshot | at most every 200 ms, only on change | [code] config/override/brain_profile_usb.xml:95, brain_profile_rs485.xml:96 |
| Brain link poll task | every 2 ms | [code] pros_link.h:44 |
| USB tx task idle / rx retry | 2 ms / 5 ms | [code] pros_usb_port.cpp:22-23 |
| Pi loop | 100 Hz; request drained per cycle, reply sent in that cycle | [code] brain_profile_usb.xml:18; brain_link_commands.h:16-23 |
| Pi reply window / turnaround guard | 40 ms / 1000 us | [code] brain_link_commands.cpp:189-190,373-374 |
| RS-485 baud | 115200 both sides (Brain kLinkBaud, Pi <Baud>) | [code] gatr2_robot.h:35; pros_link.h:42; brain_profile_rs485.xml:40-43 |
| RS-485 DE | Pi GPIO 6, driver held 174 us (2 chars) after TX empty, margin 2 ms | [code] brain_profile_rs485.xml:43; pi/.../transport/half_duplex.h:47-51 |
| Pi USB write bound | 5 ms | [code] pros_usb_link.cpp:89 |
| Pi USB read per call | up to 4 x 256 B while no frame decoded | [code] pros_usb_link.cpp:258 |
| Transport in use | USB (kUseUsb true) | [code] gatr2_robot.h:33 |

PROS output path (all Brain stdout and named streams): one 2047-byte FIFO, flushed every 2 ms into
`vexSerialWriteBuffer(1, ...)` limited by `vexSerialWriteFree(1)`; only channel 1 exists
[source] https://github.com/purduesigbots/pros/blob/develop-pros-4/src/system/dev/ser_driver.c
(lines 28, 102-108, 336) and system_daemon.c (line 121 `task_delay_until(&time, 2)`). With COBS on
(PROS default), a write of n bytes becomes COBS(4-byte stream id + n) + 0x00 = n + 6 bytes for n < 250
[source] src/common/cobs.c `cobs_encode_measure`; ser_driver.c ser_write_r.

--------------------------------------------------------------------------------------------------
## 2. Field documents (real field: pi/naviGATR/config/override/field.xml)

[measured-host] `$W/wire_sizes.exe C:/GATR2-Electronics/pi/navigatr/config/override/field.xml`;
agrees with [code] pi/naviGATR/tests/field_documents_gtest.cpp:345,818-826 (asserts 500 B / 6
chunks, 364 B / 4 chunks) and [measured-host] `final-navi/tests/navigatr_tests.exe
--gtest_filter=FieldReadDoc.OverrideDocumentsAssembleFromChunks:FieldMapDocument.OverrideField*`
(2/2 passed). map_id 0xF6B3F2C9 revision 1 matches [code] brain/operaGATR/include/field_references.h:5.

Objects: 17 = 9 goal landmarks (kind 1, flags obstacle|estimated|reference, box 155x155 mm) + 4
loaders (box 95x103) + 4 toggles (661x52 / 52x661), all obstacles. Formats [code]
translaGATR/link_documents.h:157-174 (map 24 + 28n), :233-247 (estimate 24 + 20n).

| Document | Bytes | READ_DOC chunks (96 B) | Link bytes req+rep | NG1 chars | RS-485 airtime |
|---|---|---|---|---|---|
| Field map | 500 | 6 | 132 + 692 = 824 (65% overhead) | 294 + 1414 = 1708 | 71.5 ms |
| Field estimate | 364 | 4 | 88 + 492 = 580 | 196 + 1004 = 1200 | 50.3 ms |
| Robot profile (Brain -> Pi) | 96 | 1 PROFILE_WRITE | 23+96 -> 119 + 25 | - | - |

Time to complete map + estimate after the session opens, polls running [measured-host]
budget_probe.out: USB rig 2/4/2 ms 0.25 s; USB 2/8/2 0.32 s; USB 3/10/3 1.06 s; RS-485 reply 2 ms
0.34 s; RS-485 reply 8 ms 1.11 s.

Steady field traffic: with a changing estimate (vision on) the Brain re-reads 4 chunks at most every
0.5 s = about 8 chunk exchanges/s, measured 6.4-7.9 chunks/s, costing 6-22% of state polls on the rig
(table 3a, last rows) [measured-host]. With world estimation noop the estimate never changes after
the first one [code] field_documents.h (FieldDocuments::update takes a new id only on change).

Same content as JSON for a browser [measured-host] `py -3 $W/hello_sizes.py`, json_estimate.out:

| Content | JSON B | gzip -9 B |
|---|---|---|
| Planning map equivalent (boundary + 17 objects, pose + box, no names) | 3089 | 414 |
| same with object names | 3534 | 529 |
| Estimate equivalent (17 records) | 1746 | 320 |
| Current `hello.fields[0]` (adds dimensions, features, landmark visuals, tag mounts, notes) | 22232 | 2069 |
| of which `landmarks[]` (tag mounts dominate, 1190 B each) | 16934 | 1205 |

Note: the 500 B planning map carries no display data. The viewer's 3D field (features, visuals,
tag plates) comes from the Pi's field.xml today [code] docs/inspection.md:276-281,732-733. A
Brain-authoritative field feed in standalone mode therefore draws boxes + boundary only, unless the
receiver has an offline display asset keyed by the Brain's field id [inference].

--------------------------------------------------------------------------------------------------
## 3. Link budget

### 3a. Measured on the host rig (communiGATR sims, simulated 10 s; not V5 timing)

[measured-host] `$W/budget_probe.exe`. Rig timings are fixed latencies (FakeUsb) or airtime + fixed
reply delay (FakeBus); neither models the Pi's 100 Hz loop phase (0-10 ms), which on hardware sits
between the "reply 2 ms" and "reply 8 ms" rows [inference]. TELEMETRY at 5/10/20 Hz = proxy for a
separate viz exchange (same slot rules, 87 B vs 59 B exchange).

| Setup | Load | polls/s | extra/s | poll gap p50/p99/max ms | bin up/down B/s | NG1 up/down ch/s | RS-485 bus |
|---|---|---|---|---|---|---|---|
| USB 2/4/2 ms | polls only | 48.6 | - | 21/21/22 | 1118/2867 | 2479/5978 | - |
| | extra exch 5 Hz | 46.8 (-3.7%) | 5.0 | 21/30/30 | 1416/2856 | 3092/5971 | - |
| | extra exch 10 Hz (= TELEMETRY today) | 44.9 (-7.6%) | 10.0 | 21/30/30 | 1713/2839 | 3700/5953 | - |
| | extra exch 20 Hz | 40.0 (-17.7%) | 19.0 | 21/30/32 | 2212/2721 | 4719/5737 | - |
| | tel 10 Hz + 13-pt path 2 Hz | 43.8 | 10 + 2 | 21/35/35 | 1935/2812 | 4150/5903 | - |
| | tel 10 Hz + estimate every 200 ms | 42.2 (-13%) | 8.9 + 7.9 chunks | 21/31/34 | 1752/3643 | 3799/7581 | - |
| USB 2/8/2 ms | polls only | 48.6 | - | 21/21/22 | | | - |
| | 5 / 10 / 20 Hz | 44.8 / 41.0 / 34.7 (-8/-16/-29%) | 5.0 / 10.0 / 18.3 | p99 38 | | | - |
| | tel 10 + estimate | 36.2 (-26%) | 8.7 + 7.6 | 21/38/38 | | | - |
| USB 3/10/3 ms | polls only | 45.1 | - | 23/23/24 | | | - |
| | 5 / 10 / 20 Hz | 41.9 / 38.9 / 33.8 (-7/-14/-25%) | 3.2 / 6.2 / 11.3 (overdue, below target) | p99 46 | | | - |
| RS-485 115200, reply 2 ms | polls only | 48.6 | - | 21/21/21 | 1118/2867 | | 35.6% |
| | 5 / 10 / 20 Hz | 46.1 / 43.5 / 38.6 (-5/-10/-21%) | 5.0 / 10.0 / 19.6 | p99 32 | | | 37.6 / 39.6 / 43.4% |
| | tel 10 + estimate | 38.1 (-22%) | 8.6 + 7.8 | 21/38/38 | 1633/3372 | | 44.5% |
| RS-485 115200, reply 8 ms | polls only | 46.3 | - | 22/22/22 | | | 33.9% |
| | 5 / 10 / 20 Hz | 43.0 / 40.0 / 34.7 (-7/-14/-25%) | 3.3 / 6.3 / 11.6 (overdue) | p99 44 | | | 34.0-34.4% |

Cross-check against the published cost table [code] docs/communigatr.md:764-788: fake USB default
8% (probe 7.6%), RS-485 default 11% (10.5%), USB turnaround 8 ms 16% (15.6%). Reproduced.

Reading [inference]: the scarce resource on USB is exchange slots, not bytes (under 10 kchar/s total).
Every extra request/reply transaction costs roughly its round trip + 5 ms gap out of a 20 ms poll
period; above about 5 ms round trip there is no idle slot and each extra exchange delays a poll.
On RS-485 the bytes also matter: GET_STATE alone uses 36% of the 115200 bus.

### 3b. Per-stream budget, candidates (arithmetic from section 1 sizes and proposed records in 4)

| Stream | Bytes per unit | Rate | Binary B/s | NG1 ch/s (USB) | RS-485 bus | Extra exchanges |
|---|---|---|---|---|---|---|
| GET_STATE (today) | 82 / exchange | 50 Hz | 4100 | 8700 | 35.6% | baseline |
| TELEMETRY (today) | 87 / exchange | 10 Hz | 870 | 1840 | 7.6% | 10/s; -8..-16% polls |
| (a) pose group on TELEMETRY (+22 B) | 109 / exchange | 10 Hz (grid) | 1090 (+220) | 2280 (+440) | 9.5% (+1.9) | none beyond TELEMETRY; needs new op/version |
| (b) separate VIZ_POSE exchange (40 B req + 19 B reply) | 59 | 5 / 10 / 20 Hz | 295 / 590 / 1180 | 640 / 1280 / 2560 | 2.6 / 5.1 / 10.2% | 5/10/20 per s; polls -4..-8 / -8..-16 / -18..-29% (3a) |
| (b') unsolicited one-way pose frame, no reply (USB `SG1:` line or RS-485 frame in the Brain's post-reply gap) | 41 (87 chars) | 5 / 10 / 20 Hz | 205 / 410 / 820 | 435 / 870 / 1740 | 1.8 / 3.6 / 7.1% | none; fits the 5 ms request_gap on RS-485 (3.56 ms airtime) [inference] |
| (c) path record, tank avoiding p50 6 segments | 98 B record, 1 frame 111 B | per plan (<= 12 per command + clear) | event | 227 | 9.6 ms | 1 per path (2 for > 7 segments) |
| (c) path, p90 9 / max 13 segments | 134 / 182 B record, 2 frames 186 / 234 B | event | | 382 / 478 | 16 / 20 ms | 2 |
| (c) PATH_REPORT today, p50 4 / max 8 points | 52 / 84 B request | event | | | | 1; loses turns, headings, reverse |
| (d) samples batch, 8 channels | 55 B record, 68 B frame | 10 Hz | 680 | 1410 | 5.9% | 10/s if an exchange; none if (b') |
| (d) 1 channel / 18 channels (frame full) | 26 / 128 B frame | 10 Hz | 260 / 1280 | 570 / 2610 | 2.3 / 11.1% | same |
| Field map / estimate over spectu frames (READ_DOC-style 13 B chunk header) | 668 / 476 B total (6 / 4 frames) | once + on change | | 1366 / 972 | 58 / 41 ms | none if pushed |

Plan frequency [code] actuGATR/include/actugatr/motion.h:132 (`max_plans = 12` per command);
reportPath on plan, replan, finish and new command [code] actuGATR/src/motion.cpp:172-174,229-237,
321-327. Path sizes [measured-host] path_probe3.out (3000 random pairs per model; many pairs refused
as blocked on this crowded field, successful plans only):

| Model (footprint 0.46 x 0.46 m, clearance 0.06) | ok plans | segments p50/p90/p99/max | turn-only p50/max | PATH_REPORT points p50/max | proposed record p50/p90/max B |
|---|---|---|---|---|---|
| tank avoiding | 668 | 6/9/12/13 | 3/6 | 4/8 (never thinned) | 98/134/182 |
| holonomic avoiding | 719 | 3/4/6/7 | 0/0 | 4/8 | 62/74/110 |
| tank direct | 3000 | 3/3/3/3 | 2/2 | 2/2 | 62 |

PATH_REPORT drops turn-only segments and every heading, reverse and exact flag [code]
brain/communiGATR/src/link_driver.cpp:158-183 (turn segments add only their start point, deduped),
and thins beyond 13 points [code] client.cpp:229-241. Tank-direct: 3 segments reported as 2 points.

### 3c. Standalone radio (Brain -> VEX radio -> controller USB -> laptop)

Capacity and latency are [unknown]; no repo evidence (grep of *.md/*.h/*.cpp for radio, throughput,
kbps: none relevant). Best available primary evidence:
- Host reads Brain stdout through the controller with CDC2 command 0x27 (user FIFO), one poll per
  read; pros-cli asks for 0x40 = 64 bytes per poll on the "download" channel and splits the cache on
  0x00, i.e. it expects COBS-framed output [source]
  https://github.com/purduesigbots/pros-cli/blob/develop/pros/serial/devices/vex/v5_device.py
  (`user_fifo_read`: `struct.pack("<2B", self.channel_map['download'], 0x40)`; `if b'\x00' not in
  self._serial_cache: return b''`). Its `user_fifo_write` begins with `return` (host->Brain input over
  the controller disabled there). pros-cli switches the controller to the "higher bandwidth download
  channel" for the terminal (`DownloadChannel.__enter__`, `ft_transfer_channel('download')`) and
  back to pit on exit (v5_wireless_port.py).
- vex-v5-serial: stdio over a controller uses `UserDataPacket` channel 1 for reads, writes in chunks
  of at most 224 bytes, 100 ms handshake timeout; reply decode stops at the first 0x00 byte
  [source] https://github.com/vexide/vex-v5-serial/blob/HEAD/packages/vex-v5-serial/src/serial.rs
  (`read_user`, `write_user`), packages/vex-cdc/src/cdc2/system.rs (`UserDataPacket`,
  `FixedString<224>`, decode loop `if byte == 0 { break; }`).
- Only official V5 radio data-rate figure found is for VEXlink (robot-to-robot, a different link):
  "A manager robot has double the available bandwidth (1040 bytes/second) to send information to the
  worker robot (520 bytes/second)." [source] https://api.vex.com/v5/home/cpp/VEXlink/SerialLink.html
- [inference] If one 64-byte poll per radio round trip is the limit, 20-50 ms round trips give
  1.3-3.2 kB/s. Treat as a hypothesis for the bench.

Radio stream need with COBS stream framing (n + 6 B per write) [inference, arithmetic]:

| Stream | B per write | 5 Hz | 10 Hz | 20 Hz |
|---|---|---|---|---|
| pose frame (41 B) | 47 | 235 B/s | 470 B/s | 940 B/s |
| pose + 8-channel samples in one frame (13 + 28 + 55 = 96 B) | 102 | 510 | 1020 | 2040 |
| field map + estimate once | 668 + 476 + 60 COBS = 1204 B total | | | |
| path p50 | 117 per event | | | |

If capacity is near the VEXlink figure (about 0.5-1 kB/s), a 5-10 Hz pose stream fits and samples
must be rate-limited; if it is near the 64 B/poll estimate, 10-20 Hz pose + samples fit [inference].
Field docs take 1-2 s at 1 kB/s. A carousel (one field chunk per second) costs about 100-130 B/s.

### 3d. Browser side (Pi-hosted feed today) [measured-host] `py -3 $W/ws_sizes.py`, 5 s, demo config

| Message | Rate | Median size | Rate x size |
|---|---|---|---|
| state | 30.0/s | 1082 B (robot object 848 B, attitude 324 B) | 32 kB/s |
| diag | 4.2/s | 20232 B | 85 kB/s |
| preview (binary JPEG) | 4.8/s | 8099 B | 39 kB/s |
| hello (once) | - | 23354 B (fields 22234 B) | - |
| history (once) | - | 35508 B | - |

Matches viz final_report "state about 1.1 KB, up to 30 Hz" and "151 kB/s per viewer". JSON-ish sizes of
the proposed records [measured-host] json_pose.out: pose 320 B, 6-segment path 710 B, 8-channel samples
204 B (vs 41 / 111 / 68 B binary frames). JSON is fine on loopback/LAN; never on the Brain side.

--------------------------------------------------------------------------------------------------
## 4. Proposed compact records (translaGATR conventions)

Conventions reused: little endian; positions int32 mm; headings and attitude centidegrees, CCW +z,
(-18000, 18000]; roll about robot +x positive left side up, pitch about +y positive nose down (as
TELEMETRY) [code] frames.h:268-310; stamps u32 ms on the source clock; envelope `AA 55 | type | len |
payload | crc16` with CRC-16/CCITT-FALSE and 128-byte frame max, so the existing FrameReader scan
logic applies [code] frame_codec.h:17-21,133-185; documents reuse link_documents map/estimate
formats and the READ_DOC 13-byte chunk header [code] frame_codec.h:50-58.

Frame: new type 0x20 (spectu) `AA 55 | 0x20 | len | version u8 | source_session u32 | frame_seq u16 |
records... | crc16`. Overhead 6 + 7 = 13 B. Records: `kind u8 | len u8 | body`.

| Kind | Body | Body B | Record B | When |
|---|---|---|---|---|
| 1 SESSION | contract u8, source_kind u8 (brain/navi/user), robot_id u32, clock_domain u8 (brain pros::millis, pi steady, ...), source_now_ms u32 (clock sample at send), capabilities u32, name_len u8, name <=16 | 16 + name | <= 34 | connect, then 1 Hz heartbeat |
| 2 CONTEXT | context_gen u8, placement_gen u16, field_id u32 (= map crc32, 0 none), field_revision u16, estimate_id u32, frame_flags u8 (odometry/field, anchor source), odometry_epoch u32, anchor_revision u32 | 22 | 24 | on change, and in each heartbeat |
| 3 POSE | seq u16, stamp_ms u32 (measurement, source clock), publish_delay_ms u16 (publish - measurement, saturating), flags u8, placement_gen u16, context_gen u8, x_mm i32, y_mm i32, heading_cdeg i16, roll_cdeg i16, pitch_cdeg i16 | 26 | 28 (34 with velocity) | new state only, latest-wins |
| 3 flags | b0 pose valid, b1 field frame (else odometry), b2 attitude supplied (clear = heading-only, tilt not supplied), b3 attitude measured valid, b4 velocity group present (+6: vx, vy mm/s i16, omega cdeg/s i16), b5 held/last-known (explicit stale marker, never a new measurement) | | | |
| 4 PATH | command_id u32, path_rev u16, lifecycle u8 (active, replaced, cleared, cancelled, completed, failed), mode u8, placement_gen u16, context_gen u8, chunk u8, chunks u8, segment_count u8, first_segment u8, start x i32, y i32, heading i16; then per segment: flags u8 (turn/translate, reverse, exact, turn CCW/CW), reserved u8, end x i32, y i32, heading i16 | 24 + 12 n | 26 + 12 n | on plan/replan/clear; <= 7 segments per frame, repeat header per chunk |
| 5 CHANNEL_DEF | channel_id u16, type u8 (i32 fixed / f32 / bool / enum), scale_exp i8, flags u8, unit_len u8, name_len u8, unit, name | 7 + text | about 30 | connect, heartbeat, on add |
| 6 SAMPLES | stamp_ms u32, count u8, count x (channel_id u16, value 4 B) | 5 + 6 n | 7 + 6 n (max 18 per frame) | batched per publish tick |
| 7 EVENT | seq u16, stamp_ms u32, code u16, arg u32 | 12 | 14 | durable, never coalesced |
| 8 DOC_CHUNK | READ_DOC reply body: doc_kind u8, doc_id u32, total_len u16, doc_crc32 u32, offset u16, data <= 96 | 13 + data | <= 111 | field map (500 B, 6 chunks) / estimate (364 B, 4 chunks) |

Frame sizes: single pose 41 B (NG1-style hex line 87 chars, COBS stream 47 B); pose + context 65 B;
up to 4 poses per frame (13 + 4 x 28 = 125) for capture batching only.

Brain link variants (integrated mode through the Pi):
- (a) TELEMETRY pose group: flags bit 4 + 22 B (the POSE body minus roll/pitch already there): frame
  90 B. Needs a new op or link v5 because TELEMETRY length is fixed [code] frame_codec.cpp:555-560.
- (b) VIZ op 13 with a POSE body: request 6 + 8 + 26 = 40 B, reply 19 B (header only, like
  TELEMETRY).
- (b') unsolicited 0x20 frames (no reply). USB: as a separate line marker demuxed in ProsUsbLink before
  the brain_link FrameReader. A v4 FrameReader counts an unknown type as a length error and drops
  it [code] frame_codec.cpp:56-64. On RS-485 it must be sent only in the Brain's post-reply gap; bytes
  after a request in the same drain suppress the Pi's reply [code] brain_link_commands.cpp:365;
  brain_link_commands.h:20-21.
- (r) acceptance reference on the next GET_STATE request: accepted reply request_id u16 + status u8 +
  placement_gen u16 = +5 B (23 -> 28 B, +0.43 ms airtime, +2.2% RS-485 bus at 50 Hz, +10 NG1 chars).
  Zero extra exchanges. The Pi republishes the pose it sent. Valid only while the Brain uses
  LinkDriver's pose unmodified. Same fixed-length caveat [inference; see brain-link.md d2].

Field transfer recommendation input: field_id is already a content hash (map_id = crc32 of the
document bytes) [code] link_documents.h:155; field_documents_gtest.cpp:347. A receiver cache keyed
by (field_id, total_len, revision) is therefore self-validating (CRC-32 collision risk only). The
Brain announces the field_id in CONTEXT; the receiver uses a cached or Pi-held document only when
its crc32 equals that id; otherwise it waits for the Brain's DOC_CHUNKs (pushed once on change + a
slow carousel), with no back-channel needed. Full transfer is 668 + 476 B: small next to one second
of pose traffic on USB; 1-2 s on a 0.5-1 kB/s radio [inference].

--------------------------------------------------------------------------------------------------
## 5. Latency terms (integrated USB/RS-485; from constants, no hardware numbers)

| Term | Estimate | Basis |
|---|---|---|
| Measurement -> Pi publish (source age) | about 25 ms p50 on the host synthetic demo | viz final_report section 2 (earlier host run) |
| Pi publish -> Brain holds pose (inherent to Brain authority) | poll wait 0-20 ms + Pi loop phase 0-10 ms + transfer (RS-485 7.1 ms airtime; USB unknown) + Brain poll 2 ms: mean about 15-20 ms, max about 35-40 ms | [inference] client.h:49; Loop 100 Hz; pros_link.h:44 |
| Brain -> Pi, (a) on TELEMETRY | + 0-100 ms grid wait (overdue up to 150 ms) + exchange 10-15 ms | [inference] client.h:61; docs/communigatr.md:733-750 |
| Brain -> Pi, (b) separate exchange in first slot after the state reply | + 5 ms gap + exchange 5-15 ms; costs 8-16% polls at 10 Hz | [measured-host] 3a (proxy) |
| Brain -> Pi, (b') unsolicited USB line | + tx task <= 2 ms + PROS flush <= 2 ms + USB [unknown] + Pi drain <= 10 ms | [code] pros_usb_port.cpp:22; ser_driver/system_daemon 2 ms |
| Brain -> Pi, (r) reference on next poll | + one poll period 20 ms + Pi loop phase | [inference] |
| Pi -> browser | about 0.4 ms p50 loopback | viz final_report section 2 (earlier host run) |
| Brain -> laptop over the radio | PROS flush <= 2 ms + VEXos + radio + controller + host poll [unknown] | bench |

--------------------------------------------------------------------------------------------------
## 6. Doc/code notes
- docs/communigatr.md:764-788 cost table: reproduced within 1 point by budget_probe [measured-host].
- docs/interfaces.md:490-491 says USB baud "Not applicable"; Pi pros_usb_link still sets B115200 on
  the ACM device (harmless for CDC) [code] pros_usb_link.cpp:157-158.
- docs/interfaces.md:503-506 example GET_STATE line: 46 hex digits = 23 B, 51 chars with newline;
  agrees with the codec [measured-host].
- viz final_report "46 modified files in the working tree": now committed; tree clean at ae457ee.
- Disk folder is still `pi/navigatr` (lowercase); paths above use the git name pi/naviGATR.
- PROS ser_write_r tests `if (ser_driver_runtime_config &= E_COBS_ENABLED)` (compound assignment in
  the condition) [source] ser_driver.c ser_write_r; behavior with other runtime flags not checked.
- pros-cli expects 0x00-delimited (COBS) output over the controller; with communiGATR's global COBS
  off that parser would never return data [source + inference]. Matters for Brain+Pi USB +
  controller topologies.

## Open questions / needs bench test
1. Controller radio stdio throughput and latency (VEXnet 3 pit vs download channel, Bluetooth): Brain
   test program writing stamped 41 B frames at 5/10/20/50 Hz; measure delivered B/s, loss, age.
2. Does switching the controller to the download channel (as pros-cli does) affect driving or field
   control? May a receiver do it during a match?
3. With Pi on Brain USB and a controller on a laptop, where does channel-1 output go (both, one,
   first poller)? Does COBS-off break the controller path?
4. Real GET_STATE round trip on V5 USB and RS-485 with the Pi at 100 Hz. This decides whether a
   separate 10 Hz viz exchange costs about 8% or about 16% of polls, and whether (b') unsolicited
   frames are needed.
5. V5 USB user-port throughput and `vexSerialWriteFree(1)` size; whether the 2047 B PROS buffer ever
   fills with NG1 + spectu lines.
6. V5 smart-port generic-serial max baud and accuracy (230400 / 460800 / 921600). Halving airtime
   frees about 18% of the RS-485 bus at 50 Hz polling.
7. Real replan frequency and path lengths on the robot (host planner p50 6 / max 13 segments on the
   Override map).
8. Pi read budget and parse cost with unsolicited Brain lines on USB (pros_usb_link 4 x 256 B per call).
