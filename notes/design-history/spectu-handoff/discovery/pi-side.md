# spectuGATR discovery: naviGATR (Pi) side

Date 2026-09-28. Read-only on C:/GATR2-Electronics. Host = Windows 11, MinGW ucrt64 GCC 15.2.
No hardware here: nothing below is physical behavior.
Basis tags: [code] file:line read, [source] external primary source, [measured-host] ran here,
[inference], [unknown]. Paths are relative to pi/naviGATR/ unless they start with a top folder.
Line numbers are for the .cpp/.h named (verified with grep -n).

Scratch: $S = C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad
Probe files: $S/spectu/pi-side/iso (standalone server build), $S/spectu/pi-side/probe (link probes).

---------------------------------------------------------------------------------------------
## 1. Brain transport on the Pi

### 1.1 Who owns the Brain port
| Fact | Basis |
|---|---|
| One SerialLink resource (`pros_usb_link` or `linux_serial_link`) per Brain link, shared_ptr from ResourceStore | [code] brain_link_commands.cpp:183, brain_link_publisher.cpp:125 |
| Sole reader: BrainLinkCommands::run drain loop; sole writer: BrainLinkPublisher::run (one windowed write per processed request) | [code] brain_link_commands.cpp:288-346, brain_link_publisher.cpp:454 |
| Publishing must name the same Serial resource as the commands slot | [code] brain_link_publisher.cpp:112-116 |
| Both run on the estimation worker, in order commands -> localization -> publishing, at Loop rate_hz (100 in brain_profile_usb.xml:17) | [code] runtime/system.h:25-33 |
| Inspection/capture never touch the link: they read LinkMonitor snapshots and DiagnosticsHub records | [code] diagnostics/link_monitor.h:1-9, hub.h:1-11 |
| No OS exclusivity: open(O_RDWR/O_NONBLOCK), no TIOCEXCL/flock anywhere in src | [code] pros_usb_link.cpp:149, grep TIOCEXCL/flock = none |
| Only a doc warning keeps a second reader (PROS terminal) off the port | [code] docs/usb_localization_bench.md:28-30 |

Consequence [inference]: a spectuGATR receiver running on the Pi must not open the Brain tty. Any Brain
visualization that reaches the Pi must be demultiplexed inside naviGATR's link resource (the single owner)
and handed on through a bounded queue.

### 1.2 USB (pros_usb_link)
| Item | Behavior | Basis |
|---|---|---|
| Discovery | Scans /sys/class/tty/ttyACM*, walks up sysfs for bInterfaceNumber, idVendor, idProduct; accepts only 2888:0501 interface "02"; two matches -> empty (explicit path required) | [code] pros_usb_link.cpp:180-207 |
| by-id | Not used by auto discovery. `<Device path="/dev/serial/by-id/...">` is accepted as an explicit path (docs/config comments) | [code] pros_usb_link.cpp:147; config/override/brain_profile_usb.xml:40-41; docs/usb_localization_bench.md:49-58 |
| Platforms | Linux sysfs only for discovery. POSIX open/termios for I/O; non-POSIX build: reads report closed, writes fail "POSIX hosts only" | [code] pros_usb_link.cpp:51-72, 118-121 |
| tty setup | cfmakeraw, B115200 (ignored by CDC), no CRTSCTS, VMIN=VTIME=0, tcflush(TCIOFLUSH) on every (re)open | [code] pros_usb_link.cpp:151-169 |
| Reopen | Reconnect attempted at most once per 1 s from read/write; POLLHUP/ERR/NVAL or read error closes; discovery repeated so ACM number may change | [code] pros_usb_link.cpp:27, 55-67, 143-147 |
| Write bound | host backpressure bounded to min(deadline, now+5 ms); timeout disconnects | [code] pros_usb_link.cpp:87-111 |
| Framing | TX: "NG1:" + uppercase hex + "\n" per frame (<= kMaxFrameLen 128 bytes) | [code] pros_usb_link.cpp:279-291 |
| RX line parse | strips trailing \r; uses the LAST "NG1:" in the line (rfind), text before it ignored; odd/non-hex/over-128-byte lines rejected; line limit 4+256+256 chars then discard to "\n" | [code] pros_usb_link.cpp:25-26, 216-253 |
| Console lines | lines without "NG1:" are only counted (console_lines_); content dropped; counter not exported to inspection or LinkMonitor (only a test reads it) | [code] pros_usb_link.cpp:220; grep consoleLines -> tests/pros_usb_link_gtest.cpp:360 only |
| Rejections | reported to the link's LinkMonitor as "rejected" with reason | [code] pros_usb_link.cpp:209-214 |
| Read budget | while no NG1 frame is decoded, reads at most 4 x 256 raw bytes per readAvailable call | [code] pros_usb_link.cpp:258-268 |
| DriverEnable | refused for pros_usb_link (build error) | [code] pros_usb_link.cpp:302-305 |
| Monitor kind | "brain_usb" | [code] pros_usb_link.cpp:309, brain_link_commands.cpp:219-225 |

### 1.3 RS-485 (linux_serial_link + DriverEnable)
| Item | Behavior | Basis |
|---|---|---|
| Config | `<Device path="/dev/ttyAMA5" required="true"/> <Baud 115200/> <DriverEnable gpio="6"/>` | [code] config/override/brain_profile_rs485.xml:41-43 |
| DE control | sysfs GPIO (/sys/class/gpio export/direction/value); driven low at open (listen), high only while write() sends; post_guard default 2 chars, tx_margin 2000 us | [code] serial_links.h:17-20; serial_links.cpp:191-232; transport/serial_port.cpp:152-180 |
| Half-duplex TX rule | window check, refuse while input pending, DE on, write, wait transmitter empty, guard, DE off | [code] transport/half_duplex.cpp:86-110, half_duplex.h:59-65 |
| Reopen | ReopeningLink: closed/never-opened device retried once per 1 s from readAvailable, DE setup repeated | [code] serial_links.cpp:59-87, 235-246 |
| Non-POSIX | SerialPort is a stub (open fails, gpio unsupported) | [code] transport/serial_port.cpp:256-318 |

### 1.4 Request/reply drain rules that constrain any second stream
| Rule | Basis |
|---|---|
| Drain until a read returns 0; only the newest completed request is processed; others counted "superseded" | [code] brain_link_commands.cpp:280-346 |
| Frames that are not kFrameBrainRequest (0x10) are rejected "not a brain request frame" and skipped | [code] brain_link_commands.cpp:314-318 |
| No reply when the request is followed by more decoded bytes in the same drain (half-duplex safety: "the brain may be transmitting"), or on the first drain | [code] brain_link_commands.cpp:365-372 |
| Reply window: not_before = completion read + 1 ms guard; deadline = previous drain's last read + 40 ms | [code] brain_link_commands.cpp:373-374 |
| The Brain initiates everything; the Pi never sends unsolicited | [code] brain_link_publisher.cpp:385-388 |

Measured probes (scratch copy of tests/brain_io_gtest.cpp + 4 new tests, linked against the prebuilt
$S/final-navi/libnavigatr_core.a; command:
`g++ -std=gnu++17 -Wall -Wextra -I.../pi/navigatr/src -I.../GATR2-Electronics ... probe_gtest.cpp -o probe.exe <final-navi libs> -lws2_32 && ./probe.exe --gtest_filter=SpectuProbe.*` -> 4/4 pass) [measured-host]:
| Probe | Result |
|---|---|
| valid non-request frame AFTER a GET_STATE in one drain | no reply, unanswered = 1 |
| same frame BEFORE the request in one drain | replied |
| "SG1:<hex>" line after an NG1 request line | request bytes delivered; the SG1 line counted as console (framedLines 1, consoleLines 1) |
| 1539 bytes of non-NG1 lines then an NG1 request | first readAvailable returns 0 (drain ends), second returns the 23-byte request |

### 1.5 Demultiplexing a second logical Brain stream on the Pi (design facts)
| Option | Pi-side consequence | Basis |
|---|---|---|
| A. New frame type inside NG1 (or inside RS-485 frames) | Rejected by commands today; when it trails a request in one drain it suppresses that reply (probe). Would need the drain rule changed to count only request bytes, which is the RS-485 safety rule | [measured-host] probe 1; [code] brain_link_commands.cpp:314-318, 365-372 |
| B. New op in the request/reply protocol (as TELEMETRY op 12, PATH_REPORT op 10) | Works today on USB and RS-485; Brain schedules it as a poll-slot; Pi answers header-only; displaces polls (final report claims 6-11% fewer at default timing; not re-measured here) | [code] brain_link_commands.cpp:532-606; translaGATR/frames.h:113 |
| C. Separate line marker on USB (e.g. "SG1:" prefix, hypothetical) demuxed inside ProsUsbLink::finishLine into a bounded queue | Not delivered to the brain_link FrameReader, so it cannot suppress replies or be interpreted by localization; costs parse time on the estimation worker; competes for the 1024-byte-per-read budget (a request behind >1024 B of other lines waits one loop period, 10 ms at 100 Hz; sustained >~100 KB/s would back up) | [measured-host] probes 3-4; [code] pros_usb_link.cpp:216-268; [inference] rates |
| C caveats | marker must be matched at line start (current parser uses rfind("NG1:") anywhere in the line; free text containing "NG1:" would be taken as a Brain link line); line length <= 516 chars; USB only (no equivalent on half-duplex RS-485 without Brain-side scheduling) | [code] pros_usb_link.cpp:218, 25-26 |
| D. RS-485 | Brain is bus master; Pi refuses to transmit while input is pending; any unsolicited Brain stream would starve replies. Only option B fits | [code] half_duplex.cpp:95-99; [inference] |

Forwarding path that exists today (option B): commands slot -> DiagnosticsHub (kBrainTelemetry raw body,
kPath) -> inspection (latest record) and capture (drained ring). Localization does not read PATH_REPORT
(PathReport lives in CommandState and is read only by inspection/capture) [code] state/command_state.h:71-84,
grep of localization/target_resolution for command.path = none. TELEMETRY's attitude group IS read by
localization (see 2.2 and 6).

---------------------------------------------------------------------------------------------
## 2. Brain link commands and publisher

### 2.1 Identities
| Identity | Producer / scope | Basis |
|---|---|---|
| Brain session u32 | Pi-assigned random nonzero per new HELLO nonce; retried opening HELLO keeps it; recent 4 nonces -> Stale | [code] brain_link_commands.cpp:747-785 |
| pi_instance u32 | random per BrainLinkCommands construction and reset() | [code] brain_link_commands.cpp:158-168, 249-268 |
| request_id u16 | Brain per boot; serial-arithmetic dedupe; resend of newest read-only id answered again | [code] brain_link_commands.cpp:75-97, 787-835 |
| placement | SET_POSE -> init_sequence+1, init_session; reply Pending until robot.placement_session/sequence match, then Ok with odometry_epoch, anchor_revision | [code] brain_link_commands.cpp:617-634; brain_link_publisher.cpp:404-413 |
| Pi System session | 64-bit random hex per System build + reset_count | [code] runtime/system.cpp:53-61, 150 |
| profile_id | crc32 of the profile document | [code] brain_link_commands.cpp:668-669 |
| map_id / revision | crc32 of the map document bytes; revision u16 from the field definition | [code] translaGATR/link_documents.h:155-172; impl/publishing/field_documents.h:25-36 |
| estimate_id | per publisher, +1 per snapshot, keeps counting across reset | [code] field_documents.cpp:303-341, field_documents.h:89-90 |
| Clock domains in Pi core | only kDevice (Pico) and kHost; no Brain domain in MonotonicTime. DiagClock has kBrain=3 for capture rows | [code] core/time.h:17-21; diagnostics/records.h:131-137 |

### 2.2 TELEMETRY end to end (op 12, 54-byte body)
1. Request frame 6+8+54 = 68 bytes (USB line 141 chars); reply header-only 6+13 = 19 bytes [code] translaGATR/frame_codec.h:18-24, frames.h:310.
2. process(): version, op, request_id, session checks; newest-id resend answered again without re-posting [code] brain_link_commands.cpp:457-502, 823-829.
3. execute(): attitude group -> BrainImuBench::acceptAttitude(present, stamp, roll, pitch, now=cycle time) [code] brain_link_commands.cpp:532-540; raw body -> hub kBrainTelemetry [code] :541-554.
4. Reply Ok header-only via publisher window [code] brain_link_publisher.cpp:382-454.
5. Consumers:
   - inspection pushTelemetry: hub.latest() only (intermediate reports never go live), <= 20 Hz per client (kTelemetryPeriod 50 ms), channel priority 1, sendLatest [code] inspection_service.cpp:55-67, 946-987;
   - telemetryDocument: received_host_ms = Pi post time, age_ms on Pi clock, stamp_ms Brain clock passed through, groups null when flag clear [code] inspection_document.cpp:1932-2008;
   - capture: brain_telemetry.csv with pi_host_us plus source_clock=brain, source_ms [code] capture/capture_bundle.cpp:224-237, 888-899;
   - localization: BrainImuWheelBench::publishAttitude -> AttitudeObservation measuredAt = Pi arrival -> estimator Attitude fold (profile builder adds it for VEX profiles) -> RobotState.attitude and pose history [code] impl/localization/brain_imu_wheel_bench.cpp:204-224; runtime/brain_profile_builder.cpp:537-538, 606-611.

### 2.3 PATH_REPORT (op 10)
- <= 13 points, mm, field frame; command_id, mode; no Brain timestamp, no map id, no anchor/placement id [code] translaGATR/frames.h:247, frame_codec.h:59.
- Stored as CommandState.path with Pi `received`; cleared on a new HELLO session; kPathNone clears [code] brain_link_commands.cpp:577-606, 780-784.
- Viewer gets it inside diag (<= 4 Hz, ~20 KB document, skipped when unchanged) [code] inspection_document.cpp:1406-1428.

### 2.4 Field documents sent to the Brain
| Item | Behavior | Basis |
|---|---|---|
| Map | built once at publisher build from `<Field resource_id>`; needs revision, Boundary, wire_id on every landmark; <= 128 objects, 24 + 28/object bytes (max 3608) | [code] brain_link_publisher.cpp:65-98; link_documents.h:170-174 |
| Map contents | boundary, per object id/kind/flags/nominal pose/collision box only. NOT on the wire: dimensions, walls, features, visuals, tag mounts, approaches | [code] link_documents.h:155-168 vs inspection_document.cpp:807-947 |
| Estimates | snapshot taken in a reporting cycle when records/epoch/anchor changed and >= estimate_period_ms (200) since the last; last 3 retained; record = source, valid flag, pose, age | [code] field_documents.cpp:303-341; field_documents.h:69, 73-77 |
| Chunking | READ_DOC doc_id 0 = current; <= 96 bytes (kDocChunkMax) per reply; older than retained -> Stale; map doc_id must equal map_id | [code] field_documents.cpp:344-392; frames.h:246 |
| State block | GET_STATE reply carries map_id and newest estimate_id plus odometry_epoch/anchor_revision | [code] brain_link_publisher.cpp:365-368 |
| Acceptance | No wire field tells the Pi which map/estimate the Brain accepted (not in GET_STATE request, TELEMETRY, or any ack). READ_DOC doc_id/offset are decoded but not recorded in DiagBrainRequest or the decoded summary | [code] frames.h:315-362 (BrainRequest fields); records.h:200-208; brain_link_commands.cpp:390-425 |

---------------------------------------------------------------------------------------------
## 3. Inspection: core scene vs Pi instrumentation

Rule applied: Brain owns pose, field it knows/uses, paths/targets, execution, attitude. Pi owns acquisition,
raw sensors, pins, init/calibration/freshness, estimator internals/timing, camera/detection, transport.

| Message / section | Content | Class today | Target owner | Basis |
|---|---|---|---|---|
| hello envelope, inspection rates, pong | transport | host | spectu host (generic) | inspection_document.cpp:753-784, 1902-1923 |
| hello.session / configuration / warnings | Pi System identity, config digest | Pi | Pi adapter (source identity) | :760-774, 1023-1028 |
| hello.features | capability flags (telemetry only when commands_type==brain_link) | Pi | protocol capabilities message | :786-797 |
| hello.robot_body | display box from Pi config | presented as robot body | fallback only; Brain owns footprint | :798-805; runtime/inspection_config.h:34-38 |
| hello.fields[] planning part (boundary, obstacles, landmark nominal + collision boxes, map_id, revision) | EVERY FieldMap resource on the Pi, map_id computed locally | presented as "the field" | Brain (what it accepted); Pi copy usable only as verified cache for a matching map_id | :807-947; spectaGATR/field_scene.js:2-4, 518 |
| hello.fields[] display part (dimensions, features, visuals, tag mounts, approaches), tag_families | not on the Brain wire | Pi | Pi display/camera assets (or a richer field format the Brain can publish) | :858-963 |
| hello.cameras, camera_sensors | camera devices, intrinsics, mounts | Pi | Pi instrumentation | :965-999 |
| hello.localization | estimator type, functions, history config | Pi | Pi estimator internals | :1001-1021 |
| state.robot | Pi estimator pose (odom, field, field_from_odom, velocity, covariance, placement ids, measured_at, age) | presented as THE robot | Pi instrumentation: "Pi estimate / what GET_STATE would return". Brain pose is core | :1074-1111, 1798-1835; spectaGATR/app.js:698 |
| state.robot.attitude | Brain VEX tilt at Pi arrival combined with Pi planar heading | presented as robot attitude | Brain (attitude); Pi keeps its fold as internals | :144-175, 1755-1771; brain_imu_wheel_bench.cpp:204-224 |
| state.localization, history (trail) | readiness, stationary, continuity; odom-frame trail | trail presented as robot trail | Pi internals; core trail from Brain poses | :1114-1134, 1817-1823, 1837-1854 |
| diag.localization.functions, stillness, bias | calibration, freshness | Pi | Pi | :1153-1181, 344-366 |
| diag.field_snapshot, field_objects | Pi newest landmark estimates vs nominal | drawn as landmark bodies | Pi estimator internals; core landmarks = Brain accepted estimate | :1188-1253; field_scene.js:1119, 1247 |
| diag.detection_frames, frame header + JPEG | detection, association trace, pose at exposure | Pi | Pi camera | :606-744, 1255-1260, 2023-2050 |
| diag.target | Pi TargetState (v4 has no Brain selection op) | Pi | Pi internal; not a Brain target | :1262-1279; state/command_state.h:95-97 |
| diag.command | init_pose/sequence the Pi received | Pi | Pi adapter (what arrived); placement id is frame identity | :1280-1294 |
| diag.brain_link link_open/session/pi_instance/last_request/state | link health and the state block the Pi would answer | Pi | Pi transport/adapter | :1296-1325, 420-451 |
| diag.brain_link.profile.running | Brain robot profile as applied (wheels, IMU, footprint, calibration) | footprint drawn as robot | footprint/geometry = Brain; applied wheel/IMU binding = Pi | :1326-1341, 369-417; field_scene.js:764-775 |
| diag.brain_link.calibration, wheels, operation | Pi bias calibration, READ_WHEELS view, Pico op | Pi | Pi | :1343-1404 |
| diag.brain_link.path | Brain PATH_REPORT forwarded | drawn as path | Brain (core), forwarded explicitly | :1406-1428; field_scene.js:1124-1170 |
| telemetry | Brain attitude, motion/execution, target, cmd, errors, wheel rpm | overlays, graphs | Brain (core execution + attitude) | :1932-2008; field_scene.js:1388-1438 |
| diag.pico, sources, events, workers, diagnostics, hub | Pico link, sensor freshness, lifecycle, worker timing, link counters | Pi | Pi | :1432-1536, 1647-1670 |
| diag.workers.inspection, feedStats | server/queue stats | host | spectu host | :1480-1525, 1563-1646 |
| instrumentation | link monitors, raw/decoded, Pico diag, Brain telemetry echo | Pi | Pi (transport) | diagnostics/instrumentation.cpp |
| capture status/bundle | Pi capture | Pi | Pi adapter; control API generic | capture/capture_http.h |

naviGATR must stop presenting as authoritative core scene [inference from the rule + code above]:
state.robot and history as "the robot" and its trail; robot.attitude; hello.fields as "the field";
field_objects as the landmark state; hello.robot_body. It keeps publishing all of them as its adapter
output (labeled Pi source), plus forwarded Brain data (path, telemetry) labeled source=Brain, relay=Pi.

Pi-side values the Brain does NOT (necessarily) use [code-backed]:
- Pi pose at 100 Hz loop and up to 60 Hz state feed; the Brain sees only GET_STATE samples it polled (robot_age_ms tells it the age at reply) [code] brain_link_publisher.cpp:287-290; inspection_config.h:47, 69.
- field_objects = newest FieldSnapshot (every world-estimation run), newer than the newest estimate document (rate-limited 200 ms) which may be newer than what the Brain read [code] field_documents.cpp:303-341; inspection_document.cpp:1188-1253.
- brain_link.state.estimate_id = Pi's newest id, not the Brain's accepted id [code] brain_link_publisher.cpp:365-368.
- hello.fields lists all FieldMap resources, not only the one `<Field resource_id>` publishes [code] inspection_document.cpp:807-816 vs brain_link_publisher.cpp:65-98.
- diag.target (TargetState) is not driven by the Brain in v4 [code] command_state.h:95-97.

---------------------------------------------------------------------------------------------
## 4. Extracting the server into spectuGATR/host

### 4.1 Dependencies
| Unit | LOC | Depends on | Extractable | Basis |
|---|---|---|---|---|
| inspection/http_server.{h,cpp} | 227 + 1614 | std, Winsock or POSIX sockets, std::thread/mutex, core/host_clock.h (-> core/time.h) for recent_closes host_ms only; SHA-1/base64 in-file | yes, as is | http_server.cpp:14-50, 142-225, 1243 |
| inspection/json_writer.h | 179 | std only | yes | json_writer.h:234-241 |
| capture/zip_writer.{h,cpp} | 38 + 146 | translagatr::crc32 (link_documents.cpp) | yes | zip_writer.cpp:8 |
| inspection/viewer_assets.h + cmake/embed_assets.cmake | 23 + 65 | CMake script only | yes | cmake/embed_assets.cmake:1-65 |
| core/host_clock.h + core/time.h | 39 + 69 | std | needs a receiver clock model (time.h domains are Pico/Pi only) | core/time.h:17-21 |
| core/clock_sync.h (min-offset mapper) | 81 | core/time.h | reusable pattern for Brain->receiver mapping | clock_sync.h:30-79 |
| capture/capture_http.cpp captureRoute(CaptureService*) + capture_service.h interface | ~170 + 60 | http_server.h, JsonWriter | yes (the System overload is Pi) | capture_http.h:24-31 |
| inspection/inspection_service.cpp | 1350 | System, documents, capture, hub, jpeg | client table, subscribe parsing, per-channel rate/priority scheduling reusable after refactor; the rest is Pi | inspection_service.cpp:1-16, 55-69 |
| inspection/inspection_document.cpp | 2052 | System and every Pi state type | no (Pi adapter schema) | includes :11-26 |
| inspection/jpeg_encoder | 127 | vendored stb | Pi camera side | jpeg_encoder.cpp:13 |
| Server-only tests: 6 InspectionFeed tests (latest-wins, partial frames, send order, reliable overflow, flow control, reliable budget) + Inspection.HttpServerBounds + UrlDecode + tests/inspection_test_client.h (421) | ~1200 | gtest, raw sockets (Win/POSIX split) | yes | tests/inspection_feed_gtest.cpp:273-633; tests/inspection_gtest.cpp:643, 840 |

Estimated extractable C++ as is: ~2.5k LOC in 7 files (server 1841, JSON 179, ZIP 184, assets 88,
capture route/interface ~230) plus ~1.2k LOC of portable tests. Everything is in namespace navigatr
(rename). [code + count via wc -l]

Measured [measured-host]:
- `g++ -std=c++17 -Wall -Wextra -Werror -Iinc -c http_server.cpp` with only http_server.h, host_clock.h,
  time.h copied into $S/spectu/pi-side/iso/inc: compiles clean; a 15-line main linked with `-lws2_32`
  starts on an ephemeral port and stops (printed PORT 61122).
- zip_writer.cpp compiles clean with only its header + translaGATR include; JsonWriter round-trip test passes.
- Existing server tests on the prebuilt binary: `navigatr_tests.exe --gtest_filter=InspectionFeed.*:Inspection.HttpServer*:Inspection.UrlDecode*` 23/23 pass (42 s).

### 4.2 Platform code paths in http_server.cpp
| Path | Windows | POSIX | Basis |
|---|---|---|---|
| sockets | winsock2, WSAStartup refcounted, closesocket, ioctlsocket FIONBIO | fcntl O_NONBLOCK, close | :16-38, 82-140 |
| SIGPIPE | n/a | MSG_NOSIGNAL when defined; no SO_NOSIGPIPE/signal ignore | :123-128; grep SIGPIPE = none |
| bind | SO_EXCLUSIVEADDRUSE | SO_REUSEADDR | :598-604 |
| select | FD_SETSIZE forced to 128 | maxfd+1, fd < 1024 limit [inference] | :17-18, 1296-1324 |
| send buffer | SO_SNDBUF | SO_SNDBUF + TCP_NOTSENT_LOWAT when the header defines it | :636-647 |
| TCP_NODELAY | yes | yes | :657 |
| IP | IPv4 only (AF_INET, inet_pton) | same | :586-592 |
| Origin/Host checks | none (WebSocket upgrade and POST /api/capture accept any Origin) | same | grep Origin = none |

External facts:
- TCP_NOTSENT_LOWAT: Linux `#define TCP_NOTSENT_LOWAT 25 /* limit number of unsent bytes in write queue */`
  [source] https://raw.githubusercontent.com/torvalds/linux/master/include/uapi/linux/tcp.h ;
  macOS xnu `#define TCP_NOTSENT_LOWAT 0x201 /* Low water mark for TCP unsent data */`
  [source] https://raw.githubusercontent.com/apple-oss-distributions/xnu/main/bsd/netinet/tcp.h .
  So the #ifdef also enables it on macOS; http_server.h:89-90 says "(Linux)" (doc/code nuance). Windows: not used.
- macOS MSG_NOSIGNAL: `#if __DARWIN_C_LEVEL >= 200809L #define MSG_NOSIGNAL 0x80000` and `SO_NOSIGPIPE 0x1022`
  [source] https://raw.githubusercontent.com/apple-oss-distributions/xnu/main/bsd/sys/socket.h .

### 4.3 Viewer embedding
- CMake globs top-level spectaGATR/*.{html,js,css,json,svg} (no subfolders) plus third_party/three
  (three.module.min.js, OrbitControls.js, LICENSE) under vendor/; embed_assets.cmake writes one C++ table
  [code] CMakeLists.txt:141-175; cmake/embed_assets.cmake:35-65.
- Size: ~1.06 MB source (three ~724 KB, app ~340 KB); generated viewer_assets.cpp 5.3 MB; Release exe 5.0 MB
  [measured-host] du/ls on repo and $S/final-navi/generated, $S/final-rel/navigatr.exe.
- `static_root` serves files from a directory first (traversal-checked, Cache-Control no-store), then embedded
  [code] inspection_service.cpp:209-220, 511-532, 1269-1278.
- Viewer JS: 8797 lines in 19 files; pure Pi-schema modules include link_panel.js, instrumentation_panel.js,
  camera_panel.js, diagnostics.js; generic-leaning: feed.js, dom.js, series.js, graphs.js, smoothing.js,
  latency.js, bundle.js, replay.js (replay timeline is pi_host_us) [code] file headers.

### 4.4 Gaps for a desktop receiver (not present in naviGATR)
- Serial I/O on Windows/macOS: none (stubs) [code] transport/serial_port.cpp:256-318; pros_usb_link.cpp:68-71.
- Discovery on Windows/macOS and for the V5 controller: none; discovery hard-codes Brain PID 0501 and interface 02 on Linux sysfs [code] pros_usb_link.cpp:180-207.
- WebSocket client (receiver connecting to the Pi feed as a source): none in src; the test client exists [code] tests/inspection_test_client.h. Flow control needs the client to answer pings (browsers do) [code] http_server.h:21-37.
- Browser launch, IPv6, Origin/Host validation, packaging: none [code].

---------------------------------------------------------------------------------------------
## 5. DiagnosticsHub, LinkMonitor, capture: generic vs Pi

| Unit | Generic part | Pi-specific part | Basis |
|---|---|---|---|
| DiagnosticsHub (hub.h/.cpp 215 LOC) | bounded ring, post never blocks, drop-and-count per kind, wanted mask (1 atomic load), latest-per-kind with seq, source-name registry | closed enum DiagKind (10 kinds, 32-bit mask) and std::variant of Pi records; one consumer | hub.h:1-93; records.h:112-127, 241-250 |
| LinkMonitor (428 LOC) | per-link atomics (rx, tx attempted/accepted, frames, rejects), rates, last-valid ages, raw/decoded/error rings gated by atomics | FrameReaderStats (translaGATR), Brain request staging slot | link_monitor.h:1-160 |
| CaptureRecorder (capture_service.cpp ~810 LOC of 1104) | rolling pre window, post window, triggers with cooldown/hourly cap, record/memory/duration/bundle bounds, one capture at a time, status JSON, dir write fallback | reads DiagnosticsHub/DiagRecord; CaptureFault enum is Pi (link lost, continuity, Pico reboot, profile) | capture_recorder.h:1-80; capture_config.h:26-32 |
| CaptureHost interface | already abstracts the System (session, reset count, metadata) | SystemCaptureHost is the Pi adapter | capture_recorder.h:38-50; capture_service.cpp:69-282 |
| capture_bundle.cpp (1389) | Row/CSV escaping, Segments (split on identity changes, never joined), metadata/README framework | per-DiagKind CSV schemas; pi_host_us declared "the only clock shared by every file" | capture_bundle.h:1-11; capture_bundle.cpp:183-247, 333-387, 998-1000 |
| zip_writer | store-only ZIP | none | capture/zip_writer.* |

[inference] A receiver-side capture can reuse recorder mechanics and the bundle framework if the record type
becomes a keyed (source, stream) message instead of DiagKind; the Pi keeps SystemCaptureHost, the Pi row
schemas and Pi fault triggers as its adapter.

---------------------------------------------------------------------------------------------
## 6. Pi and Brain data merged by arrival time or across clocks today

| Where | What | Basis |
|---|---|---|
| Localization, Brain VEX IMU bench | Pico wheel samples and Brain VEX rotation paired by Pi arrival time; no Brain/Pico clock sync | [code] impl/localization/brain_imu_wheel_bench.h:27-28; docs/usb_localization_bench.md:84-85 |
| Localization attitude | Brain TELEMETRY tilt timed at Pi arrival (cycle time), folded with Pi planar heading into RobotState.attitude | [code] brain_imu_wheel_bench.cpp:216; planar_motion_integrator.h:32-34 |
| World estimation | pose-history attitude at camera exposure (from the fold above) used by tag_mount_association when not noop | [code] tag_mount_association.cpp:221-235; state/pose_history.h:20, 114 |
| GET_STATE IMU sample | stamped with the cycle's host time; Brain stamp used only for duplicate/restart detection | [code] brain_link_commands.cpp:508-514; resources/brain_imu_bench.h:6-8, 35-49 |
| Viewer graphs | Brain telemetry plotted at Pi arrival (received_host_ms) on the same axis as Pi robot series at measurement time; labeled "at Pi arrival" | [code] spectaGATR/graphs.js:10-15, 61-63 |
| Viewer overlays | Brain desired-heading arrow drawn at the current Pi robot position; Brain path drawn over the Pi pose with no map/anchor/placement check (path carries session/command_id only) | [code] field_scene.js:1414-1422, 1124-1170 |
| Capture / replay | one timeline on pi_host_us; Brain source_ms kept as a column | [code] capture_bundle.cpp:998-999; spectaGATR/replay.js:1-7 |
| Latency terms | browser <-> Pi via ping/pong offset (+-RTT/2); a defined mapping, not an arrival merge | [code] spectaGATR/latency.js:1-10 |

Doc/code contradictions noticed:
1. "localization never reads it" (TELEMETRY) [code] translaGATR/frames.h:113, 267-269 and "never localization" brain_link_commands.cpp:533, versus the attitude fold into localization and world estimation above. brain_link_commands.h:50-53 is accurate.
2. docs/usb_localization_bench.md:96 "the robot stays level because this IMU path supplies no tilt" is stale since TELEMETRY attitude (profile builder adds the fold for VEX profiles).
3. http_server.h:89-90 says TCP_NOTSENT_LOWAT is Linux; macOS headers define it too (4.2).
4. link_monitor.h:49 lists kind "brain_rs485"; code names it "brain_serial" (brain_link_commands.cpp:220).
5. field_scene.js:2-4 calls hello.fields "the planning data the Brain is served"; it is every Pi FieldMap resource, not what the Brain accepted.

---------------------------------------------------------------------------------------------
## Open questions / needs bench test
- USB CDC throughput and latency of the V5 user port under extra line traffic (Brain side + Pi read budget); unknown here.
- Whether any Brain console text shares /ser/ngtr lines mid-line (rfind parser tolerates prefixes, but interleaving inside a line would corrupt it) [unknown, Brain side].
- Pi estimation-worker CPU cost of parsing an added line stream at 100 Hz on the Pi 4/5 [unknown].
- TELEMETRY over real USB and RS-485, and its poll displacement, not re-measured here (final report claims 6-11%).
- sysfs GPIO for DriverEnable on current Pi kernels (legacy /sys/class/gpio, numbering offsets) [unknown].
- Whether spectuGATR should add a Brain->Pi ack of accepted map_id/estimate_id (none exists in v4) or have the Brain publish it directly; needed to label Pi-cached field content as verified.
- Security stance for a desktop receiver on 127.0.0.1: no Origin/Host checks today (cross-site WebSocket/POST from any page the user opens) [inference].
