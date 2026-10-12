# Discovery: Brain-side link transport ownership and request scheduling (communiGATR)

Date 2026-09-28. Read-only. Repo at ae457ee (clean). Labels: [code] file:line verified,
[source] external primary/secondary with URL, [measured-host] run here (command given),
[inference], [unknown]. No hardware; nothing here is a physical V5 measurement.

Scratch (probes, downloaded PROS sources, outputs):
C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/spectu/brain-link/
  probe.cpp / probe.out          sizes + scheduling sweep on communiGATR host fakes
  e2e_probe.cpp / e2e_probe.out  copy of pi/naviGATR/tests/brain_link_e2e_gtest.cpp + one added
                                 TEST (real Pi brain_link, Loop 100 Hz), built against final-navi libs
  pros/*.c *.py                  PROS 4.2.2 kernel and pros-cli sources fetched for quoting

PROS kernel sources cited below are tag 4.2.2 (the version in brain/locaGATR/project.pros:81 and
include/pros/version.h:23):
  https://github.com/purduesigbots/pros/blob/4.2.2/src/system/dev/ser_driver.c
  https://github.com/purduesigbots/pros/blob/4.2.2/src/system/dev/ser_daemon.c
  https://github.com/purduesigbots/pros/blob/4.2.2/src/system/system_daemon.c
  https://github.com/purduesigbots/pros/blob/4.2.2/include/vdml/vdml.h
  https://github.com/purduesigbots/pros/blob/4.2.2/src/devices/vdml_serial.c
pros-cli at develop 45aae5df262eab1246f4d74c5be7bd1a4916eb73:
  https://github.com/purduesigbots/pros-cli/blob/45aae5df262eab1246f4d74c5be7bd1a4916eb73/pros/serial/devices/vex/v5_user_device.py
  https://github.com/purduesigbots/pros-cli/blob/45aae5df262eab1246f4d74c5be7bd1a4916eb73/pros/serial/terminal/terminal.py

-----------------------------------------------------------------------------------------------
## 1. Brain USB serial ownership (Transport::kUsb)

### 1.1 Who touches what [code]

| Item | Where | Detail |
|---|---|---|
| Transport choice | pros_link.cpp:84-91 | ProsUsbPort built only for Transport::kUsb; ProsSerialPort only for kSmartPort. In RS-485 mode communiGATR never opens /ser/*, never reads stdin, never calls serctl. |
| Output stream | pros_usb_port.cpp:188-193 | `::open("/ser/ngtr", O_WRONLY)` once, fd kept for program life |
| Stream activation | pros_usb_port.cpp:197 | `fdctl(fd, SERCTL_ACTIVATE, nullptr)` -> adds 'ngtr' to kernel enabled-stream set |
| COBS | pros_usb_port.cpp:195-198 | `serctl(SERCTL_DISABLE_COBS, nullptr)`; comment: "COBS off applies to all PROS output" and "SERCTL_NOBLKWRITE does nothing in 4.2.2". Called once (`configured` flag :194,201); never re-enabled. |
| Input | pros_usb_port.cpp:126-160 | rx task: `::read(STDIN_FILENO, input, 95)` (96-byte buffer, 1 byte kept for kernel NUL) |
| Tasks | pros_usb_port.cpp:203-220 | "communigatr-usb-tx" and "communigatr-usb-rx", both TASK_PRIORITY_DEFAULT (8), default stack |
| Queues | pros_usb_port.cpp:77-78 | outgoing FrameQueue<1> (poll task -> tx), incoming FrameQueue<4> (rx -> poll task); SPSC lock-free, 128-byte slots |
| tx loop | pros_usb_port.cpp:98-124 | pops, drops if older than tx_lifetime (expired++), encodes NG1 line, ONE `::write` per line; idle sleep kIdleMs=2 (:22) |
| tx lifetime | pros_link.cpp:48-52 | clamp(response_timeout*800, 1, 10000) ms = 48 ms default |
| rx loop | pros_usb_port.cpp:126-160 | decodes with UsbLineDecoder; whole frame or nothing into incoming (input_full++ on overflow); sleeps kNoInputMs=5 (:23) only after count<=0 |
| Poll task | pros_link.cpp:145-166, pros_link.h:44-46 | "communigatr", priority DEFAULT+1, period 2 ms (delay_until), `client_.poll()` under link mutex (wait <= one period) |
| Destructor | pros_usb_port.cpp:167-180 | waits 50 ms; leaks Io if a task is still inside a USB call (rx is normally blocked in read, see 1.2) |

Nothing else in the Brain tree does stdio I/O [code]: grep of brain/ for printf/cout/cerr/puts/
fwrite/stdin/stdout/stderr/serctl/fdctl//ser/ finds only pros_usb_port.cpp and snprintf into
buffers (actuGATR/src/ports.cpp:13,20; locaGATR/src/main.cpp:366,385,401; operaGATR/src/main.cpp:78)
plus host-test printf. locaGATR/operaGATR draw on pros::screen and read the controller, but never
call controller print/set_text (no radio text traffic from the apps). brain/robot/gatr2_robot.h is
config only (kLinkPort 10 PLACEHOLDER :34, kLinkBaud 115200 :35, kSendTelemetry true :60,
kTelemetryPeriodMs 100 :61).

### 1.2 What the PROS 4.2.2 kernel actually does [source, PROS 4.2.2]

Output (ser_driver.c):
- COBS is ONE global flag: `static enum { E_COBS_ENABLED = 1 } ser_driver_runtime_config;` (:88),
  set at init (:328 "start with cobs enabled"), cleared by `serctl(SERCTL_DISABLE_COBS)` (:316-317).
  ser_write_r tests it per write (:159). So after communiGATR opens USB, sout, serr, kdbg, ngtr and
  any other stream are all raw bytes. Effect is global for the program. [source]
- Streams not in the enabled set are silently discarded ("pretend like the data was shipped",
  :152-157); stdout 'sout' enabled by default (:334); stderr always delivered (:78-84).
- Raw path: `mutex_take(write_mtx, TIMEOUT_MAX)` regardless of the no-block flag (:181-185), then
  `stream_buf_send(write_stream, buf, len, noblock ? 0 : TIMEOUT_MAX)` (:110-112, :187). One write()
  call is atomic w.r.t. other writers (so an NG1 line is never split by printf), but a writer blocked
  in stream_buf_send holds write_mtx and blocks EVERY other writer (printf, the NG1 tx task).
- SERCTL_NOBLKWRITE is a no-op: ser_ctl copies the struct (`ser_file_s_t file = *(ser_file_s_t*)arg;`
  :221) and sets the flag on the copy (:236-238). Confirms the communiGATR comment.
- Named stream open: `/ser/ngtr` -> kmalloc'd ser_file_s_t, only the 4 id bytes copied (:289-291);
  `flags` is not initialised [inference: blocking vs non-blocking for ngtr depends on heap garbage;
  kmalloc zeroing not checked].
- Shared 2047-byte write stream (:28, :65-67) for ALL streams. The PROS System Daemon (priority
  MAX-2 = 14) flushes it every 2 ms: `stream_buf_recv(write_stream, ..., vexSerialWriteFree(1), 0)`
  then `vexSerialWriteBuffer(1, ...)`, printing "WARNING: some serial data has been dropped" on a short
  write (ser_driver.c:102-108; system_daemon.c:48-57, :121). No fdctl/serctl query for this buffer's
  fill level exists (ser_ctl handles only ACTIVATE/DEACTIVATE/BLKWRITE/NOBLKWRITE, :220-243).
- There is exactly one VEX serial channel in PROS stdio: channel 1 for both write (:103-104) and read
  (ser_daemon.c:107). Whether that channel is the direct-USB user port, the controller radio, or both is
  decided inside VEXos [unknown, see 7].

Input (ser_daemon.c, ser_driver.c):
- "Serial Daemon (PROS)" task, priority TASK_PRIORITY_MIN+1 = 2 (ser_daemon.c:200; rtos.h:47,55,63),
  reads channel 1 one char at a time, `task_delay(1)` when empty (:106-115), posts into a 4 KiB input
  stream (:66) with a BLOCKING post (TIMEOUT_MAX, :78-80).
- It parses in-band kernel commands from the host: 'p','R' then a|b|B|e<4 id bytes>|d<4>|c|r
  (:124-188). "pRc" = serctl(SERCTL_ENABLE_COBS) (:169-172), "pRr" = disable, "pRe"/"pRd" = (de)activate
  a stream id. Any host byte stream containing "pRc" re-enables COBS globally and breaks NG1 output.
  NG1 lines use only 'N','G','1',':','0'-'9','A'-'F','\r','\n', so they can never form 'p' [code+source].
  pros-cli terminal sends "pRb" (banner) on connect (serial/terminal/terminal.py:214-217): harmless.
- stdin read (ser_read_r :117-147): takes read_mtx with TIMEOUT_MAX, then loops
  `inp_buffer_read(0)` (forced to a 1 ms timeout, ser_daemon.c:82-93) and `continue`s while nothing
  was read. So read() BLOCKS until at least one byte, returns at '\n' or when a 1 ms wait yields no
  more, and writes a NUL at buffer[read] (:145) - matches pros_usb_port.cpp:129-131.
  [inference] The rx task therefore lives inside the kernel read holding read_mtx; its `count <= 0`
  / kNoInputMs branch only runs on an error. A second stdin reader would block on read_mtx and then
  race for bytes line by line: exactly one stdin consumer is possible.

### 1.3 Coexistence with console text [code + source]
- Pi side wraps every link frame as "NG1:" + uppercase hex + "\n" and skips lines without the marker
  (pi/naviGATR/src/impl/resources/pros_usb_link.h:1-3, :39-43); Brain decoder is the same rules
  (usb_line.h:1-6, usb_line.cpp:72-107: last marker counts, text before it ignored, \r stripped).
- Because each NG1 line is one write() under write_mtx, a printf fragment can only land between lines;
  "text" + "NG1:..\n" still decodes (last marker). Kernel banner at boot (ser_daemon.c:121) is emitted
  COBS-encoded before communiGATR disables COBS; the Pi counts it as console lines.
- With COBS off, the stock `pros terminal` (non --raw) cannot show output: V5UserDevice waits for
  0x00-delimited COBS packets (v5_user_device.py:35-49). `--raw` uses RawStreamDevice
  (pros/cli/terminal.py:26-27, :82-83; https://github.com/purduesigbots/pros-cli/blob/45aae5df262eab1246f4d74c5be7bd1a4916eb73/pros/cli/terminal.py). [source]
- Docs agree: docs/communigatr.md:236-254, docs/interfaces.md:522-524. Doc says BytePort "Neither
  blocks" (communigatr.md:224-225): true for ProsUsbPort; ProsSerialPort calls go through PROS
  claim_port -> port_mutex_take (vdml.h:51-59), which waits while the System Daemon holds all port
  mutexes each 2 ms cycle (system_daemon.c:50-56) [source; inference: short].

### 1.4 Brain-side USB latency contributors [inference from code]
request: client.poll (2 ms grid) -> SPSC slot -> tx task (<=2 ms idle sleep) -> write into PROS stream
-> System Daemon flush (2 ms grid) -> VEXos. reply: VEXos -> Serial Daemon (prio 2, 1 ms idle sleep)
-> 4 KiB stream -> rx read (per line) -> SPSC -> next client.poll (<=2 ms). Up to about 7 ms of Brain
scheduling quantization per round trip on top of VEX USB and the Pi. Not measured on hardware.

-----------------------------------------------------------------------------------------------
## 2. RS-485 path (Transport::kSmartPort) [code]

| Item | Where | Detail |
|---|---|---|
| API | pros_serial_port.cpp:295-363 | PROS generic serial on a smart port: serial_enable, serial_set_baudrate, serial_flush, serial_read, serial_get_write_free, serial_write |
| Open | :297-314 | enable, then baud+flush retried every 2 ms up to 100 ms (blocks the poll task up to ~100 ms; done outside the link mutex, pros_link.cpp:143-153) |
| Read | :323-338 | serial_read "returns only bytes already received; it never waits" |
| Write | :340-363 | all-or-nothing: needs serial_get_write_free >= len, else false ("FIFO full, not an error") |
| Errors | pros_serial_port.h:274-276 | 50 consecutive errors -> counted closed, reopened by poll task at most once per 1 s (pros_link.cpp:18,150-153) |
| Baud | pros_link.h:42, gatr2_robot.h:35 | 115200 default, must match Pi (<Baud value="115200"/>, brain_profile_rs485.xml:42) |
| Direction | pros_serial_port.h:3 | "The V5 port handles its own RS-485 direction" (no Brain-side DE). Pi side: DriverEnable gpio 6 (brain_profile_rs485.xml:43). PROS docs: "UART over RS485" [source https://pros.cs.purdue.edu/v5/pros-4/group__c-serial.html] |
| Half duplex | client.h:1-6, client_timing_gtest.cpp:1-6 | Brain is sole initiator, one request outstanding; never transmits while a Pi reply can still start (timeouts sized per op, see 3.1) |
| stdio | - | none: RS-485 mode leaves stdin/stdout/COBS entirely untouched |
| Byte time | client.h:47 | ClientConfig.byte_time = 10/115200 is NOT derived from LinkConfig.baud (pros_link.cpp:40-46). Changing kLinkBaud without byte_time leaves timeouts sized for 115200. |

-----------------------------------------------------------------------------------------------
## 3. Client scheduling (brain/communiGATR/src/client.cpp, include/communigatr/client.h)

### 3.1 Constants
| Constant | Value | Where |
|---|---|---|
| response_timeout | 60 ms from write return | client.h:46 |
| byte_time | 10/115200 s | client.h:47 |
| timeout per op | response_timeout + byte_time*max(0, req_frame + reply_max_payload + 6 - 85) | client.cpp:286-293, kBudgetFrameBytes 85 :23 |
| request_gap | 5 ms after a reply or timeout | client.h:48, client.cpp:355, :764 |
| state_period | 20 ms (GET_STATE poll) | client.h:49, next_state_ set at send :837-839 |
| link_timeout | 250 ms (connected) | client.h:50 |
| pending_retry | 20 ms | client.h:51 |
| placement | 10 sends / 1 s | client.h:52-53 |
| control | 5 sends / 1 s before first reply, retry 0.1 s, 20 s total | client.h:54-57 |
| hello_backoff | 1 s | client.h:58 |
| field_period | 0.5 s between estimate reads | client.h:59 |
| transfer_backoff | 1 s after 3 failures (kFailuresBeforeBackoff) | client.h:60, client.cpp:21 |
| telemetry_period | 100 ms (apps set it from kTelemetryPeriodMs) | client.h:61; locaGATR/src/main.cpp:70; operaGATR/src/main.cpp:137 |
| kPollsBeforeTransfer | 4 due polls a waiting transfer yields | client.cpp:22 |
| kMaxReads | 16 x 64-byte reads per receive/drain | client.cpp:19 |
| ProsLink poll period / prio / call wait | 2 ms / DEFAULT+1 / 20 ms | pros_link.h:44-46 |

Timeouts computed by the probe (Client::responseTimeout at each op's largest request) [measured-host]:
HELLO/SET_POSE/GET_STATE/PROFILE_APPLY/CONTROL 60.000 ms, TELEMETRY 60.174, READ_WHEELS 61.128,
PATH_REPORT 65.035, READ_DOC 65.642, PROFILE_WRITE 65.903.

### 3.2 Priority order of choose() [code client.cpp:869-1037]
0. No session: HELLO (resent with same bytes; :870-881)
1. Timed-out placement/control retried first (retry_first_, :883-889)
2. Placement SET_POSE (:891-908) (optionally one GET_STATE before a Pending resend, :896-900)
3. CONTROL (:910-924)
4. Profile sync PROFILE_WRITE / PROFILE_APPLY (:926-953)
5. GET_STATE when due (:955-979); before it, the TELEMETRY overdue pre-emption (:971-975)
6. READ_WHEELS (:983-990)  7. READ_DOC map chunk (:991-1005)  8. READ_DOC estimate chunk (:1006-1020)
9. PATH_REPORT (:1021-1027)
10. TELEMETRY on time (:1033-1035)
Items 6-9 go only while the poll is not due, or after 4 due polls in a row held a transfer back
(transfer_turn, :958-980). One request outstanding at a time (transmit :813-821). drain() discards
everything received before each send (:1083-1097). Only HELLO, SET_POSE and CONTROL are ever resent
(attemptFailed :761-811); everything else is one attempt.

### 3.3 TELEMETRY rules [code]
- reportTelemetry (:248-264): dropped (telemetry_dropped++) with no session, after a refusal this
  session, unknown flag bits (mask kTelemetryFlags :24-25), or wheel_count > 6; replaces an unsent
  report (telemetry_replaced++); stamps telemetry_at_ = last_poll_. No I/O.
- Slot (telemetrySlot :1041-1044): pending && after_state_ (nothing sent since the last GET_STATE
  reply; set :415, cleared on every send :850) && grid time <= that reply's time && now >= grid + late.
  => at most ONE TELEMETRY per GET_STATE reply, always the first slot after it.
- On time (late = 0): only if the poll is not due and nothing else waits (:1033).
- Overdue (late = period/2): takes the slot from a due poll or a waiting transfer, but never from a
  transfer that already yielded its 4 polls (:966-975); telemetry_overdue++.
- Grid (:840-846): next = next + period unless more than half a period late, then now + period.
- Drop if still unsent 2 periods after the report (:961-965).
- Unsupported (:358-376): reply UnsupportedOp or InvalidArgument to TELEMETRY -> telemetry_unsupported_
  for this session, session kept (unless pi_instance changed). Cleared by a new session (:454, :1143).
- Reply is header only; non-Ok counts `unexpected` (:424-429).

### 3.4 PATH_REPORT and documents [code]
- Client::reportPath (:213-246): dropped without session; replaces unsent (paths_dropped++); mode
  kPathNone forces 0 points (clear); >13 points thinned evenly keeping first/last; one attempt.
- LinkDriver::reportPath (link_driver.cpp:158-183) sends translation end points only: turn-only
  segments add just their start point (deduped), headings are not carried; allocates a std::vector
  under the link mutex (:164-165). Motion reports an empty path on start-over and stop/cancel
  (actuGATR/src/motion.cpp:172-174, :235-236) and the plan at :325-327. => PATH_REPORT loses turn-only
  segments and headings (relevant to the spectuGATR "preserve path semantics" requirement).
- Documents: READ_DOC chunks up to 96 data bytes (frames.h:246); map when state map_id != cached
  (:1255-1258); estimate at most every field_period (:1260-1276); 3 failures -> 1 s backoff.

### 3.5 ClientStats [code client.h:199-228]
requests, resends, replies, timeouts, uncorrelated, bad_frames, drained_bytes, read_errors,
write_errors, sessions, session_losses, pi_restarts, stale_hellos, unexpected, profile_writes,
doc_chunks, doc_rejects, doc_stale, maps, estimates, path_reports, paths_dropped, telemetry_reports,
telemetry_overdue, telemetry_replaced, telemetry_dropped, telemetry_refused. Plus ProsLinkStats
(poll_lock_misses, call_lock_misses, opens, serial_closes; pros_link.h:56-61) and ProsUsbStats
(frames_out, expired, short_writes, frames_in, input_full, read_errors, lines_ignored, lines_dropped;
pros_usb_port.h:210-219).

-----------------------------------------------------------------------------------------------
## 4. Bytes on the wire per op

Envelope: sync0 sync1 type len | payload | crc16 = 6 bytes (frames.h:86-93; frame_codec.h:18).
Request header 8, reply header 13 (frame_codec.h:22-24). kMaxFrameLen 128 (frames.h:19).
USB NG1 line = 4 + 2N + 1 characters for an N-byte frame (usb_line.cpp:28-40).
Values below [measured-host] from probe.exe (codec min/max lengths plus real encodes:
"GET_STATE req 23 reply 59; TELEMETRY req 68 reply 19; PATH_REPORT(13) req 124"); RS-485 time at
115200 8N1 = 86.8 us/byte [inference].

| Op | Request B | Reply (Ok) B | NG1 req chars | NG1 reply chars | RS-485 bytes/exchange | byte time |
|---|---|---|---|---|---|---|
| HELLO | 18 | 23 | 41 | 51 | 41 | 3.6 ms |
| SET_POSE | 26 | 27 | 57 | 59 | 53 | 4.6 ms |
| GET_STATE | 23 | 59 | 51 | 123 | 82 | 7.1 ms |
| PROFILE_WRITE | 23..128 | 25 | 51..261 | 55 | 48..153 | 4.2..13.3 ms |
| PROFILE_APPLY | 20 | 26 | 45 | 57 | 46 | 4.0 ms |
| READ_DOC | 22 | 33..128 | 49 | 71..261 | 55..150 | 4.8..13.0 ms |
| CONTROL | 16 | 22 | 37 | 49 | 38 | 3.3 ms |
| PATH_REPORT | 20..124 (6+8n, n<=13) | 19 | 45..253 | 43 | 39..143 | 3.4..12.4 ms |
| READ_WHEELS | 14 | 20..84 | 33 | 45..173 | 34..98 | 3.0..8.5 ms |
| TELEMETRY | 68 (54-byte body) | 19 (header only) | 141 | 43 | 87 | 7.6 ms |

Framing overhead per frame: 6 bytes binary; on USB 2x + 5 chars (GET_STATE 23 B -> 51 chars, a
122% overhead). At ~49 polls/s the USB link carries ~2.5 kB/s Brain->Pi and ~6.0 kB/s Pi->Brain;
TELEMETRY at 10 Hz adds 1.41 kB/s out, 0.43 kB/s in [inference, arithmetic].
Body headroom: payload max 122 (frame_codec.h:21); TELEMETRY request could grow by 60 bytes before
hitting 128.

-----------------------------------------------------------------------------------------------
## 5. Existing tests that quantify telemetry cost, and measured numbers

Ran: `export PATH=/c/msys64/ucrt64/bin:$PATH; cd .../scratchpad/final-brain/communiGATR &&
./communigatr_tests.exe --gtest_filter='*Telemetry*:ClientTiming.*:*BrainLink*:*Usb*'`
-> 73 tests PASSED [measured-host]. The tests print no numbers (only threshold asserts), so two
probes were built in scratch to extract them.

What the tests assert [code]:
| Test | Assertion |
|---|---|
| client_telemetry_gtest.cpp:241-257 OnePerPeriodOnAGrid | report every 1 ms: 29..31 sent in 3 s, >2500 replaced; report every 100 ms: 49..51 in 5 s, 0 replaced, 0 timeouts |
| :259-268 StatePollsKeepTheirRate | polls with >= 85% of without (comment: "about 6 to 10%" cost) |
| :475-492 SlowTelemetry.LateButNeverStarved | 30..51 sends/5 s, worst gap <= 250 ms, best >= 49 ms, <=1 exchange between polls, overdue>0, 0 drops/timeouts |
| :495-501 StatePollsKeepMostOfTheirRate | slow line: polls >= 80% |
| :506-515 SharesTheLinkWithAWaitingTransfer | paths >= 25, polls >= 4*paths, sends >= 25, worst <= 250 ms |
| :600-615 TelemetryBus default | 29..31 TM/3 s, never with poll due, overdue 0, first slot, no collision, worst poll gap <= without + TM exchange + gap + 1 ms, polls >= 85% |
| :621-636 TelemetryBus slow | 17..31 TM, overdue > 0, worst TM gap <= 250 ms, <=1 between polls, polls >= 80% |
| client_timing_gtest.cpp:60-84 | every op: timeout - (req + 40 ms Pi window + reply + release) >= 12.4 ms |
| :189-223 | worst poll gap during 128-object map transfer <= 20+18+5 ms |
| pi/naviGATR/tests/brain_link_e2e_gtest.cpp:796-838 | real Pi brain_link, 3 ms line: >= 50 TM in 10 s, worst TM gap <= 250 ms, <=1 between polls, overdue > 0 |

Probe A: communiGATR host fakes, 10 s after ready, app reports every P, telemetry_period = P
[measured-host: g++ probe.cpp linked to final-brain/communiGATR/libcommunigatr*.a; ./probe.exe].
Fake RS-485 = 115200 bytes timed, 2 ms reply delay ("slow": 8 ms). Fake USB = 2+4+2 ms fixed
latency, no byte time ("slow": 11 ms turnaround). Rig steps 1 ms (Brain poll task is 2 ms).

| Transport, line | P (s) | polls/s | worst poll gap | TM sent/s | overdue | replaced | bus busy |
|---|---|---|---|---|---|---|---|
| RS-485 default | none | 48.8 | 21 ms | 0 | 0 | 0 | 35.7% |
| RS-485 default | 0.100 | 43.6 (-10.7%) | 32 ms | 10.0 | 0 | 0 | 39.7% |
| RS-485 default | 0.050 | 39.1 (-20%) | 32 | 18.8 | 0 | 12 | 43.2% |
| RS-485 default | <=0.025 | 32.2 (-34%) | 32 | 32.3 | 0 | many | 48.6% |
| RS-485 slow | none | 46.5 | 22 | 0 | - | - | 34.0% |
| RS-485 slow | 0.100 | 40.2 (-13.5%) | 44 | 6.3 | 63 | 37 | 34.3% |
| RS-485 slow | 0.033 | 31.0 (-33%) | 44 | 15.5 | 155 | 148 | 34.7% |
| RS-485 slow | <=0.025 | 23.2 (-50%) | 44 | 23.2 | 233 | many | 35.0% |
| USB default | none | 48.7 | 22 | 0 | - | - | - |
| USB default | 0.100 | 45.1 (-7.4%) | 30 | 10.0 | 0 | 0 | - |
| USB default | 0.050 | 41.8 (-14%) | 31 | 19.1 | 0 | 9 | - |
| USB default | <=0.020 | 35.8 (-26%) | 31 | 35.8 | 0 | many | - |
| USB slow | none | 47.6 | 23 | 0 | - | - | - |
| USB slow | 0.100 | 41.2 (-13.4%) | 44 | 6.4 | 64 | 35 | - |
| USB slow | <=0.025 | 23.8 (-50%) | 44 | 23.8 | 238 | many | - |
Full table: probe.out. 0 timeouts and 0 drops in every row.

Probe B: real Pi brain_link (config brain_profile_usb.xml, Loop 100 Hz, Pi steps every 10 ms, Brain
poll every 2 ms, USB line delay each way) [measured-host: e2e_probe.exe
--gtest_filter=SpectuProbe.*; source = copy of brain_link_e2e_gtest.cpp + probe TEST, linked to
final-navi/libnavigatr_core.a]:

| line each way | P (s) | polls/s | mean/worst poll gap | TM/s | overdue | mean/max GET_STATE RTT |
|---|---|---|---|---|---|---|
| 2 ms | none | 49.7 | 20.1 / 30 ms | 0 | 0 | 6.7 / 12 ms |
| 2 ms | 0.100 | 49.1 | 20.4 / 30 | 10.0 | 2 | 4.7 / 12 |
| 2 ms | 0.050 | 41.0 | 24.4 / 30 | 19.7 | 3 | 4.3 / 12 |
| 2 ms | 0.033 | 49.0 | 20.4 / 30 | 24.7 | 5 | 4.4 / 12 |
| 3 ms | none | 48.9 | 20.5 / 40 | 0 | 0 | 9.4 / 16 |
| 3 ms | 0.100 | 42.6 (-13%) | 23.4 / 40 | 6.4 | 58 | 13.9 / 14 |
| 3 ms | 0.050 | 36.8 (-25%) | 27.1 / 40 | 12.2 | 116 | 14.0 / 14 |
| 3 ms | 0.033 | 32.6 (-33%) | 30.7 / 40 | 16.4 | 158 | 14.0 / 14 |
The sim is phase-locked (Pi at exact 10 ms multiples, Brain at 2 ms), so the 2 ms rows are
non-monotonic; treat them as "cost depends strongly on Pi loop phase", not as a rate law [inference].

Readings [inference from the above]:
- Current 10 Hz TELEMETRY costs 7-13% of GET_STATE polls and adds ~10-20 ms to the worst poll gap on
  host fakes; with the real Pi loop and a 3 ms line the effective TM rate is 6.4 Hz (overdue rule:
  at most once per 1.5 periods when the poll is always due).
- Pushing TELEMETRY toward pose rate (20-30 Hz) costs 20-50% of state polls; the slot rule caps it at
  one per state reply, so TELEMETRY can never exceed the poll rate and each one delays the next poll.
- RS-485 bytes are not the bottleneck (bus 34-49% busy); turn-taking is (reply delay + 5 ms gap +
  one outstanding request).
- Earlier claim (viz final_report.md:151, w5-cost.txt): "6 to 11% fewer at default timing and up to
  20% fewer on a slow line". Reproduced 7.4-10.7% default and 13.4-13.5% slow at 10 Hz; "20%" is the
  test threshold, not a measurement. No contradiction beyond that wording.

-----------------------------------------------------------------------------------------------
## 6. Options for Brain visualization data given the single-owner design

Hard constraints found in code:
- C1 One request outstanding; drain() discards anything received before each send (client.cpp:1083).
- C2 Pi answers only the newest request completed in a drain, and NOT AT ALL if more bytes followed it
  in the same drain (brain_link_commands.cpp:280-346, :365-371; header :15-21). On USB, ProsUsbLink
  forwards decoded NG1 frame bytes to this drain, so any extra NG1 frame arriving right after a
  request suppresses that request's reply (Brain then waits 60 ms).
- C3 Unknown op -> Pi replies UnsupportedOp (brain_link_commands.cpp:473-475). The Brain handles that
  gracefully only for TELEMETRY (client.cpp:358-376); for any other op it is incompatible() ->
  session dropped, HELLO every 1 s (client.cpp:377-381, :733-755).
- C4 Known op with a length outside its fixed range fails decode (frame_codec.cpp:555-561); the Pi
  drops it silently (brain_link_commands.cpp:320-328): no reply, Brain times out 60 ms per send, and
  telemetry_unsupported_ is never set (it needs a reply). A longer TELEMETRY against an old Pi would
  burn ~60 ms of link time every period.
- C5 Version != 4 -> UnsupportedVersion -> no link (brain_link_commands.cpp:469-472).
- C6 USB: COBS is global and off; one stdin reader; one PROS write stream shared by all output with
  no fill query; a blocked writer blocks all writers (1.2).
- C7 Publisher calls must not block. ProsLink public calls wait up to call_timeout_ms = 20 ms on the
  link mutex (pros_link.h:46, pros_link.cpp:64-82, :378-385), and the poll task holds that mutex for
  a whole client_.poll() incl. the bench_imu callback. The drive task (actuGATR ProsDrive, 10 ms,
  DEFAULT+1: pros_drive.h:27-28) already calls robot()/field()/reportPath() through this mutex.
  Client::reportTelemetry/reportPath themselves do no I/O.

(a) Piggyback a compact pose record on TELEMETRY
- Cost per record: +12..20 bytes body [inference] -> +1.0..1.7 ms per exchange on RS-485 at 115200,
  +24..40 NG1 chars on USB; timeout grows by byte_time per byte over 85 (client.cpp:291).
- No extra transaction, latest-wins and never-resent semantics already fit live pose.
- Rate is bounded by the TELEMETRY slot rules (5): ~10 Hz costs 7-13% of polls; 6.4 Hz effective when
  the poll is always due; 30 Hz would cost ~1/3 of polls. Pose age at send <= 2 periods (drop rule).
- Requires a wire change: new flag bit alone cannot carry bytes (fixed 54-byte body, frames.h:268-310);
  a longer body breaks old Pis per C4. Needs lockstep upgrade or a version/capability step (C5).
- Pi side: TELEMETRY is already "display and capture only" and posted to the DiagnosticsHub
  (brain_link_commands.cpp:532-545; header :50-60), so forwarding stays out of localization.
- Integrated-mode only; standalone Brain has no Pi to receive it.

(b) Separate viz op in the same request/reply scheme
- Each send is a full exchange (request + >=19-byte reply + Pi loop latency + 5 ms gap), so it costs
  polls like TELEMETRY; merging viz into the TELEMETRY exchange is strictly cheaper than a 2nd op.
- Must be added to choose() at lowest priority with TELEMETRY-like slot rules, and must get the
  UnsupportedOp special case (C3) or an old Pi drops the session.
- Up to 114 body bytes per exchange (122 payload - 8 header). Documents (field) could reuse the
  READ_DOC/PROFILE_WRITE chunk machinery pattern but in the Brain->Pi direction (like PROFILE_WRITE).
- On RS-485 this is the only legal shape: the Brain may only transmit in its own turn.

(c) Separate PROS output stream for viz on USB (COBS or not)
- COBS cannot be per-stream (C6): with communiGATR on USB, a viz stream is raw bytes on the same
  channel-1 byte stream as NG1. A COBS viz stream is only possible when communiGATR is on RS-485 (or
  absent) and nothing disables COBS.
- USB is full duplex, so an unsolicited Brain->Pi line needs no turn. But it must NOT be an NG1 line
  (C2 would suppress replies); use a distinct marker (e.g. "NGV:") that today's ProsUsbLink counts as
  console text and ignores (pros_usb_link.h:39-43), and add a Pi transport-level demux before
  brain_link commands.
- Must be written by the SAME tx task as NG1 (C6: a second writer blocking in stream_buf_send holds
  write_mtx and stalls link lines). Needs its own bounded lane with latest-wins coalescing, the 1-slot
  link lane always first, and a byte-rate cap because the 2047-byte PROS buffer is FIFO and unqueryable:
  every viz byte written ahead of a request delays it; if the buffer fills, link writes block or the
  48 ms lifetime expires them (ProsUsbStats.expired).
- Pi->Brain direction for viz (acks/resend requests) would have to share the one rx task and must never
  contain 'p' (kernel "pR" commands, 1.2) - uppercase hex is safe, base64 is not.
- On RS-485, (c) degenerates into (b) (C2 plus half-duplex).

(d) Other options the code suggests
- d1 RS-485 link + viz on the Brain's user port: in kSmartPort mode communiGATR never touches stdio or
  COBS (pros_link.cpp:86-90), so a spectuGATR transport task can own channel 1 (direct USB or, per
  VEX docs, controller radio) with COBS left ON and its own 4-char stream id; pros-cli's V5UserDevice
  subscribes only to sout/serr (serial/terminal/terminal.py:175-176) and would skip it. Cleanest separation found, but
  depends on unverified VEXos channel-1 routing (7).
- d2 Brain acceptance reference on GET_STATE: in integrated mode the Brain's pose IS the Pi's
  GET_STATE body plus the Brain's validity/frame decision (LinkDriver::robot, link_driver.h:29-36).
  A few bytes on the next GET_STATE request ("accepted reply id, status, frame generation") would let
  a Pi-side transport forwarder publish exactly what the Brain accepted with zero extra exchanges.
  Needs a GET_STATE length/version change (C4/C5) and a Pi reply log; only valid while the app uses
  LinkDriver's pose unmodified, so it is an optimization under a full-record API, not a replacement.
- d3 PATH_REPORT is already a Brain->Pi path channel but lossy (13 points, no headings, no turn-only
  segments, one attempt, 3.4); a spectuGATR path message needs chunking/revisioning if the Pi forwards.
- d4 Raising the RS-485 baud shrinks every exchange (bytes are ~7 ms of a ~15-20 ms GET_STATE cycle at
  115200) but ClientConfig.byte_time must follow (2) and the V5 port's supported rates are [unknown].
- Non-blocking publisher: model it on the existing SPSC slots (pros_usb_port.cpp:32-59) or a
  try-lock/seqlock latest-value mailbox per channel; the transport owner (link poll task or USB tx
  task) pulls. Never route publisher calls through ProsLink's 20 ms mutex waits (C7).

-----------------------------------------------------------------------------------------------
## 7. Doc/code notes
- docs/communigatr.md:250-254 and pros_usb_port.cpp:196 "SERCTL_NOBLKWRITE does nothing in 4.2.2":
  confirmed by PROS ser_driver.c:221,236-238 (flag set on a local copy).
- docs/communigatr.md:224-225 "Neither blocks": ProsSerialPort goes through PROS port mutexes that
  the System Daemon takes every 2 ms (short wait) [source + inference].
- pros_usb_port.cpp:135-140 kNoInputMs sleep after an empty read: with 4.2.2 ser_read_r an empty read
  cannot return (blocks until a byte), so that branch is error-only [source + inference].
- viz final_report.md:151 "up to 20% fewer on a slow line": that is the test bound; measured 13.5%.
- ClientConfig.byte_time independent of LinkConfig.baud (pros_link.cpp:40-46) - not documented as a
  coupling [code].

## Open questions / needs bench test
1. VEXos routing of serial channel 1: with a Pi on Brain USB and a controller tethered to a laptop,
   does channel-1 output go to both, one, or whichever polls first? Does controller-side UserFifo
   polling consume input/output bytes the direct USB host would otherwise get? [unknown]
   (vexide notes the controller exposes one port and stdio is fetched with UserFifoPacket:
   https://vexide.dev/blog/posts/serial-deep-dive/ - third-party, reverse engineered.)
2. vexSerialWriteFree(1) size and real V5 user-port throughput over direct USB and over the radio;
   whether the 2047-byte PROS buffer ever fills at current NG1 rates. [unknown]
3. Real GET_STATE RTT on V5 USB and RS-485 with the Pi at 100 Hz (host sims: 4-14 ms mean,
   12-16 ms max); the TELEMETRY cost depends strongly on this (7% vs 13%+ at 10 Hz).
4. Is 'ngtr' write blocking or non-blocking in practice (uninitialised flags after kmalloc)? Observe
   ProsUsbStats.short_writes with the cable pulled.
5. Supported V5 smart-port generic-serial baud rates above 115200 and turnaround behavior. [unknown]
6. Does any host tool the team uses (VS Code PROS terminal, vexcom) send "pRc" on connect, which
   would silently re-enable COBS under a running USB link? [unknown]
7. Serial Daemon runs at priority 2: under heavy DEFAULT+ CPU load, how much input latency does it
   add on hardware? [unknown]
