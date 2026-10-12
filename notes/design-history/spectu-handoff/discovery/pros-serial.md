# Discovery: PROS serial/stdio on the V5 Brain (topic: pros-serial)

2026-09-28. Read-only. No hardware here: nothing below is a physical observation.
Basis tags: [code] file:line read, [source] primary external source (URL),
[measured-host] run on this PC (command given), [inference], [unknown].

## 0. Sources (pinned)

| Short name | What | Pin |
|---|---|---|
| K | PROS kernel source | github.com/purduesigbots/pros tag 4.2.2 = af11debde205318af2dbdc1319385c8c0d7756ba. Links below: https://github.com/purduesigbots/pros/blob/4.2.2/<path>#L<n> |
| CLI | pros-cli | github.com/purduesigbots/pros-cli @ 45aae5df262eab1246f4d74c5be7bd1a4916eb73 |
| VVS | vex-v5-serial (vexide host protocol) | github.com/vexide/vex-v5-serial @ b3a712a16c9c6819b69faacfee2284826a138070 |
| C5 | cargo-v5 | github.com/vexide/cargo-v5 @ 392c10aa480974201731ca9b771714f4f64d0b0d |
| SDK | vex-sdk (vexide SDK bindings) | github.com/vexide/vex-sdk @ d4405e2bd463d6357eb18b3d7b7c8f73187cd231 |
| VX | vexide | github.com/vexide/vexide @ 7c622a14dffc67a171c27d17603e7b5b7fcd710e |
| RS | Rust std VEXos stdio | https://github.com/rust-lang/rust/blob/master/library/std/src/sys/stdio/vexos.rs (last commit 0ede3fe48c0d373d3673459218e1b9e7eaa667b8) |
| JX | PROS JINX branch | github.com/purduesigbots/pros branch jinx @ f2001aa44a5c714e503436a50210b53b521b792d (2020-05-07), src/jinx/jinx.c |
| LL | LemLib v0.5.0 logger | https://github.com/LemLib/LemLib/tree/v0.5.0/src/lemlib/logger |
| LG | LemLog | github.com/LemLib/LemLog @ d56f33c3187d2a75e9b0197685de483ba65b0408 |
| F | VEX forum posts; "jpearman" = VEX engineering staff, "hphillips" = VEX (VS Code extension) | URLs per row |

Kernel used by this repo: 4.2.2 [code] brain/locaGATR/project.pros ("version": "4.2.2"),
brain/operaGATR/project.pros, brain/locaGATR/include/pros/version.h:19-23.
Clones for re-checking: scratchpad/spectu/pros-serial/{pros-4.2.2,pros-cli,vex-v5-serial-full,cargo-v5,vex-sdk,vexide,LemLog}.

## 1. PROS 4.2.2 stdio architecture

Data path [code]:

```
task write()/printf ->  vfs _write (vfs.c:116-124)
  -> ser_write_r (ser_driver.c:149-196): stream enabled? COBS on? take write_mtx
  -> PROS stream buffer 2047 B (ser_driver.c:28,65,336)
  -> every 2 ms, PROS System Daemon (prio 14): ser_output_flush (system_daemon.c:47-56,121,126)
       moves min(buffered, vexSerialWriteFree(1)) B -> vexSerialWriteBuffer(1,..) (ser_driver.c:102-108)
  -> VEXos channel-1 FIFO (2048 B) -> CPU0 -> USB user port or radio (see 3)

VEXos channel-1 input -> Serial Daemon (prio 2) polls vexSerialReadChar(1), 1 ms sleep when empty
  (ser_daemon.c:106-115,200) -> intercepts "pR?" commands -> 4 KB input stream buffer (ser_daemon.c:66-80)
  -> ser_read_r for ANY fd read (stdin) (ser_driver.c:117-147)
```

### 1.1 Stream ids, custom streams, COBS

| Fact | Basis |
|---|---|
| A stream id is 4 bytes, the 4 ASCII chars as a little-endian uint32 ("sout" = 0x74756f73). It lives in the per-fd driver arg `ser_file_s_t {stream_id, flags}` | [code] ser_driver.c:30-39, 41-44; apix.h:814-831 |
| Reserved ids: sinp (stdin), sout, serr, kdbg on fds 0-3 | [code] ser_driver.c:41-55, 338-341; vfs.c:40 |
| Enabled-by-default: sout (added to a global set at init). serr is "guaranteed delivery" (always sent, cannot be (de)activated). kdbg and every custom stream are dropped until ACTIVATEd. | [code] ser_driver.c:76-84, 333-334 |
| Writes to a non-enabled stream are discarded but report full success (returns len) | [code] ser_driver.c:152-157 |
| Custom stream: `open("/ser/xxxx")`. VFS routes paths starting "/ser" to ser_open_r (vfs.c:105-107). Name must be <= 4 chars (ENAMETOOLONG); "sout"/"sin"/"serr" return the reserved fds; anything else kmallocs a new arg and copies strlen(name) bytes of the id (ser_driver.c:258-292). Max 31 fds total incl. 4 reserved (vfs.c:37,40). | [code] |
| Custom-stream arg is kmalloc'd and NOT zeroed: `flags` is uninitialized; names < 4 chars leave id bytes uninitialized. kmalloc = FreeRTOS heap_4, no memset. | [code] ser_driver.c:289-290; src/rtos/heap_4.c:106 |
| close() on a /ser fd does nothing (arg leaked, stream stays activated) | [code] ser_driver.c:198-201 |
| COBS on (default at boot): each write() becomes one packet `COBS(stream_id[4] + payload) + 0x00`, encoded into a stack VLA of the caller (~len+6 B of caller stack) | [code] ser_driver.c:159-179, 328; src/common/cobs.c:21-97 |
| Host demux: pros-cli decodes each 0x00-delimited packet, first 4 bytes = topic; `pros terminal` subscribes only sout/serr (+kdbg shown), unknown topics are skipped unless promiscuous. This is the documented coexistence mechanism for custom streams. | [source] CLI pros/serial/devices/vex/v5_user_device.py:35-49; pros/serial/terminal/terminal.py:175-176, 218-233 |
| COBS off: the same write() goes out as raw bytes; no id, no delimiter. Stream-enable filtering still applies. | [code] ser_driver.c:180-195 |
| DISABLE_COBS is GLOBAL: one static `ser_driver_runtime_config` bit for all streams, all tasks, the rest of the program (until something re-enables) | [code] ser_driver.c:88, 159, 313-318 |
| Only one writer at a time: global write_mtx. COBS-off path takes it with TIMEOUT_MAX even for "noblock" fds | [code] ser_driver.c:60-61, 165, 182 |
| Blocking write waits (holding write_mtx) until the WHOLE encoded message fits in the 2047 B buffer; TIMEOUT_MAX = portMAX_DELAY and INCLUDE_vTaskSuspend = 1, so it never times out. A single write whose encoded size > 2047 can never fit: that task hangs forever holding write_mtx, and every later printf/serial write in the program blocks. | [code] ser_driver.c:110-112, 172, 187; src/rtos/stream_buffer.c:513-547; include/pros/rtos.h:89; FreeRTOSConfig.h:168. Hang itself: [inference] from that code |
| Non-blocking fd would write min(len, free) bytes (partial) and still return len | [code] stream_buffer.c:662; ser_driver.c:172-179, 187-194 |
| kprintf goes to kdbg (fd 3), which is not enabled by default, so kernel warnings such as "some serial data has been dropped" are invisible unless kdbg is activated | [code] include/kapi.h:40; ser_driver.c:106, 78-84 |
| Stack-overflow hook bypasses the driver: raw vexSerialWriteBuffer(1,...) regardless of COBS | [code] src/system/rtos_hooks.c:112-114 |
| Boot banner is printed by the Serial Daemon at start (COBS on at that time); ENABLE_BANNER(false) macro suppresses it | [code] ser_daemon.c:121; apix.h:667-702 |

### 1.2 serctl / fdctl semantics (4.2.2)

| Call | Action | Effect | Basis |
|---|---|---|---|
| serctl | SERCTL_ACTIVATE (10), extra = id as uint32 | add id to global enabled set; id serr -> -1/EIO | [code] ser_driver.c:296-304 |
| serctl | SERCTL_DEACTIVATE (11) | remove id; serr -> -1/EIO | ser_driver.c:305-312 |
| serctl | SERCTL_ENABLE_COBS (14) / SERCTL_DISABLE_COBS (15) | set/clear the one global bit | ser_driver.c:313-318 |
| serctl | anything else (incl. 12/13 BLKWRITE/NOBLKWRITE) | -1, errno EINVAL | ser_driver.c:319-321 |
| fdctl(ser fd) | ACTIVATE / DEACTIVATE | same, for the fd's own id; serr silently 0 | ser_driver.c:223-232 |
| fdctl(ser fd) | SERCTL_BLKWRITE (12) / NOBLKWRITE (13) | NO EFFECT: modifies a by-value copy of the arg (bug). Matches the repo comment "SERCTL_NOBLKWRITE does nothing in 4.2.2". | [code] ser_driver.c:221, 233-238 |
| fdctl(/dev/N fd) | DEVCTL_FIONREAD (16), DEVCTL_SET_BAUDRATE (17), DEVCTL_FIONWRITE (18) | generic smart-port serial through VFS | [code] src/system/dev/dev_driver.c:110-116; apix.h:872-890 |

Host-to-Brain kernel commands, parsed from channel-1 input by the Serial Daemon
before stdin sees them [code] ser_daemon.c:124-191:

| Bytes | Effect |
|---|---|
| pRa | stderr "I'm alive!" |
| pRb / pRB | small / large banner (pros terminal sends pRb on start: CLI terminal.py:214-216) |
| pRe + 4 id bytes / pRd + 4 id bytes | serctl ACTIVATE / DEACTIVATE that id |
| pRc / pRr | serctl ENABLE_COBS / DISABLE_COBS (global) |
| p + other | both bytes passed to stdin |

### 1.3 stdin semantics

| Fact | Basis |
|---|---|
| Input is one byte stream, not multiplexed; the fd arg is ignored ("serial reads aren't stream-based"). Any fd read on the ser driver reads the same bytes. | [code] ser_driver.c:117-119 |
| ser_read_r takes read_mtx with TIMEOUT_MAX, then blocks until >= 1 byte arrives; returns at '\n' or when input is momentarily empty after >= 1 byte; writes a NUL at buffer[count] (one past the data, so a full-length read overruns by 1). | [code] ser_driver.c:120-146 (repo handles it: pros_usb_port.cpp:129-131) |
| Consequence: a blocked reader holds read_mtx while waiting, so a second stdin reader (fgets/std::cin/another library) blocks, then races for bytes line by line: exactly one stdin consumer is workable. | [inference] from ser_driver.c:121-146 |
| Serial Daemon posts each byte with TIMEOUT_MAX into the 4 KB buffer: if nobody reads stdin and 4 KB accumulate, the daemon blocks for good (pR commands stop working; VEXos input FIFO then fills). | [code] ser_daemon.c:78-80; hang: [inference] |
| Serial Daemon runs at priority 2 with a 1 ms poll: any CPU-bound user task at priority >= 3 delays stdin delivery. | [code] ser_daemon.c:106-115, 200; rtos.h:47-63; effect [inference] |

## 2. VEX SDK layer (below PROS)

| Fact | Basis |
|---|---|
| User stdio is SDK serial channel 1: vexSerialWriteChar/WriteBuffer/ReadChar/PeekChar/WriteFree(channel). SDK docs: "Use `1` for stdio". Jump table 0x898-0x8ac. PROS uses only WriteBuffer, WriteFree, ReadChar on channel 1. | [source] SDK packages/vex-sdk-jumptable/src/serial.rs:6-67; [code] ser_driver.c:103-104, ser_daemon.c:107-110 |
| Channel-1 output FIFO is 2048 B. VEX: "vexSerialWriteBuffer (not really a public API) just writes the data into a 2k fifo, that then gets sent to the other core for output via USB (or where ever, radio etc.)" and "[the user code core] has no idea as to where the data will go, that's determined by the core vexos is running on." | [source] jpearman https://www.vexforum.com/t/vexos-bug-report-serial-output-with-length-512-is-ignored/127844/3 and /9; [code] newlib_stubs.c:42 (`vexSerialWriteFree(1) != 2048`); [source] RS:92 STDOUT_BUF_SIZE = 2048 |
| VEXos drains that FIFO "roughly every millisecond by vexTasksRun"; Rust std flush spins until WriteFree == 2048 and says it "may block for up to a millisecond". | [source] RS:43-49, 80-85. Whether this holds with NO host attached: [inference] likely (else a cable-less flush would hang), not stated |
| Full FIFO: vexSerialWriteBuffer accepts what fits (returns count); Rust chunks by 2048 and flushes first. PROS never offers more than WriteFree, else logs the (invisible) kdbg drop warning. PROS itself never drops in blocking mode; writers block instead. | [source] RS:50-72; [code] ser_driver.c:102-108 |
| Stdin FIFO size: Rust uses STDIN_BUF_SIZE = 4096 | [source] RS:91 (VEXos internal size itself: [unknown]) |
| Behavior when the FIFO is full and nothing drains it (USB port closed / no controller poller): [unknown]. If VEXos stops draining, PROS buffer fills and every write blocks holding write_mtx (all printf in all tasks). | [inference] |
| Direct Brain USB is USB 2.0 HS (512 B bulk IN packets); FS-only hosts misbehave | [source] https://www.vexforum.com/t/using-v5-brain-with-usb-fs-12mbps/105278/1-2 |
| Known host-side quirk: a user-port transfer of exactly 512 B is not delivered on Windows/Linux hosts until the next transfer (VEX saw it on the bus: "not a V5 issue"). | [source] 127844/11-12. Relevance to NG1 (occasional stuck 512 B flush chunk until next write): [inference] |

## 3. Physical routing: Brain USB user port vs controller radio

### 3.1 Primary statements

| Statement | Basis |
|---|---|
| Brain USB = two CDC ports: system/admin (interface 0) and user (interface 2); "this port can be opened independently from the admin port" | [source] jpearman https://www.vexforum.com/t/wireless-uploading-and-terminal-usage-to-v5-brain/62790/7; VVS serial.rs:91-110; KB https://kb.vex.com/hc/en-us/articles/15614471185812 |
| V5 controller USB = ONE serial port (+ gamepad). "We have to tunnel user code IO for the terminal through the primary serial port as additional admin messages ... the admin port needs to be shared by the PROS cli and whatever is directing output to the terminal." | [source] 62790/7 (2019); jpearman https://www.vexforum.com/t/how-to-use-webserver/123135/12 (2024): "The V5 controller only has one serial port ... We can read data that goes to the brain user port via special commands to the controller ... There is, and never will, be anyway to open a serial port on the V5 controller via direct calls to read/write stdio data." |
| Controller -> Brain stdin works but is slow: "very little bandwidth in the reverse link ... try perhaps 15 bytes every 50mS"; "I could easily send an 8k file with no errors at approx 500 bytes/second ... via the controller"; "It's not optimized for input to the V5, more for output"; VEXlink radio-to-radio "at best about 500 bytes/second, that's about the theoretical max for over the air comms". | [source] jpearman https://www.vexforum.com/t/vex-radio-prevents-wireless-sending-to-brain-during-runtime/144920/2, /4, /5 (2026) |
| Controller output direction throughput: no primary number. Community (PROS/vexide dev, not VEX): "Controller FIFO will be highly unreliable while the radio operates in the pit channel ... switch the controller/radio into their respective download channels ... Even on the download channel, you'll probably run into possibly corrupted data". | [unknown] number; [source, non-VEX] Tropical https://www.vexforum.com/t/independent-serial-usage/133244/2 |
| No second user channel will be added; VEX recommends smart-port generic serial for extra streams: "the user cpu directly controls them and does not need to pass anything through shared memory for cpu0 to output via USB." V5 has "no active development ongoing". | [source] jpearman https://www.vexforum.com/t/how-to-communicate-with-my-debugger/143210/8 (2026-01) |
| VEX VS Code extension exposes the user port (Brain or controller) as a raw websocket ws://localhost:7071/vexrobotics.vexcode/device (?id=<pid> since ext 0.6.0); data unmodified | [source] hphillips 123135/2, /4; 123135/16 |

### 3.2 Controller protocol as implemented by the open tools

| Item | pros-cli | vex-v5-serial / cargo-v5 |
|---|---|---|
| Discovery | VID 0x2888 or 0x0501, name/location heuristics; controller = PID 0x503 or "Controller" in description; macOS by device suffix 1/2/3; Windows falls back to name order (CLI v5_device.py:34-95; cli/common.py:271) | VID 0x2888; Brain PID 0x0501 (iface 0 system, 2 user), controller PID 0x0503 (VVS serial.rs:20-37, 76-110); macOS/Windows fallbacks (serial.rs:125-180) |
| Read stdout via controller | CDC2 0x56 / ext 0x27 with payload [channel=1, len=0x40], polled; returns <= 64 B per round trip; caches until a 0x00 (expects COBS) (v5_device.py:866-889) | UserDataPacket channel 1, no length; 100 ms handshake timeout; reply decoded only up to the first 0x00 and must be UTF-8 (VVS serial.rs:491-518; cdc2/system.rs:970-1045) |
| Write stdin via controller | disabled: user_fifo_write returns immediately ("Not currently implemented") (v5_device.py:891-903) | UserDataPacket, chunks <= 224 B, channel **2** (commented "stdio channel") (serial.rs:520-546). Jerrylum v5-serial-protocol uses channel 1 for the same packet. Which is right: [unknown] |
| Radio channel | switches controller to "download" channel (FILE_CTRL 0x10, [1, 1]) before terminal, back to "pit" after; waits ~0.25 s steps up to 5 s (v5_device.py:181-214, 646-653) | switches to download (radio status channel 5) unless tethered (SYS_FLAGS bit 8) or Bluetooth (245); up to ~16 s reconnect waits (C5 connection.rs:82-199; main.rs:254-257) |
| Port sharing | one process owns the controller port (solo backend; "share" not implemented) (cli/terminal.py:35-81) | vexide plans a daemon "v5d" so several programs can share one Brain connection (https://vexide.dev/blog/posts/serial-deep-dive/) |

Implications [inference]: a laptop receiver on the controller must implement CDC2
(header C9 36 B8 47, 0x56/0x27 poll, CRC16) itself or reuse vex-v5-serial; it must
not use vex-v5-serial's reply decoder for binary data (stops at 0x00, requires UTF-8);
it becomes the exclusive owner of the controller COM port (PROS CLI, VEXcode and the
receiver cannot all open it); the effect of the download-channel switch on driving,
competition-control and latency is [unknown].

With COBS OFF (current repo): pros-cli wireless terminal shows nothing even with
--raw (V5WirelessPort.read only returns data once a 0x00 arrives, cache grows)
[inference from CLI v5_wireless_port.py:27-32 + v5_device.py:880-889]; `pros terminal`
non-raw on the direct user port also shows nothing (waits for 0x00) [inference from
v5_user_device.py:38-42]; `--raw`, cargo-v5 terminal and VS Code show the raw text.

### 3.3 Fan-out vs competition (the key question)

- Output: whether channel-1 output is COPIED to both the Brain USB user port and the
  controller user FIFO, sent to one of them, or DRAINED by whichever reads first:
  **[unknown]**. No primary source says. jpearman's "determined by the core vexos is
  running on" confirms the routing is VEXos CPU0 policy, invisible to PROS.
- Input: controller-injected bytes do reach the user program's stdin [source] 144920/5
  (+ OP's fgets(stdin) use). Whether they MERGE with Brain-USB user-port input into the
  same channel-1 input FIFO when both are attached: **[unknown]**, but PROS has only one
  input source (vexSerialReadChar(1)), so if both are delivered, they interleave in one
  stream [code] ser_daemon.c:107 + [inference].
- If they merge: laptop keystrokes via controller would split/corrupt NG1 lines from the
  Pi (the frame drops by the line rules), and a typed/sent "pRc" would re-enable COBS
  globally [inference].

Minimal bench test to settle it (one Brain, radio, controller, Pi on Brain USB, laptop
on controller USB):

1. Brain test program (PROS 4.2.2): serctl(SERCTL_DISABLE_COBS); task at 10 ms writes one
   line `T <seq> <millis> <writefree>\n` via a single write(); a second task reads stdin
   lines and counts per-source tags (`P<seq>` from Pi, `L<seq>` from laptop) and
   malformed lines; show counters and vexSerialWriteFree(1) on the Brain screen.
2. Pi: pyserial on the Brain user CDC (2888:0501 iface 2), log every line with host time;
   send `P<seq>\n` every 20 ms.
3. Laptop: cargo-v5 terminal (or a 30-line script on vex-v5-serial) on the controller;
   log output; send `L<seq>\n` every 100 ms (reverse-link friendly).
4. Runs: (a) both hosts reading; (b) Pi port closed, laptop reading; (c) Pi reading,
   laptop not polling; (d) neither host (watch writefree / whether T lines stall);
   (e) laptop in pit vs download channel.
5. Verdict: both logs contain every seq -> fan-out; complementary gaps -> shared drain;
   one side empty while the other reads -> exclusive routing. Stdin counters show
   whether L and P lines merge and whether lines get interleaved. (d) settles the
   "cable out" blocking question. (e) gives Brain->laptop throughput and loss.
6. Add joystick-to-Brain update timing (controller analog read timestamp jitter) during
   (e) to see if polling/download channel affects driving.

## 4. Smart-port generic serial (RS-485)

| Fact | Basis |
|---|---|
| PROS C API serial_enable / set_baudrate / flush / read / write / get_read_avail / get_write_free / peek / read_byte / write_byte map to vexDeviceGenericSerial* on the port's device handle; C++ pros::Serial wraps them; also /dev/<port> via VFS + DEVCTL_* | [code] K src/devices/vdml_serial.c:25-104; brain/locaGATR/include/pros/serial.hpp:32-321; K dev_driver.c:110-116 |
| Independent of the USB/radio user channel: different SDK calls, no ser_driver, no COBS, no stdin, no write_mtx | [code] as above vs ser_driver.c; [source] jpearman 143210/8 ("user cpu directly controls them") |
| Half-duplex RS-485, max user baud 921600, FIFO in/out 1024 B each, output flushed "as fast as possible (down to ~10us or so)"; generic serial "does not use buffered IO" | [source] VX packages/vexide-devices/src/smart/serial.rs:9-14, 54-58, 115-119; K include/pros/serial.h:108-111 |
| Repo uses 115200 (gatr2_robot::kLinkBaud) -> about 11.5 KB/s total, shared by both directions | [code] brain/robot/gatr2_robot.h:35; communigatr/pros_link.h:42; rate [inference] 8N1 |
| Repo write is all-or-nothing against serial_get_write_free; read never waits | [code] brain/communiGATR/pros/pros_serial_port.cpp:49-89 |

## 5. Prior art: custom telemetry next to stdout

| Project | Mechanism | Coexistence | Basis |
|---|---|---|---|
| PROS JINX (abandoned 2020 branch, never merged; "jinx has since been removed since that project is abandoned") | `open("/ser/jinx")` + fdctl ACTIVATE; low-prio daemon (MIN+2); binary msgpack: 'D' + u32 millis base + records {size, u8 time offset, u16 id, value}, flushed every 20 ms into a 1 KB buffer; schemas 'S' (name, format) sent once per id and flushed every 2 s; tracked variables sampled every 50 ms | COBS stream id "jinx"; terminal ignores unknown topics; needs COBS on (binary with zeros) | [source] JX src/jinx/jinx.c:12-14, 33-36, 64-94, 103-155, 176-192, 240-265; Tropical 143210/6 |
| LemLib v0.5.0 TelemetrySink | text on stdout: format "TELE_{level}:{message}TELE_END", wrapped in ANSI save/restore cursor + clear so terminals hide it; BufferedStdout task prints ONE queued message per 50 ms | in-band text markers (like NG1) | [source] LL telemetrySink.cpp:7-11, stdout.cpp:7-10, buffer.cpp:34-44 |
| LemLog Terminal sink | optional serctl(SERCTL_DISABLE_COBS) (global) and serctl(SERCTL_NOBLKWRITE, "sout") which serctl rejects (EINVAL) | same global-COBS hazard as communiGATR | [source] LG src/lemlog/logger/sinks/terminal.cpp:10-17; [code] ser_driver.c:319-321 |
| VEX VS Code extension | vexcom reads the user port (Brain) or tunnels via controller; raw websocket fan-out to local clients | host-side single owner + fan-out | [source] 123135/2, /12 |
| vexide v5gdb | GDB packets over stdio channel 1 ("StdioTransport") conflicts with prints; VEX advised smart-port serial instead | none; confirms one channel | [source] 143210/1, /8; https://internals.vexide.dev/technical/debugger |

## 6. What communiGATR's USB port changes globally (repo, verified)

ProsUsbPort::open() [code] brain/communiGATR/pros/pros_usb_port.cpp:182-222, used only
when LinkConfig.transport == kUsb (pros_link.cpp:84-89; RS-485 mode never touches stdio):

1. `::open("/ser/ngtr", O_WRONLY)` once (:188-193): one of 31 fds; kmalloc'd arg with
   **uninitialized flags** (ser_driver.c:289-290). If bit 0 happens to be set the fd is
   non-blocking: a line written while the PROS buffer is nearly full is silently
   truncated and still reported as fully written (:116-121 counts it as frames_out).
   [inference] low probability; not fixable via fdctl (BLKWRITE is a no-op).
2. `fdctl(fd, SERCTL_ACTIVATE)` (:197): adds "ngtr" to the global enabled set.
3. `serctl(SERCTL_DISABLE_COBS)` (:198): clears the single global COBS bit. From then on
   all PROS serial output of the whole program (printf/stdout, stderr, kdbg if active,
   any other /ser stream incl. a future spectuGATR stream) is raw on channel 1. Nothing
   re-enables it (:194, 201). Does not change input parsing or raw SDK writes.
4. Receive task reads STDIN_FILENO forever at priority 8 (:126-160, 214-215): it is the
   only practical stdin consumer (read_mtx held while waiting, 1.3).
5. Transmit task writes each NG1 line with one write() (:110-116); the line (<= 261 B,
   usb_line.h:21 with kMaxFrameLen 128, translaGATR/frames.h:19) goes in whole under
   write_mtx (blocking path). While it waits for buffer space, every other task's printf
   also waits (write_mtx), and vice versa: console bursts delay NG1 lines
   (head-of-line on one 2047 B buffer). The header's "stalls only the transmit task"
   holds for the link caller only.
6. Before open(): the PROS boot banner already went out COBS-framed (ser_daemon.c:121);
   the Pi ignores it as non-NG1 text.

Remote hazards to that configuration [inference from code]:
- Any host bytes "pRc" on stdin re-enable COBS globally (ser_daemon.c:169-172); "pRdngtr"
  deactivates the NG1 stream and writes then vanish while reporting success
  (ser_driver.c:152-157). NG1 input itself cannot trigger this (alphabet N G 1 : 0-9 A-F \n).
- If COBS were re-enabled, NG1 lines <= 249 chars (frames <= 122 B) still parse on the
  Pi (COBS code byte + "ngtr" land before the marker), longer ones get a 0xFF code byte
  inside the hex and are dropped. [measured-host] `py -3 cobs_ng1_check.py` in
  scratchpad/spectu/pros-serial (Python port of K src/common/cobs.c:58-97 + the NG1 rules
  of docs/interfaces.md:493-523): frames 8-122 B parsed_ok=True, 123-128 B False.
- If the controller path merges stdin (3.3), laptop input interleaves with Pi input.

No other Brain code in the repo uses stdio (grep printf/cout/cin/fgets/stdin outside
include/pros, tests, sim: only pros_usb_port) [code].

## 7. Consequences for the spectuGATR transport design [inference]

- One owner of channel 1 per Brain program. With communiGATR on USB, spectuGATR must
  not open its own stdin reader, must not call serctl, and cannot use a COBS stream
  (COBS is global and off). Visualization bytes on channel 1 must go through the same
  writer/framing (e.g. another line marker or a frame type inside NG1), or elsewhere.
- COBS on + own 4-char stream id is only possible when nothing disables COBS (Brain
  standalone, or communiGATR on RS-485). Then pros terminal, JINX-style, keeps working
  and ignores the viz topic. Keep every write <= ~1.9 KB encoded (2047 B limit, 1.1).
- In RS-485 mode the Brain USB user port is free: it can carry viz to the Pi (or a
  laptop) independently of the localization link, a separate physical channel.
- Controller path: receiver = CDC2 client on PID 0x0503, polling 0x56/0x27; budget is
  unknown and low (input ~300-500 B/s per VEX; output unquantified); prefer compact
  binary-safe framing that survives 0x00 (the controller tunnel carries COBS 0x00
  delimiters for pros-cli [inference from its design]), or ASCII if vex-v5-serial is
  reused.
- Do not run a controller poller and the Pi USB link at the same time in production
  until 3.3 is bench-settled.

## 8. Doc/code contradictions noticed

| Doc | Code |
|---|---|
| PROS docs Filesystem/Serial: `fopen("serr","w")` writes stderr (https://pros.cs.purdue.edu/v5/tutorials/topical/filesystem.html) | paths without "/ser" go to the SD card driver (vfs.c:105-114); must be "/ser/serr" |
| apix.h:713-721 serctl example uses SERCTL_SET_BAUDRATE; fdctl example opens "serial" | no such macro in 4.2.2; "serial" routes to the SD card (vfs.c:112-113) |
| apix.h:834-849 BLKWRITE/NOBLKWRITE "for the file" | no effect (ser_driver.c:221, 233-238) |
| LemLog passes SERCTL_NOBLKWRITE to serctl | serctl returns EINVAL (ser_driver.c:319-321) |
| VVS write_user and read_user both say "stdio channel" | channel 2 vs 1 (serial.rs:500, 530) |
| communiGATR header: blocked write "stalls only the transmit task, never the caller" | true for the link caller; other tasks' printf also block on write_mtx (6.5). "Cable out" blocking itself is unverified (2, 3.3 run d). |

## Open questions / needs bench test

1. Channel-1 output routing with Brain USB user port + controller both attached:
   fan-out, exclusive, or shared drain? (3.3 runs a-c) [unknown]
2. Does controller-injected input merge into the same stdin as Brain-USB input? [unknown]
3. Does the VEXos 2048 B FIFO drain when no host reads (cable out, port closed, no
   poller)? Decides whether PROS writers block (all printf) or data is discarded. [unknown]
4. Brain -> laptop throughput and loss via controller in pit vs download channel;
   per-poll max payload of 0x56/0x27; whether 0x00 and non-UTF-8 bytes pass. [unknown]
5. Does switching to the download channel affect driving/controller latency, and is it
   allowed/possible under competition or field control? [unknown]
6. Correct channel byte for controller writes (1 vs 2). [unknown]
7. Do real NG1 flush chunks ever hit the exact-512 B host quirk on the Pi (Linux)? [unknown]
8. Probability that the kmalloc'd /ser/ngtr flags are non-zero on a real Brain (print
   the fd's behavior under a filled buffer). [unknown]
