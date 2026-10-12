# controller-host: laptop access to a V5 Brain directly and through a V5 Controller

Scope: external research (primary sources) for the spectuGATR receiver. Read-only; nothing in
C:/GATR2-Electronics was changed. No robot hardware here: nothing below is a physical
measurement of a Brain or controller.

Basis labels: [code] file:line verified by reading source; [source] primary external source
(URL, quoted); [measured-host] run on this Windows box (command given); [inference]; [unknown].

## 0. Sources read (pinned)

| Key | What | Pin |
|---|---|---|
| PC | pros-cli (purduesigbots) | https://github.com/purduesigbots/pros-cli commit 45aae5df (2026-02-21), version file 3.5.6 |
| PK | PROS kernel 4.2.2 (the kernel our Brain apps use: brain/locaGATR/project.pros "version": "4.2.2") | https://github.com/purduesigbots/pros/tree/4.2.2 ; ser_driver.c/ser_daemon.c identical to develop-pros-4 c0897ad except copyright year [measured-host: diff of the two copies in scratch] |
| VVS | vex-v5-serial + vex-cdc (vexide), git | https://github.com/vexide/vex-v5-serial commit b3a712a1 (2026-09-22) |
| VVS052 / CDC011 | crates actually used by cargo-v5 | https://crates.io/crates/vex-v5-serial/0.5.2 , https://crates.io/crates/vex-cdc/0.1.1 |
| C5 | cargo-v5 | https://github.com/vexide/cargo-v5 commit 392c10aa (2026-04-27), version 0.12.1 |
| SDK | vex-sdk (VEXos jump-table docs) | https://github.com/vexide/vex-sdk commit d4405e2b |
| JL | v5-serial-protocol (TypeScript) | https://github.com/Jerrylum/v5-serial-protocol commit baf94584 (2022) |
| RS | Rust std VEXos stdio | https://github.com/rust-lang/rust/blob/master/library/std/src/sys/stdio/vexos.rs |
| V5X | v5x PR #329 (2026, 1-star repo, claims only) | https://github.com/v5x-dev/v5x/pull/329 |
| VEX | VEX KB + jpearman / hphillips (VEX staff) forum posts | URLs inline |

Scratch copies: C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/spectu/controller-host/

## 1. USB identities

| Device | VID:PID | Serial functions | Interface layout | Strings | Basis |
|---|---|---|---|---|---|
| V5 Brain | 2888:0501 | 2 CDC-ACM ports | if0 = system/"Communications" port (upload, CDC2 protocol); if2 = "User" port (raw stdio of user program). ttys bind to the control interfaces 1.0 and 1.2 | manufacturer "VEX Robotics, Inc"; product "VEX Robotics V5 Brain - A9EC2700" (suffix = per-Brain id); interface strings "VEX Robotics Communications Port" / "VEX Robotics User Port"; serial_number None | [source] Linux pyserial dump in https://github.com/purduesigbots/pros-cli/issues/198#issuecomment-1033891110 (2022): `'hwid': 'USB VID:PID=2888:0501 LOCATION=1-3:1.2' ... 'interface': 'VEX Robotics User Port'`, `LOCATION=1-3:1.0 ... 'VEX Robotics Communications Port'`, `'serial_number': None` |
| V5 Controller | 2888:0503 | 1 CDC-ACM port + gamepad (HID) | CDC control interface = 1 (Linux location suffix "1", macOS name suffix "2" = 1-indexed) ; interface 0 presumably the HID gamepad | lsusb: "VEX Robotics, Inc VEX Robotics V5 Controller" (no id suffix seen) | [source] jpearman (VEX) https://www.vexforum.com/t/wireless-uploading-and-terminal-usage-to-v5-brain/62790/7 : "only one serial channel and a gamepad emulation"; [code] PC v5_device.py:54,58 (mac suffix '2', location '1'), VVS serial.rs:28-29,77-81,156-161; lsusb string from third-party README https://github.com/SerrialError/2530A-Robot (weak); interface-0 = HID is [inference] |
| EXP / IQ2 / AIM / AIV / AIR | 0600 / - / 0700 / 0800 / 0a00, 0a10 | not in scope | | | [code] VVS serial.rs:25-37 |
| Brain DFU | 2888:0502 | not in scope | | | [code] JL src/Vex.ts:185-188 (1281, 1282, 1283 = 0x501..0x503) |

- VEX KB (2026-09-19): "The only exception applies to a V5 Controller which does not enumerate with a user
  serial port." https://kb.vex.com/hc/en-us/articles/21410932256660 [source]
- jpearman 2024-05-09: "The V5 controller only has one serial port, a limitation of the hardware. We can read
  data that goes to the brain user port via special commands to the controller ... There is, and never will,
  be anyway to open a serial port on the V5 controller via direct calls to read/write stdio data."
  https://www.vexforum.com/t/how-to-use-webserver/123135/12 (read via Wayback 20250908) [source]
- vexcom.exe (VEX's own CLI, installed here by the PROS extension) contains the strings "VEX V5 Communications
  Port", "VEX V5 Controller Port", "VEX V5 User Port" (likely the Windows friendly names of VEX's driver) and
  `--chan n  set radio channel 0=pit, 1=download`. [measured-host] `vexcom.exe --help`; `strings -n 6 vexcom.exe`
  in C:/Users/mbes1/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/vex-vexcom-windows (V1.0.0b23).
- Windows without VEX's driver: inbox usbser names ports "USB Serial Device (COMn)" (seen localized as
  "USB ???? (COM9)" in https://github.com/purduesigbots/pros-cli/issues/168). Name matching is unreliable on
  Windows. [source]+[inference]
- Brain SYSTEM_VERSION reply on real hardware: `TX:C9 36 B8 47 A4  RX:AA 55 A4 08 01 01 00 00 00 10 03 00`
  (product 0x10, flags 0x03). [source] same pros-cli #198 log.

### 1.1 How each tool classifies ports

| Tool | Rule | Basis |
|---|---|---|
| pros-cli | VEX if vid in [0x2888, 0x0501] or name has "VEX"/"V5". Linux/Windows: user = location endswith "2" or name/description contains "User"; system = "0" or "System"/"Communications"; joystick = "1" or "Controller". macOS: /dev name endswith 3/1/2. Fallbacks: unequal counts -> remainder; exactly 2 ports -> natural sort, user = ports[1], system = ports[0] (or the one whose description has "Brain"); else joystick ports. Controller flag: `'Controller' in description or pid == 0x503`. Multiple Brains -> prompt keyed by last token of product ("A9EC2700") | [code] PC pros/serial/devices/vex/v5_device.py:34-96; pros/cli/common.py:234-273 (259-264, 271) |
| pyserial (what pros-cli sees) on Windows | location built from SPDRP_LOCATION_PATHS plus `:x.<MI>` from the hardware id `...&MI_02`; serial_number from instance id only if alphanumeric, else parent's | [code] https://github.com/pyserial/pyserial/blob/a5c48d44/serial/tools/list_ports_windows.py#L326-L371 |
| vex-v5-serial | VID must be 0x2888; PID 0503 -> Controller; Brain-class PID -> interface 0 System, 2 User, 1 warns "controller serial port over a Brain connection"; macOS interface-1 correction and skips /dev/tty.* (uses cu.*); fallbacks: macOS last char of name 1/2/3, Windows "name order" (exactly 2 ports) | [code] VVS packages/vex-v5-serial/src/serial.rs:19-39, 55-124, 126-177, 179-213, 215-250 |
| existing GATR Pi adapter | Linux sysfs only; ttyACM* whose walk-up finds bInterfaceNumber 02 + idVendor 2888 + idProduct 0501; multiple Brains -> empty; test proves 2888:0503 if02 is NOT selected | [code] pi/navigatr/src/impl/resources/pros_usb_link.cpp:180-207; pros_usb_link.h:19-23; pi/navigatr/tests/pros_usb_link_gtest.cpp:197-204 |

Conclusion: the existing adapter cannot use a controller (wrong PID, and a controller has no raw user port);
a controller needs the CDC2 poll protocol of section 2. [code]+[source]

## 2. Wire protocol (host <-> Brain system port, or host <-> controller port)

All multi-byte integers little-endian except CRC16 (big-endian on the wire). [code] CDC011/VVS
decode.rs:125, encode.rs:19; cdc2/mod.rs:85-88; PC v5_device.py:1041.

| Frame | Layout | Basis |
|---|---|---|
| Simple command | `C9 36 B8 47 <cmd>` (no payload for the commands we need) | [code] PC vex_device.py:122-124; VVS cdc/mod.rs:61, 76-81 |
| Simple reply | `AA 55 <cmd> <len u8> <payload len bytes>` - no CRC | [code] PC vex_device.py:54-91; VVS cdc/mod.rs:64, 98-103 |
| CDC2 command | `C9 36 B8 47 56 <ecmd> <size VarU16> <payload> <crc16 BE>`; size = payload length only (excl. CRC); CRC-16/XMODEM (poly 0x1021, init 0) over all preceding bytes incl. header | [code] PC v5_device.py:1029-1043; VVS cdc2/mod.rs:51-89 |
| CDC2 reply | `AA 55 56 <size VarU16> <ecmd> <ack> <payload> <crc16 BE>`; size counts ecmd..crc; CRC over the whole packet from AA must give residue 0 | [code] PC vex_device.py:77-91, v5_device.py:964-1010; VVS cdc2/mod.rs:114-188 |
| VarU16 | < 0x80: 1 byte; else 2 bytes `(n>>8)|0x80, n&0xFF` (max 0x7FFF) | [code] PC v5_device.py:1034-1038, vex_device.py:81-84 |
| ACK / NACK | ack 0x76 = ACK; 0xFF general NACK, 0xCE CRC error, 0xD0 payload too small, 0xD1 transfer too large, ... 0xDB | [code] PC vex_device.py:18; v5_device.py:982-1001 |
| cmd ids | 0x56 USER_CDC (Brain ext), 0x58 CON_CDC (controller-only ext), 0xA4 SYSTEM_VERSION | [code] VVS cdc/mod.rs:17-35 |
| ecmd ids | 0x10 FILE_CTRL, 0x20 SYS_FLAGS, 0x22 SYS_STATUS, 0x26 RADIO_STATUS, 0x27 USER_READ (user FIFO read/write), 0x2E/0x2F KV load/save | [code] VVS cdc2/mod.rs:190-226 |

Doc/code contradiction: vex-cdc cdc2/mod.rs:44 documents the command size as "starting at payload through
crc16", but the code (cdc2/mod.rs:70-78) and pros-cli encode payload length only. Code wins. [code]

### 2.1 Messages a receiver needs

Byte strings below were computed with pros-cli's own CRC class and the verbatim logic of
`_form_extended_payload` [measured-host] `py -3` script loading pros-cli/pros/serial/devices/vex/crc.py
(check value CRC("123456789") = 0x31C3):

| Purpose | Request bytes | Reply | Basis |
|---|---|---|---|
| Identify + link state | `C9 36 B8 47 A4` | `AA 55 A4 08 b0..b7`: b0-b4 version, b5 product (0x10 Brain, 0x11 Controller), b6 flags: bit1 (0x02) controller linked to Brain by radio, bit0 (0x01) linked by cable | [code] PC v5_device.py:140-157, 638-643; VVS cdc/mod.rs:233-324 ("Bit 1 ... over a cable" / "Bit 2 ... over VEXLink") |
| Radio status (controller) | `C9 36 B8 47 56 26 00 AC 38` | payload after ACK: device u8 (0 none, 2 V5 ctrl BT, 4 V5 ctrl VEXnet, 7 host), quality u16 (0-100), strength i16, channel u8 ("Vexnet3: 5 = download, 9 = reconnecting, anything lower than 53 is pit, else comp"; BT: MTU ~240-250), timeslot u8 | [code] VVS cdc2/system.rs:882-968 (comments are vexide's reverse engineering) |
| Radio -> download | `C9 36 B8 47 56 10 02 01 01 A9 48` | ACK, empty | [code] PC v5_device.py:130, 645-653; VVS cdc2/file.rs:889-946 |
| Radio -> pit | `C9 36 B8 47 56 10 02 01 00 B9 69` | ACK, empty | same |
| Read user FIFO (stdout) | `C9 36 B8 47 56 27 02 01 40 8C 69` (channel 1, read length 0x40) | `AA 55 56 <size> 27 76 01 <n data bytes> <crc>`; size = n+5; n may be 0 ("nothing buffered"); n <= 0x40 observed by pros-cli ("RX length isn't always 0x40 (end of buffer reached)") | [code] PC v5_device.py:866-888; PR https://github.com/purduesigbots/pros-cli/pull/114 ("Fixed user_fifo_read by properly formatting the data required by the ext 0x27 command so that it would not fail with a General NACK") |
| same, vexide form | `C9 36 B8 47 56 27 02 01 00 C4 AD` (length 0) | same | [code] VVS cdc2/system.rs:970-1003; CDC011 cdc2/controller.rs:17-43 |
| Write user FIFO (stdin) | three incompatible encodings: pros-cli `[01, 00, data]` but the function returns before sending; vexide `[02, len, data<=224]`; JL `[01, len, data<=244]` marked "UNCONFIRMED" | reply ACK | [code] PC v5_device.py:890-903; VVS serial.rs:520-546; JL src/VexPacket.ts:575-590 |
| Brain identity via controller | SYS_STATUS `56 22`, schema `<x12B3xBI12xB3x`, field 13 = system_id | | [code] PC v5_device.py:852-864; that the controller relays it to the Brain is [inference] from pros-cli using it during wireless upload (v5_device.py:223-225) |

## 3. How `pros terminal` works through a controller (Q2)

Flow [code]:
1. `resolve_v5_port(None,'user')` returns the controller port with is_joystick=True (PC cli/common.py:271).
2. `terminal` wraps it in `V5WirelessPort` instead of `DirectPort`; share backend raises NotImplementedError
   (PC cli/terminal.py:73-81).
3. `V5WirelessPort.__init__` opens the port at 115200 8N1 (PC ports/direct_port.py:14-17) and immediately
   enters `DownloadChannel` (PC ports/v5_wireless_port.py:8-14).
4. `DownloadChannel.__enter__`: SYSTEM_VERSION; if product is CONTROLLER: timeout -> 2.0 s; require flag 0x02
   else raise "doesn't appear to be connected"; send FILE_CTRL download; sleep 250 ms; poll SYSTEM_VERSION every
   250 ms up to 5 s until flag 0x02 returns (PC v5_device.py:181-208). `__exit__` sends pit (210-215).
5. Read: `V5WirelessPort.read(n)` issues ONE USER_READ whenever its buffer is shorter than n
   (v5_wireless_port.py:27-32). `user_fifo_read` appends the reply data to `_serial_cache` and returns only up to
   and including the first 0x00; otherwise returns b'' (v5_device.py:878-888). `V5UserDevice.read` loops
   `port.read(1)` / `port.read(-1)` until a 0x00, COBS-decodes, splits 4-byte stream id (v5_user_device.py:35-49).
   No sleep anywhere in the loop: polling is back-to-back request/reply. Retries: 3 (pros/common/utils.py:46-57).
6. Write: `V5WirelessPort.write` -> `user_fifo_write`, which returns immediately ("Not currently implemented").
   PROS announcement 2021-03-19: "NOTE: For now wireless terminal is just read-only." (archived
   https://web.archive.org/web/20221116013744/https://www.vexforum.com/t/pros-cli-3-2-0-wireless-terminal-important/89429) [source]
7. `destroy()` closes the port BEFORE `__exit__` tries to send the pit switch (v5_wireless_port.py:16-18), so the
   switch-back likely fails; the controller falls back on its own (section 4). [code] + [inference]
8. Minor: `read(-1)` returns `buffer[:-1]`, holding one byte back until the next call (v5_wireless_port.py:27-32). [code]

Consequences for GATR [code]+[inference]:
- pros-cli wireless terminal only surfaces 0x00-terminated data. Our Brain disables COBS globally
  (brain/communiGATR/pros/pros_usb_port.cpp:195-198; kernel flag ser_driver.c:88, 159, 313-316), so with the
  communiGATR USB port open, `pros terminal` over a controller would print nothing (unless the Brain pads replies
  with 0x00, section 6.3).
- A cable-tethered controller reports flag 0x01, not 0x02; pros-cli refuses it ([source] issue #78 comment
  https://github.com/purduesigbots/pros-cli/issues/78#issuecomment-1407876144). cargo-v5 skips the channel switch
  when SYS_FLAGS bit 8 (tethered) is set (C5 src/connection.rs:82-102).

Other implementations [code]:
- cargo-v5 `terminal`: open, `switch_to_download_channel`, then loop `read_user` / stdin with a 10 ms sleep per
  iteration; never switches back to pit (C5 src/main.rs:254-258; src/commands/terminal.rs:12-37).
  `read_user` over a controller loops USER_READ (100 ms timeout, 1 retry) until non-empty data
  (VVS052 src/serial.rs:515-543). Decoder stops at the first 0x00 and requires UTF-8 (CDC011
  cdc2/controller.rs:53-85): binary data is truncated or rejected.
- cargo-v5 `field-control` reads the terminal through the controller WITHOUT switching channel
  (C5 src/commands/field_control/mod.rs:53-72, 466).
- cargo-v5 issue #34 (open, 2025-11-29): "Input to `cargo v5 terminal` gets dropped when using a controller
  connection ... perhaps this never worked in firmware." https://github.com/vexide/cargo-v5/issues/34 [source]
- VEX VS Code extension shows Brain stdout through a controller ("via special commands to the controller") and
  can expose the user port as a raw websocket ws://localhost:7071/vexrobotics.vexcode/device: "I don't modify the
  data in any way" (hphillips, VEX) https://www.vexforum.com/t/how-to-use-webserver/123135/2 [source]. Its
  implementation is closed source. [unknown]
- VEXos 1.0.6 (2019) first enabled wireless console printing via the controller (Robot Mesh announcement,
  https://www.vexforum.com/t/v5-wireless-console-output-with-robot-mesh-studio/58757). [source]

## 4. Radio channel switch: what it does (Q2)

| Claim | Basis |
|---|---|
| "Download - ... the radio will jump to a download VEXnet channel ... allows for higher data rates"; "Pit - ... the radio will remain on a pit channel, and the download will take much longer." | [source] VEX KB https://kb.vex.com/hc/en-us/articles/21410932256660 |
| "If you unplug USB from the controller, it will switch back from the high bandwidth download channel to a normal channel (what we call a pit channel) after 10 seconds ... there a limited number of download channels." Also "probably because the serial port gets closed, I think the controller can detect that as well." | [source] jpearman https://www.vexforum.com/t/controller-disconnecting-at-10-seconds/78034/3 and /10 |
| Switching drops the Brain-controller link for seconds: pros-cli users "the controller disconnects from the brain for about 5 seconds" | [source] https://github.com/purduesigbots/pros-cli/issues/112 |
| cargo-v5 treats a switch as disconnect + reconnect: waits up to 8 s for replies to stop, then up to 8 s for RADIO_STATUS channel == 5; channel 9 = "stuck", tells user to power cycle | [code] C5 src/connection.rs:104-199 |
| Community (vexide dev, not VEX): "Controller FIFO will be highly unreliable while the radio operates in the pit channel ... Even on the download channel, you'll probably run into possibly corrupted data due to bandwidth restrictions." | [source, non-authoritative] https://www.vexforum.com/t/independent-serial-usage/133244/2 (Wayback 20250712) |
| Effect of download channel on driver-control latency / joystick update rate | [unknown] - no source found |
| Whether USER_READ works on pit channel | [code] cargo-v5 field-control and v5x do it; functional evidence [unknown] |
| Competition legality / availability of download channel under field control | [unknown] - check the game manual; out of scope here |

Inference: a switch costs a multi-second control dropout, so spectuGATR must never switch implicitly; make it an
explicit user action with a warning, and never while the robot is enabled. [inference]

## 5. Throughput / latency figures (Q3)

| Item | Value | Basis |
|---|---|---|
| Brain stdio FIFO (VEXos channel 1) | 2048 B; "writes the data into a 2k fifo, that then gets sent to the other core for output via USB (or where ever, radio etc.)" | [source] jpearman https://www.vexforum.com/t/vexos-bug-report-serial-output-with-length-512-is-ignored/127844/3 ; [code] PK src/system/newlib_stubs.c:42 (`vexSerialWriteFree(1) != 2048`); SDK packages/vex-sdk-jumptable/src/serial.rs:18-32, 56-67 |
| VEXos flush cadence to USB | "flushed to USB1 roughly every millisecond by `vexTasksRun`" | [source] RS vexos.rs:43-45 (Rust std comment, vexide authors) |
| PROS flush cadence | every 2 ms the system daemon moves up to vexSerialWriteFree(1) bytes from a 2047 B PROS buffer into the FIFO | [code] PK src/system/system_daemon.c:48-57, 121; src/system/dev/ser_driver.c:28, 64-67, 102-108 |
| PROS write blocking | writes wait forever (TIMEOUT_MAX) when the 2047 B PROS buffer is full; SERCTL_NOBLKWRITE edits a stack copy and has no effect | [code] PK ser_driver.c:110-112, 181-188, 220-238 (file copied by value at 221) |
| Per-poll payload over controller | <= 0x40 B with pros-cli's request; vexide's length-0 request: max [unknown] | [code] PC v5_device.py:875-878 |
| Poll interval | pros-cli: none (back-to-back); cargo-v5: back-to-back until data, then 10 ms sleep; v5x: 500 ms reply timeout | [code] PC v5_wireless_port.py:27-32; C5 terminal.rs:35; V5X PR diff |
| USER_READ round-trip time over VEXnet | [unknown] - no primary number found | - |
| Output throughput via controller | [unknown] as a primary number. Sibling pros-serial.md quotes jpearman 2026 (https://www.vexforum.com/t/vex-radio-prevents-wireless-sending-to-brain-during-runtime/144920): ~500 B/s file send via controller, "15 bytes every 50mS" reverse link, VEXlink "at best about 500 bytes/second". I could NOT re-verify (Cloudflare 403, no Wayback snapshot). | [source, unverified by me] |
| Wireless upload speed | VEXnet 3 figure: none found; SIGBots wiki gives only legacy VEXnet 2.0 "3 KB/sec" | [source] https://wiki.purduesigbots.com/vex-electronics/vex-electronics/vexnet (legacy, not V5) |
| Direct USB hazard | a USB transfer of exactly 512 B (full-speed/high-speed packet multiple) was not delivered to Windows/Linux host apps until the next transfer; jpearman saw both transfers on a USB analyzer ("not a V5 issue") | [source] https://www.vexforum.com/t/vexos-bug-report-serial-output-with-length-512-is-ignored/127844/11 and /12 |

Budget consequence [inference]: at 64 B per round trip, the radio sustains 64 B / RTT. With an unmeasured RTT
of 20-50 ms that is ~1.3-3.2 kB/s; with the unverified ~500 B/s figure it is far less. A Brain publisher
writing faster than the drain fills 2048 B + 2047 B and then BLOCKS every PROS writer (all printf in all
tasks share write_mtx). A radio receiver therefore needs a Brain-side bounded, non-blocking, coalescing queue
and a tiny compact live-state budget (tens to low hundreds of B/s until benched).

## 6. Same bytes to both hosts, or compete? (Q4)

Evidence:
- There is one stdio FIFO per direction (SDK channel 1); the user-code core "has no idea as to where the data
  will go, that's determined by the core vexos is running on." [source] jpearman 127844/9.
- The controller path reads "data that goes to the brain user port". [source] jpearman 123135/12.
- Tropical (vexide) in 127844/8: while the direct USB user port showed the 512-byte stall, "reading FIFO off the
  controller works fine". Shows both paths exist on one Brain; says nothing about simultaneous readers. [source]
- No primary statement found on simultaneous USB user port + controller USER_READ. [unknown]

Hypotheses to discriminate [inference]:
- H1 duplicate: both hosts receive every byte.
- H2 split: one FIFO, first drainer wins; each host sees a disjoint subset (corrupts both framings).
- H3 exclusive: VEXos routes to USB when a USB host is attached/port open, radio gets nothing (or vice versa).
The Rust std comment ("flushed to USB1 roughly every millisecond") combined with the fact that the controller
path works when no USB host is present hints at routing that depends on USB state (H2/H3), but that is a guess.

Input direction: USER_READ writes (if they work at all, section 2.1) land in the same channel-1 input that the
PROS serial daemon feeds to stdin (PK ser_daemon.c:106-115), which communiGATR's USB rx task consumes as the
Pi->Brain link (pros_usb_port.cpp:126-150). A laptop writing through the controller would interleave with Pi
link input. The ser_daemon also interprets "pR?" sequences (ser_daemon.c:124-140). [code]+[inference]

Bench test (one Brain, radio, controller; Pi or 2nd laptop on Brain USB if02; laptop on controller USB):
1. Brain test app (COBS off, like communiGATR): write NUL-free records `S<seq 8 hex> T<millis 8 hex> <pad>\n`
   at 20 Hz and at 200 Hz, record lengths varying 1..9 bytes of pad (to expose 0x00 padding and 512-byte hits).
2. Host A: raw reader on Brain user port, timestamps per read. Host B: USER_READ poller (length 0x40, then 0x00,
   0x80, 0xE0), logs every raw reply with host timestamps and RTT.
3. Runs: (a) A open + B polling; (b) cable in, A closed; (c) cable out; (d) A only; (e) B only; each on pit and
   on download channel. 5 min each.
4. Classify by seq sets: complete on both = H1; disjoint with complete union = H2; one empty = H3; gaps = loss.
5. Also: B sends USER_READ writes `[02,len,data]` and `[01,len,data]`; Brain echoes stdin lines tagged; check
   whether they arrive and whether they interleave with Pi input.
6. Record joystick-update timing on the Brain (controller analog change timestamps) during max-rate polling, to
   measure any driver-control impact.

## 7. Cross-platform device concerns (Q5)

| OS | Concern | Finding | Basis |
|---|---|---|---|
| Linux | permissions | systemd default: `KERNEL=="tty[A-Z]*[0-9]|..." GROUP="dialout"` covers ttyACM*; pros-cli error text suggests "adding yourself to dialout group" | [source] https://github.com/systemd/systemd/blob/main/rules.d/50-udev-default.rules.in#L47 ; [code] PC pros/serial/ports/exceptions.py:10-15 |
| Linux | seat access without dialout | tag rule, e.g. `SUBSYSTEM=="tty", ATTRS{idVendor}=="2888", TAG+="uaccess"` in a file ordered before 73-seat-late (which runs `TAG=="uaccess" ... RUN{builtin}+="uaccess"`) | [source] https://github.com/systemd/systemd/blob/main/rules.d/73-seat-late.rules.in#L17 ; rule text is [inference]. Arch uses group uucp instead of dialout [inference] |
| Linux | ModemManager probing | strict policy probes ttyACM only when confident it is a modem; add `ENV{ID_MM_DEVICE_IGNORE}="1"` for 2888 as belt and braces | [source] https://modemmanager.org/docs/modemmanager/port-and-device-detection/ ; VEX's bInterfaceProtocol value [unknown] |
| Linux | stable identity | no iSerialNumber on the Brain (2022 dump). /dev/serial/by-id needs ID_SERIAL + interface num (60-serial.rules:22-26); ID_SERIAL falls back to vendor_model, and the model string carries the Brain id, so by-id is expected to be `usb-VEX_Robotics__Inc_VEX_Robotics_V5_Brain_-_<id>-if02`; by-path is per physical port | [source] https://github.com/systemd/systemd/blob/main/rules.d/60-serial.rules#L11-L26 ; exact by-id name [inference]; repo docs already recommend by-id (pi/navigatr/docs/usb_localization_bench.md:56) |
| macOS | node names | use /dev/cu.usbmodem* (call-out), not /dev/tty.* (vexide skips tty.*); suffix = interface+1: Brain ...1 system, ...3 user; controller ...2 | [code] VVS serial.rs:68-75, 91-94, 145-173; PC v5_device.py:51-54; [source] jpearman https://www.vexforum.com/t/vexcode-printf-error/58807/4 ("The controller should show as a device ending with 2") |
| macOS | identity | name prefix derives from USB location (no serial), so it changes with the port; IOKit USB Product Name carries the Brain id | [inference] |
| Windows | enumeration | composite interfaces get device ID `USB\VID_v(4)&PID_d(4)&MI_z(2)` (z = bInterfaceNumber): MI_00 system, MI_02 user, controller MI_01. Use SetupAPI hardware ID, not friendly names or COM order | [source] https://learn.microsoft.com/en-us/windows-hardware/drivers/install/standard-usb-identifiers ; serialport-rs parses MI_ (https://github.com/serialport/serialport-rs/blob/main/src/windows/enumerate.rs#L116) |
| Windows | stable identity | "An instance ID contains serial number information, if supported by the underlying bus, or some kind of location information." With no USB serial, COM number/instance follow the physical port | [source] https://learn.microsoft.com/en-us/windows-hardware/drivers/install/instance-ids ; per-port behaviour [inference] |
| Windows | driver | inbox Usbser.sys loads on `USB\Class_02&SubClass_02`; VEX driver optional (only changes friendly names) | [source] https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-driver-installation-based-on-compatible-ids ; "optional" is [inference] from issue #168 naming |
| Windows | name-order fallback bug | vex-v5-serial sorts `port_name` as strings: "COM10" < "COM9" [measured-host `py -3 -c "print(sorted(['COM9','COM10']))"`]; pros-cli uses natural sort. Both label the LOWER name system, HIGHER user, while both comments say the lower is user | [code] VVS serial.rs:179-213; PC v5_device.py:75-92 |
| all | exclusive owner | controller has one port shared by upload and terminal ("the admin port needs to be shared by the PROS cli and whatever is directing output to the terminal" - jpearman 62790/7); pros-cli share backend unimplemented; vexide plans a v5d daemon. Windows COM opens are exclusive; on POSIX use TIOCEXCL/flock | [source]+[code] PC cli/terminal.py:73-74; https://vexide.dev/blog/posts/serial-deep-dive/ ; TIOCEXCL is [inference] |
| all | line settings | tools open 115200 8N1 (ignored by CDC); pyserial asserts DTR and RTS by default; whether the Brain gates output on DTR [unknown] - match pyserial | [code] PC direct_port.py:14-17; https://github.com/pyserial/pyserial/blob/master/serial/serialutil.py#L210-L211 |
| all | hot plug | Brain power-off or unplug removes both ports; controller port stays while the radio link drops (flags lose 0x02) | [inference] |

## 8. Licenses (Q6)

| Project | License | Porting note | Basis |
|---|---|---|---|
| pros-cli | MPL-2.0 | file-level copyleft: a new C++ file that "contains any Covered Software" is a Modification (MPL 1.10(b)) and stays MPL-2.0 with source available; the rest of the receiver may use any license (3.3 Larger Work). Reimplementing from the protocol facts above avoids this | [code] PC LICENSE:1, 49-57, 185-191 |
| PROS kernel | MPL-2.0 | only matters if kernel code is copied | [code] PK LICENSE header |
| vex-v5-serial, vex-cdc, cargo-v5, vex-sdk | MIT | keep copyright + permission notice with substantial portions | [code] Cargo.toml `license = "MIT"`; LICENSE files |
| v5-serial-protocol | MIT | same | [code] JL LICENSE, package.json:28 |
| v5x | MIT | same | [source] GitHub API license field |
| GATR2 repo | no LICENSE at repo root | licensing choice is the user's | [measured-host] `ls C:/GATR2-Electronics` |

Not legal advice. [inference]

## 9. Minimum protocol for the C++ receiver

### 9a. Directly connected Brain (user port)
1. Discover VID 0x2888 PID 0x0501 interface 2: Linux sysfs bInterfaceNumber "02" (as the Pi adapter does);
   Windows hardware ID `...&MI_02`; macOS IOKit bInterfaceNumber 2, else /dev/cu.usbmodem*3. Identity: product
   string suffix (Brain id), else physical location. [code]+[source]
2. Open raw, 8N1, DTR+RTS asserted, exclusive. Read bytes as they come; there is no VEX framing on this port.
   Content is whatever PROS emits: COBS frames with 4-byte stream ids and 0x00 delimiters if COBS is on, raw merged
   bytes (banner, kprintf, all streams) if off - our Brain turns it off globally. [code] PK ser_driver.c:150-196
3. Writes (optional) go to Brain stdin raw; avoid "pR" sequences (ser_daemon.c:124-140); the Pi link already hex-
   wraps for this reason (pi/navigatr/src/impl/resources/pros_usb_link.h:1-3). [code]
4. Optional identity via the system port (if0): SYSTEM_VERSION, SYS_STATUS system_id, KV robotname. Strict
   request/reply, one outstanding. [code]
5. Expect occasional host-side holdback of exact-512-byte transfers; a periodic short write releases it. [source]
6. Reconnect by rescanning on port loss. [inference]

### 9b. Through a V5 Controller
1. Discover VID 0x2888 PID 0x0503 (single CDC port; Windows MI_01, Linux interface 01, macOS cu.*2). [code]+[inference]
2. Open raw 8N1, exclusive. The receiver is the ONLY client of this port (no pros upload concurrently unless routed
   through the receiver). [source]
3. SYSTEM_VERSION: require product 0x11 and flags & 0x03; 0x02 = radio, 0x01 = cable (skip any channel logic);
   neither = no Brain, retry every 250 ms. [code]
4. Optional RADIO_STATUS for link quality diagnostics (channel, quality, strength) - separate, low-rate. [code]
5. Channel: do NOT switch automatically. Offer an explicit "download channel" action: FILE_CTRL [01 01]; expect
   the link to drop; poll RADIO_STATUS every 250 ms (<= 8 s gone, <= 8 s back, channel == 5); channel 9 -> tell
   user to power-cycle. Revert with [01 00] or by closing the port (~10 s fallback). [code]+[source]
6. Poll loop: one USER_READ outstanding at a time (`56 27 [01 40]`); on reply verify CRC residue 0, ecmd 0x27,
   ack 0x76, strip channel byte; append data; empty = idle. Timeout 100-500 ms, bounded retries; resync by scanning
   for `AA 55`; do not flush input before each request (pros-cli does, vex_device.py:98) unless the previous reply
   timed out. Poll pacing adaptive: back-to-back while data arrives, back off (e.g. 10-20 ms) when empty. [code]+[inference]
7. Treat the stream as bytes, not UTF-8 (unlike vex-cdc). Until padding is benched, require a NUL-free
   self-delimiting encoding on this path (e.g. the existing NG1 hex lines) or strip only trailing 0x00 per reply
   (v5x's claim: "The brain pads a FIFO reply out to a word boundary with NULs"), never mid-stream. [code]+[unknown]
8. Stdin through the controller: treat as unsupported (pros-cli disabled, cargo-v5 #34 broken, three encodings).
   Never write in combined mode (it would merge with the Pi link's stdin). [code]+[source]
9. Brain identity through the controller: SYS_STATUS once at connect (not per poll). [inference]

## 10. Implications for spectuGATR topologies [inference unless noted]

- Standalone Brain + controller: supported by the protocol above, read-only, low bandwidth. The Brain publisher
  must write compact, rate-limited, non-blocking records (PROS blocks writers when the FIFO backs up [code]).
- Brain-Pi USB (communiGATR) + laptop on controller: UNSAFE until the section 6 bench passes; under H2 both the
  NG1 link and the viewer stream corrupt. Until then, get Brain visualization to the laptop through the Pi
  (single owner of the Brain user port, explicit fan-out), or use the controller path only when the Pi link is not
  on USB (e.g. RS-485).
- Brain-Pi RS-485 + laptop on controller: stdio channel is then free for spectu output; still subject to radio
  budget and the write-blocking rule.
- pros-cli / VEXcode terminals cannot run alongside the receiver on the same controller port.

## Open questions / needs bench test

1. Q4 routing (H1/H2/H3) and stdin merge through the controller - section 6 bench.
2. USER_READ RTT and sustained bytes/s on pit vs download channel, VEXnet vs Bluetooth mode; max useful read
   length (0x40 vs 0x00 vs larger).
3. Whether replies are NUL-padded (v5x claim) and whether binary 0x00 survives the controller path.
4. Driver-control impact of (a) download channel, (b) back-to-back polling.
5. Descriptor dump per OS: controller interface layout (HID at 0?), bInterfaceProtocol, iSerialNumber for Brain and
   controller on current VEXos; Windows instance IDs; macOS cu names; by-id link names.
6. DTR gating: does the Brain drain/hold output when the user port is closed or DTR is low?
7. Does USER_READ on the Brain's direct system port (v5x claim) work, and does it interact with an open user port?
8. Channel revert timing with the port held open but idle.
9. Competition rules for radio telemetry and download-channel use (not researched).
10. jpearman 2026 throughput quotes (thread 144920) need direct confirmation.
