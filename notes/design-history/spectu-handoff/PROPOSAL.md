# spectuGATR design proposal

2026-09-28, revised after the evidence, coverage and simplicity critiques (Revision notes at the end).
Design only: nothing is implemented, and nothing ran on a Brain, Pi, controller or radio.
Inputs: requirements.md, discovery/*.md, viz-handoff/final_report.md, restructure/SPEC.md and restructure/plan/.

## 0. Executive summary

spectuGATR is one product with four parts:
- a small versioned protocol (six record kinds);
- a Brain publisher library;
- a portable receiver/server;
- the existing spectaGATR viewer, moved in.

The Brain owns the core scene: pose, attitude, the field it accepted, paths and execution. naviGATR adds
Pi instrumentation only. The receiver combines sources by identity and mapped clocks, never by arrival time.

**Transport.** One-way `SG1:` hex lines on the Brain's single stdio channel (channel 1), written only by
that channel's one writer. No new request/reply exchanges, no second stdin reader, no serctl.
- **Standalone Brain.** The receiver reads the Brain through the V5 controller (CDC2 USER_READ polling)
  or on the direct USB user port. The line format parses with COBS on or off.
- **Brain-Pi USB + viz.** communiGATR's USB transmit task writes spectu lines in a capped aux lane, at
  most one per loop, after the link frame. The Pi's link resource splits them from the NG1 lines into a
  relay queue inside its commands drain (same worker, same read budget). The debug-side adapter decodes
  them for the Pi-hosted viewer and forwards them verbatim to laptop receivers. Localization and world
  estimation never see them.
- **Brain-Pi RS-485 + viz.** communiGATR never touches stdio in RS-485 mode (F8), so the Brain publishes
  exactly as standalone: over the radio with the Pi on the network, or over a USB cable to the Pi (D19).

**Why not new link ops.** In host sims with the real Pi loop, each extra exchange costs 17-20 ms of link
time: 12-14% of GET_STATE polls at 10 Hz and 23.6-25.9% at 20 Hz [measured-host disc:evidence-sim s3].
One-way lines cost no exchanges. In the same host rig, SG1 lines at 4-16 kB/s (one per 2 ms loop) left
poll gaps, RTT and timeouts unchanged and polls/s within 0.1% [measured-host design:sim]. The rig has no
Brain byte time or FIFO, so the Brain-side byte cost is still unmeasured (B7).

**Latency conflict.** Through the Pi relay as built, Pico sample -> receiver receipt is about 27 ms mean /
36 ms p95. The same model's Pi-authoritative path is 18 / 29, so Brain authority costs about +8 ms mean
and +7 ms p95 [estimate design:chain_model_sg1].
- Against the reported ~25 ms (Windows, p50 24.9 / p95 35.5) it is roughly level. That baseline likely
  carries Windows timer delay, so on a Pi the relative cost may be larger [inference disc:latency-chain s6].
- Phase 1 adds no latency fixes. The event-driven relay (debug side, about -2.4 ms mean) is a later,
  measured step. A Pi reader on arrival (about -3.6 ms more) changes the localization link, so it is
  offered separately (D10).
- Radio latency is unknown until benched (B2).

**Field.**
- Send the planning field in full (definition 500 B + estimate 364 B [measured-host disc:wire-budget s2],
  about 2.3 k characters as lines [estimate 5.5]): at start, on each accepted revision (ahead of
  samples), and on a carousel.
- Draw the 22 KB of display assets [measured-host disc:wire-budget s2] only when they are keyed by the
  same field_id the Brain publishes.
- Never substitute the Pi's newer field. Landmarks are shown as errors from nominal, the form world
  estimation outputs after the restructure.

**Unknowns that gate topologies** (3.5): channel-1 routing with the Pi on USB and a controller attached
(B1); controller throughput (B2); what the output FIFO does with no reader (B5); the Brain-side cost of
the aux lane (B7); the real GET_STATE round trip (B8).

**Top decisions for you** (section 8): which Brain pose to publish (D1); accepting the ~+8 ms relay cost
(D10); retiring PATH_REPORT for display (D4); the 3D view in naviGATR-only mode (D5); RS-485 viz over the
radio or a USB cable (D19); keeping the stdio transport explicit until B5 (D17).

## Conventions

- Basis tags:
  - [code] path:line (tree ae457ee); [code via disc:<file> sN] means the code refs listed in that
    discovery section;
  - [source] Sn (URL list at the end), or a local design doc by path (restructure/SPEC.md);
  - [measured-host] a host run or host simulation. `disc:<file> sN` is spectu-handoff/discovery/<file>.md
    section N. `design:<path>` is a run made for this revision under
    C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/spectu/design/;
  - [estimate] arithmetic or a model; [inference] reasoned from the cited code or source;
  - [unknown] always names the bench test that settles it (Bn, 3.5).
  - `R:N` is line N of spectu-handoff/requirements.md.
- Paths:
  - `brain/`, `translaGATR/` and `pi/naviGATR/spectaGATR/` are unchanged by the restructure (the disk
    folder on this box is `pi/navigatr`).
  - `nav:` is `pi/naviGATR/src/` in the restructured layout. Plug-in folders follow
    restructure/plan/FINAL.md and move_map_final.tsv (readable plural nouns; resources grouped by family):
    `pipeline/resources/{cameras,pico,links,...}/`, `pipeline/commands/collectors/`,
    `pipeline/publishing/publishers/`,
    `pipeline/localization/observation_models/`. Appendix B maps each to its old path.
  - Line numbers are from ae457ee; read them with `git show ae457ee:<old path>`. Restructure steps 1-2
    move files and split system.cpp; step 3 changes world estimation and inspection internals without
    changing test outcomes; moved files get include edits and re-sorted include blocks. New-layout lines
    can therefore differ, and W7 re-cites them once the restructure lands [source restructure/SPEC.md,
    restructure/plan/FINAL.md C1, C7].
- Names:
  - receiver: the spectuGATR host program;
  - adapter: naviGATR's source-specific publishing side (`nav:debug/inspection`);
  - core scene: pose, attitude, field, path, targets, execution;
  - Pico sample: the sensor sample time every latency span starts from (latprobe encodes it in heading).

## 1. Current implementation inventory

| Piece | Where | Today | Fate in spectuGATR |
|---|---|---|---|
| Viewer | `pi/naviGATR/spectaGATR/` (19 files, 8797 lines) | three.js scene, graphs, panels, replay. Single source, `navigatr.inspect/2` only. Every scene element except the relayed PATH_REPORT and TELEMETRY is Pi-sourced [code via disc:viewer s1-2] | Moves to `spectuGATR/viewer/` unchanged (M0), then becomes multi-source (M1-M6, 7.6). Generic modules stay at the viewer root (dom, series, latency, transforms, bundle, smoothing, Plot/GraphPanel); the Pi panels move to `viewer/navigatr/` |
| Viewer tests | `pi/naviGATR/tools/viewer/` (viewer_tests.py, stub_server.py, unit_tests.js), `tools/perf/` | 9 scenarios. This session on the stub: unit 41/41, live 14/14, reconnect 9/9, replay 20/20, paths 15/15; on the binary: live 16/16 [measured-host disc:viewer s6] | Move to `spectuGATR/tests/viewer/`; the stub gains a view/1 multi-source mode |
| Inspection server | `nav:debug/inspection/http_server.{h,cpp}` (227+1614), `json_writer.h`, `viewer_assets.h`, `cmake/embed_assets.cmake` | HTTP + WebSocket: one in-flight frame, a reliable FIFO, one replaceable slot per channel with a priority [code via disc:pi-side s4], a ping-acked 64 KiB window [code nav:debug/inspection/http_server.h:92] | Extracted as `spectugatr_server`, about 2.5k LOC incl. zip and the capture route [code wc -l via disc:pi-side s4.1]; naviGATR links it. So far: http_server.cpp and zip_writer.cpp compile standalone; a probe exe of http_server plus the embedded viewer links statically (1.40 MB) and serves the page; the 23 existing server tests pass in today's navigatr_tests.exe [measured-host disc:pi-side s4.1, disc:xplat-host s1.3]. The extracted target has not been built |
| Inspection service and document | `nav:debug/inspection/inspection_service.cpp` (1350), `inspection_document.cpp` (2052) | inspect/2 from the Pi System: state 30 Hz default (clients up to 60), diag 4 Hz default (max 20), telemetry at most 20 Hz, previews [code nav:debug/inspection/inspection_service.cpp:65-67; nav:debug/inspection/inspection_config.h:47-48, 69-70] (was src/runtime/inspection_config.h) | Stays in naviGATR as the adapter and the Pi's publisher. Pi pose, field and trail are relabeled as Pi instrumentation. Adds the view/1 envelope, Brain decode for the Pi viewer, the brain_frames relay and a Pi channel table (5.9) |
| Capture | `nav:debug/capture/`: recorder (~810 generic LOC), bundle (1389), zip (184), http routes | `gatr2.capture/1` on `pi_host_us`: pre and post windows, fault triggers, bounds [code via disc:pi-side s5] | Stays the Pi capture, untouched. The receiver capture (5.8) is a raw log that reuses only zip_writer and the CSV escaping |
| TELEMETRY (op 12) | `brain/communiGATR/src/client.cpp:248-264, 1033-1044`; `nav:pipeline/commands/collectors/brain_link_commands.cpp:532-554` | 54 B body every 100 ms: VEX roll/pitch plus motion and wheel targets. With the 100 Hz Pi loop, 6.7-7.0 arrive per second and GET_STATE polls drop 12-14% [measured-host disc:evidence-sim s0]. The attitude is also a Pi estimator input [code nav:brain_profile/brain_profile_builder.cpp:537-538, 605-610] | Kept for its functional attitude (D11). Its display role moves to spectuGATR |
| PATH_REPORT (op 10) | `brain/communiGATR/src/link_driver.cpp:158-183`, `client.cpp:213-246` | At most 13 xy points; no headings, no turn-only segments; thinning can draw lines the robot never drives [code link_driver.cpp:158-183; client.cpp:229-241; measured-host disc:brain-owners s3.3] | Superseded for display by the path document (D4) |
| Brain link | `brain/communiGATR` (Client, ProsLink, ProsUsbPort); `nav:pipeline/resources/links/pros_usb_link.cpp`, commands, publishing | NG1 hex lines on USB and COBS off for the whole program [code brain/communiGATR/pros/pros_usb_port.cpp:194-198]; one stdin reader [code pros_usb_port.cpp:126-160]; poll task every 2 ms [code pros_link.h:44]; GET_STATE every 20 ms, one request outstanding [code client.h:49] | Unchanged apart from neutral hooks (5.4) and an SG1 split in `pros_usb_link` |
| Pico diag and hubs | `nav:debug/diagnostics/` (hub, link_monitor, instrumentation); Pico frame 0x14 | Pi instrumentation with drop-and-count rings [code via disc:pi-side s5] | Unchanged; reaches the viewer through the adapter only |
| Field documents | `nav:pipeline/publishing/publishers/field_documents.cpp`; `translaGATR/link_documents.h:154-174, 227-247` | Map 24 + 28n bytes (Override field: 500 B, 6 chunks); estimate 24 + 20n (364 B, 4 chunks) [measured-host disc:wire-budget s2] | Stay the functional Pi -> Brain path. The Brain republishes what it accepted |
| World estimation output | `nav:pipeline/world_estimation/` after restructure step 3 | Per landmark `LandmarkError{dx, dy, dtheta}` from the map's nominal pose, in field axes; (0, 0, 0) means as built. The Brain wire keeps full poses [source restructure/SPEC.md] | The viewer shows landmark state in this error form for both sources (4.5) |

## 2. Dependencies and data ownership

```
 COMPILE TIME   (A --> B: A depends on B)

   brain/spectuGATR  include/ src/ pros/  ---------------------------------+
     (Publisher, transport; std + PROS only; no GATR library)              |
   brain/spectuGATR/integrations/ --> communiGATR, actuGATR, investiGATR   |
     (optional; the only GATR dependencies)                                v
                                                             spectuGATR/protocol
   spectuGATR/host  sources/ decode/ combine/ capture/  --->  (records, SG1 line codec,
     server/ platform/                                        crc; std only)
        ^                 ^                                                ^
        |                 |                                                |
   nav:debug/inspection (naviGATR adapter) --> host/server, host/decode ---+
   spectuGATR/viewer (one browser app, embedded by the host AND by naviGATR)

 RUNTIME

   BRAIN (owns the core scene)
     communiGATR poll task --ClientObservers----------> integrations --+
     actuGATR drive task  --PathSink, DriveObserver---> integrations --+--> Publisher slots
     app or user code (tilt, channels, events) ------------------------+    + event FIFO
                                                                            | pulled by the one owner
          ONE channel-1 writer: communiGATR usb-tx aux lane  (USB link on)
                                StdioTransport task          (otherwise)
                 |                                               |
        (b) Brain USB cable                      (a),(c) radio via controller, or USB
                 v                                               v
   PI (instrumentation only)                         RECEIVER (laptop)
     nav:pipeline/resources/links/pros_usb_link    sources/controller | sources/brain_usb
       NG1 --> commands (localization, unchanged)        |
       SG1 --> brain_viz_lines (live ring + durable)     decode --> combine --> server
               --> adapter: decode for the Pi viewer;    capture (raw log)
                   brain_frames verbatim --WS----------> sources/navigatr (needed in (c))
     hub, monitors, camera --> adapter (inspect/2)                --> viewer (browser)
```

Boundaries:
- The Brain publisher (`include/`, `src/`, `pros/`) depends only on the standard library, PROS (in
  `pros/` only) and `spectuGATR/protocol`.
- `integrations/` is optional, and it is the only place that includes communiGATR, actuGATR or
  investiGATR headers.
- communiGATR and actuGATR get neutral hooks (`std::function` members named by `using` aliases, as the
  existing `bench_imu` [code brain/communiGATR/include/communigatr/client.h:68]) and never include spectuGATR.
- Localization and world estimation never see viz bytes. On the Pi, debug/inspection includes only the
  stage-level resource type `nav:pipeline/resources/brain_viz_lines.h` (new, next to `brain_imu_bench.h`
  [source restructure/plan/move_map_final.tsv:147]), never the pros_usb_link plug-in.

Standard input and output per component:

| Component | INPUT | OUTPUT | Runs on |
|---|---|---|---|
| Publisher (`brain/spectuGATR`) | producer calls (5.1) | slots and the event FIFO | caller tasks, copy only |
| Transport owner (StdioTransport, or the usb-tx aux lane) | `nextLine()` | SG1 lines on channel 1 | its own task, or communigatr-usb-tx |
| Pi link resource + adapter (`pros_usb_link`, `nav:debug/inspection`) | Brain port bytes; System snapshots, hub, brain_viz_lines | NG1 frames to commands; inspect/2, view/1 Brain messages, brain_frames | estimation worker (split); inspection thread |
| Receiver (`spectuGATR/host`) | device bytes, Pi WebSocket messages | view/1 messages; capture bundles | one thread per source; server thread |
| Viewer (`spectuGATR/viewer`) | view/1 (and old bundles in replay) | pixels | browser main thread |

Namespaces mirror the folders: `spectugatr::protocol`; `spectugatr` for the Brain public API (users write
`spectugatr::Publisher`); `spectugatr::integrations::{communigatr, actugatr, investigatr}`;
`spectugatr::host::{sources, decode, combine, server, capture, platform}`. naviGATR keeps `navigatr` and
adds aliases for the server types.

## 3. Supported connection topologies

### 3.1 Transport facts every topology depends on

| # | Fact | Basis |
|---|---|---|
| F1 | PROS stdio has one channel (1) for all output streams and for stdin. One 2047 B buffer and one write_mtx: a writer blocked on a full buffer blocks every writer | [source S1 :28, 65, 110-112, 181-188] |
| F2 | COBS on/off is one global flag. communiGATR turns it off on USB and never back on | [code brain/communiGATR/pros/pros_usb_port.cpp:194-198]; [source S1 :88, 159, 313-318] |
| F3 | In practice there is one stdin reader: the read holds read_mtx while it waits. On USB that reader is communiGATR's rx task | [source S1 :117-147]; [code pros_usb_port.cpp:126-160]; [inference] |
| F4 | The host bytes `pRc` re-enable COBS globally | [source S2 :124-191] |
| F5 | Where channel-1 output goes (USB user port, controller, or both) is decided inside VEXos, invisibly to PROS | [source S8 /9]; [unknown B1] |
| F6 | The V5 controller has one serial port. Brain stdout is read through it by request/reply USER_READ (ext 0x27), at most 64 B per poll with pros-cli's request | [source S7]; [source S4 v5_device.py:866-889] |
| F7 | Pi: `pros_usb_link` is the sole tty owner by convention only (no TIOCEXCL or flock; a second reader such as pros terminal would split bytes). Lines without `NG1:` are counted and dropped. Decoded NG1 frame bytes after a request in the same drain suppress its reply; non-NG1 lines do not (probe 3) | [code nav:pipeline/resources/links/pros_usb_link.cpp:149, 216-253; nav:pipeline/commands/collectors/brain_link_commands.cpp:365-372; pi/naviGATR/docs/usb_localization_bench.md:28-30]; [measured-host disc:pi-side s1.4] |
| F8 | In RS-485 mode communiGATR never opens /ser, reads stdin or calls serctl. Smart-port generic serial bypasses the stdio driver | [code brain/communiGATR/pros/pros_link.cpp:84-91]; [source S14] |

### 3.2 (a) Standalone Brain

**Simplest supported: controller radio, the intended workflow.**
- On the Brain, `Publisher` plus a `StdioTransport` writes `SG1:` lines to the custom stream `/ser/sgtr`
  (activated). COBS is left as the program has it. The transport starts only when user code starts it
  (D17).
- Path: VEXos -> radio -> controller USB -> `spectugatr --source controller` (read-only CDC2 polling).

**Additional: direct Brain USB.** The user port (interface 2) carries the same lines
(`--source brain-usb`) [source S7, S8 /9; unknown B1 while a controller also polls].

**Demonstrated (host only):**
- SG1 lines of 8-122 B frames parse after PROS COBS wrapping with stream id `sgtr`; 123 B and longer do
  not [measured-host design:cobs_sg1_check.out, a port of PROS cobs.c from disc:pros-serial s6].
- The CDC2 request bytes were computed with pros-cli's own CRC [measured-host disc:controller-host s2.1].

**Untested:** every physical behavior: B2 (throughput), B3 (NUL padding), B4 (download channel), B5 (no
reader), B6 (DTR), B9 (descriptors), B11 (`/ser/sgtr` flags), B12 (Brain CPU). No hardware demonstration
exists.

### 3.3 (b) Current Brain-Pi USB + visualization

**Simplest supported: relay through the Pi.**
1. The Brain writes SG1 lines through communiGATR's USB transmit task, in an aux lane served after the
   link frame, at most one line per loop, within a byte budget.
2. The Pi's `pros_usb_link` splits SG1 lines into `brain_viz_lines` during the commands-stage drain. They
   run on the estimation worker and share the 4 x 256 B per-call read budget
   [code nav:pipeline/resources/links/pros_usb_link.cpp:216-268; nav:pipeline/commands/collectors/brain_link_commands.cpp:280-299];
   their parse cost is [unknown B7]. They never reach the FrameReader, commands or localization.
3. The adapter (debug side) decodes the frames with the shared decode library for the Pi-hosted viewer,
   and forwards them verbatim to laptop receivers on a `brain_frames` channel, each with its Pi arrival
   time labeled as relay time.
4. Viewing: the Pi-hosted viewer (SSH tunnel, as today), or a laptop receiver
   `spectugatr --source navigatr=HOST`.

The controller is not used for viz while the Pi is on USB, until B1 passes.

**Demonstrated:**
- An `SG1:` line after an NG1 request in one drain: the request bytes are delivered, and the line is
  counted as console text (probe 3) [measured-host disc:pi-side s1.4]. That the reply is then sent follows
  from the drain counting only decoded frame bytes [inference, code brain_link_commands.cpp:365]; the 7.2
  split test asserts it.
- 1539 B of non-NG1 lines ahead of a request push the request to the next read call (probe 4)
  [measured-host disc:pi-side s1.4]. This is why the aux lane sends one line per loop within a burst cap.
- Host rig (real Client, real naviGATR System, the Pi's real line parser; no Brain byte time or FIFO):
  SG1 lines at 4, 8 and 16 kB/s, one per 2 ms loop, left poll gap p95/max, RTT and timeouts identical to
  spectu off, and polls/s within 0.1%, on usb2/3/5 lines with TELEMETRY off and at 100 ms. Without the
  one-per-loop limit, 4 kB/s with 512 B bursts was still unchanged; 16 kB/s with 2 kB bursts raised the
  poll gap max to 86 ms with up to 8 timeouts per 100 s, and 64 kB/s cut polls to 23-37/s
  [measured-host design:sim/sg1_sweep_j500.md].

**Untested:** B7 (V5 USB character rate, PROS buffer fill, Pi parse cost), B8 (real round trip), B10
(`pRc` from host tools), B12 (Brain CPU), B13 (Pi -> laptop WebSocket), and A9 (the `/ser/ngtr` flags the
aux lines inherit).

### 3.4 (c) Brain-Pi RS-485 + visualization

**Simplest supported: the Brain publishes exactly as standalone.**
- stdio is free (F8), so the StdioTransport owns channel 1.
- The laptop runs `spectugatr --source controller --source navigatr=pi.local` (or `brain-usb` when
  tethered). Pi instrumentation arrives over the network.
- This is the only phase-1 reason for the laptop's WebSocket client (about 300 LOC) and cross-source clock
  mapping [estimate disc:xplat-host s2.5]; option (c2) below avoids both (D19).

**Not in phase 1: a relay over RS-485.** One-way frames in the Brain's post-reply gap plus a Pi demux
would change half-duplex timing on the localization link, and disc:pi-side s1.5 D reads it as unsafe: the
Pi refuses to transmit while input is pending [code nav:io/half_duplex.cpp:95-99].

**Demonstrated:** nothing ran. The separation is established from code and source only (F8).

**Untested:** the same as (a), plus B13.

### 3.4b (c2) Option: RS-485 + a USB cable to the Pi (D19)

- In RS-485 mode the Brain's USB user port is free [code brain/communiGATR/pros/pros_link.cpp:84-91]; a
  cable to the Pi carries only SG1 lines from the StdioTransport, with COBS on (lines still parse, 3.2).
- On the Pi, a viz-only resource (a `pros_usb_link` instance that no pipeline stage reads, drained by the
  adapter's thread; about 100 LOC [estimate]) feeds `brain_viz_lines`.
- Viewing is as in (b). The laptop needs no WebSocket client and no cross-source clock mapping in phase 1.
- Cost: one more cable on the RS-485 robot (your call, D19); the same B5, B6, B11 unknowns as (a).
- Demonstrated: nothing; established from code only.

### 3.5 Unresolved hardware facts

The user runs every bench test; agents prepare programs, scripts and expected-output sheets uncommitted
(7.2).

| Id | Unknown | Bench test | Gates |
|---|---|---|---|
| B1 | Channel-1 routing with the Pi on Brain USB and a laptop on the controller: duplicate, split, or exclusive? Does controller input merge with Pi input on stdin? | Protocol of disc:pros-serial s3.3: <br>- The Brain (COBS off) writes `T<seq> <millis> <writefree>` every 10 ms and counts stdin lines tagged P and L. <br>- The Pi logs the user port and sends `P<seq>` every 20 ms. <br>- The laptop polls the controller and sends `L<seq>` every 100 ms in both encodings, `[02,len,data]` and `[01,len,data]`. <br>- Runs: both hosts reading; Pi port closed (also the positive control for controller input); laptop not polling; neither host; pit vs download. 5 min per run [disc:controller-host s6]. <br>- Classify by sequence sets. | Controller viz in (b) |
| B2 | Controller throughput and latency, pit vs download channel, VEXnet vs Bluetooth; USER_READ round trip; useful read length (0x40, 0x00, 0x80, 0xE0) | The Brain writes stamped 42 B-frame SG1 lines at 5/10/20/50 Hz. The receiver logs delivered B/s, sequence gaps, one-way min-offset age and USER_READ round trip. 5 min per cell | Radio budgets and rates (D12) |
| B3 | NUL padding or binary loss through the controller (the v5x claim [source S5]) | Lines of varying length with a pad pattern; look for 0x00 inside and after replies | The NUL-discard rule; whether hex lines stay required |
| B4 | Download-channel side effects on driving, and whether it is allowed in competition | During B2, log joystick-change timestamps on the Brain around a channel switch and during back-to-back polling. Check the game manual | D8 (deferred) |
| B5 | Does the VEXos FIFO drain with no reader (cable out, controller not polling, port closed), or do PROS writers block? | The Brain writes 2 kB/s and shows write() duration and a printf heartbeat on its screen, for each case | D17: whether the stdio transport is safe by default |
| B6 | Does the Brain or controller gate output on DTR? | Open the user port and the controller with DTR off, then on; compare output | Receiver open settings |
| B7 | V5 USB user-port throughput, PROS buffer fill with NG1 plus SG1, Pi parse cost | operaGATR with spectu at 25/50 Hz. Record ProsUsbStats `expired`/`short_writes`, transmit write duration, `spectu.slow_writes`, Pi line gaps and Pi parse time per cycle. Count receiver stalls after reads whose size is a multiple of 512 B (the host hold-back [source S8 /11-12]) | Aux lane budget; adaptive rates |
| B8 | Real GET_STATE round trip on USB and on RS-485 with the Pi at 100 Hz, robot_age_ms, and the Pico -> Pi minimum latency | Log Client `round_trip` with Pi read/write stamps; tap the Pico UART against Pi receive stamps; several runs | Latency budget, D10, D11, the age bias |
| B9 | iSerialNumber and interface layout per OS for Brain and controller | Windows: CM device ids (`MI_00`/`MI_02`; controller `MI_01`?). Linux: sysfs `serial` and by-id. macOS: `ioreg -p IOUSB -l` and cu names | Identity rule |
| B10 | Does a host tool (VS Code PROS terminal, vexcom) send `pRc` on connect? | A Brain stdin echo program, connecting each tool in turn | Safety note for (b) |
| B11 | Are `/ser` custom stream writes blocking in practice? The flags come from an uninitialised kmalloc [source S1 :289-290]. A non-blocking fd writes min(len, free) bytes and still returns len [source S1 ser_driver.c:172-179, 187-194; S18 :662] | Fill the buffer with the receiver not reading. Count truncated or malformed lines at the receiver and write() calls returning 0, and log write durations. A non-blocking fd shows truncated lines and near-0 ms writes | StdioTransport robustness |
| B12 | Brain CPU cost of the publisher, and task deletion at competition mode switches | Poll and drive period jitter with spectu on and off; mode switches during heavy publishing | Priorities; the slot handoff |
| B13 | Pi -> laptop WebSocket latency over Wi-Fi and over the SSH tunnel | Ping RTT and publish-to-receive from the existing latency terms, 5 min each | Relay budget for a laptop viewer |

## 4. Answers to the five questions, and the name

### 4.1 Standalone wireless access (Q1)

**Discovery and identity.** The receiver never classifies ports by name or by COM order.

| Device | Match | OS details | Identity |
|---|---|---|---|
| Brain user port | VID 2888, PID 0501, interface 2 | Windows: `MI_02` in the device id [source S15]. macOS: IOKit `bInterfaceNumber`, `/dev/cu.*`. Linux: sysfs, as the Pi does [code nav:pipeline/resources/links/pros_usb_link.cpp:180-207] | Product string suffix (`V5 Brain - A9EC2700`); no iSerialNumber in the one dump found [source S16], else location [unknown B9] |
| Controller | VID 2888, PID 0503, a single CDC port | Interface 1 per pros-cli and vexide; Windows `MI_01` [inference; unknown B9] | SYS_STATUS `system_id`, read once at connect [inference from pros-cli's wireless upload, disc:controller-host s2.1; B9] |

Two Brains and no `--device`: refused, with the candidates listed.

**Controller protocol.** The receiver implements it itself: pros-cli is MPL-2.0 [source S22], and vexide
is MIT but its decoder stops at 0x00 [source S20; code via disc:controller-host s3, s8].
1. Open raw 8N1 with exclusive access, asserting DTR and RTS as pyserial does [source S9, S19].
2. Send `C9 36 B8 47 A4` (SYSTEM_VERSION). The reply `AA 55 A4 08 ...` must show product 0x11 and flags
   0x02 (radio) or 0x01 (cable). Otherwise report "no Brain linked" and retry every 250 ms [source S4
   v5_device.py:140-157].
3. Poll with `C9 36 B8 47 56 27 02 01 40 8C 69` (USER_READ, channel 1, up to 64 B). The reply is
   `AA 55 56 <size> 27 76 01 <data> <crc16 BE>`, and the CRC-16/XMODEM residue over the packet must be 0
   [source S4 :866-889, 1029-1043; measured-host disc:controller-host s2.1].
4. Keep one request outstanding, with a 100-500 ms timeout, and resync on `AA 55`.
   - Poll back to back while data flows. When a poll comes back empty, back off 10-20 ms using read
     timeouts, not sleeps: on this Windows host `sleep_for(1 ms)` has a p50 of 15.2 ms [measured-host
     disc:xplat-host s1.3].
5. Never write to stdin through the controller: it is disabled in pros-cli and broken per cargo-v5 #34
   [source S4, S23]. In (b) it would also merge with the Pi's input (F3).
6. Never switch the radio channel. A switch drops the link for seconds [source S12, S17]; the download
   channel reverts about 10 s after the USB cable is pulled [source S11]. No download-channel flag in
   phase 1 (D8 deferred).

**Framing.** Each frame is written as `SG1:` + uppercase hex + `\n`, frames at most 122 B. It survives
every channel-1 case:
- **COBS on** (standalone default): the COBS code byte and the 4-byte stream id land before the marker. A
  122 B frame gives a 249-character line inside one COBS block [measured-host design:cobs_sg1_check.out].
- **COBS off** (communiGATR on USB): the line goes out raw.
- **NUL padding** [inference; B3]: replies may be padded to a word boundary with NULs [source S5, a claim
  only], which can land inside a line split across two replies. Hex, marker and newline contain no 0x00,
  so the receiver discards every 0x00 before assembling lines.

The cost is 2n + 5 characters: a 42 B pose frame is 89 characters. The scanner uses the NG1 rules with a
different marker: the last marker in a line wins; a line with bad hex or a bad CRC is dropped and counted.

If B2 shows the radio byte-bound near 500 B/s [source S6, unverified], a base64 line variant (1.33x
instead of 2x) is the fallback to evaluate after B2, not a phase-1 feature.

### 4.2 Combined Brain + naviGATR access (Q2)

**Stream ownership (findings).** On the Brain, controller terminal output and USB user-port output share
one PROS buffer, one write_mtx and one VEXos channel-1 FIFO (F1). Past that point VEXos decides the
routing, which is [unknown B1]. Input is a single stream with a single practical reader (F3). On the Pi,
one resource owns the tty by convention (F7).

**Recommendation: one transport owner at each end, with explicit multiplexing.**

| End | Owner | Mux rule |
|---|---|---|
| Brain, USB link | communiGATR's `communigatr-usb-tx` task, the only channel-1 writer | New neutral hook `AuxLineSource` (a `std::function` alias, no spectu types) that also receives the previous aux write's duration. Each loop writes the link frame first, then at most one aux line within the budget |
| Brain, RS-485 or no link | `StdioTransport` task | Sole channel-1 writer. It never reads stdin and never calls serctl. The first transport to claim a Publisher wins; a second claim returns false and is counted |
| Pi | `pros_usb_link` | The last marker in each line decides: `NG1:` goes to the FrameReader (unchanged); `SG1:` goes to `brain_viz_lines`: a live ring (64 lines, drop-oldest, counted) and a durable queue for frames flagged durable (32 lines; overflow counted, never silent). The adapter drains both. The split runs inside the commands drain; brain_link commands and localization are untouched |

**Why not new request ops:**
- Each exchange costs 17-20 ms of link time with the 100 Hz Pi loop [measured-host disc:evidence-sim s3.1].
- An old Pi answers an unknown op with UnsupportedOp. The Brain treats that as incompatible for every op
  except TELEMETRY and drops the session [code brain/communiGATR/src/client.cpp:358-381].
- A known op with a new length is dropped silently, and the Brain times out after 60 ms per send [code
  translaGATR/frame_codec.cpp:555-561; nav:pipeline/commands/collectors/brain_link_commands.cpp:320-328].

**Why not a new frame type inside NG1.** When it trails a request in one drain, it suppresses that
request's reply [measured-host disc:pi-side s1.4 probe 1].

**Why not the controller in parallel with Pi USB:**
- The routing is unknown (B1). If the FIFO is split between readers, both the NG1 link and the viz stream
  corrupt [inference, H2 in disc:controller-host s6].
- Controller input would merge with the Pi's input on stdin.
- A tool that sends `pRc` re-enables COBS for the whole program (F4).
- If B1 shows duplication, a read-only controller path becomes a later option (D16, deferred).

**RS-485 equivalent.** stdio is free, so spectu owns it (3.4, 3.4b). A relay over the half-duplex bus is
deferred.

### 4.3 Pose rate and link budget (Q3)

**What each option costs.** The host sims use the real Client and the real naviGATR System.

| Option | Link cost | Bytes | Verdict |
|---|---|---|---|
| Any extra request/reply exchange (TELEMETRY-style or a viz op) at 10 / 20 Hz | -12 to -14% / -23.6 to -25.9% polls with the real Pi loop; poll gap p95 20 -> 40 ms; a report reaches the Pi 57-66 ms mean after it is made [measured-host disc:evidence-sim s0, s3]. The fakes show -8 to -16% / -18 to -29% [measured-host disc:wire-budget s3a] | 59-87 B per exchange | Rejected |
| Piggyback on the GET_STATE request | 0 polls up to +32 B in the RS-485 model; from +48 B an exchange crosses a Pi cycle and polls drop to 33-47/s [measured-host disc:evidence-sim s4.1] | +5 to +32 B | Wire change. Only fits an acceptance echo (D2, D3) |
| One-way SG1 lines on USB (proposed) | 0 exchanges. Host rig at 4-16 kB/s, one line per 2 ms loop: poll gaps, RTT and timeouts unchanged, polls/s within 0.1% [measured-host design:sim]. The rig has no Brain byte time or FIFO, so the Brain-side cost is [unknown B7]. On the Pi the lines share the 4 x 256 B read budget [code nav:pipeline/resources/links/pros_usb_link.cpp:258-268] | 89 characters per pose | Recommended |
| One-way frames in the RS-485 post-reply gap | 0 exchanges; a 42 B frame plus a 6 B binary envelope takes 4.2 ms and fits the 5 ms gap; +4.2% bus at 10 Hz [estimate]. disc:pi-side s1.5 D reads it as unsafe; it needs a half-duplex host test before any later phase | 48 B | Deferred (3.4) |
| Controller radio | [unknown B2]. The only official V5 radio figure found is VEXlink at 520-1040 B/s [source S13]. At 64 B per round trip and a 20-50 ms round trip the ceiling is 1.3-3.2 kB/s [estimate] | 95 B per pose line with COBS | Placeholder defaults until benched |

For scale, today's USB link carries about 8.5 k characters/s at about 49 polls/s [measured-host
disc:wire-budget s3a]. Its scarce resource is exchange slots, not bytes.

**Scheduling order** when budget is available: an empty liveness frame (only if nothing went out for one
liveness period), then a new pose within its cap, then SESSION on change, then events, then changed
documents (path, then field definition, then estimate), then samples within their cap, then carousel
repeats (SESSION, DEFs, documents).

**Initial rates and budgets** [estimate; B2 and B7 replace them]. Sizes from 5.5: pose line 89
characters; samples line 59 (1 channel) to 143 (8 channels); SESSION line 93, or 125 with the placement
ref; field (definition + estimate) 2278 characters in 10 lines; COBS adds 6 B per line.

| Transport | Budget, burst | Pose cap | Samples cap | Field on change | Field repeat |
|---|---|---|---|---|---|
| USB relay (b) | 4000 chars/s, 512 (under the Pi's 1024 B read budget) | 25 Hz | 10 Hz | about 1.3 s | 2-8 s |
| Direct USB (a) | 8000 chars/s, 1024 | 50 Hz | 20 Hz | under 1 s | 1-3.5 s |
| Radio (a), (c) | 400 B/s, 256 | 3 Hz | 1 Hz, 1 channel | about 22 s | about 1 min |

Arithmetic, one line per row:
- USB relay: pose 25 x 89 = 2225; SESSION 125 / 2 s carousel = 63; on change 4000 - 2225 - 63 = 1712/s,
  2278 / 1712 = 1.3 s; repeats also pay samples 10 x 59..143 = 590..1430, leaving 1122..282/s, 2.0-8.1 s.
  The host rig (4-channel samples, the earlier 87-character pose line) repeated the field about every 4 s
  [measured-host design:sim].
- Direct USB: pose 50 x 89 = 4450; SESSION 93 / 2 = 47; on change 3503/s, 0.65 s; repeats also pay samples
  20 x 59..143 = 1180..2860, leaving 2323..643/s, 1.0-3.5 s.
- Radio: pose 3 x 95 = 285; samples 1 x 65; SESSION 99 / 10 s carousel = 10; total 360 B/s. On change
  400 - 285 - 10 = 105 B/s for about 2340 B, 22 s; repeats get the leftover 40 B/s, about 58 s. Four
  channels (101 B/s) leave nothing for repeats, so more channels wait for B2.

At 500 B/s [source S6, unverified] pose alone fits about 5 Hz as hex lines or about 7 Hz as base64
[estimate].

**Rules:**
- **Actual new states only.** A pose goes out only when the caller supplied a new one. Nothing is repeated
  to fill a rate. POSE flags a held pose as held, never as a new measurement.
- **Liveness.** Every frame counts. The publisher sends a 14 B empty frame only when nothing else went out
  for one liveness period (1 s on every transport), so the receiver's stale rule works on the radio too.
- **Coalescing.** Live state (pose, tilt, samples per channel) sits in latest-wins slots; `replaced` and
  `dropped` counts go out as `spectu.*` channels. Events go in a bounded FIFO of 32, drop-oldest and
  counted. A changed document goes ahead of samples; repeats use leftover budget only.
- **Rate reduction under pressure.** The budget is fixed per transport. Pose waits as the latest value
  while lower kinds starve first. The transport times each write and counts writes over 5 ms in
  `spectu.slow_writes` [inference; B5, B7, B11]: a slow write can mean a full PROS buffer, or preemption,
  since usb-tx runs at DEFAULT below the DEFAULT+1 poll and drive tasks [code
  brain/communiGATR/pros/pros_usb_port.cpp:203-220; brain/actuGATR/include/actugatr/pros_drive.h:27-28].
  Adaptive rates come only if B7 shows the buffer filling at the preset.
- **Publisher calls never block.** Each call copies into a slot and returns: no waiting on locks, no
  allocation, no I/O (5.3). Encoding and writing happen only in the transport owner.
- **Localization has priority:**
  - on the Brain, the link lane always goes first, then at most one aux line per loop within the budget;
  - on the Pi, the split runs inside the commands drain on the estimation worker and shares the 1024 B
    read budget; its parse cost is [unknown B7];
  - acceptance (7.2): in the extended e2e rig, polls/s and poll-gap p95 stay within 1% of spectu off.

### 4.4 Latency (Q4)

Span: Pico sample -> receiver receipt, combined USB, SG1 relay. Values in ms, mean / range or p95.
Model rows come from chain_model.py with the SG1 terms added (1500 runs x 2 s at random phases)
[estimate design:chain_model_sg1.py, chain_sg1_*.out].

| # | Term | As built | Latency steps (later) | Class | Basis |
|---|---|---|---|---|---|
| 1 | Pico sample -> Brain first has the pose (true age) | 15.4 / p95 25.4 (model 16.8 / 25.8) | same | common + inherent | [measured-host disc:latency-chain s0-2, latprobe, USB 2 ms line model]; model [estimate design:chain_sg1_as-built.out] |
| 1a | of which the reply leg | about 3.9 / p95 5.2 (RS-485 7.7 / 8.8) | same | inherent | [estimate disc:latency-chain s1] |
| 2 | Poll-task publish -> aux line picked up (transmit idle sleep 2 ms) | 0-2 | not changed: a wake gains about 0.1 ms in the model | transport | [code brain/communiGATR/pros/pros_usb_port.cpp:22]; [estimate design:chain_model_sg1] |
| 2s | Brain encode (nextLine) | [unknown; W4 host timing, then B12] | - | serialization | - |
| 3 | PROS buffer flush every 2 ms | 0-2 | same | transport | [source S3] |
| 3s | Line character time, 89-249 characters | [unknown B7] | same | serialization | - |
| 4 | VEXos and USB, one way | 0.5-2 assumed | same | transport | [unknown; B8 bounds it via RTT/2]; VEXos drains its FIFO about every 1 ms [source S10 via disc:controller-host s5] |
| 5 | Pi reads the Brain port once per 100 Hz cycle | 0-10; about 3.6 mean in the model, because the line leaves the Brain at a fixed phase after each reply | about 0 with a reader on arrival (D10, separate) | avoidable | [code nav:pipeline/commands/collectors/brain_link_commands.cpp:280-346]; [estimate] |
| 5s | Pi split and parse | [unknown B7] | same | serialization | - |
| 6 | Adapter relay (5 ms feed poll) | 0-5, mean 2.5 | about 0 with the event-driven relay | avoidable | [code nav:debug/inspection/inspection_service.cpp:65] |
| 7 | WebSocket to the receiver | loopback p50 0.18-0.39, p95 0.28-0.51; Wi-Fi and SSH tunnel [unknown B13] | same | transport | [measured-host disc:latency-chain s1 row 17] |
| 7s | Receiver decode and JSON | decode [unknown until W5, timed on the host]; server publish -> receive p50 0.39 | same | serialization | [measured-host viz final_report s2] |
| = | Brain receipt -> receiver receipt | 9.9 / p95 12.7 | 7.4 / 8.9 with the event relay; 3.8 / 5.2 with the arrival reader too | | [estimate design:chain_sg1_*.out] |
| = | Pico sample -> receiver receipt | 26.6 / p95 36.1 | 24.2 / 33.2; 20.6 / 29.8 | | [estimate design:chain_sg1_*.out] |
| = | Model's Pi-authoritative path, same span | 18.3 / p95 28.9 | | | [estimate design:chain_sg1_*.out] |
| 8 | Browser receipt -> next frame | p50 2.1 / p95 5.1 at idle | same | browser | [measured-host disc:latency-chain s1 row 18] |
| 9 | Presentation smoothing | at most one source period (cap 100 ms today) | cap 50 ms | presentation | [code pi/naviGATR/spectaGATR/smoothing.js:1-18] |

**Conflict with the ~25 ms baseline.** The ~25 ms number is p50 24.9 / p95 35.5, Pi-authoritative,
synthetic, loopback, on Windows, before the browser frame [measured-host viz final_report s2]. About 11 ms
of it is localization-cycle age and about 13 ms inspection timing, with Windows timer quantization
suspected [measured-host + inference disc:latency-chain s6].
- **Like-for-like.** The model's Pi-authoritative path is 18.3 / 28.9; latprobe measured 17.1 / 26.8 for
  the same span [measured-host disc:latency-chain s2], so the model agrees within 2 ms.
- **As built,** Brain authority through the relay costs about +8 ms mean and +7 ms p95 against that
  [estimate]. It is roughly level with the reported 25 ms at p50 (26.5) and at p95 (36.1). On a Pi, where
  the baseline may be 16-18 ms [inference disc:latency-chain s6], the relative cost is larger. This is a
  real conflict, stated rather than hidden; it stands until B7 and B8.
- **Latency steps, not in phase 1.** The event-driven relay (debug side) would bring it to about +6 / +4.
  The aux wake gains nothing in the model and is dropped. A Pi reader on arrival would bring it to about
  +2 / +1 and also cut the model GET_STATE round trip from 11.9 to 7.3 ms mean, but it changes the
  localization link resource (D10). All are estimates; latprobe is rerun with the SG1 lane before any step
  is accepted (7.2).
- **The rejected relay shapes are worse.** TELEMETRY or piggyback relays measure 32-38 ms mean and 41-60
  ms p95 at the Pi drain [measured-host disc:latency-chain s2]. The SG1 path at the Pi drain is about 24
  mean / 33 p95 [estimate design:chain_sg1_as-built.out].

**Other paths:**
- RS-485 + radio: about 51 ms mean / 73 ms p95 with the hop assumed 10-50 ms [estimate
  disc:latency-chain s2]. No budget until B2.
- Direct USB, standalone: Brain publish -> receiver receipt about 2-4 ms mean (flush plus USB plus an
  event-driven read) [estimate], excluding the 512 B host hold-back (B7). The measurement age depends on
  the user's localization.
- Publish on GET_STATE receipt in the poll task, not from an app loop. An app loop adds up to one loop
  period (0-20 ms for the 20 ms loops); the probe's 30 Hz grid measured 26.1 against 15.4 ms mean
  [measured-host disc:latency-chain s1 row 14].

**Recommended budget** [estimate, replaced by B2, B7, B8 and B13]:

| Path | Span | Mean | p95 |
|---|---|---|---|
| USB relay | Brain receipt -> receiver receipt | <= 12 ms | <= 16 ms |
| USB relay | Pico sample -> receiver receipt | <= 30 ms | <= 40 ms |
| Direct USB | Brain publish -> receiver receipt | <= 5 ms | <= 10 ms |
| Radio | any | no budget until B2; age shown as a lower bound | |
| Browser | receipt -> frame at idle | | <= 8 ms (5.1 today) |

**No hiding:**
- The displayed age comes from the source clock through its mapping. A one-way mapping is shown as a lower
  bound ("age >= X ms"). Receipt time is never used as measurement time.
- Pi-reported ages are biased low, by about 7.7 ms mean in the host probe: the min-offset Pico mapping
  absorbs the constant delay [measured-host disc:latency-chain s0, s1 row 4a]; the hardware value is
  [unknown B8]. The Brain therefore records a measurement interval, not a point.
- Smoothing is presentation only. Its window is min(source period, 50 ms); it is off when stale or on a
  discontinuity, and its lag is shown as its own term. Readouts, graphs and exports stay exact.

### 4.5 Field geometry (Q5)

| Content | Binary | As SG1 lines | JSON | Basis |
|---|---|---|---|---|
| Planning map (Override field, 17 objects) | 500 B map doc; the spectu definition uses the same layout (24 + 28n) | 1330 characters in 6 frames (650 B framed) | 3089 B (414 gzipped) | [measured-host disc:wire-budget s2, s3b]; framing [estimate from 5.5] |
| Landmark estimate | 364 B (24 + 20n) | 948 characters in 4 frames (464 B framed) | 1746 B | same |
| Display (`hello.fields[0]`: walls, features, tag plates) | on no Brain wire | would be about 44 k characters | 22232 B (2069 gzipped); tag mounts 1190 B each | same |

**The accepted-revision problem.**
- The Pi announces a new map_id at once.
- A new map reaches the Brain after the map and estimate transfers: 0.25-1.1 s on host rigs [measured-host
  disc:wire-budget s2]. After a new map completes, the client reads the estimate at once [code
  brain/communiGATR/src/client.cpp:686], and the map is published only with it [code client.cpp:673-687,
  716-731] [inference].
- A new estimate arrives after up to 0.5 s plus its transfer [code client.cpp:1260-1276; client.h:59].
  Either can take over 1 s longer after failures (1 s backoff after 3) [code client.h:60].
- The Pi's live field_objects can be newer still: estimate snapshots are rate-limited to 200 ms [code
  nav:pipeline/publishing/publishers/field_documents.cpp:303-341], and the Pi viewer shows every FieldMap
  resource [code nav:debug/inspection/inspection_document.cpp:807-816, 1188-1253].

The hook is `Client::publish` (FieldPublication generation) [code client.cpp:1296-1315]: the Brain
republishes exactly that. Each path document also names the field it was planned on; Motion records this
privately today [code brain/actuGATR/src/motion.cpp:319], so the widened PathSink passes it along.

| Layer | Full transfer | Verified reference |
|---|---|---|
| Planning (what the Brain uses) | About 2.3 k characters per revision. One-way, no back-channel. A late joiner waits for the next repeat (4.3) | Saves those characters per revision, which hardly matters on USB. On the radio it still needs a carousel, because nothing can ask for missing chunks (the receiver never writes through the controller, 4.1 step 5) |
| Display (22 KB of decoration) | 11 s at 4 kB/s; minutes on the radio | Zero Brain bytes. Keyed by `field_id`; the Pi adapter computes field_id for each FieldMap with the same spectuGATR/protocol encoder, or a local asset file carries it |

**Recommendation:**
- **Planning layer: full transfer.** Send the definition and estimate as documents at start, on each
  accepted revision, and on the carousel. `field_id` = crc32 of the canonical definition bytes, so any
  cached copy validates itself.
- **Display layer: verified reference only.** Draw display assets only when their field_id equals the
  Brain's, labeled "display from Pi field.xml, planning map verified". Otherwise draw boundary plus boxes
  and say why. The id covers the planning map only, not the decoration, so the Brain's planning boxes are
  always drawn on top and a mismatch shows.
  - Whether mm and rad rounding keeps "Pi map doc -> Brain Field -> definition" byte-identical to "Pi
    FieldMap -> definition" is [unknown]; a round-trip host test (7.2) settles it. If it fails, display
    assets fall back to boundary plus boxes.
- The Pi's newer revision never replaces the Brain's. The Pi panel may show "Pi rev N, Brain rev M".
- Standalone behavior and the public API do not change: the same `updateField` call is used.

**How landmarks are shown.** The goal is lining up relative to field objects, not absolute field truth:
localization error appears in both the landmark error and the robot pose and cancels for targets defined
relative to a landmark; what remains is perception (AprilTag) error plus drift since the last sighting,
kept small because landmark-anchored targets are re-resolved every cycle [source restructure/SPEC.md
"world_estimation"].
- The spectu wire keeps full poses, matching the unchanged Brain wire.
- One viewer helper derives `dx, dy, dtheta` by LandmarkError's definition: field axes, dtheta =
  wrap(estimate heading - nominal heading) of the same revision; (0, 0, 0) means as built. The Brain
  landmark table and the Pi panel both use it.
- A path names its reference object (`PathUpdate::reference`, 0 = field frame), taken from Motion's
  private reference [code brain/actuGATR/include/actugatr/motion.h:194], so the viewer can say "target
  relative to landmark 7".
- Brain vs Pi absolute differences are diagnostics, shown in Pi panels.

### 4.6 Name

- spectuGATR is the canonical product name.
- The spectaGATR viewer moves into `spectuGATR/viewer/` and loses its separate name.
- The test tools switch to `window.__spectugatr` in W1; the `window.__navigatr` alias is dropped then.
- docs/naming.md and the README indexes change in W1.

## 5. Recommended Brain API, contract and receiver rules

### 5.1 Public API (C++17, PROS-compatible; sketch)

```cpp
// brain/spectuGATR/include/spectugatr/publisher.h
namespace spectugatr {

using Meters        = double;
using Radians       = double;
using Seconds       = double;    // publisher clock, since program start
using ChannelId     = uint16_t;
using EventId       = uint16_t;  // same id space as channels
using CommandId     = uint32_t;
using ObjectId      = uint16_t;
using FieldRevision = uint16_t;
using ClockFunction = Seconds (*)();

constexpr ChannelId kNoChannel   = 0xFFFF;
constexpr Seconds   kNotSupplied = -1.0;
constexpr ObjectId  kFieldFrame  = 0;

struct Pose2 {
    Meters  x       = 0;
    Meters  y       = 0;
    Radians heading = 0;
};

struct Attitude {
    Radians roll  = 0;
    Radians pitch = 0;
    Radians yaw   = 0;    // heading
};

enum class PoseStatus : uint8_t { kValid, kUnplaced, kNoPose, kNoSource, kCalibrating };

struct PoseSample {
    Pose2      pose;
    PoseStatus status          = PoseStatus::kValid;
    Seconds    measured_at     = kNotSupplied;   // unknown: publish age shown
    Seconds    measured_spread = 0;              // [at - spread, at]
};

struct PlacementRef {                            // opaque, from the pose source
    uint32_t instance = 0;
    uint32_t session  = 0;
    uint32_t epoch    = 0;
    uint32_t revision = 0;
};

struct Bounds {
    Meters min_x = 0, min_y = 0, max_x = 0, max_y = 0;
};

struct Box {
    Meters length = 0;
    Meters width  = 0;    // object frame, centered
};

enum class ObjectKind : uint8_t { kLandmark, kFixed };

struct FieldObject {
    ObjectId   id       = 0;
    ObjectKind kind     = ObjectKind::kFixed;
    bool       obstacle = true;
    Pose2      nominal;
    Box        box;
};
using FieldObjects = std::vector<FieldObject>;

struct FieldDefinition {
    FieldRevision revision = 0;
    Bounds        bounds;
    FieldObjects  objects;
};

enum class EstimateSource : uint8_t { kNone, kNominal, kObserved };

struct LandmarkError {    // from the object's nominal pose; (0, 0, 0) = as built
    Meters  dx     = 0;   // field axes
    Meters  dy     = 0;
    Radians dtheta = 0;
};

struct ObjectEstimate {
    ObjectId       id     = 0;
    EstimateSource source = EstimateSource::kNone;
    bool           valid  = false;
    LandmarkError  error;
    Seconds        age    = kNotSupplied;
};
using ObjectEstimates = std::vector<ObjectEstimate>;

struct FieldEstimate {
    ObjectEstimates objects;
};

enum class SegmentKind : uint8_t { kTurn, kTranslate };

struct PathSegment {
    SegmentKind kind           = SegmentKind::kTranslate;
    Pose2       start;
    Pose2       end;
    bool        reverse        = false;
    int8_t      turn_direction = 0;    // +1 CCW, -1 CW
    bool        exact          = false;
};
using PathSegments = std::vector<PathSegment>;

enum class PathState : uint8_t { kActive, kReplaced, kCleared, kCompleted, kCancelled, kFailed };

struct PathUpdate {
    CommandId        command    = 0;
    PathState        state      = PathState::kActive;
    PathSegments     segments;                    // kActive only
    bool             has_target = false;
    Pose2            target;
    ObjectId         reference  = kFieldFrame;    // target relative to this object
    std::string_view note;                        // <= 24 chars
};

enum class ChannelType : uint8_t { kReal, kInteger, kBool };

struct ChannelSpec {
    std::string_view name;    // "drive.cross_track"; group = up to the first dot
    std::string_view unit;    // "m"
    ChannelType      type = ChannelType::kReal;
};

enum class Severity : uint8_t { kInfo, kWarning, kError };

struct PublisherConfig {
    std::string_view robot_name;          // <= 16 chars
    ClockFunction    clock = nullptr;     // pros::micros on PROS
};

class Publisher {
public:
    explicit Publisher(const PublisherConfig& config);

    // one producer task per slot
    void updatePose(Meters x, Meters y, Radians heading);            // tilt not supplied
    void updatePose(Meters x, Meters y, const Attitude& attitude);   // heading = yaw
    void updatePose(const PoseSample& sample);
    void poseUnavailable(PoseStatus why);
    void updateTilt(Radians roll, Radians pitch, Seconds measured_at);
    void updatePlacementRef(const PlacementRef& ref);                // change = new placement
    void newPlacement();                                            // display discontinuity

    void updateField(const FieldDefinition& field);
    void updateFieldEstimate(const FieldEstimate& estimate);
    void updatePath(const PathUpdate& path);

    ChannelId defineChannel(const ChannelSpec& spec);                // at init; kNoChannel when full
    void      sample(ChannelId channel, double value);
    void      sample(ChannelId channel, double value, Seconds measured_at);
    EventId   defineEvent(std::string_view name, Severity severity); // at init
    void      event(EventId event, int32_t argument = 0);            // any task
};

} // namespace spectugatr
```

```cpp
// brain/spectuGATR/include/spectugatr/transport.h
namespace spectugatr {

constexpr std::size_t kMaxLineChars = 250;   // "SG1:" + 2 x 122 + "\n"
using LineBuffer = std::array<char, kMaxLineChars>;

struct TransportBudget {
    double  bytes_per_s = 0;
    int     burst_bytes = 0;
    double  pose_hz     = 0;
    double  samples_hz  = 0;
    Seconds carousel    = 0;
    Seconds liveness    = 1;
};

inline constexpr TransportBudget kUsbRelayBudget {4000, 512, 25, 10, 2, 1};
inline constexpr TransportBudget kDirectUsbBudget{8000, 1024, 50, 20, 2, 1};
inline constexpr TransportBudget kRadioBudget    {400, 256, 3, 1, 10, 1};

struct ByteBucket {
    double  tokens = 0;
    Seconds last   = 0;
};

struct KindStats {
    uint32_t updates = 0, replaced = 0, dropped = 0, bytes = 0;
};

struct PublisherStats {
    KindStats pose, tilt, samples, path, field, estimate;
    uint32_t  events_queued = 0, events_dropped = 0, events_contended = 0;
    uint32_t  lines = 0, slow_writes = 0, claims_refused = 0;
};

bool           claimTransport(Publisher& publisher);                   // first owner wins
std::size_t    nextLine(Publisher& publisher, LineBuffer& line, Seconds now,
                        std::size_t max_chars, const TransportBudget& budget, ByteBucket& bucket);  // 0: nothing due
PublisherStats stats(const Publisher& publisher);

class StdioTransport {                           // owns its task and bucket; writes /ser/sgtr
public:
    bool start(Publisher& publisher, const TransportBudget& budget);  // false: already claimed
};

} // namespace spectugatr
```

- `brain/spectuGATR/integrations/communigatr/usb_lane.h`: `communigatr::AuxLineSource usbLane(Publisher&,
  const TransportBudget&)`; it plugs into ProsUsbPort and claims the Publisher.
- The RS-485 relay is deferred (3.4).

Rules the API keeps:
- One static Publisher per Brain program, created in `initialize()` before `createLink` and `createDrive`;
  components get references [inference disc:brain-owners s6.5].
- No publisher or integration call issues SET_POSE, CONTROL or motor output. The core has no communiGATR
  or actuGATR dependency. Integrations are observers and hold no ProsLink or ProsDrive pointer.
  `newPlacement()` and `updatePlacementRef()` only mark a display discontinuity.
- One producer task per slot: pose and field from the poll task, path from the drive task, tilt from the
  app task, each channel from the task that samples it. Events may come from any task.
- The non-blocking rule is stated once, in brain/spectuGATR/PIPELINE.md.

### 5.2 Units and conventions (existing ones)

| Item | Convention | Basis |
|---|---|---|
| Units | m, rad, s; the wire uses mm, centidegrees and ms | [code brain/investiGATR/include/investigatr/geometry.h:11-15; translaGATR/frames.h:353-369] |
| Field frame | Origin inside the bottom-left corner seen from the audience; +x right, +y up; fixed, no alliance flip | [code docs/investigatr.md:28-30; pi/naviGATR/docs/coordinates.md:11, 18-20] |
| Heading | CCW from +x, wrapped to (-pi, pi] | [code geometry.h:2-3, 33-34] |
| Robot frame | +x forward, +y left, +z up | [code geometry.h:2-3] |
| Attitude | Roll about +x, positive = left side up. Pitch about +y, positive = nose down. yaw = heading. Heading-only input means tilt was not supplied | [code brain/communiGATR/include/communigatr/attitude.h:5-13] |
| Landmark error | dx, dy in field axes; dtheta = wrap(estimate - nominal heading); (0, 0, 0) = as built | [source restructure/SPEC.md "OUTPUT CHANGE"] |
| Clock | Seconds since PROS init (`pros::micros`); the wire uses u32 ms of the same clock | [code brain/communiGATR/pros/pros_link.cpp:135-137] |

The VEX roll/pitch signs and `kVexImuMountYawDeg` are still UNVERIFIED on hardware [code
brain/robot/gatr2_robot.h:49-53]. The attitude channel inherits that label.

### 5.3 Publisher internals

- **Slots.** Each live slot (pose, tilt, path, SESSION state, field, estimate, one per channel) is a
  single-writer seqlock with no lock and no claim.
  - The writer starts from `seq | 1`, copies, then stores `(seq | 1) + 1`. A writer deleted mid-copy leaves
    the sequence odd; readers skip the slot, and the next write heals it [inference].
  - The event FIFO is the only multi-producer structure: a CAS claim per entry, claims older than 50 ms
    broken by the transport owner (a competition task deleted mid-copy, B12), and a `contended` count.
  - No mutex is ever waited on. PROS deletes competition tasks without releasing mutexes [code
    docs/communigatr.md:203-207].
- **Memory.** Fixed at construction: 128 field objects, 32 path segments, 64 channels and events, 32 queued
  events. `std::atomic<uint32_t>` is already used on PROS [code
  brain/communiGATR/pros/pros_usb_port.cpp:32-59]; there is no `std::shared_ptr` [code
  docs/communigatr.md:208-209].
- **Frames.** `nextLine` picks the highest-priority due record (4.3), packs records up to 122 B, encodes the
  frame, hex-encodes it and returns. Documents go out as a unit: when a newer version appears mid-send, the
  old one is abandoned and counted as replaced. Pure C++ and host-testable.
- **Stats.** `PublisherStats` for host tests; on the wire the same counters go out as `spectu.*` channels
  (for example `spectu.pose.replaced`, `spectu.events.dropped`, `spectu.slow_writes`) on the carousel.

**Performance instrumentation** (R:295-301):

| Metric | Where measured | Exported as |
|---|---|---|
| Source and sample age | receiver, through the source's clock mapping | view/1 `source` and per-record ages; capture |
| Generation and serialization time | publisher: nextLine duration; transport: write duration | `spectu.next_line_us`, `spectu.slow_writes` |
| Message size and rate per kind | publisher (bytes, lines) and receiver (bytes/s per source) | `spectu.<kind>.bytes`; view/1 `source` stats |
| Queue depth, age, drops, replacements | publisher slots and event FIFO; Pi brain_viz_lines; HttpServer per client (exists) | `spectu.<kind>.replaced/dropped`; adapter stats; diag.inspection (exists) |
| Round trip | controller USER_READ; WebSocket ping (exists) | view/1 `source` stats |
| Browser processing and frame time | `metrics()` and cdp_bench (exist) | latency panel |
| Radio link quality | controller RADIO_STATUS at 0.2-1 Hz [code via disc:controller-host s2.1] | view/1 `source` stats |

These show per source in the latency and diagnostics panels.

### 5.4 Optional integrations at the real owners

| Data | Real owner and task | Hook (neutral, in our library) | Publisher call |
|---|---|---|---|
| Pose, status, frame | `Client::handleState`, communigatr poll task under the link mutex, every 2 ms; GET_STATE every 20 ms [code client.cpp:616-645; pros_link.cpp:145-166] | `ClientObservers::on_state` (`using StateObserver = std::function<void(const StateSample&)>`), called after `state_` is stored, as the `bench_imu` callback [code client.h:67-68; client.cpp:1062-1067] | `updatePose(PoseSample)`. Status maps from `LinkDriver::robot` [code link_driver.cpp:54-93]: kNoLink and kNoProfile -> kNoSource, the rest 1:1. `measured_at` = received_at - robot_age; `measured_spread` = round_trip plus the Pico mapping floor (about 3-13 ms in the host probe; hardware [unknown B8]) [code link_driver.cpp:89; measured-host disc:latency-chain s1 row 4a, s4] |
| Placement | The (pi_instance, session, odometry_epoch, anchor_revision) tuple; FrameGeneration is process-local [code link_driver.cpp:215-229] | same observer | `updatePlacementRef`; SESSION carries the four values |
| Tilt | App task reading `ProsVexImu::attitude` [code brain/operaGATR/src/main.cpp:142-147; brain/locaGATR/src/main.cpp:75-91] | none: the app owns the IMU | `updateTilt`; POSE carries the latest tilt with its own age |
| Accepted field | `Client::publish`, poll task [code client.cpp:1296-1315] | `ClientObservers::on_field` (`FieldObserver`), per FieldPublication generation | `updateField` (only when the map changed), then `updateFieldEstimate` |
| Link events | HELLO Ok, session loss, Pi restart, placement settled, profile applied [code client.cpp:446-454, 1102-1148, 1150-1156, 1214-1221] | `ClientObservers::on_event` (`LinkEventObserver`) | `event` |
| Path lifecycle | `Motion::start`, `plan`, `finish`, actuGATR drive task every 10 ms [code brain/actuGATR/src/motion.cpp:167-238, 315-327] | The existing PathSink, widened once: `reportPath(CommandId, const Path&, const PathContext&)`, PathContext = {state, note, frame, field generation, reference} [code brain/investiGATR/include/investigatr/path.h:44-48; motion.h:161, 189]. LinkDriver ignores the context. While PATH_REPORT exists (D4), a 10-line tee sink feeds both | `updatePath`; a start that replaces a planned command sends kReplaced for the old one. D9 fixes the planned_ gap |
| Execution | `ProsDrive::run` after `owner_.step`, drive task [code brain/actuGATR/pros/pros_drive.cpp:100-110] | `ProsDriveConfig` `DriveObserver` | `sample`: motion state, distance, heading and cross-track errors, commanded vx/vy/omega, wheel targets, saturation. `event` on motion state changes and drive faults |
| USB lane | communiGATR `communigatr-usb-tx` | ProsUsbPort `AuxLineSource`: `std::function` alias returning 0 when nothing is due, given the previous aux write's duration | `nextLine` |

investiGATR owns no live data: it provides types and the planner only [code
brain/investiGATR/include/investigatr/state_source.h:1-4]. Its integration is converter functions only.

**Test apps:**
- `brain/locaGATR`: pose, tilt, field, link events; no Motion.
- `brain/operaGATR`: adds path and execution.
- `brain/spectuGATR/examples/standalone`: a boundary, two boxes and odometry pose at 20 Hz, with no other
  GATR library.

### 5.5 Wire contract (spectugatr protocol v1)

**Frame.** `version u8 | flags u8 | session u32 | frame_seq u16 | sent_ms u32 | records | crc16`, 14 B of
overhead.
- flags bit 0: durable (the frame carries an EVENT). The Pi reads it from a fixed hex offset to pick its
  queue.
- `sent_ms` is the publisher clock at encode. Every frame feeds the one-way min-offset mapping, the
  per-sample method of `nav:time/clock_sync.h` [code nav:time/clock_sync.h:30-79 via disc:pi-side s4.1].
- CRC-16/CCITT-FALSE, the same convention as translaGATR [code translaGATR/frames.h:84-97]; the code is our
  own, so there is no GATR link dependency.
- At most 122 B; each frame is one line `SG1:<hex>\n`. Sync, type and length bytes come back only if a
  binary transport is added (the deferred RS-485 relay).
- Each record is `kind u8 | len u8 | body`. Unknown kinds are skipped by length; new fields are appended
  inside a record; a newer major version is refused with a visible message.

Record sizes below are computed from the listed fields (record = body + 2) [estimate]:

| Kind | Body | Record B | When | Delivery |
|---|---|---|---|---|
| 1 SESSION | flags u8 (placement ref present), placement u16, pose_period_ms u16, field_id u32, field_revision u16, estimate_id u32, path_id u32, [placement ref 4 x u32], name_len u8, name <= 16 | 22 + name (<= 38); +16 with the ref (<= 54) | At start, on change, on the carousel | State |
| 2 POSE | seq u16, measured_ms u32, spread_ms u8, flags u8 (valid, status 3 bits, tilt supplied, held), placement u16, x_mm i32, y_mm i32, heading_cdeg i16, roll_cdeg i16, pitch_cdeg i16, tilt_age_ms u16 | 28 | Each new sample, within the pose cap | Latest-wins |
| 3 DEF | id u16, kind u8 (real, integer, bool, event), unit_len u8 or severity u8, unit, name_len u8, name <= 32 | 7 + unit + name (about 25) | At start and on the carousel | State |
| 4 SAMPLES | measured_ms u32, count u8, count x (id u16, value f32) | 7 + 6n (n <= 16 per frame) | Per publish tick, within the samples cap | Latest per channel |
| 5 EVENT | seq u16, stamp_ms u32, id u16, arg i32 | 14 | When it happens | Durable FIFO; frame flagged durable |
| 6 DOC_CHUNK | doc_kind u8 (definition, estimate, path, names), doc_id u32 (crc32 of the whole document), total u16, offset u16, data <= 97 | 11 + data | Changed documents ahead of samples; repeats on the carousel | Resent |

Documents:
- definition: the link map layout, 24 + 28n [code translaGATR/link_documents.h:157-174]; field_id is its
  doc_id;
- estimate: 24 + 20n [code link_documents.h:233-247];
- path: command u32, rev u16, state u8, placement u16, field_id u32, reference u16, has_target u8, target
  (x i32, y i32, heading i16), start pose (10 B), note_len u8, note, count u8, then per segment flags u8
  (turn/translate, reverse, exact, CCW/CW), reserved u8, end x i32, y i32, heading i16: 38 + note + 12n.
  A clear, cancel, completion or replacement is a new path document with that state and no segments;
- names (optional): id u16 plus a name per object.

Assembly: DOC_CHUNKs assemble by (doc_kind, doc_id, total); a document is used only when complete and its
crc32 equals doc_id. An incomplete document is dropped when SESSION names a newer id of that kind.

Frame and line sizes [estimate]:
- pose only: 14 + 28 = 42 B, an 89-character line;
- empty liveness frame: 14 B, 33 characters;
- SESSION with an 8-character name: 44 B / 93 characters; with the placement ref 60 B / 125;
- SAMPLES: 21 + 6n B, so 59 characters for 1 channel, 95 for 4, 143 for 8;
- EVENT: 28 B, 61 characters;
- field: definition 6 frames (650 B, 1330 characters) plus estimate 4 frames (464 B, 948 characters);
- path, tank avoiding: p50 6 segments with an 8-character note is a 118 B document in 2 frames (346
  characters); p90 9 segments 154 B in 2 frames (418); max 13 segments 202 B in 3 frames (569) [segment
  counts measured-host disc:wire-budget s3b].

**Identities:**

| Identity | Carried as | Scope |
|---|---|---|
| Robot | name; the placement ref ties a Brain to its Pi link | SESSION |
| Source and session | the frame header session (random at publisher start, in the ProsLink nonce style [code pros_link.cpp:391-404]); an SG1 source is always a Brain publisher | Every frame |
| Field and revision | field_id (crc32 of the definition), revision | SESSION, DOC_CHUNK, path document |
| Frame and placement | placement u16, bumped by `newPlacement()` or a changed placement ref; compared only within one session | SESSION, POSE, path document |
| Sequence | frame_seq (loss), pose seq, event seq, path rev | Per session |
| Clock domain | the publisher clock; `sent_ms` in every frame | Every frame |

**Capabilities are declared by presence** (R:120, R:132): a capability exists once its first record or DEF
arrives (pose, attitude via the POSE tilt flag, field, estimate, path, each channel and event). Until then
its panel stays hidden. There is no capability bitmask.

### 5.6 Receiver source combination rules

Each rule has one home.

Receiver (`host/combine`):
1. A source is keyed by (transport source, session). A new session is a restart: that source's live state
   clears, its trail and path break, and its capture segment splits.
4. Data is never merged by arrival time. Within a source, order comes from seq and stamps. Across sources,
   only through a defined mapping; otherwise the view says "unmapped, not co-plotted".
5. Clock mapping per source; timestamps from different clocks are never compared without one:
   - a serial Brain: one-way minimum offset of (receipt - sent_ms) over a window; ages are lower bounds;
   - the Pi: WebSocket ping, +-RTT/2 (existing method [code pi/naviGATR/spectaGATR/latency.js:16-84]);
   - a relayed Brain: Brain -> Pi by one-way minimum offset at the Pi arrival (relay) stamp, then Pi ->
     receiver by ping; the bounds add.
8. Phase 1 accepts one Brain source per robot name; a second is refused with a reason (dedupe comes with
   D16 if B1 shows duplication).
10. Sequence gaps and the publisher's `spectu.*` loss counters are shown per source and written to the
    capture.
12. Definitions and documents are kept by id. A repeat with a known id and crc updates last-seen only and
    is not forwarded.
- Liveness: a source is stale after 2.5 liveness periods without a frame (2.5 s), and lost when its port
  is gone or after 5 periods.

Viewer:
2. The core scene comes only from a Brain source: `brain_usb`, `controller`, or a Brain relayed by
   `navigatr`. With naviGATR only, no robot is drawn, and the view says "core scene unavailable: no Brain
   source".
3. A stale or lost Brain keeps its last pose drawn, dimmed, with its age. The Pi pose is never substituted;
   recovery snaps back. A pose is fresh while its age plus the time since receipt stays under
   max(250 ms, 2.5 x the declared pose period); SESSION declares the larger of the pose cap period and the
   recent interval between new poses.
6. Paths and targets are drawn only when their placement matches the pose's. Otherwise they are dimmed and
   labeled "from an earlier placement".
7. The field is the Brain's field_id and revision. A Pi field appears only in Pi panels; display assets
   only when verified (4.5).
9. Panels mount on presence (5.5). An absent capability is hidden or shows one "not present" line, never
   an error.
11. A Pi source and a Brain source are combined (Pi overlay, camera mounts on the Brain robot) only when
    the Brain's placement ref matches the Pi's pi_instance and link session; otherwise the view says
    "different robot link" and keeps them apart.
13. Sources may arrive in either order. Pi first: Pi panels live, the core shows "waiting for Brain
    source". Brain first: core live, Pi panels hidden until the Pi declares them.

Per mode (R:70-73) [inference disc:viewer s4.3]:

| Element | Brain only | Brain + naviGATR | naviGATR only |
|---|---|---|---|
| Robot pose, attitude, trail | Brain `pose` | Brain `pose` | none; "core scene unavailable"; Pi estimate in graphs and a Pi readout |
| Field | Brain planning layer; display layer only when verified | same; a newer Pi revision is not substituted | Pi field, labeled (D5) |
| Path, targets, execution overlays | Brain | Brain | unavailable |
| Landmark state | Brain estimate as errors from nominal | Brain; Pi estimate only as a labeled Pi overlay (D6) | Pi panel and table |
| Graphs | Brain channels | Brain and Pi channels on the receiver timeline, each with its mapping bound | Pi channels |
| Camera, link, instrumentation, Pi diagnostics, Pi capture | hidden | naviGATR | naviGATR |
| Receiver capture and replay | yes | yes | yes |

### 5.7 Commands (final)

```
spectugatr                                   auto: a Brain USB user port, else a V5 controller
spectugatr --source brain-usb   [--device PATH]
spectugatr --source controller  [--device PATH]
spectugatr --source navigatr=HOST[:PORT]     Pi instrumentation plus relayed Brain frames
spectugatr --source controller --source navigatr=pi.local      RS-485 robot, topology (c)
spectugatr --replay capture.zip              no device opened; the browser replays the bundle
common: --port N (8765, next free if taken) --bind ADDR (127.0.0.1) --no-browser
        --capture-dir DIR --viewer-dir DIR
```

- One command starts the receiver and opens the browser at `http://127.0.0.1:PORT/` once the bind succeeds.
- `--source` may be repeated, so phase 1 has no config file (D15).
- `--identity` waits for B9 and a download-channel flag for B4 (D8); neither is in phase 1.
- The Pi keeps `<Inspection>` in its XML and serves the same viewer.

### 5.8 Receiver capture, export and replay

- **Log.** The capture stage keeps a bounded rolling log of SourceChunks {src, rx_us, bytes or text}: the
  raw received frames and Pi messages. Settings: pre_s and post_s windows; byte, duration and bundle
  bounds; one capture at a time.
- **Triggers.** Manual, or fault triggers with a cooldown and an hourly cap: Brain source lost, Brain
  session restart, a burst of frame_seq gaps, a spike in a `spectu.*` loss counter, Pi source lost.
- **Received data only** (R:320). The log holds only what arrived, never resampled. metadata.json gives,
  per source, frames received, sequence gaps and the publisher's replaced and dropped counts, so a radio
  capture never claims native-rate completeness.
- **Export** decodes the log into `spectugatr.capture/1`:
  - one CSV per (source, record kind), rows on rx_us plus src, session, sent_ms, seq, placement, field_id
    and revision;
  - metadata.json: configuration, contract versions, DEFs, units and frame conventions (5.2), sources[]
    with the clock mapping per segment, and loss;
  - `fields/<field_id>@<rev>` documents plus field_revisions.csv [inference disc:viewer s7];
  - Pi messages as they arrived (JSON lines per stream); Pi CSVs come from the Pi's own gatr2.capture/1.
- **Reuse.** Only zip_writer and the CSV escaping; `nav:debug/capture` is untouched and keeps the Pi
  triggers (R:322).
- **Replay** runs in the browser: `--replay FILE` starts the server with no sources and hands the bundle to
  replay.js, the same path as dropping a file on the page (there is no C++ replay source). M6 extends
  ReplayModel: the field revision in force at the playhead, trails and graphs split at session and
  placement changes, new samples only (fixes A11). A gatr2.capture/1 bundle replays as one navigatr
  source with no core robot.
- **Read-only.** Replay opens no device. No source ever writes to a Brain, including brain-usb, and nothing
  is sent through a controller to stdin.

### 5.9 Receiver -> browser contract (`spectugatr.view/1`)

W0 owns `spectuGATR/protocol/VIEW.md` and freezes it with the Brain wire.
- **Receiver messages** (the only ones not 1:1 with a wire record, because port loss and clock mappings
  exist only at the receiver):
  - `hello` {contract, client_id, viewer_build, sources[]};
  - `source` {src, kind, session, state (connecting, live, stale, lost, absent), clock mapping {offset_us,
    uncertainty_us, method, valid}, transport stats}.
- **Brain records:** one JSON message per wire record (session, pose, def, samples, event), with the 5.5
  fields plus `src` and `rx_us`; documents arrive assembled and crc-checked as one `doc` message.
- **Pi messages** pass through untouched, wrapped as `{"src":"navigatr","rx_us":N,"msg":<original>}`. The
  receiver parses only the Pi's pong (one number) for its clock mapping, so it needs no general JSON
  parser. Binary previews gain `src` in their JSON header.
- **Delivery:** reliable FIFO for hello, source, def, event and doc; one replaceable slot per (src, kind)
  for pose, samples and session, using the existing per-channel slots and priorities [code
  nav:debug/inspection/http_server.h:96-99, 197-204].
- **Client -> receiver:** ping (receiver level); subscribe and preview carry `src` and are forwarded to the
  Pi.
- **The Pi's own messages:** InspectionService stays the Pi publisher (inspect/2). For new Pi measurements
  it gains a small channel table that emits the same `def` and `samples` messages, so adding one needs no
  viewer code (R:127). The Pi has no SG1 encoder.
- **Well-known channels** the viewer draws specially: `motion.desired_heading`, `motion.follow_x`,
  `motion.follow_y`, `motion.destination_x`, `motion.destination_y` (today's TELEMETRY overlays [code
  pi/naviGATR/spectaGATR/field_scene.js:1395-1438]). Every other channel is a plain graph.

## 6. Measured evidence vs estimates

| Claim | Value | Kind | Where |
|---|---|---|---|
| TELEMETRY cost with the real 100 Hz Pi loop, 100 ms period | -12 to -14% polls (RS-485 >= 80 B: -20.5%); 6.7-7.0 delivered per second | measured-host (sim) | disc:evidence-sim s0, s3 |
| Same at 50 ms / 200 ms | -23.6 to -25.9% / -7 to -8% | measured-host | disc:evidence-sim s3 |
| Link time per exchange | 17-20 ms (54 B); 30 ms (>= 80 B on RS-485) | measured-host | disc:evidence-sim s3 |
| Any extra exchange at 10 / 20 Hz | real Pi loop -12 to -14% / -23.6 to -25.9%; fakes -8 to -16% / -18 to -29% | measured-host | disc:evidence-sim s3; disc:wire-budget s3a |
| "No cost at the 2 ms line" in the docs | Artifact of phase lock; any offset >= 0.1 ms costs 12% | measured-host | disc:evidence-sim s4.3 |
| Piggyback bytes on GET_STATE (RS-485 model) | Free up to +32 B; +48 B crosses a Pi cycle | measured-host | disc:evidence-sim s4.1 |
| SG1 aux lane in the host rig (no Brain byte time or FIFO) | One line per 2 ms loop at 4/8/16 kB/s: poll gaps, RTT, timeouts unchanged, polls/s within 0.1%. No per-loop limit: 4 kB/s still unchanged; 16 kB/s with 2 kB bursts poll gap max up to 86 ms, up to 8 timeouts per 100 s; 64 kB/s 23-37 polls/s. Field repeat about 4 s at 4 kB/s | measured-host (sim) | design:sim/sg1_sweep_j500.md |
| SG1 lines under PROS COBS | frames 8-122 B parse; 123 B and longer do not | measured-host | design:cobs_sg1_check.out |
| Pose true age when the Brain first has it | USB 15.4 / p95 25.4; RS-485 20.4 / 28.7 | measured-host (modeled transport) | disc:latency-chain s2 |
| Like-for-like Pi 30 Hz true age | 17.1 / p95 26.8 | measured-host | disc:latency-chain s2 |
| Reported baseline | p50 24.9 / p95 35.5 (Windows, synthetic) | measured-host | viz final_report s2 |
| SG1 relay, Pico sample -> receiver | 26.6 / p95 36.1 as built; 24.2 / 33.2 with the event relay; 20.6 / 29.8 with the arrival reader; the model's Pi path 18.3 / 28.9 | estimate (model) | design:chain_sg1_*.out |
| Radio hop | 10-50 ms assumed | estimate / unknown | disc:latency-chain s2; B2 |
| Radio throughput | 1.3-3.2 kB/s ceiling at 64 B per 20-50 ms round trip; VEXlink 520-1040 B/s; about 500 B/s forum figure (a file sent to the Brain, and VEXlink; no output-direction figure found) | estimate; source S13; source S6 (unverified) | disc:wire-budget s3c |
| Brain link frame sizes | GET_STATE 23/59 B, TELEMETRY 68 B; line = 2n + 5 | measured-host (encoder) | disc:wire-budget s1 |
| Field sizes | Map 500 B, estimate 364 B, display JSON 22232 B | measured-host | disc:wire-budget s2 |
| Path segments (tank avoiding) | p50 6 / p90 9 / max 13; path document 118 / 154 / 202 B in 2 / 2 / 3 frames | measured-host (segments); estimate (bytes) | disc:wire-budget s3b; 5.5 |
| PATH_REPORT loses turns and thins wrongly | Probes E and F | measured-host | disc:brain-owners s3.3 |
| Motion path-clear gap | Probes A, B and C leave the path uncleared | measured-host | disc:brain-owners s3.4 |
| SG1 line after an NG1 request | Request bytes delivered; the line counted as console. Reply sent: inference from brain_link_commands.cpp:365, asserted by the 7.2 split test | measured-host + inference | disc:pi-side s1.4 |
| Server extraction | http_server.cpp and zip_writer.cpp compile standalone; static probe exe with the viewer 1.40 MB; 23/23 existing server tests in today's binary. The extracted target is not built | measured-host | disc:pi-side s4.1; disc:xplat-host s1.3 |
| Windows sleep granularity | `sleep_for(1 ms)` p50 15.2 ms; condvar wake p50 0.059 ms | measured-host | disc:xplat-host s1.3 |
| Browser idle frame | Receive -> rAF p50 2.1 / p95 5.1 | measured-host | disc:latency-chain s1 row 18 |
| Aux budgets, rates, field times | See 4.3 | estimate | B2, B7 |
| Channel-1 routing, FIFO without a reader, DTR, NUL padding, descriptors | - | unknown | B1, B5, B6, B3, B9 |

## 7. Implementation breakdown and validation

### 7.1 Workstreams (after approval; the restructure patch lands first)

LOC figures are [estimate] unless marked.

| W | Scope and file ownership | Extracted vs new | Needs |
|---|---|---|---|
| W0 contract | `spectuGATR/protocol/**` (records, frame and SG1 line codec, crc), `spectuGATR/protocol/VIEW.md` (view/1), `spectuGATR/docs/PROTOCOL.md`, `spectuGATR/PIPELINE.md`, golden vectors | New, about 600 LOC + 400 tests | Restructure landed; freeze and review before the other workstreams |
| W1 viewer move (M0) | `spectuGATR/viewer/**` (from `pi/naviGATR/spectaGATR`, three.js physically in `viewer/vendor/`), `spectuGATR/viewer/PIPELINE.md`, `spectuGATR/tests/viewer/**` (from `tools/viewer`), `spectuGATR/cmake/embed_viewer.cmake` (from `cmake/embed_assets.cmake`, `GLOB_RECURSE` keeping subpaths); naviGATR CMake embed lines; test tools on `window.__spectugatr`; docs/naming.md and the README indexes | Moved; about 50 CMake lines | - |
| W2 server extraction | `spectuGATR/host/server/**`: http_server, json_writer, zip_writer, viewer routes, capture route, and their tests. naviGATR include updates, plus aliases in `namespace navigatr` so 218 references do not churn [code grep via disc:xplat-host s5.1]. Add SO_NOSIGPIPE, SIGPIPE ignore and Host/Origin checks | About 2.5k extracted [code wc -l via disc:pi-side s4.1], 1.2k tests moved, about 100 new | W1 |
| W3 platform | `spectuGATR/host/platform/**`: serial (win32, posix), usb_ports (win32, macos, linux), vex_ports, browser | New, about 1.15k + 0.4k tests [estimate disc:xplat-host s5, minus app_paths and process] | - |
| W4 Brain publisher | `brain/spectuGATR/{include,src,pros,tests,examples}`, `brain/spectuGATR/PIPELINE.md`; `brain/gatr2_brain.mk` gets a `spectugatr` entry that compiles spectuGATR/protocol, as it compiles translaGATR sources today [code brain/gatr2_brain.mk:22, 51-52]; `brain/CMakeLists.txt` | New, about 1.0k + 0.6k tests | W0 |
| W5 receiver | `spectuGATR/host/{main.cpp, PIPELINE.md, sources/, decode/, combine/, capture/}`, `spectuGATR/README.md`, `spectuGATR/CMakeLists.txt` (SPECTUGATR_BUILD_HOST, SPECTUGATR_BUILD_TESTS): brain_usb (about 150), controller CDC2 (about 350), navigatr WebSocket client (about 300; only if D19 keeps (c)), decode (about 250), combine (about 400), capture log and export (about 350) | New, about 1.8k | W0, W2, W3 |
| W6 Brain integrations | `brain/spectuGATR/integrations/**`; neutral hooks in communiGATR (`ClientObservers`, `AuxLineSource`) and actuGATR (widened PathSink with the tee, `DriveObserver`, the D9 fix); `brain/locaGATR`, `brain/operaGATR` main.cpp | About 400 new, 150 changed | W4 |
| W7 naviGATR adapter | `nav:pipeline/resources/brain_viz_lines.h` (new stage-level resource type); `nav:pipeline/resources/links/pros_usb_link.*` (SG1 split, live ring and durable queue); `nav:debug/inspection/**` (view/1 envelope, Brain decode through host/decode, `brain_frames`, Pi labels, Pi channel table, `PIPELINE.md`); the pros_usb_link doc | About 500 changed | W0, W2, W5 (host/decode) |
| W8 viewer multi-source (M1-M6) and replay | `spectuGATR/viewer/**`: feed split into Connection + SourceState, CoreScene, generic channels, presence gating, per-source status, multi-source replay; stub_server view/1 mode | About 1.5k JS changed | W1, W5, W7 |
| W9 packages and CI | `.github/workflows/release.yaml` (written, never triggered), `spectuGATR/packaging/**` (udev rule, install script) | About 200 new | W5 |

- **Order:** restructure patch applied, then W0 frozen (Brain wire and view/1), then W1-W4 in parallel,
  then W5-W7, then W8, then W9. After that, serial integration and validation, and finally the read-only
  audit.
- **Ownership:** each workstream owns only its listed files. Shared contracts change only through W0.
- **Bench kit:** W4 and W6 also prepare the B1-B13 programs, scripts and expected-output sheets,
  uncommitted.
- **Limits (R:12):** agents build and run host tests only. The user flashes, deploys, runs the bench and
  returns the logs. No commit, push, tag or release.

### 7.2 Validation plan

- **Host unit tests** (gtest, explicit test lists):
  - protocol: golden vectors, truncated lines (as B11 would produce), bad CRC, skipping unknown records,
    refusing a newer version, SG1 lines wrapped by COBS, NUL bytes discarded, an abandoned document;
  - publisher: latest-wins, event FIFO bounds, scheduling order, budgets, liveness frames, never blocking
    under a stalled transport, fixed memory, a writer deleted mid-copy healing, a second transport claim
    refused;
  - the CDC2 codec, using pros-cli CRC vectors; `vex_ports` fixtures (Windows `MI_xx` ids, macOS names,
    sysfs trees);
  - combination rules: restart, stale and lost on a 3 Hz source, placement, field revision, unmapped
    clocks, placement-ref mismatch, a second Brain refused, document dedupe;
  - the Pi split: an SG1 line after an NG1 request in one drain keeps the reply; the burst stays within
    the 1024 B read budget; durable-queue overflow is counted;
  - the path lifecycle: cases A-D from disc:brain-owners s3.4, plus kReplaced;
  - field_id round trip: "Pi map doc -> Brain Field -> definition" bytes equal "Pi FieldMap -> definition"
    bytes;
  - the landmark error helper against LandmarkError on random cases, including wrap near +-pi.
- **Fakes:** FakeBrainStdio (COBS on/off, NUL padding, loss); FakeController (CDC2, 64 B per poll, round
  trip, loss); the existing FakeUsb, FakeBus and FakePi; stub_server in view/1 multi-source mode.
- **E2e rig.** Extend disc:evidence-sim `e2e_sweep` (real Client, LinkDriver and Motion, real naviGATR
  System), starting from design:sim/sg1_sweep.cpp:
  - add a Brain-side model to UsbWire: a tx task (link lane first, capped aux lane, one line per loop), a
    2047 B FIFO flushed every 2 ms, and a per-character time swept over an assumed range;
  - report polls/s, poll-gap p95 and request queueing delay against aux rate, spectu off and on:
    usb2/3/5, 5 phases, 0.5 ms jitter, TELEMETRY off and at 100 ms;
  - acceptance: polls/s and poll-gap p95 within 1% of off, and 0 new timeouts. The host result bounds the
    Pi read-budget effect only; B7 settles the Brain-side cost;
  - rerun latprobe with the SG1 lane before any latency step is accepted.
- **Browser suite:**
  - the existing 9 scenarios on M0 unchanged; from W8 against the stub's view/1 mode, with the checks that
    read the Pi pose as the robot changed to "no core robot from a naviGATR-only source";
  - new scenarios: `brainonly`, `brainminimal` (pose only: no field, path or attitude panels), `combined`
    (moving the Pi pose must not move the robot), `navionly`, `sourceloss`, `radio3hz` (no false stale or
    lost), `fieldrev`, `placement`, `clocks`, `turnpath`, `replaymulti`, `roundtrip` (stub live -> capture
    -> zip -> replay: exact values, the field revision at 3 playheads, loss rows), `relayoverflow`,
    `carousel` (DOM node count flat and buildField not called across 60 s of repeats);
  - new hooks: `data-core`, `data-core-pose`, `data-sources` [inference disc:viewer s6].
- **Perf A/B.** The inspect bench plus cdp_bench idle, orbit and cpu4x, one source vs two, on a quiet
  host.
- **Bench on hardware** (the user runs it): B9 and B6 at a desk; B7, B8, B10 and B13 with Pi and Brain; B1
  and B3 with Brain, Pi and controller; B2, B4 and B5 on the radio; B11 and B12 on the Brain alone.
- **Existing suites stay green.** naviGATR 626 at ae457ee [measured-host
  restructure/logs/we-plan.base.ctestN.txt]. brain 455, translaGATR 254 and Pico 129 were measured at
  2d11c67 plus the working tree [measured-host viz final_report s3], believed equal to ae457ee [inference
  disc:wire-budget s6]. Recount at W0 start.

### 7.3 Prebuilt packages

A GitHub Actions matrix builds them; users need no compiler [source: the runner, setup-msys2, CMake and
glibc URLs in disc:xplat-host s3.1]:

| Package | Build | CI check |
|---|---|---|
| Windows zip | MinGW UCRT64, `-static` | objdump shows no libstdc++, libgcc or winpthread DLLs, the known 0xc0000139 hazard [measured-host disc:xplat-host s1.3] |
| macOS universal2 zip | arm64 + x86_64, deployment target 11.0 | lipo check |
| Linux x64 and arm64 tar.gz | Old-glibc container, static libstdc++, plus the udev rule `70-spectugatr.rules` and an install script | Highest GLIBC symbol <= 2.36 (Pi OS Bookworm) |

- The viewer is embedded, so there are no loose files; `--viewer-dir` is for development.
- Signing is D13; the package license is D18.
- The Brain library ships as source. `GATR2_BRAIN_LIBS := spectugatr` builds it alone: the entry compiles
  spectuGATR/protocol and adds its include directory. The standalone example's Makefile uses exactly that
  line. An outside team copies `brain/spectuGATR` and `spectuGATR/protocol`, or adds this repo as a
  submodule.
- CI does not build PROS programs [code brain/README.md:25-26], so the PROS builds of locaGATR, operaGATR
  and the example run locally.

### 7.4 Planned test matrix

| Platform | Done so far (discovery) | Build | Host tests | Device tests |
|---|---|---|---|---|
| Windows 11 x64 (this laptop) | probe exe of the server with the viewer; existing suites; no VEX device present (0 ports) [measured-host disc:xplat-host s1.3] | planned | all, plus the browser suite | none: no hardware here |
| Linux x64 (CI) | the existing CI job compiles today's naviGATR, server included [code .github/workflows/ci.yaml:28-45]; run history not visible [disc:xplat-host s1.2] | planned | all, plus serial over openpty | none |
| Linux arm64 (CI, Pi) | nothing | planned | all | Pi on the bench |
| macOS (CI) | nothing | planned (universal2) | pure platform tests plus server | none. Needs a Mac with hardware, not claimed before that |
| V5 Brain (PROS 4.2.2) | nothing new | locaGATR, operaGATR, examples | logic tested on the host | B1-B13 only |

Physical controller support is never claimed from fakes.

### 7.5 Receiver platform behavior

[source + inference disc:xplat-host s2.1-2.7]:

| Concern | Behavior |
|---|---|
| Permissions | Linux EACCES: the dialout (or uucp) hint, or the shipped uaccess rule. Windows ERROR_ACCESS_DENIED: "in use by PROS terminal or VEXcode". macOS: the errno text |
| Exclusive open | Share mode 0 on Windows; TIOCEXCL on POSIX |
| Reconnect | Each serial source rediscovers its own device at 1 Hz inside its thread after a loss, by identity and role, as a new transport epoch; restarts are decided by the protocol session. Controller radio loss is a protocol timeout, not a port loss |
| Paths | No config file in phase 1, so no per-OS config folder. Captures stay in memory; `--capture-dir` saves them; never Documents (OneDrive-redirected on this host [measured-host disc:xplat-host s1.3]) |
| Browser | ShellExecuteW, `open`, or xdg-open (only with DISPLAY or WAYLAND_DISPLAY, never as root). The URL is always printed |
| Waits | On I/O or events, never on `sleep_for` |
| Build root | `spectuGATR/CMakeLists.txt` with SPECTUGATR_BUILD_HOST and SPECTUGATR_BUILD_TESTS; signal handling lives in main.cpp |

### 7.6 Retain and finish

| Item (R:280-287) | Kept | Changed | Gap finished | Owner |
|---|---|---|---|---|
| Low-latency delivery, bounded backpressure | server slots, reliable FIFO, ping window | per-source channels | Host/Origin checks | W2 |
| Browser smoothness, perf tooling | rAF loop, render budget, cdp_bench | two-source perf A/B | - | W8 |
| Field, robot, path, trail | renderer, smoother, trail ring | inputs from CoreScene; field-frame trail per placement (D7) | turn-only segments drawn | W8 |
| Sensor and transport diagnostics | Pi panels | under `viewer/navigatr/`, per-source status | instrumentation table 5.3 | W7, W8 |
| Dynamic graphs | Plot, GraphPanel, series | generic channels from DEF | new measurements need no viewer code | W8 |
| Attitude honesty | measured, stale, unavailable, assumed_level | thresholds per 5.6 rule 3 | - | W8 |
| Camera preview | preview path unchanged; preview frames only gain `src`; no field correction or calibration added; a missing camera stays a warning | mounts drawn on the Brain robot only when rule 11 holds | - | W7, W8 |
| Capture, export, replay, lifecycle events | Pi capture | receiver log, export, browser replay (5.8) | A11 fixed | W5, W8 |

Single-source assumptions to remove [code disc:viewer s2]:

| Assumption | Removed in |
|---|---|
| One WebSocket, one clock estimate, one session, one hello and feature set | M1 (Connection + SourceState) |
| Connection stale = no `state` for 1 s | M1, M5 (per-source liveness, 5.6) |
| One robot, trail, path, smoother, telemetry overlay; identity = Pi session tuple | M2 (CoreScene) |
| One graph store labeled "Pi host clock" | M3 (receiver timeline, per-source mapping) |
| Panels always mounted; Pi panels at the root | M4 (presence gating, `viewer/navigatr/`) |
| One `#status` row; load held until 10 `state` | M5 (per-source hooks) |
| Replay timeline = `pi_host_us` | M6 (rx_us bundles; old bundles as one navigatr source) |

## 8. Decisions that need your input

| D | Question | Options | Recommendation |
|---|---|---|---|
| D1 | Which Brain pose to publish | (a) Every GET_STATE receipt, in the poll task. (b) The pose Motion used, in the drive task: +0-10 ms and repeats. (c) From an app loop: up to one loop period | (a). Motion's frame check stays visible through the placement on path documents |
| D2 | Pose sequence or measurement id on GET_STATE (a localization wire change) | (a) None: publish each receipt with "measurement id unknown". The probe saw 0 repeats in 3.4-4k states [measured-host disc:latency-chain s4]. (b) Add a u16 sequence and the low bits of measured-at to the state block (link v5, lockstep) | (a) now; (b) with the next planned link version |
| D4 | Path document replacing PATH_REPORT | (a) Keep both. (b) Stop PATH_REPORT while spectu is active and retire the op later. (c) Remove it now | (b). This also frees its link exchanges and later deletes the PathSink tee |
| D5 | What the 3D view shows in naviGATR-only mode | (a) Blank. (b) The Pi's configured field, Pi landmark estimates and camera diagnostics, labeled "Pi configuration, no Brain source", with no robot | (b) |
| D6 | A Pi pose overlay in combined mode | (a) Graphs and tables only. (b) An optional overlay, off by default, in a distinct style, labeled "Pi estimate, not what the Brain used", drawn only when rule 11 holds | (b) |
| D10 | Latency | (a) Accept about +8 ms mean / +7 ms p95 [estimate] for Brain authority as built; add the event-driven relay (debug side, model -2.4 ms mean) later as its own measured step. (b) Also approve a Pi reader on arrival in `pros_usb_link` now (model -3.6 ms more, and the GET_STATE round trip 11.9 -> 7.3 ms), a transport change on the localization link resource | (a). Offer (b) separately, after B7 and B8, as a localization-link change on its own merits |
| D11 | TELEMETRY | (a) Keep as is: its attitude is a Pi estimator input, not display-only (A2). (b) Drop the motion groups while spectu is active, or lower the rate. (c) Move tilt onto the GET_STATE request (+4 B, free up to +32 B in the RS-485 model) and retire TELEMETRY | (a) in phase 1; (c) with the D2 link version |
| D13 | Code signing | Apple Developer ID and a Windows certificate, or first-run prompts | Unsigned first |
| D17 | Stdio transport before B5 | (a) Started only by user code (`StdioTransport::start`) and documented as unsafe with no host attached: if nothing drains channel 1, a blocking write can wait forever holding write_mtx and stall every printf [code via disc:pros-serial s1.1]. (b) Also started by the integrations by default | (a) until B5 passes |
| D18 | License for the prebuilt public packages; the repo has none [measured-host disc:controller-host s8] | MIT, or another | MIT (your notes name a potential MIT library); needed before W9. Not legal advice |
| D19 | RS-485 + visualization | (c) The radio for the Brain and the network for the Pi: needs the laptop's WebSocket client and cross-source clock mapping. (c2) A USB cable from the Brain user port to the Pi carrying only SG1, viewed as in (b) | (c2) if one more cable on the RS-485 robot is acceptable to you, else (c). With (c2) the laptop navigatr source leaves phase 1 |

**Defaults unless you object:**
- D3: no acceptance ack to the Pi; the Brain publishes what it accepted.
- D7: a Brain field-frame trail that breaks per placement; the Pi odometry trail only inside the D6
  overlay.
- D9: fix the Motion path-clear gap in actuGATR by tracking "a path was reported for this command"
  separately from `planned_`, with 3 regression tests; it also fixes today's PATH_REPORT display. The gap
  (A1): a cancel during a halt, a refused replan, or a new command before its plan leaves a stale path
  [code brain/actuGATR/src/motion.cpp:172-174, 229-238, 435-441, 474-477; measured-host
  disc:brain-owners s3.4].
- D12: radio placeholders until B2: pose 3 Hz, samples 1 Hz (1 channel), 400 B/s, carousel 10 s.
- D14: standalone field display is boundary, boxes and optional names when no verified asset exists.
- D15: repeated `--source` flags and no config file in phase 1.

**Deferred, not phase 1:** D8 (a download-channel flag; needs B4 and the game rules), D16 (a read-only
controller in parallel with Pi USB; needs B1 to show duplication).

## Appendix A: defects and doc/code contradictions found (input for the later audit; not fixed)

| # | Finding | Basis |
|---|---|---|
| A1 | Motion clears the reported path only when `planned_` is true (see D9) | [code brain/actuGATR/src/motion.cpp:172-174, 229-238, 435-441, 474-477; measured-host disc:brain-owners s3.4] |
| A2 | TELEMETRY attitude is a Pi estimator input, not display-only. The docs say "localization never reads it" | [code nav:brain_profile/brain_profile_builder.cpp:537-538, 605-610; nav:pipeline/localization/observation_models/brain_imu_wheel_bench.cpp:204-224] vs [code translaGATR/frames.h:268-269; docs/communigatr.md:707-709; attitude.h:2-3; gatr2_robot.h:51-57] |
| A3 | `ClientConfig.byte_time` is fixed at 10/115200 and not derived from `LinkConfig.baud`. Changing `kLinkBaud` leaves the timeouts sized for 115200 | [code brain/communiGATR/include/communigatr/client.h:47; pros/pros_link.cpp:40-46] |
| A4 | PATH_REPORT drops turn-only segments and headings, and its thinning draws lines that are never driven. The viewer's follower-target check fails for turns | [code link_driver.cpp:158-183; client.cpp:229-241; pi/naviGATR/spectaGATR/field_scene.js:1426-1427] |
| A5 | "6 to 11% fewer polls" and "about 10 to 15 ms per exchange" vs 12-14% and 17-20 ms with the real Pi loop. The docs' "no cost at 2 ms" row is a phase-lock artifact | [code docs/communigatr.md:764-788; brain_setup.md:327-328; gatr2_robot.h:56-58] vs [measured-host disc:evidence-sim s3, s4.3] |
| A6 | final_report calls ~25 ms "measurement to localization publication". It is about 11 ms of cycle age plus about 13 ms of inspection timing | [measured-host disc:latency-chain s6] |
| A7 | Pi-reported ages are biased low by the min-offset mapping, so `LinkDriver` age is not an upper bound | [code nav:time/clock_sync.h:1-20; link_driver.cpp:89; measured-host disc:latency-chain s1] |
| A8 | The TELEMETRY send grid ignores report times: 57-66 ms mean lag at 100 ms. Is that intended? | [code client.cpp:840-846, 966-975; measured-host disc:evidence-sim s3.2] |
| A9 | `/ser/ngtr` flags come from an uninitialised kmalloc, so the stream may be non-blocking with silent truncation. SERCTL_NOBLKWRITE is a no-op | [source S1 :221, 233-238, 289-290] |
| A10 | `field_scene.js:2-4` says `hello.fields` is "the planning data the Brain is served". It is every Pi FieldMap resource | [code nav:debug/inspection/inspection_document.cpp:807-816] |
| A11 | Replay ignores `new_measurement` (duplicate samples). capture_fixture headers lag the runtime. Telemetry rows without a target lose overlays. Fields are not in the bundle | [code pi/naviGATR/spectaGATR/replay.js:554; nav:debug/capture/capture_bundle.cpp:186-191; pi/naviGATR/tools/viewer/capture_fixture.py:24-36] |
| A12 | There are no Host/Origin checks on /ws or POST capture. Fine on the Pi loopback, not on a laptop | [code: grep "Origin" has no hit in http_server.cpp, inspection_service.cpp or the capture routes, via disc:xplat-host s1.1] |
| A13 | `http_server.h:89-90` says TCP_NOTSENT_LOWAT is Linux-only, but macOS defines it too. SIGPIPE is unguarded on macOS | [code nav:debug/inspection/http_server.h:89-90, http_server.cpp:124-128; source S21] |
| A14 | `link_monitor.h:49` names the kind `brain_rs485`; the code uses `brain_serial` | [code nav:debug/diagnostics/link_monitor.h:49; nav:pipeline/commands/collectors/brain_link_commands.cpp:220] |
| A15 | `usb_localization_bench.md:96` says "no tilt", stale since the TELEMETRY attitude fold | [code pi/naviGATR/docs/usb_localization_bench.md:96-97] |
| A16 | `field_scene.js:1388-1394` has a stale target-valid comment. The graph labels say "Pi host clock", true only for one source | [code pi/naviGATR/spectaGATR/field_scene.js:1388-1394; graphs.js:10-15] |
| A17 | `ci.yaml:88` runs `pio run` in `pico/`, but platformio.ini moved to `pico/aggreGATR`. The disk folder is `pi/navigatr` while the index says `pi/naviGATR` | [code .github/workflows/ci.yaml:86-88]; [measured-host `ls pi` via disc:xplat-host s1.2] |
| A18 | `pros_usb_link` takes `NG1:` anywhere in a line (rfind), so free text containing it becomes a link line. `console_lines_` is not exported | [code nav:pipeline/resources/links/pros_usb_link.cpp:218-220] |

## Appendix B: naviGATR paths (restructured, was ...)

Per restructure/plan/FINAL.md and move_map_final.tsv (the draft move_map.tsv is superseded).

| Cited as | Was (ae457ee) |
|---|---|
| `nav:pipeline/resources/links/pros_usb_link.*` | `src/impl/resources/pros_usb_link.*` |
| `nav:pipeline/resources/brain_imu_bench.h` | `src/resources/brain_imu_bench.h` |
| `nav:pipeline/resources/brain_viz_lines.h` | new in W7 |
| `nav:pipeline/commands/collectors/brain_link_commands.*` | `src/impl/commands/brain_link_commands.*` |
| `nav:pipeline/publishing/publishers/brain_link_publisher.*`, `field_documents.*` | `src/impl/publishing/...` |
| `nav:pipeline/localization/observation_models/brain_imu_wheel_bench.*` | `src/impl/localization/brain_imu_wheel_bench.*` |
| `nav:pipeline/world_estimation/` (LandmarkError in field_state.h, step 3) | `src/impl/world_estimation/...`, `src/contracts/field_estimation.h` |
| `nav:brain_profile/brain_profile_builder.*` | `src/runtime/brain_profile_builder.*` |
| `nav:core/` (system.h, handoff.h, build.cpp, run.cpp) | `src/runtime/system.{h,cpp}`, `handoff.h` (system.cpp split in step 2) |
| `nav:debug/inspection/*`, including `inspection_config.h` | `src/inspection/*`, `src/runtime/inspection_config.h` |
| `nav:debug/capture/*` | `src/capture/*` |
| `nav:debug/diagnostics/*` | `src/diagnostics/*` |
| `nav:time/clock_sync.h`, `host_clock.h`, `time.h` | `src/core/...` |
| `nav:io/*` (half_duplex, serial_port) | `src/transport/*` |

## Sources

- S1 PROS 4.2.2 ser_driver.c: https://github.com/purduesigbots/pros/blob/4.2.2/src/system/dev/ser_driver.c
- S2 PROS 4.2.2 ser_daemon.c: https://github.com/purduesigbots/pros/blob/4.2.2/src/system/dev/ser_daemon.c
- S3 PROS 4.2.2 system_daemon.c: https://github.com/purduesigbots/pros/blob/4.2.2/src/system/system_daemon.c
- S4 pros-cli v5_device.py: https://github.com/purduesigbots/pros-cli/blob/45aae5df262eab1246f4d74c5be7bd1a4916eb73/pros/serial/devices/vex/v5_device.py
- S5 v5x PR 329, NUL-padded FIFO replies (a claim only): https://github.com/v5x-dev/v5x/pull/329
- S6 jpearman, about 500 B/s: a file sent to the Brain through the controller, and VEXlink "about the theoretical max for over the air comms"; no output-direction figure (quoted in disc:pros-serial s3.1; not re-verified: Cloudflare 403, no archive): https://www.vexforum.com/t/vex-radio-prevents-wireless-sending-to-brain-during-runtime/144920
- S7 jpearman, controller has one serial port: https://www.vexforum.com/t/how-to-use-webserver/123135/12
- S8 jpearman, thread 127844: /9 channel-1 FIFO routed by VEXos; /11-12 exact-512 B transfers held by the host: https://www.vexforum.com/t/vexos-bug-report-serial-output-with-length-512-is-ignored/127844/9
- S9 pyserial serialutil.py, DTR and RTS asserted by default: https://github.com/pyserial/pyserial/blob/master/serial/serialutil.py#L210-L211
- S10 Rust std VEXos stdio, FIFO flushed about every 1 ms: https://github.com/rust-lang/rust/blob/master/library/std/src/sys/stdio/vexos.rs
- S11 jpearman, download channel reverts after 10 s: https://www.vexforum.com/t/controller-disconnecting-at-10-seconds/78034/3
- S12 pros-cli issue 112, about 5 s disconnect on switch: https://github.com/purduesigbots/pros-cli/issues/112
- S13 VEXlink bandwidth: https://api.vex.com/v5/home/cpp/VEXlink/SerialLink.html
- S14 jpearman, smart-port serial bypasses shared memory: https://www.vexforum.com/t/how-to-communicate-with-my-debugger/143210/8
- S15 Microsoft USB MI_xx ids: https://learn.microsoft.com/en-us/windows-hardware/drivers/install/standard-usb-identifiers
- S16 pros-cli issue 198, Brain USB dump: https://github.com/purduesigbots/pros-cli/issues/198#issuecomment-1033891110
- S17 cargo-v5 connection.rs, a channel switch handled as disconnect and reconnect (lines 104-199): https://github.com/vexide/cargo-v5/blob/392c10aa480974201731ca9b771714f4f64d0b0d/src/connection.rs
- S18 PROS 4.2.2 stream_buffer.c, partial non-blocking writes: https://github.com/purduesigbots/pros/blob/4.2.2/src/rtos/stream_buffer.c
- S19 pros-cli direct_port.py, 115200 8N1 with pyserial defaults: https://github.com/purduesigbots/pros-cli/blob/45aae5df262eab1246f4d74c5be7bd1a4916eb73/pros/serial/ports/direct_port.py
- S20 vex-cdc 0.1.1 (cdc2/controller.rs), decoder stops at 0x00; MIT: https://crates.io/crates/vex-cdc/0.1.1
- S21 xnu tcp.h and socket.h (TCP_NOTSENT_LOWAT, MSG_NOSIGNAL, SO_NOSIGPIPE): https://github.com/apple-oss-distributions/xnu/blob/f6217f891ac0bb64f3d375211650a4c1ff8ca1ea/bsd/netinet/tcp.h and https://github.com/apple-oss-distributions/xnu/blob/f6217f891ac0bb64f3d375211650a4c1ff8ca1ea/bsd/sys/socket.h
- S22 pros-cli LICENSE (MPL-2.0): https://github.com/purduesigbots/pros-cli/blob/45aae5df262eab1246f4d74c5be7bd1a4916eb73/LICENSE
- S23 cargo-v5 issue 34, stdin through a controller dropped: https://github.com/vexide/cargo-v5/issues/34
- More (vex-v5-serial, Rust std, systemd, Apple, Microsoft): disc:controller-host s0, disc:pros-serial s0, disc:xplat-host.

## Revision notes

Applied from all three critiques unless listed here. Where critics conflicted:

1. **Latency fixes in phase 1** (coverage 9 vs simplicity 19): chose simplicity. Phase 1 adds no wake paths;
   the SG1 chain model showed the aux wake gains about 0.1 ms, so it is dropped; the event relay is a later
   measured step; the arrival reader moves out of spectuGATR (D10). Coverage's wording fix is kept.
2. **Capability bits** (coverage 19d vs simplicity 6): chose presence. `brainminimal` makes rule 9 testable
   without a bitmask. PublisherStats is defined (coverage 19d, rest).
3. **Pose staleness factor** (coverage 2: 1.5x vs evidence 6: 2.5x the declared pose period): chose 2.5x,
   for margin against unmeasured radio jitter; B2 can tighten it.
4. **Liveness** (coverage 2: a 1 Hz liveness frame plus a slower full heartbeat vs simplicity 1: one
   SESSION record): merged. Any frame counts; an empty 14 B frame goes only when nothing else went out for
   1 s; SESSION rides the carousel.
5. **Receiver capture** (coverage 1: reuse the Pi recorder mechanics vs simplicity 23: a raw log): chose
   the raw log. Coverage's content rules (received data only, metadata, fields, replay, read-only) are
   kept in 5.8.
6. **view/1** (coverage 3 vs simplicity 8): 1:1 with wire records plus exactly two receiver messages
   (hello, source), since port loss and clock mappings exist only at the receiver.
7. **Path assembly and identity** (coverage 11, 5 vs simplicity 1-2): paths are documents, so DOC_CHUNK
   assembly covers them; coverage's `kReplaced` is added. link_ref goes into SESSION as the placement ref,
   because CONTEXT is dropped.
8. **Durable relay frames** (coverage 4): added a flags byte, so the frame header is 14 B, not the 13 B in
   simplicity 4; the pose line is 89 characters, not 87.
9. **Path hook** (coverage 19c PathObserver vs simplicity 13): chose widening the existing PathSink.
10. **Pi publisher** (coverage 18 vs simplicity 18): the Pi publisher stays InspectionService, with a small
    channel table for new measurements; no SG1 encoder on the Pi.
11. **Platform paths** (coverage 14 vs simplicity 21): no app_paths and no config file in phase 1; the 7.5
    table keeps the other rows.
12. **Rate reduction** (evidence 12 vs simplicity 10): the adaptive halving rule is dropped; evidence's
    preemption caveat now labels the `spectu.slow_writes` counter.

Measurements made for this revision (host only; scratch under design:):
- `cobs_sg1_check.py`: the COBS check rerun with `SG1:` and `sgtr` (evidence 9).
- `chain_model_sg1.py`: chain_model.py with the SG1 terms, 1500 runs (evidence 3b, coverage 9). It
  replaces the hand-summed 26 / 40 ms and the "+9 / +13" figures.
- `sim/sg1_sweep.cpp`: the evidence-sim e2e sweep with an SG1 aux lane, 210 cells (coverage 8). It bounds
  the Pi read-budget effect only (evidence 1).

Declined or changed:
- Simplicity 32, moving Appendix A to findings.md: declined. A new handoff file is outside this revision,
  and dropping the findings would lose audit input; the appendix is kept and labeled as audit input, with
  A1 and A2 cited at D9 and D11.
- Evidence 36, renumbering sources: the new sources fill the unused numbers instead, so existing citations
  stay valid.
- Evidence 29, `.../` placeholders: replaced with the move-map folder names; the final check moved them
  to restructure/plan/FINAL.md names (`collectors/`, `publishers/`).
- Decision numbers are kept stable: D3, D7, D9, D12, D14 and D15 moved to defaults, D8 and D16 deferred,
  D17-D19 added.

Final check (sources and paths re-verified against discovery, design: outputs and ae457ee):
- nav: paths follow restructure/plan/FINAL.md; `http_server.h:183-188` corrected to 197-204.
- Corrected: radio field-on-change time (22 s), the Pi read term (3.6 ms), the idle browser figure (p50),
  the S6 description (input direction), and the basis of the brain/translaGATR/Pico test counts.
