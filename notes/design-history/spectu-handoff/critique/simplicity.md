# Critique: simplicity and the user's philosophy

2026-09-28. This is a read-only review of spectu-handoff/PROPOSAL.md; the proposal itself was not edited.

What the review checks:
- over-generalization;
- the number of new moving parts;
- a standard INPUT/OUTPUT for each component;
- folder and namespace clarity;
- PIPELINE.md coverage;
- fit with restructure/SPEC.md.

Notation:
- `P:<n>` is a PROPOSAL.md line.
- `nav:` is `pi/naviGATR/src/` after the restructure.
- Evidence tags: [code] path:line, [source], [measured-host] (cited as disc:<file> sN), [estimate], [unknown] (with the test that settles it).

## Verdict

**What to keep.** The transport core is already the simplest design that works:
- one-way SG1 lines, one writer at each end, and no new link exchanges. Each extra exchange costs 17-20 ms of link time [measured-host disc:evidence-sim s3.1];
- one line format that works with COBS on or off [measured-host disc:pros-serial s6];
- one viewer, embedded by both hosts, with the server extracted unchanged [measured-host disc:pi-side s4.1];
- nothing season-specific in the pipeline: the field arrives as data.

**Where it over-builds:**
- The wire contract has 10 record kinds, two chunking schemes, two definition tables, and identity fields that the transport already implies.
- The Brain slot and budget code uses claims, claim breaking, adaptive halving and three separate budget structs.
- The host duplicates work: two Brain input formats, two replay engines, and combination rules on both sides.
- Latency fixes and speculative options are pulled into phase 1.

**What is missing:**
- PIPELINE.md files;
- an INPUT/OUTPUT for each component;
- a namespace rule;
- the real restructured paths.

## Moving parts: proposal vs after these changes

| Part | Proposal | After | Item |
|---|---|---|---|
| Wire record kinds | 10 | 6: SESSION, POSE, DEF, SAMPLES, EVENT, DOC_CHUNK | 1-3 |
| Chunking/reassembly schemes | 2 (DOC_CHUNK offset/total, PATH chunk/chunks) | 1 | 2 |
| Rate/budget structs | 3 (Rates, StdioConfig, LaneBudget) | 1 (TransportBudget, 3 presets) | 9 |
| Slot concurrency | seqlock + CAS claim + 50 ms claim breaking on every slot | single-writer seqlock; a claim only on the event FIFO | 11 |
| Feedback loops | adaptive halving on writes over 5 ms | none in phase 1 | 10 |
| New hooks in our libraries | 4, plus a private tilt cell | 3, plus a widened PathSink; no tilt cell | 13-14 |
| New Pi wake paths/threads | event relay, aux wake, reader thread (D10) | none in phase 1 | 19 |
| Brain input formats at the receiver | 2 (SG1 frames, relayed view/1 JSON) | 1 (frames) | 18 |
| Replay engines | 2 (C++ replay source, replay.js) | 1 (replay.js) | 20 |
| Receiver helper units | device_watch, app_paths, process | none as separate units | 21 |
| CLI flags | adds --identity, --radio-download | both dropped in phase 1 | 25 |
| User decisions | 16 | 8, plus a defaults list | 32 |

Net new code is about 0.4k LOC smaller [estimate from P:836-838 and the disc:xplat-host s5 sizes]:
- replay source, about 200 LOC;
- device_watch, about 90;
- app_paths, about 90.

With item 26 the saving is about 0.7k LOC. The protocol also loses one chunker and one definition table (not sized).

## Changes

### Protocol (P s5.5)

1. [High] SESSION, CONTEXT and LOSS (P:728-737, 742-751)
   - Problem: three state records go out at the same heartbeat.
     - SESSION repeats `version`, which every frame header already carries (P:717).
     - `source_kind` and `clock_domain` are always "Brain" for SG1. The Pi publishes inspect/2 over WebSocket, never SG1 (P:127-131).
     - `robot_id` is the crc32 of the `name` sent right beside it.
     - The source kind `user` has no producer.
     - LOSS is a one-off counter record the viewer must special-case, although channels already graph numbers.
   - Fix:
     - Use one SESSION heartbeat record: name, placement, and the current doc ids (field definition, estimate, path).
     - Drop `version`, `source_kind`, `clock_domain`, `robot_id`, `capabilities` (item 6) and the `user` kind.
     - Drop the CONTEXT record.
     - Drop the LOSS record. Publisher counters become ordinary channels under a reserved `spectu.` prefix (for example `spectu.pose.replaced`), sent on the carousel. Capture writes them to CSV like any other channel. `stats()` stays for host tests.

2. [High] PATH record (P:731; scheduling order P:354-355)
   - Problem: PATH has its own chunk/chunks scheme (at most 7 segments per frame) alongside DOC_CHUNK's offset/total scheme. Both ends therefore carry two reassembly codes for the same job.
   - Fix:
     - Send a path as a document through DOC_CHUNK, with `doc_kind = path` and doc_id = crc32 of the path bytes.
     - SESSION names the current path doc.
     - A clear, cancel or complete is a new path doc with that state and no segments.
     - Keep the path ahead of samples in the scheduling order.
   - Cost:
     - A 6-segment tank path is a 98 B record [measured-host disc:wire-budget s3b]. As a document it needs 2 chunks of at most 94 data bytes (122 - 13 - 2 - 13), about 24 B more [estimate].
     - Paths change on at most 12 plans per command [code brain/actuGATR/include/actugatr/motion.h:132 via disc:wire-budget s3b].

3. [High] CHANNEL_DEF and EVENT_DEF (P:732, 734; API P:635-639)
   - Problem: two definition tables with the same shape: an id, a name, and a unit or a severity.
   - Fix: one DEF record: `id u16, kind (real, integer, bool, event), unit or severity, name`. SAMPLES and EVENT share one id space. `defineChannel` and `defineEvent` stay in the API and fill the same table.

4. [Med] Frame header on lines (P:717-721, 728, 765)
   - Problem:
     - Every SG1 line carries `AA 55 | 0x20 | len` inside the hex. The line marker already delimits and identifies the frame, the hex already restricts the characters, and CRC16 catches corruption.
     - Sync, type and len matter only for the deferred RS-485 binary frames (P:211-213) [inference].
     - Clock mapping gets only one source-clock sample per heartbeat (`source_now_ms`: 1 Hz, 0.2 Hz on radio).
   - Fix:
     - Line frame = `version u8 | session u32 | frame_seq u16 | sent_ms u32 | records | crc16`. That is 13 B, the same as today, so the 41 B pose frame and its 87-character line are unchanged [estimate].
     - Every frame then feeds the one-way min-offset mapping. This is the per-sample method of nav:time/clock_sync.h [code nav:time/clock_sync.h:30-79 via disc:pi-side s4.1].
     - Drop `source_now_ms`.
     - Bring sync, type and len back only when a binary transport exists.

5. [Med] Channel types and fields (P:597-604, 732)
   - Problems:
     - `kEnum` has no labels on the wire, so it can only be drawn as an integer.
     - `scale_exp` is on the wire but not in `ChannelSpec`, so nothing says who chooses it.
     - `group` repeats the name prefix (`"drive.cross_track"` and `"drive"`).
   - Fix:
     - Values are f32 on the wire (the same 4 B), and `scale_exp` is dropped.
     - Types are real, integer and bool, plus event from item 3. Add kEnum back only when labels are needed.
     - The group is the name up to the first dot; `group` is dropped from ChannelSpec and DEF.

6. [Med] `capabilities u32` (P:728; rule 9 P:776-777)
   - Problem:
     - The bits are never listed.
     - The viewer can already see presence directly, because DEFs and docs arrive at start and on the carousel.
     - The viewer already gates panels this way ("no Pico in this configuration") [code disc:viewer s3].
   - Fix: a capability is present once its first record or DEF arrives; until then its panel stays hidden. Remove the bitmask. The Pi keeps its inspect/2 `hello.features` as is.

7. [Med] `display_key` (P:459-470, 552, 729)
   - Problem: a second field identity beside `field_id`.
     - In integrated mode it equals map_id (P:459).
     - It covers the planning map only (P:468), which field_id already identifies.
     - With D14(a), standalone mode has no display assets in phase 1 (P:928).
   - Fix:
     - Key display assets by `field_id`. The Pi adapter computes field_id for each FieldMap with the same spectuGATR/protocol encoder the Brain uses.
     - Add one round-trip test: the bytes from "Pi map doc -> Brain Field -> spectu definition" must equal the bytes from "Pi FieldMap -> spectu definition".
     - Whether mm and rad rounding keeps the bytes identical is [unknown]; that test settles it.
     - Drop `DisplayKey` from the API and the wire.

8. [Med] Browser contract `spectugatr.view/1` (named only at P:840)
   - Problem:
     - The view messages are never listed, so the seam between receiver and viewer has no stated INPUT/OUTPUT.
     - The overlays for desired heading and follower target come from TELEMETRY fields today [code disc:viewer s2, field_scene.js:1395-1438]. With generic channels they depend on channel names that nobody has listed.
   - Fix:
     - In spectuGATR/docs/PROTOCOL.md, define view/1 as a 1:1 JSON rendering of each wire record plus `{src, rx_us}`, with no view-only message types.
     - Add a short table of the well-known channel names the viewer draws specially, for example `motion.desired_heading`, `motion.follow_x` and `motion.follow_y`. Every other channel is a plain graph.

### Brain library (P s4.3, s5.1-5.4)

9. [High] Rates and budgets (P:339-343, 608-618, 651-656)
   - Problem: one concept, how much viz a transport may carry, is split over three structs:
     - `Rates` in PublisherConfig;
     - a token bucket in `StdioConfig`;
     - a `LaneBudget`.

     The 4.3 table has one row per transport, but no struct in the code matches a row.
   - Fix: one data-only struct with the table's columns.
     ```cpp
     struct TransportBudget {
         double  bytes_per_s;
         int     burst_bytes;
         double  pose_hz;
         double  samples_hz;
         Seconds carousel;
         Seconds heartbeat;
     };
     ```
     - Three named presets, `kUsbRelayBudget`, `kDirectUsbBudget` and `kRadioBudget`, hold the 4.3 numbers.
     - The transport adapter gets one preset, and `nextLine` takes it.
     - Remove `Rates` from PublisherConfig.

10. [High] Adaptive rate reduction (P:356-358)
    - Problem:
      - "A write over 5 ms halves the budget for 1 s, then +10% per second" is a feedback loop built on unknowns (B5, B7).
      - On USB the aux lane cannot see write durations, because communiGATR's tx task does the write [code brain/communiGATR/pros/pros_usb_port.cpp:98-124]. The rule would need another hook back from communiGATR.
    - Fix:
      - Phase 1 uses the fixed TransportBudget plus a `spectu.slow_writes` channel.
      - The fixed budget still meets "rate reduction under pressure" (requirements.md:188): lower kinds starve by priority and live state coalesces latest-wins.
      - Add adaptation only if B7 shows the PROS buffer filling at the preset budget.

11. [High] Slot concurrency (P:675-682, 689)
    - Problem: every slot has a seqlock, a CAS claim, claim breaking after 50 ms, and a `contended` stat. Claims are only needed where two tasks write the same slot. The call-site table shows one producer task per live slot [code disc:brain-owners s7]:
      - pose and field: the poll task;
      - path: the drive task;
      - each channel: the task that samples it.

      Only events come from several tasks (poll, drive, app).
    - Fix:
      - Rule: one producer task per slot, stated next to each method.
      - Live slots are single-writer seqlocks with no claim. A writer deleted mid-copy leaves the sequence odd; readers skip the slot until the next write restores it [inference].
      - The claim and claim breaking exist only in the event FIFO, and so does `contended`.

12. [Med] Transport adapters (P:641-657)
    - Problems:
      - `bool startStdioTransport(Publisher&, const StdioConfig&)` owns a task and a token bucket but returns no handle, so it hides where that state lives.
      - `usbLane` returns a function pointer plus context, and the context must also hold bucket state.
      - `nextLine` sits on the user-facing Publisher next to `updatePose`.
    - Fix:
      - Add one small `ByteBucket` struct shared by both adapters. Each adapter becomes a loop or callback that calls `nextLine` and moves bytes, about 30 lines each [estimate].
      - Make the stdio adapter a class, `StdioTransport`, that owns its task and bucket. That follows the user's rule: data plus behavior is a class.
      - Move `nextLine` and `stats` into `transport.h`, so that `publisher.h` holds only producer calls.

13. [High] Hooks in communiGATR and actuGATR (P:93, 139, 294-295, 692-703)
    - Problems:
      - The inventory says "two neutral Brain hooks" (P:93), but 5.4 adds four: a Client observer, a Motion path observer, a ProsDrive observer and the aux-line source.
      - The Motion path observer runs parallel to the existing PathSink seam. Motion holds one sink pointer [code brain/actuGATR/include/actugatr/motion.h:161, 189], and PathSink is `reportPath(CommandId, const Path&)` [code brain/investiGATR/include/investigatr/path.h:44-48].
      - The proposal says "function pointer plus context" (P:139), but the precedent it cites is a `std::function` [code brain/communiGATR/include/communigatr/client.h:68].
    - Fix:
      - Widen PathSink once: `reportPath(CommandId, const Path&, const PathContext&)`, where PathContext = {state, reason, frame, field generation}. ProsLink ignores the context.
      - While PATH_REPORT still exists (D4), a 10-line tee sink feeds both consumers; delete it when PATH_REPORT goes.
      - Every new hook is a `std::function` member named by a `using` alias (`using StateObserver = std::function<void(const StateSample&)>;`), matching bench_imu.
      - Correct the hook count at P:93.

14. [Med] Tilt (P:698)
    - Problem: the integration's private "lock-free tilt cell" is a second handoff mechanism alongside the publisher's slots.
    - Fix:
      - Add a tilt slot to the Publisher: `void updateTilt(Radians roll, Radians pitch, Seconds measured_at);`.
      - The app owns the IMU, so the app calls it.
      - POSE already carries the latest tilt with its own age (P:730).
      - `updatePose(x, y, Attitude)` stays for users with a single loop.

15. [Low] PathUpdate `mode` and `reason` (P:585-595)
    - Problem: `PlanMode {kDirect, kAvoiding, kOther}` is actuGATR's planner vocabulary inside a generic API, and `reason` is a second free-text field.
    - Fix: one `std::string_view note` (at most 24 chars) covers both ("avoiding", "cancelled by caller"). The integration fills it.

16. [Low] Comment style in the API sketch (P:499, 522, 548, 624, 629, 641)
    - Problem: the comments are sentences and instructions, not labels. Examples: "caller may reserve once; publisher copies"; "Producers: copy into a slot and return. Never block, allocate or do I/O."
    - Fix:
      - Use bare labels, per feedback_comment_style.md: `// never blocks`, `// seconds, publisher clock`, `// 0 when nothing is due`.
      - State the non-blocking rule once, in PIPELINE.md.

### Pi side (P s3.3, s4.2, s4.4, W7, D10)

17. [Med] Relay ring seam (P:297, 840)
    - Problem: debug/inspection must read a ring owned by the `pros_usb_link` plug-in, so debug code would depend on one plug-in's header. The proposal does not name the seam type.
    - Fix:
      - Declare the ring as a stage-level resource type, `nav:pipeline/resources/brain_viz_lines.h`, next to `brain_imu_bench.h`. The move map puts that Brain-link resource type at stage level [code restructure/plan/move_map.tsv:147].
      - `pros_usb_link` fills the ring. debug/inspection includes only that header.

18. [High] One Brain decode path (P:22-25, 127-131, 786, 840)
    - Problem:
      - The Pi adapter decodes Brain frames into view/1 JSON, and the laptop's `--source navigatr` consumes that JSON.
      - The receiver therefore takes Brain data in two formats: frames from serial and JSON from the Pi. That means two decoders and two capture forms.
      - The requirement "the Pi has its own publisher instance" is left implicit.
    - Fix:
      - The Pi forwards SG1 frames verbatim on a subscribable channel (`brain_frames`), each with its Pi arrival time labeled as relay time.
      - Decoding happens once, in the shared decode library, wherever a browser is served (Pi or laptop).
      - Pi messages to a laptop are wrapped as `{"src":"navigatr","rx_us":N,"msg":<original>}`. They are never parsed and re-serialized, so the C++ receiver needs no JSON parser.
      - Add to s2: "The Pi publisher is the existing InspectionService speaking inspect/2. The Pi has no SG1 encoder."

19. [High] Latency fixes in phase 1 (P:375-379, 406-414, 924)
    - Problem: phase 1 adds three new wake paths:
      - an aux-lane wake (term 2, about 1 ms mean);
      - an event-driven relay (term 6, about 2.5 ms mean);
      - after B7 and B8, a reader thread in `pros_usb_link` (term 5, about 5 ms).

      `pros_usb_link` is a localization-link resource, and localization polling takes priority (requirements.md:190).
    - Fix:
      - Ship phase 1 with none of these.
      - Bench the relay (B7, B8), then add each fix as its own step with before/after numbers.
      - Remove the reader thread from D10. It changes the localization link's I/O model for a viz gain. Offer it later as a localization-latency change on its own merits.

### Receiver (P s5.6, s5.7, W3, W5)

20. [High] Replay (P:787, 838, 841)
    - Problem:
      - `spectugatr --replay capture.zip` implies a C++ replay source of about 200 LOC.
      - The viewer already replays bundles in the browser, with a shared playhead and no send path [code disc:viewer s7, replay.js ReplayModel/ReplayClock].
      - M6 extends that replay to multiple sources anyway.
    - Fix: `--replay FILE` starts the server with no sources and hands the bundle to replay.js, the same path as dropping a file on the page. Delete `replay_source` from W5.

21. [Med] Helper components (P:836; disc:xplat-host s5)
    - Problems:
      - `app_paths` (per-OS config, state and capture folders) serves a config file that phase 1 does not have (D15, P:795-796). Captures already default to memory [disc:xplat-host s2.7].
      - `device_watch` is a separate 1 Hz watcher, although each serial source thread already loops.
      - `process` is about 50 lines of signal setup.
    - Fix:
      - Drop `app_paths`; `--capture-dir` is explicit.
      - Each serial source rediscovers its own device at 1 Hz inside its thread after a loss. Drop `device_watch`.
      - Put the signal handling in main.cpp.

22. [High] Receiver structure and rule homes (P:107-108, 753-778, 838, 841)
    - Problems:
      - `view/` ("view and combination", about 600 LOC) has no stated input and output.
      - Several 5.6 rules are presentation: stale drawing, placement dimming, capability gating. M2-M5 re-implement the same concerns in the viewer (CoreScene, per-source status), and nothing gives each rule one home.
    - Fix: name the folders after the receiver's stages and give each one an INPUT and OUTPUT.
      ```
      spectuGATR/host/
        PIPELINE.md
        sources/   device bytes or Pi WS messages -> SourceChunk {src, rx_us, bytes|text}
        decode/    SG1 lines -> protocol records -> view/1 JSON   (naviGATR links it too)
        combine/   records per source -> keyed state, clock maps, seq gaps
        server/    view/1 messages -> browsers (slots, reliable FIFO, flow control)
        capture/   SourceChunks -> bundle
        platform/  serial, usb ports, browser launch
      ```
      Add a table to s5.6 that gives each of rules 1-10 exactly one side:
      - receiver: 1 source keying, 4 no merge by arrival, 5 clock mapping, 8 duplicates, 10 sequence gaps;
      - viewer: 2 core scene from a Brain only, 3 stale drawing, 6 placement dimming, 7 field choice, 9 capability gating.

23. [Med] Receiver capture (P:90)
    - Problem: "reuses the recorder mechanics, the bundle framework and zip" means refactoring naviGATR's working capture into generic parts plus an adapter first. That capture is tied to DiagnosticsHub and DiagRecord [code disc:pi-side s5].
    - Fix:
      - The phase-1 receiver capture is a bounded raw log of SourceChunks (frames with rx_us).
      - One export function decodes the log to one CSV per record kind, plus fields/*.json and metadata.
      - Reuse only zip_writer and the CSV escaping. Leave nav:debug/capture untouched.
      - Pi instrumentation capture stays gatr2.capture/1 (requirements.md:322).
      - With this design, "capture contains only what it received" holds by construction.

24. [Med] Duplicate-source rule (P:774-775)
    - Problem: rule 8 dedupes one Brain session that arrives over two transports. In phase 1 that can only happen with the controller and Pi USB both attached, which the proposal forbids until B1 passes (P:189, 310-315).
    - Fix:
      - Phase 1 refuses a second Brain source for the same robot name and says why.
      - Deduplication comes with D16, if B1 shows duplication.

25. [Med] CLI (P:782-791)
    - Problem:
      - `--identity` needs a stable device identity, which is [unknown] until B9.
      - `--radio-download` is pending B4 and the game rules (D8).
    - Fix:
      - Phase-1 flags: `--source`, `--device`, `--replay`, `--port`, `--bind`, `--no-browser`, `--capture-dir` and `--viewer-dir`.
      - The receiver never switches radio channels.
      - Add the two flags once B9 and B4 are answered.

26. [Low, user choice] Topology (c) (P:203-219)
    - Problem: (c) is the only phase-1 reason for two pieces of laptop code:
      - a production WebSocket client, about 300 LOC [disc:xplat-host s2.5];
      - cross-source clock mapping.
    - Fix (option):
      - In RS-485 mode communiGATR never touches stdio [code brain/communiGATR/pros/pros_link.cpp:84-91], so the Brain's USB user port is free.
      - Cable that port to the Pi and run the same SG1 split and relay as (b), using a read-only `pros_usb_link` instance. (c) then looks identical to (b) from the viewer.
      - The cost is a USB cable on the RS-485 robot; whether that is acceptable is [unknown], the user's call.
      - List this as a decision next to the controller-radio route.

### Viewer (P s1, s4.6, W1, W8)

27. [Med] Folder names and compatibility (P:86, 478, 841, 844-845)
    - Problems:
      - `viewer/sources/navigatr/` reuses the word "sources", which already means the serial and WebSocket inputs in host/.
      - The live inspect/2 compatibility path and the `window.__navigatr` alias outlive their need. W7 (the Pi speaks the envelope) lands before W8, and the test tools move in W1.
    - Fix:
      - Generic modules stay at the viewer root, and the Pi panels move to `viewer/navigatr/`.
      - Switch the test tools to `window.__spectugatr` in W1 and drop the alias.
      - Keep the inspect/2 reader only for replaying old gatr2.capture/1 bundles.

28. [Med] Landmarks and the restructure's error form (P:435-472, 559-565; restructure/SPEC.md "world_estimation")
    - Problem:
      - After the restructure, world estimation outputs `LandmarkError{dx, dy, dtheta}`: the error from the nominal pose, in field axes.
      - Inspection reads displacement and heading error from that value, while the Brain wire keeps full poses.
      - The proposal's Brain estimate and landmark table never mention this form, so the Brain-only view and the Pi view could show different quantities.
    - Fix:
      - Keep full poses on the spectu wire, matching the unchanged Brain wire.
      - One viewer helper derives dx, dy and dtheta by LandmarkError's definition: field axes, dtheta = wrap(pose heading - nominal heading). Both sources use it.
      - The viewer PIPELINE.md and the landmark panel lead with "lining up relative to field objects", the same framing as world_estimation's PIPELINE.md.

### Structure and docs

29. [High] PIPELINE.md and INPUT/OUTPUT (the whole proposal; no PIPELINE.md is mentioned)
    - Problem:
      - The user requires a concise pipeline .md for every pipeline and for the whole system, and a standard INPUT/OUTPUT per component (memory feedback_system_philosophy.md).
      - The restructure writes one for each naviGATR stage (restructure/SPEC.md).
      - The proposal plans only PROTOCOL.md (P:833).
    - Fix: add these files to W0, W1, W4, W5 and W7, and list them in s7.1.
      - `spectuGATR/PIPELINE.md`: the whole product in about 10 lines. Its lead could be: "Show what the robot knows and uses, with honest ages, without taking link time from localization."
      - `brain/spectuGATR/PIPELINE.md`: producer calls -> slots -> scheduler (`nextLine`, TransportBudget) -> transport owner -> channel 1.
      - `spectuGATR/host/PIPELINE.md`: the item 22 table.
      - `spectuGATR/viewer/PIPELINE.md`: socket -> per-source state -> core scene and Pi panels -> render loop.
      - `nav:debug/inspection/PIPELINE.md`: in = System snapshots, the diagnostics hub, BrainVizLines; out = inspect/2 plus relayed frames.
      - In s2, add a five-row table: component, INPUT, OUTPUT, owning task or thread.

30. [Med] Namespaces (P:487; disc:xplat-host s5)
    - Problem:
      - The proposal shows only `spectugatr`.
      - The host namespaces appear only in a discovery note.
      - The protocol and the integrations have none.
    - Fix: mirror the folders.
      - `spectugatr::protocol`;
      - `spectugatr` for the Brain public API, so users write `spectugatr::Publisher`;
      - `spectugatr::integrations::{communigatr, actugatr, investigatr}`;
      - `spectugatr::host::{sources, decode, combine, server, capture, platform}`.

      naviGATR keeps `navigatr` and adds aliases for the server types (P:835).

31. [High] Restructured paths (P:73-75, 449, 955-968)
    - Problem:
      - `.../` hides the real plug-in folders, so no citation can be followed. The move map fixes them [code restructure/plan/move_map.md:75-82]:
        - `pipeline/resources/resources/pros_usb_link.*`;
        - `pipeline/commands/collectors/brain_link_commands.*`;
        - `pipeline/publishing/publishers/{brain_link_publisher,field_documents}.*`;
        - `pipeline/localization/observations/brain_imu_wheel_bench.*`.
      - "The restructure is a pure move" (P:75) is wrong for two steps:
        - step 2 splits system.cpp;
        - step 3 changes world estimation, and inspection then reads the error directly.

        So citations such as `inspection_document.cpp:1188-1253` (P:449) can shift [restructure/SPEC.md].
    - Fix:
      - Spell out the real paths in the text and in Appendix B.
      - Change P:75 to: "Lines are from ae457ee. Files split in restructure step 2 or edited in step 3 are re-cited after the patch lands."

32. [Med] Decision list and appendix (P:911-953)
    - Problem:
      - There are 16 decisions, and several are defaults or bench outcomes rather than genuine user input (requirements.md:403).
      - Appendix A (18 defects) is input for the later audit, not part of the requested deliverable.
    - Fix:
      - Keep D1, D2, D4, D5, D6, D10 (without the reader thread, item 19), D11 and D13, plus item 26 if adopted.
      - Move D3, D7, D9, D12, D14 and D15 to a "defaults unless you object" list.
      - Drop D8 and D16 from phase 1 (items 24 and 25).
      - Move A3-A18 to `spectu-handoff/findings.md`. Keep A1 and A2 inline at D9 and D11.
