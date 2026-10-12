# Evidence critique of spectu-handoff/PROPOSAL.md

2026-09-28. Lens: evidence. Read-only; PROPOSAL.md not edited.
Checked against: discovery/*.md (all ten), viz-handoff/final_report.md, restructure/SPEC.md and
restructure/plan, the repo at ae457ee (disk folder pi/navigatr), and the PROS 4.2.2 clone in
scratch/spectu/pros-serial. Refs opened are listed at the end; all of them matched.

Each item: section, problem with its basis, exact fix. Most serious first.

## A. Numbers or claims that change a conclusion

1. **7.2 E2e rig; 0 "Why not new link ops"; 4.3 "Localization has priority".**
   The planned acceptance ("polls/s and poll-gap p95 within 1% of off") passes by construction.
   The rig's UsbWire replaces ProsUsbPort. It delivers every character of a line at
   write + line_us: no per-character time, no 2047 B FIFO, no 2 ms flush, no tx task
   [code pi/naviGATR/tests/brain_link_e2e_gtest.cpp:136-172; disc:evidence-sim s2 "no size effect by
   construction"]. The aux lane would live in pros/pros_usb_port.cpp, which the rig never runs. The
   only cost it can show is the Pi's 1024 B read budget.
   Fix: replace the rig bullet with:
   "Extend UsbWire with a Brain-side model: tx task (link lane first, capped aux lane), a 2047 B
   FIFO flushed every 2 ms, and a per-character time swept over an assumed range. Report polls/s,
   poll-gap p95 and request queueing delay against aux rate. The host result bounds the Pi
   read-budget effect only; B7 settles the Brain-side cost."
   In section 0 add "(the host line model has no byte time)".

2. **4.3 Rules ("on the Pi, the split happens outside the request drain"); section 0 ("Localization
   and world estimation never see it").**
   In phase 1 the split is inside the drain. finishLine() runs from readAvailable(), which
   BrainLinkCommands calls in its drain loop on the estimation worker. SG1 bytes also use the
   4 x 256 B per-call read budget [code pros_usb_link.cpp:216-268; brain_link_commands.cpp:280-299;
   disc:pi-side s1.5 C]. Only the deferred D10 reader thread moves the split out of the drain.
   Fix, 4.3: "On the Pi, SG1 lines are split inside the link resource during the commands drain.
   They run on the same worker and use the same 1024 B read budget, and the parse cost is
   [unknown B7]. They never reach the FrameReader or localization."
   Fix, section 0: append "(they share the link resource's read budget and worker time)".

3. **0 "Latency conflict"; 4.4 totals.** The 26 / 40 ms figure is a hand sum, but it is presented
   next to a measurement as if both came from one probe.
   - latency-chain measured or modeled only the piggyback, separate-request, radio and direct-USB
     shapes [disc:latency-chain s2]. The SG1 aux-lane relay was never run in chain_model.py or
     latprobe.
   - Section 0 says "in the same probe", but only 17 / 27 comes from latprobe.
   - The p95 cells (40, 31) have no method. The proposal itself says "p95 values are not additive".
   - "With fixes ... up to about 7" does not follow from the listed terms:
     0.5 + 2 + 2 + ~0 + ~0 + 0.4 = 4.9.
   Fix:
   (a) Section 0: "The SG1 relay is estimated at about 26 ms mean (terms 2-7 added to latprobe's
       measured 15.4 / 25.4). The p95 of about 40 is a rough bound, not modeled. latprobe measures
       the Pi-authoritative path at 17 / 27 ms."
   (b) 4.4: run chain_model.py with the SG1 terms (uniform 0-2, 0-2, 0.5-2, 0-10, 0-5, 0.2-0.4)
       over latprobe's receipt ages and cite the output. Otherwise label both p95 cells
       "[estimate, rough bound]".
   (c) Change "up to about 7" to "up to about 5", or name the extra terms.
   (d) 7.2: "Rerun latprobe with the SG1 lane before D10 is accepted."

4. **4.3 "Initial rates and budgets".** The field times contradict the proposal's own scheduling
   order, which sends documents last from leftover budget. Arithmetic uses the proposal's sizes:
   pose line 87 chars; samples frame 26-68 B = 57-141 chars; field 2338 chars; heartbeat about
   190 chars/s on USB [estimate].
   - USB relay: 4000 - 2175 (pose 25 Hz) - 570..1410 (samples 10 Hz, 1-8 channels) - 190 leaves
     about 225-1235 chars/s. The field takes 2-10 s, not "about 1 s".
   - Direct USB: 8000 - 4350 (50 Hz) - 1140..2820 (20 Hz) - 190 leaves about 640-2320/s.
     The field takes 1-4 s, not "under 0.5 s".
   - Radio at 400 B/s with COBS (93 B per pose line, 4.3 radio row):
     - 5 Hz pose alone is 465 B/s, which is over budget.
     - 3 Hz pose + 1 Hz samples (63-147 B) + 5 s heartbeat (about 40 B/s) is 380-465 B/s.
     - "About 6 s" is 2400 / 400, which spends the whole budget on the field. The real figure is
       about 2 min, or never.
   Fix: recompute every row from the stated order, with one line of arithmetic per row, labeled
   [estimate]. On the radio, cap pose at 3 Hz with 1-channel samples, or raise the placeholder
   budget. State the field time on the radio as "minutes at 400 B/s; B2 decides".
   Carry the change into D12.

5. **5.5 record table.** Several stated sizes do not match the listed fields.
   - CONTEXT: the listed fields sum to 17 B body (19 B record), but the table says 24. The 24 is
     wire-budget's record, which also carried odometry_epoch and anchor_revision
     [disc:wire-budget s4]. The proposal moved those two to channels (5.4).
   - PATH: the table gives wire-budget's 26 + 12n, but the proposal's fields differ:
     - field_id u32 replaces context_gen u8 (+3 B);
     - a target pose (+10 B) and a reason (<= 24 B) are added;
     - the active record is therefore about 38 + 12n.
     With 13 B overhead in a 122 B stdio frame, at most 5 segments fit per frame (wire-budget
     allowed 7). The tank-avoiding p50 of 6 segments then needs 2 frames, and section 6
     "record 98 / 182 B" is stale.
   - DOC_CHUNK: the listed fields (doc_kind, doc_id u32, total u16, offset u16) sum to 9 B, but the
     table says 13. The 13 is the READ_DOC header, which includes doc_crc32 [disc:wire-budget s4].
   - The Bytes column mixes body sizes (EVENT 12) with record sizes (SESSION <= 34, CONTEXT 24).
   Fix: use one "record B" column computed from the listed fields: SESSION 34, CONTEXT 19, POSE 28,
   PATH about 38 + 12n (+ reason), SAMPLES 7 + 6n, EVENT 14, DOC_CHUNK 11 + data (or add doc_crc32
   and 15 + data). Then recompute the section 6 path row, and the field framed totals 668 / 476 if
   the chunk header changes.

6. **5.6 rule 3, and the smoothing rule, against the radio rates in 4.3.** The numbers conflict.
   - A source goes stale after "no frame for 1 s", but the radio heartbeat is every 5 s
     (SESSION: "0.2 Hz on radio"). A standalone user who publishes only on change is marked stale
     between heartbeats.
   - The viewer marks a pose stale after 250 ms without a new state [code
     spectaGATR/smoothing.js:19, 26-34]. At the radio pose cap of 3-5 Hz (200-333 ms between
     poses), the robot flips to stale between updates.
   Fix: make both limits per transport, for example stale = max(1 s, 2.5 x heartbeat period), and
   pose-stale = max(250 ms, 2.5 x the source's declared pose period). Declare the pose period in
   SESSION or CONTEXT. Add the radio case to the `sourceloss` browser scenario.

7. **4.5 "So the Pi can be 0.5 s plus the transfer time ahead of the Brain".** This is unlabeled and
   partly wrong.
   - After a new map completes, the client reads the estimate at once
     (next_estimate_ = kLongAgo) [code brain/communiGATR/src/client.cpp:686].
   - field_period (0.5 s) gates only estimate-only updates [code client.cpp:1260-1276; client.h:59].
   - Three failed transfers add a 1 s backoff [code client.h:60].
   Fix: "[inference] A new map reaches the Brain after map + estimate transfer: 0.25-1.1 s on host
   rigs [measured-host disc:wire-budget s2]. A new estimate arrives after up to 0.5 s plus its
   transfer. Either can take over 1 s longer after failures (1 s backoff). The Pi's live
   field_objects can be newer still, because snapshots are rate-limited to 200 ms
   [code field_documents.cpp:303-341]."

## B. Overclaims: inferred or planned work shown as demonstrated

8. **1 (Inspection server row) and 6 (Server extraction row): "It compiles on its own, and 23/23
   server tests pass".**
   - Only http_server.cpp (+3 headers) and zip_writer.cpp were compiled standalone.
   - The 23/23 ran in today's navigatr_tests.exe [disc:pi-side s4.1].
   - xplat-host linked http_server + viewer_assets into a 1.40 MB static probe exe, and ran 47/47
     related tests on the existing binary [disc:xplat-host s1.3].
   Fix: "http_server.cpp and zip_writer.cpp compile standalone. A probe exe of http_server plus the
   embedded viewer links statically (1.40 MB) and serves the page. The 23 existing server tests
   pass in today's navigatr_tests.exe. The extracted spectugatr_server target has not been built."

9. **3.2 "The same holds for SG1 [measured-host disc:pros-serial s6, cobs_ng1_check.py]".**
   The script tested only the `NG1:` marker with stream id `ngtr`
   [scratch spectu/pros-serial/cobs_ng1_check.py:17, 22, 31-32].
   Fix: rerun the script with `SG1:` and `sgtr` (a one-line change) and cite the output. Otherwise
   write "[inference: same lengths, different marker and id]".

10. **Section 6 "SG1 line after an NG1 request | Reply still sent"; F7.**
    - Probe 3 measured link-level behavior only: the request bytes were delivered, and the SG1 line
      was counted as console [disc:pi-side s1.4].
    - That the reply is then sent follows from the drain counting only decoded frame bytes
      (completed_at < drained) [code brain_link_commands.cpp:365]. That is an inference.
    - F7's "Bytes after a request ... suppress" is ambiguous, because it reads as including console
      lines.
    Fix, section 6: "Request bytes delivered; SG1 line counted as console [measured-host]. Reply
    sent [inference, brain_link_commands.cpp:365]; the 7.2 split test asserts it."
    Fix, F7: "Decoded NG1 frame bytes after a request in the same drain suppress its reply. Non-NG1
    lines do not (probe 3)."

11. **4.1 Framing, "It survives every channel-1 case".** NUL padding is inferred, and surviving it
    needs a receiver rule the proposal does not state.
    - The v5x claim is that each FIFO reply is padded to a word boundary with NULs
      [disc:controller-host s9b step 7].
    - A line split across two USER_READ replies would then carry NULs inside its hex and be dropped.
    Fix: "NUL padding [inference; B3]: the hex, the marker and the newline contain no 0x00, so the
    receiver discards every 0x00 before assembling lines." Add the v5x source to Sources
    (github.com/v5x-dev/v5x/pull/329, a claim only).

12. **4.3 Rules: "A write longer than 5 ms means the PROS buffer was full [estimate; confirmed by B5
    and B7]".**
    - "confirmed" reads as done.
    - The premise is an inference. usb-tx runs at DEFAULT priority, and the DEFAULT+1 poll and drive
      tasks can preempt it [code pros_usb_port.cpp:203-220; actuGATR/pros/pros_drive.h:27-28].
    - The rule never fires on a non-blocking fd (item 17).
    Fix: "[inference; to be checked by B5, B7 and B11]. A slow write can also be preemption. A false
    trigger only lowers the viz rate, not link time."

13. **3.4 "Demonstrated: the separation, but only by code and source (F8)".**
    Code reading is not a demonstration.
    Fix: "Established from code and source only; nothing ran."

14. **4.3, "One-way 0x20 frames in the RS-485 post-reply gap ... fits the 5 ms gap".**
    - The proposal omits the contrary finding: on RS-485 only request/reply fits, because the Pi
      refuses to transmit while input is pending and bytes after a request suppress its reply
      [disc:pi-side s1.5 D; code half_duplex.cpp:95-99; brain_link_commands.cpp:365].
    - Probe 2 (a frame before the request is answered) supports the idea only if the frame ends
      before the next request.
    Fix: append "[estimate; disc:pi-side s1.5 D reads it as unsafe; needs a half-duplex host test
    before any later phase]".

15. **7.4 "What is tested where".** The table reads as done, but nothing of the receiver exists.
    - Discovery built only on Windows 11 (probes and existing binaries).
    - macOS and Linux were not built or run [disc:xplat-host header].
    - The Linux CI job compiles today's naviGATR, server included [code .github/workflows/ci.yaml:28-45],
      but its run history is not visible [disc:xplat-host s1.2].
    Fix: retitle it "Planned test matrix" and add a "Done so far" column:
    - Windows 11: probe exe and existing suites;
    - Linux x64: existing CI compiles http_server (history not visible);
    - Linux arm64 and macOS: nothing.

16. **7.2 "Counts at ae457ee: brain 455, translaGATR 254, Pico 129, naviGATR 626".**
    - The first three were measured at 2d11c67 plus 46 uncommitted files [viz final_report header, s3].
    - wire-budget s6 infers that those files became ae457ee.
    - Only naviGATR 626 was listed at ae457ee [restructure/logs/we-plan.base.ctestN.txt].
    Fix: "naviGATR 626 at ae457ee [measured-host restructure/logs/we-plan.base.ctestN.txt]. brain 455,
    translaGATR 254 and Pico 129 were measured at 2d11c67 + working tree, believed equal to ae457ee
    [inference disc:wire-budget s6]. Recount at W0 start."

## C. Bench-test gaps and hardware claims

17. **3.5 B11.** The test cannot see the failure it targets.
    - With a non-blocking fd, PROS writes min(len, free) bytes and still returns len. It returns 0
      (EIO) only when nothing fits [source S1 ser_driver.c:172-179, 187-194; stream_buffer.c:662 via
      disc:pros-serial s1.1].
    - "Watch for short writes" therefore misses truncation.
    Fix, B11: "Fill the buffer with the receiver not reading. Count truncated or malformed lines at
    the receiver and write() calls returning 0, and log write durations. A non-blocking fd shows
    truncated lines and near-0 ms writes."

18. **3.2 / 3.3 / 3.4 "Untested" lists.** B11 is in the wrong list, and (c) omits shared unknowns.
    - B11 concerns the new /ser/sgtr stream, which only (a) and (c) write.
    - In (b), aux lines go through communiGATR's existing /ser/ngtr fd (A9).
    - (c) lists only B2 and B12, but its channel-1 path is the standalone one.
    Fix:
    - (a): B2-B6, B9, B11, B12.
    - (b): B7, B8, B10 and A9.
    - (c): the same as (a).

19. **4.4 row 7 ("Wi-Fi [unknown]") and row 4 ("[unknown B7]").**
    - The conventions require a Bn on every [unknown]. Row 7 has none (disc:latency-chain open Q7).
    - B7 measures throughput and buffer fill, not one-way latency.
    Fix: add "B13: Pi -> laptop WebSocket latency over Wi-Fi and the SSH tunnel. Ping RTT and
    publish-to-receive from the existing latency terms, 5 min." Row 4: "[unknown; B8 bounds it via
    RTT/2]. VEXos drains the FIFO about every 1 ms [source Rust std vexos.rs:43-45 via
    disc:controller-host s5]."

20. **3.5 B1.** The input-merge part of the test is ambiguous.
    - Controller stdin writes have three incompatible encodings; pros-cli disables them, and
      cargo-v5 #34 reports drops [disc:controller-host s2.1, s3].
    - If no L lines arrive, B1 cannot tell "no merge" from "wrong encoding".
    Fix: "Send L lines with both [02,len,data] and [01,len,data]. Add a run with the Pi port closed
    as a positive control for controller input." Also correct the basis: "5 min per run" comes from
    disc:controller-host s6, not pros-serial s3.3.

21. **4.4 "Direct USB, standalone ... about 2-4 ms"; 4.1 direct USB.** The estimate omits a known
    quirk: a user-port transfer of exactly 512 B is held on Windows and Linux hosts until the next
    transfer [source vexforum 127844/11-12 via disc:pros-serial s2, disc:controller-host s5].
    Fix: add to B7 "count receiver stalls after reads whose size is a multiple of 512 B", and add
    "(excludes the 512 B hold-back)" to the estimate.

22. **4.4 "Pi-reported ages are already biased low, by about 7.7 ms mean".** This is the latprobe
    value (9.45 reported against 17.1 true). On hardware the bias is the Pico -> Pi transfer
    floor, which is unmeasured [disc:latency-chain s1 row 4a; open Q2, Q4].
    Fix: "by about 7.7 ms mean in the host probe [measured-host]; the hardware value is [unknown B8]".
    Add to B8: "Pico -> Pi minimum latency (UART tap against Pi receive stamps), with robot_age_ms."

23. **4.1 radio figure "near 500 B/s"; "measured fallback".**
    - 500 B/s is a jpearman forum figure (thread 144920) that controller-host could not re-verify
      (Cloudflare 403) [disc:controller-host s5; disc:pros-serial s3.1], and it is not in Sources.
    - "measured fallback" implies a measurement.
    Fix: add the thread as a source marked "unverified", and write "a base64 line variant (1.33x
    instead of 2x) is the fallback to evaluate after B2".

## D. Wrong values, labels and citations

24. **1 (Inspection service row): "state up to 30 Hz, diag up to 4 Hz" [code inspection_service.cpp:55-67].**
    - State is 30 Hz by default and clients may ask for up to 60 (kMaxStateHz).
    - Diag is 4 Hz by default, max 20 [code runtime/inspection_config.h:47-48, 69-70].
    - The cited lines hold only the priorities, the 5 ms feed poll and the 20 Hz telemetry cap.
    Fix: "state 30 Hz default (clients up to 60), diag 4 Hz default (max 20), telemetry at most
    20 Hz [code nav:debug/inspection/inspection_service.cpp:65-67; nav:debug/inspection/inspection_config.h:47-48, 69-70]
    (was src/runtime/inspection_config.h)".

25. **4.4 rows 7-8; section 6 "Browser idle frame".**
    - "2.1 / p95 5.1" is cited to viz final_report s2, which has no such row (it reports frame
      interval 8.0 / 8.3 / 24.1).
    - The value comes from final-ab-report.md via disc:latency-chain s1 row 18.
    - Row 7 "0.2-0.4" also takes its 0.18 low end from latency-chain row 17.
    Fix: cite [measured-host disc:latency-chain s1 rows 17-18]. Write row 7 as
    "p50 0.18-0.39, p95 0.28-0.51".

26. **4.4 "RS-485 + radio ... about 48-51 ms mean / 69-73 ms p95".** chain_model gives 47.8 / 68.9
    for a USB Brain link and 51.4 / 72.8 for RS-485 [disc:latency-chain s2]. The RS-485 path is
    the second row only.
    Fix: "about 51 ms mean / 73 ms p95 (hop assumed 10-50 ms) [estimate disc:latency-chain s2]".

27. **4.4 "The rejected relay shapes ... 32-37 ms mean".** latprobe gives 31.7-37.9
    [disc:latency-chain s2]. The comparison also sets a measurement at the Pi drain against an
    estimate at the receiver.
    Fix: "32-38 ms mean, 41-60 ms p95 at the Pi drain [measured-host]. The SG1 path at the Pi drain
    is about 24 ms mean [estimate: 15.4 + terms 2-5]."

28. **4.3 "Separate viz op" row; section 6 "Separate exchange at 5/10/20 Hz"; section 0.**
    - One mechanism appears twice, with numbers from different rigs.
    - Fakes understate the cost about 2x against the real Pi loop [disc:evidence-sim s6].
    - Section 6 quotes only the best fake row.
    - At 20 Hz, usb2 is -23.6% [disc:evidence-sim s3], so "24-26%" is rounded.
    Fix: merge the rows into: "Any extra request/reply exchange at 10 / 20 Hz: -12 to -14% /
    -23.6 to -25.9% polls with the real Pi loop [measured-host disc:evidence-sim s3]. The fakes
    show -8 to -16% / -18 to -29% [disc:wire-budget s3a]."

29. **Conventions, "the restructure is a pure move".** It is not a pure move.
    - Step 2 splits system.cpp.
    - Step 3 edits world estimation and inspection_document.cpp (heading_error_deg / displacement_m
      from LandmarkError) [restructure/SPEC.md "Behavior rule", "OUTPUT CHANGE";
      restructure/plan/world_estimation_proto.diff].
    - Moved files get include-path edits and clang-format.
    - So refs such as inspection_document.cpp:807-816 and 1188-1253 can shift.
    Fix: "Line numbers are from ae457ee; read them with `git show ae457ee:<old path>` (Appendix B).
    The restructure also splits system.cpp and edits world estimation and inspection_document.cpp,
    so new-layout line numbers can differ." Optionally replace the `.../` placeholders with the
    move-map names, marked "pending restructure D1/D2" (restructure/plan/move_map.md):
    resources/resources/, commands/collectors/, publishing/publishers/, localization/observations/.

30. **Basis-tag misuse.** The conventions define `disc:` for [measured-host] only, and [code] as
    path:line. These entries break that:
    - `[code disc:...]`: section 1 rows Viewer, Inspection server, Capture, Brain link and Pico
      diag; W2; A12; A15; A16; A17.
    - `[source disc:...]`: 4.1 step 1; 7.3; A13.
    Fix: cite the underlying refs.
    - Inspection server window: [code nav:debug/inspection/http_server.h:92] (64 KiB). pi-side s4
      does not state it.
    - 4.1 step 1: [source pyserial serialutil.py:210-211; pros-cli direct_port.py:14-17].
    - A12: [code grep "Origin": no hit in http_server.cpp, inspection_service.cpp, capture routes].
    - A13: [source xnu bsd/netinet/tcp.h, bsd/sys/socket.h URLs from disc:pi-side s4.2].
    - A15: [code pi/naviGATR/docs/usb_localization_bench.md:96-97].
    - A16: [code field_scene.js:1388-1394; graphs.js:10-15].
    - A17: [code .github/workflows/ci.yaml:86-88], plus [measured-host `ls pi`] for the case part.
    - 7.3: the source URLs listed in disc:xplat-host s3.1.
    Alternatively, define `[code via disc:<file> sN]` in Conventions.

31. **Missing basis labels.** Add the listed label to each:
    - 1, PATH_REPORT row: "[code; ...]" needs [code link_driver.cpp:158-183; client.cpp:229-241].
    - 4.1 identity table, controller "SYS_STATUS system_id": [inference; B9]. That the controller
      relays the Brain's status is inferred from pros-cli's wireless upload [disc:controller-host s2.1].
    - 4.1 "pros-cli is MPL-2.0, and vexide is MIT but its decoder stops at 0x00": [code pros-cli
      LICENSE:1; vex-v5-serial Cargo.toml; vex-cdc cdc2/controller.rs:53-85 via disc:controller-host s3, s8].
    - 4.1 step 6: S11 covers the revert after unplug. Cite the dropout to [source S12] and
      [code cargo-v5 connection.rs:104-199].
    - 4.4 "with fixes" column (0-0.5 wake, about 0 reader, about 0 relay): [estimate]. The
      Windows condvar figure is not a PROS or Pi figure.
    - 7.1 LOC for W0, W4, W5, W6, W7 and W9: [estimate]. W2 uses [code wc -l, disc:pi-side s4.1].
    - 4.2 "If the FIFO is split between readers, both ... corrupt": [inference, H2 in
      disc:controller-host s6].

32. **F7 "pros_usb_link is the sole tty owner".** Nothing enforces it: the port is opened without
    TIOCEXCL or flock, and only a doc warning keeps other readers off [code pros_usb_link.cpp:149;
    pi/naviGATR/docs/usb_localization_bench.md:28-30; disc:pi-side s1.1].
    Fix: "sole owner by convention (no TIOCEXCL/flock); a second reader such as pros terminal would
    split bytes".

33. **5.4 Pose row, "Status is mapped as in LinkDriver::robot".** LinkDriver yields 6 states
    (kNoLink, kNoProfile, kCalibrating, kUnplaced, kNoPose, kValid) [code link_driver.cpp:54-93].
    PoseStatus has 5, and kNoProfile has no target.
    Fix: add kNoProfile, or state the mapping (for example kNoLink and kNoProfile -> kNoSource).

34. **4.4 "An app loop adds 0-20 ms: a held pose republished on a free 30 Hz grid measured 26.1".**
    A 30 Hz grid adds 0-33 ms; 0-20 ms applies to the 20 ms app loops [disc:latency-chain s1 row 14].
    Fix: "An app loop adds up to one loop period (0-20 ms for the 20 ms loops). The probe's 30 Hz
    grid measured 26.1 against 15.4 ms mean."

35. **4.4 row 1 wording, "Pi measurement".** latprobe's true age runs from the Pico sample time
    (heading encodes it) [disc:latency-chain s2].
    Fix: rename the span to "sensor sample (Pico) -> ...", or define "Pi measurement" once as the
    Pico sample time.

36. **Sources.** Several cited items are missing: v5x PR #329, vexforum 144920 (500 B/s), pyserial
    serialutil.py (DTR/RTS default), Rust std vexos.rs, cargo-v5 connection.rs, and PROS
    stream_buffer.c. The numbering also skips S5, S6, S9, S10 and S17-S22.
    Fix: add the used sources and either renumber or state "gaps are unused numbers".

## Refs opened and matched (sample)

- Brain:
  - pros_usb_port.cpp:22, 32-59, 194-198;
  - client.cpp:248-264, 358-381, 446-454, 616-645, 673-687, 716-731, 840-846, 1033-1044,
    1062-1067, 1102-1148, 1150-1156, 1214-1221, 1296-1315;
  - client.h:47, 59, 67-68;
  - link_driver.cpp:54-93, 89, 158-183, 215-229;
  - pros_link.cpp:40-46, 84-91, 135-137, 391-404;
  - motion.cpp:172-174, 229-238, 315-327, 435-441, 474-477;
  - pros_drive.cpp:100-110;
  - state_source.h:1-4; geometry.h:2-3, 11-15, 33-34; attitude.h:2-13;
  - gatr2_robot.h:49-58; gatr2_brain.mk:21-22; brain/README.md:25-26;
  - operaGATR main.cpp:142-147; locaGATR main.cpp:75-91.
- translaGATR: frame_codec.cpp:555-561; frames.h:84-97, 268-269, 353-369;
  link_documents.h:154-174, 227-247.
- naviGATR (ae457ee paths):
  - pros_usb_link.cpp:180-207, 216-268;
  - brain_link_commands.cpp:220, 280-299, 320-328, 365-372, 532-554;
  - inspection_service.cpp:55-67; inspection_document.cpp:807-816, 1188;
  - brain_profile_builder.cpp:537-538, 605-610; brain_imu_wheel_bench.cpp:204-224;
  - clock_sync.h:1-20; http_server.h:89-92; http_server.cpp:124-128; link_monitor.h:49;
  - capture_bundle.cpp:186-191;
  - brain_link_e2e_gtest.cpp:53-57, 136-228.
- Viewer and tools: smoothing.js:1-34; field_scene.js:2-4, 1426-1427; replay.js:554;
  latency.js:16; capture_fixture.py:24-36.
- Docs and CI:
  - docs/communigatr.md:203-209, 705-709, 764-788; docs/brain_setup.md:325-328;
  - docs/investigatr.md:28-30; pi/naviGATR/docs/coordinates.md:11, 18-20;
  - usb_localization_bench.md:96; ci.yaml:86-88.
- PROS 4.2.2 ser_driver.c: 28, 65, 88, 110-112, 117-121, 159, 181-188, 221, 233-238, 289-290,
  313-318.
