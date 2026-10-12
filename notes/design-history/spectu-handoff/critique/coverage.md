# Critique: spec coverage of PROPOSAL.md

2026-09-28. Lens: spec coverage. I read these in full: requirements.md, PROPOSAL.md, all ten discovery files,
viz-handoff/final_report.md, restructure/SPEC.md and the four memory files named in the task. No repo edits.

Citations:
- R:n is spectu-handoff/requirements.md line n. P:n is spectu-handoff/PROPOSAL.md line n.
- disc:<file> sN is discovery/<file>.md section N.
- Other tags are as in the proposal: [code], [source], [measured-host], [estimate], [inference], [unknown].

## 1. Constraint checks

| Constraint | Verdict | Basis |
|---|---|---|
| No implementation before approval (R:8, R:405) | Met. The proposal is design only, and all W work comes after approval | P:3, P:844-845 |
| No PCB, flash, deploy, commit or push (R:12) | Weak. Nobody is named to flash the B1-B12 bench programs or deploy Pi builds, and W9 adds a release workflow without ruling out tags or pushes | P:879-880, P:907, P:842. Change 7 |
| Pi-authoritative pose never silently restored (R:75, R:209) | Mostly met: rules 2-3, D5, D6. The risk is that the 9 old scenarios stay "on the compat path", and their checks read the Pi pose as the drawn robot | P:757-761, P:873; [code] disc:viewer s6 (data-pose, data-replay-pose hooks). Change 6 |
| Latency conflict stated explicitly (R:209) | Met, with a wording risk. "With the fixes +1 / +4 ms" can read as both goals met, but one fix is gated by D10 and all the numbers are estimates | P:34-40, P:386-397, P:395, P:924. Change 9 |

## 2. Requirement walk

OK means covered, WEAK means partial or only implied, MISSING means not addressed. The last column gives the change number in section 3.

| Area | Requirement | Proposal | Status | Chg |
|---|---|---|---|---|
| Product | Four parts (R:17-22) | P:8-12 | OK | |
| Product | spectaGATR migrates, no needless rewrite (R:24) | P:86, M0 then M1-M6 | OK | |
| Product | Folder layout and boundaries (R:26-45) | P:102-112, 7.1 | OK (adds examples/, cmake/, packaging/) | |
| Ownership | Brain owns pose, field, paths, execution and attitude (R:49-55) | P:14, 5.4 | OK | |
| Ownership | Same API whatever the pose source (R:57) | P:472, 5.1 | OK | |
| Ownership | Pi instrumentation list, link continues (R:59-68) | P:89, P:93-95 | OK | |
| Ownership | Receiver modes 1-3 (R:70-73) | Rules 2 and 9, D5 | WEAK: no per-mode table | 17 |
| Ownership | Brain disconnect: no source switch, stale marked (R:75) | Rule 3 | OK, but the thresholds conflict | 2 |
| Ownership | Keep the Brain's accepted field revision (R:77) | P:443-455, rule 7 | OK | |
| API | One instance in the Brain process, references passed (R:81) | 5.1, 5.4 | WEAK: construction site not stated; nothing stops two transport owners | 19 |
| API | Descriptive methods, no coordinator (R:83-90) | P:625-639, P:119-120 | OK | |
| API | The Pi has its own publisher instance (R:92) | The adapter passes inspect/2 through | WEAK | 18 |
| API | Standalone use needs no other GATR library (R:94) | P:135-138, P:712 | OK; how it builds is vague | 22 |
| API | Optional integrations at the real owners (R:96-101) | 5.4 | OK | |
| API | Path semantics: turns, replacements, clears, cancellation (R:103) | P:572-595 | WEAK: no replaced state; multi-chunk assembly undefined | 11 |
| API | Never issue SET_POSE or motor commands (R:105) | Not stated | MISSING | 19 |
| API | Aliases, including for callback types (R:107) | P:489-503 | WEAK: hooks and clock have no alias or signature | 19 |
| API | updatePose overloads, units, yaw, heading-only (R:109-115) | P:625-627, 5.2 | OK | |
| Contract | Versioned contract contents (R:117-125) | 5.5 | OK for the Brain wire; receiver->browser contract undefined | 3 |
| Contract | Named channels, no viewer edit to add one (R:127) | CHANNEL_DEF/SAMPLES, M3 | WEAK: Brain only | 18 |
| Contract | No values forced into pose; no schema framework (R:129-130) | 10 fixed kinds | OK | |
| Contract | Absent capabilities not shown as broken (R:132) | Rule 9 | WEAK: capability bits undefined | 19 |
| Q1 | Radio -> controller -> receiver route; direct USB as an extra (R:136-145) | 3.2, 4.1 | OK (two claims untagged) | 27 |
| Q1 | Verify PROS and controller behavior from primary sources (R:147) | F1-F8, 4.1, S1-S23 | OK | |
| Q2 | Stream ownership, stdin, COBS, shared buffers (R:155-157) | F1-F5, 4.2 | OK (routing is B1) | |
| Q2 | No competing readers; single owner with mux (R:159-161) | 4.2 table | OK; single-writer enforcement missing | 19 |
| Q2 | Pi forwarding is explicit transport, and the Brain stays the producer (R:163-167) | 3.3, P:141 | WEAK: what is forwarded is unclear; "outside the drain" is wrong | 4, 10 |
| Q2 | Three topologies, demonstrated vs untested (R:169-174) | 3.2-3.4 | OK; RS-485 alternative not evaluated | 20 |
| Q3 | New states only, no fixed 30 Hz (R:178-180) | P:347-349 | OK | |
| Q3 | The six investigation items (R:182-188) | 4.3 | OK | |
| Q3 | Localization first; callbacks never block (R:190-192) | P:359-364 | OK; the stdio no-reader stall is open | 21 |
| Q3 | Reproduce the tests against the proposed design (R:194) | Deferred to 7.2 | WEAK | 8 |
| Q4 | Inherent delay vs queueing, serialization, radio, browser (R:198-202) | 4.4 | WEAK: no serialization terms | 9 |
| Q4 | No hiding; original stamps and identity kept (R:204-205) | P:426-433 | OK; clock chain for relayed data undefined | 4 |
| Q4 | Budget before implementation; conflict explicit (R:207-209) | P:416-424, P:386-397 | OK, apart from the wording | 9 |
| Q5 | Brain field, standalone complete, no resend per pose (R:213-217) | P:462-472 | OK (planning layer only; D14) | |
| Q5 | Cache or reference with validation, no Pi substitution; a recommendation (R:219-226) | P:456-471 | OK (the display layer is validated only through the planning map, as disclosed at P:468-469) | 24 |
| Sources | Simultaneous sources (R:230) | 5.6, P:787 | OK | |
| Sources | Explicit identities (R:232-237) | P:742-751 | WEAK: no Brain<->Pi robot match; placement tuple sent only as samples | 5 |
| Sources | First arrival, missing, restart, stale, field change, reconnect (R:239) | Rules 1-3, 6, 7, 9 | WEAK: first arrival and port-level reconnect not covered | 14, 17 |
| Sources | No arrival-time merge, no unmapped clocks, no mixed placements (R:241-243) | Rules 4-6 | OK | |
| Sources | Final commands; one command also opens the browser (R:245-251) | 5.7 | OK | |
| Xplat | CMake receiver and host tests (R:255) | W2, W3, W5 | WEAK: no root CMake target named | 14 |
| Xplat | Permissions, reconnect, paths, browser launch (R:259-264) | P:793, P:892 only | MISSING | 14 |
| Xplat | Packaging; prebuilt without a compiler (R:265-267) | 7.3 | OK | |
| Xplat | PROS build workflow kept (R:269) | P:837, P:896 | WEAK | 22 |
| Xplat | One browser app, offline, no forks (R:271) | P:894, W1 | OK | |
| Xplat | State what was built and tested (R:273) | 7.4 | WEAK: 7.4 is a plan, not a record | 15 |
| Preserve | Inventory before changing (R:277) | Section 1 | WEAK: single-source assumptions not carried over | 16 |
| Preserve | Eight retained items, generic vs adapter split (R:279-289) | Scattered | WEAK | 16 |
| Preserve | Camera minimal, no invented calibration (R:291) | Not stated | MISSING | 16 |
| Perf | Six measurements (R:295-301) | P:689, 4.4 | WEAK: no serialization time, queue depth or source RTT | 12 |
| Perf | Replace unsent live state without partial frames (R:305) | P:350-353 | WEAK: multi-chunk replacement undefined | 11 |
| Perf | Durable events kept apart from live state (R:306) | EVENT FIFO at P:735 | WEAK: the Pi relay ring drops any line | 4 |
| Perf | No repeated configuration or DOM rebuilds (R:308) | Carousel at P:352-353 | WEAK: the receiver does not dedupe | 13 |
| Perf | Smoothing rules (R:310) | P:432-433 | OK | |
| Record | Rolling history, pre/post trigger, fault triggers, loss records (R:314-318) | One cell at P:90 | MISSING | 1 |
| Record | Receiver capture holds only what it received (R:320) | Not stated | MISSING | 1 |
| Record | Pi capture kept through the adapter (R:322) | P:90 | WEAK | 1 |
| Record | Export bundle contents (R:324-329) | Not stated | MISSING | 1 |
| Record | Replay syncs field revisions and discontinuities (R:331) | P:841; A11 unowned (P:946) | MISSING | 1 |
| Record | Replay never controls the robot (R:333) | Not stated | MISSING | 1 |
| Validate | Workstreams, contracts and ownership first, serial integration (R:337-343) | 7.1 | WEAK: view contract and docs unowned | 3, 26 |
| Validate | locaGATR and operaGATR checked; standalone examples (R:345-351) | P:710-712 | OK ([code] both folders exist at ae457ee) | |
| Validate | Ten test areas (R:353-363) | 7.2 | WEAK | 25 |
| Audit | After validation, report only (R:365-391) | P:844-845 | OK | |
| Deliverable | Eight required parts (R:395-403) | Sections 1-8 | OK | |

## 3. Changes, most severe first

**1. Add 5.8 "Receiver capture, export and replay" (R:312-333).**
- Problem: recording gets one table cell (P:90) and a line count (P:838). A11 lists replay defects but gives no owner (P:946).
- Fix: add a section that states the following.
  - **Mechanics.** The receiver reuses the Pi recorder mechanics [code disc:pi-side s5]: configurable pre_s and post_s; record, memory, duration and bundle bounds; one capture at a time.
  - **Fault triggers**, each with a cooldown and an hourly cap: Brain source lost, Brain session restart, a burst of sequence gaps, a LOSS spike, Pi source lost.
  - **Received data only.** Rows are the frames actually received, never resampled. metadata.json gives per source the frames received, the sequence gaps and the publisher LOSS counts (updates vs replaced), so a radio capture never claims native-rate completeness (R:320).
  - **Bundle `spectugatr.capture/1`:**
    - one CSV per (source, stream), on rx_us plus src, src_session, src_clock, src_t_ms, seq, placement, field_id and revision;
    - metadata.json holding the configuration, contract versions, channel and event definitions, units and frame conventions (5.2), sources[] with the clock mapping per segment, and loss;
    - `fields/<field_id>@<rev>` documents plus field_revisions.csv [inference disc:viewer s7].
  - **Pi side.** Pi capture stays in naviGATR as gatr2.capture/1 with the Pi triggers. A combined receiver capture records the forwarded Pi messages as a `navigatr` source.
  - **Replay** rebuilds the scene with the field revision in force at the playhead, splits trails and graphs at session and placement changes, and honours new_measurement. That fixes A11; assign it to W8.
  - **Read-only.** `--replay` opens no device, and no source ever writes to a Brain, including brain-usb (today only the controller case is stated, at P:263).

**2. 4.3 rates, the 5.5 SESSION row, 5.6 rule 3: the heartbeat and the stale threshold conflict.**
- Problem 1: the radio heartbeat is every 5 s (P:343, P:728), but rule 3 declares a Brain lost after 1 s without a frame (P:760). A stationary robot whose code publishes only new states therefore shows "disconnected" on the radio. On USB the heartbeat period equals the threshold.
- Problem 2: the viewer marks a pose stale when its age plus the time since receipt exceeds 250 ms [code pi/naviGATR/spectaGATR/smoothing.js:19, 33]. At a 3-5 Hz radio pose rate the gaps are 200-333 ms [estimate], so the pose and attitude flicker stale.
- Fix: derive the thresholds per source from the heartbeat and pose cap declared in SESSION:
  - stale after 2.5 heartbeat periods;
  - lost when the port is gone or after 5 heartbeat periods;
  - pose fresh while its age plus the time since receipt stays under max(250 ms, 1.5 x the declared pose period).
- Also send a minimal liveness frame (header only: 13 B, 31 characters) at 1 Hz on every transport. That is 8% of the 400 B/s radio budget [estimate]. The full SESSION/CONTEXT heartbeat keeps its slower rate.

**3. Add 5.9 "Receiver -> browser contract", owned by W0.**
- Problem: `spectugatr.view/1` appears only as a W7 file note (P:840). W5, W7 and W8 all depend on it, yet P:846 says shared contracts change only through W0, and R:20 lists shared contracts as a product part.
- Fix: define the contract, following the draft in disc:viewer s4.1.
  - **Messages:**
    - hello, carrying sources[] and a viewer build hash;
    - source (identity, state, capabilities, clock mapping, transport stats);
    - pose, context, field, path, channels, samples, event and loss;
    - Pi passthrough messages with `src` added.
  - **Delivery:**
    - reliable FIFO for hello, source, field, channels, event and path end states;
    - one replaceable slot per (src, kind) for pose, samples and context.
  - **Fields and client messages:** `src` and `rx_us` on every message; subscribe and preview from the client carry `src`.
  - **inspect/2 compatibility** as in change 6.
  - **Ownership:** W0 owns `spectuGATR/protocol/VIEW.md` and freezes it together with the Brain wire.

**4. 3.3 steps 2-3, the 4.2 Pi row and rule 5: what the relay forwards, event safety, and the clock chain.**
- Problem 1: the proposal never says whether the Pi forwards raw SG1 frames or decoded messages (P:184, P:840). If the Pi re-encodes, the laptop cannot check CRC or sequence, and the Brain is no longer the byte producer (R:165-166, R:205).
- Problem 2: the relay ring is 64 lines, dropping the oldest of any line (P:297). Because a frame packs mixed records (P:687), a durable EVENT or a DOC_CHUNK can be lost while newer poses survive (R:306).
- Problem 3: rule 5 covers direct serial and Pi sources only.
- Fix:
  - The Pi forwards the exact SG1 frame bytes plus its arrival time, labeled as a relay stamp. The laptop decodes them with the W0 codec, and the Pi-hosted viewer uses the same view code.
  - The Brain marks frames that carry EVENT, DOC_CHUNK or PATH records, with a flags byte in the frame header or a second line marker. pros_usb_link queues those frames separately, and that queue never drops without a loss record.
  - Add rule 5c: a relayed Brain maps Brain -> Pi through a one-way minimum offset at Pi arrival, then Pi -> receiver through ping, and the bounds add.

**5. 5.5 identities, rules 1 and 6, D6: robot and link identity across sources.**
- Problem 1: robot_id is crc32 of the name (P:728, P:746). Nothing checks that a controller Brain and `navigatr=pi.local` (P:207, P:787) are the same robot.
- Problem 2: the placement tuple travels only as sampled channels (P:697), yet D6 compares tuples (P:920), and samples are latest-wins (P:350).
- Both sides already know the link identity: the Pi assigns the session and pi_instance [code disc:pi-side s2.1: brain_link_commands.cpp:158-168, 747-785], and the Brain stores them [code client.cpp:446-448].
- Fix:
  - Add an optional CONTEXT group `link_ref {pi_instance u32, link_session u32, odometry_epoch u32, anchor_revision u32}`, 16 B, sent on change and filled by the integration. The Pi adapter sends the same four values.
  - Add rule 11: combine a Pi source and a Brain source only when pi_instance and link_session match; otherwise show "different robot link" and keep them apart.
  - Drop the `link.*` channels.

**6. 7.2 compat path, M2, D5/D6: no Pi robot through legacy sources.**
- Problem: the compat path treats an inspect/2 server as one navigatr source [inference disc:viewer s4.1]. Its 9 scenarios assert the drawn robot from Pi `state.robot` (P:873). If they pass unchanged after M2, the Pi pose is still drawn as the robot, against rule 2 (P:757-759) and R:73.
- Fix: state that M0-M1 keep today's behavior. From M2 on, a legacy inspect/2 source or a gatr2.capture/1 replay draws no core robot, only the labeled "Pi estimate" style if D6 is accepted. List which scenario checks change, and add a check that no core robot is drawn for a legacy source.

**7. 7.2 bench, 7.3, 7.4, W9: no flashing, deploying or commits.**
- Problem: B1-B12 need programs flashed to the Brain and Pi deploys (P:879-880, P:907), and W9 adds release.yaml (P:842). R:12 forbids flashing, deploying, commits and pushes.
- Fix: add: "Agents build only. The bench programs, scripts and expected-output sheets are prepared uncommitted. The user flashes, deploys, runs B1-B12 and returns the logs. There is no commit, push, tag or release; release.yaml is written and never triggered."

**8. 4.3 and 6: reproduce the tests against the design before approval (R:194).**
- Problem: the one-way lane's link cost is asserted (P:31-32, P:330), shown only functionally (P:191-195), and its test is deferred (P:866-871).
- Today's pros_usb_link already reads, parses and drops non-NG1 lines [code disc:pi-side s1.2: pros_usb_link.cpp:216-253]. So the Pi-side cost of SG1 traffic can be measured now, without any implementation.
- Fix: rerun the evidence-sim e2e_sweep in scratch with synthetic SG1 lines on the Brain->Pi wire at the USB budgets in 4.3:
  - 87-character pose lines at 25 Hz, sample frames at 10 Hz and 512 B field bursts;
  - usb2, usb3 and usb5, 5 phases x 20 s each.
- Report polls/s, poll gap p95 and max, RTT, and the request delay caused by the bursts, with spectu off and on. Tag the result [measured-host], note that the model has no USB character time [disc:evidence-sim s2], and add the row to section 6.

**9. 4.4 and the summary: serialization terms and the conflict wording.**
- Problem 1: R:202 names serialization, but 4.4 has no row for the Brain encode, the character time of an 87-250 character line, the Pi split, or receiver decode plus JSON.
- Problem 2: P:395 reads as "both goals met", yet the arrival reader waits for D10 and for B7/B8 (P:924).
- Fix:
  - Add rows:
    - Brain encode [unknown; time the W4 host prototype, then B12 on hardware];
    - line character time [unknown B7];
    - Pi split and parse [unknown, Pi bench];
    - receiver decode and JSON: server plus WebSocket measured at 0.39 ms p50 [measured-host viz final_report s2]; decode [unknown].
  - Replace P:395 with: "Phase 1 (wake plus event relay) is estimated at about +6 ms mean over the like-for-like Pi path (sum of the 4.4 terms). The arrival reader would bring that to +1 / +4 ms, but it changes the localization link resource and waits for D10. The conflict stands until B7/B8."
  - Note that the ~25 ms reference is a Windows run, and ~16-18 ms is inferred for Linux [inference disc:latency-chain s6], so the relative cost may be larger on the Pi.

**10. 4.3 "Localization has priority": correct P:363.**
- Problem: P:363 says the Pi split happens "outside the request drain". In fact the only reader is the commands-stage drain on the estimation worker [code disc:pi-side s1.1: brain_link_commands.cpp:288-346]. Non-NG1 lines use its 4 x 256 B read budget [code pros_usb_link.cpp:258-268], and 1539 B of them push a request to the next read call [measured-host disc:pi-side s1.4].
- Fix: say that the split happens inside pros_usb_link during that drain, before the FrameReader, so SG1 never reaches the commands. Its parse time is spent on the estimation worker, and the 512 B burst cap keeps a request inside one read call. The D10 reader moves the split off that worker. Add the Pi parse cost to B7.

**11. 5.1 PathUpdate and 5.5 PATH/DOC_CHUNK: replacement and chunk assembly.**
- Problem 1: PathState has no replaced state (P:584), although R:103 names replacements.
- Problem 2: on stdio a frame over 6 segments does not fit: 13 + 26 + 12 x 7 = 123 B > 122 B (P:720) [estimate]. Tank-avoiding plans reach p90 9 and max 13 segments [measured-host disc:wire-budget s3b]. Nothing keeps chunks of rev k apart from rev k+1 (R:305).
- Fix:
  - Add `kReplaced`, sent for the old command when the new command's first plan is accepted. A newer command id also supersedes older paths in the receiver.
  - PATH assembles by (session, command, rev) and is drawn only when complete. An incomplete rev is dropped when a newer rev or state arrives, and the publisher finishes or abandons a rev as a unit, counting an abandon as replaced.
  - DOC_CHUNK assembles by (doc_id, total) and is used only when its crc32 equals doc_id.

**12. 5.3 stats: add a "Performance instrumentation" table covering R:295-301.**
- Problem: the stats list (P:689) has no generation or serialization time, queue depth, relay-queue depth, controller round trip, empty-poll ratio, radio link quality, or per-source receiver bytes/s.
- Fix: add a table of metric, where it is measured, and how it is exported:

  | Metric | Measured and exported |
  |---|---|
  | Source age | Receiver, through the mapping |
  | nextLine time and write duration | Publisher, sent in LOSS/SESSION |
  | Size and rate per kind | Publisher and receiver |
  | Depth, age, drops, replacements | Publisher slots and event FIFO, Pi relay queues, HttpServer per client (exists) |
  | Round trip | USER_READ (controller) and WebSocket ping (exists) |
  | Browser frame and handler time | metrics() and cdp_bench (exist) |
  | Controller RADIO_STATUS | At 0.2-1 Hz [disc:controller-host s9b step 4] |

- Show these per source in the latency and diagnostics panels.

**13. 4.3 carousel and 5.6: dedupe at the receiver (R:308).**
- Problem: definitions and documents repeat on the carousel (P:352-353, P:732-736), and nothing stops the repeats reaching the browser.
- Fix: add rule 12: "The receiver keeps definitions and documents by id. A repeat with a known id and crc only updates last-seen and is not forwarded. The browser rebuilds only on a new id or revision." Add a browser check that the DOM node count stays flat and buildField is not called across 60 s of carousel.

**14. Add 7.5 "Receiver platform behavior" (R:255-265).**
- Problem: permissions, reconnect, paths and browser launch are not covered. The only related text is the auto-open at P:793 and the udev rule at P:892.
- Fix: add a table built from disc:xplat-host s2.1-2.7:

  | Concern | Behavior |
  |---|---|
  | Permissions | Linux EACCES: dialout/uucp hint, or the shipped uaccess rule. Windows ERROR_ACCESS_DENIED: "in use by PROS terminal/VEXcode". macOS: errno text |
  | Exclusive open | Share mode 0 on Windows; TIOCEXCL on POSIX |
  | Reconnect | 1 Hz rediscovery by identity plus role, a new transport epoch, and restarts decided by the protocol session. Controller radio loss is a protocol timeout, not a port loss |
  | Paths | Config in RoamingAppData, Application Support or XDG_CONFIG_HOME. Captures stay in memory by default, never in Documents (OneDrive-redirected here [measured-host disc:xplat-host s1.3]); `--capture-dir` saves them |
  | Browser | ShellExecuteW, `open`, or xdg-open (xdg-open only with DISPLAY or WAYLAND_DISPLAY, never as root). Always print the URL |
  | Waits | On I/O or events, never on sleep_for |

- Also name the build root: `spectuGATR/CMakeLists.txt`, with SPECTUGATR_BUILD_HOST and SPECTUGATR_BUILD_TESTS.

**15. 7.4: separate what was built from what is planned (R:273).**
- Problem: "Windows 11 x64 (this laptop) | yes" (P:903) reads as a done fact.
- Fix: add a "Built and run during discovery" block:
  - Windows 11 x64 only: the server alone, 23/23 server tests, and a static 1.40 MB exe [measured-host disc:pi-side s4.1, disc:xplat-host s1.3];
  - macOS and Linux were not built;
  - no VEX device was present (0 ports enumerated).
- Label the rest of the table "Planned".

**16. Add 7.6 "Retain and finish" (R:277-291).**
- Problem:
  - The 13-row single-source assumption table in disc:viewer s2 shrinks to "Single source" (P:86).
  - The open items in final_report s4 and in A11 have no owner.
  - Camera handling is never mentioned.
- Fix: add one row per item in R:280-287, giving kept / changed / gap finished / owner:
  - low-latency delivery (W2);
  - smoothness and perf tooling (W8);
  - field, robot, path and trail (M2, D7);
  - diagnostics (M4 plus change 12);
  - graphs (M3);
  - attitude honesty: the measured, stale, unavailable and assumed_level states are kept; thresholds per change 2;
  - camera:
    - the preview path is unchanged, and preview frames only gain `src`;
    - no field correction or calibration is added, and a missing camera stays a warning;
    - mounts are drawn on the Brain robot only when the running profile ids match [inference disc:viewer s4.3];
  - capture, export and replay (change 1).
- Copy the disc:viewer s2 table into section 1 as the M1-M6 checklist.

**17. 5.6: modes and arrival order (R:70-73, R:239).**
- Fix: add the per-mode table from disc:viewer s4.3, plus this rule: "Sources may arrive in either order. Pi first: Pi panels live, core shows 'waiting for Brain source'. Brain first: core live, Pi panels hidden until the Pi declares them."

**18. Section 2, 5.5, W7: a Pi-side publisher (R:92, R:127).**
- Problem: Pi graphs stay on LiveCollector [code disc:viewer s3], so a new Pi measurement still needs viewer code.
- Fix: W7 gives the adapter a small publisher with the same channel and event semantics as 5.1, emitting view JSON. It routes Pi numeric instrumentation through channel definitions plus samples, at least for new measurements; the inspect/2 panels stay.

**19. 5.1 API: close the rule gaps.**
- (a) R:105: add: "No publisher or integration call issues SET_POSE, CONTROL or motor output. The core has no communiGATR or actuGATR dependency. Integrations are observers and hold no ProsLink or ProsDrive pointer. newPlacement() only marks a display discontinuity."
- (b) Single writer: nothing stops an app calling both startStdioTransport and usbLane (P:651-656). Make the first transport owner win; a second attach returns false and is counted.
- (c) R:107: alias the callbacks and give their signatures:
  - `using ClockFunction = Seconds (*)();` (P:617);
  - `AuxLineSource`, with write-duration feedback, which the 5 ms rule needs (P:357);
  - `communigatr::StateObserver`, `actugatr::PathObserver` and `actugatr::DriveObserver`.
- (d) Define `PublisherStats` (P:643) and the SESSION capability bits: pose, attitude, field, estimate, path, execution, channels, events; for the Pi also camera, pico, link and capture. Without those bits rule 9 is untestable.
- (e) State the construction site: one static Publisher created in initialize(), before createLink and createDrive [inference disc:brain-owners s6.5].

**20. 3.4: evaluate a second RS-485 topology (R:172).**
- Problem: 3.4 relies on the radio (B2) or on a tethered laptop. In RS-485 mode the Brain USB user port is a free, separate channel [code brain/communiGATR/pros/pros_link.cpp:84-91; inference disc:pros-serial s7].
- Fix: add option (c2):
  - a Brain USB cable to the Pi carrying only SG1 from the stdio task;
  - on the Pi, a viz-only resource that no pipeline stage reads;
  - viewing as in (b), with no interference on the link.
- Tradeoff: one more cable, with the same B5 and B7 unknowns. Add a D item asking the user to choose between (c) and (c2).

**21. Section 8: add the missing decisions.**
- (a) B5 (P:229): if nothing drains channel 1, a blocking PROS write waits forever while holding write_mtx, which stalls every printf [code/source disc:pros-serial s1.1]. The 5 ms rule cannot fire on a write that never returns. Add D17: "Until B5, the stdio transport is started only explicitly and is documented as unsafe with no host attached." Recommend that.
- (b) The prebuilt public packages need a license, and the repo has none [measured-host disc:controller-host s8]. Add D18 for the license, decided before W9.

**22. 7.3 and W4: how standalone users build it.**
- Problem: "ships as source through gatr2_brain.mk" (P:896) does not say how a standalone build works. The mk defaults to investigatr, communigatr and actugatr [code brain/gatr2_brain.mk:21-22], and it already compiles repo-root sources [code gatr2_brain.mk:48-49].
- Fix: state that:
  - `GATR2_BRAIN_LIBS := spectugatr` builds on its own;
  - the entry compiles spectuGATR/protocol and adds its include directory;
  - the example's Makefile uses exactly that line;
  - an outside team copies brain/spectuGATR and spectuGATR/protocol, or adds this repo as a submodule.

**23. Conventions (P:73-75) and Appendix B: the restructure is not a pure move.**
- Problem: SPEC step 3 changes the world_estimation and inspection internals: LandmarkError is added, and heading_error_deg reads from it [code restructure/SPEC.md, world_estimation]. The cited inspection_document lines (P:448, P:945) can shift, and the `.../` placeholders are not final paths.
- Fix: reword to: "Steps 1-2 are pure moves. Step 3 changes the world_estimation and inspection internals, with numerically identical results. W7 re-cites exact paths and line numbers once the restructure lands."

**24. 4.5, 5.1 and docs: follow the user's world-estimation model.**
- Problem: estimate output is an error from nominal, LandmarkError{dx, dy, dtheta}, and the goal is lining up relative to things [memory feedback_system_philosophy.md; SPEC]. The proposal shows only absolute estimate poses (P:559-565) and a resolved target (P:592-593).
- Fix (no wire change):
  - (a) The Brain landmark table shows dx, dy, dtheta = the accepted estimate minus the nominal pose of the same revision, where (0, 0, 0) means as built.
  - (b) Optionally, PathUpdate gains `ObjectId reference` (0 = field frame), taken from Motion's private reference [code motion.h:194 per disc:brain-owners s3.2].
  - (c) The docs lead with relative alignment, and treat Brain-vs-Pi absolute differences as diagnostics.

**25. 7.2: close the gaps in the test areas (R:353-363).**
- Add unit tests:
  - protocol: a truncated line (B11), an abandoned rev;
  - combination: link_ref mismatch, thresholds on a 3 Hz source;
  - Pi: overflow of the durable queue.
- Add browser scenarios:
  - `brainminimal`: a pose-only Brain declaring no field, path or attitude;
  - `roundtrip`: stub live -> capture -> zip -> replay, comparing exact values, the field revision at 3 playheads, and loss rows;
  - `radio3hz`: no false stale or lost state;
  - `relayoverflow`.

**26. 7.1 ownership: add the PIPELINE docs (the user's rule of one .md per pipeline).**
- Problem: only PROTOCOL.md is planned (P:833), and naming.md and the README indexes (P:479) have no owner.
- Fix:
  - W4 writes brain/spectuGATR/PIPELINE.md (producers -> slots -> scheduler -> transport owner).
  - W5 writes spectuGATR/host/PIPELINE.md (discover -> open -> source -> combine -> serve and capture) and spectuGATR/README.md.
  - W7 updates the nav:debug/inspection doc and the pros_usb_link doc.
  - W1 owns naming.md and the README indexes.

**27. Basis tags.**
- Problem: two claims are untagged:
  - P:166 "the user port carries the same lines";
  - P:247 the controller identity via SYS_STATUS, which is [inference] per disc:controller-host s2.1.
- Fix: tag P:166 [source S7, S8; unknown B1 while a controller also polls], and tag P:247 [inference; bench B9].
