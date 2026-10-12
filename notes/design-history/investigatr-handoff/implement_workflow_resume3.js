export const meta = {
  name: 'implement-investigatr-brain-link-v2-resume',
  description: 'Resume parallel implementation: brain link v3, Pi half-duplex/session integration, investiGATR, navigatr-interface, PROS app, e2e tests, docs',
  phases: [
    { title: 'Foundation', detail: 'common protocol, investiGATR, Pi transport, PROS scaffold (continuations)' },
    { title: 'Integration', detail: 'Pi brain_link core, navigatr client, navigatr driver' },
    { title: 'System', detail: 'e2e reboot tests, PROS app finish, Pi docs' },
    { title: 'Docs', detail: 'docs index, READMEs, CI, consistency' },
  ],
}

const S = 'C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/73859862-8cc3-4c8a-a408-13cc873c10fd/scratchpad'
const OLD_S = 'C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/ffabe616-f541-4ae8-9301-414e9aa793ac/scratchpad'

const RESULT = {
  type: 'object',
  properties: {
    summary: { type: 'string' },
    files_changed: { type: 'array', items: { type: 'string' } },
    public_api: { type: 'string', description: 'exact names/signatures/semantics downstream implementers need' },
    verification: { type: 'array', items: { type: 'object', properties: {
      what: { type: 'string' }, command: { type: 'string' }, outcome: { type: 'string' } }, required: ['what', 'outcome'] } },
    deviations_from_spec: { type: 'array', items: { type: 'string' } },
    open_issues: { type: 'array', items: { type: 'string' } },
  },
  required: ['summary', 'files_changed', 'verification', 'deviations_from_spec', 'open_issues'],
}

const COMMON = `You are one implementer in a coordinated, parallel build. FIRST read ${S}/spec.md (the design contract, v2) completely and ${S}/requirements.md (the user's verbatim request). Follow spec section 0 (repository rules) exactly: no git state changes, never touch pcb/ or pi/navigatr/docs/printables/ or .vscode/ (the user's IDE setting; if an IDE build folder such as brain/build appears, ignore it and never list it), do not edit files owned by the other Claude session (pico/**, tracking_wheel_motion.*, register_localization.cpp, its configs/tests/docs), terse plain-ASCII comments, style rules, explicit CMake test lists, tests/ folders hold only *_gtest.cpp files. Stay inside YOUR scope; other implementers edit other files at the same time. If you need something outside your scope, do not edit it: put it in open_issues. Read existing code before changing it and match its idiom. Decide within the spec without asking; record deviations. Keep it small and understandable. Iterate until builds and tests are green or you are genuinely blocked. Build directories only under ${S}. Never claim hardware validation. Final answer = the structured result; public_api must give downstream implementers exact names and semantics.`

const CONTINUE = `CONTINUATION: an earlier run of this same role was interrupted by a session restart (not because of a problem). Nothing it wrote was verified. Files it already wrote are in the tree. Read them first, keep what is right, fix what is wrong, and finish the task; do not start over.`

function up(label, r) {
  if (!r) return `\n[${label}: result unavailable; inspect the repository directly]`
  return `\n[${label} result]\nsummary: ${r.summary}\npublic_api: ${r.public_api || '(none)'}\ndeviations: ${JSON.stringify(r.deviations_from_spec)}\nopen_issues: ${JSON.stringify(r.open_issues)}\nfiles: ${JSON.stringify(r.files_changed)}`
}

const TOOL = `PROS toolchain dir: ~/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr/bin (arm-none-eabi-g++ 13.3, make, sh). PROS CLI: ~/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-cli-windows/pros.exe. Kernel template: ~/AppData/Roaming/PROS/templates/kernel@4.2.2.`

const RENAME = `RENAME AND RESTART NOTE: read spec section 0.2 first. brain/navigatr-interface is now brain/communiGATR (namespace communigatr; classes Client, Driver, ProsDriver; headers under include/communigatr/; CMake targets communigatr and communigatr_fakes; doc docs/communigatr.md; gatr2_brain.mk library name communigatr). Results quoted below were written before the rename and use the old names; translate them. The orchestrator already did the rename and re-verified the brain build. A previous launch of this role may have run briefly before it was stopped for the rename: if you find its edits, keep what is right. The user is writing their own barebones Brain test against communiGATR: never modify or delete their code (spec 0.2).`

const A1 = `${COMMON}
${CONTINUE}

YOUR ROLE: common protocol (spec section 2, 2.5, and the protocol-level parts of sections 3 and 4 for documentation).
SCOPE: common/frames.h, common/frame_codec.h, common/frame_codec.cpp, common/tests/*, docs/interfaces.md, docs/README.md (only the frames description).
TASKS:
1. Remove the pose (0x02) and command (0x03) frames, structs, enums, codecs, tests. Add brain link v3 exactly per section 2: envelope, crc16 CCITT-FALSE, request/reply structs, encode/decode, per-type len bounds, FrameReader support for 0x10/0x11 with CRC, rescan-on-rejection. Sensor frame bytes unchanged.
2. common/tests/frame_codec_gtest.cpp per 2.5 with independently checked known-byte vectors; frames_gtest.cpp meaningful; gtest FetchContent fallback in common/tests/CMakeLists.txt.
3. Build and run: cmake -S common/tests -B ${S}/build-common (spec section 0 recipe, FETCHCONTENT_SOURCE_DIR_GOOGLETEST). Compile-check frame_codec.cpp with arm-none-eabi-g++ -std=gnu++17 -c and -std=gnu++20 -c (outputs to ${S}). ${TOOL} Do NOT build navigatr (it breaks until the Pi implementer lands). Do not run pio.
4. docs/interfaces.md: replace pose/command sections with brain link v3 (envelope, CRC, ops, results, state block units/semantics, session rules from spec 4, bus ownership/timing from spec 3 with the budget formula, versioning); fix diagram and one-way/no-ack text; honest validation status. docs/README.md frames line.
public_api: every constant, struct field, function exactly as implemented. Downstream agents start the moment you finish, so be precise.`

const A2 = `${COMMON}
${CONTINUE}

YOUR ROLE: investiGATR library (spec section 6 entirely).
SCOPE: brain/investiGATR/**, brain/CMakeLists.txt (top-level host project), docs/investigatr.md.
TASKS:
1. Headers (include/investigatr/), sources (src/), host-only sim (sim/: differential drive + SimulatedSource), OdometrySource, Navigator with the exact phase logic, landmark following/statuses, input rules, per-update order, PID, slew, config validation (spec 6.1-6.5).
2. brain/CMakeLists.txt: only top-level host project; gtest once (find_package QUIET else FetchContent named googletest v1.14, gtest_force_shared_crt ON, BUILD_GMOCK OFF, INSTALL_GTEST OFF); add_subdirectory(investiGATR); add_subdirectory(navigatr-interface) ONLY if that directory's CMakeLists.txt exists (another implementer is creating it now). Targets investigatr, investigatr_sim, tests; explicit lists; -Wall -Wextra.
3. Tests per 6.6 on the sim, meaningful assertions. Tune defaults so tests pass with margin; report changed defaults.
4. Build: cmake -S brain -B ${S}/build-brain-a2 (recipe). If navigatr-interface exists but does not build yet (it is mid-work by another agent), configure with a temporary copy of the top-level list in ${S} that omits it, do not edit their files. Syntax-check every src/*.cpp: arm-none-eabi-g++ -std=gnu++20 -fsyntax-only -Ibrain/investiGATR/include. ${TOOL} No warnings.
5. docs/investigatr.md (API, input contract, implementing an InputSource incl. Brain motor-encoder/IMU odometry snippet, control law, config table + tuning, fallback and explicit source switching with alignment rules, host build/tests, limitations) and brain/investiGATR/README.md signpost.
public_api: every header, type, enum value, function, config field and default, plus sim/ helpers (drivetrain sim, SimulatedSource) used by the navigatr-interface equivalence test and the e2e test.`

const B1A = `${COMMON}
${CONTINUE}

YOUR ROLE: Pi half-duplex serial transport (spec 5.1 and link-side parts of section 3).
SCOPE: pi/navigatr/src/transport/*, pi/navigatr/src/resources/serial_link.h, pi/navigatr/src/impl/resources/serial_links.h/.cpp, pi/navigatr/tests/half_duplex_gtest.cpp, the half_duplex.cpp line in pi/navigatr/CMakeLists.txt NAVIGATR_SOURCES and the half_duplex_gtest.cpp line in pi/navigatr/tests/CMakeLists.txt (only those lines), docs/navigatr_resources.md (linux_serial_link section), docs/hardware.md (RS-485 section only; re-read right before editing), docs/pi_setup.md (DriverEnable lines only).
At the interruption half_duplex.h/.cpp and the serial_link/serial_links/serial_port edits existed and both CMake lines were added, but half_duplex_gtest.cpp was not written yet.
TASKS:
1. SerialLink: TransmitWindow{not_before_us, deadline_us}, SerialWriteResult.expired (+ failure detail), virtual nowUs() steady microseconds, virtual write(ByteSpan, const TransmitWindow&) defaulting to write(ByteSpan), virtual inputPending() default false. Existing callers compile.
2. Portable half_duplex.h/.cpp over a tiny port interface with injectable clock: window/expiry, input-pending abort, DE assert, partial writes with bounded wait, transmitter-empty wait (deadline bytes*10/baud + 2 ms), post guard, DE release on every path, discard output on failure, late_release detection; result the caller can count.
3. LinuxSerialLink half duplex when DriverEnable configured (DE low at open, value fd kept open, pwrite offset 0, FIONREAD, TIOCSERGETLSR/TEMT with tcdrain fallback, optional timing attributes); GPIO unavailable -> writes fail; SerialPort::write poll() with timeout; non-POSIX stubs compile; MemoryLink injectable clock (default real steady clock) honoring TransmitWindow.
4. tests/half_duplex_gtest.cpp: every half_duplex case in spec 5.4 plus MemoryLink window behavior.
5. Verify with a scratch CMake project under ${S}/b1a-check (transport sources + half_duplex_gtest + minimal deps, FETCHCONTENT_SOURCE_DIR_GOOGLETEST=${S}/nav-base/_deps/googletest-src). The full navigatr build is expected to be broken until the Pi integration lands; if it happens to compile, run it too.
6. Docs in scope: half duplex semantics, timing attributes, failures, GPIO numbering check, stuck-high-after-kill note, not hardware validated.
public_api: exact SerialLink/TransmitWindow/SerialWriteResult/MemoryLink clock API and half_duplex API.`

const C2A = `${COMMON}
${CONTINUE}
The interrupted run worked only in its scratch area: ${OLD_S}/pros-exp/brain/testing holds a created project (project.pros, Makefile, common.mk, include/, src/main.cpp, firmware/) and a bin/cold.package.elf from a build. Read it and reuse what works. brain/testing does not exist in the repo yet.

YOUR ROLE: PROS scaffold for brain/testing and the packaging fragment (spec section 8, build-system half). The libraries are being written concurrently by other agents; you set up everything that does not depend on their final APIs.
SCOPE: brain/testing/** (new), brain/gatr2_brain.mk (new). Do not edit library sources.
${TOOL}
TASKS:
1. Experiment in a scratch copy under ${S} first (copy brain/ and common/ there as needed). Never let common.mk's DEPFLAGS/MAKEDEPFOLDER/RENAMEDEPENDENCYFILE touch library sources (spec 8 warning: its mv would overwrite .cpp files). Find the verified commands: project creation with kernel@4.2.2 and no default libs (no liblvgl), CXX_STANDARD gnu++20 / C_STANDARD gnu17, -D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING, the two-part gatr2_brain.mk (before and after common.mk, GATR2_ROOT ?= ../..) compiling library sources in place from explicit lists (use the files currently present under brain/investiGATR/src, brain/navigatr-interface/src and pros/ if they exist, and common/frame_codec.cpp; make the lists easy to extend), kernel-files guard naming the restore command, and the offline restore command (delete the ignored kernel files in the scratch copy, restore with the PROS CLI from the local cache, pros make).
2. Create the real project in brain/testing with the verified commands: project.pros, Makefile, common.mk, include/api.h, include/main.h, .gitignore (firmware/, include/pros/, bin/, .d/ etc. per spec 8), a minimal src/main.cpp that builds with the kernel alone, include/robot_config.h skeleton with every hardware value marked placeholder (motor ports, reversed flags, gearset, voltage limit, serial port, baud, starting pose, demo destinations, landmark id/offset).
3. pros make in brain/testing must succeed. If the library sources in the tree do not compile yet (mid-work by others), keep them out of this first build via a variable in the Makefile and say so; the finishing agent will enable them.
4. Write ${S}/pros_notes.md: exact working commands (creation, restore, build), PATH setup, gotchas found, and how to add library sources. Do not write docs in the repo yet.
public_api: Makefile variables and how the finishing agent enables the libraries; verified commands.`

const B2A = (a1) => `${COMMON}

YOUR ROLE: navigatr-interface protocol client (spec section 4.2 and the client/fake-Pi parts of section 7). The investiGATR library is being finished concurrently by another agent; your part does not need it. A later agent writes NavigatrDriver, the PROS wrapper and the equivalence test on top of your client.
${up('common protocol (A1)', a1)}
SCOPE: brain/navigatr-interface/** (new): include/navigatr_interface/byte_port.h, client.h (+ any small headers), src/client.cpp, sim/ (fake Pi built on the real common codec), tests/ client tests, CMakeLists.txt.
TASKS:
1. BytePort (nonblocking read/write) and NavigatrClient implementing 4.2 exactly: correlation first; pi_instance rule only after correlation; session change handling (clear cached state, in-flight state-changing tickets -> kSessionLost never resent, clear acknowledged selection, fresh-nonce HELLO); HELLO Stale -> new nonce; scheduling one outstanding request with priority placement > select > GET_STATE poll (default 20 ms); GET_STATE fresh id each poll, never retried; placement same-bytes retries on timeout/Pending bounded by attempts and deadline; Ok recorded with (epoch, anchor) and kApplied only after a GET_STATE shows that anchor; selection reconciliation with settled Unknown/Unsupported outcomes; RX drain + FrameReader reset before each send; response timeout 60 ms from write return; request_gap 5 ms; rid wrap 65535 -> 1; unsupported version/op terminal (slow HELLO retry); ready/connected/link_age; round trip of the matched attempt; stats. Time injected via poll(now). Nonce supplied by the caller (a hook the PROS wrapper will fill).
2. sim/: scriptable fake Pi (drop/duplicate/corrupt/fragment/delay replies, restart with new pi_instance, pending placements, landmark results/sources, reply timing on a fake bus/clock), host only.
3. CMakeLists.txt: target navigatr_interface (src + \${REPO}/common/frame_codec.cpp, include dirs include/ and repo root), navigatr_interface_fakes (sim), test executable with explicit lists. Leave a clearly marked place where the driver agent adds driver.cpp and links investigatr. It must build under brain/CMakeLists.txt (A2 adds this subdirectory when your CMakeLists.txt exists) AND standalone via a scratch top-level project in ${S} while investiGATR is still in progress.
4. Tests per section 7 that concern the client: fragmented/corrupt bytes, truncated then valid reply, duplicate replies, dropped replies (placement same bytes, GET_STATE new id), stale replies (old rid/session/nonce, pre-send bytes drained), Pi restart via correlated reply and UnknownSession -> new session + cleared state, kSessionLost never resent, Ok -> kApplied only after anchor state, ready only after correlated GET_STATE, rid wrap, HELLO Stale -> new nonce, unsupported version terminal, timing on a fake bus/clock (Brain never transmits while a reply can still start; default timeout meets the spec 3 budget), selection statuses.
5. Syntax-check src/*.cpp with arm-none-eabi-g++ -std=gnu++20 -fsyntax-only -Ibrain/navigatr-interface/include -I. ${TOOL}
public_api: exact client API (config fields/defaults, poll/submit/result/status/snapshot accessors, stats), fake Pi API, source/include lists for the PROS Makefile fragment.`

const B2B = (a1, a2, b2a) => `${COMMON}

YOUR ROLE: navigatr-interface driver, PROS wrapper, equivalence test, driver docs (spec section 7, driver parts).
${up('common protocol (A1)', a1)}
${up('investiGATR (A2)', a2)}
${up('navigatr client (B2a)', b2a)}
SCOPE: brain/navigatr-interface/** (add driver.h/.cpp, pros/, driver and equivalence tests, CMake additions; small fixes to the client allowed, report them), brain/CMakeLists.txt only if the subdirectory is not added, docs/navigatr_interface.md (new; driver, client, sessions and restart behavior, placement acks, link loss, timing, PROS wrapper usage; leave two clearly titled sections, "PROS packaging and import" and "Testing application and hardware bring-up", each with a one-line note, for the PROS agent to fill).
TASKS:
1. NavigatrDriver implementing investigatr::InputSource over NavigatrClient: mm/cdeg -> m/rad; robot.valid iff PoseValid && Localized && AgeKnown && anchor accepted (command anchors; configured only with accept_configured_anchor, default false) && not waiting on a placement anchor; ages = reported + round trip + (now - receive); frame generation increments on any (pi_instance, session, epoch, anchor) change, 0 until the first usable state; landmark statuses per spec 7; connected/link_age per 4.2; placement API passthrough.
2. PROS-only code under brain/navigatr-interface/pros/ (ProsSerialPort over serial_* C API, task+mutex wrapper implementing InputSource by delegation, nonce mixing from several sources). Not in CMake. Syntax-check: arm-none-eabi-g++ -std=gnu++20 -fsyntax-only -D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING -I<kernel template>/include -Ibrain/navigatr-interface/include -Ibrain/investiGATR/include -I. ${TOOL}
3. Tests: driver conversions/validity/anchor rules/ages/frame generation/landmark statuses; stale measurement makes the Navigator not usable; equivalence test: same Navigator scenario on investigatr_sim with SimulatedSource vs NavigatrDriver fed by the fake Pi reporting the sim truth, both complete at the same pose within tolerance.
4. Build and run the whole brain suite: cmake -S brain -B ${S}/build-brain-b2 (recipe). All green.
public_api: exact driver and PROS wrapper API (constructors, config fields/defaults, placement API, status), and final source/include lists for gatr2_brain.mk.`

const B1B = (a1, b1a) => `${COMMON}

YOUR ROLE: Pi brain_link integration CODE AND TESTS (spec sections 3 Pi side, 4.1, 5.2, 5.3, and the code/test/config parts of 5.4). Pi documentation is done by a separate agent after you; do not spend time on docs.
${up('common protocol (A1)', a1)}
${up('Pi transport (B1a)', b1a)}
SCOPE: pi/navigatr/** code, tests, CMake, configs (only slot type lines and brain-link comments in the other session's configs). Small fixes to B1a's transport files allowed if integration needs them (report). No edits to common/ or brain/ (report needed changes). No docs.
TASKS:
1. brain_link CommandCollection and Publishing (delete vex_brain_serial.* and vex_brain.*; register for both categories); CommandState/PlacementRequest/PlacementEdge/RobotState changes; requestsFrom session gating; SlotInitializationContext additions and pairing build checks; world-estimation-noop flag; loop-rate check; PublishingInput diagnostics pointer; LinkStats counters in inspection links output; --inline warning; reply window from link poll timestamps (nowUs) with TransmitWindow writes; newest-request-wins; trailing-bytes suppression; first-drain-no-reply; partial-frame discard; pi_instance regeneration on reset; landmark reporting rules; anchor bits; age clamping; inspection command section updated.
2. Every "Must update" item in spec 5.4: grep vex_brain, vex_brain_serial, stream_on, CommandFrame, PoseFrame, kStatus, decodePoseFrame, landmark_associations across pi/navigatr and fix all. Convert configs to brain_link for both slots; keep DriverEnable gpio="6"; profiles with a loop rate failing the check: raise the rate or document in open_issues.
3. Tests: brain_io_gtest.cpp rewritten with every case in spec 5.4; end_to_end_gtest.cpp rewritten for brain_link (no output without a request); system_build_gtest/world_estimation_gtest fixes; localization_gtest session case; noop-WorldEstimation config answers SELECT with kResultLandmarkUnsupported and GET_STATE with landmark source none.
4. FULL navigatr suite: cmake -S pi/navigatr -B ${S}/build-nav (recipe). All green. Failures originating in the other session's files: report with evidence, do not fix.
public_api: brain_link config schema and defaults, CommandState/RobotState/PlacementRequest fields, LinkStats names, and exactly how an end-to-end test builds a System with a memory_link brain_uart, reaches the MemoryLink and its clock, and steps it.`

const B1DOCS = (b1a, b1b) => `${COMMON}
${RENAME}

YOUR ROLE: Pi documentation for brain_link (spec 5.4 Docs list), written against the ACTUAL code.
${up('Pi transport (B1a)', b1a)}
${up('Pi brain_link code (B1b)', b1b)}
SCOPE (docs only, no code): pi/navigatr/README.md (localized lines only: targets on the wire, deferred half-duplex item, brain link line; re-read right before editing, another session edits it), pi/navigatr/docs/architecture.md, landmarks.md (Brain output), coordinates.md (object fields), inspection.md (command and links schema), setup.md (DriverEnable, Commands/Publishing pairing, threaded mode, profiles have no FieldObject mappings so SELECT answers UnknownLandmark, *_no_correction profiles use noop world estimation so Unsupported), configuration.md if it lists slot types, calibration_inventory.md (brain link row only; re-read before editing).
Read the implementation (pi/navigatr/src/impl/commands, impl/publishing, state, runtime/system, transport) and describe exactly what exists. Honest: implemented and host tested, not hardware validated; list the hardware checks. Plain ASCII, no em dashes. Report code/doc mismatches you believe are bugs in open_issues.`

const C1 = (b1b, b2a, b2b, a2) => `${COMMON}
${RENAME}

YOUR ROLE: end-to-end reboot and restart tests against the REAL Pi runtime and the REAL Brain client/driver/Navigator (requirements VERIFICATION end-to-end scenario).
${up('Pi brain_link (B1b)', b1b)}
${up('navigatr client (B2a)', b2a)}
${up('navigatr driver (B2b)', b2b)}
${up('investiGATR (A2)', a2)}
SCOPE: pi/navigatr/tests/brain_link_e2e_gtest.cpp (new) and pi/navigatr/tests/CMakeLists.txt (add it plus the portable brain/communiGATR/src and brain/investiGATR/src sources and include dirs to navigatr_tests; explicit lists). You MAY fix genuine bugs you uncover in pi/navigatr brain_link code, brain/communiGATR src/include, brain/investiGATR src/include: minimal fixes, add or adjust the component's unit test when practical, list every fix with the reason in deviations_from_spec. Other agents concurrently write docs and the PROS app (they only read library code).
Build a full System (synthetic rig or planar_motion_integrator, memory_link brain_uart, brain_link both slots, a FieldObject mapping, world estimation yielding at least nominal landmarks), bridge a BytePort to the MemoryLink with controllable timing while the System steps. Scenarios:
1. Reboot: session A connects, SET_POSE applied, SELECT; A disappears while the System keeps stepping; client B (fresh objects, ids restart at 1) sends SET_POSE with the SAME request id and pose: applied (anchor_revision +1, placement_session == B); SELECT applies; replay captured A request bytes after B opened (incl. A HELLO and A SET_POSE): UnknownSession/Stale, nothing changes; inject captured A reply bytes into B: dropped; B's Navigator idle with zero output until a new command; target latch cleared if configured; no Pi reset.
2. Pi restart with the Brain on: new System instance and System::reset(): detected via correlated reply, input cleared, re-HELLO, new frame generation, in-flight placement kSessionLost not resent, an active Navigator command fails (kFrameChanged or kInputLost) and does not resume.
3. Init retries: dropped SET_POSE reply(ies): same bytes retried, applied exactly once (anchor_revision +1 only), kApplied only after a state with the new anchor; GET_STATE interleaved between retries causes no second application.
4. Frame capture race: Pi already localized by A; B places then immediately goTo: no kFrameChanged caused by the placement.
Build and run the FULL navigatr suite (${S}/build-nav-c1) and the brain suite (${S}/build-brain-c1); everything green.`

const C2B = (c2a, a2, b2a, b2b) => `${COMMON}
${RENAME}

YOUR ROLE: finish the brain/testing PROS application and the PROS docs (spec section 8).
${up('PROS scaffold (C2a)', c2a)}
${up('investiGATR (A2)', a2)}
${up('navigatr client (B2a)', b2a)}
${up('navigatr driver (B2b)', b2b)}
Read ${S}/pros_notes.md first. ${TOOL}
SCOPE: brain/testing/**, brain/gatr2_brain.mk, docs/communigatr.md (fill ONLY the "PROS packaging and import" and "Testing application and hardware bring-up" sections; re-read before editing). Do not edit library sources (another agent may fix library bugs concurrently); report needed library changes in open_issues.
TASKS:
1. Enable all library sources in gatr2_brain.mk (explicit lists from the driver result). robot_config.h complete with placeholders clearly marked.
2. App: control task as the only motor writer (Navigator + drivetrain + modes, mutex, 10 ms), mixTank + directions + voltage limit, cancel/disable stop immediately; initialize() bounded connect + starting pose placement with result check + pros::screen status, never hang; autonomous() absolute goTo, a waypoint path, landmark-relative alignment, each bounded, comparing command_id, stop on failure; opcontrol() manual tank through the owner + demo and cancel buttons; disabled() cancel/stop. Small and readable.
3. pros make in brain/testing succeeds with no warnings from our code; re-verify the restore command on a scratch copy; report size lines.
4. brain/testing/README.md signpost; docs sections: import steps for other PROS projects via gatr2_brain.mk and why PROS templates are not used; testing app setup, configuration, and hardware bring-up checklist (motor directions safely, link checks, placement, landmark ids), what is not validated.`

const C3 = (all) => `${COMMON}
${RENAME}

YOUR ROLE: final documentation, CI and consistency pass (spec sections 9 and 10, documentation parts). No code edits.
${all}
SCOPE: README.md (root: Pi runtime paragraph about what goes to the Brain; repository map adds brain/), docs/README.md (index entries for investigatr.md and communigatr.md), brain/README.md (new signpost), .github/workflows/ci.yaml (add job brain per spec 9; do not change other jobs), and consistency fixes in docs written in this effort ONLY where they contradict the code (re-read right before editing).
TASKS:
1. Cross-check docs vs code: interfaces.md byte tables vs common codec; investigatr.md vs headers/config defaults; communigatr.md vs driver/client/PROS wrapper/brain/testing; Pi docs vs brain_link code. Fix doc contradictions; report suspected code bugs in open_issues.
2. Scan new/changed docs and comments from this effort for em/en dashes, smart quotes, arrows, emojis (non-ASCII grep over brain/**, common/**, docs/interfaces.md, docs/investigatr.md, docs/communigatr.md, changed pi/navigatr files); fix docs, report code.
3. Run the brain host suite as CI would (cmake -S brain -B ${S}/build-brain-c3, recipe) to confirm the CI command works.
Report every doc changed.`

// ---------------- schedule ----------------
phase('Foundation')
const a1P = agent(A1, { label: 'A1:common-protocol', phase: 'Foundation', schema: RESULT })
const a2P = agent(A2, { label: 'A2:investigatr', phase: 'Foundation', schema: RESULT })
const b1aP = agent(B1A, { label: 'B1a:pi-half-duplex', phase: 'Foundation', schema: RESULT })
const c2aP = agent(C2A, { label: 'C2a:pros-scaffold', phase: 'Foundation', schema: RESULT })

const a1 = await a1P
if (!a1) { log('A1 failed; stopping dependent work'); return { a1, a2: await a2P, b1a: await b1aP, c2a: await c2aP } }
log('A1 done')
const b2aP = agent(B2A(a1), { label: 'B2a:navigatr-client', phase: 'Integration', schema: RESULT })

const b1a = await b1aP
log('B1a done' + (b1a ? '' : ' (no result)'))
const b1bP = agent(B1B(a1, b1a), { label: 'B1b:pi-brain-link-core', phase: 'Integration', schema: RESULT })

const a2 = await a2P
log('A2 done' + (a2 ? '' : ' (no result)'))
const b2a = await b2aP
log('B2a done' + (b2a ? '' : ' (no result)'))
const b2bP = agent(B2B(a1, a2, b2a), { label: 'B2b:navigatr-driver', phase: 'Integration', schema: RESULT })

const c2a = await c2aP
const b2b = await b2bP
log('B2b done' + (b2b ? '' : ' (no result)'))
const c2bP = agent(C2B(c2a, a2, b2a, b2b), { label: 'C2b:pros-app', phase: 'System', schema: RESULT })

const b1b = await b1bP
log('B1b done' + (b1b ? '' : ' (no result)'))
const b1docsP = agent(B1DOCS(b1a, b1b), { label: 'B1docs:pi-docs', phase: 'System', schema: RESULT })
const c1P = agent(C1(b1b, b2a, b2b, a2), { label: 'C1:e2e-reboot-tests', phase: 'System', schema: RESULT })

const c1 = await c1P
const c2b = await c2bP
const b1docs = await b1docsP
log('System phase done')

const allCtx = [up('A1', a1), up('A2', a2), up('B1a', b1a), up('B1b', b1b), up('B1docs', b1docs), up('B2a', b2a), up('B2b', b2b), up('C1', c1), up('C2a', c2a), up('C2b', c2b)].join('\n')
const c3 = await agent(C3(allCtx), { label: 'C3:docs-ci', phase: 'Docs', schema: RESULT })

return { a1, a2, b1a, c2a, b2a, b1b, b2b, c2b, b1docs, c1, c3 }
