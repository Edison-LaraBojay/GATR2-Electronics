# Verify review, round 2 (adversarial): log

2026-09-29. Workstream key `verify-review-r2`. Read-only on the worktree `S/wt-spectu`: nothing
there was edited, staged or committed (`git status` 237 entries before and after, no ignored
leftovers). Builds, probes and mutations only in `S/sp-build/verify-review-r2-*`, where
`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
`C:/GATR2-Electronics` read only (git log/diff/show, kernel files copied out). `S/wt-navi` not
touched.

Scope: all of Phase A after fix-r1, checks (a) to (g) of the task. Round 1 findings 1-6 were
re-checked; its minor items 7-12 are listed as still open.

## Verdict

pass = false.

- 1 blocking: the carousel starves. A receiver that starts after the Brain program never gets
  SESSION, DEFs or the field definition under loads shaped like the apps (host-measured with the
  real Publisher).
- 1 should-fix: the `spectu.*` counter values never go out on the radio budget.
- Everything else checked below is fine or minor.

## What was checked, and how

| Check | Method | Result |
|---|---|---|
| Host suites | fresh Debug builds of brain, spectuGATR, pi/naviGATR (old layout) from the worktree | 536/536, 263/263, 626/626; 0 warnings in all three build logs |
| (a) call paths | read publisher.cpp, slot.h, event_queue.*, next_line.cpp, take_slots.cpp, carousel.cpp, wire_encoding.cpp, transport.cpp, stdio_transport.cpp, the three integrations, usb_transmit.*, pros_usb_port.cpp, pros_drive.cpp, client.cpp hooks | producers copy into seqlock slots or the bounded CAS FIFO; no wait, no I/O, no allocation after init (allocation_gtest covers producers and nextLine for 10000 pulls). Observers run under the link mutex and only copy; DriveObserver runs with no lock. Seqlock ordering is the standard fence pattern; the FIFO's owner-side stale-claim break and the drop-oldest path were traced: bounded, and the documented torn-event residual stands. Slot producers per app: pose, placement, field, estimate from the poll task (all Client notify calls are inside poll paths); path and drive channels from the drive task; tilt from the app/background task |
| (b) unused = unchanged | `git diff HEAD` of communiGATR, actuGATR, investiGATR, apps; old vs new transmit loop by hand | with no aux source `UsbTransmitter::loop` + `delay(2)` is the old loop step for step (pop, expiry, one write per line, delay only when empty, stop check per frame). `robotStatus` refactor is equivalent. ProsDrive gets `ProsDriveConfig{}` as before. Apps: `kSpectuEnabled=false` leaves `g_viz` null, observers and `usb_aux` empty, Motion's sink `g_link`. Only unconditional change: the approved D9 fix. Aux lane: one write per line, lines <= 249 chars (cap 256), link frames before and after the one aux line, every write < 2047 B |
| (b) drift | trial `git merge-file` of 6 overlapping files (user tree now also edits docs/communigatr.md) into `S/sp-build/verify-review-r2-merge` | 0 conflicts each; textual only |
| (c) D9 | scratch copy `S/sp-build/verify-review-r2-d9src`, motion.cpp gate put back on `planned_` (finish clears it after the report) | fail on the old gate: `Motion.CancelDuringReplanHaltClearsThePath`, `Motion.RefusedReplanClearsThePath`, `Motion.NewCommandBeforeItsPlanClearsTheOldPath`, `PathFeed.MotionReplacementDuringHaltEndsTheOldPath`; all pass on the worktree. Logic: `reported_` set only when a sink got a plan, cleared once by `start()` (kReplaced, old id) or `finish()`; no other terminal transition exists |
| (d) encoder/decoder | publisher_gtest reads; new cross probe (Publisher -> host Decoder -> Combiner -> coreScene) | every kind round-trips through the protocol decoder the host wraps. Cross probe, relay budget: pose, SESSION, DEF, SAMPLES, EVENT and 3 documents forwarded; scene live, fresh pose, field, estimate, current path; 11 counters held. Two Brains, same robot: 0 items forwarded from the second, refused with the reason |
| (e) combine | combiner.cpp, core_scene.cpp vs PROPOSAL 5.6 | rules 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12 and liveness match; no Pi pose path exists; stale and lost keep the last pose with its age; takeover only by the same robot once the first is lost. The fix-r1 hold relies on SESSION arriving within one carousel: see Finding 1 |
| (f) CDC2 | cdc2.*, controller.cpp vs discovery/controller-host.md s2, s2.1, s9b | request header and VarU16, CRC-16/XMODEM BE with residue check, simple reply without crc, SYSTEM_VERSION product/flags bytes, USER_READ data at ecmd+ack+channel, RADIO_STATUS layout all match; one request outstanding, resync on timeout, no write to the Brain, no channel switch |
| (g) style | headers, PIPELINE.md files, clang-format, byte scan | namespaced, aliases, struct for data, classes where state lives, explicit test lists. clang-format 22.1.8 `--dry-run --Werror`: 179 added .h/.cpp clean (the two PROS template headers excluded as before). No byte outside tab, LF, 0x20-0x7E in added files or added lines (probe with an em dash detected) |
| PROS | scratch mirror `S/sp-build/verify-review-r2-pros` | locaGATR, operaGATR, spectuGATR off and on: 4 builds rc 0, 0 warnings |

## Findings

### Blocking

1. **The carousel starves: a late receiver never gets SESSION, DEFs or the field.**
   `brain/spectuGATR/src/carousel.cpp`, `src/next_line.cpp`.
   - SESSION repeats are not reserved.
     - They use only what is left after the pose, changed documents and samples.
     - Inside the carousel, the head (`Carousel::next`) moves only when an item finishes. A DEF
       pass or a document repeat that spans many pulls keeps the head, and while it waits for
       room every later item gets room 0.
     - PROPOSAL 4.3 budgets the SESSION carousel ahead of documents ("SESSION 125 / 2 s
       carousel = 63").
   - The Client refreshes the estimate every 0.5 s (`field_period`), and the Pi makes a new one
     every 200 ms while it corrects landmarks (`estimate_period_ms="200"`). Each refresh is a
     changed document, which goes ahead of samples and repeats. With the 25 Hz pose, that fills
     the relay lane.
   - Probe `latejoin.exe`: a receiver joins 60 s after start and watches for 240 s. Pose at
     50 Hz, channels written every 10 ms, 11 counter DEFs.

     | Budget | Load | SESSION after | DEFs | Field definition |
     |---|---|---|---|---|
     | usb relay | locaGATR-like: 16 objects, 5 events, estimate every 0.5 s | 0.0 s (estimate id changes) | 0/16, never | never |
     | usb relay | operaGATR-like: 16 objects, 12 channels, 8 events, estimate every 0.5 s | 0.0 s | 0/31, never | never |
     | usb relay | operaGATR-like, estimate never | never | 0/31 | never |
     | usb direct | operaGATR-like, estimate every 0.5 s or never | 0.0 s / never | 0/31 | never |
     | radio | operaGATR-like with 4 channels, estimate never | 86 s | 114 s | 55 s |
     | radio | example-like: 2 objects, 1 channel, 1 event | 19.8 s | 23 s | 15 s |

   - Probe `carousel.exe` (300 s from start):
     - relay and direct, 32 objects, 12 channels, 8 events, no estimate change: 2 SESSION
       records in total, none after 4.1 s;
     - relay, operaGATR-like with estimate refresh: 0 SAMPLES records in 290 s;
     - radio, 4 objects, 1 channel, 1 event/s: SESSION mean gap 93 s, max 159 s, against the
       10 s carousel.
   - Diagnosis check (scratch mutation `S/sp-build/verify-review-r2-mut`, SESSION repeat served
     first in `addCarousel`): radio mean SESSION gap goes from 23.1 s to 10.9 s (example load)
     and from 92.9 s to 14.7 s (1 event/s). The loads with no leftover budget stay starved, so
     the budget side needs its own fix.
   - Effects:
     - a receiver started after the Brain program (the usual order), a restarted Pi adapter, or
       any lost first-pass chunk on radio never shows the field (rule 7 needs SESSION's field id
       and the definition), and has no channel or event names;
     - fix-r1's rule-8 hold, which was justified as "at most one SESSION carousel (2 s USB,
       10 s radio)", can last indefinitely.
   - Fix ideas:
     - reserve SESSION's repeat ahead of changed documents and samples, as 4.3 did;
     - move the carousel head every pull, not only when an item finishes;
     - give repeats a minimum share, or cap how often a changed estimate is resent per
       transport.

### Should-fix

2. **`spectu.*` counter values never go out on the radio budget.** `carousel.cpp`,
   `carouselCounters`.
   - The counters go out as one SAMPLES record of 11 values: 7 + 6 x 11 = 73 B.
   - With a pose on, the largest rest record at the radio burst is 67 B:
     (256 - 89 - 5) / 2 - 14. `Packing::add` skips a record that can never fit, and nothing
     splits this one.
   - Probe `probe.exe`, 60 s:
     - radio with pose: 27 counter DEFs, 0 counter SAMPLES;
     - radio without pose: 4;
     - relay: 24;
     - direct: 29.
   - Cross probe: after 60 s on radio the combiner holds 0 counters.
   - Effects:
     - rule 10 and the 5.8 capture metadata (the publisher's replaced and dropped counts, "so a
       radio capture never claims native-rate completeness") are empty on exactly the transport
       they exist for;
     - `kLossSpike` can never fire on radio.
   - The 11 counter DEFs also make up most of each radio DEF pass. From start, all 13 DEFs took
     104 s.
   - Fix idea: split the counters into records that fit the room, as `addSampleRecord` does, or
     send a subset per pass.

### Minor

3. No repo test runs real Publisher output through the host Decoder and Combiner (round 1 #7).
   The cross probe here does, and it finds Finding 2. It would make a good test.
4. The session id mixes only clock readings and two addresses (`next_line.cpp` `sessionId`).
   PROPOSAL 5.5 asks for the ProsLink nonce style, which also mixes battery voltage and current.
   A repeated id across program restarts would hide a restart (rule 1). A seed or entropy
   function in `PublisherConfig` would fix it.
5. The aux lane only counts `last_write` and never backs off. If B5/B7 show that VEXos stops
   draining with no host, 4000 B/s fills the 2047 B PROS buffer in about half a second, far sooner
   than link retries alone [estimate]. Hardware behavior is unknown.
6. Still open from round 1:
   - comments cite out-of-repo documents: `PROPOSAL` in recorder.h:2, combiner.h:2,
     core_scene.h:2, combine/PIPELINE.md:20, cdc2.h:3, controller.h:3; `disc:` in cdc2.h:4 and
     cdc2_gtest.cpp:2; `W0` in fakes_gtest.cpp:2, 49;
   - `spectuGATR/PIPELINE.md` still says "Status: designed";
   - `FrameError::kNoSession` shadows `protocol::kNoSession` (`-Wshadow`: 2 hits, rechecked);
   - `usbLane` gives no RS-485 note;
   - `parseUserRead` does not check the channel byte;
   - the EventQueue retry drop bound.

### Checked and fine (beyond the table)

- NaN samples are allowed by design; the capture JSON writes `null` (VIEW.md line 32).
- The field_id round trip holds for real Pi maps. The Pi sorts objects by id and wraps headings
  to (-18000, 18000], the same as the publisher.
- The Pi's `pros_usb_link` counts SG1 lines as console text. Its line limit (4 + 2 x frame + 256)
  is above 249.

## Commands and results (Git Bash, `export PATH=/c/msys64/ucrt64/bin:$PATH`)

```
# host suites (script S/sp-build/verify-review-r2-run.sh; G = S/nav-base/_deps/googletest-src)
cmake -S S/wt-spectu/<brain|spectuGATR|pi/naviGATR> -B S/sp-build/verify-review-r2-<brain|spectu|navi> \
  -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$G
cmake --build <dir> -j12 ; ctest --test-dir <dir> -j12 --output-on-failure
  brain  -> build rc 0, 0 "warning" lines, 536/536 passed
  spectu -> build rc 0, 0 "warning" lines, 263/263 passed
  navi   -> build rc 0, 0 "warning" lines, 626/626 passed

# D9 on the old gate (S/sp-build/verify-review-r2-d9src, mutation script verify-review-r2-d9mut.py)
cmake --build S/sp-build/verify-review-r2-d9 -j8 --target actugatr_tests spectugatr_integrations_tests
actugatr_tests.exe                -> 94 passed, 3 failed (the three D9 tests)
spectugatr_integrations_tests.exe -> 12 passed, 1 failed (PathFeed.MotionReplacementDuringHaltEndsTheOldPath)

# PROS (bash S/sp-build/verify-review-r2-pros.sh; mirror of the worktree plus firmware/ and
# include/pros copied from /c/GATR2-Electronics/brain/<app>; on = kSpectuEnabled true in the mirror)
locaGATR off/on, operaGATR off/on -> make rc 0, 0 warnings each

# probes (S/sp-build/verify-review-r2-probe; build.sh links verify-review-r2-brain libs,
# build_cross.sh adds the verify-review-r2-spectu host libs)
probe.exe    -> FAILED as expected: radio + pose 0 counter SAMPLES in 60 s (relay 24, direct 29)
cross.exe    -> PublisherThroughDecoderCombinerAndCoreScene OK; TwoBrainsSameRobot... OK (0 forwarded);
                RadioSourceCountersReachTheCombiner FAILED (0 counters); RadioCarouselTimes: all
                13 DEFs at 103.9 s; RadioLateJoinTimes: SESSION after 53-143 s
carousel.exe -> SESSION gaps per budget and load (numbers under Finding 1)
latejoin.exe -> late-join table under Finding 1
S/sp-build/verify-review-r2-mut/carousel.exe (SESSION-first mutation) -> radio gaps 10.9 s / 14.7 s

# style
clang-format 22.1.8 (pip) --style=file --dry-run --Werror on 179 added .h/.cpp -> 0 failures
perl byte scan of added files and '+' lines of git diff HEAD -> nothing outside tab, LF, 0x20-0x7E
g++ -std=c++17 -O2 -Wall -Wextra -Wshadow -c protocol/src/frame.cpp -> 2 shadow lines (kNoSession)

# drift (scratch only): git merge-file of 6 files into S/sp-build/verify-review-r2-merge -> 0 conflicts
```

Nothing here ran on a Brain, controller, radio or Pi.
