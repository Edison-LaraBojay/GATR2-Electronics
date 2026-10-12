# W4 Brain publisher: implementation log

2026-09-29. Workstream key `w4-publisher`. Worktree `S/wt-spectu` (detached at ae457ee), where
`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
Edited only W4 files: `brain/spectuGATR/**` (new), `brain/CMakeLists.txt` (one
`add_subdirectory` line), `brain/gatr2_brain.mk` (the `spectugatr` entry). Nothing committed; the
new files are intent-to-add (`git add --intent-to-add brain/spectuGATR`, index only). Builds only
in `S/sp-build/w4-publisher-*`.

## What was built

Library `spectugatr` (C++17, std only in `src/`, namespace `spectugatr`), about 2.6k lines plus
1.7k lines of tests.

| File | Holds |
|---|---|
| `include/spectugatr/publisher.h` | the 5.1 API: units, `Pose2`, `Attitude`, `PoseSample`, `PlacementRef`, field, estimate (`LandmarkError`), path types, `ChannelSpec`, `PublisherConfig`, `Publisher`, `steadyClock`, `prosClock` |
| `include/spectugatr/transport.h` | `LineBuffer`, `TransportBudget` + `kUsbRelayBudget`/`kDirectUsbBudget`/`kRadioBudget`, `ByteBucket`, `KindStats`, `PublisherStats`, `claimTransport`, `nextLine`, `recordWrite`, `publisherClock`, `stats` |
| `include/spectugatr/stdio_transport.h` | `StdioTransport` (PROS builds only) |
| `src/slot.h` | `Slot<T>`: single-writer seqlock, `beginWrite`/`endWrite` heal a writer stopped mid-write |
| `src/event_queue.{h,cpp}` | `EventQueue`: bounded multi-producer ring (Vyukov), drop-oldest, bounded CAS loops, stale claims broken by the owner after `kStaleClaim` (50 ms) |
| `src/inputs.h` | what each slot holds (fixed arrays, SI units) |
| `src/publisher_state.h` | `PublisherState` (one allocation at construction) and the owner's schedule state |
| `src/publisher.cpp` | producer calls: validate, copy, return |
| `src/wire_encoding.{h,cpp}` | slot contents to protocol records and documents (pose, tilt, definition, estimate, path) |
| `src/pull.h` | one pull: `Packing` (share limits, wait rule), `Pull`, shared declarations |
| `src/take_slots.cpp` | new slot versions into encoded documents |
| `src/next_line.cpp` | `nextLine`: priority order, samples ticks, budget |
| `src/carousel.cpp` | repeats from leftover budget |
| `src/counter_channels.{h,cpp}` | the `spectu.*` counter channels (DEF ids from 0xFF00) |
| `src/transport.cpp` | `claimTransport`, `recordWrite`, `publisherClock`, `stats` |
| `pros/stdio_transport.cpp`, `pros/pros_clock.cpp` | PROS only |
| `tests/*_gtest.cpp` | 51 tests, explicit list in `brain/spectuGATR/CMakeLists.txt` |
| `examples/standalone/` | PROS project with `GATR2_BRAIN_LIBS := spectugatr` only |
| `PIPELINE.md`, `README.md` | component docs |

Memory: `sizeof(PublisherState)` = 69,344 B on the x64 host (one `new` at construction; the
tests prove no allocation afterwards).

Tests (`spectugatr_tests`, 51):
- `slot_gtest` (5): versions, latest wins, read during a write skipped, a writer stopped mid-write
  heals on the next write, a write overlapping a read discards the read.
- `event_queue_gtest` (7): FIFO order and seq, full ring drops oldest and counts, ring laps, stale
  claim broken after 50 ms of owner time and the late publish refused, a published claim is not
  broken, a full ring behind a stale claim gives up without waiting.
- `publisher_gtest` (35): every record and document through `nextLine` into the W0 decoder
  (`lineFrame`, `decodeFrame`, `DocAssembler`, `decodeDefinition/Estimate/Path`): pose (plain,
  Attitude, PoseSample with measured time and spread, held, unavailable), separate tilt with its
  age, SESSION placement and placement ref, definition (sorted, id 0 removed, box center, flags,
  field_id = crc32, SESSION ref), estimate = nominal + LandmarkError (also wrap near +-pi),
  path with a turn-only segment, CW turn, reverse, exact, target, reference, note, then kCompleted
  with no segments, the path's field id when the field changes before the pull, channel and event
  DEFs before samples, sample stamps and NaN, events durable and ordered, wrong ids and non-finite
  input counted, definitions full. Scheduling: latest-wins and replaced counts, pose only when
  new, pose cap (25 Hz), slow pose source declares its own period, samples cap per channel, byte
  budget in every window under full load, pose keeps its cap under full load, SESSION ahead of
  its document and documents ahead of samples, liveness only after 1 s of silence, carousel
  repeats state and counters, radio budget still carries pose (3 Hz) and field, a newer document
  abandons the old, the same field is not resent, max_chars per line and never-fitting content
  skipped, no budget no lines, a second claim refused, slow writes counted, a stalled transport
  never blocks producers (100k poses, 1000 events: 968 dropped oldest, the newest 32 arrive),
  frame seq and line counts.
- `allocation_gtest` (2): construction allocates exactly once; 10,000 pulls with every producer
  call (poses, tilt, samples, events, placement, 128-object field and estimate, 32-segment path)
  allocate 0 (counting `operator new`).
- `concurrency_gtest` (2): real threads, one producer thread per slot plus 4 event threads and
  one owner thread, paced over about 0.5 s: every line decodes, frame seq has no gaps, no record
  mixes two writes (pose y = -x, tilt pitch = -roll, every definition's objects share one y,
  every path's segments carry their command), pose seq rises, samples rise per channel, events
  keep per-thread order, `events_queued == received + dropped`. Second test: 8 threads x 2000
  unpaced events overflow the FIFO; order and counts still hold.

## Decisions and deviations (with reasons)

1. **`Box` gains `center` (Pose2) and `FieldObject` gains `estimated` and `reference`.** W0 review
   S5: without them the Brain cannot re-encode the link map byte for byte, so field_id could never
   equal the Pi's map id. The shapes now mirror `investigatr::Box` and `investigatr::FieldObject`,
   so W6 converts without loss. The round-trip check itself (7.2) needs the integrations (W6).
2. **Clock.** `PublisherConfig::clock` nullptr falls back to `steadyClock()`. On PROS pass
   `prosClock` (pros::micros in s, the clock communiGATR stamps with); it is declared in
   `publisher.h` and defined in `pros/pros_clock.cpp` because `src/` has no PROS. PROS 4.2.2
   `clock_gettime(CLOCK_MONOTONIC)` returns -1 even on success (disassembled from `libpros.a`
   `newlib_stubs.c.o`), so steady_clock is not trusted as the PROS clock.
3. **Pimpl.** `Publisher` holds `std::unique_ptr<PublisherState>`: the public header shows only
   the API; memory is fixed at construction (one allocation, tested).
4. **transport.h additions** not in the sketch: `recordWrite(Publisher&, Seconds)` (the owner
   reports each write's duration; the aux lane gets it from `AuxLineSource`), `publisherClock`
   (owners pass the publisher's own clock as `now`), `kSlowWrite` (5 ms),
   `PublisherStats::next_line_max_us`, `kNoEvent`, capacity constants aliased to the protocol's.
   `kMaxLineChars` is W0's 249, not the sketch's 250.
5. **Counters on the wire.** 11 of the stats go out as integer channels on the carousel, ids from
   0xFF00: `spectu.{pose,samples,path,field,estimate}.replaced`, `spectu.updates.dropped` (all
   kinds), `spectu.events.dropped`, `spectu.events.contended`, `spectu.lines`,
   `spectu.slow_writes`, `spectu.next_line_max_us` (the longest `nextLine`, renamed from the
   proposal's `spectu.next_line_us`). Not every field: the radio has about 40 B/s of leftover.
6. **Pose.** Held = a supplied `measured_at` equal to the previous sample's. POSE seq = pose slot
   version, so gaps show replaced poses. `poseUnavailable` keeps the last pose values with the new
   status and no measured time; `kValid` is refused and counted. The Attitude overload keeps its
   tilt in the pose slot (one producer per slot); otherwise POSE carries the latest `updateTilt`
   with its age at sent_ms. Tilt has no bytes of its own (it rides in POSE).
7. **Estimate document.** One record per definition object in definition order (PROTOCOL.md).
   Pose = nominal + error added in wire units, so the error survives to the mm and centidegree.
   An object without an estimate is source none, invalid, at nominal. Observed with no age gives
   65535 ms. `estimate_id` = estimate slot version; epoch and anchor revision from the placement
   ref. Estimates wait until their field is taken in.
8. **Path document.** Start = the first segment's start (zeros without segments); segments chain
   by end pose (as 5.5 defines). Ended states keep the target and drop segments; segments are
   validated only for kActive. Revision +1 per path document. `field_id` = the field slot version
   when `updatePath` ran, resolved through the current or previous definition, else none. A
   kReplaced followed at once by the next kActive collapses (latest-wins slot); the path document
   is "the current path".
9. **Field input.** Objects sorted by id in the slot; id 0 and repeats removed; over 128 cut; each
   counted once in `field.dropped`. Any non-finite value refuses the whole update. A re-published
   identical field (same id) is not resent.
10. **Scheduling rules the proposal left open** (each found necessary in a host probe):
    - the pose share may use every token; everything else leaves one pose line (89 characters) in
      the bucket, so the next pose always fits;
    - splittable content (document chunks, samples) waits for a fuller bucket instead of going
      out in small frames (`Packing::worthPacking`). Without it the 128-object definition went out
      in 20-byte chunks at about 5x overhead (probe `estimate_probe.cpp`);
    - a kind waiting for room holds every lower kind in that frame. Without it samples (239-char
      lines) always beat 249-char chunks to the tokens and the estimate never went out under load;
    - content that can never fit the caller's `max_chars` and burst is skipped, not waited for;
    - samples: a tick opens at most `samples_hz` times a second and sends every channel that had
      a new value when it opened (latest value), grouped by measured ms; a tick closes after one
      period even if a channel is stuck mid-write;
    - carousel items (SESSION, the DEF pass with the counter DEFs, three documents, counter
      values) repeat at most once per carousel period from leftover budget and take turns going
      first; the first repeat is one period after start;
    - SESSION `pose_period_ms` = max(cap period, smoothed interval between sent poses, weight
      1/8), changed only when it moves more than 25%;
    - every period check has 1 us of slack for clock rounding;
    - the budget counts line characters; COBS adds 6 B per line that is not charged (the radio
      preset is a placeholder until B2).
11. **Event FIFO.** Vyukov ring of 32; a producer finding it full drops the oldest itself (counted
    `dropped`); CAS loops stop after 8 attempts (counted `contended`). The owner breaks a head
    claim left unpublished for 50 ms of its own time; the late writer's publish is refused and
    counted. Known residual: a writer stalled over 50 ms that then resumes can race a new claim
    of the same cell 32 events later (one torn event). `events_contended` = exhausted retries +
    broken claims + refused late publishes; `events_dropped` includes events with unknown ids.
12. **Session id** is mixed on the first `nextLine` from the clock at construction, at the first
    pull and now, plus two addresses; never 0. The ProsLink battery entropy is not available to
    the portable core.
13. **Layout.** `stdio_transport.h` sits in `include/spectugatr/`, like communiGATR's
    `pros_usb_port.h`, not in `pros/` (STRUCT-9; its legacy note defers exactly this); its source
    is in `pros/`. Tests include two private headers from `src/` (`slot.h`, `event_queue.h`). No
    `sim/` folder (outside W4's list), so the small decode helpers live in the two test files that
    need them.
14. **`brain/spectuGATR/CMakeLists.txt`** is W4's (the `add_subdirectory` target). It adds
    `spectuGATR/protocol` when no parent did, and links `Threads::Threads` into the tests. W6
    appends `add_subdirectory(integrations)` there.
15. **StdioTransport.** Opens and activates `/ser/sgtr` first, then claims (closes the stream on a
    refused claim); a failed `task_create` keeps the claim, so nothing else writes for that
    Publisher. Task `spectugatr-stdio`, `TASK_PRIORITY_DEFAULT`, default stack, at most one line
    per 2 ms wake, one `write` per line (one COBS packet), write time through `recordWrite`. The
    write return is ignored; truncation counting belongs to bench test B11.
16. **Example.** A demo circle at 20 Hz, not odometry: with no other GATR library and no sensors
    there is nothing to integrate. Radio budget, one channel, one event, stats on the screen.
    `common.mk`, `include/api.h`, `include/main.h` are copies of locaGATR's PROS template files.

## Notes for W6

- Apps need `spectugatr` in `GATR2_BRAIN_LIBS`; `brain/locaGATR/Makefile` and
  `brain/operaGATR/Makefile` are in no Phase A ownership list (coordinator's call). Checked here:
  operaGATR with `GATR2_BRAIN_LIBS="investigatr communigatr actugatr spectugatr"` builds with 0
  warnings.
- USB lane: `claimTransport(publisher)` once, then per tx loop after the link frame
  `nextLine(publisher, line, publisherClock(publisher), max_chars, kUsbRelayBudget, bucket)`, and
  `recordWrite(publisher, previous_write_duration)`.
- Field: `investigatr::Field` maps field by field to `FieldDefinition` (box center and flags
  included). Estimate: `LandmarkError` = estimate pose - nominal, field axes, dtheta wrapped.
- `brain/README.md` has no spectuGATR line yet (in no ownership list).

## Commands and results (Git Bash, `export PATH=/c/msys64/ucrt64/bin:$PATH`)

```
# baseline, before any W4 edit
cmake -S S/wt-spectu/brain -B S/sp-build/w4-publisher-base -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src
cmake --build S/sp-build/w4-publisher-base -j16      -> exit 0, 0 warnings
ctest --test-dir S/sp-build/w4-publisher-base -j16    -> 100% passed, 455/455

# with W4
cmake -S S/wt-spectu/brain -B S/sp-build/w4-publisher-host (same flags)
cmake --build S/sp-build/w4-publisher-host -j16       -> exit 0, 0 warnings
ctest --test-dir S/sp-build/w4-publisher-host -j16 --output-on-failure
                                                     -> 100% passed, 506/506 (455 + 51)
spectugatr_tests.exe --gtest_shuffle --gtest_repeat=3 -> 51 passed, 3 times
spectugatr_tests.exe --gtest_filter=Concurrency* --gtest_repeat=10  -> 2 passed, 10 times
spectugatr_tests.exe --gtest_filter=Concurrency.EventBurst* --gtest_repeat=10 -> passed 10 times

# Release
cmake ... -B S/sp-build/w4-publisher-release -DCMAKE_BUILD_TYPE=Release
cmake --build                                        -> exit 0; warnings only in
    brain/investiGATR/src/planner.cpp (-Warray-bounds, existing code) and
    spectuGATR/protocol/src/doc_assembler.cpp (W0 review S3); none in brain/spectuGATR
ctest                                                -> 506/506 passed

# PROS (arm-none-eabi-g++ 13.3.1, PROS 4.2.2 kernel files copied from /c/GATR2-Electronics)
cd S/wt-spectu/brain/spectuGATR/examples/standalone && make -j16
                                                     -> exit 0, 20 units, 0 warnings,
                                                        hot image 18.91 KB text
make clean; kernel copies removed again
# apps, in a scratch mirror (git archive HEAD brain translaGATR + the W4 files), so W6's
# folders in the worktree were not touched: S/sp-build/w4-publisher-apps
locaGATR make -j16                                   -> exit 0, 0 warnings
operaGATR make -j16                                  -> exit 0, 0 warnings
operaGATR make GATR2_BRAIN_LIBS="investigatr communigatr actugatr spectugatr"
                                                     -> exit 0, 0 warnings, 19 spectu units

# formatting and text
clang-format.exe (pip 22.1.8, repo .clang-format) -i, then --dry-run --Werror on every
  .h/.cpp under brain/spectuGATR (the PROS template copies api.h and main.h excepted) -> clean
every W4 file plain ASCII (LC_ALL=C grep for non-printables: 0), LF endings
```

Mutation check (TEST-12), scratch copy in `S/sp-build/w4-publisher-mut`, one mutation at a time,
forced rebuild, all non-concurrency tests; baseline 49/49 before and after. Each mutation was
caught:

| Mutation | Caught by |
|---|---|
| slot sequence left odd after a write | 32 tests incl. every `Slot` test |
| no "waiting holds lower kinds" | `PoseKeepsItsCapUnderFullLoad`, `Allocation...NeverAllocate` |
| no pose reserve | `PoseKeepsItsCapUnderFullLoad`, `RadioBudgetStillCarriesPoseAndField` |
| no drop-oldest | `FullDropsOldestAndCounts`, `StalledTransportNeverBlocksProducers` |
| held never set | `MeasuredTimeSpreadAndHeld` |
| abandoned document not counted | `NewerDocumentAbandonsTheOldOne` |
| landmark error ignored | `EstimateIsNominalPlusLandmarkError` |
| slot read without the second sequence check | `WriteOverlappingReadDiscardsIt` |
| stale claims never broken | `StaleClaimBrokenAfterTimeout`, `FullBehindStaleClaimGivesUpWithoutWaiting` |
| pose cap ignored | `PoseRateCap`, `PoseKeepsItsCapUnderFullLoad`, `RadioBudget...`, `Allocation...` |

`nextLine` time on this Windows host (x64, Release libraries at -O3, probe at -O2, full test
load, 100k calls, `S/sp-build/w4-publisher-probe/timing_probe.cpp`, two runs): p50 0.5 us, p99
2.4-2.5 us, p99.9 4.9-5.8 us, max 266-472 us (the box was loaded by other builds). This fills the 4.4 row 2s "W4 host timing"
only; the Brain figure waits for B12.

Nothing here ran on a Brain, controller, radio or Pi.
