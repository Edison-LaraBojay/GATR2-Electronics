# Verification of CODING_STANDARD.md (draft 2)

Target: `coding-standard/CODING_STANDARD.md`, fixed in place (5014 lines before, 5077 after).
Backup of the file before this pass:
`C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/standard/verify/CODING_STANDARD.before-verify.md`.
Scripts, the venv (clang-tidy 22.1.8, clang-format 22.1.8, lizard 1.24) and raw results sit in the
same `verify/` folder. The repo was only read (`git show ae457ee:<path>`, `git grep`); HEAD is
`ae457ee` and the tree is clean.

## 1. Citations

Method: `cites.py` pulls every `path:line` and `:line` continuation from the standard, resolves
short paths by unique suffix in `git ls-files`, reads the file at `ae457ee`, and checks the quoted
backtick or double-quote snippets near each citation against the cited lines (whitespace collapsed,
comment markers stripped, `...` treated as a gap). A second run with an exact line window found the
off-by-one cases. Every citation the script could not confirm was opened by hand.

Result: 355 citations. 277 matched on the exact lines; the rest were matcher artifacts (the quote
sits before the path, spans a wrapped comment, or the citation has no quote) and were checked by
hand. No missing file, no line past end of file, no ambiguous short path.

Fixed:
- `pi/naviGATR/src/math/se3.h:73` -> `:73-75` (the quoted ternary continues on 74-75).
- `pi/naviGATR/src/contracts/sensor.h:46-47` -> `:47-48` (46 is a comment; the alias ends on 48).
- `pi/naviGATR/src/impl/resources/pros_usb_link.cpp:143-160` -> `:151-168` (the termios setup is
  `termios settings{};` at 151 through the `tcsetattr` check ending at 168).
- `pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:163-179` -> `:163-178` (179 is
  `ok = addWheel(...)`, not part of the quoted block).
- `pi/naviGATR/src/runtime/system.h:1-84` -> `:1-83`: the header comment is 83 lines (84 is blank).
  "84-line"/"84 lines" also fixed in phase 6 and Appendix C.

Checked by hand and kept (examples):
- `pi/naviGATR/config/override/brain_profile_usb.xml:55-86` and `brain_profile_rs485.xml:56-87`:
  `diff` shows the two 32-line blocks identical.
- `pi/naviGATR/src/impl/world_estimation/apriltag_world_estimator.h:12-17`: the quote "A step fault
  leaves the previous field untouched and publishes nothing" is on 15-17.
- `pi/naviGATR/src/impl/commands/brain_link_commands.cpp:158`: the constructor line, where the
  default-constructed engine would be reported; `std::seed_seq` reseeding is on 164-166.
- Appendix A.2 sources: `SourceState` `kNoDataYet`/`kUnavailable`/`kFault` (`core/records.h:37-42`),
  `MeasurementRecord` with `state` and `diagnostic` (`:94-100`), the four `make_*` channel factories.
- Appendix B numbers: `kPiBaud = 115200`, `kSampleHz = 50`, `kStatusPeriodUs = 200000`,
  `state_period = 0.020`, `field_period = 0.5`, `telemetry_period = 0.1`, `state_hz = 30.0`,
  `diag_hz = 4.0`, `preview_hz = 5.0`, `<Loop rate_hz="100"/>`.
- Path-only claims: `function_status.h` has `kOk`, `kNoData`, `kFault`; `attitude_channel.cpp:22`
  has the quoted `err = node.path() + ": needs <Source ...`; `field_documents.cpp:262`
  `fieldReferencesHeader`; `capture_service.cpp` defines `CaptureRecorder::`; `transforms.js` is
  all exported functions; Pico `main.cpp:329-333` has the skip-the-tick block.
- Every repo-rooted path in the standard exists at `ae457ee` except the proposed files
  (`PIPELINE.md` files, `docs/validation.md`, `docs/glossary.md`, `docs/toolchains.md`,
  `.clang-tidy` overlays, `tools/field_refs/field_refs.cpp`, `translaGATR/README.md`).
- Symbols named in Appendices A.2, C and D exist at `ae457ee`, except the ones the text marks as
  after step 2c or step 3 (`make_commands`, `make_targets`, `make_publishing`, `runInline`,
  `runThreaded`, `applyLandmarkError`).
- 11.4 move-map summary: per-destination file counts equal `restructure/plan/move_map_final.tsv`
  exactly (201 rows, 189 moved, 12 `same`: config 7, math 5).

Examples that did not show what they claimed (fixed):
- DOC-3 "Good: `brain/README.md`, `brain/actuGATR/README.md`, `pico/README.md`, `bench/README.md`":
  none has `Status:`; `brain/README.md` is 38 lines (DOC-1 max is 20); `bench/README.md` is titled
  "# Bench tools", not the folder name. Now "Closest today: `pico/README.md` (7 lines) and
  `brain/actuGATR/README.md` (15 lines) ... Neither has `Status:` yet."
- DOC-5 "Good: `bench/rs485_link/README.md`": headings are Wiring, Pi side, Brain side, Success; no
  Troubleshooting, no `Last run on hardware:`. Now "Closest today" with what is missing.
- WIRE-6 named `kNoTicket` as a reserved wire value. `git grep` finds no ticket in translaGATR:
  `PlacementTicket` is a Brain-local counter (`brain/communiGATR/src/client.cpp:99`
  `last_ticket_ = last_ticket_ == UINT32_MAX ? 1 : last_ticket_ + 1;`). Now `kNoSession`,
  `kNoDocument` (proposed names) plus a real Bad: `translaGATR/frames.h:316`
  `uint32_t session = 0; // 0 in HELLO` and `:365` `... profile_id = 0; // ..., 0 = none`
  (spacing trimmed).
- ALIAS-2 Good `using PlacementTicket = uint32_t; // 0 = none or refused` is a good alias but an
  in-band sentinel outside the wire, which TYPE-16 bans. Kept for the alias, with that noted.

Counts (not `path:line`) were spot-checked, all correct: `brain_profile_builder.cpp` 616 lines, 73
lines calling `SetAttribute` (requirements.md says 78 calls; the code has 73 calls on 73 lines, so
the standard is right); `system.cpp` 1450, `system.h` 479, `runtime/` 26 files, `field_scene.js`
1582, `frame_codec_gtest.cpp` 3554, `brain_profile_gtest.cpp` 2926, `viewer_tests.py` 1131; 6
literal `kPi` definitions (plus one alias); 0 structured bindings; 426 first-party files start with
`// <file name>`.

## 2. Tool names and flags

clang-tidy:
- All 81 check names the standard mentions are in `clang-tidy --list-checks -checks='*'` of the
  pinned 22.1.8 wheel (605 checks). The docs list (clang.llvm.org, 24.0git) also has the newer
  ones checked by name (`bugprone-capturing-this-in-member-variable`,
  `bugprone-random-generator-seed`, `bugprone-throwing-static-initialization`,
  `cppcoreguidelines-use-enum-class`, `misc-use-internal-linkage`, and others), none as aliases.
- The 9.2 YAML, extracted from the file, passes `clang-tidy --verify-config`, and so does the tests
  overlay. A planted bad option name is reported ("unknown check option"), so the option keys
  are verified too, not just the check names.
- `readability-function-size.NestingThreshold`: tested. 4 nested `if`s are flagged at threshold 4
  ("nesting level 5 starts here") and pass at 5, so "`NestingThreshold: N + 1` enforces nesting over
  N" is right. `LineThreshold: none` is accepted.
- TEST-15's claim that `misc-use-internal-linkage` sees types and `-Wmissing-declarations` does
  not: confirmed (a global-scope `struct BusRun` is flagged only by the clang-tidy check).

GCC (MSYS2 GCC 15.2.0): every `-W`/`-f` flag in the standard compiles a test file cleanly, except:
- `-Wthread-safety`: clang only. The standard already says "clang `-Wthread-safety` if clang is
  adopted (D-13)". Kept.
- bare `-Wstack-usage` (two mentions): GCC wants `-Wstack-usage=<bytes>`. Fixed.
- `-Wdangling-reference` "in `-Wextra` on GCC 15.2": confirmed with a dangling `std::max`
  reference (warns under `-Wextra`, not under `-Wall`).

Other tools: lizard 1.24 accepts `-w -C 15 -Tnloc=60 -a 5`. clang-format is not at
`/c/msys64/ucrt64/bin` on this box, so the pinned 22.1.8 wheel was installed in the venv. The 9.3
base block is identical to the repo `.clang-format`; with the additions, C++ gets inserted braces
and `public:` at column 0, and the JavaScript section parses.

## 3. Conflicts with the lead's rules and philosophy (fixed)

- Missing rules for the lead's explicit conventions. C++ had no rule for "clang-format pinned at
  22.1.8 with the repo `.clang-format`" or "braces on every if" (only JS-15 and JS-13 did), and
  `readability-braces-around-statements` was configured in 9.2 but mapped to no rule. Added 4.20
  Formatting: FMT-1 (MUST) Pinned clang-format and FMT-2 (MUST) Braces on every control
  statement (Bad: `pi/naviGATR/src/impl/resources/pros_usb_link.cpp:144`
  `if (fd_ >= 0) return true;`). Also added to the contents, the quick checklist, the 0.1 list of
  lead rules (0.3 now points to it), 9.3, and 11.6 (phase 3).
- PY-1 and the D-4 default said "no new non-C++ file enters `pi/naviGATR/`", which would also ban
  the PIPELINE.md files the lead asked for (PIPE-1). Now "no new file in a language other than C++,
  CMake or XML ... Markdown docs such as README.md and PIPELINE.md are not code".
- Phase 4 added `spectaGATR/README.md`, but DOC-1 says a folder with a PIPELINE.md and no build
  commands has no README (spectaGATR builds inside naviGATR). Dropped from phase 4 and 11.3. DOC-1
  now says the PIPELINE.md states the folder's intent (the lead's "directories carry a README
  stating intent").
- CMT-2 allowed "a noun phrase, or one short sentence". The lead's calibration says "a bare label
  is the target ... not a sentence". Now "a bare noun phrase by default (`# Test files.`), one short
  sentence only when a phrase cannot say it".
- CFG-3's Good code block modeled a narrating comment (`// Call site: one line of intent, errors
  carry the node path.`). Now `// Field table.` and `// Call site.`, with the explanation in the
  caption.
- D-1 dependence: the lead's 2026-09-28 rule is `-Wall -Wextra`, no `-Werror`, and it is D-1's
  default. 9.9 still gated "tier-1 warnings, warnings as errors (D-1)", and many rules name
  tier-1 flags. 9.1 now says flags beyond `-Wall -Wextra` wait for D-1 and are checked by review
  until then. The 9.9 row now says "the warning set and `-Werror` per D-1 (default: `-Wall -Wextra`,
  no `-Werror`)".
- D-25 dependence: HDR-2 (enforced by `IncludeBlocks: Regroup`) and JS-15 (a JavaScript section in
  the lead's `.clang-format`) relied on D-25 knobs without saying so, while D-25's default is
  "none". Both now name D-25.

No rule was found that contradicts the design principles (extract-don't-invent, no speculative
abstraction, degrade-don't-hang, narrow seam) or the requirements' philosophy (generic pipeline
with specific plug-ins, no over-generalization, standard INPUT/OUTPUT, readability first,
aliases, struct/class/functional, namespaces, folder structure, a PIPELINE.md per pipeline, the
naviGATR gripe). Remaining deviations from lead rules are already in section 12 as `(lead rule)`
decisions: D-1, D-4, D-19, D-25.

## 4. Internal consistency

- IDs: 343 rule headings (282 MUST, 54 SHOULD, 7 retired; one heading covers CPLX-1 to 5 and
  CPLX-10), 12 principles, 22 REV items. No duplicate IDs. Every retired rule points to its
  replacement. Every non-retired rule has a level, a `Scope:` and an `Enforce:` line. Every Scope
  token is from the 0.6 vocabulary. Every referenced rule, REV, D-, F- and B- ID is defined
  (`D-31`/`D-32` appear only in the revision notes as the critique's own numbering).
- Conflicts fixed:
  - OWN-1 recommended `std::shared_ptr<const T>` across threads, while RT-5 bans `shared_ptr`
    across PROS tasks. OWN-1 now says "(never across PROS tasks, RT-5)".
  - CMT-5 allowed 5 label lines plus a PIPELINE.md pointer, but its script warns over line 1 plus
    5. The pointer now counts within the 5.
  - NAM-2 and STRUCT-1 say `register_<stage>`, but the move keeps `register_target_resolution` and
    `register_publishers` (move map; stage folders `targets/`, `publishing/`). NAM-2 now has a
    Legacy line, and phase 5 renames them.
  - Appendix E's `XyzOutput` lacked the `[[nodiscard]]` that ERR-3 uses as its own example
    (`struct [[nodiscard]] XyzOutput`). Added.
  - Appendix B said every number names its source, but "3 quadrature encoders" had none. Now
    `kChannels` (`pico/aggreGATR/src/quadrature.h:11`).
- 9.2 enables 28 hygiene checks that no rule names (for example `modernize-use-nullptr`,
  `cppcoreguidelines-slicing`). A line now says they are general hygiene, cited by check name.
- Appendix PIPELINE.md drafts keep PIPE-2's section order and sizes (A.2 29 lines, B 69, C 93,
  D 49), and their relative `validation.md` links resolve from their target folders.

## 5. ASCII

`grep -nP '[^\x00-\x7F]'`: 0 before and 0 after. All added text is plain ASCII and wrapped at 100
columns.

## Remaining concerns

- Only the counts listed in section 1 were rechecked. Other survey numbers (for example 406
  `auto`, 124 missing braces, 369 of 437 files unformatted, 147/60 CCN counts) are taken from the
  surveys.
- 0.1 makes every MUST rule that cites an open decision a SHOULD until the decision is recorded,
  but the headings still say MUST (TYPE-1, TYPE-13, ALIAS-5, NAM-2, HDR-2, HDR-3, TEST-4, ERR-1,
  PY-1, XML-5, XML-7, CMT-11, DOC-14, JS-15 and others). This is consistent with 0.1, but a reader
  scanning headings sees MUST.
- DOC-3 and DOC-5 have no real Good example in the tree today, only "closest".
- The `bugprone-random-generator-seed` line (`brain_link_commands.cpp:158`) was not re-run with a
  compile database; the constructor line is where a default-constructed engine would be reported.
