# Critique: usability and application

Target: `coding-standard/CODING_STANDARD.md`, Draft 1 (3689 lines). `L123` is a draft line.
Code cites are at `ae457ee`; the Windows checkout shows `pi/naviGATR` as `pi/navigatr`.
Lens: can a teammate follow it day to day, and can the plan be executed as written.

Numbers (counted by script over the draft):
- 309 rules with a level: 281 MUST, 28 SHOULD, 0 MAY. About 120 MUST rules are enforced by
  review only.
- Scope tags: 0.6 defines 7 values; the rules use about 30 (`config`, `All docs`, `All builds`,
  `Brain apps`, `Pi mains`, `Brain shells`, `Pi contracts`, ...).
- 0 non-ASCII characters, 0 ` -- `. Audience words: "the user" 27 times, "per the survey" 49,
  "this session" 2, "memory" in D-1, D-4, D-19.
- About 15 rule pairs say the same thing twice (item 17); two of them have already drifted.

Items are ordered by how much they block. Each has: problem, then the fix.

---

## A. Blocking: the plan and the running restructure

**1. 11.2 phase order (L2985-3063) collides with the restructure that is running now.**
- Problem: Phase 1 reformats 369 files, inserts braces and rewrites CMake glue before Phase 2
  moves 189 naviGATR files. The restructure already runs in a worktree at `ae457ee`, and its split
  is line-anchored: `restructure/plan/split_plan.md` "Line numbers below are from ae457ee", and
  `split_system.py` cuts `system.cpp` by those ranges. Formatting first changes almost every line
  of 235 of 249 naviGATR files, so git rename detection fails for most moves and the split ranges
  stop matching. Phase 1.3 also edits `pi/naviGATR/CMakeLists.txt` and `tests/CMakeLists.txt`,
  which the move rewrites. Phase 1.6 (CI) comes last, so nothing gates phase 1.
- Fix: replace the phase list head with:
  ```text
  Phase 0  baseline; only the decisions that block phases 1-2 (D-5, D-8/D-31, D-28, D-29).
  Phase 1  safety net: CI runs today's builds and tests (Linux, MinGW, pio run) with
           today's flags. No new gates, no tree changes outside .github/.
  Phase 2  naviGATR restructure steps 1, 2, 2b (item 2), 3, in that order.
  Phase 3  mechanical: hygiene, .clang-format and the reformat, InsertBraces (item 34),
           build glue, tier-1 warnings, tidy fix-its; then turn on the new CI gates.
  Phase 4  structure elsewhere and the PIPELINE.md tree.
  Phase 5+ types and names, comments and docs, complexity, follow-ups (as today).
  ```
  Add to 11.1: "The restructure is pinned to ae457ee line numbers. Nothing that rewrites naviGATR
  files lands before restructure steps 1 and 2."

**2. 11.4: the restructure stops before it fixes the gripe. Add step 2b.**
- Problem: after steps 1 to 3 both "start here" functions still break STRUCT-4 and CPLX:
  - `main` (`pi/naviGATR/src/app/main.cpp:72` `int main(int argc, char** argv) {`) moves as is to
    `src/main.cpp`. lizard: 112 NLOC, CCN 25 (fail tier is > 100 NLOC). STRUCT-4 says
    "`main.cpp` is tiny (parse, build, run)". It also holds a SYS-5 string compare,
    `:118` `if (inline_mode && system->commandsType() == "brain_link") {`.
  - `System::build` (`pi/naviGATR/src/runtime/system.cpp:148`) moves verbatim to
    `core/build.cpp`: 171 NLOC, CCN 31, over both fail limits, still with `:30-33`
    `template <typename T> struct Tag {` and `:259` `const auto buildSlot = [&](...`.
  - commands, targets and publishing still have no `make_*`; Appendix C says "`System::build`
    slot" three times. `grep make_` still misses 3 of 7 stages.
  - STRUCT-1 (L402) allows "a stage built directly by the core has none", STRUCT-4 (L433) wants
    "each stage's `make_*`". They disagree until F4 lands.
- Fix: add to 11.4, before the PIPELINE.md work:
  ```text
  Step 2b, entry points (behavior-free):
  - F4: make_commands, make_targets, make_publishing in pipeline/<stage>/stage.cpp;
    Tag and buildSlot deleted; System::build reads as the 7 builders in run order.
  - main(): parse, build, runInline or runWorkers, printReport. printDiagnostics and
    printWorker move to cli/report.cpp; inspection startup is one function. main() under 30 lines.
  - Gates: A and C; B as the extraction gate in item 35.
  ```
  Mark F4 in 11.5 "done in step 2b". Delete "(a stage built directly by the core has none)" from
  STRUCT-1. Change the D-5 recommendation to "(a) plus step 2b; then F1".

**3. STRUCT-2 (L411) and the 11.4 tree disagree with `move_map_final.tsv`.**
- Problem: the draft uses `collectors/`, `resolvers/`, `publishers/` and says "When
  `move_map_final.tsv` exists it supersedes the TSV, and this summary must match it". That file
  exists (written before this draft) and uses `commands/command_collections/`,
  `targets/target_resolutions/` and `publishing/publishings/`
  (`restructure/plan/move_map_final.tsv:52`, `:74`, `:112`; also
  `restructure/logs/final_map_check.txt:4-10`). `publishings` is not a word. The tree, the
  move-map table, STRUCT-2's slot sentence, Appendix C and D-28 all disagree with the executed
  map.
- Fix: open D-29 (item 42) and sync one way. Generate the 11.4 tree and table from
  `move_map_final.tsv` (extend `restructure/scripts/move_map_tables.py`, which already builds the
  `move_map.md` tables) and add under them: "Generated from move_map_final.tsv. Change the map,
  not this table."

**4. NAM-2 (L887): the literal `make_<type>` rule does not compile for repeated type names.**
- Problem: `type="noop"` is registered in five stages (all under `pi/naviGATR/src/impl/`:
  `commands/register_commands.cpp:11`, `localization/register_localization.cpp:27`,
  `target_resolution/register_target_resolution.cpp:11`, `publishing/register_publishers.cpp:11`,
  `world_estimation/register_world_estimation.cpp:14`) and `type="brain_link"` in two
  (`register_commands.cpp:12`, `register_publishers.cpp:12`). The literal rule gives five `make_noop` and two `make_brain_link` in the one flat `navigatr`
  namespace (D-6). `registerOrDie` takes `Fn fn` by value (`runtime/register_all.h:34`
  `inline void registerOrDie(FunctionRegistry& functions, const char* key, Fn fn) {`), so
  `&make_noop` hands an overload set to a `std::function` parameter and does not compile without a
  cast. Separately, NAM-11 (L943) keeps `makeX` as a verb family while NAM-2 bans `makeNoopX`.
- Fix, NAM-2 first sentence: "A factory registered under an XML `type` is `make_<type>` when that
  type name is unique in the component, else `make_<type>_<stage noun>` (`make_noop_commands`,
  `make_brain_link_publisher`)." NAM-11: replace "`makeX`" with "`makeX` (unregistered helpers
  only; factories follow NAM-2)". Record as D-31, or add to D-8.

**5. SYS-15 (L324), CON-1 (L1240) and OWN-4 (L1208) contradict RT-5 (L1321) on the Brain.**
- Problem: all three have Scope "Pi, Brain" and prescribe `std::shared_ptr<const T>` snapshots.
  RT-5: "No `std::shared_ptr` crosses PROS tasks (no atomic reference counts on this toolchain)."
- Fix, SYS-15 second sentence: "Readers get an immutable snapshot: on the Pi a
  `std::shared_ptr<const T>` swapped under a mutex, on the Brain a copy taken under the owning
  task's lock (RT-5), or the newest value through a latest-wins slot." OWN-4: "Readers get a copy,
  or on the Pi a `std::shared_ptr<const Snapshot>`." Retire CON-1 (item 17).

---

## B. Contradictions inside the draft

**6. ALIAS-3 (L715) vs Appendix E and the TYPE-5 Good example.** ALIAS-3: "Every `std::function`
type that appears in a header ... gets an alias". Appendix E writes `std::function<void()> reset;`
in a header, as does the TYPE-5 Good cite (`contracts/sensor.h:30-37`). Fix: add
`using ResetFunction = std::function<void()>;` to `pipeline/framework/` (proposed), use it in
Appendix E, and tag the TYPE-5 cite "(before the alias)".

**7. ALIAS-1 (L697) vs ALIAS-5 (L737).** ALIAS-1 wants an alias for "a builtin type carrying a
domain meaning (an id, a unit, a code)"; ALIAS-5 says Pi, wire, firmware and config carry the
unit as a name suffix. Fix ALIAS-1: "(an id or a code; units follow ALIAS-5)".

**8. ERR-8 (L1140) vs CONST-5 (L1664).** ERR-8 bans positional braces for any multi-field result
struct; CONST-5 allows them up to 3 obvious fields. Fix: retire ERR-8; move its Bad example
(`pros_usb_link.cpp:79` `return {false, true, false, false, "USB reply expired"};`) to CONST-5.

**9. TEST-1 (L1973) vs TEST-4 (L1994).** TEST-1: "One flat `tests/` folder per component";
TEST-4 adds `tests/support/`. Fix TEST-1: "One `tests/` folder per component. Test files sit
directly in it; shared support sits in `tests/support/` (TEST-4)."

**10. CMT-11 (L1852) vs HDR-8 (L1629).** Both claim line 1. Fix CMT-11, append: "Generated files:
line 1 is the HDR-8 line." Retire the second sentence of STRUCT-7 (L453); CMT-11 already puts the
path on every file.

**11. CMT-5 (L1819) numbers.** Rule: "at most 5 label lines after line 1". Enforce: "warn when
the leading comment block is over 8 lines". Fix: "over 6 lines (line 1 plus 5)".

**12. NAM-1 table (L876).** "functions, methods | camelCase verb | `estimationCycle`": a noun
phrase. Fix: use `parseCommandLine` (`pi/naviGATR/src/app/main.cpp:75`
`navigatr::parseCommandLine(argc, argv, ...)`, trimmed).

**13. NAM-8 (L927) vs 10.3 have already drifted.** 10.3 accepts `cfg`, `dy`, `uy`; NAM-8's copy
does not. Fix NAM-8: "No abbreviations beyond the accepted list in 10.3." Drop the inline list.

**14. SYS-2 (L221) scope.** It requires `Input`/`Output` structs, declared outputs, a build context
and a make-function alias, but its Scope includes "Brain seams" and "the wire", which have none of
those. Fix: Scope "Pi stages". Add: "A Brain seam or a wire format is defined in exactly one
header: `investigatr/state_source.h`, `investigatr/planner.h`, `actugatr/motor_output.h`,
`communigatr/byte_port.h`, `translaGATR/frames.h`, `translaGATR/link_documents.h`."

**15. L134-135 "When two rules seem to conflict, the lower-numbered principle wins."** Rules do not
say which principle they serve, so this cannot be applied. Fix: "Two rules that conflict are a
bug in this standard. Raise it in the PR; until it is fixed, the rule with the narrower Scope
wins."

**16. 0.5 (L68) makes some rules inert without saying so.** The lead's earlier rules win until the
lead rules on a D- item, so TEST-4 (D-19), PY-1 (D-4), DOC-14 (D-11) and the DOC-1 README size
(item 43) are not in force yet. Fix, add to 0.1: "A rule that cites an open decision (D-n) is
SHOULD until that decision is recorded."

---

## C. Duplicates (the standard breaks its own P-11 and DOC-8)

**17. Keep one text per fact; the other becomes a pointer or is retired (0.2).**

| Keep | Retire or point | Note |
|---|---|---|
| SYS-15 | CON-1 | CON-1 already says "(SYS-15)" |
| CON-9 | SYS-11 | SYS-11 becomes "A new queue or handoff answers CON-9 in the PR." |
| WIRE-7 | SYS-12, REPO-8 | SYS-12: "A new wire message answers WIRE-7 in the PR." |
| SYS-14 | TYPE-6 | same two-uses test |
| DOC-9 | SYS-22 | status vocabulary |
| SYS-23 | TEST-9 | host build of non-adapter code |
| XML-8 | TEST-14 | every config loads |
| CPLX-4 | API-4 | parameter limit |
| CONST-5 | ERR-8 | item 8 |
| CMT-11 | STRUCT-7 second sentence | item 10 |
| CMAKE-7 | 4 intro, L545-548 | language level |
| CPLX-5 | JS-3 "about 600 lines" | size limit |
| CMT-5 | JS-14 "a few label lines" | header limit |
| NS-4 | TEST-15 `using namespace` sentence | test import |
| RT-4 | LOG-3 | never block output on target |
| section 8 | 0.8 "Reviewing, in addition" | item 28 |
| 10.3 | NAM-8 list | item 13 |

Result: about 15 fewer rules, and no second copy left to drift.

---

## D. Levels and checkability

**18. MUST inflation.** 281 of 309 rules are MUST, none is MAY, and about 120 MUST rules are
checked only by a reviewer's judgment. A reviewer cannot hold 120 judgment calls, so in practice
none of them binds.
- Fix, 0.1: "MUST: a tool checks it, or a reviewer can answer yes or no from the diff alone. The
  lead's own rules (comment style, ASCII, naviGATR C++ only, namespace braces, test naming) are
  MUST regardless. Everything else is SHOULD." Delete the MAY line and add: "Anything no rule
  covers is the author's call."
- Then move these to SHOULD (judgment calls): PIPE-6, TYPE-3 (keep the REV-5 question), TYPE-15,
  ALIAS-7, API-2, API-7, CPLX-7, CONST-3, DOC-2, DOC-8, OWN-6, NAM-12, CFG-8, JS-6, JS-12,
  CMAKE-12, SH-3, SH-6, PY-9. STRUCT-6 splits: "The coordinator hosts no feature" stays MUST (it
  can be checked against the list in 11.4); "one responsibility per file" becomes SHOULD.

**19. "About" and "a few" cannot be checked.** Replace:
- STRUCT-10 (L472) "more than about 12 source files" -> "more than 12 files directly in it";
- PIPE-6 (L521) "more than about 8 rows" -> "more than 8 rows";
- CPLX-6 (L1747) "about 25" -> "25";
- HDR-5 (L1617) "longer than a few lines" -> "longer than 5 lines";
- SH-5 (L2391) "about 150 lines" -> "150 lines";
- STRUCT-1 (L400) "`PIPELINE.md`: a few lines" -> "`PIPELINE.md` (PIPE-2 stage size)".

**20. PIPE-2 (L501) headings cannot be checked against the drafts it ships.**
- Problem: PIPE-2 requires 13 headings including "code map"; Appendix A has no `## Code map`;
  Appendix B has no Start here, Input, Output or Inside; Appendix C adds `## Brain profile`,
  Appendix D adds `## Code map`, `## Error budget`, `## Not here`; and a stage file "may be 10
  lines", which cannot hold 13 headings. `check_docs.py` "required headings" would fail all three
  drafts.
- Fix, PIPE-2 text: "Required, in order: `# <name>`, a purpose line, `Status:`, `## Start here`,
  `## Input`, `## Output`, `## Change points`, `## Failure behavior`. Optional: `## Stages`,
  `## Code map`, `## Inside` (required when a child PIPELINE.md exists), `## Rules`,
  `## Threads and rates`, `## Exceptions`, and any section the pipeline needs. Root and component
  files: at most 120 lines. Stage and sub-pipeline files: at most 40 lines." Add `## Code map` to
  Appendix A; add Start here, Input, Output to Appendix B.

**21. PIPE-1 (L492) sub-pipeline threshold.** "one per sub-pipeline (a plug-in or sensor group with
internal steps)": every multi-phase plug-in qualifies (CPLX-7 lists six phases in
`TrackingWheelMotion::run`), which means dozens of files. Fix: "one per plug-in folder that holds
one sensor group or one multi-file plug-in (`estimators/apriltag/`). A single-file plug-in is one
row in its stage's code map." Add to phase 4 the exact list, so `check_docs.py` has a fixed set:
root; `pico/aggreGATR`; `pi/naviGATR`; its 7 stages; `src/brain_profile`;
`estimators/apriltag`; `spectaGATR`; `brain`; `brain/communiGATR`; `brain/actuGATR` (16 files;
investiGATR and translaGATR keep a README only).

**22. XML-11 (L2364) is unsatisfiable until step 2b and sits at the wrong level.** It asks the
naviGATR PIPELINE.md for "XML element, the `make_*` that builds it, and its file". Three stages
have no `make_*` today; the `type -> make_<type>` table is per stage and PIPE-7 puts it one level
down. Fix: "The component PIPELINE.md maps each stage element to its `make_<stage>`. Each stage
PIPELINE.md maps each registered `type` to its `make_<type>` and file."

**23. Only `Legacy: deferred` rules get a ratchet (0.4, L61-63).** Many tool-checked MUST rules
already have violations and no Legacy line, so their checks are red on day one or quietly skipped:
- CFG-1: `tinyxml2` outside `config/` in `impl/publishing/field_documents.cpp`,
  `runtime/brain_profile_builder.{h,cpp}`, `runtime/system.cpp` (`:152`
  `tinyxml2::XMLDocument doc;`, which lands in `core/build.cpp`);
- TIME-1: 56 lines outside `app/` and `core/host_clock.h` read `steady_clock::now` or
  `HostClock::now` (grep; some are allowed self-timing);
- SYS-19 and STRUCT-8: `restructure/logs/final_map_check.txt` lists the post-move edges, e.g.
  `math/camera_model.h -> pipeline/resources/camera.h`, `pipeline/framework/slot_init.h ->
  pipeline/localization/contract.h`, 5 `debug -> core/system.h`, 4 `debug -> plug-in folder`;
- NS-4 (`pico/aggreGATR/src/main.cpp:16`), ERR-1 (`runtime/system.cpp:934`).
- Fix, 0.4 third bullet: "Every script, lizard and clang-tidy check starts from a baseline of
  today's violations, recorded in phase 0, that may only shrink. 11.6 lists only the deadlines by
  which a baseline must reach zero." Seed the SYS-19 baseline from `final_map_check.txt`, and add
  the math/config/framework edges to F2.

**24. 0.4 "Changed code" (L59) makes small fixes expensive.** With the CPLX fail tier on "new or
changed" functions, a one-line fix in `TrackingWheelMotion::run` (CCN 105, 310 NLOC) forces a
300-line split inside a fix PR, which REPO-7 then forbids mixing. Fix: "A changed function may not
get worse on any CPLX metric. It must meet the limits when it is new, or when its phase comes."

**25. Enforce lines a stdlib script cannot deliver, and one that fires on required code.**
- TYPE-1 "script (struct with non-trivial member functions)" and CPLX-6 "script (member count
  report)" need a C++ parser. Change to "review; report only".
- 9.8 `check_patterns.py` greps `new`/`delete` for OWN-2, but OWN-5 and TYPE-7 require `= delete`,
  and comments say "new". Add to 9.8: "`delete` matches only `\bdelete\s*(\[\s*\])?\s+[A-Za-z_(]`
  not after `=`; `new` only `\bnew\s+[A-Za-z_:]+\s*[({\[]`; comment and string text is skipped
  (same for `throw`)."
- CPLX-3 warn tier (nesting > 3) has no tool: lizard's nesting crashes (9.6) and clang-tidy has one
  threshold, used for fail. Say "warn: review".

**26. CPLX-5 is two limits under one ID** (L1734-1735, production and tests). `EXCEPTION(CPLX-5)`
is then ambiguous. Fix: the tests row becomes CPLX-10.

---

## E. Navigation and format

**27. Rules are bold paragraphs, so they have no anchors and no outline entries.** In a 3689-line
file, "(SYS-15)" cannot be clicked and the GitHub outline shows only section headings. Fix: every
rule becomes a heading with a 2 to 5 word title, e.g. `#### SYS-15 (MUST) One writer per state`;
Ctrl+F on `SYS-15 (` lands on it. Generate a one-line-per-rule index from those headings
(`tools/ci/rule_index.py`) so it cannot drift. Mechanical, one commit.

**28. The quick checklist is buried and doubled.** It starts at L92 after 60 lines of meta, and its
"Reviewing, in addition" block repeats REV-3, REV-4, REV-7 to 9, REV-16 and REV-17. Fix: put 0.8
right after a 5-line intro as section 0; keep only "Writing code"; last line "Reviewers: section 8".

**29. Scope tags cannot be filtered.** 0.6 lists 7 values; the rules use about 30. Fix 0.6: "Scope
values: All, C++, Brain, Pi, Pico, codec, viewer, tools, config, build, docs. A qualifier goes in
parentheses: `Brain (apps)`, `Pi (stages)`." Rewrite every Scope line to that set.

**30. Adoption material is mixed into the rule book, and it cites files the team cannot open.**
Sections 9 (424 lines), 11 (298), 12 (45) and Appendices B to D expire after adoption, and they
cite `restructure/` and survey files under `C:/Users/mbes1/.claude/`. The draft also never says
where the standard lives in the repo. Fix:
- Add to the header: "Home: `docs/contributing/coding_standard.md`, linked from `CONTRIBUTING.md`"
  (today `CONTRIBUTING.md` only points at the PCB guide).
- Split into "Part 1: rules" (sections 0 to 8, 10, Appendices A and E) and "Part 2: adoption" (9,
  11, 12, B to D). Only Part 1 ships to the repo. Section 9 shrinks to a table "check -> config
  file"; the configs themselves become the real files. Section 12 becomes decision records
  (DOC-7 already says so). Appendices B to D become the real PIPELINE.md files in phase 4.
- Add 0.9 "Changing this standard: a PR that edits this file. A new rule takes the next free ID;
  a retired rule keeps its ID. A rule change that adds violations updates the baseline in the same
  PR."

**31. Bug classes are cited but never defined.** SYS-16, SYS-17, TIME-7, CON-9, ERR-9, WIRE-5 and
WIRE-8 cite B1, B3, B4, B7, B10, B12, B16, and REPO-7 requires every fix to "name the bug class",
yet the list lives only in `survey/complexity-bugs.md:693-1034`. Fix: add 10.4 "Bug classes", one
row each, names from the survey: B1 clock-domain mixing; B2 sentinel read as data; B3 stale cached
state; B4 missing reset on identity change; B5 snapshot consistency; B6 locks, blocking I/O, task
lifecycle; B7 unbounded queue or FIFO where latest-wins was meant; B8 units, ranges, rounding; B9
name tables drifting from enums; B10 retries, dedupe, wrap; B11 liveness; B12 silent loss, unread
counters; B13 docs drifting from behavior; B14 timing-dependent tests; B15 target-only code paths;
B16 wire buffer bounds. The TEST-10 "Bug class it prevents" column uses these IDs. REPO-7 example:
`encoder deltas stay current after Pico frames stop (B3)`.

**32. ID collisions.** PIPE-7 (L529) "(survey finding F7)" vs follow-up F7 (11.5, one event call);
11.4 "decisions D1 to D16" (move_map.md) next to D-1 to D-28. Fix: delete "(survey finding F7)";
write "the move map's own decisions (move_map.md D1 to D16, not section 12)".

**33. Two path tenses with no marker (0.7, L84-89).** Citations are pre-restructure; rule text
already uses post-restructure paths (HDR-3 `pipeline/localization/contract.h`, ALIAS-8
`pipeline/framework/ids.h`, `time/time.h`). Fix 0.7: "Rule text uses post-restructure paths.
Bad/Good citations are at ae457ee. When step 1 lands, citations are rewritten through
`move_map_final.tsv` in one commit; line numbers stay at ae457ee."

---

## F. Application plan

**34. InsertBraces is not guaranteed behavior-free (9.3, L2726; 11.2 Phase 1.2).** clang-format's
own option docs warn that InsertBraces can produce incorrect code because it lacks semantic
information, and ask for review of its changes. Fix: "InsertBraces runs as its own commit after the
whitespace reformat, with gate A and a read of each of the 124 sites."

**35. 11.1 gates (L2973-2981) fit moves only.** Phase 3 renames (aggreGATR namespace, snake_case
fields) change every symbol name, and step 2b and phase 5 extract functions, so gate B fails by
design. Fix, add to 11.1:
```text
Gates by commit kind:
- move, file split:        A, B, C.
- rename:                  A; B after applying the rename map to the old symbol list.
- function extraction:     A; old symbols minus the split functions are a subset of the
                           new set; every new symbol is named in the commit message.
- logic:                   A plus new tests.
```

**36. No PROS gate.** Draft phase 2.4 (communiGATR header moves) and phases 3.3 and 3.5 (Brain
libraries and apps) touch PROS code, but 9.9 build-pros may be "an arm-none-eabi syntax-only
compile". Fix, add to
11.1: "Brain changes: both PROS apps build with `pros make` (or the 9.9 syntax check) before and
after; `brain/gatr2_brain.mk` differs only in moved paths."

**37. The config reader (Phase 3.4, L3035) is logic, not "types and names".** CFG-4 turns unknown
attributes into errors, error texts change, and defaults move into structs, so a config that loads
today may be refused. Fix: mark it "logic"; gate with XML-8 (every committed config loads) and the
existing error-path tests; one parser per commit, starting with `<TrackingWheel>`.

**38. 11.6 (L3237) is missing rules the restructure leaves violated.** With item 23 it only needs
deadlines. Add rows: STRUCT-4 and CPLX for `main` and `System::build` (step 2b); STRUCT-6 and
CPLX-6 (`System` keeps its Brain-profile members; 11.4 says `core/system.h` "stays about 489
lines"; F1); STRUCT-8 and SYS-19 (the `final_map_check.txt` edges; F2); CFG-1 (the config
reader and one XML writer, draft phase 3); STRUCT-10 (item 40).

---

## G. naviGATR restructure plan

**39. The move map, the system.cpp split and Appendices C and D are concrete enough to execute.**
The table counts match `move_map_final.tsv` per folder (framework 11, resources 16,
resources/resources 26, localization 12, ...), and the split table matches
`restructure/plan/split_preview/` line counts (build.cpp 300, run.cpp 370, ...). The gaps are
items 2, 3, 4, 22, 23 and 40 to 41.

**40. STRUCT-10 vs the approved layout.** Counted from `move_map_final.tsv`:
`pipeline/resources/resources/` holds 26 files, `pipeline/resources/` 16, `debug/inspection/` 13.
The reference layout breaks STRUCT-10 on day one, and D-28 recommends the doubled name without
this fact. Fix, D-28 options restated:
- (a) keep `resources/resources/` (the lead's literal rule) and add family subfolders from the
  current 26 files: `camera/` (9: `apriltag_c.*`, `camera_capture.h`, `cameras.*`,
  `libcamera_camera.*`, `tag_detectors.*`), `geometry/` (8: field map, robot frame map, target set,
  wheel geometry), `links/` (4: `serial_links.*`, `pros_usb_link.*`), `pico/` (2), `synthetic/`
  (2), with `register_resources.cpp` at the top;
- (b) the same families directly under `pipeline/resources/`.
- Recommend (a): it keeps the rule the lead stated for every stage and fixes the size.
  `debug/inspection/` at 13 gets a SHOULD waiver in its PIPELINE.md `## Exceptions`.

**41. No stage-level PIPELINE.md example.** Appendix A says a stage file "may be ten lines", but the
only drafts are a 60-line component file and a 70-line sub-pipeline file. Add as Appendix A2
(proposed; symbols verified at ae457ee, post-move paths, failure line left to fill):
````markdown
# Sensors

Turn resource outputs into timed measurements, one record per configured `<Sensor>`.

Status: host-tested, see [validation](../../../../../docs/validation.md)

## Start here
- `stage.cpp`, `make_sensors`: builds every `<Sensor>` through the registry

## Input
- `ResourceMap`, from resources

## Output
- `SensorMap`, to localization and world estimation

## Code map
| `type` | Built by | File |
|---|---|---|
| `pico_encoder_channel` | `make_pico_encoder_channel` | `sensors/pico_channels.cpp` |
| `pico_imu_channel` | `make_pico_imu_channel` | `sensors/pico_channels.cpp` |
| `camera_frame` | `make_camera_frame` | `sensors/camera_channel.cpp` |
| `attitude_channel` | `make_attitude_channel` | `sensors/attitude_channel.cpp` |

## Change points
- a new sensor: a file in `sensors/`, one line in `sensors/register_sensors.cpp`

## Failure behavior
- <from sensor_stage.h: what a sensor with a dead resource reports>
````
Sources: `runtime/sensor_stage.h:4-5`, `:101`; `core/records.h:102`, `:115`;
`impl/sensors/register_sensors.cpp:12-17`; `impl/sensors/pico_channels.h:41`, `:44`,
`camera_channel.h:21`, `attitude_channel.h:27`.

---

## H. Decisions (section 12)

**42. Add D-29: slot plug-in folder names.** (a) agent nouns `collectors/`, `resolvers/`,
`publishers/` (STRUCT-2 today; they match `register_publishers`, `BrainLinkPublisher`); (b) literal
plurals `command_collections/`, `target_resolutions/`, `publishings/` (the executed
`move_map_final.tsv`). Recommend (a): `publishings` is not a word (P-5). Blocks step 1.

**43. Add D-30: README size.** The lead's rule: "READMEs are short signposts (a few lines)". DOC-1
allows 40 lines and DOC-3 requires title, purpose, Status, pipeline link, one line per subfolder,
build and test commands, and links. Options: (a) 40 lines as drafted; (b) 20 lines, with build and
test commands moved to the PIPELINE.md or a how-to. Recommend (b).

**44. Add D-31 (or extend D-8): factory names that repeat across stages.** Item 4. It interacts
with D-6: with per-stage namespaces the plain `make_noop` would be legal again.

**45. D-4 (L3274) is missing the option that keeps the lead's rule.** The lead said naviGATR is C++
only, "literally zero other languages". The recommendation reverses that without listing the
files, and the count misses 2 JS tools. Fix: list them (`tools/calibrate_camera.py`,
`tools/test_calibrate_camera.py`, `tools/viewer/{capture_fixture,cdp,stub_server,viewer_tests}.py`,
`tools/install_service.sh`, `tools/perf/run_perf.sh`, `tools/viewer_smoke.sh`,
`tools/perf/perf_probe.js`, `tools/viewer/unit_tests.js`, all under `pi/naviGATR/`) and add option
(c): move them to a repo-level `tools/<tool>/`, with spectaGATR as the one named exception (its
files are embedded in the binary). Recommend (c) unless the lead prefers (b).

**46. D-1 (L3271) can avoid the CMake bump.** `CMAKE_COMPILE_WARNING_AS_ERROR` needs 3.24; every
CMakeLists says 3.16. Option: an option `GATR2_WARNINGS_AS_ERRORS` (default OFF, CI sets ON) that
adds `-Werror` to the `gatr2::warnings` target only, so vendored code and googletest are
untouched. Same result as (b), no version bump.

**47. Make section 12 a ballot.** Order the rows by what they block (now: D-5, D-8/D-31, D-28,
D-29, D-10; before phase 3: D-1, D-9, D-22, D-25; the rest later), end each row with "Default if no
answer: ...", and mark the ones the running restructure already baked in (D-10 path comments,
D-28 doubled names) "applied by the restructure; say no to change". D-29 is the exception: the map
and the draft disagree, so it needs an answer before step 1 lands. The lead can then answer
"defaults, except D-4".

**48. Add D-32: how far phase 5 goes.** 60 functions are over the CCN fail limit (CPLX table).
Splitting all of them is the largest single effort in the plan, and the lead has not seen the
thresholds. Options: (a) all 60; (b) naviGATR entry points and stage run functions first, the rest
when next touched. Recommend (b).

---

## I. Tone and audience

**49. Words that only make sense in this chat.** Replace "the user" (27 times) with "the lead";
"Memory says" (D-1), "(memory: ...)" (D-4) and "the memory rule" (D-19) with "the lead's rule of
2026-07-19"; "this session" (L2540, L3271) with the date. Delete "(per the survey)" (49 times): the
`path:line` cite is the evidence, and counts become "(count at ae457ee)". Optional: in `Why:` lines,
cite the principle (P-5) and keep the lead's verbatim quotes in requirements.md.

**50. A few phrases read as narration.** Plain versions:
- CMT-4 (L1815) "Good (wire layout, the one place long comments earn their space)" -> "Good
  (wire layout; long comments allowed here)".
- 11.4 (L3084) "The plan is decided and running; this section summarizes it and points at the
  authoritative files instead of re-planning it:" -> "Plan: `restructure/SPEC.md`. Summary:".
- 11.4 heading "(the answer to the gripe)" -> drop; the first line already quotes the gripe.

The rest of the draft is terse, ASCII and plain. Examples are short except CFG-3 and Appendix E,
which earn their length.
