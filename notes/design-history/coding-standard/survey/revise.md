# Revision of CODING_STANDARD.md (draft 1 to draft 2)

Target: `coding-standard/CODING_STANDARD.md`, revised in place (5014 lines, plain ASCII, 0
non-ASCII bytes). Backup of draft 1:
`C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/standard/revise/CODING_STANDARD.orig.md`.
The standard's own "Revision notes" section (end of file) records every conflict choice.

## Key finding before editing

`restructure/SPEC.md`, `restructure/plan/FINAL.md` and `move_map_final.tsv` changed at 15:04-15:05,
after all three critiques. The lead's correction at the top of FINAL.md sets plug-in folder names
(readable plural nouns: `channels/`, `collectors/`, `observation_models/`, `resolvers/`,
`publishers/`; resource families `cameras/`, `tag_detectors/`, `pico/`, `links/`, `geometry/`,
`synthetic/`; no stutter). This overrides both fit 1 (literal plurals) and usability 3/42 (agent
nouns). STRUCT-2, 11.4 (tree and move-map table, re-derived from the current TSV) and D-28 follow
the correction.

## Applied

Fit (44 items): all applied except as noted under Declined. Main ones: 0.3 and PY-1 restate the
lead's "C++ only" rule, PY-1 pending D-4 with the full file list; TEST-4 moves shared test support
to `<component>/sim/`; CMT-11 keeps `// <file name>`; TYPE-5/0.4/D-12/D-16/F10 no "convert when
touched"; STRUCT-1 scope excludes Brain and Pico; D-1 and D-25 reworded as lead-rule decisions with
costs; step 2c (F4 `make_*` for every stage, F11 `main` split); SYS-19 full layer order; Appendix D
flow fixed; F12/D-29 for perception/association; register-file placement; PIPE-1 pipeline list;
DOC-1 README 20 lines; ALIAS-15 (`auto`); API-8 (performance needs a number); CFG-3 two parsers,
writer later, reader moved to phase 2; D-6 option (c); ALIAS-1/5, ALIAS-3/Appendix E, TYPE-15,
WIRE-3, CMT-9, SYS-2/SYS-4, API-4, JS-3, season path; TYPE-2 pimpl; D-24 keep `.xml.in`; D-20
retired; PY-1 single-file tools; NestingThreshold 5; LineThreshold/ParameterThreshold none; lizard
for Python; baselines after step 3 keyed by name; STRUCT-10 units; TYPE-4/ERR-3/RT-1 enforcement;
ERR-1 crash fix in phase 2; citation fixes 39 to 44.

Coverage (53 items): all applied. clang-tidy options (FixMode None, unused-return-value types,
exception-escape entries, mt-unsafe posix, macro-usage, g_ prefix), tests overlay keeps
use-internal-linkage, CONST-7 always-defined build macros, CON-5 SPSC ring exception, CON-11,
TYPE-15 callable and integer templates, RT-5 toolchain facts, ERR-6 reported check, new TYPE-16,
SYS-24 to SYS-27, RT-14, RT-15, CON-10, LOG-5, PIPE-10 (Appendix B numbers name constants),
TIME-4 unset domain, TIME-10, TIME-11, CON-9 OS buffers, new 4.19 NUM-1 to NUM-6, OWN-9, OWN-10,
ERR-1 non-throwing forms, ERR-11, P-9, API-9 (was proposed API-8), WIRE-9, REPO-9, RT-16 (was an
RT-13 append), TEST-6 timeouts and shuffle, TEST-19, TEST-10 rows, TEST-17, TEST-20, TEST-21 (was a
TEST-11 append), 0.3 pragma and clang-format exception forms, RT-3 per core, checklist and REV-19
to REV-22. The revised 9.2 config passes clang-tidy 22.1.8 `--verify-config`.

Usability (50 items): phase order (safety net, restructure, mechanical, structure, types, docs,
complexity, follow-ups); step 2c; NAM-2 repeated type names (D-8); SYS-15/OWN-4 vs RT-5; ERR-8
retired into CONST-5; CMT-5 6 lines; NAM-1 example; NAM-8 points to 10.3; SYS-2 scope; conflict
rule in 0.8; open decisions make a rule SHOULD; duplicates retired or pointed (CON-1, TYPE-6,
SYS-22, ERR-8, LOG-3, TEST-9, TEST-14; SYS-11, SYS-12); MUST/SHOULD definition and 19 demotions;
STRUCT-6 split (new STRUCT-13); exact numbers; PIPE-2 checkable headings; XML-11 two levels;
baselines for every check (0.4) and 11.6 as deadlines; changed functions may not get worse;
enforcement lines a script can deliver; CPLX-10; every rule a heading with a title; checklist
first, writing half only; scope vocabulary normalized; Home line, Part 1/Part 2, 0.8 changing
the standard; 10.4 bug classes; ID collisions; path tenses in 0.7; InsertBraces own commit;
gates by commit kind; PROS gate; config reader as logic; 11.6 rows; A.2 stage example (Sensors,
symbols verified at ae457ee); ballot-style section 12 with defaults; D-30 phase-7 extent; "the
user" to "the lead", survey mentions removed from Part 1; narration phrases.

## Declined or changed, with reasons

- Fit 1, usability 3/42/40 (folder names, D-28, D-29 slot names): superseded by the lead's 15:05
  correction.
- Usability 9 (`tests/support/`): conflicts with the lead's rule that `tests/` holds only test
  files; fit 3's `sim/` kept.
- Usability 10, second half (retire STRUCT-7's path sentence): fit 4 chosen; the path stays only for
  repeated basenames.
- Fit 16 level (SHOULD for ALIAS-15): made MUST; it is checkable from the diff.
- Coverage 16 order (InsertBraces before the reformat): after, so the commit holds only braces.
- Usability 17 for API-4, REPO-8, STRUCT-7: kept; each carries content the other rule lacks.
- Usability 43 (README size as a decision): not opened; the lead's own rule decides it.
- Usability 30 (physically splitting Parts 1 and 2): declared in the header instead, so section
  numbers and references stay valid.
- Usability 3 (generate the move-map table by script): table synced by hand; the text says the
  TSV wins.
- Usability 49 optional (cite principles instead of the lead's quotes): not done.
- Coverage 22 (note on the operaGATR function-local static): not added; no race shown.
- Exact thresholds chosen between critics: PIPE-2 stage files at most 50 lines (fit 30, usability
  40; Appendix D needs the error budget); HDR-5 10 lines (usability 5, coverage 15); STRUCT-10 15
  units (fit) instead of 12 files (usability).
- Numbering adjustments: usability's "step 2b" is step 2c (FINAL.md uses 2a and 2b); proposed
  decisions renumbered to D-29 (step contracts) and D-30 (phase 7 extent).
