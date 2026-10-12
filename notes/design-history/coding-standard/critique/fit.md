# Critique: fit to the user's philosophy and to this codebase

Target: `coding-standard/CODING_STANDARD.md` (Draft 1, 3689 lines). Lens: does it encode
requirements.md faithfully, do its examples exist, does it contradict the user's standing rules or
the platforms, and does it cause churn for its own sake. Read-only on the repo; tree at `ae457ee`.

What I checked:
- every `path:line` in the draft (226 primary references plus about 40 `:N` follow-ups) against
  the tree (scratch: `critic-fit/cites_out.txt`);
- the draft's naviGATR plan against `restructure/SPEC.md`, `restructure/plan/move_map_final.tsv`
  (exists now, 201 rows) and `restructure/logs/final_map_check.txt`;
- the 9.2 `.clang-tidy` with clang-tidy 22.1.8 (pip wheel in a scratch venv): `--verify-config`
  passes and every check name the draft cites exists; two threshold experiments (items 30, 31);
- the 9.3 JavaScript clang-format section with clang-format 22.1.8 on a copy of spectaGATR;
- the survey lizard CSV for the functions the gripe plan points newcomers at.

What holds up: the CPLX table equals survey section 1.6; `SetAttribute` really is 73 calls in
616 lines; the UNIT-3 -18000 bug is real (`telemetry.cpp` uses `std::lround` after `wrapAngle`);
the XML-6 BrainProfile blocks are byte-identical; 0 of 1427 gtest names have an underscore, so
TEST-2 costs nothing; the draft is plain ASCII.

Items are ordered by weight. Each has the rule id, the problem with evidence, and the fix.

---

## A. Conflicts with the user's own rules or the approved restructure

**1. STRUCT-2, 11.4 (tree, move-map table, "Restructure decisions"), D-28: slot folder names do
not match the final move map or the user's rule.**
- Problem: STRUCT-2 says "A slot selected by a `type` attribute on the slot element uses the agent
  noun (`collectors/`, `resolvers/`, `publishers/`)". The final map, which requirements.md says
  the summary must match, uses the literal plurals:
  `src/impl/commands/brain_link_commands.cpp	src/pipeline/commands/command_collections/brain_link_commands.cpp`,
  `.../targets/target_resolutions/...`, `.../publishing/publishings/...`
  (`restructure/plan/move_map_final.tsv`; also `final_map_check.txt`
  "src/pipeline/publishing/publishings/ <Publishing>"). The user's rule (requirements.md, 13:45)
  is literal: "apply the same rule to every stage ... using each stage's real XML element names."
  The 11.4 paragraph "agent nouns for slots (`collectors/`, `resolvers/`, `publishers/`,
  STRUCT-2)" is now false. Note `restructure/plan/move_map.md` D2 still says agent nouns, so the
  restructure folder disagrees with itself; the TSV is the file the requirements name.
- Fix:
  - STRUCT-2, replace the last sentence with: "The rule is literal for every plug-in folder,
    slots included: `<CommandCollection type>` gives `command_collections/`,
    `<TargetResolution type>` gives `target_resolutions/`, `<Publishing type>` gives
    `publishings/`."
  - 11.4 tree and move-map rows: `commands/command_collections/`, `targets/target_resolutions/`,
    `publishing/publishings/`. Delete the "agent nouns" clause from "Restructure decisions".
  - D-28: "The literal rule also gives `command_collections/`, `target_resolutions/` and
    `publishings/`." If agent nouns are still wanted, that is a new decision for the user
    (D-29), not a rule.
  - 0.7 and 11.4: cite `restructure/plan/move_map_final.tsv` directly; drop "When it exists".
  - Tell the restructure owner that `move_map.md` D2 is stale.

**2. PY-1, 0.3, D-4: the user's naviGATR language rule is misstated and pre-empted.**
- Problem: the user's rule is "navigatr (pi/navigatr) is C++ ONLY ... literally zero other
  languages; any exception must be justified to the user" with CMake and XML accepted
  (`memory/feedback_conventions.md:36`). The draft narrows it to "naviGATR `src/` is C++ only"
  (0.3, PY-1) and makes PY-1 a MUST that admits Python and shell under `tools/` while D-4 is still
  open. `src/` is not C++ only today: `pi/naviGATR/src/impl/resources/apriltag_c.c:1`
  `/* apriltag_c.c */`. `tools/` also holds JS (`tools/perf/perf_probe.js`,
  `tools/viewer/unit_tests.js`, `tools/viewer/unit.html`) and JSON test data, and the standard
  itself adds new file types there (CMakePresets.json by CMAKE-10, `tests/.clang-tidy` YAML, an
  ESLint config under D-21 option A).
- Fix:
  - 0.3: "... naviGATR is C++ only (CMake and XML accepted) ...".
  - PY-1: prefix "Pending D-4. Until the user rules, no new non-C++ file enters `pi/naviGATR/`."
  - D-4 option (b) lists every current and proposed exception with a one-line reason, recorded
    under `## Exceptions` in `pi/naviGATR/PIPELINE.md`: `spectaGATR/` (JS, HTML, CSS),
    `tools/` Python, shell and JS, `src/.../apriltag_c.c` (C shim over the vendored AprilTag C
    library), `tools/perf/testdata/*.json`, CMakePresets.json, `.clang-tidy`, the ESLint config.
  - Move D-4 to the top of 12.1; the JS-, PY- and SH- rules for naviGATR depend on it.
  - `check_patterns.py`: flag `.c` as well as `.py` under `src/`.

**3. TEST-4, TEST-1, D-19: shared test helpers placed where the user said they must not go.**
- Problem: the user's rule: "A `tests/` folder holds ONLY test mains - no headers, no shared
  helpers" and "one `tests/` folder per top-level component, never nested deeper"
  (`feedback_conventions.md:27`, `:30`). TEST-4 puts helpers in `tests/support/`, which is both a
  helper and a nested folder, and contradicts TEST-1 "One flat `tests/` folder". D-19 is filed as
  Minor although it overrides a standing user rule (0.5 puts user rules first).
- Fix: TEST-4: "Shared test support (fakes, XML fixture builders, polling helpers) lives in
  `<component>/sim/`, a sibling of `tests/`, listed explicitly; `tests/` keeps only
  `*_gtest.cpp` files." The Brain already works this way (`brain/communiGATR/sim/fake_pi.cpp`,
  tested by `brain/communiGATR/tests/fake_pi_gtest.cpp`). Move D-19 to 12.1 with this as
  option (a) and `tests/support/` as (b).

**4. CMT-11, STRUCT-7, D-10: a path on line 1 of every file is churn and restates the obvious.**
- Problem: 426 of 454 tracked first-party `.h/.cpp/.c/.js` files already start with
  `// <file name>` (the user's frames.h convention); only 7 use a path, all in
  `pi/naviGATR/src/payloads/` (`// payloads/field_object_evidence.h`). CMT-11 (MUST, all files)
  rewrites about 430 files, while STRUCT-7 (SHOULD) asks for the path only where a basename
  repeats. The comment rules say to state a format only where it is not self-evident.
- Fix: CMT-11: "Line 1 is `// <file name>`. A file whose basename repeats in its component
  (`contract.h`, `stage.h`, `stage.cpp`, `noop.h`, `noop.cpp`) uses the component-relative path
  instead (STRUCT-7)." D-10 recommendation: "Keep `// <file name>`; the path only for repeated
  basenames." Only restructured naviGATR files change.

**5. TYPE-5, 0.4, D-16: "convert when touched" makes interface changes a side effect of feature
work.**
- Problem: TYPE-5: "Existing virtual seams stay until touched for another reason." P-4 says a
  contract change is "never a side effect of feature work", and REPO-7 keeps rename and logic
  commits apart. Commands, localization (two seams), world estimation, targets and publishing
  are virtual classes today (for example `pi/naviGATR/src/contracts/publishing.h`
  `class Publishing`). D-16 ("existing `is*` renamed when touched") has the same shape.
- Fix:
  - TYPE-5 last sentence: "An existing virtual seam is converted only by its own approved change
    (F10), never because a feature touched the file."
  - 0.4 "Changed code": "The edited lines follow the rules. Renames, seam conversions and type
    changes that reach past the edited function are their own commits (REPO-7)."
  - D-16: "Existing `is*` names stay until a rename pass."

**6. STRUCT-1 scope "by analogy Brain and Pico": imports a plug-in shape the user ruled out.**
- Problem: `feedback_design_principles.md:15`: "no XML config, no plugin framework on the Pico".
  STRUCT-1's stage shape (contract.h, stage.cpp, plug-in folders, `register_*.cpp`) applied "by
  analogy" to the Pico invites exactly that.
- Fix: Scope: "Pi. Brain libraries follow STRUCT-9 and the Pico follows STRUCT-10; neither gets
  stage folders, registries or plug-in folders."

**7. D-25 is a change to the user's own `.clang-format`; D-1 omits the flag set.**
- Problem: the `.clang-format` knobs are a user decision (`feedback_conventions.md:19`); D-25
  adds five knobs as a Minor decision. D-1 asks only about `-Werror`, but 9.1 also adds about 20
  gate flags beyond the "`-Wall -Wextra`" the user named this session (requirements.md,
  Warnings).
- Fix: move D-25 to 12.1 and add the JS cost I measured: under the 9.3 JavaScript section
  (Google base, clang-format 22.1.8) 776 of 7884 spectaGATR lines change. Reword D-1: "Which
  warning flags, and are they errors?" Options: (a) `-Wall -Wextra`, no `-Werror` (the user's
  words); (b) the 9.1 tier-1 set, `-Werror` in CI only; (c) tier-1 with `-Werror` everywhere.

---

## B. The naviGATR gripe: the plan stops short

**8. 11.4, 11.5, STRUCT-4: the two files a newcomer is sent to still fail the standard after the
restructure.**
- Problem: 11.4 sends readers to `src/main.cpp`, then `src/core/build.cpp`. Both are pure moves.
  - `System::build` (`pi/naviGATR/src/runtime/system.cpp:148-351`) is 171 NLOC, CCN 31 (survey
    lizard CSV), over the CPLX-1 and CPLX-2 fail tiers, and still hides three stages behind
    `:259` `const auto buildSlot = [&](const char* slot_name, auto& target, auto make_tag,`.
  - `main` (`pi/naviGATR/src/app/main.cpp:72-195`) is 112 NLOC, CCN 25. It holds both run
    loops: the inline loop calling `system->step(navigatr::HostClock::now());` and the threaded
    loop at `:174-176` `system->applyPendingProfile();` ... `sleep_for(...milliseconds(20))`.
    STRUCT-4 says main is "tiny (parse, build, run)"; the 11.4 tree says so too.
  - Neither function is in Phase 5 or 11.5. The run side is fine: `step` 8 NLOC,
    `estimationCycle` 26.
- Fix:
  - F4 add: "`System::build` becomes the seven `make_*` calls in run order; each stage's XML
    reading moves into its `make_*`. Target: under 40 lines."
  - New F11: "`main` keeps parse, build, run, report. The inline loop and the profile-boundary
    loop move to `core/run.cpp` as `runInline` and `runThreaded`." Pure move plus one call site.
  - 11.6: add "STRUCT-4 | F4 and F11".
  - D-5 recommendation: "then F4, F11, F1, in that order: F4 and F11 move existing code and
    finish the gripe; F1 is a design change."
  - STRUCT-1: delete "(a stage built directly by the core has none)"; it contradicts STRUCT-4
    and F4. Until F4, cite it in 11.6.

**9. SYS-19, STRUCT-8, 11.6: the dependency order leaves out half the folders, and the reference
layout breaks it.**
- Problem: SYS-19's order ("framework and contracts, then stage builders, then plug-ins, then
  registration, then the composition root, then `main`") never places `math/`, `time/`, `io/`,
  `config/`, `brain_profile/`, `debug/` or `cli/`. `final_map_check.txt` lists 14 edges that
  break any sensible order, and none is deferred:
  - framework -> stage: `pipeline/framework/slot_init.h -> pipeline/localization/contract.h`;
  - math and config -> pipeline: `math/transforms.h -> pipeline/framework/ids.h`,
    `math/camera_model.h -> pipeline/resources/camera.h`,
    `config/field_map.h -> pipeline/framework/ids.h`;
  - debug -> plug-in folder (4), for example
    `debug/inspection/inspection_state.h -> pipeline/world_estimation/estimators/apriltag/tag_observations.h`;
  - earlier stage -> later stage (3), for example
    `pipeline/resources/resources/cameras.cpp -> pipeline/sensors/camera_frames.h`;
  - stage -> another stage's plug-in: `publishings/brain_link_publisher.cpp ->
    command_collections/brain_link_commands.h`;
  - stage level -> own plug-in folder outside a register file (2), for example
    `pipeline/resources/stage.cpp -> pipeline/resources/resources/serial_links.h`.
- Fix: SYS-19 text: "Dependencies point one way, bottom up: `math/`, `time/`, `io/` -> `config/`
  -> `pipeline/framework/` -> stage contracts and stage-level types (a stage may use an earlier
  stage's types, never a later one's) -> stage builders -> plug-ins -> `register_*.cpp` ->
  `core/` -> `brain_profile/` -> `debug/` -> `cli/`, `main.cpp`. Two headers are allowed
  downward: `debug/diagnostics/hub.h` (STRUCT-8) and `brain_profile/brain_profile.h`. Only
  register files, tests and tools include a plug-in folder from outside it." STRUCT-8 add:
  "Observers include no plug-in folder." 11.6 add: "SYS-19, STRUCT-8 | the 14 edges in
  `restructure/logs/final_map_check.txt` are the F2 baseline for `check_includes.py`."

**10. Appendix D, 11.4: world estimation's flow is drawn at stage level but runs inside the
group.**
- Problem: Appendix D shows
  `SensorMap -> estimator (one sensor group) -> FieldObjectPoseEvidence -> landmark estimation -> FieldState`
  as stage steps. In code the AprilTag group owns the fold:
  `pi/naviGATR/src/impl/world_estimation/apriltag_world_estimator.cpp:120`
  `estimator->landmarks_ = LandmarkEstimator::create(landmarks, field, err);`. After the move
  `landmark_estimator.*` sits at stage level, but `stage.cpp` never calls it; a reader who trusts
  the diagram searches the wrong file.
- Fix: Appendix D Stages block:
  ```text
  SensorMap -> estimator (one sensor group) -> FieldState, evidence
  inside the group: extraction -> association -> FieldObjectPoseEvidence -> LandmarkEstimator
  ```
  and a Rules line: "`landmark_estimator.*` is generic code the group calls; a second group
  calls it the same way."

**11. SYS-14, STRUCT-3: perception.h and association.h are "Bad" in one rule and generic in the
approved layout.**
- Problem: SYS-14 calls `contracts/perception.h` and `contracts/association.h` speculative (one
  private implementation each). STRUCT-3's generic list leaves them out. But SPEC.md:68 keeps
  "generic step contracts (perception, association)" and the final map puts them at stage level
  (`src/contracts/perception.h	src/pipeline/world_estimation/perception.h`). The user's model
  says association stays inside the sensor group (requirements.md, 13:25), and
  `pi/naviGATR/src/contracts/perception.h:2-4` calls itself "An internal step contract of a
  world estimator's subpipeline, never a coordinator slot" (line breaks removed).
- Fix: STRUCT-3 Good list: add "`perception.h`, `association.h` (generic today by SPEC; F12)".
  New F12: "Move `perception.h` and `association.h` into `estimators/apriltag/`, or collapse them
  into concrete classes (SYS-14 and the user's sensor-group model)." New D-30 for the user,
  since SPEC placed them.

**12. STRUCT-1, 11.4: register-file placement.**
- Problem: STRUCT-1 says a plug-in folder holds "implementations and their `register_*.cpp`";
  11.4 says "one `register_*.cpp` per plug-in folder". The final map puts
  `src/pipeline/localization/register_localization.cpp` at stage level (two plug-in kinds) and
  `register_world_estimation.cpp` in `estimators/`, above the group folders.
- Fix: STRUCT-1 bullet: "one `register_<stage>.cpp` per stage: in the plug-in folder when the
  stage has one plug-in kind, at stage level when it has two." 11.4: "then one
  `register_<stage>.cpp` per stage".

---

## C. Philosophy points the draft does not yet encode

**13. PIPE-2: "as concise as possible" and standard INPUT/OUTPUT are not what the check
enforces.**
- Problem: PIPE-2 requires 13 headings and `check_docs.py` checks "required headings", yet "A
  stage PIPELINE.md may be 10 lines", which cannot hold 13 headings. None of the three drafts
  follow the list: Appendix B has no Start here, Input, Output or Inside; C has no Code map; D
  adds Error budget and Not here.
- Fix: PIPE-2: "Required, in order: title, one problem line (P-1), `Status:`, `## Start here`,
  `## Input`, `## Output`. Every other Appendix A heading appears only when it has content.
  Root and component files: target 60, at most 100 lines. Stage and sub-pipeline files: at most
  30." This puts the standard INPUT/OUTPUT at the center of every pipeline doc. Update
  Appendix A and `check_docs.py` to match.

**14. PIPE-1: which folders are pipelines is not defined.**
- Problem: Appendix B links `brain/investiGATR/` and `translaGATR/` with no PIPELINE.md, and
  phase 2.2 omits them, so the rule and the plan disagree. One-file plug-ins with internal
  phases, such as `TrackingWheelMotion` (CCN 105, the worst function in the tree), have no folder
  under the flat plug-in folders of the move map, so "any existing pipeline within its pipeline
  must also do so" has no home for them.
- Fix: PIPE-1 add: "Pipelines: the repo root; `pico/aggreGATR`; `pi/naviGATR`, each stage under
  `src/pipeline/`, `src/brain_profile/` and each sensor-group folder; `spectaGATR`; `brain/`
  (link to motors); `brain/communiGATR`; `brain/actuGATR`. `translaGATR`, `investiGATR`, `bench`
  and `tools` are libraries or tools with a README. A plug-in becomes a sub-pipeline, with its
  own folder and PIPELINE.md, once it spans more than one .h/.cpp pair." DOC-1 add: "A folder
  with a PIPELINE.md and no build commands has no README."

**15. DOC-1: README size contradicts "short signposts (a few lines)".**
- Problem: `feedback_conventions.md:32`: "READMEs are short signposts (a few lines)." DOC-1
  allows 40 lines, and DOC-3 adds build and test commands.
- Fix: DOC-1 Signpost row: "target 10, at most 20 lines". DOC-3: the build and test lines are
  preset names (CMAKE-10), one line each; anything longer goes to `docs/`.

**16. New ALIAS-15: nothing limits `auto`, so the aliases never reach the call site.**
- Problem: production C++ (317 files in naviGATR `src/`, `brain/`, `translaGATR/`, `pico/`,
  tests and sims excluded) has 406 `auto`, for example
  `pi/naviGATR/src/capture/capture_bundle.cpp:354` `const auto it = current_.find(key);`
  (spacing trimmed). The user asked to "abuse" `using` so call sites read
  like the domain; `auto` hides the alias. The draft rejects `modernize-use-auto` but has no
  rule.
- Fix: "ALIAS-15 (SHOULD) `auto` only where the type is spelled on the same line
  (`std::make_unique<T>`, a cast, a named constructor), for iterators, lambdas and structured
  bindings. Everywhere else write the alias. Scope: All C++. Enforce: review."

**17. New API-8: P-5 (readability over speed) has no C++ rule.**
- Problem: only the viewer has one (JS-12: performance patterns need a measured number). The
  user: "if something is slightly slower but more readable, readable wins everyday".
- Fix: "API-8 (MUST) A performance pattern in C++ (cache, preallocation, lock-free structure,
  hand-unrolled loop, a copy avoided at a readability cost) names its measured number in a
  one-line label; otherwise write the plain version. RT-1, RT-2 and RT-10 are target
  correctness rules and exempt. Scope: Pi, Brain. Enforce: review."

**18. CFG-3: a new generic abstraction without its second use, scheduled late.**
- Problem: SYS-14 and P-6 require two real uses converted in the same change; CFG-3 promises the
  table "drives XML writing" before any writer exists. requirements.md asks to introduce the
  reader "early, since it shrinks the most code", but 11.2 puts it in phase 3.4, after phases 1,
  2 and 3.1 to 3.3.
- Fix: CFG-3 add: "The reader lands with two real parsers converted in the same series
  (TrackingWheel plus one more, for example the `Freshness` read at
  `attitude_channel.cpp:57`). The writer half lands only with `brain_profile_builder.cpp`."
  11.2: move "the config reader" to phase 2, right after restructure step 3, for naviGATR.

**19. D-6 rationale, CFG-3 namespace: "everything namespaced appropriately" gets a false
premise.**
- Problem: D-6 keeps one flat `navigatr` because "class names are already stage-prefixed". The
  localization plug-ins are not (`TrackingWheelMotion`, `WeightedPlanarFusion`,
  `PlanarMotionIntegrator`, `StationaryWindow`), nor are framework types. The CFG-3 example
  already writes `config::readAll(...)`, a nested naviGATR namespace D-6 does not allow.
- Fix: D-6 rationale: "(a) for now: nesting touches every file; revisit after F5." Add option
  (c): "nest shared helpers by role (`config::`, `json::`); stage code stays flat." Recommend
  (c) if CFG-3 keeps `config::`.

---

## D. Contradictions inside the standard

**20. ALIAS-1 vs ALIAS-5.** ALIAS-1 aliases "a builtin type carrying a domain meaning (an id, a
unit, a code)"; ALIAS-5 and D-14 keep Pi units as name suffixes. Fix ALIAS-1: "(an id, a code,
or a unit where ALIAS-5 gives the component unit aliases)".

**21. ALIAS-3 vs TYPE-5 Good and Appendix E.** ALIAS-3 aliases every `std::function` in a header,
but TYPE-5's Good (`pi/naviGATR/src/contracts/sensor.h:33`, `:36`) and Appendix E use a raw
`std::function<void()> reset;`, as does `contracts/resource.h:51`. Fix: Appendix E uses
`using ResetFunction = std::function<void()>;` from `pipeline/framework/`, and TYPE-5's Good
notes "(the raw `std::function` members still need ALIAS-3 aliases)".

**22. TYPE-15 forbids the CFG-3 reader.** "Templates only for containers, strong types and
type-safe erasure" excludes `config::Fields<TrackingWheelConfig>` with member pointers and the
existing `registerOrDie<SensorMakeFunction>` (`pi/naviGATR/src/runtime/register_all.h:34`). Fix:
"Templates live only in shared helpers named by role (containers, strong types, type-safe
erasure, the config field table, registration). Feature and stage code defines no templates; no
tag dispatch or generic lambdas where two or three explicit calls would do."

**23. WIRE-3 vs NAM-5 and its own example.** WIRE-3 demands "`enum NameCode : uint8_t`" and
cites `translaGATR/frames.h:22` `enum FrameType : uint8_t {` as Good. Read literally it renames
every wire enum in three components; NAM-5 wants `Code` only for a wire twin of a domain enum.
Fix WIRE-3: "Wire code enums are plain `enum Name : uint8_t` (the `Code` suffix only for a wire
twin, NAM-5) ...".

**24. CMT-9 vs CMT-4's Good example.** CMT-4 cites `translaGATR/frames.h:32-36` as the model wire
layout; line 32 is `// -------...`, a banner CMT-9 bans. frames.h has 11 banners (translaGATR
65, communiGATR 48, naviGATR about 110), and it is the user's reference file
(`feedback_conventions.md:17` "like the existing frames.h"). Fix CMT-9: "(SHOULD, new code) No
section banners in a file of functions; a wire-layout header may separate message layouts with
one rule line each. Existing banners go only where the file is split anyway."

**25. SYS-2, SYS-4 reshape existing contracts with no deferral.**
- SYS-4 (MUST) "a status and a diagnostic string": `CommandsOutput`
  (`pi/naviGATR/src/contracts/commands.h:28`), `TargetResolutionOutput`
  (`target_resolution.h:41`) and `PublishingOutput` (`publishing.h:40`) have no `diagnostic`.
- SYS-2 wants one build context per stage; every slot shares `SlotInitializationContext`
  (`contracts/slot_init.h:41`), and localization has two seams.
- Fix: SYS-2 wording "its seam or seams, its build context (the shared
  `SlotInitializationContext` counts)". 11.6 add: "SYS-4 diagnostic on commands, targets,
  publishing outputs | F10".

**26. API-4 vs CPLX-4; JS-3 vs CPLX-5.** API-4 is MUST at more than 5 parameters while CPLX-4
makes 6 a warn (19 production functions have 6). JS-3 is MUST at "about 600 lines" while CPLX-5
warns at 600. Fix: API-4 "(SHOULD) More than 5 parameters become an input struct; more than 7 is
a MUST (CPLX-4)". JS-3 "A module over the CPLX-5 fail size, or holding two panels, is split."

**27. XML-5 vs the appendices: the season folder is spelled three ways.** XML-5 has
`config/seasons/<season>/`; Appendices B and C and the glossary use `config/<season>/field.xml`;
the tree has `config/override/field.xml`. Fix: `config/seasons/<season>/field.xml` everywhere
(D-23).

---

## E. Churn for its own sake

**28. TYPE-2: the pimpl clause.** `struct HttpServer::Impl`
(`pi/naviGATR/src/inspection/http_server.cpp:401`) is read through `impl_->` 53 times from
`HttpServer`'s own methods (`:1393` `impl_->config  = std::move(config);`). A class with no
public data needs accessors or `friend` for each: more code, same invariant. Fix TYPE-2: "A
pimpl `Impl` defined in the .cpp is the owner's private state and stays a `struct`." Drop the
Bad example.

**29. D-24: keep `.xml.in`.** Six tracked templates use it
(`pi/naviGATR/config/shared/robots/gatr2_as5047_imu.xml.in` and five more) and the loader already
refuses it: `pi/naviGATR/src/config/composition.cpp:198` `if (endsWith(normalized, ".xml.in")) {`.
Nothing runs `configure_file` on them. Fix: D-24 recommends keeping `.xml.in`.

**30. D-20: keep `window.__navigatr`.** Renaming touches the viewer and its Python tests for a
page-local hook that names the runtime it talks to, like `navigatr.inspect/2`, which D-20 keeps.
Fix: drop D-20 and the `__spectagatr` clause in JS-5.

**31. PY-1 "one folder per tool".** It moves single-file scripts (`tools/install_service.sh`,
`tools/viewer_smoke.sh`). Fix: "A tool with more than one file (its test counts) gets
`tools/<tool>/`; a single-file script may sit in `tools/`."

---

## F. Thresholds and enforcement vs the survey

**32. CPLX-3: clang-tidy's nesting gate is one level stricter than the table.** Measured with
clang-tidy 22.1.8: `readability-function-size` counts the function body as level 1, so four
nested `if` blocks give "nesting level 5 starts here (threshold 4)". NestingThreshold 4 therefore
fails at survey depth 4 (the warn tier, 32 functions), not above 4 (15). Lambdas count as a
level, as in the survey heuristic. Fix 9.2: `readability-function-size.NestingThreshold: 5`.
Verified: at 5, a lambda plus three `if`s passes and five nested `if`s fail. Report the warn
tier at 4 separately.

**33. CPLX-2 is gated by two tools that count differently.** clang-tidy's LineThreshold counts
"lines including whitespace and comments" (measured: a function lizard counts as 9 NLOC is
reported as 20 lines), while lizard gates NLOC. At 100 each, clang-tidy fails functions lizard
passes. Fix 9.2: `readability-function-size.LineThreshold: none` and `ParameterThreshold: none`
(both parse in 22.1.8). lizard owns NLOC, CCN and parameters; clang-tidy owns nesting. Update the
CPLX Enforce line.

**34. PY-7 and 9.6: Python is gated at the warn number and not at all for size.** ruff
`max-complexity = 15` is an error in the blocking python job, so Python fails at CCN 15 while C++
and JS fail at 25. ruff has no NLOC, parameter or file-length gate, yet CPLX says "Python uses
the same numbers through ruff". `pi/naviGATR/tools/viewer/viewer_tests.py` is 1131 lines, over
the fail tier, and nothing sees it. Fix: run lizard over `*.py` with the same two commands, drop
`C90` from ruff (or set 25), and include `.py` in the file-length script.

**35. 11.2 phase 0: a path-keyed baseline will not survive the move.** Phase 0 takes the CPLX
baseline at `ae457ee`, and then the restructure (already running, ahead of phases 0 and 1) moves
189 of 201 naviGATR files. Fix: "Take the CPLX, warning and exception baselines after
restructure step 3 lands. Key the CPLX baseline by component plus qualified function name, not
by path." Name the diff-aware wrapper that gates new or changed functions in 9.8
(`check_complexity.py`).

**36. STRUCT-10 flags the reference layout.** After the move `src/pipeline/resources/resources/`
has 26 files (14 .h/.cpp units), `src/pipeline/resources/` 16, `src/debug/inspection/` 13;
`brain/communiGATR/include/communigatr/` has 16. Fix: "A folder with more than about 15 units (a
.h/.cpp pair is one unit) is split by concern." Note `resources/resources/` is at 14 and name
D-28's family folders (`links/`, `cameras/`, `pico/`, `field/`) as its split.

**37. Enforcement lines that do not enforce.**
- TYPE-4 cites `readability-convert-member-functions-to-static`, which suggests making methods
  static, the opposite of TYPE-4. Fix: Enforce: "review; `check_patterns.py` (a class or struct
  whose every member function is static)".
- ERR-3 cites `bugprone-unused-return-value`, which checks a fixed list of standard functions
  and types, not whether project functions carry `[[nodiscard]]`. Fix: Enforce: "compiler
  (`-Wunused-result` once the attribute is there); review for the attribute".
- RT-1's "link-time trap on `malloc` in the Pico build" also traps the arduino-pico core linked
  into the same image (not verified here: the framework package is not installed on this box).
  Fix: "`arm-none-eabi-nm` over our own objects in `.pio/build/<env>/src/` finds no `malloc`,
  `free`, `_Znwj`, `_Znaj`."

**38. ERR-1: its Bad example is a live crash path that no phase fixes.**
`pi/naviGATR/src/runtime/system.cpp:934` `throw std::runtime_error(err);` is reached from
`pi/naviGATR/src/app/main.cpp:174` `system->applyPendingProfile();` with no `try`. Fix: 11.2
phase 3.4, as a separate logic commit with a test: "a `resumeWorkers` failure becomes a status
and a fault event; `main` exits 1 cleanly."

---

## G. Citation fixes

Everything else I opened matched its quote.

**39.** TYPE-5: "four noop classes ... (`:11`, `:37`, `:47`, `:55`)". There are five; add
`pi/naviGATR/src/impl/noop/noops.cpp:19` `class NoopStateEstimator : public StateEstimator`.

**40.** TYPE-1 Good: `PublishingInput` is described as "const views and nothing else", but it
has `MonotonicTime now;` and `Diagnostics* diagnostics = nullptr;`
(`pi/naviGATR/src/contracts/publishing.h:36-37`), which SYS-3 cites as Bad. Replace with
`pi/naviGATR/src/contracts/field_estimation.h:50` `struct FieldEstimationOutput {` (plain fields
with status and diagnostic).

**41.** CMT-2's second Good, `translaGATR/frames.h:316` `// 0 in HELLO`, is the same line ALIAS-2
cites as Bad, and WIRE-6 replaces that comment with `kNoSession`. Drop it; `# Test files.`
already covers the rule.

**42.** TYPE-3's second Good, `brain/investiGATR/include/investigatr/motion_model.h:45`, carries
the `const char** why` that ERR-2 retires. Keep only `actugatr/telemetry.h:23`.

**43.** XML-6 cites `brain_profile_usb.xml:55-86` and `brain_profile_rs485.xml:56-87` without a
directory, against 0.7. Write
`pi/naviGATR/config/override/brain_profile_usb.xml:55-86` and
`pi/naviGATR/config/override/brain_profile_rs485.xml:56-87` (verified identical, 32 lines).

**44.** PIPE-9's label `// Pipeline: see pi/naviGATR/PIPELINE.md` reads as an instruction; the
user's calibration wants a bare label. Fix: `// Pipeline: pi/naviGATR/PIPELINE.md`.

---

## Decisions this critique adds or moves

- Move to 12.1 (they override standing user rules): D-4 (first), D-19, D-25.
- Reword D-1 to cover the flag set (item 7).
- New D-29: slot folder names, literal (final map, the user's rule) or agent nouns (item 1).
- New D-30: `perception.h` and `association.h`, generic (SPEC) or inside the AprilTag group
  (item 11).
- Change recommendations: D-10 (item 4), D-16 (item 5), D-20 (item 30), D-24 (item 29), D-6
  option (c) (item 19).
- New follow-ups: F11 (main, item 8), F12 (step contracts, item 11); extend F4 (item 8).
