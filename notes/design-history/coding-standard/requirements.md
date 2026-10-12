# Coding standard request (2026-09-28, user's words)

1. "Also while you're at it go ahead and do that preliminary code review of shit. You should primarily be looking for bugs, overly complicated shit that could be refactored (including systems!), where can use using especially making it more readable for the user, improving documentation to be more industry, etc..."
2. "btw this is OF THE ENTIRE SYSTEM"
3. "actually I think its a better use of time of deeloping a rule for coding that we'll then apply to the entire system"
4. "also we can lowk ignore those workflows for now we'lll rework them based off of how we'd like do it right if that makes sense?" (read as: the current CI workflows in .github/workflows are not a constraint; CI gets rebuilt around the standard)

Deliverable: a coding standard (rules) for the whole system, to be applied to the entire codebase afterwards. Nothing in the repo changes yet.

It targets:
- bug-prone patterns;
- unnecessary complexity in code and in whole subsystems;
- readability, especially `using` aliases and named types;
- documentation to industry standard.

## Standing user rules the standard must respect or explicitly flag
- Comment style (memory feedback_comment_style.md):
  - plain ASCII, no emojis, no em dashes, no symbols off a standard keyboard;
  - terse, self-describing labels;
  - not AI-ish, no over-explaining, casual grammar fine.
- Conventions (memory feedback_conventions.md):
  - C++17; clang-format pinned at 22.1.8 with the repo `.clang-format`;
  - braces on every if;
  - Allman braces for namespaces only, contents unindented, with closing comments;
  - tests named `*_gtest.cpp`, one `tests/` per component, listed explicitly in CMake (no globbing);
  - READMEs are short signposts and deep docs go in `docs/`; no empty placeholder files;
  - vendor small third-party libs in `third_party/`;
  - naviGATR is C++ only (CMake and XML accepted; Python tooling there needs justification).
- Warnings:
  - memory says -Werror everywhere;
  - this session the user said to use -Wall -Wextra with no -Werror. Flag this as a decision.
- Design principles (memory feedback_design_principles.md):
  - interfaces extracted, not invented; no speculative abstraction;
  - consumers degrade instead of hanging; the narrow seam is the product.
- Memory notes on namespaces (`gatr2`, `chomp`) are outdated: the code now uses `translagatr`, `communigatr` and others. Verify against the code.

## The user's philosophy (2026-09-28, verbatim; this drives the standard and overrides generic section guidance)

"for now write an in depth shit in there you think is good and vaguely matches what I said here

The overarching goal of this system is it MUST be reusable through seasons, the general pipeline must NOT be too specific unless ABSOLUTELY required however if that is required you are likely doing something wrong. Implementations within the pipeline of course have to be specific (e.g sensors, etc..) however where they live and work in must be generic to survive seasons

With that being said do NOT over generalize beyond what is absolutely required

For example, NaviGATR the biggest part of development is TRULY understanding your problem, what is this truly for?
* custom electronics allow for custom pcb shapes, mechanical and electronics now go hand in hand rather than mechanical always having to build around electronics
* this acts as the interface for it
* but also what do we need from these electronics?

that's where I realized oh we always need to know where we are, and generally where something else is to line up, hence the whole separation of the two, etc.. and this allows for expansion into more general computer vision shit and other things like tof sensors etc... into the field correction right but at its core its asking "where am I, where are things that are around me" which are coupled only in the sense if you want to translate "things around me" to absolute positioning you must use your localization which assumed some initial position right....

but again same thing with everything thats a systems design, however the systems existing currently likely ought not to change, and they are ought to have standard INPUT/OUTPUT that way different teams can work inside of them.

Readability is king, if something is slightly slower but more readable, readable wins everyday, especially if we're throttled by our sensors anyways.

Wait I didn't finish so things like

using aliasing is great
use struct for attribute only things
use class for attribute + methods

avoid using classes for the sake of, truly ask if this is necessary? can I use a functional approach?

use namespaces appropriately in general everything is namespaced

a system should also be almost as clear as possible simply from its folder structure, a folder structure + general knowledge of it should give a very clear picture, and each new pipeline ought to have an explanation that is as concise as possible, as an md for each, e.g navigatr shall explain its entire pipeline and any existing pipeline within its pipeline must also do so as well.

In fact the entire system ought to also explain its pipeline

But again the idea is standard IO, readability, etc...

etc...

also this is not a document thing but more of a gripe, fucking navigatr is hard ash to read its not clear here the main pipeline is or where the make pipeline shit is (if you even get that if not then fuck we're cooked because then its really unclear)"

### What this means for the standard (lead's reading)

**Ordered principles:**
1. Understand the problem first.
2. Season-reusable generic pipelines with specific implementations plugged into them, but no over-generalization.
3. Existing systems keep their shape; each has a standard INPUT/OUTPUT contract so teams can work inside it.
4. Readability beats performance unless a measured need says otherwise.

**Concrete rules the user asked for:**
- `using` aliases liberally.
- `struct` for data-only types; `class` only for data plus behavior (invariants and methods).
- Prefer free functions and a functional approach over classes made for their own sake.
- Everything is namespaced, appropriately.

**Structure and pipeline docs:**
- A system must be understandable from its folder structure.
- Every pipeline, and every sub-pipeline inside it, has one concise PIPELINE doc (.md) explaining it; the whole system has one too.

**The naviGATR gripe (feed the application plan):**
- It is hard to find the main pipeline and the make_* pipeline construction.
- The standard needs rules that prevent this, and a concrete restructure/doc plan for naviGATR in "Applying this to the existing system".
- Behavior does not change during that restructure.

## User-approved naviGATR layout direction (2026-09-28 13:10)

The user approved this direction ("your reconfiguration looks good I like that") and added: "ideally we also split it up slightly so main isnt 1400 lines of code and something like a /core folder right or smth very clear".

```
pi/naviGATR/src/
  main.cpp              tiny entry: parse args, build, run
  cli/                  command-line parsing (was app/; field_refs generator moves to tools/)
  core/                 the engine and nothing else: builds the pipeline and runs it
    system.h            thin facade: build, start, stop, reset, snapshots
    build.cpp           calls each stage's make_* in order (was System::build)
    run.cpp             the two workers and step() (latest-wins handoff lives here)
  pipeline/             the stages, in run order, each the same shape
    framework/          registry, typed payloads, records, ids (stage machinery from old core/)
    resources/ sensors/ commands/ localization/ world_estimation/ targets/ publishing/
      (each: PIPELINE.md, contract.h = standard input/output, stage.cpp = make_*, impl/ = plug-ins + registration)
  brain_profile/        Brain-driven reconfiguration, profile swap, sensor loss, recovery (split out of system.cpp)
  config/ io/ time/ math/ debug/   (io = old transport/, time = clocks from old core/, debug = inspection/capture/diagnostics)
```

- The old `contracts/`, `payloads/`, `state/`, `runtime/` and `core/` grab bags dissolve; each type moves to the stage that owns it.
- runtime/system.cpp (1450 lines) and system.h (479) are split by concern.
  - No file mixes construction, the run loop, profile swapping and recovery.
  - Target: no source file over the standard's size threshold.
- The move is a pure move: include paths and CMake only, no logic changes, identical tests before and after.
- Timing: after the running discovery/survey agents finish, and before any spectuGATR code.
- Open question to the user: whether namespaces should mirror the stage folders (a separate later pass).

The standard's naviGATR plan must use this layout and give the full move map.

## More from the user (2026-09-28 13:15, verbatim)

"and make sure to cleanly folder shit cause I figure pipeline has some shit in like the field world estimation ensure its clearly defined in folders and shit and nicely wrapped up and clear as can be right cause fuck this unreadable shit and also abuse the shit out of using cause I can't read shit and if you have to write customer drivers even for xml if thats more clear or like anything to avoid fucking have to do ten million lines for one command that makes it so unclear, the code it self when being read should be understood at a system level easily ish go in depth if you want more shit by ctrl + click if you want to see true definitions but yea"

### Lead's reading: these become rules

**Sub-pipelines get their own folders.** Every sub-pipeline inside a stage gets its own sub-folder and a short PIPELINE.md.
- world_estimation reads as extraction/ -> association/ -> landmarks/, with the field snapshot as its output.
- localization reads as observation models/ -> estimator/ -> pose history.

Nothing important hides in a grab-bag file.

**Use `using` aggressively, not sparingly.**
- Alias every non-trivial type that appears in a signature, member, container, callback, ID or unit.
- A call site should read like the domain, not like template plumbing.
- The limit: never hide ownership or pointer-ness.

**Read at a system level; details are one ctrl+click away.**
- Functions read at one level of abstraction.
- Repeated low-level sequences get wrapped in small named helpers ("drivers"), so a call site states intent in one line and the detail lives one click away: XML/config reading, XML writing, JSON writing, frame building, serial setup, error plumbing.
- This is extraction of an existing repeated pattern (allowed), not speculative abstraction.
- Custom helpers are encouraged when they make call sites clearer, even for XML.

**Real example, `pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp` ~150-185.** About 30 lines parse one `<TrackingWheel>`:
- manual `requireDouble`/`requireAttr`/`getDouble` chains;
- `ok` flags and early returns;
- manual enum-string checks.

Better: a data-only config struct plus one declarative field table (required / optional-with-default / one-of), read by a shared config reader:
- `auto wheels = config::readAll<TrackingWheelConfig>(node, "TrackingWheel", err);`
- The same table can drive XML writing (runtime/brain_profile_builder.cpp has 78 SetAttribute calls in 616 lines).
- Errors keep the node path.

The standard should include this pattern as a rule with a Bad/Good example. The application plan should include introducing the config reader early, since it shrinks the most code.

## Correction (2026-09-28 13:20): folders split generic vs sensor-specific, not step vs step

The user asked whether extraction/association/landmarks is generic enough for any sensor ("world estimation ... ought to never be specific to a sensor"). Answer, verified in code: the generic parts are the stage contract (contracts/field_estimation.h) and the evidence seam (payloads/field_object_evidence.h FieldObjectPoseEvidence), plus the estimator that folds evidence. Extraction is sensor-specific; association is measurement-type-specific (tags by id+mount; ToF would need geometric matching).

RULE for every stage: folders split GENERIC (contract, evidence/seam types, generic estimator, stage builder) from SENSOR/SEASON-SPECIFIC plug-ins under impl/<family>/. Do not create top-level step folders (extraction/, association/) that mix generic and specific code.

world_estimation layout (supersedes the earlier step-folder sketch):
  pipeline/world_estimation/{PIPELINE.md, contract.h, evidence.h, estimation/ (generic fold), stage.cpp, impl/apriltag/ (tag extraction + tag association + TagAssociationTrace), impl/noop/}

- Known leak to fix in the move: TagAssociationTrace/TagAssociationTraceSet live in the generic evidence header; move them to impl/apriltag/.
- Known limit, NOT to be built now: the evidence is pose-only. A range/bearing form waits for a real second sensor (no over-generalization).

The same pattern applies to localization:
  generic contract + estimator + pose history
  sensor-specific observation models under impl/<family>/

## World estimation model, confirmed by the user (2026-09-28 13:25)

User: "something should generate a full pose which means really what it is is groups of things, april tag on its own is its own thing, ToF can't technically maybe generate a full pose ... it may be grouped with another sensor to generate a full pose right so then they're both grouped into an implementation ... each implementation ought to generate a pose and then the world estimation does fusion on them if necessary"

Rules:
- The world_estimation seam is ONE type: full-pose evidence of a field object (frame, measurement time, odometry epoch). There are no range/bearing evidence types.
- An implementation is a SENSOR GROUP (impl/<group>/). It turns its sensors into full-pose evidence. Partial measurements, time alignment and association stay inside the group. AprilTag alone is one group; ToF plus whatever completes a pose is another.
- world_estimation/estimation/ only fuses evidence; it never knows sensors.
- Planned extension, built only when a second group exists (no over-generalization now):
  - per-axis uncertainty (variance or 3x3 covariance) instead of the scalar `confidence` in FieldObjectPoseEvidence;
  - provenance as producing implementation plus its sensor list instead of a single `SensorId source`; each sensor feeds at most one group, so fusion never double counts.

## World estimation: user's model vs code (2026-09-28 13:35)

**The user's model:**
- Input: the SensorMap.
- The XML picks implementations (sensor groups such as "apriltag" or "ToF+X"). Each declares its own extra config (camera calibration, field map, gates).
- Each group outputs landmark (x, y, theta), where theta is the heading error from the map's nominal orientation (theta = 0 means as built; target heading = nominal + error + offset, e.g. "90 + error").
- World estimation fuses the groups' outputs.

**Code today (verified):**
- The SensorMap input and per-implementation XML config match: `<WorldEstimation><Estimator id type="apriltag">` with FieldMap / ObservationExtraction / Association / LandmarkEstimation children.
- GAP 1: fusion (LandmarkEstimation, landmark_estimator) lives INSIDE the apriltag implementation (impl/world_estimation/apriltag_world_estimator.h). The stage rejects more than one Estimator (runtime/world_estimation_stage.cpp:90 "combining estimators is not supported yet").
- GAP 2: headings are stored absolute (evidence in the odometry frame; FieldState pose in the field frame). The heading error from nominal is computed only for display (src/inspection/inspection_document.cpp:1233). "theta = error" holds only because field.xml's comment convention makes every landmark's nominal heading zero; no type enforces it.

**Plan (for the standard's application section; each step needs user approval, and each is tested separately after the pure move):**

S1. Move fusion out of the apriltag implementation into world_estimation/estimation/ at stage level. Allow N `<Estimator>` sensor groups, each emitting evidence. With one group the result is identical.

S2. Make the output explicit:
- `struct LandmarkEstimate { FieldPosition position; Radians heading_error; }`, with heading error measured from the map's nominal orientation.
- One helper for target heading = nominal + error + offset.
- Fusion math is unchanged (error = heading - fixed nominal).
- The zero-nominal convention stops being load-bearing.

## Corrections from the user (2026-09-28 13:45); these supersede the earlier S1/S2 and the impl/ naming

User: "we can then correctly call it 'Estimators' as the folder right that would make more sense then 'impl'" ... "no fusion yet" ... "by zero degrees btw it should be a conveniention not defined by the game ... we could send it out as (dx, dy, dtheta) more accurately which describes the error ideally its (0, 0, 0) for a perfectly set up field"

RULE (folder naming): no folder named `impl`. A plug-in folder is named after the concept that plugs in, matching its XML element, in plural form:
- `<Estimator>` becomes `world_estimation/estimators/apriltag/`;
- apply the same rule to every stage (localization, resources, sensors, targets, publishing), using each stage's real XML element names.

S1 DROPPED: there is no fusion step and no multiple estimators now. The stage keeps exactly one `<Estimator>`.

S2 REVISED: world estimation outputs a pure error from nominal, not (x, y, theta).
- `struct LandmarkError { Meters dx; Meters dy; Radians dtheta; }`: a displacement in field axes plus a rotation from the map's nominal pose. (0, 0, 0) means the field is as built.
- The map's heading zero is an authoring convention only and no longer carries meaning.
- One helper applies a landmark's error to a nominal target: target heading = nominal + dtheta.
- Caveat for world_estimation PIPELINE.md: dx/dy include the robot localization error at observation time; relative targeting cancels it.
- OPEN (wire decision, not in this change): whether the Brain field documents switch from full poses to this error form.

## Framing for world estimation docs (user, 2026-09-28 13:50)

User: "go back to our problem 'we want to be somewhere relative to somewhere a lot of the time' not 'I want to know exactly where this is on the field' ... the error exists in localization and here therefore they cancel out right, the main error is therefore than just april tag shit ... we can list it out but again reference that like we're solving the ultimate goal of lining up"

The naviGATR and world_estimation PIPELINE.md docs lead with the goal: line up relative to field objects. Absolute field truth is NOT the goal. The error budget follows from that:
- Localization error appears in both the landmark error (dx, dy) and the robot pose, so it cancels for targets defined relative to a landmark.
- Residual 1 (main): perception error (AprilTag detection, camera calibration, mount geometry).
- Residual 2: localization drift since the last sighting. It does not cancel. Landmark-anchored targets re-resolve every cycle while the tag stays in view, which keeps it small.

## Status (2026-09-28 14:00): the naviGATR restructure is being executed now

It runs in an isolated worktree per `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/restructure/SPEC.md`. That spec is authoritative for the layout, the `estimators/` naming, the system.cpp split and LandmarkError.

The standard should:
- describe that layout as the reference example of the structure rules;
- point the naviGATR part of "Applying this" at restructure/SPEC.md and restructure/plan/ instead of re-planning it;
- keep its move-map summary consistent with restructure/plan/move_map_final.tsv once that file exists.

## Naming correction (lead, 2026-09-28 15:05)

The literal rule "plug-in folder = plural of the XML element" produced unreadable names (resources/resources, sensors/sensors, publishings, command_collections, target_resolutions). The rule is now:

Plug-in folders are named with a readable plural noun for what the plug-ins ARE. Examples: estimators, collectors, resolvers, publishers, channels, observation_models.
- A stage with many plug-ins groups them by family instead: resources/cameras, resources/pico, resources/links, resources/geometry, resources/tag_detectors, resources/synthetic.
- Never a stutter, never an awkward mechanical plural, never impl.

## User decisions on the standard (2026-09-28 ~16:30, verbatim then interpreted)

"wdym make warnings errors? I figure that depends, if its a true warning then yes, but its an error if we genuienly cannot continue... yes to d5 every stage should have its make, but the main isnt called main its like called something more descriptive, e.g associate would have something called like associate_tag(tag) or smth yk? some main function that interfaces into it and you have make_associate() which returns some lambda to be called with associate_tag with some shit embdeded into it like a lambda with the config or smth et yk? d6, namespaces should miror folders for the most part up to a certain point, no to a big namespace, but for each pipeline yes have it, so basically yea each folder has its namespace unless truly its like doesnt make sense idrk. d7, not strict, just highly recommended, d2 sure go ahead and format clang-format settings, d4, sure"

**D-1, warnings: SELECTIVE.**
- Warnings that are always real bugs are errors everywhere via `-Werror=<name>`. Candidates: return-type, switch, implicit-fallthrough, format, non-virtual-dtor, delete-non-virtual-dtor, init-self, and others; verify each flag exists in GCC 15 and arm-none-eabi.
- Every other warning stays a warning: `-Wall -Wextra` plus the tier-1 set. It never blocks a local build, but CI rejects new warnings (zero warnings on main).
- This reads the user's "a true warning, yes; an error if we genuinely cannot continue" as: a build fails only on warnings that always mean broken code.

**D-5, APPROVED (b): every stage has a `make_*`, and `main` is split (F4 + F11).**
- A user refinement becomes a rule: every stage AND every sub-step exposes `make_<step>(config...)`. It returns a callable (a lambda capturing its config and state) whose type has a descriptive `using` alias.
- The call site binds it to a verb name, so construction and the run loop read as the pipeline:
  ```cpp
  const auto associate_tags = make_tag_association(cfg);
  auto evidence = associate_tags(observations, poses);
  ```
- Stage entry names: read_resources, read_sensors, collect_commands, localize, estimate_world (inside apriltag: extract_tags, associate_tags, fold_landmarks), resolve_targets, publish.
- Existing class-based implementations are wrapped by their `make_*` (the lambda captures the instance). Internals are not rewritten. Behavior-free.

**D-6, namespaces mirror folders, stopping at a sensible depth.**
- Mapping:
  - `navigatr::<area>` for top folders: cli, core, config, io, time, math, brain_profile, and inspection / capture / diagnostics (the debug/ grouping level is skipped);
  - `navigatr::framework` for pipeline/framework;
  - `navigatr::<stage>` for each pipeline stage (the `pipeline` grouping level is skipped);
  - one more level for a self-contained plug-in family or sensor-group folder, e.g. `navigatr::world_estimation::apriltag`, `navigatr::resources::cameras`.
- Pure grouping folders (estimators/, observation_models/, channels/, collectors/, resolvers/, publishers/, pipeline/, debug/) add no level.
- Not one big namespace. Where a level truly does not make sense, skip it and say why.

**D-7:** TYPE-1 "struct is data only" becomes SHOULD (highly recommended), not MUST.

**D-25 APPROVED:** adopt the proposed clang-format settings and reformat. The user wrote "d2", read as D-25 (clang-format). D-2 (no exceptions) keeps its recommended default.

**D-4 APPROVED:** PIPELINE.md and README.md are allowed inside pi/naviGATR.

**All other decisions:** the standard's recommended defaults stand unless the user objects.
