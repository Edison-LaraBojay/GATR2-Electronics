# GATR2 Coding Standard

Draft 3, 2026-09-28 (the lead's decisions of about 16:30 applied). Covers all software in the repo: `brain/`, `pi/naviGATR/` (including
spectaGATR and its tools), `pico/aggreGATR/`, `translaGATR/`, `bench/`.

Home once adopted: `docs/contributing/coding_standard.md`, linked from `CONTRIBUTING.md`.

Two parts:
- Part 1, the rules, ships to the repo: the quick checklist, sections 0 to 8 and 10, Appendices
  A and E.
- Part 2, adoption, is used once and then replaced by real files: section 9 (becomes the config
  files), section 11 (the plan), section 12 (becomes decision records), Appendices B to D (become
  the real PIPELINE.md files).

Contents:
- Quick checklist
0. How to use this
1. Principles (P-)
2. System design (SYS-)
3. Structure and pipeline docs (STRUCT-, PIPE-)
4. C++ (TYPE-, ALIAS-, NS-, NAM-, UNIT-, TIME-, ERR-, LOG-, OWN-, CON-, RT-, API-, CFG-, WIRE-,
   HDR-, CONST-, ENUM-, CPLX-, NUM-, FMT-)
5. Comments and documentation (CMT-, DOC-)
6. Testing (TEST-)
7. Other languages and files (JS-, PY-, CMAKE-, XML-, SH-, REPO-)
8. Review checklist (REV-)
9. Tooling appendix (Part 2)
10. Glossary and bug classes
11. Applying this to the existing system, including the naviGATR restructure (Part 2)
12. Decisions for the lead (Part 2)
- Appendix A: PIPELINE.md template, with a stage example
- Appendix B: draft `PIPELINE.md` for the whole system (Part 2)
- Appendix C: draft `pi/naviGATR/PIPELINE.md` (Part 2)
- Appendix D: draft `pi/naviGATR/src/pipeline/world_estimation/PIPELINE.md` (Part 2)
- Appendix E: stage contract template
- Revision notes

---

## Quick checklist

Writing code:

- [ ] I can say in one line what this is for and where it sits in its pipeline (P-1, PIPE-5).
- [ ] It lives in the stage or plug-in folder that owns it, not in a grab-bag folder (STRUCT-1 to 5).
- [ ] Generic code names no season, game element, sensor, wire type or implementation (SYS-5, SYS-7).
- [ ] Stage input and output are the contract structs; a failure is a status (SYS-2 to 4).
- [ ] Data is a `struct` (highly recommended); a `class` only for an invariant or a resource;
      could it be a free function? (TYPE-1 to 4)
- [ ] Every stage and sub-step is built by `make_<step>(config...)`, returns a callable with a
      `using` alias, and is called by a verb (`associate_tags = make_tag_association(config)`)
      (TYPE-17).
- [ ] Every non-trivial type in a signature, member, container, callback, id or unit has a
      `using` alias; no alias hides a pointer; `auto` only where the type is on the same line
      (ALIAS-1 to 6, ALIAS-15).
- [ ] Everything is namespaced; the namespace mirrors the folder (`navigatr::localization`,
      `navigatr::world_estimation::apriltag`); file-local things in an anonymous namespace; no
      `using namespace` in a header (NS-1 to 4).
- [ ] Every quantity names its unit, every pose its frames (`T_a_b`), every time its clock
      (UNIT-1, UNIT-4, TIME-3).
- [ ] No clock read below the entry point; time arrives as `now` (TIME-1).
- [ ] No exceptions; fallible functions return optional or status and are `[[nodiscard]]`
      (ERR-1 to 3).
- [ ] Absence is `std::optional`, an empty container or a status, never 0 or -1 (TYPE-16).
- [ ] Every floating value that crosses a seam is checked finite; no loop on a float; float to
      integer through the named helper (NUM-1 to 3).
- [ ] Every wait has a timeout; every queue a capacity, a drop policy and a counter (CON-4, CON-9).
- [ ] A lambda that outlives the call has an explicit capture list and captures by value
      (OWN-9).
- [ ] Every actuator command has an age limit (RT-14).
- [ ] No magic numbers; one named constant per meaning; a default written once (CONST-2, CONST-3,
      CFG-5).
- [ ] Switches over an `enum class` have no `default`; one name function per enum (ENUM-2, ENUM-3).
- [ ] Functions within CCN 15, 60 lines, nesting 3, 5 parameters, one level of abstraction
      (CPLX-1 to 7).
- [ ] clang-format 22.1.8 clean; braces on every `if`, `else` and loop body (FMT-1, FMT-2).
- [ ] No new compiler warnings; the always-a-bug warnings are errors everywhere and CI fails on
      any warning (CMAKE-13).
- [ ] Comments are terse ASCII labels; rationale lives in PIPELINE.md or docs (CMT-1 to 5).
- [ ] Tests use injected time, never sleep, and cover the fault patterns of any new seam
      (TEST-6, TEST-10).
- [ ] PIPELINE.md updated if input, output, stages, threads or entry points changed (PIPE-4).

Reviewers: section 8.

---

## 0. How to use this

### 0.1 Levels

- **MUST**: a tool checks it, or a reviewer can answer yes or no from the diff alone. The lead's
  own rules (comment style, ASCII, naviGATR C++ only, the pinned clang-format, braces on every if,
  namespace braces, test naming) are MUST regardless. A violation blocks the merge unless an
  exception is recorded (0.3).
- **SHOULD**: everything else; the default. Deviating needs a one-line reason, in the review or in
  a code comment.
- A rule that cites an open decision (D-n, section 12) is SHOULD until that decision is recorded.
- Anything no rule covers is the author's call.

### 0.2 Rule IDs

- Every rule has a stable ID, `AREA-N` (for example `TYPE-3`). Principles are `P-N`.
- Each rule is a heading, `ID (LEVEL) Title`, so a search for `SYS-15 (` lands on it and the
  outline lists every rule.
- IDs are never renumbered or reused. A dropped rule keeps its heading, marked `retired`, with a
  pointer to the rule that replaced it. New rules take the next free number in their area.
- Review comments, commit messages and exception tags cite rules by ID.

### 0.3 Exceptions

- **In code:** a tagged comment on the line or the block it covers:
  `// EXCEPTION(<RULE-ID>): <reason>`. The reason is one label, not a paragraph.
- **For a clang-tidy check:** its own marker plus the tag:
  `// NOLINT(<check-name>) EXCEPTION(<RULE-ID>): <reason>`.
- **For a compiler warning:** `#pragma GCC diagnostic push` / `ignored "-W<flag>"` / `pop` around
  the smallest block, the push line carrying `// EXCEPTION(<RULE-ID>): <reason>`.
- **For formatting:** `// clang-format off` with the same tag, closed by `// clang-format on` in
  the same function.
- **For a file, folder or whole component:** a line under `## Exceptions` in the nearest
  `PIPELINE.md`: rule ID, what it covers, reason.
- CI counts exception tags per component and prints the count. The count is reported, not gated.
- An exception to a rule the lead set personally (the list in 0.1) needs the lead's OK in the PR.

### 0.4 New code and existing code

- **New code** (new files, new functions) follows every rule from now on.
- **Changed code:** the edited lines follow the rules. A changed function may not get worse on
  any CPLX metric; it must meet the CPLX limits when it is new, or when its phase comes (11.2).
  Renames, seam conversions and type changes that reach past the edited function are their own
  commits (REPO-7). Moving a function is not changing it.
- **Existing code:** every script, lizard and clang-tidy check starts from a baseline of today's
  violations, recorded when its gate turns on (11.2), that may only shrink. Section 11.6 lists
  the deadline by which each deferred baseline reaches zero; a rule marked `Legacy: deferred`
  has its row there.
- Move-only, format-only, rename-only and logic commits are never mixed (REPO-7).

### 0.5 Precedence

1. The lead's own rules (comment style, ASCII, conventions) and the lead's approved decisions
   (section 12; `restructure/SPEC.md`, `restructure/SPEC_ADDENDUM.md` and the lead's
   corrections at the top of `restructure/plan/FINAL.md`).
2. This standard.
3. Industry sources (C++ Core Guidelines, Google, LLVM, JPL Power of Ten, AUTOSAR, MISRA).
   Where they ask for prose comments, a comment on every declaration or typographic dashes, the
   lead's rules win (CMT-10).

### 0.6 Tags used in rules

- **Scope:** one or more of `All`, `C++`, `Brain` (`brain/` libraries and apps), `Pi` (naviGATR
  C++), `Pico` (aggreGATR), `codec` (translaGATR), `viewer` (spectaGATR JS), `tools` (scripts,
  perf, generators, bench), `config` (XML), `build` (CMake, Make, PlatformIO), `docs`. A qualifier
  goes in parentheses: `Brain (apps)`, `Pi (stages)`, `C++ (tests)`.
- **Enforce:** compiler (a warning flag), clang-format, clang-tidy (a named check), lizard,
  script (a small repo check in `tools/ci/`), test, review, report (advisory, never gated).
  Section 9 has the configs.

### 0.7 Citations

- Code is cited as `path:line` plus a short quote, at commit `ae457ee` (2026-09-28), with git
  paths (`pi/naviGATR/...`; a Windows checkout shows the folder as `pi/navigatr`).
- Rule text uses post-restructure paths (`pipeline/localization/contract.h`). Bad and Good
  citations are pre-restructure, at `ae457ee`. When restructure step 1 lands, the citations are
  rewritten through `restructure/plan/move_map_final.tsv` in one commit; line numbers stay at
  `ae457ee`.
- Counts are at `ae457ee` unless a date is given.
- Marks: `(trimmed)` is a shortened quote, `(paraphrased)` a summary, `(proposed)` code that does
  not exist yet.
- `restructure/` means `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/restructure/`. Only
  Part 2 cites it.

### 0.8 Changing this standard

- A change is a PR that edits this file.
- A new rule takes the next free ID in its area; a retired rule keeps its ID (0.2).
- A rule change that adds violations updates the baseline in the same PR.
- Two rules that conflict are a bug in this standard. Raise it in the PR; until it is fixed, the
  rule with the narrower Scope wins.

---

## 1. Principles

The principles come first, in order. Every rule below serves one of them. Conflicting rules are
handled by 0.8.

### P-1 Understand the problem first

Before code, write down in one line what the system is truly for. Structure follows from that
answer, not from the parts you happen to have.

Worked example, naviGATR:
- Custom electronics mean custom PCB shapes, so mechanical and electronics design go hand in
  hand instead of the mechanism always building around the electronics. naviGATR is the
  software interface to those electronics.
- What do we need from them? To line up relative to things on the field.
- That is two questions: "where am I" (localization) and "where are the things around me"
  (world estimation).
- They are separate systems. They couple at exactly one point: turning a relative observation
  ("a tag 1.2 m ahead") into a field position needs the robot pose at the moment of that
  observation, which localization supplies through its pose history. In code that is one input
  field, `const PoseLookup& history;   // pose and attitude at measurement time`
  (`pi/naviGATR/src/contracts/field_estimation.h:45`).
- Because the goal is lining up and not absolute field truth, localization error shows up in
  both the landmark estimate and the robot pose and cancels for landmark-relative targets. What
  remains is perception error plus drift since the last sighting.
- The same split takes new sensors (ToF, more cameras) as new sensor groups inside world
  estimation, with no new system.

### P-2 Generic pipeline, specific implementations

The pipeline (its stage order, contracts, builders and runner) is generic and survives seasons.
Implementations inside it are specific: a sensor driver, a sensor group, a field file. They plug in
by registration and configuration. If the pipeline itself has to become season-specific to work, the
decomposition is probably wrong: go back to P-1.
- Good: `pi/naviGATR/src/impl/world_estimation/register_world_estimation.cpp:2-4` "Adding one
  means its implementation, a line here, and a build entry; the coordinator and the stage
  builder stay as they are."

### P-3 Do not over-generalize

Build what the current problem needs. An abstraction earns its place with a second concrete use, not
with a guess.
- Example: world-estimation evidence is full-pose only. A range/bearing form waits for a real
  second sensor group.
- Counter-example: `class SpiBus` and `class SpiDevice` (`pi/naviGATR/src/resources/spi_bus.h:35`,
  `:45`) have zero production implementations.

### P-4 Existing systems keep their shape; each has standard INPUT/OUTPUT

Teams work inside a system through its contract. Changing a contract is a reviewed interface change,
never a side effect of feature work.

### P-5 Readability is king

Slightly slower but clearer wins every day, unless a measurement says otherwise. We are throttled by
sensors (Pico 50 Hz, Pi loop 100 Hz, Brain state poll every 20 ms), not by CPU. Code reads at the
system level; the detail is one ctrl+click away.

### P-6 Interfaces are extracted, not invented

Write one concrete implementation end to end. Name the stages early; freeze a signature after two
implementations exist.

### P-7 Degrade, don't hang

Every consumer treats its upstream as possibly dead. Every wait is bounded and every consumer has a
defined answer when its input is missing. Any device can reboot mid-match and recover by resyncing.

### P-8 The narrow seam is the product

The wire contract (translaGATR) and the stage contracts are what other teams build against. A change
that widens a seam, or leaks hardware knowledge across it, is pushed back.

### P-9 Bounded everything

Queues, waits, retries, loops, memory, recursion, input sizes, open connections and files on disk
each have a stated bound, a policy for what happens at the bound, and a counter someone reads.

### P-10 One writer per state, one owner per fact

Mutable state has one writing thread; readers get snapshots. Every constant, name table, unit
conversion, default and wire limit has one definition that everything else refers to.

### P-11 Say it once, next to the code

A fact is written in one place and linked from the others. Each pipeline explains itself in a short
PIPELINE.md beside its code.

### P-12 Claims match evidence

Host-tested is not bench-tested is not robot-validated. A doc, status line or commit message never
claims more than was run.

---

## 2. System design (SYS-)

### 2.1 Pipelines, stages, contracts

#### SYS-1 (MUST) One construction and cycle entry

Every system is a pipeline with one construction entry point and one per-cycle entry point, each
reachable from `main` (or the platform callback) in two steps.
- Why: the composition root is the first thing a reader looks for (P-1, P-5).
- Scope: All. Enforce: review; the PIPE-3 symbol check.
- Bad: `System::build` (`pi/naviGATR/src/runtime/system.cpp:148`) and `System::step` (`:1022`)
  sit in one 1450-line file with 569 lines of Brain-profile, Pico and recovery code between them.
- Good: `pico/aggreGATR/src/main.cpp:319` `void loop() {` reads as the pipeline:
  `imu::service(); receiveCommands(); watchImu();` then sample, encode, write (trimmed).

#### SYS-2 (MUST) One contract header per seam

Each Pi stage has exactly one contract header that declares its `Input` struct, its `Output` struct,
its declared outputs, its seam or seams, its build context (the shared `SlotInitializationContext`
counts) and its make-function alias. Nothing else defines stage input or output. A Brain seam or a
wire format is defined in exactly one header: `investigatr/state_source.h`, `investigatr/planner.h`,
`actugatr/motor_output.h`, `communigatr/byte_port.h`, `translaGATR/frames.h`,
`translaGATR/link_documents.h`.
- Why: the contract is the standard INPUT/OUTPUT other teams build against (P-4).
- Scope: Pi (stages), Brain, codec. Enforce: review. Template: Appendix E.
- Good: `pi/naviGATR/src/contracts/field_estimation.h:42` `struct FieldEstimationInput {` and
  `:50` `struct FieldEstimationOutput {`.

#### SYS-3 (MUST) Stage input is read-only views

A stage input is a struct of read-only views (`const T&`) plus the execution context (clock, cycle,
diagnostics). No loose `now` or `Diagnostics*` parameters.
- Why: one shape for every stage; a stage cannot write another stage's state.
- Scope: Pi. Enforce: review. Legacy: deferred for commands, targets and publishing
  (`PublishingInput` carries `MonotonicTime now;` and `Diagnostics* diagnostics`,
  `pi/naviGATR/src/contracts/publishing.h:36-37`).

#### SYS-4 (MUST) Outputs carry their own health

A stage output carries its own health: a status and a diagnostic string. A failure is a status,
never an empty result, a sentinel or an exception.
- Why: downstream stages and the viewer must tell "no data" from "zero" (P-7).
- Scope: Pi (stages), Brain (libraries). Enforce: review. Legacy: deferred to F10 for the
  outputs with no diagnostic: `CommandsOutput` (`pi/naviGATR/src/contracts/commands.h:28`),
  `TargetResolutionOutput` (`target_resolution.h:41`), `PublishingOutput` (`publishing.h:40`).
- Good: `pi/naviGATR/src/core/function_status.h` `kOk`, `kNoData`, `kFault`.

#### SYS-5 (MUST) Generic code is implementation-neutral

Contracts, stage builders and the coordinator are implementation-neutral: no wire types, device
types, implementation names, or string compares on a registered type name. Implementation extras
travel as declared typed payloads, or stay private.
- Why: a new command source or publisher must plug in without editing the coordinator (P-2).
- Scope: Pi. Enforce: review; script (grep for `"brain_link"` and `translagatr::` outside
  plug-in folders). Legacy: deferred to follow-up F3 (section 11.5).
- Bad: `pi/naviGATR/src/runtime/system.cpp:339`
  `if (commands_type_.value == "brain_link" && publishing_type.value != "brain_link") {`
- Bad: `pi/naviGATR/src/contracts/publishing.h:45`
  `std::optional<translagatr::BrainState> brain_state;` in the generic publishing contract.

#### SYS-6 (MUST) Adding an implementation is local

Adding an implementation is local: its files in the plug-in folder, one registration line, one
build-list line, a config example and a test. If it needs more than that, the contract is missing
something: fix the contract first, in its own commit.
- Why: this is what makes the pipeline reusable across seasons (P-2).
- Scope: Pi, Brain. Enforce: review.
- Good: `pi/naviGATR/src/impl/sensors/register_sensors.cpp:12-13`
  `registerOrDie<SensorMakeFunction>(functions, "pico_encoder_channel", &make_pico_encoder_channel);`
  (trimmed: one line in place of two).

#### SYS-7 (MUST) Season code only in implementations

Season-specific code and data live only in implementations and configuration: the season config
folder, the field file, and application folders such as `brain/operaGATR`. Generic code, its tests
and its comment examples use neutral names.
- Why: next season the generic code must not need an edit (P-2).
- Scope: All. Enforce: review; script (grep for known season ids outside season folders).
- Bad: `pi/naviGATR/spectaGATR/field_scene.js:293` strips `neutral_goal_`, `blue_goal_`,
  `red_goal_` in the generic viewer.
- Bad: comment examples `<Estimator id="goals"` in
  `pi/naviGATR/src/impl/world_estimation/apriltag_world_estimator.h:21` and
  `pi/naviGATR/src/runtime/world_estimation_stage.h:9`.

### 2.2 When something new is allowed

Each of SYS-8 to SYS-14 is an admission test. The PR description answers it in one or two lines.

#### SYS-8 (MUST) Admission: a new system or stage

A new system or stage is allowed only when it answers a question no existing stage answers (P-1),
has a contract (SYS-2), a PIPELINE.md (PIPE-1) and a place in the parent PIPELINE.md stage list.
- Scope: All. Enforce: review.

#### SYS-9 (MUST) Admission: a new mechanism

A new mechanism (event channel, logging path, timer scheme, config path, scheduler, cache) names the
existing mechanism it replaces, or says why none fits. The replaced one is removed in the same
series or tracked for removal.
- Why: naviGATR has seven channels for "something happened" (Diagnostics, LinkMonitor,
  DiagnosticsHub, noteEvent, noteFault, warnings, status fields) (paraphrased).
- Scope: All. Enforce: review.

#### SYS-10 (MUST) Admission: a new thread or task

A new thread or task states its owner, the state it writes (SYS-15), its rate, its stop path (joined
or cooperative) and its entry in the PIPELINE.md "Threads and rates" section.
- Scope: Pi, Brain, Pico (a second core counts). Enforce: review.

#### SYS-11 (MUST) Admission: a new queue

A new queue or handoff answers CON-9 in the PR: capacity, drop policy, drop counter and who reads
it.
- Scope: All. Enforce: review.

#### SYS-12 (MUST) Admission: a new wire item

A new wire message, field or code goes into translaGATR only and answers WIRE-7 in the PR.
- Scope: codec, Pi, Brain, Pico. Enforce: review; CODEOWNERS.

#### SYS-13 (MUST) Admission: a new config knob

A new config element or attribute has a consumer now, a unit suffix where it is a quantity (XML-2),
a default written once (CFG-5), and appears in a shipped config or is documented as a tuning knob.
Knobs kept only "so older profiles load" are deleted.
- Scope: config, Brain (robot config). Enforce: review; test (every config loads, XML-8).
- Bad: `pi/naviGATR/src/runtime/inspection_config.h:21` "snapshot_hz is the inspect/1 push
  rate. It is still accepted and..." (trimmed).

#### SYS-14 (MUST) Abstractions need two real uses

A new abstraction (interface, base class, template, generic helper) needs two real uses in the tree,
and the same change converts both. A noop that serves a real configuration counts as a use. A test
fake counts only at a hardware or process boundary. A virtual interface with fewer uses is replaced
by the concrete type.
- Why: P-3, P-6.
- Scope: All. Enforce: review.
- Bad: `pi/naviGATR/src/resources/spi_bus.h:35` `class SpiDevice` and `:45` `class SpiBus`, zero
  production implementations.
- Good: every Brain seam has a real and a sim implementation (`PathPlanner`: `GeometricPlanner`
  and `actuGATR/sim/straight_planner.h`; `investigatr::StateSource`,
  `brain/investiGATR/include/investigatr/state_source.h:38`, with the link as the real source and
  `SimulatedState` as the sim).
- Open: `perception.h` and `association.h` are step contracts with one implementation each; SPEC
  keeps them generic (D-29, F12).

### 2.3 Data, identity, time, validation

#### SYS-15 (MUST) One writer per state

One writer per mutable state. Readers get an immutable snapshot: on the Pi a `std::shared_ptr<const
T>` swapped under a mutex, on the Brain a copy taken under the owning task's lock (RT-5), or the
newest value through a latest-wins slot. Motion increments are never passed through a latest-wins
handoff.
- Why: P-10; no reader ever sees half an update.
- Scope: Pi, Brain. Enforce: review; TSan job (9.7).
- Good: `pi/naviGATR/src/runtime/handoff.h:5-10` LatestSlot "is the wrong tool for motion
  increments, which must never be replaced" (trimmed).

#### SYS-16 (MUST) Identity-bound state follows its identity

State tied to an identity (session, boot, epoch, connection, profile) stores that identity and is
dropped or re-keyed when it changes. Prefer keying by identity, or a generation checked at read
time, over a `reset()` the owner must remember to call. A `reset()` assigns a value-initialized
whole (`state_ = State{};`), never member by member.
- Why: missing resets on identity change are a recurring bug class (B4: HELLO left
  `init_*` untouched; deltas spanned lost frames).
- Scope: All. Enforce: review; test pattern "identity change mid-flow" (TEST-10).

#### SYS-17 (MUST) Cached copies carry their age

A cached copy carries the time it was taken; readers compute freshness against a named limit. A busy
or unanswered read returns a distinct status, never the last value as if it were new. A decision
that drives actuators reads state in the same cycle it acts.
- Why: stale cached state is a recurring bug class (B3: encoder deltas stayed "current" after
  Pico frames stopped).
- Scope: All. Enforce: review; test pattern "source stops" (TEST-10).

#### SYS-18 (MUST) Explicit clock domains

Clock domains are explicit. Each device stamps with its own clock; the wire carries the device
stamp; the receiver maps it to its own domain through a named mapper. Values from two domains are
never compared or subtracted raw. Details in TIME-.
- Scope: All. Enforce: review; TIME-3, TIME-4.
- Good: `translaGATR/frames.h:71` `uint32_t stamp_ms; // Pico acquisition clock; the Pi maps it
  to host time`.

#### SYS-19 (MUST) One-way dependencies

Dependencies point one way, bottom up: `math/`, `time/`, `io/` -> `config/` -> `pipeline/framework/`
-> stage contracts and stage-level types (a stage may use an earlier stage's types, never a later
one's) -> stage builders -> plug-ins -> `register_*.cpp` -> `core/` -> `brain_profile/` -> `debug/`
-> `cli/`, `main.cpp`. Two headers may be included from below their layer: `debug/diagnostics/hub.h`
(STRUCT-8) and `brain_profile/brain_profile.h`. Only register files, tests and tools include a
plug-in folder from outside it. No include cycles.
- Why: a reader must be able to trust folder names to separate generic from specific.
- Scope: Pi (the order), All (no cycles). Enforce: script (`check_includes.py`); clang-tidy
  `misc-header-include-cycle`. Legacy: deferred to F2; the post-move edges listed in
  `restructure/plan/FINAL.md` section 5 are the baseline.
- Bad: `pi/naviGATR/src/runtime/resource_stage.cpp:6` `#include "impl/resources/serial_links.h"`.

#### SYS-20 (MUST) No mode flags in the cycle

No mode flags inside the per-cycle loop. Optional behavior is chosen at build time as a stage, a
policy object or a supervisor called once per cycle.
- Why: one pipeline must not have two behaviors chosen by `if (mode)`; the sensor-loss rule
  exists only on the profiled path today.
- Scope: All. Enforce: review. Legacy: deferred to follow-up F1 (ProfileSupervisor).
- Bad: `pi/naviGATR/src/runtime/system.cpp:1041`, `:1048`, `:1055` `if (profiled_) {` three times
  inside `estimationCycle`.

#### SYS-21 (SHOULD) One module per request and reply

A request/reply exchange lives in one module. Per-operation reply data is a `std::variant` of per-op
structs, not one struct holding every op's fields.
- Why: the Brain link request is decoded in the commands slot and answered in the publishing slot
  through a 25-field `BrainReplyContext` (`pi/naviGATR/src/state/command_state.h:30-58`,
  paraphrased), and nothing checks the pairing.
- Scope: Pi, Brain. Enforce: review.

#### SYS-22 (retired) Validation vocabulary

See DOC-9.

#### SYS-23 (MUST) Portable core, platform shell

Portable core, platform shell. PROS, Arduino, libcamera, sockets and serial ports live only in
adapter files or folders (`pros/`, the Pico driver files, device plug-ins, `io/`). Everything else
compiles and runs its tests on the host.
- Why: host tests cover the logic; target builds only add adapters.
- Scope: All. Enforce: CI host build and host tests of every non-adapter source.
- Good: `brain/actuGATR/pros/pros_drive.cpp` next to host-built `src/` and `sim/`.

### 2.4 Correct by construction

#### SYS-24 (MUST) One snapshot per document

A document or message is built from one snapshot. Its labels (session, epoch, reset count,
generation) are read from that same snapshot or under the same lock; when that is impossible, the
builder reads the label before and after and rebuilds on a change.
- Why: bug class B5 (a reset label read after the snapshot it labels).
- Scope: Pi, Brain. Enforce: review; test pattern "reset under load" (TEST-10).
- Good: `pi/naviGATR/src/inspection/inspection_document.cpp:1055`
  `std::string underOneReset(const System& system, Build build, bool* stable = nullptr) {`,
  tested by `pi/naviGATR/tests/inspection_feed_gtest.cpp:1058`
  `TEST(InspectionFeed, ResetsUnderLoadNeverLabelAnOldPoseWithTheNewSession) {`.

#### SYS-25 (MUST) No hand-listed field sets

No mechanism depends on a per-field annotation or a hand-listed field set that a new field can
silently miss. Prefer a design that is correct by construction (compare encoded bytes; hash the
inputs, not the output minus exceptions). If a field list is unavoidable, a test or `static_assert`
fails when the struct gains a field.
- Why: the identical-diag skip needs every varying value marked, and a missed mark was a bug
  (`pi/naviGATR/src/inspection/inspection_document.cpp:49` `// Times, ages and stamps are varying:
  a diag identity check leaves them out.`).
- Scope: C++. Enforce: review; test.
- Bad: field-by-field equality of one wire struct written twice:
  `pi/naviGATR/src/impl/commands/brain_link_commands.cpp:54` and
  `brain/communiGATR/sim/fake_pi.cpp:33`, both
  `bool sameTelemetry(const translagatr::BrainTelemetry& a, const translagatr::BrainTelemetry& b) {`
  (WIRE-2).

#### SYS-26 (MUST) Every platform branch is built

Every preprocessor branch and every platform-only file is compiled by at least one blocking CI job.
A branch no job can build is listed under `## Exceptions` in its PIPELINE.md with its manual build
recipe. A vendor or platform behavior the code relies on is a one-line label at the call.
- Why: bug class B15 (ABI integer aliasing, a vendor API that reports 0 or 1, a PROS call that
  fails right after enable, Linux-only code never compiled on the dev box).
- Scope: All. Enforce: CI (9.9); review.
- Good: `pico/aggreGATR/src/main.cpp:91`
  `// Serial1.availableForWrite() only reports 0 or 1 on this core.` and
  `brain/communiGATR/pros/pros_serial_port.cpp:29`
  `// Right after serial_enable the port may still refuse its settings.`

#### SYS-27 (MUST) Explicit registration only

Registration is an explicit call chain from `main`: `registerAll`, then one `register_<stage>` per
stage. No static object whose constructor registers, allocates or does I/O; no work before `main`.
- Why: a self-registering plug-in would hide the make pipeline again; C++ Core Guidelines I.22.
- Scope: C++. Enforce: review; clang-tidy `cppcoreguidelines-interfaces-global-init`.
- Good: `pi/naviGATR/src/runtime/register_all.cpp:9-17` `void registerAll(FunctionRegistry&
  functions) {` `register_resources(functions);` ... `register_publishers(functions);` (trimmed).

---

## 3. Structure and pipeline docs

### 3.1 Structure (STRUCT-)

#### STRUCT-1 (MUST) Folder tree mirrors the pipeline

The folder tree mirrors the pipeline. A component folder is one pipeline. Stages sit in one folder
in run order (the order is written in PIPELINE.md), and every stage folder has the same shape:
- `PIPELINE.md` (PIPE-2 stage size);
- `contract.h`: the stage's standard input and output;
- `stage.h/.cpp`: its `make_<stage>` builder;
- generic helpers the stage owns (catalogs, stores, the state and payload types it produces);
- its plug-in folders (STRUCT-2), holding implementations;
- one `register_<stage>.cpp`: in the plug-in folder when the stage has one plug-in folder, at
  stage level when it has two or more (localization; resources with its family folders).
- Why: "a folder structure + general knowledge of it should give a very clear picture" (the
  lead).
- Scope: Pi (reference layout in 11.4). Brain libraries follow STRUCT-9 and the Pico follows
  STRUCT-10; neither gets stage folders, registries or plug-in folders. Enforce: review; script
  (every stage folder has PIPELINE.md, contract.h and stage.cpp). Legacy: deferred to restructure
  step 5 for the missing `stage.cpp` of commands, targets and publishing (F4, approved with D-5).

#### STRUCT-2 (MUST) Plug-in folder names

A plug-in folder is named with a readable plural noun for what its plug-ins are: `<Estimator>` gives
`estimators/`, `<Observation>` gives `observation_models/`, `<Sensor>` gives `channels/`,
`<CommandCollection type>` gives `collectors/`, `<TargetResolution type>` gives `resolvers/`,
`<Publishing type>` gives `publishers/`. Never a mechanical plural that reads badly
(`publishings/`), never a stutter (`resources/resources/`), never `impl`. A stage with many plug-ins
groups them by family instead (`resources/cameras/`, `resources/pico/`, `resources/links/`). The
stage PIPELINE.md maps each XML element to its folder (XML-11).
- Why: the folder name says what is inside; the lead's correction of 2026-09-28 15:05
  (`restructure/plan/FINAL.md`).
- Scope: Pi. Enforce: script (no `impl/` folder, no folder named like its parent); review.

#### STRUCT-3 (MUST) Generic and specific folders apart

Folders split generic from sensor-specific or season-specific code. Never create top-level step
folders (`extraction/`, `association/`) that mix generic and specific code. A sensor group is one
plug-in folder holding everything specific to it: its extraction, its association, its private trace
types.
- Why: the generic seam (contract, evidence type, generic estimator) must be visible as generic.
- Scope: Pi. Enforce: review.
- Good (approved layout): `pipeline/world_estimation/{PIPELINE.md, contract.h, evidence.h,
  field_state.h, landmark_estimator.*, stage.*}` generic, plus `perception.h` and `association.h`
  (generic step contracts by SPEC; D-29, F12); `estimators/apriltag/` holds tag extraction,
  tag-mount association, tag observations and the tag association trace.
- Bad: `TagAssociationTrace` and `TagAssociationTraceSet` live in the generic evidence header
  (`pi/naviGATR/src/payloads/field_object_evidence.h:63`, `:74`).

#### STRUCT-4 (MUST) Obvious entry points

Entry points are obvious by name. `main` parses, builds, runs and reports, in under 30 lines.
Construction lives in `core/build.cpp`: `System::build` reads as each stage's `make_*` call in run
order, each bound to its verb (TYPE-17), in under 40 lines. The per-cycle order and the run loops
live in `core/run.cpp`. The per-cycle function reads top to bottom as the verbs (`read_resources`,
`read_sensors`, `collect_commands`, `localize`, `estimate_world`, `resolve_targets`, `publish`),
fits on one screen and calls feature logic out; it never inlines it.
- Why: the lead's gripe: "its not clear here the main pipeline is or where the make pipeline"
  is.
- Scope: Pi, Brain (apps name theirs in `brain/PIPELINE.md`, RT-, 11.3). Enforce: review; PIPE-3;
  lizard. Legacy: deferred to restructure step 5 (`main` 112 NLOC, `System::build` 171 NLOC; F4
  and F11, approved with D-5).

#### STRUCT-5 (MUST) No grab-bag folders

No grab-bag folders. `runtime/`, `misc/`, `util/`, `utils/`, `helpers/`, `common/`, and taxonomy
folders (`contracts/`, `payloads/`, `state/`) are not allowed unless the folder holds one stated
role. A type lives with the stage that produces it; machinery shared by all stages goes in
`pipeline/framework/`, `time/`, `math/` or `io/`.
- Why: one naviGATR stage is spread over 6 to 8 folders today; `runtime/` holds 26 files with at
  least 7 roles.
- Scope: All. Enforce: script (folder name denylist); review.

#### STRUCT-6 (MUST) Coordinator hosts no feature

The coordinator hosts no feature (Brain profile, device control, recovery, snapshot assembly);
features sit in their own folder and are called from the cycle. The list of what `core/` holds is in
11.4.
- Scope: All. Enforce: review. Legacy: deferred to F1 (`System` keeps its Brain-profile members).

#### STRUCT-7 (SHOULD) Unique basenames

Basenames are unique within a component, except the fixed stage-shape names (`PIPELINE.md`,
`contract.h`, `stage.h`, `stage.cpp`, `noop.h`, `noop.cpp`). A file whose basename repeats carries
its component-relative path on line 1 (`// pipeline/localization/contract.h`, CMT-11).
- Scope: All. Enforce: script.

#### STRUCT-8 (MUST) Observers sit apart

Observers are read-only and sit apart from the pipeline (naviGATR `debug/inspection`,
`debug/capture`, `debug/diagnostics`). They read published snapshots only and include no plug-in
folder. Pipeline code may post to the diagnostics hub through its header and includes nothing else
from `debug/`.
- Why: 29 percent of naviGATR `src/` is observer code sitting as peers of the stages.
- Scope: Pi. Enforce: script (`check_includes.py`). Legacy: deferred to F2 (the debug to plug-in
  edges in `restructure/plan/FINAL.md` section 5).

#### STRUCT-9 (MUST) Library component layout

A library component uses `include/<namespace>/` for public headers, `src/` for sources and private
headers, `pros/` (or `hardware/`) for platform code including its headers, `sim/` for host fakes,
simulations and shared test support (TEST-4), `tests/` for tests.
- Scope: Brain (libraries), codec (when it gets a CMake target). Enforce: review.
  Legacy: deferred (`communigatr/pros_link.h` and `actugatr/pros_drive.h` sit in the portable
  include folder).

#### STRUCT-10 (SHOULD) Split crowded folders

A folder with more than 15 units directly in it (a .h/.cpp pair is one unit) is split by concern.
- Scope: All. Enforce: script (report).
- Bad: `pico/aggreGATR/src/` is flat with 18 units (29 files) for 7 namespaces. Target: `imu/`,
  `encoders/`, `link/`, `board/` (11.3).
- Also over today: `brain/communiGATR/include/communigatr/` (16 headers).

#### STRUCT-11 (MUST) Programs live in tools

A separate program lives in `tools/<tool>/`, never inside the runtime's `src/`.
- Scope: All. Enforce: review.
- Bad: `pi/naviGATR/src/app/field_refs.cpp` is a generator program in the app folder.

#### STRUCT-12 (MUST) Shared app helpers in the library

Application helpers that several programs share move into the library as a named function; programs
keep only their screens, buttons and wiring. The program's construction chain is one named function
per chain.
- Why: the Brain version of the gripe: `createLink()` and `createDrive()`
  (`brain/operaGATR/src/main.cpp:120`, `:149`) build the whole chain and each app re-derives it;
  8 helpers are duplicated between the two apps.
- Scope: Brain (apps). Enforce: review.

#### STRUCT-13 (SHOULD) One responsibility per file

One responsibility per file. File size limits are in CPLX-5.
- Scope: All. Enforce: review; lizard/script (file length).

### 3.2 Pipeline docs (PIPE-)

#### PIPE-1 (MUST) A PIPELINE.md per pipeline

Every pipeline has one `PIPELINE.md`, linked from its parent under `## Inside`, so the docs form a
tree from the root. The pipelines:
- the repo root (the whole system);
- `pico/aggreGATR`;
- `pi/naviGATR`, each stage under `src/pipeline/`, `src/brain_profile/`, and each sensor-group
  folder (`estimators/apriltag/`);
- `pi/naviGATR/spectaGATR`;
- `brain/` (the chain from link to motors), `brain/communiGATR`, `brain/actuGATR`.

A plug-in becomes a sub-pipeline, with its own folder and PIPELINE.md, once it spans more than one
.h/.cpp pair; a single-pair plug-in is one row in its stage's code map. `translaGATR`,
`brain/investiGATR`, `bench` and `tools` are libraries or tools with a README only.
- Why: the lead: "navigatr shall explain its entire pipeline and any existing pipeline within its
  pipeline must also do so as well", and "the entire system ought to also explain its pipeline".
- Scope: All. Enforce: script (the list above; a folder named under `Inside` has a
  PIPELINE.md).

#### PIPE-2 (MUST) PIPELINE.md shape and size

A PIPELINE.md follows Appendix A. Required, in this order: `# <name>`, a purpose line (the problem,
P-1), `Status:`, `## Start here`, `## Input`, `## Output`, `## Change points`, `## Failure
behavior`. The other Appendix A sections (`## Stages`, `## Code map`, `## Inside`, `## Rules`, `##
Threads and rates`, `## Exceptions`) appear only when they have content, in the Appendix A order;
`## Inside` is required when a child PIPELINE.md exists. A pipeline may add a section it needs (`##
Error budget`). Size: root and component files target 60, at most 100 lines; stage and sub-pipeline
files target 30, at most 50 lines.
- Why: "an explanation that is as concise as possible" (the lead); Input and Output are the
  standard INPUT/OUTPUT (P-4).
- Scope: All. Enforce: script (required headings, order, length); review.

#### PIPE-3 (MUST) Names, never line numbers

A PIPELINE.md names files and symbols, never line numbers. CI checks that every named file exists
and every named symbol appears in its file. Relative links are allowed because CI checks them.
- Why: line numbers go stale on every edit; a checked name does not.
- Scope: All. Enforce: script.

#### PIPE-4 (MUST) Updated with the code

A PIPELINE.md changes in the same commit as the code it describes. A change to a pipeline's input,
output, stages, threads, rates or entry points updates it.
- Scope: All. Enforce: review.

#### PIPE-5 (MUST) Lead with the problem

A PIPELINE.md leads with the problem, not the implementation (P-1). The world-estimation doc leads
with "line up relative to field objects", not "estimate landmark poses".
- Scope: All. Enforce: review.

#### PIPE-6 (SHOULD) A long code map is a structure defect

If a code map needs more than 8 rows to reach the entry points, or points into more than 3 unrelated
folders, that is a structure defect. Fix the structure (STRUCT-); do not add more documentation
around it.
- Scope: All. Enforce: review.

#### PIPE-7 (MUST) One level per parent doc

A parent PIPELINE.md names a child's stages at one level only and links the child for the rest. No
detail is written twice.
- Why: the naviGATR construction order is written three times today and has drifted: the README
  version omits the CommandCollection build.
- Scope: All. Enforce: review.

#### PIPE-8 (MUST) Change points listed

`## Change points` lists every season-specific or robot-specific item and where it plugs in.
`nothing` is a valid answer. The root and component PIPELINE.md files are re-read at each season
start.
- Scope: All. Enforce: review.

#### PIPE-9 (MUST) Entry files point at PIPELINE.md

Header comments do not repeat PIPELINE.md. A file that is a pipeline entry point carries one label
line pointing at it: `// Pipeline: pi/naviGATR/PIPELINE.md`.
- Scope: All. Enforce: review; script (header length, CMT-5).

#### PIPE-10 (MUST) Doc numbers name their source

A number in a PIPELINE.md or reference doc names the constant or config attribute it comes from, in
code font next to the number (`50 Hz, kSampleHz`). A number with no named source is a CONST-2 defect
in the code, or is dropped from the doc.
- Why: bug class B13 (docs understated the poll delay).
- Scope: docs. Enforce: script (the name exists; where the value is a literal initializer, the
  value matches).

---

## 4. C++

Language level: C++17, per CMAKE-7.

### 4.1 Types: struct, class, functions (TYPE-)

#### TYPE-1 (SHOULD) Struct is data only

Highly recommended, not strict (D-7): a `struct` holds data only, public fields with default member
initializers and no invariant. Fine as extras: operators, a nested `Hash`, and one-line `const`
queries with no side effects (`isSet()`, `empty()`). Anything that mutates to keep fields
consistent, or computes more than one line, is better as a `class` or a free function; a struct
that keeps such a method says why in one line.
- Why: the lead: "use struct for attribute only things" and, on strictness, "not strict, just
  highly recommended". A reader knows at a glance it is just data.
- Scope: C++. Enforce: review (report only).
- Bad: `pi/naviGATR/src/resources/brain_imu_bench.h:14` `struct BrainImuBench {` with `reset()`
  (:27), `accept()` (:35) and `acceptAttitude()` (:54) that keep `sequence` and `epoch`
  consistent.
- Good: `pi/naviGATR/src/contracts/field_estimation.h:50` `struct FieldEstimationOutput {`, plain
  fields with a status and a diagnostic.

#### TYPE-2 (MUST) Class guards an invariant

A `class` is private data plus the methods that keep its invariant, or an owner of a resource
(thread, socket, device, mutex, file), or an interface. A class has no public data members. A pimpl
`Impl` defined in the .cpp is the owner's private state and stays a `struct`.
- Why: the lead: "use class for attribute + methods". Public data next to private state means the
  invariant is not protected.
- Scope: C++. Enforce: clang-tidy `misc-non-private-member-variables-in-classes`
  (IgnoreClassesWithAllMemberVariablesBeingPublic).
- Good: `class LatestSlot` (`pi/naviGATR/src/runtime/handoff.h:27`) keeps "replaces the pending
  entry and counts the replacement" behind methods.

#### TYPE-3 (SHOULD) Free function first

Before writing a class, write the free-function version. Keep the class only when state must be
threaded through many calls. Behavior on a data struct is a free function in the same namespace and
header.
- Why: the lead: "avoid using classes for the sake of, truly ask if this is necessary? can I use a
  functional approach?"
- Scope: C++. Enforce: review (REV-5 asks "which invariant does this class guard?"); report
  (clang-tidy `readability-convert-member-functions-to-static`: a method that never uses `this`
  is a free-function candidate).
- Good: `brain/actuGATR/include/actugatr/telemetry.h:23`
  `translagatr::BrainTelemetry telemetryOf(const DriveSnapshot& snapshot);`
- Bad: `class System::ProfileHost` (`pi/naviGATR/src/runtime/system.cpp:79`), a class whose five
  methods each forward to a `System` private (paraphrased).

#### TYPE-4 (MUST) No static-only classes

No class or struct that only groups static members. Use a namespace with free functions.
- Scope: C++. Enforce: script (a class or struct whose every member function is `static` and
  that has no non-static data member); review.
- Bad: `pi/naviGATR/src/core/host_clock.h:16` `class HostClock` with only `static now()`,
  `static nowUs()`, `static origin()`.
- Good (proposed): `namespace host_clock { MonotonicTime now(); HostMicros nowUs(); }`.

#### TYPE-5 (SHOULD) Functional plug-in seams

A plug-in seam is a struct of `std::function` (run, reset, declared outputs), not a virtual
interface. A virtual interface is used only when there are two implementations with shared state
logic worth a base class. An existing virtual seam is converted only by its own approved change
(F10), never because a feature touched the file (D-12). Wrapping an existing class in its
`make_<step>` lambda (TYPE-17) is not a conversion: the class stays as it is.
- Why: the functional form needs no class per plug-in; five noop classes exist only to satisfy
  virtual seams (`pi/naviGATR/src/impl/noop/noops.cpp:11`, `:19`, `:37`, `:47`, `:55`).
- Scope: Pi. Enforce: review.
- Good: `pi/naviGATR/src/contracts/sensor.h:30-37` `struct SensorExecutable {` with
  `std::function<PollResult(const ResourceMap&, const ExecutionContext&)> execute;` and
  `std::function<void()> reset;` (the shape; the raw `std::function` members still need their
  ALIAS-3 aliases).

#### TYPE-6 (retired) Virtual interfaces need two uses

See SYS-14.

#### TYPE-7 (MUST) Value semantics, rule of zero

Value semantics by default: small structs by value or `const&`, results returned by value. Rule of
zero; if any of copy, move or destructor is declared, declare or delete all of them.
- Scope: C++. Enforce: clang-tidy `cppcoreguidelines-special-member-functions`
  (AllowSoleDefaultDtor, AllowMissingMoveFunctionsWhenCopyIsDeleted).

#### TYPE-8 (MUST) override, virtual, final

Every override says `override`; `virtual` appears only in the base; a polymorphic base has a public
virtual destructor; leaf implementations are `final`.
- Scope: C++. Enforce: compiler `-Wsuggest-override -Wnon-virtual-dtor` (host); clang-tidy
  `modernize-use-override`, `cppcoreguidelines-virtual-class-destructor`; review for `final`.
- Good: `brain/actuGATR/include/actugatr/follower.h:125`
  `class DifferentialFollower final : public SegmentFollower {`.

#### TYPE-9 (MUST) Values that travel together get a struct

Two or more values that travel together get a struct, from the second signature that passes them. No
parallel arrays. No `std::pair` or `std::tuple` whose members need names; if a comment has to say
what `.first` is, write a struct.
- Why: "ten million lines for one command" (the lead) often comes from loose values.
- Scope: C++. Enforce: review.
- Bad: `pi/naviGATR/src/capture/capture_bundle.h:44-46` `// (from host_us, reset count),
  ascending; ...` then `std::vector<std::pair<int64_t, uint64_t>> resets;` (trimmed).
- Good (proposed): `struct ResetMark { int64_t host_us = 0; uint64_t reset_count = 0; };` and
  `using ResetTimeline = std::vector<ResetMark>;`.

#### TYPE-10 (MUST) Return values, not output parameters

Return values, not output parameters, for results up to a small struct; `std::optional<T>` for
"maybe". Output parameters are allowed for large buffers the caller reuses and for the `std::string&
err` convention (ERR-2).
- Scope: C++. Enforce: review.
- Bad: `pi/naviGATR/src/math/se3.h:71-72` `inline void eulerFromRotation(const Rotation3& R,
  double& roll_rad, double& pitch_rad, double& yaw_rad) {` (trimmed).
- Good (proposed): `struct EulerAngles { double roll_rad = 0.0; double pitch_rad = 0.0; double
  yaw_rad = 0.0; };` returned by value.

#### TYPE-11 (MUST) optional, not have flags

No `bool have_x` beside `x`; use `std::optional<X>`.
- Scope: C++. Enforce: review.
- Bad: `brain/actuGATR/include/actugatr/motion.h:198-199` `bool have_last_update_ = false;` and
  `Seconds last_update_ = 0;`.
- Good (proposed): `std::optional<Seconds> last_update_;`.

#### TYPE-12 (MUST) No bare bool parameters

No `bool` parameter unless the function name says what it is (`setEnabled(bool)`). Otherwise use an
`enum class` or an options struct. More than one `bool` parameter is always an options struct.
- Why: a bare `false` at a call site says nothing.
- Scope: C++, viewer. Enforce: review; clang-tidy `bugprone-easily-swappable-parameters`
  (advisory).
- Bad: `pi/naviGATR/src/runtime/localization_stage.cpp:149`
  `feed_->publish(state_, status, false, ++publication_);`

#### TYPE-13 (MUST) Wire codes keep their enum type

A field or parameter that holds a wire code has the code's enum type, not `uint8_t` plus a comment
naming the type (D-15).
- Why: a type cannot go stale; a comment can (one already names a type that does not exist:
  `SensorMaskBit` in `pi/naviGATR/src/diagnostics/records.h:79`).
- Scope: C++. Enforce: review; script (grep `uint8_t .*// translagatr::`).
- Bad: `pi/naviGATR/src/runtime/system.h:152`
  `uint8_t result = 0; // translagatr::BrainResult, Pending until settled` (spacing trimmed).

#### TYPE-14 (SHOULD) Variants for closed sets

A closed set of alternatives is a `std::variant` with no separate kind tag, dispatched with
`std::visit` or a `std::get_if` chain. An open, plug-in-extended set uses the typed payload map
(`TypedPayload`).
- Scope: Pi, Brain. Enforce: review.
- Bad: `pi/naviGATR/src/diagnostics/records.h:146` `using DiagPayload = std::variant<...>` kept
  beside a `DiagKind kind` field, so the two can disagree (paraphrased).

#### TYPE-15 (SHOULD) Templates only where plain

Templates only where they are the plain answer: containers, strong types, type-safe erasure, the
config field table (CFG-3), registration (`registerOrDie`), a function that takes a callable
(`underOneReset(system, build)`), and one constrained template that replaces an overload set over
integer widths (NUM-5). No tag dispatch or generic lambdas where two or three explicit calls would
do.
- Why: P-5; cleverness in the construction sequence is exactly where the gripe bites.
- Scope: C++. Enforce: review.
- Bad: `pi/naviGATR/src/runtime/system.cpp:30-33` `template <typename T> struct Tag { using type
  = T; };` feeding `:259` `const auto buildSlot = [&](const char* slot_name, auto& target, auto
  make_tag,` (trimmed).
- Good (approved, restructure step 5): three named builders, `make_commands(...)`,
  `make_targets(...)`, `make_publishing(...)`, so `grep make_` finds every stage (TYPE-17).
- Good: `pi/naviGATR/src/inspection/json_writer.h:85-90`
  `// Fixed-width integers may alias long or long long, depending on the ABI.` then
  `template <typename Integer, std::enable_if_t<std::is_integral_v<Integer> && ...> = 0>`
  `void value(Integer v) {` (trimmed): the fix for bug class B15.

#### TYPE-16 (MUST) Typed absence, no sentinels

Outside the wire, absence is typed: `std::optional<T>`, an empty container, or a status enum. No
in-band sentinel (0, -1, NaN, empty string, a clamped 1) in a C++ field, parameter, return or local.
On the wire a reserved value is a named constant (WIRE-6) and is converted to `std::optional` at the
decoder.
- Why: bug class B2 (a valid pose with no age was drawn as live).
- Scope: C++. Enforce: review; clang-tidy `bugprone-unchecked-optional-access`. Legacy: deferred
  (phase 5).
- Bad: `pi/naviGATR/src/diagnostics/link_monitor.h:57` `int64_t last_rx_us = -1;` (and two more,
  spacing trimmed); `pi/naviGATR/src/diagnostics/hub.cpp:59`
  `record.host_us = std::max<int64_t>(1, HostClock::nowUs());` (forces 1 so that 0 can mean
  unset); `pi/naviGATR/src/capture/capture_recorder.h:58`
  `std::function<int64_t()> wall_ms; // unix ms or -1; the system clock when empty`.

#### TYPE-17 (MUST) Made by make_<step>, called by a verb

Every stage, and every sub-step named in a PIPELINE.md `## Stages` list, is built by
`make_<step>(config...)`. It returns the step as a callable: a lambda that captures its config and
state by value (OWN-9). The callable's type has a `using` alias named for the step and ending in
`Step` (`TagAssociationStep`, ALIAS-3). The caller binds it to a verb, so construction reads as a list of `make_*` calls and the
cycle as a list of verbs. A stored step is a member named by its verb (`localize_`).
- An existing class is wrapped, not rewritten: its `make_<step>` builds the instance and returns a
  lambda that captures it.
- A step that also has reset or declared outputs returns its TYPE-5 executable struct; the verb
  binds its `run`.
- Construction that can fail follows ERR-2: `std::optional<FooStep>` plus `err`.
- naviGATR verbs: `read_resources`, `read_sensors`, `collect_commands`, `localize`,
  `estimate_world`, `resolve_targets`, `publish`; inside `estimators/apriltag/`: `extract_tags`,
  `associate_tags`, `fold_landmarks`.
- Why: the lead: "the main isnt called main its like called something more descriptive, e.g
  associate would have something called like associate_tag(tag) ... and you have make_associate()
  which returns some lambda ... with the config" (trimmed). `grep make_` finds every step, and the
  call site says what happens.
- Scope: Pi (stages; sub-steps of stages and sensor groups). Enforce: review; script (every stage
  folder's `stage.cpp` defines `make_<stage>`). Legacy: deferred to restructure step 5 (D-5).
- Bad: `pi/naviGATR/src/runtime/system.cpp:259` `const auto buildSlot = [&](const char*
  slot_name, auto& target, auto make_tag,` (trimmed) builds three stages through one generic
  lambda, so neither the make pipeline nor the run order reads by name.
- Good (proposed; `TagMountAssociation` is today's class, the config struct follows CFG-2):

```cpp
using TagAssociationStep = std::function<AssociationOutput(const AssociationInput&)>;

TagAssociationStep make_tag_association(const TagAssociationConfig& config) {
    auto association = std::make_shared<TagMountAssociation>(config);   // existing class, wrapped
    return [association](const AssociationInput& input) { return association->run(input); };
}

// Call site.
const TagAssociationStep associate_tags = make_tag_association(config);
const AssociationOutput  evidence       = associate_tags(input);
```

### 4.2 using aliases (ALIAS-)

The lead: "abuse the shit out of using cause I can't read shit". Aliases are the default, not
the exception. A call site should read like the domain, not like template plumbing.

#### ALIAS-1 (MUST) Alias every non-trivial type

Every non-trivial type that appears in a signature, a member, a container, a callback, an ID or a
unit gets a `using` alias named for what it means. Non-trivial means: a template instantiation, a
nested type, a function type, or a builtin type carrying a domain meaning (an id or a code; units
follow ALIAS-5).
- Scope: C++. Enforce: review.

#### ALIAS-2 (MUST) One alias per integer concept

Every integer that is an identity, counter, epoch, revision, generation, sequence, ticket, handle or
code gets one alias per concept, defined once in the component that owns the concept. Wire concepts
live in translaGATR.
- Why: `session` is spelled `uint32_t` 33 times; `odometry_epoch` is `uint32_t` in 6 declarations
  and `uint64_t` in 10 (counts at ae457ee).
- Scope: C++. Enforce: review.
- Bad: `translaGATR/frames.h:316` `uint32_t session = 0; // 0 in HELLO` (spacing trimmed).
- Good: `brain/communiGATR/include/communigatr/client.h:71`
  `using PlacementTicket = uint32_t; // 0 = none or refused` (the alias; its in-band 0 is a
  TYPE-16 case, since the ticket never crosses the wire).
- Good (proposed, in `translaGATR/frames.h`): `using SessionId = uint32_t;`,
  `using RequestId = uint16_t;`, `using PiInstanceId = uint32_t;`, `using SensorMask = uint16_t;`.

#### ALIAS-3 (MUST) Alias std::function by role

Every `std::function` type that appears in a header, or twice anywhere, gets an alias named by role:
`FooCallback` (notifies, returns void), `FooSource` (returns a value), `FooClock` (a time source),
`FooHandler` (request in, response out), `FooMakeFunction` (registry factory), `FooHook` (test or
sim injection), `FooStep` (a pipeline step returned by `make_<step>`, TYPE-17).
- Scope: C++. Enforce: review.
- Good: `pi/naviGATR/src/contracts/sensor.h:47-48` `using SensorMakeFunction =
  std::function<std::optional<SensorExecutable>(const ConfigNode&,
  SensorInitializationContext&, std::string& err)>;` (trimmed).
- Bad: `pi/naviGATR/src/capture/capture_recorder.h:57-58` two `std::function<int64_t()>` members,
  one in microseconds and one in unix milliseconds, told apart only by comments.

#### ALIAS-4 (MUST) Alias repeated containers

A container type spelled twice or more, or nested (a template argument that is itself a template),
in a signature or member gets an alias.
- Scope: C++. Enforce: review.
- Bad: `std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>>` spelled at
  `pi/naviGATR/src/runtime/system.h:303`, `:456`, `system.cpp:1014` and `:1221`.
- Good: `pi/naviGATR/src/core/records.h:102`
  `using SensorMap = std::unordered_map<SensorId, MeasurementRecord, SensorId::Hash>;`
- Good (proposed): `using DetectionFrameMap = std::map<SensorId,
  std::shared_ptr<const DetectionFrameSnapshot>>;` (the pointer stays visible inside the alias
  target; the alias names the map).

#### ALIAS-5 (MUST) Units in aliases or suffixes

Units. A floating SI quantity uses the component's unit alias where the component has them (Brain:
`Meters`, `Seconds`, `Radians`, `MetersPerSecond`, `RadiansPerSecond`). Pi code, wire structs,
firmware and config keep the unit as a name suffix (`x_m`, `heading_cdeg`, `stamp_ms`). A raw
`double` whose unit lives only in a comment is not allowed anywhere (D-14).
- Scope: C++. Enforce: review.
- Good: `brain/investiGATR/include/investigatr/geometry.h:11-15` `using Seconds = double;
  using Meters = double; using Radians = double; ...` (trimmed).
- Bad: `brain/locaGATR/include/robot_config.h:24`
  `constexpr double kCalibrationDistance = 1.0; // meters`.

#### ALIAS-6 (MUST) Aliases never hide ownership

An alias never hides ownership, pointer-ness, reference-ness or `const`. Alias the pointee and spell
the smart pointer at the use. An alias of a container of pointers is fine because the container is
what it names.
- Why: the lead's limit; `const Frame& frame` must not secretly be a nullable shared pointer.
- Scope: C++. Enforce: review.
- Bad: `pi/naviGATR/src/inspection/http_server.h:160`
  `using Frame = std::shared_ptr<const std::string>; // one complete WebSocket frame`
  (spacing trimmed).
- Good: `std::shared_ptr<const std::vector<uint8_t>> y8;`
  (`pi/naviGATR/src/resources/camera.h:74`) becomes `std::shared_ptr<const Y8Pixels> y8;` with
  `using Y8Pixels = std::vector<uint8_t>;` (proposed).

#### ALIAS-7 (SHOULD) No abbreviation aliases

No alias that only abbreviates a standard type or renames an already clear one.
- Scope: C++. Enforce: review.
- Bad: `pi/naviGATR/src/inspection/inspection_service.cpp:50` `using Ms = std::chrono::milliseconds;`

#### ALIAS-8 (MUST) One home per alias

One home per alias. Class scope when only that class's interface uses it; namespace scope in the
header that owns the concept when several files use it; the component's vocabulary header for units,
ids and clocks that cut across the component (translaGATR: `frames.h`, `link_documents.h`; naviGATR:
`pipeline/framework/ids.h`, `time/time.h`, `time/host_clock.h`; Brain: `investigatr/geometry.h`).
Other components import it with a using-declaration or qualify it; they never define the same name
again.
- Scope: C++. Enforce: review; script (duplicate alias names across components).
- Bad: `brain/communiGATR/include/communigatr/client.h:30` `using Seconds = double;` duplicates
  `brain/investiGATR/include/investigatr/geometry.h:11`.

#### ALIAS-9 (MUST) Tests reuse production aliases

Tests use the production aliases; they never redefine them.
- Scope: C++ (tests). Enforce: review.
- Bad: `using Bytes = std::vector<uint8_t>;` defined separately in
  `translaGATR/tests/frame_codec_gtest.cpp:21`, `translaGATR/tests/link_documents_gtest.cpp:21`
  and `pico/aggreGATR/tests/pico_commands_gtest.cpp:18`.

#### ALIAS-10 (MUST) using, not typedef

`using`, never `typedef`, except inside an `extern "C"` header that C code also compiles.
- Scope: C++. Enforce: clang-tidy `modernize-use-using` (IgnoreExternC).
- Good (allowed): `pi/naviGATR/src/impl/resources/apriltag_c.h:23`
  `typedef struct nav_apriltag_detector nav_apriltag_detector;` inside `extern "C"`.

#### ALIAS-11 (MUST) Typed ids stay typed

A typed id is always passed and stored as its typed id, never as a raw `std::string`.
- Scope: Pi. Enforce: review.
- Good: `pi/naviGATR/src/core/ids.h:40` `using SensorId = TypedId<struct SensorIdTag>;`
- Bad: `pi/naviGATR/src/contracts/slot_init.h:51`
  `std::vector<std::string> observation_functions;` although `ObservationFunctionId` exists.

#### ALIAS-12 (SHOULD) Alias naming

Alias names are UpperCamelCase nouns. Identity suffixes follow the glossary (section 10): `FooId`,
`FooEpoch`, `FooRevision`, `FooGeneration`, `FooSequence`, `FooTicket`. Containers by content:
`FooList`, `FooMap`, `FooSet`, `FooQueue`. Units are plural SI names with `Per`
(`MetersPerSecondSquared`).
- Scope: C++. Enforce: clang-tidy `readability-identifier-naming` (TypeAliasCase); review.

#### ALIAS-13 (SHOULD) Plain aliases before strong types

Plain alias by default. A strong type (a wrapper struct the compiler checks) only when a mix-up has
happened or would be silent and costly; its comment says which. The existing strong types stay:
`TypedId` and `MonotonicTime` (D-3).
- Scope: C++. Enforce: review.

#### ALIAS-14 (SHOULD) std::hash for TypedId

Specialize `std::hash` for `TypedId` once, so maps keyed by typed ids drop the third template
argument (`std::unordered_map<SensorId, uint64_t, SensorId::Hash>` is spelled 11 ways today).
- Scope: Pi. Enforce: review.

#### ALIAS-15 (MUST) auto only when the type is visible

`auto` only where the type is spelled on the same line (`std::make_unique<Foo>()`, a cast, a named
constructor), for iterators, lambdas and structured bindings, or where the type is an unnamed
template product. Everywhere else write the type or its alias.
- Why: an alias that `auto` hides never reaches the call site; Google style: "use type deduction
  only to make the code clearer or safer, and do not use it merely to avoid the inconvenience of
  writing an explicit type."
- Scope: C++. Enforce: review. Legacy: deferred (406 `auto` in production code, phase 5).
- Bad: `pi/naviGATR/src/inspection/http_server.cpp:394` `const auto q  = parseQuery(query);`
  hides `std::map<std::string, std::string>` (declared at `:372`); with an alias it reads
  `const QueryParams q = parseQuery(query);` (proposed).
- Good: `pi/naviGATR/src/capture/capture_bundle.cpp:354` `const auto it = current_.find(key);`
  (an iterator; spacing trimmed).

### 4.3 Namespaces (NS-)

#### NS-1 (MUST) Everything is namespaced

Every declaration is in a named namespace. The global namespace holds only `main`, platform
callbacks whose names the platform fixes (PROS `initialize`, `disabled`, `competition_initialize`,
`autonomous`, `opcontrol`; Arduino `setup`, `loop`), gtest `TEST` bodies, and `extern "C"` shims.
Platform callbacks are one-line trampolines into namespaced code.
- Why: the lead: "in general everything is namespaced".
- Scope: C++. Enforce: script (top-level declarations outside a namespace).
- Bad: `brain/operaGATR/include/field_references.h:4` generated
  `struct Field : investigatr::FieldReferences {` at global scope.

#### NS-2 (MUST) Namespaces mirror folders

Each component has one root namespace equal to its lowercased folder name: `navigatr`,
`communigatr`, `actugatr`, `investigatr`, `translagatr`, `aggregatr`. Below the root, namespaces
mirror folders up to a sensible depth: one per pipeline stage or area, plus one level for a
self-contained plug-in family or sensor-group folder. Never one big namespace for a component with
sub-folders. Robot and program description namespaces are nouns (`gatr2_robot`, `robot_config`).
- naviGATR mapping (D-6):
  - `navigatr::<area>` for `cli`, `core`, `config`, `io`, `time`, `math`, `brain_profile`, and
    `inspection`, `capture`, `diagnostics` (under `debug/`);
  - `navigatr::framework` for `pipeline/framework/`;
  - `navigatr::<stage>` for each stage folder under `pipeline/` (`navigatr::localization`,
    `navigatr::world_estimation`);
  - one more level for each sensor-group and resource-family folder:
    `navigatr::world_estimation::apriltag`, `navigatr::resources::cameras` (also `pico`, `links`,
    `geometry`, `tag_detectors`, `synthetic`);
  - `main.cpp` and `pipeline/register_all.*` stay in `navigatr`.
- Folders that add no level: pure grouping folders (`pipeline/`, `debug/`, `estimators/`,
  `observation_models/`, `channels/`, `collectors/`, `resolvers/`, `publishers/`) and layout
  folders (`include/<ns>/`, `src/`, `pros/`, `sim/`, `tests/`).
- A level that truly makes no sense is skipped, with a one-line reason under `## Exceptions` in the
  nearest PIPELINE.md. A namespace that mirrors no folder is allowed only in place of a
  static-only class (TYPE-4).
- Why: the lead: "namespaces should miror folders for the most part up to a certain point, no to a
  big namespace, but for each pipeline yes have it".
- Scope: C++. Enforce: script (path vs namespace). Legacy: deferred; naviGATR to restructure step
  6, aggreGATR to phase 4.
- Bad: the Pico has seven top-level namespaces and no `aggregatr`: `board` (`board.h:12`), `cfg`
  (`config.h:17`), `encoder`, `imu` (`imu.h:11`), `bno08x`, `asm330` (`asm330_supervisor.h:12`),
  `pilink` (`pi_link.h:8`), all under `pico/aggreGATR/src/`.
- Bad: naviGATR is one flat `navigatr`: `class TrackingWheelMotion`
  (`pi/naviGATR/src/impl/localization/tracking_wheel_motion.h:76`) and `class HttpServer`
  (`pi/naviGATR/src/inspection/http_server.h:156`) share it.

#### NS-3 (MUST) File-local means anonymous namespace

File-local functions, types and constants go in an anonymous namespace inside the component
namespace. No namespace-scope `static`. No anonymous namespace in a header.
- Scope: C++. Enforce: clang-tidy `misc-use-internal-linkage` (FixMode None, so its fix-it adds
  no `static`), `misc-use-anonymous-namespace` (no namespace-scope `static`),
  `misc-anonymous-namespace-in-header`; compiler `-Wmissing-declarations`.
- Bad: `pi/naviGATR/src/inspection/http_server.cpp:723`
  `enum ChoiceKind { kChooseControl, kChooseReliable, kChooseSlot };` at `navigatr` scope after the
  anonymous namespace closed.

#### NS-4 (MUST) No using namespace

No `using namespace` in headers or production sources. Tests may write `using namespace <component
under test>;` after the includes. A name from another namespace comes in by a using-declaration or
an alias at namespace scope (NS-5), or is qualified.
- Scope: C++. Enforce: clang-tidy `google-build-using-namespace` (all production code; off in the
  tests overlay); script for headers.
- Bad: `pico/aggreGATR/src/main.cpp:16` `using namespace translagatr;`
- Bad: `pi/naviGATR/src/inspection/inspection_document.cpp:1935` `using namespace translagatr;`
  inside a function body.

#### NS-5 (SHOULD) Vocabulary using-declarations

A header may import names from another namespace with using-declarations at namespace scope when
its own API uses them: another component's vocabulary (units, ids), or on the Pi a sibling
namespace's types (`using framework::ExecutionContext;` in a stage contract). Anything else is
qualified.
- Scope: Brain, Pi. Enforce: review.
- Good: `using investigatr::Seconds;` in `brain/actuGATR/include/actugatr/pid.h:12`.

#### NS-6 (MUST) Namespace brace style

Namespace braces: the opening brace on its own line (Allman, namespaces only), contents not
indented, one brace per namespace (no `namespace a::b`), closing comment `} // namespace x` managed
by clang-format.
- Scope: C++. Enforce: clang-format (`BraceWrapping.AfterNamespace`,
  `NamespaceIndentation: None`, `FixNamespaceComments`, `CompactNamespaces: false`).
- Bad: `pico/aggreGATR/src/bno08x_reports.h:9` `namespace bno08x {`.

#### NS-7 (SHOULD) Namespace aliases use full names

A namespace alias for another component uses that component's name, not an abbreviation.
- Scope: C++. Enforce: review.
- Bad: `pi/naviGATR/tests/brain_link_e2e_gtest.cpp:48` `namespace ag = actugatr;`

#### NS-8 (MUST) Generated code is namespaced

Generated code is namespaced; the generator is fixed, not the output.
- Scope: All. Enforce: NS-1 script on generated files.
- Bad: the generator `fieldReferencesHeader` in
  `pi/naviGATR/src/impl/publishing/field_documents.cpp` writes the global `struct Field`
  (paraphrased).

### 4.4 Naming (NAM-)

#### NAM-1 (MUST) Case rules

| Kind | Case | Example |
|---|---|---|
| types, aliases, enums | PascalCase noun | `SensorExecutable` |
| functions, methods | camelCase verb | `parseCommandLine` |
| variables, parameters, struct fields | snake_case | `stale_after_ms` |
| private and protected data members | snake_case with trailing `_` | `retired_epochs_` |
| constants, enumerators | `k` + PascalCase | `kSampleHz`, `kNoData` |
| macros (build variants only) | `GATR2_` SCREAMING_CASE | `GATR2_IMU_BNO08X` |
| namespaces, files, folders | lowercase snake_case | `robot_state_feed.h` |

Component folders keep their -GATR spelling (`naviGATR/`). Platform callbacks keep the platform's
names.
- Scope: C++. Enforce: clang-tidy `readability-identifier-naming` (9.2).

#### NAM-2 (MUST) Factory names

Named exception to NAM-1: a factory registered under an XML `type` is a free function `make_<type>`
when that type name is unique in the component (`type="pico_encoder_channel"` gives
`make_pico_encoder_channel`), else `make_<type>_<stage noun>` (`type="noop"` is registered in five
stages: `make_noop_commands`; `type="brain_link"` in two: `make_brain_link_publisher`). A stage
builder is `make_<stage>`, a sub-step builder is `make_<step noun>` (`make_tag_association`), and
the step it returns is bound to a snake_case verb (`associate_tags`, TYPE-17); a registration
function is `register_<stage>`. There is one factory spelling: no static `create`, no camelCase
`makeNoopX` (D-8).
- Why: `grep make_<type>` goes straight from a config file to the code that builds it.
- Scope: Pi. Enforce: clang-tidy naming with `FunctionIgnoredRegexp` (9.2); review. Legacy:
  deferred to phase 5 (`X::create`, `makeNoopX`, and `register_target_resolution` and
  `register_publishers`, which the move keeps and which become `register_targets` and
  `register_publishing`).
- Bad: three spellings for one role: `&make_pico_encoder_channel`
  (`sensors/register_sensors.cpp:13`), `&TrackingWheelMotion::create`
  (`localization/register_localization.cpp:21`) and `&makeNoopStateEstimator` (`:27`), all under
  `pi/naviGATR/src/impl/`.

#### NAM-3 (MUST) One name per concept

One name per concept across the XML element, folder, contract, class, doc and log text.
- Scope: All. Enforce: review; glossary (section 10).
- Bad: stage 5 is `<WorldEstimation>` in XML and `world_estimation` in folders, but its contract
  is `class FieldEstimation` (`pi/naviGATR/src/contracts/field_estimation.h:58`) and its worker is
  the "field worker".

#### NAM-4 (MUST) No namespace in type names

A type name does not repeat its namespace. Exception: a stage's contract and vocabulary types that
other stages import (`LocalizationRequests`, `ResourceStore`, SYS-2) keep the stage name, so they read
the same everywhere. Restructure step 6 changes namespaces only and renames no type.
- Scope: C++. Enforce: review.
- Bad: `class Bno08xReports` in `namespace bno08x` (`pico/aggreGATR/src/bno08x_reports.h:14`); its
  neighbors are `bno08x::Supervisor` and `asm330::Supervisor`.

#### NAM-5 (MUST) Wire twins end in Code

A wire-code enum and its domain enum never share a name; the wire twin ends in `Code`.
- Scope: codec, Brain, Pi. Enforce: review.
- Bad: `translagatr::LocalizationTopology` (`translaGATR/link_documents.h:72`) and
  `communigatr::LocalizationTopology` (`brain/communiGATR/include/communigatr/robot_profile.h:22`).

#### NAM-6 (MUST) snake_case struct fields

Timestamp and other struct fields follow the field rule (snake_case).
- Scope: Pi. Enforce: clang-tidy naming. Legacy: deferred (38 fields).
- Bad: `pi/naviGATR/src/core/records.h:71` `MonotonicTime measuredAt;` next to snake_case
  neighbors.

#### NAM-7 (MUST) Names match their types

A member or parameter named after a type holds that type.
- Scope: C++. Enforce: review.
- Bad: a member called `diagnostics` is `Diagnostics*` in `core/execution_context.h:19` but
  `DiagnosticsHub*` in `contracts/slot_init.h:47` (both `pi/naviGATR/src/`).

#### NAM-8 (MUST) Accepted abbreviations only

No abbreviations beyond the accepted list in 10.3.
- Scope: All. Enforce: review.
- Bad: `wr16`, `rd16` (`translaGATR/frame_codec.cpp:12-31`); `h_seq`, `c_seq`,
  `r_hello` (`pi/naviGATR/src/inspection/inspection_service.cpp:600-611`).

#### NAM-9 (MUST) k means compile-time constant

The `k` prefix means compile-time constant and nothing else.
- Scope: C++. Enforce: clang-tidy naming.
- Bad: `double kP = 0;` (mutable PID gain fields, `brain/actuGATR/include/actugatr/pid.h:15`).

#### NAM-10 (MUST) One declarator per line

One declarator per line.
- Scope: C++. Enforce: clang-tidy `readability-isolate-declaration` (auto-fix).
- Bad: `pico/aggreGATR/src/imu_bno08x.cpp:75`
  `uint32_t g_boots = 0, g_retries = 0, g_rx = 0, g_bad_headers = 0;`

#### NAM-11 (SHOULD) Existing verb families

Keep the verb families already in use: `encodeX`/`decodeX`, `toX`/`fromX`, `xOf`, `makeX`
(unregistered helpers only; factories follow NAM-2), `valid`/`finite`, `toString` or `xName`.
Accessors are bare nouns with no `get`. Boolean queries are bare adjectives or participles
(`ready()`, `open()`, `terminal()`) (D-16).
- Scope: C++. Enforce: review.

#### NAM-12 (SHOULD) Role suffixes

Role suffixes: `Config`, `Status`, `Stats`, `Result`, `Request`, `Reply`, `Input`, `Output`.
Interfaces are plain role nouns with no `I` prefix.
- Scope: C++. Enforce: review.

#### NAM-13 (MUST) No leftover names

Leftover names from a rename are bugs: project, target, test and folder names follow the component.
- Scope: All. Enforce: review; script (old names list from `docs/naming.md`).
- Bad: `translaGATR/tests/CMakeLists.txt:2` `project(gatr2_common_tests CXX)`.

#### NAM-14 (MUST) Globals are g_ with an owner

A global or file-scope mutable variable is prefixed `g_`, sits in an anonymous namespace, and its
comment names the thread or task that owns it (CON-2).
- Scope: Brain (apps), Pico, Pi (mains). Enforce: clang-tidy `readability-identifier-naming`
  (GlobalVariablePrefix `g_`); review for the owner comment.
- Good: `pico/aggreGATR/src/main.cpp` `g_next_tick` (practice across 38 Pico globals).

### 4.5 Units and frames (UNIT-)

#### UNIT-1 (MUST) Every quantity names its unit

Every physical quantity carries its unit in its type (ALIAS-5) or its name suffix: `_m _mm _um _rad
_deg _cdeg _mdeg _s _ms _us _hz _m_s _rad_s _dps _mdps _px`. Constants use the camel form
(`kStaleMs`). A comment never carries the unit.
- Scope: All. Enforce: review.
- Bad: `pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:262`
  `double still_travel = 0.005;` read by `:267` `getDouble("max_calibration_travel_m", 0.005,
  still_travel, err)` (trimmed); the key has the unit and the variable does not.

#### UNIT-2 (MUST) SI inside, wire units at the edge

SI inside a pipeline (meters, radians, seconds). Wire units (mm, cdeg, us counts) appear only in the
code that encodes or decodes the wire, converted through named helpers.
- Scope: All. Enforce: review; script (grep for `180.0 /` and `3.14159` outside conversion
  modules).

#### UNIT-3 (MUST) One conversion module per side

One conversion module per side owns pi, the unit conversions, the wrap rule and the rounding policy
(saturate or refuse, in the function name). translaGATR owns wire units (`toCdeg`, `fromCdeg`,
`toMm`, ...); the Brain has one units header in investiGATR; naviGATR has `math/angles.h`.
- Why: `kPi` has 6 definitions and the wire heading rule "(-18000, 18000]" is written 3 times;
  one copy still has the bug the Pi fixed.
- Scope: All. Enforce: script (literal pi); test (edge values +-pi, 0, type limits).
- Bad: `brain/actuGATR/src/telemetry.cpp:42-47`: the comment says
  `// Angle to centidegrees in (-18000, 18000].` but the code is
  `return i16(investigatr::wrapAngle(rad) * kCdegPerRad);` with no mapping of -18000, so an angle
  just above -pi yields -18000 (display only; found in review, not by a test).

#### UNIT-4 (MUST) Transforms are T_a_b

A pose or transform is named `T_a_b`: the pose of frame b in frame a, so `compose(T_a_b, T_b_c)` is
`T_a_c` (`pi/naviGATR/src/math/transforms.h:2-4`). A quantity whose frame is not the module's stated
default carries the frame in its name.
- Scope: All. Enforce: review.
- Bad: `pi/naviGATR/src/state/robot_state.h:45-46` `Pose2D odom_pose; // T_odom_robot` and
  `Pose2D field_from_odom; // T_field_odom, identity until re-anchored`: the frames live in
  comments while a method on the same struct is `T_odom_robot3()`.

#### UNIT-5 (SHOULD) State the default frame

Each module header states its default frame in one line.
- Scope: All. Enforce: review.
- Good: `brain/investiGATR/include/investigatr/geometry.h:2-3`.

#### UNIT-6 (MUST) Checked narrowing

Narrowing to a wire or fixed-width type goes through a named saturating or checked helper (NUM-3 for
floating to integer). Type limits come from `std::numeric_limits`, never written out.
- Scope: All. Enforce: compiler `-Wconversion -Wsign-conversion` (gate for codec and Pico,
  advisory elsewhere); review.
- Bad: `std::clamp(mm, -2147483648.0, 2147483647.0)` at
  `pi/naviGATR/src/impl/publishing/brain_link_publisher.cpp:39` and the same limits at
  `pi/naviGATR/src/runtime/system.cpp:670` (trimmed).

### 4.6 Time and clock domains (TIME-)

#### TIME-1 (MUST) Portable code reads no clock

Portable code never reads a clock. A time-dependent function takes `now` (and `dt` where needed) as
a parameter. Clock reads live in the program entry, the platform shells and one clock module per
program; code may time its own work for statistics.
- Why: deterministic tests with injected time; no hidden time base.
- Scope: All. Enforce: script (grep `steady_clock::now`, `system_clock`, `HostClock::now`,
  `pros::millis`, `micros(` outside an allowlist).
- Good: `brain/communiGATR/include/communigatr/client.h:5-6` "No clock and no threads:
  poll(now) does all I/O and never blocks" (trimmed).

#### TIME-2 (MUST) One steady clock per process

One steady clock source per process: `HostClock` on the Pi, `ProsLink::now` on the Brain, one time
module on the Pico. A second clock with another origin is removed, not aliased.
- Scope: Pi, Brain, Pico. Enforce: review; TIME-1 script.
- Bad: `pi/naviGATR/src/resources/serial_link.h:32-33` "Steady clock microseconds, arbitrary
  origin." `inline int64_t steadyNowUs() {` next to `HostClock::nowUs()` counting from process
  start; both are `int64_t` microseconds.

#### TIME-3 (MUST) Timestamps name clock and unit

Every timestamp names its clock domain and unit, in its type (`MonotonicTime` with its
`ClockDomain`) or its name (`_host_us`, `_device_ms`) or an alias (`HostMicros`, `MicrosU32`).
Values from two domains are never compared or subtracted without a mapping function
(`DeviceToHostClock`).
- Scope: All. Enforce: review ("which clock?" is REV-7).

#### TIME-4 (MUST) MonotonicTime checks its domain

`MonotonicTime` equality checks the domain the same way ordering does, and the domain check runs in
every build, not only Debug. An unset time never compares or subtracts: `sameDomain` is true only
for two equal set domains, and code tests `isSet()` first. The difference of two `MonotonicTime`
values is a duration type or alias with its unit in the name (`Millis`).
- Scope: Pi. Enforce: test.
- Bad: `pi/naviGATR/src/core/time.h:46`
  `inline bool operator==(MonotonicTime a, MonotonicTime b) { return a.ms == b.ms; }` while
  `operator<` at `:48` asserts `sameDomain(a, b)`.
- Bad: `pi/naviGATR/src/core/time.h:36-38` `return a.domain == b.domain || a.domain ==
  ClockDomain::kUnset ||` / `b.domain == ClockDomain::kUnset;` lets an unset time pass the check;
  `:41` `inline int64_t operator-(MonotonicTime a, MonotonicTime b) {` returns a unitless
  `int64_t` (TIME-3).

#### TIME-5 (MUST) Duration types

Duration types. Pi: `int64_t` with `_ms` or `_us`, or `double` with `_s`; never `long` or `int`;
config readers return `int64_t`. Brain: `Seconds`; PROS task periods and delays stay `uint32_t
..._ms` at the PROS call. Pico: `uint32_t ..._us` for the wrapping clock, `uint64_t` for ages that
must not wrap (D-17).
- Why: `long` is 32-bit on the Windows host build and 64-bit on the Pi.
- Scope: All. Enforce: review; script (grep `long ` in declarations). Legacy: deferred to the
  config reader (CFG-3).
- Bad: `pi/naviGATR/src/config/config_node.h:43`
  `bool getInt(const char* key, long def, long& out, std::string& err) const;`
- Bad: `pi/naviGATR/src/impl/localization/tracking_wheel_motion.h:129` `long max_gap_ms = 250;`
  (spacing trimmed).

#### TIME-6 (MUST) Wrapping clocks through one helper

A wrapping clock is compared only through one named helper per clock type, never through an inline
cast.
- Scope: Pico, Brain. Enforce: review; script (grep `static_cast<int32_t>(.* - `).
- Good: `pico/aggreGATR/src/gyro_window.h:12`
  `constexpr bool atOrBefore(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) <= 0; }`
- Bad: `pico/aggreGATR/src/main.cpp:326` writes the same compare inline
  (`static_cast<int32_t>(now - g_next_tick) >= 0`); 9 such sites today.

#### TIME-7 (MUST) Measurement time at measurement

A time that means "when measured" is taken at the measurement, never at send time or build time.
- Why: bug class B1 (telemetry plotted at message build time; publish latency that included the
  document build).
- Scope: All. Enforce: review.

#### TIME-8 (MUST) Injected clocks

Code with timing logic takes its clock injected (a clock alias, ALIAS-3), so tests step time instead
of sleeping.
- Scope: All. Enforce: review; TEST-6 grep.
- Bad: `pi/naviGATR/src/runtime/handoff.h:153`
  `const auto now = std::chrono::steady_clock::now();` inside `WorkerStats`, which forces
  `std::this_thread::sleep_for(Ms(2200));` in `pi/naviGATR/tests/workers_gtest.cpp:573`.
- Good: `pi/naviGATR/src/capture/capture_recorder.h:57`
  `std::function<int64_t()> clock; // Pi host us; HostClock::nowUs when empty` (injectable; still
  needs its alias).

#### TIME-9 (SHOULD) std::chrono only at edges

`std::chrono` appears only at sleep and wait edges; pipeline structs use `MonotonicTime` and
unit-suffixed integers.
- Scope: Pi. Enforce: review.

#### TIME-10 (MUST) Clock resolution fits the deadline

A clock that times an interval has a resolution at least ten times finer than the shortest interval
it decides on; sub-millisecond deadlines use a microsecond clock.
- Why: bug class B1 (`MonotonicTime` counts whole milliseconds, so the 1000 us and 200 us guards
  could not be measured; the fix moved those deadlines to microseconds).
- Scope: All. Enforce: review.

#### TIME-11 (MUST) Wall clock for labels only

Wall-clock time (`system_clock`, unix ms, `gmtime`) is used only for labels and file names, never
for durations, timeouts, freshness or ordering, and it is injected like any clock (TIME-8).
- Why: the Pi has no RTC; `system_clock` jumps when NTP syncs.
- Scope: All. Enforce: script (TIME-1 list includes `system_clock`); review.
- Good: `pi/naviGATR/src/capture/capture_service.cpp:323-324`
  `return std::chrono::duration_cast<std::chrono::milliseconds>(`
  `std::chrono::system_clock::now().time_since_epoch())` behind the injectable `wall_ms`, used for
  labels only.

### 4.7 Errors (ERR-)

#### ERR-1 (MUST) No exceptions

Project code throws no exceptions on any target. A third-party call that can throw (`std::thread`
creation, libcamera, `std::filesystem` without `error_code`) is wrapped at the call and turned into
the local error form (D-2). Project code calls the non-throwing forms: `std::from_chars` instead of
`std::stoi`/`std::stod`, a checked index instead of `.at()`, `*opt` after a check instead of
`.value()`, `std::get_if` instead of `std::get`.
- Why: P-7; one error style; the Pico builds with `-fno-exceptions -fno-rtti`.
- Scope: C++. Enforce: script (grep `throw`, `std::sto`, `.at(`, `.value()`); clang-tidy
  `bugprone-exception-escape` with the thread entries listed (9.2). Legacy: deferred (phase 2
  fixes the `system.cpp:934` throw).
- Bad: `pi/naviGATR/src/runtime/system.cpp:934` `throw std::runtime_error(err);` in
  `resumeWorkers`, reachable from `applyPendingProfile()`, which `pi/naviGATR/src/app/main.cpp:174`
  calls with no `try`, so a failed thread start ends the process.
- Bad: `pi/naviGATR/src/inspection/http_server.cpp:363`
  `out.push_back(static_cast<char>(std::stoi(s.substr(i + 1, 2), nullptr, 16)));`, safe only
  because of the `std::isxdigit` checks on `:361-362`.

#### ERR-2 (MUST) One error shape per layer

- Pi construction and parsing: return `std::optional<T>` (or `std::unique_ptr<T>` for a
  polymorphic result) and take `std::string& err` as the last parameter; the message starts with
  the XML path. `bool` plus an output parameter only when filling a field of an existing struct.
- Brain, Pico, codec: `bool` for pass/fail; an `enum class` status, or a result struct whose first
  field is `status`, when the caller branches on why; `toString(enum)` for text. The
  `const char** why` form is retired.
- Scope: C++. Enforce: review.
- Good: `err = node.path() + ": needs <Source resource_id=... output_id=.../>";` in
  `pi/naviGATR/src/impl/sensors/attitude_channel.cpp` (trimmed).

#### ERR-3 (MUST) nodiscard results

Status and result types carry `[[nodiscard]]` on the type (`enum class [[nodiscard]]
FunctionStatus`, `struct [[nodiscard]] XyzOutput`), which covers every function returning them. A
function that reports success as `bool` or `std::optional` carries `[[nodiscard]]` on its
declaration. Each component's status types are listed in
`bugprone-unused-return-value.CheckedReturnTypes` (9.2). A deliberate discard is
`static_cast<void>(call()); // best effort`.
- Scope: C++. Enforce: compiler (a discarded `[[nodiscard]]` value warns by default); clang-tidy
  `bugprone-unused-return-value` with the 9.2 options (0 findings on the host production code
  today); review for `bool` returns. Legacy: deferred (0 `[[nodiscard]]` today; added per
  component in phase 5).
- Bad: `brain/communiGATR/pros/pros_link.cpp:172` `serial_->open();` with its result dropped.

#### ERR-4 (MUST) Failures are values

Runtime failures are values (`FunctionStatus`, `SourceState`, a diagnostic string, a status enum).
Programming errors found at startup (duplicate registration, broken table) abort loudly with a
message.
- Scope: C++. Enforce: review.
- Good: `pi/naviGATR/src/runtime/register_all.h:34-39` `registerOrDie` prints
  "duplicate function registration" and calls `std::abort()` (paraphrased).

#### ERR-5 (MUST) Missing hardware degrades

Absent or failing hardware at startup is a build warning plus a runtime status, never a failed build
and never a blocking wait.
- Scope: All. Enforce: test (device missing); review.
- Good: `pi/naviGATR/src/impl/resources/serial_links.cpp:183`
  `context.warnings->push_back(node.path() + ": " + open_err);`

#### ERR-6 (MUST) Asserts and reported checks

An assert never replaces a check a caller can violate, and has no side effects. Brain and Pico code
has no `assert()`; it uses `static_assert` and reported checks. On the Pi, asserts guard impossible
states only and stay active in test builds. A reported check is `if (!cond) { ++faults_.<name>;
return <status>; }`, with the counter surfaced per ERR-9. This is a named deviation from Power of
Ten rule 5: the check degrades instead of halting.
- Why: `assert` vanishes under NDEBUG, and when active it stops the robot, the opposite of P-7.
- Scope: C++. Enforce: clang-tidy `bugprone-assert-side-effect`; script (no `assert(` under
  `brain/`, `pico/`, `translaGATR/`).

#### ERR-7 (MUST) static_assert constant relations

Every relation between compile-time constants that code relies on is a `static_assert` with a short
message: frame sizes, capacities, ports, pin budgets, SDK id matches.
- Scope: C++. Enforce: review.
- Bad: `translaGATR/frame_codec.h:21`
  `constexpr uint8_t kBrainPayloadMax = kMaxFrameLen - kLinkEnvelopeLen; // 122` (spacing
  trimmed): the comment is an unchecked assertion.
- Good: `pico/aggreGATR/src/config.h:77`
  `static_assert(pinsUnique(kAllPins, kPinsAssigned), "pin assigned twice or not a usable Pico GPIO");`

#### ERR-8 (retired) Positional braces for results

See CONST-5.

#### ERR-9 (MUST) Every discard is counted

Every discard is counted where it happens and surfaced in status, diagnostics or capture metadata. A
counter nobody reads is a defect. Every output states its own completeness (truncated, dropped,
stale).
- Why: bug class B12 (the VEX bench dropped 200 to 250 ms gaps silently; `drops_` was unread).
- Scope: All. Enforce: review; test pattern "overflow reports itself" (TEST-10).

#### ERR-10 (MUST) errno only in shells

`errno` is read or set only in platform shells (`pros/`, POSIX adapters).
- Scope: Brain, Pi. Enforce: review.

#### ERR-11 (MUST) Catch-all at thread entries

`main` and every thread or task entry on the Pi end in one catch-all that records a fault event with
the thread name and exits nonzero, so the service restart is explained in the log.
- Why: an exception escaping a thread function calls `std::terminate` (CERT ERR51-CPP). The
  service restarts (`pi/naviGATR/tools/install_service.sh:87-88` `Restart=always` /
  `RestartSec=3`), so the cost is a 3 s outage and a lost placement with no record of why.
- Scope: Pi. Enforce: clang-tidy `bugprone-exception-escape` with the entries listed (9.2);
  review.
- Bad: neither worker loop catches anything: `pi/naviGATR/src/runtime/system.cpp:1411`
  `void System::estimationWorker() {` and `:1436` `void System::fieldWorker() {`.

### 4.8 Logging and events (LOG-)

#### LOG-1 (MUST) Libraries never print

Libraries never print. Only program entry files and tools write to stdout or stderr. Libraries
report through return values, warning lists, status plus diagnostic, counters, event rings and hub
records.
- Scope: C++. Enforce: script (grep `printf`, `std::cout`, `std::cerr` outside mains, tools
  and the abort path).
- Good: naviGATR `printf` appears only in `app/main.cpp`, `app/field_refs.cpp` and the
  `registerOrDie` abort.

#### LOG-2 (SHOULD) One typed event call

One event call with a typed kind fans out to the log, the diagnostics hub and the capture triggers.
Tests match the kind, never the text.
- Why: callers raise `noteEvent` and `noteFault` with the same text at
  `pi/naviGATR/src/runtime/system.cpp:748-749` ("Brain link quiet for 1 s" twice).
- Scope: Pi. Enforce: review. Legacy: deferred to follow-up F7.

#### LOG-3 (retired) Non-blocking target output

See RT-4.

#### LOG-4 (SHOULD) Short event text

Event and warning text is short, lowercase, and names the thing (id or XML path) first.
- Scope: All. Enforce: review.

#### LOG-5 (MUST) Counters say what they count

A counter counts one event kind and its name says which (`replaced`, `refused`, `expired`); two
kinds are two counters. A report column or label names its counter exactly.
- Why: bug class B9 ("Snapshots refused" also counted replacements; a seq-gap column overstated
  replacements).
- Scope: All. Enforce: review.
- Good: `brain/communiGATR/include/communigatr/pros_usb_port.h:22`
  `uint32_t expired       = 0; // queued frames older than the lifetime, never written`.

### 4.9 Ownership and lifetime (OWN-)

#### OWN-1 (MUST) Ownership in the type

Ownership is visible in the type:
- `std::unique_ptr<T>` owns;
- `std::shared_ptr<const T>` for immutable data shared across threads and for snapshots (never
  across PROS tasks, RT-5);
- non-const `std::shared_ptr<T>` only for a device or service several consumers use for their
  whole life, with a stated thread rule;
- `T&` for a required non-owning dependency that outlives the holder;
- `T*` for an optional or rebindable non-owning dependency (may be null).
- A raw pointer never owns.
- Scope: C++. Enforce: review.

#### OWN-2 (MUST) make_unique and make_shared

Create with `std::make_unique` / `std::make_shared`. No naked `new` except inside a factory for a
private constructor; no `delete`.
- Scope: C++. Enforce: clang-tidy `modernize-make-unique`, `modernize-make-shared`,
  `cppcoreguidelines-no-malloc`.
- Bad: `brain/operaGATR/src/main.cpp:133`
  `g_vex.reset(new communigatr::ProsVexImu(gatr2_robot::kVexImuPort));`
- Bad: `brain/communiGATR/include/communigatr/pros_usb_port.h:56` `Io* io_;` with
  `delete io_;` at `brain/communiGATR/pros/pros_usb_port.cpp:179`.

#### OWN-3 (MUST) Visible deliberate leaks

A deliberate leak is a `std::unique_ptr` plus `release()` with a label comment at the release.
- Why: the `Io` leak above is intentional ("a task may own a kernel serial mutex")
  but invisible in the type.
- Scope: Brain. Enforce: review.

#### OWN-4 (MUST) No references into shared state

No public API returns a reference or pointer into state that another thread writes. Readers get a
copy, or on the Pi a `std::shared_ptr<const Snapshot>` (RT-5).
- Scope: Pi, Brain. Enforce: review.
- Bad: `pi/naviGATR/src/runtime/system.h:278` `Diagnostics& diagnostics() { return diagnostics_; }`
  (spacing trimmed).

#### OWN-5 (MUST) Handle holders are not copyable

A class that holds a reference member or a hardware handle deletes copy and move unless it has a
reason to allow them.
- Why: `Client`, `Drive`, `Motion` and `LinkDriver` hold references and are implicitly copyable;
  a copy would alias the port or the motors.
- Scope: C++. Enforce: script (a class with a `&` data member or a device handle and no
  `= delete` copy constructor); review.

#### OWN-6 (SHOULD) Lifetimes stated at the declaration

A lifetime requirement is stated at the declaration: who must outlive whom, and where member order
matters for destruction.
- Scope: C++. Enforce: review.
- Good: `brain/actuGATR/include/actugatr/pros_drive.h:34`
  `// owner and drive must outlive this object; ...` (trimmed).
- Good: `pi/naviGATR/src/runtime/system.h:382` `// destruction order: declared first, destroyed last`.

#### OWN-7 (MUST) Config views do not outlive the document

A configuration view never outlives its document; implementations copy what they keep. This is the
config case of OWN-10.
- Scope: Pi. Enforce: review.
- Good: `pi/naviGATR/src/config/config_node.h:5-6` "Views are valid only while the document is
  loaded, i.e. during initialization; implementations copy what they keep."

#### OWN-8 (SHOULD) Share large buffers

Large buffers (images, point sets) are shared as `std::shared_ptr<const T>`, never copied per
consumer.
- Scope: Pi. Enforce: review.
- Good: `pi/naviGATR/src/resources/camera.h:74` camera pixels held as one shared const buffer.

#### OWN-9 (MUST) Stored lambdas capture by value

A lambda that outlives the current call (stored in a member, an executable, a callback, a thread or
a task) has an explicit capture list and captures by value: copies or `std::shared_ptr`. `[&]` only
for a lambda that dies before the call returns. `[this]` only when the owner joins or stops the
callee before destruction, and the class deletes copy and move.
- Why: Google style: "Use default capture by reference ([&]) only when the lifetime of the lambda
  is obviously shorter than any potential captures"; C++ Core Guidelines F.53. The stage design
  stores executables, so this is where a dangling capture would live.
- Scope: C++. Enforce: clang-tidy `bugprone-capturing-this-in-member-variable`; review.
- Good: `pi/naviGATR/src/impl/resources/pico_telemetry.cpp:847`
  `executable.execute = [telemetry, outputs](const ExecutionContext& context) {`.
- Good: `brain/communiGATR/pros/pros_link.cpp:92` `client_(port_, [this] { return nonce(); }, ...`
  (trimmed), with copy deleted at `brain/communiGATR/include/communigatr/pros_link.h:106-107`.

#### OWN-10 (MUST) Views are call-scoped

A struct of references (stage inputs, `ConfigNode`) is a call-scoped view: built at the call, passed
down, never stored in a member, container, lambda capture or return value. No object is used after
`std::move`; no reference or `string_view` outlives its source.
- Scope: C++. Enforce: clang-tidy `bugprone-use-after-move`, `bugprone-dangling-handle`,
  `bugprone-return-const-ref-from-parameter`; compiler `-Wdangling-reference`; review.

### 4.10 Concurrency (CON-)

#### CON-1 (retired) One writer per state

See SYS-15.

#### CON-2 (MUST) Thread tags

Every type or global touched by more than one thread or task carries a thread tag in one fixed form:
`// thread: any`, `// thread: estimation worker`, `// thread: field worker`, `// thread: stopped or
inline`, `// task: drive`, `// task: link poll`. A member that differs from its type's tag carries
its own.
- Why: 52 thread annotations exist today in many spellings ("Estimation worker only", "any
  thread", "Inline mode or stopped workers only").
- Scope: Pi, Brain. Enforce: review; script (tag grammar).

#### CON-3 (MUST) One lock at a time

Never hold two mutexes at once; if nesting ever becomes necessary the class comment states the
order. No I/O, sleep, callback or call into another module while holding a lock: copy under the
lock, act outside it.
- Scope: Pi, Brain. Enforce: review; clang `-Wthread-safety` if clang is adopted (D-13).

#### CON-4 (MUST) Bounded waits, joined threads

Every blocking wait has a predicate and a timeout, and the timeout is counted. Threads are joined,
never detached. Tasks stop cooperatively, never by deletion while they may hold a lock.
- Scope: Pi, Brain. Enforce: review; clang-tidy `bugprone-spuriously-wake-up-functions`.
- Good: `pi/naviGATR/src/runtime/handoff.h:58`
  `cv_.wait_for(lock, timeout, [&] { return stopped_ || pending_.has_value(); });`

#### CON-5 (MUST) Atomics for flags and counters

Atomics only for independent flags and counters, never to publish multi-field state. Exception: a
single-producer single-consumer ring, one reviewed type per platform, with its producer and consumer
named in its thread tag and its own tests, where a lock cannot be used (an ISR, or a PROS task that
can block or be deleted). `volatile` is never used for thread synchronization on the Pi or Brain
(the only exception is a `volatile std::sig_atomic_t` signal flag).
- Scope: Pi, Brain. Enforce: script (grep `volatile` outside `pico/`, with an allowlist).
- Good (the ring exception): `brain/communiGATR/pros/pros_usb_port.cpp:40-41`
  `slots_[head % Capacity] = frame;` then `head_.store(head + 1, std::memory_order_release);`;
  its header says why: "a blocked USB write (cable out) stalls only the transmit task, never the
  caller" (`brain/communiGATR/include/communigatr/pros_usb_port.h:5-7`, trimmed).

#### CON-6 (MUST) RAII locks only

RAII lock guards only; no manual `take`/`give` or `lock`/`unlock` pairs.
- Scope: Pi, Brain. Enforce: script (`.take(`, `.give(`, `.lock()`, `.unlock()` outside the one
  guard type per platform); clang-tidy `bugprone-unused-raii` for guards that die at once.
- Bad: `brain/actuGATR/pros/pros_drive.cpp:28-34` `if (!mutex_.take(config_.lock_timeout_ms)) {`
  ... `mutex_.give();` (trimmed); an early return added later leaks the lock.
- Good: `class ProsLink::Lock` (`brain/communiGATR/pros/pros_link.cpp:64-82`).

#### CON-7 (SHOULD) Locked suffix

A function that must be called with a lock held says so by a `Locked` suffix, not by a comment.
- Scope: Pi, Brain. Enforce: review.

#### CON-8 (MUST) Stop workers before lifecycle changes

Lifecycle changes (reset, profile swap, stop) stop and join the workers first, then restart them. No
stage is reset while running.
- Scope: Pi. Enforce: review; test.

#### CON-9 (MUST) Bounded queues with counters

Every queue, ring or handoff has a capacity, a drop policy (drop newest, drop oldest, replace) and a
drop counter that something reads. Streaming state is latest-wins; events and documents are bounded
FIFO; memory limits count the bytes actually held. A latest-wins stream over a socket or serial port
bounds the OS buffer too (a small send buffer, `TCP_NOTSENT_LOWAT`, `TIOCOUTQ`, flow control), or it
is not latest-wins.
- Why: bug class B7 (a FIFO of whole snapshots made a slow viewer see poses 9 to 12 s old; later,
  kernel and tunnel buffering bypassed latest-wins).
- Scope: All. Enforce: review; test pattern "stalled consumer" (TEST-10).
- Good: `pi/naviGATR/src/diagnostics/hub.h:5-7` "When the ring is full the new record is dropped
  and counted per kind, so a stalled consumer can never slow a producer or grow memory."
- Good: `pi/naviGATR/src/inspection/http_server.cpp:632-633` "A small kernel send buffer bounds
  what already-written frames can wait in the kernel; TCP_NOTSENT_LOWAT (Linux) bounds the unsent
  part" (trimmed); `pi/naviGATR/src/transport/serial_port.cpp:242` polls `TIOCOUTQ`.

#### CON-10 (MUST) Deadlines checked on every call

A state that waits for an input or reply checks its deadline on every invocation, before it looks at
input. Every scheduler or priority policy states its fairness bound (the longest any request class
can wait) in its PIPELINE.md, and a test asserts that bound under the worst link timing.
- Why: bug class B11 (a deadline checked only when the awaited input arrived; scheduler
  starvation).
- Scope: All. Enforce: review (REV-9); test patterns "no input at all", "slow link" (TEST-10).
- Good: `pi/naviGATR/src/impl/localization/motion_step.h:135-136` "Every motion window gets a
  deadline from the first call that sees it, checked on every call whether or not a heading is
  offered" (trimmed).

#### CON-11 (MUST) Default memory order

Atomics use the default `memory_order_seq_cst`, except `memory_order_relaxed` for statistics
counters and acquire/release inside the CON-5 ring type. Any other lock-free code needs an EXCEPTION
tag.
- Why: C++ Core Guidelines CP.100: "Don't use lock-free programming unless you absolutely have
  to".
- Scope: Pi, Brain. Enforce: script (grep `memory_order_` outside the ring type); review.
- Good: `brain/communiGATR/pros/pros_usb_port.cpp:61-62` `void bump(std::atomic<uint32_t>&
  counter) {` `counter.fetch_add(1, std::memory_order_relaxed);`.

### 4.11 Real-time and embedded (RT-)

#### RT-1 (MUST) No heap on the Pico or codec

Pico firmware and translaGATR never touch the heap: no `new`, no allocating standard containers, no
`std::function`, no `std::string`. Capacities come from named constants, with `static_assert` where
sizes interact.
- Scope: Pico, codec. Enforce: script (grep; `arm-none-eabi-nm` over our own objects in
  `.pio/build/<env>/src/` finds no `malloc`, `free`, `_Znwj`, `_Znaj`).
- Good: `pico/aggreGATR/src/gyro_window.h:46` `Report buf_[kCapacity] = {};`

#### RT-2 (MUST) No steady-state allocation on the Brain

Brain periodic loops (link poll task, drive task, USB tasks) do not allocate in steady state.
Allocation happens at startup and on events (a new plan, a new field document, a profile change),
each bounded by a named limit, and the header says so in one line.
- Scope: Brain. Enforce: review.
- Good: `brain/investiGATR/include/investigatr/planner.h:63`
  `std::size_t max_vertices = 512; // graph bound, kNoPath beyond it; ...` (trimmed).

#### RT-3 (MUST) Minimal ISRs

Pico ISRs read hardware and write one-word `volatile` flags, counters or timestamps only: no
allocation, `delay()`, Serial, SPI or floating point. Multi-field reads or resets of ISR state
happen with interrupts off, in one named function. ISR functions are named `on*` or `isr*`. This
holds for single-core firmware (no `setup1`/`loop1` today); state shared with core 1 uses an RP2040
spin lock or the multicore FIFO, never `noInterrupts()` alone.
- Scope: Pico. Enforce: review.
- Good: `pico/aggreGATR/src/imu_bno08x.cpp:87-88` `void onInt() { g_edge_us = time_us_32(); ...`
  (trimmed) and `pico/aggreGATR/src/quadrature.cpp:24` `volatile int32_t count = 0;`

#### RT-4 (MUST) Target output never blocks

Output on a real-time target never blocks: check for space, then drop and count. The Pico skips a
tick rather than block: a frame is written only into an empty UART FIFO, and every frame fits the
FIFO (checked by `static_assert`).
- Scope: Pico, Brain. Enforce: review.
- Good: `pico/aggreGATR/src/main.cpp` loop: `// Skip the tick rather than block.` then
  `if (!txFifoEmpty()) { ++g_ticks_skipped; return; }` (trimmed).
- Good: `pico/aggreGATR/src/main.cpp:239` "USB is optional: never wait for a monitor or for space
  in its transmit FIFO."

#### RT-5 (MUST) No threads in Brain libraries

Brain portable libraries have no threads, locks or atomics; tasks, mutexes and atomics live in
`pros/` shells. No `std::shared_ptr` crosses PROS tasks, and no function-local static with a dynamic
initializer is reachable from two tasks (the PROS toolchain is built with `--disable-threads`:
`shared_ptr` counts are not atomic and function-local static initialization is not guarded; checked
by compiling `__gnu_cxx::__default_lock_policy`, which is `_S_single`). A three-line `static_assert`
test in the PROS build notices a toolchain change. State shared between tasks is atomic or guarded
and names its lock or owning task.
- Scope: Brain. Enforce: script (grep `pros::Mutex`, `std::atomic`, `pros::Task` outside `pros/`
  and apps); review.
- Bad: `brain/operaGATR/src/main.cpp:55-56` `const char* g_message` and `double g_speed_scale`
  written in `opcontrol()` and in the background task with no guard.

#### RT-6 (MUST) Bounded PROS locks, named tasks

Every PROS mutex take is bounded and goes through one shared RAII guard type (CON-6). Tasks are
created through one API (`pros::Task`) with a name, and priority and stack from config constants.
One task owns each motor or device. Every task loop delays.
- Scope: Brain. Enforce: review.

#### RT-7 (MUST) Competition callbacks only post

Competition callbacks (`autonomous`, `opcontrol`) only post requests or make bounded calls. Loops
that talk to the link run in the program's own task.
- Why: PROS deletes competition tasks at mode changes without releasing held mutexes.
- Scope: Brain (apps). Enforce: review.
- Bad: `brain/operaGATR/src/main.cpp:471`
  `g_message = g_link->place(gatr2_robot::kStartPose) != 0 ? ...` called directly in
  `opcontrol()` (trimmed).

#### RT-8 (MUST) Long blocking calls off control tasks

Calls that block for long (smart port open, about 100 ms; VEX IMU recalibration, about 1 s; a USB
write while the cable is out) stay off control tasks.
- Scope: Brain. Enforce: review.

#### RT-9 (MUST) No recursion on targets

No recursion in Brain, Pico or codec code. Every loop has a bound except one server loop per task or
thread; every busy-wait has a timeout. On the Pi, recursion needs a stated depth bound (D-18).
- Scope: Brain, Pico, codec, Pi (review only). Enforce: clang-tidy `misc-no-recursion` overlay on
  `brain/`, `pico/`, `translaGATR/`; review.
- Good: `pico/aggreGATR/src/imu_bno08x.cpp:193-194` `while (!intLow()) {` guarded by
  `if (time_us_32() - t0 > kWakeUs) {`.

#### RT-10 (MUST) Deliberate floating point on the RP2040

On the RP2040 (Cortex-M0+, no FPU) floating point stays out of ISRs and per-tick paths unless
measured; `float` versus `double` is deliberate.
- Scope: Pico. Enforce: compiler `-Wdouble-promotion` on the Pico target.

#### RT-11 (MUST) Hardware-free Pico logic

Pico logic lives in hardware-free modules that include no Arduino or SDK header and are host-tested.
Only the driver files touch hardware (`main.cpp`, `imu_bno08x.cpp`, `imu_asm330.cpp`,
`quadrature.cpp` today).
- Scope: Pico. Enforce: test (host build of every non-driver source).

#### RT-12 (MUST) translaGATR builds everywhere

translaGATR compiles on all three targets: C++17, no heap, no exceptions, no RTTI, fixed-width
types, explicit little-endian byte access (never `memcpy` of a struct), and no include of any other
component.
- Scope: codec. Enforce: CI builds on host, PROS and Pico; script (includes).
- Good: `translaGATR/frames.h:2-3` "Wire format shared by the Pico, Pi, and brain. No hardware
  dependencies. Must compile on all three targets."

#### RT-13 (MUST) Per-function stack budgets

Per-function stack budgets on targets: 1024 bytes on the Pico, 4096 bytes on the Brain.
- Scope: Pico, Brain. Enforce: compiler `-Wstack-usage=1024` (Pico), `-Wstack-usage=4096`
  (Brain PROS build).

#### RT-14 (MUST) Actuator commands expire

Every actuator output has an age limit enforced by the task that owns the device, not by its
callers: a command not refreshed within the limit is replaced by the device's safe output (stop,
brake) and counted. The limit is a named config value. Test pattern "source stops" applies to every
actuator.
- Why: bug class B6 (the motors kept their last voltage after a deadlocked drive task).
- Scope: Brain (actuators), Pi (actuators), Pico (actuators). Enforce: review; test.
- Good: `brain/actuGATR/include/actugatr/drive.h:2-5` "the motors stop at once for anything Drive
  cannot execute: ... or a command older than command_timeout" (trimmed), `:26`
  `kStale,             // older than command_timeout`, `:32`
  `Seconds         command_timeout = 0.1;`.

#### RT-15 (MUST) Bounded external input

Every reader of bytes from outside the process (socket, serial port, file, XML, capture bundle) caps
the size it accepts and rejects the rest with a status; every server caps its connections; every
writer of files caps total size and count and states what it deletes.
- Why: P-9; WIRE-8 covers only wire bytes.
- Scope: Pi, tools. Enforce: review; test pattern "upstream sends garbage", extended to oversize
  input.
- Good: `pi/naviGATR/src/inspection/http_server.cpp:60-61`
  `constexpr std::size_t kMaxRequestHead   = 16 * 1024;` /
  `constexpr std::size_t kMaxRequestBody   = 64 * 1024;`, `:68`
  `constexpr std::size_t kMaxHttpConnections = 32;`;
  `pi/naviGATR/src/capture/capture_config.h:54-55`
  `long        max_mb            = 64;` / `long        keep              = 3;`.

#### RT-16 (SHOULD) Call-chain stack report

Target builds also emit `-fstack-usage` and `-fcallgraph-info=su`; a report sums the deepest chain
from each task or loop entry and compares it with that task's stack.
- Why: RT-13 is per function; eight 1 KB frames in one chain pass it and overflow the 8 KB Pico
  core-0 stack.
- Scope: Pico, Brain. Enforce: report.

### 4.12 Interfaces and call sites (API-)

#### API-1 (MUST) Public API is the contract headers

A component's public API is its contract headers (and, for a library, `include/<ns>/`). Everything
else is private: an anonymous namespace, a private header next to its source, or a nested `detail`
type.
- Scope: C++. Enforce: review.

#### API-2 (SHOULD) One level of abstraction

A function reads at one level of abstraction. A policy function reads as the policy (a priority
list, a stage list); the building, parsing and byte work under it are named helpers one ctrl+click
away.
- Why: the lead: "the code itself when being read should be understood at a system level easily".
- Scope: All. Enforce: review; CPLX limits.
- Bad: `communigatr::Client::choose` (`brain/communiGATR/src/client.cpp:869`) is the request
  priority policy, but each branch also builds the request byte by byte, e.g. `:941-946`
  `request.op = translagatr::kOpProfileWrite; request.offset = offset; ...` (trimmed).

#### API-3 (MUST) Wrap repeated low-level sequences

A low-level sequence repeated at two or more call sites is wrapped in a small named helper (a
"driver") so the call site states intent in one line: XML or config reading, XML writing, JSON
writing, frame building, serial setup, reopen-with-retry, error plumbing. This is extraction of an
existing pattern and is allowed without a second design review (P-6).
- Scope: All. Enforce: review.
- Good: `ConfigNode` (`pi/naviGATR/src/config/config_node.h`) and `JsonWriter`
  (`pi/naviGATR/src/inspection/json_writer.h`).
- Bad: `pi/naviGATR/src/impl/resources/pros_usb_link.cpp:151-168` repeats the termios setup of
  `transport/serial_port.cpp`.

#### API-4 (SHOULD) Input structs over long parameter lists

More than 5 parameters become an input struct, the same pattern the stage contracts use. More than 7
fails CPLX-4.
- Scope: All. Enforce: lizard `-a` (CPLX-4).
- Bad: `pi/naviGATR/src/impl/localization/motion_step.cpp:367` `prepareStep`, 7 parameters.

#### API-5 (MUST) Plain over clever

Plain over clever: standard helpers over hand-rolled ones (`std::clamp` over nested ternaries),
structured bindings over `.first`/`.second`, early exits over deep nesting, no `else` after
`return`.
- Scope: C++. Enforce: clang-tidy `readability-else-after-return`; review.
- Bad: `pi/naviGATR/src/math/se3.h:73-75` `pitch_rad = std::asin(-R.m[2][0] < -1.0 ? -1.0 : ...`
  (trimmed); plain: `std::asin(std::clamp(-R.m[2][0], -1.0, 1.0))`.
- Bad: 44 loops read `kv.first` / `kv.second` and the tree has 0 structured bindings (count at
  ae457ee).

#### API-6 (MUST) One JSON writer

Every JSON document is written through the one JSON writer. No string-built JSON, even for one-line
replies. Code never reads its own JSON back as text.
- Scope: Pi. Enforce: script (grep `"{\"`); review.
- Bad: `pi/naviGATR/src/inspection/inspection_service.cpp:560`
  `std::string body = "{\"ok\":true,\"running\":";` then string concatenation (trimmed).

#### API-7 (SHOULD) Shared code in a neutral place

Code used by more than one subsystem lives in a neutral place (the component's framework or utility
folder named for its role), not inside one consumer's folder.
- Scope: All. Enforce: review.
- Bad: capture and diagnostics include `inspection/json_writer.h`.

#### API-8 (MUST) Performance needs a number

A performance pattern in C++ (cache, preallocation, lock-free structure, hand-unrolled loop, a copy
avoided at a readability cost) names its measured number in a one-line label; otherwise write the
plain version. RT-1, RT-2 and RT-10 are target correctness rules and exempt.
- Why: P-5: "if something is slightly slower but more readable, readable wins everyday" (the
  lead). JS-12 is the viewer twin.
- Scope: Pi, Brain. Enforce: review.

#### API-9 (MUST) Blocking is part of the contract

A public function that can block states its worst-case bound in a label (`// blocks up to 100 ms`);
a class that never blocks says so once in its header. No label means it returns without waiting.
- Why: RT-8 keeps long blocking off control tasks, but a caller cannot know which calls block
  unless it is stated.
- Scope: C++. Enforce: review.
- Good: `brain/communiGATR/include/communigatr/client.h:5-6` "No clock and no threads: poll(now)
  does all I/O and never blocks" (trimmed).

### 4.13 Configuration in code (CFG-)

#### CFG-1 (MUST) One XML reader and writer

All XML reading goes through `ConfigNode`; all XML writing goes through one shared writer.
`tinyxml2` appears only in `config/`.
- Scope: Pi. Enforce: script (grep `tinyxml2` outside `config/`). Legacy: deferred (phase 5);
  today also in `impl/publishing/field_documents.cpp`, `runtime/brain_profile_builder.{h,cpp}` and
  `runtime/system.cpp:152` `tinyxml2::XMLDocument doc;`.
- Bad: `pi/naviGATR/src/runtime/brain_profile_builder.cpp` has 73 lines calling `SetAttribute`
  in 616 lines.

#### CFG-2 (MUST) Parse, validate, build

A factory is three free functions: parse (XML to a data-only config struct), validate (config struct
to errors, pure), build (config struct to object). Each is testable without XML.
- Why: 16 of the 40 most complex functions parse, validate, bind and construct in one body
  (`TrackingWheelMotion::create` CCN 67, `make_synthetic_rig` CCN 79).
- Scope: Pi. Enforce: review; CPLX limits.

#### CFG-3 (MUST) Declarative config reader

New parsers use the shared declarative reader: a data-only config struct plus one field table
(required, optional with the struct's default, one-of a fixed set), read by one generic reader.
Errors keep the XML node path. The reader lands with two real parsers converted in the same series
(`<TrackingWheel>` plus one more, for example the `Freshness` read at
`pi/naviGATR/src/impl/sensors/attitude_channel.cpp:57`), per SYS-14. The same table can drive XML
writing; that writer half lands only with the conversion of `brain_profile_builder.cpp`.
- Why: the lead: "if you have to write customer drivers even for xml if thats more clear ...
  anything to avoid ... ten million lines for one command" (trimmed). This removes the most code
  of any single change.
- Scope: Pi. Enforce: review. Legacy: deferred; introduced right after the restructure (11.2
  phase 2).
- Bad: `pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:163-178` parses one
  `<TrackingWheel>` by hand:

```cpp
double      radius_m = 0.0, x = 0.0, y = 0.0, angle_deg = 0.0, scale = 1.0;
std::string direction;
if (!w.requireDouble("radius_m", radius_m, err) ||
    !w.requireDouble("position_x_m", x, err) ||
    !w.requireDouble("position_y_m", y, err) ||
    !w.requireDouble("measurement_angle_deg", angle_deg, err) ||
    !w.requireAttr("direction", direction, err) ||
    !w.getDouble("travel_scale", 1.0, scale, err)) {
    ok = false;
    return;
}
if (direction != "positive" && direction != "negative") {
    err = w.path() + ": direction must be positive or negative";
    ok  = false;
    return;
}
```
(trimmed: the surrounding `forEach` lambda and the `sensor_id` check are left out.)

- Good (proposed; names are placeholders until the reader exists): the field table sits next to
  the struct, the call site is one line of intent, and errors carry the node path:

```cpp
enum class WheelDirection : uint8_t { kPositive, kNegative };

struct TrackingWheelConfig {
    SensorId       sensor_id;
    std::string    label;
    double         radius_m              = 0.0;
    double         position_x_m          = 0.0;
    double         position_y_m          = 0.0;
    double         measurement_angle_deg = 0.0;
    WheelDirection direction             = WheelDirection::kPositive;
    double         travel_scale          = 1.0;
};
using TrackingWheelList = std::vector<TrackingWheelConfig>;

// Field table.
const config::Fields<TrackingWheelConfig>& trackingWheelFields() {
    static const config::Fields<TrackingWheelConfig> fields = {
        config::required("sensor_id", &TrackingWheelConfig::sensor_id),
        config::optional("label", &TrackingWheelConfig::label),
        config::required("radius_m", &TrackingWheelConfig::radius_m),
        config::required("position_x_m", &TrackingWheelConfig::position_x_m),
        config::required("position_y_m", &TrackingWheelConfig::position_y_m),
        config::required("measurement_angle_deg", &TrackingWheelConfig::measurement_angle_deg),
        config::oneOf("direction", &TrackingWheelConfig::direction,
                      {{"positive", WheelDirection::kPositive},
                       {"negative", WheelDirection::kNegative}}),
        config::optional("travel_scale", &TrackingWheelConfig::travel_scale),
    };
    return fields;
}

// Call site.
const std::optional<TrackingWheelList> wheels =
    config::readAll(node, "TrackingWheel", trackingWheelFields(), err);
if (!wheels) {
    return nullptr;
}
```

#### CFG-4 (MUST) Unknown config is an error

Every element a parser accepts is checked with `onlyChildren` and `onlyAttributes`; unknown children
and attributes are errors, never ignored. (The field table in CFG-3 does this by construction.)
- Scope: Pi. Enforce: review; test (a config with a misspelled attribute fails to load).

#### CFG-5 (MUST) Defaults written once

A default is written once, as the config struct's field initializer. Required means `require*` (or
`required` in the table); calibration-critical values are always required.
- Scope: Pi, Brain (robot config). Enforce: review.
- Bad: `pi/naviGATR/src/impl/sensors/attitude_channel.cpp:33` `long stale_after_ms = 250;` and
  `:57` `getInt("stale_after_ms", 250, state->stale_after_ms, err)` (spacing trimmed).

#### CFG-6 (MUST) Parsers live in .cpp files

A section's parser lives with the code that owns the section, in a `.cpp`. Parsers never live in
headers.
- Scope: Pi. Enforce: script (HDR-5 header function report); review.
- Bad: `pi/naviGATR/src/runtime/inspection_config.h:74` `parseInspectionConfig`, about 100 lines
  at CCN 51, in a header.

#### CFG-7 (MUST) Attribute names equal field names

Config attribute names are snake_case with a unit suffix and equal the C++ field name
(`stale_after_ms` in XML is `stale_after_ms` in the struct).
- Scope: Pi, config. Enforce: CFG-3 table makes it visible; review.

#### CFG-8 (SHOULD) One path to each setting

One path to each setting. A new path (another file, a CLI flag, a profile field) needs a consumer
that cannot use the existing one (SYS-9).
- Why: robot geometry lives in three places today (`brain/robot/gatr2_robot.h`, Pi robot XML
  files, each app's `robot_config.h`).
- Scope: All. Enforce: review.

### 4.14 Wire contract (WIRE-)

#### WIRE-1 (MUST) Only translaGATR touches wire bytes

Only translaGATR parses or builds wire bytes. A component that needs a partial decode asks
translaGATR for a function.
- Scope: All. Enforce: review; script (byte helpers outside `translaGATR/`).
- Bad: `pico/aggreGATR/src/pico_commands.cpp:12`
  `uint16_t rd16(const uint8_t* p) { ... }` and `readHeader` re-read a command header the codec
  refused (trimmed).

#### WIRE-2 (MUST) One definition per wire fact

Every wire constant, wire limit, wire enum name table and wire unit conversion has one definition,
in translaGATR. So does every wire-level predicate (equality, well-formedness, repeatability): the
Pi server and the Brain fake call it (SYS-25).
- Scope: All. Enforce: script (grep `const char\* .*Name(uint8_t` outside translaGATR).
- Bad: `constexpr uint8_t kMaxDiagHz = 5;` in `pico/aggreGATR/src/pico_commands.h:17` and again
  as `static constexpr uint8_t kMaxDiagHz = 5;` in
  `pi/naviGATR/src/impl/resources/pico_telemetry.h:187` (spacing trimmed).
- Bad: `brainResultName` at `pi/naviGATR/src/impl/commands/brain_link_commands.cpp:33` and again
  at `pi/naviGATR/src/capture/capture_bundle.cpp:51`, plus `resultName` at
  `pi/naviGATR/src/inspection/inspection_document.cpp:229` covering 6 of the 12 values.

#### WIRE-3 (MUST) Wire enums

Wire code enums are plain `enum Name : uint8_t` (the `Code` suffix only for a wire twin of a domain
enum, NAM-5) with every value written out, a group prefix on the enumerators, and retired values
named in a comment. A switch over a raw wire byte has a `default` that maps to "unknown" or rejects
the input, because newer peers send values this build does not know.
- Scope: codec. Enforce: review.
- Good: `translaGATR/frames.h:22` `enum FrameType : uint8_t {`.

#### WIRE-4 (SHOULD) One byte cursor

One little-endian reader and writer set (a byte cursor) in translaGATR, used by `frame_codec` and
`link_documents`, in place of hand-numbered offsets.
- Why: about 210 literal offsets in the codec, with `wr16/rd16` written twice in translaGATR and
  once in the Pico.
- Scope: codec. Enforce: review; the existing known-byte tests prove no byte moved.

#### WIRE-5 (MUST) Idempotent requests

Every state-changing request has an id, a documented dedupe rule (per session, serial-number
arithmetic for wrap) and a ticket the caller holds. Retries resend the same bytes. Readers flush
stale input before a request. Wrap is tested.
- Why: bug class B10 (a GET_STATE between SET_POSE retries made the retry "new").
- Scope: codec, Brain, Pi, Pico. Enforce: test patterns "retry after lost reply", "id wrap".

#### WIRE-6 (MUST) Named reserved values

A reserved wire value is a named constant with a helper (`kNoSession`, `kNoDocument`; proposed
names); code never compares against a bare `0` or `-1`.
- Scope: All. Enforce: review.
- Bad: `translaGATR/frames.h:316` `uint32_t session = 0; // 0 in HELLO` and `:365`
  `uint32_t profile_id = 0; // applied, applying or rejected profile, 0 = none` (spacing
  trimmed): the reserved 0 lives only in comments.

#### WIRE-7 (MUST) Versioned layout changes

A layout change bumps the version, updates `docs/interfaces.md` and its `Version history` in the
same commit, and is reviewed by an owner of each side (CODEOWNERS).
- Scope: codec. Enforce: CODEOWNERS; review.

#### WIRE-8 (MUST) Bounded decoders and encoders

Every decoder checks lengths against both the buffer and the declared total; every encoder takes the
destination size; static buffers are bounded by `kMaxFrameLen`.
- Why: bug class B16 (header encoders accepted a 24-byte buffer for a whole document).
- Scope: codec. Enforce: test (known-byte vectors, seeded random decode, ASan).

#### WIRE-9 (MUST) Version compatibility rules

Each versioned contract states its compatibility rule in its reference doc: what a reader does with
an older and with a newer version (refuse with a named status, or read the known part). Both
directions are tested.
- Scope: codec, Pi, Brain, Pico, viewer. Enforce: review; test.
- Good: `docs/interfaces.md:969` "RESTART_ACQUISITION on the Pico (v2 firmware, else Failed with
  PicoLink)" (trimmed); the test `perf_report_v1_refused`
  (`pi/naviGATR/tools/perf/CMakeLists.txt:66`).

### 4.15 Headers and includes (HDR-)

#### HDR-1 (MUST) Self-contained headers

Headers use `#pragma once`, are self-contained and include what they use.
- Scope: C++. Enforce: CI header-check target (each header compiled alone).

#### HDR-2 (MUST) Include order

Include order: the file's own header; a blank line; `<...>` includes (standard, platform, installed
libraries); a blank line; `"..."` project includes (vendored `third_party` included). Each block
sorted by clang-format. Platform-guarded includes go last, inside their `#if`.
- Scope: C++. Enforce: clang-format (`IncludeBlocks: Regroup`, 9.3; D-25). Legacy: deferred to the
  phase 3 format pass.

#### HDR-3 (MUST) Rooted include paths

Project includes are rooted, never relative to the including file: naviGATR from `src/` (`#include
"pipeline/localization/contract.h"`), libraries as `"<component>/<file>.h"`, and translaGATR as
`"translaGATR/<file>.h"` everywhere, the Pico included (D-9).
- Scope: C++. Enforce: script (no `../` includes; no bare `"frames.h"`).
- Bad: the Pico writes `#include "frames.h"` through `-I$PROJECT_DIR/../../translaGATR`
  (`pico/aggreGATR/platformio.ini:16`).

#### HDR-4 (MUST) .cpp named after its header

A `.cpp` is named after the header it implements.
- Scope: C++. Enforce: script.
- Bad: `pi/naviGATR/src/capture/capture_service.cpp` implements the class declared in
  `capture/capture_recorder.h`.

#### HDR-5 (MUST) Headers declare

Headers declare. A non-template function body over 10 lines goes in the `.cpp`.
- Scope: C++. Enforce: script (lizard over `*.h`: a non-template function over 10 NLOC is
  reported, baseline ratchet; 20 production header functions are over 15 NLOC today, led by
  `parseInspectionConfig` at 101); clang-tidy `misc-definitions-in-headers` for the ODR part only
  (it does not flag inline functions).

#### HDR-6 (SHOULD) Forward declarations

Forward-declare instead of including when only pointers or references are used.
- Scope: C++. Enforce: review.

#### HDR-7 (MUST) No platform headers in portable headers

A portable header never includes a platform header (PROS, Arduino, pico-sdk, libcamera, POSIX).
- Scope: C++. Enforce: script.

#### HDR-8 (MUST) Generated file first line

A committed generated file starts with a line naming its generator and input and "Do not edit", and
is excluded from formatting and naming checks by path.
- Scope: All. Enforce: script.
- Good: `brain/operaGATR/include/field_references.h:1` "Generated by navigatr_field_refs from
  pi/naviGATR/config/override/field.xml. Do not edit."

### 4.16 Constants, initialization, macros (CONST-)

#### CONST-1 (MUST) const by default

`const` by default: every local not reassigned, every reference parameter not written, every member
function that does not mutate.
- Scope: C++. Enforce: review; clang-tidy `misc-const-correctness` later (advisory).

#### CONST-2 (MUST) No magic numbers

Any number that changes behavior (threshold, gain, period, timeout, window, capacity, retry count,
screen coordinate) is a named `constexpr` with its unit in its name or type, or a configuration
attribute. Allowed bare literals: 0, 1, -1, 2 for halving, loop bounds over a named array, wire
field widths inside translaGATR encode/decode, print precision.
- Scope: All. Enforce: review (the clang-tidy magic-number checks are too noisy here, 9.2).
- Bad: `pi/naviGATR/src/runtime/handoff.h:159` `0.9 * snapshot_.mean_cycle_ms + 0.1 * work_ms`.
- Bad: `pros::screen::erase_rect(0, y, 479, y + 19);` (`brain/operaGATR/src/main.cpp:229`),
  duplicated in locaGATR.
- Good: `pico/aggreGATR/src/config.h:24` `constexpr uint32_t kSampleHz = 50;`.

#### CONST-3 (SHOULD) One constant per meaning

One named constant per meaning. Two values that happen to be equal are not a shared meaning; one
policy (sensor freshness) is one constant.
- Why: the literal 250 (ms) appears 36 times under at least eight names.
- Scope: All. Enforce: review.

#### CONST-4 (MUST) Default member initializers

Every scalar, pointer and enum member has a default member initializer, wire structs included,
unless a constructor always sets it. Enums default to their unset or none value.
- Scope: C++. Enforce: clang-tidy `cppcoreguidelines-pro-type-member-init`,
  `cppcoreguidelines-init-variables`.
- Bad: `SensorSample` in `translaGATR/frames.h:69-81` has fields without initializers,
  relying on `SensorSample s{};` at every use.

#### CONST-5 (MUST) Positional braces up to 3 fields

Positional brace initialization only for types of up to 3 fields whose order is obvious (`Point{x,
y}`, `Pose{x, y, heading}`). Anything longer is built by named field assignment or by a named
factory function (C++17 has no designated initializers).
- Scope: C++. Enforce: review.
- Bad: `brain/actuGATR/include/actugatr/follower.h:34`
  `PidGains along{3.0, 0.0, 0.0, 0.0, 100.0}; // m/s per m remaining` (spacing trimmed).
- Bad: `pi/naviGATR/src/impl/resources/pros_usb_link.cpp:79`
  `if (window.missed(nowUs())) return {false, true, false, false, "USB reply expired"};`
  (also unbraced).

#### CONST-6 (MUST) explicit constructors

Every constructor callable with one argument, and every conversion operator, is `explicit` unless
the implicit conversion is the point (and then a label says so).
- Scope: C++. Enforce: clang-tidy `google-explicit-constructor`; review.
- Bad: `pi/naviGATR/src/impl/resources/synthetic_rig.cpp:201`
  `SyntheticRigExecutable(std::shared_ptr<SyntheticRig> rig) : rig_(std::move(rig)) {}`

#### CONST-7 (MUST) Macros only for build variants and platforms

No macros for constants or functions; use `constexpr`. Two kinds of macro are allowed:
1. Build-variant macros, `GATR2_SCREAMING_CASE` or `<COMPONENT>_HAVE_<DEP>`, always defined by the
   build to 0 or 1 (`NAVIGATR_HAVE_LIBCAMERA=$<BOOL:${NAVIGATR_WITH_LIBCAMERA}>`,
   `-DGATR2_IMU_BNO08X=1 -DGATR2_IMU_ASM330=0`) and tested with `#if X`, never `#ifdef` or
   `defined(X)`, so `-Wundef` catches a typo or a missing definition.
2. Configuration macros a platform or vendored header reads, defined on the line before that
   include with a label (`#define NOMINMAX`, `#define STB_IMAGE_WRITE_IMPLEMENTATION`).
- Why: `-Wundef` warns only when an undefined name is evaluated; `defined(X)` never evaluates it,
  so a misspelled `#if defined(GATR2_IMU_BN08X)` compiles the file empty with no warning.
- Scope: C++. Enforce: compiler `-Wundef` (gate on every target); clang-tidy
  `cppcoreguidelines-macro-usage` (AllowedRegexp in 9.2; measured before it gates). The one
  stringify helper (`pi/naviGATR/src/impl/resources/libcamera_camera.cpp:38`
  `#define NAVIGATR_STRINGIFY_INNER(x) #x`) takes an EXCEPTION tag.
- Bad: `pi/naviGATR/src/impl/resources/cameras.cpp:12` `#if NAVIGATR_HAVE_LIBCAMERA` relies on an
  undefined macro reading as 0, because `pi/naviGATR/CMakeLists.txt:195`
  `target_compile_definitions(navigatr_core PUBLIC NAVIGATR_HAVE_LIBCAMERA=1)` defines it only
  when on.
- Bad: `pico/aggreGATR/src/config.h:13` `#if defined(GATR2_IMU_BNO08X) == defined(GATR2_IMU_ASM330)`
  guards the definition, not each later test of it.

### 4.17 Enums and switches (ENUM-)

#### ENUM-1 (MUST) enum class for domain enums

A domain enum is `enum class Name : <fixed type>` with `k`-prefixed enumerators. Plain enums are
only wire codes (WIRE-3).
- Scope: C++. Enforce: clang-tidy naming (EnumConstantPrefix), `cppcoreguidelines-use-enum-class`
  outside translaGATR (advisory).
- Bad: `pico/aggreGATR/src/asm330_supervisor.h:20-27`
  `enum class State : uint8_t { Off, Wait, Probe, Reset, Configure, Run, };` (trimmed).

#### ENUM-2 (MUST) Exhaustive switches, no default

A switch over an `enum class` lists every enumerator and has no `default`. Enumerators that share an
outcome are stacked as case labels. The fallback return goes after the switch, so `-Wswitch` flags a
new enumerator.
- Scope: C++. Enforce: compiler `-Wswitch` (in `-Wall`), `-Wswitch-enum` (advisory);
  review. Legacy: deferred for the 9 Brain switches (phase 5).
- Bad: `brain/actuGATR/src/motion.cpp:252-256` `switch (robot.status) {` ...
  `default: return MotionReason::kInputUnavailable;` (trimmed).
- Good: `pi/naviGATR/src/core/function_status.h:18-25` `functionStatusName` switches over every
  enumerator, then `return "unknown";` after the switch.

#### ENUM-3 (MUST) One name function per enum

One name function per enum, defined next to the enum. Wire enums get theirs in translaGATR, so the
Pi, Brain and Pico print the same text. A machine name (snake_case, for CSV and JSON) and display
text are two functions if both are needed.
- Scope: C++, viewer. Enforce: script (WIRE-2 grep); test (every value has a unique non-empty
  name).

#### ENUM-4 (MUST) Positive membership tests

A membership test is a positive list or a switch, never a chain of `!=`.
- Why: a new enumerator must not silently count as a member.
- Scope: All. Enforce: review.
- Bad: `brain/locaGATR/src/main.cpp:146-150` `sensorsReady` returns `r != Readiness::kConnecting
  && r != Readiness::kReconnecting && ...` (trimmed), so a new `Readiness` value reads as ready.

#### ENUM-5 (MUST) Enums for signs and modes

A named sign, direction or mode is an enum, not an `int` with a comment.
- Scope: C++. Enforce: review.
- Bad: `brain/investiGATR/include/investigatr/path.h:29`
  `int turn_direction = 0; // kTurn: +1 CCW, -1 CW` (spacing trimmed).

### 4.18 Complexity and size (CPLX-)

Thresholds come from a lizard 1.24 run over the tree at `ae457ee` (2586 production functions:
median CCN 2 and 8 NLOC; the problem is a tail concentrated in naviGATR). Warn means the reviewer looks and
either the function is split or a one-line reason is written. Fail means CI blocks new or changed
code; existing offenders sit in a baseline file that may only shrink (0.4).

| ID | Metric | Warn | Fail | Over warn today | Over fail today |
|---|---|---|---|---|---|
| CPLX-1 | cyclomatic complexity (CCN) per function | > 15 | > 25 | 147 | 60 |
| CPLX-2 | function length (NLOC) | > 60 | > 100 | 113 | 46 |
| CPLX-3 | nesting depth (control and lambda blocks) | > 3 | > 4 | 32 | 15 |
| CPLX-4 | parameters | > 5 | > 7 | 30 | 0 |
| CPLX-5 | file length, production (physical lines) | > 600 | > 1000 | 21 | 10 |
| CPLX-10 | file length, tests (physical lines) | > 1500 | > 2500 | 6 | 2 |

Nesting is measured by clang-tidy `readability-function-size`, which counts the function body as
level 1, so `NestingThreshold: N + 1` enforces nesting over N: the gate runs at 5, the warn report
at 4. The "today" nesting counts come from a heuristic, not the gate tool; phase 3 recomputes them
with the gate tools before the baseline is written.

#### CPLX-1 to CPLX-5, CPLX-10 (MUST) Size and complexity limits

The table above, for C++, JS and Python.
- Scope: C++, viewer, tools. Enforce: lizard owns CCN, NLOC and parameters, for C++, JS and
  Python (`-C 15 -Tnloc=60 -a 5` warn, `-C 25 -Tnloc=100 -a 7` fail, through
  `check_complexity.py`); clang-tidy `readability-function-size` owns nesting only (C++); ESLint
  `max-depth` or review for JS nesting; script for file length.
- Bad: `TrackingWheelMotion::run`
  (`pi/naviGATR/src/impl/localization/tracking_wheel_motion.cpp:510`): CCN 105, 310 NLOC,
  nesting 6.
- Readability decides, not the number: a flat 20-case name table at CCN 21 may carry a baseline
  waiver; a 60-line function nested 5 deep may not.

#### CPLX-6 (SHOULD) Class size

A class with more than 25 data members or 25 member functions is split along its reasons to change.
- Scope: C++. Enforce: review (report only).
- Bad: `navigatr::System` (70 data members, 75 methods) and `communigatr::Client` (73 private
  data members).

#### CPLX-7 (SHOULD) Phases as named functions

A multi-phase function reads as its list of phases: one named function per phase.
- Scope: All. Enforce: review.
- Bad: `TrackingWheelMotion::run` ingests wheels and gyro, applies the calibration and baseline
  gates, collects intervals, checks alignment, solves and commits in one body (paraphrased).

#### CPLX-8 (MUST) One writer per sub-object

A document writer is one writer per sub-object and a top writer that lists them.
- Scope: Pi, viewer. Enforce: review; CPLX limits.
- Bad: `writeSnapshotBody` (`pi/naviGATR/src/inspection/inspection_document.cpp:1140`): 367
  NLOC, 7 parameters.

#### CPLX-9 (SHOULD) Cognitive complexity

Cognitive complexity at most 25 per function (clang-tidy
`readability-function-cognitive-complexity`), reported as advisory with a ratchet.
- Scope: C++. Enforce: clang-tidy (report).

### 4.19 Numbers (NUM-)

#### NUM-1 (MUST) Finite at every seam

Every floating value that crosses a seam (stage output, wire encode, actuator command, config value)
is checked finite; a non-finite value is a fault status, never passed on.
- Why: a NaN pose reaches the Brain as a plausible number.
- Scope: C++. Enforce: review; test pattern "non-finite input" (TEST-10).
- Good: `brain/actuGATR/include/actugatr/drive.h:2-4` "the motors stop at once for anything Drive
  cannot execute: ... non-finite values" (trimmed);
  `brain/investiGATR/include/investigatr/geometry.h:45` `bool finite(const Pose& p);`.
- Gap: translaGATR has no finite check today.

#### NUM-2 (MUST) No loops on floats

A loop never iterates on a floating value; a numeric helper states and handles its domain for NaN
and +-infinity.
- Why: `wrapAngle(+inf)` never returns (probe against the real header, killed by a 5 s timeout);
  it runs on every localization step, so one infinite heading would stop the estimation worker
  (P-7). Reachability was not shown.
- Scope: C++. Enforce: review; test (edge values). Legacy: deferred (phase 5: UNIT-3's single
  wrap helper plus an edge test).
- Bad: `pi/naviGATR/src/math/angles.h:14-22` `inline double wrapAngle(double rad) {`
  `while (rad > kPi) {` `rad -= 2.0 * kPi;` ... (trimmed), called at
  `pi/naviGATR/src/impl/localization/weighted_planar_fusion.cpp:310`
  `r.odom_pose.heading_rad = wrapAngle(h + dtheta);`.

#### NUM-3 (MUST) Named float-to-integer helpers

Floating to integer goes through one named helper per policy that defines NaN, +-infinity and
out-of-range results (saturate or refuse, in its name). A raw `static_cast` from floating to integer
is not allowed: out of range it is undefined behavior (CERT FLP34-C).
- Scope: C++. Enforce: compiler `-Wfloat-conversion` (9.1); review.
- Good (refuse): `brain/communiGATR/src/link_driver.cpp:32-34` `bool toWire(double value,
  int32_t& out) {` ... `if (!std::isfinite(rounded) || rounded < INT32_MIN || rounded >
  INT32_MAX) {` (trimmed).
- Good (saturate): `brain/actuGATR/src/telemetry.cpp:17-18`
  `long saturate(double value, long lo, long hi) {` `if (!std::isfinite(value)) {`.

#### NUM-4 (SHOULD) Floating equality

Floating equality only against an exact sentinel or an exact assigned value (0 meaning off), through
a named helper where it repeats; otherwise a named tolerance.
- Why: MISRA C++:2023 Dir 0.3.1; `-Wfloat-equal` is rejected (9.1) because all 29 hits are such
  values, so the rule says what is allowed instead.
- Scope: C++. Enforce: review.
- Good: `brain/actuGATR/src/pid.cpp:22` `if (gains_.kI != 0) {`.

#### NUM-5 (MUST) No integer-width overload sets

No overload set that distinguishes `int`, `long`, `long long` or fixed-width aliases; use one
constrained template or distinct names.
- Why: bug class B15 (JsonWriter overloads collided across ABIs, commit 3af0e98).
- Scope: C++. Enforce: review.
- Good: `pi/naviGATR/src/inspection/json_writer.h:85-90` (TYPE-15).

#### NUM-6 (SHOULD) Signed arithmetic

Signed types for arithmetic and differences; unsigned only for bits, wire fields, sizes from the
standard library and wrapping clocks (TIME-6).
- Why: C++ Core Guidelines ES.100 to ES.102.
- Scope: C++. Enforce: compiler `-Wsign-conversion` (report, 9.1).

### 4.20 Formatting (FMT-)

#### FMT-1 (MUST) Pinned clang-format

C++ is formatted by clang-format 22.1.8, the pinned version, with the repo root `.clang-format`,
including the knobs D-25 adopted (9.3). Output differs between versions, so local and CI use exactly
that version.
- Why: the lead's rule; formatting is never a review topic.
- Scope: C++. Enforce: clang-format (`--dry-run --Werror`, the 9.9 format job). Legacy:
  deferred (369 of 437 C++ files differ today; the whole-repo format-only pass in phase 3, after
  the restructure and spectuGATR Phase A patches are applied).

#### FMT-2 (MUST) Braces on every control statement

Every `if`, `else`, `for`, `while` and `do` body is a braced block, one-line bodies included.
- Why: the lead's rule: braces on every if, no single-line unbraced conditionals.
- Scope: C++ (JS-13 for the viewer). Enforce: clang-tidy `readability-braces-around-statements`
  (`ShortStatementLines: 0`, 9.2); clang-format `InsertBraces` (D-25). Legacy: deferred (124
  missing today; the `InsertBraces` commits right after the phase 3 format pass).
- Bad: `pi/naviGATR/src/impl/resources/pros_usb_link.cpp:144` `if (fd_ >= 0) return true;`

---

## 5. Comments and documentation

The lead's comment rules are the base, and they are stricter than any industry guide: plain
ASCII, terse self-describing labels, not AI-ish, no over-explaining, casual grammar fine.
Industry structure (what an API comment covers, doc types, templates) lives in docs and in short
API labels, never in narrated comments.

### 5.1 Comments (CMT-)

#### CMT-1 (MUST) Plain ASCII

Plain ASCII in comments, docs, commit messages, UI text and config comments. No emojis, no em or en
dashes, no smart quotes, no Unicode arrows, no symbol that is not on a standard keyboard. `->` is
allowed where it carries meaning (a flow, pseudo code, a diagram). Punctuation is used only when it
helps.
- Scope: All. Enforce: script (`grep -P '[^\x00-\x7F]'` over tracked first-party text files).
- Bad: `pi/naviGATR/spectaGATR/app.js:850`
  `const DOT = ' ' + String.fromCharCode(0xb7) + ' ';` builds a middle dot for the UI.

#### CMT-2 (MUST) Comments are labels

A comment is a terse, self-describing label saying what a thing is or what is needed: a bare noun
phrase by default (`# Test files.`), one short sentence only when a phrase cannot say it. It is
not an explanation delivered to the reader. No "This function ...", no narrated rationale, no
formal prose.
- Scope: All. Enforce: review.
- Good: `pi/naviGATR/tests/CMakeLists.txt:18` `# Test files.`
- Bad: `pi/naviGATR/src/runtime/system.cpp:1027-1028` `// the inline cycle lets the same cycle's
  evidence reach target` / `// resolution and publishing, as the synchronous tests expect`:
  rationale that justifies code by pointing at tests.

#### CMT-3 (MUST) No restating

Do not restate what the code, the name, the type or a self-evident pattern already says. A format is
stated only where it is not obvious (a comment saying "every file here is named `*_gtest.cpp`" is
noise once the files show it).
- Scope: All. Enforce: review.

#### CMT-4 (MUST) What a comment may carry

What a comment may carry, in order of preference:
1. a unit, range, wire sentinel (WIRE-6) or frame the type cannot carry: `// 0 = none`,
   `// (-pi, pi]`;
2. a caller-visible contract the signature leaves out: blocking and its bound, which thread,
   ownership and lifetime, what happens on bad input or dead upstream;
3. a wire layout.

Rationale, method, procedure, history and schemas go to the PIPELINE.md or a doc, with at most a
one-line pointer.
- Scope: All. Enforce: review.
- Good: `brain/investiGATR/include/investigatr/state_source.h:42`
  `// Newest robot state, ages relative to now.`
- Good: `brain/actuGATR/include/actugatr/motion.h:150`
  `// Active command -> kCancelled. No effect once terminal.`
- Good (wire layout; long comments allowed here): `translaGATR/frames.h:32-36`
  `// Sensor frame (Pico -> Pi)` then `//   v2 (0x04)  sync0 sync1 | type | seq u8 | stamp_ms u32
  | boot_id u16` (trimmed).

#### CMT-5 (MUST) Short file headers

A file header is at most 5 label lines after line 1: what the file holds, and the one rule a reader
must not break. No pipeline narrative, threading essay, lifecycle story or XML schema. A pipeline
entry file spends one of the 5 lines pointing at its PIPELINE.md (PIPE-9).
- Scope: All. Enforce: script (warn when the leading comment block is over 6 lines, line 1 plus
  5; blocking after the comment pass in phase 6). Legacy: deferred (36 naviGATR headers exceed
  it).
- Bad: `pi/naviGATR/src/runtime/system.h:1-83`, an 83-line header covering construction,
  workers, threading, Brain profiles, recovery and Pico CONTROL.
- Good: `translaGATR/frames.h:1-3` `// frames.h` / `// Wire format shared by the Pico, Pi, and
  brain.` / `// No hardware dependencies. Must compile on all three targets.`

#### CMT-6 (MUST) Types over comments

When a type can carry the information (an alias, an enum, a unit type), the type carries it and the
comment goes (ALIAS-2, TYPE-13).
- Scope: C++. Enforce: review.

#### CMT-7 (MUST) Fixed label forms

Fixed forms for recurring labels: thread tags per CON-2; wire sentinels as `// 0 = none` (WIRE-6;
elsewhere absence is typed, TYPE-16); generated files per HDR-8; exceptions per 0.3; `TODO` only as
`// TODO(<name>): <label>`.
- Scope: All. Enforce: script (tag grammar).

#### CMT-8 (MUST) No commented-out code

No commented-out code.
- Scope: All. Enforce: review.

#### CMT-9 (SHOULD) No section banners

New code has no section banners (`// ---- cycles ----`) in a file of functions; a banner means the
file wants splitting. A wire-layout header may separate message layouts with one rule line each
(`translaGATR/frames.h`, the lead's reference file). Existing banners go only when their file is
split anyway. Bare noun labels over groups of members (`// Session.`) are fine.
- Scope: All. Enforce: review.
- Bad: `pi/naviGATR/src/runtime/system.cpp:1020` `// ---- cycles ----...` (trimmed), one of about
  110 banners in naviGATR.

#### CMT-10 (MUST) Industry prose rules do not apply

Industry comment rules that ask for prose or for a comment on every declaration (LLVM's "English
prose", Google's "almost every function declaration should have comments", AUTOSAR A2-7-3) do not
apply. Google's own "do not state the completely obvious" does.
- Scope: All. Enforce: review.

#### CMT-11 (MUST) Line 1 names the file

Line 1 of a source file is `// <file name>`, nothing else on that line. A file whose basename
repeats in its component (`contract.h`, `stage.h`, `stage.cpp`, `noop.h`, `noop.cpp`) uses its
component-relative path instead (`// pipeline/localization/contract.h`, STRUCT-7). A generated
file's line 1 is its HDR-8 line (D-10).
- Why: 426 of 454 first-party `.h/.cpp/.c/.js` files already start with `// <file name>` (the
  `frames.h` convention); only restructured files change.
- Scope: C++, viewer. Enforce: script.

### 5.2 Documentation (DOC-)

#### DOC-1 (MUST) Doc types and places

Every doc is one of these types and lives in the stated place:

| Type | File | Where | Size |
|---|---|---|---|
| Signpost | `README.md` | repo root, each platform folder, each component root, each folder a newcomer lands in | target 10, at most 20 lines |
| Pipeline | `PIPELINE.md` | the pipelines listed in PIPE-1 | PIPE-2 sizes |
| Reference | `docs/<topic>.md` | cross-component in root `docs/`; component-only in `<component>/docs/` | one topic |
| How-to | `docs/<task>.md` | same rule as reference | one task |
| Explanation | `docs/<topic>.md` | same rule as reference | one topic |
| Decision record | `docs/decisions/NNNN-title.md` | root | one page |
| Glossary | `docs/glossary.md` | root | one line per term |
| Validation record | `docs/validation.md` | root | one table |
| Vendored dependency | `<component>/third_party/README.md` | per component | one entry per library |

A folder with a PIPELINE.md and no build commands has no README; its PIPELINE.md states its intent.
- Why: the lead: "READMEs are short signposts (a few lines). Deep documentation goes in docs/."
- Scope: docs. Enforce: script (README length, PIPELINE.md length and headings); review.

#### DOC-2 (SHOULD) One file, one type

One file, one type. A how-to links to the reference instead of restating it.
- Scope: docs. Enforce: review.
- Bad: `pi/naviGATR/README.md` (278 lines) mixes a signpost, the run diagram, an implementation
  coverage list, the config layout, build commands and a run reference.

#### DOC-3 (MUST) Signpost README shape

A signpost README has, in order: `# <folder name>` spelled as the folder; one purpose line under 120
characters; `Status:`; `Pipeline: [PIPELINE.md](PIPELINE.md)` when there is one; one line per
subfolder or key file; the build and test commands as preset names (CMAKE-10), one line each (only
commands CI runs literally, or lines marked `manual`); links to deep docs. Anything longer goes to
`docs/`. No empty placeholder files: a folder carries a README stating intent, and code files appear
when there is code.
- Scope: All. Enforce: script (title, length, `Status:` value); review.
- Closest today: `pico/README.md` (7 lines) and `brain/actuGATR/README.md` (15 lines): title
  spelled as the folder, a purpose line, one line per subfolder. Neither has `Status:` yet.
  `brain/README.md` is 38 lines, over the DOC-1 limit.

#### DOC-4 (MUST) Reference docs name their source of truth

A reference doc starts with its scope and a `Source of truth:` line naming the header or file that
wins when the doc disagrees.
- Scope: docs. Enforce: review.
- Good: `docs/interfaces.md:7-8` "When this document and the headers disagree, the headers win
  and this document is the bug." (trimmed).

#### DOC-5 (MUST) How-to shape

A how-to or bring-up doc has: `Goal`, `Needs`, numbered `Steps` each with its expected result,
`Check`, `Troubleshooting` (a symptom-to-check table), and `Last run on hardware:` with a date and
setup, or `never`.
- Scope: docs. Enforce: review.
- Closest today: `bench/rs485_link/README.md` (Wiring, Pi side, Brain side, Success); it has no
  Troubleshooting table and no `Last run on hardware:` line.

#### DOC-6 (SHOULD) Explanation shape

An explanation doc has `Problem`, `Approach`, `Limits` (what it does not model) and optionally
`Rejected`.
- Scope: docs. Enforce: review.

#### DOC-7 (MUST) Decision records

A decision that changes a boundary, a contract or a repo-wide convention, or that the lead rules on,
gets a decision record: `# NNNN. Title`, `Status:` (proposed, accepted, superseded by NNNN),
`Context` (what is this truly for), `Decision`, `Consequences`. One page. Numbers are never reused;
a superseded record is kept.
- Scope: All. Enforce: review. The decisions in section 12 are the first records.

#### DOC-8 (SHOULD) One fact, one place

One fact, one place. Order of authority: code and headers, then reference docs, then PIPELINE.md and
explanation, then READMEs; when a lower one disagrees, the lower one is the bug. Link, do not copy.
- Scope: docs. Enforce: review.
- Bad: the Pico command rules are written three times (`docs/interfaces.md`,
  `pi/naviGATR/docs/pico_link.md`, `pico/aggreGATR/README.md`).

#### DOC-9 (MUST) Status vocabulary

Status and value vocabulary, in docs, READMEs, PIPELINE.md files and commit messages:
- validation claims use `designed`, `host-tested`, `bench-tested`, `robot-validated`; a level is
  never raised on host or synthetic evidence;
- `Status:` is one of those four and links to `docs/validation.md`;
- measured values are marked `PLACEHOLDER` (not measured), `PROVISIONAL` (measured once) or
  `MEASURED`;
- `docs/validation.md` is the single record: item, level, evidence (date, commit, setup, log or
  capture location), open checks;
- docs never hard-code test counts.
- Why: P-12; 17 docs word "not run on hardware" 17 different ways today (SYS-22 merged here).
- Scope: docs. Enforce: script (Status values); review.

#### DOC-10 (MUST) Docs change with behavior

Docs change in the same PR as the behavior they describe. A rename sweeps prose and anchors too:
grep for the old name, not only the old path. Dead docs are deleted.
- Scope: All. Enforce: review; script (old-name list).
- Bad: `brain/locaGATR/README.md:1` `# Localization test` after the rename to locaGATR.

#### DOC-11 (MUST) Checked doc snippets

A code snippet in a doc is either copied from a compiled file (the source path named under the
block) or labeled `sketch`. A `sh` block in a README equals a CI step or is marked `manual`.
- Scope: docs. Enforce: script.

#### DOC-12 (MUST) -GATR spelling in prose

Names in prose use the -GATR spelling (naviGATR, aggreGATR, spectaGATR, ...). `navigatr` appears
only in code font for the namespace, executable or target. A term that means different things in two
components is qualified (`investigatr::RobotState` vs `navigatr::RobotState`).
- Scope: docs. Enforce: script (spelling list); review.

#### DOC-13 (MUST) One glossary

`docs/glossary.md` defines every cross-component term once, with its defining symbol and file.
Section 10 is its seed.
- Scope: All. Enforce: review.

#### DOC-14 (MUST) Component docs with the component

Component-only deep docs live in `<component>/docs/`; root `docs/` keeps only cross-component
material (`interfaces.md`, `hardware.md`, `naming.md`, `glossary.md`, `validation.md`, `decisions/`,
`contributing/`) (D-11).
- Scope: docs. Enforce: review.
- Bad: `docs/navigatr_resources.md` and `docs/navigatr_sensors.md` sit outside naviGATR.

#### DOC-15 (MUST) Version history per contract

Every versioned contract (Brain link v4, sensor frame v2, `navigatr.inspect/2`, `gatr2.capture/1`)
keeps a `Version history` section in its reference doc. There is no repo CHANGELOG; git log covers
the rest.
- Scope: codec, Pi, viewer. Enforce: review.

#### DOC-16 (MUST) Vendored library entries

Every vendored library has an entry in its `third_party/README.md`: upstream, exact version or
commit, license, files kept, who uses it.
- Scope: All. Enforce: review.
- Bad: `pi/naviGATR/third_party/README.md:15-16` "Version: the vendored copy records no version
  or commit; nothing in the files names one."

#### DOC-17 (SHOULD) Doc prose style

Doc prose: sentence-case headings, numbered lists for sequences, code font for code, descriptive
link text, `you` and imperatives in how-tos. Same ASCII and tone rules as comments.
- Scope: docs. Enforce: review.

---

## 6. Testing (TEST-)

### TEST-1 (MUST) GoogleTest, explicit lists

Tests use GoogleTest. Files are `<unit>_gtest.cpp` with no `main()` (link `GTest::gtest_main`). One
flat `tests/` folder per component. Test files are listed explicitly in `TEST_SOURCES`; sources
compiled only for tests are listed in `CODE_UNDER_TEST` or come from linking the library. No
globbing; a new test is a deliberate line in the diff.
- Scope: C++. Enforce: review; script (no `file(GLOB` in test CMake).
- Bad: `pi/naviGATR/tests/CMakeLists.txt:21` `../src/app/command_line.cpp` inside
  `set(TEST_SOURCES`.

### TEST-2 (MUST) Test names

Names are `TEST(Unit, BehaviorSentenceInPascalCase)` with no underscores. `TEST_F` only when three
or more tests share non-trivial setup; `TEST_P` for a matrix (transports, variants).
- Scope: C++. Enforce: script (no `_` in test names).
- Good: `TEST(Workers, StopAndResetWakeAWorkerWaitingForALongPeriod)`.

### TEST-3 (SHOULD) One test file per unit

A test file maps to the unit it tests (tests for `sensor_loss.cpp` are in `sensor_loss_gtest.cpp`).
A test file past the CPLX-10 warn size (1500 lines) is split by behavior group.
- Scope: C++ (tests). Enforce: CPLX-10.
- Bad: `translaGATR/tests/frame_codec_gtest.cpp` is 3554 lines;
  `pi/naviGATR/tests/brain_profile_gtest.cpp` is 2926 lines and also holds the `SensorLoss` and
  `StationaryPrecheck` suites.

### TEST-4 (MUST) Shared test support in sim

Shared test support (fakes, XML fixture builders, polling helpers) lives in `<component>/sim/`, a
sibling of `tests/`, listed explicitly; `tests/` keeps only `*_gtest.cpp` files (the lead's rule).
Fakes are never copy-pasted between test files, and a helper is never re-declared by hand in another
test file (D-19).
- Why: with nowhere to put shared helpers, the naviGATR tests copied them: `takeReplies` in 4
  files, `ScriptedDetector`, `ScriptedCamera` and `FakePico` twice each.
- Scope: C++. Enforce: review; clang-tidy `misc-use-internal-linkage` (TEST-15) catches a
  helper shared by re-declaration.
- Good: the Brain already works this way: `brain/communiGATR/sim/fake_pi.cpp`, tested by
  `brain/communiGATR/tests/fake_pi_gtest.cpp`.
- Bad: `pi/naviGATR/tests/brain_profile_gtest.cpp:1501`
  `bool validJson(const std::string& s);   // inspection_document_gtest.cpp` re-declares a helper
  defined in another test file.

### TEST-5 (MUST) Real code first, then fakes

Use real implementations first, then fakes at seams (a hardware or process boundary). No mocking
framework. A fake is named `Fake<Contract>` (canned behavior) or `Scripted<Contract>` (replays a
list). The owner of the real code maintains the fake, and the fake has its own tests.
- Scope: C++. Enforce: review.
- Good: `brain/communiGATR/sim/fake_pi.cpp` is tested by
  `brain/communiGATR/tests/fake_pi_gtest.cpp`.

### TEST-6 (MUST) Deterministic tests

Tests are deterministic. Time is injected and stepped; no `sleep_for` waits for a result. A test
that needs real threads waits on a condition with a generous deadline, asserts the same observable
it waited for, and ends its name in `Threaded`. Every discovered test has a timeout:
`gtest_discover_tests(<target> PROPERTIES TIMEOUT 60)` (threaded suites may set more, with a label).
Order never matters: CI runs each suite once with `--gtest_shuffle`.
- Why: bug classes B11 and B14. No `TIMEOUT` is set today (e.g.
  `pi/naviGATR/tests/CMakeLists.txt:70` `gtest_discover_tests(navigatr_tests)`), so a hang holds
  CI for ctest's default 1500 s.
- Scope: C++. Enforce: script (grep `sleep_for` in tests, with an allowlist that shrinks; every
  `gtest_discover_tests` has `TIMEOUT`); CI repeats threaded suites (`ctest --repeat
  until-fail:3`) and shuffles.
- Bad: `pi/naviGATR/tests/capture_gtest.cpp:2025`
  `std::this_thread::sleep_for(std::chrono::milliseconds(2500));` because "the recorder reads the
  host once a second" (:2023).

### TEST-7 (MUST) Fixed seeds

A randomized test uses fixed seeds and reports the seed on failure.
- Scope: All. Enforce: review.
- Good: `brain/investiGATR/tests/planner_random_gtest.cpp:2`
  `// Randomized fields with fixed seeds: every plan is checked against exact` (trimmed), with
  `SCOPED_TRACE` of the seed.

### TEST-8 (MUST) Tests do not print

Tests do not print. Timing checks are asserts with a bound, or a separate benchmark target.
- Scope: All. Enforce: script (grep `printf`, `std::cout` in tests).
- Bad: `brain/investiGATR/tests/planner_edge_gtest.cpp:500`
  `std::printf("128 boxes over the 512 vertex bound: %.0f us\n",` (trimmed).

### TEST-9 (retired) Host build of non-adapter code

See SYS-23.

### TEST-10 (MUST) Standard fault patterns

Every new seam, stateful component or protocol gets the standard fault patterns that apply:

| Pattern | Asserts | Bug class it prevents |
|---|---|---|
| source stops | freeze the producer, pass the freshness limit: output is stale or absent, never old-as-new | B3 stale state (SYS-17, RT-14) |
| identity change mid-flow | change session, boot, epoch or connection mid-operation: nothing from the old identity leaks | B4 missing reset (SYS-16) |
| no input at all | every deadline still fires; no state lacks a timeout exit | B11 liveness (P-7, CON-10) |
| slow link | worst-case timing: scheduler fairness bound holds | B11 starvation (CON-10) |
| stalled consumer | stop the reader, push many times the capacity: memory bound and drop counter hold | B7 unbounded queue (CON-9) |
| retry after lost reply | the retried request applies once | B10 idempotency (WIRE-5) |
| interleaved requests | another request between a request and its retry: the retry still applies once | B10 idempotency (WIRE-5) |
| id wrap | ids and sequences wrap at their maximum | B10 wrap (WIRE-5) |
| enum names complete | every enumerator has a unique, non-empty name | B9 name drift (ENUM-3) |
| overflow reports itself | a dropped or truncated output says so | B12 silent loss (ERR-9) |
| upstream reboots or sends garbage | resync on sync bytes; no crash, no hang; oversize input is refused | B16 bounds, degrade (P-7, RT-15) |
| field absent | each optional input field missing: the consumer shows or sends absent, never 0 or old | B2 in-band sentinel (TYPE-16) |
| busy caller | hold the owner's lock: every public call returns busy within its bound | B6 blocked caller (CON-3, RT-6) |
| reset under load | resets race the reader: no document carries an old value with a new label | B5 snapshot labels (SYS-24) |
| non-finite input | NaN and +-inf at every floating input: a fault status, no hang, nothing sent | B8 numbers (NUM-1, NUM-2) |

Bug class IDs are defined in 10.4.
- Scope: All. Enforce: review (REV-10 asks which patterns apply).

### TEST-11 (MUST) Codec property tests

Codec property tests: a seeded round trip `decode(encode(x)) == x` over generated valid frames and
documents; the decoder never crashes or over-reads on random or truncated input; both run under ASan
and UBSan in Linux CI.
- Why: translaGATR is the narrow seam compiled into three targets; its tests are example-based
  today, with no encode/decode round trip.
- Scope: codec. Enforce: test; sanitizer job.

### TEST-12 (MUST) See a new test fail

A new test is seen failing once (a mutation or a negative check) before it is trusted. A fix carries
its regression test in the same commit.
- Scope: All. Enforce: review.

### TEST-13 (MUST) Behavior-preserving changes keep tests identical

A behavior-preserving change (move, split, rename, format) keeps the test name list and the results
identical: diff `ctest -N` (or `--gtest_list_tests`) and the results before and after. A pure move
also keeps defined-symbol parity (`nm -C --defined-only`).
- Scope: All. Enforce: script in the restructure gates (11.4).

### TEST-14 (retired) Every configuration loads

See XML-8.

### TEST-15 (MUST) Test helpers are internal

Test helpers (functions, variables and types) sit in an anonymous namespace and never have external
linkage (all test files of a component link into one binary). Imports follow NS-4.
- Scope: C++ (tests). Enforce: clang-tidy `misc-use-internal-linkage` (it also sees types, which
  the compiler flag does not); compiler `-Wmissing-declarations`.
- Bad: `brain/communiGATR/tests/client_telemetry_gtest.cpp:524` `struct BusRun {` at global scope
  after the anonymous namespace closed; `pi/naviGATR/tests/inspection_document_gtest.cpp:31`
  `struct JsonCheck {` and `:154` `bool validJson(const std::string& s) {`.

### TEST-16 (MUST) Same warnings in tests

Test code gets the same warnings and the same always-a-bug errors as production code (CMAKE-13),
and a lighter clang-tidy overlay (long bodies, literals and optional access after an `ASSERT` are
fine).
- Scope: C++. Enforce: compiler; `tests/.clang-tidy` overlay (9.2).

### TEST-17 (SHOULD) Second implementations checked against the real one

A second implementation of a versioned contract in any language (a stub server, a fixture writer, a
reader in the viewer) is checked in CI against output of the real producer: the C++ tests write a
bundle or a document that the other side's tests load, and column and field lists are compared, not
copied.
- Why: the lead's CI list includes a cross-language agreement test.
- Scope: tools, viewer. Enforce: review; test.
- Bad: `pi/naviGATR/tools/viewer/capture_fixture.py:12-13` "proves nothing about the Pi's capture
  code" (trimmed): a second writer of `gatr2.capture/1` with hand-copied column lists.

### TEST-18 (MUST) Sanitizers

Host tests run under ASan and UBSan, and naviGATR's threaded tests under TSan, in Linux CI. (They do
not link on the MinGW host; only UBSan trap mode does.)
- Scope: C++. Enforce: CI sanitizer jobs (9.7).

### TEST-19 (MUST) Harnesses check their peer

A test or tool that starts a server binds port 0 or a checked free port and confirms the peer's
identity (instance id or pid) before it measures or asserts.
- Why: bug class B14 (the perf harness measured whatever process already held the port).
- Scope: tools, C++ (tests). Enforce: review.

### TEST-20 (SHOULD) Coverage report

The Linux host test job reports line and branch coverage per component (gcov/gcovr). Coverage is
informational and never blocks.
- Why: the lead's CI rule: "coverage is informational, never blocking."
- Scope: build. Enforce: CI (9.9, non-blocking).

### TEST-21 (SHOULD) Fuzz external decoders

A libFuzzer target per external decoder (frame reader, link documents, HTTP request head, config
composition) runs for a fixed time in the Linux clang job; each crash becomes a known-byte test.
- Why: TEST-11's seeded inputs find less than coverage-guided fuzzing on untrusted bytes.
- Scope: codec, Pi. Enforce: CI (9.9).

---

## 7. Other languages and files

### 7.1 JavaScript, spectaGATR (JS-)

#### JS-1 (MUST) Plain ES modules

ES modules loaded directly by the browser. No framework, bundler, transpiler or npm is needed to
build, run or test the viewer. Imports are relative with the `.js` extension, or the one bare name
`three` resolved by the import map.
- Scope: viewer. Enforce: script (import specifiers).

#### JS-2 (MUST) Vendored browser code

Third-party browser code lives in `pi/naviGATR/third_party/<lib>/` with a README entry (DOC-16),
served under `vendor/`. A version bump is its own commit.
- Scope: viewer. Enforce: review.
- Good: three.js r170, pinned (`pi/naviGATR/third_party/README.md:41`).

#### JS-3 (MUST) One concern per module

One concern per module, named for what it exports. `app.js` holds only wiring: create the parts, the
frame loop, the order things happen in. A module over the CPLX-5 fail size, or holding two panels,
is split.
- Scope: viewer. Enforce: lizard/script; review.
- Bad: `pi/naviGATR/spectaGATR/field_scene.js` is 1582 lines, one class with 44 methods;
  `bundle.js` is the capture-bundle reader, not a build bundle.

#### JS-4 (MUST) Functions first

Functions first. A plain object literal is the JS struct; an exported function is the default; a
`class` only when an instance owns state or DOM nodes between calls. No class that is only a
namespace for functions.
- Scope: viewer, tools (JS). Enforce: review.
- Good: `pi/naviGATR/spectaGATR/transforms.js` is all exported functions.

#### JS-5 (MUST) JS names

Names: files `snake_case.js`; classes PascalCase; functions and variables camelCase; module
constants UPPER_SNAKE; every numeric name with a unit carries it (`periodMs`, `STALE_MS`). Fields
that mirror a wire or config document keep the document's exact snake_case (`age_ms`). The
page-local test hook keeps the runtime's name, `window.__navigatr`, like the contract name
`navigatr.inspect/2` (D-20, retired).
- Scope: viewer. Enforce: review.
- Good: `pi/naviGATR/spectaGATR/feed.js:18` `const RECONNECT_MIN_MS = 500;`.

#### JS-6 (SHOULD) Module state at the top

Mutable module state is declared at the top of the module, grouped in named objects by concern,
never below the functions that read it.
- Scope: viewer. Enforce: review.

#### JS-7 (MUST) Store, then apply

Store, then apply. Network handlers parse, store the newest value and bump a version; they never
touch the DOM or the scene. The frame loop applies changes by comparing versions.
- Scope: viewer. Enforce: review.
- Good: `pi/naviGATR/spectaGATR/feed.js:2` "Handlers only parse and store" (trimmed).

#### JS-8 (MUST) DOM writes through dom.js

DOM writes go through `dom.js` helpers; text through `textContent`; `innerHTML` only with a literal.
- Scope: viewer. Enforce: script.
- Good: `pi/naviGATR/spectaGATR/dom.js:19` `export function setText(node, text) {`

#### JS-9 (MUST) One home per helper

One home per helper: DOM helpers in `dom.js`, math and angles in `transforms.js`, formatting in one
`format.js`. No local copies, no re-exports.
- Scope: viewer. Enforce: script (duplicate export names).
- Bad: angle wrap three times: `transforms.js:11` `export function wrapDeg(d) {`,
  `series.js:243` `function wrapDeg(d) {`, `smoothing.js:45` `function wrap180(d) {` (all in
  `pi/naviGATR/spectaGATR/`).

#### JS-10 (MUST) JS errors

Errors: parsers throw an `Error` that says what and where; the UI edge catches and shows it; network
handlers never throw; no empty `catch` except for per-viewer conveniences (storage), with a label.
- Scope: viewer. Enforce: review.

#### JS-11 (MUST) ?? and ?. for optional data

Optional data uses `??` and `?.`. `||` only combines booleans; `x || 0` turns a real 0 into a
default and a missing value into a number.
- Scope: viewer. Enforce: script (grep `|| <number>`).
- Bad: `pi/naviGATR/spectaGATR/transforms.js:57`
  `R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + (T.x_m || 0),`

#### JS-12 (SHOULD) Performance needs a number

Performance patterns (preallocation, caches, budgets) need a number from the perf harness, named in
a one-line label. Otherwise write the plain version (P-5, API-8).
- Scope: viewer. Enforce: review.

#### JS-13 (MUST) Strict equality, const, braces

`===` only, `const`/`let` only, braces on every block, no shadowing.
- Scope: viewer, tools (JS). Enforce: ESLint or the Python checker (D-21).

#### JS-14 (MUST) Short module headers

Each module starts with its line-1 file name (CMT-11) and a header within CMT-5. The design
narrative (data flow, the test contract of `#status` `data-*` attributes and the test hook, replay
isolation) moves to `spectaGATR/PIPELINE.md`.
- Scope: viewer. Enforce: CMT-5 script.

#### JS-15 (MUST) JS formatting

JS is formatted by the pinned clang-format 22.1.8 through a `Language: JavaScript` section in the
root `.clang-format` (9.3, adopted by D-25). Braces are not inserted by clang-format for JS, so
JS-13 covers them.
- Scope: viewer, tools (JS). Enforce: clang-format.

### 7.2 Python (PY-)

#### PY-1 (MUST) Non-C++ code is tooling

naviGATR is C++ only, CMake and XML accepted (the lead's rule). PIPELINE.md and README.md are
allowed inside `pi/naviGATR/` (D-4, decided 2026-09-28). The rest of D-4 is open: until the lead
rules, no other new file in a language other than C++, CMake or XML enters `pi/naviGATR/`, and
existing ones stay. Everywhere, a component's runtime and
library code is written in the component's language. Host-side tools that are never built into or
loaded by the runtime live in `tools/`: a tool with more than one file (its test counts) gets
`tools/<tool>/`; a single-file script may sit in `tools/`. A tool says what it is and how to run it
in its `--help` or a short README.
- Scope: tools. Enforce: script (no `.py` or `.c` under any `src/`, except the D-4 list); review.

#### PY-2 (MUST) Standard library first

Standard library only by default. A tool that needs packages lists them pinned in its own
`requirements.txt`, imports them lazily so `--help` works, and prints the install command when they
are missing. A script never installs packages.
- Scope: tools. Enforce: review; script (grep `pip", "install`).
- Good: `pi/naviGATR/tools/calibrate_camera.py:46-49` `def dependencies(): try: import cv2 ...`
  (trimmed).
- Bad: `bench/rs485_link/pi_transmit.py:51`
  `[sys.executable, "-m", "pip", "install", "--break-system-packages", package],`

#### PY-3 (MUST) Python 3.9 floor

Python 3.9 is the floor (the calibration tool already states it).
- Scope: tools. Enforce: ruff `target-version`.

#### PY-4 (MUST) Tool shape

A tool's shape: `argparse`; `def main(argv: list[str] | None = None) -> int`; `if __name__ ==
"__main__": sys.exit(main())`. No `sys.path` edits.
- Scope: tools. Enforce: review.
- Bad: `pi/naviGATR/tools/viewer/viewer_tests.py:65` `sys.path.insert(0, HERE)`.

#### PY-5 (MUST) Type hints

Type hints on every function signature, with `from __future__ import annotations` at the top of the
module while the floor is 3.9 (so `list[str] | None` parses). No type-checker gate yet.
- Scope: tools. Enforce: review.

#### PY-6 (MUST) pathlib, with, ElementTree

`pathlib` for paths, `with` for every file, `xml.etree.ElementTree` for XML (never regular
expressions over XML).
- Scope: tools. Enforce: ruff (SIM115); review.
- Bad: `pi/naviGATR/tools/viewer/viewer_tests.py:148`
  `text = open(src, encoding='utf-8').read()`

#### PY-7 (MUST) ruff and lizard

`ruff check` and `ruff format`, line length 100, configured once in root `ruff.toml`. Complexity and
size follow CPLX-1 to CPLX-5 through lizard, like C++ (9.6).
- Scope: tools. Enforce: ruff; lizard; script (file length).

#### PY-8 (MUST) Python tests

Tests are `test_<module>.py` with stdlib `unittest`, in the tool's folder. Tests that drive the C++
runtime run against the real binary in CI.
- Scope: tools. Enforce: CI.

#### PY-9 (SHOULD) Python docstrings

Docstrings and comments follow CMT-: a one-line module docstring; the `--help` text is the manual.
- Scope: tools. Enforce: review.

### 7.3 CMake, Make, PlatformIO (CMAKE-)

#### CMAKE-1 (MUST) One library target per component

One library target per component, and every CMake consumer links it. No CMake file compiles another
component's `.cpp` by path. Make and PlatformIO, which cannot link CMake targets, list the files
once each.
- Scope: build. Enforce: script (grep other components' `.cpp` in CMake).
- Bad: `brain/communiGATR/CMakeLists.txt:20-21` `${GATR2_REPO_ROOT}/translaGATR/frame_codec.cpp`
  and `link_documents.cpp`; translaGATR has no library target and six places compile it.

#### CMAKE-2 (MUST) One source list per library

One source list per library. The CMake target is the source of truth; `brain/gatr2_brain.mk` mirrors
it and a CI check fails when the two differ. Cross-component tests link targets instead of
re-listing sources.
- Scope: build (Brain, Pi tests). Enforce: script.
- Bad: `pi/naviGATR/tests/CMakeLists.txt:76-100` lists 23 Brain `.cpp` files for
  `brain_link_e2e_tests`, 6 of 11 communiGATR and 7 of 11 actuGATR sources.

#### CMAKE-3 (MUST) Shared root CMake module

A shared root module `cmake/gatr2.cmake` provides the repo root variable, C++17 with extensions off,
a `gatr2::warnings` interface target carrying the warning set and the always-a-bug errors (9.1,
CMAKE-13), and the googletest provider (system package first, else FetchContent pinned by
`URL_HASH SHA256`). Projects include it instead of copying blocks.
- Why: the gtest fetch block exists 4 times without a hash; `-Wall -Wextra` is written 22 times
  in 12 build files.
- Scope: build (CMake). Enforce: review.

#### CMAKE-4 (MUST) Target scope only

Target scope only: `target_*` commands with `PRIVATE`, `PUBLIC` or `INTERFACE`. No
`include_directories`, `add_compile_options`, `link_libraries`, or scope-less
`target_link_libraries`.
- Scope: build (CMake). Enforce: script.
- Bad: `translaGATR/tests/CMakeLists.txt:6` `add_compile_options(-Wall -Wextra)`.

#### CMAKE-5 (MUST) No globbing

Sources, tests and embedded assets are listed by name. No `file(GLOB)` (D-22).
- Scope: build (CMake). Enforce: script.
- Bad: `pi/naviGATR/CMakeLists.txt:143` `file(GLOB NAVIGATR_VIEWER_FILES CONFIGURE_DEPENDS`.

#### CMAKE-6 (MUST) CMake names

Names: `project(<component> LANGUAGES ...)` in lowercase; the library is `<component>`, its tests
`<component>_tests`; options `<COMPONENT>_BUILD_<X>` and `<COMPONENT>_WITH_<DEP>` (hardware
dependencies default OFF); compile definitions `<COMPONENT>_HAVE_<DEP>`.
- Scope: build (CMake). Enforce: review.
- Good: `NAVIGATR_BUILD_TESTS`, `NAVIGATR_WITH_LIBCAMERA`, `NAVIGATR_HAVE_LIBCAMERA`.

#### CMAKE-7 (MUST) C++17 everywhere

Library code is C++17 and built as C++17 on the host in CI. A platform build may compile it at a
newer standard when a platform header forces it (PROS `gnu++20`, libcamera C++20); that never
licenses newer features. `CMAKE_CXX_EXTENSIONS OFF` everywhere.
- Scope: build. Enforce: CI host build at C++17.
- Good: `brain/CMakeLists.txt:6` `set(CMAKE_CXX_EXTENSIONS OFF)`.
- Note: `pi/naviGATR/CMakeLists.txt:197` `target_compile_features(navigatr_core PUBLIC
  cxx_std_20)` applies only with libcamera on.

#### CMAKE-8 (MUST) Vendored dependencies

Runtime dependencies are vendored in `<component>/third_party/<lib>/` (DOC-16), built without our
warnings and included as SYSTEM. FetchContent only for test-only dependencies, always hash-pinned.
PlatformIO platforms are pinned to a commit.
- Scope: build. Enforce: review.
- Good: `pi/naviGATR/CMakeLists.txt:11-13` `# Vendored, built without our warning set.` then
  `target_include_directories(navigatr_tinyxml2 SYSTEM PUBLIC third_party)`.
- Good: `pico/aggreGATR/platformio.ini:10`
  `platform = https://github.com/maxgerhardt/platform-raspberrypi.git#a1df30f`.

#### CMAKE-9 (MUST) Generated outputs

Build-time outputs go to the build tree. A committed generated file has its generator committed, a
line 1 naming generator and input (HDR-8), and a test that fails when it is stale. A season input is
a cache variable (`GATR2_SEASON`), not a path literal in build glue.
- Scope: build. Enforce: test; review.
- Bad: `pi/naviGATR/CMakeLists.txt:224-239` hard-codes `config/override/field.xml` for the
  `field_references` target.

#### CMAKE-10 (SHOULD) CMake presets

Each CMake project ships `CMakePresets.json` with a fixed `binaryDir` under root
`build/<project>-<preset>`, and a Windows preset that sets the MinGW generator and the ucrt64
runtime path. Scripts and docs name presets, not folders.
- Scope: build (CMake). Enforce: review.

#### CMAKE-11 (MUST) PROS project files

PROS projects: `common.mk` and `project.pros` are PROS-owned and never edited; Makefiles are edited
only above the PROS "nothing below this line" marker; template placeholders are filled or deleted.
PROS headers are treated as system headers (`-isystem` or a diagnostic pragma around `api.h`) so our
warning set does not report them.
- Scope: Brain (apps). Enforce: review.
- Bad: `brain/operaGATR/Makefile:38` `# TODO: CHANGE THIS!`

#### CMAKE-12 (SHOULD) Build file comments

Comments in build files are noun-phrase labels.
- Scope: build. Enforce: review.
- Good: `brain/actuGATR/CMakeLists.txt:6` `# Library.`

#### CMAKE-13 (MUST) Warnings fail the build only when always a bug

Every build, local and CI, on every target, turns the always-a-bug warnings into errors with
`-Werror=<name>`: `return-type`, `switch`, `implicit-fallthrough=5`, `format`,
`delete-non-virtual-dtor`, `uninitialized`, `return-local-addr`, `mismatched-new-delete`,
`shift-count-overflow`, `shift-count-negative`, `div-by-zero` (9.1). Every other warning stays a
warning and never blocks a local build. CI builds with `-Werror`, so a new warning fails the PR and
`main` stays at zero warnings. The flag lists are written once per build system (`gatr2::warnings`,
`brain/gatr2_brain.mk`, `pico/aggreGATR/platformio.ini`) and every target uses them.
- Why: the lead (D-1): "if its a true warning then yes, but its an error if we genuienly cannot
  continue". Broken code never builds; a warning never stops a local experiment.
- Scope: build (every C++ target, tests included, TEST-16). Enforce: CI (9.9); review of the flag
  lists.

### 7.4 XML configuration (XML-)

#### XML-1 (MUST) XML case

Element names are PascalCase (acronyms as words: `Imu`, `BrainImu`); attribute names are snake_case.
- Scope: config. Enforce: test (a schema walk over `config/`).

#### XML-2 (MUST) Units in attribute names

Every physical-quantity attribute ends in its unit (`_m`, `_mm`, `_s`, `_ms`, `_us`, `_hz`, `_deg`,
`_rad`, `_dps`, `_px`; compounds spelled out, `_rad_per_sqrt_s`). Unitless by nature: ids, indices,
counts, channels, ports, GPIO numbers, ratios, enums, upstream library knobs.
- Scope: config. Enforce: review; test (a list of known unitless names).
- Bad: `pi/naviGATR/config/override/brain_profile_usb_camera.xml:28` `preview_max_width="640"`.

#### XML-3 (MUST) One root kind per file

One root kind per file, and the folder says the kind: `Configuration` (runnable), `Robot`,
`Pipeline`, section fragments (`Localization`), field (`Resource type="field_map"`). The flat
`<System>` dialect is either retired or documented as the flat form.
- Scope: config. Enforce: test.
- Bad: `pi/naviGATR/src/config/composition.cpp:144`
  `if (name != "Configuration" && name != "System") {` accepts two undocumented dialects.

#### XML-4 (MUST) id equals file stem

A configuration's `id` equals its file stem.
- Scope: config. Enforce: test.
- Bad: `pi/naviGATR/config/demo/synthetic_field_demo.xml:15`
  `<Configuration id="demo_synthetic_field">`.

#### XML-5 (MUST) Season data folder

Season data lives only in `config/seasons/<season>/` (the field file today). Robot descriptions,
device profiles and bring-up profiles are not season data and live outside it (D-23).
- Scope: config. Enforce: review.
- Bad: `config/override/` holds the season field and also the season-independent
  `brain_profile_usb.xml`, `brain_profile_rs485.xml` and `diagnostics/`.

#### XML-6 (MUST) Compose, do not copy

Compose, do not copy. A block used by two runnable files lives in one fragment referenced by
`file=`; runnable variants differ only in what differs.
- Scope: config. Enforce: review.
- Bad: the 32-line `BrainProfile` block is identical in
  `pi/naviGATR/config/override/brain_profile_usb.xml:55-86` and
  `pi/naviGATR/config/override/brain_profile_rs485.xml:56-87`.

#### XML-7 (MUST) Config templates

A template has `@UPPER_SNAKE@` tokens for every unmeasured value and a name the loader refuses
(D-24). A file with no tokens is not a template. The measured file is committed next to its
template.
- Scope: config. Enforce: test (the loader refusal exists: `composition.cpp:198-199` "is an
  intentionally non-runnable template", trimmed).

#### XML-8 (MUST) Every configuration loads

Every committed runnable configuration loads in a test: one test walks `config/`, resolves every
runnable root that is not a template, and fails with the file name.
- Scope: config. Enforce: test.

#### XML-9 (SHOULD) Scalars as attributes

A new scalar setting is an attribute on the element it configures, not a single-attribute child
element. Existing forms stay (the parser keeps its shape).
- Scope: config. Enforce: review.

#### XML-10 (MUST) XML comments and header

Comments are one label line per block. Provenance goes in `source=` and `revision=` attributes;
procedures go to docs. Files start with `<?xml version="1.0" encoding="UTF-8"?>` and are ASCII.
- Scope: config. Enforce: script (ASCII, header).

#### XML-11 (MUST) Config-to-code map

Reading a config tells you which code builds it. The component PIPELINE.md maps each stage element
to its `make_<stage>`. Each stage PIPELINE.md maps each plug-in folder to its XML element, and each
registered `type` to its `make_<type>` and file.
- Scope: Pi. Enforce: PIPE-3 symbol check. Drafts: Appendix C (component), Appendix A (stage
  example).

### 7.5 Shell (SH-)

#### SH-1 (MUST) Bash header

`#!/usr/bin/env bash`, one label line, `set -euo pipefail`.
- Scope: tools. Enforce: script.
- Good: `pi/naviGATR/tools/install_service.sh:3` `set -euo pipefail`.
- Bad: `pi/naviGATR/tools/viewer_smoke.sh:31` `set -u` only.

#### SH-2 (MUST) shellcheck clean

shellcheck clean (from the `shellcheck-py` wheel; no node, no system package).
- Scope: tools. Enforce: shellcheck.
- Bad: `pi/naviGATR/tools/viewer_smoke.sh:50` `--keep) KEEP=1; shift;;` sets a flag nothing
  reads, so the documented `[--keep]` (:20) does nothing (found by shellcheck).

#### SH-3 (SHOULD) usage heredoc

`--help` prints a `usage()` heredoc; comments stay labels (no help text grepped from comments).
- Scope: tools. Enforce: review.

#### SH-4 (MUST) Git Bash and Linux

A script runs in Git Bash on Windows and bash on Linux, or refuses early with a clear message.
Windows specifics (cygpath, browser path, runtime DLL path) live in one sourced helper,
`tools/lib/host.sh`.
- Scope: tools. Enforce: review.
- Good: `pi/naviGATR/tools/install_service.sh:62` `[[ $(uname -s) == Linux ]] || fail 'Install
  this service on the Raspberry Pi, not on the host computer'`.

#### SH-5 (MUST) Shell is glue

Shell is glue. Past 150 lines, or once it parses structured data (JSON, DOM, XML), it becomes a C++
or Python tool (PY-1).
- Scope: tools. Enforce: review.

#### SH-6 (SHOULD) Flag names

Flags are kebab-case with units in the name (`--duration-s`); binary paths come from preset names,
never a hard-coded `build-<something>/`.
- Scope: tools. Enforce: review.

### 7.6 Repository (REPO-)

#### REPO-1 (MUST) One build output root

Build output lives only under root `build/<project>-<preset>/`. No build folders inside components;
Python virtual environments live in the tool's `.venv/` and are ignored.
- Scope: All. Enforce: `.gitignore`; review.

#### REPO-2 (MUST) .gitattributes

`.gitattributes` marks binary types (9.5), followed by one renormalize commit.
- Why: the committed PDFs are classified as text today (`.gitattributes` is only
  `* text=auto eol=lf`).
- Scope: All. Enforce: review.

#### REPO-3 (MUST) .editorconfig

A root `.editorconfig` (9.4).
- Scope: All. Enforce: editor; review.

#### REPO-4 (MUST) Nothing machine-specific tracked

Nothing machine-specific is tracked: no absolute paths, logs, build output or secrets.
- Scope: All. Enforce: script (tracked-file denylist).
- Bad: `debug.log` is tracked; `.vscode/settings.json:2`
  `"cmake.sourceDirectory": "C:/GATR2-Electronics/brain"`.

#### REPO-5 (MUST) Generators next to output

A generator is committed next to what it maintains, and a check fails when output and generator
disagree.
- Scope: All. Enforce: test or script.
- Bad: the printables generator lives in the ignored `build-printables/` and still writes to the
  old path.

#### REPO-6 (MUST) Case-only renames

Case-only renames use the two-step `git mv` (`docs/naming.md`); CI on Linux fails on a case
mismatch.
- Scope: All. Enforce: CI.

#### REPO-7 (MUST) Commit rules

Commits: plain descriptive messages. A fix names the symptom and the bug class (10.4) and carries
its regression test (`encoder deltas stayed current after Pico frames stopped (B3)`). Move-only,
format-only and logic changes are separate commits, marked as such. A mass reformat is listed in
`.git-blame-ignore-revs`. Branch per feature, PR to `main`, CI required; small PRs once scaffolding
is done.
- Why: the commit history carries no bug information today ("bug fixes" twice, "janky fix",
  "new stuff" ten times).
- Scope: All. Enforce: review; PR template.

#### REPO-8 (MUST) Contract reviews

Changes to stage contract headers need a reviewer from each side of the seam (CODEOWNERS).
translaGATR changes follow WIRE-7.
- Scope: Pi (contracts). Enforce: CODEOWNERS.

#### REPO-9 (MUST) Pinned toolchains

Every compiler and platform package (host GCC, the PROS toolchain and kernel template, the Pico
toolchain package and platform commit) is pinned by exact version in one file, `docs/toolchains.md`,
and CI installs exactly those. A version bump is its own commit with the full build and test run.
- Why: bug class B15 (the PROS template defaulted to gnu++26, which arm gcc 13.3 cannot build).
  Installed today: Arm GNU Toolchain 13.3.Rel1 (PROS), arm-none-eabi 14.3.0 from
  `toolchain-rp2040-earlephilhower 5.140200` (Pico), MSYS2 GCC 15.2 (host).
- Scope: build. Enforce: review; CI.

---

## 8. Review checklist (REV-)

Each item is a question the reviewer answers. Items are MUST unless marked.

| ID | Question | Rules |
|---|---|---|
| REV-1 | What is this for, in one line, and is that line in the PIPELINE.md? | P-1, PIPE-5 |
| REV-2 | Is it in the right folder: the stage or plug-in that owns it, generic vs specific split? Does its namespace match the folder? | STRUCT-1 to 6, NS-2 |
| REV-3 | Does it widen a seam, or leak hardware, wire or season knowledge into generic code? | P-8, SYS-5, SYS-7, SYS-19 |
| REV-4 | Is every new mechanism, thread, queue, message, config element or abstraction admitted? | SYS-8 to 14 |
| REV-5 | For each new class: which invariant or resource does it guard? Could it be a function? | TYPE-1 to 6 |
| REV-6 | Does every call site read like the domain: aliases, named structs, no bare bools, steps called by their verb? | ALIAS-, TYPE-9 to 12, TYPE-17, API-2 |
| REV-7 | Which clock, which unit, which frame, for every time and quantity? | UNIT-, TIME- |
| REV-8 | For every queue: capacity, drop policy, counter, reader of the counter? | CON-9, ERR-9 |
| REV-9 | For every state machine and wait: what are the exits, and does every wait time out? | P-7, CON-4 |
| REV-10 | Which TEST-10 fault patterns apply, and are they here? Was the new test seen failing? | TEST-10, TEST-12 |
| REV-11 | Errors: no throw, the layer's error shape, `[[nodiscard]]`, nothing ignored silently? | ERR- |
| REV-12 | Concurrency: one writer, no nested locks, no I/O under a lock, RAII only? | CON-, RT-5 to 7 |
| REV-13 | Target code: no heap in steady state, no blocking in ISRs or control tasks? | RT- |
| REV-14 | Within CPLX limits, or a one-line reason? | CPLX- |
| REV-15 | Comments ASCII terse labels; docs and PIPELINE.md updated in the same PR? | CMT-, PIPE-4, DOC-10 |
| REV-16 | Do status claims match the evidence? | P-12, DOC-9 |
| REV-17 | Is the commit move-only, format-only, rename-only or logic, and not mixed? | REPO-7, TEST-13 |
| REV-18 (SHOULD) | Is anything here more general than the problem needs? | P-3 |
| REV-19 | Does anything depend on a field list a new field can miss? Are labels read with their data? | SYS-24, SYS-25 |
| REV-20 | Every stored callable: what does it capture, and who outlives whom? | OWN-9, OWN-10 |
| REV-21 | Every float at a seam: finite? Every float to integer: through the named helper? | NUM-1 to NUM-3 |
| REV-22 | Every platform branch: which CI job builds it? Every vendor quirk: labelled? | SYS-26 |

---

## 9. Tooling appendix

Config text lives here only until adoption. Nothing below is in the repo yet; each piece lands in
phase 3 of section 11 as a real file, and this section then shrinks to a table of check and
config file. Tool versions: clang-format and clang-tidy 22.1.8 (pip wheels; clang-tidy is not
installed system-wide on the dev box), lizard 1.24, ruff 0.16, shellcheck 0.11 (`shellcheck-py`
wheel). Noise counts below are from the surveys' runs on this tree.

### 9.1 Compiler warnings

D-1, decided 2026-09-28 (CMAKE-13): a warning fails a build only when it always means broken code.
Those are the always-a-bug errors at the end of this section, errors in every build. Every flag
below is adopted as a warning: it never blocks a local build, and CI builds with `-Werror`, so a
new warning fails the PR. "Gate" below means in that build; "advisory" means a separate report job
that never blocks. The baseline `-Wall -Wextra` is already clean on all 210 host translation units
and all 55 ARM translation units measured. Vendored code keeps `-w` and SYSTEM includes.

**Host GCC** (CMake: Brain libraries and sims, naviGATR, translaGATR, Pico host tests), gate, as
the `gatr2::warnings` interface target:

```text
-Wall -Wextra -Wpedantic
-Wshadow -Wundef -Wmissing-declarations
-Wnon-virtual-dtor -Woverloaded-virtual -Wsuggest-override
-Wold-style-cast -Wcast-qual -Wcast-align=strict
-Wformat=2 -Wimplicit-fallthrough=5
-Wduplicated-cond -Wduplicated-branches -Wlogical-op
-Wextra-semi -Wzero-as-null-pointer-constant -Wredundant-decls
-Wmismatched-tags -Wredundant-tags -Wvla -Walloca
-Wfloat-conversion
```

Cost to adopt: 18 existing sites (`-Wshadow` 9, `-Wundef` 5, `-Wmissing-declarations` 3,
`-Woverloaded-virtual` 1). `-Wfloat-conversion` (NUM-3) adds none: the survey's `-Wconversion`
logs hold 47 unique sites in GATR2 code, all integer narrowing or integer to double, and none
tagged `[-Wfloat-conversion]`; measure it on the Pico and PROS targets before adding it there.
Kept on purpose: `-Wdangling-reference` (in `-Wextra` on GCC 15.2; it caught a test bug, B14);
never add `-Wno-dangling-reference`.

**Host GCC, component extras:**
- translaGATR and Pico host modules, gate: `-Wconversion -Wsign-conversion` (0 hits in their
  sources today; the codec tests have 22 to fix or drop the pair).
- Everything else, advisory build (counts reported, not gated):
  `-Wconversion -Wsign-conversion -Wswitch-enum -Wdouble-promotion`. naviGATR has about 125
  conversion hits, 58 of them in `core/time.h`.

**Pico target** (PlatformIO `build_src_flags`), gate; only flags silent in both our code and the
arduino-pico headers:

```text
-Wall -Wextra -Wno-psabi
-Wdouble-promotion -Wimplicit-fallthrough=5
-Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wformat=2
-Wcast-qual -Wcast-align=strict -Wmissing-declarations
-Wvla -Walloca -Wstack-usage=1024
```

**Brain PROS target** (`GATR2_WARNFLAGS` in `brain/gatr2_brain.mk:82` and the app Makefiles),
gate:

```text
-Wall -Wextra
-Wundef -Wzero-as-null-pointer-constant -Woverloaded-virtual
-Wimplicit-fallthrough=5 -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wformat=2
-Wcast-qual -Wcast-align=strict -Wvla -Walloca -Wstack-usage=4096
```

The PROS-noisy flags (`-Wsuggest-override`, `-Wnon-virtual-dtor`, `-Wextra-semi`,
`-Wredundant-tags`, `-Wshadow`: 415 warnings from PROS headers) stay on the host build of the
same libraries until the PROS headers are system headers (CMAKE-11).

**Rejected everywhere:** `-Wfloat-equal` (all 29 hits are intended sentinel checks),
`-Wuseless-cast` (contradicts itself across targets because `uint32_t` is `unsigned int` on
x86_64 and `unsigned long` on arm-none-eabi), `-Wnull-dereference` (optimizer-path false
positives), `-Wstack-usage=<bytes>` on host (fires only in gtest bodies).

**Always-a-bug errors (D-1, CMAKE-13).** Every build, local and CI, every target. Each name was
checked on the three installed compilers (host MSYS2 GCC 15.2.0, Pico arm-none-eabi 14.3.0, PROS
Arm GNU Toolchain 13.3.Rel1): it is accepted, and a sample of its bug fails the build. Each is
documented in the GCC 15.2 manual (Warning Options; the C++-only ones under C++ Dialect Options).

| Flag | Catches | Why always a bug |
|---|---|---|
| `-Werror=return-type` | a non-void function that can end without `return` | undefined behavior |
| `-Werror=switch` | a switch on an enum type missing an enumerator, with no `default` | ENUM-2: a new enumerator is unhandled |
| `-Werror=implicit-fallthrough=5` | a case that runs into the next without `[[fallthrough]];` | a missing `break` or a missing mark |
| `-Werror=format` | printf-style arguments that do not match the format, or extra ones | undefined behavior or a wrong print |
| `-Werror=delete-non-virtual-dtor` | `delete` through a base with virtual functions and no virtual destructor | undefined behavior |
| `-Werror=uninitialized` | a variable read before it is ever set, `int i = i;` included | undefined behavior |
| `-Werror=return-local-addr` | returning a pointer or reference to a local | dangling on return |
| `-Werror=mismatched-new-delete` | `new[]` freed by `delete`, and similar pairs | undefined behavior |
| `-Werror=shift-count-overflow`, `-Werror=shift-count-negative` | a shift by the type width or more, or by a negative count | undefined behavior |
| `-Werror=div-by-zero` | integer division by a constant zero | undefined behavior |

Measured traps (same on all three compilers):
- `-Werror=<name>` resets a leveled warning: after `-Wimplicit-fallthrough=5` it drops back to
  level 3, and after `-Wformat=2` it drops `format-nonliteral` and `format-security`. So the level
  is part of the name, and the list goes before the warning set, never after. CMake drops a
  repeated option from a target, so no flag is written in two lists.
- `-Werror=<name>` also promotes what `<name>` enables: `format` pulls in `format-overflow`,
  `format-truncation`, `format-extra-args` and, with `-Wformat=2`, `format-nonliteral`,
  `format-security` and `format-y2k`; `uninitialized` pulls in `maybe-uninitialized`. The heuristic
  ones go back to warnings in local builds; `format-extra-args` stays an error.
- `-Wno-error=<name>` beats a later `-Werror`, so CI leaves the demotions out.

Dropped from the candidates:
- `init-self`: accepted, but it never fires: GCC 13 to 15 report C++ self-initialization as
  `-Wuninitialized`, which the list has.
- `non-virtual-dtor`: not always a bug (a base never deleted through a pointer is fine, and
  `delete-non-virtual-dtor` catches the real case); it also fires in PROS headers. It stays a host
  warning (TYPE-8).

Cost today: 0 sites. Every name is in `-Wall`, `-Wextra` or on by default, which are clean on every
measured translation unit, and the tier-1 survey found 0 `-Wformat=2` and 0
`-Wimplicit-fallthrough=5` hits.

Order in every build: the always-a-bug list, the target's warning set, then the local demotions, or
`-Werror` in CI.

```text
# Always-a-bug errors.
-Werror=return-type -Werror=switch -Werror=implicit-fallthrough=5 -Werror=format
-Werror=delete-non-virtual-dtor -Werror=uninitialized -Werror=return-local-addr
-Werror=mismatched-new-delete -Werror=shift-count-overflow -Werror=shift-count-negative
-Werror=div-by-zero

# Local demotions.
-Wno-error=format-nonliteral -Wno-error=format-security -Wno-error=format-y2k
-Wno-error=format-overflow -Wno-error=format-truncation -Wno-error=maybe-uninitialized
```

CMake, in `cmake/gatr2.cmake` (CMAKE-3), with `GATR2_BUG_ERRORS`, `GATR2_HOST_WARNINGS` and
`GATR2_LOCAL_DEMOTIONS` set to the lists above. Only first-party targets link it, so vendored code
and googletest are untouched, and CMake 3.16 is enough:

```cmake
option(GATR2_WARNINGS_AS_ERRORS "Every warning is an error (CI)" OFF)

add_library(gatr2_warnings INTERFACE)
add_library(gatr2::warnings ALIAS gatr2_warnings)
target_compile_options(gatr2_warnings INTERFACE ${GATR2_BUG_ERRORS} ${GATR2_HOST_WARNINGS})
if(GATR2_WARNINGS_AS_ERRORS)
    target_compile_options(gatr2_warnings INTERFACE -Werror)
else()
    target_compile_options(gatr2_warnings INTERFACE ${GATR2_LOCAL_DEMOTIONS})
endif()
```

CMake's own `CMAKE_COMPILE_WARNING_AS_ERROR` is not used: it needs CMake 3.24 (every CMakeLists
says 3.16 today), and an older CMake silently ignores it.

Make, in `brain/gatr2_brain.mk` (today `GATR2_WARNFLAGS ?= -Wall -Wextra`, line 82); the apps'
`WARNFLAGS` lines use the same variables, and CI runs `make GATR2_WERROR=1`:

```make
GATR2_WARNFLAGS ?= $(GATR2_BUG_ERRORS) $(GATR2_PROS_WARNINGS) \
                   $(if $(filter 1,$(GATR2_WERROR)),-Werror,$(GATR2_LOCAL_DEMOTIONS))
```

PlatformIO, in `pico/aggreGATR/platformio.ini` (today `build_src_flags = -Wall -Wextra` under
`[env]`); CI builds the `_ci` envs:

```ini
[gatr2]
bug_errors = <the always-a-bug list>
warnings = <the Pico set above>
local_demotions = <the local demotions>

[env]
build_src_flags = ${gatr2.bug_errors} ${gatr2.warnings} ${gatr2.local_demotions}

[env:hat2_bno08x_ci]
extends = env:hat2_bno08x
build_src_flags = ${gatr2.bug_errors} ${gatr2.warnings} -Werror
```

`hat2_asm330_ci` is the same. Result: broken code never builds anywhere, `main` has zero warnings,
and a warning never blocks a local build.

### 9.2 clang-tidy

Root `.clang-tidy`. Every check name exists in clang-tidy 22.1.8; the file passes
`clang-tidy --verify-config` (re-run on this revision), and the `FunctionIgnoredRegexp` was tested
to accept `make_*` and `register_*` while still flagging other snake_case functions.

```yaml
---
Checks: >
  -*,
  bugprone-*,
  -bugprone-easily-swappable-parameters,
  -bugprone-narrowing-conversions,
  -bugprone-implicit-widening-of-multiplication-result,
  -bugprone-throwing-static-initialization,
  cppcoreguidelines-avoid-goto,
  cppcoreguidelines-init-variables,
  cppcoreguidelines-interfaces-global-init,
  cppcoreguidelines-macro-usage,
  cppcoreguidelines-missing-std-forward,
  cppcoreguidelines-no-malloc,
  cppcoreguidelines-noexcept-destructor,
  cppcoreguidelines-noexcept-move-operations,
  cppcoreguidelines-noexcept-swap,
  cppcoreguidelines-prefer-member-initializer,
  cppcoreguidelines-pro-type-cstyle-cast,
  cppcoreguidelines-pro-type-member-init,
  cppcoreguidelines-rvalue-reference-param-not-moved,
  cppcoreguidelines-slicing,
  cppcoreguidelines-special-member-functions,
  cppcoreguidelines-virtual-class-destructor,
  google-build-using-namespace,
  google-explicit-constructor,
  misc-*,
  -misc-include-cleaner,
  -misc-const-correctness,
  -misc-no-recursion,
  modernize-avoid-bind,
  modernize-avoid-c-style-cast,
  modernize-deprecated-headers,
  modernize-loop-convert,
  modernize-make-shared,
  modernize-make-unique,
  modernize-redundant-void-arg,
  modernize-use-bool-literals,
  modernize-use-equals-default,
  modernize-use-equals-delete,
  modernize-use-nullptr,
  modernize-use-override,
  modernize-use-using,
  performance-*,
  -performance-enum-size,
  readability-braces-around-statements,
  readability-container-size-empty,
  readability-duplicate-include,
  readability-else-after-return,
  readability-function-cognitive-complexity,
  readability-function-size,
  readability-identifier-naming,
  readability-inconsistent-declaration-parameter-name,
  readability-isolate-declaration,
  readability-misleading-indentation,
  readability-redundant-casting,
  readability-redundant-control-flow,
  readability-redundant-declaration,
  readability-simplify-boolean-expr,
  readability-static-accessed-through-instance,
  readability-use-anyofallof,
  concurrency-*
WarningsAsErrors: ''
HeaderFilterRegex: '.*[/\\](brain|pi|pico|translaGATR)[/\\].*'
ExcludeHeaderFilterRegex: '.*[/\\](third_party|_deps|include[/\\]pros)[/\\].*'
SystemHeaders: false
FormatStyle: none
CheckOptions:
  modernize-use-using.IgnoreExternC: true
  cppcoreguidelines-special-member-functions.AllowSoleDefaultDtor: true
  cppcoreguidelines-special-member-functions.AllowMissingMoveFunctionsWhenCopyIsDeleted: true
  misc-non-private-member-variables-in-classes.IgnoreClassesWithAllMemberVariablesBeingPublic: true
  readability-function-cognitive-complexity.Threshold: 25
  readability-function-cognitive-complexity.IgnoreMacros: true
  readability-function-size.LineThreshold: none
  readability-function-size.ParameterThreshold: none
  readability-function-size.NestingThreshold: 5
  readability-braces-around-statements.ShortStatementLines: 0
  misc-use-internal-linkage.FixMode: None
  bugprone-unused-return-value.AllowCastToVoid: true
  bugprone-unused-return-value.CheckedReturnTypes: '^::std::error_code$;^::std::error_condition$;^::std::errc$;^::std::optional$;^::navigatr::FunctionStatus$'
  bugprone-exception-escape.FunctionsThatShouldNotThrow: 'estimationWorker,fieldWorker,captureLoop'
  concurrency-mt-unsafe.FunctionSet: posix
  cppcoreguidelines-macro-usage.AllowedRegexp: '^(GATR2_|[A-Z]+_HAVE_|[A-Z]+_WITH_|NOMINMAX$|WIN32_LEAN_AND_MEAN$|FD_SETSIZE$|STBI?_|PROS_USE_)'
  readability-identifier-naming.NamespaceCase: lower_case
  readability-identifier-naming.ClassCase: CamelCase
  readability-identifier-naming.StructCase: CamelCase
  readability-identifier-naming.EnumCase: CamelCase
  readability-identifier-naming.TypeAliasCase: CamelCase
  readability-identifier-naming.TemplateParameterCase: CamelCase
  readability-identifier-naming.EnumConstantCase: CamelCase
  readability-identifier-naming.EnumConstantPrefix: k
  readability-identifier-naming.FunctionCase: camelBack
  readability-identifier-naming.FunctionIgnoredRegexp: '^(make|register)_[a-z0-9_]+$|^(main|setup|loop|initialize|disabled|competition_initialize|autonomous|opcontrol)$'
  readability-identifier-naming.MethodCase: camelBack
  readability-identifier-naming.VariableCase: lower_case
  readability-identifier-naming.ParameterCase: lower_case
  readability-identifier-naming.PrivateMemberCase: lower_case
  readability-identifier-naming.PrivateMemberSuffix: _
  readability-identifier-naming.ProtectedMemberCase: lower_case
  readability-identifier-naming.ProtectedMemberSuffix: _
  readability-identifier-naming.PublicMemberCase: lower_case
  readability-identifier-naming.ConstexprVariableCase: CamelCase
  readability-identifier-naming.ConstexprVariablePrefix: k
  readability-identifier-naming.GlobalConstantCase: CamelCase
  readability-identifier-naming.GlobalConstantPrefix: k
  readability-identifier-naming.GlobalVariableCase: lower_case
  readability-identifier-naming.GlobalVariablePrefix: g_
  readability-identifier-naming.MacroDefinitionCase: UPPER_CASE
...
```

Enabled checks that no rule names (for example `modernize-use-nullptr`,
`cppcoreguidelines-slicing`) are general hygiene; a finding is fixed and cited by its check name.

Overlays (each `InheritParentConfig: true`):
- `brain/.clang-tidy`, `pico/.clang-tidy`, `translaGATR/.clang-tidy`: `Checks: 'misc-no-recursion'`
  (RT-9).
- every `tests/.clang-tidy`:
  `Checks: '-bugprone-unchecked-optional-access,-readability-function-cognitive-complexity,-readability-function-size,-misc-non-private-member-variables-in-classes,-google-build-using-namespace'`
  (TEST-16; tests may `using namespace` their component, NS-4). `misc-use-internal-linkage` stays
  on in tests: it is the TEST-15 check, and it reports 5 real test findings today.
- advisory only, run as a report: `cppcoreguidelines-use-enum-class` outside translaGATR,
  `readability-convert-member-functions-to-static`, `bugprone-easily-swappable-parameters`,
  `misc-const-correctness`.

Checks that need tuning before they gate (report-only until then):
- `readability-identifier-naming`: 349 findings today (225 non-test). It gates after the naming
  decisions (D-8, D-16) and the rename pass; the Pico enumerators, `measuredAt`-style fields and
  lambdas named like functions are most of it.
- `readability-function-cognitive-complexity`, `readability-function-size` (nesting only): gate
  on new and changed functions only; the existing offenders sit in the CPLX baseline. lizard owns
  NLOC and parameters, because clang-tidy's `LineThreshold` counts physical lines and would fail
  functions lizard passes.
- `cppcoreguidelines-macro-usage`, `bugprone-exception-escape` (with the thread entries): measure
  before gating. The HTTP and inspection thread bodies are both `Impl::run` and need unique names
  before they can be listed (ERR-11).
- `bugprone-unused-return-value` with the options above: 0 findings over the 119 host production
  translation units today, so it gates at once; it flags a discarded `std::optional` or
  `FunctionStatus` and accepts `static_cast<void>(...)`.
- `concurrency-mt-unsafe`: gates only in the Linux job (the MinGW run found 0 while
  `std::strerror` runs on the HTTP thread, `pi/naviGATR/src/inspection/http_server.cpp:137`).
- `misc-non-private-member-variables-in-classes`: 11 Brain source findings to fix first.
- `bugprone-exception-escape`: one known finding (`CaptureRecorder::~CaptureRecorder` calls a
  `stop()` that can throw).
- `cppcoreguidelines-virtual-class-destructor`: fires on the tinyxml2 `XMLElement` forward
  declaration; exclude the vendored type.
- `bugprone-random-generator-seed`: one false positive at
  `pi/naviGATR/src/impl/commands/brain_link_commands.cpp:158` (reseeded from a `std::seed_seq`).

Checks considered and rejected (noise measured on this tree): the magic-number checks (5626),
`readability-identifier-length` (5440), `misc-include-cleaner` on MinGW (4373), the GSL bounds
checks (hundreds; fixed arrays and pointer arithmetic are the codec's nature),
`modernize-use-nodiscard` (439, too broad; ERR-3 targets status functions),
`modernize-use-auto` (conflicts with P-5), `readability-named-parameter` (contradicts
unnamed-unused parameters), `cppcoreguidelines-avoid-const-or-ref-data-members` (stage inputs are
views on purpose), `modernize-concat-nested-namespaces` (conflicts with NS-6).

Runs on a Linux compile database in CI (`CMAKE_EXPORT_COMPILE_COMMANDS=ON`). It saw only host
translation units in the survey; Pico drivers, Brain `pros/` and the apps get GCC target checks.

### 9.3 clang-format

The pinned repo `.clang-format` (clang-format 22.1.8) stays the authority. D-25 (decided
2026-09-28) adopts every addition below, the JavaScript section included. Tested with 22.1.8 on
scratch files (the C++ part inserts braces, puts `public:` at column 0, keeps short
case labels, regroups includes into own / `<...>` / `"..."`, and the JavaScript section parses):

```yaml
BasedOnStyle: LLVM
IndentWidth: 4
ColumnLimit: 100
PointerAlignment: Left
DerivePointerAlignment: false
NamespaceIndentation: None
FixNamespaceComments: true
AlignConsecutiveAssignments: true
AlignConsecutiveDeclarations: true
BreakBeforeBraces: Custom
BraceWrapping:
  AfterNamespace: true
# Additions.
InsertBraces: true
AccessModifierOffset: -4
AllowShortCaseLabelsOnASingleLine: true
CompactNamespaces: false
IncludeBlocks: Regroup
IncludeCategories:
  - Regex: '^<.*>'
    Priority: 1
  - Regex: '^".*"'
    Priority: 2
---
Language: JavaScript
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 100
```

- `InsertBraces: true` enforces braces on every control statement (FMT-2; 124 missing today, 40
  of them in `pi/naviGATR/src/config/composition.cpp`). It applies to C++ only; JS braces are
  JS-13. Without it, clang-tidy `readability-braces-around-statements` still reports them.
  clang-format's own documentation warns that it can produce incorrect code, so it runs as its
  own commit per component after the whitespace reformat, built and tested (gate A), with each
  inserted brace read as code.
- `AccessModifierOffset: -4` and `AllowShortCaseLabelsOnASingleLine: true` match what the Brain and
  naviGATR code already writes. The JavaScript section changes 776 of 7884 spectaGATR lines.
- The tree is not formatted today: 369 of 437 C++ files differ under the pinned config, 356 even
  with the knobs above; most of it is alignment drift. The additions land with one whole-repo
  format-only pass on the main tree, after the restructure patch and the spectuGATR Phase A patch
  are both applied, so the reformat cannot conflict with them (`restructure/SPEC_ADDENDUM.md`). One
  format-only commit per component, each listed in `.git-blame-ignore-revs`; then CI runs
  `clang-format --dry-run --Werror`.
- Excluded by path (a `.clang-format-ignore` file): `third_party/`, the PROS template headers
  under `brain/*/include/pros/` and `main.h`/`api.h`, generated files (HDR-8).
- JS: `x_m: value` object spacing comes from the Google base; matrix literals such as
  `pi/naviGATR/spectaGATR/transforms.js:40-42` need `// clang-format off` / `on`.

### 9.4 .editorconfig

```ini
root = true

[*]
charset = utf-8
end_of_line = lf
insert_final_newline = true
trim_trailing_whitespace = true
indent_style = space
indent_size = 4

[{Makefile,*.mk}]
indent_style = tab

[*.{yml,yaml}]
indent_size = 2

[*.ps1]
end_of_line = crlf
```

### 9.5 .gitattributes

```text
* text=auto eol=lf
*.pdf  binary
*.png  binary
*.jpg  binary
*.zip  binary
*.step binary
*.stl  binary
*.a    binary
*.ps1  text eol=crlf
```

Followed by one `git add --renormalize .` commit (106 tracked files are CRLF in the working tree
today).

### 9.6 Other linters

`ruff.toml` at the repo root (PY-7):

```toml
line-length = 100
target-version = "py39"

[lint]
extend-select = ["B", "UP", "SIM"]
```

Python complexity and size use lizard and the file-length script, with the same numbers as C++
(CPLX); ruff's `C90` would fail at CCN 15, the warn tier, and has no NLOC, parameter or file
gate. `pi/naviGATR/tools/viewer/viewer_tests.py` is 1131 lines today.

Measured: 37 findings with ruff defaults over the 7 tracked Python files, 4 in the classic
E4/E7/E9/F set; `ruff format` would touch all 7 files once.

JavaScript lint (D-21). Option A, recommended when node is acceptable in CI only: ESLint flat
config (7 findings today under `recommended`):

```js
import js from "@eslint/js";
import globals from "globals";
export default [
    js.configs.recommended,
    {
        files: ["spectaGATR/**/*.js", "tools/**/*.js"],
        languageOptions: {ecmaVersion: 2022, sourceType: "module", globals: globals.browser},
        rules: {
            eqeqeq: ["error", "always", {null: "ignore"}],
            "no-var": "error",
            "prefer-const": "error",
            curly: ["error", "all"],
            "no-shadow": "error",
            "no-implicit-coercion": "warn",
            complexity: ["warn", 15],
            "max-depth": ["warn", 4],
            "max-lines-per-function": ["warn", {max: 60, skipBlankLines: true, skipComments: true}],
        },
    },
];
```

Option B, no node anywhere: a stdlib Python checker in `tools/ci/` covering the grep-able subset
(no `var`, no `==`/`!=`, no `console.`, import specifiers, `innerHTML =` only with a literal,
`|| <number>`, duplicate helper exports, ASCII, header), with syntax checked by the headless
browser run that already imports every module.

Shell: `shellcheck` over every `*.sh` (SH-2).

Complexity (CPLX-1 to CPLX-5, CPLX-10), C++, JS and Python:

```text
lizard -w -C 15 -Tnloc=60 -a 5    # warn report
lizard -w -C 25 -Tnloc=100 -a 7   # fail, new or changed functions; baseline for the rest
```

`check_complexity.py` wraps the fail run: it gates only new or changed functions and keys the
baseline by component plus qualified function name, not by path, so the restructure moves do not
reset it. lizard's nesting extension crashes on this code; nesting is checked by clang-tidy
`readability-function-size` (C++, NestingThreshold 5, report at 4) and ESLint `max-depth` or
review (JS).

### 9.7 Sanitizers and runtime checks

- Linux CI, host tests: `-fsanitize=address,undefined -fno-sanitize-recover=all
  -fno-omit-frame-pointer`, Debug.
- A separate TSan job for naviGATR (TSan cannot combine with ASan).
- All host Debug builds: `-D_GLIBCXX_ASSERTIONS` (libstdc++ bounds checks).
- Windows local: ASan, UBSan and TSan do not link with MinGW GCC 15.2 (measured); only
  `-fsanitize=undefined -fsanitize-trap=undefined` works.
- Target builds: no sanitizers; stack budgets through `-Wstack-usage=<bytes>` (RT-13).

### 9.8 Repo check scripts

Stdlib Python scripts in `tools/ci/`, each small, each named after what it checks. Every check
compares against `git ls-files` with exact case (the Windows filesystem hides case errors). Pattern
checks skip comment and string text. `delete` matches only `\bdelete\s*(\[\s*\])?\s+[A-Za-z_(]`
not after `=`, and `new` only `\bnew\s+[A-Za-z_:]+\s*[({\[]`, so `= delete` and the word "new" in
a label never match.

| Script | Checks | Rules |
|---|---|---|
| `check_ascii.py` | non-ASCII in tracked first-party text | CMT-1, XML-10 |
| `check_layout.py` | the PIPE-1 pipeline folders and every stage folder have PIPELINE.md; stage folders have contract.h and a stage.cpp that defines `make_<stage>`; no `impl/`, `runtime/`, `utils/` folders, no folder named like its parent; basename uniqueness; folders over 15 units (report); `.py` or `.c` under `src/` outside the D-4 list | STRUCT-1, 2, 5, 7, 10, PIPE-1, PY-1, TYPE-17 |
| `check_includes.py` | the SYS-19 layer order; only register files, tests and tools include a plug-in folder from outside it; no `../` includes; portable headers include no platform header; translaGATR includes nothing else; pipeline includes only the hub from `debug/`; observers include no plug-in folder | SYS-19, STRUCT-8, HDR-3, HDR-7, RT-12 |
| `check_namespaces.py` | top-level declarations outside a namespace (allowlist of platform callbacks); path vs namespace per the NS-2 mapping (grouping and layout folders add no level) | NS-1, NS-2, NS-8 |
| `check_patterns.py` | `throw`, `std::sto`, `.at(`, `.value()`; `assert(` under brain/pico/translaGATR; `using namespace` in headers; `printf`/`std::cout` outside mains and tools; `typedef` outside `extern "C"`; `new`/`delete`; `volatile` outside pico; `memory_order_` outside the ring type; manual `take`/`give`/`lock`/`unlock`; a class whose members are all static; a class with a reference member and no deleted copy; heap types in pico and translaGATR; `uint8_t ... // translagatr::`; `Name(uint8_t` outside translaGATR; literal pi; string-built JSON; `tinyxml2` outside config; `long` durations; `#ifdef` or `defined(` on a build-variant macro | ERR-1, ERR-6, NS-4, LOG-1, ALIAS-10, OWN-2, OWN-5, CON-5, CON-6, CON-11, TYPE-4, RT-1, TYPE-13, WIRE-2, UNIT-3, API-6, CFG-1, TIME-5, CONST-7 |
| `check_clocks.py` | clock reads, including `system_clock`, outside the allowlist | TIME-1, TIME-2, TIME-11 |
| `check_tests.py` | `sleep_for` in tests (shrinking allowlist); underscores in test names; `file(GLOB` in test CMake; printing in tests; `gtest_discover_tests` without `TIMEOUT` | TEST-1, 2, 6, 8 |
| `check_complexity.py` | lizard fail tier on new or changed functions (C++, JS, Python), baseline keyed by component plus qualified name; non-template header functions over 10 NLOC (report, ratchet) | CPLX-1 to 5, CPLX-10, HDR-5, CFG-6 |
| `check_docs.py` | links and anchors; PIPELINE.md required headings, order, length and named files/symbols; numbers name their constant; README title, length and `Status:` values; `sh` blocks vs CI steps | PIPE-1 to 3, PIPE-10, DOC-1, DOC-3, DOC-9, DOC-11 |
| `check_headers.py` | file header length; line-1 file name or path; generated-file first line; thread-tag grammar; exception-tag grammar and count | CMT-5, CMT-7, CMT-11, HDR-8, CON-2 |
| `check_names.py` | leftover names from renames; -GATR spelling in prose | NAM-13, DOC-10, DOC-12 |
| `check_build.py` | `.mk` vs CMake source lists; other components' `.cpp` in CMake; directory-scoped CMake commands; `file(GLOB` | CMAKE-1, 2, 4, 5 |
| `check_js.py` | option B JS lint; `x \|\| <number>` defaults | JS-8, JS-9, JS-11, JS-13 |
| `check_tracked.py` | logs, absolute paths, build output tracked | REPO-4 |
| `rule_index.py` | prints one line per rule heading of the standard (the index is generated, never hand-kept) | 0.2 |

### 9.9 What the rebuilt CI gates on

Phase 1 of section 11 first runs today's builds and tests with today's flags (the safety net);
the gates below turn on in phase 3, each from its baseline (0.4).

| Job | Runs | Blocking |
|---|---|---|
| format | `clang-format --dry-run --Werror` over first-party C++ and JS | yes, after the phase 3 format pass |
| build-host-linux | every CMake project, gcc, C++17 extensions off; the always-a-bug list, the 9.1 warning set and `-Werror` (`GATR2_WARNINGS_AS_ERRORS=ON`): any warning fails (CMAKE-13) | yes |
| build-host-mingw | the same on Windows MinGW (the dev box) | yes |
| build-pico | `pio run` for the two `_ci` envs from `pico/aggreGATR`: Pico flags plus `-Werror` | yes |
| build-pros | both apps with the PROS toolchain, Brain flags and `GATR2_WERROR=1`; if the full PROS build cannot run in CI, an arm-none-eabi syntax-only compile of the Brain libraries at `gnu++20` with the same flags; the RT-5 toolchain probe | yes |
| build-pi-libcamera | naviGATR with `NAVIGATR_WITH_LIBCAMERA=ON` on Linux | yes where a runner has libcamera; otherwise listed under `## Exceptions` with its manual recipe (SYS-26) |
| tests | `ctest` for every project, every test with a timeout; threaded suites again with `--repeat until-fail:3`; each suite once with `--gtest_shuffle`; every runnable config loads; `field_references.h` regenerates byte-identical | yes |
| sanitizers | ASan+UBSan on host tests; TSan on naviGATR | yes, after the backlog is clear |
| fuzz | libFuzzer targets for the external decoders, fixed time, Linux clang (TEST-21) | no until the first clean run, then yes |
| coverage | gcov/gcovr line and branch coverage per component (TEST-20) | no, never |
| tidy | 9.2 config on changed files; overlays | yes for gating checks; report for the rest |
| complexity | `check_complexity.py`: lizard warn report; fail tier on new or changed functions (C++, JS, Python); baseline ratchet | fail tier yes |
| advisory-warnings | tier-2 flags; counts per component | no |
| docs | `check_docs.py`, `check_ascii.py`, `check_names.py` | yes |
| repo-checks | the other `tools/ci/` scripts; exception-tag counts printed | yes (counts are report-only) |
| js | ESLint or `check_js.py`; headless viewer tests against the real binary | yes |
| python | `ruff check`, `ruff format --check` | yes |
| shell | `shellcheck` | yes |
| build files | `check_build.py` | yes |

---

## 10. Glossary and bug classes

One meaning per word, across all components. This is the seed for `docs/glossary.md` (DOC-13).
"Today" notes where the code still disagrees.

### 10.1 Terms

| Term | Means | Defined at, or today |
|---|---|---|
| pose | position and heading only: (x, y, heading). No z (flat field), no velocity, no validity. | `investigatr::Pose`, `navigatr::Pose2D` (`pi/naviGATR/src/math/transforms.h`) |
| transform `T_a_b` | the pose of frame b expressed in frame a; `compose(T_a_b, T_b_c)` is `T_a_c` | `pi/naviGATR/src/math/transforms.h:2-4` |
| frame | a named coordinate system: field, odom, robot, camera, tag | `FrameId`, `pi/naviGATR/src/core/ids.h:49-50` "Not a wire packet." |
| field frame | the map's frame; heading CCW from +x | `brain/investiGATR/include/investigatr/geometry.h:2-3` |
| odometry frame (odom) | where localization integrates motion; continuous within one odometry epoch | `pi/naviGATR/src/state/robot_state.h` |
| anchor | `T_field_odom`, set by a placement; identity until the first one | `robot_state.h:46` |
| placement | a field pose given to localization for the robot (the wire op is `SET_POSE`) | today also "init" (`init_sequence`, `init_session`): retire "init" |
| state | what a thing is now. Robot state is pose plus velocity, validity and anchors; also a lifecycle enum (`MotionState`) | `navigatr::RobotState` (Pi estimate) and `investigatr::RobotState` (Brain view) differ; qualify in docs |
| estimate | an output of an estimator: the robot pose (localization) or a field object's pose (world estimation) | |
| measurement, sample | one reading from one sensor with its time | `sensor.encoder_sample`, `sensor.imu_sample` payloads |
| observation | an estimator input derived from measurements (a wheel increment, a heading increment, a tag detection) | `pi/naviGATR/src/contracts/localization.h` |
| evidence | a statement that a configured feature of a configured field object implies this object pose, with frame, measurement time and odometry epoch; the one world-estimation seam type | `FieldObjectPoseEvidence`, `pi/naviGATR/src/payloads/field_object_evidence.h:37` |
| landmark, field object | a mapped object on the field with a nominal pose (goals, tag mounts) | field map resource |
| nominal pose | where the field map says a landmark is (as built) | `config/seasons/<season>/field.xml` (XML-5) |
| landmark error | `(dx, dy, dtheta)`: displacement in field axes and rotation of the estimated landmark from its nominal pose; (0, 0, 0) means as built | `LandmarkError`, planned in restructure step 3 |
| target | a pose to drive to, defined as anchor plus offset; the anchor is the odometry origin or a landmark | target resolution |
| sensor group | one world-estimation plug-in: the sensors that together yield full-pose evidence (AprilTag alone; ToF plus whatever completes a pose) | `estimators/<group>/` |
| session | the Brain link session opened by HELLO; nothing else | today the Pi process id is also called `session` (`inspection_document.cpp:760`) |
| instance | one lifetime of a process: `pi_instance` is a random nonzero id per Pi process start and reset | `translaGATR/frames.h:378` |
| boot | one power-on of a device; `boot_id` | `translaGATR/frames.h` sensor frame |
| epoch | a counter that bumps when continuity breaks; values from different epochs are never combined (odometry, acquisition, IMU, camera) | today some comments call an epoch a "generation" |
| revision | a counter of edits to one continuous thing (`anchor_revision`) | |
| generation | identity of one published immutable snapshot; changes whenever its content changes (`FrameGeneration`, `Field::generation`) | today also used for counts (profile applies, target activations): rename those |
| sequence, seq | position in an ordered stream within one epoch; may wrap | |
| cycle | one pass of a worker loop | |
| invocation | one call of a stage function | |
| ticket | the caller's token for an async request; 0 = refused | `PlacementTicket` (`brain/communiGATR/include/communigatr/client.h:71`); the Pi calls it `handle` today |
| id | a stable identity; crc32-based for link documents | |
| result | the outcome code of a request (`BrainResult`, `PlacementResult`) | |
| status | a struct reporting a result or state with details (`ControlStatus`) | |
| stage | one step of a pipeline with a contract (resources, sensors, commands, localization, world estimation, targets, publishing) | |
| slot | a stage filled by exactly one registered implementation chosen by the slot element's `type` (commands, targets, publishing) | |
| plug-in | an implementation registered under an XML `type` and built by `make_<type>` | |
| contract | a stage's `contract.h`: its standard input and output | |
| seam | a boundary other teams build against: a stage contract, a Brain interface, the wire | |
| executable, executor | executable: one configured item's runner; executor: the stage-level runner. Today one letter apart (`SensorExecutable` / `SensorExecutor`); rename one side when touched | |
| resource (Pi) | a device or data source the Pi opens (serial link, camera, Pico telemetry, field map) | |
| sensor (Pi) | a channel that turns resource output into measurements | |
| snapshot | an immutable copy published for readers on other threads | `std::shared_ptr<const T>` |
| latest-wins | a handoff that keeps only the newest entry and counts replacements | `LatestSlot`, `pi/naviGATR/src/runtime/handoff.h` |
| robot profile | the robot description the Brain sends to the Pi | `translaGATR/link_documents.h` `RobotProfileDoc` |
| host clock | the Pi's one steady clock, milliseconds or microseconds since process start | `HostClock` |
| device clock | a device's own clock as stamped on the wire: the Pico acquisition clock, the Brain clock | `translaGATR/frames.h:71` |
| clock domain | which clock a time belongs to; values from two domains are never compared raw | `ClockDomain` in `pi/naviGATR/src/core/time.h` |
| degrade | answer with a defined "no data" or safe value instead of waiting | P-7 |

### 10.2 Other meanings of "frame"

"Frame" alone means a coordinate frame. Say **wire frame** for a translaGATR packet, **camera
frame** for an image, **WebSocket message** for a viewer message (today `HttpServer::Frame`).

### 10.3 Accepted abbreviations

`id`, `seq`, `acq` (acquisition, wire name), `imu`, `enc` (encoder, wire name), `diag`, `doc`,
`rx`, `tx`, `dt`, `prev`, `crc`, `cfg` only as the existing Pico namespace, unit suffixes (`_m`,
`_mm`, `_um`, `_rad`, `_deg`, `_cdeg`, `_mdeg`, `_s`, `_ms`, `_us`, `_hz`, `_dps`, `_mdps`, `_px`),
and math notation (`T_a_b`, `R`, `q`, `K`, `dx`, `dy`, `dtheta`, `ux`, `uy`). Everything else is
spelled out.

### 10.4 Bug classes

The classes of bug this project has hit, named in commit messages (REPO-7) and in the TEST-10
table. Evidence for each is in the metrics survey (`survey/complexity-bugs.md`, Part 2 material).

| ID | Bug class | Rules that prevent it |
|---|---|---|
| B1 | clock-domain and time-base mixing | SYS-18, TIME-3, TIME-4, TIME-7, TIME-10 |
| B2 | zero, -1 or null as a sentinel; missing data read as zero | TYPE-16, WIRE-6 |
| B3 | stale cached state; freshness not carried with the data | SYS-17, RT-14 |
| B4 | missing reset on identity change (session, epoch, boot, connection, profile) | SYS-16 |
| B5 | snapshot consistency: data and labels read at different moments | SYS-24 |
| B6 | locks, blocking I/O and task lifecycle | CON-3, CON-4, RT-6 to RT-8, RT-14 |
| B7 | unbounded queues, or FIFO where latest-wins was meant | CON-9 |
| B8 | units, ranges and rounding | UNIT-, NUM- |
| B9 | name tables and labels drifting from enums or meaning | ENUM-3, LOG-5 |
| B10 | retries, idempotency, dedupe and sequence wrap | WIRE-5 |
| B11 | liveness: starvation, deadlines checked on one path only | CON-4, CON-10 |
| B12 | silent data loss, and counters nobody reads | ERR-9 |
| B13 | docs and claims drifting from behavior | PIPE-3, PIPE-10, DOC-9, DOC-10 |
| B14 | timing-dependent and non-failing tests | TEST-6, TEST-12, TEST-19 |
| B15 | platform, ABI and target-only code paths | SYS-26, NUM-5, REPO-9 |
| B16 | buffer bounds on wire data | WIRE-8, RT-15 |

---

## 11. Applying this to the existing system

### 11.1 Ground rules

- **Behavior stays identical** in every step except the ones marked "logic", which get their own
  commit, their own tests and, where section 12 says so, the lead's approval.
- **Order:** a safety net first, then the naviGATR restructure that is already running, then the
  mechanical pass, then structure elsewhere, types and names, docs, complexity, and last the
  system follow-ups.
- **The restructure is pinned to `ae457ee` line numbers** (`restructure/plan/split_plan.md`, and
  `split_system.py` cuts `system.cpp` by line ranges). It ships as one cumulative patch (steps 1
  to 7). Nothing that rewrites naviGATR files lands before that patch is applied.
- **One kind per commit:** format-only, move-only, rename-only, function extraction or logic
  (REPO-7).
- **Gates**, the checks a commit must pass:
  - A. tests: the test name list and every result identical before and after (`ctest -N` or
    `--gtest_list_tests`, plus the run), for every affected project;
  - B. symbol parity: `nm -C --defined-only` of the library, reduced to sorted unique names,
    identical;
  - C. diff shape: `git diff -M --stat` shows renames only; the filtered diff (minus include lines
    and build lists) is empty, or contains only moved blocks;
  - generated outputs identical (`brain/operaGATR/include/field_references.h` byte for byte);
  - the Linux-only branches built on Linux (libcamera, POSIX serial) when their files move.
- **Gates by commit kind:**

  ```text
  move, file split:      A, B, C.
  rename, namespace:     A; B after applying the rename or namespace map to the old symbol list.
  function extraction:   A; the old symbols minus the split functions are a subset of the new
                         set; every new symbol is named in the commit message.
  format-only:           A, B.
  logic:                 A plus new tests.
  ```

- **Brain changes:** both PROS apps build with `pros make` (or the 9.9 syntax check) before and
  after; `brain/gatr2_brain.mk` differs only in moved paths.
- **One component per PR**, except phase 3 mechanical commits, which may touch several.

### 11.2 Phases

**Phase 0: blocking decisions.**
- The lead rules on the section 12.1 decisions (D-4, D-5, D-8); the rest have defaults.
- Record per project the test lists and results that the phase 1 safety net compares against.

**Phase 1: safety net.**
- CI runs today's builds and tests (Linux, MinGW, `pio run`, the PROS build or its syntax
  check) with today's flags. No new gates, no tree changes outside `.github/`.

**Phase 2: the naviGATR restructure, then its first logic fixes** (11.4).
1. Restructure step 1 (move), step 2a (noop split), step 2b (`system.cpp` split), step 3 (world
   estimation representation), per `restructure/plan/FINAL.md`.
2. Step 2c, entry points (behavior-free, needs D-5): 11.4.
3. Baselines, taken now so the moves cannot reset them: the CPLX report keyed by component plus
   qualified function name (not by path), warning counts per flag, clang-format diff counts,
   exception tags (zero), and the include edges of `restructure/plan/FINAL.md` section 5 (F2).
4. The config reader (CFG-3). Logic, not a rename: unknown attributes become errors (CFG-4),
   error texts change and defaults move into structs, so a config that loads today may be
   refused. One parser per commit, `<TrackingWheel>` first with one more parser in the same
   series; gated by XML-8 (every committed config loads) and the existing error-path tests.
5. Logic, own commit and test: a `resumeWorkers` failure becomes a status and a fault event, and
   `main` exits 1 cleanly (ERR-1, `pi/naviGATR/src/runtime/system.cpp:934`).

**Phase 3: mechanical, then the gates.**
1. Repo hygiene: `.gitattributes` plus renormalize, `.editorconfig`, `.gitignore` additions
   (`__pycache__/`, `*.pyc`, `.venv/`, `*.log`), untrack `debug.log` and the absolute-path
   `.vscode/settings.json` files, commit the printables generator next to its output, fill the
   PROS template placeholders (REPO-).
2. `.clang-format` additions (9.3, after D-25), then one format-only commit per component, each
   listed in `.git-blame-ignore-revs`.
3. `InsertBraces` as its own commit per component, after the whitespace reformat: built and
   tested (gate A), each of the 124 inserted braces read as code (clang-format warns the option
   can produce incorrect code).
4. Build glue: `cmake/gatr2.cmake` with the warnings target and the hash-pinned googletest; a
   `translagatr` CMake library that every consumer links; the Pico includes
   `"translaGATR/frames.h"`; the naviGATR e2e test links the Brain targets; the `.mk` drift
   check; the season cache variable; `common_tests` renamed `translagatr_tests`; toolchains
   pinned in `docs/toolchains.md` (CMAKE-, REPO-9).
5. Tier-1 warnings (9.1, after D-1): fix the 18 sites.
6. Mechanical clang-tidy fix-its, one check per commit per component:
   `readability-isolate-declaration` (148), `modernize-make-unique`, `modernize-use-override`,
   `modernize-use-using`.
7. Turn on the rebuilt CI gates (9.9), each from its baseline.

**Phase 4: structure elsewhere and the PIPELINE.md tree.**
1. The PIPELINE.md tree, the 16 files of PIPE-1: root (Appendix B), naviGATR (Appendix C), its
   seven stages (Appendix A has a stage example; Appendix D is world estimation), the apriltag
   sensor group, `brain_profile/`, aggreGATR, spectaGATR, `brain/` (the chain from link to
   motors), communiGATR, actuGATR. READMEs cut to signposts (DOC-1); `translaGATR/README.md`
   added (spectaGATR builds inside naviGATR, so its PIPELINE.md is its only doc, DOC-1).
2. aggreGATR: `src/` split into `board/`, `imu/`, `encoders/`, `link/` with an `aggregatr` root
   namespace (move plus namespace-only edits; host tests identical; `build_src_filter = +<*>`
   already recurses).
3. communiGATR: application helpers to `include/communigatr/app/`; PROS-only headers under
   `pros/` (STRUCT-9). Header moves only.
4. naviGATR config layout (XML-5) after D-23: paths are data; the service file and docs follow.

**Phase 5: types, aliases and names, one component at a time, seam first.**
1. translaGATR: wire aliases (`SessionId`, `RequestId`, `PiInstanceId`, `OdometryEpoch`,
   `BootId`, `DocumentId`, `SensorMask`); enum types in wire-code fields (D-15); one name function
   per wire enum, deleting the Pi and Brain copies (ENUM-3, WIRE-2); the wire units module
   (UNIT-3); the byte cursor (WIRE-4); `static_assert` for sizes (ERR-7); finite checks at encode
   (NUM-1); seeded round-trip and garbage-input tests (TEST-11). Logic: the actuGATR -18000
   heading fix moves to the shared helper in its own commit with an edge test.
2. aggreGATR: `k` enumerators, `MicrosU32` plus one elapsed helper (TIME-6), one declarator per
   line, the flat include spelling gone.
3. Brain libraries: one `Seconds`; callback aliases (`NonceSource`, `BenchImuSource`);
   `[[nodiscard]]` status types; the 9 enum switches without `default`; one RAII guard in
   `ProsDrive`; `make_unique`; `final` leaves; deleted copies on reference-holding classes.
4. naviGATR: the aliases (`DetectionFrameMap`, `WarningList`, `HostMicros`, clock aliases,
   `std::hash` for `TypedId`); `long` durations to `int64_t`; one host clock (`steadyNowUs` gone);
   `MonotonicTime` equality and the unset-domain hole (TIME-4); `HostClock` as a namespace; named
   structs for pairs and output parameters (`ResetMark`, `EulerAngles`, `Point3`, `Pixel`,
   `ProfileVerdict`, `ControlReply`); enum types for wire codes; typed absence for the in-band
   sentinels (TYPE-16); snake_case time fields; `make_*` as the one factory spelling and
   `register_<stage>` for every stage (NAM-2).
   Logic, own commit: one wrap helper that cannot hang on a non-finite angle, with an edge test
   (NUM-2).
5. Brain apps: shared helpers into communiGATR (readiness predicates, recalibration service,
   link snapshot). Logic, separate commits: guard `g_message` and `g_speed_scale`; move the link
   calls out of `opcontrol()` into the program task (RT-5, RT-7).
6. spectaGATR: `format.js`, one angle wrap, `dom.js` helpers only, `bundle.js` renamed, module
   state hoisted, header narrative to `spectaGATR/PIPELINE.md`. Logic, separate commits with
   tests: `??` for defaults (a real 0 stops becoming a default).

**Phase 6: comments and docs.** Header trims, starting with `core/system.h` (83 lines),
`brain_profile_builder.h`, `tracking_wheel_motion.h`, `brain_link_commands.h`,
`weighted_planar_fusion.h`; line-1 labels per CMT-11; `docs/validation.md`; `docs/glossary.md`;
the first decision records (section 12); component docs moved under their components (D-11); the
survey's doc fixes (a stale link in `pcb/README.md`, the broken anchor in
`brain/locaGATR/README.md`, the pre-rename titles). CMT-5 becomes blocking at the end of this
phase.

**Phase 7: complexity** (extent per D-30). Split the fail-tier functions, each covered by existing
tests before and after, largest first: `TrackingWheelMotion::run` and `::create`,
`writeSnapshotBody`, `make_synthetic_rig`, `TagMountAssociation::run`, `parseFieldMap`,
`Client::choose` (policy plus request builders), `ConfiguredTargetResolution::run`,
`PicoTelemetry::refresh`, `Motion::update`, the two Brain app loops, and the viewer's
`renderStatus`.

**Phase 8: system follow-ups** (logic; each needs the lead's approval; 11.5).

### 11.3 Per-component effort

Relative sizes, from the surveys' counts. S: a few mechanical commits. M: a week-sized series.
L: several series.

| Component | Mechanical (3) | Structure (2, 4) | Types and names (5) | Complexity (7) | Size | Notes |
|---|---|---|---|---|---|---|
| translaGATR | format 8 of 8 files; CMake target | README, PIPELINE line in root | aliases, enums, name tables, units, cursor, property tests | `validateRobotProfile` CCN 52 | M | the seam: do it first, review both sides |
| pico/aggreGATR | format; flags; include spelling | folders and `aggregatr` namespace; PIPELINE.md; README trim (211 lines) | enumerators, time helper | none over fail tier | S | cleanest component today |
| brain libraries | format (102 of 135 files) | PIPELINE.md for `brain/`, communiGATR, actuGATR; `app/` helpers | aliases, nodiscard, switches, RAII, make_unique | `Client::choose`, `Motion::update`, planner | M | `Client` has 73 private data members |
| brain apps | format | the chain named in `brain/PIPELINE.md` | dedupe 8 helpers; task and guard fixes (logic) | `run` CCN 45, `opcontrol` CCN 32 | M | |
| pi/naviGATR src | format (235 of 249 files) | the restructure (189 of 201 files move), step 2c, the config reader | aliases, clocks, names, sentinels | about 40 functions over CCN 25 | L | the gripe; largest tail |
| pi/naviGATR tests | format | include paths only | `sim/` support (TEST-4), no sleeps, timeouts, split big files | | M | 45 `sleep_for` |
| spectaGATR | JS format section | PIPELINE.md | helper dedupe, names, `??` | `renderStatus` CCN 64, `field_scene.js` 1582 lines | M | |
| tools, config, bench | ruff, shellcheck, hygiene | config README and layout | requirements files, no auto-install | | S | |

### 11.4 The naviGATR restructure

The lead's gripe: "its not clear here the main pipeline is or where the make pipeline shit is".
Plan: `restructure/SPEC.md` (approved layout, behavior rules, world estimation) and
`restructure/plan/FINAL.md` (the executable plan; its lead correction of 2026-09-28 15:05 on
plug-in folder names overrides the rest). Summary:
- `restructure/plan/move_map_final.tsv`: every file's destination (201 rows, relative to
  `pi/naviGATR/`). The table below summarizes it; the TSV wins.
- `restructure/scripts/split_system.py`: the `system.cpp` split as a verified pure move.
- The move map's own decisions are `move_map.md` D1 to D16 (not section 12); FINAL.md section 3
  says which were adopted.

**Where things are after the restructure** (and step 2c):
- the main pipeline, one cycle: `src/core/run.cpp`, `System::step`, `System::estimationCycle`,
  `System::fieldCycle`, `System::reportingCycle`, the two worker loops, and after step 2c the run
  loops `runInline` and `runThreaded`;
- the make pipeline, construction: `src/core/build.cpp`, `System::build`, which calls
  `make_resources`, `make_sensors`, `make_commands`, `make_localization`,
  `make_world_estimation`, `make_targets`, `make_publishing`, in that order (before step 2c,
  commands, targets and publishing are built by the generic `buildSlot`); each `make_<stage>`
  sits in `src/pipeline/<stage>/stage.cpp`;
- plug-in registration: `src/pipeline/register_all.cpp` `registerAll`, then one
  `register_<stage>.cpp` per stage (STRUCT-1);
- a newcomer reads, in order: `pi/naviGATR/PIPELINE.md`, `src/main.cpp`, `src/core/build.cpp`,
  `src/core/run.cpp`, then any `src/pipeline/<stage>/PIPELINE.md`.

**Target tree** (`pi/naviGATR/src/`, from SPEC and `restructure/plan/FINAL.md` section 2):

```text
main.cpp                 tiny entry: parse args, build, run
cli/                     command-line parsing (was app/)
core/                    the engine only: builds the pipeline and runs it
  system.h               facade: build, start, stop, reset, snapshots
  build.cpp              System::build, calling each stage's make_* in order
  run.cpp                step, the three cycles, the two workers (latest-wins handoff)
  handoff.h
pipeline/                the stages, run order in PIPELINE.md, each the same shape
  register_all.*         every stage's register_*, in run order
  framework/             registry, typed payloads, payload descriptors, records, ids,
                         function status, execution context, slot build context
  resources/             contract.h, stage.*, store, catalog, resource kinds,
                         register_resources.cpp
    cameras/  tag_detectors/  pico/  links/  geometry/  synthetic/     <Resource> families
  sensors/               contract.h, stage.*, catalog, sensor payloads
    channels/            <Sensor> plug-ins + register_sensors.cpp
  commands/              contract.h, command_state.h
    collectors/          brain_link, noop + register_commands.cpp
  localization/          contract.h, stage.*, robot state, feed, pose history,
                         register_localization.cpp
    observation_models/  <Observation> plug-ins
    estimators/          <Estimator> plug-ins, noop
  world_estimation/      contract.h, stage.*, perception.h, association.h, evidence.h,
                         field_state.h, landmark_estimator.*
    estimators/          register_world_estimation.cpp
      apriltag/          the AprilTag sensor group (extraction, association, tag types)
      noop/
  targets/               contract.h, target_state.h
    resolvers/           configured_targets, noop + register_target_resolution.cpp
  publishing/            contract.h
    publishers/          brain_link_publisher, field_documents, noop + register_publishers.cpp
brain_profile/           Brain-driven reconfiguration, swap, Pico control, sensor loss, recovery
config/                  XML parsing (unchanged)
io/                      serial ports and byte streams (was transport/)
time/                    host clock, clock sync, time types (from old core/)
math/                    unchanged
debug/                   inspection/, capture/, diagnostics/ (read-only observers)
```

`pi/naviGATR/tools/field_refs/field_refs.cpp` is the generator that used to sit in `src/app/`.

**Move map summary** (189 of 201 files move, 12 stay: `config/` 7, `math/` 5):

| From (src/) | To (src/) | Files |
|---|---|---|
| `app/main.cpp` | `main.cpp` | 1 |
| `app/command_line.*`, `app/build_config.h.in` | `cli/` | 3 |
| `app/field_refs.cpp` | `../tools/field_refs/` | 1 |
| `runtime/system.*`, `runtime/handoff.h` | `core/` (system.cpp split in step 2b) | 3 |
| `runtime/register_all.*` | `pipeline/` | 2 |
| `core/{ids,records,typed_payload,payload_descriptor,function_registry,function_status,execution_context,diagnostics}.h`, `runtime/{build_options,record_bookkeeping}.h`, `contracts/slot_init.h` | `pipeline/framework/` | 11 |
| `contracts/resource.h`, `runtime/resource_stage.*`, `runtime/resource_catalog.h`, `runtime/pico_control_ref.h`, `resources/*` kinds and store (not `serial_link.h`, `spi_bus.h`), `payloads/pico_telemetry_samples.h`, `impl/resources/register_resources.cpp` | `pipeline/resources/` | 17 |
| `impl/resources/{cameras,camera_capture,libcamera_camera}.*` | `pipeline/resources/cameras/` | 5 |
| `impl/resources/{tag_detectors,apriltag_c}.*` | `pipeline/resources/tag_detectors/` | 4 |
| `impl/resources/pico_telemetry.*` | `pipeline/resources/pico/` | 2 |
| `impl/resources/{pros_usb_link,serial_links}.*` | `pipeline/resources/links/` | 4 |
| `impl/resources/{field_map_resource,robot_frame_map,target_set_resource,wheel_geometry_resource}.*` | `pipeline/resources/geometry/` | 8 |
| `impl/resources/synthetic_rig.*` | `pipeline/resources/synthetic/` | 2 |
| `contracts/sensor.h`, `runtime/sensor_stage.*`, `runtime/sensor_catalog.h`, `payloads/{sensor_samples,attitude_samples,camera_frames}.h` | `pipeline/sensors/` | 7 |
| `impl/sensors/*` | `pipeline/sensors/channels/` | 7 |
| `contracts/commands.h`, `state/command_state.h` | `pipeline/commands/` | 2 |
| `impl/commands/*` | `pipeline/commands/collectors/` | 3 |
| `contracts/localization.h`, `runtime/localization_stage.*`, `state/{robot_state,robot_state_feed,pose_history,attitude,stillness}.*`, `payloads/robot_observations.h`, `impl/localization/register_localization.cpp` | `pipeline/localization/` | 12 |
| `impl/localization/{planar_motion_integrator,weighted_planar_fusion,motion_step}.*` | `pipeline/localization/estimators/` | 6 |
| `impl/localization/{tracking_wheel_motion,imu_heading_increment,attitude_reference,brain_imu_wheel_bench,stationary_window}.*` | `pipeline/localization/observation_models/` | 10 |
| `contracts/{field_estimation,perception,association}.h`, `runtime/world_estimation_stage.*`, `payloads/field_object_evidence.h` (as `evidence.h`), `state/field_state.h`, `impl/world_estimation/landmark_estimator.*` | `pipeline/world_estimation/` | 9 |
| `impl/world_estimation/register_world_estimation.cpp` | `pipeline/world_estimation/estimators/` | 1 |
| `impl/world_estimation/{apriltag_*,tag_mount_association}.*`, `payloads/tag_observations.h` | `pipeline/world_estimation/estimators/apriltag/` | 7 |
| `impl/noop/noops.*` | `pipeline/world_estimation/estimators/noop/` in step 1; split into one `noop.*` per stage plug-in folder in step 2a | 2 |
| `contracts/target_resolution.h`, `state/target_state.h` | `pipeline/targets/` | 2 |
| `impl/target_resolution/*` | `pipeline/targets/resolvers/` | 3 |
| `contracts/publishing.h` | `pipeline/publishing/` | 1 |
| `impl/publishing/*` | `pipeline/publishing/publishers/` | 5 |
| `contracts/brain_profile.h`, `runtime/{brain_profile_builder,sensor_loss,stationary_precheck}.*` | `brain_profile/` | 7 |
| `transport/*`, `resources/{serial_link,spi_bus}.h` | `io/` | 9 |
| `core/{time,host_clock,clock_sync}.h` | `time/` | 3 |
| `inspection/*`, `runtime/{inspection_config,inspection_state}.h` | `debug/inspection/` | 13 |
| `capture/*` | `debug/capture/` | 10 |
| `diagnostics/*` | `debug/diagnostics/` | 7 |

Contract headers become `contract.h` and stage builders `stage.h/.cpp` in their stage folder.
The references that change: 744 include lines in 206 files, the CMake source lists, the doc
links listed in `move_map.md` (with FINAL.md C11), and the generated `viewer_assets.cpp` include
written by `cmake/embed_assets.cmake`.

**Step 2b, the `system.cpp` split** (bodies copied verbatim; output sizes from a dry run of
`split_system.py` on this tree):

| New file | Functions | Lines |
|---|---|---|
| `core/build.cpp` | ctor, `buildFromFile`, `buildFromString`, dtor, `build` (+ its private helpers) | 300 |
| `core/run.cpp` | `step`, `estimationCycle`, `requestsFrom`, `reportingCycle`, `fieldCycle`, `resetStages`, `reset`, `start`, `startWorkers`, `requestStop`, `stop`, `estimationWorker`, `fieldWorker`, `pauseWorkers`, `resumeWorkers`, snapshot readers, `publishBindingView`, `bindingView` | 370 |
| `brain_profile/profile_host.h/.cpp` | `ProfileHost`, `buildProfileHost`, `buildWaitingLocalization`, `prepareProfile`, `applyPendingProfile`, `swapProfile`, `swapToWaiting`, `profileBinding` | 41 + 229 |
| `brain_profile/control.cpp` | `controlProfile`, `readWheels` | 141 |
| `brain_profile/pico_operation.h/.cpp` | `PicoOperation`, `submitPico`, `controlProgress`, `updatePicoOperation` | 23 + 95 |
| `brain_profile/recovery.cpp` | `watchSensors`, `watchModels`, `losePlacement`, `noteRecovery` | 137 |
| `debug/diagnostics/events.h/.cpp` | `RuntimeEvent`, `noteEvent`, `noteFault`, `events` | 19 + 51 |
| `debug/inspection/inspection_state.cpp` | `sourceHealth`, `detectionFrames`, `publishSourceHealth`, `publishDetectionFrames` | 184 |

No file mixes construction, the run loop, profile swapping and recovery, and every new `.cpp` is
under the CPLX-5 warn size. `core/system.h` stays about 489 lines because `System` keeps its
members (FINAL.md O4); F1 shrinks it.

**Step 2c, entry points** (behavior-free; proposed, needs D-5; FINAL.md O4 and O5 note that steps
1 to 3 leave both entry points as they are):
- F4: `make_commands`, `make_targets` and `make_publishing` in `pipeline/<stage>/stage.cpp`, each
  reading its own XML; `Tag` and `buildSlot` deleted; `System::build` reads as the seven
  builders in run order, under 40 lines (today 171 NLOC, CCN 31).
- F11: `main` keeps parse, build, run, report, under 30 lines (today 112 NLOC, CCN 25). The inline
  loop and the profile-boundary loop move to `core/run.cpp` as `runInline` and `runThreaded`;
  `printDiagnostics` and `printWorker` move to `cli/report.cpp`; inspection startup is one
  function. The `"brain_link"` compare at `pi/naviGATR/src/app/main.cpp:118` stays until F3.
- Gates: function extraction (11.1) and C.

**Step 3, world estimation** (representation only, numerically identical, per SPEC):
- `TagAssociationTrace`, `TagAssociationTraceSet`, `TagAssociationCandidate` and their payload
  name leave the generic `evidence.h` for `estimators/apriltag/tag_association_trace.h`
  (STRUCT-3);
- each field object exposes `LandmarkError { dx, dy, dtheta }` from its nominal pose, in
  `field_state.h`; the field pose equals nominal plus error; one helper, `applyLandmarkError`,
  applies a landmark's error to a target defined relative to it, and target resolution uses it;
- the Brain wire is unchanged (D-26);
- new tests: (0, 0, 0) is the identity; the helper equals the old pose composition over random
  cases including wrap near +-pi; the error round-trips with the field pose.

**Gates:** 626 naviGATR tests at `ae457ee`, 634 after step 3 (8 new); the same names pass after
each step (11.1 gates A to C); `field_references.h` byte-identical; the Pi libcamera build for the
moved camera files.

**Restructure decisions that touch this standard:**
- plug-in folders named with readable plural nouns, resources grouped by family, no stutter (the
  lead's correction of 15:05; STRUCT-2; D-28 resolved);
- one `register_<stage>.cpp` per stage, at stage level for localization and resources (STRUCT-1);
- flat plug-in folders except sensor groups and resource families;
- src-relative line-1 labels for repeated basenames and the 7 payload headers (move map D16;
  CMT-11 keeps the path for repeated basenames only);
- namespaces unchanged, `navigatr` (D-6);
- `perception.h` and `association.h` stay generic at stage level (D-29).

### 11.5 System follow-ups (logic unless marked, each approved separately)

| ID | Change | Rules it satisfies |
|---|---|---|
| F1 | Extract a `ProfileSupervisor` class in `brain_profile/` that owns the Brain-profile state `System` carries (about 20 of its 70 members); the `if (profiled_)` hooks in the cycle become one call | SYS-20, STRUCT-6, CPLX-6 |
| F2 | Fix dependency direction: replay through the registry or build options instead of including `serial_links.h`; `PicoControl` from the store by its contract; field documents and family geometry as functions next to their kinds; the math, config and framework edges (`math/transforms.h`, `config/field_map.h` -> `pipeline/framework/ids.h`; `math/camera_model.h` -> `pipeline/resources/camera.h`; `pipeline/framework/slot_init.h` -> `pipeline/localization/contract.h`); the debug to plug-in edges | SYS-19, STRUCT-8 |
| F3 | Take Brain-link wire types out of generic contracts (`CommandState`, `PublishingOutput`, the slot build context) and the `"brain_link"` string compares out of the core and `main` | SYS-5 |
| F4 | Named `make_commands`, `make_targets`, `make_publishing` builders in their stage folders, so every stage has a `make_*` (behavior-free; step 2c) | TYPE-15, STRUCT-1, STRUCT-4, NAM-2 |
| F5 | One name per stage: `FieldEstimation` becomes `WorldEstimation`; pick `Commands` or `CommandCollection` | NAM-3 |
| F6 | One time type per process on the Pi: one host clock, `MonotonicTime` equality with domain, durations as `int64_t` | TIME- |
| F7 | One event call with a typed kind replacing the paired `noteEvent`/`noteFault` calls | LOG-2, SYS-9 |
| F8 | Delete unused generality: `SpiBus`/`SpiDevice`, the `snapshot_hz` knob and the knobs no config sets, the `continuous` vision policy parsed only to be rejected | P-3, SYS-13, SYS-14 |
| F9 | Brain link request and reply in one module, per-op reply data as a variant | SYS-21 |
| F10 | Functional seams for the virtual stage contracts, one approved change per seam, never as a side effect of feature work; a diagnostic on the commands, targets and publishing outputs | TYPE-5, SYS-3, SYS-4 |
| F11 | `main` as parse, build, run, report; `runInline` and `runThreaded` in `core/run.cpp`; printing in `cli/report.cpp` (behavior-free; step 2c) | STRUCT-4, CPLX-1, CPLX-2 |
| F12 | Move `perception.h` and `association.h` into `estimators/apriltag/`, or collapse them into concrete classes, if D-29 says so | SYS-14, STRUCT-3 |

### 11.6 Deadlines for deferred baselines

Every check starts from a baseline (0.4). This table lists when each deferred baseline must reach
zero.

| Rule | Reaches zero by |
|---|---|
| STRUCT-1 missing `stage.cpp`; STRUCT-4 and CPLX for `main` and `System::build` | step 2c (phase 2) |
| ERR-1 the `system.cpp:934` throw | phase 2 (logic fix) |
| CFG-3 declarative config reader | phase 2 (new parsers use it as soon as it exists) |
| TIME-5 `long` durations | phase 2 with the config reader, phase 5 for the rest |
| FMT-1 unformatted files; FMT-2 missing braces | phase 3 |
| STRUCT-9 PROS headers in the portable include folder; NS-2 `aggregatr`; NS-4 `using namespace translagatr;` in `pico/aggreGATR/src/main.cpp:16`; STRUCT-10 Pico `src/` | phase 4 |
| TIME-1 clock reads below the entry point (56 lines in naviGATR, some allowed self-timing) | phase 5 (F6) |
| NAM-6 snake_case time fields, NAM-2 one factory spelling | phase 5 (naviGATR) |
| ERR-3 `[[nodiscard]]` on existing APIs | phase 5, per component |
| ENUM-2 the 9 Brain enum switches with `default` | phase 5 (Brain) |
| TYPE-16 in-band sentinels; ALIAS-15 `auto`; NUM-2 wrap helper; CFG-1 `tinyxml2` outside `config/` | phase 5 |
| CMT-5 file header length | end of phase 6 |
| CPLX-1 to 5 fail tier on existing functions | phase 7 (extent per D-30) |
| SYS-20 mode flags in the cycle; STRUCT-6 and CPLX-6 for `System` | F1 |
| SYS-19, STRUCT-8 the edges in `restructure/plan/FINAL.md` section 5 | F2 |
| SYS-5 wire types in generic contracts | F3 |
| LOG-2 one event call | F7 |
| SYS-3 context passing, SYS-4 diagnostics, TYPE-5 functional seams | F10 |

---

## 12. Decisions for the lead

Genuine choices only: places where the lead's earlier rules conflict, where the surveys disagree,
or where the cost is large. Rows are ordered by what they block. Each row ends with its default if
there is no answer, so "defaults, except D-n" is a complete reply. `(lead rule)` marks a decision
that would change one of the lead's own rules; its default keeps the rule. Once decided, each row
becomes a decision record (DOC-7).

Decided by the lead on 2026-09-28: D-1, D-4, D-5, D-6, D-7, D-25 (rows marked "Decided"). Every other row: its recommendation stands as the default unless the lead objects.

### 12.1 Now: before restructure step 2c

| ID | Question | Options | Recommendation | Default if no answer |
|---|---|---|---|---|
| D-4 | (lead rule) Non-C++ files inside `pi/naviGATR/`. The rule: "C++ ONLY ... literally zero other languages", CMake and XML accepted. Today: `spectaGATR/` (17 JS, 1 HTML, 1 CSS, embedded in the binary); `tools/calibrate_camera.py`, `tools/test_calibrate_camera.py`, `tools/viewer/{capture_fixture,cdp,stub_server,viewer_tests}.py`; `tools/install_service.sh`, `tools/perf/run_perf.sh`, `tools/viewer_smoke.sh`; `tools/perf/perf_probe.js`, `tools/viewer/unit_tests.js`, `tools/viewer/unit.html`; `tools/perf/testdata/*.json`; `src/impl/resources/apriltag_c.c` (C shim over the vendored AprilTag C library). Proposed by this standard: `CMakePresets.json` (CMAKE-10), `.clang-tidy` files, an ESLint config (D-21). | (a) strict: move everything non-C++ out of `pi/naviGATR/`; (b) named exceptions, each with a one-line reason under `## Exceptions` in `pi/naviGATR/PIPELINE.md`; (c) move the tools to a repo-level `tools/<tool>/`; spectaGATR, `apriltag_c.c` and the build-glue files are the named exceptions | Decided 2026-09-28 by the lead: yes. Markdown docs (PIPELINE.md, README.md) are allowed inside `pi/naviGATR/`; other non-C++ files follow (c) | n/a (decided) |
| D-5 | How far the naviGATR restructure goes now. | (a) the approved SPEC: steps 1, 2a, 2b, 3, then stop; (b) (a) plus step 2c (F4 and F11: a `make_*` for every stage, `main` split; behavior-free); (c) also F1 to F12 now | Decided 2026-09-28 by the lead: (b). Every stage and sub-step gets a `make_*` that returns a verb-named callable (TYPE-17, NAM-2); `main` becomes parse, build, run, report (F11). Scheduled as restructure step 5 | n/a (decided) |
| D-8 | Factory naming. | (a) `make_<type>`, or `make_<type>_<stage noun>` when the type name repeats across stages (`noop` in five), and `register_<stage>`, as a named snake_case exception; (b) camelCase `makeX` everywhere | (a) (NAM-2): `grep make_<type>` goes from XML to code, and the stage suffix keeps overloads out of `registerOrDie`. `X::create` and `makeNoopX` convert in phase 5. | (a) for new factories; existing names stay until phase 5 |

### 12.2 Before phase 3 (the mechanical pass and the CI gates)

| ID | Question | Options | Recommendation | Default if no answer |
|---|---|---|---|---|
| D-1 | (lead rule) Which warning flags, and are they errors? The lead's rule of 2026-07-19: `-Wall -Wextra -Werror` everywhere; on 2026-09-28: `-Wall -Wextra` without `-Werror`. | (a) `-Wall -Wextra`, no `-Werror`; (b) the 9.1 tier-1 set, `-Werror` in CI only through `GATR2_WARNINGS_AS_ERRORS` (no CMake bump); (c) tier-1 with `-Werror` everywhere | Decided 2026-09-28 by the lead: selective. The always-a-bug list in 9.1 is `-Werror=<name>` everywhere; every other warning stays a warning locally and CI rejects new ones | n/a (decided) |
| D-25 | (lead rule) clang-format knobs beyond the lead's `.clang-format`. Cost: 369 of 437 C++ files differ under today's config (356 with the knobs); the JS section changes 776 of 7884 spectaGATR lines. FINAL.md O6 notes the config's own style differs from the tree. | (a) `InsertBraces`, `AccessModifierOffset: -4`, `AllowShortCaseLabelsOnASingleLine`, `IncludeBlocks: Regroup`, and the JavaScript section (9.3); (b) the C++ knobs only; (c) none | Decided 2026-09-28 by the lead: (a). One format-only pass per component after the restructure and spectuGATR Phase A patches land | n/a (decided) |
| D-19 | (lead rule) Shared test support, against "a `tests/` folder holds only test mains, no headers, no shared helpers". | (a) `<component>/sim/`, a sibling of `tests/`, listed explicitly (the Brain already works this way); (b) `tests/support/` | (a) (TEST-4): keeps the rule and ends the copy-pasted fakes | (a) |
| D-9 | translaGATR include spelling. | (a) `"translaGATR/frames.h"` everywhere, the Pico included, `-I` at the repo root; (b) keep each platform's spelling | (a) (HDR-3) | (a) |
| D-13 | Adopt clang in CI? | (a) clang-tidy on Linux CI; thread-safety annotations only if a lock bug recurs; (b) also `-Wthread-safety` now; (c) no clang | (a) | (a) |
| D-21 | JS lint. | (a) ESLint in CI only (node in CI; its config is a D-4 file); (b) a stdlib Python checker | (a) if node in CI is acceptable, else (b); nothing needs node locally | (b) |
| D-22 | Viewer assets: one `file(GLOB)` or an explicit list? | (a) explicit list; (b) glob | (a), like sources and tests (CMAKE-5) | (a) |

### 12.3 Later

| ID | Question | Options | Recommendation | Default if no answer |
|---|---|---|---|---|
| D-2 | Exceptions on the Pi and Brain? | (a) none thrown by project code, third-party throws wrapped at the call; (b) allowed on the Pi | (a): one error style (ERR-1, ERR-2); the Pico and codec cannot use them anyway | (a) |
| D-3 | Strong typedefs or plain aliases? | (a) plain aliases by default, strong only after a real mix-up; (b) strong ids and units everywhere | (a) (ALIAS-13); keep `TypedId` and `MonotonicTime` | (a) |
| D-6 | Should naviGATR namespaces mirror its folders? Applied by the restructure: SPEC keeps one `navigatr`. | (a) one flat `navigatr`; (b) nested per stage (`navigatr::localization`); (c) shared helpers nested by role (`config::`, `json::`), stage code flat | Decided 2026-09-28 by the lead: namespaces mirror folders up to a sensible depth (NS-2 mapping): one per area and pipeline stage, one more for a self-contained family folder, none for grouping folders, never one big namespace. Scheduled as restructure step 6 | n/a (decided) |
| D-7 | How strict is "struct for attribute only things"? | (a) strict: no member functions; (b) lenient: operators, `Hash` and one-line const queries | Decided 2026-09-28 by the lead: not strict, highly recommended. TYPE-1 is SHOULD | n/a (decided) |
| D-10 | Line 1 comment. Applied by the restructure for repeated basenames (move map D16). | (a) `// <file name>` (426 of 454 files today), the component-relative path only for repeated basenames; (b) the path on every file | (a) (CMT-11): only restructured files change; the 7 payload headers that got a path switch to the file name in phase 6 | (a) |
| D-11 | Where component docs live. | (a) `<component>/docs/`, root `docs/` for cross-component material; (b) all in root `docs/` | (a) (DOC-14) | (a) |
| D-12 | Stage seam form. | (a) new stages functional (struct of `std::function`); an existing virtual seam is converted only by its own approved change (F10); (b) convert all now | (a) (TYPE-5) | (a) |
| D-14 | Unit style. | (a) Brain SI aliases (`Meters`), Pi and wire suffixes (`_m`); (b) one style everywhere | (a) (ALIAS-5): converting 756 Pi `_m` names buys nothing; a unit only in a comment is banned everywhere | (a) |
| D-15 | Enum types in wire structs, or `uint8_t` aliases? | (a) the enum type everywhere, translaGATR structs included; (b) `uint8_t` aliases | (a): layout unchanged, all byte values stay representable (TYPE-13) | (a) |
| D-16 | Boolean query prefix. | (a) bare adjectives (`ready()`, `open()`); (b) `is*` | (a); existing `is*` names stay until a rename pass | (a) |
| D-17 | Pi durations. | (a) `int64_t` with a unit suffix (or a `Millis` alias) plus `MonotonicTime`, `std::chrono` only at sleep and wait edges; (b) `std::chrono` throughout | (a) (TIME-5, TIME-9) | (a) |
| D-18 | Recursion on the Pi (XML include expansion). | (a) allowed with a stated depth bound; none on Brain, Pico, codec; (b) none anywhere | (a) (RT-9) | (a) |
| D-23 | Config layout: season data vs robot data. | (a) `config/seasons/<season>/` for the field only, robot and device profiles outside; (b) keep `config/override/` | (a) (XML-5); the Pi service file and docs change with it | (b) |
| D-24 | Config template suffix. | (a) keep `.xml.in` (6 tracked templates, e.g. `pi/naviGATR/config/shared/robots/gatr2_as5047_imu.xml.in`; the loader already refuses it at `pi/naviGATR/src/config/composition.cpp:198`; nothing runs `configure_file` on them); (b) `.template.xml` | (a) | (a) |
| D-26 | Brain wire field documents: full poses or the landmark error form? | (a) keep full poses (nominal plus error); (b) switch to `(dx, dy, dtheta)` | (a) until a Brain consumer needs the error form; it is a wire version change (WIRE-7). FINAL.md O3. | (a) |
| D-27 | inspect/1 compatibility in the viewer. | (a) drop it; (b) keep it | (a): the page ships inside the runtime binary it talks to | (a) |
| D-29 | `perception.h` and `association.h`: SPEC keeps them as generic step contracts, but each has one implementation and the lead's sensor-group model keeps association inside the group. | (a) keep them generic at stage level; (b) move them into `estimators/apriltag/` or collapse them into concrete classes (F12) | (b), after the restructure (SYS-14) | (a) |
| D-30 | How far phase 7 (complexity) goes. 60 functions are over the CCN fail limit. | (a) all 60; (b) naviGATR entry points and stage run functions first; the rest stay in the baseline with a one-line waiver and are split in their own later series | (b) | (b) |

### 12.4 Retired or resolved

| ID | Outcome |
|---|---|
| D-20 | Retired. `window.__navigatr` stays: it names the runtime the page talks to, like `navigatr.inspect/2`, and a rename touches the viewer and its Python tests for nothing (JS-5). |
| D-28 | Resolved by the lead on 2026-09-28 15:05 (`restructure/plan/FINAL.md`): no stutter; resources grouped by family under `pipeline/resources/`, sensors in `channels/` (STRUCT-2). |

---

## Appendix A: PIPELINE.md template

Angle brackets are replaced. Required sections (PIPE-2): the title, the purpose line, `Status:`,
Start here, Input, Output, Change points, Failure behavior. The others appear only when they have
content, in this order; Inside is required when a child PIPELINE.md exists. Root and component
files: target 60, at most 100 lines. Stage and sub-pipeline files: target 30, at most 50 lines.
Files and symbols are named, never line numbers (PIPE-3); a number names its constant (PIPE-10).

````markdown
# <Name> pipeline

<One or two lines: the problem this pipeline solves (P-1), then what goes in and what comes out.>

Status: <designed | host-tested | bench-tested | robot-validated>, see [validation](<path>/docs/validation.md)

## Start here
- <entry file>: <what it does>
- <construction file>, <function>: <what it builds, in order>
- <per-cycle file>, <function>: <the order each cycle>

## Input
- <what>, from <who>: `<Type>` in `<file>`; <unit, frame, clock if the type does not say>

## Output
- <what>, to <who>: `<Type>` in `<file>`

## Stages
```text
<input> -> <stage> -> <stage> -> <output>
```

| # | XML element | Folder | Built by | Out |
|---|---|---|---|---|
| 1 | <Element> | `<folder>/` | `make_<stage>` | `<Type>` |

## Code map
| `type` | Built by | File |
|---|---|---|
| `<xml type>` | `make_<type>` | `<plug-in folder>/<file>.cpp` |

## Inside
- [<sub-pipeline>](<folder>/PIPELINE.md)

## Rules
- <an invariant, often an absence: "never sends unasked">

## Change points
- <what is season- or robot-specific, where it plugs in, how it is selected; "nothing" is valid>

## Threads and rates
- <thread or task>: <stages>, <rate> (`<constant or attribute>`)

## Failure behavior
- <input missing, stale, or device failing>: <what happens, who sees it>

## Exceptions
- <RULE-ID>: <what it covers>, <reason>
````

### A.2 Stage example: `pi/naviGATR/src/pipeline/sensors/PIPELINE.md` (proposed)

Symbols verified at `ae457ee`; paths are post-restructure. Sources:
`pi/naviGATR/src/runtime/sensor_stage.h:4-5`, `:101`; `pi/naviGATR/src/core/records.h:37-42`,
`:94-102`, `:115`; `pi/naviGATR/src/impl/sensors/register_sensors.cpp:11-17`;
`pi/naviGATR/src/impl/sensors/pico_channels.h:41`, `:44`; `camera_channel.h:21`;
`attitude_channel.h:27`.

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
| `pico_encoder_channel` | `make_pico_encoder_channel` | `channels/pico_channels.cpp` |
| `pico_imu_channel` | `make_pico_imu_channel` | `channels/pico_channels.cpp` |
| `camera_frame` | `make_camera_frame` | `channels/camera_channel.cpp` |
| `attitude_channel` | `make_attitude_channel` | `channels/attitude_channel.cpp` |

## Change points
- a new sensor: a file in `channels/`, one line in `channels/register_sensors.cpp`

## Failure behavior
- a sensor with nothing new: its `MeasurementRecord.state` is `kNoDataYet`, `kUnavailable` or
  `kFault`, with a `diagnostic`
````

---

## Appendix B: draft `PIPELINE.md` for the whole system

Every number below was checked against the code and names its source (PIPE-10). The Status value
needs the lead's confirmation. The constants live in `pico/aggreGATR/src/config.h:21`, `:24`,
`:30`; `pico/aggreGATR/src/quadrature.h:11` (`kChannels`);
`brain/communiGATR/include/communigatr/client.h:49`, `:59`, `:61`;
`pi/naviGATR/src/runtime/inspection_config.h:47-49`; `rate_hz` in
`pi/naviGATR/config/override/brain_profile_usb.xml:18`. The failure lines come from the component
docs.

````markdown
# GATR2 pipeline

The robot needs to line up relative to things on the field. Custom electronics measure where
the robot is and where things around it are; the V5 Brain plans and drives from that.

Status: host-tested per component, see [validation](docs/validation.md)

## Start here
- this file, then each component's PIPELINE.md in the Stages table, in order
- the bytes on every link: [translaGATR](translaGATR/), spec in [docs/interfaces.md](docs/interfaces.md)

## Input
- encoder counts and IMU samples, on the Pico; camera frames, on the Pi
- the robot profile and the program's goals, on the Brain

## Output
- motor commands, on the Brain
- the viewer page and capture bundles, for people

## Flow
```text
encoders, IMU -> Pico (aggreGATR) --UART--> Pi (naviGATR) <--Brain link--> Brain (communiGATR)
                          camera ---------^        |                            |
                                                   |                investiGATR plans, actuGATR drives
                                                   +--HTTP/WebSocket--> browser (spectaGATR)
```

## Stages
| # | Component | In | Out | Contract |
|---|---|---|---|---|
| 1 | [pico/aggreGATR](pico/aggreGATR/PIPELINE.md) | 3 quadrature encoders (`kChannels`), 1 IMU (BNO08X or ASM330, chosen at build), Pi commands | sensor frames, status, diag | [translaGATR/frames.h](translaGATR/frames.h) |
| 2 | [pi/naviGATR](pi/naviGATR/PIPELINE.md) | Pico frames, camera frames, Brain requests and robot profile | robot pose, field-object estimates, Brain replies | [pi/naviGATR/PIPELINE.md](pi/naviGATR/PIPELINE.md) |
| 3 | [brain](brain/PIPELINE.md): communiGATR, investiGATR, actuGATR | Brain link replies, the program's goals | motor commands, motion status | [state_source.h](brain/investiGATR/include/investigatr/state_source.h), [motion.h](brain/actuGATR/include/actugatr/motion.h) |
| 4 | [pi/naviGATR/spectaGATR](pi/naviGATR/spectaGATR/PIPELINE.md) | inspect/2 feed, capture bundles | the viewer page | [inspection.md](pi/naviGATR/docs/inspection.md) |

Programs that assemble stage 3: [locaGATR](brain/locaGATR/) (bench, no motors) and
[operaGATR](brain/operaGATR/) (driving).

## Inside
- [aggreGATR](pico/aggreGATR/PIPELINE.md), [naviGATR](pi/naviGATR/PIPELINE.md),
  [brain](brain/PIPELINE.md), [spectaGATR](pi/naviGATR/spectaGATR/PIPELINE.md)

## Rules
- the Pico and the Pi never wait on each other; any device can reboot and resync on the sync bytes
- the Pi never sends to the Brain unasked; at most one reply per request
- where-am-I (localization) and where-are-things (world estimation) meet only through the pose history

## Change points
- season: `pi/naviGATR/config/seasons/<season>/field.xml` (landmarks, tag mounts, wire ids,
  obstacles); `brain/operaGATR/include/field_references.h` is generated from it
- program behavior and test destinations: `brain/<program>/include/robot_config.h`
- robot, not season: `brain/robot/gatr2_robot.h` (sent to the Pi as the robot profile), Pi devices
  in the Pi XML, Pico pins in `pico/aggreGATR/src/board.h`

## Threads and rates
- Pico: a sensor frame every 20 ms (50 Hz, `kSampleHz`), UART 115200 baud (`kPiBaud`); status
  every 200 ms (`kStatusPeriodUs`)
- Pi: estimation loop at the config's Loop `rate_hz` (100 in the robot configs); field worker per
  new sensor snapshot, latest wins
- Brain: GET_STATE every 20 ms (`state_period`), field estimate every 0.5 s (`field_period`),
  TELEMETRY every 0.1 s (`telemetry_period`)
- viewer: state up to 30 Hz (`state_hz`), diag 4 Hz (`diag_hz`), camera preview 5 Hz
  (`preview_hz`), defaults

## Failure behavior
- Pico IMU missing or aligning: gyro bit clear, encoders keep flowing
- Pi loses a source it uses: robot unplaced, event logged, the Brain places again
- Brain link quiet: the Brain reports no link; motion waits, then fails
- slow viewer: skipped, never waited for
````

---

## Appendix C: draft `pi/naviGATR/PIPELINE.md`

Paths are the post-restructure ones (11.4), with step 2c done. This replaces the 83-line header
comment of `runtime/system.h` and the "What runs" part of the README. Sources:
`pi/naviGATR/src/app/main.cpp:1-4` "a configuration error exits before anything starts" and
`:99-101` `return 1;`; `:174-176` `applyPendingProfile()` then
`sleep_for(std::chrono::milliseconds(20))`; `pi/naviGATR/src/runtime/handoff.h:5-10` (latest
wins); `pi/naviGATR/src/runtime/system.cpp:1022-1031` (`step`), `:339` (the pairing rule).

````markdown
# naviGATR pipeline

Where the robot is and where the things it lines up against are, from Pico and camera
measurements; answers the Brain. One process, one XML configuration.

Status: host-tested, see [validation](../../docs/validation.md)

## Start here
- `src/main.cpp`, `main`: parse the command line, `registerAll`, `System::buildFromFile`, run
- `src/core/build.cpp`, `System::build`: the construction order (the make pipeline)
- `src/core/run.cpp`, `System::step`, `estimationCycle`, `fieldCycle`, `reportingCycle`: one
  cycle; `runInline`, `runThreaded`: the run loops
- `src/pipeline/register_all.cpp`, `registerAll`: every plug-in, by stage

## Input
- Pico sensor and status frames over UART: `pico_telemetry` resource
- camera frames: `libcamera_camera` resource (Pi build) or `synthetic_rig`
- Brain requests and the robot profile, Brain link v4 over RS-485 or V5 USB: `brain_link` collector
- configuration: one `Configuration` XML under `config/`

## Output
- Brain replies (state block, field documents): `brain_link` publisher
- inspection feed `navigatr.inspect/2` for spectaGATR: `src/debug/inspection/`, read-only
- capture bundles `gatr2.capture/1`: `src/debug/capture/`, read-only

## Stages
```text
build      registerAll -> System::build: 1 .. 7 in order; any failure: nothing runs

estimation resources -> sensors -> commands -> localization
worker     -> hand the sensor map to the field worker (latest wins)
           -> targets -> publishing, against the newest field snapshot

field      world estimation on the newest sensor map -> field snapshot
worker
```

| # | XML element | Folder | Built by | Out |
|---|---|---|---|---|
| 1 | `Resources` | `src/pipeline/resources/` | `make_resources` | `ResourceMap` |
| 2 | `Sensors` | `src/pipeline/sensors/` | `make_sensors` | `SensorMap` |
| 3 | `CommandCollection` | `src/pipeline/commands/` | `make_commands` | `CommandState` |
| 4 | `Localization` | `src/pipeline/localization/` | `make_localization` | `RobotState`, pose history |
| 5 | `WorldEstimation` | `src/pipeline/world_estimation/` | `make_world_estimation` | `FieldState`, evidence |
| 6 | `TargetResolution` | `src/pipeline/targets/` | `make_targets` | `TargetState` |
| 7 | `Publishing` | `src/pipeline/publishing/` | `make_publishing` | Brain reply |

## Inside
- [Resources](src/pipeline/resources/PIPELINE.md), [Sensors](src/pipeline/sensors/PIPELINE.md),
  [Commands](src/pipeline/commands/PIPELINE.md),
  [Localization](src/pipeline/localization/PIPELINE.md)
- [World estimation](src/pipeline/world_estimation/PIPELINE.md),
  [Targets](src/pipeline/targets/PIPELINE.md), [Publishing](src/pipeline/publishing/PIPELINE.md)
- [Brain profile](src/brain_profile/PIPELINE.md)

## Rules
- one writer per mutable state; readers get snapshots
- the build is all or nothing
- the Pi never sends unasked; at most one reply per request
- a slow camera never delays localization or a Brain reply
- localization and world estimation meet only through the pose history

## Change points
- new device or sensor: a plug-in in a `resources/` family folder or in `sensors/channels/`,
  plus its register line; no core change
- new world estimator (a sensor group): `world_estimation/estimators/<group>/` plus its register
  line
- robot geometry: the Brain robot profile, not Pi code
- season: `config/seasons/<season>/field.xml` only; nothing in `src/`
- not yet local: a collector or publisher other than `brain_link` (the core pairs them by name;
  F3)

## Brain profile
With `BrainProfile` under `Localization`, stages 2 and 4 come from the robot profile the Brain
sends and swap in at a cycle boundary; until then localization is noop. Sensor loss, CONTROL and
READ_WHEELS belong to this feature: `src/brain_profile/`.

## Threads and rates
- estimation worker, Loop `rate_hz`: 1, 2, 3, 4, then 6, 7
- field worker, per new sensor map, latest wins: 5
- main thread, `runThreaded`: applies a pending Brain profile between cycles
- inspection service, capture recorder: own threads, read snapshots only
- `--inline`, `runInline`: 1 to 7 in order on one thread (`System::step`)

## Failure behavior
- configuration error: nothing runs, exit 1
- device fails to open: build warning; its outputs report no data
- used source lost (Brain profile): robot unplaced, event logged, the Brain places again
- field worker behind: the pending sensor map is replaced and counted
- slow browser: skipped, never waited for

## Exceptions
- SYS-5, SYS-20: `brain_link` pairing and `if (profiled_)` in the cycle until F3 and F1
````

---

## Appendix D: draft `pi/naviGATR/src/pipeline/world_estimation/PIPELINE.md`

The stage example with a sensor group inside. It leads with the problem (PIPE-5), states the
error budget, and describes the code after restructure step 3 (`LandmarkError`). Sources:
`pi/naviGATR/src/contracts/field_estimation.h:42-63`;
`pi/naviGATR/src/payloads/field_object_evidence.h:1-12`;
`pi/naviGATR/src/impl/world_estimation/apriltag_world_estimator.h:12-17` ("A step fault leaves
the previous field untouched and publishes nothing");
`pi/naviGATR/src/impl/world_estimation/apriltag_world_estimator.cpp:120`
`estimator->landmarks_ = LandmarkEstimator::create(landmarks, field, err);` (the group owns the
fold); `restructure/SPEC.md` (the goal, the error budget, `LandmarkError`);
`pi/naviGATR/src/runtime/world_estimation_stage.cpp:90` (one Estimator).

````markdown
# World estimation

Line up relative to field objects: how far each landmark sits from where the field map says it
is, so targets defined relative to it land in the right place. Absolute field truth is not the goal.

Status: host-tested, see [validation](../../../../../docs/validation.md)

## Start here
- `stage.cpp`, `make_world_estimation`: builds the one configured `<Estimator>`
- `estimators/apriltag/apriltag_world_estimator.cpp`, `AprilTagWorldEstimator::run`: one invocation
- `contract.h`, `evidence.h`, `field_state.h`: the contract, the seam type, `FieldState` and `LandmarkError`

## Input
- `SensorMap` (camera frames), from sensors
- the pose history (pose at measurement time) and `RobotState`, from localization: the only coupling
- the previous `FieldState`; the field map (nominal poses): `field_map` resource

## Output
- `FieldState`: per landmark its pose and `LandmarkError` (dx, dy, dtheta) from nominal; (0, 0, 0) is as built
- `FieldObjectPoseEvidence`, published for inspection and target resolution

## Stages
```text
SensorMap -> estimator (one sensor group) -> FieldState, evidence
inside the group: extraction -> association -> FieldObjectPoseEvidence -> LandmarkEstimator
```

## Inside
- [AprilTag sensor group](estimators/apriltag/PIPELINE.md)

## Rules
- exactly one `<Estimator>`; no fusion; range or bearing evidence waits for a second sensor group (P-3)
- the seam is one type: full-pose evidence (frame, measurement time, odometry epoch)
- a sensor group keeps partial measurements, time alignment and association inside it
- `landmark_estimator.*` is generic code the group calls; it never knows sensors
- the map's heading zero is an authoring convention with no meaning

## Error budget
- localization error is in both the landmark error and the robot pose: it cancels for landmark-relative targets
- residual 1, the main one: perception (tag detection, camera calibration, mount geometry)
- residual 2: drift since the last sighting; landmark-anchored targets re-resolve every cycle while in view

## Change points
- a new sensor group: `estimators/<group>/` with its register line; it emits full-pose evidence
- season: the field file only

## Failure behavior
- a step fault: the previous field is kept and nothing is published
- no association configured: observations still publish; no landmark commits
````

---

## Appendix E: stage contract template

A template for SYS-2, not repo code. `Xyz` is replaced by the stage name. It is the functional
seam of TYPE-5, with the aliases ALIAS-3 and ALIAS-4 ask for. `ResetFunction`
(`using ResetFunction = std::function<void()>;`) and `WarningList` are proposed shared aliases in
`pipeline/framework/`.

```cpp
// pipeline/xyz/contract.h
// Xyz stage: sensor map in, xyz state out.

#pragma once
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "pipeline/framework/execution_context.h"
#include "pipeline/framework/function_status.h"
#include "pipeline/framework/payload_descriptor.h"
#include "pipeline/framework/records.h"

namespace navigatr
{

class ResourceStore;
class SensorCatalog;

struct XyzInput {
    const SensorMap&        sensors;
    const XyzState&         previous;
    const ExecutionContext& context;   // host clock, cycle, diagnostics
};

struct [[nodiscard]] XyzOutput {
    XyzState       state;
    FunctionStatus status = FunctionStatus::kOk;
    std::string    diagnostic;
};

struct XyzOutputDecl {
    XyzId             id;
    PayloadDescriptor payload;
};
using XyzOutputDeclList = std::vector<XyzOutputDecl>;

using XyzRunFunction = std::function<XyzOutput(const XyzInput&)>;

struct XyzExecutable {
    XyzOutputDeclList outputs;
    XyzRunFunction    run;
    ResetFunction     reset;   // may be empty
};

struct XyzBuildContext {
    const ResourceStore* resources = nullptr;
    const SensorCatalog* sensors   = nullptr;
    WarningList*         warnings  = nullptr;
};

using XyzMakeFunction = std::function<std::optional<XyzExecutable>(
    const ConfigNode&, XyzBuildContext&, std::string& err)>;

} // namespace navigatr
```

---

## Revision notes

Draft 2 applies the three critiques (`critique/fit.md`, `critique/coverage.md`,
`critique/usability.md`). No existing rule ID was renumbered; new rules took the next free ID and
dropped rules are marked retired. One line per choice:

- Plug-in folder names follow the lead's 15:05 correction in `restructure/plan/FINAL.md`, which postdates the critiques: it supersedes fit 1 (literal plurals) and usability 3 and 42 (agent-noun decision); D-28 is resolved and no slot-name decision was opened.
- Shared test support goes to `<component>/sim/` (fit 3, the lead's `tests/` rule), not `tests/support/` (usability 9).
- Line 1 stays `// <file name>`, the path only for repeated basenames (fit 4); usability 10's retirement of STRUCT-7's second sentence was declined.
- PIPE-2 requires fit's six items plus Change points and Failure behavior (usability 20), which PIPE-8 and P-7 already demand.
- PIPE-2 sizes: root and component at most 100 lines (fit); stage and sub-pipeline at most 50, between fit's 30 and usability's 40, so Appendix D keeps the error budget the lead asked for.
- ALIAS-15 is MUST (coverage 40), not SHOULD (fit 16): it can be answered from the diff under the new 0.1; existing `auto` is deferred to phase 5.
- Fit's performance rule keeps API-8; coverage's blocking rule, also proposed as API-8, became API-9.
- InsertBraces runs after the whitespace reformat (usability 34), not before (coverage 16), so its commit holds only braces.
- D-1 recommends a `GATR2_WARNINGS_AS_ERRORS` option (usability 46); the CMake 3.24 variable (coverage 14) is kept in 9.1 as the alternative.
- Section 12 is ordered by what each decision blocks (usability 47); decisions that change a lead rule are marked `(lead rule)` and default to the rule, instead of a separate Major table (fit 7).
- The entry-point step is step 2c, not 2b (usability 2): FINAL.md already names 2a (noop split) and 2b (system split).
- Decision IDs: D-29 is fit's step-contract question (fit's D-30); D-30 is usability's phase-extent question (its D-32); usability's factory-name question (its D-31) folded into D-8.
- Usability 43 (README size decision) not opened: DOC-1 applies the lead's "short signposts (a few lines)" rule directly, at most 20 lines.
- STRUCT-10 counts units (fit 36) with an exact limit of 15 (usability 19), not "12 files".
- HDR-5 uses 10 lines (Google's inline guidance), between usability 19's 5 and coverage 6's 15 NLOC report.
- Duplicates (usability 17): retired CON-1, TYPE-6, SYS-22, ERR-8, LOG-3 (into RT-4), TEST-9, TEST-14; SYS-11 and SYS-12 became pointers.
- Duplicates declined: API-4 stays as the SHOULD remedy with CPLX-4 as the limit (fit 26); REPO-8 stays for stage contracts, which WIRE-7 does not cover; STRUCT-7 stays.
- STRUCT-6 split (usability 18): the coordinator part stays MUST; "one responsibility per file" became STRUCT-13 (SHOULD).
- One level per rule: coverage's appended SHOULD clauses became RT-16 (call-chain stack report) and TEST-21 (fuzzing); CPLX-5's test row became CPLX-10 (usability 26).
- Part 1 and Part 2 (usability 30) are declared in the header, not by moving sections, so every section reference stays valid.
- The rule index (usability 27) is not embedded; `rule_index.py` (9.8) generates it from the rule headings.
- Not applied: generating the 11.4 move-map table from the TSV (usability 3); the summary was synced by hand and says the TSV wins.
- Not applied: usability 49's optional switch of Why lines to principle citations; the lead's quotes stay where a rule comes from the lead.
- Not applied: coverage 22's note on the `brain/operaGATR/src/main.cpp:241` function-local static; no race was shown, and RT-5 now bans the pattern.
- Not applied: fit 36's resource family split as a proposal; the lead's correction already made it.
- SYS-19, STRUCT-8 and 11.6 cite FINAL.md section 5 for the baseline edges, with no count: `restructure/logs/final_map_check.txt` still shows the pre-correction folder names.
- Plan phases were renumbered (not rule IDs): old 0 -> 0; old 1 -> 1 (CI safety net) and 3 (mechanical); old 2 -> 2 (restructure) and 4 (structure elsewhere); old 3 -> 5, except the config reader -> 2; old 4 -> 6; old 5 -> 7; old 6 -> 8.
- Citation fixes found on the way: REPO-3 pointed at 9.5 (now 9.4); `angles.h:14-22`; `hub.cpp:59`; `motion_step.h:135-136`; `pros_serial_port.cpp:29`.
- The 9.2 config was re-run through clang-tidy 22.1.8 `--verify-config`: "No config errors detected".

Verification pass on draft 2 (details in `survey/verify.md`):
- Citations: about 355 `path:line` references checked against `ae457ee`. Line ranges fixed:
  `se3.h:73-75`, `sensor.h:47-48`, `pros_usb_link.cpp:151-168`, `tracking_wheel_motion.cpp:163-178`,
  `system.h:1-83` (an 83-line header, also in phase 6 and Appendix C).
- Examples that did not show the rule: DOC-3 and DOC-5 "Good" became "Closest today" with what is
  missing; WIRE-6 `kNoTicket` (the ticket is Brain-local, not wire) became `kNoDocument`, marked
  proposed, plus a real Bad; ALIAS-2's `PlacementTicket` notes its TYPE-16 sentinel.
- New FMT-1 (pinned clang-format) and FMT-2 (braces on every control statement): the lead's rules
  had no rule ID; `readability-braces-around-statements` was configured but unmapped.
- Conflicts fixed: PY-1 and D-4 no longer ban Markdown docs in naviGATR (PIPE-1 needs them);
  phase 4 no longer adds a spectaGATR README (DOC-1); OWN-1 shared pointers never cross PROS tasks
  (RT-5); CMT-5 counts the PIPELINE.md pointer within its 5 lines; HDR-2, JS-15 and 9.1 name their
  dependence on D-25 or D-1; 9.9 no longer presumes D-1 option (b); NAM-2 records the two
  `register_*` names the move keeps; the Appendix E output struct is `[[nodiscard]]` (ERR-3).
- CMT-2 now prefers a bare noun phrase (the lead's calibration); the CFG-3 Good block's comments
  are bare labels.
- 2026-09-28 lead decisions applied: D-1 selective `-Werror=` list (9.1); D-4 Markdown allowed in naviGATR; D-5 `make_*` for every stage and sub-step returning verb-named callables (TYPE-17, NAM-2), `main` split (F11); D-6 folder-mirroring namespaces (NS-2); D-7 TYPE-1 is SHOULD; D-25 clang-format knobs adopted.
- Section 12 rows for those six rewritten as decision records; remaining rows default to their recommendation.
