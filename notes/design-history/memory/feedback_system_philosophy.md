---
name: feedback-system-philosophy
description: user's core design philosophy (2026-09-28): season-reusable generic pipelines, no over-generalization, standard I/O, readability over speed, struct/class/functional rules, folder structure + PIPELINE.md
metadata:
  type: feedback
---
The user's words (2026-09-28), condensed:

- **Reusable across seasons.** Generic pipelines, specific implementations plugged in (sensors etc.). If the pipeline itself must be season-specific, you are probably doing something wrong. But do NOT over-generalize beyond what is required.
- **Understand the problem first.** naviGATR's core question is "where am I, and where are things around me". Localization and world estimation are separate systems, coupled only when relative observations become field positions through the pose.
- **Stable systems with standard I/O.** Existing systems keep their shape; each has a standard INPUT/OUTPUT so different teams can work inside them.
- **Readability is king.** Slightly slower but clearer wins, especially since we are sensor-throttled.
- **Code rules:**
  - `using` aliases liberally;
  - `struct` for data-only types, `class` for data plus methods;
  - avoid classes for their own sake and ask whether a functional approach works;
  - everything namespaced.
- **Structure.** A system should be clear from its folder structure. Every pipeline and sub-pipeline, and the whole system, gets a concise .md explaining its pipeline.
- **Gripe.** naviGATR is hard to read: the main pipeline and the make_* construction are hard to find (both live in runtime/system.cpp System::build).

**Why:** the system must survive seasons and be workable by different teams; readability over cleverness.

**How to apply:** check new designs and code against these rules first. Prefer free functions and plain structs. Keep the season-specific parts in implementations and config. Keep folder layouts that mirror the pipeline. Pair each pipeline change with its PIPELINE.md. Related: [[feedback-design-principles]], [[feedback-comment-style]], [[project-coding-standard]].

**World estimation model (confirmed and corrected 2026-09-28):**
- The XML picks ONE estimator (a sensor group such as apriltag, or later ToF+X), with its own config.
- There is no fusion yet.
- The output is a pure error from the map's nominal pose, `LandmarkError{dx, dy, dtheta}` in field axes; (0, 0, 0) means the field is as built. The map's heading zero is an authoring convention only.
- Plug-in folders get a readable plural noun for what the plug-ins are (estimators/, publishers/, resolvers/, channels/). A stage with many plug-ins groups them by family (resources/cameras, resources/pico). Never a mechanical XML plural (no publishings), never a stutter, never `impl`.

**The ultimate goal is lining up relative to things, not absolute field truth.**
- Shared localization error cancels in relative targeting.
- What remains: perception (AprilTag) error, plus drift since the last sighting (kept small by re-resolving landmark-anchored targets each cycle).
- Lead world estimation docs with this goal.

**Code-shape decisions (2026-09-28):**
- Every stage and sub-step has `make_<step>(config)`. It returns a callable (a lambda capturing config and state) with a descriptive `using` alias. Call sites bind verb names, e.g. `const auto associate_tags = make_tag_association(cfg); associate_tags(obs)`.
- Namespaces mirror folders up to a sensible depth: one per pipeline stage or area, plus one level for a self-contained family folder. Grouping folders add no level. Never one big namespace.
- "struct = data only" is highly recommended (SHOULD), not strict.
- Warnings: only always-a-bug warnings fail the build (`-Werror=<name>`). Everything else warns, and CI allows no new warnings. The user thinks of a build error as "we genuinely cannot continue".
