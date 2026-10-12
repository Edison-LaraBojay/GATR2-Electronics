# Survey: documentation

Scope: every first-party README.md and docs/*.md in C:/GATR2-Electronics
(git-tracked, third_party excluded), plus header and API comment practice in
C++ and JS. Checked against industry practice and the user's own rules
(requirements.md, feedback_comment_style.md, feedback_conventions.md,
feedback_design_principles.md). Surveyed 2026-09-28 at commit ae457ee, clean
tree. Read-only; nothing in the repo was changed.

Citations are `path:line` with a short quote. Paths use the git spelling
(`pi/naviGATR/...`); see finding F1 about the on-disk spelling.

---

## 0. Key findings

1. The docs are mostly accurate. Every behavioral claim spot-checked against
   code held up (section 1.4). The staleness that exists comes almost entirely
   from the 2026-09-28 rename and from the same fact being written in two or
   three places.
2. The repo has 58 first-party Markdown files, about 11,250 lines. Most of
   that is deep material under `docs/` (4,843 lines) and `pi/naviGATR/`
   (5,467 lines). Three READMEs are far past "short signpost":
   `pi/naviGATR/README.md` (278 lines), `pico/aggreGATR/README.md` (211) and
   the root `README.md` (98).
3. No pipeline has a dedicated, concise PIPELINE doc. Parts exist in several
   places:
   - a good system wire diagram at `docs/interfaces.md:23-31`;
   - a good naviGATR run diagram at `pi/naviGATR/README.md:62-77`;
   - the best short model, the four-step list at `docs/actugatr.md:3-9`;
   - the `Layers` code map at `docs/communigatr.md:39-56`.

   None of them names the file and function where the pipeline actually runs.
4. The naviGATR gripe is real and can be traced. The main pipeline is
   `System::estimationCycle`, `fieldCycle` and `reportingCycle`, and
   construction is `System::build`. All four sit in the 1,450-line
   `src/runtime/system.cpp`. The `make_*` builders are spread over four
   `src/runtime/*_stage.cpp` files. The only doc that points there is the
   last table of a 381-line explanation (`pi/naviGATR/docs/architecture.md:362-381`).
   The README never mentions `src/`.
5. Comment practice splits by component:
   - Brain, translaGATR and aggreGATR headers are short labels. Median header
     block: 5 lines for the Brain, under 4 for the other two.
   - naviGATR headers are long narrative: mean 16 lines; 36 of 128 headers run
     20 lines or more; `system.h` runs 83. They repeat pipeline explanation
     that also lives in docs.
   - Units and clock domains are usually carried in names and types. This is a
     strength to keep.
6. The repo has no doc tooling: no link check, no Markdown lint, no ASCII
   check, no glossary, no decision records, no CHANGELOG. Hardware-validation
   status is written in free text in at least 17 docs, each worded
   differently.

---

## 1. Inventory

### 1.1 Numbers

| Area | .md files | Lines |
|---|---|---|
| root README.md, CONTRIBUTING.md | 2 | 102 |
| docs/ (incl. docs/contributing) | 13 | 4,843 |
| brain/** | 7 | 191 |
| pi/** (pi/README.md, naviGATR README, docs, tools/perf) | 23 | 5,467 |
| pico/** | 2 | 218 |
| bench/** | 2 | 55 |
| pcb/** READMEs | 11 | 371 |
| Total | 58 | 11,247 |

Not counted: 3 `third_party/README.md` files. They are a good provenance
template: `pico/aggreGATR/third_party/README.md:9-17` records upstream,
version, commit, license, kept files and users.

READMEs over 40 lines: `pi/naviGATR/README.md` 278,
`pico/aggreGATR/README.md` 211, `README.md` 98, `docs/README.md` 75, the Pi HAT
v3/v4 iteration READMEs 54 each, `pcb/PiHat/PiHat_v2/README.md` 46,
`bench/rs485_link/README.md` 46.

Folders with no README: `translaGATR/`, even though `README.md:86` links the
folder and `docs/naming.md:20` describes it; `pi/naviGATR/spectaGATR/`; and
every `pi/naviGATR/src/*` folder. The Brain libraries do not need per-folder
READMEs because each library README lists its folders (for example
`brain/actuGATR/README.md:9-13`).

### 1.2 Per-file inventory

Type follows Diataxis: T tutorial, H how-to, R reference, E explanation. S
means signpost or index, and M means one file mixes three or more types.

| File | Lines | Type | Audience | Notes |
|---|---|---|---|---|
| README.md | 98 | S+E | newcomer | System sketch at :6-9. :34-50 repeats naviGATR wire details. |
| CONTRIBUTING.md | 4 | S | PCB contributor | PCB only; there is no software contributing guide. |
| docs/README.md | 75 | S | everyone | Doc index. |
| docs/contributing/README.md | 32 | H | PCB contributor | Iteration-folder rule is contradicted by the tree (F6). |
| docs/naming.md | 53 | R | everyone | -GATR names and rename migration notes. |
| docs/hardware.md | 173 | E+R | hardware, firmware | |
| docs/interfaces.md | 1478 | R | implementers on all three targets | Wire pipeline at :23-31, "headers win" rule at :7-8, validation status at :44-50. |
| docs/pi_setup.md | 267 | H | Pi operator | |
| docs/brain_setup.md | 368 | H | Brain programmer | Stale program names; dangling "final report" (F3, F4). |
| docs/investigatr.md | 468 | R+E | Brain programmer | |
| docs/communigatr.md | 1126 | R+E | Brain programmer | Status line at :22-25, `Layers` code map at :39-56. Stale "testing" (F3). |
| docs/actugatr.md | 280 | E+R+H | Brain programmer | Short pipeline statement at :3-9. |
| docs/navigatr_resources.md | 390 | R | naviGATR config author | Registry; matches code. Lives outside naviGATR. |
| docs/navigatr_sensors.md | 133 | R | naviGATR config author | Registry; matches code. |
| brain/README.md | 38 | S | Brain programmer | Good signpost. |
| brain/actuGATR/README.md | 15 | S | Brain programmer | Good. |
| brain/communiGATR/README.md | 37 | S | Brain programmer | Good; includes an API list. |
| brain/investiGATR/README.md | 14 | S | Brain programmer | Good. |
| brain/robot/README.md | 11 | S | Brain programmer | Good. |
| brain/locaGATR/README.md | 40 | S+H | bench operator | Stale title, broken anchor (F2, F3). |
| brain/operaGATR/README.md | 36 | S+H | driver, tuner | |
| pi/README.md | 19 | S | everyone | Repeats root README text. |
| pi/naviGATR/README.md | 278 | M | everyone | Good run diagram at :62-77, but no code map and no mention of `src/`. |
| pi/naviGATR/docs/setup.md | 721 | T+H | first-time builder | "Start here" (:3). |
| pi/naviGATR/docs/architecture.md | 381 | E+R | naviGATR developer | Only place with a source map (:362-381). |
| pi/naviGATR/docs/brain_profile.md | 384 | E+R | naviGATR and Brain developer | |
| pi/naviGATR/docs/configuration.md | 159 | H+R | config author | |
| pi/naviGATR/docs/coordinates.md | 165 | E+R | everyone | Frames and clocks. |
| pi/naviGATR/docs/landmarks.md | 246 | E+R | naviGATR developer | Mini glossary at :8-39. |
| pi/naviGATR/docs/localization_fusion.md | 199 | E | estimator developer | |
| pi/naviGATR/docs/inspection.md | 768 | R+E | viewer and tool developer | Protocol reference. |
| pi/naviGATR/docs/capture.md | 394 | R+H | debugging | |
| pi/naviGATR/docs/pico_link.md | 298 | R+E | naviGATR developer | Pico rules repeated (F8). |
| pi/naviGATR/docs/camera_preview.md | 159 | H | bench operator | |
| pi/naviGATR/docs/pi_camera_setup.md | 290 | H | Pi operator | "Not verified on hardware" list. |
| pi/naviGATR/docs/parallel_wheel_bringup.md | 264 | H | bench operator | Has troubleshooting. |
| pi/naviGATR/docs/usb_localization_bench.md | 139 | H | bench operator | |
| pi/naviGATR/docs/vex_imu_bench.md | 80 | E+H | bench operator | |
| pi/naviGATR/docs/calibration_inventory.md | 95 | R | builder | Status table with PLACEHOLDER, PROVISIONAL and UNMEASURED markers (:15-20). |
| pi/naviGATR/docs/field_assets.md | 307 | R+E | field data maintainer | Provenance of field numbers. |
| pi/naviGATR/docs/attitude_firmware_followup.md | 60 | E | firmware | Future work. |
| pi/naviGATR/docs/printables/README.md | 35 | H | builder | |
| pi/naviGATR/tools/perf/README.md | 26 | R+H | perf work | Dense bullets; states the host-only limit on line 3. |
| pico/README.md | 7 | S | everyone | Good. |
| pico/aggreGATR/README.md | 211 | M | firmware, bench | Repeats Pico command rules from interfaces.md (F8). |
| bench/README.md | 9 | S | bench | Good. |
| bench/rs485_link/README.md | 46 | H | bench | Good bring-up shape: Wiring, Pi side, Brain side, Success. |
| pcb/README.md + 10 board and iteration READMEs | 371 | S / R | hardware | Consistent template (breadcrumb, bold status, changes, files). Stale link (F1); v4 duplicates v3 (F5); middle dot (F10). |

### 1.3 Staleness and contradictions (verified)

I ran a relative-link and anchor checker over all 58 files. It compares link
targets against git-tracked paths with exact case. The script is at
`scratchpad/standard/docs/linkcheck.py`. Result: 2 broken links. The other
findings below came from reading the docs and grepping.

- **F1. Stale link in `pcb/README.md:38`.** It reads
  `[Pi sensing and estimation software](../pi/navigatr/README.md)`, but git
  tracks `pi/naviGATR/`.
  - The link works on this Windows box only because the on-disk folder is
    still lowercase (`ls pi` prints `navigatr`, and `core.ignorecase=true`).
    It breaks on GitHub and on the Pi.
  - A checker that looks at the filesystem on Windows misses this. My first
    checker version wrongly flagged 232 correct links for the same reason. A
    link check must compare against `git ls-files` or run on Linux.
- **F2. Broken anchor in `brain/locaGATR/README.md:39`.** It links
  `../../docs/brain_setup.md#3-locaGATR-program`, but the heading is
  `## 3. Localization test program` (`docs/brain_setup.md:96`). The anchor
  looks like it was produced by a text replace during the rename.
- **F3. Pre-rename names left in prose.** The rename changed the paths but not
  the names used in running text.
  - `brain/locaGATR/README.md:1` has the title `# Localization test`.
  - `docs/brain_setup.md:96` has the heading `## 3. Localization test program`.
  - `docs/communigatr.md:42` says `application (locaGATR, testing)`, where
    "testing" is now operaGATR (`docs/naming.md:31`).
  - "the testing program" also appears at `docs/actugatr.md:112` and `:251`,
    and "UP (testing)" at `docs/brain_setup.md:204` and `:272`.
- **F4. Dangling reference in `docs/brain_setup.md:288`.** It says "The final
  report lists the exact counts." No such report is in the repository.
- **F5. `pcb/PiHat/PiHat_v4/README.md` is byte-identical to
  `pcb/PiHat/PiHat_V3/README.md`.** Its title reads `# Pi HAT v3` (line 1), and
  v4 is missing from the iterations table (`pcb/PiHat/README.md:22-26`).
- **F6. Folder case contradicts the contributing guide.**
  `docs/contributing/README.md:7` says iteration folders are named
  "`{BOARDNAME}_V{number}`". The tree has `PiHat_v1`, `PiHat_v2`, `PiHat_V3`,
  `PiHat_v4`, `IMU_v1`, `IMU_v2` and `MagneticEncoder_v1`.
- **F7. The same pipeline written three times has already drifted.**
  - The naviGATR construction order appears at `pi/naviGATR/README.md:63-65`,
    `docs/architecture.md:17-26` and `src/runtime/system.h:17-20`. The README
    version omits the CommandCollection factory, which `system.cpp:276` builds
    between sensors and localization
    (`buildSlot("CommandCollection", commands_, ...`). `architecture.md:21`
    has it.
  - The apriltag sub-pipeline appears at `architecture.md:190-194`,
    `landmarks.md:42-48` and `apriltag_world_estimator.h:1-19`. The last stage
    is `FieldState` in the first and `FieldSnapshot` in the second.
- **F8. Pico command rules written three times.** They appear at
  `docs/interfaces.md:360-390`, `pi/naviGATR/docs/pico_link.md:71-103` and
  `pico/aggreGATR/README.md:29-42`. They agree today, but three copies mean
  three edits for every change.
- **F9. Inconsistent titles and spellings.**
  - README titles vary: `# navigatr` (`pi/naviGATR/README.md:1`),
    `# Pico acquisition firmware` (`pico/aggreGATR/README.md:1`) and
    `# brain/operaGATR` (`brain/operaGATR/README.md:1`).
  - The component name is spelled three ways in prose: "naviGATR" 102 times,
    "navigatr" 81 times (partly legitimate code-font uses of the namespace or
    executable) and "Navigatr" 16 times (for example
    `README.md:30 [Set up and run Navigatr]`).
- **F10. Non-ASCII.** The only non-ASCII characters in all tracked
  first-party text files are 7 middle dots (U+00B7) in the PCB iteration
  breadcrumbs, for example
  `pcb/IMU/IMU_v1/README.md:3 [IMU overview](../README.md) . [All PCBs]`
  (the dot is shown here as `.`). Code comments and all other docs are ASCII.
- Minor, and outside the standard because CI is being rebuilt: the CI job is
  still named `common` (`.github/workflows/ci.yaml:9`), although its folder is
  `translaGATR/`.

### 1.4 Checked and accurate

These show that doc drift is not widespread.

| Claim | Doc | Code |
|---|---|---|
| Brain link v4 | `docs/interfaces.md:468` | `translaGATR/frames.h:100 kBrainLinkVersion = 4` |
| Pico sends at 50 Hz | `pico/aggreGATR/README.md:5` | `pico/aggreGATR/src/config.h:24 kSampleHz = 50` |
| Largest sensor frame is 31 bytes | `pico/aggreGATR/README.md:22` | 14-byte v2 header + 16-byte payload + 1-byte xor (`frames.h:60-65`); accel is not populated |
| Profile swap every 20 ms | `architecture.md:289` | `src/app/main.cpp:174-176` `applyPendingProfile(); ... milliseconds(20)` |
| Default config | `pi/naviGATR/README.md:231-232` | `CMakeLists.txt:205 "config/demo/synthetic_field_demo.xml"` |
| 13 resource and 4 sensor types | `docs/navigatr_resources.md`, `docs/navigatr_sensors.md` headings | `register_resources.cpp:20-43`, `register_sensors.cpp:11-17` |
| Localization and world-estimation types | `architecture.md:99-104`, `:182-184` | `register_localization.cpp`, `register_world_estimation.cpp` |
| Stationary window defaults to 2 s | `architecture.md:120` | `stationary_window.h:45 window_ms = 2000` |
| actuGATR example API | `docs/actugatr.md:38-40` | `motion.h:144-148`, `operaGATR/include/field_references.h:10` |
| `--inline` warns with brain_link | `pi/naviGATR/README.md:240` | `src/app/main.cpp:118` |

No doc hard-codes a test count (grep for "N tests" found nothing). That is
good, because counts go stale on every commit.

### 1.5 Pipelines and where they are explained today

| Pipeline | Where it is explained now | Concise? | Where it runs in code |
|---|---|---|---|
| Whole system | `README.md:6-14` (two-line sketch and a paragraph); `docs/interfaces.md:23-50` (wire diagram) | Partly. Wire diagram only; no system-level Brain flow; no single doc. | none (distributed) |
| aggreGATR (Pico loop) | `pico/aggreGATR/README.md:1-10` (prose) | No. 211-line README; no stage list. | `pico/aggreGATR/src/main.cpp:319-356` `loop()`, which reads as the pipeline itself: `imu::service(); receiveCommands(); watchImu();` then tick, sample, encode, write |
| naviGATR run | `pi/naviGATR/README.md:57-87`; `architecture.md:322-360`; `system.h:22-45` | The diagram is concise but has no code pointers, and the text is repeated three times. | `system.cpp:1033 estimationCycle`, `:1066 reportingCycle`, `:1106 fieldCycle`, `:1411 estimationWorker`, `:1436 fieldWorker` |
| naviGATR construction (`make_*`) | `architecture.md:15-46`; `README.md:63-65`; `system.h:17-20`; `*_stage.h:1-5` | Scattered; nothing points to `System::build`. | `src/app/main.cpp:94 registerAll`, `:98 buildFromFile`; `system.cpp:148 System::build`; `resource_stage.cpp:87`, `sensor_stage.cpp:77`, `localization_stage.cpp:199`, `world_estimation_stage.cpp:71` |
| Localization | `architecture.md:84-91` (diagram) plus 70 lines of prose; `localization_stage.h:1-26` | Diagram yes; prose long. | `localization_stage.cpp:30 LocalizationExecutor::operator()` |
| World estimation (apriltag) | `architecture.md:190-194`; `landmarks.md:42-48`; `apriltag_world_estimator.h:1-19` | Concise, but written three times (F7). | `world_estimation_stage.cpp:59`, `apriltag_world_estimator.cpp:164 AprilTagWorldEstimator::run` |
| Brain link (requests to replies) | `architecture.md:216-278`; `docs/interfaces.md:468-1120` | No (long). | `impl/commands/brain_link_commands.cpp`, `impl/publishing/brain_link_publisher.cpp` |
| Brain profile apply | `brain_profile.md` (384 lines); `architecture.md:280-309` | No. It is a lifecycle, not a pipeline. | `system.cpp:816 applyPendingProfile`, `:827 swapProfile` |
| Inspection feed | `inspection.md` (768 lines) | No. It is protocol reference. | `src/inspection/` |
| spectaGATR | `spectaGATR/app.js:1-14`, `feed.js:1-14`; `inspection.md:726-768` | The only short description is in JS header comments. | `spectaGATR/app.js` |
| Capture | `capture.md` (394 lines) | No. | `src/capture/` |
| Brain movement | `docs/actugatr.md:3-19`: "resolves the destination ... asks investiGATR for a path ... follows the path ... drives tank or mecanum wheels" | Yes. This is the best existing model. | `actugatr/motion.h:169 update(Seconds now)` |
| communiGATR link | `docs/communigatr.md:1-56` (purpose list, Status, paths, `Layers` diagram) | Yes. A good model, apart from the stale "testing" at :42. | `communigatr/client.h` `Client::poll` |

What already works:
- a one-line purpose;
- a short arrow diagram in a `text` fence;
- class names placed in the diagram (communiGATR `Layers`);
- a single `Status:` line (`docs/communigatr.md:22`).

What is missing everywhere:
- named INPUT and OUTPUT (the user's "standard INPUT/OUTPUT");
- a code map naming the function where the pipeline runs;
- a pointer from the component README to that explanation.

### 1.6 The naviGATR gripe, traced

A reader who opens `pi/naviGATR/` sees `README.md`, `src/` and 14 `src/`
subfolders. The steps to reach the main pipeline are:

1. README "What runs" (`README.md:57-87`) shows the diagram, with no file
   names.
2. README "Runtime design" (`:23-55`) is a table of 17 docs. The architecture
   doc is described as "Captured resource/sensor functions, ResourceMap and
   SensorMap result contracts, all runtime paths, ..." (`:44`). Nothing says
   that the main loop is there.
3. `docs/architecture.md` spends 360 lines on explanation before its
   "Source map" (`:362`) names `src/runtime/system.cpp`.
4. `system.cpp` is 1,450 lines. The functions that matter are `build` (:148),
   `estimationCycle` (:1033), `reportingCycle` (:1066) and `fieldCycle`
   (:1106). They sit among profile, Pico and recovery helpers.
5. `src/runtime/` holds 26 files that mix the coordinator, the `make_*`
   builders, catalogs, the profile builder and sensor-loss logic. Folder names
   do not separate "contract" from "implementation" consistently:
   - resource interfaces live in `src/resources/`;
   - their implementations live in `src/impl/resources/`;
   - stage contracts live in `src/contracts/`;
   - shared types also live in `src/state/`, `src/payloads/` and `src/core/`.

On the documentation side this is fixed by a `pi/naviGATR/PIPELINE.md` with a
code map (template and draft in section 4.8), linked from the first lines of
the README. The structural part (splitting `system.cpp`, folder names) belongs
to the structure rules. The doc rule DOC-12 makes the need visible: when a
code map needs more than about 8 rows to reach the entry points, the structure
should be fixed.

### 1.7 Header and API comment practice

Measured over git-tracked first-party files; `third_party` and PROS kernel
headers are excluded.

| Component | Headers | Mean header block (lines) | Median | Headers with 20+ lines | Max | Comment:code ratio |
|---|---|---|---|---|---|---|
| pi/naviGATR/src | 128 | 16.3 | 11 | 36 | 83 (`runtime/system.h`) | 0.13 |
| brain | 54 | 5.9 | 5 | 0 | 18 (`communigatr/wheel_calibration.h`) | 0.09 |
| translaGATR | 3 | 3.7 | - | 0 | 5 | 0.08 |
| pico/aggreGATR/src | 15 | 3.6 | - | 0 | 5 | 0.12 |
| spectaGATR (JS) | 17 | 12.7 | - | 2 | 29 (`field_scene.js`) | 0.09 |

House conventions already in place:
- 426 of 457 first-party .h, .cpp and .js files start with `// <filename>`
  (93 percent). Most of the exceptions are in `pico/aggreGATR` and a few
  naviGATR tests.
- Comments use only `//`. There is no Doxygen or JSDoc; the only `/**` blocks
  are the PROS template files `brain/*/include/api.h` and `main.h`.
- First-party code has no TODO or FIXME markers.
- Enum values carry trailing labels, for example
  `investigatr/state_source.h:17-22`:
  `kNoLink, // source not connected`.

Patterns that match the user's rules and should become the standard:
- Units in the name or type instead of a comment:
  - headers contain 970 unit-suffixed identifiers (`window_ms`, `x_m`,
    `heading_deg`);
  - `brain/investiGATR/include/investigatr/geometry.h:11-14` has
    `using Seconds = double; using Meters = double; ...`.
- Clock domain in the type: `pi/naviGATR/src/core/time.h:2-4` reads
  "Monotonic millisecond count tagged with its clock domain. ... must never be
  differenced against each other."
- Wire units as field labels: `translaGATR/frames.h:64`
  `4, // kSensorGyroZ int32, millidegrees per second`.
- Thread notes as a short suffix: `src/runtime/system.h:240`
  `// The running Brain profile, null when none; any thread.`
- Declaration comments as labels:
  - `investigatr/state_source.h:42` `// Newest robot state, ages relative to now.`
  - `actugatr/motion.h:150` `// Active command -> kCancelled. No effect once terminal.`
- A short file header: `pico/aggreGATR/src/asm330_supervisor.h:2-5`
  "ASM330 bring-up and health without blocking. step() returns the one
  register action the driver performs next; ..."

Patterns that break the user's rules (terse, not AI-ish, no narration):
- `src/runtime/system.h:1-83`. An 83-line essay repeats the pipeline, the
  threading model and the profile lifecycle, all of which are also in
  `architecture.md`. For example, :37-39 reads "One writer per mutable state:
  the estimation worker owns the executors and their retained maps, command
  and target state; ..."
- `src/core/records.h:5-10` narrates: "A slow camera does not disappear between
  frames, and fault states do not erase history."
- `src/runtime/system.cpp:1027-1028` gives rationale for a test:
  "// the inline cycle lets the same cycle's evidence reach target resolution
  and publishing, as the synchronous tests expect".
- JS headers lean the same way. `spectaGATR/app.js:2-8`: "The page never
  estimates: it draws what the runtime reported, ages it on the right clock,
  and says when it is stale."

The naviGATR headers hold real contract facts: ownership, clock, failure
behavior. The fix is to keep those facts as labels and move the explanation to
PIPELINE.md or `docs/`, not to delete them.

### 1.8 How validation status is recorded today

Statements like "not run on hardware" or "host tested" appear in 17 docs, each
worded differently. For example:

- `README.md:77-78` "none of the Brain code has run on the robot yet"
- `pi/naviGATR/README.md:133` "Implemented and host tested, not run on hardware:"
- `docs/interfaces.md:44` "Validation status: host tests only."
- `docs/communigatr.md:22` "Status: host tested against a fake Pi ..."
- `docs/brain_setup.md:278` "## 9. Tests run and what is not validated"
- `pico/aggreGATR/README.md:209-211` "checked only by the builds and host
  tests, not on a real Pico"
- `pi/naviGATR/tools/perf/README.md:3` "These are host numbers, not hardware
  validation."

Measured values use markers that already work well:
`calibration_inventory.md:15-20` uses PLACEHOLDER, PROVISIONAL and
UNMEASURED, and `operaGATR/README.md:11` says "PLACEHOLDER values are not
measured".

The repo has no single place that answers "what has run on real hardware, and
when".

### 1.9 Doc tooling and records

- No link checker, Markdown lint, spell check or ASCII check. The CI jobs are
  `common`, `navigatr`, `brain` and `pico` (`ci.yaml:9,28,49,68`), all
  build and test.
- No CHANGELOG, decision records (ADRs), glossary or CODEOWNERS.
  - `docs/naming.md:22-53` is in effect a decision record and migration note.
  - `landmarks.md:8-39` "Definitions and retained state" is a local glossary.
- No repo LICENSE file. Vendored code keeps its own license files.
- The same name refers to different types in different components:
  `investigatr::RobotState` (`brain/investiGATR/include/investigatr/state_source.h:27`,
  with status, pose and age) and `navigatr::RobotState`
  (`pi/naviGATR/src/state/robot_state.h:44`, with odometry pose, anchor and
  velocity). This is a glossary case.
- Source of truth is stated once, and it is a good rule. `docs/interfaces.md:7-8`:
  "When this document and the headers disagree, the headers win and this
  document is the bug."

---

## 2. Industry practice

Short summaries, each with its source.

- **Diataxis** ([diataxis.fr](https://diataxis.fr/)). There are four kinds of
  documentation, and each serves a different need:
  - tutorials for learning;
  - how-to guides for tasks;
  - reference for lookup;
  - explanation for understanding.

  Docs "should itself be organised around the structures of those needs",
  and the kinds are kept separate.
- **Google documentation best practices**
  ([docguide](https://google.github.io/styleguide/docguide/best_practices.html)):
  - "A small set of fresh and accurate docs is better than a large assembly of
    documentation in various states of disrepair."
  - Update docs in the same change as the code.
  - Delete dead docs ("Dead docs are bad").
  - Duplication is evil: link, do not copy.
- **README conventions.**
  - Google ([docguide READMEs](https://google.github.io/styleguide/docguide/READMEs.html)):
    every top-level directory of a package has a README with "What is in this
    package or library and what's it used for", points of contact, status,
    usage (copyable commands) and links.
  - standard-readme
    ([spec](https://github.com/RichardLitt/standard-readme/blob/main/spec.md)):
    "Title must match repository, folder and package manager names"; the short
    description is under 120 characters.
- **API reference in code** (Google C++ Style Guide, Comments;
  [cppguide](https://google.github.io/styleguide/cppguide.html#Function_Comments)).
  - A declaration comment covers inputs and outputs, whether the object keeps
    pointer or reference arguments past the call, whether a pointer may be
    null, what happens to output arguments, and performance implications.
  - A class comment documents "the synchronization assumptions the class
    makes". A data member comment covers invariants and sentinel values "not
    clearly expressed by the type and name".
  - "do not be unnecessarily verbose or state the completely obvious";
    "Self-describing code doesn't need a comment."
  - Prefer an enum to a bool argument, and named constants to magic literals.
- **Architecture docs.**
  - ARCHITECTURE.md
    ([matklad](https://matklad.github.io/2021/02/06/ARCHITECTURE.md.html)):
    a bird's eye view, a codemap that answers "where's the thing that does X?",
    architectural invariants ("often ... expressed as an _absence_ of
    something"), and layer boundaries.
    - "the shorter it is, the less likely it will be invalidated".
    - "_Do_ name important files, modules, and types. Do _not_ directly link
      them (links go stale)."
    - "A codemap is a map of a country, not an atlas of maps of its states."
  - C4 model ([c4model.com/diagrams](https://c4model.com/diagrams)): "the
    system context and container diagrams are sufficient for most software
    development teams."
- **Decision records** (Nygard,
  [Documenting Architecture Decisions](https://cognitect.com/blog/2011/11/15/documenting-architecture-decisions);
  [adr.github.io](https://adr.github.io/)).
  - Sections: Title, Context, Decision, Status, Consequences.
  - "one or two pages long", numbered "sequential and monotonically".
  - A superseded record is kept and marked superseded.
  - "large documents are never kept up to date".
- **CHANGELOG** ([Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/)).
  - "Changelogs are _for humans_, not machines."
  - Entries are grouped as Added, Changed, Deprecated, Removed, Fixed and
    Security, with an Unreleased section at the top.
- **Runbooks and playbooks** (Google SRE Workbook,
  [On-call](https://sre.google/workbook/on-call/)). A playbook entry gives
  severity and impact, debugging suggestions and mitigation. It goes out of
  date as fast as the system changes. Deterministic steps should become
  automation.
- **Docs as code.**
  - Write the Docs
    ([guide](https://www.writethedocs.org/guide/docs-as-code/)): docs use the
    same tools as code (issue trackers, version control, plain-text markup,
    code review, automated tests). Teams can "block merging of new features if
    they don't include documentation".
  - Link checking: lychee
    ([repo](https://github.com/lycheeverse/lychee)) has `--offline` ("Only
    check local files and block network requests") and `--include-fragments`
    for anchors. markdownlint
    ([repo](https://github.com/DavidAnson/markdownlint)) rule MD051: "Link
    fragments should be valid".
  - Prose lint: Vale ([docs.vale.sh](https://docs.vale.sh/)) is "a
    command-line tool that brings code-like linting to prose", with custom
    rules.
  - Tested examples: rustdoc
    ([doc tests](https://doc.rust-lang.org/rustdoc/write-documentation/documentation-tests.html))
    runs examples "to make sure that examples within your documentation are up
    to date and working". The C++ equivalent is to take snippets from compiled
    files.
- **Google developer documentation style guide**
  ([highlights](https://developers.google.com/style/highlights)):
  - second person, active voice, present tense;
  - "sentence case" headings, "numbered lists for sequences", code in code
    font, "descriptive link text", "Write for a global audience".
  - On dashes ([dashes](https://developers.google.com/style/dashes)) it
    recommends em dashes. This conflicts with the user's rule (section 3).

---

## 3. Industry practice vs the user's rules

| Industry says | User rule | Resolution for GATR2 |
|---|---|---|
| Google C++: "Almost every function declaration should have comments" | Comments are terse, self-describing labels; skip anything obvious; not AI-ish. | Comment a declaration only when its name and types leave something out. The API checklist (DOC-9) lists what might be missing: units, frame, clock, thread, ownership, blocking, failure. Write it as a label fragment, not a sentence about the function. Google's own "do not state the completely obvious" supports this. |
| Google style: em dashes; smart typography | Plain ASCII only; no em or en dashes; no symbols off the keyboard. | ASCII wins everywhere: comments, docs, commit text. `->` and `--` are allowed because they are typed and carry meaning. CI enforces it (DOC-15). |
| Google style: "conversational", second person | Casual grammar is fine; not hyper formal; not AI-ish. | Compatible. Docs use "you" and imperatives for steps. Comments stay noun-phrase labels (feedback_comment_style.md:26: "a bare label is the target"). |
| standard-readme: TOC, Install, Usage, API, Contributing, License | READMEs are short signposts; deep docs go in docs/. | README = title, one-line purpose, status, contents, commands, links. No TOC, because the file is short. License sits at the repo root once the user decides on one. |
| Diataxis: four separate kinds | "each new pipeline ought to have an explanation that is as concise as possible, as an md" | PIPELINE.md is the concise explanation kind, placed next to the code. Tutorials, how-tos and reference stay in `docs/`. A file is one kind (DOC-2). |
| matklad: name files and types, do not link them because links go stale | The main pipeline must be easy to find (the naviGATR gripe). | A PIPELINE.md code map names file and symbol, without line numbers. Relative links are allowed because CI checks every link and every named symbol (DOC-15, DOC-16), so stale links fail the build. |
| Nygard ADRs for significant decisions | "Understand the problem first"; interfaces are extracted, not invented; no speculative abstraction. | Write short records only for decisions that change a boundary or contract, or that the user rules on. The Context section is the "what is this truly for" question. No template ceremony beyond the five fields. |
| Keep a Changelog for releases | No speculative abstraction, applied to docs as well. | There are no releases, so there is no repo CHANGELOG. Versioned contracts (brain link v4, sensor frame v2, `navigatr.inspect/2`, `gatr2.capture/1`) each keep a "Version history" section in their reference doc. git log covers the rest. |
| C4: context and container diagrams | The whole system explains its pipeline. | The root PIPELINE.md is the context-plus-container view: devices, programs, links. Component PIPELINE.md files are the component view. No code-level diagrams. |
| SRE playbooks | Degrade, do not hang; bench bring-up is frequent. | Bring-up and bench docs follow one how-to shape with a symptom-to-check table (DOC-6). They are not paging playbooks. |
| Docs as code, block merges without docs | CI will be rebuilt around the standard. | A `docs` CI job runs the link, anchor, ASCII, symbol and README-command checks. It blocks merges. Spelling is informational only (DOC-15). |

User rules not covered by industry practice, and kept as they are:
- no emojis;
- casual grammar;
- "Formats vs requirements: state a format only where it is not already
  self-evident" (feedback_comment_style.md:20);
- directories carry a README stating intent, with no empty placeholder files
  (feedback_conventions.md:31).

Outdated memory note: feedback_design_principles.md:15 says "no XML config".
naviGATR is configured by XML and the conventions note accepts XML
(feedback_conventions.md:36). The standard should follow the code.

---

## 4. Proposed documentation rules

Rule IDs are DOC-n. "Check" says how each rule is enforced: CI means a script
in the rebuilt CI, review means the PR reviewer checks it.

### 4.1 Doc types and where they live

**DOC-1. Every doc is one of these types and lives in the stated place.**

| Type | File | Where | Purpose | Size |
|---|---|---|---|---|
| Signpost | `README.md` | repo root; each platform folder (`brain/`, `pi/`, `pico/`, `pcb/`, `bench/`, `docs/`); each component root (`brain/actuGATR/`, `pi/naviGATR/`, `pico/aggreGATR/`, `translaGATR/`, `pi/naviGATR/spectaGATR/`, ...) | What this folder is, where to go next | at most 40 lines |
| Pipeline | `PIPELINE.md` | repo root (whole system); each component that runs a pipeline; each sub-pipeline folder | How data flows, and where in the code | target 60 lines, at most 120 |
| Reference | `docs/<topic>.md` | cross-component contracts in root `docs/`; component-only reference in `<component>/docs/` | Lookup: wire formats, registries, config schemas, feeds | one topic per file |
| How-to | `docs/<task>.md` | same rule as reference | Do one task: setup, bring-up, calibration, bench test | one task per file |
| Explanation | `docs/<topic>.md` | same rule as reference | Why and how, beyond what PIPELINE.md covers (algorithms, coordinates) | one topic per file |
| Decision record | `docs/decisions/NNNN-short-title.md` | root `docs/decisions/` | One boundary or contract decision | one page |
| Glossary | `docs/glossary.md` | root | One line per term | - |
| Validation status | `docs/validation.md` | root | What has run where, with evidence | one table |
| Iteration README | `pcb/<Board>/<Board>_vN/README.md` | existing PCB template | Changes, status, files | - |
| Vendored dependency | `<component>/third_party/README.md` | existing template | Upstream, version, commit, license, kept files, used by | - |

Decision for the user: where component-only deep docs live. Today the Brain
library docs sit in root `docs/` (`docs/actugatr.md`), naviGATR docs sit in
`pi/naviGATR/docs/`, and the naviGATR registries sit in root `docs/`
(`docs/navigatr_resources.md`). My recommendation is component-local
`<component>/docs/`, so that a folder carries its own explanation, as the user
asked. Root `docs/` then keeps cross-component material only: `interfaces.md`,
`hardware.md`, `naming.md`, `glossary.md`, `validation.md`, `decisions/` and
`contributing/`. The other option is one root `docs/` with a subfolder per
component. Either option satisfies "deep docs go in docs/". Mixing both, as
the repo does now, is the part to stop.

**DOC-2. One file, one type.** Do not mix a tutorial, a reference and an
explanation in one file. Example of a mixed file:
`pi/naviGATR/README.md` combines a signpost, the run diagram, an
implementation-coverage list, the config layout, build commands and a run
reference. Check: review.

### 4.2 Required sections per type

**DOC-3. Signpost README.** Sections in this order:

1. `# <folder name>`. Title equals the folder name, spelled the same way
   (for example `# naviGATR`, `# aggreGATR`).
2. One line of purpose, under 120 characters.
3. `Status:` line (DOC-13).
4. `Pipeline: [PIPELINE.md](PIPELINE.md)` when the component has one.
5. Contents: one line per subfolder or key file.
6. Build and test commands: only commands that CI runs literally, or blocks
   marked `manual` (hardware steps).
7. Links to deep docs.

Check:
- CI checks the title against the folder and the line count;
- CI checks that each fenced command block labeled `sh` appears in a CI step
  or carries a `manual` marker (DOC-16);
- review covers the rest.

Current models: `brain/README.md`, `brain/actuGATR/README.md`,
`pico/README.md`, `bench/README.md`.

**DOC-4. PIPELINE.md.** Uses the template in 4.8. Required sections, in order:

1. purpose line (the problem it solves);
2. Status;
3. Input;
4. Output;
5. Stages (one ASCII diagram in a `text` fence);
6. Code map;
7. Inside (links to sub-pipeline PIPELINE.md files);
8. Rules (invariants, at most 6).

Optional:
- "Not here", for what the pipeline deliberately does not do (matklad's
  "absence" invariants);
- "Change points": where a season-specific implementation plugs in. This
  serves the user's first principle, "Implementations within the pipeline of
  course have to be specific (e.g sensors, etc..) however where they live and
  work in must be generic to survive seasons".

**DOC-5. Reference doc.** Sections:

1. one-line scope;
2. `Source of truth:` line naming the header or file that wins when the doc
   disagrees (precedent: `docs/interfaces.md:7-8`);
3. the reference tables;
4. `Version history`, for versioned contracts only (see section 3).

Examples live in `Examples` subsections and follow DOC-16.

**DOC-6. How-to and bring-up doc.** Sections:

1. `Goal` (one line);
2. `Needs` (hardware, build, config file);
3. numbered `Steps`, each with the expected result;
4. `Check` (what success looks like);
5. `Troubleshooting` (a symptom-to-check table);
6. `Last run on hardware:` date and setup, or `never`.

Current model: `bench/rs485_link/README.md` (Wiring, Pi side, Brain side,
Success). One task per file. A how-to links to the reference instead of
restating it.

**DOC-7. Explanation doc.** Sections:
- `Problem`;
- `Approach`;
- `Limits` (what it does not model);
- optionally `Rejected`, for alternatives that were rejected.

Precedent: `localization_fusion.md` ("what the fusion estimator does not
model", per `pi/naviGATR/README.md:51`).

**DOC-8. Decision record.** Five fields from Nygard:
- `# NNNN. Title`;
- `Status: proposed | accepted | superseded by NNNN`;
- `Context` (the forces; "what is this truly for");
- `Decision` ("We will ...");
- `Consequences` (good and bad).

One page. Numbers are never reused. A superseded record is kept. Write one
only for decisions that change a boundary, a contract or a repo-wide
convention, or that the user rules on. Candidates from this repo:
- the -GATR renames (`docs/naming.md:22-53`);
- -Werror vs no -Werror (requirements.md:29-31);
- component-local vs root docs (DOC-1);
- sensor frame v2 identity.

### 4.3 Comment rules for code (C++ and JS)

**DOC-9. Public API comment rule.** A comment on a public declaration (a
header in `include/` or any `.h` another folder includes) states the items
below only when the name and types do not already say them:

- **Units**: prefer a suffix (`_ms`, `_m`, `_deg`, `_dps`) or an alias
  (`Seconds`, `Meters`, per `investigatr/geometry.h:11-14`). Comment only
  when neither is possible, for example raw wire ints: `frames.h:64`
  `int32, millidegrees per second`.
- **Frame**: field or robot or camera, when a pose or vector crosses frames.
  Example: `state_source.h:29` `// robot origin, field frame; only with kValid`.
- **Clock domain**: host, device or Brain, when a time crosses components.
  Prefer a tagged type (`core/time.h`).
- **Thread**: which thread may call it. Use the house suffix: `; any thread.`
  (`system.h:240`) or `; estimation worker only.`
- **Ownership and lifetime**: whether a pointer or reference argument is kept
  past the call, and whether a returned pointer may be null
  (`system.h:240` "null when none").
- **Blocking**: whether it can block and the bound, for example "at most
  5 ms of backpressure".
- **Failure**: what happens on bad input or no data (returns false, keeps the
  previous value, refuses). Example: `motion.h:150`
  `// Active command -> kCancelled. No effect once terminal.`

Form: label fragments joined with `;`, present tense, no "This function...",
no rationale. When a rule needs a paragraph, the paragraph belongs in
PIPELINE.md or a doc, and the comment says `See PIPELINE.md` in three words.
Check: review, plus a CI line-count warning (DOC-10).

**DOC-10. File header comment.** Line 1 is `// <filename>`, which 93 percent
of files already follow. After it come at most 5 lines saying what the file
holds. No pipeline narrative, threading essay or lifecycle story; those go in
PIPELINE.md. A file that is a pipeline entry point adds one line:
`// Pipeline: see pi/naviGATR/PIPELINE.md`. Check: CI warns when a leading
comment block is over 8 lines. The warning is informational at first, because
36 naviGATR headers exceed it today. After the naviGATR cleanup it becomes
blocking.

Decision for the user: keep the `// <filename>` first line. It is consistent
in 426 files and costs nothing, but it does restate the obvious, which the
comment-style rule argues against. My recommendation is to keep it: removing
it is churn, and it identifies a snippet when code is pasted or printed.

**DOC-11. Implementation comments.**
- A comment labels a non-obvious step, sentinel or invariant.
- It never narrates, and never justifies code by pointing at tests (compare
  `system.cpp:1027-1028`).
- A magic literal becomes a named constant, and a bool argument becomes an
  enum, instead of getting a comment (Google C++, "Function Argument
  Comments").
- `TODO` is allowed only with an owner or issue: `// TODO(name): ...`. The
  repo has none today, which is fine.
- JS follows the same rules. JSDoc is not required. Message and document
  shapes are documented once, in the reference doc (`inspection.md`), and JS
  headers link to it instead of restating it.

### 4.4 Pipelines and structure

**DOC-12. What counts as a pipeline, and where its PIPELINE.md goes.**
- A pipeline is code with a defined input, a defined output and ordered
  stages, reached from one entry point (a loop, a cycle function or an
  executor).
- Libraries of types, math or contracts are not pipelines. They get a README
  only if they are a top-level component; `translaGATR` is an example.
- A PIPELINE.md sits in the folder that holds the pipeline's entry point.
- Every PIPELINE.md is linked from its parent PIPELINE.md under `Inside`,
  which gives a tree from the root.
- If a pipeline's code map needs more than about 8 rows, or points into more
  than 3 unrelated folders, record that as a structure defect and fix it under
  the structure rules. Do not paper over it with more docs.

Proposed PIPELINE.md set, for the application plan:

| PIPELINE.md | Entry point today |
|---|---|
| `/PIPELINE.md` (whole system) | devices and links: sensors -> aggreGATR -> naviGATR <-> communiGATR -> actuGATR -> motors; translaGATR is the contract; spectaGATR observes |
| `pico/aggreGATR/PIPELINE.md` | `src/main.cpp` `loop()` |
| `pi/naviGATR/PIPELINE.md` | `src/app/main.cpp` `main`; `src/runtime/system.cpp` `System::build`, `estimationCycle`, `fieldCycle`, `reportingCycle` |
| naviGATR localization | `LocalizationExecutor::operator()` (`src/runtime/localization_stage.cpp`) |
| naviGATR world estimation (apriltag) | `AprilTagWorldEstimator::run` (`src/impl/world_estimation/`) |
| naviGATR brain link (request to reply) | `brain_link_commands.cpp` and `brain_link_publisher.cpp` |
| naviGATR inspection feed | `src/inspection/` |
| `pi/naviGATR/spectaGATR/PIPELINE.md` | `app.js`: feed -> store -> render loop and panels |
| `brain/PIPELINE.md` | StateSource (communiGATR) -> reference -> plan (investiGATR) -> follow -> drive (actuGATR) -> motors |
| `brain/communiGATR/PIPELINE.md` | `Client::poll`: session -> profile -> field docs -> state polls. The `Layers` block at `docs/communigatr.md:39-56` moves here. |

Where the sub-pipeline files go depends on the naviGATR restructure (today
localization is split between `src/runtime/` and `src/impl/localization/`).
Capture and the Brain profile apply are lifecycles, not pipelines. They stay
in reference or explanation docs.

**DOC-13. Status line and validation record.**
- Levels, fixed vocabulary:
  - `designed`: code or doc exists; no automated test.
  - `host-tested`: host tests pass (unit, sim, fake peers, synthetic rig).
  - `bench-tested`: ran on the real device or devices on a bench; date and
    setup recorded.
  - `robot-validated`: ran on the robot in its operating conditions; date
    recorded.
- Value markers, already in use: `PLACEHOLDER` (not measured),
  `PROVISIONAL` (measured once, not confirmed) and `MEASURED`. UNMEASURED and
  UNVERIFIED fold into PLACEHOLDER.
- `docs/validation.md` is the single record. Each row has: item, level,
  evidence, open checks. Evidence is the date, commit, setup, and the log or
  capture bundle location. READMEs and PIPELINE.md carry one `Status:` line
  that links to it. The scattered lists move into it:
  - `brain_setup.md:278-298`;
  - `communigatr.md:1101-1126`;
  - `setup.md:658-671`;
  - `interfaces.md:44-50`;
  - `pi_camera_setup.md` "Not verified on hardware";
  - `aggreGATR/README.md:209-211`.
- Never raise a level on host or synthetic evidence. Precedents:
  `pi/naviGATR/README.md:166-167` ("Synthetic tests and desktop browser checks
  do not establish Pi camera performance") and `tools/perf/README.md:3`.
- Never hard-code test counts in docs.

Check: CI rejects a `Status:` value outside the four levels. Review checks
the evidence.

### 4.5 One fact, one place

**DOC-14. Single source of truth and duplication.**
- Order of authority: code and headers, then reference docs, then PIPELINE.md
  and explanation, then READMEs. When a lower one disagrees, the lower one is
  the bug.
- A fact is written once and linked from everywhere else. A parent
  PIPELINE.md may name a child's stages at one level only, as a diagram box,
  and never repeats the child's detail.
- Current duplicates to collapse: F7 (the naviGATR pipeline three times; the
  apriltag sub-pipeline three times) and F8 (Pico command rules three times).
  The root README's naviGATR wire paragraph (`README.md:40-50`) should become
  one link.
- Headers do not repeat PIPELINE.md (DOC-10).

Check: review. The renames showed that duplicated prose drifts first (F3, F7).

### 4.6 Doc review and checks

**DOC-15. CI `docs` job.** Blocking unless marked otherwise. Checks:

1. Relative links and `#anchors` resolve against **git-tracked paths with
   exact case**. Use lychee `--offline --include-fragments`, or the roughly
   80-line script used in this survey. On Windows, never trust the
   filesystem (F1).
2. ASCII only in `.md` files and in comments of first-party source. Today's
   only violations are the 7 PCB middle dots (F10).
3. Every folder named in a PIPELINE.md `Inside` section has a PIPELINE.md;
   every README and PIPELINE.md has a valid `Status:` value.
4. A small markdownlint subset: MD001 (heading increment), MD040 (fence
   language, so every diagram is `text`), MD051 (fragments) and MD042 (empty
   links).
5. Informational, not blocking:
   - external URLs checked weekly;
   - spell check with a project word list (the -GATR names, `navigatr`,
     BNO08X, ...).

   No prose-style linter (Vale) until a real need shows up. That follows "no
   speculative abstraction".

**DOC-16. Examples and commands are tested.**
- Code snippets in docs are either copied from a compiled file, with the
  source path named under the block, or labeled `sketch`. Precedent:
  `motion.h:5-6` carries the first two calls of `docs/actugatr.md:38-39`
  (the landmark name differs), so today the two copies can drift.
- `sh` command blocks in READMEs equal a CI step, or carry `manual`
  (hardware).
- Every file and symbol named in a PIPELINE.md code map must exist. CI greps
  for the symbol in the named file.

This replaces line numbers in docs, which go stale on every edit.

**DOC-17. Review rules.**
- Docs change in the same PR as the behavior they describe. A PR that changes
  a pipeline's input, output, stages or entry points updates that
  PIPELINE.md.
- A rename sweeps prose too: grep for the old name and the old anchor text,
  not only paths. The 2026-09-28 rename left F2 and F3 behind.
- Dead docs are deleted, not kept "for history". git keeps history, and
  decision records keep decisions.
- Re-read the root and component PIPELINE.md files at each season start.
  This is when the "survive seasons" check happens: every season-specific
  item should sit under "Change points".
- Doc prose follows the user's style: ASCII, casual, short, not AI-ish. Use
  sentence-case headings, numbered lists for sequences, code font for code,
  and descriptive link text (Google style), without em dashes.

### 4.7 Names and glossary

**DOC-18. Names in prose.**
- A component is written with its -GATR spelling: naviGATR, aggreGATR,
  spectaGATR, and so on.
- `navigatr` appears only in code font, for the namespace, executable or
  target (`docs/naming.md:35-38`). "Navigatr" is retired (F9).
- A term that means different things in two components is qualified by
  namespace in docs, for example `investigatr::RobotState` vs
  `navigatr::RobotState`.

**DOC-19. Glossary.** `docs/glossary.md`, alphabetical, one line per term:
term, meaning, and the defining symbol and file. Only terms that cross
components or that a newcomer would misread go in. Candidates, with defining
files verified:

| Term | Defined at |
|---|---|
| Resource, ResourceStore | `pi/naviGATR/src/resources/resource_store.h` |
| Sensor, MeasurementRecord, Provenance | `pi/naviGATR/src/core/records.h` (:94, :61) |
| Observation, Estimator | `pi/naviGATR/src/contracts/localization.h` |
| Stage, slot, executor | `pi/naviGATR/src/runtime/system.h` |
| FunctionRegistry | `pi/naviGATR/src/core/function_registry.h:27` |
| RobotState (Pi) | `pi/naviGATR/src/state/robot_state.h:44` |
| RobotState (Brain), StateSource | `brain/investiGATR/include/investigatr/state_source.h:27`, `:38` |
| RobotStateFeed | `pi/naviGATR/src/state/robot_state_feed.h:85` |
| FieldState, FieldSnapshot | `pi/naviGATR/src/state/field_state.h:62`, `system.h:134` |
| Field, Reference, PathPlanner | `brain/investiGATR/include/investigatr/` (`field.h:59`, `reference.h:16`, `planner.h:46`) |
| Robot profile | `translaGATR/link_documents.h:114 RobotProfileDoc` |
| Session, placement, pi_instance | `docs/interfaces.md` "Sessions and dedupe" |
| boot_id, acq_epoch, imu_epoch | `translaGATR/frames.h` sensor frame block |
| Epoch (odometry), anchor | `pi/naviGATR/docs/coordinates.md` |
| Landmark, tag mount, target, nominal, observed | `pi/naviGATR/docs/landmarks.md:8-39` |
| Host clock, device clock | `pi/naviGATR/src/core/time.h:2-4` |
| StationaryWindow | `pi/naviGATR/src/impl/localization/stationary_window.h:71` |

### 4.8 Templates

**Template A: PIPELINE.md.** Everything in angle brackets is replaced. Keep it
to about 60 lines.

````markdown
# <Name> pipeline

<One line: the problem this pipeline solves.>

Status: <designed | host-tested | bench-tested | robot-validated>, see [validation](<path>/docs/validation.md)

## Input

- <what>, from <who>: `<type>`; <units, frame, clock if not in the type>

## Output

- <what>, to <who>: `<type>`

## Stages

```text
<input> -> <stage> -> <stage> -> <output>
```

## Code map

| Step | File | Symbol |
|---|---|---|
| <entry> | `<path>` | `<Function>` |

## Inside

- [<sub-pipeline>](<folder>/PIPELINE.md)

## Rules

- <invariant, often an absence: "never sends unasked">

## Change points

- <where a season-specific implementation plugs in, and how it is selected>
````

**Template A, filled in: `pi/naviGATR/PIPELINE.md` (draft).** Built only from
verified code and docs. The code map uses file plus symbol, without line
numbers (DOC-16). The Status value needs the user's confirmation. Sub-pipeline
paths are placeholders until the restructure decides the folders.

````markdown
# naviGATR pipeline

Where the robot is and where field objects are, from Pico and camera measurements; answers the Brain.

Status: host-tested, see [validation](../../docs/validation.md)

## Input

- Pico sensor and status frames over UART: `pico_telemetry` resource
- Camera frames: `libcamera_camera` resource, Pi builds only
- Brain requests, brain link v4 over RS-485 or USB: `brain_link` CommandCollection
- Robot profile from the Brain; Pi devices and tuning from the XML config

## Output

- Brain replies: robot pose, health, field documents
- Inspection feed `navigatr.inspect/2` for spectaGATR
- Capture bundles `gatr2.capture/1`

## Stages

```text
build          registerAll -> System::build
                 make_resources -> make_sensors -> CommandCollection
                 -> make_localization -> make_world_estimation
                 -> TargetResolution -> Publishing        any failure: nothing runs

estimation     resources -> sensors -> commands -> localization
worker         -> hand sensors to field worker (latest wins)
(loop rate)    -> target resolution -> publishing

field worker   world estimation on newest sensors -> FieldSnapshot
```

## Code map

| Step | File | Symbol |
|---|---|---|
| Program start | `src/app/main.cpp` | `main` |
| Register implementations | `src/runtime/register_all.cpp` | `registerAll` |
| Build | `src/runtime/system.cpp` | `System::build` |
| Stage builders | `src/runtime/*_stage.cpp` | `make_resources`, `make_sensors`, `make_localization`, `make_world_estimation` |
| One estimation cycle | `src/runtime/system.cpp` | `System::estimationCycle` |
| Target and reply | `src/runtime/system.cpp` | `System::reportingCycle` |
| One field cycle | `src/runtime/system.cpp` | `System::fieldCycle` |
| Workers | `src/runtime/system.cpp` | `System::estimationWorker`, `System::fieldWorker` |

## Inside

- [Localization](<folder>/PIPELINE.md)
- [World estimation, apriltag](<folder>/PIPELINE.md)
- [Brain link](<folder>/PIPELINE.md)
- [Inspection feed](src/inspection/PIPELINE.md)

## Rules

- One writer per mutable state; readers get snapshots
- Build is all or nothing
- The Pi never sends unasked; one reply per request at most
- A slow camera never delays localization or a Brain reply

## Change points

- New sensor or device: a resource or sensor type plus its register line; no coordinator change
- New field estimator: a WorldEstimation type plus its register line
- Robot geometry: the Brain robot profile, not Pi code
````

Sources for the draft:
- stages: `system.cpp:194-335` and `:1022-1031`; `main.cpp:94`, `:98`,
  `:160`, `:167`;
- rules: `system.h:17-20` and `:37-45`, `docs/interfaces.md:38-39`,
  `architecture.md:339-341`;
- change points: `architecture.md:182-185` ("another selectable
  implementation needs its class, a registration line, and a build entry,
  not a coordinator change") and `pi/naviGATR/README.md:16-21`.

**Template B: signpost README.**

````markdown
# <folder name>

<One line, under 120 characters: what this is.>

Status: <level>, see [validation](<path>/docs/validation.md)
Pipeline: [PIPELINE.md](PIPELINE.md)

- `<subfolder>/`: <label>
- `<key file>`: <label>

```sh
<build and test commands CI runs>
```

Docs: [<topic>](<path>), [<topic>](<path>)
````

---

## 5. Application notes (docs side only)

Order, with behavior unchanged:

1. Fix F1 to F6 and F10. These are mechanical and take one PR.
2. Add the root `PIPELINE.md`, `pi/naviGATR/PIPELINE.md` and
   `pico/aggreGATR/PIPELINE.md`. Add
   `Pipeline: [PIPELINE.md](PIPELINE.md)` near the top of each README.
3. Cut `pi/naviGATR/README.md` and `pico/aggreGATR/README.md` down to
   signposts:
   - "What runs" moves into PIPELINE.md;
   - implementation coverage moves into `docs/validation.md`;
   - the run options move into `docs/configuration.md`;
   - the Pico rules link to `docs/interfaces.md`;
   - the USB monitor becomes a how-to.
4. Collapse the duplicates (F7, F8). Trim naviGATR header comments to labels
   (DOC-10), starting with `system.h`, `brain_profile_builder.h`,
   `tracking_wheel_motion.h`, `brain_link_commands.h` and
   `weighted_planar_fusion.h` (65, 60, 60 and 58 lines).
5. Add `docs/validation.md`, `docs/glossary.md`, `docs/decisions/`, a
   `translaGATR/README.md` and a `spectaGATR/README.md`.
6. Turn on the CI `docs` job (DOC-15, DOC-16). Header length stays a warning
   until step 4 is done.

Out of scope for this survey, noted for other sections: naviGATR ships Python
tools (`tools/calibrate_camera.py`, `tools/viewer/viewer_tests.py`,
referenced at `pi/naviGATR/README.md:276`). The C++-only rule
(feedback_conventions.md:36) says each needs a justification.

## 6. Open decisions for the user

1. Deep docs: component-local `<component>/docs/`, or root `docs/<component>/`
   (DOC-1). The recommendation is component-local.
2. Keep the `// <filename>` first line (DOC-10). The recommendation is to keep
   it.
3. Size limits: README at most 40 lines, PIPELINE.md 60 target and 120
   maximum, file header at most 5 label lines after the filename. Adjust if
   these feel wrong.
4. A repo LICENSE: the memory mentions a possible MIT release; nothing is in
   the repo.
5. Whether `docs/validation.md` replaces the per-doc "not validated" lists
   outright (recommended), or they stay with a link.

## Sources

- Diataxis: https://diataxis.fr/
- Google documentation best practices: https://google.github.io/styleguide/docguide/best_practices.html
- Google README guidance: https://google.github.io/styleguide/docguide/READMEs.html
- standard-readme spec: https://github.com/RichardLitt/standard-readme/blob/main/spec.md
- Google C++ Style Guide, Comments: https://google.github.io/styleguide/cppguide.html#Function_Comments
- ARCHITECTURE.md (matklad): https://matklad.github.io/2021/02/06/ARCHITECTURE.md.html
- C4 model diagrams: https://c4model.com/diagrams
- Nygard, Documenting Architecture Decisions: https://cognitect.com/blog/2011/11/15/documenting-architecture-decisions
- ADR resources: https://adr.github.io/
- Keep a Changelog 1.1.0: https://keepachangelog.com/en/1.1.0/
- Google SRE Workbook, On-call: https://sre.google/workbook/on-call/
- Write the Docs, Docs as Code: https://www.writethedocs.org/guide/docs-as-code/
- lychee: https://github.com/lycheeverse/lychee
- markdownlint: https://github.com/DavidAnson/markdownlint
- Vale: https://docs.vale.sh/
- rustdoc documentation tests: https://doc.rust-lang.org/rustdoc/write-documentation/documentation-tests.html
- Google developer documentation style guide: https://developers.google.com/style/highlights and https://developers.google.com/style/dashes
