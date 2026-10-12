# Survey: non-C++ code, build and config files

Scope: JavaScript (spectaGATR viewer and its test/perf helpers), Python tools,
CMake / Make / PlatformIO, XML configuration, shell scripts, repository hygiene.
Snapshot of `main` at ae457ee (working tree clean), 2026-09-28. Read-only survey;
nothing in the repo was changed.

Method:
- Read every JS, Python, shell, CMake, Make, PlatformIO and config XML file
  listed below; the vendored three.js and PROS `common.mk` only skimmed.
- Measured with tools in a scratch venv (Python 3.14.3), none installed system-wide:
  ruff 0.16.9, lizard 1.24.0, shellcheck 0.11.0 (the `shellcheck-py` wheel),
  cmake-lint (cmakelang), and clang-format 22.1.8 in JavaScript mode on copies.
- Note: clang-format 22.1.8 is not at `/c/msys64/ucrt64/bin` on this box. The one
  found is `C:\Users\mbes1\AppData\Roaming\Python\Python314\Scripts\clang-format.exe`
  (it runs; version 22.1.8).
- Citations are `path:line` from the repo root with a short quote. Quotes are
  verbatim unless marked "(trimmed)".

Rule ids are prefixed per area (JS, PY, CM, XML, SH, REPO) so they can be
merged into the main standard. Items that need the user's call are collected
in "Decisions for the user" at the end.

---

## 0. Inventory

| Where | Language | Files | Lines | Notes |
|---|---|---|---|---|
| `pi/naviGATR/spectaGATR/` | JS (ES modules) | 17 | 7,884 | plus `index.html` 258, `style.css` 655; compiled into the runtime binary |
| `pi/naviGATR/third_party/three/` | JS (vendored) | 2 | - | three.js r170, pinned, MIT |
| `pi/naviGATR/tools/viewer/` | Python + JS + HTML | 4 py, 1 js, 1 html | 2,419 py, 686 js | browser tests, stub server, CDP client, fixture writer |
| `pi/naviGATR/tools/` | Python | 2 | 466 | camera calibration (needs OpenCV + numpy) and its test |
| `pi/naviGATR/tools/perf/` | C++ + JS + Bash | 12 cpp/h, 1 js, 1 sh | 4,208 cpp, 250 js, 265 sh | perf harness |
| `pi/naviGATR/tools/*.sh` | Bash | 2 | 452 | service installer, viewer smoke test |
| `bench/rs485_link/` | Python | 1 | 91 | Pi-side RS-485 transmit test |
| CMake | CMake | 10 files | ~830 | brain (4), naviGATR (3 + `cmake/embed_assets.cmake`), translaGATR tests, pico tests |
| PROS | Make | `brain/gatr2_brain.mk` + 2 x (Makefile, common.mk, project.pros) | - | `common.mk` identical in both apps |
| PlatformIO | ini | `pico/aggreGATR/platformio.ini` | 25 | two envs |
| `pi/naviGATR/config/` | XML | 34 | ~2,750 | 6 are `.xml.in` templates |

Not tracked but present locally: `build-printables/generate.py` and
`build-printables/render.ps1` (see REPO-5).

---

## 1. JavaScript (spectaGATR, unit_tests.js, perf_probe.js)

### 1.1 What is already consistent (keep it)

- Plain ES modules, no framework, no bundler, no npm. One bare specifier,
  `three`, resolved by an import map:
  `pi/naviGATR/spectaGATR/index.html:10` `<script type="importmap">{"imports":{"three":"./vendor/three.module.min.js"}}</script>`.
- three.js is vendored and pinned with version, license and use recorded:
  `pi/naviGATR/third_party/README.md:41` "Version: r170".
- Naming: files `snake_case.js`; classes PascalCase (`Feed`, `KeyedTable`,
  `PoseSmoother`); functions camelCase; module constants UPPER_SNAKE with unit
  suffix (`pi/naviGATR/spectaGATR/feed.js:18` `const RECONNECT_MIN_MS = 500;`).
  Wire document fields keep the wire's snake_case with units (`x_m`, `age_ms`).
- `const`/`let` only (0 `var`), strict equality only (0 `==`/`!=`), 0 `console.*`
  calls, 0 non-ASCII characters in all 19 JS files (measured).
- State model: WebSocket handlers only parse and store latest-wins slots plus a
  version counter; the animation frame applies what changed.
  `pi/naviGATR/spectaGATR/feed.js:2` "Handlers only parse and store".
  `pi/naviGATR/spectaGATR/app.js:426` `if (feed.stateVersion !== seen.state && s) {`.
- DOM updates are keyed and write-on-change through one helper module:
  `pi/naviGATR/spectaGATR/dom.js:19` `export function setText(node, text) {`,
  `dom.js:53` `export class KeyedTable {`. `innerHTML` appears once, to clear:
  `camera_panel.js:174` `this.select.innerHTML = '';`.
- Error handling: parsers throw specific `Error`s
  (`bundle.js:98` "ZIP entry ${name}: CRC-32 ... expected ..."), UI edges catch and
  show them (`app.js:543-545` `capturePanel.say('cannot replay: ' + ...)`), socket
  handlers never throw and route to one recorder that tests read
  (`app.js:83-91`, `#status[data-errors]`). The only silent catches are per-viewer
  storage, commented (`graphs.js:828` "storage can be unavailable; defaults then").
- Pure logic is already written as exported functions, which is the functional
  style the user asked for: `transforms.js` (all functions), `bundle.js`
  `parseCsvRows`/`readZip`, `smoothing.js:26` `export function poseFreshness(ageMs, sinceMs) {`.
  Classes are used where an object owns state across calls (`Feed`, `PoseSmoother`,
  `KeyedTable`, `FieldScene`).
- Performance work is backed by a harness (`tools/perf`, `perf_probe.js`) and the
  patterns are named: preallocated trail queue (`app.js:108`), reused per-frame
  options (`app.js:1025` "reused every frame: no per-frame allocation"), panel rate
  cap (`app.js:40` `PANEL_HZ = 10`), render budget (`app.js:1038`), hidden panels
  unsubscribe (`app.js:278-287`).

### 1.2 Problems found

Size and complexity (lizard, CCN = cyclomatic complexity):
- Two modules carry most of the page: `field_scene.js` 1,582 lines, one class
  `FieldScene` with 44 methods (`field_scene.js:297`); `app.js` 1,157 lines of
  top-level code mixing status bar, dock tabs, preview budget, replay, panels
  and the frame loop.
- Functions over CCN 30: `app.js:852` `renderStatus` CCN 64 / 126 lines,
  `link_panel.js:29` `readinessOf` CCN 46, `camera_panel.js:363` `redraw` CCN 44,
  `graphs.js:408` `draw` CCN 41 / 162 lines, `capture_panel.js:251` `render` CCN 40,
  `diagnostics.js:94` `render` CCN 35. About 30 of 496 functions exceed CCN 15 or
  80 lines.
- `app.js` module state is ~25 `let`s spread over the file (`app.js:109`, `:148`,
  `:239`, `:309-311`, `:358-366`, `:530-537`, `:739-741`, `:1030-1031`). Some are
  read by functions declared earlier (`updateSubscription` at `app.js:283` reads
  `mode`, declared `app.js:358` `let mode = 'live';`). Safe today because those
  functions only run from events after load; a synchronous call during start-up
  would hit the TDZ. Readability cost either way.

Duplicated helpers (same job, several copies, some already diverged):
- Angle wrap x3: `transforms.js:11` `export function wrapDeg(d) {`,
  `series.js:243` `function wrapDeg(d) {`, `smoothing.js:45` `function wrap180(d) {`.
- `el()` x2: `dom.js:7` and `camera_panel.js:59` `function el(tag, cls, text) {`.
- `kvText()` x2 with different results: `instrumentation_panel.js:31-33` returns
  `'n/a'` for a non-object and formats 3 decimals; `latency_panel.js:14-16` returns
  `String(obj)` and formats 2 decimals.
- `field_scene.js:36` `export { attitudeStatus, enumName };` re-exports two
  `transforms.js` helpers, so there are two import paths for the same function.

Naming drift after the rename (docs/naming.md renamed viewer to spectaGATR):
- Test hook `window.__navigatr` (`app.js:76`), perf marks `'navigatr:' + type`
  (`feed.js:232`), contract `'navigatr.inspect/2'` (`feed.js:24`), storage key
  `'spectagatr.graphs'` (`graphs.js:824`), test folder `tools/viewer/`,
  script `tools/viewer_smoke.sh`. The wire contract name is fine to keep; the
  page-local names should say spectaGATR.
- `bundle.js` is the capture-bundle (ZIP/CSV) reader, not a build bundle
  (`bundle.js:2` "Reads a capture bundle (gatr2.capture/1)"). In a no-build JS
  tree the name reads like a bundler artifact.
- Unit suffixes are applied unevenly inside one class: `smoothing.js:70`
  `this.periodMs = 33;` next to `smoothing.js:69` `this.dur = 0;` and `this.t0`.

Null handling mixes three styles for the same thing:
`capture_panel.js:262` `lim.max_pre_s ?? 'n/a'`, `app.js:460`
`(prev.configuration || {}).digest`, `index.html:246`
`e && e.message ? e.message : e`. The page already requires import maps
(modern browsers only), so `?.` and `??` are available everywhere.

Formatting has no tool and drifts:
- 2-space indent in `tools/perf/perf_probe.js` (e.g. line 9 `  if (window.__perf) {`),
  4-space everywhere else.
- Lines over 100 columns: 378 in `spectaGATR/` (app.js 58, field_scene.js 69,
  graphs.js 60, replay.js 52), 439 counting `unit_tests.js` (57) and
  `perf_probe.js` (4); longest 283 (`field_scene.js:1231`).

Comments versus the user's comment rule (terse labels, no narration):
- Every module opens with `// name.js` then a 5 to 29 line narrative
  (`app.js:1-22`, `camera_panel.js:1-16`, `field_scene.js:1-29`,
  `smoothing.js:1-14`). Inline rationale paragraphs too: `app.js:302-307`,
  `app.js:1033-1037`, `app.js:1137-1142`, `index.html:235-239`.
- The content is good design documentation; it is in the wrong place. It belongs
  in a spectaGATR PIPELINE.md (see JS-14), with a one-line label left in the file.
- The filename line (`// app.js`) restates what the editor already shows.

Test contract spread across comments: the headless-test contract (the `#status`
`data-*` attributes and `window.__navigatr`) is only described in `app.js:14-22`
and the initial markup `index.html:14`.

Hardcoded field size in the viewer: `field_scene.js:303-304`
`this.fieldCenter = new THREE.Vector3(1.8, 1.8, 0);` / `this.fieldSpan = 3.6;`
and again `field_scene.js:519-520`. They are fallbacks (real size comes from
`hello.fields[].dimensions`, `field_scene.js:524`), but a season value written
twice in code is how season assumptions creep in.

Possible dead compatibility path (PLAUSIBLE, needs a decision): `feed.js:9-11`
"still reads an inspect/1 'snapshot' ... so the page also works against an older
runtime". The page is compiled into the same binary it talks to
(`pi/naviGATR/CMakeLists.txt:141-175`), so an older runtime only meets this page
through a `static_root` override or the perf baseline. It costs branches in
`app.js:868`, `app.js:953` (`s.compat ? 'snapshot' : 'state'`) and `feed.js:412-427`.

### 1.3 Proposed JS rules

JS-1 Platform. ES modules loaded directly by the browser. No framework, no
bundler, no transpiler, no npm, no node needed to build, run or test. Imports are
relative with the `.js` extension, or the one bare name `three`.

JS-2 Vendored libraries. Third-party browser code lives in
`pi/naviGATR/third_party/<lib>/` with a README entry (upstream, version, license,
what is kept, who uses it), as three.js has today. Served under `vendor/`. A
version bump is its own commit.

JS-3 Modules. One concern per module, named for what it exports
(`capture_bundle.js`, not `bundle.js`). A module over ~600 lines or holding two
panels is split. `app.js` keeps only wiring: create parts, the frame loop, and
the order things happen in.

JS-4 Functions first, classes when state lives across calls. Same rule as C++:
a plain object literal is the JS `struct`; an exported function is the default;
a class only when an instance owns state or DOM nodes between calls (`Feed`,
`KeyedTable`, `PoseSmoother`). No class that is only a namespace for functions.

JS-5 Naming. Files `snake_case.js`; classes PascalCase; functions and variables
camelCase; module constants UPPER_SNAKE. Every numeric name with a unit carries
it (`periodMs`, `spanM`, `headingDeg`; constants `STALE_MS`). Fields that mirror a
wire or config document keep the document's exact snake_case name (`age_ms`).
Page-local identifiers use the component name: `spectagatr` / `__spectagatr`.

JS-6 Module state. Mutable module state is declared at the top of the module,
grouped in named objects by concern (for example `live`, `replay`, `dock`), never
declared below the functions that read it.

JS-7 Store, then apply. Network handlers parse, store the newest value, bump a
version and never touch the DOM or scene. The frame loop applies changes by
comparing versions. Existing rule, written down.

JS-8 DOM. Writes go through `dom.js` helpers (`setText`, `setClass`, `KeyedTable`,
`BadgeSet`, `Lines`). Text via `textContent`; `innerHTML` only with a literal.

JS-9 One home per helper. DOM helpers in `dom.js`; math and angle helpers in
`transforms.js`; value formatting in one `format.js` (split out of `transforms.js`,
which today also holds `fmt`, `fmtMs`, `attitudeStatus`, `enumName`). No local
copies and no re-exports.

JS-10 Errors. Parsers throw `Error` with what and where. The UI edge catches and
shows the message. Handlers never throw. No empty `catch` except for per-viewer
conveniences (storage), with a label comment. Global `error` and
`unhandledrejection` stay wired to the recorder.

JS-11 Optional data. Use `?.` and `??` for optional document fields; drop the
`(x || {}).y` and `a && a.b ? a.b : c` forms.

JS-12 Readability first; performance patterns need a number. A preallocated
buffer, cache or budget is allowed when a perf harness result shows the need;
it gets a one-line label naming the measurement. Otherwise write the plain version.

JS-13 Size limits (checked by lizard from Python, no node): function <= 60 lines
and CCN <= 15 for new code; existing offenders listed in the application plan.

JS-14 Docs and comments. Each module starts with one label line (no filename).
The design narrative moves to `spectaGATR/PIPELINE.md`: data flow
(socket -> feed slots -> frame loop -> scene and panels), the test contract
(`#status` `data-*` attributes, `window.__spectagatr`), replay isolation.
Plain ASCII; UI text too (today `app.js:850` builds a middle dot at runtime with
`String.fromCharCode(0xb7)`; use an ASCII separator instead).

JS-15 Formatting. clang-format 22.1.8 (already pinned for C++) formats JS too:
add a `Language: JavaScript` section to the root `.clang-format`, 4-space indent,
column limit 100. Trial on copies: all 17 modules parse with no errors (top-level
await included). With a Google-based JS section, changed lines per file range from
1 (`dom.js`, `latency.js`) to 277 (`app.js`); almost all are line wraps. Two known
wrinkles to settle when the config is written: object-literal spacing
(an LLVM base gives `x_m : value`, a Google base gives `x_m: value`), and matrix
literals like `transforms.js:40-42` need `// clang-format off` / `on` to keep rows.

JS-16 Lint without node. A small stdlib Python checker at repo level (see PY-1)
enforces the grep-able subset: no `var`, no `==`/`!=`, no `console.`, imports only
`./x.js` or `three`, no `innerHTML =` with a non-literal, ASCII only, no second
definition of a `dom.js`/`transforms.js`/`format.js` export name, file header
present. Syntax is checked by the headless browser run that already imports the
modules (`tools/viewer/unit_tests.js:8-18` imports 11 modules directly and the
rest load through `app.js` in the live scenario).

---

## 2. Python

### 2.1 What exists

| File | Deps | Purpose |
|---|---|---|
| `pi/naviGATR/tools/calibrate_camera.py` | OpenCV, numpy (lazy import) | capture JPEGs from the runtime, fit intrinsics, write XML |
| `pi/naviGATR/tools/test_calibrate_camera.py` | OpenCV, numpy | unittest for the above |
| `pi/naviGATR/tools/viewer/viewer_tests.py` | stdlib | headless browser scenarios (unit, live, replay, ...) |
| `pi/naviGATR/tools/viewer/stub_server.py` | stdlib | inspect/2 test double of the runtime's server |
| `pi/naviGATR/tools/viewer/cdp.py` | stdlib | Chrome DevTools Protocol + WebSocket client |
| `pi/naviGATR/tools/viewer/capture_fixture.py` | stdlib | writes capture ZIP fixtures |
| `bench/rs485_link/pi_transmit.py` | pyserial, gpiozero | Pi-side RS-485 bring-up |

### 2.2 Good patterns to keep

- Stdlib-only where possible, said up front:
  `tools/viewer/viewer_tests.py:3` "Python standard library only: no pip, no global installs."
- Heavy deps imported lazily so `--help` works without them:
  `pi/naviGATR/tools/calibrate_camera.py:46-55` `def dependencies(): try: import cv2 ...` (trimmed).
- `main(argv=None)` returning an exit code:
  `calibrate_camera.py:297` `def main(argv=None):` and `:312-313`
  `if __name__ == "__main__": sys.exit(main())`.
- CLI flags kebab-case with the unit in the name:
  `calibrate_camera.py:287` `"--square-size-m"`.

### 2.3 Problems found

- The naviGATR language rule. Memory rule: naviGATR is C++ only, CMake and XML
  accepted, "Do not add Python tooling/scripts inside navigatr". Today
  `pi/naviGATR/` holds 6 Python files (2,885 lines), 3 Bash scripts, 2 JS tools
  outside the viewer, and the viewer itself (JS/HTML/CSS). The viewer is also not
  on the accepted list, though `docs/naming.md:13` names it a component.
- Two implementations of the same client in two languages in one component:
  a Python CDP + WebSocket client (`tools/viewer/cdp.py`, 306 lines) and a C++ one
  (`tools/perf/cdp_bench.cpp` 791 lines plus `perf_net.cpp` 327).
- A Python re-implementation of the runtime's inspect/2 server
  (`tools/viewer/stub_server.py`, 742 lines). Useful for tests without a binary,
  but a second implementation of a C++ contract that can drift; the suite can
  also run against the real binary (`viewer_tests.py:31` `--binary PATH`).
- Auto-install at import time, including past the OS guard:
  `bench/rs485_link/pi_transmit.py:50-51` `[sys.executable, "-m", "pip", "install", package],`
  / `[..., "--break-system-packages", package],`.
- No declared dependencies: no `requirements.txt` anywhere; the calibration
  venv sits in an ignored build folder (`pi/naviGATR/build-camera-tools/venv/`).
- Type hints: 0 annotated signatures out of 148 `def`s across all 7 files.
- Style split: double quotes in `calibrate_camera.py` (153 vs 1),
  `test_calibrate_camera.py`, `pi_transmit.py`; single quotes in the four viewer
  files (e.g. `stub_server.py` 921 vs 6). `pathlib` in the calibration tool,
  `os.path` in the viewer tools (23 uses in `viewer_tests.py`).
- `ruff format` (line length 100) would reformat all 7 files (about 3,000 diff lines).
- ruff lint findings worth fixing (real, not style):
  - `viewer_tests.py:53` `import shutil` unused (F401);
  - `viewer_tests.py:632` `m0 = metrics(b)` never used (F841);
  - `viewer_tests.py:148`, `:169`, `:887`, `cdp.py:150` open files without a
    context manager (SIM115), e.g. `viewer_tests.py:148` `text = open(src, encoding='utf-8').read()`;
  - `viewer_tests.py:394` re-raise without `from` (B904);
  - `cdp.py:291` bare `except Exception: pass` (S110/BLE001).
- Clever over readable:
  `calibrate_camera.py:306` `except (ValueError, OSError, cv.error if "cv" in locals() else ValueError) as error:`.
- Path hacks for sibling imports: `viewer_tests.py:65` `sys.path.insert(0, HERE)`
  then `:68` `import capture_fixture as cf  # noqa: E402`.
- XML edited with regular expressions: `viewer_tests.py:149`
  `text = re.sub(r'file="([^"]+)"', ...)` (trimmed); stdlib `xml.etree` is available.
- Complexity: `capture_fixture.py:124` `synthetic` CCN 28 / 109 lines,
  `viewer_tests.py:959` `scenario_paths` CCN 25 / 105 lines.
- Test naming: `test_calibrate_camera.py` (unittest default pattern) next to
  `viewer_tests.py`; C++ uses `*_gtest.cpp`.

### 2.4 Proposed Python rules

PY-1 Where tooling may live (DECISION D1). Recommended wording:
"A component's runtime and library code is written in the component's language.
Host-side tools that are never built into or loaded by the runtime live under
`<component>/tools/<tool>/`, one folder per tool with a two-line README stating
what it is, how to run it, and why it is not C++. Repo-wide checks live in root
`tools/`." For naviGATR this means `src/` is C++ only; `config/` is XML;
`spectaGATR/` is the one browser app; `tools/` may hold Python and Bash with the
README justification. The strict alternative is in D1.

PY-2 Stdlib only by default. A tool that needs packages (OpenCV) lists them with
pinned versions in `requirements.txt` in its own folder, imports them lazily so
`--help` works, and prints the install command when they are missing. Never
install packages from inside a script.

PY-3 One Python floor: 3.9 (the calibration tool already states it,
`calibrate_camera.py:4`). Raspberry Pi OS Bookworm ships 3.11; this box runs 3.14.

PY-4 Shape of a tool: `argparse`, `def main(argv: list[str] | None = None) -> int`,
`if __name__ == "__main__": sys.exit(main())`. No `sys.path` edits; sibling
modules are imported by running the folder as a package or by keeping one file.

PY-5 Type hints on every function signature. No type-checker gate at first.

PY-6 `pathlib` for paths, `with` for every file, `xml.etree.ElementTree` for XML.

PY-7 Format and lint with ruff (pip, no node), config in root `ruff.toml`:
line length 100, ruff's default double quotes, rules `E, W, F, B, UP, SIM` plus
`PLR0911/0912/0915` as warnings. Same CCN/length limits as JS-13.

PY-8 Tests: `test_<module>.py` with stdlib `unittest`, in the tool's folder.
Tests that drive the C++ runtime run against the real binary in CI; a stub of a
C++ contract is allowed only when the same scenarios also run against the binary.

PY-9 Comments and docstrings follow the comment rule: module docstring = one line
of what it is plus usage; `--help` text is the manual. No narrative docstrings.

---

## 3. CMake, Make, PlatformIO

### 3.1 Good patterns to keep

- Sources and tests listed explicitly almost everywhere:
  `brain/actuGATR/CMakeLists.txt:36` `set(ACTUGATR_TEST_SOURCES`,
  `translaGATR/tests/CMakeLists.txt:35-39` `# Sources under test.` / `set(CODE_UNDER_TEST`.
- Brain libraries are real targets consumed by linking:
  `brain/actuGATR/CMakeLists.txt:22` `target_link_libraries(actugatr PUBLIC investigatr_types)`.
- Comments are the terse labels the user asked for:
  `brain/actuGATR/CMakeLists.txt:6` `# Library.`, `:25` `# Host simulation.`, `:35` `# Test files.`.
- Vendored runtime deps built without our warnings and as SYSTEM includes:
  `pi/naviGATR/CMakeLists.txt:11-13` `# Vendored, built without our warning set.`
- Option names carry the component prefix: `NAVIGATR_BUILD_TESTS`,
  `NAVIGATR_WITH_LIBCAMERA`, `NAVIGATR_BUILD_PERF_TOOLS`, compile definition
  `NAVIGATR_HAVE_LIBCAMERA` (`pi/naviGATR/CMakeLists.txt:9`, `:68`, `:195`, `:247`).
- Build-time generated sources go to the build tree:
  `pi/naviGATR/CMakeLists.txt:160` `${CMAKE_CURRENT_BINARY_DIR}/generated/viewer_assets.cpp`.
- PlatformIO platform pinned to a commit: `pico/aggreGATR/platformio.ini:10`
  `platform = https://github.com/maxgerhardt/platform-raspberrypi.git#a1df30f`.

### 3.2 Problems found

translaGATR has no library target; six places compile its sources by path:
- `brain/communiGATR/CMakeLists.txt:20-21` `${GATR2_REPO_ROOT}/translaGATR/frame_codec.cpp`,
- `pi/naviGATR/CMakeLists.txt:180-181` `${REPO_ROOT}/translaGATR/frame_codec.cpp`,
- `pico/aggreGATR/tests/CMakeLists.txt:51` `../../../translaGATR/frame_codec.cpp`,
- `translaGATR/tests/CMakeLists.txt:37-38`, `brain/gatr2_brain.mk:51-52`,
  `pico/aggreGATR/platformio.ini:18` `build_src_filter = +<*> +<../../../translaGATR/frame_codec.cpp>`.
- Include style splits with it: brain and Pi write `#include "translaGATR/frames.h"`
  (57 includes), the Pico writes `#include "frames.h"` (16 files, e.g.
  `pico/aggreGATR/src/imu_bno08x.cpp:24`) because `platformio.ini:16` puts
  `-I$PROJECT_DIR/../../translaGATR` on the path.

Brain source lists exist three times and have already diverged:
- `brain/*/CMakeLists.txt` (the targets), `brain/gatr2_brain.mk:25-67` (PROS), and
  `pi/naviGATR/tests/CMakeLists.txt:76-101` (the e2e test compiles brain sources by
  path). The e2e list has 6 of 11 communiGATR sources and 7 of 11 actuGATR
  sources; it links today only because nothing referenced the missing files yet.

Same boilerplate copied per project:
- googletest fetch block x4 (`brain/CMakeLists.txt:8-23`,
  `pi/naviGATR/tests/CMakeLists.txt:1-16`, `translaGATR/tests/CMakeLists.txt:8-23`,
  `pico/aggreGATR/tests/CMakeLists.txt:7-22`), each without `URL_HASH`.
- `-Wall -Wextra` written 22 times in 12 build files, mostly per target inside
  `if(NOT MSVC)` (MSVC is not a supported compiler here).
- Repo root variable spelled two ways: `pi/naviGATR/CMakeLists.txt:7`
  `set(REPO_ROOT ...)` vs `brain/actuGATR/CMakeLists.txt:4` `GATR2_REPO_ROOT`; the
  Pico tests use a raw `../../../`.

Directory-scoped commands instead of target-scoped:
`translaGATR/tests/CMakeLists.txt:6` `add_compile_options(-Wall -Wextra)`,
`:26` `include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)`,
`:46` `target_link_libraries(common_tests GTest::gtest_main)` (no scope).

Stale names after the rename: `translaGATR/tests/CMakeLists.txt:2`
`project(gatr2_common_tests CXX)` and `:41` `add_executable(common_tests`;
CI job still named `common`.

Globbing: `pi/naviGATR/CMakeLists.txt:143` `file(GLOB NAVIGATR_VIEWER_FILES CONFIGURE_DEPENDS`.
Every `spectaGATR/*.js|html|css|json|svg` is embedded in the binary, including a
stray file. It is the only glob in the build (DECISION D3).

Code under test mixed into the test list:
`pi/naviGATR/tests/CMakeLists.txt:21` `../src/app/command_line.cpp` inside `TEST_SOURCES`.

C++ standard and extensions differ by project: only `brain/CMakeLists.txt:6`
sets `CMAKE_CXX_EXTENSIONS OFF`; naviGATR, translaGATR tests and Pico tests get
the gnu++17 default. PROS builds the same brain libraries as `gnu++20`
(`brain/locaGATR/Makefile:22` `CXX_STANDARD:=gnu++20`, forced by kernel headers);
the libcamera build lifts naviGATR to C++20 (`pi/naviGATR/CMakeLists.txt:197`
`target_compile_features(navigatr_core PUBLIC cxx_std_20)`).

`project(navigatr CXX)` then `enable_language(C)` later
(`pi/naviGATR/CMakeLists.txt:2`, `:46`); list the languages in `project()`.

Season path hardwired in build glue: `pi/naviGATR/CMakeLists.txt:234-236`
regenerates `brain/operaGATR/include/field_references.h` from
`config/override/field.xml`. Next season this line has to change.

PROS template leftovers: `brain/operaGATR/Makefile:38` `# TODO: CHANGE THIS!`
and `:41` `LIBNAME:=libbest`. The two apps' Makefiles also give different kernel
restore instructions (`locaGATR/Makefile:54-58` vs `operaGATR/Makefile:54-59`).

Vendored library without a version: `pi/naviGATR/third_party/README.md:15-16`
"Version: the vendored copy records no version or commit" (apriltag).

The Windows runtime-DLL workaround lives in five places (see section 7):
the MinGW build's `navigatr.exe` needs ucrt64 on PATH or exits 0xc0000139.

### 3.3 Proposed build rules

CM-1 One library target per component. `translaGATR/CMakeLists.txt` defines
`translagatr` (STATIC, PUBLIC include = repo root) and every CMake consumer links
it. No CMake file compiles another component's `.cpp` by path. Make and
PlatformIO, which cannot link CMake targets, list the files once each.

CM-2 One include spelling: `#include "translaGATR/frames.h"` everywhere,
Pico included (`-I` points at the repo root, not into `translaGATR/`).

CM-3 One source list per library. The CMake target is the source of truth;
`gatr2_brain.mk` mirrors it and a check (repo `tools/`, run in CI) fails when the
two lists differ. Cross-component tests link targets
(`communigatr`, `actugatr_sim`, ...) instead of re-listing sources.

CM-4 Shared CMake module at root `cmake/gatr2.cmake`, included by each project:
`GATR2_REPO_ROOT`, C++17 with extensions OFF, a `gatr2::warnings` INTERFACE
target (`-Wall -Wextra`, plus `-Werror` per DECISION D2), and the googletest
provider (system package first, else FetchContent pinned by `URL_HASH SHA256`).
Removes 4 copies of the fetch block and 22 flag lines.

CM-5 Target scope only: `target_*` commands with PRIVATE/PUBLIC/INTERFACE. No
`include_directories`, `add_compile_options`, `link_libraries`, and no
scope-less `target_link_libraries`.

CM-6 Explicit lists. Sources, tests and embedded assets are listed by name;
no `file(GLOB)`. Tests list test files in `TEST_SOURCES` and code under test
in `CODE_UNDER_TEST` or by linking the library.

CM-7 Names. `project(<component> LANGUAGES ...)` lowercase component name;
library `<component>`, tests `<component>_tests`; options
`<COMPONENT>_BUILD_<X>` and `<COMPONENT>_WITH_<DEP>` (hardware deps default OFF);
resulting compile definitions `<COMPONENT>_HAVE_<DEP>`. Existing NAVIGATR_* names
already fit; `common_tests` and `gatr2_common_tests` become `translagatr_tests`.

CM-8 Standard. Library code is C++17 and is built as C++17 on the host in CI.
A platform build may compile it at a newer standard when a platform header forces
it (PROS, libcamera); that never licenses newer features in library code.

CM-9 Dependencies. Runtime deps are vendored in `<component>/third_party/<lib>/`
with a README entry (upstream, exact version or commit, license, files kept,
users). FetchContent only for test-only deps and always hash-pinned. PlatformIO
platforms pinned to a commit.

CM-10 Generated files. Build-time outputs go to the build tree. A generated file
that is committed (`brain/operaGATR/include/field_references.h`) has its
generator committed, a first line naming generator and input
(`field_references.h:1` "Generated by navigatr_field_refs from
pi/naviGATR/config/override/field.xml. Do not edit."), and a test that fails
when it is stale (naviGATR has this). Season inputs are a cache variable
(`GATR2_SEASON`), not a path literal.

CM-11 Build entry points. Each CMake project ships `CMakePresets.json` with fixed
`binaryDir` under root `build/<project>-<preset>` and a Windows preset that sets
the MinGW generator and the ucrt64 PATH. Scripts and docs name presets, not
folders. Alternative worth testing: link MinGW builds with
`-static-libstdc++ -static-libgcc` so the PATH workaround disappears.

CM-12 PROS projects. `common.mk` and `project.pros` are PROS-owned and never
edited; project Makefiles are edited only above the "Nothing below this line"
marker; template placeholders are filled (`LIBNAME` = app name) or deleted.

CM-13 Comments in build files are noun-phrase labels (`# Library.`), as brain does.

---

## 4. XML configuration (pi/naviGATR/config)

### 4.1 Good patterns to keep

- Naming is already uniform: 86 element names all PascalCase, 157 attribute
  names all snake_case (measured over all 34 files, comments stripped).
- Units ride in attribute names: `rate_hz`, `stale_after_ms`, `still_rate_dps`,
  `angle_random_walk_rad_per_sqrt_s` (`config/override/brain_profile_usb.xml:83`).
- Composition by reference: `<Robot file="../shared/robots/synthetic_rig.xml"/>`
  (`config/demo/synthetic_field_demo.xml:22`).
- Season data is config, not code: `config/override/field.xml:2`
  `<Resource id="override_field" type="field_map" name="V5RC Override 2026-2027"`.
- Provenance recorded as data: `field.xml` `Dimensions` carries `source=` and
  `revision=` attributes (`field.xml:53-54`).
- Templates are enforced by the loader, not by convention:
  `pi/naviGATR/src/config/composition.cpp:198-199` refuses `.xml.in` as
  "an intentionally non-runnable template"; tokens look like `@SELECT_CAMERA_INDEX@`.
- Each top-level section maps to one construction function, which is also the
  answer to the "where is make_*" gripe:
  `<Resources>` -> `make_resources` (`src/runtime/resource_stage.h:4`),
  `<Sensors>` -> `make_sensors` (`src/runtime/sensor_stage.h:4`),
  `<Localization>` -> `make_localization` (`src/runtime/localization_stage.h:4`),
  called in order from `src/runtime/system.cpp:194`, `:202`.

### 4.2 Problems found

- Two root dialects: `<System>` with flat `Resources/Sensors/Pipeline`
  (`config/examples/modular/main.xml:4`) and `<Configuration>` with
  `Robot/Field/Pipeline` (every other runnable file). Both accepted:
  `src/config/composition.cpp:144` `if (name != "Configuration" && name != "System") {`.
- Configuration `id` does not match the file name:
  `config/demo/synthetic_field_demo.xml:15` `<Configuration id="demo_synthetic_field">`;
  `config/override/diagnostics/three_wheel_imu.xml.in:10`
  `id="override_three_wheel_imu_diagnostic"`; `brain_profile_usb.xml` matches.
- Missing unit: `config/override/brain_profile_usb_camera.xml:28`
  `preview_max_width="640"` next to `:54` `<Capture width_px="640" height_px="480"`.
- The season folder mixes season and non-season files: `config/override/` holds
  the field (season) but also the Brain profile mains and `diagnostics/`
  bring-up profiles, which do not depend on the game. Fragments by kind are
  mixed too: `config/shared/pipelines/parallel_wheels_bno08x_localization.xml` has
  root `<Localization>` inside a `pipelines/` folder.
- Copy instead of compose: `brain_profile_usb.xml` and `brain_profile_rs485.xml`
  differ in about 10 of 100 lines (the Brain link resource and its references);
  `brain_profile_usb_camera.xml` differs from the USB file in 76 diff lines. The
  32-line `BrainProfile` block is identical in the USB and RS-485 files
  (`brain_profile_usb.xml:55-86`, `brain_profile_rs485.xml:56-87`); the camera
  file carries a shorter variant (`brain_profile_usb_camera.xml:72-93`).
- `.xml.in` has two meanings in one component: CMake `configure_file` input
  (`pi/naviGATR/src/app/build_config.h.in`) and "hand-completed template" for XML.
  And not every `.xml.in` is a template: `three_wheel_imu.xml.in` has no `@...@`
  tokens; it is `.in` because the robot file it references does not exist yet
  (`three_wheel_imu.xml.in:3-7`).
- The completed files a user makes (`cp -n ...xml.in ...xml`,
  `pi/naviGATR/docs/setup.md:211-213`) are neither ignored nor planned to be
  committed; measured robot data has no defined home.
- Only part of the checked-in config is load-tested: the honesty test lists 4 of
  the 8 shared pipelines by name (`tests/profiles_gtest.cpp:560-564`); other files
  are covered only when some test happens to load them.
- Numbered, meaningless names in the example: `examples/modular/pipelines/pipeline1.xml`,
  `resources/resource_config1.xml`, `sensors/sensor_config1.xml`.
- Name as child element in one place and attribute in another:
  `<ConfigurationName>` element (`synthetic_field_demo.xml:16`) vs `name=` on
  `<Resource>` (`field.xml:2`). Scalar-as-element: `<Baud value="115200"/>`.
- Long prose comments (`field.xml:3-35`, `brain_profile_usb.xml:2-15`,
  `gatr2_as5047_imu.xml.in:2-25`). Some of it is provenance that should be data or
  docs; some is instructions that belong in the calibration inventory.
- No `config/README.md` saying what `demo/`, `examples/`, `override/`, `shared/` mean.

### 4.3 Proposed XML rules

XML-1 Names. Elements PascalCase with acronyms as words (`Imu`, `BrainImu`);
attributes snake_case. Already true; keep it.

XML-2 Units. Every physical-quantity attribute ends in its unit: `_m`, `_mm`,
`_s`, `_ms`, `_us`, `_hz`, `_deg`, `_rad`, `_dps`, `_px`, compound units spelled
out (`_rad_per_sqrt_s`). Unitless by nature: ids, indices, counts, channels,
ports, gpio numbers, ratios, enums, upstream library knobs (`nthreads`,
`quad_decimate`). New attributes follow this; existing misses
(`preview_max_width`) are renamed with a loader alias for one season.

XML-3 One root per file kind, and the folder says the kind:
`Configuration` (runnable main), `Robot`, `Pipeline`, `Localization` (and other
section fragments), `Resource type="field_map"` (field). The `<System>` dialect is
either retired (examples moved to `<Configuration>`) or documented as the flat
form; not both undocumented.

XML-4 Configuration `id` equals the file stem.

XML-5 Layout (proposal; paths are config data, the runtime does not change):
```
config/
  README.md              what each folder holds, which file to run
  seasons/<season>/      season data only: field.xml (today config/override/field.xml)
  robots/                robot descriptions and their templates
  pipelines/             Pipeline fragments
  localization/          Localization fragments
  profiles/              runnable Configuration files (brain_profile_*, bring-up)
  demo/                  hardware-free runnable demos
```
Build glue and tests take the season from `GATR2_SEASON` (CM-10).

XML-6 Compose, do not copy. A block used by two runnable files lives in one
fragment referenced by `file=`. Runnable variants differ only in what differs
(USB vs RS-485 link).

XML-7 Templates. Rename `*.xml.in` to `*.template.xml` (DECISION D4) so `.in`
keeps its CMake meaning; the loader refuses `*.template.xml`. A template has
`@UPPER_SNAKE@` tokens for every unmeasured value; a file with no tokens is not a
template. The measured file is committed next to it (it is the robot's data) and
the template stays as the reference.

XML-8 Every committed runnable file loads in a test: one test walks `config/`,
resolves every `Configuration` root that is not a template, and fails with the
file name. Fragments are covered through the files that include them.

XML-9 Scalars are attributes; a new setting is an attribute on the element it
configures (`<Device path="..." baud="115200"/>`), not a single-attribute child.
Existing forms stay (the parser is a system that keeps its shape).

XML-10 Comments: one label line per block. Provenance goes in `source=`/
`revision=` attributes; procedures go to docs (calibration inventory). Header:
`<?xml version="1.0" encoding="UTF-8"?>`, ASCII content.

XML-11 The pipeline doc for naviGATR carries one table: XML element ->
`make_*` function -> file, so reading a config tells you which code builds it.

---

## 5. Shell scripts

### 5.1 Findings

- Strict mode differs: `pi/naviGATR/tools/install_service.sh:3` `set -euo pipefail`;
  `tools/viewer_smoke.sh:31` and `tools/perf/run_perf.sh:25` only `set -u`.
- shellcheck (0.11.0, via the `shellcheck-py` wheel, no system install):
  `install_service.sh` and `run_perf.sh` clean; `viewer_smoke.sh`:
  - `:43`/`:50` `--keep) KEEP=1; shift;;` sets a flag nothing reads; the documented
    `[--keep]` option (`:20`) does nothing (real bug);
  - `:36` `cd "$repo"` unchecked, with no `set -e` to catch it;
  - `:137` unused loop variable.
- Help text by grepping the script's own comments:
  `viewer_smoke.sh:51` and `run_perf.sh:58` `grep '^#' "$0" | sed 's/^# \{0,1\}//'`.
  This ties the help to comment prose, so the comments cannot be terse.
  `install_service.sh:5-24` uses a `usage()` heredoc instead.
- Windows specifics duplicated with drift: path helper `winpath` (`cygpath -w`,
  `viewer_smoke.sh:109`) vs `native` (`cygpath -m`, `run_perf.sh:73`); browser
  discovery in `viewer_smoke.sh:81-89`, `run_perf.sh:86-92` and `tools/viewer/cdp.py:24-36`
  with three different candidate lists; ucrt64 PATH prepend in
  `viewer_smoke.sh:71-78` (checks mingw64 too), `run_perf.sh:68-72` (ucrt64 only),
  `viewer_tests.py:164-168`.
- Default build folders hardwired and all different: `install_service.sh:31`
  `binary="$project_dir/build-bench/navigatr"`, `viewer_smoke.sh:59`
  `build-viewer/navigatr.exe ...`, README `build/`.
- Good: `install_service.sh` validates inputs, refuses to run off the Pi
  (`:62` `[[ $(uname -s) == Linux ]] || fail ...`), renders before installing,
  backs up the old unit; every script cleans up with `trap ... EXIT`.
- Scope creep: `viewer_smoke.sh` is 330 lines that parse a dumped DOM with grep
  and sed; `viewer_tests.py` already drives the same page over CDP.

### 5.2 Proposed shell rules

SH-1 Header: `#!/usr/bin/env bash`, one label line, `set -euo pipefail`.

SH-2 shellcheck clean (run from the pip wheel, so no node and no system package).

SH-3 `usage()` heredoc for `--help`; comments stay labels.

SH-4 Portability: every script runs in Git Bash on Windows and bash on Linux,
or refuses early with a clear message (as `install_service.sh:62`). Windows
specifics (cygpath, browser path, runtime DLL path) live in one sourced helper,
`tools/lib/host.sh`, not copied per script.

SH-5 Shell is glue. Over ~150 lines, or once it parses structured data (JSON,
DOM, XML), it becomes a C++ tool or a Python tool per PY-1.

SH-6 Flags kebab-case, `--flag value`, units in names (`--duration-s`), same as
PY and the runtime (which accepts both `--config_file` and `--config-file`,
`src/app/command_line.cpp:42`; document the kebab form).

SH-7 Paths to built binaries come from the preset name (CM-11), never a
hardcoded `build-<something>/`.

---

## 6. Repository hygiene

### 6.1 Findings

- Build output sprawl, all correctly ignored (`.gitignore:75-78`) but
  inconsistent: root `build/` (11 subfolders plus
  `build/navigatr-bench-source.tar.gz` and `build/review-common-tests.exe`),
  7 root `build-*` folders (`build-analysis-navigatr`, `build-analysis-navigatr-tests`,
  `build-printables`, `build-review-common`, `build-review-common-mingw`,
  `build-review-navigatr`, `build-review-navigatr-mingw`), and 7 more under
  `pi/naviGATR/` (`build`, `build-camera-tools`, `build-field`, `build-libcamera`,
  `build-server`, `build-viewer`, `build-workers`). Each has its own googletest
  download. A Python venv lives inside one (`pi/naviGATR/build-camera-tools/venv/`).
- Committed output with an uncommitted generator: the printables
  (`pi/naviGATR/docs/printables/*.pdf|svg`) are produced by
  `build-printables/generate.py`, which is ignored; it also still writes to the
  old path (`build-printables/generate.py:7` `OUT = ROOT / 'pi/navigatr/docs/printables'`).
- `.gitattributes` is one line, `* text=auto eol=lf`. The three committed PDFs are
  hand-generated uncompressed PDFs, so git classifies them as text
  (`git ls-files --eol`: `i/lf ... checkerboard-9x6-25mm-a4.pdf`); a PDF's xref
  offsets break if its bytes are ever normalized.
- 106 tracked files are CRLF in the working tree while LF in the index (vendored
  apriltag and tinyxml2, `tools/perf/perf_net.cpp/.h`, PROS `project.pros`,
  `.vscode` json, KiCad). Harmless for git, noisy for tools that read the tree.
- No `.editorconfig`, no `CMakePresets.json`, no formatter config for JS, Python
  or CMake.
- Machine-specific files tracked: `.vscode/settings.json:2`
  `"cmake.sourceDirectory": "C:/GATR2-Electronics/brain"` and
  `pico/aggreGATR/.vscode/settings.json:2` `"C:/GATR2-Electronics/pico/aggreGATR/tests"`.
- A stray log is tracked: `debug.log` (a Chrome crashpad error line), added in
  commit 0501178.
- Case-only rename not reflected on this disk: git tracks `pi/naviGATR/` but the
  folder on disk is `pi/navigatr` (`core.ignorecase=true`). `docs/naming.md:48-53`
  records the two-step `git mv` fix; Linux checkouts see `naviGATR`.
- `.gitignore` lacks `__pycache__/`, `*.pyc`, `.venv/`, `venv/`.
  `pico/aggreGATR/.gitignore` repeats root rules (`.pio`, `.vscode/...`); the PROS
  apps' `.gitignore` files are rewritten by the PROS CLI (`operaGATR/Makefile:57`
  "git checkout -- .gitignore").
- Folders without a signpost: `pi/naviGATR/config/`, `spectaGATR/`, `tools/`,
  `tools/viewer/` have no README. naviGATR docs are split between
  `pi/naviGATR/docs/` and root `docs/navigatr_resources.md`, `docs/navigatr_sensors.md`.
- Folder names out of step with the renames: `tools/viewer/` and
  `tools/viewer_smoke.sh` test `spectaGATR/`. PCB iteration folders mix case
  (`PiHat_v1`, `PiHat_v2`, `PiHat_V3`, `PiHat_v4`) while
  `docs/contributing/README.md:8` says `{BOARDNAME}_V{number}`.
- (CI is not a constraint, one line for the rebuild) the pico job runs
  `pio run` with `working-directory: pico` (`.github/workflows/ci.yaml:88`) but
  `platformio.ini` is in `pico/aggreGATR/`.

### 6.2 Proposed hygiene rules

REPO-1 Build output. One tree: root `build/<project>-<preset>/`, from presets
(CM-11). No build folders inside components, no venvs inside build folders
(venvs go in `.venv/` at the tool folder, ignored). One-off review builds are
deleted when done.

REPO-2 `.gitattributes`:
```
* text=auto eol=lf
*.pdf  binary
*.png  binary
*.zip  binary
*.step binary
*.a    binary
*.ps1  text eol=crlf
```
then one `git add --renormalize .` commit.

REPO-3 `.editorconfig` at root: UTF-8, LF, final newline, trim trailing space,
indent 4 spaces; tabs for `Makefile` and `*.mk`; 2 spaces for `*.yaml`/`*.yml`.

REPO-4 Nothing machine-specific is tracked: no absolute paths (use
`${workspaceFolder}` in editor settings or keep them untracked), no logs, no
build output, no secrets (`secrets.local` is correctly ignored, `.gitignore:15`).
Add `__pycache__/`, `*.pyc`, `.venv/`, `*.log` to `.gitignore`. Nested
`.gitignore` files only for tool-owned folders (PROS).

REPO-5 Generated files: the generator is committed next to what it maintains
(for printables: `pi/naviGATR/docs/printables/generate.py`, stdlib Python per
PY-1), every generated file says so on line 1 when the format allows, and a
check fails when output and generator disagree.

REPO-6 Renames: case-only renames use the two-step `git mv`; CI runs on Linux,
so a case mismatch in a path fails the build there.

REPO-7 Signposts: every folder a newcomer lands in has a short README (what it is,
where to start); a component with a pipeline has `PIPELINE.md` (see the main
standard). Component docs live under the component's `docs/`; root `docs/` holds
only cross-component material.

REPO-8 Names follow `docs/naming.md`: tool and test folders are named after the
component they serve (`tools/spectagatr_tests/` or `spectaGATR/tests/`).

---

## 7. Cross-cutting: the same job done more than once

These are subsystem-level simplifications (the user's "overly complicated shit
that could be refactored, including systems").

| Job | Copies | Where |
|---|---|---|
| Compile translaGATR sources | 6 | CM-1 list above |
| Brain source lists | 3 (diverged) | `brain/*/CMakeLists.txt`, `brain/gatr2_brain.mk:25-67`, `pi/naviGATR/tests/CMakeLists.txt:76-101` |
| googletest fetch | 4 | CM section 3.2 |
| `-Wall -Wextra` | 22 lines / 12 files | CM-4 |
| MinGW DLL PATH workaround | 5 | `viewer_smoke.sh:71-78`, `run_perf.sh:68-72`, `viewer_tests.py:164-168`, `pi/naviGATR/README.md:206`, `pi/naviGATR/docs/configuration.md:14` |
| Headless browser discovery | 3 (different lists) | `viewer_smoke.sh:81-89`, `run_perf.sh:86-92`, `cdp.py:24-36` |
| CDP + WebSocket client | 2 languages | `tools/viewer/cdp.py`, `tools/perf/cdp_bench.cpp` + `perf_net.cpp` |
| `vendor/` to `third_party/three` mapping | 3 | `pi/naviGATR/CMakeLists.txt:155-159`, `viewer_tests.py:437-439`, `stub_server.py:708-710` |
| Viewer browser harnesses | 3 | `viewer_smoke.sh` (DOM dump), `viewer_tests.py` (CDP), `tools/perf` (CDP) |
| Angle wrap in JS | 3 | JS section 1.2 |

Suggested direction, smallest first: CM-1 and CM-4 (pure build glue, no
behavior change); one host helper for Windows specifics (SH-4) or static MinGW
linking (CM-11); fold `viewer_smoke.sh` into `viewer_tests.py live --binary`;
then decide whether the Python CDP client or the C++ one survives (D1).

---

## 8. Decisions for the user

D1 naviGATR language rule. (a) Strict, as remembered: naviGATR holds C++, CMake,
XML only; spectaGATR, the Python tools and the shell scripts move out (for example
`pi/spectaGATR/`, `pi/tools/`), and the viewer tests could be folded into the
existing C++ CDP harness, which removes Python and one duplicate client.
(b) Recommended: `src/` is C++ only; the component may also hold `config/` (XML),
`spectaGATR/` (the one browser app the runtime serves) and `tools/<tool>/` in
Python or Bash, each tool with a README line saying why it is not C++. Either way
the viewer needs to be named explicitly, since it is on no accepted list today.

D2 Warnings. Memory says `-Wall -Wextra -Werror`; this session says `-Wall -Wextra`
without `-Werror`; the build today has no `-Werror` anywhere. Options: no
`-Werror`; `-Werror` in CI only (a `GATR2_WERROR` option, default OFF locally);
`-Werror` everywhere.

D3 Viewer assets: keep the one `file(GLOB)` for `spectaGATR/` (adding a module
needs no CMake edit) or list assets explicitly like sources (a new file is a
deliberate diff line, matching the test-list rule).

D4 Template suffix: keep `.xml.in` (already enforced by the loader) or rename to
`.template.xml` so `.in` only means CMake `configure_file`.

D5 inspect/1 support in the viewer: keep for older runtimes, or drop it since the
page ships inside the runtime binary (removes `feed.js:412-427` and `compat` branches).

D6 JS formatter: add a JavaScript section to the pinned clang-format 22.1.8
(no node) and pick the object-literal spacing; or leave JS unformatted and rely
on review.

D7 Python quote style: ruff default (double, matches the calibration tool) or
single (matches the viewer tools, 4 of 7 files). Recommendation: ruff default,
no config knob.

D8 Test hook rename: `window.__navigatr` to `window.__spectagatr` (touches the
viewer tests and perf harness in the same commit).

---

## 9. Feed for "Applying this to the existing system"

Behavior-preserving order, each step one reviewable commit:

1. Hygiene: `.gitattributes` binary lines plus renormalize, `.editorconfig`,
   `.gitignore` additions, untrack `debug.log` and both `.vscode/settings.json`,
   commit the printables generator (fixed output path), fill PROS `LIBNAME`/TODO.
2. Build glue: root `cmake/gatr2.cmake` (warnings target, gtest with hash,
   C++17 extensions OFF), `translaGATR/CMakeLists.txt` target, consumers link it,
   rename `common_tests`, Pico include spelling, e2e test links brain targets,
   `.mk` drift check, presets per project, season cache variable.
3. Shell and Python lint fixes that are real bugs (`--keep`, unchecked `cd`,
   unused imports and variables, file handles), strict mode, `usage()` heredocs,
   shared host helper, `requirements.txt` for the calibration tool, remove
   auto-install from `pi_transmit.py`.
4. Config: `config/README.md`, ids equal file stems, `preview_max_width_px` with
   alias, compose the three Brain profile files from fragments, a test that loads
   every runnable config, then the folder layout (XML-5) and template suffix (D4).
5. Viewer: move design prose to `spectaGATR/PIPELINE.md`, dedupe helpers
   (`wrapDeg`, `el`, `kvText`, `format.js`), rename `bundle.js`, hoist `app.js`
   state, then split `app.js` (status bar, replay controller, dock) and
   `field_scene.js` (field build, robot and trail, overlays) along existing method
   groups, then the formatter pass as its own commit (D6).
6. Tool consolidation per D1: one browser harness, one CDP client.
