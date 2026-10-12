# spectaGATR implementation contract (coordinator, 2026-10-09)

Read first: `requirements.md` (the user's full request, verbatim) in this folder, then
`C:/GATR2-Electronics/spectaGATR/docs/PROTOCOL.md` (conventions, wire line, browser feed).
The wire format library already exists and is tested:
`spectaGATR/protocol/include/spectagatr/protocol/{pose_line.h,units.h}` +
`spectaGATR/protocol/src/*.cpp` + tests (9/9 pass, also compiles with the PROS ARM toolchain).
Do NOT invent another message format; do not edit protocol/ (report needed changes to the
coordinator instead).

Repo: `C:/GATR2-Electronics`, branch main at 20f20e2, clean before this work. The demo is
TOMORROW: working, simple, correct beats elaborate.

## Global rules

- Never commit, push, stash, reset, checkout, or run any destructive git command. Never flash
  or upload to hardware, never touch the Pi or remote services. Never edit `pcb/`.
- Edit only the paths you own (below). Need a change elsewhere: say so in your final report.
- Build only in your own scratch dir `S/sg-build/<your key>/...` where
  `S = C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`.
- Code style: the repo's (`.clang-format` at the root, C++17 for host code, 4-space indent,
  100 columns, namespaces mirror folders, `using` aliases for units (Meters, Radians,
  Milliseconds...), `struct` for plain data, `class` only where state and methods belong
  together, free functions preferred, no `using namespace` in headers). Comments: plain ASCII,
  terse, no em dashes, not chatty. Docs: plain ASCII, short.
- Explicit test lists in CMake (no globbing). gtest via FetchContent; pass
  `-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src` to stay offline.
- Distinguish host-tested, compile-only and hardware-tested in everything you write. Nothing
  here has hardware; never claim hardware, radio, Linux or macOS results.

## Toolchains on this Windows box

- Host C++: `export PATH=/c/msys64/ucrt64/bin:$PATH` (GCC 15.2), `cmake -G "MinGW Makefiles"`.
  Test exes need ucrt64 first on PATH (else 0xc0000139).
- PROS (arm-none-eabi GCC 13.3.1): `TC=/c/Users/mbes1/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr; export PROS_TOOLCHAIN="$(cygpath -w $TC)"; PATH="$TC/bin:$PATH" make -j8`
  in a PROS project dir. Kernel 4.2.2 files (`firmware/`, `include/pros/`) are gitignored per
  project (`brain/*/firmware/`, `brain/*/include/pros/`); copies exist in the untracked
  leftover `brain/operaGATR/firmware` and `brain/operaGATR/include/pros`. NOTE the toolchain's
  own `grep` shadows msys grep inside that PATH; run greps in a separate command.
- Python 3.12 (`python`), PlatformIO 6.2 (`pio`), headless Chrome
  (`"/c/Program Files/Google/Chrome/Application/chrome.exe" --headless=new ...`) and Edge. No
  Node.
- clang-format 22: `/c/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/clang-format.exe`.

## Ownership

| Key | Owner | Paths |
|---|---|---|
| coordinator | me | `spectaGATR/protocol/**`, `spectaGATR/docs/**`, `spectaGATR/README.md`, root `README.md`, `docs/**` index edits, `.github/**` |
| pub | Agent "publisher" | `brain/spectaGATR/**`, `brain/spectaGATR-demo/**`, `brain/CMakeLists.txt`, `brain/gatr2_brain.mk`, `brain/README.md` |
| rx | Agent "receiver" | `spectaGATR/receiver/**`, `spectaGATR/tools/**` (except `mock_feed.py`), `spectaGATR/CMakeLists.txt` (extend; keep `add_subdirectory(protocol)`) |
| web | Agent "browser" | `spectaGATR/web/**`, `spectaGATR/fields/**`, `spectaGATR/tools/mock_feed.py` |
| fb | Agent "fallback" | `deprecated/**` only |

## Decisions everyone builds on

- Conventions and units: PROTOCOL.md. API is SI (meters, radians); wire is mm/centidegrees via
  `spectagatr::protocol::{toMillimeters,toCentidegrees,toMeters,toRadians}`.
- Default publish rate 10 Hz (configurable 1..50 Hz). Default receiver stale threshold 750 ms.
  Default HTTP port 8790, bind 127.0.0.1.
- Robot box default (laptop side): front 0.1685, back 0.1685, left 0.1465, right 0.1465,
  height 0.25 m (from the archived robot footprint, deprecated/brain/robot/gatr2_robot.h).
- The publisher writes ONLY to stdout (one `write` per line), never calls serctl/fdctl, never
  reads stdin. The receiver NEVER writes to the Brain user port (stdin bytes like "pRc" toggle
  PROS COBS globally). Through the controller the receiver only sends the read-only CDC2
  queries (SYSTEM_VERSION, RADIO_STATUS, USER_READ ch 1); never radio channel switches.
- Coexistence: Brain-to-Pi USB link (communiGATR) + laptop polling the controller is UNTESTED
  and may steal bytes from the Pi link (one VEXos stdio FIFO; see
  `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/spectu-handoff/discovery/controller-host.md`
  section 6). Documented as two operating modes; never claimed.

## Reusable prior work (copy, adapt, do not import at build time)

- Phase A worktree `S/wt-spectu` (our own code, reviewed, 263/263 host tests then):
  - `spectuGATR/host/platform/**`: serial port (Win32 sync COMMTIMEOUTS, POSIX termios),
    USB port enumeration (SetupAPI/IOKit/sysfs), VEX port classifier (Brain 2888:0501 if0
    system / if2 user; controller 2888:0503), browser launch, stop signal, win32 text.
  - `spectuGATR/host/sources/**`: `brain_usb` (direct Brain user port reader), `cdc2` +
    `controller` (CDC2 USER_READ polling, RADIO_STATUS, SYSTEM_VERSION), `vex_port` opener,
    `source.h`/`chunk.h`, `sim/` fakes; tests in `spectuGATR/tests/{platform,host}`.
  - `brain/spectuGATR/src/slot.h` (seqlock latest-wins slot), `brain/spectuGATR/pros/stdio_transport.cpp`.
  - Discovery notes with PROS/VEX facts and citations:
    `C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/spectu-handoff/discovery/{pros-serial,controller-host,brain-link,xplat-host}.md`
- Old Pi viewer (archived): `deprecated/pi/naviGATR/spectaGATR/*.js` (field_scene.js has the
  field rendering), `deprecated/pi/naviGATR/third_party/three/{three.module.min.js,OrbitControls.js,LICENSE}`,
  HTTP + WebSocket server `deprecated/pi/naviGATR/src/inspection/http_server.{h,cpp}` (Winsock
  and POSIX; depends only on `core/host_clock.h`), field definition
  `deprecated/pi/naviGATR/config/override/field.xml`, and how the old server turned it into
  viewer JSON: `deprecated/pi/naviGATR/src/inspection/inspection_document.cpp` (~833-899).

## Final report (your last message, plain ASCII, concise)

Files written (paths), what works, exact build/test commands you ran with results (counts),
what is compile-only vs host-tested, known gaps, anything you need from another owner.
