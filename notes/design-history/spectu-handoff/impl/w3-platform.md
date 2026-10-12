# W3 platform: implementation log

Workstream key `w3-platform`. Owns `spectuGATR/host/platform/**`, `spectuGATR/tests/platform/**` and
`spectuGATR/host/CMakeLists.txt` (later workstreams append). Worktree `S/wt-spectu`, index only
(`git add`), never committed.

`S` = `C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad`

## Status

- **Done and host-tested on Windows 11.**
  - Debug: 140/140 (106 protocol + 34 platform).
  - Release: 140/140.
  - Platform suite shuffled x5: 34/34 each time.
- **Linux and macOS: cross-compiled only, never run.**
  - No WSL, docker or Mac on this box.
  - macOS IOKit listing compiled against a hand-written declaration shim, not the SDK (see Verification).
- **No hardware claims.** No VEX device was attached; the live listing found 0 USB serial ports.

### Resume note (2026-09-29)

- A first run (session cut off) wrote every file, built Debug and Release (137/137) and ran a zig
  syntax check, but wrote no log.
- This run:
  - reviewed that code against PROPOSAL 4.1, 7.1 W3, 7.5 and disc:xplat-host s2;
  - fixed four defects (Deviations 1-4);
  - added 3 Windows-run tests and 1 POSIX-only test;
  - redid every check.
- The first run's files are snapshotted at `S/sp-build/w3-platform-orig/` for comparison.

## What exists

`namespace spectugatr::host::platform`, library `spectugatr_platform` (static; include root
`spectuGATR/`, so `#include "host/platform/serial_port.h"`).

| File | What |
|---|---|
| `serial_port.h/.cpp` | `SerialSettings`, `IoStatus`, `IoResult`, RAII move-only `class SerialPort` (`read`, `writeAll`, `close`), free `openSerialPort(path, settings, err)`, `ioStatusName` |
| `serial_port_win32.cpp` | `CreateFileW` share 0, DCB 8N1 raw (fNull off keeps 0x00), DTR/RTS enable, `SetupComm` 4096, `PurgeComm`; reads: COMMTIMEOUTS `{MAXDWORD, MAXDWORD, wait}` return on the first bytes; writes: `WriteTotalTimeoutConstant = budget`; unplug: `ClearCommError` fails, then `kClosed`; `ERROR_ACCESS_DENIED` maps to "in use by another program (PROS terminal, VEXcode)"; `COMn` becomes `\\.\COMn` |
| `serial_port_posix.cpp` | `open(O_RDWR, O_NOCTTY, O_NONBLOCK, O_CLOEXEC)`, `TIOCEXCL`, cfmakeraw, CLOCAL/CREAD, no CRTSCTS, VMIN=VTIME=0, TIOCMBIS DTR/RTS; waits with `select`; readable-and-empty means hung up, so `kClosed`; EACCES on Linux names dialout/uucp and the udev rule |
| `usb_ports.h/.cpp` | `UsbSerialPort` (path, vid, pid, optional CDC function interface, serial, product, location), `listUsbSerialPorts()`; the portable pure parts: `parseWindowsDeviceId`, `serialFromWindowsInstance`, `listSysfsSerialPorts(class_root, device_root)`, `cdcFunctionInterface` |
| `usb_ports_win32.cpp` | SetupAPI `GUID_DEVCLASS_PORTS` + `PortName`; `CM_Get_Device_IDW` parsed for `VID/PID/MI_xx`; `CM_Get_Parent` gives the composite parent for the serial (instance id without `&`); `DEVPKEY_Device_BusReportedDeviceDesc` (iProduct); `CM_DRP_LOCATION_INFORMATION` |
| `usb_ports_macos.cpp` | IOKit `IOSerialBSDClient`, `IOCalloutDevice` (`/dev/cu.*`); ancestors `IOUSBHostDevice`/`IOUSBDevice` (idVendor, idProduct, locationID, USB Serial Number, USB Product Name) and `IOUSBHostInterface`/`IOUSBInterface` (bInterfaceNumber, bInterfaceClass) |
| `usb_ports_linux.cpp` | `listSysfsSerialPorts("/sys/class/tty", "/dev")` |
| `vex_ports.h/.cpp` | Pure: `vexRoleOf`, `vexIdentityOf`, `classifyVexPorts`, `pickVexPort(ports, role, optional identity, err)`, `describeVexPort`, `vexRoleName`. Brain 2888:0501 if0 is `brain_system`, if2 is `brain_user`; controller 2888:0503 (any interface) is `controller`; everything else is `unknown` |
| `browser.h/.cpp` | `openBrowser(url, err)`: ShellExecuteW (COM STA, OLE1DDE off) / `open` / `xdg-open` through `sh -c` in the background. Only http(s) URLs pass `browserUrlAllowed`. Linux: DISPLAY or WAYLAND_DISPLAY required, never as root (`linuxLaunchAllowed`, pure) |
| `stop_signal.h/.cpp` | `StopSignal` (`request`, `requested`, `waitFor(ms)`): condvar waits that a stop ends at once, used instead of `sleep_for` |
| `clock.h` | `SteadyClock`, `Milliseconds` aliases |
| `win32_text.h/.cpp` | Windows only: UTF-8 <-> UTF-16, `win32ErrorText` |
| `PIPELINE.md` | stages, rules, per-OS status, failure behavior |
| `host/CMakeLists.txt` | `spectugatr_platform`; Win32 links setupapi, cfgmgr32, ole32, shell32; Apple IOKit and CoreFoundation frameworks; adds `tests/platform` when `SPECTUGATR_BUILD_TESTS` |
| `tests/platform/*_gtest.cpp` | 34 tests on Windows (list below), explicit list in `tests/platform/CMakeLists.txt`; Linux links `util` for openpty |

Consumer sketch (W5):
```text
listUsbSerialPorts() -> classifyVexPorts() -> pickVexPort(ports, VexRole::kBrainUser, identity, err)
openSerialPort(path, {}, err) -> SerialPort; read(buf, cap, wait) / writeAll(buf, n, budget) -> IoResult
kClosed: port already closed; rediscover at 1 Hz with StopSignal::waitFor(1000ms), by identity + role
```

Tests (Windows run, 34):
- VexPorts 10
- WindowsDeviceId 5
- SysfsTree 5 (pure filesystem, runs on Windows)
- CdcFunctionInterface 2 (new)
- UsbPorts.LiveListingOnThisHost 1
- StopSignal 3
- SerialPort 5: missing port, `\\.\COM250`, NUL refused and released twice, empty path and baud 0, closed port and move
- Browser 3

POSIX-only, not run here (compiled and linked for Linux):
- `PtyLoopback.ReadsWakeOnDataAndWritesArrive`;
- `PtyLoopback.HangUpClosesThePort` (new);
- `PtyLoopback.SecondOpenIsRefused` (TIOCEXCL; skipped as root).

## Deviations and decisions (with reasons)

Fixed in this run (defects in the first run's code):
1. **POSIX waits use `select`, not `poll`.** The xnu poll(2) man page, BUGS: "The poll system call
   currently does not support devices" (apple-oss-distributions/xnu `bsd/man/man2/poll.2`). mio,
   crossterm and libuv hit the same limit on macOS ttys (nathancraddock.com/blog/macos-dev-tty-polling).
   pyserial also uses select. Consequences:
   - one POSIX path;
   - `openSerialPort` refuses fd >= FD_SETSIZE ("too many open files for select");
   - hang-up is detected as readable followed by read() == 0 or an error, both `kClosed`.
2. **Browser: a missing opener is reported.**
   - The first run's `sh -c '"$0" "$1" &'` exited 0 even when xdg-open or open did not exist.
   - Now `command -v "$0" || exit 127` runs first, giving "xdg-open not found".
   - The opener's stdin is also /dev/null.
3. **`pickVexPort(..., VexRole::kUnknown, ...)` is refused.** The header promised "never picked" but
   the code would have picked an unknown-role port if asked. Test `VexPorts.UnknownRoleIsNeverPicked`.
4. **The macOS function-interface rule is now a pure, tested function**, `cdcFunctionInterface(number, class)`.
   - It used to live inside the IOKit file, which cannot be compiled or run here.
   - The rule: a CDC data interface (class 0x0A) gives number - 1, anything else is kept, and out-of-range values give no interface.
   - Tests: `CdcFunctionInterface.*`.
   - The IOKit variable was renamed `usb_interface` (`interface` is a macro on some platforms).

Carried from the first run and kept after review:
5. **Per-OS files guard themselves with `#if`, and CMake compiles all of them on every OS**, as the
   task says. disc:xplat-host s5 suggested CMake picking files per OS; `#if` keeps one source list.
   The other OSes' files compile to empty objects.
6. **`openSerialPort` is a free function returning `std::optional<SerialPort>`**, not a static
   `SerialPort::open` (disc:xplat-host s2 sketch). The user's rule prefers free functions.
   - One `IoResult{status, bytes, error}` serves read and write, instead of `ReadResult{bytes, closed}` plus a bool.
   - A write timeout reports how many bytes went out.
   - `kClosed` carries the reason for the log.
7. **No `SerialSettings::exclusive`**: every open is exclusive (7.5), and no caller wants a shared open.
8. **Identity order: USB serial, then product suffix, then location, then path.** Product suffix
   ("V5 Brain - A9EC2700" gives "A9EC2700") is proposal 4.1's identity column.
   - The form is `vex:<pid hex4>:<serial or suffix>` or `vex:<pid>@<location or path>`.
   - Both Brain ports share it, since every part is device-level.
   - `pickVexPort` takes an optional identity so the 1 Hz rediscovery can re-find the same device (7.5).
   - There is no `--identity` flag in phase 1 (5.7).
9. **The controller is any interface of 2888:0503** (proposal 4.1: one CDC port; the interface number is unknown, B9).
   - If a controller turns out to have two CDC ports, `pickVexPort` refuses and lists both.
   - It fails safe; it never picks a wrong port.
10. **macOS "data interface minus one" is an inference.** On macOS the tty hangs under the CDC data interface.
    - pros-cli's macOS name suffixes are 1 (system), 3 (user) and 2 (controller), which fits data interfaces 1, 3, 2 and functions 0, 2, 1.
    - Unconfirmed until B9 on a Mac.
11. **Not built: `app_paths` and `process`.** 7.1 W3 says "minus app_paths and process", and 7.5 puts signal handling in main.cpp (W5).
12. **Added `StopSignal` and `clock.h`** for "event-driven waits, never sleep-paced": the 1 Hz
    rediscovery, the controller back-off (10-20 ms) and shutdown all wait on it.
13. **`tests/platform` is added from `host/CMakeLists.txt`** (`add_subdirectory(../tests/platform ...)`).
    - The top level is W0's file and adds only `tests/protocol`.
    - W5 can do the same for `tests/host`, or W0 can move both lines to the top level later.
    - W0 review m5 (configure failed with the host default ON) is resolved now that `host/` exists.
14. **The listings include more than VEX devices.**
    - Linux lists ttyUSB* as well as ttyACM*.
    - Windows lists only USB-enumerated COM ports: Bluetooth (BTHENUM) and FTDI VCP (FTDIBUS) nodes are skipped, which does not matter for VEX.
    - Callers filter through `classifyVexPorts`.
15. **Windows location is `CM_DRP_LOCATION_INFORMATION`** of the composite parent (`Port_#0002.Hub_#0001`).
    - It is the weakest identity; it is used only when there is no iSerialNumber and no product suffix.
    - DEVPKEY_Device_LocationPaths would be steadier across hubs; left for B9 if location turns out to matter.
16. **Warnings: `-Wall -Wextra` in CMake**, as W0's protocol target has. The draft standard's
    `gatr2::warnings` target does not exist yet; its full flag set was run by hand (Verification).
17. **Linux EACCES text names "the spectuGATR udev rule".** W9 ships the rule file; W3 did not create it.

## Open issues (for later workstreams or bench)

- **W2/W5, POSIX: fd inheritance.** `openBrowser` spawns `sh`, which starts `xdg-open`/`open`, which may start
  the browser.
  - Any fd without CLOEXEC is inherited. The serial fds have O_CLOEXEC; the HTTP listener and client sockets must too (SOCK_CLOEXEC or FD_CLOEXEC).
  - Otherwise a browser started by the receiver keeps the port bound after the receiver exits.
  - Windows is unaffected (ShellExecuteW does not inherit handles).
- **B9 (bench):**
  - the Windows `MI_xx` and COM node class for Brain and controller, with and without the VEX driver;
  - whether an iSerialNumber exists;
  - the controller's interface number;
  - macOS IOKit parentage and bInterfaceClass under the data interface;
  - the Windows error code on hot unplug;
  - whether DTR gates output.
- **CI:** run the POSIX pty tests on Linux and macOS, and build the macOS listing against the real SDK.
  Nothing here proves it links against the real IOKit.
- **W0:** the Release build warns `-Wmaybe-uninitialized` in `protocol/src/doc_assembler.cpp:46` (W0 review S3). It is not in W3 files.
- **Brain (455) and naviGATR (626) suites were not rerun by W3:**
  - No file they compile changed: brain/spectuGATR/CMakeLists.txt adds only `spectuGATR/protocol`, and naviGATR references nothing in spectuGATR.
  - W4 is editing brain in parallel.

## Verification: exact commands and results (Git Bash)

Build and test (Windows, GCC 15.2 UCRT64, CMake, MinGW Makefiles):
```text
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake -S S/wt-spectu/spectuGATR -B S/sp-build/w3-platform-host2 -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug \
      -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=S/nav-base/_deps/googletest-src      -> exit 0
cmake --build S/sp-build/w3-platform-host2 -j16                                    -> exit 0, 0 warnings
ctest --test-dir S/sp-build/w3-platform-host2 -j16 --output-on-failure              -> 140/140 passed
(same with -DCMAKE_BUILD_TYPE=Release in S/sp-build/w3-platform-release2)          -> exit 0; 140/140 passed;
                                                                                      1 warning, W0 doc_assembler.cpp:46
S/sp-build/w3-platform-host2/tests/platform/spectugatr_platform_tests.exe --gtest_shuffle --gtest_repeat=5
                                                                                   -> 5 x "PASSED 34 tests"
... --gtest_filter=UsbPorts.* --gtest_output=json:S/sp-build/w3-platform-live2.json -> "ports": "0"
S/sp-build/w3-platform-probe/probe.exe (first run's raw SetupAPI probe)             -> 10 Ports-class devices,
                                                                                      all BTHENUM, all skipped; 0 USB ports
```
Logs: `S/sp-build/w3-platform-host2.{cfg,build,build2,test,test2}.log`,
`S/sp-build/w3-platform-release2.{cfg,build,test}.log`.

Warning gate, advisory (GCC 15.2, every W3 .cpp, -O0 and -O2), `S/sp-build/w3-platform-xcheck/gcc.log`:
- Flags: the CMAKE-13 `-Werror=` list plus the draft standard's host gate set
  (`-Wall -Wextra -Wpedantic -Wshadow -Wundef -Wmissing-declarations ... -Wold-style-cast -Wcast-align=strict ... -Wfloat-conversion`).
- Result: 32 compiles, 0 warnings, 0 errors.
- `-Wconversion -Wsign-conversion` over the 11 library files: 0.

Cross-compile (zig 0.13.0 = clang 18, unpacked at `S/sp-build/w3-platform-zig/whl/ziglang/zig.exe`), script
`S/sp-build/w3-platform-xcheck/xcheck.sh`, log `xcheck.log`:
- Targets: x86_64-linux-gnu, aarch64-linux-gnu, aarch64-macos, x86_64-macos.
- Files: every host/platform and tests/platform .cpp, compiled at `-O1` with the clang subset of the gate set plus the `-Werror=` names clang knows.
- Result: 64 compiles, 0 warnings, 0 errors.
- The x86_64 Linux test binary links with gtest, `-lutil -lpthread`: `spectugatr_platform_tests.elf`, ELF x86-64. Not run.
- macOS: zig ships no IOKit, CoreFoundation or `<util.h>`, so `S/sp-build/w3-platform-xcheck/macshim/` holds a hand-written transcription of the only
  declarations used: IOKitLib, IOSerialKeys, CoreFoundation types and functions, openpty.
- Real system headers came from zig: mach, termios, select, spawn.
- The macOS branch really compiled: `usb_ports_macos.o` is 195 KB on aarch64-macos and 0.9 KB on Linux.
- Negative control: a bad `IOObjectRelease` call fails ("no matching function").

Format and text:
- `clang-format 22.1.8` (`C:/Users/mbes1/AppData/Roaming/Python/Python314/Scripts/clang-format.exe`), with
  `--style=file --dry-run --Werror` over every W3 .h and .cpp: exit 0, after `-i` on
  usb_ports.cpp and usb_ports_macos.cpp. `/c/msys64/ucrt64/bin/clang-format` does not exist on this box.
- Every W3 file is plain ASCII (byte check).

git (worktree index only): `git add spectuGATR/host spectuGATR/tests/platform`; 26 files staged as `A`. No commit.
