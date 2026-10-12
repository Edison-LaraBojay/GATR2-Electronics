# Discovery: portable desktop receiver (Windows, macOS, Linux)

Topic: serial, device discovery, sockets, browser launch, paths, packaging, dependencies,
file layout, and Pi reuse of the same server code. Design input only; nothing in the repo
was changed.

Basis tags: [code] file:line verified in C:/GATR2-Electronics (HEAD ae457ee, clean tree),
[source] primary external source (URL, commit sha where the source is a repo),
[measured-host] run on this Windows 11 laptop (command given), [inference], [unknown].

Scratch: C:/Users/mbes1/AppData/Local/Temp/claude/c--GATR2-Electronics/827c2c12-2ec6-4e01-8e3e-e8b7e9ed237b/scratchpad/spectu/xplat-host/
  probe/ (serve_probe.cpp, enum_win.cpp, CMakeLists.txt), b2-dyn/, b2-static/, sleepprobe/,
  src/ + src2/ (downloaded primary sources used below)

Platforms actually built/run here: Windows 11 x64, MinGW-w64 UCRT64 GCC 15.2, CMake 4.2.
macOS and Linux were NOT built or run here (no WSL distro, no docker: `wsl -l -v` ->
"not installed"). No VEX hardware; the COM port enumeration probe found 0 ports.

-------------------------------------------------------------------------------

## 0. Summary

- The repo already has one portable piece: the HTTP/WebSocket server
  (pi/naviGATR/src/inspection/http_server.*) with a Winsock/POSIX shim. It links alone
  (depends only on core/host_clock.h, core/time.h) and runs on Windows with the embedded
  viewer: 1.40 MB fully static exe, no MinGW DLLs [measured-host].
- Everything serial is POSIX-only or Linux-only: termios SerialPort with a Windows stub
  that fails open; Brain USB discovery walks Linux sysfs; no Windows serial, no SetupAPI,
  no IOKit, no libudev, no browser launch, no per-OS paths anywhere in the repo [code].
- CI is ubuntu-latest only, 4 jobs, no Windows/macOS, no release packaging [code].
- Recommendation: zero-dependency native platform layer (~1.3k new LOC in 12 small files,
  free functions + data structs, one RAII class for the serial handle), reuse the existing
  server unchanged as a shared static library that naviGATR also links, embed the one
  viewer via one CMake function, ship static-ish single binaries from a GitHub Actions
  matrix (Windows MinGW static zip, macOS universal2 zip, Linux x64/arm64 tar.gz + udev
  rule). Libraries surveyed (libserialport, asio, cpp-httplib) either lack the one thing we
  need (USB interface number), or would replace the tested flow-control server.

-------------------------------------------------------------------------------

## 1. What the repo already has

### 1.1 Inventory

| Piece | Where | Portability | Basis |
|---|---|---|---|
| HTTP/1.1 + WebSocket server, flow control, replaceable slots | pi/naviGATR/src/inspection/http_server.h (227), .cpp (1614) | Winsock + POSIX select() shim | [code] http_server.cpp:16-38, 82-140 |
| Winsock refcounted WSAStartup | http_server.cpp:99-120 | Windows | [code] |
| SIGPIPE avoidance | http_server.cpp:124-128 uses MSG_NOSIGNAL only `#ifdef`; no signal(SIGPIPE) or SO_NOSIGPIPE anywhere | Linux ok; macOS depends on SDK defining MSG_NOSIGNAL | [code] grep SIGPIPE: only app/main.cpp:140-141 (SIGINT/SIGTERM) |
| Bind | AF_INET only, inet_pton numeric, default 127.0.0.1:8765 | all | [code] http_server.cpp:583-592, http_server.h:82-83 |
| Listener reuse | Windows SO_EXCLUSIVEADDRUSE, POSIX SO_REUSEADDR | all | [code] http_server.cpp:598-604 |
| Send-buffer bound | SO_SNDBUF + `#ifdef TCP_NOTSENT_LOWAT` | header says "(Linux)" but XNU also defines it (0x201) | [code] http_server.h:89-91, http_server.cpp:636-647; [source] https://github.com/apple-oss-distributions/xnu/blob/f6217f891ac0bb64f3d375211650a4c1ff8ca1ea/bsd/netinet/tcp.h (line 255) |
| Event loop | select(), 20 ms timeout; Windows FD_SETSIZE forced to 128 | POSIX select limited to fd < FD_SETSIZE (1024) | [code] http_server.cpp:17-18, 63, 1292-1323 |
| Host/Origin checks | none (no grep hit for origin/host in server, service, capture routes) | - | [code] |
| Embedded viewer | cmake/embed_assets.cmake -> generated viewer_assets.cpp; served paths incl. vendor/three.module.min.js | CMake script, portable | [code] pi/naviGATR/CMakeLists.txt:141-175, cmake/embed_assets.cmake:1-65, inspection/viewer_assets.h:13-21 |
| Dev override | static_root serves a directory first (traversal checked), else embedded | portable (std::filesystem) | [code] inspection_service.cpp:513-530, 1272-1290; runtime/inspection_config.h:18-19 |
| Viewer URLs | relative only: ws URL from location.host; fetch('/api/...'); importmap ./vendor/ | host-agnostic | [code] spectaGATR/feed.js:95-96, capture_panel.js:168-238, index.html:10 |
| Serial transport | transport/serial_port.cpp POSIX termios (cfmakeraw, VMIN/VTIME 0, O_NONBLOCK, CRTSCTS off); RS-485 DE via sysfs GPIO | `#if __unix__ || __APPLE__`; else stub: open fails "serial not supported on this platform" | [code] serial_port.cpp:21, 77-118, 256-318 (err at :268) |
| Exclusive open | none: no TIOCEXCL / flock | two processes can read the same tty | [code] serial_port.cpp:84 (open flags only) |
| Baud table | 9600..230400, 460800/921600 only `#ifdef` | macOS lacks B460800/B921600 constants | [code] serial_port.cpp:41-57 |
| Brain USB link (bench) | impl/resources/pros_usb_link.cpp: ReconnectingUsbPort (termios, poll POLLHUP -> close, 1 s rediscovery, 5 ms write bound) | POSIX; Windows path returns "Brain USB is supported on POSIX hosts only" | [code] pros_usb_link.cpp:27, 51-68, 89, 120, 143-171 |
| Brain USB discovery | findProsUsbUserPort: walks /sys/class/tty/ttyACM*/device parents for bInterfaceNumber=02, idVendor=2888, idProduct=0501; refuses >1 Brain | Linux sysfs only; no serial number read; macOS always empty | [code] pros_usb_link.cpp:180-207 (ambiguity :203), pros_usb_link.h:19-23 |
| Discovery tests | fake sysfs tree under temp dir; controller 2888:0503 if02 is explicitly NOT selected | run on Windows too (pure filesystem) | [code] tests/pros_usb_link_gtest.cpp:46-68, 197-216 |
| Reopen policy | ReopeningLink: reopen at most once per 1 s from read, never waits | portable logic | [code] impl/resources/serial_links.h:74-84 |
| WebSocket client | tests only (loopback 127.0.0.1, fixed mask) | test helper | [code] tests/inspection_test_client.h:3-4, 90, 317 |
| Process/CPU probes | tools/perf/perf_sys.cpp: GetSystemTimes vs /proc/stat | Windows/Linux | [code] perf_sys.cpp:3-10, 25-50, 51-63 |
| Browser finding (perf only) | run_perf.sh hard-codes chrome.exe/msedge.exe paths, google-chrome/chromium | Windows/Linux bash | [code] tools/perf/run_perf.sh:88-90 |
| Pi serial permissions | service gets SupplementaryGroups=dialout gpio | Pi | [code] tools/install_service.sh:69, 84 |
| bench/ | rs485_link: Pi-side python transmit + Brain main | Pi/Brain only, not relevant to host | [code] bench/README.md, bench/rs485_link/pi_transmit.py:1-40 |

### 1.2 CI

| Job | Runner | What | Basis |
|---|---|---|---|
| common | ubuntu-latest | translaGATR/tests via apt libgtest-dev | [code] .github/workflows/ci.yaml:9-26 |
| navigatr | ubuntu-latest | cmake -S pi/naviGATR, ctest (so POSIX http_server + serial compile on Linux here) | [code] ci.yaml:28-45 |
| brain | ubuntu-latest | brain host libs | [code] ci.yaml:49-66 |
| pico | ubuntu-latest | `pio run -e hat2_bno08x -e hat2_asm330` with working-directory `pico` | [code] ci.yaml:86-88 |

- No Windows, no macOS, no arm64, no artifacts, no release job [code].
- Contradiction: commit 2d11c67 moved platformio.ini to pico/aggreGATR (cache key updated,
  ci.yaml:81) but the build step still runs in `pico` (ci.yaml:88), which has no
  platformio.ini (`ls pico` -> aggreGATR README.md). The firmware step will likely fail
  [code + inference]. CI run history not visible (GitHub API 404, private repo) [unknown].
- Case hazard: working tree folder is `pi/navigatr` but the index says `pi/naviGATR`
  (core.ignorecase=true) [measured-host `ls -la pi`; `git ls-files -s pi/naviGATR/CMakeLists.txt`].
  Harmless on Windows/macOS default FS; Linux CI checks out the index spelling. New
  spectuGATR folders and include paths must use one exact case; keep the Linux CI job.

### 1.3 Measurements on this host

| What | Command | Result |
|---|---|---|
| Server tests on Winsock | `export PATH=/c/msys64/ucrt64/bin:$PATH; final-navi/tests/navigatr_tests.exe --gtest_filter='Inspection.*:InspectionFeed.*:UsbDiscovery.*:ProsUsbLink.*:SerialReopen.*:CaptureHttp.*'` | 47/47 passed [measured-host] |
| Server linked alone + embedded viewer | xplat-host/probe (http_server.cpp + generated viewer_assets.cpp + 60-line main), `cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DPROBE_STATIC=ON` | builds with -Wall -Wextra, no navigatr_core needed [measured-host] |
| Static exe size / DLLs | `objdump -p b2-static/serve_probe.exe` | 1,396,736 bytes; imports KERNEL32, WS2_32 (+ UCRT api-ms-win-crt-*) only [measured-host] |
| Dynamic exe DLLs | `objdump -p b2-dyn/serve_probe.exe`, `final-rel/navigatr.exe` | libstdc++-6, libgcc_s_seh-1, libwinpthread-1 [measured-host] |
| Serving | `serve_probe 8797 4` + curl + py raw WS client (ws_client.py) | GET / 200 11,872 B; /vendor/three.module.min.js 200 691,648 B; /ws 101, 17 state frames in 1 s at 50 ms cadence [measured-host] |
| Dynamic-DLL hazard | Git Bash default PATH (/mingw64/bin, Git's own libstdc++ first): `final-rel/navigatr.exe --help` | exit 127, no message; with ucrt64 first on PATH exit 0 [measured-host]. This is the known 0xc0000139 symptom; a user machine without MSYS2 would miss the DLLs entirely [inference]. Static link removes it. |
| Embedded asset weight | ls | viewer 338,115 B + three 691,648 + OrbitControls 32,134 = ~1.06 MB; generated .cpp 5.3 MB text, Release .obj 1.07 MB [measured-host] |
| Windows timer granularity | sleepprobe/sleep_probe.exe (static, -lwinmm) | sleep_for(1 ms): p50 15.2 ms, p90 16.1, max 30.5; with timeBeginPeriod(1): p50 15.3 ms (no improvement in this process) [measured-host] |
| Event-driven wake | sleepprobe/wake_probe.exe | condvar notify->wake p50 0.059 ms, p90 0.158; CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 1 ms: p50 1.62 ms [measured-host] |
| COM enumeration + known folders | b2-static/enum_win.exe (SetupAPI Ports class, CfgMgr32) | ports: 0 (no device); LocalAppData C:\Users\mbes1\AppData\Local; Documents C:\Users\mbes1\OneDrive\Documents (OneDrive-redirected) [measured-host] |

Consequence of the timer rows: on Windows the receiver must not pace or poll with
sleep_for; waits must be on I/O completion or events (serial ReadFile timeouts, select on
sockets, condition variables), else ~15 ms is added per sleep [measured-host + inference].
timeBeginPeriod semantics changed in Windows 10 2004 and Windows 11 [source]
https://learn.microsoft.com/en-us/windows/win32/api/timeapi/nf-timeapi-timebeginperiod
("Starting with Windows 10, version 2004, this function no longer affects global timer resolution.")

-------------------------------------------------------------------------------

## 2. Proposed platform layer

Scope: only what the desktop receiver needs. naviGATR keeps its own Linux/RS-485
transport (transport/, impl/resources/serial_links.*, pros_usb_link.*) unchanged.

Style: namespace spectugatr::host::platform; data in structs; free functions; one class
(SerialPort) because it owns an OS handle (RAII, move-only). Aliases for containers.

```cpp
namespace spectugatr::host::platform {

using Milliseconds = std::chrono::milliseconds;

struct SerialSettings {
    int  baud       = 115200;   // ignored by USB CDC, set for parity with PROS/pyserial
    bool assert_dtr = true;     // pyserial default, which pros-cli uses
    bool exclusive  = true;     // one reader per device
};

struct ReadResult {
    std::size_t bytes  = 0;
    bool        closed = false; // device gone: close, rediscover by identity
};

class SerialPort {              // move-only, closes in destructor
public:
    static std::optional<SerialPort> open(const std::string& path, const SerialSettings& settings,
                                          std::string& err);
    ReadResult read(uint8_t* destination, std::size_t capacity, Milliseconds wait); // first bytes or wait
    bool       writeAll(const uint8_t* source, std::size_t size, Milliseconds budget);
};

struct UsbSerialPort {          // one OS serial node
    std::string path;           // "\\\\.\\COM7", "/dev/cu.usbmodem1103", "/dev/ttyACM1"
    uint16_t    vid = 0, pid = 0;
    int         interface_number = -1;  // bInterfaceNumber, -1 unknown
    std::string serial;         // USB iSerialNumber, empty when absent
    std::string location;       // bus topology, fallback identity
};
using UsbSerialPorts = std::vector<UsbSerialPort>;
UsbSerialPorts listUsbSerialPorts();                 // one .cpp per OS

enum class VexRole { kBrainSystem, kBrainUser, kController, kUnknown };
struct VexPort { UsbSerialPort port; VexRole role; std::string identity; };
using VexPorts = std::vector<VexPort>;
VexPorts classifyVexPorts(const UsbSerialPorts& ports);          // pure, testable on any OS
std::optional<VexPort> pickPort(const VexPorts& ports, VexRole role,
                                const std::string& wanted_identity, std::string& err); // refuses ambiguity

bool openBrowser(const std::string& url, std::string& err);

struct AppPaths { std::filesystem::path config, state, captures; };
AppPaths appPaths();
}
```

### 2.1 Serial open/config/read/write

| | Windows | macOS | Linux |
|---|---|---|---|
| Open | CreateFileW(L"\\\\.\\COMn", GENERIC_READ/WRITE, share 0, OPEN_EXISTING) | open("/dev/cu.*", O_RDWR/O_NOCTTY/O_NONBLOCK/O_CLOEXEC) | same as macOS on /dev/ttyACMn (existing code serial_port.cpp:84) |
| Exclusive | inherent: "dwShareMode parameter must be zero (exclusive access)" | ioctl(TIOCEXCL) after open | ioctl(TIOCEXCL) |
| Config | GetCommState/SetCommState DCB: baud, 8N1, fBinary, fDtrControl=DTR_CONTROL_ENABLE, fRtsControl enable, no flow control; SetupComm(4096,4096); PurgeComm | cfmakeraw, CLOCAL/CREAD, CRTSCTS off, VMIN=VTIME=0 (reuse serial_port.cpp:98-106 logic); TIOCMBIS DTR/RTS | same |
| Read | synchronous handle, COMMTIMEOUTS {ReadInterval=MAXDWORD, ReadTotalMultiplier=MAXDWORD, ReadTotalConstant=wait}: returns as soon as bytes exist, else after wait | poll(POLLIN, wait) then read() | same |
| Write | WriteFile with WriteTotalTimeoutConstant = budget | loop write + poll(POLLOUT) until budget (pattern in pros_usb_link.cpp:89-117) | same |
| Threading | one reader/writer thread per device owns the handle (single transport owner); no second opener | same | same |

Sources:
- CreateFile communications resources: "the dwCreationDisposition parameter must be
  OPEN_EXISTING, the dwShareMode parameter must be zero (exclusive access)" and
  "To specify a COM port number greater than 9, use the following syntax: \\.\COM10"
  [source] https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilew
- COMMTIMEOUTS remark: with ReadIntervalTimeout and ReadTotalTimeoutMultiplier = MAXDWORD
  and 0 < ReadTotalTimeoutConstant < MAXDWORD, "If there are any bytes in the input buffer,
  ReadFile returns immediately ... If there are no bytes ..., ReadFile waits until a byte
  arrives and then returns immediately ... If no bytes arrive within the time specified by
  ReadTotalTimeoutConstant, ReadFile times out." [source]
  https://learn.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-commtimeouts
- macOS: "The callout device should almost always be used ... dialin device (/dev/tty.xxxxx)
  when monitoring a serial port for incoming calls"; "multiple open() calls to the same file
  will succeed unless the TIOCEXCL ioctl is issued" [source]
  https://developer.apple.com/library/archive/documentation/DeviceDrivers/Conceptual/WorkingWSerial/WWSerial_SerialDevs/SerialDevices.html
- pyserial (used by pros-cli): Windows opens with share 0 "exclusive access" and
  FILE_FLAG_OVERLAPPED (serialwin32.py:56-60), DTR/RTS default True (serialutil.py:210-211),
  POSIX exclusivity only via optional flock (serialposix.py:392-400) [source]
  https://github.com/pyserial/pyserial/tree/a5c48d445fbc1943d4fabf8d9090a50fda3172fd/serial
- pros-cli opens the V5 port at 115200 8N1 with pyserial defaults [source]
  https://github.com/purduesigbots/pros-cli/blob/45aae5df262eab1246f4d74c5be7bd1a4916eb73/pros/serial/ports/direct_port.py (lines 11-18)

Notes:
- Why synchronous + COMMTIMEOUTS instead of overlapped: one thread per device, reads wake
  on data (driver event, not the 15 ms timer), shutdown bounded by `wait`; ~100 fewer LOC
  than OVERLAPPED + WaitForMultipleObjects [inference]. Overlapped only if a device must be
  read and written concurrently from two threads, which the single-owner rule forbids.
- TIOCEXCL on POSIX makes a PROS terminal opened later fail instead of silently splitting
  bytes; a PROS terminal opened earlier (no TIOCEXCL, no flock) cannot be detected cheaply,
  so print "if data looks garbled, close other terminals" [inference]. On Windows the
  second opener gets an error; map ERROR_ACCESS_DENIED to "COMn in use by another program
  (PROS terminal, VEXcode)" [source above + inference].
- Existing naviGATR serial open has no exclusivity (serial_port.cpp:84) [code]; the same
  TIOCEXCL line would protect the Pi against a stray `pros terminal`/`screen` [inference].

### 2.2 Device discovery with stable identity

Device facts (primary sources):

| Device | VID:PID | Serial nodes | Role by interface | Source |
|---|---|---|---|---|
| V5 Brain (direct USB) | 2888:0501 | 2 | if 0 = system (VEX protocol), if 2 = user (program stdio) | vexide serial.rs:20-29, 91-117; pros-cli v5_device.py:56-57; naviGATR pros_usb_link.cpp:202 [code] |
| V5 Controller (tethered, radio to Brain) | 2888:0503 | 1 | single port; user stdio only via UserFifo request/reply on it | vexide serial.rs:28-29, 77-81, 480-503; https://vexide.dev/blog/posts/serial-deep-dive/ ("only one serial port"; "stdio output has to be received with a UserFifoPacket") |
| Baud | 115200 | | | vexide serial.rs:39 |

vexide: https://github.com/vexide/vex-v5-serial/blob/b3a712a16c9c6819b69faacfee2284826a138070/packages/vex-v5-serial/src/serial.rs
pros-cli: https://github.com/purduesigbots/pros-cli/blob/45aae5df262eab1246f4d74c5be7bd1a4916eb73/pros/serial/devices/vex/v5_device.py (lines 34-96)

How existing tools classify (why they are fragile): pros-cli matches pyserial `location`
suffix '2'/'0'/'1' or name text "User"/"System"/"Controller", on macOS the /dev name
suffix 3/1/2, and falls back to natural sort order of names when only two ports exist
(v5_device.py:51-92). vexide: "This does not appear to work on windows" and falls back to
name order on Windows and to the /dev name suffix on macOS (serial.rs:55-56, 89, 127-178,
180-217) because the serialport crate did not report interface numbers there.

Proposed per-OS enumeration (all give vid, pid, interface, serial, location):

| OS | Enumerate | Interface number | Serial / location | Notes |
|---|---|---|---|---|
| Windows | SetupDiGetClassDevsW(GUID_DEVCLASS_PORTS, DIGCF_PRESENT); PortName from SetupDiOpenDevRegKey | parse `USB\VID_2888&PID_0501&MI_02` from CM_Get_Device_IDW | CM_Get_Parent -> composite parent instance id (serial when the device has one); DEVPKEY_Device_ContainerId groups interfaces of one device | probe written and run: xplat-host/probe/enum_win.cpp (70 LOC) [measured-host, 0 ports] |
| macOS | IOServiceMatching(kIOSerialBSDServiceValue), take kIOCalloutDeviceKey (/dev/cu.*) | walk parents to IOUSBHostInterface, read "bInterfaceNumber" | parent device "USB Serial Number", "locationID" | serialport-rs does exactly this (enumerate.rs port_type) [source]; skip /dev/tty.* |
| Linux | /sys/class/tty/ttyACM*/device -> interface dir (bInterfaceNumber) -> parent (idVendor, idProduct, serial) | sysfs | serial file; devpath (e.g. 1-1.2) as location | generalizes findProsUsbUserPort (pros_usb_link.cpp:180-207); no libudev needed |

Sources:
- Windows MI_ format: "Each interface has a device ID of the following form:
  USB\VID_v(4)&PID_d(4)&MI_z(2) ... z(2) is the interface number that is extracted from the
  bInterfaceNumber field" [source]
  https://learn.microsoft.com/en-us/windows-hardware/drivers/install/standard-usb-identifiers
- Composite parent id parsing precedent (serialport-rs, MIT/Apache): parent HWID used for
  composite devices, examples `USB\VID_1D50&PID_6018&MI_02\6&A694CA9&0&0002` [source]
  https://github.com/serialport/serialport-rs/blob/cb9822a9956f2f828537c33db8604267ab2354df/src/windows/enum.rs (lines 115-184, 277-290)
- macOS/Linux precedent [source]
  https://github.com/serialport/serialport-rs/blob/cb9822a9956f2f828537c33db8604267ab2354df/src/posix/enumerate.rs (lines 446-480 IOKit, 496-509 matching, 700-745 sysfs)
- ContainerId: hub driver creates one "from the combination of the device's product ID (PID),
  vendor ID (VID), revision number, and serial number ... applies only to devices that have a
  unique serial number" [source]
  https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-containerids-in-windows
- Windows loads inbox Usbser.sys for class 02/subclass 02 CDC devices, no INF needed [source]
  https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-driver-installation-based-on-compatible-ids
- Linux by-id names are `$ID_BUS-$ID_SERIAL-if$ID_USB_INTERFACE_NUM` and only exist when
  ID_SERIAL and the interface number are known [source]
  https://github.com/systemd/systemd/blob/be1e78ec49f8c84c634d9e58dae834b27ea28436/rules.d/60-serial.rules (lines 21-25)

Identity rule (pure function, classifyVexPorts):
- role: PID 0503 -> controller; PID 0501 + if 0 -> brain_system, if 2 -> brain_user;
  interface unknown -> kUnknown (never guess by name order).
- identity string: `vex:<pid>:<serial>` when a USB serial exists, else `vex:<pid>@<location>`;
  the interface is part of the role, not the identity, so both Brain ports share one identity.
- Never persist COM numbers or ttyACM indices. `--device <path>` bypasses discovery.
- Ambiguity (two Brains, no --device/--identity): refuse and list candidates, as today
  (pros_usb_link.cpp:203) [code].
- Whether VEX devices report an iSerialNumber is unknown; without one, two identical
  devices are only distinguishable by bus location, and Linux by-id links collide or are
  absent [unknown -> bench].

### 2.3 Permissions

| OS | Default | Receiver behavior | Source |
|---|---|---|---|
| Windows | COM ports openable by the user; exclusive | map ERROR_ACCESS_DENIED to "in use" | CreateFile doc above |
| macOS | /dev/cu.* normally usable without admin | report errno text | [unknown] exact mode on current macOS -> bench |
| Linux | ttyACM nodes get GROUP="dialout" | on EACCES print: add user to dialout (Debian/Ubuntu/Pi OS; Arch uses uucp) and re-login, or install the shipped udev rule | [source] https://github.com/systemd/systemd/blob/be1e78ec49f8c84c634d9e58dae834b27ea28436/rules.d/50-udev-default.rules.in (line 47) |

Shipped Linux rule (packaging/linux/70-spectugatr.rules), no group change or re-login:

```
# VEX V5 Brain and Controller: seat user access, keep ModemManager away
SUBSYSTEM=="tty", ATTRS{idVendor}=="2888", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1"
```

- `uaccess` is applied by the builtin in 73-seat-late (`TAG=="uaccess|xaccess-*",
  ENV{MAJOR}!="", RUN{builtin}+="uaccess"`), so the rule file must sort before 73 [source]
  https://github.com/systemd/systemd/blob/be1e78ec49f8c84c634d9e58dae834b27ea28436/rules.d/73-seat-late.rules.in (line 17).
  It grants the active local seat user only; SSH sessions still need dialout [inference].
- ModemManager marks every tty a candidate (`SUBSYSTEM=="tty", ENV{ID_MM_CANDIDATE}="1"`)
  [source] https://github.com/linux-mobile-broadband/ModemManager/blob/338ecc06f294d57edea003b866080aacb4055917/src/80-mm-candidate.rules (line 21).
  Whether it actually probes the VEX ACM ports (strict filter policy) is [unknown]; the
  ID_MM_DEVICE_IGNORE convention is what PlatformIO ships for dev boards [source]
  https://github.com/platformio/platformio-core/blob/ff26956ecf0cdd44f6599894f11a83a93189e07f/platformio/assets/system/99-platformio-udev.rules (line 28 etc.).
- WSL: "Support for connecting USB devices is not natively available in WSL" (usbipd-win
  needed) [source] https://learn.microsoft.com/en-us/windows/wsl/connect-usb -> tell Windows
  users to run the Windows binary, not the Linux one under WSL.

### 2.4 Disconnect / reconnect

| Event | Detection | Action |
|---|---|---|
| Unplug (POSIX) | poll POLLHUP/POLLERR/POLLNVAL, or read() == 0/error (existing pattern pros_usb_link.cpp:51-68) | close, mark source "link lost" (stale, not replaced) |
| Unplug (Windows) | ReadFile/WriteFile/ClearCommError returns FALSE (exact error code [unknown]); plus the 1 Hz rediscovery no longer lists the identity | same |
| Replug / renumbered port | 1 Hz re-enumerate, match identity + role, reopen (existing 1 s policy pros_usb_link.cpp:27, serial_links.h:80) | new transport epoch; session identity still comes from the protocol hello, not the port |
| Controller power cycle / radio loss | port may stay present while the Brain is unreachable; that is a protocol timeout, not a port event | handled by the controller source [inference] |

- Polling enumeration at 1 Hz instead of OS notifications: zero extra APIs, identical logic
  on all three OSes, and matches existing code [inference]. Upgrade path if needed:
  Windows CM_Register_Notification (Windows 8+; "does not provide notification of existing
  device interfaces") [source]
  https://learn.microsoft.com/en-us/windows/win32/api/cfgmgr32/nf-cfgmgr32-cm_register_notification ;
  macOS IOServiceAddMatchingNotification; Linux netlink/udev monitor [inference].
- Enumeration cost with real devices not measured (0 ports here) [unknown].

### 2.5 Sockets: gaps to close when the server leaves the Pi

| Gap | Why it matters on a laptop | Fix | LOC |
|---|---|---|---|
| SIGPIPE on macOS | MSG_NOSIGNAL is defined in XNU only `#if __DARWIN_C_LEVEL >= 200809L`; if absent the fallback is flags 0 (http_server.cpp:124-128) and a peer reset kills the process | set SO_NOSIGPIPE on accepted sockets under `#ifdef SO_NOSIGPIPE` and ignore SIGPIPE at startup | ~10 |
| No Host/Origin check | any web page open in the same browser can open ws://127.0.0.1:port/ws or POST capture routes (browsers do not apply same-origin to WebSocket) | allowlist Host (127.0.0.1, localhost, [::1], --bind name) and Origin on /ws and POST; 403 otherwise | ~40 |
| AF_INET only | "localhost" may resolve to ::1 first | open the browser at http://127.0.0.1:PORT/, not localhost | 0 |
| select() fd limit (POSIX) | fds >= 1024 undefined with FD_SET | fine for a small receiver; switch to poll() only if needed | 0 |
| TCP_NOTSENT_LOWAT comment | header says Linux; macOS compiles it too | fix comment when moving | 0 |
| Outgoing connection to the Pi (combined mode) | needs a production WebSocket client (masked frames, reconnect); only a test client exists (tests/inspection_test_client.h) | new sources/navigatr_source | ~300 |

- XNU MSG_NOSIGNAL / SO_NOSIGPIPE [source]
  https://github.com/apple-oss-distributions/xnu/blob/f6217f891ac0bb64f3d375211650a4c1ff8ca1ea/bsd/sys/socket.h (lines 174, 636-638)
- RFC 6455 10.2: "Servers that are not intended to process input from any web page but only
  for certain sites SHOULD verify the |Origin| field" [source] https://www.rfc-editor.org/rfc/rfc6455#section-10.2
- macOS 15+ local network privacy: "Listening for and accepting incoming TCP connections: no",
  "Making an outgoing TCP connection: yes"; auto-allowed for "Command-line tools run from
  Terminal or over SSH" [source]
  https://developer.apple.com/documentation/technotes/tn3179-understanding-local-network-privacy
  -> loopback serving is fine; connecting to the Pi works when launched from Terminal, may
  prompt or be blocked if launched another way; mDNS (.local) discovery is UDP multicast and
  also needs the privilege [source same].

### 2.6 Browser launch

| OS | Call | Condition | Source |
|---|---|---|---|
| Windows | CoInitializeEx(STA, DISABLE_OLE1DDE); ShellExecuteW(nullptr, L"open", url, ...); success when > 32 | always (desktop) | [source] https://learn.microsoft.com/en-us/windows/win32/api/shellapi/nf-shellapi-shellexecutew |
| macOS | posix_spawnp("open", url) (LaunchServices; avoids linking AppKit NSWorkspace) | not over SSH | [source] https://developer.apple.com/documentation/appkit/nsworkspace/open(_:) (API behind `open`) + [inference] |
| Linux | posix_spawnp("xdg-open", url) | only if DISPLAY or WAYLAND_DISPLAY set; not as root | [source] https://portland.freedesktop.org/doc/xdg-open.html ("xdg-open is for use inside a desktop session only. It is not recommended to use xdg-open as root.") |

Always print the URL; open only after bind succeeded (port known); `--no-browser`.

### 2.7 Per-OS paths

| Use | Windows | macOS | Linux | Default policy |
|---|---|---|---|---|
| config | FOLDERID_RoamingAppData\spectuGATR | ~/Library/Application Support/spectuGATR | $XDG_CONFIG_HOME/spectugatr (~/.config) | optional file; CLI flags win |
| state (last device identity, logs) | FOLDERID_LocalAppData\spectuGATR | ~/Library/Application Support/spectuGATR | $XDG_STATE_HOME/spectugatr (~/.local/state) | small |
| captures | none by default | none | none | keep bundles in memory (same as Pi: capture_config.h:11 "directory empty keeps bundles in memory only"); export goes through the browser download (capture_panel.js:238); `--capture-dir` for auto-save |

- Avoid defaulting captures into Documents: on this host Documents is OneDrive-redirected
  [measured-host enum_win.exe], so bundles would sync to the cloud.
- Sources: SHGetKnownFolderPath https://learn.microsoft.com/en-us/windows/win32/api/shlobj_core/nf-shlobj_core-shgetknownfolderpath ;
  Application Support "Contains all app-specific data and support files" [source]
  https://developer.apple.com/library/archive/documentation/FileManagement/Conceptual/FileSystemProgrammingGuide/MacOSXDirectories/MacOSXDirectories.html ;
  XDG: "$XDG_STATE_HOME ... default equal to $HOME/.local/state" [source]
  https://specifications.freedesktop.org/basedir-spec/latest/

### 2.8 Offline embedded assets

- Keep cmake/embed_assets.cmake (moved to spectuGATR/cmake/) behind one function
  `spectugatr_embed_viewer(<target>)`; naviGATR and the desktop exe both call it on
  spectuGATR/viewer/. No loose files in the package; three.js license already embedded as
  vendor/LICENSE (pi/naviGATR/CMakeLists.txt:155-159) [code]. `--viewer-dir` = existing static_root dev
  override [code inspection_config.h:18-19].
- Cost: ~1.06 MB of bytes per binary; the hex-regex generator writes 5.3 MB of C++ that
  compiles to a 1.07 MB object [measured-host]. Acceptable; if build time matters, the
  generator can emit `#embed`-free `.incbin`/objcopy later [inference].

### 2.9 Console/lifecycle (small)

- Ctrl-C/SIGTERM -> stop (existing std::signal in app/main.cpp:140-141 works on MinGW
  consoles [inference]); Windows console-window close -> SetConsoleCtrlHandler
  (CTRL_CLOSE_EVENT) to flush captures [inference].
- Port in use: Windows SO_EXCLUSIVEADDRUSE makes a second receiver fail to bind
  (http_server.cpp:598-600) [code]; print "already running? open http://127.0.0.1:PORT/".

-------------------------------------------------------------------------------

## 3. Packaging prebuilt binaries (no compiler for users)

### 3.1 Matrix

| Artifact | Runner label | Toolchain | Link | Needs | Source |
|---|---|---|---|---|---|
| spectugatr-windows-x64.zip | windows-latest (Server 2025) | msys2/setup-msys2 msystem UCRT64, mingw-w64-ucrt-x86_64-gcc/cmake/ninja | `-static -s` | nothing on user side beyond Windows 10+ (UCRT is an OS component) | runner table [source] https://github.com/actions/runner-images/blob/7ef9dd0112f1827264dbca2dff8b13eb9f1a5534/README.md ; setup-msys2 https://github.com/msys2/setup-msys2 (c678c72) ; UCRT "included as part of the operating system in Windows 10 or later" https://learn.microsoft.com/en-us/cpp/windows/universal-crt-deployment |
| spectugatr-macos-universal.zip | macos-15 or macos-latest (arm64) | Apple clang, `-DCMAKE_OSX_ARCHITECTURES="arm64;x86_64" -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0` | libSystem/libc++ dynamic (system), no other deps | optional Developer ID + notarytool secrets | CMake var doc https://cmake.org/cmake/help/latest/variable/CMAKE_OSX_ARCHITECTURES.html ; x64 test runner `macos-15-intel` (runner table) |
| spectugatr-linux-x64.tar.gz | ubuntu-24.04 + container with old glibc (e.g. debian:bullseye or manylinux_2_28) | GCC | dynamic glibc, `-static-libstdc++ -static-libgcc` | tar + 70-spectugatr.rules + install-udev.sh | glibc warning below |
| spectugatr-linux-arm64.tar.gz | ubuntu-24.04-arm (GA partner image) | same container, arm64 | same | covers Pi 64-bit and ARM laptops | https://github.com/actions/partner-runner-images/blob/d14c9de077568464aa602b2009ead4771dac7ea6/README.md (line 36) |

CI checks per artifact (cheap, catch the known failure modes):
- Windows: `objdump -p spectugatr.exe | grep -E "libstdc|libgcc|libwinpthread"` must be empty
  (the 0xc0000139 class above) [measured-host basis].
- macOS: `lipo -archs` == "x86_64 arm64"; run ctest on arm64; optional run on macos-15-intel.
- Linux: max GLIBC_x.y symbol version from `objdump -T` <= target (Pi OS Bookworm glibc
  2.36-9+deb12u14, Trixie 2.41, Ubuntu 22.04 2.35, 24.04 2.39) [source]
  https://packages.debian.org/bookworm/libc6 , https://packages.debian.org/trixie/libc6 ,
  https://packages.ubuntu.com/jammy/libc6 , https://packages.ubuntu.com/noble/libc6

### 3.2 OS trust prompts (no signing budget assumed)

- Windows: SmartScreen checks "downloaded files against a list of files that are well known
  and downloaded frequently. If the file isn't on that list, Microsoft Defender SmartScreen
  shows a warning" [source]
  https://learn.microsoft.com/en-us/windows/security/operating-system-security/virus-and-threat-protection/microsoft-defender-smartscreen/
  -> unsigned zip works after "More info -> Run anyway"; a code-signing certificate is the
  only real fix [inference].
- macOS: arm64 requires a signature, "a simple ad-hoc signature is sufficient", which the
  linker applies, "However ... binaries signed this way cannot pass through Gatekeeper"
  [source] macOS Big Sur 11.0.1 universal apps release notes
  (https://developer.apple.com/documentation/macos-release-notes/macos-big-sur-11_0_1-universal-apps-release-notes).
  Developer ID software "must be notarized" (10.15+) [source]
  https://developer.apple.com/documentation/security/notarizing-macos-software-before-distribution .
  For a bare CLI binary in a zip: "you can't staple to it directly ... Although tickets are
  created for standalone binaries, it's not currently possible to staple tickets to them"
  [source] https://developer.apple.com/documentation/security/customizing-the-notarization-workflow
  -> notarized binary still needs an online Gatekeeper check on first run; unsigned binary
  needs System Settings > Privacy & Security > "Open Anyway" [source]
  https://support.apple.com/en-us/102445 , or `xattr -d com.apple.quarantine` [inference].
  Downloads by curl do not get the quarantine attribute [inference].
- Linux: none. AppImage not recommended: "AppImages require FUSE to run" [source]
  https://docs.appimage.org/user-guide/troubleshooting/fuse.html ; it targets GUI apps and
  adds a runtime for a CLI that only opens the system browser [inference].
- Fully static glibc is not recommended if the receiver resolves hostnames (Pi by name or
  .local): glibc marks getaddrinfo "in statically linked applications requires at runtime
  the shared libraries from the glibc version used for linking" [source]
  https://github.com/bminor/glibc/blob/04e750e75b73957cf1c791535a3f4319534a52fc/include/libc-symbols.h (lines 244-248).
  musl static would drop NSS/.local resolution [inference].

-------------------------------------------------------------------------------

## 4. Dependency choice

| Option | License | Size / form | Gives | Missing / cost | Verdict |
|---|---|---|---|---|---|
| Native, zero-dependency (ours) | none | ~1.3k new LOC; server already written | exactly the APIs above; static single binary proven at 1.40 MB incl. viewer [measured-host] | we own ~700 LOC of per-OS code | recommend |
| libserialport (sigrok) | LGPL-3.0+ ("released under the LGPL3+ license") | C library, autotools or VS2019 project (no CMake) | serial I/O on Win/mac/Linux/FreeBSD; enumeration: sp_get_port_usb_vid_pid, _serial, _bus_address, _manufacturer, _product, _description | no USB interface-number API (no match for "interface" in libserialport.h), so Brain system vs user port still needs our own per-OS code; LGPL relink obligations for a static prebuilt binary; foreign build system | no |
| asio standalone | BSL-1.0 | header-only, large | async sockets, serial_port (overlapped/POSIX) | no enumeration, no HTTP/WS; replacing a tested server | no |
| cpp-httplib | MIT | single header 802,863 B [measured-host curl] | HTTP + WebSocket | "blocking I/O WebSocket implementation using a thread-per-connection model (plus one heartbeat thread per connection)"; no hook for our replaceable slots and ping-acked window, which W1 validated | no |
| serialport-rs, pyserial | MIT/Apache, BSD | other languages | reference implementations only | - | cite, do not link |
| libudev | LGPL-2.1+ | Linux .so | enumeration | dynamic dep; sysfs gives the same fields | no |

Sources: libserialport README lines 14, 20, 25-28, 43-53 and libserialport.h lines 633-714
https://github.com/sigrokproject/libserialport/tree/21b3dfe5f68c205be4086469335fd2fc2ce11ed2 ;
asio COPYING https://github.com/chriskohlhoff/asio/blob/8806a6803cde7054c3049d3666d3ec36786568c5/COPYING ;
cpp-httplib LICENSE (MIT) and README-websocket.md line 6
https://github.com/yhirose/cpp-httplib/tree/e11dbec7b3761b428224044578491b62c776466b .

Reasons for native: the server (the hard part) is already portable and tested [measured-host];
no library supplies interface-aware VEX discovery on all three OSes (serialport-rs needed
per-OS fixes, vexide still falls back to name order); zero deps keeps universal2 and static
Windows builds trivial; repo precedent is vendored single-purpose code (tinyxml2, stb,
apriltag, three) with no package manager [code pi/naviGATR/CMakeLists.txt:11-65].

-------------------------------------------------------------------------------

## 5. File layout and LOC

Each pipeline folder gets a short README.md explaining its stage (requested style).
Namespaces: spectugatr::server, spectugatr::host::platform, spectugatr::host::sources.

```
spectuGATR/
  README.md                         whole product: sources -> receiver -> server -> viewer
  CMakeLists.txt                    options: SPECTUGATR_BUILD_HOST, SPECTUGATR_BUILD_TESTS
  cmake/
    embed_viewer.cmake              moved embed_assets.cmake + spectugatr_embed_viewer()      ~80
  viewer/                           moved spectaGATR (one implementation)
  host/
    README.md                       receiver pipeline: discover -> open -> source -> merge -> serve
    main.cpp                        args, start server, start sources, open browser            ~120
    command_line.h/.cpp             --source, --device, --identity, --config, --replay,
                                    --port, --bind, --no-browser, --capture-dir, --viewer-dir  ~200
    server/                         lib spectugatr_server (shared with naviGATR)
      README.md
      http_server.h/.cpp            moved as is + SO_NOSIGPIPE + Host/Origin allowlist    1841+50
      json_writer.h                 moved                                                    179
      zip_writer.h/.cpp             moved (crc32 from translaGATR)                           184
      viewer_routes.h/.cpp          embedded table + static_root, from inspection_service    ~120
    platform/                       lib spectugatr_platform (desktop only)
      README.md
      serial_port.h                 SerialSettings, ReadResult, class SerialPort              ~60
      serial_port_posix.cpp         termios, TIOCEXCL, DTR, poll                             ~170
      serial_port_win32.cpp         CreateFileW, DCB, COMMTIMEOUTS, ClearCommError           ~200
      usb_ports.h                   UsbSerialPort, UsbSerialPorts, listUsbSerialPorts()       ~40
      usb_ports_linux.cpp           sysfs                                                    ~110
      usb_ports_macos.cpp           IOKit + CoreFoundation                                   ~160
      usb_ports_win32.cpp           SetupAPI + CfgMgr32                                      ~150
      vex_ports.h/.cpp              classifyVexPorts, pickPort (pure)                        ~110
      browser.h/.cpp                ShellExecuteW / open / xdg-open                           ~70
      app_paths.h/.cpp              known folders / Application Support / XDG                 ~90
      process.h/.cpp                SIGPIPE ignore, stop signals, console close               ~50
    sources/                        desktop source adapters (other discovery agents size these)
      README.md
      device_watch.h/.cpp           1 Hz rediscovery, reopen by identity, link epochs         ~90
      brain_usb_source.*            Brain user port stream
      controller_source.*           UserFifo polling through the controller port
      navigatr_source.*             WebSocket client to the Pi feed                          ~300
      replay_source.*               capture bundle playback
  packaging/
    README.md
    linux/70-spectugatr.rules, install-udev.sh                                               ~20
  tests/
    host/vex_ports_gtest.cpp        fixtures for Windows MI_xx ids, macOS names, sysfs trees  ~200
    host/serial_posix_gtest.cpp     openpty loopback (POSIX CI only)                         ~120
    host/paths_browser_gtest.cpp    pure parts (URL, env fallbacks)                           ~80
    server/                         moved server tests (inspection_gtest, inspection_feed_gtest parts)
.github/workflows/release.yaml      matrix of section 3                                      ~150
```

New code outside sources/: ~1.3k LOC platform + ~0.4k tests + ~0.2k packaging/CI.
Per-OS files are the only `#ifdef`-free split: CMake picks
`*_win32.cpp` / `*_macos.cpp` / `*_linux.cpp` / `*_posix.cpp` by platform, so no file mixes
OS branches (the existing server keeps its shim; rewriting it is not proposed).

### 5.1 How naviGATR reuses the same server without forking

- naviGATR CMakeLists.txt today compiles http_server.cpp, zip_writer.cpp, the embedded
  viewer, and everything else into one navigatr_core (pi/naviGATR/CMakeLists.txt:128-138, 141-182) [code].
- Change: naviGATR adds the shared library the same way it already consumes translaGATR by
  REPO_ROOT (pi/naviGATR/CMakeLists.txt:7, 180-183) [code]:

```cmake
if(NOT TARGET spectugatr_server)
    add_subdirectory(${REPO_ROOT}/spectuGATR/host/server ${CMAKE_BINARY_DIR}/spectugatr_server)
endif()
target_link_libraries(navigatr_core PUBLIC spectugatr_server)
spectugatr_embed_viewer(navigatr_core)
```

- naviGATR keeps InspectionService (depends on runtime/system.h, inspection_service.h:44)
  as its source-specific adapter; it does not link spectugatr_platform.
- Churn: 16 files include the four moved headers; 218 references to server types [code grep].
  Either update includes and add `namespace navigatr { using spectugatr::server::HttpServer; ... }`
  in one naviGATR header, or rename references; no behavior change. Server tests move with the
  server; naviGATR keeps its feed/service tests.
- Pi artifacts stay native builds on the Pi; optionally CI on ubuntu-24.04-arm inside a
  debian:bookworm container to match Pi OS glibc [inference].
- The extraction inventory of what else is generic (capture routes, JSON) is in
  discovery/pi-side.md section 4; not repeated here.

-------------------------------------------------------------------------------

## 6. Doc/code contradictions noticed

1. http_server.h:89-90 says TCP_NOTSENT_LOWAT is Linux; XNU defines it, so macOS builds use it too [code + source].
2. ci.yaml:88 runs `pio run` in `pico` but platformio.ini moved to pico/aggreGATR (2d11c67) [code].
3. Working-tree folder `pi/navigatr` vs index `pi/naviGATR` [measured-host].
4. serial_port.h:2-3 says non-POSIX "open fails so the rest of the app still builds" - true, and
   pros_usb_link.cpp:120 says POSIX only; any doc implying the desktop can talk to a Brain today is wrong [code].
5. vexide says Windows interface detection "does not appear to work" (serial.rs:55-56); the Windows
   device ID does carry MI_xx per Microsoft's documented format, so the limitation is the
   serialport crate path they used, not Windows [source + inference; needs a real device to confirm].

-------------------------------------------------------------------------------

## 7. Open questions / needs bench test

1. Do the V5 Brain and V5 Controller report a USB iSerialNumber? (Windows: parent instance id
   without '&'; Linux: `cat /sys/class/tty/ttyACM*/device/../serial`, `udevadm info`; macOS:
   `ioreg -p IOUSB -l`.) Decides whether identity is serial- or location-based.
2. Controller (2888:0503) interface layout: exactly one CDC ACM port? which bInterfaceNumber?
   (pros-cli's '1' location suffix and vexide's macOS name suffix '2' suggest interface 1.)
3. Windows device IDs on a real Brain: confirm `USB\VID_2888&PID_0501&MI_00` / `MI_02` and the
   COM friendly names with and without the VEXcode driver package installed.
4. macOS node names and the bInterfaceNumber IOKit property for Brain and controller; confirm
   /dev/cu.* permissions without admin.
5. Does the Brain or controller gate output on DTR? (pyserial/pros-cli assert DTR by default;
   test with DTR off and on.)
6. Windows unplug: which error ReadFile/ClearCommError return after a hot unplug with the
   COMMTIMEOUTS mode chosen; does a replug get the same COM number.
7. Linux: does ModemManager probe the VEX ACM ports without the ignore rule (watch for
   injected AT bytes on the Brain console)?
8. Does the Brain's USB stay enumerated across program stop/start and field-control
   transitions, or does the receiver see a disconnect?
9. Enumeration cost at 1 Hz with devices present on each OS (0 ports here).
10. Build and run the platform layer on macOS (universal2) and Linux (x64, arm64) in CI;
    nothing here was compiled for those targets.
11. Code signing: is there budget/appetite for an Apple Developer ID and a Windows code-signing
    certificate, or do users accept the first-run prompts? (user decision)
