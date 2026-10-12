# Pi/USB fallback procedure

**Local archive update:** communiGATR and translaGATR have also been removed
from the active tree and preserved under this directory. Use the pinned
`10c568e` checkout in section 1 when rebuilding the fallback. The later
in-place build instructions predate this retirement and no longer apply.

Fallback for a demonstration without MCU localization:

```
Pico (encoders) --UART--> Pi naviGATR --USB--> V5 Brain (operaGATR)
                          Pi viewer (old spectaGATR) <-- ssh tunnel <-- laptop browser
```

This is the pre-refactor system. Commit `20f20e2` moved it under `deprecated/`.
Commit `10c568e` (2026-09-30) is the last commit that has it in its original place.
Nothing in this file has been run on hardware since the move. The table in
section 6 lists what was checked and what was not.

## What the fallback is

| Part | Source | Notes |
|---|---|---|
| Pi runtime and old viewer | `deprecated/pi/naviGATR` | All 425 files are byte-identical to `pi/naviGATR` at `10c568e`, apart from the build fixes in section 3. The Pi sources have not changed since `ae457ee`. The viewer (`spectaGATR/*.js`, `third_party/three`) is compiled into the executable. |
| Pi config | `config/override/brain_profile_usb.xml` | USB Brain link, Pico on `/dev/ttyAMA0` at 115200 baud, viewer on 127.0.0.1:8765. Holds no robot geometry: the Brain sends it. |
| Brain app | `deprecated/brain/operaGATR` (slot 1) | This drive app ran with the Pi viewer: its README says manual driving was tested. The sticks drive the robot. A, X and Y start planned moves, and `autonomous()` runs the test moves. |
| Brain robot profile | `deprecated/brain/robot/gatr2_robot.h` | USB link. VEX IMU on Smart Port 20. Forward wheel on encoder 1, sideways wheel on encoder 0, both reversed. Start pose (7 in, 24 in, 0 deg). Byte-identical to `10c568e`. |
| Motor-free Brain app | `deprecated/brain/locaGATR` (slot 2) | Uses the same profile, and you move the robot by hand. Its README says "not yet run on a robot". |
| Pico firmware | `pico/aggreGATR` **at 10c568e**, env `hat2_bno08x` (default) or `hat2_asm330` | Only this source sends the Pi telemetry. |

Pico firmware warnings:

- The current `pico/aggreGATR` (20f20e2 and later) does acquisition only and has no
  Pi UART stream. Never flash it for this fallback.
- `deprecated/pico/aggreGATR` is a partial snapshot and does not build (section 4).
- Pick the env that matches the IMU chip on the HAT. With the VEX IMU profile the Pico
  IMU is not used. A missing chip only clears the gyro bit, and encoders still stream.

Archived docs under `deprecated/` still use the old paths, such as `pi/naviGATR` and
`brain/operaGATR`. Prefix them with `deprecated/`, or use this file.

## 0. Check first: maybe nothing needs rebuilding

The Pi and Pico sources did not change from `ae457ee` to `10c568e`, and the Brain
libraries did not change from `a2f5dd9` to `10c568e`. If the Pi and Pico still run
builds from that range, they need no rebuilding or reflashing.

On the Pi:

```sh
systemctl status navigatr --no-pager
systemctl cat navigatr | grep -E 'ExecStart|WorkingDirectory'
ls -l $(systemctl cat navigatr | sed -n 's/^ExecStart=\([^ ]*\).*/\1/p')
ls -l $(systemctl cat navigatr | sed -n 's/.*--config_file=\([^ ]*\).*/\1/p')
```

The service is fine when:

- it is `active (running)`;
- both files exist;
- the viewer (section 5) shows the field;
- with the Brain connected, the Brain link panel's raw wheel readings change when the
  wheels turn. If they never change, the Pico is not sending: see 1c.

If the unit's config path is missing, the Pi checkout was probably updated to
20f20e2: see 1d.

On the Brain, `operaGATR` must be in a slot and built from the `10c568e` profile above.
If you are not sure, rebuild and upload it (1b). This takes about a minute.

## 1. Recommended route: pinned commit 10c568e

This builds the last committed pre-refactor source, and the active tree stays as it
is. The Pi and Pico sources are the ones used in the 2026-09-29/30 tests. The Brain
app is its last committed version. That commit is titled "failed testing day 2", and
the repo does not record what failed. Its diff adds link diagnostics on the Brain
screen, wheel travel multipliers and a new start pose.

### 1a. Laptop: get the source (PowerShell)

```powershell
cd C:\GATR2-Electronics
git worktree add ..\GATR2-fallback 10c568e
```

After the demo, remove it with `git worktree remove ..\GATR2-fallback`.

### 1b. Brain: build and upload operaGATR

The PROS kernel files are not in git. Copy the local leftovers, or run
`pros c apply kernel@4.2.2 --force-apply --no-download` in the project folder,
then `git checkout -- .gitignore`.

```powershell
Copy-Item -Recurse C:\GATR2-Electronics\brain\operaGATR\firmware C:\GATR2-fallback\brain\operaGATR\
Copy-Item -Recurse C:\GATR2-Electronics\brain\operaGATR\include\pros C:\GATR2-fallback\brain\operaGATR\include\
cd C:\GATR2-fallback\brain\operaGATR
pros make
pros upload --slot 1 --name operaGATR --after screen
```

The upload goes over the Brain USB port, or over a tethered or radio controller. If
the Brain USB cable is in the Pi, move it to the laptop for the upload, then put it
back in the Pi.

For locaGATR, use the same steps in `brain\locaGATR` with `--slot 2 --name locaGATR`.

### 1c. Pico: only if it does not run pre-refactor firmware

Connect the Pico to the laptop by USB.

```powershell
cd C:\GATR2-fallback\pico\aggreGATR
pio run -e hat2_bno08x
pio run -e hat2_bno08x -t upload
```

Use `-e hat2_asm330` in both commands if the HAT carries an ASM330.

### 1d. Pi: only if its build or service is missing or broken

Laptop (PowerShell), copy the pinned Pi source:

```powershell
git -C C:\GATR2-Electronics archive --format=tar.gz -o "$env:TEMP\navigatr-10c568e.tar.gz" 10c568e pi/naviGATR translaGATR
scp "$env:TEMP\navigatr-10c568e.tar.gz" USER@PI_HOST:/tmp/
```

Pi: build and install the service. A Pi 4 build takes several minutes with `-j2`.

```sh
mkdir -p ~/gatr2-10c568e
tar -xzf /tmp/navigatr-10c568e.tar.gz -C ~/gatr2-10c568e
cd ~/gatr2-10c568e/pi/naviGATR
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_TESTS=OFF -DNAVIGATR_WITH_LIBCAMERA=OFF
cmake --build build-bench -j2
sudo bash tools/install_service.sh --user "$USER"
```

If the Pi is a git clone that already has `10c568e`, you can replace the copy steps
with `git -C ~/GATR2-Electronics worktree add ~/gatr2-10c568e 10c568e`.

**About `install_service.sh`:**

- The installer defaults to `PROJECT/build-bench/navigatr` and
  `PROJECT/config/override/brain_profile_usb.xml`, where PROJECT is the folder above
  `tools/`.
- It writes absolute paths into `/etc/systemd/system/navigatr.service` and backs up
  any previous unit.
- It then enables and restarts the service.
- Use `--dry-run` to print the unit without installing it.

**If a git checkout on the Pi was pulled to 20f20e2:**

- The old unit points at `<checkout>/pi/naviGATR/config/override/brain_profile_usb.xml`.
  The pull deleted that file, so navigatr exits with
  `config error: cannot open ...` and systemd restarts it every 3 s.
- The old ignored build directory `<checkout>/pi/naviGATR/build-bench/` survives the
  pull, but it can no longer be rebuilt there.
- A fresh configure in `deprecated/pi/naviGATR` at 20f20e2 fails until the section 3
  fixes are present. They are uncommitted.
- The quickest recovery needs no build. Reuse the surviving binary with the moved
  config, which is byte-identical:

```sh
ls -l ~/GATR2-Electronics/pi/naviGATR/build-bench/navigatr
cd ~/GATR2-Electronics/deprecated/pi/naviGATR
sudo bash tools/install_service.sh --user "$USER" --binary "$HOME/GATR2-Electronics/pi/naviGATR/build-bench/navigatr"
```

If that binary is gone, use the pinned build above.

After this recovery the service runs from files inside the Pi's git checkout.
**Do not `git pull` on the Pi again before the demo:** if a later commit stops
tracking `deprecated/` (a local-only archive is being considered), a pull would
delete those files and the service would restart-loop with
`config error: cannot open`. The pinned build in `~/gatr2-10c568e`, outside any
checkout, does not have this problem.

## 2. Alternative route: build in the deprecated location

This uses the current tree with the section 3 fixes. It is checked by host builds
only.

The Brain app runs with the **current** active communiGATR, investiGATR and actuGATR,
which 20f20e2 refactored. Prefer route 1 for the demo.

Brain (PowerShell). Run the two `Copy-Item` lines once only:

```powershell
cd C:\GATR2-Electronics\deprecated\brain\operaGATR
Copy-Item -Recurse ..\..\..\brain\operaGATR\firmware .
Copy-Item -Recurse ..\..\..\brain\operaGATR\include\pros include\
pros make
pros upload --slot 1 --name operaGATR --after screen
```

`firmware/`, `include/pros/` and `bin/` are ignored by the project `.gitignore`.

The Pi needs the uncommitted fixes, so copy the working tree. On the laptop
(PowerShell, from `C:\GATR2-Electronics`):

```powershell
tar.exe -czf "$env:TEMP\navigatr-fallback.tar.gz" --exclude=build --exclude=build-* deprecated/pi/naviGATR translaGATR
scp "$env:TEMP\navigatr-fallback.tar.gz" USER@PI_HOST:/tmp/
```

On the Pi:

```sh
mkdir -p ~/gatr2-fallback
tar -xzf /tmp/navigatr-fallback.tar.gz -C ~/gatr2-fallback
cd ~/gatr2-fallback/deprecated/pi/naviGATR
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_TESTS=OFF -DNAVIGATR_WITH_LIBCAMERA=OFF
cmake --build build-bench -j2
sudo bash tools/install_service.sh --user "$USER"
```

There is no in-place Pico build (section 4). Use 1c.

## 3. Fixes made for the deprecated location

All fixes are inside `deprecated/`. No active library was changed.

- `pi/naviGATR/CMakeLists.txt`:
  - `REPO_ROOT` is now three levels up, so `translaGATR/` resolves again.
  - New `DEPRECATED_ROOT` for `brain/operaGATR/include/field_references.h`.
- `pi/naviGATR/tests/CMakeLists.txt`: the field-references path uses `DEPRECATED_ROOT`.
- `pi/naviGATR/tests/brain_link_e2e_gtest.cpp`: the active `actugatr::Motion` takes the
  pose source and the field source separately, so the test passes the link driver
  for both.
- `brain/operaGATR/Makefile` and `brain/locaGATR/Makefile`:
  - `GATR2_ROOT ?= ../../..`.
  - `deprecated/brain/robot` added to the include path; it was removed from
    `brain/gatr2_brain.mk`.
  - operaGATR also compiles the archived TELEMETRY adapter
    `deprecated/brain/actuGATR/src/telemetry.cpp`, with its include directory.
- `brain/operaGATR/src/main.cpp`: `Motion(*g_link, *g_link, ...)`. The ProsLink is both
  pose and field source, as before the refactor.

## 4. Pico in-place blocker

`deprecated/pico/aggreGATR` holds only the Pi-link parts. It cannot build alone:

- `platformio.ini:16,18` point at `../../translaGATR`, which does not exist under
  `deprecated/`.
- `src/` has no quadrature decoder, IMU drivers or supervisors, and no `third_party/sh2`.
- The active copies of those files no longer define `imu::firmware()`, which the
  archived `src/main.cpp:196,218` calls.
- The active `board.h` and `config.h` dropped `kPiTxPin`, `kPiRxPin` and `kPiBaud`,
  which `main.cpp` uses.

Mixing the archived main loop with the current drivers would give firmware that never
ran on hardware, so use the pinned build (1c).

## 5. Run the demo

1. Pico powered and wired to the Pi HAT. Pi powered, with the service running.
   Brain USB plugged into the Pi.
2. Place the robot at the configured start pose: (7 in, 24 in), facing field +x. You
   can edit `kStartX`, `kStartY` and `kStartHeadingDegrees` in `gatr2_robot.h`, then
   rebuild and upload the Brain.
3. Start `operaGATR` from the Brain screen and keep the robot still while the VEX IMU
   calibrates (about 2 s). The Brain sends its profile. The Pi logs
   `event: profile ... applied`, then the Brain places the robot. The screen shows
   link diagnostics until the link connects.
4. Open the viewer from the laptop with
   `ssh -N -L 8765:127.0.0.1:8765 USER@PI_HOST`, then browse to
   <http://localhost:8765/>. The viewer needs no internet: three.js is compiled in.
   When the link is up, the Brain link panel reads `ready` and the robot moves on the
   field.
5. After a USB outage longer than 250 ms, or a Brain program restart, the Pi drops the
   placement. The link reconnects by itself. Put the robot back at the start pose and
   press UP in operaGATR, or restart the program.

Service control on the Pi:

```sh
sudo systemctl restart navigatr
sudo systemctl stop navigatr                     # before a manual run
journalctl -u navigatr -b -n 80 --no-pager
./build-bench/navigatr --config_file=config/override/brain_profile_usb.xml   # manual run, from the naviGATR folder
```

Operating mode: while the Brain USB feeds the Pi, the old Pi viewer is the display.

- Do not run the new spectaGATR receiver against the controller. Its controller polling
  next to the Brain-to-Pi USB link has not been tested, and it may take bytes from the
  same Brain console stream.
- Do not open `pros terminal` on the Brain in this mode.
- Do not upload a program that uses the new spectaGATR publisher in this mode.

## 6. What was verified

All checks ran on Windows (2026-10-09) in scratch copies. Nothing ran on Linux or on
hardware.

| Check | Result |
|---|---|
| `deprecated/pi/naviGATR` blobs vs `10c568e:pi/naviGATR` (425 files, viewer and third_party included) | identical |
| Pinned `10c568e` naviGATR host build and tests (MinGW GCC 15.2) | 626/626 passed |
| In-place `deprecated/pi/naviGATR` host build and tests (section 3 fixes) | 626/626 passed, 0 warnings |
| In-place build serves the embedded viewer (synthetic demo config) | HTTP 200, byte sizes match; headless Chrome rendered the field |
| `install_service.sh --dry-run` from `deprecated/pi/naviGATR` | unit points at `deprecated/pi/naviGATR/build-bench/navigatr` and its `brain_profile_usb.xml` |
| Pinned `10c568e` operaGATR and locaGATR, PROS 4.2.2, arm-none-eabi GCC 13.3.1 | compile and link OK |
| In-place operaGATR and locaGATR against the current active libraries | compile and link OK, 0 warnings |
| Archived TELEMETRY adapter tests against the current actuGATR (scratch harness) | 7/7 passed |
| Pinned `10c568e` Pico firmware, `pio run -e hat2_bno08x` and `-e hat2_asm330` | build OK (no upload) |
| In-place Pico firmware | not possible (section 4) |
| Pi (Linux) build of either route, the service on the Pi | not verified |
| Any upload or flash, USB link, Pico UART, live pose, viewer over ssh | not verified |
