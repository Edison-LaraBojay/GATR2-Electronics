# spectaGATR

Live robot pose viewer. The Brain program publishes the pose it already
computed; a small laptop receiver passes it to a browser that draws the field,
the robot, its heading or full attitude, a trail and the numbers.

```
Brain program --stdout--> V5 controller --radio/USB--> laptop: spectagatr receiver --> browser
     (brain/spectaGATR)     or Brain USB user port       (spectaGATR/receiver)          (spectaGATR/web)
```

Position and orientation come from the Brain program. The laptop owns the
field and robot drawing. For the active Pico -> Pi relay -> Brain odometry
path, start with the
[encoder localization demo guide](../brain/localization-demo/README.md),
including its visualization routes and hardware limitations.

| Folder | What |
|---|---|
| [`protocol/`](protocol/include/spectagatr/protocol/pose_line.h) | Wire format v1, shared by the Brain and the receiver ([PROTOCOL.md](docs/PROTOCOL.md)) |
| [`receiver/`](receiver/README.md) | Laptop program (C++17, CMake): controller, Brain USB, sim and replay sources, web server |
| [`web/`](web/README.md) | Browser app (JavaScript, three.js vendored, offline) |
| [`fields/`](fields/README.md) | Field definitions: `override.json` (V5RC Override, from the archived field.xml), `empty.json` |
| [`tools/`](tools/) | `mock_feed.py` (browser-only mock), `ws_probe.py` (feed checker) |
| [`../brain/spectaGATR/`](../brain/spectaGATR/README.md) | Brain publisher library |
| [`../brain/spectaGATR-demo/`](../brain/spectaGATR-demo/README.md) | Example PROS program; never moves motors |
| [`../brain/localization-demo/`](../brain/localization-demo/README.md) | Encoder and VEX IMU odometry from the active Pico and Pi relay path; never moves motors |

## Status (2026-10-09)

| Part | Checked |
|---|---|
| Wire format, Brain publisher core, receiver | Host tests on Windows: protocol 9, receiver 158, publisher 30, 0 warnings |
| Publisher -> receiver -> browser | Host run: the real publisher core's output, as PROS COBS packets with console text, garbage, truncation, a pose reset and a program restart, replayed through the real receiver into headless Chrome. 584 of 600 lines shown; the other 16 were truncated on purpose. Ordering, restarts and counters correct |
| Browser | 27 assertion tests (axes, heading sign and wrap, roll/pitch signs, trail breaks, stale logic); screenshots of every state |
| PROS demo, publisher adapter | Build with the PROS 4.2.2 toolchain, 0 warnings. Not run on a Brain |
| Controller, radio, Brain USB, VEX IMU signs | **Not tested on hardware.** Controller and Brain USB sources pass tests against fakes only |
| Linux, macOS receiver | Cross-compiled only, never run |

## Build the receiver (Windows)

Needs CMake 3.16+ and MSYS2 ucrt64 GCC (`C:\msys64\ucrt64\bin` on PATH while
building). Tests fetch googletest on the first configure.

```sh
cmake -S spectaGATR -B build-spectagatr -G "MinGW Makefiles"
cmake --build build-spectagatr --parallel
ctest --test-dir build-spectagatr --output-on-failure
```

The program is `build-spectagatr/receiver/spectagatr.exe`. It is linked
statically, and each build copies `web/` and `fields/` next to it. For a
self-contained folder: `cmake --install build-spectagatr --prefix C:/spectagatr`.
Linux and macOS use the same commands without `-G` (untested).

## Try it without hardware

```sh
build-spectagatr/receiver/spectagatr --source sim                  # figure eight
build-spectagatr/receiver/spectagatr --sim attitude                # yaw, roll, pitch sweeps: check axis signs
build-spectagatr/receiver/spectagatr --sim restart                 # new session, pose reset: trail breaks
build-spectagatr/receiver/spectagatr --sim dropout                 # 3 s gaps: STALE banner
build-spectagatr/receiver/spectagatr --replay capture.bin          # raw bytes recorded from a port
```

The browser opens at <http://127.0.0.1:8790/>. Use `--no-browser` to only
print the address.

## Brain side

Add the library to a PROS 4 project Makefile, as the demo does:

```make
GATR2_ROOT ?= ../..                  # repository root, relative, no drive letter
GATR2_BRAIN_LIBS := spectagatr       # or add it to your list
include $(GATR2_ROOT)/brain/gatr2_brain.mk
-include ./common.mk
include $(GATR2_ROOT)/brain/gatr2_brain.mk
```

Publish from your own odometry:

```cpp
#include "main.h"
#include "spectagatr/pros_publisher.h"

spectagatr::ProsPublisher viz; // 10 lines/s; ProsPublisher viz(20) for 20 (1..50)

void initialize() { viz.start(); } // one background task writes stdout

void opcontrol() {
    while (true) {
        // Your odometry, here in inches and degrees (CCW from field +x).
        const double x_in = odom_x(), y_in = odom_y(), heading_deg = odom_heading();
        viz.updatePose(spectagatr::inchesToMeters(x_in), spectagatr::inchesToMeters(y_in),
                       spectagatr::degreesToRadians(heading_deg));
        // Full attitude instead, radians:
        // viz.updatePose(x_m, y_m, spectagatr::Attitude{roll, pitch, yaw});
        pros::delay(10);
    }
}
```

- `updatePose` never blocks or allocates and is safe from any task at any
  rate. Only the newest pose is sent, at most at the configured rate.
- A heading-only update sends roll = pitch = 0, even after a full-attitude
  update.
- Non-finite values are rejected and counted; the last good pose stays.
- Call `viz.resetTrail()` after re-placing the robot or resetting odometry.
  `viz.stats()` reports lines, rejected, overwritten and write errors.
- The publisher writes only stdout, one `write()` per line (about 60 bytes on
  the wire). It never reads stdin and never changes PROS serial settings.
  Library details: [brain/spectaGATR](../brain/spectaGATR/README.md).

The example program `brain/spectaGATR-demo` builds and uploads with:

```sh
cd brain/spectaGATR-demo
pros make
pros upload --slot 3 --name spectaGATR-demo --after screen
```

The demo never commands a motor. It has three pose sources, cycled with
LEFT/RIGHT, and each one labels where x/y come from:

| Source | x/y | Orientation |
|---|---|---|
| SYNTHETIC | A figure eight computed on the Brain | Alternates between heading only and full attitude |
| VEX IMU | Fixed point, because an IMU gives no position | Real attitude from the IMU on port 20 |
| CALLER ODOMETRY | Your `currentPose()` in `src/pose_source.cpp` | From your code |

Buttons: A resets the trail, B pauses (the viewer goes stale), X recalibrates
the IMU.

## Connect and view

1. Start the Brain program.
2. Connect the laptop in one of two ways:
   - by USB to the **V5 controller** that is linked to the Brain by radio or
     tether;
   - or by USB directly to the **Brain**.
3. Run `build-spectagatr/receiver/spectagatr`. The browser opens by itself.

| Option | Effect |
|---|---|
| (default) `--source auto` | A controller if one is plugged in, else the Brain's user port; rescans about every second |
| `--source controller` / `--source brain` | Force one kind |
| `--port COM7` | Force one port |
| `--list-ports` | Show the VEX ports found and their roles |
| `--field override` / `--field empty` | Field shown first; the Field menu in the page switches, and the choice is remembered |
| `--robot F,B,L,R[,H]` | Robot box in meters from the pose point (default 0.1685, 0.1685, 0.1465, 0.1465, 0.25) |
| `--stale-ms 750` | Age after which the pose is marked stale |

Ports are picked by USB id, never by COM number.
- Through a controller, the receiver only sends read-only queries (identity,
  radio status, stdout read).
- On a Brain it opens only the user port and never writes to it.

Viewer controls:
- Overhead (default), 3D orbit, Follow robot and Reset cameras.
- Clear trail.
- Optional 100 ms smoothing (display only; the numbers stay raw).

New fields go in `fields/<name>.json` ([schema](fields/README.md)) and appear
in the menu.

## Units and conventions

Same as the rest of the repository; full table in
[PROTOCOL.md](docs/PROTOCOL.md):
- **Field origin:** the inside bottom left corner of the field diagram. +X
  points right on the diagram, +Y up, +Z out of the floor.
- **Heading:** robot forward measured from +X, counterclockwise positive.
  Heading 0 faces right on the diagram, not up the screen.
- **Robot frame:** +x forward, +y left, +z up.
- **Roll:** about robot forward; positive lifts the left side.
- **Pitch:** about robot left; positive tips the nose down.
- **Rotation:** robot to field is Rz(yaw) Ry(pitch) Rx(roll).
- Attitude never moves the robot up or down.
- **API units:** meters and radians. Inches and degrees go through the helpers.
- **Wire units:** mm and centidegrees.

The viewer draws the origin, +X/+Y arrows, the heading direction and the
robot's forward/left/up markers, and repeats this list in its side panel.

## States and reconnection

| Viewer shows | Meaning |
|---|---|
| LIVE | Poses arriving within `--stale-ms` |
| STALE n s | No new pose: the robot is dimmed and frozen at its last received pose, never extrapolated |
| Searching for VEX device | Receiver running, no controller or Brain found, or the link dropped |
| Receiver disconnected, retrying | The browser lost the receiver; it reconnects by itself |

- **Unplug and replug:** the receiver reconnects without a restart. If the
  Brain program kept running, the same session continues.
- **Trail breaks:** a new Brain program run (new session), `resetTrail()`
  (new epoch), a gap longer than the stale time, and a browser reconnect each
  start a new trail. The trail never joins across a break.
- **Receive age:** how long ago the receiver got the newest pose, on the
  laptop's own clock. End-to-end latency from your measurement on the Brain to
  the screen is not measured, and the Brain's `t_ms` is never compared with
  laptop time.

## Connection limits

With `spectaGATR-demo`, the laptop reads the Brain through the controller or
the Brain's USB. With `localization-demo`, the Pi relay occupies Brain USB.
Simultaneous controller viewing is untested: the Pi may consume stdout bytes
before the controller receives them. The
[localization demo guide](../brain/localization-demo/README.md#9-visualization)
describes how to check that route and forward the output through the Pi with
`--echo-brain` over SSH if needed. Neither localization viewing route has
been tested on hardware.

- **One program per port.** While the receiver holds the controller port,
  `pros upload` and `pros terminal` through that controller cannot open it. Stop
  the receiver (Ctrl+C) first.
- **Uploading over Brain USB** uses the Brain's other (system) port. Running
  the receiver at the same time is untested.
- **Radio throughput is unknown.**
  - The default of 10 lines/s is about 0.6 kB/s.
  - The only official V5 radio figure found is for VEXlink: about
    0.5-1 kB/s.
  - Start at 10 Hz. If the receive age jumps or the stale banner flickers over
    the radio, lower the rate.
  - Direct USB is much faster, but that says nothing about the radio.
- **The Brain's stdout is shared.** Every `printf` in the program shares the
  2047-byte PROS output buffer. PROS blocks writers while it is full; the
  publisher's writes happen only in its own task.
- **Unverified on hardware:**
  - The VEX IMU roll/pitch signs in the demo; `kImuRollSign` and
    `kImuPitchSign` flip them.
  - Whether PROS output with no laptop attached ever fills that buffer.
  - The controller's reply padding.

## Hardware check (first time, about 10 minutes)

1. **Upload and connect.** Upload `spectaGATR-demo` and start it. The Brain
   screen shows `SYNTHETIC x/y`, a session id, and "Lines" growing by about 10
   per second. Plug the laptop into the Brain's USB, then run
   `spectagatr --source brain`.
   - **Expect:** the page turns LIVE.
   - **Expect:** the robot runs a figure eight with its forward marker along
     the path.
   - **Expect:** "roll/pitch" switches between "heading only" and values
     every 10 s.
   - **Expect:** the receive age stays under about 150 ms.
2. **Pause.** Press B on the controller.
   - **Expect:** within 0.75 s the banner reads STALE and counts up, and the
     robot is dimmed and frozen.
   - **Expect:** B again returns to LIVE, with a new trail piece.
3. **Unplug and replug.** Unplug the USB cable.
   - **Expect:** STALE, then "Searching for VEX device".
   - **Expect:** on replug, LIVE again with the same session id.
   - Also note whether the Brain's "Lines" kept growing while unplugged.
4. **Restart.** Restart the program on the Brain.
   - **Expect:** a new session id on the Brain and in the page, and a new
     trail.
5. **Controller.** Stop the receiver, plug the laptop into the controller
   (linked to the Brain), and run `spectagatr --source controller`.
   - **Expect:** the same as step 1, with status detail "Brain linked by
     radio".
   - Note the receive age and whether `counts` show drops.
   - Try `ProsPublisher viz(5)` and `viz(20)` to find what the radio sustains.
6. **IMU.** Switch to VEX IMU with LEFT/RIGHT and keep the robot still while it
   calibrates.
   - **Expect:** turning the robot counterclockwise (seen from above)
     increases the heading.
   - **Expect:** lifting the left side gives positive roll.
   - **Expect:** lifting the nose gives negative pitch.
   - If a sign is wrong, flip `kImuRollSign` or `kImuPitchSign` in
     `include/demo_config.h`.
