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
| [`../brain/localization-demo/`](../brain/localization-demo/README.md) | Active PROS program: manual eight-motor mecanum drive and encoder/VEX IMU odometry from the Pico and Pi relay path |

## Status (2026-10-10)

| Part | Checked |
|---|---|
| Wire format, Brain publisher core, receiver | Host tests on Windows: protocol 9, receiver 178, publisher 30, 0 warnings |
| Controller radio | Measured on the bench 2026-10-10 (table under Connection limits). With `--radio-download`: 10 Hz, one pose per reply, 110-180 ms gaps, no loss. The Brain's output gating is built but not yet uploaded to the Brain |
| Publisher -> receiver -> browser | Host run: the real publisher core's output, as PROS COBS packets with console text, garbage, truncation, a pose reset and a program restart, replayed through the real receiver into headless Chrome. 584 of 600 lines shown; the other 16 were truncated on purpose. Ordering, restarts and counters correct |
| Browser | 27 assertion tests (axes, heading sign and wrap, roll/pitch signs, trail breaks, stale logic); screenshots of every state |
| Publisher adapter | Builds with the PROS 4.2.2 toolchain, 0 warnings. Not run on a Brain |
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
  the wire), and only while the Brain's output queue is short. It never reads
  stdin and never changes PROS serial settings.
  Library details: [brain/spectaGATR](../brain/spectaGATR/README.md).

The active example is [brain/localization-demo](../brain/localization-demo/README.md).
It publishes pose from two tracking wheels and a VEX IMU while the controller
commands eight mecanum-drive motors. Left stick Y drives forward/backward,
left stick X strafes, and right stick X turns. Configure the eight signed
motor ports in `include/drive_config.h` before driving; they default to zero
and keep drive disabled until filled in correctly. Follow the demo guide for
the PROS kernel, build, upload, startup and motor checks.

Drive waits for IMU calibration, a first fresh encoder baseline and centered
sticks. A later localization-link loss holds the displayed pose while
robot-relative manual drive remains usable. Controller disconnect/read errors,
disabled mode and autonomous mode stop the motors; center the sticks before
resuming control. A motor-command fault stops drive until the program is
restarted. There is no autonomous driving or path following.

## Connect and view

For the active localization demo, leave the Pi attached to Brain USB and
follow its [visualization steps](../brain/localization-demo/README.md#9-visualization).
The following receiver options also apply to other programs that use the
publisher library.

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
| `--debug-link` | Once a second: controller transport counters and pose delivery (rate, gaps, lost seq) |
| `--radio-download` | Practice only: move a radio-linked controller to the faster download channel, back to the pit channel on exit |
| `--user-read-length N` | Controller read length byte (default 0: replies of up to 224 B; 64 caps replies at 64 B) |
| `--pipeline N` | Controller reads kept outstanding, 1 to 4 (default 1); experimental |

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

The standalone publisher can be read through the controller or Brain USB.
With `localization-demo`, the Pi relay occupies Brain USB. Simultaneous
controller viewing is untested: the Pi may consume stdout bytes
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
- **The radio channel decides the update rate.** 10 lines/s needs about
  0.6 kB/s. On the default pit channel each controller read costs about
  235 ms of dead time plus about 0.7 ms per byte. Bench measurements,
  2026-10-10, publisher at 10 Hz:

  | Setup | Delivered |
  |---|---|
  | Pit channel, read length 64 (old default) | About 200-230 B/s, about 3 Hz, backlog and loss |
  | Pit channel, read length 0 (current default) | About 520 B/s, 6-7 Hz in bursts of about 4 poses every 425 ms, 4-7 lost seq/s |
  | Pit channel, length 0, `--pipeline 2` | About 570 B/s, 10 Hz with no loss, still bursts every about 400 ms |
  | Download channel (`--radio-download`) | 10 Hz, one pose per reply, 28-47 ms round trips, 110-180 ms gaps, no loss |

  - For smooth viewing in practice, use `--radio-download`. It stays opt-in:
    the link drops for a few seconds during the switch, and its effect on
    driving latency and its legality under field control are unverified.
  - On the pit channel, lowering the publisher rate (`ProsPublisher viz(5)`)
    avoids the loss.
  - The publisher writes a line only while at most about 128 B wait in the
    Brain's VEXos output FIFO, whose empty size it learns from the largest
    free count seen. A slow link then gets the newest pose instead of a
    growing queue. This gating is built but not yet run on a Brain; the
    measurements above predate it.
  - Direct USB is much faster, but that says nothing about the radio.
- **The Brain's stdout is shared.** Every `printf` in the program shares that
  FIFO. Heavy `printf` output delays pose lines, because the publisher waits
  for the queue to drain instead of adding to it. If the Brain ever reports an
  invalid FIFO count, the publisher falls back to plain writes, which PROS
  blocks while its 2047-byte buffer is full. That only ever blocks the
  publisher's own task.
- **Unverified on hardware:**
  - The localization demo's encoder and VEX IMU signs.
  - The output gating's FIFO counts with radio, USB and no host attached.
  - The controller's reply padding.

## Hardware check for the active demo

The complete route has not been tested on hardware. Start with the
[demo guide](../brain/localization-demo/README.md) for wiring, port and motor
direction configuration, calibration and the first motor checks.

1. Place the robot at its configured start pose and keep the controller
   sticks centered. Start the Pico, Pi relay and Brain program. Wait for IMU
   calibration and the first fresh encoder baseline; the Brain's
   `spectaGATR: lines` counter should then grow at about 10 per second.
2. With the Pi still on Brain USB, try `spectagatr --source controller` from
   a laptop connected to the linked controller. Expect LIVE and a heading-only
   pose at the configured start position. If the Brain's line counter grows
   but the viewer is empty or intermittent, use the guide's `--echo-brain`
   SSH route. That route leaves the controller available for manual drive.
3. With sticks centered, perform the guide's hand-push sign tests: forward
   at heading 0 increases x, left increases y, and counterclockwise turning
   increases heading. Verify the same values on the Brain and viewer before
   relying on the position during manual driving.
4. While stationary, stop the relay briefly. Expect the Brain to hold its
   pose and the viewer to become STALE. Restart the relay: odometry rebases
   and the viewer returns to LIVE with a trail break. Travel during the gap
   is lost. Manual drive remains available after successful startup, so keep
   the sticks centered during this check.
5. Restart the Brain program at the configured start pose. Expect startup
   calibration again, a new publisher session, and a new trail. Check
   controller disconnect and competition-mode motor stops separately as
   part of the demo's motor checks.
