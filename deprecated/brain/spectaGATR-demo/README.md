# brain/spectaGATR-demo

Example PROS 4.2.2 program for the [spectaGATR publisher](../spectaGATR/README.md).
It publishes a pose to the laptop viewer and shows what it sends on the Brain
screen. **Nothing moves: it never commands a motor**, in any competition mode.
It uses only the spectaGATR library (`GATR2_BRAIN_LIBS := spectagatr`).

Status: builds with the PROS 4.2.2 toolchain with no warnings. Not yet run on
a Brain.

## Build and upload

Kernel files (`firmware/`, `include/pros/`) are not in git. Restore them in
this folder from the local PROS cache:

```sh
pros c apply kernel@4.2.2 --force-apply --no-download
git checkout -- .gitignore
```

Then, Brain connected over USB:

```sh
pros make
pros upload --slot 3 --name spectaGATR-demo --after screen
```

Start the program on the Brain, then start the laptop receiver and open the
viewer as described in [spectaGATR/README.md](../../spectaGATR/README.md)
(Brain USB or controller).

## Pose sources

LEFT/RIGHT on the controller cycles the source; the default is
`kDefaultSource` in `include/demo_config.h`. Changing the source also resets
the trail, since each source is its own frame.

| Source | x/y from | Attitude | Screen label |
|---|---|---|---|
| 1 SYNTHETIC | A figure eight computed on the Brain, centered at (70 in, 70 in), 80 by 48 in, 16 s per loop; stays inside the Override field | Heading along the path. Alternates every 10 s: heading only, then full attitude with a 5 deg roll and 3 deg pitch wobble | `SYNTHETIC x/y` |
| 2 VEX IMU | Fixed at (70 in, 70 in): an IMU gives no position | VEX IMU on Smart Port 20, full attitude | `x/y FIXED: no position source` |
| 3 CALLER ODOMETRY | `currentPose()` in `src/pose_source.cpp` | Heading only | `currentPose() PLACEHOLDER` until replaced |

SYNTHETIC is deterministic: synthetic time only advances while updates run,
so a pause freezes the path and resume continues where it stopped.

VEX IMU: heading = `kImuStartHeadingDegrees` minus the PROS rotation (PROS is
clockwise positive, the viewer counterclockwise). Roll and pitch use the
conversion copied from the retired communiGATR (`src/attitude.cpp` at commit 10c568e), with
the mount yaw. If the IMU is
unplugged or calibrating, nothing is published and the screen says so. It
calibrates at start-up when present; X recalibrates (robot still).

CALLER ODOMETRY: replace `currentPose()` (and make
`currentPoseIsPlaceholder()` return false) with your own odometry, converting
inches and degrees with the helpers. The placeholder returns a fixed pose
(24 in, 36 in, 90 deg).

All settings are in `include/demo_config.h`, in inches and degrees.

## Controls and screen

| Button | Action |
|---|---|
| A | `resetTrail()`: the viewer starts a new trail |
| B | Pause or resume pose updates: lines stop and the viewer marks the pose stale |
| LEFT / RIGHT | Previous / next pose source |
| X | Recalibrate the VEX IMU (IMU source only, keep the robot still) |

The demo reads the buttons in its own task, in every competition mode.

The screen shows the source and where x/y come from, pause or IMU state, the
session id, the rate cap and measured lines per second, lines sent, rejected,
overwritten (poses replaced before sending, normal at 100 Hz updates and
10 Hz lines) and write errors, and the last pose supplied in meters and
degrees. If "Lines" stops growing while not paused, stdout is not draining.

## Bench checks for tomorrow

1. SYNTHETIC: the viewer draws the figure eight, heading along the path; the
   attitude display switches between heading only and full every 10 s.
2. B: the viewer goes stale within its stale threshold; B again resumes. A:
   the trail restarts. Unplug and replug the laptop cable: the viewer goes
   stale, the receiver reconnects and the same session continues. Note
   whether "Lines" kept growing while unplugged; PROS output with no host
   attached is unverified.
3. Restart the program: a new session id on the screen and in the viewer, and
   the trail breaks.
4. VEX IMU, robot level: roll and pitch near 0. Lift the front 10 to 15 deg:
   pitch negative (nose up). Lift the left side: roll positive. Turn the
   robot counterclockwise seen from above: heading increases. If lifting the
   front moves roll, set `kImuMountYawDegrees` to 90 or 270; if one sign is
   wrong at every mount, flip `kImuRollSign` or `kImuPitchSign`. The VEX
   roll and pitch convention is unverified on hardware.
