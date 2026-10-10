# brain/spectaGATR

Brain side of [spectaGATR](../../spectaGATR/README.md): a small library that
publishes the robot pose your program already computed to the laptop viewer.
It does no localization, owns no `initialize()`/`opcontrol()`, and needs no
investiGATR, communiGATR or Pi.

- `include/spectagatr/pose.h`: `Meters`, `Radians`, `Attitude`, unit helpers
- `include/spectagatr/publisher.h`, `src/`: portable core, host tested
- `include/spectagatr/pros_publisher.h`, `pros/`: PROS adapter, PROS builds only
- `tests/`: host tests
- Wire format: [spectaGATR/protocol](../../spectaGATR/protocol/include/spectagatr/protocol/pose_line.h)

Example PROS project: [brain/spectaGATR-demo](../spectaGATR-demo/README.md).

## Add it to a PROS project

In the project Makefile, as in the demo:

```make
GATR2_ROOT ?= ../..
GATR2_BRAIN_LIBS := spectagatr        # or add it to your existing list
include $(GATR2_ROOT)/brain/gatr2_brain.mk
-include ./common.mk
include $(GATR2_ROOT)/brain/gatr2_brain.mk
```

`GATR2_ROOT` is the repository root relative to the project, without a drive
letter.

## Use

```cpp
#include "main.h"
#include "spectagatr/pros_publisher.h"

spectagatr::ProsPublisher viz; // 10 Hz; ProsPublisher viz(20) for 20 Hz (1..50)

void initialize() {
    viz.start(); // creates the one background task
}

void opcontrol() {
    while (true) {
        const MyPose p = my_odometry.pose(); // your code: x/y inches, heading CCW degrees
        viz.updatePose(spectagatr::inchesToMeters(p.x), spectagatr::inchesToMeters(p.y),
                       spectagatr::degreesToRadians(p.heading));
        // or with full attitude, radians:
        // viz.updatePose(x, y, spectagatr::Attitude{roll, pitch, yaw});
        pros::delay(10);
    }
}
```

`viz.resetTrail()` after re-placing the robot or resetting odometry, before
the first pose in the new frame. `viz.stats()` counts lines, rejected and
overwritten poses and write errors; `viz.stop()` pauses output.
`viz.session()` is this run's id, picked at the first `start()` by hashing
`pros::micros()` and battery readings (splitmix64): best-effort unique, not
guaranteed.

## Units and conventions

SI in the API: meters, radians. Field origin at the bottom left corner of the
field diagram, +x right, +y up, heading counterclockwise from +x (0 faces +x).
Roll about robot forward (+ lifts the left side), pitch about robot left (+
tips the nose down), robot to field `Rz(yaw) * Ry(pitch) * Rx(roll)`. Full
table: [PROTOCOL.md](../../spectaGATR/docs/PROTOCOL.md). Units are never
guessed from magnitudes; convert inches and degrees with `inchesToMeters` and
`degreesToRadians`.

The heading overload publishes `Attitude{0, 0, heading}` with the full
attitude flag off, also after earlier full-attitude updates. Non-finite values
and |x| or |y| over 2000 m are rejected and counted; the previous pose stays.

## Threading and timing

- `updatePose` and `resetTrail`: any task, any rate, never block, never
  allocate. The newest pose overwrites the previous one in a lock-free slot
  (seqlock over atomic words plus a checksum). Overlapping calls from two tasks
  are safe; a torn slot is never sent and the next update repairs it.
- The background task (priority `TASK_PRIORITY_DEFAULT - 1`) checks every
  5 ms and writes a line only when a new pose arrived and the rate period has
  passed. `t_ms` is `pros::millis()` when you called `updatePose`.
- No new poses, no lines: the viewer marks the pose stale.
- Fixed memory after construction; one task for the program's life.

## stdout, COBS, other console output

Each line goes out with one `write(STDOUT_FILENO, ...)`. In PROS 4.2.2 that
call bypasses stdio buffering and becomes one COBS packet on the `sout` stream
(line + 6 bytes), so `pros terminal` and the receiver both see whole lines.
Lines are 54 bytes typical, 88 at most. The library never reads stdin, never
calls `serctl`/`fdctl` and never changes COBS. Your own `printf` output can
share stdout; the receiver skips text that is not a pose line.

PROS blocks a stdout write while its 2047 byte output buffer is full, and then
every `printf` in the program waits too. The library writes only from its own
task, so a control loop that only calls `updatePose` never waits for the
laptop. Whether VEXos keeps
draining the output with no host attached is not verified.

## Not with the retired communiGATR USB link

Do not run this publisher and communiGATR, the retired Pi fallback link (commit 10c568e), in one
program. That link turns COBS off for all output, owns stdin and uses the same
single user channel and output buffer: pose lines would go out raw next to its
frames and delay them, to whichever host reads the channel. Run the Pi fallback
and spectaGATR as separate programs. A laptop on the controller while the Pi
uses the Brain USB port is untested.

## Tests

```sh
cmake -S brain -B build-brain
cmake --build build-brain --parallel
ctest --test-dir build-brain -R "^Specta"
```

Host tested: both overloads decoded through `protocol::decodeLine`, attitude
to heading switch, units, heading wrap, rejection, rate cap, seq and epoch,
`t_ms` wrap, no allocation, a writer/reader thread stress test. The PROS
adapter is compile-checked with the PROS 4.2.2 toolchain only; nothing here
has run on a Brain.
