# brain/spectaGATR

Brain side of [spectaGATR](../../spectaGATR/README.md): a small library that
publishes the robot pose your program already computed to the laptop viewer.
The caller owns localization and `initialize()`/`opcontrol()`.

- `include/spectagatr/pose.h`: `Meters`, `Radians`, `Attitude`, unit helpers
- `include/spectagatr/publisher.h`, `src/`: portable core, host tested
- `include/spectagatr/pros_publisher.h`, `pros/`: PROS adapter, PROS builds only
- `tests/`: host tests
- Wire format: [spectaGATR/protocol](../../spectaGATR/protocol/include/spectagatr/protocol/pose_line.h)

The active [drive and localization demo](../localization-demo/README.md) is
the example PROS project. It uses this publisher for its encoder and VEX IMU
pose while its own controller loop commands the mecanum drive. The publisher
library itself only publishes poses.

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
  5 ms and writes a line only when a new pose arrived, the rate period has
  passed, and the output has room: at most 128 bytes (about two lines) waiting
  in the VEXos stdio FIFO (`vexSerialWriteFree(1)`). `t_ms` is
  `pros::millis()` when you called `updatePose`.
- A reader slower than the rate (the controller radio carries a few hundred
  bytes per second) gets fewer lines, each carrying the newest pose, instead of
  a growing queue of old ones. With no reader at all the task stops writing.
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
every `printf` in the program waits too. The publisher avoids that by waiting
for room in the VEXos FIFO, which the PROS buffer drains into every 2 ms; it
does not see the PROS buffer itself. Your own `printf` output shares the FIFO:
heavy printing now delays pose lines instead of blocking the publisher task.

## Encoder localization demo

The active demo receives encoder records from the Pi relay on stdin and
publishes pose lines on stdout, with COBS enabled. A laptop on the controller
while the Pi uses Brain USB is untested; the Pi may consume the output first.
See the [demo's visualization steps](../localization-demo/README.md#9-visualization)
for the controller check and the `--echo-brain` route through the Pi over SSH.
Neither route has been tested on hardware.

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
