# brain/operaGATR

PROS drive program for the V5 Brain: localization from the Pi through
communiGATR, planning from investiGATR, following and drive control from
actuGATR. It runs a direct, an avoiding and a landmark-relative test, drives
manually, and shows the command state for tuning. Built with the PROS
toolchain; not yet run on a robot.

- `include/robot_config.h`: drivetrain (tank or mecanum example), motor
  ports and directions, limits, follower gains, manual speeds, test
  destinations, startup placement. PLACEHOLDER values are not measured.
- `include/field_references.h`: generated field object names
  (`Field::RedGoal2West`, ...) for the map id the Pi serves.
- `src/main.cpp`: link and drive setup, the screen, `autonomous()`,
  `opcontrol()`, `disabled()`. The actuGATR drive task is the only motor
  writer.
- The robot description (tracking wheels, IMU, footprint, link, start pose)
  is shared with locaGATR in
  [brain/robot/gatr2_robot.h](../robot/gatr2_robot.h).
- `Makefile`: PROS kernel 4.2.2 project that compiles the libraries in place
  through [brain/gatr2_brain.mk](../gatr2_brain.mk).

Kernel files (`firmware/`, `include/pros/`) are not in git. Restore them once,
offline, in this folder:

```
pros c apply kernel@4.2.2 --force-apply --no-download
git checkout -- .gitignore
pros make
pros upload --slot 1 --name operaGATR --after screen
```

The CLI replaces `.gitignore` with its template copy, hence the checkout.

Controls, tests, tuning and the planner limits:
[Brain setup, section 7](../../docs/brain_setup.md#7-drive-test-program).
