# brain/operaGATR

PROS drive program for the V5 Brain: localization from the Pi through
communiGATR, planning from investiGATR, following and drive control from
actuGATR. It runs a direct, an avoiding and a landmark-relative test, drives
manually, and shows the command state for tuning. Built with the PROS
toolchain; not yet run on a robot.

The current test uses USB to the Pi, a VEX IMU on Smart Port 1, a forward
tracking wheel on Pico port 0 and a sideways wheel on port 1. The tank drive
uses blue 600 RPM motors, 1:1 external gearing, 2.75-inch driven wheels and
10.65-inch track width. Tracking wheels are approximately 48 mm in diameter;
their perpendicular offsets are 0 m (forward wheel) and +0.1125 m (sideways
wheel, ahead of center).

Keep the robot still at startup. Once the profile, sensors and placement are
ready, its initial pose is (0, 0, 0 degrees). Left stick Y drives forward/back
and right stick X turns. Press A with the sticks released to plan from the
current pose to the fixed target (55 inches, 70 inches, 90 degrees), without
obstacle checks. Moving manually first changes the path's start, not its target.
B cancels; moving the sticks also takes over. UP reassigns the current pose to
the initial pose, so only use it after returning to the physical starting spot.
Encoder directions and follower gains still need checking on the robot.
To request a fresh VEX IMU calibration, press DOWN while stopped, keep the
robot still until it finishes, then press UP at the physical starting spot.

The Pi still serves the pose and path visualization; no camera is required.
Use `pi/naviGATR/config/override/brain_profile_usb.xml`. The Brain sends these
robot measurements to the Pi, so changing them only requires rebuilding and
uploading this Brain program. The (0, 0) floor-test placement overlaps the
configured field boundary: X/Y avoidance examples require a valid field
placement, even though the direct A test works at the origin.

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
