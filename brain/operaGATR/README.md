# brain/operaGATR

PROS drive program for the V5 Brain: localization from the Pi through
communiGATR, planning from investiGATR, following and drive control from
actuGATR. It runs a direct, an avoiding and a landmark-relative test, drives
manually, and shows the command state for tuning. Built with the PROS
toolchain; manual driving has been tested, and autonomous following still
needs hardware verification.

The current test uses USB to the Pi, a VEX IMU on Smart Port 20, a forward
tracking wheel on Pico port 1 and a sideways wheel on port 0. Four-motor
mecanum is selected, with one independently driven motor per corner, confirmed
blue 600 RPM cartridges and 1:1 direct drive (motor, shaft, wheel). Tracking
wheels remain approximately 48 mm in diameter with 4,000 counts per turn.
The driven-wheel rectangle is 22.3 cm wide between tread centerlines and
24.2 cm long between front/rear shafts; its center is the robot origin.

The mecanum wheels are approximately 70 mm in diameter. Motor Smart Ports are
front-left 12, front-right 11, rear-left 2 and rear-right 1, named while facing
forward from behind the robot.
The reversal flags in `include/robot_config.h` set ports 2/12 normal and
ports 1/11 reversed so positive wheel commands should roll forward. Verify
forward/backward, strafe and turning with this configuration before path tests.
The tracking-wheel perpendicular offsets in `brain/robot/gatr2_robot.h` use these approximate
measurements: the forward wheel is at the right tread centerline, giving
y = -11.15 cm; the sideways wheel is assumed 11 cm behind the front axle,
giving x = +1.1 cm. Robot +x is forward and +y is left.
Both tracking encoders use `reversed = true`. At field heading 0 degrees,
a forward push should increase X and a leftward push should increase Y;
confirm those signs after uploading before running a path.
The outer body measures approximately 33.7 by 29.3 cm. Its footprint uses
16.85 cm forward/backward and 14.65 cm left/right from the drivetrain origin:
front/back overhangs are roughly equal, and left/right symmetry is assumed.
Refine individual extents if needed before close obstacle passes.
The geometry is approximate; motor/encoder directions and gains still need
checking and tuning on the robot.

Mount the mirrored wheels so the roller directions form an X when viewed from
above. Matching wheel types occupy opposite diagonals. The motor groups are
front-left, front-right, rear-left and rear-right; motor reversal makes each
wheel roll forward for a positive command, and cannot correct a misplaced
wheel type. The software does not have a per-wheel A/B handedness option.

Keep the robot still at startup. Once the profile, sensors and placement are
ready, its initial pose is (0, 0, 0 degrees). Left stick Y drives forward/back,
left stick X strafes, and right stick X turns (robot-relative controls).
Press A with the sticks released to plan from the
current pose to the fixed target (10 inches, 15 inches, 90 degrees), without
obstacle checks. Moving manually first changes the path's start, not its target.
The holonomic follower can translate and change heading at the same time.
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
