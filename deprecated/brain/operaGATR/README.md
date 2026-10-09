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
Temporary distance multipliers in `brain/robot/gatr2_robot.h` are **2.10144**
for the forward wheel (port 1) and **1.96700** for the sideways wheel (port 0),
from separate 23-inch pushes that reported 0.278 m and 0.297 m respectively.
These multiply the radius sent in the robot profile because the existing Pi
limits `travel_scale` to 0.9–1.1; the transmitted radii are effective calibration
values, not physical wheel sizes. Rebuild/upload the Brain and restart its
program to send the profile; no Pi update is needed. Repeat the distance test
before path testing. Reset these multipliers to 1.0 before correcting the
underlying CPR, gearing or wheel diameter. Port 2 retains its nominal radius.
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
ready, its initial pose is (7 inches, 24 inches, 0 degrees). Left stick Y drives forward/back,
left stick X strafes, and right stick X turns (robot-relative controls).
Press A with the sticks released to plan from the
current pose to the fixed target (72 inches, 24 inches, 90 degrees), without
obstacle checks. X uses that same target with obstacle avoidance.
Moving manually first changes the path's start, not its target.
On the configured field, the direct route crosses `red_goal_3_south` near
(46.66 inches, 23.11 inches), so use X with the field assembled. The initial
rear clearance to the red wall is only about 9 mm with the approximate
footprint. Avoidance supports a straight exit from this near-wall placement.
The holonomic follower can translate and change heading at the same time.
B cancels; moving the sticks also takes over. UP reassigns the current pose to
the initial pose, so only use it after returning to the physical starting spot.
Encoder directions and follower gains still need checking on the robot.
To request a fresh VEX IMU calibration, press DOWN while stopped, keep the
robot still until it finishes, then press UP at the physical starting spot.

The Pi still serves the pose and path visualization; no camera is required.
Use `pi/naviGATR/config/override/brain_profile_usb.xml`. The Brain sends these
robot measurements to the Pi, so changing them only requires rebuilding and
uploading this Brain program. A ignores obstacles and field bounds; X/Y
check them using the configured footprint and planning clearance.

Edit the initial pose in `brain/robot/gatr2_robot.h`: `kStartX`, `kStartY`
(inches multiplied by `kInch`) and `kStartHeadingDegrees` (degrees).
Edit `kDirectGoal` in `include/robot_config.h` for the A/X destination:
`{x_inches * kInch, y_inches * kInch, heading_degrees * kDeg}`.
`kAvoidGoal` takes the same value automatically. Keep the robot's physical
center at the configured start and facing the configured heading at startup,
or when using UP to reassign its pose.

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

While the link is disconnected, the screen replaces the button hints with
connection diagnostics from the existing status snapshot. `I/O on` means
the Brain transport is initialized, not that the Pi has answered. USB `tx`
counts complete line writes, `rx` decoded incoming frames, and `short`
incomplete writes. `Reply` counts correlated protocol replies, `TO` timeouts,
`bad` invalid reply frames, and `stray` valid replies without a matching
request. The last diagnostic row shows USB read errors/dropped lines and
whether the polling task started, or an explicit protocol mismatch.
Photograph these rows if the program remains at connecting; the counters
distinguish a transmit stall, missing responses, and rejected responses.
