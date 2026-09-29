# Localization test

Motor-free PROS bench program for checking naviGATR localization. It sends
the robot profile from [brain/robot/gatr2_robot.h](../robot/gatr2_robot.h)
to the Pi, places the robot once at program start, and shows pose,
readiness, sensors and recovery history on the Brain screen. It also runs
the per-wheel travel calibration and IMU recalibration. Move the robot by
hand. Built with the PROS toolchain; not yet run on a robot.

Current bench setup (all in `gatr2_robot.h`):
- USB link to the Pi;
- two perpendicular tracking wheels, forward on encoder port 1 and sideways
  on port 0;
- the VEX IMU on Smart Port 20.

The Pi runs `config/override/brain_profile_usb.xml`. No BNO08X, camera or
AprilTags are needed.

- `include/robot_config.h`: startup placement policy and wait, wheel
  calibration reference distance and limits, pose age limit, loop periods.
- `src/main.cpp`: three pages (Status, Wheel calibration, Recovery). Every
  link call is in its first section.
- `Makefile`: PROS kernel 4.2.2 project that compiles the libraries in place
  through [brain/gatr2_brain.mk](../gatr2_brain.mk).

Kernel files (`firmware/`, `include/pros/`) are not in git. Restore them once,
offline, in this folder:

```
pros c apply kernel@4.2.2 --force-apply --no-download
git checkout -- .gitignore
pros make
pros upload --slot 2 --name locaGATR --after screen
```

The CLI replaces `.gitignore` with its template copy, hence the checkout.

Pages, controls, the calibration procedure and recovery:
[Brain setup, sections 3 to 6 and 8](../../docs/brain_setup.md#3-locaGATR-program).
Pi build and service: [Brain setup, section 2](../../docs/brain_setup.md#2-build-and-upload).
