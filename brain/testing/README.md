# brain/testing

Minimal PROS application for the V5 Brain using investiGATR and communiGATR.
It connects to the Pi, sends the starting placement, runs a navigation demo
(absolute goal, waypoint path, landmark relative alignment) and drives
manually. Built with `pros make`; not yet run on a robot.

- `include/robot_config.h`: every robot value (motor ports and directions,
  limits, gains, link port and baud, starting pose, demo). PLACEHOLDER values
  are not measured; set them before driving.
- `include/drive_control.h`, `src/drive_control.cpp`: drivetrain owner, the
  only motor writer.
- `src/main.cpp`: `initialize()`, `autonomous()`, `opcontrol()`, `disabled()`.
- `Makefile`: PROS kernel 4.2.2 project that compiles the libraries in place
  through [brain/gatr2_brain.mk](../gatr2_brain.mk).

Kernel files (`firmware/`, `include/pros/`) are not in git. Restore them once,
offline, in this folder:

```
pros c apply kernel@4.2.2 --force-apply --no-download
git checkout -- .gitignore
pros make
```

The CLI replaces `.gitignore` with its template copy, hence the checkout.

Setup, controls, screen, and the hardware bring-up checklist:
[docs/communigatr.md](../../docs/communigatr.md#testing-application-and-hardware-bring-up).
