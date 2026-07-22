# bringup

Standalone hardware bring-up tests. Each one proves a single piece of hardware
works on its own, before anything is built on top of it.

These are meant to be clone-and-run. They do not depend on the rest of the
repo, they are not part of CI, and they are not the real firmware. Delete or
ignore once the thing they test is trusted.

- rs485_link/ - confirm the Pi to brain RS-485 link carries bytes.
