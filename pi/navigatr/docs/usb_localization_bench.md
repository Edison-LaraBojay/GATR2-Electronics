# USB localization bench test

Connect Pi USB-A to V5 Brain micro-USB with a data cable. Keep the Brain powered
by its V5 battery. This bypasses the HAT's RS-485 circuit. The Pico still supplies
one forward tracking wheel (port 0) and one sideways wheel (port 1) over the
existing UART; the VEX IMU stays on Brain
Smart Port 1. Placement, pose replies, localization, and the 3D viewer use the
existing naviGATR flow. This test has no motor control.

The Brain test configuration currently selects USB. The service installer still
defaults to the RS-485 `bench_vex_imu.xml` profile, so pass the USB configuration
explicitly as shown below.

## Brain

In `brain/localization-test/include/robot_config.h`, use `kUseUsbBench = true`
and `kUseVexImuBench = true`. The Smart Port link setting is ignored in USB mode.
The starting field pose is still configured by `kStartX`, `kStartY`, and
`kStartHeadingDegrees`.

With the Brain USB cable connected to the computer, run in a PROS terminal:

```sh
cd brain/localization-test
pros make
pros upload --slot 2 --name localization-test --after screen
```

Move the cable to the Pi and run the Brain app. Its first screen line must say
**USB to Pi**. Hold still during IMU calibration. USB text logging is suppressed
in this mode; do not run a PROS terminal or another serial reader on this link.

## Pi

After updating the checkout with the USB files, run on the Pi:

```sh
cd ~/GATR2-Electronics/pi/navigatr
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_TESTS=OFF -DNAVIGATR_WITH_LIBCAMERA=OFF
cmake --build build-bench -j1
sudo bash tools/install_service.sh --user "$USER" --config config/override/diagnostics/bench_vex_imu_usb.xml
sudo systemctl status navigatr --no-pager -l
```

The service now starts the USB profile at boot. Its `brain_usb` resource looks
for the sole VEX V5 USB **user interface** (vendor `2888`, product `0501`, interface
`02`). It waits and retries if the Brain is absent, so the viewer can still start.
If auto selection fails or multiple Brains are connected, inspect the devices:

```sh
ls -l /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null
```

Then replace `path="auto"` in the new XML with the user interface's stable path:

```xml
<Resource id="brain_usb" type="pros_usb_link">
    <Device path="/dev/serial/by-id/YOUR_VEX_USER_INTERFACE"/>
</Resource>
```

Do not use the upload/system interface or assume `ttyACM0`/`ttyACM1` numbering is
stable. This resource has no `DriverEnable`. After editing the XML, run
`sudo systemctl restart navigatr`.

The profile retains `/dev/ttyAMA0` for the Pico and uses `brain_imu_planar_bench`
to resolve both translation axes using the configured wheel directions and IMU
rotation. Port 0 measures forward (+robot x, angle 0 degrees); port 1 measures
leftward (+robot y, angle 90 degrees). Both contribute full measured travel,
without averaging the forward and sideways distances together.

Wheel radius, offsets, directions, and counts per revolution are editable bench
defaults. The current radius is 0.024 m, with the forward wheel 0.15 m left of
the origin and the sideways wheel 0.15 m ahead. Measure actual wheel offsets
before judging position during turns: the model removes wheel travel caused by
rotation around the robot origin. `calibration_status` is only a reader annotation.
Set a wheel's `direction="negative"` if counts decrease during its positive rolling
direction. The VEX IMU bench path supplies heading, not live pitch/roll.

With heading placed at zero and the IMU held still, spin only port 0: field x
should change. Spin only port 1: field y should change. Reverse each wheel to
check both signs. At other headings, these body directions rotate into the field
axes. Test slowly: this bench adapter pairs sensor readings by Pi arrival time.

## Placement and viewer

Wait for **Link: connected** and **Placement: applied** on the Brain. If the Pi
was not ready during the first ten seconds, press **A** or tap the bottom screen
button after it connects. Once **Pose: LIVE** appears, push and rotate the robot.
USB connection alone does not establish placement or fresh wheel data.

In a separate **Windows CMD** window, leave this tunnel running:

```bat
ssh -N -L 8765:127.0.0.1:8765 gatr2@gatr2.local
```

Open <http://localhost:8765/>. The same static field and live robot pose are
displayed. No camera or AprilTag estimation is required; the model stays level
because this bench IMU path supplies no tilt. Drawn field objects do not block
localization.

For link diagnosis, inspect `diagnostics.estimation.links` in `/api/snapshot`
for `brain_usb`, and the Brain's reply/timeout counters. Pi link byte counters
refer to decoded inner protocol bytes, excluding unrelated console output.

## Scope and reverting

The USB adapter wraps existing protocol frames in bounded uppercase hexadecimal
lines, avoiding PROS console input control sequences. It lives in the test app;
the shared communiGATR Smart Port driver and existing Pi UART resource are
unchanged. Separate USB I/O workers keep the Brain display and protocol polling
responsive. The Pi resource reconnects when its device reappears. Disconnects
make poses stale; reconnecting does not automatically reapply a previous placement.

To return to RS-485, set `kUseUsbBench = false`, check `kNavigatrPort`, rebuild and
upload the Brain, and install the service with `bench_vex_imu.xml` again. Preserve
that profile's Pi-specific GPIO number and calibration values. That supplied
RS-485 profile describes two parallel wheels; for this perpendicular mounting,
also select `brain_imu_planar_bench` and copy the USB profile's wheel geometry
and wheel references. Transport selection does not change physical wheel geometry.
