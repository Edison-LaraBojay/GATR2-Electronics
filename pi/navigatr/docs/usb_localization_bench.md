# USB localization bench test

The current bench: Pi USB-A to the V5 Brain's micro-USB port with a data
cable, the Brain on its V5 battery. This bypasses the HAT's RS-485 circuit.
The Pico supplies one forward tracking wheel (encoder port 0, J2) and one
sideways wheel (port 1, J3) over its UART; the VEX IMU stays on Brain Smart
Port 1. The Brain program is `brain/locaGATR`, which never drives
motors: push the robot by hand. No camera, AprilTags or external IMU are
needed.

The robot is described on the Brain and sent as a robot profile; the Pi runs
[brain_profile_usb.xml](../config/override/brain_profile_usb.xml). See
[Brain robot profiles](brain_profile.md) for what the Pi does with it.

## Brain

In [brain/robot/gatr2_robot.h](../../../brain/robot/gatr2_robot.h):

- `kUseUsb = true`;
- `kSetup = Setup::kTwoWheelVexImu` and `kVexImuPort = 1`;
- `forwardWheel()` and `sidewaysWheel()`: port, radius (0.024 m,
  PROVISIONAL), counts per revolution, polarity, gearing, mounting position
  (UNMEASURED) and measuring direction;
- the start pose `kStartX`, `kStartY`, `kStartHeadingDegrees`.

Build and upload `locaGATR` as in
[Brain setup](../../../docs/brain_setup.md#2-build-and-upload). USB text
logging stays off on this link: do not run a PROS terminal or another serial
reader on the Brain's user port while the Pi uses it.

## Pi

On the Pi, from `pi/naviGATR`:

```sh
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_TESTS=OFF -DNAVIGATR_WITH_LIBCAMERA=OFF
cmake --build build-bench -j1
sudo bash tools/install_service.sh --user "$USER" --config config/override/brain_profile_usb.xml
sudo systemctl status navigatr --no-pager -l
```

The service starts this config at boot. Its `brain_usb` resource looks for the
sole VEX V5 USB **user interface** (vendor `2888`, product `0501`, interface
`02`) and retries once per second while the Brain is absent, so the viewer
starts anyway. If auto selection fails or several Brains are connected,
inspect the devices:

```sh
ls -l /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null
```

Then replace `path="auto"` with the user interface's stable path:

```xml
<Resource id="brain_usb" type="pros_usb_link">
    <Device path="/dev/serial/by-id/YOUR_VEX_USER_INTERFACE"/>
</Resource>
```

Do not use the upload/system interface or assume `ttyACM0`/`ttyACM1`
numbering is stable. After editing the XML, run `sudo systemctl restart navigatr`.

## Run

1. Start the Pi service and the Brain program with the robot still.
2. The Pi waits with no pose until the Brain connects. The Brain sends the
   profile; the Pi log shows `event: profile <id> applied (two wheels + IMU;
   port 0 ... port 1 ...; IMU Brain VEX smart port 1; footprint ...)`, or the
   refusal reason.
3. Hold still while the VEX IMU calibrates on the Brain. The Pi does no IMU
   calibration with this profile.
4. The Brain places the robot at its start pose once; its readiness line reads
   Ready and the pose LIVE. Then push and rotate the robot.

The Pi model is `brain_imu_planar_bench`: the VEX IMU gives the rotation, and
the two wheels resolve forward and sideways travel after removing the travel a
turn about the robot origin causes. Neither distance is averaged with the
other. With heading placed at zero and the robot not turning, roll only port
0: field x changes. Roll only port 1: field y changes. Reverse each to check
the signs; a wrong sign is the wheel's `reversed` (encoder polarity) flag in
`gatr2_robot.h`. At other headings these body directions rotate into the field
axes. Measure the wheel offsets before judging position during turns.

This bench model pairs wheel and IMU samples by Pi arrival time. It does not
synchronize the Brain and Pico clocks; test slowly.

## Viewer

From the viewing computer (a Windows CMD window works), leave this running:

```bat
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
```

Open <http://localhost:8765/>. The field, its collision boxes and the robot
with the profile's footprint are drawn with no camera; the robot stays level
because this IMU path supplies no tilt. The Brain link panel shows the
readiness line (link, profile, sensors, map, calibration, placement), the
profile, the raw wheel readings with their active corrections, and the
recovery events. See [inspection](inspection.md).

For link diagnosis, inspect `diagnostics.estimation.links` in `/api/snapshot`
for `brain_usb`, and the Brain's reply and timeout counters. The Pi's byte
counters count decoded protocol bytes, not unrelated console output.

## Cable pulls and restarts

The VEX IMU samples ride on the Brain link, so a USB outage longer than
`sensor_loss_ms` (250 ms) stops them: the Pi ends the placement ("sensor lost:
brain_vex_imu stale ...: place again", or "motion lost: profile_motion: no VEX
IMU and wheel pair for ... ms" when the outage ends between two checks) and
the Brain shows Needs placement. A shorter outage loses nothing: the next step
measures the wheel travel and rotation across it. The link itself recovers on
its own: the Pi reopens the device once per second and the Brain resumes its
session. Put the robot at the start pose and place it again. A Brain program
restart does the same; startup placement handles it at program start. See
[Brain robot profiles](brain_profile.md#sensor-loss-and-recovery).

## USB envelope

Each protocol frame travels as one ASCII line: `NG1:` plus the frame in
uppercase hexadecimal plus a newline. That keeps PROS console control
sequences out of the stream; lines without the marker are ignored. The Brain
side is `ProsUsbPort` in communiGATR, with its own I/O tasks so the display and
polling stay responsive. See [pros_usb_link](../../../docs/navigatr_resources.md#pros_usb_link).

## RS-485 instead

Set `kUseUsb = false` and `kLinkPort` in `gatr2_robot.h`, rebuild and upload,
and install the service with `config/override/brain_profile_rs485.xml`. Check
that config's DE GPIO number for the Pi's kernel (see
[setup](setup.md#serial-resource)). Nothing else changes: the transport never
changes localization geometry.

The older XML-configured bench profile
[bench_vex_imu_usb.xml](../config/override/diagnostics/bench_vex_imu_usb.xml)
describes the same bench in Pi XML. It refuses Brain profiles, and the
current Brain programs, which always send one, then refuse to place; use it
only with a Brain client built without a profile.
