# Camera preview (no field correction)

How to see the live camera in the viewer while the robot runs the current
Brain-profile USB setup: two tracking wheels on Pico encoder ports 0 and 1,
the VEX IMU on Brain Smart Port 1, the Brain link over USB. The camera only
adds pictures. Localization, the Brain link, the Pico link and the field are
exactly those of `brain_profile_usb.xml`.

Status: host tests only. The configuration builds and runs in tests with a
fake camera (a dead one and a streaming one) and memory links. It has not
run with the physical camera or on the Pi.

## What preview is, and is not

| | Preview (`brain_profile_usb_camera.xml`) | Calibrated field estimation (later) |
|---|---|---|
| Camera intrinsics | not needed (frames carry none) | required, at the capture mode |
| Camera mounting on the robot | not needed | required, measured |
| AprilTag detection | off | on |
| Tag association, landmark correction | off: world estimation is `noop` | on |
| Pose source | wheels and IMU only | wheels and IMU, the field corrected by landmarks |
| Field shown | nominal, from `field.xml` | nominal plus observed estimates |

Turning the camera on never turns field correction on. The world estimator in
the preview configuration is `noop`: it detects nothing, associates nothing
and corrects nothing. Frames are published raw for the viewer with
`has_observations` false. The viewer labels metric output unavailable because
the frames carry no intrinsics.

## A missing camera

Software optionality only: a camera that is absent, unplugged, busy or fails
to start at launch is a startup warning and an unavailable `front_camera`
sensor. Localization, the Brain link, the Pico link and the viewer run on
(tested with a camera that never opens: same pose, placed, healthy state
block). This is not a promise about hardware: do not connect or disconnect a
CSI ribbon camera while the Pi is powered.

One case does stop startup: a navigatr binary built without the camera
backend refuses `libcamera_camera` and names the missing backend. Use
`brain_profile_usb.xml` with such a binary.

## Bounded preview

- Capture: 640x480 Y8 at 15 Hz (`<Capture>` in the config). The capture
  thread copies each frame once; nothing queues frames.
- Viewer: at most `preview_hz="5"` frames per second, scaled to at most
  `preview_max_width="640"` pixels, JPEG quality 70, per client and replaced
  rather than queued when a client is slow (see [inspection](inspection.md)).
- The capture mode is a selection, not a measurement. If libcamera refuses it
  on your camera, pick a mode from `rpicam-hello --list-cameras` and change
  `width_px`, `height_px` and `frame_rate_hz` (see
  [Raspberry Pi camera setup](pi_camera_setup.md#choosing-the-capture-mode)).

## Setup on the Pi

### Packages

```sh
sudo apt update
sudo apt install libcamera-dev rpicam-apps pkg-config cmake g++
```

Check the camera is detected, and note its index (the config uses index 0):

```sh
rpicam-hello --list-cameras
```

### Build with the camera backend

Build into its own directory so the camera-free `build-bench` stays as it is:

```sh
cd ~/GATR2-Electronics/pi/naviGATR
cmake -S . -B build-camera -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_BUILD_TESTS=OFF -DNAVIGATR_WITH_LIBCAMERA=ON
cmake --build build-camera -j2
```

### Select the configuration

Run it once by hand (stop the service first so two processes do not compete
for the UARTs and the viewer port):

```sh
sudo systemctl stop navigatr
./build-camera/navigatr config/override/brain_profile_usb_camera.xml
```

Or install it as the service:

```sh
sudo bash tools/install_service.sh --user "$USER" --binary build-camera/navigatr \
  --config config/override/brain_profile_usb_camera.xml
sudo systemctl status navigatr --no-pager
sudo journalctl -u navigatr -n 50 --no-pager
```

The journal names a camera that failed to open as a warning; everything else
starts normally.

### View it (Windows)

Tunnel the viewer port and open the viewer, as for any configuration:

```powershell
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
```

Then open `http://127.0.0.1:8765/` in the browser and choose `front_camera`
in the camera panel. The field, the robot pose, the path and the trail show
as without the camera.

### Back to no camera

```sh
sudo bash tools/install_service.sh --user "$USER" --binary build-bench/navigatr \
  --config config/override/brain_profile_usb.xml
```

## What needs restarting or re-uploading

| Change | Pi | Brain | Pico |
|---|---|---|---|
| Switch to or from the preview config | rerun the installer with the other `--binary` and `--config` (it restarts the service) | nothing | nothing |
| First camera build | build `build-camera` once | nothing | nothing |
| Change the capture mode or preview limits | edit the XML, restart navigatr | nothing | nothing |
| Robot geometry, encoders, IMU source | nothing | edit `gatr2_robot.h`, rebuild, upload | nothing |

No Brain or Pico upload is needed for the preview: the Brain profile, the
Brain link and the Pico link are unchanged. (The Pico diagnostic frames and
the Brain TELEMETRY report are separate, optional additions; see
[Pico link](pico_link.md#diagnostics) and docs/interfaces.md.)

## Later: calibrated estimation

Needed before any camera measurement may correct the field, in this order:

1. Intrinsic calibration at the exact capture mode used for estimation
   (usually larger than the preview mode): capture checkerboard images
   through inspection and export the camera XML with
   [tools/calibrate_camera.py](../tools/calibrate_camera.py), as in
   [setup, camera calibration](setup.md#bring-up-and-calibrate-the-camera).
   Add the `<Calibration>` element (with `<Extrinsic frame_id=.../>`) to the
   camera resource.
2. The measured camera mounting (position and rotation on the robot) in a
   `robot_frame_map`, as in `config/shared/robots/gatr2_front_camera.xml.in`.
3. Measured tag sizes on the physical goals (`apriltag_detector`), see
   [field assets](field_assets.md).
4. A world estimation pipeline with the AprilTag estimator and
   `tag_mount_association` (see `config/shared/pipelines/` and the
   three-wheel camera templates), with its landmark commit policy chosen
   deliberately.
5. Check the whole chain with the camera still and the robot moving before
   letting estimates commit.

A Brain-profile configuration with calibrated estimation has not been built
or tested yet. Brain profiles that carry camera mounts are refused today
(`camera`): the mount would come from Pi XML, as in step 2.
