# Raspberry Pi camera setup

For the complete robot workflow, use [Set up and run Navigatr](setup.md).
Its [camera calibration section](setup.md#bring-up-and-calibrate-the-camera)
uses the included [OpenCV helper](../tools/calibrate_camera.py) to capture
full-size images through inspection and export the camera XML calibration.

How to build and run the `libcamera_camera` backend on the Pi, what it
assumes about the camera stack, and what has not been checked on hardware.
The backend lives in `src/impl/resources/libcamera_camera.cpp` and is
compiled only with `-DNAVIGATR_WITH_LIBCAMERA=ON`.

Nothing in this backend has run on a Pi yet. The last section lists the
checks to perform before any capture-timing number is trusted.

## Supported stack

- Raspberry Pi OS Bookworm or newer with its matching libcamera packages.
  Record the installed version with `pkg-config --modversion libcamera`;
  no particular Pi package version has been compiled or tested here.
- `rpicam-apps` supplies `rpicam-hello` to list cameras and modes.
- Camera Module 3 (IMX708), HQ Camera (IMX477) and Global Shutter Camera
  (IMX296) all appear through the same pipeline; the backend does not care
  which sensor it is, only which mode it is asked for.
- `CMakeLists.txt` raises the core library to C++20 with the camera option
  enabled so newer libcamera headers can be used; ordinary host builds
  remain C++17.

## Packages

```text
sudo apt update
sudo apt install libcamera-dev rpicam-apps pkg-config cmake g++ libgtest-dev
```

`libcamera-dev` provides the `libcamera` pkg-config module the build looks
up (`pkg-config --modversion libcamera` prints the installed version).
`rpicam-apps` is preinstalled on the desktop image and `rpicam-apps-lite` on
the Lite image; either is enough. `libcamera-tools` (the `cam` utility) is
optional. `libgtest-dev` lets `tests/` use the system gtest instead of
downloading one.

## Enabling the camera

`/boot/firmware/config.txt` ships with `camera_auto_detect=1`, which probes
the official modules. A module that is not auto-detected needs
`camera_auto_detect=0` plus the sensor overlay, for example
`dtoverlay=imx708`, `dtoverlay=imx477` or `dtoverlay=imx296`. Reboot after
changing it.

Confirm detection and note the index:

```text
rpicam-hello --list-cameras
```

Each camera is printed as `<index> : <sensor> [<size> ...] (<device path>)`
followed by its modes. The leading number is the value for
`<Device index="..."/>`. The backend selects `CameraManager::cameras()[index]`
and fails with the enumerated count when the index is out of range. With
one CSI camera both orderings are index 0; with several cameras, or a USB
webcam attached, rpicam-apps lists its own order (USB cameras dropped,
sorted by id), so confirm the choice against the camera id the device
diagnostic prints.

Another process holding the camera causes `acquire()` to fail with
`EBUSY`. Stop that process before starting capture or resetting the device.

## Choosing the capture mode

`rpicam-hello --list-cameras` prints one line per sensor mode, for example
`1536x864 [120.13 fps - (768, 432)/3072x1728 crop]` for the IMX708 or
`1456x1088 [60.38 fps - (0, 0)/1456x1088 crop]` for the IMX296. Rules:

- `width_px` and `height_px` specify the ISP output size. Invalid
  configurations and adjustments to that size or YUV420 format fail open.
  Adjustments to stride, buffer count or colour space are permitted because
  they do not change image geometry. The sensor mode itself is selected by
  libcamera, so calibrate with this backend and capture configuration; a
  matching output resolution alone does not guarantee a matching crop.
- `frame_rate_hz` must not exceed the listed fps of that mode. The backend
  requests a fixed rate through equal `FrameDurationLimits`, after checking
  the configured mode's supported duration range. The sensor may quantize
  the period; measured `FrameDuration` metadata is used when computing the
  exposure uncertainty. Verify the achieved rate on the Pi.
- `pixel_format="Y8"` is the only value the parser accepts. The Pi ISP has
  no greyscale output through libcamera (a request for `R8` is reclassified
  as a raw sensor stream and adjusted to Bayer), so the backend configures
  `YUV420` and publishes the Y plane, row by row through the stream stride,
  as a packed 8-bit image. The U and V planes are never read.
- The intrinsics must be calibrated at this same size and mode. The parser
  refuses a `Calibration` whose `calibrated_width_px`/`calibrated_height_px`
  differ from `Capture`, and a different sensor crop at the same output size
  is a different calibration too.

Four driver buffers are allocated (`bufferCount=4`), each bound to one
request. A frame handed to the runtime is a copy; the driver buffer is
requeued before the copy is published, so buffer reuse can never touch a
frame being processed or displayed. CPU reads are bracketed with
`DMA_BUF_IOCTL_SYNC` START/END operations, as in
[Raspberry Pi's capture application](https://github.com/raspberrypi/rpicam-apps/blob/main/core/rpicam_app.cpp).
Completed-plane bytes-used and stride are checked before copying; truncated
or errored frames are counted and skipped.

## Building

```text
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DNAVIGATR_WITH_LIBCAMERA=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

Without the option the binary has no capture backend. Selecting
`type="libcamera_camera"` then fails at configuration time with
`libcamera_camera needs the libcamera backend, which is not compiled into
this binary; configure with -DNAVIGATR_WITH_LIBCAMERA=ON ...`; no device is
constructed and nothing pretends to capture. With the option ON but
`libcamera-dev` missing, `cmake` itself stops at
`pkg_check_modules(LIBCAMERA REQUIRED ...)`.

The host tests never exercise the backend: on a machine without libcamera
the tests assert the explicit configuration error instead.

## Exposure timestamp convention

Every frame carries `exposureAt`, `receivedAt`, `exposure_uncertainty_ms`
and `exposure_time_reliable` (see `src/resources/camera.h`).

Source metadata, read from each completed request:

- `SensorTimestamp` (int64, nanoseconds): libcamera defines this on
  `CLOCK_BOOTTIME`, at exposure of the first active sensor row. See the
  [control definition](https://github.com/raspberrypi/libcamera/blob/main/src/libcamera/control_ids_core.yaml).
  The [Picamera2 manual](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf)
  instead describes the Pi frame-start interrupt as the first pixel is
  read out. That difference requires a conservative timing bound and an
  eventual physical measurement on the installed camera/kernel.
- `ExposureTime` (int32, microseconds): the exposure the sensor actually
  used for that frame.

What the backend publishes:

```text
exposureAt  = host(SensorTimestamp) + ExposureTime / 2
receivedAt  = host clock when libcamera completed the request
uncertainty = ceil(max(configured period, measured period, exposure)) ms
              + clock sampling / rounding uncertainty
reliable    = true
```

`host()` samples `CLOCK_BOOTTIME` between two host-clock readings for each
completion and maps the sensor timestamp using that correspondence.
`CLOCK_BOOTTIME` includes suspended time while Linux's steady clock does
not; subtracting the steady-clock origin directly would mix the clocks.
The bracket width and millisecond quantization are included in uncertainty.

The bound covers first-row exposure versus readout-start interpretations
and an unknown rolling readout lasting at most a frame period. Half a
period does not cover all of those cases. This is an estimated temporal
interval for a frame, not a calibrated per-row rolling-shutter correction.
Driver timestamps that violate the documented clock domain still require
hardware diagnosis; a source timestamp cannot identify its own clock.

Degradations, all visible in the frame:

- `SensorTimestamp` absent, the clock sample unavailable, or invalid/future
  metadata: `exposureAt = receivedAt`, `exposure_time_reliable = false`.
  Receipt-time fallback does not supply a bounded physical exposure time.
- `ExposureTime` absent: `exposureAt = host(SensorTimestamp)`, reliable,
  uncertainty remains at least one full frame period plus clock conversion.

The consumer side is unchanged: the association looks the robot pose up at
`exposureAt` and the inspection documents show `exposure_host_ms`,
`received_host_ms`, `exposure_uncertainty_ms` and `exposure_time_reliable`
per detection frame. Receipt is never exposure.

Manual exposure and gain are a follow-up. The backend leaves auto exposure
and gain at their defaults; a fixed exposure would use `AeEnable` on
libcamera 0.3/0.4 and `ExposureTimeMode`/`AnalogueGainMode` from 0.5, set in
the request controls (where `Camera::queueRequest` translates them), behind
`LIBCAMERA_VERSION_MAJOR`/`MINOR` guards.

## Running

Live camera inspection before metric calibration, the profile the README
names (uncalibrated camera fragment, no metric pose, detections drawn on
the image):

```text
./build/navigatr config/override/diagnostics/live_camera_inspection.xml
```

The hardware-free demo works on the Pi as well and needs no camera:

```text
./build/navigatr config/demo/synthetic_field_demo.xml
```

Any profile can expose the inspection service without editing it:

```text
./build/navigatr <profile.xml> --inspect-port 8765
```

At startup navigatr prints `warning: <resource path>: <reason>` when the
camera could not open; the resource then reports `unavailable` with that
reason and the robot keeps running without it. The `hello` document lists
each camera with `alive` and `diagnostic`; for a live device the diagnostic
reads
`libcamera <runtime version> (headers <compiled version>); camera <id>;
stream <WxH-YUV420> stride <n> (Y plane as Y8); frames captured N,
replaced M, errors E; request sequence gaps G, frame sequence gaps F`,
with `; error: ...` appended after a failure. `replaced` counts frames a
newer one displaced before the runtime took them (expected to be small at a
loop rate above the frame rate), `frame sequence gaps` counts jumps in the
driver's frame counter (dropped frames), `request sequence gaps` jumps in
libcamera's request counter.

From the viewing computer, forward the loopback port and open the page:

```text
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
http://127.0.0.1:8765/
```

## Orderly shutdown

Ctrl-C (SIGINT or SIGTERM) stops the inspection service first, then the
estimation worker, then the field worker, then destroys the system. The
camera resource is destroyed with it, in this order: the capture thread is
signalled and joined; `Camera::stop()` cancels the requests still in
flight (they complete as `RequestCancelled` and are dropped); the
`requestCompleted` signal is disconnected; requests are destroyed; the
mapped Y planes are unmapped; the buffers are freed; the camera is released;
the process-wide `CameraManager` stops when the last camera has released.
No thread is ever detached.

`reset()` (an in-process restart) does the stop half, bumps the frame epoch,
releases and reacquires the camera, and restarts sequence numbering at 1.
This also retries an initial open failure or a reconnected device. A
restart that fails leaves the device unavailable with a diagnostic.
Reset must be invoked with System workers stopped, as for other resources.

Unexpected cancelled requests, disconnection, queue failures, CPU access
errors, and no completions for the larger of one second or five configured
frame periods mark capture unavailable. The capture thread exits and reset
can retry. No background automatic reconnect loop is implemented. Failure
cannot clear an error raised by the capture thread during startup.

## Not verified on hardware

Nothing in this backend has run on a Pi. Before relying on it, run these
checks with the live inspection profile and record the results:

1. The camera opens at the configured mode: no `warning:` line at startup,
   `hello.cameras[].alive` true, the diagnostic shows the requested size,
   `YUV420`, and a stride at least the width. Try a size the pipeline must
   adjust and confirm `open()` fails naming that size. Ordinary padded
   stride must continue working.
2. The frame rate matches `frame_rate_hz`: `sources[]` for the camera
   resource and `detection_frames[].sequence` advance at that rate (the
   field worker itself runs at the loop rate and only detects on new
   frames). `FrameDuration` in the metadata should equal the configured
   period; add it to the diagnostic if it does not.
3. `SensorTimestamp` is present: every detection frame shows
   `exposure_time_reliable` true and an uncertainty of at least a frame
   period. False means metadata or clock conversion was unavailable or
   unusable and receipt-time fallback is in use.
4. Exposure latency is plausible: `received_host_ms - exposure_host_ms`
   should be a stable few tens of milliseconds (one frame period plus ISP
   and copy time), never negative and never growing. Then measure the
   actual offset once, for example with an LED switched by a GPIO at a
   known host time, and decide whether `SensorTimestamp` on this kernel
   behaves as readout start or exposure start; adjust the convention above
   if the measured offset says so.
5. Detector throughput at the chosen resolution: `detector_processing_ms`
   in the detection frame and the field worker's `mean_cycle_ms` must stay
   below the frame period, otherwise `workers.field.dropped` climbs and the
   camera path runs at a fraction of the frame rate. Lower the resolution
   or raise `quad_decimate` before trusting timing numbers.
6. No dropped frames at rest: with the robot still, the diagnostic's
   `frame sequence gaps` and `errors` stay at zero over several minutes and
   `replaced` stays small.
7. Restart and shutdown: a `System::reset()` (no runtime trigger exists
   yet; drive it from a small test binary or a temporary hook) produces a
   new epoch and frames keep flowing; also test starting while another
   process owns the camera, releasing it, then resetting. Ctrl-C returns
   promptly with no libcamera warning about unreleased buffers.
8. Two navigatr processes must not share one camera; the second should
   open dead with the `EBUSY` reason, not hang.
