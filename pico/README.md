# Pico acquisition firmware

RP2040 firmware for the Pi HAT. It reads three A/B quadrature encoder channels
and IMU yaw rate over SPI1, and sends timestamped sensor frames to
the Pi over UART at 50 Hz. Localization runs on the Pi.

The IMU model is picked at build time: a BNO08X (three-axis gyro projected onto
the up axis learned at startup, averaged per frame) or an ASM330LHHG1 (physical gyro
Z). A missing or failed IMU, or a BNO08X still aligning, leaves the gyro bit
clear in the frame; encoders and the UART stream keep running.

[Pins](src/board.h), [rates and build selection](src/config.h), and
[the shared wire format](../common/frames.h) define the hardware and protocol.
The BNO08X uses CEVA's sh2 driver, [vendored](third_party/README.md).

## BNO08X startup alignment

The driver enables accelerometer report `0x01` at 100 Hz and uncalibrated gyro
report `0x07` at 200 Hz. Keep the robot stationary and level at startup and
after an IMU reset. The fixed IMU mounting can be upside down or sideways.
The accelerometer average defines sensor-space up; projecting all three gyro
axes onto it makes counterclockwise yaw positive.

Alignment needs at least two seconds and 200 unique acceleration samples,
with acceleration magnitude within 0.5 m/s² of 9.80665, vector variation at
most 0.25 m/s², gyro magnitude at most 0.10 rad/s, and stream gaps/skew at most
50 ms. Motion restarts collection. Yaw remains unavailable until aligned and
both reports are acknowledged and fresh. On localization startup, the Pi
collects its 200 bias samples after yaw becomes available, about four seconds
at 50 Hz: allow about six seconds of stillness for a complete startup.

The learned axis stays fixed until reset. This does not measure live pitch/roll
or compensate dynamic rocking; the Brain still supplies field heading. The
ASM330 path keeps its existing physical gyro-Z behavior. A hub reset repeats
Pico alignment but does not itself restart Pi bias collection. After remounting
the IMU, restart both the IMU and localization and repeat the stationary startup.

## Build

One PlatformIO environment per HAT revision and IMU model:

- `hat2_bno08x`, the default: HAT revision 2 with a BNO08X
- `hat2_asm330`: HAT revision 2 with an ASM330LHHG1

```sh
pio run -e hat2_bno08x
```

Upload by hand with `pio run -e hat2_bno08x --target upload` once the board is
connected.

Host tests for the pure driver logic live in [tests](tests/CMakeLists.txt):

```sh
cmake -S tests -B build/tests && cmake --build build/tests && ctest --test-dir build/tests
```

## USB sensor monitor

The Pico runs by itself; no Pi, Brain, or robot XML is needed for this test.
Flash `hat2_bno08x`, connect the Pico's USB to your computer with a data cable,
and open its serial port at 115200 baud. From `pico/`:

```sh
pio device list
pio device monitor --port COM7 --baud 115200
```

Replace `COM7` with the Pico's listed port (`/dev/ttyACM0` is common on Linux).
Exit the PlatformIO monitor with Ctrl+C. Output updates five times per second:

```text
t_ms=3040 enc0=120 enc1=-48 enc2=0 imu_yaw_mdps=235
```

- `enc0` is J2, `enc1` is J3, and `enc2` is J4. Counts are cumulative since
  Pico startup. Turn each connected wheel by hand and check that its count
  changes and reverses direction. An unused channel may stay zero; a reported
  zero does not prove that an encoder is connected.
- `imu_yaw_mdps` is yaw rate in millidegrees per second: `1000` means 1 degree
  per second. Counterclockwise rotation viewed from above should be positive.
  This is rate, not heading, and Pi gyro-bias removal has not been applied.
- `NO_SAMPLE` means that telemetry tick had no usable IMU measurement. This
  is expected during the BNO08X's initial two-second stationary, level alignment;
  it can also mean missing data, a reset, or an IMU wiring/communication fault.
  A persistent `NO_SAMPLE` needs investigation; it is not a zero rotation rate.

An additional `imu=BNO08X` status line prints once per second:

- `state=WAIT_BOOT` / `RESET_WAIT`: waiting for a hub boot response or holding
  reset before a retry. `last` records the last recovery cause; `retry` counts
  recovery attempts. `boot` counts actual hub boot notifications.
- `state=WAIT_FEATURES`: waiting for report acknowledgements. `ack=1` means
  acceleration acknowledged, `2` gyro, and `3` both.
- `state=ALIGNING`: report streams are fresh; `cal` shows the current stationary
  acceleration sample count. `READY` means alignment and fresh streams are available.
- `a` / `g` count accepted acceleration/gyro reports since Pico startup; `reject`
  counts rejected callbacks. `age_us` is the last callback's receipt time minus
  its sample timestamp, not the age of the current displayed yaw.
- `rx` counts valid SPI headers, `bad` invalid nonempty headers, and `hdr` is the
  last four SPI header bytes. `int`, `rst`, and `wake` are the Pico pin levels.
  These distinguish a transport problem from calibration; a successful software
  open alone does not prove that the physical IMU responded.

Counters persist across IMU retries; `last` can therefore describe an earlier
failure even after recovery. Paste several status lines when diagnosing a fault.

The monitor uses the same sample as the 50 Hz binary Pi UART output. It does
not consume another IMU reading or wait for USB to connect; log lines are
skipped when the USB buffer is full. Set `cfg::kUsbDebug` to `false` in
[`src/config.h`](src/config.h) to disable the text output and rebuild.
