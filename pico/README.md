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
[docs/interfaces.md](../docs/interfaces.md) describes the sensor frames and the
Pico control frames. The BNO08X uses CEVA's sh2 driver,
[vendored](third_party/README.md).

## Pi link

- Sensor frames are v2 (type 0x04): each carries `boot_id` (random and nonzero
  per boot, from RP2040 ROSC random bits mixed with the timer), `acq_epoch` and
  `imu_epoch`. The largest frame is 31 bytes and goes out only when the 32-byte
  UART TX FIFO is empty; a tick with a busy FIFO is skipped, never delayed.
- The Pico reads Pi commands (type 0x12) from the same UART without blocking
  and answers in status frames (type 0x13): every 200 ms, and about one tick
  after a command or an IMU state change. A status frame goes only while the
  TX FIFO is empty and at least its airtime plus 1 ms before the next sensor
  tick, so it never delays a sensor frame.
- Commands run only when `target_boot_id` is this boot. The last 4 commands
  (request id, op, body) are recorded; a resend is answered from its record
  and never run again.
  - CONFIGURE sets whether the IMU is enabled (enabled at boot). Disabling
    holds the IMU idle and stops its retries.
  - REINIT_IMU restarts IMU port 0 with fresh quick attempts and a new
    `imu_epoch`; `Running` until the IMU is ready, then `Completed`, or
    `Failed` (ImuAbsent, ImuDisabled, NoSuchPort).
  - RESTART_ACQUISITION zeroes the three encoder counters with interrupts off,
    advances `acq_epoch` and completes at once.
  - Unknown ops, wrong body lengths and bad values fail with a detail.
- Pi software that predates v2 frames ignores type 0x04. Update the Pi before
  flashing this firmware. The Pi side is described in
  [pi/navigatr/docs/pico_link.md](../pi/navigatr/docs/pico_link.md).

## IMU retry

While enabled, a failing IMU is retried by the Pico itself; encoders and
frames keep flowing.

- An episode starts at boot, at enable, at REINIT_IMU, or when a device that
  had run for 1 s fails. Five quick attempts wait 0.5, 1, 2 and 4 s between
  them (doubling, capped at 8 s). After the fifth failure the state is
  `Failed` and an attempt runs every 30 s until one succeeds.
- Every (re)initialization advances `imu_epoch`: each BNO08X reset or hub
  reboot and each ASM330 probe.
- The status frame reports state (Disabled, Initializing, Aligning, Ready,
  Retrying, Failed), the last failure reason and the attempts in the episode.
- BNO08X: the supervisor keeps its boot (500 ms), acknowledge (200 ms) and
  stale-stream (100 ms) timeouts. A failed `sh2_open` is retried like any
  other attempt.
- ASM330: a wrong WHO_AM_I is retried, the software reset is polled without
  blocking (100 ms bound), and every 250 ms WHO_AM_I and the gyro
  configuration are checked; two failed checks in a row (an unplugged or reset
  chip) start a new attempt.

Bounded blocking left in the loop: the BNO08X `sh2_open` (10 ms reset pulse,
then up to 200 ms waiting for the boot) until it first succeeds, normally
once in `setup()`, and the BNO08X WAKE handshake (at most 2 ms). A failed
open returns after the pulse.

## BNO08X startup alignment

The driver enables accelerometer report `0x01` at 100 Hz and uncalibrated gyro
report `0x07` at 200 Hz. Keep the robot stationary and level at startup and
after an IMU reset. The fixed IMU mounting can be upside down or sideways.
The accelerometer average defines sensor-space up; projecting all three gyro
axes onto it makes counterclockwise yaw positive.

Alignment needs at least two seconds and 200 unique acceleration samples,
with acceleration magnitude within 0.5 m/s^2 of 9.80665, vector variation at
most 0.25 m/s^2, gyro magnitude at most 0.10 rad/s, and stream gaps/skew at most
50 ms. Motion restarts collection. Yaw remains unavailable until aligned and
both reports are acknowledged and fresh; the status frame shows `Aligning`
meanwhile. The Pi then collects its gyro bias while the robot stays still:
allow about six seconds of stillness for a complete startup.

The learned axis stays fixed until reset. This does not measure live pitch/roll
or compensate dynamic rocking; the Brain still supplies field heading. The
ASM330 path keeps its existing physical gyro-Z behavior. A hub reset repeats
Pico alignment and advances `imu_epoch`, which tells the Pi to restart its bias
calibration. After remounting the IMU, reinitialize it and localization and
repeat the stationary startup.

## Build

One PlatformIO environment per HAT revision and IMU model:

- `hat2_bno08x`, the default: HAT revision 2 with a BNO08X
- `hat2_asm330`: HAT revision 2 with an ASM330LHHG1

```sh
pio run -e hat2_bno08x -e hat2_asm330
```

Upload by hand with `pio run -e hat2_bno08x --target upload` once the board is
connected.

Host tests for the portable logic (IMU retry and supervisors, command handling
and records, status scheduling, report decoding and alignment) live in
[tests](tests/CMakeLists.txt):

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
  Pico startup or the last RESTART_ACQUISITION. Turn each connected wheel by
  hand and check that its count changes and reverses direction. An unused
  channel may stay zero; a reported zero does not prove that an encoder is
  connected.
- `imu_yaw_mdps` is yaw rate in millidegrees per second: `1000` means 1 degree
  per second. Counterclockwise rotation viewed from above should be positive.
  This is rate, not heading, and Pi gyro-bias removal has not been applied.
- `NO_SAMPLE` means that telemetry tick had no usable IMU measurement. This
  is expected during the BNO08X's initial two-second stationary, level alignment;
  it can also mean missing data, a reset, a disabled IMU, or an IMU
  wiring/communication fault. A persistent `NO_SAMPLE` needs investigation;
  it is not a zero rotation rate.

An additional `imu=BNO08X` status line prints once per second:

- `state=WAIT_BOOT` / `RESET_WAIT`: waiting for a hub boot response or holding
  reset before a retry. `DISABLED`: turned off by CONFIGURE. `OPEN_ERROR`:
  sh2 is not open yet; the open is retried. `last` records the last recovery
  cause; `retry` counts recovery attempts since startup, `attempts` the
  attempts in the current episode, and `epoch` the `imu_epoch`. `boot` counts
  actual hub boot notifications.
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

The ASM330 build prints `imu=ASM330 state=... attempts=... reason=... epoch=...
hold_ms=...` instead.

A `link` line follows with the Pi link identity and command state: `boot`
(hex `boot_id`), `acq`, `imu_epoch`, `imu_state`, `reason` and `attempts` as
sent in status frames, `status_sent`, commands received (`cmd`), duplicates
(`dup`), ignored frames, and `last` as request id/op/status/detail.

Counters persist across IMU retries; `last` can therefore describe an earlier
failure even after recovery. Paste several status lines when diagnosing a fault.

The monitor uses the same sample as the 50 Hz binary Pi UART output. It does
not consume another IMU reading or wait for USB to connect; log lines are
skipped when the USB buffer is full. Set `cfg::kUsbDebug` to `false` in
[`src/config.h`](src/config.h) to disable the text output and rebuild.

The Pi link commands, IMU retries and status frames have been checked only by
the builds and host tests, not on a real Pico.
