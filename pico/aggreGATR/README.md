# Pico encoder firmware

The default RP2040 firmware counts two A/B quadrature encoder channels and sends
16 byte encoder records to the Pi over UART at 50 Hz. It runs from
[`src/main.cpp`](src/main.cpp), with no IMU code, host commands or localization.
The Pi forwards the counts to the Brain, where the VEX IMU supplies heading.
See the [localization demo guide](../../brain/localization-demo/README.md) for
the complete setup.

## Build and upload

From `pico/aggreGATR`, with the Pico connected by USB:

```sh
pio run
pio run -t upload
```

Both commands select the default `hat2_encoder_demo` environment. The explicit
form, `pio run -e hat2_encoder_demo -t upload`, is equivalent. The default build
uses `src/main.cpp`, `quadrature.cpp` and the shared
[encoder record codec](../../encoder-record/README.md).

## Wiring and status

The firmware uses the existing HAT v2 wiring:

| Signal | Pico pins | Connector / role |
|---|---|---|
| Encoder channel 0 | GP0 / GP1 | J2, sideways tracking wheel |
| Encoder channel 1 | GP2 / GP3 | J3, forward tracking wheel |
| UART0 to Pi | TX GP16 / RX GP17 | 115200 baud |

Pins are defined in [`src/board.h`](src/board.h); UART settings, the 50 Hz send
rate and USB diagnostics are in [`src/encoder_config.h`](src/encoder_config.h)
under `encoder_cfg`. The firmware runs without a receiver. Counts accumulate
from boot and are never reset while running; each boot selects a random run id
so the Brain can detect a restart.

`pio device monitor` shows status twice a second when USB is connected:

```text
encoder demo run=3F2A seq=1234 ch0=-152 ch1=8801 sent=1235 skipped=0
```

The firmware never waits for USB to connect. Set `encoder_cfg::kUsbDebug` to
`false` and rebuild to disable USB text. The complete Pico-to-Brain route has
not been tested on hardware.

## Optional IMU sensor monitor

The separate sensor monitor is retained in
[`src/sensor_monitor/main.cpp`](src/sensor_monitor/main.cpp) for local bench
diagnostics. It reads three A/B quadrature encoder channels and IMU yaw rate over
SPI1, taking a local sensor snapshot at 50 Hz. It has no Pi or Brain transport,
accepts no host commands, and performs no localization. Select an IMU environment
explicitly to build or upload it:

```sh
pio run -e hat2_bno08x -e hat2_asm330
pio run -e hat2_bno08x -t upload
```

- `hat2_bno08x`: revision 2 sensor wiring with a BNO08X.
- `hat2_asm330`: revision 2 sensor wiring with an ASM330LHHG1; use this environment
  in the upload command for that sensor.

Uploading a sensor monitor replaces the encoder sender. Run `pio run -t upload`
to restore the default encoder firmware. The sections below describe only the
optional sensor monitor.

The IMU model is selected at build time: BNO08X (three-axis gyro projected onto
the up axis learned at startup, averaged per acquisition interval) or ASM330LHHG1
(physical gyro Z). A missing or failed IMU, or a BNO08X still aligning, makes yaw
unavailable for that sample; encoder acquisition continues.

[Pins](src/board.h), [sensor monitor settings](src/config.h), and
[local IMU health labels](src/imu_status.h) belong to this monitor. It does not
use the retired shared communication protocol. The former Pi UART loop,
commands, frame scheduling and diagnostic packet code are not included here.

The existing revision 2 sensor pin mapping and PlatformIO environment names
remain for hardware bring-up. The corresponding board design is archived in
[pcb/PiHat](../../pcb/PiHat/). GPIO 16/17 are not
configured or reserved for Pi UART by the sensor monitor; this does not change the
wiring on an existing board. The BNO08X uses CEVA's sh2 driver,
[vendored here](third_party/README.md).

### IMU retry

The IMU is enabled at boot. A failing device is retried by the Pico itself;
encoder acquisition continues during recovery.

- An episode starts at boot or when a device that had run for 1 s fails. The
  local driver API also retains enable/reinitialize operations. Five quick
  attempts wait 0.5, 1, 2 and 4 s between them (doubling, capped at 8 s). After
  the fifth failure the state is `Failed` and an attempt runs every 30 s until
  one succeeds.
- Every (re)initialization advances the local IMU epoch: each BNO08X reset or
  hub reboot and each ASM330 probe.
- Local status reports state (Disabled, Initializing, Aligning, Ready, Retrying,
  Failed), the last failure reason and the attempts in the episode.
- BNO08X: the supervisor keeps its boot (500 ms), acknowledge (200 ms) and
  stale-stream (100 ms) timeouts. A failed `sh2_open` is retried like any
  other attempt.
- ASM330: a wrong WHO_AM_I is retried, the software reset is polled without
  blocking (100 ms bound), and every 250 ms WHO_AM_I and the gyro configuration
  are checked; two failed checks in a row start a new attempt.

Bounded blocking left in the loop: the BNO08X `sh2_open` (10 ms reset pulse,
then up to 200 ms waiting for the boot) until it first succeeds, normally
once in `setup()`, and the BNO08X WAKE handshake (at most 2 ms). A failed
open returns after the pulse.

### BNO08X startup alignment

The driver enables accelerometer report `0x01` at 100 Hz and uncalibrated gyro
report `0x07` at 200 Hz. Keep the robot stationary and level at startup and
after an IMU reset. The fixed IMU mounting can be upside down or sideways.
The accelerometer average defines sensor-space up; projecting all three gyro
axes onto it makes counterclockwise yaw positive.

Alignment needs at least two seconds and 200 unique acceleration samples,
with acceleration magnitude within 0.5 m/s^2 of 9.80665, vector variation at
most 0.25 m/s^2, gyro magnitude at most 0.10 rad/s, and stream gaps/skew at most
50 ms. Motion restarts collection. Yaw remains unavailable until aligned and
both reports are acknowledged and fresh; local diagnostics show `ALIGNING`
meanwhile. Acquisition does not remove gyro bias.

The learned axis stays fixed until reset. It does not measure live pitch/roll
or compensate dynamic rocking. The ASM330 path keeps its existing physical
gyro-Z behavior. A hub reset repeats BNO08X alignment. After remounting the
IMU, reset it and repeat the stationary startup.

### Sensor monitor host tests

Host tests for the portable acquisition logic (IMU retry and supervisors,
report decoding, SPI headers, gyro windows and alignment) live in
[tests](tests/CMakeLists.txt):

```sh
cmake -S tests -B build/tests
cmake --build build/tests
ctest --test-dir build/tests --output-on-failure
```

### Local USB sensor monitor

The Pico runs without a USB host, Pi, Brain, or robot XML. Flash the selected
IMU variant, connect the Pico's USB to your computer with a data cable, and
open its serial port at 115200 baud. From `pico/aggreGATR`:

```sh
pio device list
pio device monitor --port COM7 --baud 115200
```

Replace `COM7` with the Pico's listed port (`/dev/ttyACM0` is common on Linux).
Exit the PlatformIO monitor with Ctrl+C. Text output updates at most five times
per second:

```text
t_ms=3040 enc0=120 enc1=-48 enc2=0 imu_yaw_mdps=235
```

- `enc0` is J2, `enc1` is J3, and `enc2` is J4. Counts are cumulative since
  Pico startup. Turn each connected wheel by hand and check that its count
  changes and reverses direction. An unused channel may stay zero; a reported
  zero does not prove that an encoder is connected.
- `imu_yaw_mdps` is yaw rate in millidegrees per second: `1000` means 1 degree
  per second. Counterclockwise rotation viewed from above should be positive.
  This is rate, not heading; gyro bias has not been removed.
- `NO_SAMPLE` means that acquisition tick had no usable IMU measurement. This
  is expected during the BNO08X's initial stationary, level alignment; it can
  also mean missing data, a reset, a disabled IMU, or an IMU wiring/communication
  fault. A persistent `NO_SAMPLE` needs investigation; it is not zero rotation.

An additional `imu=BNO08X` status line prints at most once per second:

- `state=WAIT_BOOT` / `RESET_WAIT`: waiting for a hub boot response or holding
  reset before a retry. `DISABLED`: disabled through the local driver API.
  `OPEN_ERROR`: sh2 is not open yet; the open is retried. `last` records the
  last recovery cause; `retry` counts recovery attempts since startup,
  `attempts` the attempts in the current episode, and `epoch` the local IMU
  epoch. `boot` counts actual hub boot notifications.
- `state=WAIT_FEATURES`: waiting for report acknowledgements. `ack=1` means
  acceleration acknowledged, `2` gyro, and `3` both.
- `state=ALIGNING`: report streams are fresh; `cal` shows the current stationary
  acceleration sample count. `READY` means alignment and fresh streams are available.
- `a` / `g` count accepted acceleration/gyro reports since Pico startup; `reject`
  counts rejected callbacks. `age_us` is the last callback's receipt time minus
  its sample timestamp, not the age of the current displayed yaw.
- `rx` counts valid SPI headers, `bad` invalid nonempty headers, and `hdr` is the
  last four SPI header bytes. `int`, `rst`, and `wake` are the Pico pin levels.
  These distinguish a sensor transport problem from alignment; a successful
  software open alone does not prove that the physical IMU responded.

The ASM330 build prints `imu=ASM330 state=... attempts=... reason=... epoch=...
hold_ms=...` instead. Counters persist across IMU retries; `last` can therefore
describe an earlier failure even after recovery.

USB text is local instrumentation, not a Pi/Brain protocol. It uses the same
50 Hz acquisition snapshot without consuming another IMU reading. The firmware
never waits for USB to connect and skips lines when its buffer is full.
Set `cfg::kUsbDebug` to `false` in [`src/config.h`](src/config.h) and rebuild to
disable text output; acquisition continues. Firmware builds and host tests do
not replace validation on a real Pico and connected sensors.
