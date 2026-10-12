# Inventory: Pico firmware, hardware mapping and Pi telemetry consumption

Read-only pass. No repo files were changed and nothing was built. `git status` is clean. Line numbers are per file.

---

## 0. File map (what exists)

**Pico firmware** (`pico/src/`):
- `board.h`: pin map
- `config.h`: rates and pin budget
- `main.cpp`: tick loop and frame send
- `quadrature.{h,cpp}`: encoders
- `imu.h`: IMU interface
- `imu_bno08x.cpp`, `imu_asm330.cpp`: one driver per build
- `bno08x_supervisor.{h,cpp}`: bring-up and recovery state machine
- `bno08x_reports.{h,cpp}`: SH-2 report decode and gating
- `gravity_alignment.{h,cpp}`: startup up-axis
- `gyro_window.{h,cpp}`: per-tick averaging
- `shtp_header.h`: SPI header validation

`pico/gatr2-pico/` is an empty directory. `pico/third_party/sh2` is CEVA sh2 v1.4.0, vendored (`pico/third_party/README.md`).

**Pico host tests** (`pico/tests/CMakeLists.txt`): 53 gtest cases in 5 files. They cover only the pure logic (`bno08x_reports.cpp`, `gravity_alignment.cpp`, `gyro_window.cpp`, `bno08x_supervisor.cpp`, plus the header-only `shtp_header.h`).
- `bno08x_reports_gtest.cpp`: 9
- `bno08x_supervisor_gtest.cpp`: 9
- `gravity_alignment_gtest.cpp`: 15
- `gyro_window_gtest.cpp`: 13
- `shtp_header_gtest.cpp`: 7

No test covers `quadrature.cpp`, `main.cpp`, `imu_bno08x.cpp` or `imu_asm330.cpp`. CI builds both PlatformIO envs (`.github/workflows/ci.yaml:87`, `pio run -e hat2_bno08x -e hat2_asm330`) and runs the host tests (`ci.yaml:92-94`).

**Shared wire:**
- `common/frames.h`
- `common/frame_codec.{h,cpp}`. The Pico compiles it in via `build_src_filter` (`pico/platformio.ini:18`).

**Pi consumers:**
- `pi/navigatr/src/impl/resources/pico_telemetry.{h,cpp}`: resource type `pico_telemetry`
- `pi/navigatr/src/impl/sensors/pico_channels.{h,cpp}`: sensor types `pico_encoder_channel`, `pico_imu_channel`
- `pi/navigatr/src/payloads/pico_telemetry_samples.h`. It lives under `src/payloads/`, not `src/impl/payloads/`.

Registration:
- `register_resources.cpp:31`: `"pico_telemetry"`
- `register_sensors.cpp:12-15`: `"pico_encoder_channel"`, `"pico_imu_channel"`

**Docs:**
- `docs/hardware.md`
- `pi/navigatr/docs/parallel_wheel_bringup.md`
- `pico/README.md`
- `docs/interfaces.md` (sensor frame section, lines 156-225)
- `docs/navigatr_resources.md:117-150`
- `docs/navigatr_sensors.md:15-70`
- `pi/navigatr/docs/attitude_firmware_followup.md`
- `pi/navigatr/docs/calibration_inventory.md`
- `pi/navigatr/docs/vex_imu_bench.md`
- `pi/navigatr/docs/usb_localization_bench.md`

---

## 1. Encoders

### 1.1 HAT v2 pin and connector mapping

`pico/src/board.h:15-18`:
```cpp
// Encoder A/B inputs. enc0 GP0/GP1 (pins 1/2, J2), enc1 GP2/GP3 (pins 4/5, J3),
// enc2 GP4/GP5 (pins 6/7, J4).
constexpr uint8_t kEncPinA[3] = {0, 2, 4};
constexpr uint8_t kEncPinB[3] = {1, 3, 5};
```

| Channel | HAT v2 connector | A | B | Wire bit |
|---|---|---|---|---|
| 0 | J2 | GP0 (pin 1) | GP1 (pin 2) | `kSensorEnc0` (bit 0) |
| 1 | J3 | GP2 (pin 4) | GP3 (pin 5) | `kSensorEnc1` (bit 1) |
| 2 | J4 | GP4 (pin 6) | GP5 (pin 7) | `kSensorEnc2` (bit 2) |

The same mapping appears in:
- `docs/hardware.md:68-73`
- `parallel_wheel_bringup.md:36-39, 107` ("Wheel A is channel 0 (J2), wheel B is channel 1 (J3)")
- `pico/README.md:76` ("`enc0` is J2, `enc1` is J3, and `enc2` is J4")

I did not trace the schematic nets. The v2 schematic (`pcb/PiHat/PiHat_v2/RP4Hat/RP4Hat.kicad_sch`) has J2, J3 and J4 as `Conn_01x04` and net labels `A0/B0`, `A1/B1`, `A2/B2`, but I did not follow connector pins to GPIOs. `board.h` is the effective source of truth.

Only HAT rev 2 exists in firmware. `board.h:8-11,38-40` gives `#error` for any other `GATR2_HAT_REV`. There is no v3 or v4 pin map. `docs/hardware.md:36` notes that "the v3 HAT's six-pin IMU connector drops [INT]".

The role comments on the wire bits in `common/frames.h:38-40` are stale relative to current use:
- `kSensorEnc0`: "parallel tracking wheel"
- `kSensorEnc1`: "perpendicular tracking wheel"
- `kSensorEnc2`: "spare / second parallel"

The parallel-wheel profile uses channel 1 as the second parallel wheel. The USB bench uses channel 1 as a sideways wheel. The AS5047 template uses left/right/rear on channels 0/1/2. Roles are assigned only on the Pi, by `wheel_geometry`. Nothing enforces the comments.

### 1.2 Decoding

`quadrature.cpp`:
- x4 decode by lookup table. `kDelta[16]` is indexed `(prev<<2)|now` (`quadrature.cpp:15-17`), and illegal transitions (both lines changed) count 0.
- One ISR per channel is attached with `CHANGE` to both A and B (`quadrature.cpp:62-63`).
- Pins use `INPUT_PULLUP` (`:56-57`).
- The count is `volatile int32_t`, set to 0 in `begin()` (`:60`).
- `count(ch)` returns the raw accumulated value (`:67-72`).

The interface is `quadrature.h:11-19`:
```cpp
constexpr uint8_t kChannels = 3;
void begin(uint8_t ch, uint8_t pin_a, uint8_t pin_b);
int32_t count(uint8_t ch);   // Accumulated count
bool started(uint8_t ch);    // True once begin has run
```

`setup()` calls `begin` for all three channels unconditionally (`main.cpp:102-104`). So `started(ch)` is always true, and **all three encoder bits are set in every frame**, whether or not an encoder is plugged in. No connected-encoder detection exists. `pico/README.md:78-79`: "An unused channel may stay zero; a reported zero does not prove that an encoder is connected."

### 1.3 What goes on the wire

- **Cumulative absolute counts**, `int32`, little-endian, 4 bytes each (`frames.h:48-50`, `kSensorWidth`). Counts are cumulative since Pico boot, not deltas (`pico/README.md:76-77`).
- They wrap at 32 bits. The firmware's `c.count += ...` on a signed int would overflow formally but wraps in practice. The Pi handles wrap with modular subtraction (`pico_channels.cpp:193-196`).
- Payload order is ascending bit order. Only present bits are carried (`frames.h:29-35`, `frame_codec.cpp:193-222`).

### 1.4 Timestamp, clock, rate

- `cfg::kSampleHz = 50` and `kTickUs = 1000000u / kSampleHz` = 20 ms (`config.h:24-25`). One tick sends at most one frame.
- The tick cadence is fixed: `g_next_tick += cfg::kTickUs` (`main.cpp:117`), with a signed compare for `micros()` rollover (`main.cpp:114`).
- A tick is **skipped, not delayed**, if the UART TX FIFO is not empty (`main.cpp:121-123`). Nothing is sampled on that tick, so the gyro window carries into the next frame.
- `sample()` (`main.cpp:33-54`) takes one snapshot `now_us = time_us_64()`. It sets `stamp_ms = now_us / 1000` as `uint32_t`, reads the three encoder counts, then calls `imu::readGyroZ(static_cast<uint32_t>(now_us), gyro)`. The encoder latch and the gyro window cut share that snapshot; the encoder reads happen a few microseconds after it.
- `stamp_ms` is Pico milliseconds since boot, truncated from microseconds. It wraps after about 49.7 days (`docs/interfaces.md:187-190`).
- `seq` is `uint8_t`. It increments only after a successful write (`main.cpp:132`) and restarts at 0 on reboot.
- Link: UART0/`Serial1`, TX GP16 (pin 21), RX GP17 (pin 22), 115200 baud (`board.h:33-34`, `config.h:21`).
- **The Pico never reads the Pi UART.** A grep of `pico/src` finds no `Serial1.read` or `available`. The link is strictly one-way. No Pi-to-Pico configuration channel exists.

### 1.5 Frame format (Pico telemetry part of `common/frames.h`)

`frames.h:28-63`:
```
sync0 0xAA | sync1 0x55 | type 0x01 | seq u8 | stamp_ms u32 | mask u16
| payload (present sensors only, ascending bit order) | xor u8
```

| Bit | Constant | Width | Meaning |
|---|---|---|---|
| 0 | `kSensorEnc0` | 4 | int32 counts |
| 1 | `kSensorEnc1` | 4 | int32 counts |
| 2 | `kSensorEnc2` | 4 | int32 counts |
| 3 | `kSensorGyroZ` | 4 | int32 mdeg/s |
| 4 | `kSensorAccelXY` | 8 | 2 x int32 mg, "reserved" |

The struct, from `frames.h:56-63`:
```cpp
struct SensorSample {
    uint8_t  seq;
    uint32_t stamp_ms;   // Pico acquisition clock; the Pi maps it to host time
    uint16_t mask;
    int32_t  enc[3];
    int32_t  gyro_z;     // mdeg/s, raw, bias not removed
    int32_t  accel[2];   // mg
};
```

- All fields are little-endian (`frame_codec.cpp` `wr32`/`rd32`).
- The checksum is the XOR of every preceding byte, so a valid frame XORs to 0 (`frames.h:189-195`, `frame_codec.cpp:221, 237`).
- `kSensorHeaderLen = 10` (`frame_codec.h:14`).
- The largest frame the Pico actually sends is 10 + 3x4 + 4 + 1 = 27 bytes, statically asserted ≤ 32, the TX FIFO depth (`main.cpp:21-23`).
- The codec rejects unknown mask bits (`frame_codec.cpp:145-148`).
- **There is no version byte in the sensor frame** (`docs/interfaces.md:619-622`).
- **Bit 4 (accel) is never set by any firmware** (`docs/hardware.md:52-53`, `attitude_firmware_followup.md`).

Codec signatures (`frame_codec.h:47,53,68-97`):
```cpp
uint16_t encodeSensorFrame(const SensorSample& in, uint8_t* buf, uint16_t cap);
bool decodeSensorFrame(const uint8_t* buf, uint16_t len, SensorSample& out);
class FrameReader { void reset(); bool push(uint8_t b); bool next(); ... uint8_t frameType() const; };
```

---

## 2. The IMU

### 2.1 Build selection

`pico/platformio.ini`:
- `default_envs = hat2_bno08x` (`:7`)
- `[env:hat2_bno08x]`: `-DGATR2_HAT_REV=2 -DGATR2_IMU_BNO08X` (`:20-21`)
- `[env:hat2_asm330]`: `-DGATR2_HAT_REV=2 -DGATR2_IMU_ASM330`, with `lib_ignore = sh2` (`:23-25`)

`config.h:13-15` requires exactly one of the two defines. Each driver file is wrapped in `#if defined(...)` (`imu_bno08x.cpp:7`, `imu_asm330.cpp:5`).

**The Pi cannot tell which chip is running.** Neither the frame nor any other channel identifies it. The only chip ID is the USB-debug text line (`imu=BNO08X ...` or `imu=ASM330 ...`).

Hardware (SPI1, J7), from `board.h:20-30` and `docs/hardware.md:20-37`:
- INT GP22 (J7-2)
- MISO GP8 (J7-3)
- MOSI GP11 (J7-4)
- SCK GP10 (J7-5)
- CS GP9 (J7-6)
- 3.3 V on J7-7; GND on J7-1 and J7-8
- BNO08X only, extra wires off the PCB: RST GP12 (pin 16), PS0/WAKE GP13 (pin 17), PS1 tied to 3.3 V

Pin budget (`config.h:4-5, 37-45, 66-67`): 15 of 26 usable GPIOs are assigned with the BNO08X. My count of the free ones: GP6, GP7, GP14, GP15, GP18-21, GP26-28.

There is exactly one IMU slot on the Pico: one SPI bus, one CS, one driver.

### 2.2 The common interface (`imu.h:13-29`)

```cpp
void begin();
void service();   // non-blocking, every loop pass
// Yaw rate in millidegrees per second, bias not removed. cut_us is the tick's
// time snapshot on the time_us_32() clock. True only for a new measurement.
bool readGyroZ(uint32_t cut_us, int32_t& gyro_z_mdps);
size_t formatDiagnostics(char* output, size_t capacity);  // USB text only
```

### 2.3 What is sent

- **One scalar yaw rate**, `int32` mdeg/s, CCW positive, **bias not removed**, in bit 3.
- **Validity is only the presence bit.** Clear means "no usable measurement this tick" (`imu.h:3-4`). The wire does not distinguish "aligning", "failed" or "absent".
- **No epochs, reset counters, calibration state, chip ID, temperature or quality word are sent.**
- No roll, pitch, quaternion or accel is sent (`docs/hardware.md:50-56`).

### 2.4 BNO08X path (`imu_bno08x.cpp`)

**Transport.** SPI mode 3 at 3 MHz (`:35`) over CEVA sh2, with a non-blocking HAL.
- The only busy wait is the WAKE handshake in `halWrite`, bounded at 2 ms (`:182-191`).
- `begin()` calls `sh2_open`. That resets the hub through `halOpen`, holding reset 10 ms, "then polls up to 200 ms for its boot" (`:276-277`). This is the one blocking section, in `setup()`.

**Reports enabled** in `enableReport()` (`:237-250`):
- `SH2_GYROSCOPE_UNCALIBRATED` (0x07) every 5000 µs, 200 Hz (`kReportUs`, `:40`)
- `SH2_ACCELEROMETER` (0x01) every 10000 µs, 100 Hz (`kAccelReportUs`, `:41`)

**Sample timestamps** are the INT edge time minus the hub's reported delay, low 32 bits of `time_us_32()` (`:217`, `onSensor :218-235`).

**Decode** in `Bno08xReports::add` (`bno08x_reports.cpp:34-62`):
- XYZ int16 at bytes 4..9.
- Accel is Q8 m/s² (includes gravity); gyro is Q9 rad/s (`:48-52`).
- It rejects wrong ID, truncated reports, stale or future reports (freshness is 0 ≤ receipt − sample ≤ 100 ms, `kMaxReportAgeUs`, `bno08x_reports.h:16`, `.cpp:10-13`), and duplicate or out-of-order stamps per stream (`:42-44`).

**Gravity alignment** (`gravity_alignment.h:17-36`, `.cpp:8-14, 57-85, 87-106`):
- Needs at least 2 s (`kWindowUs = 2000000`) and at least 200 acceleration reports. Every report must satisfy:
  - |a| within 9.80665 ± 0.5 m/s²
  - |gyro| ≤ 0.10 rad/s
  - the accel vector within 0.25 m/s² of the window's first report
  - report gaps and accel/gyro skew ≤ 50 ms
- Any violation clears the partial window.
- When done, `up_ = normalize(sum of accel)`, `ready_ = true`. The axis is **frozen until `reset()`**.
- Yaw is `dot(gyro_radps, up_)`, which makes CCW positive for any fixed mounting: sideways, upside down or oblique.
- It does not correct dynamic tilt and does not remove bias (`gravity_alignment.h:31-35`).

**Per-tick averaging** (`GyroWindow`):
- Projected rates are stored as float Q9 with sample time; capacity 32 (`gyro_window.h:20`).
- `take(cut_us)` returns the mean of reports with t in (previous cut, cut_us] and consumes them (`gyro_window.cpp:36-57`).
- Conversion: `kMdpsPerLsb = 180000/(512π)` ≈ 111.9, with `lround` (`gyro_window.cpp:13-19`).
- Late and overflow reports are counted and dropped (`gyro_window.cpp:21-34`).

**Output gating** in `Bno08xReports::take` (`bno08x_reports.cpp:69-77`):
- Nothing is output unless `acknowledged()` (both feature responses with nonzero interval, `ack_mask_ == 3`), `aligned()` and `healthy(cut)` (both streams fresh within 100 ms).
- When gated, it clears the window and advances the cut, "without inventing a measurement".

**Recovery supervisor** (`bno08x_supervisor.h:11-38`, `.cpp:20-79`):
- States: `Reset` → `Boot` → `WaitAck` → `Run`.
- Timeouts:
  - boot 500 ms
  - ack 200 ms
  - stale 100 ms with no healthy gyro report while in `Run`
- Failure means `HoldReset` with backoff 0.5 s, doubling to an 8 s cap, reset to 0.5 s after 1 s of stable run.
- An `SH2_RESET` event (spontaneous hub reboot) in any state leads to `Enable`.
- `HoldReset` and `Enable` both drop samples: `g_reports.reset()` clears acks, alignment and the window (`imu_bno08x.cpp:205-215, 312-314`, `bno08x_reports.cpp:17-23`).
- **So any hub reset repeats the ≥2 s stationary alignment, and yaw is absent from the wire until it completes.**
- Lifetime counters (`g_boots`, `g_retries`, report counts, `g_last_failure`) survive recovery (`imu_bno08x.cpp:67-74`) but are **only printed on USB text** (`formatDiagnostics`, `:332-361`), never sent to the Pi.

### 2.5 ASM330 path (`imu_asm330.cpp`)

- SPI1 mode 3 at 10 MHz. It checks WHO_AM_I = 0x6B, software-resets the chip (because "the Pico can reboot while the IMU keeps running"), and configures 208 Hz ODR, ±2000 dps, accelerometer off (`:87-112`).
- `readGyroZ` returns the **newest** raw Z sample × 70 mdps/LSB, only when STATUS flags new gyro data. `cut_us` is unused (`:116-125`).
- There is no averaging, no alignment and no recovery. If `begin()` fails, `g_ready` stays false until the Pico reboots.
- Sign follows the physical Z axis. The Pi's `invert` handles mounting (`gatr2_as5047_imu.xml.in:102`).

### 2.6 Calibration behavior (end to end)

- **Pico:** BNO08X gravity-axis alignment only (automatic). There is no bias removal on the Pico for either chip.
- **Pi:** gyro bias comes from `tracking_wheel_motion`'s `<HeadingConstraint bias_samples="200" max_gap_ms="250" max_calibration_travel_m="0.005"/>` (`parallel_wheels_bno08x_localization.xml:19-20`; parser at `tracking_wheel_motion.cpp:243-268`).
  - It collects 200 samples, about 4 s at 50 Hz, and restarts if wheels travel more than 5 mm.
  - It prefers accumulated-angle ÷ elapsed time over the mean of rates (`tracking_wheel_motion.cpp:546-578`).
- **Documented startup:** about 6 s stationary and level, meaning about 2 s Pico alignment and then about 4 s Pi bias (`docs/hardware.md:44-45`, `pico/README.md:28-30`).
- **An IMU hub reset does not restart Pi bias collection** (`pico/README.md:34-36`, `parallel_wheel_bringup.md:177-180`, `calibration_inventory.md:65`).

### 2.7 Freshness bits

- **Wire (Pico to Pi):** none beyond the mask bits.
- **Brain link v3 state health** (`frames.h:120-125`):
  - `kHealthEncodersFresh`
  - `kHealthGyroFresh`
  - `kHealthVisionAlive`
  - `kHealthBiasCalibrated`

  The Pi computes them in `brain_link_publisher.cpp:206-221`. They come from the configured `<Health fresh_ms="150"><Encoder sensor_id/>…<Gyro sensor_id/><BiasCal function_id/>` (e.g. `parallel_wheels_bno08x_no_camera.xml:22-27`). "Fresh" means the sensor record is Valid and received within `fresh_ms` of now, on the host clock.
- **Pico USB text only:** states `WAIT_BOOT`, `RESET_WAIT`, `WAIT_FEATURES`, `ALIGNING`, `STREAM_STALE`, `READY` (`imu_bno08x.cpp:333-345`; described in `pico/README.md:88-106`).

---

## 3. How the Pi consumes telemetry

### 3.1 Resource `pico_telemetry` (`pico_telemetry.h/.cpp`)

XML, from `pico_telemetry.h:10-19` and every live robot file:
```xml
<Resource id="pico_uart" type="linux_serial_link">
    <Device path="/dev/ttyAMA0"/><Baud value="115200"/>
</Resource>
<Resource id="pico_telemetry" type="pico_telemetry">
    <Serial resource_id="pico_uart"/>
    <Output id="encoder_a" channel="0"/>
    <Output id="encoder_b" channel="1"/>
    <Output id="encoder_c" channel="2"/>   <!-- 3-wheel template only -->
    <Output id="imu" channel="imu"/>
</Resource>
```

- **The channel index lives on the resource's `<Output channel=…>`**, not on the sensor. `channel` is either the literal string `"imu"` (`pico_telemetry.cpp:191`) or an integer 0..2 (`:196-205`; the error text is "channel must be 0..2 or imu").
- Output ids are free-form configuration. Duplicate ids fail the build.
- Two `<Output>`s may both use `channel="imu"`; that is not rejected. There is **no output for accel** even though `accel_` is decoded (`pico_telemetry.cpp:135-137`).

Output payloads (`src/payloads/pico_telemetry_samples.h:14-31`):
```cpp
constexpr const char* kPicoEncoderCounts = "pico.encoder_counts";
constexpr const char* kPicoGyroRate      = "pico.gyro_rate";
struct PicoEncoderCounts { int32_t counts = 0; };          // absolute, wraps at 32 bits
struct PicoGyroRate {
    int32_t  rate_mdps = 0;            // latest packet
    double   accumulated_mdeg  = 0.0;  // integrated over every decoded packet
    uint64_t accumulated_epoch = 0;    // bumps on every dropped interval
};
```

**Refresh** (`pico_telemetry.cpp:32-77`):
- It drains the serial link once per cycle with a 256-byte read loop and feeds `FrameReader`. It decodes only `kFrameSensor` frames and counts decode errors in `Diagnostics.links[link_id]`.
- `applyPacket` (`:79-138`):
  - `seq` gaps go to `stats.seq_gaps`; this is diagnostic only.
  - `stamp = deviceTime(s.stamp_ms)`, a `MonotonicTime` in `ClockDomain::kDevice` (`:91`).
  - **Reboot detection:** `if (have_stamp_ && stamp < last_stamp_) { ++device_epoch_; gyro_have_prev_ = false; ++gyro_accum_epoch_; }` (`:92-98`). Detection is **solely by device-clock regression**. The `seq` reset is not used. A u32 `stamp_ms` wrap after about 49.7 days would also read as a reboot.
  - Encoder channels with their bit set get `value`, `measuredAt = stamp` and `++updates` (`:110-114`).
  - Gyro (`:115-134`): trapezoid integration `0.5*(prev+cur)*gap_ms/1000` into `gyro_accum_raw_` (mdeg) when 0 < gap ≤ `kGyroGapMs` = 250 ms (`pico_telemetry.h:70`). Otherwise it does `++gyro_accum_epoch_`, meaning "dropped interval, not zero rotation".

  Observation, not verified as an issue: the BNO08X value is the **mean over the preceding interval** (prev cut, cut], so the exact integral is `v_k * dt`. The trapezoid `0.5(v_{k-1}+v_k)dt` introduces a half-tick smear. For ASM330 (instantaneous samples) the trapezoid is appropriate.

**Publish** (`:219-271`). For each output whose channel `updates` advanced, it publishes one sample with the latest value. Several packets in one cycle collapse to one publication; encoders lose nothing because counts are absolute, and the gyro keeps its accumulator. Provenance (`:240-244`):
```cpp
publication.upstream.source   = "pico:" + telemetry->clockId();   // "pico:pico_uart"
publication.upstream.clock    = telemetry->clockId();              // "pico_uart" (the Serial resource id)
publication.upstream.sequence = telemetry->packetsDecoded();
publication.upstream.epoch    = telemetry->deviceEpoch();          // Pico restart generation
```

**Resource state:**
- `kFault` ("telemetry link dead") only if `readAvailable` reports `closed` (`:223-225`).
- Otherwise `kValid` once any packet has arrived, else `kNoDataYet`.
- A hardware UART (`/dev/ttyAMA0`) does not close when the Pico reboots, so **a Pico reboot is not a link fault**. It shows up as silence and then a clock regression.

**`reset()`** (`:16-30`) clears the decoder, channels and gyro accumulator, including `gyro_accum_epoch_ = 0`, but **not `device_epoch_`**.

### 3.2 Sensors `pico_encoder_channel` and `pico_imu_channel` (`pico_channels.cpp`)

XML, from `pico_channels.h:7-16` and `docs/navigatr_sensors.md`:
```xml
<Sensor id="tracking_encoder_a" type="pico_encoder_channel">
    <Source resource_id="pico_telemetry" output_id="encoder_a"/>
    <Calibration counts_per_revolution="4000" invert="false"/>
    <Freshness stale_after_ms="250"/>   <!-- optional, default 250, 0 disables -->
</Sensor>
<Sensor id="robot_imu" type="pico_imu_channel">
    <Source resource_id="pico_telemetry" output_id="imu"/>
    <Calibration invert="false"/>
</Sensor>
```

- **There is no `channel` attribute on `pico_encoder_channel`.** A sensor binds by `(resource_id, output_id)` (`pico_channels.cpp:38-62`).
- `counts_per_revolution` must be > 0; `invert` defaults to false (`:144-154`). They are folded into `radians_per_count = (2π/cpr)·(invert?-1:1)` at build time (`:155`) and captured in the closure state. No runtime setter exists.

Shared health logic, `ChannelInput::fresh` (`:68-109`):
- A new sequence or epoch means the sample is consumed.
- **Restart** is flagged when the record epoch or `upstream.epoch` (the Pico `device_epoch_`) changed since the last consumed sample (`:80-81`).
- Otherwise the output's Fault or Unavailable state propagates, then `NoDataYet`, then **Unavailable after `stale_after_ms` of silence** (`:102-105`).
- Published provenance: `upstream.source = "<resource>.<output>"` (for example `pico_telemetry.imu`), clock carried through (`"pico_uart"`), `upstream.epoch = stored.epoch + stored.upstream.epoch` (`:112-125`).

`pico_encoder_channel` produces `EncoderSample { double angle_rad; uint64_t discontinuity_epoch; }` (`payloads/sensor_samples.h:24-27`):
- On restart it does `discontinuity += 1`, rebases `prev_counts` and keeps `angle_rad` continuous (`:188-192`).
- Otherwise `angle += int32(uint32(counts) - uint32(prev)) * radians_per_count`, which is wrap-safe (`:193-197`).

`pico_imu_channel` produces `ImuSample` (`sensor_samples.h:29-42`):
- `yaw_rate_rad_s = sign·degToRad(rate_mdps/1000)`
- `accumulated_angle_rad = sign·degToRad(accumulated_mdeg/1000)`
- `accumulated_epoch = raw.accumulated_epoch + (restarts << 32)`
- `has_accumulated = true`

On restart it does `restarts += 1` (`:254-264`).

### 3.3 Wheel geometry (Pi-side; relevant to RobotProfile)

`wheel_geometry` resource, from `src/resources/wheel_geometry.h:19-28`:
```cpp
struct WheelDecl { std::string id; SensorId sensor; std::string label;
    double radius_m, position_x_m, position_y_m, measurement_angle_deg; bool direction_positive; };
```

XML: `<Wheel id sensor_id label radius_m position_x_m position_y_m measurement_angle_deg direction="positive|negative" calibration_status=.../>`. `calibration_status` is a reader note only.

Profile selection is a localization-fragment concern:
- `<Wheels resource_id><Use wheel_id/>…`
- `<LateralMotion assume="zero"/>` for the 2-parallel case (`tracking_wheel_motion.cpp:270-297`)
- `<Timing interval_tolerance_ms="20" max_pending_ms="500"/>` (`:299-306`)

Existing profiles:
- **2 parallel + BNO08X:** `gatr2_parallel_wheels_bno08x.xml.in`, channels 0 and 1, `LateralMotion assume="zero"`.
- **3-wheel:** `gatr2_as5047_imu.xml.in` with `three_wheel_imu_no_correction.xml`. Channels 0, 1, 2 as left/right/rear. Its header comment still describes the ASM330.
- **Forward + sideways (2 perpendicular):** USB bench, `bench_vex_imu_usb.xml`. Channel 0 at 0°, channel 1 at 90°, using `brain_imu_planar_bench`.

All of these are fixed at configuration and build time.

---

## 4. "IMU port" today

**No logical IMU index exists anywhere in the Pico or Pi code.**
- **Pico:** one IMU, one SPI bus (SPI1/J7), chip fixed at compile time. `imu::` has no index parameter (`imu.h`).
- **Wire:** a single `kSensorGyroZ` bit (`frames.h:41`). There is no room for a second IMU without a new bit or frame and a codec change. Old parsers reject unknown bits (`docs/interfaces.md:223-225`).
- **Pi:** the only IMU selector is the string literal `channel="imu"` on a `pico_telemetry` `<Output>` (`pico_telemetry.cpp:191`). Encoder channels are numeric 0..2; the IMU is not numbered.
- **Brain (bench):** `constexpr uint8_t kVexImuPort = 1;` in `brain/localization-test/include/robot_config.h:20`. That is a **1-based V5 Smart Port**, `static_assert` 1..21 and ≠ `kNavigatrPort` (`:47`); `kNavigatrPort = 4` (`:14`). The Pi-side `brain_imu_bench` resource has **no port attribute** (`bench_vex_imu*.xml`: `<Resource id="brain_imu" type="brain_imu_bench"/>`). Brain IMU data arrives via op `kOpGetStateWithImu = 5` with body `flags u8 (bit0 kBenchImuValid) | stamp_ms u32 (Brain clock) | rotation_mdeg i32 (CCW continuous)` (`frames.h:87-92, 146-148`). Its epoch bumps on session change, on valid→invalid, or on Brain stamp regression (`src/resources/brain_imu_bench.h:16-36`).
- **"Port N" wording precedent:** the USB bench docs and XML call Pico **encoder channels** "port 0" and "port 1" (`usb_localization_bench.md:5, 68-69`; `bench_vex_imu_usb.xml:5, 32-33`). So "encoder logical port k" = Pico channel k = SensorBit k = J2/J3/J4 already has precedent.

**What "IMU port 0" would map to (speculation, not code):** the only Pico-side IMU is the single SPI1/J7 device. A logical "IMU port 0" would naturally mean the Pico's sole IMU, which is `pico_telemetry` `channel="imu"` / `kSensorGyroZ`. Whether that is a BNO08X or ASM330 is decided by which firmware was flashed and cannot be seen from the Pi. A Brain VEX IMU would be a separate source (Brain Smart Port 1..21, 1-based) delivered over the brain link, not a Pico port. Mixing 0-based logical IMU ports with 1-based VEX Smart Ports would need an explicit convention in the contract.

---

## 5. What a Pico or sensor restart looks like, and how localization reacts

### 5.1 On the wire

**Pico reboot:**
- UART frames stop through `setup()`. BNO08X `sh2_open` blocks up to about 210 ms (10 ms reset hold plus up to 200 ms boot poll, `imu_bno08x.cpp:276-277`). ASM330 reset polls up to 100 ms (`imu_asm330.cpp:104-106`). I have not measured the real gap.
- Frames then resume with:
  - `seq` from 0
  - `stamp_ms` near 0
  - all three encoder bits set with counts at 0
  - gyro bit **clear for ≥2 s plus ack** on the BNO08X (alignment; the robot must be still and level), or present immediately on the ASM330
- `docs/interfaces.md:156-158`: frames carry no cross-frame state and resync on the next sync pair.

**BNO08X hub reset only (Pico keeps running):**
- Encoders and frames continue uninterrupted (`docs/hardware.md:18`).
- The gyro bit goes clear during recovery and realignment: at least 2 s, plus 0.5 to 8 s backoff if bring-up fails.
- **Nothing on the wire says a reset happened.** The Pi sees only a gyro gap. There is no IMU epoch or reset counter on the wire.

**ASM330 failure:** the gyro bit stays clear permanently until the Pico reboots. There is no in-firmware recovery.

### 5.2 Pi reaction to a Pico reboot (traced)

1. **`pico_telemetry`:** the first post-boot packet has `stamp < last_stamp_`. That bumps `device_epoch_` and `gyro_accum_epoch_` and reseeds the gyro integrator (`pico_telemetry.cpp:92-98`). The resource stays `kValid`; it is not faulted.
2. **Encoder sensors:** `upstream.epoch` changed, so `restart`. That bumps `discontinuity_epoch`, rebases counts and keeps the angle continuous (`pico_channels.cpp:188-192`).
3. **IMU sensor:** Unavailable after 250 ms of no gyro packets (`stale_after_ms`). When the gyro returns, `restart` bumps `restarts`, and `accumulated_epoch` jumps by `1<<32`.
4. **`tracking_wheel_motion`** (the sensor-record `stored->epoch` does not change; only payload epochs do):
   - Encoders: `discontinuity_epoch` changed, so `dropWindow("encoder discontinuity")`, rebase, and `restartCalibration()` if bias collection was still running (`tracking_wheel_motion.cpp:481-492`).
   - With `HeadingConstraint`, `awaiting_baselines_` waits for a **heading** baseline too (`:640-649`). With the BNO08X, **odometry stalls until alignment finishes** (≥2 s still and level). If the robot is moving, alignment keeps restarting and no motion is produced.
   - When the gyro returns, its new-clock stamp is less than the old `prev_stamp`, so `dropWindow("nonpositive gyro interval")` and baseline (`:582-590`). Other drop paths are "gyro gap" when dt > `max_gap_ms` (`:591-596`) and "gyro accumulator discontinuity" (`:597-601`).
   - **Gyro bias is not re-estimated.** `calibrated` stays true; only `reset()` clears it (`:389-414`). The old bias is reused even though the BNO08X re-learned its up axis.
5. **`planar_motion_integrator` / `motion_step.cpp:460-477`:** the next motion increment's device stamp is less than `r.measuredAt`. The result: "source time regression; odometry epoch advanced". It does `r.odometry_epoch += 1`, zeroes velocities, resets `DeviceToHostClock` (it needs 8 observations again to be valid, `clock_sync.h:72-73`) and resets the attitude and heading aligner. It also rejects that increment.
   - Pose x/y/θ and the field anchor are not touched in that block, so the pose is held across the gap and motion during the gap is lost.
   - `measuredAtHost` is cleared, so `kRobotAgeKnown` drops until the clock refits.
   - Observed landmark estimates tagged with the old `odometry_epoch` stop being reported in GET_STATE (`brain_link_publisher.cpp:241-243`; `landmark_estimator.cpp:68,110`).
   - The Brain sees the new `odometry_epoch` in `BrainState` (`frames.h:158`).
6. The Brain health bits `kHealthEncodersFresh` and `kHealthGyroFresh` drop during the outage (150 ms `fresh_ms`).

**BNO08X hub reset only:** steps 1 and 2 do not fire. The gyro gap (>250 ms) bumps `gyro_accum_epoch_` in `pico_telemetry` (`:127`). The IMU sensor goes Unavailable. `tracking_wheel_motion` wheels accumulate pending travel with no heading, then `dropWindow("sources misaligned longer than max_pending_ms")` after 500 ms (`:683-687`). When the gyro returns, "gyro gap" drops and rebases. No odometry-epoch bump (same device clock). Bias is not recollected.

---

## 6. Explicitly absent

- A Pi-to-Pico command or config path. The Pico never reads UART RX.
- A runtime IMU chip selection or IMU identity on the wire.
- A sensor-frame version byte.
- IMU validity, quality, calibration-state or reset-epoch fields on the wire. There is only the presence bit.
- Encoder connection detection. All three channels are always reported.
- Accel, roll/pitch or quaternion on the wire. Bit 4 is reserved and never emitted, and there is no Pi output for it.
- Any logical IMU port or index on the Pico or Pi. The VEX Smart Port number exists only as a Brain compile-time constant in `brain/localization-test`.
- A HAT v3/v4 pin map. Only `GATR2_HAT_REV == 2` compiles.
- Runtime reconfiguration of CPR, invert, wheel geometry or IMU source on the Pi. In the files I read, all of these are parsed once in factory functions and captured in state. Whether a system-level rebuild or reset path exists belongs to another area of the inventory.
- Hardware validation. `parallel_wheel_bringup.md:27-30`: "Pico firmware `hat2_bno08x` | Builds. Not yet run on the robot… Nothing here has been validated on hardware."