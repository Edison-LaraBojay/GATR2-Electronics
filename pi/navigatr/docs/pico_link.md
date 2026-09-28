# Pico link (Pi side)

The `pico_telemetry` resource reads the Pico UART, decodes sensor and status
frames, publishes the encoder and gyro channels, and sends Pico commands on
the same link. The wire layouts are in [docs/interfaces.md](../../../docs/interfaces.md)
and the firmware side is in [pico/README.md](../../../pico/README.md).

```xml
<Resource id="pico_uart" type="linux_serial_link">
    <Device path="/dev/ttyAMA0" required="true"/>
    <Baud value="115200"/>
</Resource>
<Resource id="pico_telemetry" type="pico_telemetry">
    <Serial resource_id="pico_uart"/>
    <Output id="encoder_0" channel="0"/>
    <Output id="encoder_1" channel="1"/>
    <Output id="encoder_2" channel="2"/>
    <Output id="imu_0" channel="imu"/>
</Resource>
```

Consumers `require<PicoTelemetry>` by resource id and use it only through the
`PicoControl` contract (`src/resources/pico_control.h`).

## Identity

v2 sensor frames (type 0x04) carry `boot_id` (random, nonzero per Pico boot),
`acq_epoch` and `imu_epoch`. The resource keeps two restart generations and
uses each as the upstream epoch of its outputs:

| Event on the wire | Encoder epoch | IMU epoch | Counted in `link()` |
|---|---|---|---|
| new `boot_id` | +1 | +1 | `reboots` |
| frame version changed (v1 and v2 firmware swapped) | +1 | +1 | `reboots` |
| device clock ran backwards | +1 | +1 | `reboots` |
| new `acq_epoch`, same boot | +1 | | `restarts` |
| new `imu_epoch`, same boot | | +1 | `imu_restarts` |
| link closed and reopened, same identity | | | nothing |

- A changed upstream epoch makes the channel sensors restart: an encoder
  sensor rebases its count and keeps its angle continuous under a new
  discontinuity epoch; the IMU sensor moves its accumulator epoch.
  Localization rebases on either, so counts returning to zero never read as
  movement.
- An IMU restart or reboot also reseeds the gyro accumulator, so no rotation
  is integrated across it. Localization restarts gyro bias calibration on an
  IMU restart; the robot must be still for it to finish.
- With a Brain profile applied, a restart of a source the profile uses also
  invalidates the pose until the robot is placed again; see
  [Reboot, restart and IMU reinitialization](#reboot-restart-and-imu-reinitialization).
- A clock regression with an unchanged `boot_id` is a reboot whose random id
  repeated (about 1 in 65535 reboots).

## v1 fallback

Firmware that sends v1 frames (type 0x01) has no identity:
- a reboot is found only by the device clock running backwards;
- `link().identity` is false and the resource diagnostic reads
  `v1 sensor frames: no Pico identity, ...`;
- commands are refused (`submit` returns 0), so CONTROL actions 3 and 4 fail
  with detail PicoLink.

## Commands

| Op | Body | Pico effect | Settles |
|---|---|---|---|
| CONFIGURE (1) | imu_enabled | enable or disable IMU port 0 | Completed at once |
| REINIT_IMU (2) | imu_port (0) | fresh IMU attempts, `imu_epoch` + 1 | Completed when the IMU is ready; Failed ImuAbsent after the Pico's attempts |
| RESTART_ACQUISITION (3) | none | encoder counters zeroed, `acq_epoch` + 1 | Completed at once |

- **Request ids** start at a random base per Pi process and count up, never
  0, so a restarted Pi does not repeat ids the Pico recorded. Every command
  targets the boot it was submitted for.
- **Resend.** A command is written from the resource refresh and resent every
  200 ms under the same id until a status frame of its boot names it. At most
  one command frame goes out per 100 ms. A running command that the newest
  status already names is not resent; the Pico reports it when it settles.
- **Duplicates.** The Pico records its last 4 commands by (id, op, body) and
  never runs a resend again, so a lost report cannot restart acquisition or
  the IMU twice.
- **Completion** comes only from status frames. The gyro returning is not
  completion; neither is a sensor frame with a new epoch.
- **Bound.** Each command has a timeout. Unreported at the timeout: Failed,
  PicoLink. Reported running at the timeout: Failed, TimedOut. Nothing is
  resent after that.
- **Reboot.** A new boot fails every unsettled command of the old boot with
  PicoLink; none is sent to the new boot.
- `submit` returns 0 without v2 identity, without a frame in the last 250 ms,
  for an unknown op, or for a nonpositive timeout. At most 16 commands are
  kept; settled ones go first.

Pico failure details map to CONTROL details:

| Pico detail | CONTROL detail |
|---|---|
| ImuAbsent | ImuAbsent |
| WrongTarget | PicoLink |
| UnknownOp, BadBody, NoSuchPort, ImuDisabled | PicoRefused |

A RESTART_ACQUISITION can be reported Completed before the first sensor
frame with the new `acq_epoch` arrives; wait for `link().acq_epoch` (or
`restarts`) to change before treating the counters as restarted.

## Status

`link()` is safe from any thread (inspection reads it):
- `frames_fresh`: a sensor frame within the last 250 ms on the host clock and
  the serial link open;
- `identity`, `boot_id`, `acq_epoch`, `imu_epoch` from the newest sensor
  frame, and the `reboots`, `restarts` and `imu_restarts` counters;
- `status_known` and `status`: the newest status frame of the current boot
  (IMU state, reason, attempts, enabled flag, last command, firmware);
  cleared by a reboot until the new boot reports;
- `last_frame` and `last_status` on the host clock.

The Pico sends a status frame every 200 ms and shortly after each command or
IMU state change.

## Serial reopen

`linux_serial_link` keeps its device usable:
- a device that failed to open, or that reports closed (a read error, or a
  hangup seen by poll), is reopened at most once a second, from the reading
  side, without waiting; meanwhile reads report closed and writes fail;
- `<Device required="true"/>` still makes the first open at startup a hard
  error; afterwards the link retries like any other;
- with `<DriverEnable>`, each reopen sets the driver enable up again (driven
  low while listening). A freshly exported GPIO can take up to 200 ms to
  become writable; that wait happens only when the pin was not exported.

While the link is closed the telemetry resource reports a fault, `link()`
shows the frames stale, and commands are refused. When the same Pico comes
back its identity is unchanged, so nothing is rebased: the counts it kept
while the cable was out are still valid measurements. With a Brain profile,
a gap longer than `sensor_loss_ms` still invalidates the pose (next section).

## Reboot, restart and IMU reinitialization

Localization rebases the encoders and restarts IMU bias calibration on its
own. The pose depends on the configuration:
- **Brain profile** (`<BrainProfile>`, spec 8.10): a source the profile
  uses (its encoder ports; the Pico IMU only when the IMU source is pico)
  that restarts, stays stale longer than `sensor_loss_ms` (250 ms default),
  or, for a used Pico IMU, reports anything but ready ends pose continuity.
  The Pi clears Localized, advances the odometry epoch, withdraws pending
  SET_POSE requests and logs `sensor lost: <source>: place again`; the
  Brain shows Needs placement. The encoders still rebase, so nothing is
  integrated as movement. Sources the profile does not use never matter.
- `<BrainProfile><Timing on_sensor_loss="warn"/>` only logs the loss and
  keeps the pose.
- **XML-configured localization** (no `<BrainProfile>`): no sensor-loss
  rule; the pose holds. CONTROL is refused without a profile, so the Pi
  sends no REINIT_IMU or RESTART_ACQUISITION.

| What happened | Encoders | IMU | Pose, Brain profile | Pose, XML config |
|---|---|---|---|---|
| Pico reboot | rebased, new discontinuity | restarted, bias recalibrates | invalid: place again | holds; travel during the reboot is not measured |
| RESTART_ACQUISITION (CONTROL 4) | rebased, new discontinuity | unchanged | invalid: place again | not sent |
| REINIT_IMU (CONTROL 3, Pico IMU profiles only) | continuous | restarted, bias recalibrates | invalid: place again | not sent |
| Local Pico IMU retry | continuous | restarted, bias recalibrates | invalid with the pico IMU source; holds with brain_vex or none | holds |
| Cable out and back, same Pico | continuous, no rebase | continuous (gyro gap only) | invalid when frames stopped longer than `sensor_loss_ms`; holds for a shorter gap | no reset from the Pico link; the gap follows the model's own rules |

Placement procedure and the Brain side of recovery:
[docs/brain_setup.md, section 8](../../../docs/brain_setup.md#8-sensor-loss-and-recovery)
and [section 6](../../../docs/brain_setup.md#6-placement).

## Not verified on hardware

- The Pi TX to Pico RX wire on HAT v2 (Pi UART TX to Pico GP17) has not been
  checked on hardware. Before this work the link was Pico to Pi only.
- Command receipt, status frame timing and command resends on the real
  UART; everything here is tested with memory links and the common codec.
- Reopening a real `/dev/ttyAMA0` or RS-485 device. The pseudo terminal
  test in `tests/pico_link_gtest.cpp` runs only on Linux hosts (CI), not on
  the Windows build machine.
- Update the Pi software before flashing v2 Pico firmware: older Pi software
  ignores type 0x04 frames and would see no encoder data.

## Tests

`tests/pico_link_gtest.cpp`: identity epochs and the v1 fallback; a v2 and a
v1 Pico reboot, an acquisition restart and an IMU restart through the real
telemetry, sensor and localization path (an XML-configured three-wheel
pipeline) with no displacement; a pulled and
replaced cable; command ids, targets, resend timing, lost reports, running
and failed commands, timeouts, reboots and two commands in flight against a
fake Pico built from the common codec; thread-safe reads; serial reopening.
