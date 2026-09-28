# Brain robot profiles

The primary Pi configurations take the robot description from the Brain. The
Brain program sends a typed robot profile (tracking wheels, encoders, IMU
source, footprint) each time it connects; the Pi checks it against its own
hardware and builds the matching localization models. Changing the robot's
geometry or its localization setup is a Brain edit and upload, not a Pi XML
edit.

- Pi configs: [brain_profile_usb.xml](../config/override/brain_profile_usb.xml)
  and [brain_profile_rs485.xml](../config/override/brain_profile_rs485.xml).
- Brain side: [brain/robot/gatr2_robot.h](../../../brain/robot/gatr2_robot.h),
  procedures in [Brain setup](../../../docs/brain_setup.md).
- Wire format: [Brain link v4](../../../docs/interfaces.md) and
  `common/link_documents.h`.

The XML-configured profiles (`bench_vex_imu*.xml`, `parallel_wheels_bno08x*`,
the three-wheel templates, the synthetic demos) still run. They describe the
robot in Pi XML and ignore any profile a Brain sends (PROFILE_APPLY answers
ProfileRejected, reason "not accepted").

Nothing here has been validated on the robot. The tests run the real Pi
pipeline over memory links with synthetic Pico frames and Brain requests.

## Who owns what

| Owner | Settings |
|---|---|
| Brain (profile, C++) | encoder port per wheel, counts per revolution, polarity, gearing, radius, mounting position, measuring direction, travel scale; two-wheel, two-forward-wheel or three-wheel topology; IMU source (none, Pico IMU port, Brain VEX IMU) and VEX Smart Port; footprint; IMU calibration settings (window, still rate, still travel; 0 keeps the Pi default) |
| Pi (XML) | Pico and Brain link devices, which encoder ports and Pico IMU port are wired, the Brain IMU mailbox, model tuning (timing, fusion noise, history), calibration defaults, the sensor loss limit, the field, cameras, inspection |
| Pico (firmware build) | board revision, GPIO mapping, which IMU chip (BNO08X or ASM330), sensor bring-up and retries |

Choosing the Pico IMU in the profile selects Pico IMU port 0; it never changes
the chip the Pico firmware was built for.

**Transport exception.** USB versus RS-485 is Pi configuration: the Pi must
run `brain_profile_usb.xml` when the Brain uses USB (`kUseUsb = true` in
`gatr2_robot.h`) and `brain_profile_rs485.xml` when it uses a Smart Port. The
two files differ only in the Brain link resource (a test pins that), so
localization is the same over either. There is no automatic transport
detection.

## The configs

```xml
<Localization>
    <BrainProfile>
        <Encoders resource_id="pico_telemetry" stale_after_ms="250">
            <Port index="0" output_id="encoder_0"/>      HAT v2: 0 J2, 1 J3, 2 J4
            <Port index="1" output_id="encoder_1"/>
            <Port index="2" output_id="encoder_2"/>
        </Encoders>
        <Imu port="0" resource_id="pico_telemetry" output_id="imu_0" stale_after_ms="250"/>
        <BrainImu resource_id="brain_imu"/>
        <Calibration bias_samples="20" window_ms="2000" max_gap_ms="250"
                     still_travel_m="0.001" still_rate_dps="1" max_rate_dps="5"
                     evidence_gap_ms="100" attempt_s="60"/>
        <Timing interval_tolerance_ms="20" max_pending_ms="500" sensor_loss_ms="250"
                on_sensor_loss="unplace"/>
        <Fusion max_wait_ms="100">                        three wheels with the Pico IMU
            <MotionNoise .../> <HeadingNoise .../>
        </Fusion>
        <History retention_s="5" capacity="1024" max_interpolation_gap_ms="150"/>
    </BrainProfile>
</Localization>
```

- A Brain-profiled `Localization` holds only `BrainProfile`. The element is
  strict: unknown children and attributes fail the build.
- `Encoders` lists the wired ports; `Imu` is the Pico IMU port the Pi reads;
  `BrainImu` names the `brain_imu_bench` mailbox, which must also be the
  CommandCollection's `<BenchImu>`.
- `Calibration` and `Timing` are the Pi defaults. A profile's calibration
  window, still rate and still travel replace `window_ms`, `still_rate_dps`
  and `still_travel_m` when they are not 0. `max_gap_ms` is the longest gyro
  gap integrated; `sensor_loss_ms` is how long a used source may stay quiet,
  and also the longest step of the VEX IMU bench model (see Sensor loss).
- The noise values in `Fusion` are PLACEHOLDER assumptions, used only by three
  wheels with the Pico IMU.
- `<Pico resource_id="pico_telemetry"/>` on the CommandCollection enables
  CONTROL 3 and 4; on Publishing it adds the Pico health bits.
- Both configs also have `pico_telemetry` with all three encoder outputs and
  the IMU output, noop world estimation, and `<Field>` publishing of
  `field.xml`. A camera variant does not exist: a profile with camera mounts is
  refused (see below).

## Lifecycle

**Waiting.** Until a profile is applied, localization is noop: no pose,
`profile_state` none, and SET_POSE, CONTROL and READ_WHEELS answer NotReady.
The Pico link, the Brain link, the field documents and inspection run; raw
encoder counts stay visible in inspection's sources.

**Apply.**
1. PROFILE_WRITE stages the document in chunks, keyed by its id (the crc32 of
   the bytes). Staging survives a new Brain session; a resend of identical
   bytes is Ok.
2. PROFILE_APPLY runs the shared validation (`validateRobotProfile`), then this
   Pi's capability checks, then builds the candidate sensors and models from
   fixed Pi templates through the ordinary factories. Nothing running is
   touched; a failure answers ProfileRejected with its reason and is
   remembered for that id.
3. The reply is Pending. The swap happens at the next controlled boundary (the
   main thread every 20 ms, or the top of an inline step). The next APPLY of
   the same id answers Ok.

**Continuity.**

| Case | Result |
|---|---|
| A new profile (another id) | odometry epoch + 1, history cleared, robot unplaced, every earlier SET_POSE withdrawn; the anchor revision continues. A new placement is needed. |
| The running profile again (a Brain restart with an unchanged profile) | nothing resets |
| A refused profile | the running one keeps running; the state reports the refused id and reason |
| Pi restart | waits for a profile again, new `pi_instance`, unplaced |
| `System::reset()` | waits for a profile again |

A later APPLY wins over a candidate still waiting for the boundary: if the
Brain re-applies the running profile, or has another refused, before the
boundary runs, the waiting candidate is dropped.

## Supported setups

| Topology | IMU source | Pi models |
|---|---|---|
| two wheels | Pico IMU | `tracking_wheel_motion` with a HeadingConstraint, `planar_motion_integrator`; the gyro is counted once |
| two wheels | Brain VEX IMU | `brain_imu_planar_bench`, `planar_motion_integrator` |
| two forward wheels | Pico IMU | `tracking_wheel_motion` with a HeadingConstraint and `<LateralMotion assume="zero"/>`, `planar_motion_integrator` |
| two forward wheels | Brain VEX IMU | `brain_imu_parallel_bench` (a 180 degree wheel counts as reversed), `planar_motion_integrator` |
| three wheels | none | `tracking_wheel_motion`, `planar_motion_integrator` |
| three wheels | Pico IMU | `tracking_wheel_motion` without a constraint, `imu_heading_increment`, `weighted_planar_fusion`: wheel rotation and an independent gyro fused once |

The Brain VEX IMU models pair wheel and IMU samples by Pi arrival time. They
are bench models and do not synchronize the Brain and Pico clocks.

**Refusals** (ProfileRejected, reason and detail):

| Reason | When |
|---|---|
| format | not a profile document |
| topology, wheel count | unknown topology, or not 2 / 2 / 3 wheels |
| encoder port | a port used twice, or not wired in `<Encoders>` (detail: the wheel) |
| wheel geometry | counts, radius, offsets or angle out of range, gearing outside 0.1..10, travel scale outside 0.9..1.1 |
| observability | two wheels whose directions are nearly parallel, forward wheels not at 0 or 180 degrees, three wheels that cannot resolve planar motion |
| IMU source | Pico IMU without `<Imu>`, Brain VEX IMU without `<BrainImu>`, unknown flags |
| IMU port | a Pico IMU port this Pi does not have, or a VEX Smart Port outside 1..21 |
| IMU combination | two wheels without an IMU, three wheels with the VEX IMU (Brain clock), three wheels with the Pico IMU without `<Fusion>` |
| footprint | a side out of range, or zero length or width |
| calibration | window outside 0.5..20 s, still rate outside 0.1..20 deg/s, still travel outside 0.02..5 mm |
| camera | any camera mount: the Pi does not apply camera mounts from a profile yet |
| build | the Pi could not build the models; the reason is in the Pi log and events |

## Corrections, each applied once

```text
pico_telemetry          raw counts, Pico identity
  -> encoder sensor     wheel angle = counts * 2 pi / (counts_per_rev * gear), sign by polarity
  -> observation model  travel = wheel angle change * radius * travel_scale, along the measuring direction
  -> estimator          pose
```

- The profile's reversed flag is encoder polarity, applied in the sensor. The
  model's geometric direction is always positive for profile-built models.
- The Pico IMU sign (`kImuInvert`) is applied in its sensor, the gyro bias once
  in the model that owns the calibration.
- The Brain VEX IMU rotation arrives counterclockwise and already calibrated
  by the VEX firmware; the Pi adds no bias to it.
- `travel_scale` is one measured distance correction per wheel. A push test
  observes the product of radius, gearing, tire compression and slip; it cannot
  separate them, and a constant scale fixes none of slip, missed counts,
  load-dependent compression or angle-dependent magnet error. The procedure is
  in [Brain setup](../../../docs/brain_setup.md#4-per-wheel-calibration).

**READ_WHEELS** answers one record per profile wheel, in profile order: port,
fresh (received within 150 ms) and valid flags, the discontinuity counter (low
16 bits of the encoder restart count plus its record epoch), the raw counts
last received, travel with counts per revolution, gearing, polarity and radius
applied but never the travel scale, and the age. The Brain's wheel calibration
reads it. NotReady before a profile is applied.

## Calibration and stationary handling

One `StationaryWindow` serves every IMU bias path and the stationary status:

- Only new samples count: a repeated or retained record is no evidence.
- Every source the model uses must keep delivering: the profile encoders plus
  the Pico gyro or the VEX rotation. A gap over `evidence_gap_ms` (100), a
  source quiet that long, an out-of-order sample, or a source restart
  (record epoch, encoder discontinuity, source epoch) restarts the window as
  waiting for data.
- Movement restarts it as waiting for stillness: a wheel moving more than
  `still_travel_m` (1 mm), a gyro rate above `max_rate_dps` (5) or further
  than `still_rate_dps` (1) from the window mean, a VEX rotation change above
  the still rate times the window.
- It qualifies on elapsed sample time: every source spans `window_ms` (2 s)
  with at least `bias_samples` (20) samples. It is time, not a sample count;
  `window_ms` must be positive, and XML models without it use 2 s as well.

The gyro bias (Pico IMU profiles):

- The first qualified window after a start gives the bias: the accumulated
  angle over the elapsed time.
- No qualified window within `attempt_s` (60 s) fails the calibration; CONTROL
  recalibrate retries. Nothing blocks acquisition or the Brain link meanwhile.
- Later qualified windows maintain the bias: 0.2 of the difference, at most
  0.2 deg/s per window. A window that restarts contributes nothing.
- A gyro restart, IMU epoch change or Pico reboot invalidates the bias and
  starts a new attempt.
- Two wheels: the HeadingConstraint in `tracking_wheel_motion` owns it.
  Neither the wheels nor the gyro integrate while it calibrates, so a placed
  robot must stay still until calibration is done: movement the window sees,
  or wheel travel over `still_travel_m` in all, ends pose continuity (motion
  lost, below). The sample that completes the window starts integration for
  the wheels and the gyro alike. Three wheels: `imu_heading_increment` owns
  it, with the wheels as stillness evidence; the wheels keep measuring
  rotation meanwhile, so nothing is lost.

The state block's `calibration` is none, collecting, done, waiting for
stillness, waiting for data or failed.

**VEX IMU profiles** have no Pi calibration (`calibration` none): the Brain
owns the VEX calibration (`pros::Imu::reset()`), so it is never applied twice.
Stationary detection there uses the wheels and the VEX rotation only; the VEX
IMU gives no rate or acceleration. An absent or failed BNO08X never blocks a
VEX profile.

**The BNO08X gravity alignment** runs on the Pico at every IMU start (about 2 s
still and level). It finds the up axis, which makes yaw correct for any fixed
mounting. It is not dynamic tilt compensation and gives neither the mounting
yaw nor the field heading; the field heading comes only from placement.

**Stationary handling.** While a window stays qualified, the Pi reports zero
velocity and health bit 7 (stationary). The pose is not changed. This is gated
stationary handling, not a ZUPT filter: no estimator here models velocity
with uncertainty, and nothing recovers position error from before the stop.
Movement above the thresholds ends it at once.

**Limits.**
- A quadrature encoder whose cable is pulled keeps reporting its last count,
  which reads as standing still. That cannot be detected.
- The thresholds are defaults, not measured against the real sensors' noise.

## CONTROL

| Action | What the Pi does |
|---|---|
| 1 recalibrate | Stationary precheck, then the running model restarts its bias calibration. The pose holds while the robot stays still; moving before calibration is done needs a placement (two-wheel Pico IMU profiles). VEX profiles: Ok with nothing to do on the Pi. |
| 2 reinitialize | Stationary precheck, then a localization reset: odometry epoch + 1, unplaced, placements withdrawn, bias recalibrates. |
| 3 reinitialize the Pico IMU | Needs a Pico IMU profile (else Failed, IMU unused) and Pico firmware with identity (else Failed, Pico link). Pending while the Pico works on it, Ok once it reports completion; the bias then recalibrates. |
| 4 restart acquisition | Stationary precheck, then the Pico zeroes its counters under a new acquisition epoch. Pending until frames of the new epoch arrive, then Ok. |

- The stationary precheck: over the last 300 ms every profile wheel moved less
  than 1 mm, a used Pico gyro stayed below 2 deg/s, or the VEX rotation changed
  less than 0.6 degree (2 deg/s over the window); every source fresh,
  continuous and covering the window. Otherwise NotStationary and nothing
  starts.
- CONTROL is deduplicated like SET_POSE: a resend with the same request id
  reports the current progress and never runs or resubmits anything.
- 3 and 4 are bounded at 15 s (Failed, timed out). A Pico failure answers
  Failed with its detail.
- Completing 3 or 4 restarts a source the profile uses, so the pose needs a
  placement afterwards (sensor loss, below).

## Sensor loss and recovery

The rule: if a source the running profile uses stops delivering, the pose is
invalid until the operator places the robot again. Sources the profile does
not use never matter.

| Used source | Lost when |
|---|---|
| profile encoders | stale longer than `sensor_loss_ms` (250), missing from Pico frames, or restarted (Pico reboot, acquisition restart, discontinuity) |
| Pico IMU (Pico IMU profiles) | stale, restarted or reinitialized, or (firmware that sends status) reporting anything but ready |
| Brain VEX IMU (VEX profiles) | no valid sample for `sensor_loss_ms`; an invalid sample (VEX calibration) counts as missing, a new mailbox epoch (new Brain session) as a restart |

A loss while placed: odometry epoch + 1, unplaced (the pose is shown where it
was), earlier placements withdrawn, and an event "sensor lost: <source> ...:
place again". A placement made while the source is still lost does not hold.
`on_sensor_loss="warn"` keeps the pose and only logs.

**Motion lost.** The loss check runs at the start of each cycle, so a gap
that ends between two checks is never seen stale. The models close that gap:
whenever one discards measured motion after it had a baseline, its
`dropped_intervals` rises, and a rise while placed ends continuity the same
way, with the event "motion lost: <function>: <why>: place again". A
placement sent in that very cycle is withdrawn with it. The models discard
motion on:

- a step longer than their limit. The VEX IMU bench model's limit is
  `sensor_loss_ms` (the builder writes it), for the VEX samples and the wheel
  samples alike; it also stops when an encoder sensor goes stale (`Encoders
  stale_after_ms`, 250). The Pico gyro model integrates gaps up to
  `Calibration max_gap_ms` (250), and the Pico link cuts its gyro accumulator
  at 250 ms. A limit shorter than `sensor_loss_ms` therefore ends continuity
  first: raising `sensor_loss_ms` alone never lets a gap pass unmeasured.
- a source restart or discontinuity, a nonpositive or misaligned interval,
  or a VEX rotation jump (an unannounced zeroing).
- movement while the gyro bias calibrates (above).

Nothing is lost, and nothing unplaces, for gaps within every limit: the next
step measures across them. A model that measures across a longer gap that
ended between two checks keeps the placement too, since nothing went
unmeasured: three wheels read cumulative counts and see rotation themselves.
A Pi stall longer than a model's limit looks like a gap to that model (the
two-wheel Pico gyro model sees only the newest sample afterwards), so it
unplaces too. Warn mode logs motion lost too.

| Situation | Automatic | Needs the operator |
|---|---|---|
| Brain link cable out, then back | The Pi keeps localizing; the Brain resumes its session. | VEX profile, out longer than `sensor_loss_ms` (250 ms): place again. |
| Brain program restart | New session; the unchanged profile re-applies with no reset. | VEX profile: place again (startup placement does it). |
| Pi restart | Waits, takes the profile again, serves the field again. | Place again. |
| Pico reboot or acquisition restart | New identity: encoders rebase with no false displacement; Pico IMU bias recalibrates. | Place again. |
| Pico IMU fails (used) | The Pico retries on its own; health shows it initializing or failed. | Place again once it is ready. |
| Pico IMU fails (VEX or no-IMU profile) | Nothing changes. | Nothing. |
| Pico UART closes | Reopened at most once per second. | Nothing, unless a used source went stale meanwhile. |
| Profile refused | The running profile keeps running. | Fix the profile on the Brain. |

Interrupted movements never resume by themselves; the Brain programs handle
that (see [Brain setup](../../../docs/brain_setup.md#8-sensor-loss-and-recovery)).

## State block health

| Bit | Meaning |
|---|---|
| 0 encoders fresh | every profile encoder received within `fresh_ms` (150) |
| 1 gyro fresh | the profile IMU source: the Pico IMU sensor, or the Brain mailbox's newest valid sample |
| 2 vision alive | world estimation published observations |
| 3 bias calibrated | calibration done |
| 4 Pico link | Pico frames arriving |
| 5 IMU initializing | the Pico reports its IMU initializing, aligning or retrying |
| 6 IMU failed | the Pico's quick attempts are used up; slow retries continue |
| 7 stationary | a stationary window qualified and nothing moved since |

Bits 4 to 6 are reported whatever the profile uses; they block nothing for a
profile that does not use the Pico IMU.

## Inspection

The inspection snapshot's `brain_link` section carries the session and
`pi_instance`, the state block as the Brain reads it, the running profile with
every wheel's active corrections, the bias calibration, the wheel readings,
a running Pico operation and the last reported path; `pico` and `events` carry
the Pico link and the lifecycle log. The viewer's Brain link panel turns them
into a readiness line (link, profile, sensors, map, calibration, placement)
and draws the profile footprint, the field boundary and collision boxes, and
the path. See [inspection](inspection.md).
