# naviGATR localization inventory: models, fusion, timing, lineage and reset rules

This was a read-only pass: no files were changed and nothing was built. The git tree was clean at HEAD `c2c5654`. Paths below are relative to `pi/navigatr/` unless they start with `common/` or `brain/`. Line numbers refer to HEAD.

---

## 0. Summary

- **Observation models.** Six registry keys map to five observation functions (`src/impl/localization/register_localization.cpp:15-25`):
  - `brain_imu_parallel_bench` → `BrainImuWheelBench::createParallel`
  - `brain_imu_planar_bench` → `BrainImuWheelBench::createPlanar`
  - `tracking_wheel_motion`, `imu_heading_increment`, `attitude_reference`
- **Estimators.** Three keys (`:27-31`): `noop`, `planar_motion_integrator`, `weighted_planar_fusion`.
- **The bench rename is partial.**
  - Commit `c2c5654` renamed the files and class `brain_imu_parallel_bench.{h,cpp}` / `BrainImuParallelBench` to `brain_imu_wheel_bench.{h,cpp}` / `BrainImuWheelBench`.
  - It also added a second registry key, `brain_imu_planar_bench`.
  - There is **no registry key `brain_imu_wheel_bench`**. XML still selects `brain_imu_parallel_bench` or `brain_imu_planar_bench`.
- **No runtime reconfiguration exists.**
  - The whole `System` is built once from XML (`System::buildFromFile/buildFromString`), and construction is atomic.
  - `reset()` only returns to power-on state (`src/runtime/system.cpp:575-595`).
  - The Pi has no `RobotProfile` type or concept. A grep for `RobotProfile|robot_profile` outside `build*` finds nothing.
- **The two IMU paths differ in kind, not just in parameters.**
  - The Pico gyro path is a device-clock sensor with exact-interval alignment.
  - The Brain VEX IMU path is a host-arrival-time mailbox, used only by the two bench observation types.
  - They cannot be fused with each other. Brain IMU with 3 wheels does not exist.

---

## 1. XML schema per model: what each validates and rejects

### 1.1 `<Localization>` aggregate (`src/runtime/localization_stage.cpp`, `make_localization` at 133-272; schema comment at `localization_stage.h:7-18`)

- **Allowed children:** `Observation`, `Estimator`, `History`, `InitialPlacement`. Anything else fails with "has unknown element" (`.cpp:235-238`).
- **`<Observation id type>`**
  - Both attributes are required (`:160-163`).
  - A duplicate `id` is rejected (`:164-169`).
  - The type must be registered (`:170-175`).
  - The factory owns the subtree.
  - Every declared output needs an id, and duplicate output ids across functions are rejected (`:183-196`).
- **`<Estimator type>`**
  - Exactly one is required: "needs exactly one Estimator; select an implementation or the noop" (`:241-245`). A second one fails with "more than one Estimator" (`:203-206`).
  - `type` is required (`:246-250`).
  - The estimator receives the declared outputs through `StateEstimatorInitializationContext` (`:257-261`).
- **`<History>`**
  - Attributes: `retention_s` (default 5), `capacity` (1024), `max_interpolation_gap_ms` (100), `attitude_gap_ms` (100).
  - All must be positive and `capacity >= 2` (`:209-226`).
  - `attitude_gap_ms` is accepted in code but is missing from the header comment.
- **`<InitialPlacement x_m y_m heading_deg>`**
  - All three attributes are required (`src/config/pose3_config.h:34-43`).
  - It becomes `PlacementRequest{origin="configuration", session=0, sequence=1}` (`.cpp:227-234`).
- **Shared attribute parsing.** `ConfigNode::getDouble/getInt` reject non-numeric and non-finite values with "invalid value" (`src/config/config_node.cpp:59-75`). A missing required attribute gives "missing required attribute X" (`:115-131`). **Unknown attributes are silently ignored everywhere.**

### 1.2 `tracking_wheel_motion` (`src/impl/localization/tracking_wheel_motion.{h,cpp}`; schema at `.h:8-19`)

```xml
<Observation id=".." type="tracking_wheel_motion">
  <TrackingWheel sensor_id label radius_m position_x_m position_y_m measurement_angle_deg direction/>...
    | <Wheels resource_id="wheel_geometry"><Use wheel_id=".."/>...</Wheels>
  <HeadingConstraint sensor_id bias_samples="200" max_calibration_travel_m="0.005" max_gap_ms="250"/>   optional
  <LateralMotion assume="zero"/>                                                                        optional
  <Timing interval_tolerance_ms="20" max_pending_ms="500"/>                                             optional
  <Output observation_id=".."/>
</Observation>
```

**Wheels**
- Needs the sensor catalog (`.cpp:107-110`).
- Inline `TrackingWheel` needs `sensor_id` (`:152-156`).
- `radius_m`, `position_x_m`, `position_y_m`, `measurement_angle_deg` and `direction` are all *required*, with no defaults (`:157-167`).
- `direction` must be `positive` or `negative` (`:168-172`).
- `radius_m <= 0` is rejected (`:131-134`).
- A sensor referenced by two wheels is rejected (`:119-125`).
- The sensor must bind with payload `EncoderSample` (`:127`); the `ConfigurationErrors` test checks for "different payload".
- Inline wheels and `<Wheels>` together are rejected (`:183-187`).
- `<Wheels>` needs a `resource_id` that resolves to `const WheelGeometryMap`, at least one `<Use>`, and every `wheel_id` must exist (`:188-234`).
- No wheels at all is rejected (`:237-241`).

**`HeadingConstraint`**
- Needs `sensor_id` bound as `ImuSample` (`:246-254`).
- `bias_samples >= 0`; `max_gap_ms > 0`; `max_calibration_travel_m > 0` (`:261-266`).

**`LateralMotion`**
- `assume` is required and must be `zero` (`:273-279`).
- At most one is allowed (`:280-283`).
- It requires `HeadingConstraint` (`:284-287`).
- Every wheel must have `|uy| <= 1e-9`, which means 0° or 180°, else it fails with "measures sideways travel" (`:288-295`).

**`Timing`:** `interval_tolerance_ms >= 0`, `max_pending_ms > 0` (`:299-308`).

**Observability is checked at build time** (`:311-355`):
- Zero-lateral: `solveForward` must succeed.
- `HeadingConstraint` without zero-lateral: the 2×2 `solvePlanar` needs `|det| >= 1e-6`, else "wheels are collinear in direction. Forward-only wheels need an explicit `<LateralMotion assume="zero"/>`".
- No constraint: the 3×3 `solveFull` needs `|det| >= 1e-9`, else "cannot observe planar motion without a HeadingConstraint".

**`Output`:** `observation_id` is required, and exactly one `Output` is allowed (`:357-366`).

**Not validated**
- A duplicate `HeadingConstraint`, `Timing` or `Wheels` element: only the first is read.
- Unknown child elements.
- `calibration_status`.
- *Observation from code (untested):* a single forward wheel with `HeadingConstraint` and `LateralMotion zero` passes the build, because `solveForward` with one row succeeds.

### 1.3 `wheel_geometry` resource (`src/impl/resources/wheel_geometry_resource.{h,cpp}`)

```xml
<Resource id type="wheel_geometry">
  <Wheel id sensor_id label radius_m position_x_m position_y_m measurement_angle_deg direction/>
</Resource>
```

- `id`, `sensor_id`, `radius_m`, `position_x_m`, `position_y_m`, `measurement_angle_deg` and `direction` are required (`.cpp:23-50`).
- Duplicate wheel `id` and duplicate `sensor_id` are rejected (`:30-42`).
- `radius > 0` (`:51-55`); `direction` must be `positive` or `negative` (`:56-64`); at least one `Wheel` (`:70-73`).
- The resource is published as `const WheelGeometryMap`, which is immutable (`:74`).
- The `WheelDecl` struct is at `src/resources/wheel_geometry.h:19-28`.
- `calibration_status` is ignored by the composer (`src/config/composition.cpp:215,292`).

### 1.4 `brain_imu_parallel_bench` / `brain_imu_planar_bench` (`src/impl/localization/brain_imu_wheel_bench.cpp:19-78`)

```xml
<Observation id type="brain_imu_planar_bench|brain_imu_parallel_bench">
  <Wheels resource_id="wheel_geometry"><Use wheel_id=".."/><Use wheel_id=".."/></Wheels>
  <Imu resource_id="brain_imu"/>
  <Output observation_id=".."/>
</Observation>
```

- Requires the resources and sensors contexts plus a non-empty `Output observation_id` (`:27-30`).
- `Imu` must resolve to a `BrainImuBench` resource, and `Wheels` to a `WheelGeometryMap` (`:31-35`).
- **Only the resource-reference form is supported.** There is no inline `TrackingWheel`, `HeadingConstraint`, `LateralMotion` or `Timing`, and unknown children are ignored.
- More than two `Use` entries, or an unknown `wheel_id`, fails with "bench model needs exactly two declared wheels" (`:41-45`).
- Fewer than two, or the same encoder sensor twice, fails with "needs two distinct encoder sensors" (`:61-64`).
- **Parallel:** each wheel must have `|measurement_angle_deg| <= 1e-6`, so 180° is also rejected, unlike `tracking_wheel_motion` (`:46-50`).
- **Planar:** `|ux_a*uy_b - uy_a*ux_b| < 1e-3` fails with "planar bench needs two nonparallel wheel directions" (`:66-72`).
- Both variants push a "BENCH ONLY: pairs latest wheels and Brain IMU by Pi arrival time" build warning (`:73-76`).
- Freshness (200 ms) and the rotation-jump guard are hard-coded and cannot be configured.

**`brain_imu_bench` resource** (`src/impl/resources/register_resources.cpp:21-23`): `<Resource id type="brain_imu_bench"/>` takes no attributes.

**Command side** (`src/impl/commands/brain_link_commands.cpp:91-95`): `<CommandCollection type="brain_link"><Serial .../><BenchImu resource_id=".."/></CommandCollection>`.

### 1.5 `imu_heading_increment` (`imu_heading_increment.h:9-13`; `.cpp:8-45`)

- `<Input sensor_id>` is required and binds `ImuSample` (`.cpp:18-25`).
- `<Calibration bias_samples="200" max_gap_ms="250"/>`: `bias_samples >= 0`, `max_gap_ms > 0` (`:27-36`).
- `<Output observation_id>` is required (`:39-43`).

### 1.6 `attitude_reference` (`attitude_reference.h:8-12`; `.cpp:8-39`)

- `<Input sensor_id>` binds `AttitudeSample`.
- `<Freshness max_age_ms="100"/>` must be > 0.
- `<Output observation_id>` is required.
- Today only the synthetic rig produces `AttitudeSample`. The Pico sends no quaternion (`docs/coordinates.md:163-164`).

### 1.7 `planar_motion_integrator` (`planar_motion_integrator.h:9-13`; `.cpp:13-35`)

- `<Motion observation_id>` is required and must be declared as a `BodyMotionIncrement` payload (`.cpp:18-26`, using `requireObservation` at `contracts/localization.h:166-180`).
- `<Heading observation_id max_wait_ms="100">` is optional (`configureHeading`, `motion_step.cpp:328-361`):
  - At most one `Heading`.
  - `observation_id` must be a declared `HeadingIncrement`.
  - The retired `interval_tolerance_ms` is refused with an explicit message (`:348-352`).
  - `max_wait_ms` must be > 0.
- `<Attitude observation_id max_age_ms="200">` is optional (`AttitudeFold::configure`, `motion_step.cpp:99-123`). It must be an `AttitudeObservation`, and `max_age_ms` must be > 0.
- **Not validated:** unknown children, a duplicate `Motion`, a duplicate `Attitude`.

### 1.8 `weighted_planar_fusion` (`weighted_planar_fusion.h:9-20`; `.cpp:123-190`)

- Children are limited to `Motion`, `Heading` and `Attitude`; anything else fails with "has unknown element" (`.cpp:127`). The `Motion` and `Heading` children may contain only `Noise` (`:145,171`).
- Exactly one `Motion` (`:137-140`).
- `Noise` is required, and a second `Noise` is rejected (`parseNoise`, `:79-102`). Every attribute is required and must be `>= 0`.
  - Motion noise: `translation_floor_m translation_per_m rotation_floor_rad rotation_per_rad rotation_per_m`. The two floors must be > 0 (`:158-162`).
  - Heading noise: `angle_random_walk_rad_per_sqrt_s bias_rad_per_s`. ARW must be > 0 (`:180-183`).
- `Heading` and `Attitude` follow the same rules as 1.7. A duplicate `Attitude` is not detected.
- Landmark evidence payloads are refused at build ("different payload"; test `LandmarkEvidenceIsNotARobotObservation`).

### 1.9 Sensor-side schema that the models depend on (`src/impl/sensors/pico_channels.{h,cpp}`)

- **`pico_encoder_channel`**
  - `<Source resource_id output_id/>`, `<Calibration counts_per_revolution invert/>`, `<Freshness stale_after_ms="250"/>`.
  - CPR must be > 0 (`.cpp:144-155`).
  - `radians_per_count = 2π/CPR · (invert ? -1 : 1)`.
- **`pico_imu_channel`:** `<Source/>`, `<Calibration invert/>` (`:217-264`).
- **`pico_telemetry` resource:** `<Output id channel="0|1|2|imu"/>`. There are 3 encoder channels (`pico_telemetry.h:55`) and one gyro channel (`pico_telemetry.cpp:191-210`).

---

## 2. Two-wheel Brain-IMU bench model (`BrainImuWheelBench`)

### 2.1 Combination math (`brain_imu_wheel_bench.cpp:84-167`)

**Build-time geometry per wheel** (`:53-58`):
- `ux = cos(angle)`, `uy = sin(angle)`
- `k = x*uy - y*ux` (the lever arm)
- `sign = direction_positive ? +1 : -1`
- `determinant = ux_a*uy_b - uy_a*ux_b` (`:67`)

**Per emitted step:**
```cpp
motion.dtheta_rad = degToRad((static_cast<double>(imu_->rotation_mdeg) - rotation_) / 1000.0);   // :122
travel[i] = (samples[i]->angle_rad - w.angle) * w.radius * w.sign - w.k * motion.dtheta_rad;      // :130-131
// planar  (:145-146) — Cramer's rule on [ux uy] d = travel
motion.dx_m = (travel[0] * b.uy - a.uy * travel[1]) / determinant_;
motion.dy_m = (a.ux * travel[1] - travel[0] * b.ux) / determinant_;
// parallel (:148)
motion.dx_m = 0.5 * (travel[0] + travel[1]);   // dy stays 0
```

- The rotation comes **only** from the Brain VEX IMU. The Brain sends continuous `rotation_mdeg`, CCW positive (`common/frames.h:148`); the Brain app converts PROS CW-positive rotation to CCW (`docs/vex_imu_bench.md:39-40`). The Pi side has no invert knob.
- **Turn-offset (lever-arm) compensation** is the `- k*dθ` term.
  - A forward wheel at angle 0: `k = -y`. A CCW turn moves a left-mounted wheel backward by `y·dθ`, and the term adds it back.
  - A sideways wheel at 90°: `k = x`. A CCW turn moves a wheel ahead of the origin leftward by `x·dθ`, and the term removes it.
- For a perpendicular pair at (0°, 90°), `det ≈ 1`, so `dx = travel_forward` and `dy = travel_sideways`. Neither distance is averaged with the other.
- The payload uses `has_rotation = true` (the default). `has_rotation_coupling` stays false, so there is no J re-solve.

### 2.2 Timing and pairing (arrival time, not measurement time)

- **Freshness:** both the IMU `received` time and each wheel `stored.receivedAt` must be host-domain and at most 200 ms old against `context.now` (`:85-97`). Otherwise the model sets `baseline_=ready_=false` with the note "missing/stale; hold pose and rebaseline" and returns NoData.
- **Waiting rules:**
  - While a previous output is still offered, the model does nothing (`:103`).
  - With no new IMU sequence, it waits (`:104-105`).
  - It emits only when *both* wheels have a new stored sequence: "Do not emit heading-only updates while wheel records are retained" (`:113-116`).
- **Rebase (baseline updated, nothing emitted)** happens on any of these (`:106-112, :124`):
  - no baseline yet
  - an IMU epoch change
  - IMU receipt not advancing, or advancing by more than 200 ms
  - a wheel stored epoch change or a `discontinuity_epoch` change
  - `|dθ| > 12·dt + 0.1` rad (the jump guard)
- **Window:** `startAt = previous IMU host receipt`, `endAt = current IMU host receipt`, host clock domain (`:119-121`). The record's `measuredAt` and `receivedAt` are both `endAt` (`:161-162`).
- **The IMU mailbox** (`src/resources/brain_imu_bench.h:9-37`):
  - It is written by `brain_link` commands on op `kOpGetStateWithImu = 5` (`common/frames.h:87`; `brain_link_commands.cpp:264-266`), using the cycle `now` as `received`.
  - Handling rules:
    - A session change resets it.
    - `flags & 1 == 0` invalidates it.
    - A repeated `stamp_ms` is ignored ("retained sample cannot refresh health").
    - A stamp regression resets it, and `epoch` bumps on every reset.
  - Op 5 is `UnsupportedOp` unless `<BenchImu>` is configured (`brain_link_commands.cpp:232-235`). Unknown flag bits give `InvalidArgument` (`:236-239`). A new HELLO resets the mailbox (`:314`).
- **Lineage:**
  - Wheel sources keep their upstream provenance: measurement `pico_telemetry.encoder_x`, clock `pico_uart` (`:132-136`).
  - The IMU source is `Provenance{"brain_imu_bench","brain.vex_imu","brain",seq,epoch}` (`:150-151`).
  - Because the stamp domain is host, `observationClock` returns `"host"` without looking at the source clocks (`motion_step.cpp:37-40`). The "sources combine different clocks" fault therefore **never fires** for this mixed-device motion. That bypass is what makes arrival-time pairing possible.

### 2.3 What "corrected perpendicular-wheel bench behavior" means

- **Before `c2c5654`:** the only model was the parallel one, with `dx += 0.5*(travel + y·dθ)`, `dy = 0`, and 0° required. The USB profile described two parallel wheels.
- **The commit:**
  - added the planar variant (2×2 solve, general lever arm `k`);
  - switched `config/override/diagnostics/bench_vex_imu_usb.xml` to a forward wheel at (0, 0.15, 0°) and a sideways wheel at (0.15, 0, 90°) using `brain_imu_planar_bench` (`:35-42, :62`);
  - kept the RS-485 `bench_vex_imu.xml` on `brain_imu_parallel_bench` with two 0° wheels (`:31-38, :58`).
- **Tests** (`tests/brain_imu_bench_gtest.cpp`; geometry fixture at `:21-22`, perpendicular `{0,0.15,0°},{0.10,0,90°}`):
  - `PerpendicularWheelsMeasureForwardAndSidewaysIndependently` (`:223`): 1 rev on port 0 gives x = 2πr, y = 0; then 1 rev on port 1 gives y = 2πr. Neither is averaged.
  - `PerpendicularWheelsCompensateBothLeverArmsDuringRotation` (`:242`): a pure 5° CCW turn with counts `-0.15·dθ` and `+0.10·dθ` leaves x and y within one count of 0, and heading equals the IMU value.
  - `PerpendicularWheelDirectionCanReverseEncoderPolarity` (`:261`)
  - `PlanarWheelsAllowIndependentNonRightAngleDirections` (`:274`): a 45° second wheel gives y = L(√2−1).
  - `PlanarWheelsRejectCollinearAndNearlyCollinearDirections` (`:288`): angles 0, 180 and 0.001° fail with "nonparallel".
  - `ParallelProfileStillRejectsSidewaysWheelConfiguration` (`:303`): fails with "forward".
  - `PerpendicularMotionUsesSameFieldAxesForBrainAndInspection` (`:314`): after a 90° placement, body-forward travel becomes field +y in both the Brain reply and the inspection snapshot.
  - Pre-existing bench tests: `WirePathPlacementStraightTurnAndInspection` (`:145`; symmetric turn gives no x change and `clock_mapped` true), `DuplicateInvalidStaleAndRestartDoNotBridgeTravel` (`:180`), `OpRequiresExplicitConfiguration` (`:216`).
  - The USB profile end to end: `ProsUsbLink.UsbProfileHandshakeImuPlacementMotionAndInspectionUseExistingPipeline` (`tests/pros_usb_link_gtest.cpp:191`).

---

## 3. Three-wheel model and wheel + gyro fusion

### 3.1 `tracking_wheel_motion` solve (`tracking_wheel_motion.cpp`)

**Model.** Per wheel, `m_i = u_i·d + k_i·dθ` (`.h:21-22`). Travel accumulates as `(angle - prev_angle) * radius * sign` (`:522`).

**Three ways to solve:**
- **No constraint:** 3×3 normal-equation least squares over N ≥ 3 wheels for `(dx, dy, dθ)`, `solveFull` (`:53-89`, `:735-736`).
- **`HeadingConstraint`:** `dθ = gyro pending_dtheta`, then `m_i -= k_i·dθ`, then the 2×2 least squares `solvePlanar` (`:723-733`). The wheels' own rotation information is **discarded**; the gyro fully defines rotation.
- **Zero-lateral:** `dy = 0` and `dx = Σux·m / Σux²` (`solveForward`, `:39-50`). For forward wheels this is the mean lever-arm-corrected travel.

**Coupling `J = d(d)/d(dθ)`** is computed at build: `J = solvePlanar(ux, uy, -k)` (`:346-354`), or `solveForward(ux, -k)` in zero-lateral mode (`:325-330`). It is published on every increment (`:747-749`).

**Gyro bias calibration** (inside `HeadingConstraint`):
- It runs over `bias_samples` samples (`:549-579`) and restarts if wheel travel exceeds `max_calibration_travel_m` (`:552-554`).
- Wheels rebase and do not accumulate while calibrating (`:511-517`).
- It prefers `(accum_end - accum_start)/elapsed` over the mean of rates (`:568-576`).

**Integration.** The model integrates the difference of the producer's accumulator minus `bias·dt`, or uses a trapezoid when there is no accumulator (`:602-611`).

**Window validity.**
- The whole pending window is dropped by `dropWindow`, followed by a re-baseline, on any of:
  - a nonpositive interval
  - an encoder discontinuity or record epoch change
  - a gyro gap longer than `max_gap_ms`
  - a gyro accumulator epoch change
  - a gyro restart
- The drop points are at `:481-502` and `:585-622`, and the drop clears every source (`:418-430`).
- A solve requires every source to be pending, with start and end within `interval_tolerance_ms` of each other (`:654-706`).
- If sources stay misaligned longer than `max_pending_ms`, the window is dropped (`:683-705`).
- **This intra-model tolerance is NOT exact.** Exact alignment applies only at the estimator Heading level (3.4).

**Lineage.** Sources are the wheels' provenance plus the `HeadingConstraint` gyro provenance (`:741-746`), built by `provenanceOf` (`:91-100`).

### 3.2 `imu_heading_increment` (`imu_heading_increment.cpp`)

- Bias is the plain **mean of the first `bias_samples` rates**, with no motion gate (`:123-131`). This differs from `HeadingConstraint`.
- Then there is one increment per new sample: accumulator difference minus `bias·dt`, or a trapezoid (`:170-178`).
- A gap, nonpositive interval, accumulator epoch change, or source restart reseeds and drops (`:142-168`).
- While offered, contiguous same-epoch increments are merged into the pending one (`:83-97`); a non-contiguous one replaces it.

### 3.3 `weighted_planar_fusion` math (`weighted_planar_fusion.cpp:192-328`; documented at `.h:30-58` and `docs/localization_fusion.md:56-108`)

- **Variances** (`:207-221`):
  - `var_d = floor_m² + (per_m·|d|)²`
  - `var_w = floor_rad² + (per_rad·dθ_w)² + (per_m·|d|)²`
  - `var_g = ARW²·dt_g + (bias·dt_g)²`
- **With rotation and an aligned independent heading** (`:232-239`):
  - `w = var_w/(var_w+var_g)`
  - `dθ = dθ_w + w(dθ_g-dθ_w)`
  - `var = var_w·var_g/(var_w+var_g)`
  - `d += J(dθ-dθ_w)`, with `jj = w²(var_w+var_g)`
- **Without rotation** (`has_rotation=false`, `:240-247`): `dθ = dθ_g`, `d += J·dθ_g`, and the cross term is `J·var_g`. No production model emits `has_rotation=false` today; the flag is only set in tests (grep).
- **Propagation:** a chord Jacobian `B` (`:256-278`), then `P' = F P F' + G Q G'` with `F[0][2] = -g_y` and `F[1][2] = g_x` (`:288-306`).
- **Contrast:** `planar_motion_integrator` simply replaces `dθ` with the heading and does **not** re-solve translation through `J` (`planar_motion_integrator.cpp:46-49`).

### 3.4 Shared rules in `motion_step` (both estimators)

**Rule that prevents double counting a gyro that is also a `HeadingConstraint`** (`motion_step.cpp:257-263`):
```cpp
if (sharesMeasurement(heading->sources, motion.sources)) {
    result.outcome        = stash_.active ? Outcome::kWaiting : Outcome::kNone;
    result.reject_heading = true;
    result.diagnostic += "heading shares a measurement with the motion observation; not counted twice";
```
- `sharesMeasurement` returns true if any `measurementOf(pa) == measurementOf(pb)` (`:23-32`), where `measurementOf` is `p.measurement.empty() ? p.source : p.measurement` (`motion_step.h:44-46`).
- The motion carries the constraint gyro's provenance, so a `Heading` from the same Pico output is rejected on every step.
- **This is a runtime check, not a build check.** A config with both builds fine (`kTwoWheelConstrainedPlusSameGyro`).

**Measurement identity chain**
- `pico_telemetry` publishes `upstream.source = "pico:<link>"` with an empty measurement (`pico_telemetry.cpp:241-244`).
- The channel sensor sets `upstream.source = "<resource>.<output>"` and measurement = the upstream measurement or that source (`pico_channels.cpp:112-125`).
- The models copy `stored.upstream.measurement` (`tracking_wheel_motion.cpp:94-95`; `imu_heading_increment.cpp:183-184`).
- **Result:** two sensor ids on the same Pico output are one measurement, while different outputs of one Pico stay independent (`src/core/records.h:54-67`).

**Named clock identity**
- `observationClock` (`:34-56`):
  - host stamp → `"host"`;
  - device stamp → the sources' single `clock` name;
  - sources that disagree → false ("sources combine different clocks").
- `comparableClocks` (`:58-68`):
  - an empty name gives "insufficient clock identity";
  - different names give "different clocks a and b".
- The Pico clock name is the link resource id: `upstream.clock = telemetry->clockId()` (`pico_telemetry.cpp:242`), for example `pico_uart`.
- The heading must be on the motion's clock (`:247-256`).
- **A motion clock change is a discontinuity** (`:437-459`): `odometry_epoch += 1`, velocities are zeroed, `measuredAtHost` is cleared, the step is rejected, and the clock mapping, attitude and aligner are reset.

**Exact interval support** (`HeadingAligner::alignWindow`, `:223-326`)
- The first heading must start at `motion.startAt`. Later ones must start exactly at the stash end (`expected_start`, `:278-287`, "does not continue").
- A heading that ends after `motion.endAt` is rejected and releases any partial support (`:288-299`, "overruns").
- A heading needs a positive interval (`:264-270`).
- A stash is discarded when the heading's clock or `sources.front().epoch` changes (`:272-276`, "heading source restarted"), or when the motion window changes (`:229-233`).
- Angles are summed, and `dt` is summed (`:308-311`).
- The step is fused only when `stash_.end.ms == motion.endAt.ms` (`:314-320`).

**Bounded waiting** (`HeadingAligner::align`, `:188-221`)
- The deadline is anchored to the first sighting of the motion window, on the host clock (`window_.since`).
- `expired = (now - since) > max_wait_ms`. When waiting and expired, the outcome becomes kNone with `released` set and the diagnostic "never completed ... released without it".
- **A motion without rotation and without a fused heading** (`prepareStep`, `:529-541`) stays pending until release or expiry, then is rejected: "motion without observed rotation rejected after max_wait_ms".

**Exactly-once consumption**
- Contract: every id is in at most one of `accepted`/`rejected`, and unlisted ids stay pending (`contracts/localization.h:139-142`).
- The executor:
  - keeps a `pending_` map;
  - refuses a new output with the same id while one is pending (diagnostic `output_still_pending`, `localization_stage.cpp:51-55`);
  - erases on disposition, calling `settle` on the producer (`:75-94`);
  - flags an unknown disposition as `invalid_disposition`.
- Models hold an `offered_` flag. They keep accumulating but never overwrite the offered output (`localization.h:85-90`; for example `tracking_wheel_motion.cpp:650-652`).
- The estimator:
  - rejects unlisted observations (`motion_step.cpp:382-388`);
  - rejects a repeated effective time with "motion effective time already consumed" (`:480-487`), along with a heading for the same window (`rejectHeadingFor`, `:368-381`);
  - accepts stashed headings as they arrive (`:506-511`).

**Other rules**
- **Placement:** `PlacementEdge` is edge-triggered on the `(origin, session, sequence)` triple. It sets `field_from_odom = pose ∘ inverse(odom_pose)` and bumps `anchor_revision` (`:70-86`).
- **Source time regression:** epoch+1, the step is not integrated, and the clock, attitude and aligner are reset (`:461-479`).
- **Hold:** with no motion, the status is NoData and the effective time is unchanged (`:397-405`).
- **Host-time mapping:** `DeviceToHostClock` takes the minimum offset over the last 64 samples and is valid after 8 samples (`src/core/clock_sync.h:33-79`). It is fed `(step.stamp, motion_record.receivedAt)` (`motion_step.cpp:555-558`).
- **Attitude:** tilt only, with the source yaw discarded. It is aged on its own host time, and borrows the motion clock mapping only when `obs->source.clock == motion_clock` (`:125-186`, `:141-143`).
- **History:** entries use a host key `measuredAtHost` and are appended only when the step advanced (`robot_state_feed.cpp:17-24`).
  - An epoch change clears history (`:13`; `pose_history.cpp:36-69`).
  - Lookups return `kPending` for times newer than the newest entry, never a clamp, and `kGap` past `max_interpolation_gap_ms` (`pose_history.cpp:125-166`).
- **Reset:** `LocalizationExecutor::reset()` resets every function and the estimator, clears pending, sets `RobotState{}` with `odometry_epoch+1`, and re-arms the configured placement (`localization_stage.cpp:112-131`).

### 3.5 What the checked-in configs actually use

- **Three-wheel hardware profiles** use `tracking_wheel_motion` with `HeadingConstraint` and `planar_motion_integrator`, not `weighted_planar_fusion`:
  - `config/shared/pipelines/three_wheel_imu_no_correction.xml:13-30`
  - `three_wheel_imu_camera_diagnostic.xml:15-30`
  - `three_wheel_imu_camera_pipeline.xml.in`
- **Three wheels without a constraint, plus `imu_heading_increment` and `weighted_planar_fusion`,** exist only in `config/shared/pipelines/synthetic_fusion_demo.xml:18-50` and in the tests.
- **Parallel wheels:** `parallel_wheels_bno08x_localization.xml:14-26` uses `HeadingConstraint` + `LateralMotion zero` + `planar_motion_integrator`.

---

## 4. Coordinates, units and signs

- **Robot frame:** +x forward, +y left, +z up; the origin is a fixed chassis point. **Field frame:** +x right and +y toward the top of the audience view (`docs/coordinates.md:9-13`).
- **Units:** meters and radians internally. Only XML attributes ending in `_deg` are degrees. Heading is CCW-positive from +x, and `wrapAngle` returns values in `(-π, π]` (`coordinates.md:22-24`; `src/math/angles.h:14-22`).
- **Brain wire:** positions in mm; headings in centidegrees wrapped to `(-18000, 18000]` (`coordinates.md:25-27`).
- **Wheel direction vector:** `u = (cos a, sin a)` with `a = measurement_angle_deg`. Lever arm `k = x·u_y − y·u_x`. Offsets `position_x_m` and `position_y_m` are measured from the robot origin (`tracking_wheel_motion.cpp:137-140`; `coordinates.md:65-66`).
- **Radius vs CPR:**
  - CPR and electrical sign live in the **sensor** (`pico_encoder_channel`), which publishes the unwrapped shaft angle in radians: `delta_counts·2π/CPR·(invert?-1:1)`, with 32-bit modular delta (`pico_channels.cpp:144-197`; `sensor_samples.h:19-27`).
  - Radius and `direction` (the geometric sign) live in the **geometry**.
  - `travel = Δangle·radius·sign`.
  - That gives two sign knobs per wheel. The docs say to keep `invert="false"` and use `direction` (`docs/parallel_wheel_bringup.md:105`).
  - Raw counts are `int32` on the wire (`pico_telemetry_samples.h:19-21`).
- **Gyro:**
  - Pico wire is mdps plus a Pi-side accumulator in mdeg (`pico_telemetry.cpp:119-133`).
  - The sensor emits `yaw_rate_rad_s = sign·rad(mdps/1000)`, with the accumulator epoch shifted by restarts (`pico_channels.cpp:260-264`).
  - CCW positive. BNO08X firmware gravity alignment makes it CCW-positive for any mounting, so use `invert="false"` (`parallel_wheel_bringup.md:78-85,111-113`).
- **Brain IMU:** `rotation_mdeg` continuous and CCW-positive (`common/frames.h:148`).
- **Clock domains:** `kDevice` is the Pico clock, `kHost` is the Pi steady clock. Differencing across domains asserts (`src/core/time.h:17-39`).

---

## 5. How the IMU source is chosen today

**Pico gyro (BNO08X or ASM330).**
- The chip is chosen **at Pico firmware build time**: `pio run -e hat2_bno08x` or `-e hat2_asm330` (`pico/platformio.ini:20,23`).
- The Pi only sees chip-independent gyro Z (`kSensorGyroZ`, mdps; `common/frames.h:41,51`), and no Pi XML names the chip.
- Wiring:
  - `pico_telemetry <Output id="imu" channel="imu"/>`
  - then `<Sensor type="pico_imu_channel">`
  - then either `tracking_wheel_motion <HeadingConstraint sensor_id=...>` or `imu_heading_increment <Input sensor_id=...>`.
- Examples: `gatr2_parallel_wheels_bno08x.xml.in:46-49,93-96`; `gatr2_as5047_imu.xml.in:35-41,100-103`.

**Brain VEX IMU (bench only).**
- Pi side needs three pieces:
  - `<Resource type="brain_imu_bench"/>`
  - `<CommandCollection type="brain_link"><BenchImu resource_id/>`
  - `<Observation type="brain_imu_parallel_bench|brain_imu_planar_bench"><Imu resource_id/>` (`bench_vex_imu.xml:26,53-65`).
- There is no `pico_imu_channel` and no `channel="imu"` in the bench profiles.
- The VEX Smart Port is **Brain-only**: `constexpr uint8_t kVexImuPort = 1;` with `static_assert(... != kNavigatrPort)`, alongside `kUseVexImuBench` and `kUseUsbBench` (`brain/localization-test/include/robot_config.h:11,19,20,47`). The Pi has no field for it.
- The publisher's Health config in the bench profiles lists encoders only, not a Gyro (`bench_vex_imu.xml:75-78`), so `kHealthGyroFresh` is never set on the bench.

---

## 6. Switching modes at runtime in one executable

### 6.1 Where each RobotProfile field lives today

| Profile field | Current location | Mutability |
|---|---|---|
| Encoder logical port | `pico_telemetry <Output channel 0..2>` → `pico_encoder_channel <Source output_id>` → `wheel_geometry <Wheel sensor_id>` | Build time. Resource and sensor bindings are frozen (`docs/architecture.md:27-28`). |
| Wheel radius, offsets, angle, direction | `wheel_geometry` (`const WheelGeometryMap`) or inline `TrackingWheel` | Build time, immutable. |
| CPR, invert | `pico_encoder_channel <Calibration>` | Build time. The angle accumulates with it. |
| 2-perpendicular vs 3-wheel vs parallel | Which `Use` entries, plus `HeadingConstraint` / `LateralMotion` presence, plus observation type | Build time. |
| IMU source | Observation type (`tracking_wheel_motion` / `imu_heading_increment` vs `brain_imu_*_bench`) plus resources and commands | Build time. |
| Logical IMU port | Pico: exactly one `channel="imu"`. Brain: `kVexImuPort` constexpr | Not configurable on the Pi. |
| Footprint | `Inspection <RobotBody>`, display only (`three_wheel_imu_camera.xml.in:18-23`) | Build time. |

### 6.2 Changes needed (proposal, labeled speculative)

1. **A typed Pi-side profile struct and a single validator.** It would generate either XML fragments or direct objects for sensors, geometry and `Localization`.
2. **A controlled swap point on the estimation worker between cycles.**
   - Candidate location: inside `System::estimationCycle` before `execute_localization_` (`system.cpp:326-339`), or a stop-swap-start like `reset()`.
   - Only the Sensors, `wheel_geometry` and `LocalizationExecutor` would be rebuilt. The links, `pico_telemetry` (which drains the UART) and `brain_link` commands must be kept, because the profile arrives over that link and a full rebuild would drop `pi_instance` and the session.
3. **The epoch must advance on apply.** History clears automatically via `RobotStateFeed::publish` (`robot_state_feed.cpp:13`).
4. **Placement hazard.** `requestsFrom` re-offers the last command placement every cycle while `init_session == session` (`system.cpp:279-285`). A localization-only reset clears `PlacementEdge`, so **the old placement would re-apply to the new odometry origin**. Profile apply must either withdraw or retain the placement identity explicitly.
5. **Always configure all encoder outputs and the gyro output.** Declare all three encoder outputs plus `imu` on `pico_telemetry`, with one channel sensor each, so logical ports are a selection rather than a rebuild. CPR would need to move into the model, or the sensors would need rebuilding with a discontinuity.
6. **Always configure `<BenchImu>`** so op 5 is accepted in every mode. Select the bench model only when the IMU source is Brain.
7. **Health lists follow the profile.** The publisher's `Health Encoder/Gyro` sensor lists are build time (`brain_link_publisher.cpp:62-88,207-217`) and would need to follow the profile.
8. **Gyro bias calibration restarts on every apply** (model `reset()`), so the robot must be stationary.

### 6.3 Valid combinations vs combinations to reject

**Implementable today with existing types**

| # | Mode | Types |
|---|---|---|
| 1 | 2 non-parallel wheels + Pico gyro | `tracking_wheel_motion` + `HeadingConstraint` (planar solve) |
| 2 | 2 parallel wheels + Pico gyro | + `LateralMotion zero` |
| 3 | 3 wheels + Pico gyro as constraint | wheel rotation discarded |
| 4 | 3 wheels + Pico gyro fused | no constraint, `imu_heading_increment`, `weighted_planar_fusion` |
| 5 | 3 wheels, no gyro | `tracking_wheel_motion` alone |
| 6 | 2 non-parallel wheels + Brain VEX IMU | `brain_imu_planar_bench` |
| 7 | 2 forward (0°) wheels + Brain VEX IMU | `brain_imu_parallel_bench` |

**Rejected at build by current code**
- 2 wheels with no gyro.
- Collinear wheels without `LateralMotion`.
- `LateralMotion` without a constraint, or with a sideways wheel.
- Planar bench with collinear wheels (angles 0, 180, 0.001°).
- Parallel bench with a non-0° wheel, including 180°.
- Bench with a number of wheels other than 2.
- A duplicate encoder sensor.
- `radius <= 0`, `CPR <= 0`.

**Should be rejected explicitly by a profile validator (no code exists for these)**
- 3 wheels + Brain VEX IMU: the bench is fixed at two wheels.
- Brain IMU as a separate `HeadingIncrement`: no producer exists.
- Pico gyro plus Brain IMU together.
- Bench motion plus a Pico `Heading`: always rejected at runtime anyway ("different clocks host and pico_uart").
- A `Heading` from the same gyro as the constraint: accepted at build, but every step rejects it at runtime.
- `planar_motion_integrator` with a separate heading on 3 wheels: translation is not re-solved.
- More than 3 encoder ports: `kEncoderChannels = 3`.
- An IMU port on the Pico other than the one gyro channel.

**Scope note.** A single forward wheel with a gyro and `LateralMotion zero` builds but is untested; decide whether to reject it.

### 6.4 Tests that pin these behaviors

- **`tests/localization_models_gtest.cpp`**
  - TrackingWheelMotion: `ThreeWheelsSolvePlanarMotionWithProvenance`, `OutstandingOfferRetainsLaterMotionUntilDisposition`, `SensorIdsAreOpaqueRenamingChangesNothing`, `LabelsDoNotAffectBehaviorGeometryDoes`, `TwoWheelsNeedHeadingConstraintAndCalibrateFirst`, `CalibrationRebasesWheelsAndRestartsOnMotion`, `ConfigurationErrors`, `ImuOutageDropsTheWholeFusionWindow`, `UnhealthySourcesAreNotConsumed`, `RetainedRecordsAreNeverConsumedTwice`, `NonpositiveIntervalsAndDiscontinuitiesDropTheWindow`, `MisalignedIntervalsWaitForAlignmentThenSolveOnce`, `GyroAccumulatorDiscontinuityIsNotBridged`.
  - ImuHeadingIncrement: `PendingOfferStillIntegratesEveryRateSample`, `CalibratesThenIntegratesWithProvenance`, `OutagesDiscontinuitiesAndBadIntervalsReseed`.
  - AttitudeReference: `ForwardsFreshSamplesAndRejectsStaleOrBroken`.
- **`tests/localization_gtest.cpp`**
  - PlanarMotionIntegrator: `StraightLineAndVelocity`, `QuarterCircleChordAndHeadingFrame`, `PlacementRequestsAreEdgeTriggeredPerOrigin`, `SamePlacementFromANewSessionReAnchors`, `HeadingReplacesRotationOnlyWhenAlignedAndIndependent`, `UnobservedRotationIsNeverFabricated`, `MissingMotionHoldsPoseAndEffectiveTime`, `InvalidIntervalsAndTimeRegression`, `DeviceToHostMappingAfterWarmupUsesReceipt`, `AttitudeFusesUnderThePlanarHeadingAndAges`, `ReferencesValidateAtBuild`, `DispositionsDistinguishPendingAcceptedAndRejectedMotion`, `MissingReceiptNeverUsesProcessingTimeForClockMapping`, `InvalidQuaternionAndFutureAttitudeDoNotBecomeMeasuredLevel`, `AttitudeCannotBorrowAnUnrelatedDeviceClockMapping`, `HeadingSupportMustEqualTheMotionWindowExactly`.
  - `DeviceToHostClock.WarmsUpThenTracksMinimumLatencyOffset`.
  - `MakeLocalization.ValidatesTheAggregateSchema`.
  - LocalizationExecutor: `PublishesHistoryOnlyForAdvancesAndAnswersLookups`, `ConfiguredPlacementAppliesOnceThenCommandsWin`, `UndeclaredObservationOutputsNeverReachTheEstimator`, `DeferredObservationIsRetriedThenConsumedExactlyOnce`.
- **`tests/localization_fusion_gtest.cpp`**
  - WeightedPlanarFusion: `BuildRequiresExplicitNoiseAndKnownReferences`, `LandmarkEvidenceIsNotARobotObservation`, `RotationIsThePrecisionWeightedMean`, `InfluenceFollowsUncertainty`, `CouplingReattributesTranslationWhenRotationIsRevised`, `PartialMotionTakesRotationFromTheHeading`, `CovarianceGrowsWithHeadingCoupledIntoPosition`, `RotatedFrameKeepsTheCovarianceConsistent`, `SharedSourceIsNeverCountedTwice`, `MismatchedIntervalsAreNotFused`, `RepeatedSampleIsConsumedExactlyOnce`, `GyroDropoutGrowsHeadingUncertaintyFaster`, `HoldsWithoutMotionAndRejectsBadIntervals`, `SourceTimeRegressionAdvancesTheEpoch`, `ResetRestartsAtAnExactOrigin`, `PlacementReanchorsWithoutTouchingTheCovariance`, `TiltOnlyAttitudeNeverBecomesYaw`, `UnlistedObservationsAreRejectedNotSilentlyDropped`, `DifferentDurationsInsideTheOldToleranceAccumulateInstead`, `NonOverlappingWindowsInsideTheOldToleranceAreRejected`, `BatchedWindowsFuseAndALateHeadingNeverBlocks`, `PartialSupportTimesOutAndReleasesTheMotion`, `IdenticalStampsOnDifferentNamedClocksNeverCombine`, `UnnamedDeviceClockIsInsufficientToCombine`, `HeadingSourceRestartDiscardsPartialSupport`, `MotionClockChangeIsADiscontinuity`, `RepeatedWindowRejectsItsHeadingToo`, `ContinuousMismatchedHeadingsStillHitTheDeadline`, `MotionWithoutRotationHasADeadlineEvenWithoutAnyStash`.
  - LocalizationFusion: `ThreeWheelsAndAnIndependentGyroBlend`, `TwoWheelsWithTheConstraintStillWorkAndTheGyroCountsOnce`, `CheckedInSyntheticFusionDemoBuildsAndTracksTheRig`, `TwoSensorIdsOnOneImuOutputAreOneMeasurement`, `DistinctOutputsOfOnePicoStayIndependent`, `AliasedImuSensorsThroughTheRealPicoPathCountOnce`.
- **`tests/parallel_wheels_gtest.cpp`**
  - ParallelWheels: `DefaultModelStillRejectsParallelWheels`, `ZeroLateralAssumptionIsExplicitAndValidated`, `StraightTravel`, `TurnInPlaceWithUnequalOffsetsHasNoTranslation`, `ArcUsesLeverArmCorrectedForwardTravel`, `WheelSignsAndReversedMounting`, `RotationCouplingIsTheMeanLateralOffset`, `ImuIsPartOfTheLineage`, `MissingGyroNeverFabricatesHeadingFromWheels`, `WirePathStraightTurnAndArcsTrackTheTruth`, `InvertedEncoderChannelIsCorrectedByCalibration`.
  - ParallelWheelProfiles: `CheckedInProfilesNameTheMissingMeasuredRobot`, `LocalizationOnlyNeedsNoCameraAndNoThirdWheel`, `CameraProfileSharesTheExactLocalization`, `NoCameraPipelineTracksARigThroughTheOrdinaryExecutors`, `BrainPlacesTheRobotAndLandmarksAreUnsupported`.
- **`tests/brain_imu_bench_gtest.cpp`** (BrainImuBench): all 10 tests listed in 2.3.
- **`tests/pros_usb_link_gtest.cpp:191`:** the USB profile end to end.

### 6.5 Docs and code mismatches found

- `docs/architecture.md:91-97` lists the observation implementations but omits both bench types.
- `docs/localization_fusion.md:35` says "at most one Attitude", but a duplicate `<Attitude>` is not rejected (`AttitudeFold::configure` reads `child("Attitude")` only).
- `planar_motion_integrator` rejects neither unknown children nor a duplicate `Motion`.
- `localization_stage.h:15-16` omits the `History attitude_gap_ms` attribute that the code accepts.