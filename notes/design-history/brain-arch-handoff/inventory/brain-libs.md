# Inventory: `brain/` and `common/` in GATR2-Electronics

Read-only inventory, 2026-09-27. Line numbers are 1-based and taken from the working tree. The git working tree was clean at session start (HEAD `c2c5654`).

---

## 0. Tree facts and surprises

- **`brain/testing/localization-test/` is an empty directory.** It was created Sep 26 16:30. Git does not track empty directories, and no Makefile, doc, CMake or CI file references it. It looks like a leftover from creating or moving a PROS project. The real app is `brain/localization-test/`.
- Kernel files (`firmware/`, `include/pros/`), `bin/`, `.d/` and `.vscode/` are git-ignored per project (`.gitignore` lines under "PROS projects under brain/"). They exist locally in both apps.
- `brain/localization-test/bin/` holds a real local build: `hot.package.bin` from Sep 27 00:28, plus `bin/gatr2/brain/communiGATR/src/*.o` and `bin/gatr2/common/frame_codec.cpp.o`. `brain/testing/` has no `bin/`, so it has not been built in this tree.
- `pi/navigatr/tests/CMakeLists.txt:58-68` compiles these Brain sources from its own explicit list: investiGATR `src/*.cpp`, `sim/differential_drive_sim.cpp`, communiGATR `src/client.cpp` and `src/driver.cpp`. It uses them for `brain_link_e2e_gtest.cpp`. Renaming or adding a Brain source means updating that list too (docs/communigatr.md:646-652).
- Nothing in `brain/`, `common/` or `.github/workflows/ci.yaml` uses `-Werror`. Everything builds with `-Wall -Wextra` only, which is at odds with the stated "warnings-as-errors" convention.

---

## 1. `common/`: frame protocol (Q7)

### 1.1 `common/frames.h` (namespace `gatr2`, header-only, `<stdint.h>` only)

Framing:
```cpp
constexpr uint8_t kSync0 = 0xAA;                 // :15
constexpr uint8_t kSync1 = 0x55;                 // :16
constexpr uint16_t kMaxFrameLen = 128;           // :19
enum FrameType : uint8_t {                       // :22 (0x02,0x03 retired)
    kFrameSensor       = 0x01, // Pico  -> Pi
    kFrameBrainRequest = 0x10, // brain -> Pi
    kFrameBrainReply   = 0x11, // Pi    -> brain
};
```
Sensor frame (Pico to Pi, :29-63), with layout `sync0 sync1 | type | seq u8 | stamp_ms u32 | mask u16 | payload | xor u8`:
```cpp
enum SensorBit : uint16_t { kSensorEnc0=1u<<0, kSensorEnc1=1u<<1, kSensorEnc2=1u<<2,
                            kSensorGyroZ=1u<<3, kSensorAccelXY=1u<<4 };
constexpr uint8_t kSensorWidth[] = {4,4,4,4,8};
constexpr uint8_t kSensorBitCount = sizeof(kSensorWidth);   // 5
struct SensorSample { uint8_t seq; uint32_t stamp_ms; uint16_t mask;
                      int32_t enc[3]; int32_t gyro_z /*mdeg/s*/; int32_t accel[2] /*mg*/; };
```
Brain link v3 (:65-185):
- Envelope: `sync0 sync1 | type | len u8 | payload[len] | crc u16`. The CRC is CRC-16/CCITT-FALSE over type, len and payload.
- Request payload: `version u8 | op u8 | session u32 | request_id u16 | body`.
- Reply payload: `version u8 | op u8 | session u32 | request_id u16 | result u8 | pi_instance u32 | body`.
- The header offsets are "frozen for every version".

```cpp
constexpr uint8_t kBrainLinkVersion = 3;                      // :80
enum BrainOp : uint8_t {                                       // :82
    kOpHello = 1,          // body nonce u32
    kOpSetPose = 2,        // body x_mm i32, y_mm i32, heading_cdeg i32
    kOpSelectLandmark = 3, // body landmark_id u8, flags u8
    kOpGetState = 4,       // no body
    kOpGetStateWithImu = 5 // opt-in bench: flags u8, stamp_ms u32, rotation_mdeg i32
};
enum BenchImuFlagBit : uint8_t { kBenchImuValid = 1u << 0 };   // :90
enum BrainResult : uint8_t {                                   // :94
    kResultOk=0, kResultPending=1, kResultUnknownSession=2, kResultUnsupportedVersion=3,
    kResultUnsupportedOp=4, kResultInvalidArgument=5, kResultUnknownLandmark=6,
    kResultLandmarkUnsupported=7, kResultStale=8 };
enum SelectFlagBit : uint8_t { kSelectFlagSelected = 1u << 0 };   // :106
enum RobotFlagBit : uint8_t { kRobotPoseValid=1<<0, kRobotLocalized=1<<1, kRobotAgeKnown=1<<2,
                              kRobotAnchorCommand=1<<3, kRobotAnchorConfigured=1<<4 };  // :111
enum HealthBit : uint8_t { kHealthEncodersFresh=1<<0, kHealthGyroFresh=1<<1,
                           kHealthVisionAlive=1<<2, kHealthBiasCalibrated=1<<3 };       // :120
enum LandmarkSourceCode : uint8_t { kLandmarkSourceNone=0, kLandmarkSourceNominal=1,
                                    kLandmarkSourceObserved=2 };                        // :127
```
The structs have default initializers. They are in-memory only and are not the wire layout.
- `BrainRequest` (:134-149): `version, op, session, request_id, nonce, x_mm, y_mm, heading_cdeg, landmark_id, select_flags, imu_flags, imu_stamp_ms, imu_rotation_mdeg`.
- `BrainState` (:152-167): `robot_flags, x_mm, y_mm, heading_cdeg (-18000,18000], robot_age_ms u16, odometry_epoch u32, anchor_revision u32, health, landmark_id u8, landmark_source, lm_x_mm, lm_y_mm, lm_heading_cdeg, landmark_age_ms u16`.
- `BrainReply` (:170-185): `version, op, session, request_id, result, pi_instance, nonce, odometry_epoch, anchor_revision, landmark_id, select_flags, BrainState state`.
- `inline uint8_t checksum(const uint8_t* data, uint16_t len)` (:189) is the XOR checksum.

**The state block carries exactly one landmark: the session's single selected id (u8).** No multi-landmark, map, profile or chunk op exists.

### 1.2 `common/frame_codec.h` / `.cpp`

Constants (h:14-24):
```cpp
constexpr uint16_t kSensorHeaderLen = 10;
constexpr uint16_t kLinkEnvelopeLen = 6;
constexpr uint8_t kBrainRequestHeaderLen = 8;
constexpr uint8_t kBrainRequestMaxLen    = 32;
constexpr uint8_t kBrainReplyHeaderLen   = 13;
constexpr uint8_t kBrainReplyMaxLen      = 64;
constexpr uint8_t kBrainStateLen         = 40;
```
Functions (h:27-60):
```cpp
uint16_t crc16(const uint8_t* data, uint16_t len);          // poly 0x1021, init 0xFFFF
bool     sensorMaskValid(uint16_t mask);
uint16_t sensorPayloadLen(uint16_t mask);
uint16_t sensorFrameLen(uint16_t mask);
uint8_t  brainRequestLen(uint8_t op);
uint8_t  brainReplyLen(uint8_t op, uint8_t result);
uint16_t encodeSensorFrame(const SensorSample& in, uint8_t* buf, uint16_t cap);
uint16_t encodeBrainRequest(const BrainRequest& in, uint8_t* buf, uint16_t cap);
uint16_t encodeBrainReply(const BrainReply& in, uint8_t* buf, uint16_t cap);
bool decodeSensorFrame(const uint8_t* buf, uint16_t len, SensorSample& out);
bool decodeBrainRequest(const uint8_t* buf, uint16_t len, BrainRequest& out);
bool decodeBrainReply(const uint8_t* buf, uint16_t len, BrainReply& out);
class FrameReader { public: void reset(); bool push(uint8_t b); bool next();
  const uint8_t* frame() const; uint16_t frameLen() const; uint8_t frameType() const;
  private: uint8_t buf_[kMaxFrameLen]; ... };                // h:68-97
```
Lengths:

| Direction | Op | Payload (bytes) | Frame (bytes) | Source |
|---|---|---|---|---|
| Request | HELLO | 12 | | cpp:164-173 |
| Request | SET_POSE | 20 | 26, the largest request | cpp:164-173 |
| Request | SELECT | 10 | | cpp:164-173 |
| Request | GET_STATE | 8 | | cpp:164-173 |
| Request | GET_STATE_WITH_IMU | 17 | | cpp:164-173 |
| Reply | HELLO (any result) | 17 | | cpp:175-191 |
| Reply | SET_POSE Ok/Pending | 21 | | cpp:175-191 |
| Reply | SELECT Ok | 15 | | cpp:175-191 |
| Reply | GET_STATE / WITH_IMU Ok | 53 | 59, the largest reply | cpp:175-191 |
| Reply | Any other (op, result ≤ 8) | 13 (header only) | | cpp:175-191 |
| Reply | result > 8, non-HELLO | returns 0 (unknown) | | cpp:175-191 |

Headroom:
- Request body ≤ 24 bytes and reply body ≤ 51 bytes under the current Max constants.
- `kMaxFrameLen` 128 would allow a payload of up to 122 bytes.
- `FrameReader` buffers are sized to 128.

**Implication (speculation):** any RobotProfile or field-map transfer either needs chunking or raised `kBrainRequestMaxLen`/`kBrainReplyMaxLen`. Both are shared by all versions (interfaces.md:616-617).

Decode rules:
- A version other than 3 decodes the header only and returns true.
- A known op or (op, result) with the wrong length returns false.
- An unknown op or result decodes the header only and returns true (cpp:294-418).

`FrameReader`:
- It rejects a bad `len` as soon as byte 3 arrives.
- On rejection it drops only sync0 and rescans.
- `next()` reports further buffered frames (cpp:420-503).

### 1.3 `common/tests`

- `frames_gtest.cpp` checks the constant values. It does **not** assert `kOpGetStateWithImu == 5` or `kBenchImuValid`.
- `frame_codec_gtest.cpp` (1096 lines) covers known-byte vectors, CRC, reader relock and rescan, and two `BenchImuCodec` tests (:1047, :1080).

### 1.4 Documentation drift

`docs/interfaces.md` documents "Four operations" (:250-279). It does **not** document op 5 `GET_STATE_WITH_IMU`, `kBenchImuValid`, or the USB `NG1:` transport. No file in `docs/` mentions bench or USB for the Brain; the only hit is interfaces.md:629.

---

## 2. communiGATR public API (Q1)

Namespace `communigatr`. `using Seconds = double;` (client.h:17).

### 2.1 `byte_port.h`
```cpp
class BytePort { public: virtual ~BytePort() = default;
  virtual int  read(uint8_t* buf, int max) = 0;          // count, 0 none, <0 error; never blocks
  virtual bool write(const uint8_t* data, int len) = 0;  // all queued or false; never blocks
};
```

### 2.2 `client.h`
```cpp
struct BenchImuSample { bool valid=false; uint32_t stamp_ms=0; int32_t rotation_mdeg=0; };
struct ClientConfig {
  Seconds response_timeout=0.060, request_gap=0.005, state_period=0.020, link_timeout=0.25,
          pending_retry=0.020; int placement_attempts=10; Seconds placement_deadline=1.0;
  int select_attempts=5; Seconds select_retry=0.5, hello_backoff=1.0;
  std::function<BenchImuSample()> bench_imu;   // empty = pure v3 GET_STATE
};
using PlacementTicket = uint32_t; // 0 = none
enum class PlacementResult : uint8_t { kNone, kPending, kApplied, kRejected, kTimedOut, kSessionLost };
struct PlacementStatus { PlacementTicket ticket; PlacementResult state; uint8_t result;
                         uint32_t odometry_epoch, anchor_revision; };
enum class SelectionState : uint8_t { kNotRequested, kPending, kActive, kUnknownLandmark, kUnsupported };
enum class LinkError : uint8_t { kNone, kUnsupportedVersion, kUnsupportedOp };
struct StateSample { bool valid; uint32_t pi_instance, session; gatr2::BrainState state;
                     Seconds received_at, round_trip; };
struct ClientStats { uint32_t requests, resends, replies, timeouts, uncorrelated, bad_frames,
  drained_bytes, read_errors, write_errors, sessions, session_losses, pi_restarts,
  stale_hellos, unexpected; };
class Client {
  Client(BytePort& port, std::function<uint32_t()> nonce, const ClientConfig& config = {});
  void poll(Seconds now);
  PlacementTicket submitPlacement(int32_t x_mm, int32_t y_mm, int32_t heading_cdeg);
  PlacementResult placementResult(PlacementTicket) const;
  PlacementStatus placementStatus(PlacementTicket) const;
  bool placementPending() const;
  void selectLandmark(uint8_t id); uint8_t wantedLandmark() const; SelectionState selection() const;
  bool ready() const; bool connected(Seconds now) const; Seconds linkAge(Seconds now) const;
  uint32_t session() const; uint32_t piInstance() const; const StateSample& state() const;
  LinkError error() const; uint8_t peerVersion() const;
  const ClientStats& stats() const; const ClientConfig& config() const;
};
```

Behavior (client.cpp):

- **`poll(now)` order** (:34-48):
  1. Placement deadline check.
  2. `receive`: up to 16 reads of 64 bytes.
  3. Response timeout.
  4. `transmit`: at most one request.
- **Scheduling in `choose()`** (:390-464):
  - With no session, only HELLO is sent, gated by `next_hello_`.
  - Otherwise, in priority order:
    1. A forced bench IMU poll between placement retries (`bench_poll_before_retry_`).
    2. The placement, if pending and not acked.
    3. The selection, if needed.
    4. The GET_STATE poll every `state_period`. It becomes `kOpGetStateWithImu` when `bench_imu` is set.
- **Before every send** it calls `drain()`, which reads and discards the input and resets the reader (:489-503).
- **Retries** (:331-354):
  - HELLO resends the same bytes with no attempt limit.
  - SET_POSE resends the same bytes on timeout, and again after `pending_retry` on a `Pending` result. Attempts and the deadline bound it.
  - SELECT resends the same bytes up to `select_attempts`, then waits `select_retry` before a new transaction.
  - GET_STATE is never resent; each poll gets a new id.
- **Correlation** (:155-170): op and request_id must match. HELLO must also match the nonce; every other op must match the session.
- **Session loss** (`loseSession` :507-522) happens on a `pi_instance` change or `kResultUnknownSession`. It clears state, readiness and selection. A pending placement becomes `kSessionLost` and is never resent.
- **Incompatible peer** (:311-325): the client sets `error_`, drops the session, and retries HELLO every `hello_backoff`.
- **Pi silence alone never drops the session.** Only `connected()` goes false after `link_timeout`.
- **Placement applied** (:303-306): a GET_STATE after the Ok shows `anchor_revision >= ok.anchor_revision` using serial arithmetic (`anchorReached` :19-21).
- **Request ids** count 1..65535 and wrap to 1 (:549-553). A nonce of 0 or a repeat is replaced by `+0x9E3779B9` (:555-565).

### 2.3 `driver.h`
```cpp
struct DriverConfig { bool accept_configured_anchor = false; };
class Driver : public investigatr::InputSource {
  explicit Driver(Client& client, const DriverConfig& config = {});
  void request(const investigatr::InputRequest&) override;   // ids 1..255 -> wire; else release
  investigatr::InputSnapshot latest(Seconds now) override;
  PlacementTicket submitPlacement(const investigatr::Pose& pose);   // 0 if pending/non-finite/out of int32
  PlacementResult placementResult(PlacementTicket) const; PlacementStatus placementStatus(PlacementTicket) const;
  bool placementPending() const; Client& client(); const Client& client() const; const DriverConfig& config() const;
};
```
- **Unit conversion:** wire mm, cdeg and ms become m, rad (wrapped) and s. Placement rounds to the nearest unit and maps -18000 to 18000 (driver.cpp:82-95).
- **`robotValid`** (:129-136) requires all of:
  - `PoseValid`, `Localized` and `AgeKnown`;
  - `AnchorCommand`, or `AnchorConfigured` when the config allows it;
  - no placement pending.
- **Ages:**
  - robot age = `robot_age_ms/1000 + round_trip + (now - received_at)` (:46-51, :77).
  - landmark age = the same formula with `landmark_age_ms`.
- **Frame generation** increments whenever (pi_instance, session, odometry_epoch, anchor_revision) changes, and is 0 with no state (:109-125).
- **Landmark status order** (:138-181): not requested → no wire id gives `kUnknownLandmark` → not connected gives `kStale` → the selection state (`kPending`, `kUnknownLandmark`, `kUnsupported`) → source none gives `kUnavailable` → nominal or observed gives `kAvailable`.

### 2.4 `pros_driver.h` (PROS only)
```cpp
struct ProsDriverConfig { uint8_t port=0; int32_t baud=115200; uint32_t poll_period_ms=2;
  uint32_t task_priority=TASK_PRIORITY_DEFAULT+1; ClientConfig client; DriverConfig driver; };
struct ProsDriverStatus { bool started, ready, connected; Seconds link_age; uint32_t session, pi_instance;
  LinkError error; uint8_t peer_version; SelectionState selection; ClientStats stats; };
class ProsDriver : public investigatr::InputSource {
  explicit ProsDriver(const ProsDriverConfig&); ~ProsDriver() override;  // non-copyable
  bool start(); static Seconds now();   // micros()*1e-6
  void request(const investigatr::InputRequest&) override; investigatr::InputSnapshot latest(Seconds) override;
  PlacementTicket submitPlacement(const investigatr::Pose&);
  PlacementResult placementResult(PlacementTicket) const; PlacementStatus placementStatus(PlacementTicket) const;
  ProsDriverStatus status() const;
};
```
- There is no `placementPending()` and no `client()` accessor.
- It owns a `ProsSerialPort`, a `Client`, a `Driver`, one `pros::Mutex` and one `pros::Task` named `"communigatr"`.
- The task polls every `poll_period_ms` using `delay_until` (pros_driver.cpp:109-119).
- Every public call takes the mutex.
- The destructor only calls `task_->remove()`.
- The nonce comes from a murmur3 mix of `micros`, battery voltage and current, and addresses (:125-138).

### 2.5 `pros_serial_port.h`
```cpp
class ProsSerialPort : public BytePort { ProsSerialPort(uint8_t port, int32_t baud);
  bool open(); bool isOpen() const; uint8_t port() const; int32_t baud() const;
  int read(uint8_t*, int) override; bool write(const uint8_t*, int) override; };
```
- `open()` calls `serial_enable`, then `serial_set_baudrate`, then `serial_flush`.
- `write` is all-or-nothing: it checks `serial_get_write_free` against `len` first (pros_serial_port.cpp:14-46).

### 2.6 Host fakes (`sim/`, not in the PROS build)

- `FakeBusConfig{baud, brain_tx_latency, brain_rx_latency, reply_delay=0.002, reply_window=0.040, release=0.0002}`
- `enum class BusFault {kNone,kDropRequest,kDropReply,kDuplicate,kCorrupt,kFragment,kTruncate,kDelay}`
- `FakeBus`: `brainPort()`, `advanceTo`, `fault`, `setPiPresent`, `failWrites`, `sendToBrain`, `log`, `brainRequests`, `collision`, `silentReplies`.
- `FakePi(uint32_t pi_instance=0x0BADF00D)`: `receive`, `answer`, `restart`, `robot()`, `setLandmark(id, FakeLandmark)`, `setWorldEstimationNoop`, `setApplyDelay`, `setVersion`, `setUnsupportedOp`, plus counters.
  - FakePi's duplicate path only treats `kOpGetState` as a repeat-state op (fake_pi.cpp:111).
  - `sameBody` ignores the IMU fields.
- `LinkRig(ClientConfig, FakeBusConfig)`: `step`, `runUntil`, `run`, `rebootBrain`, `client()`, `now()`, a `nonces` deque, `pi`, `bus`.

### 2.7 Tests

| File | Test count |
|---|---|
| `client_gtest` | 39, including 4 `ClientBenchImu` |
| `client_timing_gtest` | 5, budget `T >= A_req+W+A_rep+R+L` |
| `driver_gtest` | 12 |
| `equivalence_gtest` | 3, same Navigator scenario on `SimulatedSource` and on `Driver` + FakePi |
| `fake_pi_gtest` | 6 |

---

## 3. investiGATR public API (Q1)

Namespace `investigatr`.

- **`geometry.h`**
  - `using Seconds=double, Meters=double, Radians=double; constexpr double kPi;`
  - `struct Pose{Meters x=0,y=0; Radians heading=0;}`
  - `Radians wrapAngle(Radians)` returns (-pi, pi].
  - `Pose compose(const Pose& a, const Pose& b)` computes `a*b`.
- **`pid.h`**
  - `struct PidGains{double kP=0,kI=0,kD=0,integral_limit=0,output_limit=1;}`
  - `class Pid{ explicit Pid(const PidGains& = {}, bool angular=false); void reset(); double update(double error, Seconds dt); const PidGains& gains() const; }`
  - `dt == 0` is P only. There is no D on the first sample. The integral is clamped so |I term| ≤ `integral_limit`. Angular mode differentiates the wrapped error difference (pid.cpp:85-102).
- **`drive.h`**
  - `struct DriveCommand{double forward=0, turn=0;}` in [-1,1]; +turn is CCW.
  - `struct TankOutput{double left=0,right=0;}`
  - `TankOutput mixTank(const DriveCommand&)` computes `left=f-t`, `right=f+t`, then scales so the larger magnitude is ≤ 1.
  - `double slewLimit(double previous, double target, double rate, Seconds dt)` limits increases only. A sign reversal drops to 0 first.
- **`input.h`**
  - `using LandmarkId=uint16_t; using FrameGeneration=uint32_t;`
  - `struct RobotEstimate{bool valid; Pose pose; Seconds age;}`
  - `enum class LandmarkSource{kNone,kNominal,kObserved}`
  - `enum class LandmarkStatus{kNotRequested,kPending,kAvailable,kUnknownLandmark,kUnsupported,kUnavailable,kStale}`
  - `struct LandmarkEstimate{LandmarkId id; LandmarkStatus status; LandmarkSource source; Pose pose; bool age_known; Seconds age;}`
  - `struct InputRequest{bool landmark=false; LandmarkId landmark_id=0;}` asks for **one landmark at most.**
  - `struct InputSnapshot{FrameGeneration frame; bool connected; Seconds link_age; RobotEstimate robot; LandmarkEstimate landmark;}` carries **one landmark.**
  - `class InputSource{ virtual void request(const InputRequest&)=0; virtual InputSnapshot latest(Seconds now)=0; }`
- **`odometry_source.h`**
  - `class OdometrySource : public InputSource{ explicit OdometrySource(Meters track_width); void align(const Pose&, Seconds now); bool alignFrom(const InputSnapshot&, Seconds max_age, Seconds now); void invalidate(); void update(Seconds now, Meters left, Meters right); void update(Seconds now, Meters left, Meters right, Radians heading); bool aligned() const; Pose pose() const; FrameGeneration frame() const; ... }`
  - It is a two-wheel differential model only (odometry_source.cpp:152-192). Landmarks report `kUnsupported`.
- **Host sims**
  - `DifferentialDriveConfig{track_width=0.30, max_wheel_speed=1.2, wheel_time_constant=0.05, Pose origin_offset}`.
  - `DifferentialDriveSim`: `step(TankOutput, dt)`, `pose`, `setPose`, `centerPose`, `left/rightSpeed`, `left/rightTravel`.
  - `SimulatedSource` with a `SimulatedLandmark` table.

---

## 4. The Navigator (Q2)

### 4.1 Signatures (navigator.h)
```cpp
struct Destination { enum class Kind : uint8_t { kField, kLandmark, kRobot };
  Kind kind; LandmarkId landmark; Pose pose;
  static Destination field(const Pose&); static Destination relative(LandmarkId, const Pose& offset);
  static Destination robotRelative(const Pose& offset); };
struct Waypoint { Destination destination; bool stop = false; };   // last always stops
using Path = std::vector<Waypoint>; using CommandId = uint32_t;
struct MotionOptions { Seconds timeout = 0; bool require_observed_landmark = true; };
enum class MotionState : uint8_t { kIdle,kWaiting,kTurning,kDriving,kAligning,kCompleted,kCancelled,kFailed };
enum class MotionReason : uint8_t { kNone,kInvalidCommand,kInputUnavailable,kInputLost,kFrameChanged,
  kLandmarkUnknown,kLandmarkUnsupported,kLandmarkUnavailable,kLandmarkJump,kTimedOut,kSourceChanged,kCancelledByCaller };
bool isTerminal(MotionState); const char* toString(MotionState); const char* toString(MotionReason);
struct MotionStatus { CommandId command_id; MotionState state; MotionReason reason; size_t waypoint_index,
  waypoint_count; bool has_destination; Pose destination; Meters distance_error; Radians bearing_error,
  heading_error; Seconds elapsed; };
class Navigator {
  explicit Navigator(InputSource& source, const NavigatorConfig& config = {});
  CommandId goTo(const Pose&, const MotionOptions& = {});
  CommandId goToRelative(LandmarkId, const Pose& landmark_local_offset, const MotionOptions& = {});
  CommandId goToRobotRelative(const Pose& offset, const MotionOptions& = {});
  CommandId follow(const Path&, const MotionOptions& = {});
  void cancel(); void setSource(InputSource&);
  const MotionStatus& status() const; const NavigatorConfig& config() const;
  DriveCommand update(Seconds now);
  static bool valid(const NavigatorConfig&, const char** why = nullptr);
};
```
The `NavigatorConfig` defaults are at navigator.h:90-121 and are also tabled in docs/investigatr.md:391-417. Summary:
- Tolerances and phases: `position_tolerance .03`, `arrive_distance .02`, `heading_tolerance .035`, `settle_time .25`, `near_distance .10`, `waypoint_pass_radius .15`, `turn_in_place_threshold .5`, `turn_exit .1`.
- PIDs: `drive_pid{2,0,.1,0,1}`, `heading_pid{1.5,0,.05,0,1}`, `turn_pid{1.2,0,.06,0,1}`.
- Output shaping: `max_forward .8`, `max_turn .7`, `forward_slew 2`, `turn_slew 3`, `min_forward 0`, `min_turn 0`.
- Input freshness: `max_pose_age .25`, `max_link_age .25`, `input_wait_timeout 2`, `input_loss_timeout .3`, `default_timeout 10`.
- Landmarks: `max_landmark_age 1.5`, `max_landmark_jump .15`, `max_landmark_jump_heading .25`, `max_nominal_correction .5`, `max_nominal_correction_heading .5`, `landmark_follow_speed .3`, `landmark_follow_turn_rate .6`.
- `max_dt .1`.

`valid()` also enforces the orderings `arrive < position_tolerance`, `pass_radius >= near`, and `turn_exit < turn_in_place` (navigator.cpp:161-219).

### 4.2 Semantics

- **Command identity:** `last_id_` increments on every command and wraps `UINT32_MAX` to 1 (cpp:236). One counter is shared by all command kinds. An invalid command still gets an id and ends immediately as `kFailed/kInvalidCommand`.
  - Invalid means: bad config, empty path, non-finite pose, landmark id 0, or a negative timeout (cpp:247-254).
  - The one-shot calls are one-waypoint `follow` calls with `stop=true` (cpp:221-233).
- **Replacement:** a new command resets status and timers. PIDs reset through `activate(0,0)`. The slew memory `output_` is **not** reset.
- **Cancellation:** `cancel()` gives `kCancelled/kCancelledByCaller` if the command is active, and does nothing when terminal. `setSource()` calls `request({})` on the old source, then cancels with `kSourceChanged` (cpp:260-272). Terminal states persist, and nothing resumes.
- **`update(now)` order** (cpp:274-338):
  1. dt, clamped to [0, `max_dt`]. A gap longer than `max_dt` calls `resetControl()`.
  2. `source_->request(need())`, **every call, even when idle**.
  3. If not active, return zero.
  4. The first update records `start_time_` and `activated_at_`.
  5. Timeout: `options.timeout`, or `default_timeout` when that is 0, measured from the first update.
  6. `latest(now)`.
  7. Frame check: a captured frame, a nonzero snapshot frame, and a different value give `kFrameChanged`.
  8. Robot usability.
  9. `navigate`.
- **Stale-state handling:** `robotUsable` requires `valid && frame!=0 && connected && finite && age<=max_pose_age && link_age<=max_link_age` (cpp:360-363). If the input is unusable:
  - Before the frame is captured: wait, and give `kInputUnavailable` after `input_wait_timeout` from the start.
  - After capture with `!connected`: `kInputLost` immediately.
  - After capture while still connected: wait, and give `kInputLost` after `input_loss_timeout`.
  - Waiting returns `{0,0}` exactly and clears the slew memory through `zero()`.
- **How landmarks are consumed:** `need()` requests the landmark only while a `kLandmark` waypoint is active, and only **one landmark id at a time** (cpp:348-358). In `followLandmark` (cpp:484-533):
  - `kUnknownLandmark` and `kUnsupported` for that id fail immediately.
  - Usable means `kAvailable`, the id matches, the source is not `kNone`, the pose is finite, nominal is allowed only when `require_observed_landmark` is false, and an observed estimate is within `max_landmark_age` (cpp:365-378).
  - The first usable estimate resolves `destination_ = compose(landmark, offset)`. After that, a same-source jump beyond the limits gives `kLandmarkJump`. Nominal to observed is accepted within `max_nominal_correction`.
  - The destination is slewed toward the new target by `approach(...)` at `landmark_follow_speed*dt` and `landmark_follow_turn_rate*dt`.
  - An unresolved landmark waits up to `input_wait_timeout` from **waypoint activation**.
- **`goToRobotRelative`:** the target is resolved once from the first usable pose after activation, as `compose(robot_pose, offset)` (cpp:472-477).
- **Waypoint advance:** a non-stop waypoint advances on `distance <= waypoint_pass_radius`, or when it is Driving, `along` was positive, and `along <= 0` now (the perpendicular was crossed) (cpp:557-562).
  - The loop runs up to `path.size()+1` passes per update, so several advances can happen in one tick.
  - Pass-through waypoints ignore heading.
  - While Driving toward a pass-through waypoint, the along-track error is augmented by `remainingPath()`: the summed straight-line lengths of the following **field** waypoints up to the next stop. It stops at the first non-field waypoint (cpp:380-395, 619).
- **Phase machine and control** (cpp:440-653):
  - **Selection:** a stop waypoint with `d<=arrive` goes to Aligning; `|bearing|>turn_exit` goes to Turning; otherwise Driving.
  - **Turning:** `turn = turn_pid(bearing)`, with the direction held beyond pi minus 5 degrees (`holdTurnSign`).
  - **Driving:** `forward = clamp(drive_pid(along)*max(0,cos(bearing)), 0, max_forward)`. **Forward is never negative (no reverse).** `turn = heading_pid(bearing)` while `d > near_distance`.
  - **Aligning:** `turn = turn_pid(heading_err)`.
  - **Settling:** inside both tolerances continuously for `settle_time`, accumulated by dt. Any phase change resets it (cpp:633-647).
  - `min_forward`/`min_turn` raise small demands.
  - PIDs reset on phase change, activation and dt gaps.
- **Output:** `DriveCommand{forward, turn}` is normalized. `shape()` clamps to `max_forward`/`max_turn` and slew-limits (cpp:655-665). It is non-holonomic; there is no lateral axis. The caller mixes the command (e.g. `mixTank`).

### 4.3 Planning vs following split (for investiGATR planner and actuGATR follower)

| Concern | Where | Proposed home |
|---|---|---|
| `Destination` kinds and their resolution (field, landmark-compose, robot-relative compose) | cpp:78-98, 467-482 | Planner (goal resolution). The landmark part needs live estimates. |
| Landmark tracking: jump checks, nominal to observed, `approach` smoothing | cpp:484-533 | Ambiguous: goal-tracking policy. It moves the goal during execution, so it probably belongs with the follower's target feed. |
| `Path`/`Waypoint` data, stop flags, `remainingPath` | h:33-38, cpp:380-395 | Path representation goes to the planner; `remainingPath` is used by the follower's speed law. |
| Waypoint advance (pass radius, perpendicular) | cpp:556-574 | Follower. |
| Phase machine (Turning, Driving, Aligning), PIDs, settle, min outputs, slew, clamps | cpp:440-465, 578-670 | Follower/controller (actuGATR). |
| Input gating, frame capture, timeouts, command ids, cancel, `setSource` | cpp:235-338 | Command executive. It spans both, and today sits in the Navigator. |
| `mixTank`, `slewLimit` | drive.cpp | Drivetrain adapter (tank). |
| Obstacle avoidance, collision boxes, holonomic model, trajectory generation | none | **Does not exist.** Header says "Forward driving only, no obstacle routing" (h:3-4). |

---

## 5. `brain/testing` DriveControl (Q3)

`include/drive_control.h`:
```cpp
enum class DriveMode : uint8_t { kDisabled, kNavigate, kManual };               // :15
class DriveControl {
  DriveControl(investigatr::InputSource& source, investigatr::Seconds (*now)()); // :21
  investigatr::CommandId goTo(const Pose&, const MotionOptions& = {});
  investigatr::CommandId goToRelative(LandmarkId, const Pose& offset, const MotionOptions& = {});
  investigatr::CommandId follow(const Path&, const MotionOptions& = {});
  void manual(const investigatr::DriveCommand& demand);
  void stop();
  investigatr::MotionStatus status() const; DriveMode mode() const;
private: void run(); void apply(const investigatr::TankOutput&);
  Seconds (*now_)(); pros::MotorGroup left_, right_; investigatr::Navigator navigator_;
  DriveMode mode_ = kDisabled; DriveCommand manual_; mutable pros::Mutex mutex_; std::unique_ptr<pros::Task> task_;
};
```
`goToRobotRelative` is **not** exposed.

- **Motors** (robot_config.h:19-20):
  - `kLeftMotorPorts = {-1, -2, -3}`, all reversed.
  - `kRightMotorPorts = {4, 5, 6}`.
  - **Flag: a left drive motor is on Smart Port 1**, reversed via the negative port.
  - Built with `pros::MotorGroup(std::vector<int8_t>, kGearset)` (drive_control.cpp:33-34). `ports()` converts the arrays to vectors (:18-20). `set_brake_mode_all(kBrakeMode)` is applied (:36-37).
  - Gearing is `pros::MotorGears::blue` and brake mode is `brake`; both are PLACEHOLDERs. Encoder units are left at the kernel default.
  - **DriveControl never reads encoders.**
- **Task:** `pros::Task` `"drive"` at `TASK_PRIORITY_DEFAULT + 1`, 10 ms `delay_until` (:16, :38-39, :91-108). It is created in the constructor.
- **Ownership:** each period, under `mutex_`, the task:
  1. calls `navigator_.update(now_())` **in every mode**, so the landmark requests and status stay live;
  2. picks the demand: `kNavigate` uses the navigator output, `kManual` uses `manual_`, `kDisabled` uses zero;
  3. calls `apply(mixTank(demand))`.
- **Mode changes:**
  - `goTo`, `goToRelative` and `follow` set `kNavigate`.
  - `manual()` cancels the navigator and sets `kManual`.
  - `stop()` cancels, sets `kDisabled`, zeroes the manual demand, and calls `apply({})` **immediately in the caller's task** (:64-77).
- **Normalized to motor output** (:22-28, :111-119):
  - If both sides are exactly 0, it calls `brake()` on both groups.
  - Otherwise each side gets `move_voltage(lround(clamp(frac,-1,1) * kMaxVoltageMv))` with `kMaxVoltageMv = 8000`. Non-finite values give 0 mV.
- **Mutexes and lock order:** `DriveControl::mutex_` is taken first, then `ProsDriver::mutex_` inside `navigator_.update()` via `request()` and `latest()`. `ProsDriver` never takes the DriveControl mutex, so there is no inversion. Both tasks run at `DEFAULT+1`.
- **`main.cpp`:**
  - `initialize()` (:180-210) creates the `ProsDriver` and `DriveControl`, validates the config, calls `start()`, waits ≤3 s for `connected`, submits `kStartPose`, and waits ≤2 s.
  - `autonomous()` runs the demo: goal, then path, then landmark.
  - `opcontrol()` calls `manual()` every 10 ms with `turn = -rightX` (right is CW) and runs the demo on A, with B to cancel.
  - `disabled()` calls `stop()`.

---

## 6. `localization-test` USB bench driver (Q4)

`include/usb_bench_driver.h:13-33`:
```cpp
class UsbBenchDriver final : public investigatr::InputSource {
  explicit UsbBenchDriver(const ProsDriverConfig& config); ~UsbBenchDriver() override;
  bool start(); static Seconds now();
  void request(const investigatr::InputRequest&) override; investigatr::InputSnapshot latest(Seconds) override;
  PlacementTicket submitPlacement(const investigatr::Pose&);
  PlacementResult placementResult(PlacementTicket) const; PlacementStatus placementStatus(PlacementTicket) const;
  ProsDriverStatus status() const;
private: struct Impl; std::shared_ptr<Impl> impl_; };
```
The class copies the `ProsDriver` API surface by duplication; there is no shared interface beyond `InputSource`. `main.cpp:18-19` picks the implementation at compile time: `std::conditional_t<kUseUsbBench, UsbBenchDriver, ProsDriver>`. The driver lives in the **app**, not in communiGATR or `gatr2_brain.mk`.

**Wrapping.** `Impl` derives from both `BytePort` and `enable_shared_from_this`. It owns an ordinary `Client(*this, nonce, settings.client)` and a `Driver(client, settings.driver)` (usb_bench_driver.cpp:70-79). It uses `config.poll_period_ms`, `task_priority`, `client` and `driver`. **`config.port` and `config.baud` are ignored.**

**Tasks** (:93-131):

| Task | Priority | Role |
|---|---|---|
| `"bench-usb-tx"` | `TASK_PRIORITY_DEFAULT` | USB writes |
| `"bench-usb-rx"` | `TASK_PRIORITY_DEFAULT` | USB reads |
| `"bench-usb-link"` | `config.task_priority` | Runs `client.poll(now())` under `mutex` every `poll_period_ms` (:156-166) |

The public methods take the same mutex.

**USB console handling:**
- **Output:**
  - `output_fd = ::open("/ser/ngtr", O_WRONLY)`, a named PROS serial stream `"ngtr"`.
  - `pros::c::fdctl(output_fd, SERCTL_ACTIVATE, nullptr)`.
  - `pros::c::serctl(SERCTL_DISABLE_COBS, nullptr)` (:100-105). **This disables COBS multiplexing globally**, so there is no stream-id framing on any output, including stdout.
  - The comment says `SERCTL_NOBLKWRITE` is ineffective in kernel 4.2.2, so all USB I/O is isolated in worker tasks (:107-111).
  - Each frame is written as one `::write(output_fd, line, length)` and the result is ignored (:187).
  - There is no `fwrite` or `printf` in the driver.
- **Input:**
  - `::read(STDIN_FILENO, input, 95)`. The comment says kernel 4.2.2 `ser_read_r` appends a NUL, so the driver reserves one byte and uses the returned count (:197-199).
  - On a read count ≤ 0 it waits 5 ms (:201-203).
  - A line is terminated by `'\n'`. A trailing `'\r'` is stripped.
  - `NG1:` may appear anywhere in the line; leading diagnostic text is skipped by a forward search (:222-225).
  - Hex must be **uppercase** (`hexDigit` rejects lowercase), with an even length between 2 and 256 characters.
  - An overflowing line (more than 261 characters) is dropped whole (:205-216, :220-239).
- **Envelope bytes:** `"NG1:"` followed by uppercase hex of the whole binary link frame (`AA 55 type len payload crc`), then `'\n'`. The line capacity is 4 + 2×128 + 1 (:18-20, :168-189).
  - The header comment gives the reason: "so PROS cannot consume binary pR commands" (h:10-12).
  - The Pi counterpart is `pi/navigatr/src/impl/resources/pros_usb_link.{h,cpp}`. It uses `rfind("NG1:")` and finds the V5 Brain's user interface 02 (2888:0501).
- **Non-blocking behavior:**
  - `write()` from the client only pushes onto a 1-slot lock-free SPSC `FrameQueue<1>`. If the slot is full it returns false; the client counts `write_errors` and fails the attempt.
  - The rx queue is a 4-slot `FrameQueue<4>` (:28-52, :145-154, :250-251).
  - The tx task discards frames older than `tx_lifetime_ms = clamp(response_timeout*800, 1, 10000)`, which is 48 ms by default. A stuck USB writer therefore never replays expired requests (:74-78, :178).
  - A blocked USB write stalls only the tx task. The client poll task never touches USB.
  - The destructor only sets `stopping`, and tasks are never removed while inside kernel serial calls (:259-264).
  - stdout logging in `main.cpp` runs only when `!kUseUsbBench && kUsbDebug` (:236). `kUsbDebug=false` by default; the comment says "PROS stdout may wait for buffer space" (robot_config.h:39-42).
- **Observed (labeled) edge case:** if task creation fails, `stopping` becomes permanently true, and every later `start()` returns false with `ENOMEM` (:96-99, :122-128). `main.cpp` retries `start()` every 1 s, but it can never recover.
- **Bench IMU:**
  - `main.cpp:23-38` builds `readBenchImu` from a `pros::Imu` on `kVexImuPort`. It uses `rotation_mdeg = -get_rotation()*1000` to convert to CCW.
  - The sample is invalid while the IMU errors or calibrates.
  - **`stamp_ms = pros::millis()` at every call.** This is the read time, not a sensor acquisition time. It departs from client.h:19-20 ("Keep the acquisition stamp unchanged when reusing a sample").
  - It is installed as `config.client.bench_imu`, so it is called inside `client.poll` in the link task under the driver mutex.
  - The config overrides are `placement_deadline=2.0` and `placement_attempts=100` (:131-140).
- **App loop** (`runTest`, :125-249):
  - It runs in `pros::Task("localization-test")`, created from `initialize()`, with a 20 ms period.
  - Placement is attempted once automatically at startup, but only when the link is connected, the IMU is ready, and the time is within 10 s.
  - After that, controller A or a tap on the bottom screen strip places the robot.
  - It never auto-replaces after a timeout, disconnect or restart.
  - `opcontrol()` idles, and there is no motor control.

---

## 7. `robot_config.h` values (Q5)

**`brain/testing/include/robot_config.h`** (namespace `robot_config`; includes `api.h`; units m/rad/s). Unless noted, every value is marked PLACEHOLDER.

| Name | Value | Line |
|---|---|---|
| `kLeftMotorPorts` | `{-1,-2,-3}` | :19 |
| `kRightMotorPorts` | `{4,5,6}` | :20 |
| `kGearset` | `pros::MotorGears::blue` | :23 |
| `kBrakeMode` | `pros::MotorBrake::brake` | :24 |
| `kMaxVoltageMv` | 8000 (static_assert 0 < v ≤ 12000) | :27-28 |
| `kNavigatrPort` | 10 | :32 |
| `kNavigatrBaud` | 115200 | :33 |
| `kStartPose` | {0,0,0} | :36 |
| `navigatorConfig()` | position_tolerance .03, heading_tolerance .035, max_forward .8, max_turn .7, min_forward 0, min_turn 0, the default PIDs; other fields keep defaults | :40-52 |
| `kDemoGoal` | {0.6,0,0} | :56 |
| `kDemoPath` | {0.6,0.6,0}, {0,0.6,0}, {0,0,0} | :57-61 |
| `kDemoLandmarkId` | 1 | :65 |
| `kDemoLandmarkOffset` | {-0.5,0,0} | :66 |
| `kDemoRequireObserved` | true | :69 |
| `kDemoTimeout` | 15.0 | :72 |
| `kDemoButton` | A | :75 |
| `kCancelButton` | B | :76 |
| `kStickDeadband` | 5 | :77 |

**`brain/localization-test/include/robot_config.h`** (no `api.h`; `namespace robot_config {`):

| Name | Value | Line |
|---|---|---|
| `kUseUsbBench` | true | :11 |
| `kNavigatrPort` | 4 ("confirmed for this test") | :14 |
| `kNavigatrBaud` | 115200 | :15 |
| `kUseVexImuBench` | true | :19 |
| `kVexImuPort` | 1 (**Smart Port 1**) | :20 |
| `kStartX`, `kStartY`, `kStartHeadingDegrees` | 0, 0, 0 | :25-27 |
| `kStartPose` | converts degrees to radians | :28-29 |
| `kStartupWaitSeconds` | 10.0 | :33 |
| `kPlacementDeadlineSeconds` | 2.0 | :34 |
| `kMaxPoseAgeSeconds` | 0.25 | :35 |
| `kLoopPeriodMs` | 20 | :37 |
| `kDisplayPeriodMs` | 100 | :38 |
| `kUsbDebug` | false | :42 |
| `kLogPeriodMs` | 1000 | :43 |

The static_asserts (:45-50) check port ranges 1..21, and that the IMU port differs from the link port.

**The two apps' port maps conflict.**
- Smart Port 1 is a reversed left drive motor in `testing` and the VEX IMU in `localization-test`.
- Smart Port 4 is a right drive motor in `testing` and the RS-485 link in `localization-test`.

---

## 8. Build mechanics (Q6)

**`brain/gatr2_brain.mk`** is included twice from each Makefile: before `-include ./common.mk` and after it (testing Makefile:65-67, localization-test :64-66).
- **First pass** (`GATR2_BRAIN_SETUP`, :13-46):
  - `GATR2_ROOT ?= ../..` must be a **relative** path; a drive letter breaks `-iquote`.
  - `GATR2_BRAIN_LIBS ?= investigatr communigatr`. The dependency is not automatic; both must be listed.
  - Explicit source lists:
    - investigatr: drive, geometry, navigator, odometry_source, pid.
    - communigatr: `src/client.cpp`, `src/driver.cpp`, `pros/pros_serial_port.cpp`, `pros/pros_driver.cpp`, `common/frame_codec.cpp`.
  - Objects go to `$(BINDIR)/gatr2/<repo-relative path>.o`.
  - `EXTRA_INCDIR += brain/investiGATR/include brain/communiGATR/include $(GATR2_ROOT)`, which enables `#include "common/frame_codec.h"`.
  - `ELF_DEPS += $(GATR2_OBJ)`.
- **Second pass** (:48-64): one custom rule per source:
  `$(CXX) -c $(INCLUDE) $(CXXFLAGS) $(EXTRA_CXXFLAGS) $(GATR2_WARNFLAGS) -MMD -MP -MF <obj>.d`, with `GATR2_WARNFLAGS ?= -Wall -Wextra`. It avoids common.mk's `DEPFLAGS`/`mv`, which assume `src/`.
- **Includes:** common.mk builds `INCLUDE` with `-iquote`, so library headers must be included with quotes.
- **Consequence:** `pros_driver.cpp` and `pros_serial_port.cpp` are always compiled, even in USB mode.

**Makefiles** (both apps, the same pattern):
- `C_STANDARD:=gnu17`, `CXX_STANDARD:=gnu++20`, because the template's gnu23/gnu++26 are rejected by arm gcc 13, and the kernel headers need C++20.
- `WARNFLAGS+=-Wall -Wextra`.
- `EXTRA_CXXFLAGS=-D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING`.
- `USE_PACKAGE:=1` (hot/cold), `IS_LIBRARY:=0`, `.DEFAULT_GOAL=quick`.
- A hard `$(error ...)` fires if `firmware/libpros.a` or `include/pros/rtos.hpp` is missing (testing :58-60, localization-test :57-59).

**common.mk** is identical in both apps and is the stock kernel file.
- `MFLAGS=-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -Os -g -mthumb`.
- `WARNFLAGS+=-Wno-psabi`.
- Links with `--gc-sections` and puts `ELF_DEPS` in the hot image.

**`project.pros`:** both use kernel `"version": "4.2.2"` and target `v5`.
- `testing` points at `C:\Users\mbes1\AppData\Roaming\PROS\templates\kernel@4.2.2`.
- `localization-test` uses `"location": "."` and was re-serialized with PowerShell spacing.

**Host CMake:**
- `brain/CMakeLists.txt`: C++17, `CXX_EXTENSIONS OFF`. It uses a system GTest, or FetchContent googletest v1.14.0 when none is found. It adds the `investiGATR` and `communiGATR` subdirectories.
- Targets:
  - `investigatr` (STATIC), `investigatr_sim`, `investigatr_tests`;
  - `communigatr` (STATIC: `client.cpp`, `driver.cpp`, `common/frame_codec.cpp`; PUBLIC link to investigatr; includes `include/` and the repo root);
  - `communigatr_fakes`, `communigatr_tests`.
  - `pros/` is never built on the host.
- `common/tests/CMakeLists.txt` is a separate project with `common_tests`.
- CI (`.github/workflows/ci.yaml`) has a `common` job (`cmake -S common/tests`) and a `brain` job (`cmake -S brain -B build-brain`). PROS apps are not built in CI.

---

## 9. What does NOT exist (relative to the target architecture)

1. **No RobotProfile** of any kind: no type, op, or field for encoder ports, wheel radius, CPR, reversal, offsets, 2-wheel vs 3-wheel geometry, IMU source or port, footprint, or camera mounts. Robot geometry lives in Pi XML. The Brain sends only a SET_POSE placement, and optional bench IMU rotation via op 5.
2. **No field-map or field-estimate snapshot transfer and no chunking.** The state block carries one selected landmark (u8 id), `InputRequest` asks for one landmark, and `InputSnapshot` holds one `LandmarkEstimate`. There is no "all landmarks" publication.
3. **No path planning:** no obstacles, collision boxes, holonomic model, mecanum, strafe axis, reverse driving, motion profiling, pure pursuit or latency compensation. `DriveCommand` is `{forward, turn}` only.
4. **No actuGATR:**
   - Motor I/O exists only as app code in `brain/testing/src/drive_control.cpp`, tank only.
   - Brain-side wheel odometry exists only as `OdometrySource` plus a doc sketch (docs/investigatr.md:256-289); no app uses it.
5. **No USB transport inside communiGATR.** `UsbBenchDriver` is app-local and duplicates the `ProsDriver` surface. There is no common "link driver" interface, and no stop/close API on either driver.
6. **No session-level reconnect trigger on silence.** A silent Pi only makes `connected()` false; the session drops only on `UnknownSession`, a `pi_instance` change, or an incompatible version or op.
7. **No docs** for op 5, `kBenchImuValid`, the `bench_imu` config, `UsbBenchDriver`, or `NG1:`.
8. **No `-Werror`** anywhere in the brain, common, or CI builds.