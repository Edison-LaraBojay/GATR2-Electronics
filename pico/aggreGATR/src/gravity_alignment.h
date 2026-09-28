// Startup gravity alignment for a rigidly mounted BNO08X. This is independent
// of transport and report encoding; timestamps use its wrapping 32-bit us clock.
#pragma once

#include <stdint.h>

namespace bno08x {

struct Vector3 {
    double x;
    double y;
    double z;
};

class GravityAlignment {
  public:
    // The robot must be stationary and level during calibration. An
    // accelerometer at rest measures specific force pointing UP, so projection
    // onto this axis makes counterclockwise yaw positive for any fixed mount.
    // Gravity supplies tilt, never the robot's field heading.
    //
    // Calibration needs 2 seconds and 200 unique acceleration reports, with:
    //   acceleration magnitude 9.80665 +/- 0.5 m/s^2;
    //   gyro magnitude <= 0.10 rad/s;
    //   acceleration vector within 0.25 m/s^2 of the window's first report;
    //   report gaps and accel/gyro timestamp skew <= 50 ms.
    // These are stationary-start checks, not a general motion detector.
    void reset();
    void addAcceleration(uint32_t sample_us, Vector3 acceleration_mps2);

    // Always supplies gyro data to the startup checks. Returns false until
    // aligned, or for nonfinite, repeated or late gyro reports. On success,
    // yaw_radps = dot(gyro_radps, startup_up). The axis then stays frozen until
    // reset(): later motion cannot silently redefine it. This does not correct
    // dynamic pitch/roll and does not remove gyro bias.
    bool projectGyro(uint32_t sample_us, Vector3 gyro_radps, double& yaw_radps);

    bool ready() const { return ready_; }
    uint32_t samples() const { return samples_; }

  private:
    void clearPartial();
    bool acceptTimestamp(uint32_t sample_us, bool& seen, uint32_t& previous);

    bool ready_ = false;
    bool has_accel_time_ = false;
    bool has_gyro_time_ = false;
    bool has_gyro_ = false;
    uint32_t last_accel_us_ = 0;
    uint32_t last_gyro_us_ = 0;
    uint32_t first_us_ = 0;
    uint32_t samples_ = 0;
    Vector3 gyro_{};
    Vector3 reference_{};
    Vector3 sum_{};
    Vector3 up_{};
};

} // namespace bno08x
