// readiness.h
// One readiness summary for status displays and start-up decisions, derived
// from a client: what the link, the robot profile, the sensors, the Pi IMU
// calibration and the placement still wait for. Also the decoded health
// bits of the latest state. Observation only; it never acts.

#pragma once
#include <cstdint>

#include "communigatr/client.h"

namespace communigatr
{

// First unmet condition, in this order.
enum class Readiness : uint8_t {
    kConnecting,          // no state reply yet since start
    kReconnecting,        // had one; the link or session is lost now
    kProfilePending,      // configured profile not applied on the Pi yet
    kProfileRejected,     // refused by the Brain check or the Pi, see ProfileStatus
    kSensorsUnavailable,  // profile encoders or IMU source not fresh, or the Pico IMU failed
    kSensorsInitializing, // Pico IMU starting, Brain VEX IMU calibrating, or no pose yet
    kWaitingStill,        // Pi IMU calibration saw movement; hold the robot still
    kCalibrating,         // Pi IMU calibration collecting a stationary window
    kCalibrationFailed,   // no qualified window within the Pi bound; recalibrate retries
    kNeedsPlacement,      // not placed, anchor not accepted, or a placement in flight
    kReady,               // placed pose from the Pi
};

const char* toString(Readiness readiness);

// gatr2::HealthBit of one state block.
struct HealthBits {
    bool encoders_fresh   = false; // every profile encoder
    bool imu_fresh        = false; // the profile IMU source, Pico or Brain bench
    bool vision_alive     = false;
    bool bias_calibrated  = false;
    bool pico_link        = false; // Pico frames arriving
    bool imu_initializing = false; // Pico IMU initializing or aligning
    bool imu_failed       = false; // Pico IMU failed; the Pico keeps retrying
    bool stationary       = false; // Pi qualified stationary window right now
};

HealthBits decodeHealth(uint8_t health);

// Short name of a gatr2::CalibrationState.
const char* calibrationName(uint8_t calibration);

enum class ImuUse : uint8_t {
    kUnknown, // no Brain profile: the Pi XML decides
    kNone,
    kPico,
    kBrainVex,
};

struct LinkReadiness {
    Readiness  state = Readiness::kConnecting;
    ImuUse     imu   = ImuUse::kUnknown; // from the configured profile
    HealthBits health;                   // latest state of this session; all false without one
    uint8_t    calibration           = gatr2::kCalibrationNone;
    bool       brain_imu_calibrating = false; // bench sample of the latest state poll
    bool       localized             = false;
    bool       pose_valid            = false;
};

// accept_configured_anchor as in LinkDriverConfig. Only the configured
// profile's IMU source counts: an absent or failed Pico IMU does not hold
// back a Brain VEX IMU profile.
LinkReadiness readinessOf(const Client& client, Seconds now, bool accept_configured_anchor);

} // namespace communigatr
