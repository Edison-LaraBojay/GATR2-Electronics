// readiness.cpp

#include "communigatr/readiness.h"

#include "translaGATR/link_documents.h"

namespace communigatr
{

namespace
{

ImuUse imuUse(const Client& client) {
    if (!client.profileConfigured()) {
        return ImuUse::kUnknown;
    }
    const ProfileDocument& doc = client.config().profile;
    translagatr::RobotProfileDoc decoded;
    if (!translagatr::decodeRobotProfile(doc.bytes, doc.len, decoded)) {
        return ImuUse::kUnknown;
    }
    switch (decoded.imu_source) {
    case translagatr::kImuSourcePico: return ImuUse::kPico;
    case translagatr::kImuSourceBrainVex: return ImuUse::kBrainVex;
    case translagatr::kImuSourceNone: return ImuUse::kNone;
    default: return ImuUse::kUnknown;
    }
}

Readiness sensors(const LinkReadiness& r) {
    const HealthBits& h = r.health;
    if (!h.encoders_fresh) {
        return Readiness::kSensorsUnavailable;
    }
    switch (r.imu) {
    case ImuUse::kPico:
        if (h.imu_failed) {
            return Readiness::kSensorsUnavailable;
        }
        if (h.imu_initializing) {
            return Readiness::kSensorsInitializing;
        }
        return h.imu_fresh ? Readiness::kReady : Readiness::kSensorsUnavailable;
    case ImuUse::kBrainVex:
        if (r.brain_imu_calibrating) {
            return Readiness::kSensorsInitializing;
        }
        return h.imu_fresh ? Readiness::kReady : Readiness::kSensorsUnavailable;
    default: return Readiness::kReady;
    }
}

} // namespace

const char* toString(Readiness readiness) {
    switch (readiness) {
    case Readiness::kConnecting: return "connecting";
    case Readiness::kReconnecting: return "reconnecting";
    case Readiness::kProfilePending: return "profile pending";
    case Readiness::kProfileRejected: return "profile rejected";
    case Readiness::kSensorsUnavailable: return "sensors unavailable";
    case Readiness::kSensorsInitializing: return "sensors initializing";
    case Readiness::kWaitingStill: return "waiting for stillness";
    case Readiness::kCalibrating: return "calibrating";
    case Readiness::kCalibrationFailed: return "calibration failed";
    case Readiness::kNeedsPlacement: return "needs placement";
    case Readiness::kReady: return "ready";
    }
    return "?";
}

HealthBits decodeHealth(uint8_t health) {
    HealthBits h;
    h.encoders_fresh   = (health & translagatr::kHealthEncodersFresh) != 0;
    h.imu_fresh        = (health & translagatr::kHealthGyroFresh) != 0;
    h.vision_alive     = (health & translagatr::kHealthVisionAlive) != 0;
    h.bias_calibrated  = (health & translagatr::kHealthBiasCalibrated) != 0;
    h.pico_link        = (health & translagatr::kHealthPicoLink) != 0;
    h.imu_initializing = (health & translagatr::kHealthImuInitializing) != 0;
    h.imu_failed       = (health & translagatr::kHealthImuFailed) != 0;
    h.stationary       = (health & translagatr::kHealthStationary) != 0;
    return h;
}

const char* calibrationName(uint8_t calibration) {
    switch (calibration) {
    case translagatr::kCalibrationNone: return "none";
    case translagatr::kCalibrationRunning: return "collecting";
    case translagatr::kCalibrationDone: return "done";
    case translagatr::kCalibrationWaitingStill: return "waiting for stillness";
    case translagatr::kCalibrationWaitingData: return "waiting for data";
    case translagatr::kCalibrationFailed: return "failed";
    }
    return "?";
}

LinkReadiness readinessOf(const Client& client, Seconds now, bool accept_configured_anchor) {
    LinkReadiness r;
    r.imu                   = imuUse(client);
    r.brain_imu_calibrating = client.benchImu().calibrating;
    const StateSample& sample = client.state();
    if (sample.valid) {
        const translagatr::BrainState& s = sample.state;
        r.health      = decodeHealth(s.health);
        r.calibration = s.calibration;
        r.localized   = (s.robot_flags & translagatr::kRobotLocalized) != 0;
        r.pose_valid  = (s.robot_flags & translagatr::kRobotPoseValid) != 0 &&
                       (s.robot_flags & translagatr::kRobotAgeKnown) != 0;
    }

    if (!client.connected(now)) {
        r.state = client.everReady() ? Readiness::kReconnecting : Readiness::kConnecting;
        return r;
    }
    if (client.profileConfigured()) {
        const ProfileSync p = client.profile().state;
        if (p == ProfileSync::kInvalid || p == ProfileSync::kRejected) {
            r.state = Readiness::kProfileRejected;
            return r;
        }
        if (p != ProfileSync::kApplied) {
            r.state = Readiness::kProfilePending;
            return r;
        }
    }
    r.state = sensors(r);
    if (r.state != Readiness::kReady) {
        return r;
    }
    switch (r.calibration) {
    case translagatr::kCalibrationWaitingStill: r.state = Readiness::kWaitingStill; return r;
    case translagatr::kCalibrationRunning:
    case translagatr::kCalibrationWaitingData: r.state = Readiness::kCalibrating; return r;
    case translagatr::kCalibrationFailed: r.state = Readiness::kCalibrationFailed; return r;
    default: break;
    }
    const uint8_t flags  = sample.state.robot_flags;
    const bool    anchor = (flags & translagatr::kRobotAnchorCommand) != 0 ||
                        (accept_configured_anchor && (flags & translagatr::kRobotAnchorConfigured) != 0);
    if (!r.localized || !anchor || client.placementPending()) {
        r.state = Readiness::kNeedsPlacement;
        return r;
    }
    r.state = r.pose_valid ? Readiness::kReady : Readiness::kSensorsInitializing;
    return r;
}

} // namespace communigatr
