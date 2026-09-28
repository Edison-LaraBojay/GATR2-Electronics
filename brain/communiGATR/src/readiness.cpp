// readiness.cpp

#include "communigatr/readiness.h"

#include "common/link_documents.h"

namespace communigatr
{

namespace
{

ImuUse imuUse(const Client& client) {
    if (!client.profileConfigured()) {
        return ImuUse::kUnknown;
    }
    const ProfileDocument& doc = client.config().profile;
    gatr2::RobotProfileDoc decoded;
    if (!gatr2::decodeRobotProfile(doc.bytes, doc.len, decoded)) {
        return ImuUse::kUnknown;
    }
    switch (decoded.imu_source) {
    case gatr2::kImuSourcePico: return ImuUse::kPico;
    case gatr2::kImuSourceBrainVex: return ImuUse::kBrainVex;
    case gatr2::kImuSourceNone: return ImuUse::kNone;
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
    h.encoders_fresh   = (health & gatr2::kHealthEncodersFresh) != 0;
    h.imu_fresh        = (health & gatr2::kHealthGyroFresh) != 0;
    h.vision_alive     = (health & gatr2::kHealthVisionAlive) != 0;
    h.bias_calibrated  = (health & gatr2::kHealthBiasCalibrated) != 0;
    h.pico_link        = (health & gatr2::kHealthPicoLink) != 0;
    h.imu_initializing = (health & gatr2::kHealthImuInitializing) != 0;
    h.imu_failed       = (health & gatr2::kHealthImuFailed) != 0;
    h.stationary       = (health & gatr2::kHealthStationary) != 0;
    return h;
}

const char* calibrationName(uint8_t calibration) {
    switch (calibration) {
    case gatr2::kCalibrationNone: return "none";
    case gatr2::kCalibrationRunning: return "collecting";
    case gatr2::kCalibrationDone: return "done";
    case gatr2::kCalibrationWaitingStill: return "waiting for stillness";
    case gatr2::kCalibrationWaitingData: return "waiting for data";
    case gatr2::kCalibrationFailed: return "failed";
    }
    return "?";
}

LinkReadiness readinessOf(const Client& client, Seconds now, bool accept_configured_anchor) {
    LinkReadiness r;
    r.imu                   = imuUse(client);
    r.brain_imu_calibrating = client.benchImu().calibrating;
    const StateSample& sample = client.state();
    if (sample.valid) {
        const gatr2::BrainState& s = sample.state;
        r.health      = decodeHealth(s.health);
        r.calibration = s.calibration;
        r.localized   = (s.robot_flags & gatr2::kRobotLocalized) != 0;
        r.pose_valid  = (s.robot_flags & gatr2::kRobotPoseValid) != 0 &&
                       (s.robot_flags & gatr2::kRobotAgeKnown) != 0;
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
    case gatr2::kCalibrationWaitingStill: r.state = Readiness::kWaitingStill; return r;
    case gatr2::kCalibrationRunning:
    case gatr2::kCalibrationWaitingData: r.state = Readiness::kCalibrating; return r;
    case gatr2::kCalibrationFailed: r.state = Readiness::kCalibrationFailed; return r;
    default: break;
    }
    const uint8_t flags  = sample.state.robot_flags;
    const bool    anchor = (flags & gatr2::kRobotAnchorCommand) != 0 ||
                        (accept_configured_anchor && (flags & gatr2::kRobotAnchorConfigured) != 0);
    if (!r.localized || !anchor || client.placementPending()) {
        r.state = Readiness::kNeedsPlacement;
        return r;
    }
    r.state = r.pose_valid ? Readiness::kReady : Readiness::kSensorsInitializing;
    return r;
}

} // namespace communigatr
