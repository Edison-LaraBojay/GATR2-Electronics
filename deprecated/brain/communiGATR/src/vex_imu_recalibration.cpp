// vex_imu_recalibration.cpp

#include "communigatr/vex_imu_recalibration.h"

namespace communigatr
{

namespace
{

// A started calibration shows up in the samples well within this.
constexpr Seconds kStartGrace = 1.0;

} // namespace

const char* toString(VexRecalibrationState state) {
    switch (state) {
    case VexRecalibrationState::kIdle: return "idle";
    case VexRecalibrationState::kChecking: return "checking stillness";
    case VexRecalibrationState::kStart: return "starting";
    case VexRecalibrationState::kCalibrating: return "VEX IMU calibrating, hold still";
    case VexRecalibrationState::kDone: return "VEX IMU calibrated";
    case VexRecalibrationState::kMoving: return "robot moving, hold still and retry";
    case VexRecalibrationState::kRefused: return "refused by the Pi or no answer";
    case VexRecalibrationState::kImuFailed: return "VEX IMU did not calibrate";
    }
    return "?";
}

VexImuRecalibration::VexImuRecalibration(Seconds limit) : limit_(limit) {}

bool VexImuRecalibration::active() const {
    return state_ == VexRecalibrationState::kChecking ||
           state_ == VexRecalibrationState::kStart ||
           state_ == VexRecalibrationState::kCalibrating;
}

bool VexImuRecalibration::begin(ControlTicket ticket) {
    if (ticket == 0 || active()) {
        return false;
    }
    check_        = ControlStatus{};
    check_.ticket = ticket;
    check_.state  = ControlResult::kPending;
    seen_         = false;
    state_        = VexRecalibrationState::kChecking;
    return true;
}

VexRecalibrationState VexImuRecalibration::update(const ControlStatus& check,
                                                  const BenchImuSample& imu, Seconds now) {
    switch (state_) {
    case VexRecalibrationState::kChecking:
        if (check.ticket != check_.ticket) {
            return state_; // another ticket's answer
        }
        check_ = check;
        switch (check.state) {
        case ControlResult::kPending: break;
        case ControlResult::kOk: state_ = VexRecalibrationState::kStart; break;
        case ControlResult::kNotStationary: state_ = VexRecalibrationState::kMoving; break;
        default: state_ = VexRecalibrationState::kRefused; break; // incl. replaced, lost
        }
        return state_;
    case VexRecalibrationState::kCalibrating:
        if (imu.calibrating) {
            seen_ = true;
            if (now - started_at_ > limit_) {
                state_ = VexRecalibrationState::kImuFailed;
            }
            return state_;
        }
        if (!seen_) {
            if (now - started_at_ > kStartGrace) {
                state_ = VexRecalibrationState::kImuFailed;
            }
            return state_;
        }
        state_ = imu.valid ? VexRecalibrationState::kDone : VexRecalibrationState::kImuFailed;
        return state_;
    default:
        return state_;
    }
}

void VexImuRecalibration::started(bool ok, Seconds now) {
    if (state_ != VexRecalibrationState::kStart) {
        return;
    }
    started_at_ = now;
    state_      = ok ? VexRecalibrationState::kCalibrating : VexRecalibrationState::kImuFailed;
}

} // namespace communigatr
