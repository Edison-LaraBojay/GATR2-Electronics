// recovery.cpp

#include "runtime/system.h"

namespace navigatr
{

void System::watchSensors(const SensorMap& sensors, MonotonicTime now) {
    sensor_lost_ = false;
    if (!sensor_loss_.active()) {
        return;
    }
    PicoLinkState        link;
    const PicoLinkState* pico = nullptr;
    if (pico_ != nullptr) {
        link = pico_->link();
        pico = &link;
    }
    const SensorLossMonitor::Result r = sensor_loss_.update(sensors, pico, now);
    if (!r.edge.empty()) {
        losePlacement(now, "sensor lost: " + r.edge, true);
    } else if (!r.level.empty()) {
        losePlacement(now, "sensor lost: " + r.level, !loss_warned_);
    }
    loss_warned_ = !r.level.empty();
    sensor_lost_ = !r.edge.empty() || !r.level.empty();
}

void System::watchModels(MonotonicTime now) {
    const uint64_t dropped = execute_localization_.droppedIntervals();
    if (dropped > models_dropped_) {
        // a sensor loss found this cycle already explains it
        losePlacement(now, "motion lost: " + execute_localization_.lastDrop(), !sensor_lost_);
    }
    models_dropped_ = dropped;
}

void System::losePlacement(MonotonicTime now, const std::string& why, bool note) {
    if (!robot_.initialized) {
        return;   // nothing placed, nothing to lose
    }
    if (!profile_config_.unplace_on_sensor_loss) {
        if (note) {
            noteEvent(now, why + " (warn only; the pose is kept)");
        }
        return;
    }
    execute_localization_.loseContinuity(why);
    robot_           = execute_localization_.state();
    placement_floor_ = command_.init_sequence;
    noteEvent(now, why + ": place again (odometry epoch " +
                       std::to_string(robot_.odometry_epoch) + ")");
    noteFault(CaptureFault::kContinuityLost, why);
}

void System::noteRecovery(MonotonicTime now) {
    RecoverySeen& s = seen_;

    // Brain link
    if (command_.reply.pending) {
        if (!s.brain_active && s.last_request.isSet()) {
            noteEvent(now, "Brain link requests resumed");
        }
        s.brain_active = true;
        s.last_request = now;
    } else if (s.brain_active && sameDomain(now, s.last_request) &&
               now - s.last_request > 1000) {
        s.brain_active = false;
        noteEvent(now, "Brain link quiet for 1 s");
        noteFault(CaptureFault::kLinkLost, "Brain link quiet for 1 s");
    }
    if (command_.session != s.session) {
        if (command_.session != 0) {
            noteEvent(now, "Brain session opened");
        }
        s.session = command_.session;
    }

    // Pico identity
    if (pico_ != nullptr) {
        const PicoLinkState link = pico_->link();
        if (s.pico_known) {
            if (link.frames_fresh != s.pico_fresh) {
                noteEvent(now, link.frames_fresh ? "Pico frames restored" : "Pico frames lost");
                if (!link.frames_fresh) {
                    noteFault(CaptureFault::kLinkLost, "Pico frames lost");
                }
            }
            if (link.reboots != s.reboots) {
                noteEvent(now, "Pico rebooted (boot " + std::to_string(link.boot_id) +
                                   "); encoders rebased, IMU bias recalibrates");
                noteFault(CaptureFault::kPicoReboot,
                          "Pico rebooted (boot " + std::to_string(link.boot_id) + ")");
            }
            if (link.restarts != s.restarts) {
                noteEvent(now, "Pico acquisition epoch " + std::to_string(link.acq_epoch));
            }
            if (link.imu_restarts != s.imu_restarts) {
                noteEvent(now, "Pico IMU restarted (epoch " + std::to_string(link.imu_epoch) + ")");
            }
        }
        s.pico_known   = true;
        s.pico_fresh   = link.frames_fresh;
        s.reboots      = link.reboots;
        s.restarts     = link.restarts;
        s.imu_restarts = link.imu_restarts;
    }

    // calibration of every function with a window
    const std::vector<ObservationFunctionStatus> functions = feed_->status().functions;
    if (s.stillness.size() != functions.size()) {
        s.stillness.assign(functions.size(), StillnessStatus{});
        for (std::size_t i = 0; i < functions.size(); ++i) {
            s.stillness[i] = functions[i].stillness;
        }
        return;
    }
    for (std::size_t i = 0; i < functions.size(); ++i) {
        const StillnessStatus& now_s = functions[i].stillness;
        StillnessStatus&       was   = s.stillness[i];
        if (now_s.calibration != was.calibration &&
            (now_s.calibration == BiasCalibration::kDone ||
             now_s.calibration == BiasCalibration::kFailed)) {
            noteEvent(now, functions[i].id + ": IMU bias calibration " +
                               toString(now_s.calibration) +
                               (now_s.calibration == BiasCalibration::kFailed
                                    ? " (" + now_s.reason + ")"
                                    : std::string()));
        } else if (now_s.attempts != was.attempts && now_s.attempts > 1) {
            noteEvent(now, functions[i].id + ": IMU bias calibration restarted (" +
                               now_s.reason + ")");
        }
        was = now_s;
    }
}

} // namespace navigatr
