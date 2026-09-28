// pros_drive.cpp

#include "actugatr/pros_drive.h"

namespace actugatr
{

ProsDrive::ProsDrive(DriveOwner& owner, Drive& drive, Seconds (*now)(),
                     const ProsDriveConfig& config)
    : owner_(owner), drive_(drive), now_(now), config_(config) {
    task_.reset(new pros::Task([this] { run(); }, config_.task_priority, TASK_STACK_DEPTH_DEFAULT,
                               "actugatr drive"));
}

ProsDrive::~ProsDrive() {
    quit_.store(true);
    for (int i = 0; i < 50 && running_.load(); ++i) {
        pros::delay(config_.period_ms);
    }
    if (running_.load()) {
        task_->remove();
    }
    drive_.stop();
}

CommandId ProsDrive::goToDirect(const Pose& destination, const Reference& relative_to,
                                const MoveOptions& options) {
    if (!mutex_.take(config_.lock_timeout_ms)) {
        return 0;
    }
    const CommandId id = requests_.goTo(PlanMode::kDirect, destination, relative_to, options);
    pending_sequence_  = ++sequence_;
    mutex_.give();
    return id;
}

CommandId ProsDrive::goToAvoiding(const Pose& destination, const Reference& relative_to,
                                  const MoveOptions& options) {
    if (!mutex_.take(config_.lock_timeout_ms)) {
        return 0;
    }
    const CommandId id = requests_.goTo(PlanMode::kAvoiding, destination, relative_to, options);
    pending_sequence_  = ++sequence_;
    mutex_.give();
    return id;
}

bool ProsDrive::manual(const ManualDemand& demand) {
    const Seconds now = now_();
    if (!mutex_.take(config_.lock_timeout_ms)) {
        return false;
    }
    requests_.manual(demand, now);
    pending_sequence_ = ++sequence_;
    mutex_.give();
    return true;
}

bool ProsDrive::cancel() {
    if (!mutex_.take(config_.lock_timeout_ms)) {
        forced_stop_.store(++sequence_);
        return false;
    }
    requests_.stop();
    pending_sequence_ = ++sequence_;
    mutex_.give();
    return true;
}

bool ProsDrive::status(DriveSnapshot& out) const {
    if (!mutex_.take(config_.lock_timeout_ms)) {
        return false;
    }
    out = requests_.snapshot();
    mutex_.give();
    return true;
}

void ProsDrive::run() {
    uint32_t wake = pros::millis();
    while (!quit_.load()) {
        DriveRequest request;
        uint32_t     sequence = 0;
        if (mutex_.take(config_.lock_timeout_ms)) {
            request  = requests_.take();
            sequence = pending_sequence_;
            mutex_.give();
        } else {
            lock_timeouts_.fetch_add(1);
        }
        const uint32_t forced = forced_stop_.load();
        if (forced != handled_stop_) {
            handled_stop_ = forced;
            owner_.disable();
        }
        if (sequence < handled_stop_) {
            request = DriveRequest{}; // sent before the stop
        }
        apply(request, owner_);
        owner_.step(now_());

        DriveSnapshot snapshot;
        snapshot.motion = owner_.motion();
        snapshot.drive  = owner_.drive();
        snapshot.mode   = owner_.mode();
        if (mutex_.take(config_.lock_timeout_ms)) {
            requests_.publish(snapshot);
            mutex_.give();
        } else {
            lock_timeouts_.fetch_add(1);
        }
        pros::Task::delay_until(&wake, config_.period_ms);
    }
    running_.store(false);
}

} // namespace actugatr
