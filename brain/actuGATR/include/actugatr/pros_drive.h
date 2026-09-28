// pros_drive.h
// The drive task: the only code that touches the DriveOwner and writes the
// drive motors. Application calls post a request (DriveRequests, latest
// wins) and read the last published snapshot under a mutex held only to
// copy; planning, path reports and Pi link reads all run in the drive task.
//
// PROS deletes competition tasks at mode changes without releasing mutexes
// they hold. Every take is bounded. When the task cannot take the mutex it
// keeps stepping the owner without new requests and counts a lock timeout,
// so the current command still runs closed loop, and a manual demand goes
// stale and stops. A stop that cannot take the mutex is carried by an atomic
// instead, ordered against posted requests by a sequence number.
// Source in pros/, PROS builds only.

#pragma once
#include <atomic>
#include <cstdint>
#include <memory>

#include "actugatr/drive_requests.h"
#include "pros/rtos.hpp"

namespace actugatr
{

struct ProsDriveConfig {
    uint32_t period_ms       = 10;
    uint32_t task_priority   = TASK_PRIORITY_DEFAULT + 1;
    uint32_t lock_timeout_ms = 5;
};

class ProsDrive {
public:
    // owner and drive must outlive this object; now is the clock the state
    // source uses (communigatr::ProsLink::now).
    ProsDrive(DriveOwner& owner, Drive& drive, Seconds (*now)(), const ProsDriveConfig& config = {});
    // Ends the task (bounded wait) and stops the drive.
    ~ProsDrive();
    ProsDrive(const ProsDrive&)            = delete;
    ProsDrive& operator=(const ProsDrive&) = delete;

    // The command id, 0 when the mutex is busy. The command starts at the
    // next task step; until then status() shows it waiting.
    CommandId goToDirect(const Pose& destination, const Reference& relative_to = Reference::origin(),
                         const MoveOptions& options = {});
    CommandId goToAvoiding(const Pose&        destination,
                           const Reference&   relative_to = Reference::origin(),
                           const MoveOptions& options     = {});

    // False when the mutex is busy.
    bool manual(const ManualDemand& demand);

    // Cancels navigation and disables at the next task step, even when the
    // mutex is busy (then false).
    bool cancel();
    void disable() { cancel(); }

    // Last published snapshot. False (out unchanged) when the mutex is busy.
    bool     status(DriveSnapshot& out) const;
    uint32_t lockTimeouts() const { return lock_timeouts_.load(); }

private:
    void run();

    DriveOwner&         owner_;
    Drive&              drive_;
    Seconds             (*now_)();
    ProsDriveConfig     config_;
    mutable pros::Mutex mutex_;

    // Under mutex_.
    DriveRequests requests_;
    uint32_t      pending_sequence_ = 0; // of the request in requests_

    std::atomic<uint32_t> sequence_{0};
    std::atomic<uint32_t> forced_stop_{0}; // sequence of a stop sent without the mutex
    uint32_t              handled_stop_ = 0; // task only
    std::atomic<uint32_t> lock_timeouts_{0};
    std::atomic<bool>     quit_{false};
    std::atomic<bool>     running_{true};

    std::unique_ptr<pros::Task> task_;
};

} // namespace actugatr
