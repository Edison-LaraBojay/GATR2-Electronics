// drive_requests.h
// Application requests to the drive task and the status the application
// reads back. The latest request wins: a goTo, a manual demand or a stop
// replaces one the task has not taken yet. goTo ids are handed out here and
// become the Motion command ids. Not synchronized: ProsDrive guards it with
// a mutex held only to copy, and only the drive task calls take(), apply()
// and publish().

#pragma once
#include <cstdint>

#include "actugatr/drive_owner.h"

namespace actugatr
{

struct DriveSnapshot {
    MotionStatus motion;
    DriveStatus  drive;
    DriveMode    mode = DriveMode::kDisabled;
};

struct DriveRequest {
    enum class Kind : uint8_t { kNone, kGoTo, kManual, kStop };

    Kind         kind = Kind::kNone;
    CommandId    id   = 0; // kGoTo
    PlanMode     mode = PlanMode::kDirect;
    Pose         destination;
    Reference    relative_to;
    MoveOptions  options;
    ManualDemand demand; // kManual
    Seconds      at = 0; // kManual, when the application sent it
};

class DriveRequests {
public:
    CommandId goTo(PlanMode mode, const Pose& destination, const Reference& relative_to,
                   const MoveOptions& options);
    void      manual(const ManualDemand& demand, Seconds now);
    void      stop();

    // Drive task: the pending request, cleared.
    DriveRequest take();
    void         publish(const DriveSnapshot& snapshot) { published_ = snapshot; }

    // The last published snapshot; a goTo the task has not taken yet shows
    // as waiting under its id, so the caller never sees it as idle.
    DriveSnapshot snapshot() const;

private:
    DriveRequest  pending_;
    DriveSnapshot published_;
    CommandId     last_id_ = 0;
};

// Drive task: hands a taken request to the owner.
void apply(const DriveRequest& request, DriveOwner& owner);

// A command that has not ended, including one requested but not yet taken.
bool moving(const DriveSnapshot& snapshot);

// Operator loop rule for manual input. Sticks off center always take over.
// Centered sticks send a zero demand only while nothing moves, and never in
// the cycle that requested a command: the demand would replace that request
// (latest wins) or cancel it.
bool sendManual(const ManualDemand& demand, bool requested_this_cycle,
                const DriveSnapshot& snapshot);

} // namespace actugatr
