// drive_owner.h
// The one writer of drive commands. Each step() applies the command of the
// current mode through Drive: Motion in kNavigate, the manual demand in
// kManual, a stop in kDisabled. A manual demand older than manual_timeout
// stops the drive. Every mode change that leaves navigation cancels the
// active command; nothing resumes on its own.

#pragma once
#include <cstdint>

#include "actugatr/drive.h"
#include "actugatr/motion.h"

namespace actugatr
{

enum class DriveMode : uint8_t { kDisabled, kManual, kNavigate };

const char* toString(DriveMode mode);

// Fractions of the manual limits in [-1, 1]. forward +x, strafe +y (left,
// ignored by drives that cannot move sideways), turn CCW.
struct ManualDemand {
    double forward = 0;
    double strafe  = 0;
    double turn    = 0;
};

struct DriveOwnerConfig {
    MetersPerSecond  manual_speed   = 1.0;
    RadiansPerSecond manual_omega   = 3.0;
    Seconds          manual_timeout = 0.25;
};

class DriveOwner {
public:
    DriveOwner(Motion& motion, Drive& drive, const DriveOwnerConfig& config = {});

    // Cancels navigation; the drive stops at the next step.
    void disable();

    // Switches to kManual and cancels navigation.
    void manual(const ManualDemand& demand, Seconds now);

    // Switch to kNavigate; see Motion.
    CommandId goToDirect(const Pose& destination, const Reference& relative_to = Reference::origin(),
                         const MoveOptions& options = {});
    CommandId goToAvoiding(const Pose&        destination,
                           const Reference&   relative_to = Reference::origin(),
                           const MoveOptions& options     = {});

    // Cancels navigation and disables.
    void cancel();

    void step(Seconds now);

    DriveMode           mode() const { return mode_; }
    const MotionStatus& motion() const { return motion_.status(); }
    const DriveStatus&  drive() const { return drive_.status(); }
    Motion&             motionControl() { return motion_; }

private:
    Motion&          motion_;
    Drive&           drive_;
    DriveOwnerConfig config_;
    DriveMode        mode_ = DriveMode::kDisabled;
    ManualDemand     manual_;
    Seconds          manual_at_ = 0;
};

} // namespace actugatr
