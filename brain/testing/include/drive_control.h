// drive_control.h
// Drivetrain owner for the testing application. Its task runs every 10 ms and
// is the only code that writes the drive motors: the Navigator's demand in
// kNavigate, the latest manual demand in kManual, zero in kDisabled. Every
// call takes one mutex and returns at once; stop() also stops the motors
// immediately. Motors and tuning come from robot_config.h.

#pragma once
#include <cstdint>
#include <memory>

#include "api.h"
#include "investigatr/navigator.h"

enum class DriveMode : uint8_t { kDisabled, kNavigate, kManual };

class DriveControl {
public:
    // Creates the motors and the Navigator and starts the task. now: the clock
    // the source is polled with. Keep the object for the life of the program.
    DriveControl(investigatr::InputSource& source, investigatr::Seconds (*now)());
    DriveControl(const DriveControl&)            = delete;
    DriveControl& operator=(const DriveControl&) = delete;

    // Navigator commands. Each switches to kNavigate and replaces any command.
    investigatr::CommandId goTo(const investigatr::Pose&          pose,
                                const investigatr::MotionOptions& options = {});
    investigatr::CommandId goToRelative(investigatr::LandmarkId           landmark,
                                        const investigatr::Pose&          offset,
                                        const investigatr::MotionOptions& options = {});
    investigatr::CommandId follow(const investigatr::Path&          path,
                                  const investigatr::MotionOptions& options = {});

    // Manual demand in robot terms. Switches to kManual and cancels navigation.
    void manual(const investigatr::DriveCommand& demand);

    // Cancels navigation, clears the manual demand, stops the motors now.
    void stop();

    investigatr::MotionStatus status() const;
    DriveMode                 mode() const;

private:
    void run();
    void apply(const investigatr::TankOutput& tank);

    investigatr::Seconds (*now_)();
    pros::MotorGroup            left_;
    pros::MotorGroup            right_;
    investigatr::Navigator      navigator_;
    DriveMode                   mode_ = DriveMode::kDisabled;
    investigatr::DriveCommand   manual_;
    mutable pros::Mutex         mutex_;
    std::unique_ptr<pros::Task> task_;
};
