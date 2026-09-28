// state_source.cpp

#include "investigatr/state_source.h"

namespace investigatr
{

const char* toString(RobotStatus status) {
    switch (status) {
    case RobotStatus::kValid: return "valid";
    case RobotStatus::kNoLink: return "no link";
    case RobotStatus::kNoProfile: return "no robot profile";
    case RobotStatus::kCalibrating: return "calibrating";
    case RobotStatus::kUnplaced: return "placement required";
    case RobotStatus::kNoPose: return "no pose";
    }
    return "?";
}

} // namespace investigatr
