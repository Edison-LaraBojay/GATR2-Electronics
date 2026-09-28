// state_source.h
// What a robot state and field source hands planning and control. Source
// independent: communiGATR implements it over the Pi link, simulations and
// fallbacks implement it directly. Both calls are nonblocking.

#pragma once
#include <cstdint>

#include "investigatr/field.h"
#include "investigatr/geometry.h"

namespace investigatr
{

// Why a robot pose is or is not usable, in the order a source checks.
enum class RobotStatus : uint8_t {
    kValid,       // placed pose in frame
    kNoLink,      // source not connected
    kNoProfile,   // localizer has not accepted the robot configuration
    kCalibrating, // IMU calibration running, robot must hold still
    kUnplaced,    // no field placement since the localizer started or reset
    kNoPose,      // localizer has no current pose, e.g. sensors missing
};

const char* toString(RobotStatus status);

struct RobotState {
    RobotStatus     status = RobotStatus::kNoLink;
    Pose            pose;          // robot origin, field frame; only with kValid
    Seconds         age   = 0;     // measurement age at the query time
    FrameGeneration frame = 0;     // field frame of pose, 0 without kValid
    bool            connected = false;
    Seconds         link_age  = 0; // since the last good exchange

    bool valid() const { return status == RobotStatus::kValid; }
};

class StateSource {
public:
    virtual ~StateSource() = default;

    // Newest robot state, ages relative to now.
    virtual RobotState robot(Seconds now) = 0;

    // Copies the newest complete field into out unless out already holds that
    // generation. True when out holds a complete field afterwards.
    virtual bool field(Field& out) = 0;
};

} // namespace investigatr
