// differential_drive_sim.h
// Host-only tank drive model: first-order wheel lag, speed limit, and an
// optional robot origin offset from the turning center. Not a robot model.

#pragma once

#include "investigatr/drive.h"
#include "investigatr/geometry.h"

namespace investigatr
{

struct DifferentialDriveConfig {
    Meters  track_width         = 0.30;
    double  max_wheel_speed     = 1.2;  // m/s at output 1
    Seconds wheel_time_constant = 0.05; // 0 = no lag
    Pose    origin_offset;              // robot origin in the turning center frame
};

class DifferentialDriveSim {
public:
    explicit DifferentialDriveSim(const DifferentialDriveConfig& config = {},
                                  const Pose&                    start  = {});

    void step(const TankOutput& output, Seconds dt);

    // Robot origin, field frame.
    Pose pose() const;
    void setPose(const Pose& pose);

    Pose   centerPose() const { return center_; }
    double leftSpeed() const { return left_speed_; }
    double rightSpeed() const { return right_speed_; }

    // Cumulative wheel travel, m.
    Meters leftTravel() const { return left_travel_; }
    Meters rightTravel() const { return right_travel_; }

private:
    DifferentialDriveConfig config_;
    Pose                    center_;
    double                  left_speed_   = 0;
    double                  right_speed_  = 0;
    Meters                  left_travel_  = 0;
    Meters                  right_travel_ = 0;
};

} // namespace investigatr
