// odometry_source.h
// Dead reckoning InputSource from left/right travel and an optional absolute
// heading. No pose until align() places it in a known field frame.

#pragma once

#include "investigatr/input.h"

namespace investigatr
{

class OdometrySource : public InputSource {
public:
    // track_width: left to right measuring wheel spacing, used when no
    // heading is supplied.
    explicit OdometrySource(Meters track_width);

    // Sets the pose at time now and starts a new frame generation.
    void align(const Pose& pose, Seconds now);

    // Aligns from a usable snapshot no older than max_age. False, and no
    // change, for anything else.
    bool alignFrom(const InputSnapshot& snapshot, Seconds max_age, Seconds now);

    // Back to no pose, no frame.
    void invalidate();

    // Cumulative travel in meters, heading from any absolute heading sensor.
    void update(Seconds now, Meters left, Meters right);
    void update(Seconds now, Meters left, Meters right, Radians heading);

    bool            aligned() const { return aligned_; }
    Pose            pose() const { return pose_; }
    FrameGeneration frame() const { return aligned_ ? frame_ : 0; }

    void          request(const InputRequest& request) override;
    InputSnapshot latest(Seconds now) override;

private:
    void step(Seconds now, Meters left, Meters right, bool has_heading, Radians heading);

    Meters          track_width_;
    bool            aligned_     = false;
    Pose            pose_;
    Seconds         measured_at_ = 0;
    FrameGeneration frame_       = 0;
    InputRequest    request_;

    bool    have_travel_  = false;
    Meters  last_left_    = 0;
    Meters  last_right_   = 0;
    bool    have_heading_ = false;
    Radians last_heading_ = 0;
    bool    heading_ref_  = false;
    Radians heading_offset_ = 0;
};

} // namespace investigatr
