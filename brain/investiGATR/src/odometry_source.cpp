// odometry_source.cpp

#include "investigatr/odometry_source.h"

#include <algorithm>
#include <cmath>

namespace investigatr
{

OdometrySource::OdometrySource(Meters track_width) : track_width_(track_width) {}

void OdometrySource::align(const Pose& pose, Seconds now) {
    pose_        = pose;
    measured_at_ = now;
    aligned_     = true;
    frame_       = frame_ == UINT32_MAX ? 1 : frame_ + 1;
    heading_ref_ = have_heading_;
    if (heading_ref_) {
        heading_offset_ = wrapAngle(pose.heading - last_heading_);
    }
}

bool OdometrySource::alignFrom(const InputSnapshot& snapshot, Seconds max_age, Seconds now) {
    const RobotEstimate& robot = snapshot.robot;
    const bool fresh = robot.age >= 0 && robot.age <= max_age && std::isfinite(robot.pose.x) &&
                       std::isfinite(robot.pose.y) && std::isfinite(robot.pose.heading);
    if (!robot.valid || snapshot.frame == 0 || !snapshot.connected || !fresh) {
        return false;
    }
    align(robot.pose, now - robot.age);
    return true;
}

void OdometrySource::invalidate() {
    aligned_     = false;
    heading_ref_ = false;
}

void OdometrySource::update(Seconds now, Meters left, Meters right) {
    step(now, left, right, false, 0);
}

void OdometrySource::update(Seconds now, Meters left, Meters right, Radians heading) {
    step(now, left, right, true, heading);
}

void OdometrySource::step(Seconds now, Meters left, Meters right, bool has_heading,
                          Radians heading) {
    if (!have_travel_) {
        have_travel_ = true;
        last_left_   = left;
        last_right_  = right;
    }
    const Meters dl = left - last_left_;
    const Meters dr = right - last_right_;
    last_left_      = left;
    last_right_     = right;
    if (has_heading) {
        have_heading_ = true;
        last_heading_ = heading;
    }
    if (!aligned_) {
        return;
    }

    Radians turn = 0;
    if (has_heading && heading_ref_) {
        turn = wrapAngle(heading + heading_offset_ - pose_.heading);
    } else if (track_width_ > 0) {
        turn = (dr - dl) / track_width_;
    }
    const Meters  ds  = (dl + dr) / 2.0;
    const Radians mid = pose_.heading + turn / 2.0;
    pose_.x += ds * std::cos(mid);
    pose_.y += ds * std::sin(mid);
    pose_.heading = wrapAngle(pose_.heading + turn);
    measured_at_  = now;

    if (has_heading && !heading_ref_) {
        heading_ref_    = true;
        heading_offset_ = wrapAngle(pose_.heading - heading);
    }
}

void OdometrySource::request(const InputRequest& request) { request_ = request; }

InputSnapshot OdometrySource::latest(Seconds now) {
    InputSnapshot snapshot;
    if (request_.landmark) {
        snapshot.landmark.id     = request_.landmark_id;
        snapshot.landmark.status = LandmarkStatus::kUnsupported;
    }
    if (!aligned_) {
        return snapshot;
    }
    snapshot.frame       = frame_;
    snapshot.connected   = true;
    snapshot.robot.valid = true;
    snapshot.robot.pose  = pose_;
    snapshot.robot.age   = std::max(0.0, now - measured_at_);
    snapshot.link_age    = snapshot.robot.age;
    return snapshot;
}

} // namespace investigatr
