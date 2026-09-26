// simulated_source.cpp

#include "sim/simulated_source.h"

namespace investigatr
{

namespace
{

constexpr Seconds kHistory = 2.0;

} // namespace

void SimulatedSource::setRobot(const Pose& pose, Seconds now) {
    samples_.push_back(Sample{now, pose});
    while (samples_.size() > 1 && samples_[1].at <= now - latency_ - kHistory) {
        samples_.pop_front();
    }
}

void SimulatedSource::clearRobot() { samples_.clear(); }

void SimulatedSource::setLandmark(LandmarkId id, const SimulatedLandmark& landmark) {
    landmarks_[id] = landmark;
}

void SimulatedSource::removeLandmark(LandmarkId id) { landmarks_.erase(id); }

void SimulatedSource::request(const InputRequest& request) {
    request_ = request;
    ++request_count_;
}

InputSnapshot SimulatedSource::latest(Seconds now) {
    InputSnapshot snapshot;
    snapshot.frame     = frame_;
    snapshot.connected = connected_;
    snapshot.link_age  = link_age_;

    const Sample* newest = nullptr;
    for (const Sample& sample : samples_) {
        if (sample.at <= now - latency_) {
            newest = &sample;
        }
    }
    if (newest != nullptr) {
        snapshot.robot.valid = robot_valid_;
        snapshot.robot.pose  = newest->pose;
        snapshot.robot.age   = now - newest->at + extra_age_;
    }

    if (request_.landmark) {
        LandmarkEstimate& out = snapshot.landmark;
        out.id                = request_.landmark_id;
        const auto found      = landmarks_.find(request_.landmark_id);
        if (found == landmarks_.end()) {
            out.status = LandmarkStatus::kUnknownLandmark;
        } else {
            const SimulatedLandmark& in = found->second;
            out.status                  = in.status;
            if (in.status == LandmarkStatus::kAvailable) {
                out.source    = in.source;
                out.pose      = in.pose;
                out.age_known = in.age_known;
                out.age       = in.age;
            }
        }
    }
    return snapshot;
}

} // namespace investigatr
