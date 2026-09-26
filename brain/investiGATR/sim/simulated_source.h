// simulated_source.h
// Host-only InputSource fed with truth. Pose latency and extra age, frame
// generation, link state, and a landmark table with status and source.

#pragma once
#include <deque>
#include <map>

#include "investigatr/input.h"

namespace investigatr
{

struct SimulatedLandmark {
    LandmarkStatus status    = LandmarkStatus::kAvailable;
    LandmarkSource source    = LandmarkSource::kObserved;
    Pose           pose;
    bool           age_known = true;
    Seconds        age       = 0;
};

class SimulatedSource : public InputSource {
public:
    // Truth sample at time now. latest() reports the newest sample at least
    // latency old, with age = now - sample time + extra age.
    void setRobot(const Pose& pose, Seconds now);
    void clearRobot();

    void setLatency(Seconds latency) { latency_ = latency; }
    void setExtraAge(Seconds extra) { extra_age_ = extra; }
    void setRobotValid(bool valid) { robot_valid_ = valid; }
    void setConnected(bool connected) { connected_ = connected; }
    void setLinkAge(Seconds age) { link_age_ = age; }
    void setFrame(FrameGeneration frame) { frame_ = frame; }

    // Ids missing from the table report kUnknownLandmark.
    void setLandmark(LandmarkId id, const SimulatedLandmark& landmark);
    void removeLandmark(LandmarkId id);

    const InputRequest& lastRequest() const { return request_; }
    int                 requestCount() const { return request_count_; }

    void          request(const InputRequest& request) override;
    InputSnapshot latest(Seconds now) override;

private:
    struct Sample {
        Seconds at = 0;
        Pose    pose;
    };

    std::deque<Sample>                      samples_;
    std::map<LandmarkId, SimulatedLandmark> landmarks_;
    InputRequest                            request_;
    int                                     request_count_ = 0;
    Seconds                                 latency_       = 0;
    Seconds                                 extra_age_     = 0;
    bool                                    robot_valid_   = true;
    bool                                    connected_     = true;
    Seconds                                 link_age_      = 0;
    FrameGeneration                         frame_         = 1;
};

} // namespace investigatr
