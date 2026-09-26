// input.h
// What a localization source hands the Navigator. Every estimate in one
// snapshot shares the field frame named by its generation.

#pragma once
#include <cstdint>

#include "investigatr/geometry.h"

namespace investigatr
{

using LandmarkId      = uint16_t; // 0 is never a valid landmark
using FrameGeneration = uint32_t; // 0 = no frame, unique per source instance

struct RobotEstimate {
    bool    valid = false;
    Pose    pose;    // robot origin, field frame
    Seconds age = 0; // measurement age at the snapshot time
};

enum class LandmarkSource : uint8_t { kNone, kNominal, kObserved };

enum class LandmarkStatus : uint8_t {
    kNotRequested,
    kPending,
    kAvailable,
    kUnknownLandmark,
    kUnsupported,
    kUnavailable,
    kStale,
};

struct LandmarkEstimate {
    LandmarkId     id     = 0; // the requested id, 0 when none requested
    LandmarkStatus status = LandmarkStatus::kNotRequested;
    LandmarkSource source = LandmarkSource::kNone;
    Pose           pose;       // physical landmark pose, field frame
    bool           age_known = false;
    Seconds        age       = 0; // observation age at the snapshot time
};

struct InputRequest {
    bool       landmark    = false;
    LandmarkId landmark_id = 0;
};

struct InputSnapshot {
    FrameGeneration  frame     = 0;
    bool             connected = false;
    Seconds          link_age  = 0;
    RobotEstimate    robot;
    LandmarkEstimate landmark;
};

class InputSource {
public:
    virtual ~InputSource() = default;

    // Data the active command needs. Nonblocking, idempotent.
    virtual void request(const InputRequest& request) = 0;

    // Newest available data, ages relative to now. Nonblocking.
    virtual InputSnapshot latest(Seconds now) = 0;
};

} // namespace investigatr
