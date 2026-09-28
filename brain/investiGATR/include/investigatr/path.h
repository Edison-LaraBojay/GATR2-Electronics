// path.h
// A planned path: typed segments a follower executes one at a time. A
// follower finishes each segment before starting the next, so it never cuts
// from one segment into the next.

#pragma once
#include <cstdint>
#include <vector>

#include "investigatr/geometry.h"

namespace investigatr
{

using CommandId = uint32_t; // 0 = none

enum class SegmentKind : uint8_t {
    kTurn,      // rotate in place at start from start.heading to end.heading
    kTranslate, // straight line from start to end
};

// kTranslate heading: non-holonomic keeps the travel direction (plus pi when
// reverse); holonomic moves linearly from start.heading to end.heading.
struct PathSegment {
    SegmentKind kind = SegmentKind::kTranslate;
    Pose        start; // field frame
    Pose        end;
    bool        reverse        = false; // non-holonomic translate, driving backward
    int         turn_direction = 0;     // kTurn: +1 CCW, -1 CW
    bool        exact = false; // swept exact footprint checked, not the circle (escape, approach)
};

enum class PlanMode : uint8_t { kDirect, kAvoiding };

struct Path {
    PlanMode                 mode = PlanMode::kDirect;
    std::vector<PathSegment> segments;

    Meters length() const; // translation only
    bool   empty() const { return segments.empty(); }
};

// Receives every new plan of a command for display. An empty path clears it.
class PathSink {
public:
    virtual ~PathSink() = default;

    virtual void reportPath(CommandId command, const Path& path) = 0;
};

} // namespace investigatr
