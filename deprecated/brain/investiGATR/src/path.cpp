// path.cpp

#include "investigatr/path.h"

#include <cmath>

namespace investigatr
{

Meters Path::length() const {
    Meters total = 0;
    for (const PathSegment& s : segments) {
        if (s.kind == SegmentKind::kTranslate) {
            total += std::hypot(s.end.x - s.start.x, s.end.y - s.start.y);
        }
    }
    return total;
}

} // namespace investigatr
