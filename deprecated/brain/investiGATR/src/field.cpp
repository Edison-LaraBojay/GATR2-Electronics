// field.cpp

#include "investigatr/field.h"

#include <algorithm>

namespace investigatr
{

const FieldObject* Field::find(ObjectId id) const {
    const auto it = std::lower_bound(objects.begin(), objects.end(), id,
                                     [](const FieldObject& o, ObjectId v) { return o.id < v; });
    if (it == objects.end() || it->id != id) {
        return nullptr;
    }
    return &*it;
}

void boxCorners(const Box& box, const Pose& owner, Point corners[4]) {
    const Pose   center = compose(owner, box.center);
    const double hl     = 0.5 * box.length;
    const double hw     = 0.5 * box.width;
    const Pose   local[4] = {{hl, -hw, 0}, {hl, hw, 0}, {-hl, hw, 0}, {-hl, -hw, 0}};
    for (int i = 0; i < 4; ++i) {
        const Pose p = compose(center, local[i]);
        corners[i]   = Point{p.x, p.y};
    }
}

} // namespace investigatr
