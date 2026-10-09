// field_source.h
// Brain-owned field geometry, independent of the robot pose input. Static
// maps and field updates share this interface; a localizer does not need it.
// Geometry uses the same field coordinates as the pose source. If a provider
// includes observed estimates, their frame identities and received_at times
// must use that pose source's frame namespace and the controller's clock.

#pragma once

#include "investigatr/field.h"

namespace investigatr
{

class FieldSource {
public:
    virtual ~FieldSource() = default;

    // Nonblocking. Copies the newest complete field into out unless out
    // already holds that generation. True when out holds a complete field
    // afterwards. A false return leaves out unchanged; consumers can retain
    // their last complete snapshot. Generations belong to this source.
    virtual bool field(Field& out) = 0;
};

} // namespace investigatr
