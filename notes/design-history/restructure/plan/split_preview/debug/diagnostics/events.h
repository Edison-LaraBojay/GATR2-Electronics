// events.h

#pragma once
#include <cstdint>
#include <string>

#include "core/time.h"

namespace navigatr
{

// A lifecycle event worth a line in a log or the inspection page.
struct RuntimeEvent {
    uint64_t      sequence = 0;   // counts every event since build
    MonotonicTime at;             // host clock
    std::string   text;
};

} // namespace navigatr
