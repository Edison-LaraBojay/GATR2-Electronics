// source.cpp

#include "receiver/sources/source.h"

namespace spectagatr::receiver::sources
{

const char* portStateName(PortState state) {
    switch (state) {
    case PortState::kSearching:
        return "searching";
    case PortState::kOpen:
        return "open";
    case PortState::kNoBrainLinked:
        return "no_brain_linked";
    case PortState::kClosed:
        return "closed";
    case PortState::kStopped:
        return "stopped";
    }
    return "unknown";
}

} // namespace spectagatr::receiver::sources
