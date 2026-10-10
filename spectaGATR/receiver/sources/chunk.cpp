// chunk.cpp

#include "receiver/sources/chunk.h"

namespace spectagatr::receiver::sources
{

const char* sourceKindName(SourceKind kind) {
    switch (kind) {
    case SourceKind::kNone:
        return "none";
    case SourceKind::kBrainUsb:
        return "brain";
    case SourceKind::kController:
        return "controller";
    case SourceKind::kSim:
        return "sim";
    case SourceKind::kFile:
        return "file";
    }
    return "unknown";
}

} // namespace spectagatr::receiver::sources
