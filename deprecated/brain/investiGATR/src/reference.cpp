// reference.cpp

#include "investigatr/reference.h"

namespace investigatr
{

const char* toString(ResolveStatus status) {
    switch (status) {
    case ResolveStatus::kOk: return "ok";
    case ResolveStatus::kNoField: return "no field";
    case ResolveStatus::kMapMismatch: return "map mismatch";
    case ResolveStatus::kUnknownObject: return "unknown object";
    case ResolveStatus::kNotReference: return "not a reference";
    case ResolveStatus::kNotObserved: return "not observed";
    case ResolveStatus::kNoEstimate: return "no estimate";
    case ResolveStatus::kStale: return "estimate stale";
    case ResolveStatus::kFrameMismatch: return "frame mismatch";
    }
    return "?";
}

Resolved resolve(const Reference& reference, const Pose& relative, const Field* field,
                 const Pose& robot_at_start, FrameGeneration robot_frame,
                 const ReferencePolicy& policy, Seconds now) {
    Resolved out;
    switch (reference.kind) {
    case Reference::Kind::kOrigin:
        out.status      = ResolveStatus::kOk;
        out.reference   = Pose{};
        out.destination = compose(out.reference, relative);
        return out;
    case Reference::Kind::kRobotAtStart:
        out.status      = ResolveStatus::kOk;
        out.reference   = robot_at_start;
        out.destination = compose(out.reference, relative);
        return out;
    case Reference::Kind::kObject: break;
    }

    if (field == nullptr || field->generation == 0) {
        out.status = ResolveStatus::kNoField;
        return out;
    }
    if (reference.map != field->map.id) {
        out.status = ResolveStatus::kMapMismatch;
        return out;
    }
    const FieldObject* object = field->find(reference.object_id);
    if (object == nullptr) {
        out.status = ResolveStatus::kUnknownObject;
        return out;
    }
    if (!object->reference) {
        out.status = ResolveStatus::kNotReference;
        return out;
    }
    if (!object->valid || object->source == EstimateSource::kNone) {
        out.status = ResolveStatus::kNoEstimate;
        return out;
    }
    if (object->source == EstimateSource::kObserved && robot_frame != 0 &&
        field->frame != robot_frame) {
        out.status = ResolveStatus::kFrameMismatch;
        return out;
    }
    if (policy.require_observed && object->estimated &&
        object->source != EstimateSource::kObserved) {
        out.status = ResolveStatus::kNotObserved;
        return out;
    }
    if (object->source == EstimateSource::kObserved && policy.max_age > 0 && object->age_known &&
        object->age + (now - field->received_at) > policy.max_age) {
        out.status = ResolveStatus::kStale;
        return out;
    }
    out.status      = ResolveStatus::kOk;
    out.source      = object->source;
    out.reference   = object->pose;
    out.destination = compose(out.reference, relative);
    return out;
}

} // namespace investigatr
