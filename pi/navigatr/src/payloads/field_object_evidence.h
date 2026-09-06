// payloads/field_object_evidence.h
// Generic field-object pose evidence: an association's statement that a
// specific configured feature of a specific configured field object implied
// this object pose. Any associator may emit it - AprilTag mounts today, a
// line-pair match-loader associator tomorrow - and field estimation folds
// it without knowing which sensor or geometry produced it.
//
// The implied pose names its frame explicitly (currently the odometry
// frame). Provenance identifies the source sensor and its frame/sequence so
// consumers can deduplicate per exposure; navigation intent (generations,
// preferred sources, allowed features) is deliberately absent - filtering
// by intent belongs to target resolution.

#pragma once
#include <string>
#include <vector>

#include "core/ids.h"
#include "core/time.h"
#include "math/transforms.h"

namespace navigatr
{

namespace payload_names
{
constexpr const char* kFieldObjectPoseEvidenceSet =
    "association.field_object_pose_evidence_set";
} // namespace payload_names

struct FieldObjectPoseEvidence {
    FieldObjectId object;             // configured field object
    std::string   feature_instance;   // configured feature that won, e.g. a tag mount
    FrameId       frame;              // frame of the implied pose
    Pose2D        T_frame_object;
    MonotonicTime measuredAt;         // measurement/exposure, host monotonic
    SensorId      source;             // producing sensor
    uint32_t      source_sequence = 0;   // source frame identity for dedupe
    double        confidence      = 0.0;
};

struct FieldObjectPoseEvidenceSet {
    std::vector<FieldObjectPoseEvidence> entries;
};

} // namespace navigatr
