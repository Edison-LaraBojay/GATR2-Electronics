// payloads/field_object_evidence.h
// Generic field-object pose evidence: an association's statement that a
// specific configured feature of a specific configured field object implied
// this object pose. Any associator may emit it and field estimation folds
// it without knowing which sensor or geometry produced it.
//
// The implied pose names its frame explicitly (currently the odometry
// frame). Provenance identifies the source sensor and its frame/sequence so
// consumers can deduplicate per exposure; navigation intent (generations,
// preferred sources, allowed features) is deliberately absent - filtering
// by intent belongs to target resolution.
//
// The trace set is the same decision made visible: every decoded tag with
// its candidates, the accepted mount or the reason it was rejected, for
// inspection. It never feeds estimation.

#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "core/ids.h"
#include "core/time.h"
#include "math/transforms.h"
#include "state/pose_history.h"

namespace navigatr
{

namespace payload_names
{
constexpr const char* kFieldObjectPoseEvidenceSet =
    "association.field_object_pose_evidence_set";
constexpr const char* kTagAssociationTraceSet = "association.tag_association_trace_set";
} // namespace payload_names

struct FieldObjectPoseEvidence {
    FieldObjectId object;             // configured field object
    std::string   feature_instance;   // configured feature that won, e.g. a tag mount
    FrameId       frame;              // frame of the implied pose
    Pose2D        T_frame_object;
    MonotonicTime measuredAt;         // measurement/exposure, host monotonic
    SensorId      source;             // producing sensor
    uint32_t      source_sequence = 0;   // source frame identity for dedupe
    uint64_t      odometry_epoch  = 0;   // epoch the pose was expressed in
    double        confidence      = 0.0;
    bool          attitude_assumed = false;   // tilt at exposure was assumed level
};

struct FieldObjectPoseEvidenceSet {
    std::vector<FieldObjectPoseEvidence> entries;
};

struct TagAssociationCandidate {
    FieldObjectId object;
    std::string   mount;
    Pose2D        implied;   // T_odom_object
    double        translation_error_m = 0.0;
    double        heading_error_rad   = 0.0;
    double        score               = 0.0;
};

struct TagAssociationTrace {
    std::size_t tag_index   = 0;   // index into the observation set's tags
    std::string family;
    int         observed_id = -1;
    bool        accepted    = false;
    std::string rejection;   // empty when accepted
    FieldObjectId object;    // accepted object
    std::string   mount;     // accepted mount
    std::vector<TagAssociationCandidate> candidates;
};

struct TagAssociationTraceSet {
    SensorId      camera;
    uint32_t      frame_sequence = 0;
    uint64_t      frame_epoch    = 0;
    MonotonicTime exposureAt;
    std::string   frame_note;   // whole-frame rejection, e.g. no pose at exposure
    std::vector<TagAssociationTrace> tags;

    // Preserve the exact lookup used by association for frame inspection.
    bool has_exposure_context = false;
    PoseSampleLookupResult exposure_context;
    Pose2D field_from_odom;
    uint64_t anchor_revision = 0;
};

} // namespace navigatr
