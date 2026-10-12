// tag_association_trace.h
// The tag mount association's decision made visible: every decoded tag with
// its candidates, the accepted mount or the reason it was rejected, for
// inspection. It never feeds estimation.

#pragma once
#include <cstddef>
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
constexpr const char* kTagAssociationTraceSet = "association.tag_association_trace_set";
} // namespace payload_names

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
