// inspection_state.h
// What the workers publish for read-only inspection, as immutable
// snapshots. Every struct here is assembled by the worker that owns the
// underlying state and handed over as a shared const pointer; the
// inspection service reads whatever is newest and never reaches into a
// worker's mutable maps.

#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "core/diagnostics.h"
#include "core/ids.h"
#include "core/records.h"
#include "core/time.h"
#include "payloads/camera_frames.h"
#include "payloads/field_object_evidence.h"
#include "payloads/tag_observations.h"
#include "state/pose_history.h"

namespace navigatr
{

// One camera frame the field worker processed, bound to its exact
// identity: the pixels, the detections decoded from those pixels, the
// association decisions for them, and the robot pose the history gave for
// the exposure instant. Overlays drawn from this must never be mixed with
// another frame's image.
struct DetectionFrameSnapshot {
    SensorId camera;                // producing sensor
    FrameId  engineering_frame;     // camera mounting frame, empty if none
    uint64_t frame_epoch    = 0;
    uint32_t frame_sequence = 0;

    MonotonicTime exposureAt;   // host clock
    MonotonicTime receivedAt;   // host clock
    MonotonicTime processedAt;  // host clock, field cycle that consumed it
    int64_t       exposure_uncertainty_ms = 0;
    bool          exposure_time_reliable  = true;

    int                                         width_px  = 0;
    int                                         height_px = 0;
    std::shared_ptr<const std::vector<uint8_t>> y8;   // may be null (injected frames)

    std::shared_ptr<const CameraIntrinsics> intrinsics;   // null when uncalibrated

    // Perception and association products for exactly this frame.
    bool                   has_observations = false;
    TagObservationSet      observations;
    bool                   has_trace = false;
    TagAssociationTraceSet trace;

    // Robot pose and attitude the history answered for the exposure time,
    // in the odometry frame, plus the anchor they were derived under.
    PoseLookupResult     pose_at_exposure;
    AttitudeLookupResult attitude_at_exposure;
    Pose2D               field_from_odom;
    uint64_t             anchor_revision = 0;

    uint64_t field_invocation = 0;   // the field cycle that produced this
};

// Health of one measurement record as the owning stage last left it.
struct SourceHealthEntry {
    std::string   kind;   // "resource", "sensor"
    std::string   id;     // resource_id.output_id or sensor_id
    SourceState   state = SourceState::kNoDataYet;
    std::string   diagnostic;
    std::string   payload;   // stable payload name of the latest sample
    bool          has_sample = false;
    MonotonicTime measuredAt;
    MonotonicTime receivedAt;
    MonotonicTime lastPolledAt;
    uint64_t      sequence = 0;
    uint64_t      epoch    = 0;
    std::string   upstream_source;
    std::string   upstream_clock;
    uint64_t      upstream_sequence = 0;
    uint64_t      upstream_epoch    = 0;
};

struct SourceHealthSnapshot {
    MonotonicTime                  at;   // host clock of the cycle
    uint64_t                       cycle = 0;
    std::vector<SourceHealthEntry> entries;
};

// Diagnostics copied out of a worker at the end of its cycle.
struct DiagnosticsSnapshot {
    Diagnostics estimation;   // resources, sensors, commands, localization, targets, publishing
    Diagnostics field;        // field estimation and its children
};

} // namespace navigatr
