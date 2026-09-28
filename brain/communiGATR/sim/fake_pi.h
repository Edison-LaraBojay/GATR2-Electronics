// fake_pi.h
// Host-only stand-in for Navigatr's brain_link slots, v4: session and dedupe
// rules, placement, robot profile staging and apply, field map and estimate
// documents, calibration control, path reports and telemetry over the real
// codec, with a scripted robot state. No timing; FakeBus and FakeUsb add it.

#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "translaGATR/frame_codec.h"
#include "translaGATR/link_documents.h"

namespace communigatr
{

struct FakeField {
    uint16_t                              revision = 1;
    int32_t                               min_x_mm = 0;
    int32_t                               min_y_mm = 0;
    int32_t                               max_x_mm = 3658;
    int32_t                               max_y_mm = 3658;
    std::vector<translagatr::FieldObjectRecord> objects; // sorted by id
};

// count objects with ids 10, 20, ...: landmarks (estimated, reference,
// obstacle with an offset rotated box) alternating with fixed obstacles.
FakeField makeFakeField(uint16_t count, uint16_t revision = 1);

// Latest PATH_REPORT of the current session.
struct FakePath {
    bool                          have       = false;
    uint32_t                      command_id = 0;
    uint8_t                       mode       = translagatr::kPathNone;
    std::vector<translagatr::PathPoint> points;
};

class FakePi {
public:
    explicit FakePi(uint32_t pi_instance = 0x0BADF00D);

    // Request bytes in, one reply frame per decoded request out.
    std::vector<std::vector<uint8_t>> receive(const uint8_t* data, std::size_t len);

    // One request under the Pi's rules. Tests may call it directly.
    translagatr::BrainReply answer(const translagatr::BrainRequest& request);

    // Process restart: new instance, no session, no placement, no profile or
    // staging, estimate ids from 1 again with a nominal estimate. The field
    // map comes from the configuration and stays.
    void restart(uint32_t pi_instance);

    // Robot part of the state block as localization reports it.
    translagatr::BrainState& robot() { return robot_; }

    // A new SET_POSE applies after this many later requests of any op:
    // 0 = before its own answer, negative = never.
    void setApplyDelay(int requests) { apply_delay_ = requests; }

    // Replies carry this version; any other than the request's answers
    // kResultUnsupportedVersion.
    void setVersion(uint8_t version) { version_ = version; }

    // Op answered kResultUnsupportedOp, 0 = none. kOpTelemetry is a Pi from
    // before TELEMETRY.
    void setUnsupportedOp(uint8_t op) { unsupported_op_ = op; }

    // --- Telemetry ---------------------------------------------------------
    // Result of every new TELEMETRY; kResultOk keeps it. Display only: it
    // never changes the robot, placement or profile.
    void setTelemetryResult(uint8_t result) { telemetry_result_ = result; }

    // Latest kept TELEMETRY body and how many were kept.
    const translagatr::BrainTelemetry& telemetry() const { return telemetry_; }
    int                                telemetryKept() const { return telemetry_kept_; }

    // --- Robot profile -----------------------------------------------------
    // On: localization waits for an applied Brain profile (no pose, SET_POSE
    // and CONTROL NotReady). Off: XML localization; PROFILE_APPLY answers
    // ProfileRejected with kProfileReasonNotAccepted.
    void setProfileMode(bool on) { profile_mode_ = on; }

    // Capability check result for every new profile id; kProfileReasonNone
    // accepts. Runs after decode and the shared check.
    void setProfileRejection(uint8_t reason, uint8_t detail = 0) {
        capability_reason_ = reason;
        capability_detail_ = detail;
    }

    // An accepted profile is swapped in at the boundary this many requests
    // after its APPLY; until then APPLY answers Pending.
    void setProfileApplyDelay(int requests) { profile_delay_ = requests; }

    // Staging taken over by another document id, as another writer would.
    void replaceStaging(uint32_t profile_id, uint16_t total_len) {
        staging_id_       = profile_id;
        staging_total_    = total_len;
        staging_received_ = 0;
    }

    uint32_t appliedProfile() const { return applied_profile_; }
    int      profilesApplied() const { return profiles_applied_; }
    uint16_t stagingReceived() const { return staging_received_; }
    int      profileWrites() const { return profile_writes_; }

    // --- Field documents ---------------------------------------------------
    // Configures the map and publishes a nominal estimate for it.
    void     setField(const FakeField& field);
    void     clearField();

    // Serves these bytes as the map, valid or not; the map id is their crc32.
    void setMapDocument(const std::vector<uint8_t>& doc);

    // Later estimates name this map id instead of the served one; 0 = served.
    void setEstimateMapId(uint32_t map_id) { estimate_map_id_ = map_id; }
    uint32_t mapId() const { return map_id_; }
    const std::vector<uint8_t>& mapDocument() const { return map_doc_; }

    // New estimate snapshot, records in map order; returns its id. The last
    // three stay readable.
    uint32_t publishEstimate(const std::vector<translagatr::FieldEstimateRecord>& records);
    uint32_t publishNominalEstimate();
    std::vector<translagatr::FieldEstimateRecord> nominalRecords() const;
    uint32_t newestEstimate() const { return estimates_.empty() ? 0 : estimates_.back().id; }

    // Edits every READ_DOC Ok reply, for chunk faults.
    std::function<void(translagatr::BrainReply&)> doc_hook;

    // --- Control -----------------------------------------------------------
    void setMoving(bool moving) { moving_ = moving; }

    // A control answers Pending for this many later requests (the Pico at
    // work), then completes; with a failure detail it then fails.
    void setControlPendingRequests(int requests) { control_delay_ = requests; }
    void setControlFailure(uint8_t detail) { control_failure_ = detail; }

    // Recalibration runs for this many requests, then reports done.
    void setCalibrationRequests(int requests) { calibration_requests_ = requests; }
    int  controlsExecuted() const { return controls_executed_; }

    // --- Observation -------------------------------------------------------
    uint32_t        piInstance() const { return pi_instance_; }
    uint32_t        session() const { return session_; }
    int             sessionsOpened() const { return sessions_opened_; }
    int             placementsApplied() const { return placements_applied_; }
    int             imuSamples() const { return imu_samples_; }
    const FakePath& path() const { return path_; }

    // READ_WHEELS answer.
    void setWheels(const std::vector<translagatr::WheelReading>& wheels) { wheels_ = wheels; }

    // Every decoded request, in order.
    const std::vector<translagatr::BrainRequest>& requests() const { return requests_; }

private:
    struct Placement {
        bool     pending   = false;
        uint32_t session   = 0;
        uint32_t sequence  = 0;
        int32_t  x_mm      = 0;
        int32_t  y_mm      = 0;
        int32_t  heading   = 0;
        int      countdown = 0;
    };

    struct Record {
        bool                have = false;
        translagatr::BrainRequest request;
        uint8_t             result      = translagatr::kResultOk; // CONTROL
        uint8_t             calibration = translagatr::kCalibrationNone;
        uint32_t            sequence    = 0; // SET_POSE placement sequence
        uint8_t             detail      = translagatr::kControlDetailNone;
    };

    struct Estimate {
        uint32_t             id = 0;
        std::vector<uint8_t> doc;
    };

    struct Rejection {
        uint32_t id     = 0;
        uint8_t  reason = 0;
        uint8_t  detail = 0;
    };

    void     openSession(const translagatr::BrainRequest& request);
    void     execute(translagatr::BrainReply& reply, const translagatr::BrainRequest& request, bool repeat);
    void     repeat(translagatr::BrainReply& reply, const translagatr::BrainRequest& request);
    void     setPose(translagatr::BrainReply& reply, const translagatr::BrainRequest& request);
    void     answerPlacement(translagatr::BrainReply& reply, uint32_t sequence) const;
    void     answerState(translagatr::BrainReply& reply) const;
    void     profileWrite(translagatr::BrainReply& reply, const translagatr::BrainRequest& request);
    void     profileApply(translagatr::BrainReply& reply, const translagatr::BrainRequest& request);
    void     rejectProfile(translagatr::BrainReply& reply, uint32_t id, uint8_t reason, uint8_t detail);
    void     readDoc(translagatr::BrainReply& reply, const translagatr::BrainRequest& request);
    void     control(translagatr::BrainReply& reply, const translagatr::BrainRequest& request);
    void     finishControl();
    void     readWheels(translagatr::BrainReply& reply) const;
    void     tick();
    void     applyPlacement();
    void     applyProfile();
    void     loseContinuity();
    bool     localizing() const { return !profile_mode_ || applied_profile_ != 0; }
    uint32_t nextSession();

    uint32_t pi_instance_;
    uint32_t rng_;
    uint8_t  version_        = translagatr::kBrainLinkVersion;
    uint8_t  unsupported_op_ = 0;

    // Session.
    uint32_t            session_         = 0;
    uint32_t            open_nonce_      = 0;
    uint16_t            open_request_id_ = 0;
    bool                accepted_other_  = false;
    uint32_t            nonce_ring_[4]   = {};
    int                 nonce_count_     = 0;
    bool                have_newest_     = false;
    translagatr::BrainRequest newest_;
    Record              last_set_pose_;
    Record              last_control_;

    // Placement.
    uint32_t  init_sequence_    = 0;
    Placement placement_;
    uint32_t  applied_session_  = 0;
    uint32_t  applied_sequence_ = 0;
    int       apply_delay_      = 0;

    // Profile.
    bool                   profile_mode_      = false;
    uint8_t                capability_reason_ = translagatr::kProfileReasonNone;
    uint8_t                capability_detail_ = 0;
    int                    profile_delay_     = 0;
    uint32_t               staging_id_        = 0;
    uint16_t               staging_total_     = 0;
    uint16_t               staging_received_  = 0;
    uint8_t                staging_[translagatr::kProfileMaxLen] = {};
    uint32_t               applied_profile_   = 0;
    uint32_t               applying_profile_  = 0;
    int                    applying_countdown_ = 0;
    uint8_t                profile_state_     = translagatr::kProfileNone;
    uint32_t               profile_id_        = 0;
    uint8_t                profile_reason_    = translagatr::kProfileReasonNone;
    uint8_t                profile_detail_    = 0;
    std::vector<Rejection> rejected_;
    int                    profiles_applied_  = 0;
    int                    profile_writes_    = 0;

    // Field.
    std::vector<uint8_t>  map_doc_;
    uint32_t              map_id_ = 0;
    std::vector<Estimate> estimates_; // oldest first, at most 3
    uint32_t              next_estimate_id_ = 1;
    uint32_t              estimate_map_id_  = 0;

    // Control.
    bool    moving_               = false;
    int     calibration_requests_ = 3;
    int     calibration_left_     = 0;
    int     controls_executed_    = 0;
    int     control_delay_        = 0;
    int     control_left_         = 0; // requests until the pending control completes
    uint8_t control_failure_      = translagatr::kControlDetailNone;

    std::vector<translagatr::WheelReading> wheels_;

    uint8_t                     telemetry_result_ = translagatr::kResultOk;
    translagatr::BrainTelemetry telemetry_;
    int                         telemetry_kept_ = 0;

    translagatr::BrainState                robot_;
    FakePath                         path_;
    translagatr::FrameReader               reader_;
    std::vector<translagatr::BrainRequest> requests_;
    int                              sessions_opened_    = 0;
    int                              placements_applied_ = 0;
    int                              imu_samples_        = 0;
};

} // namespace communigatr
