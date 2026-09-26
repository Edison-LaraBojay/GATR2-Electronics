// fake_pi.h
// Host-only stand-in for Navigatr's brain_link slots: the brain link v3
// session, dedupe, placement and selection rules over the real common codec,
// with a scripted robot state and landmark table. No timing; FakeBus adds it.

#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "common/frame_codec.h"

namespace communigatr
{

struct FakeLandmark {
    uint8_t  source       = gatr2::kLandmarkSourceObserved;
    int32_t  x_mm         = 0; // physical landmark pose, field frame
    int32_t  y_mm         = 0;
    int32_t  heading_cdeg = 0;
    uint16_t age_ms       = 0; // observed only
};

class FakePi {
public:
    explicit FakePi(uint32_t pi_instance = 0x0BADF00D);

    // Request bytes in, one reply frame per decoded request out.
    std::vector<std::vector<uint8_t>> receive(const uint8_t* data, std::size_t len);

    // One request under the Pi's rules. Tests may call it directly.
    gatr2::BrainReply answer(const gatr2::BrainRequest& request);

    // Process restart: new instance, no session, anchor and selection lost.
    void restart(uint32_t pi_instance);

    // Robot part of the GET_STATE block; landmark fields come from the table.
    gatr2::BrainState& robot() { return robot_; }

    // Mapped wire ids. Unmapped ids answer kResultUnknownLandmark.
    void setLandmark(uint8_t id, const FakeLandmark& landmark) { landmarks_[id] = landmark; }
    void setWorldEstimationNoop(bool noop) { world_noop_ = noop; }

    // A new SET_POSE applies after this many later requests of any op:
    // 0 = before its own answer, negative = never.
    void setApplyDelay(int requests) { apply_delay_ = requests; }

    // Replies carry this version; any other than the request's answers
    // kResultUnsupportedVersion.
    void setVersion(uint8_t version) { version_ = version; }

    // Op answered kResultUnsupportedOp, 0 = none.
    void setUnsupportedOp(uint8_t op) { unsupported_op_ = op; }

    uint32_t piInstance() const { return pi_instance_; }
    uint32_t session() const { return session_; }
    int      sessionsOpened() const { return sessions_opened_; }
    int      placementsApplied() const { return placements_applied_; }
    bool     landmarkRequested() const { return object_requested_; }
    uint8_t  landmarkWireId() const { return object_wire_id_; }
    uint32_t objectSequence() const { return object_sequence_; }

    // Every decoded request, in order.
    const std::vector<gatr2::BrainRequest>& requests() const { return requests_; }

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
        gatr2::BrainRequest request;
        uint32_t            sequence = 0; // SET_POSE placement sequence
    };

    void     openSession(const gatr2::BrainRequest& request);
    void     apply(gatr2::BrainReply& reply, const gatr2::BrainRequest& request);
    void     answerPlacement(gatr2::BrainReply& reply, uint32_t sequence) const;
    void     answerSelect(gatr2::BrainReply& reply, const gatr2::BrainRequest& request) const;
    void     answerState(gatr2::BrainReply& reply) const;
    void     tickPlacement();
    void     applyPlacement();
    uint32_t nextSession();

    uint32_t pi_instance_;
    uint32_t rng_;
    uint8_t  version_        = gatr2::kBrainLinkVersion;
    uint8_t  unsupported_op_ = 0;

    // Session.
    uint32_t session_         = 0;
    uint32_t open_nonce_      = 0;
    uint16_t open_request_id_ = 0;
    bool     accepted_other_  = false;
    uint32_t nonce_ring_[4]   = {};
    int      nonce_count_     = 0;
    bool     have_newest_     = false;
    uint16_t newest_          = 0;
    Record   last_set_pose_;
    Record   last_select_;

    // Command state.
    uint32_t  init_sequence_ = 0;
    Placement placement_;
    uint32_t  applied_session_  = 0;
    uint32_t  applied_sequence_ = 0;
    bool      object_requested_ = false;
    uint8_t   object_wire_id_   = 0;
    uint32_t  object_sequence_  = 0;

    gatr2::BrainState               robot_;
    std::map<uint8_t, FakeLandmark> landmarks_;
    bool                            world_noop_  = false;
    int                             apply_delay_ = 0;

    gatr2::FrameReader               reader_;
    std::vector<gatr2::BrainRequest> requests_;
    int                              sessions_opened_    = 0;
    int                              placements_applied_ = 0;
};

} // namespace communigatr
