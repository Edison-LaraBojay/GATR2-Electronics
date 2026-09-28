// link_events.h
// Recovery history for status displays: turns successive link snapshots into
// timestamped events (link lost and restored with the outage, new session,
// Pi restart, profile applied or rejected, placement lost or set, Pico link
// and IMU changes, calibration progress). Sessions and Pi instances compare
// with the last nonzero ones and state fields with the last state reply, so
// the gap while the client has no session hides nothing. Observation only;
// it never acts.

#pragma once
#include <cstddef>
#include <cstdint>

#include "communigatr/client.h"

namespace communigatr
{

// What a program sees of the link at one moment.
struct LinkSnapshot {
    bool        connected       = false;
    uint32_t    session         = 0;
    uint32_t    pi_instance     = 0;
    ProfileSync profile         = ProfileSync::kNone;
    bool        state_valid     = false; // a state reply of this session exists
    bool        localized       = false;
    uint8_t     health          = 0; // translagatr::HealthBit
    uint8_t     calibration     = translagatr::kCalibrationNone;
    uint32_t    odometry_epoch  = 0;
    uint32_t    anchor_revision = 0;
};

// Fills a snapshot from a client, for programs that hold one.
LinkSnapshot snapshotOf(const Client& client, Seconds now);

enum class LinkEvent : uint8_t {
    kLinkLost,
    kLinkRestored,
    kNewSession,
    kPiRestarted,
    kProfileApplied,
    kProfileRejected,
    kPlaced,
    kPlacementLost,
    kOdometryReset,
    kPicoLost,
    kPicoRestored,
    kImuFailed,
    kImuReady,
    kCalibrating,
    kCalibrated,
    kCalibrationFailed,
};

const char* toString(LinkEvent event);

struct LinkEventRecord {
    Seconds   at       = 0;
    LinkEvent event    = LinkEvent::kLinkLost;
    Seconds   duration = 0; // kLinkRestored: the outage
};

class LinkEvents {
public:
    static constexpr std::size_t kCapacity = 16;

    void update(const LinkSnapshot& snapshot, Seconds now);

    // Newest first; i < size().
    std::size_t            size() const { return count_; }
    const LinkEventRecord& at(std::size_t i) const;
    uint32_t               total() const { return total_; }

private:
    void add(LinkEvent event, Seconds now, Seconds duration = 0);
    void compareStates(const LinkSnapshot& before, const LinkSnapshot& s, Seconds now);

    bool            connected_   = false;
    bool            lost_        = false;
    Seconds         lost_at_     = 0;
    uint32_t        session_     = 0; // last nonzero
    uint32_t        pi_instance_ = 0; // last nonzero
    ProfileSync     profile_     = ProfileSync::kNone;
    bool            have_state_  = false;
    LinkSnapshot    state_; // last snapshot with a state reply
    LinkEventRecord ring_[kCapacity];
    std::size_t     next_  = 0;
    std::size_t     count_ = 0;
    uint32_t        total_ = 0;
};

} // namespace communigatr
