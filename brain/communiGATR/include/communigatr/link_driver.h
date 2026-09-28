// link_driver.h
// investiGATR StateSource and PathSink over a Client. Converts brain link
// state and field documents (mm, centidegrees, ms) to meters, radians and
// seconds, derives the robot status, and numbers field frames. No I/O: the
// owner polls the client, on the same clock it passes here.

#pragma once
#include <cstdint>

#include "communigatr/client.h"
#include "communigatr/readiness.h"
#include "communigatr/robot_profile.h"
#include "investigatr/path.h"
#include "investigatr/state_source.h"

namespace communigatr
{

struct LinkDriverConfig {
    // Also accept a Pi pose anchored by its configured initial placement.
    // Off: only anchors set by a Brain SET_POSE make the robot valid.
    bool accept_configured_anchor = false;
};

class LinkDriver : public investigatr::StateSource, public investigatr::PathSink {
public:
    explicit LinkDriver(Client& client, const LinkDriverConfig& config = {});

    // Status in this order: kNoLink (not connected), kNoProfile (configured
    // profile not applied), kCalibrating (Pi calibration running or waiting
    // for stillness or data), kUnplaced (not localized, anchor not accepted,
    // or a placement pending), kNoPose (no pose or unknown age), kValid.
    // Age: robot_age_ms plus the round trip plus time since the reply. The
    // frame changes with pi_instance, session, odometry_epoch and
    // anchor_revision.
    investigatr::RobotState robot(Seconds now) override;

    // The client's newest published map and estimate as one Field. Objects
    // carry observation ages at received_at; an estimate taken before the
    // session began has an infinite observed age. frame numbers the
    // estimate's anchor the same way robot() numbers the robot's.
    bool field(investigatr::Field& out) override;

    // Translation end points as the path report, best effort.
    void reportPath(investigatr::CommandId command, const investigatr::Path& path) override;

    // Field pose sent as SET_POSE, rounded to mm and centidegrees. 0 when the
    // client refuses it, or the pose is not finite or out of the wire range.
    PlacementTicket place(const investigatr::Pose& pose);

    // Configures the robot profile through Client::setProfile, same rules
    // and result, and keeps this SI copy whenever the client took it.
    bool                setProfile(const RobotProfile& profile);
    const RobotProfile& profile() const { return profile_; }
    bool                hasProfile() const { return has_profile_; }

    LinkReadiness readiness(Seconds now) const {
        return readinessOf(client_, now, config_.accept_configured_anchor);
    }

    Client&                 client() { return client_; }
    const Client&           client() const { return client_; }
    const LinkDriverConfig& config() const { return config_; }

private:
    struct Identity {
        uint32_t pi_instance     = 0;
        uint32_t session         = 0;
        uint32_t odometry_epoch  = 0;
        uint32_t anchor_revision = 0;
    };

    struct Numbered {
        Identity                     identity;
        investigatr::FrameGeneration generation = 0; // 0 = empty slot
    };

    investigatr::FrameGeneration generationOf(const Identity& identity);

    Client&                      client_;
    LinkDriverConfig             config_;
    RobotProfile                 profile_;
    bool                         has_profile_ = false;
    Numbered                     frames_[4]; // newest first
    investigatr::FrameGeneration last_generation_ = 0;
};

} // namespace communigatr
