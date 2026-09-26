// driver.h
// investiGATR InputSource over a Client. Converts brain link state
// (mm, centidegrees, ms) to meters, radians and seconds, decides robot and
// landmark usability, and numbers coordinate frames. No I/O: whoever owns the
// client polls it.

#pragma once
#include <cstdint>

#include "communigatr/client.h"
#include "investigatr/input.h"

namespace communigatr
{

struct DriverConfig {
    // Also accept a Pi pose anchored by its <InitialPlacement>. Off: only
    // anchors set by a Brain SET_POSE make the robot valid.
    bool accept_configured_anchor = false;
};

class Driver : public investigatr::InputSource {
public:
    explicit Driver(Client& client, const DriverConfig& config = {});

    // Selects the landmark on the Pi. Ids 1..255 are wire ids; other ids
    // report kUnknownLandmark and release the selection.
    void request(const investigatr::InputRequest& request) override;

    // now: the clock the client is polled with.
    investigatr::InputSnapshot latest(Seconds now) override;

    // Field pose sent as SET_POSE. 0 when another placement is pending or the
    // pose is not finite or out of the wire range.
    PlacementTicket submitPlacement(const investigatr::Pose& pose);
    PlacementResult placementResult(PlacementTicket ticket) const;
    PlacementStatus placementStatus(PlacementTicket ticket) const;
    bool            placementPending() const;

    Client&             client() { return client_; }
    const Client&       client() const { return client_; }
    const DriverConfig& config() const { return config_; }

private:
    struct Identity {
        uint32_t pi_instance     = 0;
        uint32_t session         = 0;
        uint32_t odometry_epoch  = 0;
        uint32_t anchor_revision = 0;
    };

    investigatr::FrameGeneration  frameOf(const StateSample& sample);
    bool                          robotValid(const gatr2::BrainState& state) const;
    investigatr::LandmarkEstimate landmark(const StateSample& sample, bool connected,
                                           Seconds now) const;

    Client&                      client_;
    DriverConfig                 config_;
    investigatr::InputRequest    request_;
    Identity                     identity_;
    investigatr::FrameGeneration generation_ = 0; // last issued, 0 = none yet
};

} // namespace communigatr
