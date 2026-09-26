// client.h
// Brain side of brain link v3: session, request scheduling, retries and reply
// correlation over a BytePort. One request outstanding at a time, priority
// placement > landmark selection > state poll. No clock and no threads:
// poll(now) does all I/O and never blocks.

#pragma once
#include <cstdint>
#include <functional>

#include "common/frame_codec.h"
#include "communigatr/byte_port.h"

namespace communigatr
{

using Seconds = double;

// Optional bench IMU uplink. Keep the acquisition stamp unchanged when reusing
// a sample; report invalid during calibration, disconnect or sensor errors.
struct BenchImuSample {
    bool valid = false;
    uint32_t stamp_ms = 0;
    int32_t rotation_mdeg = 0; // continuous rotation, CCW positive
};

struct ClientConfig {
    Seconds response_timeout   = 0.060; // from the write call's return
    Seconds request_gap        = 0.005; // after a reply or a timeout
    Seconds state_period       = 0.020; // GET_STATE poll period
    Seconds link_timeout       = 0.25;  // connected while a reply is this recent
    Seconds pending_retry      = 0.020; // SET_POSE resend after Pending
    int     placement_attempts = 10;    // sends per placement
    Seconds placement_deadline = 1.0;   // first send through confirming GET_STATE
    int     select_attempts    = 5;     // sends per SELECT transaction
    Seconds select_retry       = 0.5;   // before a new SELECT after a failed one
    Seconds hello_backoff      = 1.0;   // HELLO period after an unsupported version or op
    // Called once per state request in the driver task. Empty preserves v3
    // GET_STATE traffic exactly. Requires a Pi supporting the bench extension.
    std::function<BenchImuSample()> bench_imu;
};

using PlacementTicket = uint32_t; // 0 = none

enum class PlacementResult : uint8_t {
    kNone,        // not the latest ticket, or 0
    kPending,     // queued, in flight, or Ok and waiting for a state with its anchor
    kApplied,     // a state reply after the Ok shows the placement's anchor
    kRejected,    // error result, see PlacementStatus::result
    kTimedOut,    // attempts or deadline used up; outcome unknown, may still apply
    kSessionLost, // session ended while pending; never resent
};

struct PlacementStatus {
    PlacementTicket ticket          = 0;
    PlacementResult state           = PlacementResult::kNone;
    uint8_t         result          = gatr2::kResultOk; // last reply result
    uint32_t        odometry_epoch  = 0;                // from the Ok reply
    uint32_t        anchor_revision = 0;
};

enum class SelectionState : uint8_t {
    kNotRequested,
    kPending,         // not acknowledged, or no state reply since the ack
    kActive,          // acknowledged, and the latest state reply carries the id
    kUnknownLandmark, // settled until the wanted id or the session changes
    kUnsupported,     // settled; world estimation is noop on the Pi
};

enum class LinkError : uint8_t { kNone, kUnsupportedVersion, kUnsupportedOp };

// Latest GET_STATE Ok of the current session.
struct StateSample {
    bool              valid       = false;
    uint32_t          pi_instance = 0;
    uint32_t          session     = 0;
    gatr2::BrainState state;
    Seconds           received_at = 0;
    Seconds           round_trip  = 0; // this request's write to its reply
};

struct ClientStats {
    uint32_t requests       = 0; // frames written, resends included
    uint32_t resends        = 0; // byte-identical retries
    uint32_t replies        = 0; // correlated
    uint32_t timeouts       = 0;
    uint32_t uncorrelated   = 0; // valid replies matching no outstanding request
    uint32_t bad_frames     = 0; // complete frames that are not decodable replies
    uint32_t drained_bytes  = 0; // discarded before a send
    uint32_t read_errors    = 0;
    uint32_t write_errors   = 0;
    uint32_t sessions       = 0; // opened
    uint32_t session_losses = 0;
    uint32_t pi_restarts    = 0; // pi_instance changes
    uint32_t stale_hellos   = 0;
    uint32_t unexpected     = 0; // correlated replies with a result the op never gets
};

class Client {
public:
    // nonce: entropy for each new HELLO transaction. A 0 or a repeat of the
    // previous nonce is replaced by a deterministic step.
    Client(BytePort& port, std::function<uint32_t()> nonce, const ClientConfig& config = {});

    // Receives, times out, and sends at most one request. Call every few ms
    // with a steady clock; now also stands for the write call's return.
    void poll(Seconds now);

    // Field pose in wire units. One placement at a time: 0 while another is
    // pending. Only the latest ticket is tracked. Queued until a session opens;
    // the deadline starts at the first send, not at submission.
    PlacementTicket submitPlacement(int32_t x_mm, int32_t y_mm, int32_t heading_cdeg);
    PlacementResult placementResult(PlacementTicket ticket) const;
    PlacementStatus placementStatus(PlacementTicket ticket) const;
    bool            placementPending() const;

    // Wanted landmark wire id, 0 = none. Idempotent.
    void           selectLandmark(uint8_t id) { want_ = id; }
    uint8_t        wantedLandmark() const { return want_; }
    SelectionState selection() const;

    // Session open and a GET_STATE Ok received in it.
    bool ready() const { return ready_; }

    // Ready and a correlated reply within link_timeout.
    bool    connected(Seconds now) const;
    Seconds linkAge(Seconds now) const; // infinity without a session

    uint32_t           session() const { return session_; }        // 0 = none
    uint32_t           piInstance() const { return pi_instance_; } // recorded at HELLO Ok
    const StateSample& state() const { return state_; }

    // Last incompatibility; cleared when ready again.
    LinkError error() const { return error_; }
    uint8_t   peerVersion() const { return peer_version_; }

    const ClientStats&  stats() const { return stats_; }
    const ClientConfig& config() const { return config_; }

private:
    enum class Kind : uint8_t { kNone, kHello, kPlacement, kSelect, kState };

    struct Transaction {
        gatr2::BrainRequest request;
        uint8_t             frame[gatr2::kMaxFrameLen] = {};
        uint16_t            len                        = 0; // 0 = no transaction
        int                 attempts                   = 0;
        Seconds             first_sent                 = 0;
    };

    void         receive(Seconds now);
    void         handleFrame(const uint8_t* frame, uint16_t len, Seconds now);
    bool         correlates(const gatr2::BrainReply& reply) const;
    void         handleReply(const gatr2::BrainReply& reply, Seconds now);
    void         handleHello(const gatr2::BrainReply& reply, Seconds now);
    void         handlePlacement(const gatr2::BrainReply& reply, Seconds now);
    void         handleSelect(const gatr2::BrainReply& reply, Seconds now);
    void         handleState(const gatr2::BrainReply& reply, Seconds now, Seconds round_trip);
    void         incompatible(Kind kind, const gatr2::BrainReply& reply, Seconds now);
    void         attemptFailed(Seconds now);
    void         transmit(Seconds now);
    Kind         choose(Seconds now);
    Transaction& transaction(Kind kind);
    void         start(Transaction& tx, gatr2::BrainRequest request);
    void         drain();
    void         loseSession();
    void         settlePlacement(PlacementResult state, uint8_t result);
    bool         placementExhausted(Seconds now) const;
    bool         selectionNeeded() const;
    void         selectFailed(Seconds now);
    uint16_t     takeRequestId();
    uint32_t     takeNonce();

    BytePort&                 port_;
    std::function<uint32_t()> nonce_source_;
    ClientConfig              config_;
    gatr2::FrameReader        reader_;
    ClientStats               stats_;

    // Exchange.
    Transaction         hello_;
    Transaction         placement_tx_;
    Transaction         select_tx_;
    Transaction         state_tx_;
    Kind                outstanding_ = Kind::kNone;
    gatr2::BrainRequest sent_; // the outstanding request
    Seconds             sent_at_         = 0;
    Seconds             next_send_       = 0;
    Seconds             next_hello_      = 0;
    Seconds             next_state_      = 0;
    uint16_t            next_request_id_ = 1;
    uint32_t            last_nonce_      = 0;

    // Session.
    uint32_t    session_     = 0;
    uint32_t    pi_instance_ = 0;
    bool        ready_       = false;
    Seconds     last_reply_  = 0;
    StateSample state_;
    uint32_t    state_count_  = 0; // GET_STATE Ok replies in this session
    LinkError   error_        = LinkError::kNone;
    uint8_t     peer_version_ = 0;

    // Placement.
    PlacementStatus     placement_;
    PlacementTicket     last_ticket_ = 0;
    gatr2::BrainRequest placement_request_;
    bool                placement_acked_ = false;
    uint32_t            placement_mark_  = 0; // state_count_ at the Ok
    Seconds             next_placement_  = 0;
    bool                bench_poll_before_retry_ = false;

    // Selection. pi_selection_ is what the Pi recorded for this session.
    uint8_t  want_               = 0;
    uint8_t  pi_selection_       = 0;
    bool     pi_selection_known_ = true;
    uint8_t  selection_result_   = gatr2::kResultOk;
    uint32_t selection_mark_     = 0; // state_count_ at the ack
    Seconds  next_select_        = 0;
};

} // namespace communigatr
