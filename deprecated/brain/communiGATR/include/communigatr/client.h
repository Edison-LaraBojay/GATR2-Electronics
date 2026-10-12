// client.h
// Brain side of brain link v4 over a BytePort: session, request scheduling,
// retries and reply correlation, robot profile upload, field map and
// estimate transfer, placement, control, wheel readings and path reports. One
// request outstanding at a time. No clock and no threads: poll(now) does all
// I/O and never blocks.
//
// Priority of the next request: HELLO without a session, placement,
// control, profile sync, the due state poll, wheel readings, map chunk,
// estimate chunk, path report, telemetry. Reads and transfers go while the
// state poll is not due, or after four due polls in a row held one back.
// Telemetry only ever goes in the first slot after a state reply: on time
// with the poll not due and nothing waiting, or half a period late ahead of
// the due poll or a waiting transfer.

#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "translaGATR/frame_codec.h"
#include "communigatr/byte_port.h"
#include "communigatr/doc_assembly.h"
#include "communigatr/robot_profile.h"

namespace communigatr
{

using Seconds = double;

// Bench IMU sample carried by every GET_STATE. Keep the acquisition stamp
// unchanged when reusing a sample; report invalid during calibration,
// disconnect or sensor errors.
struct BenchImuSample {
    bool     valid         = false;
    uint32_t stamp_ms      = 0;
    int32_t  rotation_mdeg = 0;     // continuous rotation, CCW positive
    bool     calibrating   = false; // Brain side IMU calibration; not sent, for readiness
};

struct ClientConfig {
    // Per request response timeout: response_timeout, plus byte_time for
    // every byte the request frame and the op's largest reply frame add
    // beyond the 85 byte v3 budget pair.
    Seconds response_timeout   = 0.060; // from the write call's return
    Seconds byte_time          = 10.0 / 115200;
    Seconds request_gap        = 0.005; // after a reply or a timeout
    Seconds state_period       = 0.020; // GET_STATE poll period
    Seconds link_timeout       = 0.25;  // connected while a reply is this recent
    Seconds pending_retry      = 0.020; // SET_POSE and PROFILE_APPLY resend after Pending
    int     placement_attempts = 10;    // sends per placement
    Seconds placement_deadline = 1.0;   // first send through confirming GET_STATE
    int     control_attempts   = 5;     // sends per control before its first reply
    Seconds control_deadline   = 1.0;   // first send through its first reply
    Seconds control_retry      = 0.1;   // CONTROL resend after Pending or a lost reply
    Seconds control_wait       = 20.0;  // first send through the final result, once answered
    Seconds hello_backoff      = 1.0;   // HELLO period after an unsupported version or op
    Seconds field_period       = 0.5;   // between field estimate reads
    Seconds transfer_backoff   = 1.0;   // after 3 failed transfers of one kind in a row
    Seconds telemetry_period   = 0.1;   // TELEMETRY sends on this grid, never within half of it

    // Brain robot profile. Not configured: the Pi uses its XML localization
    // and nothing is uploaded.
    ProfileDocument profile;

    // Called once per state request in the polling task. Empty: flags 0.
    std::function<BenchImuSample()> bench_imu;
};

using PlacementTicket = uint32_t; // 0 = none or refused

enum class PlacementResult : uint8_t {
    kNone,        // not the latest ticket, or 0
    kPending,     // in flight, or Ok and waiting for a state with its anchor
    kApplied,     // a state reply after the Ok shows the placement's anchor
    kRejected,    // error result, see PlacementStatus::result (NotReady included)
    kTimedOut,    // attempts or deadline used up; outcome unknown, may still apply
    kSessionLost, // session ended while pending; never resent
};

struct PlacementStatus {
    PlacementTicket ticket          = 0;
    PlacementResult state           = PlacementResult::kNone;
    uint8_t         result          = translagatr::kResultOk; // last reply result
    uint32_t        odometry_epoch  = 0;                // from the Ok reply
    uint32_t        anchor_revision = 0;
};

using ControlTicket = uint32_t; // 0 = none or refused

enum class ControlResult : uint8_t {
    kNone,          // not the latest ticket, or 0
    kPending,       // queued, in flight, or the Pi answered Pending (Pico working)
    kOk,            // done; see ControlStatus::calibration
    kFailed,        // ran and failed, see ControlStatus::detail
    kNotStationary, // the robot moved in the Pi's stationary window; nothing started
    kNotReady,      // the Pi has no applied robot profile
    kRejected,      // other error result, see ControlStatus::result
    kTimedOut,      // no reply to control_attempts sends or within control_deadline, or
                    // answered but not final within control_wait; outcome unknown
    kSessionLost,   // session ended while pending; never resent
};

struct ControlStatus {
    ControlTicket ticket      = 0;
    uint8_t       action      = 0; // translagatr::ControlAction
    ControlResult state       = ControlResult::kNone;
    uint8_t       result      = translagatr::kResultOk;
    uint8_t       calibration = translagatr::kCalibrationNone;    // last Ok or Pending reply
    uint8_t       detail      = translagatr::kControlDetailNone; // Failed and Pending
};

// READ_WHEELS readings: the profile wheels as the Pi last received them.
struct WheelReadings {
    uint32_t            sequence    = 0; // Ok replies so far, 0 = none
    Seconds             received_at = 0;
    Seconds             round_trip  = 0;
    uint8_t             result      = translagatr::kResultOk; // last reply result
    uint8_t             count       = 0;
    translagatr::WheelReading wheels[translagatr::kWheelReadingsMax];
    bool                busy = false; // ProsLink only: link busy, nothing above is current
};

using WheelTicket = uint32_t; // 0 = none or refused

enum class WheelResult : uint8_t {
    kNone,        // not the latest ticket, or 0
    kPending,     // queued or in flight
    kOk,          // answered; WheelStatus::readings holds it
    kRejected,    // error result, see WheelStatus::result (NotReady, Unavailable)
    kTimedOut,    // no reply to the one send; ask again
    kSessionLost, // session ended first
};

struct WheelStatus {
    WheelTicket   ticket = 0;
    WheelResult   state  = WheelResult::kNone;
    uint8_t       result = translagatr::kResultOk; // this read's reply result
    WheelReadings readings;                  // kOk: this read's readings
};

enum class ProfileSync : uint8_t {
    kNone,     // no profile configured
    kInvalid,  // failed the Brain side check; never sent
    kWaiting,  // no state in this session yet
    kWriting,  // sending the document
    kApplying, // PROFILE_APPLY sent, the Pi answered Pending, or Ok and no state shows it yet
    kApplied,  // a state reply shows the Pi running this profile
    kRejected, // refused; settled until the session changes or resubmitProfile()
};

struct ProfileStatus {
    ProfileSync state    = ProfileSync::kNone;
    uint32_t    id       = 0; // crc32 of the configured document
    uint8_t     reason   = translagatr::kProfileReasonNone; // Brain or Pi reason
    uint8_t     detail   = 0;
    uint8_t     result   = translagatr::kResultOk; // last PROFILE_* reply result
    uint16_t    received = 0;                // bytes the Pi holds of this document
};

enum class LinkError : uint8_t { kNone, kUnsupportedVersion, kUnsupportedOp };

// Latest GET_STATE Ok of the current session.
struct StateSample {
    bool              valid       = false;
    uint32_t          pi_instance = 0;
    uint32_t          session     = 0;
    translagatr::BrainState state;
    Seconds           received_at = 0;
    Seconds           round_trip  = 0; // this request's write to its reply
};

// The newest complete field: a map and an estimate checked against it,
// published together. Never a partial document.
struct FieldPublication {
    uint32_t             generation = 0; // per published pair, 0 = none yet
    uint32_t             map_id     = 0;
    std::vector<uint8_t> map;            // validated map document
    uint32_t             estimate_id = 0;
    std::vector<uint8_t> estimate;       // validated against map
    uint32_t             pi_instance = 0; // exchange that delivered the estimate
    uint32_t             session     = 0;
    Seconds              completed_at = 0; // last estimate chunk received
    // The Pi took the estimate snapshot no earlier than this, on the poll
    // clock. Observation ages at completed_at are at most
    // age_ms + (completed_at - snapshot_after).
    Seconds snapshot_after = 0;
};

// Transfer progress for status displays.
struct FieldSyncStatus {
    uint32_t map_id           = 0; // complete map held, 0 = none
    bool     map_reading      = false;
    uint16_t map_received     = 0; // bytes of the map being read
    uint32_t estimate_reading = 0; // estimate id being read, 0 = none
};

struct ClientStats {
    uint32_t requests        = 0; // frames written, resends included
    uint32_t resends         = 0; // byte-identical retries
    uint32_t replies         = 0; // correlated
    uint32_t timeouts        = 0;
    uint32_t uncorrelated    = 0; // valid replies matching no outstanding request
    uint32_t bad_frames      = 0; // complete frames that are not decodable replies
    uint32_t drained_bytes   = 0; // discarded before a send
    uint32_t read_errors     = 0;
    uint32_t write_errors    = 0;
    uint32_t sessions        = 0; // opened
    uint32_t session_losses  = 0;
    uint32_t pi_restarts     = 0; // pi_instance changes
    uint32_t stale_hellos    = 0;
    uint32_t unexpected      = 0; // correlated replies with a result or body the op never gets
    uint32_t profile_writes  = 0; // PROFILE_WRITE Ok replies
    uint32_t doc_chunks      = 0; // READ_DOC Ok replies taken
    uint32_t doc_rejects     = 0; // inconsistent chunks, crc or validation failures
    uint32_t doc_stale       = 0; // READ_DOC Stale replies
    uint32_t maps            = 0; // maps completed
    uint32_t estimates       = 0; // estimates published
    uint32_t path_reports    = 0; // sent
    uint32_t paths_dropped   = 0; // replaced before sending, or no session

    uint32_t telemetry_reports  = 0; // TELEMETRY sent
    uint32_t telemetry_overdue  = 0; // of those, half a period late, ahead of a due poll or a transfer
    uint32_t telemetry_replaced = 0; // a newer report replaced an unsent one
    uint32_t telemetry_dropped  = 0; // no session, refused this session, bad body, session lost
    uint32_t telemetry_refused  = 0; // UnsupportedOp or InvalidArgument answers
};

class Client {
public:
    // nonce: entropy for each new HELLO transaction. A 0 or a repeat of the
    // previous nonce is replaced by a deterministic step.
    Client(BytePort& port, std::function<uint32_t()> nonce, const ClientConfig& config = {});

    // Receives, times out, and sends at most one request. Call every few ms
    // with a steady clock; now also stands for the write call's return.
    void poll(Seconds now);

    // Field pose in wire units. 0 (refused) unless ready, the configured
    // profile is applied, and no other placement is pending. Only the latest
    // ticket is tracked.
    PlacementTicket submitPlacement(int32_t x_mm, int32_t y_mm, int32_t heading_cdeg);
    PlacementResult placementResult(PlacementTicket ticket) const;
    PlacementStatus placementStatus(PlacementTicket ticket) const;
    bool            placementPending() const;

    // Pi control, a translagatr::ControlAction. 0 (refused) unless ready and no
    // other control is pending. The Pi checks the stationary condition. A
    // Pending answer is asked again with the same request id every
    // control_retry until it settles; a duplicate never runs twice. Before
    // the first reply control_attempts and control_deadline bound it; once
    // answered, the Pi holds its record and only control_wait does, so
    // lost replies (a pulled cable) do not end it.
    ControlTicket control(uint8_t action);
    ControlTicket recalibrate() { return control(translagatr::kControlRecalibrate); }
    ControlTicket reinitialize() { return control(translagatr::kControlReinitialize); }
    ControlTicket reinitImu() { return control(translagatr::kControlReinitImu); }
    ControlTicket restartAcquisition() { return control(translagatr::kControlRestartAcquisition); }
    ControlStatus controlStatus(ControlTicket ticket) const;
    bool          controlPending() const;

    // One READ_WHEELS read of the raw profile wheels. 0 (refused) without a
    // session or while another read is pending. One send: a lost reply
    // settles kTimedOut. Only the latest ticket is tracked.
    WheelTicket requestWheels();
    WheelStatus wheelStatus(WheelTicket ticket) const;
    bool        wheelsPending() const { return wheel_.state == WheelResult::kPending; }

    // Readings of the latest Ok reply of any read; result is the last reply's.
    const WheelReadings& wheelReadings() const { return wheels_; }

    // Uploads and applies the configured profile again, clearing a
    // settled rejection.
    void resubmitProfile();

    // Replaces the configured profile at run time. The next state shows
    // whether the Pi already runs it; otherwise it is uploaded and applied,
    // and the Pi starts a new odometry epoch that needs a new placement. The
    // same document again changes nothing. False when doc is empty or fails
    // the Brain side check: a valid current profile stays; without one, doc
    // is kept as kInvalid so its reason shows.
    bool setProfile(const ProfileDocument& doc);

    const ProfileStatus& profile() const { return profile_; }
    bool                 profileConfigured() const { return profile_.state != ProfileSync::kNone; }
    bool                 profileApplied() const { return profile_.state == ProfileSync::kApplied; }

    // Latest planned path for Pi inspection, points in field mm. Best effort:
    // one attempt, a newer report replaces an unsent one, dropped without a
    // session. More than kPathReportMaxPoints points are thinned to that many,
    // first and last kept. False when dropped at once.
    bool reportPath(uint32_t command_id, uint8_t path_mode, const translagatr::PathPoint* points,
                    std::size_t count);

    // Robot telemetry for Pi display and recording. Best effort, lowest
    // priority: one attempt, a newer report replaces an unsent one, sends on
    // a telemetry_period grid, only in the first slot after a state reply.
    // On time it waits for a slot with the state poll not due; half a period
    // late it takes that slot from the due poll or a waiting transfer, so a
    // slow link delays it instead of starving it. Dropped without a session,
    // with unknown flags or too many wheels, once the Pi refused TELEMETRY in
    // this session (UnsupportedOp from an older Pi, or InvalidArgument; the
    // session itself stays), and when still unsent two periods later (a link
    // outage). False when dropped at once.
    bool reportTelemetry(const translagatr::BrainTelemetry& telemetry);

    // The Pi refused TELEMETRY in this session; cleared by a new session.
    bool telemetryUnsupported() const { return telemetry_unsupported_; }

    const FieldPublication& field() const { return field_; }
    FieldSyncStatus         fieldSync() const;

    // Session open and a GET_STATE Ok received in it.
    bool ready() const { return ready_; }

    // Ready at least once since this client started.
    bool everReady() const { return ever_ready_; }

    // Bench IMU sample of the latest state poll.
    const BenchImuSample& benchImu() const { return bench_sample_; }

    // Ready and a correlated reply within link_timeout.
    bool    connected(Seconds now) const;
    Seconds linkAge(Seconds now) const; // infinity without a session

    uint32_t           session() const { return session_; }        // 0 = none
    uint32_t           piInstance() const { return pi_instance_; } // recorded at HELLO Ok
    const StateSample& state() const { return state_; }

    // Last incompatibility; cleared when ready again.
    LinkError error() const { return error_; }
    uint8_t   peerVersion() const { return peer_version_; }

    // Response timeout for a request frame of frame_len bytes with this op.
    Seconds responseTimeout(uint8_t op, uint16_t frame_len) const;

    const ClientStats&  stats() const { return stats_; }
    const ClientConfig& config() const { return config_; }

private:
    enum class Kind : uint8_t {
        kNone,
        kHello,
        kPlacement,
        kControl,
        kProfileWrite,
        kProfileApply,
        kState,
        kWheels,
        kMapChunk,
        kEstimateChunk,
        kPathReport,
        kTelemetry,
    };

    struct Transaction {
        translagatr::BrainRequest request;
        uint8_t             frame[translagatr::kMaxFrameLen] = {};
        uint16_t            len                        = 0; // 0 = no transaction
        int                 attempts                   = 0;
        Seconds             first_sent                 = 0;
    };

    // Estimate ids seen in state replies, with the send time of the last poll
    // before the id appeared.
    struct EstimateSeen {
        uint32_t id    = 0;
        Seconds  after = 0;
    };

    void receive(Seconds now);
    void handleFrame(const uint8_t* frame, uint16_t len, Seconds now);
    bool correlates(const translagatr::BrainReply& reply) const;
    void handleReply(const translagatr::BrainReply& reply, Seconds now);
    void handleHello(const translagatr::BrainReply& reply, Seconds now);
    void handlePlacement(const translagatr::BrainReply& reply, Seconds now);
    void handleControl(const translagatr::BrainReply& reply, Seconds now);
    void handleWheels(const translagatr::BrainReply& reply, Seconds now, Seconds round_trip);
    void handleProfileWrite(const translagatr::BrainReply& reply);
    void handleProfileApply(const translagatr::BrainReply& reply, Seconds now);
    void handleState(const translagatr::BrainReply& reply, Seconds now, Seconds round_trip);
    void handleMapChunk(const translagatr::BrainReply& reply, Seconds now);
    void handleEstimateChunk(const translagatr::BrainReply& reply, Seconds now);
    void incompatible(Kind kind, const translagatr::BrainReply& reply, Seconds now);

    void attemptFailed(Seconds now);
    void transmit(Seconds now);
    Kind choose(Seconds now);
    Kind statePoll();
    bool telemetrySlot(Seconds now, Seconds late) const;
    bool startTelemetry();
    void start(Transaction& tx, translagatr::BrainRequest request);
    void drain();
    void loseSession();

    void settlePlacement(PlacementResult state, uint8_t result);
    bool placementExhausted(Seconds now) const;
    void settleControl(ControlResult state, uint8_t result);
    bool controlExhausted(Seconds now) const;
    void settleWheels(WheelResult state, uint8_t result);

    void configureProfile(const ProfileDocument& doc);
    void profileFromState(const translagatr::BrainState& state);
    void profileFailure(uint8_t result);
    void settleProfileRejected(uint8_t result, uint8_t reason, uint8_t detail);

    bool mapWanted(Seconds now) const;
    bool estimateWanted(Seconds now) const;
    void mapFailure(Seconds now);
    void estimateFailure(Seconds now);
    void publish(Seconds now);
    void noteEstimateId(uint32_t id, Seconds after);

    uint16_t takeRequestId();
    uint32_t takeNonce();

    BytePort&                 port_;
    std::function<uint32_t()> nonce_source_;
    ClientConfig              config_;
    translagatr::FrameReader        reader_;
    ClientStats               stats_;

    // Exchange.
    Transaction         hello_;
    Transaction         placement_tx_;
    Transaction         control_tx_;
    Transaction         once_tx_; // every request sent once: state, profile, docs, path
    Kind                outstanding_ = Kind::kNone;
    translagatr::BrainRequest sent_; // the outstanding request
    Seconds             sent_at_          = 0;
    Seconds             sent_timeout_     = 0;
    Seconds             next_send_        = 0;
    Seconds             next_hello_       = 0;
    Seconds             next_state_       = 0;
    Kind                retry_first_      = Kind::kNone; // timed out; retried before anything else
    bool                state_before_retry_ = false;      // bench IMU poll before a SET_POSE resend
    int                 starved_polls_    = 0; // due polls sent while a transfer waited
    uint16_t            next_request_id_  = 1;
    uint32_t            last_nonce_       = 0;

    // Session.
    uint32_t    session_       = 0;
    uint32_t    pi_instance_   = 0;
    bool        ready_         = false;
    bool        ever_ready_    = false;
    Seconds     last_reply_    = 0;
    StateSample state_;
    uint32_t    state_count_   = 0; // GET_STATE Ok replies in this session
    Seconds     last_state_sent_ = 0; // send time of the previous state poll
    LinkError   error_         = LinkError::kNone;
    uint8_t     peer_version_  = 0;

    // Placement.
    PlacementStatus     placement_;
    PlacementTicket     last_ticket_ = 0;
    translagatr::BrainRequest placement_request_;
    bool                placement_acked_ = false;
    uint32_t            placement_mark_  = 0; // state_count_ at the Ok
    Seconds             next_placement_  = 0;

    // Control.
    ControlStatus control_;
    ControlTicket last_control_     = 0;
    bool          control_answered_ = false; // a reply arrived; control_wait applies
    int           control_misses_   = 0;     // sends without a reply, in a row
    Seconds       next_control_     = 0;

    // Wheel readings.
    WheelStatus   wheel_;
    WheelTicket   last_wheel_    = 0;
    bool          wheels_wanted_ = false; // the pending read is not sent yet
    WheelReadings wheels_;

    // Profile.
    uint16_t profile_len_        = 0;
    int      profile_failures_   = 0;     // InvalidArgument or no progress, in a row
    bool     profile_confirming_ = false; // APPLY Ok; waiting for a state that shows it
    Seconds  next_profile_       = 0;
    ProfileStatus profile_;

    // Field documents.
    std::vector<uint8_t> map_cache_; // complete validated map, keyed by map_cache_id_
    uint16_t             map_cache_len_ = 0;
    uint32_t             map_cache_id_  = 0;
    DocAssembly          map_asm_;
    int                  map_failures_ = 0;
    Seconds              next_map_     = 0;
    DocAssembly          estimate_asm_;
    int                  estimate_failures_ = 0;
    Seconds              next_estimate_     = 0;
    EstimateSeen         estimate_seen_[4];
    FieldPublication     field_;

    // Bench IMU.
    BenchImuSample bench_sample_;

    // Path report.
    bool                path_pending_ = false;
    translagatr::BrainRequest path_request_;

    // Telemetry.
    bool                      telemetry_pending_     = false;
    bool                      telemetry_unsupported_ = false; // this session
    bool                      after_state_           = false; // nothing sent since a state reply
    Seconds                   state_reply_at_        = 0;
    Seconds                   next_telemetry_        = 0;
    Seconds                   telemetry_at_          = 0; // last poll time before the report
    Seconds                   last_poll_             = 0;
    translagatr::BrainRequest telemetry_request_;
};

} // namespace communigatr
