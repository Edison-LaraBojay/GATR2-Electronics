// client.cpp

#include "communigatr/client.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "translaGATR/link_documents.h"

namespace communigatr
{

namespace
{

constexpr Seconds  kLongAgo   = std::numeric_limits<Seconds>::lowest();
constexpr Seconds  kUnknown   = -std::numeric_limits<Seconds>::infinity();
constexpr int      kMaxReads  = 16; // 64 byte reads per receive or drain
constexpr uint32_t kNonceStep = 0x9E3779B9u;
constexpr int      kFailuresBeforeBackoff = 3;
constexpr int      kPollsBeforeTransfer   = 4; // due polls a waiting transfer yields to
constexpr int      kBudgetFrameBytes      = 85; // v3 largest request plus largest reply
constexpr uint8_t  kTelemetryFlags = translagatr::kTelemetryAttitude | translagatr::kTelemetryMotion |
                                    translagatr::kTelemetryWheels;

// Anchor revisions only grow within one pi_instance; serial order handles wrap.
bool anchorReached(uint32_t reported, uint32_t wanted) {
    return static_cast<uint32_t>(reported - wanted) < 0x80000000u;
}

// The same shared check the Pi runs, before anything is sent.
bool profileValid(const ProfileDocument& doc, uint8_t& reason, uint8_t& detail) {
    reason = doc.reason;
    detail = doc.detail;
    if (reason != translagatr::kProfileReasonNone) {
        return false;
    }
    translagatr::RobotProfileDoc decoded;
    if (!translagatr::decodeRobotProfile(doc.bytes, doc.len, decoded)) {
        reason = translagatr::kProfileReasonFormat;
        detail = 0;
        return false;
    }
    return translagatr::validateRobotProfile(decoded, reason, detail);
}

} // namespace

Client::Client(BytePort& port, std::function<uint32_t()> nonce, const ClientConfig& config)
    : port_(port), nonce_source_(std::move(nonce)), config_(config),
      map_cache_(translagatr::kFieldMapMaxLen), map_asm_(translagatr::kFieldMapMaxLen),
      estimate_asm_(translagatr::kFieldEstimateMaxLen) {
    next_send_      = kLongAgo;
    next_hello_     = kLongAgo;
    next_state_     = kLongAgo;
    next_placement_ = kLongAgo;
    next_profile_   = kLongAgo;
    next_map_       = kLongAgo;
    next_estimate_  = kLongAgo;
    next_telemetry_ = kLongAgo;
    field_.map.reserve(translagatr::kFieldMapMaxLen);
    field_.estimate.reserve(translagatr::kFieldEstimateMaxLen);
    if (config_.profile.configured()) {
        configureProfile(config_.profile);
    }
}

void Client::poll(Seconds now) {
    // Operation deadlines include confirmation. Checked before receive so a
    // reply processed after the deadline cannot revive them. An outstanding
    // bus response window is kept until its reply or timeout.
    if (placementPending() && placement_tx_.attempts > 0 &&
        now - placement_tx_.first_sent >= config_.placement_deadline) {
        settlePlacement(PlacementResult::kTimedOut, placement_.result);
    }
    if (controlPending() && control_tx_.attempts > 0 &&
        now - control_tx_.first_sent >=
            (control_answered_ ? config_.control_wait : config_.control_deadline)) {
        settleControl(ControlResult::kTimedOut, control_.result);
    }
    last_poll_ = now;
    receive(now);
    if (outstanding_ != Kind::kNone && now - sent_at_ >= sent_timeout_) {
        ++stats_.timeouts;
        attemptFailed(now);
    }
    transmit(now);
}

// ---------------------------------------------------------------------------
// Public operations
// ---------------------------------------------------------------------------

PlacementTicket Client::submitPlacement(int32_t x_mm, int32_t y_mm, int32_t heading_cdeg) {
    if (placementPending() || !ready_ || (profileConfigured() && !profileApplied())) {
        return 0;
    }
    last_ticket_ = last_ticket_ == UINT32_MAX ? 1 : last_ticket_ + 1;

    placement_          = PlacementStatus{};
    placement_.ticket   = last_ticket_;
    placement_.state    = PlacementResult::kPending;
    placement_acked_    = false;
    placement_tx_       = Transaction{};
    next_placement_     = kLongAgo;
    state_before_retry_ = false;

    placement_request_              = translagatr::BrainRequest{};
    placement_request_.op           = translagatr::kOpSetPose;
    placement_request_.x_mm         = x_mm;
    placement_request_.y_mm         = y_mm;
    placement_request_.heading_cdeg = heading_cdeg;
    return last_ticket_;
}

PlacementResult Client::placementResult(PlacementTicket ticket) const {
    return placementStatus(ticket).state;
}

PlacementStatus Client::placementStatus(PlacementTicket ticket) const {
    if (ticket == 0 || ticket != placement_.ticket) {
        PlacementStatus none;
        none.ticket = ticket;
        return none;
    }
    return placement_;
}

bool Client::placementPending() const { return placement_.state == PlacementResult::kPending; }

ControlTicket Client::control(uint8_t action) {
    if (controlPending() || !ready_) {
        return 0;
    }
    last_control_     = last_control_ == UINT32_MAX ? 1 : last_control_ + 1;
    control_          = ControlStatus{};
    control_.ticket   = last_control_;
    control_.action   = action;
    control_.state    = ControlResult::kPending;
    control_tx_       = Transaction{};
    control_answered_ = false;
    control_misses_   = 0;
    next_control_     = kLongAgo;
    return last_control_;
}

WheelTicket Client::requestWheels() {
    if (session_ == 0 || wheelsPending()) {
        return 0;
    }
    last_wheel_    = last_wheel_ == UINT32_MAX ? 1 : last_wheel_ + 1;
    wheel_         = WheelStatus{};
    wheel_.ticket  = last_wheel_;
    wheel_.state   = WheelResult::kPending;
    wheels_wanted_ = true;
    return last_wheel_;
}

WheelStatus Client::wheelStatus(WheelTicket ticket) const {
    if (ticket == 0 || ticket != wheel_.ticket) {
        WheelStatus none;
        none.ticket = ticket;
        return none;
    }
    return wheel_;
}

ControlStatus Client::controlStatus(ControlTicket ticket) const {
    if (ticket == 0 || ticket != control_.ticket) {
        ControlStatus none;
        none.ticket = ticket;
        return none;
    }
    return control_;
}

bool Client::controlPending() const { return control_.state == ControlResult::kPending; }

void Client::resubmitProfile() {
    if (profile_.state == ProfileSync::kNone || profile_.state == ProfileSync::kInvalid) {
        return;
    }
    profile_.state      = ready_ ? ProfileSync::kWriting : ProfileSync::kWaiting;
    profile_.reason     = translagatr::kProfileReasonNone;
    profile_.detail     = 0;
    profile_.result     = translagatr::kResultOk;
    profile_.received   = 0;
    profile_failures_   = 0;
    profile_confirming_ = false;
    next_profile_       = kLongAgo;
}

bool Client::setProfile(const ProfileDocument& doc) {
    if (!doc.configured()) {
        return false;
    }
    uint8_t    reason  = 0;
    uint8_t    detail  = 0;
    const bool valid   = profileValid(doc, reason, detail);
    const bool running = profile_.state != ProfileSync::kNone &&
                         profile_.state != ProfileSync::kInvalid;
    if (!valid && running) {
        return false;
    }
    if (valid && running && profileId(doc) == profile_.id && doc.len == profile_len_) {
        return true;
    }
    configureProfile(doc);
    return valid;
}

bool Client::reportPath(uint32_t command_id, uint8_t path_mode, const translagatr::PathPoint* points,
                        std::size_t count) {
    if (session_ == 0 || path_mode > translagatr::kPathAvoiding || (count > 0 && points == nullptr)) {
        ++stats_.paths_dropped;
        return false;
    }
    if (path_pending_) {
        ++stats_.paths_dropped;
    }
    if (path_mode == translagatr::kPathNone) {
        count = 0;
    }
    translagatr::BrainRequest request;
    request.op         = translagatr::kOpPathReport;
    request.command_id = command_id;
    request.path_mode  = path_mode;
    const std::size_t max = translagatr::kPathReportMaxPoints;
    if (count <= max) {
        request.point_count = static_cast<uint8_t>(count);
        for (std::size_t i = 0; i < count; ++i) {
            request.points[i] = points[i];
        }
    } else {
        // Evenly spaced, rounded; first and last kept.
        request.point_count = static_cast<uint8_t>(max);
        for (std::size_t i = 0; i < max; ++i) {
            const std::size_t at = (2 * i * (count - 1) + (max - 1)) / (2 * (max - 1));
            request.points[i]    = points[at];
        }
    }
    path_request_ = request;
    path_pending_ = true;
    return true;
}

bool Client::reportTelemetry(const translagatr::BrainTelemetry& telemetry) {
    const bool wheels = (telemetry.flags & translagatr::kTelemetryWheels) != 0;
    if (session_ == 0 || telemetry_unsupported_ || (telemetry.flags & ~kTelemetryFlags) != 0 ||
        (wheels && telemetry.wheel_count > translagatr::kTelemetryWheelsMax)) {
        ++stats_.telemetry_dropped;
        return false;
    }
    if (telemetry_pending_) {
        ++stats_.telemetry_replaced;
    }
    telemetry_request_           = translagatr::BrainRequest{};
    telemetry_request_.op        = translagatr::kOpTelemetry;
    telemetry_request_.telemetry = telemetry;
    telemetry_pending_           = true;
    telemetry_at_                = last_poll_;
    return true;
}

FieldSyncStatus Client::fieldSync() const {
    FieldSyncStatus s;
    s.map_id           = map_cache_id_;
    s.map_reading      = map_asm_.active();
    s.map_received     = map_asm_.active() ? map_asm_.offset() : 0;
    s.estimate_reading = estimate_asm_.active() ? estimate_asm_.docId() : 0;
    return s;
}

bool Client::connected(Seconds now) const {
    return ready_ && now - last_reply_ <= config_.link_timeout;
}

Seconds Client::linkAge(Seconds now) const {
    if (session_ == 0) {
        return std::numeric_limits<Seconds>::infinity();
    }
    return now - last_reply_;
}

Seconds Client::responseTimeout(uint8_t op, uint16_t frame_len) const {
    uint8_t reply = translagatr::brainReplyMaxLen(op, translagatr::kResultOk);
    if (reply == 0) {
        reply = translagatr::kBrainReplyHeaderLen;
    }
    const int extra = frame_len + reply + translagatr::kLinkEnvelopeLen - kBudgetFrameBytes;
    return config_.response_timeout + (extra > 0 ? extra * config_.byte_time : 0.0);
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------

void Client::receive(Seconds now) {
    uint8_t buf[64];
    for (int i = 0; i < kMaxReads; ++i) {
        const int n = port_.read(buf, static_cast<int>(sizeof(buf)));
        if (n < 0) {
            ++stats_.read_errors;
            return;
        }
        if (n == 0) {
            return;
        }
        for (int k = 0; k < n; ++k) {
            if (reader_.push(buf[k])) {
                do {
                    handleFrame(reader_.frame(), reader_.frameLen(), now);
                } while (reader_.next());
            }
        }
    }
}

void Client::handleFrame(const uint8_t* frame, uint16_t len, Seconds now) {
    translagatr::BrainReply reply;
    if (len < 3 || frame[2] != translagatr::kFrameBrainReply ||
        !translagatr::decodeBrainReply(frame, len, reply)) {
        ++stats_.bad_frames;
        return;
    }
    if (!correlates(reply)) {
        ++stats_.uncorrelated;
        return;
    }
    handleReply(reply, now);
}

bool Client::correlates(const translagatr::BrainReply& reply) const {
    if (outstanding_ == Kind::kNone) {
        return false;
    }
    if (reply.op != sent_.op || reply.request_id != sent_.request_id) {
        return false;
    }
    // Another version's body is unreadable: header echo only.
    if (reply.version != translagatr::kBrainLinkVersion) {
        return reply.session == sent_.session;
    }
    if (sent_.op == translagatr::kOpHello) {
        return reply.nonce == sent_.nonce;
    }
    return reply.session == sent_.session;
}

void Client::handleReply(const translagatr::BrainReply& reply, Seconds now) {
    const Kind    kind       = outstanding_;
    const Seconds round_trip = now - sent_at_;
    outstanding_             = Kind::kNone;
    next_send_               = now + config_.request_gap;
    ++stats_.replies;

    // An older Pi, or a body it refuses: no more TELEMETRY this session,
    // which stays. From another instance it is a restarted Pi.
    if (kind == Kind::kTelemetry && reply.version == translagatr::kBrainLinkVersion &&
        (reply.result == translagatr::kResultUnsupportedOp ||
         reply.result == translagatr::kResultInvalidArgument)) {
        if (reply.pi_instance != pi_instance_) {
            ++stats_.pi_restarts;
            loseSession();
            return;
        }
        last_reply_            = now;
        telemetry_unsupported_ = true;
        ++stats_.telemetry_refused;
        if (telemetry_pending_) {
            ++stats_.telemetry_dropped;
            telemetry_pending_ = false;
        }
        return;
    }
    if (reply.version != translagatr::kBrainLinkVersion ||
        reply.result == translagatr::kResultUnsupportedVersion ||
        reply.result == translagatr::kResultUnsupportedOp) {
        incompatible(kind, reply, now);
        return;
    }
    if (kind == Kind::kHello) {
        handleHello(reply, now);
        return;
    }
    if (reply.pi_instance != pi_instance_) {
        ++stats_.pi_restarts;
        loseSession();
        return;
    }
    if (reply.result == translagatr::kResultUnknownSession) {
        loseSession();
        return;
    }
    last_reply_ = now;
    switch (kind) {
    case Kind::kPlacement:
        handlePlacement(reply, now);
        break;
    case Kind::kControl:
        handleControl(reply, now);
        break;
    case Kind::kWheels:
        handleWheels(reply, now, round_trip);
        break;
    case Kind::kProfileWrite:
        handleProfileWrite(reply);
        break;
    case Kind::kProfileApply:
        handleProfileApply(reply, now);
        break;
    case Kind::kState:
        handleState(reply, now, round_trip);
        after_state_    = true;
        state_reply_at_ = now;
        break;
    case Kind::kMapChunk:
        handleMapChunk(reply, now);
        break;
    case Kind::kEstimateChunk:
        handleEstimateChunk(reply, now);
        break;
    case Kind::kPathReport:
    case Kind::kTelemetry:
        if (reply.result != translagatr::kResultOk) {
            ++stats_.unexpected;
        }
        break;
    default:
        break;
    }
}

void Client::handleHello(const translagatr::BrainReply& reply, Seconds now) {
    hello_.len = 0;
    if (reply.result == translagatr::kResultStale) {
        ++stats_.stale_hellos;
        return;
    }
    if (reply.result != translagatr::kResultOk || reply.session == 0) {
        ++stats_.unexpected;
        next_hello_ = now + config_.hello_backoff;
        return;
    }
    ++stats_.sessions;
    session_               = reply.session;
    pi_instance_           = reply.pi_instance;
    last_reply_            = now;
    ready_                 = false;
    state_                 = StateSample{};
    state_count_           = 0;
    next_state_            = now;
    telemetry_unsupported_ = false;
}

void Client::handlePlacement(const translagatr::BrainReply& reply, Seconds now) {
    if (!placementPending() || placement_acked_ || placement_tx_.attempts == 0 ||
        sent_.request_id != placement_tx_.request.request_id) {
        return;
    }
    placement_.result = reply.result;
    if (reply.result == translagatr::kResultOk) {
        placement_acked_           = true;
        placement_.odometry_epoch  = reply.odometry_epoch;
        placement_.anchor_revision = reply.anchor_revision;
        placement_mark_            = state_count_;
        placement_tx_.len          = 0;
        return;
    }
    if (reply.result == translagatr::kResultPending) {
        if (placementExhausted(now)) {
            settlePlacement(PlacementResult::kTimedOut, reply.result);
        } else {
            // The Pi holds the record: a state poll may go before the resend.
            next_placement_     = now + config_.pending_retry;
            state_before_retry_ = static_cast<bool>(config_.bench_imu);
        }
        return;
    }
    settlePlacement(PlacementResult::kRejected, reply.result);
}

void Client::handleControl(const translagatr::BrainReply& reply, Seconds now) {
    if (!controlPending() || control_tx_.attempts == 0 ||
        sent_.request_id != control_tx_.request.request_id) {
        return;
    }
    control_.result   = reply.result;
    control_answered_ = true;
    control_misses_   = 0;
    const bool body   = reply.result == translagatr::kResultOk || reply.result == translagatr::kResultPending ||
                      reply.result == translagatr::kResultFailed;
    if (body && reply.action != control_.action) {
        ++stats_.unexpected;
        settleControl(ControlResult::kRejected, reply.result);
        return;
    }
    if (body) {
        control_.calibration = reply.calibration;
        control_.detail      = reply.control_detail;
    }
    switch (reply.result) {
    case translagatr::kResultOk:
        settleControl(ControlResult::kOk, reply.result);
        return;
    case translagatr::kResultPending:
        // Asked again with the same bytes; the Pi answers from its record.
        next_control_ = now + config_.control_retry;
        return;
    case translagatr::kResultFailed:
        settleControl(ControlResult::kFailed, reply.result);
        return;
    case translagatr::kResultNotStationary:
        settleControl(ControlResult::kNotStationary, reply.result);
        return;
    case translagatr::kResultNotReady:
        settleControl(ControlResult::kNotReady, reply.result);
        return;
    default:
        settleControl(ControlResult::kRejected, reply.result);
        return;
    }
}

// The only read in flight is the latest ticket's: a new ticket waits for it.
void Client::handleWheels(const translagatr::BrainReply& reply, Seconds now, Seconds round_trip) {
    wheels_.result = reply.result;
    if (reply.result != translagatr::kResultOk) {
        if (wheelsPending()) {
            settleWheels(WheelResult::kRejected, reply.result);
        }
        return;
    }
    ++wheels_.sequence;
    wheels_.received_at = now;
    wheels_.round_trip  = round_trip;
    wheels_.count       = std::min<uint8_t>(reply.wheel_count, translagatr::kWheelReadingsMax);
    for (uint8_t i = 0; i < wheels_.count; ++i) {
        wheels_.wheels[i] = reply.wheels[i];
    }
    if (wheelsPending()) {
        wheel_.readings = wheels_;
        settleWheels(WheelResult::kOk, reply.result);
    }
}

void Client::handleProfileWrite(const translagatr::BrainReply& reply) {
    if (profile_.state != ProfileSync::kWriting || sent_.profile_id != profile_.id) {
        return; // replaced by setProfile while in flight
    }
    profile_.result = reply.result;
    if (reply.result != translagatr::kResultOk) {
        profileFailure(reply.result); // a gap or other bytes: rewrite from 0
        return;
    }
    if (reply.profile_id != profile_.id) {
        ++stats_.unexpected;
        profileFailure(reply.result);
        return;
    }
    ++stats_.profile_writes;
    const uint16_t held = std::min<uint16_t>(reply.received, profile_len_);
    const uint32_t end  = static_cast<uint32_t>(sent_.offset) + sent_.data_len;
    profile_.received   = held;
    if (held < end) {
        // The Pi did not keep this chunk; continue from what it holds.
        if (++profile_failures_ >= kFailuresBeforeBackoff) {
            settleProfileRejected(reply.result, translagatr::kProfileReasonNone, 0);
        }
        return;
    }
    profile_failures_ = 0;
    if (held >= profile_len_) {
        profile_.state      = ProfileSync::kApplying;
        profile_confirming_ = false;
        next_profile_       = kLongAgo;
    }
}

void Client::handleProfileApply(const translagatr::BrainReply& reply, Seconds now) {
    if (profile_.state != ProfileSync::kApplying || sent_.profile_id != profile_.id) {
        return;
    }
    profile_.result = reply.result;
    switch (reply.result) {
    case translagatr::kResultOk:
        if (reply.profile_id != profile_.id) {
            ++stats_.unexpected;
            profileFailure(reply.result);
            return;
        }
        // The latest state may predate the Pi's swap and describe the old
        // profile's frame and placement: applied only once a state shows it.
        profile_confirming_ = true;
        profile_.reason     = translagatr::kProfileReasonNone;
        profile_.detail     = 0;
        profile_failures_   = 0;
        return;
    case translagatr::kResultPending:
        next_profile_ = now + config_.pending_retry;
        return;
    case translagatr::kResultProfileRejected:
        settleProfileRejected(reply.result, reply.profile_reason, reply.profile_detail);
        return;
    case translagatr::kResultInvalidArgument:
        profileFailure(reply.result); // staging incomplete or replaced: rewrite
        return;
    default:
        ++stats_.unexpected;
        profileFailure(reply.result);
        return;
    }
}

void Client::handleState(const translagatr::BrainReply& reply, Seconds now, Seconds round_trip) {
    if (reply.result != translagatr::kResultOk) {
        ++stats_.unexpected;
        return;
    }
    // An estimate id first seen now was taken after the previous poll was
    // answered; one already current at the first state predates the session.
    noteEstimateId(reply.state.estimate_id, state_count_ == 0 ? kUnknown : last_state_sent_);
    last_state_sent_ = sent_at_;

    state_.valid       = true;
    state_.pi_instance = reply.pi_instance;
    state_.session     = reply.session;
    state_.state       = reply.state;
    state_.received_at = now;
    state_.round_trip  = round_trip;
    ++state_count_;
    ready_      = true;
    ever_ready_ = true;
    error_      = LinkError::kNone;

    if (placementPending() && placement_acked_ && state_count_ > placement_mark_ &&
        anchorReached(reply.state.anchor_revision, placement_.anchor_revision)) {
        settlePlacement(PlacementResult::kApplied, translagatr::kResultOk);
    }
    profileFromState(reply.state);
    if (map_asm_.active() && map_asm_.docId() != reply.state.map_id) {
        map_asm_.clear(); // map replaced; read the new one
    }
}

void Client::handleMapChunk(const translagatr::BrainReply& reply, Seconds now) {
    if (!map_asm_.active() || sent_.doc_id != map_asm_.docId() ||
        sent_.doc_offset != map_asm_.offset()) {
        return;
    }
    switch (reply.result) {
    case translagatr::kResultOk:
        break;
    case translagatr::kResultStale:
        ++stats_.doc_stale;
        mapFailure(now);
        return;
    case translagatr::kResultUnavailable:
        map_asm_.clear();
        next_map_ = now + config_.field_period;
        return;
    default:
        ++stats_.unexpected;
        mapFailure(now);
        return;
    }
    ++stats_.doc_chunks;
    const DocAssembly::Step step = map_asm_.accept(reply);
    if (step == DocAssembly::Step::kMore) {
        return;
    }
    const uint32_t id = map_asm_.docId();
    if (step != DocAssembly::Step::kComplete || map_asm_.crc() != id ||
        translagatr::validateFieldMap(map_asm_.data(), map_asm_.length()) != translagatr::DocError::kNone) {
        ++stats_.doc_rejects;
        mapFailure(now);
        return;
    }
    std::copy(map_asm_.data(), map_asm_.data() + map_asm_.length(), map_cache_.begin());
    map_cache_len_ = map_asm_.length();
    map_cache_id_  = id;
    map_failures_  = 0;
    map_asm_.clear();
    estimate_asm_.clear();     // any estimate in progress names the old map
    next_estimate_ = kLongAgo; // an estimate for the new map at once
    ++stats_.maps;
}

void Client::handleEstimateChunk(const translagatr::BrainReply& reply, Seconds now) {
    if (!estimate_asm_.active() || sent_.doc_id != estimate_asm_.docId() ||
        sent_.doc_offset != estimate_asm_.offset()) {
        return;
    }
    switch (reply.result) {
    case translagatr::kResultOk:
        break;
    case translagatr::kResultStale:
        ++stats_.doc_stale;
        estimateFailure(now); // restarts from the newest id
        return;
    case translagatr::kResultUnavailable:
        estimate_asm_.clear();
        next_estimate_ = now + config_.field_period;
        return;
    default:
        ++stats_.unexpected;
        estimateFailure(now);
        return;
    }
    ++stats_.doc_chunks;
    const DocAssembly::Step step = estimate_asm_.accept(reply);
    if (step == DocAssembly::Step::kMore) {
        return;
    }
    translagatr::FieldEstimateHeader header;
    const bool                 good =
        step == DocAssembly::Step::kComplete && map_cache_id_ != 0 &&
        translagatr::decodeFieldEstimateHeader(estimate_asm_.data(), estimate_asm_.length(), header) &&
        header.estimate_id == estimate_asm_.docId() &&
        translagatr::validateFieldEstimate(estimate_asm_.data(), estimate_asm_.length(),
                                     map_cache_.data(), map_cache_len_,
                                     map_cache_id_) == translagatr::DocError::kNone;
    if (!good) {
        ++stats_.doc_rejects;
        estimateFailure(now);
        return;
    }
    estimate_failures_ = 0;
    publish(now);
}

// Unsupported version or op: a terminal error, not a retry storm. The session
// is dropped and HELLO repeats every hello_backoff until the Pi answers.
void Client::incompatible(Kind kind, const translagatr::BrainReply& reply, Seconds now) {
    const bool version = reply.version != translagatr::kBrainLinkVersion ||
                         reply.result == translagatr::kResultUnsupportedVersion;
    error_             = version ? LinkError::kUnsupportedVersion : LinkError::kUnsupportedOp;
    peer_version_      = reply.version;
    const uint8_t result = version ? uint8_t{translagatr::kResultUnsupportedVersion} : reply.result;
    if (kind == Kind::kPlacement && placementPending()) {
        settlePlacement(PlacementResult::kRejected, result);
    }
    if (kind == Kind::kControl && controlPending()) {
        settleControl(ControlResult::kRejected, result);
    }
    if (kind == Kind::kWheels && wheelsPending()) {
        settleWheels(WheelResult::kRejected, result);
    }
    if (session_ != 0) {
        loseSession();
    }
    hello_.len  = 0;
    next_hello_ = now + config_.hello_backoff;
}

// ---------------------------------------------------------------------------
// Send
// ---------------------------------------------------------------------------

void Client::attemptFailed(Seconds now) {
    const Kind kind = outstanding_;
    outstanding_    = Kind::kNone;
    next_send_      = now + config_.request_gap;
    switch (kind) {
    case Kind::kPlacement:
        if (!placementPending() || placement_tx_.attempts == 0 ||
            sent_.request_id != placement_tx_.request.request_id) {
            break;
        }
        if (placementExhausted(now)) {
            settlePlacement(PlacementResult::kTimedOut, placement_.result);
        } else if (!placement_acked_) {
            // The Pi may not have it: resend before any newer request id,
            // which would make this one stale.
            retry_first_        = Kind::kPlacement;
            state_before_retry_ = false;
        }
        break;
    case Kind::kControl:
        if (!controlPending() || control_tx_.attempts == 0 ||
            sent_.request_id != control_tx_.request.request_id) {
            break;
        }
        ++control_misses_;
        if (controlExhausted(now)) {
            settleControl(ControlResult::kTimedOut, control_.result);
        } else if (control_answered_) {
            // The Pi holds the record and answers this id after newer ones:
            // asked again after control_retry, state polls in between.
            next_control_ = now + config_.control_retry;
        } else {
            // The Pi may not have it: resend before a newer id makes it stale.
            retry_first_ = Kind::kControl;
        }
        break;
    case Kind::kWheels:
        if (wheelsPending() && !wheels_wanted_) {
            settleWheels(WheelResult::kTimedOut, wheel_.result);
        }
        break;
    case Kind::kProfileWrite:
    case Kind::kProfileApply:
        // Idempotent by content, sent again with a new id; a state poll first.
        next_profile_ = now + config_.state_period;
        break;
    default:
        // HELLO resends the same bytes; everything else is never resent.
        break;
    }
}

void Client::transmit(Seconds now) {
    if (outstanding_ != Kind::kNone || now < next_send_) {
        return;
    }
    const Kind kind = choose(now);
    retry_first_    = Kind::kNone;
    if (kind == Kind::kNone) {
        return;
    }
    Transaction& tx = kind == Kind::kHello       ? hello_
                      : kind == Kind::kPlacement ? placement_tx_
                      : kind == Kind::kControl   ? control_tx_
                                                 : once_tx_;
    if (tx.len == 0) {
        return;
    }
    drain();

    if (tx.attempts == 0) {
        tx.first_sent = now;
    } else {
        ++stats_.resends;
    }
    ++tx.attempts;
    if (kind == Kind::kState) {
        next_state_ = now + config_.state_period;
    }
    if (kind == Kind::kTelemetry) {
        // Keeps the grid unless half a period late: one report per period
        // from the application is never replaced by drift.
        const Seconds period = config_.telemetry_period;
        next_telemetry_ =
            now - next_telemetry_ <= period / 2 ? next_telemetry_ + period : now + period;
    }
    if (kind == Kind::kHello && error_ != LinkError::kNone) {
        next_hello_ = now + config_.hello_backoff;
    }
    after_state_  = false;
    outstanding_  = kind;
    sent_         = tx.request;
    sent_at_      = now;
    sent_timeout_ = responseTimeout(tx.request.op, tx.len);
    if (!port_.write(tx.frame, static_cast<int>(tx.len))) {
        ++stats_.write_errors;
        attemptFailed(now);
        return;
    }
    ++stats_.requests;
    if (kind == Kind::kPathReport) {
        ++stats_.path_reports;
    }
    if (kind == Kind::kTelemetry) {
        ++stats_.telemetry_reports;
    }
}

Client::Kind Client::choose(Seconds now) {
    if (session_ == 0) {
        if (now < next_hello_) {
            return Kind::kNone;
        }
        if (hello_.len == 0) {
            translagatr::BrainRequest request;
            request.op    = translagatr::kOpHello;
            request.nonce = takeNonce();
            start(hello_, request);
        }
        return Kind::kHello;
    }

    if (retry_first_ == Kind::kPlacement && placementPending() && !placement_acked_ &&
        placement_tx_.len != 0) {
        return Kind::kPlacement;
    }
    if (retry_first_ == Kind::kControl && controlPending() && control_tx_.len != 0) {
        return Kind::kControl;
    }

    // 2. Placement.
    if (placementPending() && !placement_acked_ && now >= next_placement_) {
        if (placement_tx_.len != 0 && placementExhausted(now)) {
            settlePlacement(PlacementResult::kTimedOut, placement_.result);
        } else {
            // A pending placement needs live bench IMU reports to become
            // applicable: one state poll between Pending resends.
            if (state_before_retry_ && config_.bench_imu) {
                return statePoll();
            }
            if (placement_tx_.len == 0) {
                translagatr::BrainRequest request = placement_request_;
                request.session             = session_;
                start(placement_tx_, request);
            }
            return Kind::kPlacement;
        }
    }

    // 3. Control.
    if (controlPending() && now >= next_control_) {
        if (control_tx_.len != 0 && controlExhausted(now)) {
            settleControl(ControlResult::kTimedOut, control_.result);
        } else {
            if (control_tx_.len == 0) {
                translagatr::BrainRequest request;
                request.op      = translagatr::kOpControl;
                request.session = session_;
                request.action  = control_.action;
                start(control_tx_, request);
            }
            return Kind::kControl;
        }
    }

    // 4. Profile sync. After an APPLY Ok only a state poll can confirm it.
    if (ready_ && now >= next_profile_ &&
        (profile_.state == ProfileSync::kWriting ||
         (profile_.state == ProfileSync::kApplying && !profile_confirming_))) {
        if (profile_.state == ProfileSync::kWriting && profile_.received >= profile_len_) {
            profile_.state      = ProfileSync::kApplying;
            profile_confirming_ = false;
        }
        translagatr::BrainRequest request;
        request.session    = session_;
        request.profile_id = profile_.id;
        request.total_len  = profile_len_;
        if (profile_.state == ProfileSync::kWriting) {
            const uint16_t offset = profile_.received;
            const uint16_t left   = static_cast<uint16_t>(profile_len_ - offset);
            request.op            = translagatr::kOpProfileWrite;
            request.offset        = offset;
            request.data_len      = static_cast<uint8_t>(
                std::min<uint16_t>(left, translagatr::kProfileChunkMax));
            std::copy(config_.profile.bytes + offset,
                      config_.profile.bytes + offset + request.data_len, request.data);
            start(once_tx_, request);
            return Kind::kProfileWrite;
        }
        request.op = translagatr::kOpProfileApply;
        start(once_tx_, request);
        return Kind::kProfileApply;
    }

    // 5. State poll. When exchanges outlast the poll period the poll is
    // always due; a waiting transfer then goes after kPollsBeforeTransfer
    // polls in a row instead of never.
    const bool transfer      = wheels_wanted_ || mapWanted(now) || estimateWanted(now) || path_pending_;
    const bool due           = now >= next_state_;
    const bool transfer_turn = transfer && starved_polls_ >= kPollsBeforeTransfer;
    if (telemetry_pending_ && now - telemetry_at_ > 2 * config_.telemetry_period) {
        // Held through an outage: it would reach the Pi looking fresh.
        ++stats_.telemetry_dropped;
        telemetry_pending_ = false;
    }
    // When every exchange outlasts the poll's idle window the on-time slot
    // (10) never comes. Half a period late, a report takes the first slot
    // after a state reply from the due poll or a waiting transfer: once per
    // 1.5 periods at most, and never from a transfer that already yielded
    // its polls, so one exchange at most between two polls.
    if ((due || transfer) && !transfer_turn &&
        telemetrySlot(now, config_.telemetry_period / 2) && startTelemetry()) {
        ++stats_.telemetry_overdue;
        return Kind::kTelemetry;
    }
    if (due && !transfer_turn) {
        starved_polls_ = transfer ? starved_polls_ + 1 : 0;
        return statePoll();
    }
    starved_polls_ = 0;

    // 6-9. Reads and transfers, normally only while the state poll is not due.
    if (wheels_wanted_) {
        translagatr::BrainRequest request;
        request.op      = translagatr::kOpReadWheels;
        request.session = session_;
        wheels_wanted_  = false;
        start(once_tx_, request);
        return Kind::kWheels;
    }
    if (mapWanted(now)) {
        const uint32_t target = state_.state.map_id;
        if (!map_asm_.active() || map_asm_.docId() != target) {
            map_asm_.begin(translagatr::kDocFieldMap, target);
        }
        translagatr::BrainRequest request;
        request.op         = translagatr::kOpReadDoc;
        request.session    = session_;
        request.doc_kind   = translagatr::kDocFieldMap;
        request.doc_id     = target;
        request.doc_offset = map_asm_.offset();
        request.max_len    = map_asm_.maxLen();
        start(once_tx_, request);
        return Kind::kMapChunk;
    }
    if (estimateWanted(now)) {
        if (!estimate_asm_.active()) {
            estimate_asm_.begin(translagatr::kDocFieldEstimate, state_.state.estimate_id);
            next_estimate_ = now + config_.field_period;
        }
        translagatr::BrainRequest request;
        request.op         = translagatr::kOpReadDoc;
        request.session    = session_;
        request.doc_kind   = translagatr::kDocFieldEstimate;
        request.doc_id     = estimate_asm_.docId();
        request.doc_offset = estimate_asm_.offset();
        request.max_len    = estimate_asm_.maxLen();
        start(once_tx_, request);
        return Kind::kEstimateChunk;
    }
    if (path_pending_) {
        translagatr::BrainRequest request = path_request_;
        request.session             = session_;
        path_pending_               = false;
        start(once_tx_, request);
        return Kind::kPathReport;
    }

    // 10. Telemetry on time. Not a waiting transfer, so not the turn of a due
    // poll. Only in the first slot after a state reply, the earliest in the
    // poll's idle window, which least pushes the next poll back; a grid time
    // inside the window waits for the next reply.
    if (now < next_state_ && telemetrySlot(now, 0) && startTelemetry()) {
        return Kind::kTelemetry;
    }
    return Kind::kNone;
}

// The first slot after a state reply, the grid time passed before that reply
// and now at least late past it.
bool Client::telemetrySlot(Seconds now, Seconds late) const {
    return telemetry_pending_ && after_state_ && next_telemetry_ <= state_reply_at_ &&
           now >= next_telemetry_ + late;
}

bool Client::startTelemetry() {
    translagatr::BrainRequest request = telemetry_request_;
    request.session                   = session_;
    telemetry_pending_                = false;
    start(once_tx_, request);
    if (once_tx_.len == 0) {
        ++stats_.telemetry_dropped; // the codec refused the body
        return false;
    }
    return true;
}

Client::Kind Client::statePoll() {
    translagatr::BrainRequest request;
    request.op      = translagatr::kOpGetState;
    request.session = session_;
    if (config_.bench_imu) {
        bench_sample_             = config_.bench_imu();
        request.imu_flags         = bench_sample_.valid ? uint8_t{translagatr::kBenchImuValid} : uint8_t{0};
        request.imu_stamp_ms      = bench_sample_.stamp_ms;
        request.imu_rotation_mdeg = bench_sample_.rotation_mdeg;
    }
    start(once_tx_, request);
    state_before_retry_ = false;
    return Kind::kState;
}

void Client::start(Transaction& tx, translagatr::BrainRequest request) {
    request.version    = translagatr::kBrainLinkVersion;
    request.request_id = takeRequestId();
    tx.request         = request;
    tx.len             = translagatr::encodeBrainRequest(request, tx.frame, sizeof(tx.frame));
    tx.attempts        = 0;
    tx.first_sent      = 0;
}

// Anything received before a request is stale by definition.
void Client::drain() {
    uint8_t buf[64];
    for (int i = 0; i < kMaxReads; ++i) {
        const int n = port_.read(buf, static_cast<int>(sizeof(buf)));
        if (n < 0) {
            ++stats_.read_errors;
            break;
        }
        if (n == 0) {
            break;
        }
        stats_.drained_bytes += static_cast<uint32_t>(n);
    }
    reader_.reset();
}

// Pi restart, unknown session, or an incompatible peer. Nothing of the old
// session carries over and nothing in flight is ever resent. The complete
// map and the published field stay.
void Client::loseSession() {
    ++stats_.session_losses;
    session_     = 0;
    ready_       = false;
    state_       = StateSample{};
    state_count_ = 0;
    if (placementPending()) {
        settlePlacement(PlacementResult::kSessionLost, placement_.result);
    }
    if (controlPending()) {
        settleControl(ControlResult::kSessionLost, control_.result);
    }
    if (wheelsPending()) {
        settleWheels(WheelResult::kSessionLost, wheel_.result);
    }
    if (profile_.state != ProfileSync::kNone && profile_.state != ProfileSync::kInvalid) {
        profile_.state    = ProfileSync::kWaiting;
        profile_.reason   = translagatr::kProfileReasonNone;
        profile_.detail   = 0;
        profile_.received = 0;
        profile_failures_ = 0;
        next_profile_     = kLongAgo;
    }
    profile_confirming_ = false;
    map_asm_.clear();
    estimate_asm_.clear();
    map_failures_      = 0;
    estimate_failures_ = 0;
    next_map_          = kLongAgo;
    next_estimate_     = kLongAgo;
    for (EstimateSeen& seen : estimate_seen_) {
        seen = EstimateSeen{};
    }
    if (path_pending_) {
        ++stats_.paths_dropped;
        path_pending_ = false;
    }
    if (telemetry_pending_) {
        ++stats_.telemetry_dropped;
        telemetry_pending_ = false;
    }
    telemetry_unsupported_ = false;
    hello_.len          = 0;
    retry_first_        = Kind::kNone;
    state_before_retry_ = false;
    starved_polls_      = 0;
}

void Client::settlePlacement(PlacementResult state, uint8_t result) {
    placement_.state    = state;
    placement_.result   = result;
    placement_acked_    = false;
    placement_tx_.len   = 0;
    state_before_retry_ = false;
}

bool Client::placementExhausted(Seconds now) const {
    return placement_tx_.attempts >= config_.placement_attempts ||
           now - placement_tx_.first_sent >= config_.placement_deadline;
}

void Client::settleControl(ControlResult state, uint8_t result) {
    control_.state  = state;
    control_.result = result;
    control_tx_.len = 0;
}

// Before the first reply the Pi may not have the request: a few sends within
// control_deadline. Once answered it holds the record: control_wait only.
bool Client::controlExhausted(Seconds now) const {
    if (control_answered_) {
        return now - control_tx_.first_sent >= config_.control_wait;
    }
    return control_misses_ >= config_.control_attempts ||
           now - control_tx_.first_sent >= config_.control_deadline;
}

void Client::settleWheels(WheelResult state, uint8_t result) {
    wheel_.state   = state;
    wheel_.result  = result;
    wheels_wanted_ = false;
}

// A new document: nothing of an earlier upload counts for it.
void Client::configureProfile(const ProfileDocument& doc) {
    config_.profile     = doc;
    profile_            = ProfileStatus{};
    profile_.id         = profileId(doc);
    profile_len_        = doc.len;
    uint8_t reason      = 0;
    uint8_t detail      = 0;
    profile_.state      = profileValid(doc, reason, detail) ? ProfileSync::kWaiting
                                                            : ProfileSync::kInvalid;
    profile_.reason     = reason;
    profile_.detail     = detail;
    profile_failures_   = 0;
    profile_confirming_ = false;
    next_profile_       = kLongAgo;
}

// The Pi is authoritative for what it runs. A mismatch restarts the upload
// unless a rejection is settled. Only here does a profile become kApplied, so
// the state that robot() and readiness read always describes it.
void Client::profileFromState(const translagatr::BrainState& state) {
    switch (profile_.state) {
    case ProfileSync::kNone:
    case ProfileSync::kInvalid:
    case ProfileSync::kRejected:
        return;
    default:
        break;
    }
    const bool mine = state.profile_state == translagatr::kProfileApplied && state.profile_id == profile_.id;
    if (mine) {
        profile_.state      = ProfileSync::kApplied;
        profile_.reason     = translagatr::kProfileReasonNone;
        profile_.detail     = 0;
        profile_failures_   = 0;
        profile_confirming_ = false;
        return;
    }
    if (profile_confirming_) {
        // Ok for this id, yet this later state shows another: upload again.
        ++stats_.unexpected;
        profileFailure(translagatr::kResultOk);
        next_profile_ = kLongAgo;
        return;
    }
    if (profile_.state == ProfileSync::kWaiting || profile_.state == ProfileSync::kApplied) {
        profile_.state    = ProfileSync::kWriting;
        profile_.received = 0;
        next_profile_     = kLongAgo;
    }
}

void Client::profileFailure(uint8_t result) {
    profile_.state      = ProfileSync::kWriting;
    profile_.received   = 0;
    profile_confirming_ = false;
    if (++profile_failures_ >= kFailuresBeforeBackoff) {
        settleProfileRejected(result, translagatr::kProfileReasonNone, 0);
    }
}

void Client::settleProfileRejected(uint8_t result, uint8_t reason, uint8_t detail) {
    profile_.state      = ProfileSync::kRejected;
    profile_.result     = result;
    profile_.reason     = reason;
    profile_.detail     = detail;
    profile_failures_   = 0;
    profile_confirming_ = false;
}

bool Client::mapWanted(Seconds now) const {
    return ready_ && state_.valid && state_.state.map_id != 0 &&
           state_.state.map_id != map_cache_id_ && now >= next_map_;
}

bool Client::estimateWanted(Seconds now) const {
    if (!ready_ || !state_.valid) {
        return false;
    }
    const translagatr::BrainState& s = state_.state;
    if (map_cache_id_ == 0 || s.map_id != map_cache_id_ || s.estimate_id == 0) {
        return false;
    }
    if (estimate_asm_.active()) {
        return true;
    }
    if (now < next_estimate_) {
        return false;
    }
    return s.estimate_id != field_.estimate_id || field_.pi_instance != pi_instance_ ||
           field_.map_id != map_cache_id_;
}

void Client::mapFailure(Seconds now) {
    map_asm_.clear();
    if (++map_failures_ >= kFailuresBeforeBackoff) {
        map_failures_ = 0;
        next_map_     = now + config_.transfer_backoff;
    }
}

void Client::estimateFailure(Seconds now) {
    estimate_asm_.clear();
    if (++estimate_failures_ >= kFailuresBeforeBackoff) {
        estimate_failures_ = 0;
        next_estimate_     = now + config_.transfer_backoff;
    } else {
        next_estimate_ = kLongAgo; // again at once, from the newest id
    }
}

void Client::publish(Seconds now) {
    field_.generation = field_.generation == UINT32_MAX ? 1 : field_.generation + 1;
    if (field_.map_id != map_cache_id_ || field_.map.empty()) {
        field_.map.assign(map_cache_.begin(), map_cache_.begin() + map_cache_len_);
        field_.map_id = map_cache_id_;
    }
    field_.estimate.assign(estimate_asm_.data(), estimate_asm_.data() + estimate_asm_.length());
    field_.estimate_id    = estimate_asm_.docId();
    field_.pi_instance    = pi_instance_;
    field_.session        = session_;
    field_.completed_at   = now;
    field_.snapshot_after = kUnknown;
    for (const EstimateSeen& seen : estimate_seen_) {
        if (seen.id == field_.estimate_id) {
            field_.snapshot_after = seen.after;
        }
    }
    estimate_asm_.clear();
    ++stats_.estimates;
}

void Client::noteEstimateId(uint32_t id, Seconds after) {
    if (id == 0) {
        return;
    }
    for (const EstimateSeen& seen : estimate_seen_) {
        if (seen.id == id) {
            return;
        }
    }
    for (int i = 3; i > 0; --i) {
        estimate_seen_[i] = estimate_seen_[i - 1];
    }
    estimate_seen_[0] = EstimateSeen{id, after};
}

// Per boot counter, 1..65535, wraps to 1, never 0.
uint16_t Client::takeRequestId() {
    const uint16_t id = next_request_id_;
    next_request_id_  = next_request_id_ == 0xFFFF ? 1 : static_cast<uint16_t>(id + 1);
    return id;
}

uint32_t Client::takeNonce() {
    uint32_t nonce = nonce_source_ ? nonce_source_() : 0;
    if (nonce == 0 || nonce == last_nonce_) {
        nonce = last_nonce_ + kNonceStep;
        if (nonce == 0) {
            nonce = kNonceStep;
        }
    }
    last_nonce_ = nonce;
    return nonce;
}

} // namespace communigatr
