// client.cpp

#include "communigatr/client.h"

#include <limits>
#include <utility>

namespace communigatr
{

namespace
{

constexpr Seconds  kLongAgo   = std::numeric_limits<Seconds>::lowest();
constexpr int      kMaxReads  = 16; // 64 byte reads per receive or drain
constexpr uint32_t kNonceStep = 0x9E3779B9u;

// Anchor revisions only grow within one pi_instance; serial order handles wrap.
bool anchorReached(uint32_t reported, uint32_t wanted) {
    return static_cast<uint32_t>(reported - wanted) < 0x80000000u;
}

} // namespace

Client::Client(BytePort& port, std::function<uint32_t()> nonce, const ClientConfig& config)
    : port_(port), nonce_source_(std::move(nonce)), config_(config) {
    next_send_      = kLongAgo;
    next_hello_     = kLongAgo;
    next_state_     = kLongAgo;
    next_placement_ = kLongAgo;
    next_select_    = kLongAgo;
}

void Client::poll(Seconds now) {
    // The operation deadline includes confirmation after SET_POSE Ok. Check
    // before receive so a reply processed after the deadline cannot revive it.
    // Keep any outstanding bus response window until its reply or timeout.
    if (placementPending() && placement_tx_.attempts > 0 &&
        now - placement_tx_.first_sent >= config_.placement_deadline) {
        settlePlacement(PlacementResult::kTimedOut, placement_.result);
    }
    receive(now);
    if (outstanding_ != Kind::kNone && now - sent_at_ >= config_.response_timeout) {
        ++stats_.timeouts;
        attemptFailed(now);
    }
    transmit(now);
}

PlacementTicket Client::submitPlacement(int32_t x_mm, int32_t y_mm, int32_t heading_cdeg) {
    if (placementPending()) {
        return 0;
    }
    last_ticket_ = last_ticket_ == UINT32_MAX ? 1 : last_ticket_ + 1;

    placement_        = PlacementStatus{};
    placement_.ticket = last_ticket_;
    placement_.state  = PlacementResult::kPending;
    placement_acked_  = false;
    placement_tx_     = Transaction{};
    next_placement_   = kLongAgo;
    bench_poll_before_retry_ = false;

    placement_request_              = gatr2::BrainRequest{};
    placement_request_.op           = gatr2::kOpSetPose;
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

SelectionState Client::selection() const {
    if (want_ == 0) {
        return SelectionState::kNotRequested;
    }
    if (!pi_selection_known_ || pi_selection_ != want_ || select_tx_.len != 0) {
        return SelectionState::kPending;
    }
    if (selection_result_ == gatr2::kResultUnknownLandmark) {
        return SelectionState::kUnknownLandmark;
    }
    if (selection_result_ == gatr2::kResultLandmarkUnsupported) {
        return SelectionState::kUnsupported;
    }
    const bool reported =
        state_.valid && state_count_ > selection_mark_ && state_.state.landmark_id == want_;
    return reported ? SelectionState::kActive : SelectionState::kPending;
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
    gatr2::BrainReply reply;
    if (len < 3 || frame[2] != gatr2::kFrameBrainReply ||
        !gatr2::decodeBrainReply(frame, len, reply)) {
        ++stats_.bad_frames;
        return;
    }
    if (!correlates(reply)) {
        ++stats_.uncorrelated;
        return;
    }
    handleReply(reply, now);
}

bool Client::correlates(const gatr2::BrainReply& reply) const {
    if (outstanding_ == Kind::kNone) {
        return false;
    }
    if (reply.op != sent_.op || reply.request_id != sent_.request_id) {
        return false;
    }
    // Another version's body is unreadable: header echo only.
    if (reply.version != gatr2::kBrainLinkVersion) {
        return reply.session == sent_.session;
    }
    if (sent_.op == gatr2::kOpHello) {
        return reply.nonce == sent_.nonce;
    }
    return reply.session == sent_.session;
}

void Client::handleReply(const gatr2::BrainReply& reply, Seconds now) {
    const Kind    kind       = outstanding_;
    const Seconds round_trip = now - sent_at_;
    outstanding_             = Kind::kNone;
    next_send_               = now + config_.request_gap;
    ++stats_.replies;

    if (reply.version != gatr2::kBrainLinkVersion ||
        reply.result == gatr2::kResultUnsupportedVersion ||
        reply.result == gatr2::kResultUnsupportedOp) {
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
    if (reply.result == gatr2::kResultUnknownSession) {
        loseSession();
        return;
    }
    last_reply_ = now;
    switch (kind) {
    case Kind::kPlacement:
        handlePlacement(reply, now);
        break;
    case Kind::kSelect:
        handleSelect(reply, now);
        break;
    case Kind::kState:
        handleState(reply, now, round_trip);
        break;
    default:
        break;
    }
}

void Client::handleHello(const gatr2::BrainReply& reply, Seconds now) {
    hello_.len = 0;
    if (reply.result == gatr2::kResultStale) {
        ++stats_.stale_hellos;
        return;
    }
    if (reply.result != gatr2::kResultOk || reply.session == 0) {
        ++stats_.unexpected;
        next_hello_ = now + config_.hello_backoff;
        return;
    }
    ++stats_.sessions;
    session_            = reply.session;
    pi_instance_        = reply.pi_instance;
    last_reply_         = now;
    ready_              = false;
    state_              = StateSample{};
    state_count_        = 0;
    next_state_         = now;
    pi_selection_       = 0;
    pi_selection_known_ = true;
    selection_result_   = gatr2::kResultOk;
    select_tx_.len      = 0;
    next_select_        = kLongAgo;
}

void Client::handlePlacement(const gatr2::BrainReply& reply, Seconds now) {
    if (!placementPending() || placement_acked_ || placement_tx_.attempts == 0 ||
        sent_.request_id != placement_tx_.request.request_id) {
        return;
    }
    placement_.result = reply.result;
    if (reply.result == gatr2::kResultOk) {
        placement_acked_           = true;
        placement_.odometry_epoch  = reply.odometry_epoch;
        placement_.anchor_revision = reply.anchor_revision;
        placement_mark_            = state_count_;
        placement_tx_.len          = 0;
        return;
    }
    if (reply.result == gatr2::kResultPending) {
        if (placementExhausted(now)) {
            settlePlacement(PlacementResult::kTimedOut, reply.result);
        } else {
            next_placement_ = now + config_.pending_retry;
            bench_poll_before_retry_ = static_cast<bool>(config_.bench_imu);
        }
        return;
    }
    settlePlacement(PlacementResult::kRejected, reply.result);
}

void Client::handleSelect(const gatr2::BrainReply& reply, Seconds now) {
    const gatr2::BrainRequest& request   = sent_;
    const bool                 selecting = (request.select_flags & gatr2::kSelectFlagSelected) != 0;
    select_tx_.len                       = 0;

    const bool echo_ok =
        reply.landmark_id == request.landmark_id && reply.select_flags == request.select_flags;
    if (reply.result == gatr2::kResultOk && echo_ok) {
        pi_selection_ = selecting ? request.landmark_id : 0;
    } else if (selecting && (reply.result == gatr2::kResultUnknownLandmark ||
                             reply.result == gatr2::kResultLandmarkUnsupported)) {
        pi_selection_ = request.landmark_id;
    } else {
        ++stats_.unexpected;
        selectFailed(now);
        return;
    }
    pi_selection_known_ = true;
    selection_result_   = reply.result;
    selection_mark_     = state_count_;
}

void Client::handleState(const gatr2::BrainReply& reply, Seconds now, Seconds round_trip) {
    if (reply.result != gatr2::kResultOk) {
        ++stats_.unexpected;
        return;
    }
    state_.valid       = true;
    state_.pi_instance = reply.pi_instance;
    state_.session     = reply.session;
    state_.state       = reply.state;
    state_.received_at = now;
    state_.round_trip  = round_trip;
    ++state_count_;
    ready_ = true;
    error_ = LinkError::kNone;

    if (placementPending() && placement_acked_ && state_count_ > placement_mark_ &&
        anchorReached(reply.state.anchor_revision, placement_.anchor_revision)) {
        settlePlacement(PlacementResult::kApplied, gatr2::kResultOk);
    }
}

// Unsupported version or op: a terminal error, not a retry storm. The session
// is dropped and HELLO repeats every hello_backoff until the Pi answers.
void Client::incompatible(Kind kind, const gatr2::BrainReply& reply, Seconds now) {
    const bool version = reply.version != gatr2::kBrainLinkVersion ||
                         reply.result == gatr2::kResultUnsupportedVersion;
    error_             = version ? LinkError::kUnsupportedVersion : LinkError::kUnsupportedOp;
    peer_version_      = reply.version;
    if (kind == Kind::kPlacement && placementPending()) {
        const uint8_t result = version ? uint8_t{gatr2::kResultUnsupportedVersion} : reply.result;
        settlePlacement(PlacementResult::kRejected, result);
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
        if (placementPending() && placement_tx_.attempts > 0 &&
            sent_.request_id == placement_tx_.request.request_id && placementExhausted(now)) {
            settlePlacement(PlacementResult::kTimedOut, placement_.result);
        }
        if (placementPending() && config_.bench_imu) {
            bench_poll_before_retry_ = true;
        }
        break;
    case Kind::kSelect:
        if (select_tx_.attempts >= config_.select_attempts) {
            selectFailed(now);
        }
        break;
    default:
        // HELLO resends the same bytes; GET_STATE is never resent.
        break;
    }
}

void Client::transmit(Seconds now) {
    if (outstanding_ != Kind::kNone || now < next_send_) {
        return;
    }
    const Kind kind = choose(now);
    if (kind == Kind::kNone) {
        return;
    }
    Transaction& tx = transaction(kind);
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
    if (kind == Kind::kHello && error_ != LinkError::kNone) {
        next_hello_ = now + config_.hello_backoff;
    }
    outstanding_ = kind;
    sent_        = tx.request;
    sent_at_     = now;
    if (!port_.write(tx.frame, static_cast<int>(tx.len))) {
        ++stats_.write_errors;
        attemptFailed(now);
        return;
    }
    ++stats_.requests;
}

Client::Kind Client::choose(Seconds now) {
    if (session_ == 0) {
        if (now < next_hello_) {
            return Kind::kNone;
        }
        if (hello_.len == 0) {
            gatr2::BrainRequest request;
            request.op    = gatr2::kOpHello;
            request.nonce = takeNonce();
            start(hello_, request);
        }
        return Kind::kHello;
    }

    const auto statePoll = [this]() {
        gatr2::BrainRequest request;
        request.op = gatr2::kOpGetState;
        request.session = session_;
        if (config_.bench_imu) {
            const BenchImuSample sample = config_.bench_imu();
            request.op = gatr2::kOpGetStateWithImu;
            request.imu_flags = sample.valid ? uint8_t{gatr2::kBenchImuValid} : uint8_t{0};
            request.imu_stamp_ms = sample.stamp_ms;
            request.imu_rotation_mdeg = sample.rotation_mdeg;
        }
        start(state_tx_, request);
        bench_poll_before_retry_ = false;
        return Kind::kState;
    };
    // A pending placement needs live IMU reports to become applicable. Always
    // send one between retries, even when placement is already due again.
    if (bench_poll_before_retry_ && config_.bench_imu) {
        return statePoll();
    }

    if (placementPending() && !placement_acked_ && now >= next_placement_) {
        if (placement_tx_.len != 0 && placementExhausted(now)) {
            settlePlacement(PlacementResult::kTimedOut, placement_.result);
        } else {
            if (placement_tx_.len == 0) {
                gatr2::BrainRequest request = placement_request_;
                request.session             = session_;
                start(placement_tx_, request);
            }
            return Kind::kPlacement;
        }
    }

    if (selectionNeeded() && now >= next_select_) {
        const uint8_t id    = want_;
        const uint8_t flags = want_ != 0 ? gatr2::kSelectFlagSelected : 0;
        if (select_tx_.len != 0 &&
            (select_tx_.request.landmark_id != id || select_tx_.request.select_flags != flags)) {
            // Abandoned after a send: the Pi may hold either selection.
            if (select_tx_.attempts > 0) {
                pi_selection_known_ = false;
            }
            select_tx_.len = 0;
        }
        if (select_tx_.len == 0) {
            gatr2::BrainRequest request;
            request.op           = gatr2::kOpSelectLandmark;
            request.session      = session_;
            request.landmark_id  = id;
            request.select_flags = flags;
            start(select_tx_, request);
        }
        return Kind::kSelect;
    }

    if (now >= next_state_) {
        return statePoll();
    }
    return Kind::kNone;
}

Client::Transaction& Client::transaction(Kind kind) {
    switch (kind) {
    case Kind::kHello:
        return hello_;
    case Kind::kPlacement:
        return placement_tx_;
    case Kind::kSelect:
        return select_tx_;
    default:
        return state_tx_;
    }
}

void Client::start(Transaction& tx, gatr2::BrainRequest request) {
    request.version    = gatr2::kBrainLinkVersion;
    request.request_id = takeRequestId();
    tx.request         = request;
    tx.len             = gatr2::encodeBrainRequest(request, tx.frame, sizeof(tx.frame));
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
// session carries over and nothing in flight is ever resent.
void Client::loseSession() {
    ++stats_.session_losses;
    session_     = 0;
    ready_       = false;
    state_       = StateSample{};
    state_count_ = 0;
    if (placementPending()) {
        settlePlacement(PlacementResult::kSessionLost, placement_.result);
    }
    pi_selection_       = 0;
    pi_selection_known_ = true;
    selection_result_   = gatr2::kResultOk;
    select_tx_.len      = 0;
    next_select_        = kLongAgo;
    hello_.len          = 0;
}

void Client::settlePlacement(PlacementResult state, uint8_t result) {
    placement_.state  = state;
    placement_.result = result;
    placement_acked_  = false;
    placement_tx_.len = 0;
    bench_poll_before_retry_ = false;
}

bool Client::placementExhausted(Seconds now) const {
    return placement_tx_.attempts >= config_.placement_attempts ||
           now - placement_tx_.first_sent >= config_.placement_deadline;
}

bool Client::selectionNeeded() const {
    return select_tx_.len != 0 || !pi_selection_known_ || pi_selection_ != want_;
}

// The Pi may or may not have taken it; a new transaction follows later.
void Client::selectFailed(Seconds now) {
    select_tx_.len      = 0;
    pi_selection_known_ = false;
    next_select_        = now + config_.select_retry;
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
