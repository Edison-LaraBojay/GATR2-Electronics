// fake_pi.cpp

#include "sim/fake_pi.h"

namespace communigatr
{

namespace
{

uint32_t seed(uint32_t pi_instance) {
    const uint32_t s = pi_instance ^ 0xA5A5A5A5u;
    return s == 0 ? 1 : s;
}

bool sameBody(const gatr2::BrainRequest& a, const gatr2::BrainRequest& b) {
    return a.op == b.op && a.x_mm == b.x_mm && a.y_mm == b.y_mm &&
           a.heading_cdeg == b.heading_cdeg && a.landmark_id == b.landmark_id &&
           a.select_flags == b.select_flags;
}

} // namespace

FakePi::FakePi(uint32_t pi_instance) : pi_instance_(pi_instance), rng_(seed(pi_instance)) {}

std::vector<std::vector<uint8_t>> FakePi::receive(const uint8_t* data, std::size_t len) {
    std::vector<std::vector<uint8_t>> replies;
    for (std::size_t i = 0; i < len; ++i) {
        if (!reader_.push(data[i])) {
            continue;
        }
        do {
            gatr2::BrainRequest request;
            if (reader_.frameType() != gatr2::kFrameBrainRequest ||
                !gatr2::decodeBrainRequest(reader_.frame(), reader_.frameLen(), request)) {
                continue;
            }
            const gatr2::BrainReply reply = answer(request);
            std::vector<uint8_t>    frame(gatr2::kMaxFrameLen);
            frame.resize(
                gatr2::encodeBrainReply(reply, frame.data(), static_cast<uint16_t>(frame.size())));
            replies.push_back(frame);
        } while (reader_.next());
    }
    return replies;
}

gatr2::BrainReply FakePi::answer(const gatr2::BrainRequest& request) {
    requests_.push_back(request);
    tickPlacement();

    gatr2::BrainReply reply;
    reply.version     = version_;
    reply.op          = request.op;
    reply.session     = request.session;
    reply.request_id  = request.request_id;
    reply.pi_instance = pi_instance_;
    reply.nonce       = request.nonce;

    if (request.version != version_) {
        reply.result = gatr2::kResultUnsupportedVersion;
        return reply;
    }
    if (gatr2::brainRequestLen(request.op) == 0 || request.op == unsupported_op_) {
        reply.result = gatr2::kResultUnsupportedOp;
        return reply;
    }

    if (request.op == gatr2::kOpHello) {
        if (session_ != 0 && request.nonce == open_nonce_ &&
            request.request_id == open_request_id_ && !accepted_other_) {
            reply.session = session_; // retry of the opening HELLO
            return reply;
        }
        for (int i = 0; i < nonce_count_; ++i) {
            if (nonce_ring_[i] == request.nonce) {
                reply.result = gatr2::kResultStale;
                return reply;
            }
        }
        openSession(request);
        reply.session = session_;
        return reply;
    }

    if (session_ == 0 || request.session != session_) {
        reply.result = gatr2::kResultUnknownSession;
        return reply;
    }
    const uint16_t id = request.request_id;
    if (id == 0) {
        reply.result = gatr2::kResultInvalidArgument;
        return reply;
    }
    const uint16_t ahead = static_cast<uint16_t>(id - newest_);
    if (!have_newest_ || (ahead >= 1 && ahead <= 32767)) {
        have_newest_    = true;
        newest_         = id;
        accepted_other_ = true;
        apply(reply, request);
        return reply;
    }

    // Not newer: a duplicate of a record, a repeated GET_STATE, or stale.
    if (last_set_pose_.have && id == last_set_pose_.request.request_id &&
        sameBody(request, last_set_pose_.request)) {
        answerPlacement(reply, last_set_pose_.sequence);
    } else if (last_select_.have && id == last_select_.request.request_id &&
               sameBody(request, last_select_.request)) {
        answerSelect(reply, request);
    } else if (id == newest_ && request.op == gatr2::kOpGetState) {
        answerState(reply);
    } else if (id == newest_ || (last_set_pose_.have && id == last_set_pose_.request.request_id) ||
               (last_select_.have && id == last_select_.request.request_id)) {
        reply.result = gatr2::kResultInvalidArgument;
    } else {
        reply.result = gatr2::kResultStale;
    }
    return reply;
}

void FakePi::restart(uint32_t pi_instance) {
    pi_instance_     = pi_instance;
    rng_             = seed(pi_instance);
    session_         = 0;
    open_nonce_      = 0;
    open_request_id_ = 0;
    accepted_other_  = false;
    nonce_count_     = 0;
    have_newest_     = false;
    newest_          = 0;
    last_set_pose_   = Record{};
    last_select_     = Record{};

    init_sequence_    = 0;
    placement_        = Placement{};
    applied_session_  = 0;
    applied_sequence_ = 0;
    object_requested_ = false;
    object_wire_id_   = 0;
    object_sequence_  = 0;

    robot_.robot_flags &= static_cast<uint8_t>(
        ~(gatr2::kRobotLocalized | gatr2::kRobotAnchorCommand | gatr2::kRobotAnchorConfigured));
    robot_.odometry_epoch  = 0;
    robot_.anchor_revision = 0;
    reader_.reset();
}

void FakePi::openSession(const gatr2::BrainRequest& request) {
    session_         = nextSession();
    open_nonce_      = request.nonce;
    open_request_id_ = request.request_id;
    accepted_other_  = false;
    have_newest_     = true;
    newest_          = request.request_id;

    for (int i = 3; i > 0; --i) {
        nonce_ring_[i] = nonce_ring_[i - 1];
    }
    nonce_ring_[0] = request.nonce;
    nonce_count_   = nonce_count_ < 4 ? nonce_count_ + 1 : 4;

    // Client state of the old session ends; localization is untouched.
    last_set_pose_    = Record{};
    last_select_      = Record{};
    object_requested_ = false;
    object_wire_id_   = 0;
    object_sequence_ += 1;
    placement_ = Placement{}; // withdraws an unapplied placement
    ++sessions_opened_;
}

void FakePi::apply(gatr2::BrainReply& reply, const gatr2::BrainRequest& request) {
    if (request.op == gatr2::kOpSetPose) {
        init_sequence_ += 1;
        placement_.pending   = true;
        placement_.session   = session_;
        placement_.sequence  = init_sequence_;
        placement_.x_mm      = request.x_mm;
        placement_.y_mm      = request.y_mm;
        placement_.heading   = request.heading_cdeg;
        placement_.countdown = apply_delay_;
        last_set_pose_       = Record{true, request, init_sequence_};
        if (apply_delay_ == 0) {
            applyPlacement();
        }
        answerPlacement(reply, init_sequence_);
    } else if (request.op == gatr2::kOpSelectLandmark) {
        object_requested_ = (request.select_flags & gatr2::kSelectFlagSelected) != 0;
        if (object_requested_) {
            object_wire_id_ = request.landmark_id;
        }
        object_sequence_ += 1;
        last_select_ = Record{true, request, 0};
        answerSelect(reply, request);
    } else {
        answerState(reply);
    }
}

void FakePi::answerPlacement(gatr2::BrainReply& reply, uint32_t sequence) const {
    const bool applied    = applied_session_ == session_ && applied_sequence_ == sequence;
    reply.result          = applied ? gatr2::kResultOk : gatr2::kResultPending;
    reply.odometry_epoch  = robot_.odometry_epoch;
    reply.anchor_revision = robot_.anchor_revision;
}

void FakePi::answerSelect(gatr2::BrainReply& reply, const gatr2::BrainRequest& request) const {
    const bool selecting = (request.select_flags & gatr2::kSelectFlagSelected) != 0;
    if (selecting && world_noop_) {
        reply.result = gatr2::kResultLandmarkUnsupported;
    } else if (selecting && landmarks_.count(request.landmark_id) == 0) {
        reply.result = gatr2::kResultUnknownLandmark;
    } else {
        reply.result       = gatr2::kResultOk;
        reply.landmark_id  = request.landmark_id;
        reply.select_flags = request.select_flags;
    }
}

void FakePi::answerState(gatr2::BrainReply& reply) const {
    reply.result                = gatr2::kResultOk;
    reply.state                 = robot_;
    reply.state.landmark_id     = object_requested_ ? object_wire_id_ : 0;
    reply.state.landmark_source = gatr2::kLandmarkSourceNone;
    reply.state.lm_x_mm         = 0;
    reply.state.lm_y_mm         = 0;
    reply.state.lm_heading_cdeg = 0;
    reply.state.landmark_age_ms = 0;
    if (!object_requested_ || world_noop_) {
        return;
    }
    const auto it = landmarks_.find(object_wire_id_);
    if (it == landmarks_.end()) {
        return;
    }
    const FakeLandmark& lm      = it->second;
    reply.state.landmark_source = lm.source;
    reply.state.lm_x_mm         = lm.x_mm;
    reply.state.lm_y_mm         = lm.y_mm;
    reply.state.lm_heading_cdeg = lm.heading_cdeg;
    if (lm.source == gatr2::kLandmarkSourceObserved) {
        reply.state.landmark_age_ms = lm.age_ms;
    }
}

void FakePi::tickPlacement() {
    if (!placement_.pending || placement_.countdown <= 0) {
        return;
    }
    placement_.countdown -= 1;
    if (placement_.countdown == 0) {
        applyPlacement();
    }
}

void FakePi::applyPlacement() {
    robot_.x_mm         = placement_.x_mm;
    robot_.y_mm         = placement_.y_mm;
    robot_.heading_cdeg = placement_.heading;
    robot_.robot_flags |= gatr2::kRobotPoseValid | gatr2::kRobotLocalized | gatr2::kRobotAgeKnown |
                          gatr2::kRobotAnchorCommand;
    robot_.robot_flags &= static_cast<uint8_t>(~gatr2::kRobotAnchorConfigured);
    robot_.anchor_revision += 1;
    applied_session_   = placement_.session;
    applied_sequence_  = placement_.sequence;
    placement_.pending = false;
    ++placements_applied_;
}

uint32_t FakePi::nextSession() {
    do {
        rng_ ^= rng_ << 13;
        rng_ ^= rng_ >> 17;
        rng_ ^= rng_ << 5;
    } while (rng_ == 0 || rng_ == session_);
    return rng_;
}

} // namespace communigatr
