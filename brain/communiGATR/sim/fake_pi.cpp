// fake_pi.cpp

#include "sim/fake_pi.h"

#include <algorithm>
#include <cstring>

namespace communigatr
{

namespace
{

uint32_t seed(uint32_t pi_instance) {
    const uint32_t s = pi_instance ^ 0xA5A5A5A5u;
    return s == 0 ? 1 : s;
}

// 16-bit serial arithmetic: a is newer when (a - b) mod 65536 is in 1..32767.
bool newerId(uint16_t a, uint16_t b) {
    const uint16_t d = static_cast<uint16_t>(a - b);
    return d != 0 && d < 0x8000;
}

// Read-only, or idempotent by content: the newest id is answered again.
bool repeatable(uint8_t op) {
    return op == translagatr::kOpGetState || op == translagatr::kOpProfileWrite ||
           op == translagatr::kOpProfileApply || op == translagatr::kOpReadDoc ||
           op == translagatr::kOpPathReport || op == translagatr::kOpReadWheels ||
           op == translagatr::kOpTelemetry;
}

bool sameTelemetry(const translagatr::BrainTelemetry& a, const translagatr::BrainTelemetry& b) {
    if (a.flags != b.flags || a.stamp_ms != b.stamp_ms || a.roll_cdeg != b.roll_cdeg ||
        a.pitch_cdeg != b.pitch_cdeg || a.command_id != b.command_id ||
        a.motion_state != b.motion_state || a.motion_reason != b.motion_reason ||
        a.plan_mode != b.plan_mode || a.segment != b.segment ||
        a.segment_count != b.segment_count || a.target_x_mm != b.target_x_mm ||
        a.target_y_mm != b.target_y_mm || a.target_heading_cdeg != b.target_heading_cdeg ||
        a.cmd_vx_mm_s != b.cmd_vx_mm_s || a.cmd_vy_mm_s != b.cmd_vy_mm_s ||
        a.cmd_omega_cdeg_s != b.cmd_omega_cdeg_s || a.cross_track_mm != b.cross_track_mm ||
        a.distance_error_mm != b.distance_error_mm ||
        a.heading_error_cdeg != b.heading_error_cdeg || a.drive_fault != b.drive_fault ||
        a.wheel_count != b.wheel_count) {
        return false;
    }
    for (uint8_t i = 0; i < translagatr::kTelemetryWheelsMax; ++i) {
        if (a.wheel_rpm_x10[i] != b.wheel_rpm_x10[i]) {
            return false;
        }
    }
    return true;
}

bool sameRequest(const translagatr::BrainRequest& a, const translagatr::BrainRequest& b) {
    if (a.op != b.op) {
        return false;
    }
    switch (a.op) {
    case translagatr::kOpSetPose:
        return a.x_mm == b.x_mm && a.y_mm == b.y_mm && a.heading_cdeg == b.heading_cdeg;
    case translagatr::kOpGetState:
        return a.imu_flags == b.imu_flags && a.imu_stamp_ms == b.imu_stamp_ms &&
               a.imu_rotation_mdeg == b.imu_rotation_mdeg;
    case translagatr::kOpProfileWrite:
        return a.profile_id == b.profile_id && a.total_len == b.total_len &&
               a.offset == b.offset && a.data_len == b.data_len &&
               std::memcmp(a.data, b.data, a.data_len) == 0;
    case translagatr::kOpProfileApply:
        return a.profile_id == b.profile_id && a.total_len == b.total_len;
    case translagatr::kOpReadDoc:
        return a.doc_kind == b.doc_kind && a.doc_id == b.doc_id && a.doc_offset == b.doc_offset &&
               a.max_len == b.max_len;
    case translagatr::kOpControl:
        return a.action == b.action && a.action_arg == b.action_arg;
    case translagatr::kOpPathReport:
        if (a.command_id != b.command_id || a.path_mode != b.path_mode ||
            a.point_count != b.point_count) {
            return false;
        }
        for (uint8_t i = 0; i < a.point_count; ++i) {
            if (a.points[i].x_mm != b.points[i].x_mm || a.points[i].y_mm != b.points[i].y_mm) {
                return false;
            }
        }
        return true;
    case translagatr::kOpReadWheels:
        return true;
    case translagatr::kOpTelemetry:
        return sameTelemetry(a.telemetry, b.telemetry);
    default:
        return false;
    }
}

bool wellFormed(const translagatr::BrainRequest& r) {
    switch (r.op) {
    case translagatr::kOpGetState:
        return (r.imu_flags & ~translagatr::kBenchImuValid) == 0;
    case translagatr::kOpProfileWrite:
        return r.total_len >= translagatr::kProfileHeaderLen && r.total_len <= translagatr::kProfileMaxLen &&
               static_cast<uint32_t>(r.offset) + r.data_len <= r.total_len;
    case translagatr::kOpProfileApply:
        return r.total_len >= translagatr::kProfileHeaderLen && r.total_len <= translagatr::kProfileMaxLen;
    case translagatr::kOpReadDoc:
        return (r.doc_kind == translagatr::kDocFieldMap || r.doc_kind == translagatr::kDocFieldEstimate) &&
               r.max_len > 0;
    case translagatr::kOpControl:
        return r.action >= translagatr::kControlRecalibrate &&
               r.action <= translagatr::kControlRestartAcquisition;
    case translagatr::kOpPathReport:
        return r.path_mode <= translagatr::kPathAvoiding;
    default:
        return true;
    }
}

} // namespace

FakeField makeFakeField(uint16_t count, uint16_t revision) {
    FakeField field;
    field.revision = revision;
    for (uint16_t i = 0; i < count; ++i) {
        translagatr::FieldObjectRecord r;
        r.object_id    = static_cast<uint16_t>(10 * (i + 1));
        r.x_mm         = 300 + (i % 8) * 400;
        r.y_mm         = 300 + (i / 8) * 200;
        r.heading_cdeg = (i % 8) * 4500 - 13500;
        if (i % 6 == 5) {
            r.kind  = translagatr::kObjectFixed; // reference only, no box
            r.flags = translagatr::kObjectReference;
        } else if (i % 2 == 0) {
            r.kind             = translagatr::kObjectLandmark;
            r.flags            = translagatr::kObjectObstacle | translagatr::kObjectEstimated |
                      translagatr::kObjectReference;
            r.box_x_mm         = 20;
            r.box_y_mm         = -10;
            r.box_heading_cdeg = 4500;
            r.box_length_mm    = 150;
            r.box_width_mm     = 100;
        } else {
            r.kind          = translagatr::kObjectFixed;
            r.flags         = translagatr::kObjectObstacle;
            r.box_length_mm = 200;
            r.box_width_mm  = 50;
        }
        field.objects.push_back(r);
    }
    return field;
}

FakePi::FakePi(uint32_t pi_instance) : pi_instance_(pi_instance), rng_(seed(pi_instance)) {}

std::vector<std::vector<uint8_t>> FakePi::receive(const uint8_t* data, std::size_t len) {
    std::vector<std::vector<uint8_t>> replies;
    for (std::size_t i = 0; i < len; ++i) {
        if (!reader_.push(data[i])) {
            continue;
        }
        do {
            translagatr::BrainRequest request;
            if (reader_.frameType() != translagatr::kFrameBrainRequest ||
                !translagatr::decodeBrainRequest(reader_.frame(), reader_.frameLen(), request)) {
                continue;
            }
            const translagatr::BrainReply reply = answer(request);
            std::vector<uint8_t>    frame(translagatr::kMaxFrameLen);
            frame.resize(
                translagatr::encodeBrainReply(reply, frame.data(), static_cast<uint16_t>(frame.size())));
            replies.push_back(frame);
        } while (reader_.next());
    }
    return replies;
}

translagatr::BrainReply FakePi::answer(const translagatr::BrainRequest& request) {
    requests_.push_back(request);
    tick();

    translagatr::BrainReply reply;
    reply.version     = version_;
    reply.op          = request.op;
    reply.session     = request.session;
    reply.request_id  = request.request_id;
    reply.pi_instance = pi_instance_;
    reply.nonce       = request.nonce;

    if (request.version != version_) {
        reply.result = translagatr::kResultUnsupportedVersion;
        return reply;
    }
    if (translagatr::brainRequestMinLen(request.op) == 0 || request.op == unsupported_op_) {
        reply.result = translagatr::kResultUnsupportedOp;
        return reply;
    }
    if (request.request_id == 0 || !wellFormed(request)) {
        reply.result = translagatr::kResultInvalidArgument;
        return reply;
    }

    if (request.op == translagatr::kOpHello) {
        if (session_ != 0 && request.nonce == open_nonce_ &&
            request.request_id == open_request_id_ && !accepted_other_) {
            reply.session = session_; // retry of the opening HELLO
            return reply;
        }
        for (int i = 0; i < nonce_count_; ++i) {
            if (nonce_ring_[i] == request.nonce) {
                reply.result = translagatr::kResultStale;
                return reply;
            }
        }
        openSession(request);
        reply.session = session_;
        return reply;
    }

    if (session_ == 0 || request.session != session_) {
        reply.result = translagatr::kResultUnknownSession;
        return reply;
    }
    if (have_newest_ && !newerId(request.request_id, newest_.request_id)) {
        repeat(reply, request);
        return reply;
    }
    have_newest_    = true;
    newest_         = request;
    accepted_other_ = true;
    execute(reply, request, false);
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
    newest_          = translagatr::BrainRequest{};
    last_set_pose_   = Record{};
    last_control_    = Record{};

    init_sequence_    = 0;
    placement_        = Placement{};
    applied_session_  = 0;
    applied_sequence_ = 0;

    staging_id_         = 0;
    staging_total_      = 0;
    staging_received_   = 0;
    applied_profile_    = 0;
    applying_profile_   = 0;
    applying_countdown_ = 0;
    profile_state_      = translagatr::kProfileNone;
    profile_id_         = 0;
    profile_reason_     = translagatr::kProfileReasonNone;
    profile_detail_     = 0;
    rejected_.clear();

    robot_.robot_flags &= static_cast<uint8_t>(
        ~(translagatr::kRobotLocalized | translagatr::kRobotAnchorCommand | translagatr::kRobotAnchorConfigured));
    robot_.odometry_epoch  = 0;
    robot_.anchor_revision = 0;
    robot_.calibration     = translagatr::kCalibrationNone;
    calibration_left_      = 0;

    estimates_.clear();
    next_estimate_id_ = 1;
    if (map_id_ != 0) {
        publishNominalEstimate();
    }
    path_      = FakePath{};
    telemetry_ = translagatr::BrainTelemetry{};
    reader_.reset();
}

// ---------------------------------------------------------------------------
// Field documents
// ---------------------------------------------------------------------------

void FakePi::setField(const FakeField& field) {
    const uint16_t count = static_cast<uint16_t>(field.objects.size());
    map_doc_.assign(translagatr::fieldMapLen(count), 0);
    translagatr::FieldMapHeader header;
    header.revision     = field.revision;
    header.object_count = count;
    header.min_x_mm     = field.min_x_mm;
    header.min_y_mm     = field.min_y_mm;
    header.max_x_mm     = field.max_x_mm;
    header.max_y_mm     = field.max_y_mm;
    const uint16_t cap  = static_cast<uint16_t>(map_doc_.size());
    translagatr::encodeFieldMapHeader(header, map_doc_.data(), cap);
    for (uint16_t i = 0; i < count; ++i) {
        translagatr::encodeFieldObjectRecord(field.objects[i], i, map_doc_.data(), cap);
    }
    map_id_ = translagatr::crc32(map_doc_.data(), cap);
    estimates_.clear();
    publishNominalEstimate();
}

void FakePi::setMapDocument(const std::vector<uint8_t>& doc) {
    map_doc_ = doc;
    map_id_  = translagatr::crc32(doc.data(), static_cast<uint32_t>(doc.size()));
    estimates_.clear();
}

void FakePi::clearField() {
    map_doc_.clear();
    map_id_ = 0;
    estimates_.clear();
}

std::vector<translagatr::FieldEstimateRecord> FakePi::nominalRecords() const {
    std::vector<translagatr::FieldEstimateRecord> records;
    translagatr::FieldMapHeader                   header;
    const uint16_t                          len = static_cast<uint16_t>(map_doc_.size());
    if (!translagatr::decodeFieldMapHeader(map_doc_.data(), len, header)) {
        return records;
    }
    for (uint16_t i = 0; i < header.object_count; ++i) {
        translagatr::FieldObjectRecord o;
        translagatr::decodeFieldObjectRecord(map_doc_.data(), len, i, o);
        translagatr::FieldEstimateRecord e;
        e.object_id    = o.object_id;
        e.source       = translagatr::kEstimateSourceNominal;
        e.flags        = translagatr::kEstimateValid;
        e.x_mm         = o.x_mm;
        e.y_mm         = o.y_mm;
        e.heading_cdeg = o.heading_cdeg;
        records.push_back(e);
    }
    return records;
}

uint32_t FakePi::publishEstimate(const std::vector<translagatr::FieldEstimateRecord>& records) {
    const uint16_t count = static_cast<uint16_t>(records.size());
    Estimate       e;
    e.id = next_estimate_id_++;
    e.doc.assign(translagatr::fieldEstimateLen(count), 0);
    translagatr::FieldEstimateHeader header;
    header.object_count    = count;
    header.map_id          = estimate_map_id_ != 0 ? estimate_map_id_ : map_id_;
    header.estimate_id     = e.id;
    header.odometry_epoch  = robot_.odometry_epoch;
    header.anchor_revision = robot_.anchor_revision;
    const uint16_t cap     = static_cast<uint16_t>(e.doc.size());
    translagatr::encodeFieldEstimateHeader(header, e.doc.data(), cap);
    for (uint16_t i = 0; i < count; ++i) {
        translagatr::encodeFieldEstimateRecord(records[i], i, e.doc.data(), cap);
    }
    estimates_.push_back(e);
    if (estimates_.size() > 3) {
        estimates_.erase(estimates_.begin());
    }
    return e.id;
}

uint32_t FakePi::publishNominalEstimate() { return publishEstimate(nominalRecords()); }

// ---------------------------------------------------------------------------
// Requests
// ---------------------------------------------------------------------------

void FakePi::openSession(const translagatr::BrainRequest& request) {
    session_         = nextSession();
    open_nonce_      = request.nonce;
    open_request_id_ = request.request_id;
    accepted_other_  = false;
    have_newest_     = false;
    newest_          = translagatr::BrainRequest{};

    for (int i = 3; i > 0; --i) {
        nonce_ring_[i] = nonce_ring_[i - 1];
    }
    nonce_ring_[0] = request.nonce;
    nonce_count_   = nonce_count_ < 4 ? nonce_count_ + 1 : 4;

    // Client state of the old session ends; localization, the profile and
    // its staging are untouched.
    last_set_pose_ = Record{};
    last_control_  = Record{};
    placement_     = Placement{}; // withdraws an unapplied placement
    path_          = FakePath{};
    ++sessions_opened_;
}

void FakePi::execute(translagatr::BrainReply& reply, const translagatr::BrainRequest& request, bool repeat) {
    switch (request.op) {
    case translagatr::kOpGetState:
        if (!repeat && (request.imu_flags & translagatr::kBenchImuValid) != 0) {
            ++imu_samples_;
        }
        answerState(reply);
        break;
    case translagatr::kOpSetPose:
        setPose(reply, request);
        break;
    case translagatr::kOpProfileWrite:
        profileWrite(reply, request);
        break;
    case translagatr::kOpProfileApply:
        profileApply(reply, request);
        break;
    case translagatr::kOpReadDoc:
        readDoc(reply, request);
        break;
    case translagatr::kOpControl:
        control(reply, request);
        break;
    case translagatr::kOpReadWheels:
        readWheels(reply);
        break;
    case translagatr::kOpPathReport:
        path_ = FakePath{};
        if (request.path_mode != translagatr::kPathNone) {
            path_.have       = true;
            path_.command_id = request.command_id;
            path_.mode       = request.path_mode;
            path_.points.assign(request.points, request.points + request.point_count);
        }
        break;
    case translagatr::kOpTelemetry:
        reply.result = telemetry_result_;
        if (telemetry_result_ == translagatr::kResultOk && !repeat) {
            telemetry_ = request.telemetry;
            ++telemetry_kept_;
        }
        break;
    default:
        break;
    }
}

void FakePi::repeat(translagatr::BrainReply& reply, const translagatr::BrainRequest& request) {
    const uint16_t id = request.request_id;
    if (last_set_pose_.have && id == last_set_pose_.request.request_id) {
        if (!sameRequest(request, last_set_pose_.request)) {
            reply.result = translagatr::kResultInvalidArgument;
        } else if (last_set_pose_.result == translagatr::kResultNotReady) {
            reply.result = translagatr::kResultNotReady;
        } else {
            answerPlacement(reply, last_set_pose_.sequence);
        }
        return;
    }
    if (last_control_.have && id == last_control_.request.request_id) {
        if (!sameRequest(request, last_control_.request)) {
            reply.result = translagatr::kResultInvalidArgument;
        } else {
            reply.result         = last_control_.result;
            reply.action         = last_control_.request.action;
            reply.calibration    = last_control_.calibration;
            reply.control_detail = last_control_.detail;
        }
        return;
    }
    if (id == newest_.request_id) {
        if (repeatable(request.op) && sameRequest(request, newest_)) {
            execute(reply, request, true);
        } else {
            reply.result = translagatr::kResultInvalidArgument;
        }
        return;
    }
    reply.result = translagatr::kResultStale;
}

void FakePi::setPose(translagatr::BrainReply& reply, const translagatr::BrainRequest& request) {
    if (!localizing()) {
        reply.result   = translagatr::kResultNotReady;
        last_set_pose_ = Record{true, request, translagatr::kResultNotReady, 0, 0};
        return;
    }
    init_sequence_ += 1;
    placement_.pending   = true;
    placement_.session   = session_;
    placement_.sequence  = init_sequence_;
    placement_.x_mm      = request.x_mm;
    placement_.y_mm      = request.y_mm;
    placement_.heading   = request.heading_cdeg;
    placement_.countdown = apply_delay_;
    last_set_pose_       = Record{true, request, translagatr::kResultPending, 0, init_sequence_};
    if (apply_delay_ == 0) {
        applyPlacement();
    }
    answerPlacement(reply, init_sequence_);
}

void FakePi::answerPlacement(translagatr::BrainReply& reply, uint32_t sequence) const {
    const bool applied    = applied_session_ == session_ && applied_sequence_ == sequence;
    reply.result          = applied ? translagatr::kResultOk : translagatr::kResultPending;
    reply.odometry_epoch  = robot_.odometry_epoch;
    reply.anchor_revision = robot_.anchor_revision;
}

void FakePi::answerState(translagatr::BrainReply& reply) const {
    reply.result = translagatr::kResultOk;
    reply.state  = robot_;
    if (!localizing()) {
        reply.state.robot_flags  = 0;
        reply.state.x_mm         = 0;
        reply.state.y_mm         = 0;
        reply.state.heading_cdeg = 0;
        reply.state.robot_age_ms = 0;
    }
    reply.state.profile_state  = profile_state_;
    reply.state.profile_id     = profile_id_;
    reply.state.profile_reason = profile_reason_;
    reply.state.profile_detail = profile_detail_;
    reply.state.map_id         = map_id_;
    reply.state.estimate_id    = newestEstimate();
}

void FakePi::profileWrite(translagatr::BrainReply& reply, const translagatr::BrainRequest& request) {
    const bool same = staging_total_ != 0 && request.profile_id == staging_id_ &&
                      request.total_len == staging_total_;
    const uint16_t held = same ? staging_received_ : 0;
    if (request.offset > held) {
        reply.result = translagatr::kResultInvalidArgument; // a gap
        return;
    }
    const uint16_t end     = static_cast<uint16_t>(request.offset + request.data_len);
    const uint16_t overlap = static_cast<uint16_t>(std::min(end, held) - request.offset);
    if (std::memcmp(staging_ + request.offset, request.data, overlap) != 0) {
        reply.result = translagatr::kResultInvalidArgument; // a resend with other bytes
        return;
    }
    if (!same) {
        staging_id_       = request.profile_id;
        staging_total_    = request.total_len;
        staging_received_ = 0;
    }
    if (end > staging_received_) {
        std::memcpy(staging_ + staging_received_, request.data + overlap,
                    end - staging_received_);
        staging_received_ = end;
    }
    ++profile_writes_;
    reply.profile_id = staging_id_;
    reply.received   = staging_received_;
}

void FakePi::profileApply(translagatr::BrainReply& reply, const translagatr::BrainRequest& request) {
    const uint32_t id = request.profile_id;
    if (staging_total_ == 0 || staging_id_ != id || staging_total_ != request.total_len ||
        staging_received_ != staging_total_ || translagatr::crc32(staging_, staging_total_) != id) {
        reply.result = translagatr::kResultInvalidArgument; // staging incomplete or corrupt
        return;
    }
    reply.profile_id = id;
    if (id == applied_profile_) {
        profile_state_       = translagatr::kProfileApplied;
        profile_id_          = id;
        profile_reason_      = translagatr::kProfileReasonNone;
        profile_detail_      = 0;
        reply.profile_state  = translagatr::kProfileApplied;
        return; // idempotent, nothing resets
    }
    if (profile_state_ == translagatr::kProfileApplying && profile_id_ == id) {
        reply.result        = translagatr::kResultPending;
        reply.profile_state = translagatr::kProfileApplying;
        return;
    }
    for (const Rejection& seen : rejected_) {
        if (seen.id == id) {
            rejectProfile(reply, id, seen.reason, seen.detail);
            return;
        }
    }
    if (!profile_mode_) {
        rejectProfile(reply, id, translagatr::kProfileReasonNotAccepted, 0);
        return;
    }
    translagatr::RobotProfileDoc profile;
    if (!translagatr::decodeRobotProfile(staging_, staging_total_, profile)) {
        rejectProfile(reply, id, translagatr::kProfileReasonFormat, 0);
        return;
    }
    uint8_t reason = translagatr::kProfileReasonNone;
    uint8_t detail = 0;
    if (!translagatr::validateRobotProfile(profile, reason, detail)) {
        rejectProfile(reply, id, reason, detail);
        return;
    }
    if (capability_reason_ != translagatr::kProfileReasonNone) {
        rejectProfile(reply, id, capability_reason_, capability_detail_);
        return;
    }
    profile_state_      = translagatr::kProfileApplying;
    profile_id_         = id;
    profile_reason_     = translagatr::kProfileReasonNone;
    profile_detail_     = 0;
    applying_profile_   = id;
    applying_countdown_ = profile_delay_;
    reply.result        = translagatr::kResultPending;
    reply.profile_state = translagatr::kProfileApplying;
}

// A running profile keeps running; only the report names the refused id.
void FakePi::rejectProfile(translagatr::BrainReply& reply, uint32_t id, uint8_t reason,
                           uint8_t detail) {
    rejected_.erase(std::remove_if(rejected_.begin(), rejected_.end(),
                                   [id](const Rejection& r) { return r.id == id; }),
                    rejected_.end());
    rejected_.push_back(Rejection{id, reason, detail});
    if (rejected_.size() > 4) {
        rejected_.erase(rejected_.begin());
    }
    profile_state_        = translagatr::kProfileRejected;
    profile_id_           = id;
    profile_reason_       = reason;
    profile_detail_       = detail;
    reply.result          = translagatr::kResultProfileRejected;
    reply.profile_id      = id;
    reply.profile_state   = translagatr::kProfileRejected;
    reply.profile_reason  = reason;
    reply.profile_detail  = detail;
}

void FakePi::readDoc(translagatr::BrainReply& reply, const translagatr::BrainRequest& request) {
    const std::vector<uint8_t>* doc = nullptr;
    uint32_t                    id  = 0;
    if (request.doc_kind == translagatr::kDocFieldMap) {
        if (map_doc_.empty()) {
            reply.result = translagatr::kResultUnavailable;
            return;
        }
        if (request.doc_id != 0 && request.doc_id != map_id_) {
            reply.result = translagatr::kResultStale;
            return;
        }
        doc = &map_doc_;
        id  = map_id_;
    } else {
        if (estimates_.empty()) {
            reply.result = translagatr::kResultUnavailable;
            return;
        }
        for (const Estimate& e : estimates_) {
            if (request.doc_id == 0 ? &e == &estimates_.back() : e.id == request.doc_id) {
                doc = &e.doc;
                id  = e.id;
            }
        }
        if (doc == nullptr) {
            reply.result = translagatr::kResultStale;
            return;
        }
    }
    if (request.doc_offset >= doc->size()) {
        reply.result = translagatr::kResultInvalidArgument;
        return;
    }
    const std::size_t left = doc->size() - request.doc_offset;
    const std::size_t n =
        std::min<std::size_t>({left, request.max_len, std::size_t{translagatr::kDocChunkMax}});
    reply.doc_kind      = request.doc_kind;
    reply.doc_id        = id;
    reply.doc_total_len = static_cast<uint16_t>(doc->size());
    reply.doc_crc32     = translagatr::crc32(doc->data(), static_cast<uint32_t>(doc->size()));
    reply.doc_offset    = request.doc_offset;
    reply.data_len      = static_cast<uint8_t>(n);
    std::memcpy(reply.data, doc->data() + request.doc_offset, n);
    if (doc_hook) {
        doc_hook(reply);
    }
}

void FakePi::control(translagatr::BrainReply& reply, const translagatr::BrainRequest& request) {
    reply.action  = request.action;
    last_control_ = Record{true, request, translagatr::kResultOk, 0, 0, translagatr::kControlDetailNone};
    if (profile_mode_ && applied_profile_ == 0) {
        last_control_.result = translagatr::kResultNotReady;
    } else if (moving_) {
        last_control_.result = translagatr::kResultNotStationary;
    } else {
        ++controls_executed_;
        if (control_delay_ > 0) {
            last_control_.result = translagatr::kResultPending;
            control_left_        = control_delay_;
        } else {
            finishControl();
        }
    }
    last_control_.calibration = robot_.calibration;
    reply.result              = last_control_.result;
    reply.calibration         = last_control_.calibration;
    reply.control_detail      = last_control_.detail;
}

// Completes the recorded control; its duplicates see the outcome.
void FakePi::finishControl() {
    control_left_ = 0;
    if (control_failure_ != translagatr::kControlDetailNone) {
        last_control_.result = translagatr::kResultFailed;
        last_control_.detail = control_failure_;
        return;
    }
    const uint8_t action = last_control_.request.action;
    if (action == translagatr::kControlReinitialize) {
        loseContinuity();
    }
    if (action != translagatr::kControlRestartAcquisition) {
        robot_.calibration = translagatr::kCalibrationRunning;
        calibration_left_  = calibration_requests_;
    }
    last_control_.result      = translagatr::kResultOk;
    last_control_.calibration = robot_.calibration;
}

void FakePi::readWheels(translagatr::BrainReply& reply) const {
    if (profile_mode_ && applied_profile_ == 0) {
        reply.result = translagatr::kResultNotReady;
        return;
    }
    reply.wheel_count = static_cast<uint8_t>(
        std::min<std::size_t>(wheels_.size(), translagatr::kWheelReadingsMax));
    for (uint8_t i = 0; i < reply.wheel_count; ++i) {
        reply.wheels[i] = wheels_[i];
    }
}

void FakePi::tick() {
    if (placement_.pending && placement_.countdown > 0) {
        placement_.countdown -= 1;
        if (placement_.countdown == 0) {
            applyPlacement();
        }
    }
    if (applying_profile_ != 0) {
        if (applying_countdown_ <= 0) {
            applyProfile();
        } else {
            applying_countdown_ -= 1;
        }
    }
    if (control_left_ > 0 && --control_left_ == 0) {
        finishControl();
    }
    if (robot_.calibration == translagatr::kCalibrationRunning && calibration_left_ > 0) {
        calibration_left_ -= 1;
        if (calibration_left_ == 0) {
            robot_.calibration = translagatr::kCalibrationDone;
        }
    }
}

void FakePi::applyPlacement() {
    robot_.x_mm         = placement_.x_mm;
    robot_.y_mm         = placement_.y_mm;
    robot_.heading_cdeg = placement_.heading;
    robot_.robot_flags |= translagatr::kRobotPoseValid | translagatr::kRobotLocalized | translagatr::kRobotAgeKnown |
                          translagatr::kRobotAnchorCommand;
    robot_.robot_flags &= static_cast<uint8_t>(~translagatr::kRobotAnchorConfigured);
    robot_.anchor_revision += 1;
    applied_session_   = placement_.session;
    applied_sequence_  = placement_.sequence;
    placement_.pending = false;
    ++placements_applied_;
}

// Controlled boundary. A different profile loses continuity; the same one
// changes nothing.
void FakePi::applyProfile() {
    const bool changed = applying_profile_ != applied_profile_;
    applied_profile_   = applying_profile_;
    applying_profile_  = 0;
    profile_state_     = translagatr::kProfileApplied;
    profile_id_        = applied_profile_;
    profile_reason_    = translagatr::kProfileReasonNone;
    profile_detail_    = 0;
    ++profiles_applied_;
    if (changed) {
        loseContinuity();
    }
}

// New odometry epoch, unplaced, placement requests withdrawn.
void FakePi::loseContinuity() {
    robot_.odometry_epoch += 1;
    robot_.robot_flags = translagatr::kRobotPoseValid | translagatr::kRobotAgeKnown;
    placement_         = Placement{};
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
