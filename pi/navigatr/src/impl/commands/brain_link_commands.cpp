// brain_link_commands.cpp

#include "impl/commands/brain_link_commands.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>

#include "math/angles.h"
#include "resources/resource_store.h"

namespace navigatr
{

namespace
{

constexpr std::size_t kRecentNonces   = 4;
constexpr std::size_t kRejectionsKept = 4;

// 16-bit serial arithmetic: a is newer when (a - b) mod 65536 is in 1..32767.
bool newerId(uint16_t a, uint16_t b) {
    const uint16_t d = static_cast<uint16_t>(a - b);
    return d != 0 && d < 0x8000;
}

std::seed_seq::result_type seedPart(int64_t v, int shift) {
    return static_cast<std::seed_seq::result_type>(static_cast<uint64_t>(v) >> shift);
}

// Read-only, or idempotent by content: the newest id is answered again.
bool repeatable(uint8_t op) {
    switch (op) {
    case gatr2::kOpGetState:
    case gatr2::kOpProfileWrite:
    case gatr2::kOpProfileApply:
    case gatr2::kOpReadDoc:
    case gatr2::kOpReadWheels:
    case gatr2::kOpPathReport: return true;
    default: return false;
    }
}

bool sameRequest(const gatr2::BrainRequest& a, const gatr2::BrainRequest& b) {
    if (a.op != b.op) {
        return false;
    }
    switch (a.op) {
    case gatr2::kOpSetPose:
        return a.x_mm == b.x_mm && a.y_mm == b.y_mm && a.heading_cdeg == b.heading_cdeg;
    case gatr2::kOpGetState:
        return a.imu_flags == b.imu_flags && a.imu_stamp_ms == b.imu_stamp_ms &&
               a.imu_rotation_mdeg == b.imu_rotation_mdeg;
    case gatr2::kOpProfileWrite:
        return a.profile_id == b.profile_id && a.total_len == b.total_len &&
               a.offset == b.offset && a.data_len == b.data_len &&
               std::memcmp(a.data, b.data, a.data_len) == 0;
    case gatr2::kOpProfileApply:
        return a.profile_id == b.profile_id && a.total_len == b.total_len;
    case gatr2::kOpReadDoc:
        return a.doc_kind == b.doc_kind && a.doc_id == b.doc_id &&
               a.doc_offset == b.doc_offset && a.max_len == b.max_len;
    case gatr2::kOpControl: return a.action == b.action && a.action_arg == b.action_arg;
    case gatr2::kOpReadWheels: return true;
    case gatr2::kOpPathReport:
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
    default: return false;
    }
}

// Body checks that need no Pi state.
bool wellFormed(const gatr2::BrainRequest& r) {
    switch (r.op) {
    case gatr2::kOpGetState: return (r.imu_flags & ~gatr2::kBenchImuValid) == 0;
    case gatr2::kOpProfileWrite:
        return r.total_len >= gatr2::kProfileHeaderLen && r.total_len <= gatr2::kProfileMaxLen &&
               static_cast<uint32_t>(r.offset) + r.data_len <= r.total_len;
    case gatr2::kOpProfileApply:
        return r.total_len >= gatr2::kProfileHeaderLen && r.total_len <= gatr2::kProfileMaxLen;
    case gatr2::kOpReadDoc:
        return (r.doc_kind == gatr2::kDocFieldMap || r.doc_kind == gatr2::kDocFieldEstimate) &&
               r.max_len > 0;
    case gatr2::kOpControl:
        return r.action >= gatr2::kControlRecalibrate &&
               r.action <= gatr2::kControlRestartAcquisition;
    case gatr2::kOpPathReport: return r.path_mode <= gatr2::kPathAvoiding;
    default: return true;
    }
}

} // namespace

BrainLinkCommands::BrainLinkCommands() {
    // distinct per process start and per instance in one process
    static std::atomic<uint32_t> instances{0};
    std::random_device           device;
    const int64_t                now = steadyNowUs();

    std::seed_seq seed{device(), device(), seedPart(now, 0), seedPart(now, 32),
                       static_cast<std::seed_seq::result_type>(instances.fetch_add(1))};
    random_.seed(seed);
    pi_instance_ = randomNonzero(0);
}

std::unique_ptr<Commands> BrainLinkCommands::create(const ConfigNode& node,
                                                    SlotInitializationContext& context,
                                                    std::string& err) {
    const ResourceId link_id{node.child("Serial").attr("resource_id")};
    if (link_id.empty()) {
        err = node.path() + ": needs <Serial resource_id=.../>";
        return nullptr;
    }
    if (context.resources == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }
    std::string inner;
    auto        link = context.resources->require<SerialLink>(link_id, inner);
    if (link == nullptr) {
        err = node.path() + ": " + inner;
        return nullptr;
    }

    long             window_ms = 40;
    long             guard_us  = 1000;
    const ConfigNode reply     = node.child("Reply");
    if (reply.valid() && (!reply.getInt("window_ms", window_ms, window_ms, err) ||
                          !reply.getInt("turnaround_guard_us", guard_us, guard_us, err))) {
        return nullptr;
    }
    if (window_ms <= 0 || guard_us < 0 || guard_us >= window_ms * 1000) {
        err = node.path() + ": Reply needs window_ms > 0 and 0 <= turnaround_guard_us < "
                            "window_ms * 1000";
        return nullptr;
    }
    // one loop period plus processing must fit the window
    if (context.loop_rate_hz > 0.0 && 1000.0 / context.loop_rate_hz > window_ms / 2.0) {
        char why[160];
        std::snprintf(why, sizeof(why),
                      ": loop period %.1f ms exceeds half the reply window (%ld ms); raise "
                      "Loop rate_hz or Reply window_ms",
                      1000.0 / context.loop_rate_hz, window_ms);
        err = node.path() + why;
        return nullptr;
    }

    auto commands             = std::make_unique<BrainLinkCommands>();
    commands->link_           = std::move(link);
    commands->diagnostics_id_ = link_id.value;
    commands->window_us_      = window_ms * 1000;
    commands->guard_us_       = guard_us;
    commands->profile_host_   = context.brain_profile;
    if (node.child("BenchImu").valid()) {
        commands->bench_imu_ = context.resources->require<BrainImuBench>(
            ResourceId{node.child("BenchImu").attr("resource_id")}, err);
        if (!commands->bench_imu_) {
            return nullptr;
        }
    }
    return commands;
}

uint32_t BrainLinkCommands::randomNonzero(uint32_t differs_from) {
    for (;;) {
        const uint32_t v = static_cast<uint32_t>(random_());
        if (v != 0 && v != differs_from) {
            return v;
        }
    }
}

void BrainLinkCommands::reset() {
    if (bench_imu_) {
        bench_imu_->reset();
    }
    reader_.reset();
    have_last_read_ = false;
    last_read_us_   = 0;
    pi_instance_    = randomNonzero(pi_instance_);
    session_        = 0;
    opening_nonce_  = 0;
    opening_rid_    = 0;
    accepted_       = false;
    recent_nonces_.clear();
    have_newest_ = false;
    newest_      = gatr2::BrainRequest{};
    set_pose_    = SetPoseRecord{};
    control_     = ControlRecord{};
    staging_     = Staging{};
    rejected_.clear();
}

CommandsOutput BrainLinkCommands::run(const CommandsInput& in) {
    CommandsOutput out;
    out.command       = in.previous;
    out.command.reply = BrainReplyContext{};

    LinkStats* stats =
        in.diagnostics != nullptr ? &in.diagnostics->links[diagnostics_id_] : nullptr;

    // Drain until empty; the newest completed request wins.
    bool                have_request = false;
    gatr2::BrainRequest request;
    int64_t             completed_us = 0;   // read that delivered its last byte
    std::size_t         completed_at = 0;   // drained bytes through its last byte
    std::size_t         drained      = 0;
    int64_t             read_us      = 0;
    uint8_t             buf[128];
    for (;;) {
        read_us = link_->nowUs();   // before the read: the request was incomplete then
        const SerialReadResult read = link_->readAvailable(MutableByteSpan{buf, sizeof(buf)});
        if (read.closed) {
            have_last_read_ = false;
            out.status      = FunctionStatus::kNoData;   // dead link; state persists
            return out;
        }
        if (read.bytes == 0) {
            break;
        }
        if (stats != nullptr) {
            stats->bytes += static_cast<uint32_t>(read.bytes);
        }
        for (std::size_t i = 0; i < read.bytes; ++i) {
            if (!reader_.push(buf[i])) {
                continue;
            }
            do {
                if (stats != nullptr) {
                    ++stats->packets;
                }
                if (reader_.frameType() != gatr2::kFrameBrainRequest) {
                    continue;
                }
                gatr2::BrainRequest decoded;
                if (!gatr2::decodeBrainRequest(reader_.frame(), reader_.frameLen(), decoded)) {
                    if (stats != nullptr) {
                        ++stats->decode_errors;
                    }
                    continue;
                }
                if (stats != nullptr) {
                    ++stats->requests;
                    if (have_request) {
                        ++stats->superseded;
                    }
                }
                have_request = true;
                request      = decoded;
                completed_us = read_us;
                completed_at = drained + i + 1;
            } while (reader_.next());
        }
        drained += read.bytes;
    }

    const bool    had_previous     = have_last_read_;
    const int64_t previous_read_us = last_read_us_;
    have_last_read_                = true;
    last_read_us_                  = read_us;
    if (drained == 0) {
        reader_.reset();   // requests are one burst: a partial frame never completes
    }
    if (!have_request) {
        return out;
    }

    process(request, out.command, stats, in.now);
    BrainReplyContext& reply = out.command.reply;
    if (!had_previous || completed_at < drained) {
        reply.pending = false;   // first drain, or the brain may be transmitting
        if (stats != nullptr) {
            ++stats->unanswered;
        }
        return out;
    }
    reply.window.not_before_us = completed_us + guard_us_;
    reply.window.deadline_us   = previous_read_us + window_us_;
    return out;
}

void BrainLinkCommands::process(const gatr2::BrainRequest& req, CommandState& c,
                                LinkStats* stats, MonotonicTime now) {
    BrainReplyContext& r = c.reply;
    r.pending            = true;
    r.op                 = req.op;
    r.session            = req.session;
    r.request_id         = req.request_id;
    r.pi_instance        = pi_instance_;
    r.nonce              = req.nonce;
    r.result             = gatr2::kResultOk;

    if (req.version != gatr2::kBrainLinkVersion) {
        r.result = gatr2::kResultUnsupportedVersion;
        return;
    }
    if (gatr2::brainRequestMinLen(req.op) == 0) {
        r.result = gatr2::kResultUnsupportedOp;
        return;
    }
    if (req.request_id == 0 || !wellFormed(req)) {
        r.result = gatr2::kResultInvalidArgument;
        return;
    }
    if (req.op == gatr2::kOpHello) {
        hello(req, c, stats);
        return;
    }
    if (session_ == 0 || req.session != session_) {
        r.result = gatr2::kResultUnknownSession;
        if (stats != nullptr) {
            ++stats->unknown_session;
        }
        return;
    }
    if (have_newest_ && !newerId(req.request_id, newest_.request_id)) {
        repeat(req, c, stats, now);
        return;
    }

    have_newest_ = true;
    newest_      = req;
    accepted_    = true;
    execute(req, c, now, false);
}

void BrainLinkCommands::execute(const gatr2::BrainRequest& req, CommandState& c,
                                MonotonicTime now, bool repeat) {
    BrainReplyContext& r = c.reply;
    switch (req.op) {
    case gatr2::kOpGetState:
        if (bench_imu_ && !repeat) {
            bench_imu_->accept(session_, req.imu_flags, req.imu_stamp_ms, req.imu_rotation_mdeg,
                               now);
        }
        break;
    case gatr2::kOpSetPose: setPose(req, c); break;
    case gatr2::kOpProfileWrite: profileWrite(req, r); break;
    case gatr2::kOpProfileApply: profileApply(req, c); break;
    case gatr2::kOpReadDoc:
        r.doc_kind    = req.doc_kind;
        r.doc_id      = req.doc_id;
        r.doc_offset  = req.doc_offset;
        r.doc_max_len = req.max_len;
        break;
    case gatr2::kOpControl: {
        uint8_t detail = gatr2::kControlDetailNone;
        r.result       = profile_host_ == nullptr
                             ? static_cast<uint8_t>(gatr2::kResultNotReady)
                             : profile_host_->control(req.action, req.action_arg, now, detail);
        r.action         = req.action;
        r.control_detail = detail;
        control_ = ControlRecord{true, req.request_id, req.action, req.action_arg, r.result, detail};
        break;
    }
    case gatr2::kOpPathReport: {
        PathReport path;
        if (req.path_mode != gatr2::kPathNone) {
            path.session    = session_;
            path.command_id = req.command_id;
            path.mode       = req.path_mode;
            path.count      = req.point_count;
            for (uint8_t i = 0; i < req.point_count; ++i) {
                path.points[i] = {req.points[i].x_mm / 1000.0, req.points[i].y_mm / 1000.0};
            }
            path.received = now;
        }
        c.path = path;
        break;
    }
    case gatr2::kOpReadWheels:
        // wheel readings come with the applied profile
        r.result = profile_host_ == nullptr ? gatr2::kResultUnavailable : gatr2::kResultNotReady;
        break;
    default: break;
    }
}

void BrainLinkCommands::setPose(const gatr2::BrainRequest& req, CommandState& c) {
    BrainReplyContext& r = c.reply;
    set_pose_ = SetPoseRecord{true, req.request_id, req.x_mm, req.y_mm, req.heading_cdeg,
                              gatr2::kResultNotReady, 0};
    if (profile_host_ != nullptr && c.profile.applied_id == 0) {
        r.result = gatr2::kResultNotReady;   // no odometry to anchor before a profile
        return;
    }
    c.init_pose.x_m         = req.x_mm / 1000.0;
    c.init_pose.y_m         = req.y_mm / 1000.0;
    c.init_pose.heading_rad = cdegToRad(req.heading_cdeg);
    c.init_session          = session_;
    c.init_sequence += 1;
    set_pose_.result     = gatr2::kResultPending;
    set_pose_.sequence   = c.init_sequence;
    r.result             = gatr2::kResultPending;
    r.placement_sequence = c.init_sequence;
}

void BrainLinkCommands::profileWrite(const gatr2::BrainRequest& req, BrainReplyContext& r) {
    Staging&       s    = staging_;
    const bool     same = s.total_len != 0 && req.profile_id == s.id &&
                      req.total_len == s.total_len;
    const uint16_t held = same ? s.received : 0;
    if (req.offset > held) {
        r.result = gatr2::kResultInvalidArgument;   // a gap
        return;
    }
    const uint16_t end     = static_cast<uint16_t>(req.offset + req.data_len);
    const uint16_t overlap = static_cast<uint16_t>(std::min(end, held) - req.offset);
    if (std::memcmp(s.bytes.data() + req.offset, req.data, overlap) != 0) {
        r.result = gatr2::kResultInvalidArgument;   // a resend with other bytes
        return;
    }
    if (!same) {
        s           = Staging{};
        s.id        = req.profile_id;
        s.total_len = req.total_len;
    }
    if (end > s.received) {
        std::memcpy(s.bytes.data() + s.received, req.data + overlap, end - s.received);
        s.received = end;
    }
    r.profile_id = s.id;
    r.received   = s.received;
}

void BrainLinkCommands::profileApply(const gatr2::BrainRequest& req, CommandState& c) {
    BrainReplyContext& r  = c.reply;
    const uint32_t     id = req.profile_id;
    const Staging&     s  = staging_;
    if (s.total_len == 0 || s.id != id || s.total_len != req.total_len ||
        s.received != s.total_len || gatr2::crc32(s.bytes.data(), s.total_len) != id) {
        r.result = gatr2::kResultInvalidArgument;   // staging incomplete or corrupt
        return;
    }

    ProfileStatus& p = c.profile;
    r.profile_id     = id;
    if (id == p.applied_id) {
        // the running profile: idempotent, nothing resets
        p.state         = gatr2::kProfileApplied;
        p.id            = id;
        p.reason        = gatr2::kProfileReasonNone;
        p.detail        = 0;
        r.profile_state = gatr2::kProfileApplied;
        return;
    }
    if (p.state == gatr2::kProfileApplying && p.id == id) {
        r.result        = gatr2::kResultPending;
        r.profile_state = gatr2::kProfileApplying;
        return;
    }
    if (p.state == gatr2::kProfileRejected && p.id == id) {
        rejectProfile(id, p.reason, p.detail, c);
        return;
    }
    for (const Rejection& seen : rejected_) {
        if (seen.id == id) {
            rejectProfile(id, seen.reason, seen.detail, c);
            return;
        }
    }
    if (profile_host_ == nullptr) {
        rejectProfile(id, gatr2::kProfileReasonNotAccepted, 0, c);
        return;
    }
    gatr2::RobotProfileDoc profile;
    if (!gatr2::decodeRobotProfile(s.bytes.data(), s.total_len, profile)) {
        rejectProfile(id, gatr2::kProfileReasonFormat, 0, c);
        return;
    }
    uint8_t reason = gatr2::kProfileReasonNone;
    uint8_t detail = 0;
    if (!gatr2::validateRobotProfile(profile, reason, detail) ||
        !profile_host_->prepare(profile, id, reason, detail)) {
        rejectProfile(id, reason, detail, c);
        return;
    }
    p.state         = gatr2::kProfileApplying;
    p.id            = id;
    p.reason        = gatr2::kProfileReasonNone;
    p.detail        = 0;
    r.result        = gatr2::kResultPending;
    r.profile_state = gatr2::kProfileApplying;
}

void BrainLinkCommands::rejectProfile(uint32_t id, uint8_t reason, uint8_t detail,
                                      CommandState& c) {
    rejected_.erase(std::remove_if(rejected_.begin(), rejected_.end(),
                                   [id](const Rejection& seen) { return seen.id == id; }),
                    rejected_.end());
    rejected_.push_back(Rejection{id, reason, detail});
    if (rejected_.size() > kRejectionsKept) {
        rejected_.erase(rejected_.begin());
    }
    // a running profile keeps running; only the report names the refused id
    c.profile.state  = gatr2::kProfileRejected;
    c.profile.id     = id;
    c.profile.reason = reason;
    c.profile.detail = detail;

    BrainReplyContext& r = c.reply;
    r.result             = gatr2::kResultProfileRejected;
    r.profile_id         = id;
    r.profile_state      = gatr2::kProfileRejected;
    r.profile_reason     = reason;
    r.profile_detail     = detail;
}

void BrainLinkCommands::hello(const gatr2::BrainRequest& req, CommandState& c,
                              LinkStats* stats) {
    BrainReplyContext& r = c.reply;
    if (session_ != 0 && req.nonce == opening_nonce_ && req.request_id == opening_rid_ &&
        !accepted_) {
        r.session = session_;   // retry of the opening HELLO
        return;
    }
    if (std::find(recent_nonces_.begin(), recent_nonces_.end(), req.nonce) !=
        recent_nonces_.end()) {
        r.result = gatr2::kResultStale;
        if (stats != nullptr) {
            ++stats->stale;
        }
        return;
    }

    session_ = randomNonzero(session_);
    if (bench_imu_) {
        bench_imu_->reset(session_);
    }
    opening_nonce_ = req.nonce;
    opening_rid_   = req.request_id;
    accepted_      = false;
    recent_nonces_.push_back(req.nonce);
    if (recent_nonces_.size() > kRecentNonces) {
        recent_nonces_.erase(recent_nonces_.begin());
    }
    have_newest_ = false;
    newest_      = gatr2::BrainRequest{};
    set_pose_    = SetPoseRecord{};
    control_     = ControlRecord{};

    // client state is per session; localization, placement, the profile and
    // its staging are not
    c.session = session_;
    c.path    = PathReport{};
    r.session = session_;
}

void BrainLinkCommands::repeat(const gatr2::BrainRequest& req, CommandState& c,
                               LinkStats* stats, MonotonicTime now) {
    BrainReplyContext& r = c.reply;
    if (set_pose_.valid && req.request_id == set_pose_.rid) {
        if (req.op == gatr2::kOpSetPose && req.x_mm == set_pose_.x_mm &&
            req.y_mm == set_pose_.y_mm && req.heading_cdeg == set_pose_.heading_cdeg) {
            r.result             = set_pose_.result;
            r.placement_sequence = set_pose_.sequence;
            if (stats != nullptr) {
                ++stats->duplicates;
            }
        } else {
            r.result = gatr2::kResultInvalidArgument;
        }
        return;
    }
    if (control_.valid && req.request_id == control_.rid) {
        if (req.op == gatr2::kOpControl && req.action == control_.action &&
            req.action_arg == control_.arg) {
            r.result         = control_.result;
            r.action         = control_.action;
            r.control_detail = control_.detail;
            if (stats != nullptr) {
                ++stats->duplicates;
            }
        } else {
            r.result = gatr2::kResultInvalidArgument;
        }
        return;
    }
    if (req.request_id == newest_.request_id) {
        if (repeatable(req.op) && sameRequest(req, newest_)) {
            execute(req, c, now, true);
        } else {
            r.result = gatr2::kResultInvalidArgument;
        }
        return;
    }
    r.result = gatr2::kResultStale;
    if (stats != nullptr) {
        ++stats->stale;
    }
}

} // namespace navigatr
