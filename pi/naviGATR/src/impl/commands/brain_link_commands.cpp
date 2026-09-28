// brain_link_commands.cpp

#include "impl/commands/brain_link_commands.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>

#include "diagnostics/hub.h"
#include "math/angles.h"
#include "resources/resource_store.h"

namespace navigatr
{

const char* brainOpName(uint8_t op) {
    switch (op) {
    case translagatr::kOpHello: return "HELLO";
    case translagatr::kOpSetPose: return "SET_POSE";
    case translagatr::kOpGetState: return "GET_STATE";
    case translagatr::kOpProfileWrite: return "PROFILE_WRITE";
    case translagatr::kOpProfileApply: return "PROFILE_APPLY";
    case translagatr::kOpReadDoc: return "READ_DOC";
    case translagatr::kOpControl: return "CONTROL";
    case translagatr::kOpPathReport: return "PATH_REPORT";
    case translagatr::kOpReadWheels: return "READ_WHEELS";
    case translagatr::kOpTelemetry: return "TELEMETRY";
    default: return "unknown op";
    }
}

const char* brainResultName(uint8_t result) {
    switch (result) {
    case translagatr::kResultOk: return "Ok";
    case translagatr::kResultPending: return "Pending";
    case translagatr::kResultUnknownSession: return "UnknownSession";
    case translagatr::kResultUnsupportedVersion: return "UnsupportedVersion";
    case translagatr::kResultUnsupportedOp: return "UnsupportedOp";
    case translagatr::kResultInvalidArgument: return "InvalidArgument";
    case translagatr::kResultStale: return "Stale";
    case translagatr::kResultNotReady: return "NotReady";
    case translagatr::kResultProfileRejected: return "ProfileRejected";
    case translagatr::kResultUnavailable: return "Unavailable";
    case translagatr::kResultNotStationary: return "NotStationary";
    case translagatr::kResultFailed: return "Failed";
    default: return "unknown result";
    }
}

namespace
{

bool sameTelemetry(const translagatr::BrainTelemetry& a, const translagatr::BrainTelemetry& b) {
    bool same = a.flags == b.flags && a.stamp_ms == b.stamp_ms && a.roll_cdeg == b.roll_cdeg &&
                a.pitch_cdeg == b.pitch_cdeg && a.command_id == b.command_id &&
                a.motion_state == b.motion_state && a.motion_reason == b.motion_reason &&
                a.plan_mode == b.plan_mode && a.segment == b.segment &&
                a.segment_count == b.segment_count && a.target_x_mm == b.target_x_mm &&
                a.target_y_mm == b.target_y_mm && a.target_heading_cdeg == b.target_heading_cdeg &&
                a.cmd_vx_mm_s == b.cmd_vx_mm_s && a.cmd_vy_mm_s == b.cmd_vy_mm_s &&
                a.cmd_omega_cdeg_s == b.cmd_omega_cdeg_s && a.cross_track_mm == b.cross_track_mm &&
                a.distance_error_mm == b.distance_error_mm &&
                a.heading_error_cdeg == b.heading_error_cdeg && a.drive_fault == b.drive_fault &&
                a.wheel_count == b.wheel_count;
    for (uint8_t i = 0; same && i < translagatr::kTelemetryWheelsMax; ++i) {
        same = a.wheel_rpm_x10[i] == b.wheel_rpm_x10[i];
    }
    return same;
}

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
    case translagatr::kOpGetState:
    case translagatr::kOpProfileWrite:
    case translagatr::kOpProfileApply:
    case translagatr::kOpReadDoc:
    case translagatr::kOpReadWheels:
    case translagatr::kOpPathReport:
    case translagatr::kOpTelemetry: return true;
    default: return false;
    }
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
        return a.doc_kind == b.doc_kind && a.doc_id == b.doc_id &&
               a.doc_offset == b.doc_offset && a.max_len == b.max_len;
    case translagatr::kOpControl: return a.action == b.action && a.action_arg == b.action_arg;
    case translagatr::kOpReadWheels: return true;
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
    case translagatr::kOpTelemetry: return sameTelemetry(a.telemetry, b.telemetry);
    default: return false;
    }
}

// Body checks that need no Pi state.
bool wellFormed(const translagatr::BrainRequest& r) {
    switch (r.op) {
    case translagatr::kOpGetState: return (r.imu_flags & ~translagatr::kBenchImuValid) == 0;
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
    case translagatr::kOpPathReport: return r.path_mode <= translagatr::kPathAvoiding;
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
    if (context.diagnostics != nullptr) {
        // the pros_usb_link resource made this monitor already, as brain_usb
        std::string kind = "brain_serial";
        for (const ResourceStore::Record& r : context.resources->records()) {
            if (r.id == link_id && r.implementationType.value == "pros_usb_link") {
                kind = "brain_usb";
            }
        }
        commands->hub_       = context.diagnostics;
        commands->source_id_ = context.diagnostics->sourceId(link_id.value);
        commands->monitor_   = context.diagnostics->links().monitor(link_id.value, kind);
    }
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
    newest_      = translagatr::BrainRequest{};
    set_pose_    = SetPoseRecord{};
    control_     = ControlRecord{};
    staging_     = Staging{};
    rejected_.clear();
}

CommandsOutput BrainLinkCommands::run(const CommandsInput& in) {
    CommandsOutput out;
    out.command             = in.previous;
    out.command.reply       = BrainReplyContext{};
    out.command.pi_instance = pi_instance_;
    out.command.link_open   = true;

    LinkStats* stats =
        in.diagnostics != nullptr ? &in.diagnostics->links[diagnostics_id_] : nullptr;

    // Drain until empty; the newest completed request wins.
    bool                have_request = false;
    translagatr::BrainRequest request;
    int64_t             completed_us = 0;   // read that delivered its last byte
    std::size_t         completed_at = 0;   // drained bytes through its last byte
    std::size_t         drained      = 0;
    int64_t             read_us      = 0;
    uint8_t             buf[128];
    for (;;) {
        read_us = link_->nowUs();   // before the read: the request was incomplete then
        const SerialReadResult read = link_->readAvailable(MutableByteSpan{buf, sizeof(buf)});
        if (read.closed) {
            have_last_read_       = false;
            out.command.link_open = false;
            out.status            = FunctionStatus::kNoData;   // dead link; state persists
            return out;
        }
        if (read.bytes == 0) {
            break;
        }
        if (stats != nullptr) {
            stats->bytes += static_cast<uint32_t>(read.bytes);
        }
        if (monitor_ != nullptr) {
            monitor_->rx(buf, read.bytes);
        }
        for (std::size_t i = 0; i < read.bytes; ++i) {
            if (!reader_.push(buf[i])) {
                continue;
            }
            do {
                if (stats != nullptr) {
                    ++stats->packets;
                }
                if (reader_.frameType() != translagatr::kFrameBrainRequest) {
                    if (monitor_ != nullptr) {
                        monitor_->rejected("not a brain request frame");
                    }
                    continue;
                }
                translagatr::BrainRequest decoded;
                if (!translagatr::decodeBrainRequest(reader_.frame(), reader_.frameLen(), decoded)) {
                    if (stats != nullptr) {
                        ++stats->decode_errors;
                    }
                    if (monitor_ != nullptr) {
                        monitor_->rejected("request body invalid for its op");
                    }
                    continue;
                }
                if (stats != nullptr) {
                    ++stats->requests;
                    if (have_request) {
                        ++stats->superseded;
                    }
                }
                noteDecoded(decoded);
                have_request = true;
                request      = decoded;
                completed_us = read_us;
                completed_at = drained + i + 1;
                request_len_ = static_cast<uint8_t>(reader_.frameLen());
                std::memcpy(request_frame_, reader_.frame(), reader_.frameLen());
            } while (reader_.next());
        }
        drained += read.bytes;
    }
    if (monitor_ != nullptr) {
        monitor_->readerStats(reader_.stats());
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

    out.command.last_request = in.now;
    process(request, out.command, stats, in.now);
    BrainReplyContext& reply = out.command.reply;
    if (!had_previous || completed_at < drained) {
        reply.pending = false;   // first drain, or the brain may be transmitting
        if (stats != nullptr) {
            ++stats->unanswered;
        }
        noteProcessed(request, reply);
        return out;
    }
    reply.window.not_before_us = completed_us + guard_us_;
    reply.window.deadline_us   = previous_read_us + window_us_;
    noteProcessed(request, reply);
    return out;
}

void BrainLinkCommands::noteDecoded(const translagatr::BrainRequest& req) {
    if (monitor_ == nullptr) {
        return;
    }
    std::string fields;
    if (monitor_->decodedOn()) {
        char buf[160];
        int  n = std::snprintf(buf, sizeof(buf), "session=%08lX rid=%u",
                               static_cast<unsigned long>(req.session),
                               static_cast<unsigned>(req.request_id));
        const std::size_t at = n > 0 ? static_cast<std::size_t>(n) : 0;
        switch (req.op) {
        case translagatr::kOpGetState:
            std::snprintf(buf + at, sizeof(buf) - at, " imu_flags=%u imu_t=%lu rot_mdeg=%ld",
                          static_cast<unsigned>(req.imu_flags),
                          static_cast<unsigned long>(req.imu_stamp_ms),
                          static_cast<long>(req.imu_rotation_mdeg));
            break;
        case translagatr::kOpSetPose:
            std::snprintf(buf + at, sizeof(buf) - at, " x_mm=%ld y_mm=%ld h_cdeg=%ld",
                          static_cast<long>(req.x_mm), static_cast<long>(req.y_mm),
                          static_cast<long>(req.heading_cdeg));
            break;
        case translagatr::kOpControl:
            std::snprintf(buf + at, sizeof(buf) - at, " action=%u arg=%u",
                          static_cast<unsigned>(req.action), static_cast<unsigned>(req.action_arg));
            break;
        case translagatr::kOpPathReport:
            std::snprintf(buf + at, sizeof(buf) - at, " command=%lu mode=%u points=%u",
                          static_cast<unsigned long>(req.command_id),
                          static_cast<unsigned>(req.path_mode),
                          static_cast<unsigned>(req.point_count));
            break;
        case translagatr::kOpTelemetry:
            std::snprintf(buf + at, sizeof(buf) - at, " flags=0x%02X t=%lu roll_cdeg=%d pitch_cdeg=%d",
                          static_cast<unsigned>(req.telemetry.flags),
                          static_cast<unsigned long>(req.telemetry.stamp_ms),
                          static_cast<int>(req.telemetry.roll_cdeg),
                          static_cast<int>(req.telemetry.pitch_cdeg));
            break;
        case translagatr::kOpProfileWrite:
            std::snprintf(buf + at, sizeof(buf) - at, " id=%08lX offset=%u len=%u",
                          static_cast<unsigned long>(req.profile_id),
                          static_cast<unsigned>(req.offset), static_cast<unsigned>(req.data_len));
            break;
        default: break;
        }
        fields = buf;
    }
    monitor_->frame(true, brainOpName(req.op), fields);
}

// The request this cycle processed: straight to the hub when no reply is
// owed, else staged for the publisher to complete with what it sent.
void BrainLinkCommands::noteProcessed(const translagatr::BrainRequest& req,
                                      const BrainReplyContext& reply) {
    if (hub_ == nullptr) {
        return;
    }
    DiagBrainRequest r;
    r.session     = req.session;
    r.request_id  = req.request_id;
    r.op          = req.op;
    r.result      = reply.result;
    r.request_len = request_len_;
    r.reply_len   = 0;
    r.duplicate   = repeated_;
    if (reply.pending && monitor_ != nullptr) {
        monitor_->stageRequest(r);
        return;
    }
    DiagRecord record;
    record.kind    = DiagKind::kBrainRequest;
    record.source  = source_id_;
    record.payload = r;
    hub_->post(std::move(record));
}

void BrainLinkCommands::process(const translagatr::BrainRequest& req, CommandState& c,
                                LinkStats* stats, MonotonicTime now) {
    BrainReplyContext& r = c.reply;
    r.pending            = true;
    r.op                 = req.op;
    r.session            = req.session;
    r.request_id         = req.request_id;
    r.pi_instance        = pi_instance_;
    r.nonce              = req.nonce;
    r.result             = translagatr::kResultOk;
    repeated_            = false;

    if (req.version != translagatr::kBrainLinkVersion) {
        r.result = translagatr::kResultUnsupportedVersion;
        return;
    }
    if (translagatr::brainRequestMinLen(req.op) == 0) {
        r.result = translagatr::kResultUnsupportedOp;
        return;
    }
    if (req.request_id == 0 || !wellFormed(req)) {
        r.result = translagatr::kResultInvalidArgument;
        return;
    }
    if (req.op == translagatr::kOpHello) {
        hello(req, c, stats);
        return;
    }
    if (session_ == 0 || req.session != session_) {
        r.result = translagatr::kResultUnknownSession;
        if (stats != nullptr) {
            ++stats->unknown_session;
        }
        return;
    }
    if (have_newest_ && !newerId(req.request_id, newest_.request_id)) {
        repeated_ = true;
        repeat(req, c, stats, now);
        return;
    }

    have_newest_ = true;
    newest_      = req;
    accepted_    = true;
    execute(req, c, now, false);
}

void BrainLinkCommands::execute(const translagatr::BrainRequest& req, CommandState& c,
                                MonotonicTime now, bool repeat) {
    BrainReplyContext& r = c.reply;
    switch (req.op) {
    case translagatr::kOpGetState:
        if (!repeat) {
            const uint64_t before = bench_imu_ ? bench_imu_->sequence : 0;
            if (bench_imu_) {
                bench_imu_->accept(session_, req.imu_flags, req.imu_stamp_ms,
                                   req.imu_rotation_mdeg, now);
            }
            // at the request rate, like kBrainRequest; the live view reads latest()
            if (hub_ != nullptr) {
                DiagVexImu v;
                v.session       = req.session;
                v.request_id    = req.request_id;
                v.flags         = req.imu_flags;
                v.stamp_ms      = req.imu_stamp_ms;
                v.rotation_mdeg = req.imu_rotation_mdeg;
                v.accepted      = bench_imu_ && bench_imu_->sequence != before;
                DiagRecord record;
                record.kind    = DiagKind::kVexImu;
                record.source  = source_id_;
                record.payload = v;
                hub_->post(std::move(record));
            }
        }
        break;
    case translagatr::kOpTelemetry:
        // display and capture only: never localization, never a reply body
        if (!repeat) {
            if (bench_imu_) {
                bench_imu_->acceptAttitude(
                    (req.telemetry.flags & translagatr::kTelemetryAttitude) != 0,
                    req.telemetry.stamp_ms, req.telemetry.roll_cdeg, req.telemetry.pitch_cdeg,
                    now);
            }
            if (hub_ != nullptr && request_len_ >= translagatr::kLinkEnvelopeLen +
                                                      translagatr::kBrainRequestHeaderLen +
                                                      translagatr::kTelemetryBodyLen) {
                DiagBrainTelemetry t;
                t.session = req.session;
                t.len     = translagatr::kTelemetryBodyLen;
                std::memcpy(t.body, request_frame_ + 4 + translagatr::kBrainRequestHeaderLen,
                            translagatr::kTelemetryBodyLen);
                DiagRecord record;
                record.kind    = DiagKind::kBrainTelemetry;
                record.source  = source_id_;
                record.payload = t;
                hub_->post(std::move(record));
            }
        }
        break;
    case translagatr::kOpSetPose: setPose(req, c); break;
    case translagatr::kOpProfileWrite: profileWrite(req, r); break;
    case translagatr::kOpProfileApply: profileApply(req, c); break;
    case translagatr::kOpReadDoc:
        r.doc_kind    = req.doc_kind;
        r.doc_id      = req.doc_id;
        r.doc_offset  = req.doc_offset;
        r.doc_max_len = req.max_len;
        break;
    case translagatr::kOpControl: {
        uint8_t detail = translagatr::kControlDetailNone;
        r.result       = profile_host_ == nullptr
                             ? static_cast<uint8_t>(translagatr::kResultNotReady)
                             : profile_host_->control(req.action, req.action_arg, now, detail);
        r.action         = req.action;
        r.control_detail = detail;
        control_ =
            ControlRecord{true, req.request_id, req.action, req.action_arg, r.result, detail};
        break;
    }
    case translagatr::kOpPathReport: {
        PathReport path;
        if (req.path_mode != translagatr::kPathNone) {
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
        if (hub_ != nullptr && !repeat) {
            DiagPath p;
            p.session    = req.session;
            p.command_id = req.command_id;
            p.mode       = req.path_mode;
            p.count      = req.point_count;
            for (uint8_t i = 0; i < req.point_count && i < translagatr::kPathReportMaxPoints; ++i) {
                p.points[i] = req.points[i];
            }
            DiagRecord record;
            record.kind    = DiagKind::kPath;
            record.source  = source_id_;
            record.payload = p;
            hub_->post(std::move(record));
        }
        break;
    }
    case translagatr::kOpReadWheels:
        // wheel readings come with the applied profile
        r.result = profile_host_ == nullptr
                       ? static_cast<uint8_t>(translagatr::kResultUnavailable)
                       : profile_host_->readWheels(now, r.wheel_count, r.wheels.data());
        break;
    default: break;
    }
}

void BrainLinkCommands::setPose(const translagatr::BrainRequest& req, CommandState& c) {
    BrainReplyContext& r = c.reply;
    set_pose_ = SetPoseRecord{true, req.request_id, req.x_mm, req.y_mm, req.heading_cdeg,
                              translagatr::kResultNotReady, 0};
    if (profile_host_ != nullptr && c.profile.applied_id == 0) {
        r.result = translagatr::kResultNotReady;   // no odometry to anchor before a profile
        return;
    }
    c.init_pose.x_m         = req.x_mm / 1000.0;
    c.init_pose.y_m         = req.y_mm / 1000.0;
    c.init_pose.heading_rad = cdegToRad(req.heading_cdeg);
    c.init_session          = session_;
    c.init_sequence += 1;
    set_pose_.result     = translagatr::kResultPending;
    set_pose_.sequence   = c.init_sequence;
    r.result             = translagatr::kResultPending;
    r.placement_sequence = c.init_sequence;
}

void BrainLinkCommands::profileWrite(const translagatr::BrainRequest& req, BrainReplyContext& r) {
    Staging&       s    = staging_;
    const bool     same = s.total_len != 0 && req.profile_id == s.id &&
                      req.total_len == s.total_len;
    const uint16_t held = same ? s.received : 0;
    if (req.offset > held) {
        r.result = translagatr::kResultInvalidArgument;   // a gap
        return;
    }
    const uint16_t end     = static_cast<uint16_t>(req.offset + req.data_len);
    const uint16_t overlap = static_cast<uint16_t>(std::min(end, held) - req.offset);
    if (std::memcmp(s.bytes.data() + req.offset, req.data, overlap) != 0) {
        r.result = translagatr::kResultInvalidArgument;   // a resend with other bytes
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

void BrainLinkCommands::profileApply(const translagatr::BrainRequest& req, CommandState& c) {
    BrainReplyContext& r  = c.reply;
    const uint32_t     id = req.profile_id;
    const Staging&     s  = staging_;
    if (s.total_len == 0 || s.id != id || s.total_len != req.total_len ||
        s.received != s.total_len || translagatr::crc32(s.bytes.data(), s.total_len) != id) {
        r.result = translagatr::kResultInvalidArgument;   // staging incomplete or corrupt
        return;
    }

    ProfileStatus& p = c.profile;
    r.profile_id     = id;
    if (id == p.applied_id) {
        // the running profile: idempotent, nothing resets
        p.state         = translagatr::kProfileApplied;
        p.id            = id;
        p.reason        = translagatr::kProfileReasonNone;
        p.detail        = 0;
        r.profile_state = translagatr::kProfileApplied;
        return;
    }
    if (p.state == translagatr::kProfileApplying && p.id == id) {
        r.result        = translagatr::kResultPending;
        r.profile_state = translagatr::kProfileApplying;
        return;
    }
    if (p.state == translagatr::kProfileRejected && p.id == id) {
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
        rejectProfile(id, translagatr::kProfileReasonNotAccepted, 0, c);
        return;
    }
    translagatr::RobotProfileDoc profile;
    if (!translagatr::decodeRobotProfile(s.bytes.data(), s.total_len, profile)) {
        rejectProfile(id, translagatr::kProfileReasonFormat, 0, c);
        return;
    }
    uint8_t reason = translagatr::kProfileReasonNone;
    uint8_t detail = 0;
    if (!translagatr::validateRobotProfile(profile, reason, detail) ||
        !profile_host_->prepare(profile, id, reason, detail)) {
        rejectProfile(id, reason, detail, c);
        return;
    }
    p.state         = translagatr::kProfileApplying;
    p.id            = id;
    p.reason        = translagatr::kProfileReasonNone;
    p.detail        = 0;
    r.result        = translagatr::kResultPending;
    r.profile_state = translagatr::kProfileApplying;
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
    c.profile.state  = translagatr::kProfileRejected;
    c.profile.id     = id;
    c.profile.reason = reason;
    c.profile.detail = detail;

    BrainReplyContext& r = c.reply;
    r.result             = translagatr::kResultProfileRejected;
    r.profile_id         = id;
    r.profile_state      = translagatr::kProfileRejected;
    r.profile_reason     = reason;
    r.profile_detail     = detail;
}

void BrainLinkCommands::hello(const translagatr::BrainRequest& req, CommandState& c,
                              LinkStats* stats) {
    BrainReplyContext& r = c.reply;
    if (session_ != 0 && req.nonce == opening_nonce_ && req.request_id == opening_rid_ &&
        !accepted_) {
        r.session = session_;   // retry of the opening HELLO
        return;
    }
    if (std::find(recent_nonces_.begin(), recent_nonces_.end(), req.nonce) !=
        recent_nonces_.end()) {
        r.result = translagatr::kResultStale;
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
    newest_      = translagatr::BrainRequest{};
    set_pose_    = SetPoseRecord{};
    control_     = ControlRecord{};

    // client state is per session; localization, placement, the profile and
    // its staging are not
    c.session = session_;
    c.path    = PathReport{};
    r.session = session_;
}

void BrainLinkCommands::repeat(const translagatr::BrainRequest& req, CommandState& c,
                               LinkStats* stats, MonotonicTime now) {
    BrainReplyContext& r = c.reply;
    if (set_pose_.valid && req.request_id == set_pose_.rid) {
        if (req.op == translagatr::kOpSetPose && req.x_mm == set_pose_.x_mm &&
            req.y_mm == set_pose_.y_mm && req.heading_cdeg == set_pose_.heading_cdeg) {
            r.result             = set_pose_.result;
            r.placement_sequence = set_pose_.sequence;
            if (stats != nullptr) {
                ++stats->duplicates;
            }
        } else {
            r.result = translagatr::kResultInvalidArgument;
        }
        return;
    }
    if (control_.valid && req.request_id == control_.rid) {
        if (req.op == translagatr::kOpControl && req.action == control_.action &&
            req.action_arg == control_.arg) {
            if (control_.result == translagatr::kResultPending && profile_host_ != nullptr) {
                // a Pico operation still running: report how far it got, the
                // Pico is never asked again
                control_.result = profile_host_->controlProgress(control_.action, control_.arg,
                                                                 now, control_.detail);
            }
            r.result         = control_.result;
            r.action         = control_.action;
            r.control_detail = control_.detail;
            if (stats != nullptr) {
                ++stats->duplicates;
            }
        } else {
            r.result = translagatr::kResultInvalidArgument;
        }
        return;
    }
    if (req.request_id == newest_.request_id) {
        if (repeatable(req.op) && sameRequest(req, newest_)) {
            execute(req, c, now, true);
        } else {
            r.result = translagatr::kResultInvalidArgument;
        }
        return;
    }
    r.result = translagatr::kResultStale;
    if (stats != nullptr) {
        ++stats->stale;
    }
}

} // namespace navigatr
