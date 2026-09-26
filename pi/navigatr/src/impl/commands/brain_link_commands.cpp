// brain_link_commands.cpp

#include "impl/commands/brain_link_commands.h"

#include <algorithm>
#include <atomic>
#include <cstdio>

#include "math/angles.h"
#include "resources/resource_store.h"

namespace navigatr
{

namespace
{

constexpr std::size_t kRecentNonces = 4;

// 16-bit serial arithmetic: a is newer when (a - b) mod 65536 is in 1..32767.
bool newerId(uint16_t a, uint16_t b) {
    const uint16_t d = static_cast<uint16_t>(a - b);
    return d != 0 && d < 0x8000;
}

std::seed_seq::result_type seedPart(int64_t v, int shift) {
    return static_cast<std::seed_seq::result_type>(static_cast<uint64_t>(v) >> shift);
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
    if (node.child("BenchImu").valid()) {
        commands->bench_imu_ = context.resources->require<BrainImuBench>(
            ResourceId{node.child("BenchImu").attr("resource_id")}, err);
        if (!commands->bench_imu_) return nullptr;
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
    if (bench_imu_) bench_imu_->reset();
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
    newest_rid_  = 0;
    set_pose_    = SetPoseRecord{};
    select_      = SelectRecord{};
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
    if (gatr2::brainRequestLen(req.op) == 0) {
        r.result = gatr2::kResultUnsupportedOp;
        return;
    }
    if (req.op == gatr2::kOpGetStateWithImu && !bench_imu_) {
        r.result = gatr2::kResultUnsupportedOp;
        return;
    }
    if (req.op == gatr2::kOpGetStateWithImu && (req.imu_flags & ~gatr2::kBenchImuValid)) {
        r.result = gatr2::kResultInvalidArgument;
        return;
    }
    if (req.request_id == 0) {
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
    if (have_newest_ && !newerId(req.request_id, newest_rid_)) {
        repeat(req, r, stats);
        return;
    }

    have_newest_ = true;
    newest_rid_  = req.request_id;
    accepted_    = true;
    switch (req.op) {
    case gatr2::kOpGetStateWithImu:
        bench_imu_->accept(session_, req.imu_flags, req.imu_stamp_ms,
                           req.imu_rotation_mdeg, now);
        break;
    case gatr2::kOpSetPose:
        c.init_pose.x_m         = req.x_mm / 1000.0;
        c.init_pose.y_m         = req.y_mm / 1000.0;
        c.init_pose.heading_rad = cdegToRad(req.heading_cdeg);
        c.init_session          = session_;
        c.init_sequence += 1;
        set_pose_ = SetPoseRecord{true, req.request_id, req.x_mm, req.y_mm, req.heading_cdeg,
                                  c.init_sequence};
        r.result             = gatr2::kResultPending;
        r.placement_sequence = c.init_sequence;
        break;
    case gatr2::kOpSelectLandmark:
        if ((req.select_flags & gatr2::kSelectFlagSelected) != 0) {
            c.object_requested = true;
            c.object_wire_id   = req.landmark_id;
        } else {
            c.object_requested = false;
        }
        c.object_sequence += 1;
        select_        = SelectRecord{true, req.request_id, req.landmark_id, req.select_flags};
        r.landmark_id  = req.landmark_id;
        r.select_flags = req.select_flags;
        break;
    default:
        break;   // GET_STATE changes nothing
    }
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

    session_       = randomNonzero(session_);
    if (bench_imu_) bench_imu_->reset(session_);
    opening_nonce_ = req.nonce;
    opening_rid_   = req.request_id;
    accepted_      = false;
    recent_nonces_.push_back(req.nonce);
    if (recent_nonces_.size() > kRecentNonces) {
        recent_nonces_.erase(recent_nonces_.begin());
    }
    have_newest_ = false;
    set_pose_    = SetPoseRecord{};
    select_      = SelectRecord{};

    // client state is per session; localization and placement are not
    c.session          = session_;
    c.object_requested = false;
    c.object_wire_id   = 0;
    c.object_sequence += 1;
    r.session = session_;
}

void BrainLinkCommands::repeat(const gatr2::BrainRequest& req, BrainReplyContext& r,
                               LinkStats* stats) {
    if (set_pose_.valid && req.request_id == set_pose_.rid) {
        if (req.op == gatr2::kOpSetPose && req.x_mm == set_pose_.x_mm &&
            req.y_mm == set_pose_.y_mm && req.heading_cdeg == set_pose_.heading_cdeg) {
            r.result             = gatr2::kResultPending;
            r.placement_sequence = set_pose_.sequence;
            if (stats != nullptr) {
                ++stats->duplicates;
            }
        } else {
            r.result = gatr2::kResultInvalidArgument;
        }
        return;
    }
    if (select_.valid && req.request_id == select_.rid) {
        if (req.op == gatr2::kOpSelectLandmark && req.landmark_id == select_.landmark_id &&
            req.select_flags == select_.flags) {
            r.landmark_id  = req.landmark_id;
            r.select_flags = req.select_flags;
            if (stats != nullptr) {
                ++stats->duplicates;
            }
        } else {
            r.result = gatr2::kResultInvalidArgument;
        }
        return;
    }
    if (req.request_id == newest_rid_) {
        if (req.op != gatr2::kOpGetState && req.op != gatr2::kOpGetStateWithImu) {
            r.result = gatr2::kResultInvalidArgument;   // fresh state for a GET_STATE
        }
        return;
    }
    r.result = gatr2::kResultStale;
    if (stats != nullptr) {
        ++stats->stale;
    }
}

} // namespace navigatr
