// brain_link_commands.h
// Brain link v4 requests in, command state and the reply owed out. The Pi
// side of the session rules: Pi-assigned sessions, per-session request_id
// dedupe, a recent HELLO nonce ring, and pi_instance, new per construction
// and per reset. The brain_link publisher sends the reply in the same cycle.
//
//   <CommandCollection type="brain_link">
//       <Serial resource_id="brain_uart"/>
//       <Reply window_ms="40" turnaround_guard_us="1000"/>   optional
//       <BenchImu resource_id="brain_imu"/>                   optional
//       <Pico resource_id="pico_telemetry"/>                  optional, Brain profiles:
//                                                             CONTROL 3/4 (IMU reinit,
//                                                             acquisition restart)
//   </CommandCollection>
//
// Each run drains the link until a read returns nothing, stamping every read
// with the link clock. Only the newest request a drain completes is
// processed. Its reply may start from its completion read +
// turnaround_guard_us until the last read of the previous drain + window_ms.
// No reply for a request completed in the first drain since construction or
// reset, or followed by more bytes in the same drain; it is still applied.
// A drain with no bytes discards a partial frame. The loop period must be at
// most half the window. Every run records pi_instance, whether the link read
// without closing, and the host time of the newest request in the command
// state, for inspection.
//
// Dedupe: a request id that is not newer is never applied again. The last
// SET_POSE and CONTROL are answered from their records; the newest id of a
// read-only or idempotent op with the same body is answered again (GET_STATE
// without re-accepting its IMU sample); any other reuse is InvalidArgument,
// anything older Stale.
//
// Robot profile: PROFILE_WRITE stages one document, keyed by profile_id and
// kept across sessions. PROFILE_APPLY checks the staged bytes, decodes and
// validates them, then hands them to the System's BrainProfileHost; without
// one the configuration takes no profile and APPLY is ProfileRejected
// (kProfileReasonNotAccepted). With a host, SET_POSE is NotReady until a
// profile is applied. Rejections are remembered per id.
//
// BenchImu: GET_STATE samples go to this mailbox; without it they are
// ignored.
//
// CONTROL goes to the profile host (NotReady without one) and is recorded
// like SET_POSE: a duplicate reports the recorded result and never runs
// again; while that result is Pending (a Pico operation) the duplicate
// reports the operation's current progress instead. READ_WHEELS is
// Unavailable without a profile host; with one the host reads the profile
// wheels (NotReady before a profile is applied).
//
// TELEMETRY is answered Ok (header only) and recorded for display and
// capture only: its attitude group goes to the BenchImu mailbox as the
// robot tilt (without the group the tilt is unavailable), and nothing else
// changes. A resend of the newest id is answered again.
//
// Instrumentation (with the System's DiagnosticsHub): the link's
// LinkMonitor sees every read at the decoded frame layer (for USB, the
// frame bytes the NG1 lines carried), every decoded request, and every
// rejected frame; the hub gets each GET_STATE bench IMU sample (when
// wanted), PATH_REPORT, TELEMETRY, and one record per processed request
// with the result actually sent (completed by the brain_link publisher).

#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "translaGATR/frame_codec.h"
#include "translaGATR/link_documents.h"
#include "contracts/brain_profile.h"
#include "contracts/commands.h"
#include "resources/brain_imu_bench.h"
#include "resources/serial_link.h"

namespace navigatr
{

class DiagnosticsHub;
class LinkMonitor;

// Display names of brain link v4 ops and results.
const char* brainOpName(uint8_t op);
const char* brainResultName(uint8_t result);

class BrainLinkCommands : public Commands
{
public:
    static std::unique_ptr<Commands> create(const ConfigNode& node,
                                            SlotInitializationContext& context,
                                            std::string& err);

    BrainLinkCommands();

    CommandsOutput run(const CommandsInput& in) override;

    void reset() override;

    uint32_t piInstance() const { return pi_instance_; }

private:
    void     process(const translagatr::BrainRequest& req, CommandState& c, LinkStats* stats,
                     MonotonicTime now);
    void     execute(const translagatr::BrainRequest& req, CommandState& c, MonotonicTime now,
                     bool repeat);
    void     hello(const translagatr::BrainRequest& req, CommandState& c, LinkStats* stats);
    void     repeat(const translagatr::BrainRequest& req, CommandState& c, LinkStats* stats,
                    MonotonicTime now);
    void     setPose(const translagatr::BrainRequest& req, CommandState& c);
    void     profileWrite(const translagatr::BrainRequest& req, BrainReplyContext& r);
    void     profileApply(const translagatr::BrainRequest& req, CommandState& c);
    void     rejectProfile(uint32_t id, uint8_t reason, uint8_t detail, CommandState& c);
    uint32_t randomNonzero(uint32_t differs_from);
    void     noteDecoded(const translagatr::BrainRequest& req);
    void     noteProcessed(const translagatr::BrainRequest& req, const BrainReplyContext& reply);

    std::shared_ptr<SerialLink>    link_;
    std::shared_ptr<BrainImuBench> bench_imu_;
    BrainProfileHost*              profile_host_ = nullptr;
    std::string                    diagnostics_id_;
    int64_t                        window_us_ = 40000;
    int64_t                        guard_us_  = 1000;

    DiagnosticsHub*              hub_       = nullptr;
    uint16_t                     source_id_ = 0;
    std::shared_ptr<LinkMonitor> monitor_;
    bool                         repeated_  = false;   // the processed request was a resend
    uint8_t                      request_len_ = 0;     // frame bytes of the newest request
    uint8_t                      request_frame_[translagatr::kMaxFrameLen] = {};

    translagatr::FrameReader reader_;
    bool               have_last_read_ = false;   // a previous drain exists
    int64_t            last_read_us_   = 0;       // its final, empty read

    std::mt19937 random_;
    uint32_t     pi_instance_ = 0;

    uint32_t              session_       = 0;
    uint32_t              opening_nonce_ = 0;
    uint16_t              opening_rid_   = 0;
    bool                  accepted_      = false;   // a non-HELLO request in this session
    std::vector<uint32_t> recent_nonces_;           // last opening nonces, newest last

    bool                have_newest_ = false;
    translagatr::BrainRequest newest_;   // the newest processed request

    struct SetPoseRecord {
        bool     valid        = false;
        uint16_t rid          = 0;
        int32_t  x_mm         = 0;
        int32_t  y_mm         = 0;
        int32_t  heading_cdeg = 0;
        uint8_t  result       = 0;   // Pending or NotReady
        uint64_t sequence     = 0;   // init_sequence it was recorded under
    };
    struct ControlRecord {
        bool     valid  = false;
        uint16_t rid    = 0;
        uint8_t  action = 0;
        uint8_t  arg    = 0;
        uint8_t  result = 0;
        uint8_t  detail = 0;
    };
    SetPoseRecord set_pose_;
    ControlRecord control_;

    struct Staging {
        uint32_t id        = 0;
        uint16_t total_len = 0;   // 0 = nothing staged
        uint16_t received  = 0;   // contiguous from offset 0
        std::array<uint8_t, translagatr::kProfileMaxLen> bytes{};
    };
    struct Rejection {
        uint32_t id     = 0;
        uint8_t  reason = 0;
        uint8_t  detail = 0;
    };
    Staging                staging_;
    std::vector<Rejection> rejected_;   // newest last
};

} // namespace navigatr
