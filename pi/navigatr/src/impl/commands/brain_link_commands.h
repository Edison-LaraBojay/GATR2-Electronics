// brain_link_commands.h
// Brain link v3 requests in, command state and the reply owed out. The Pi
// side of the session rules: Pi-assigned sessions, per-session request_id
// dedupe, a recent HELLO nonce ring, and pi_instance, new per construction
// and per reset. The brain_link publisher sends the reply in the same cycle.
//
//   <CommandCollection type="brain_link">
//       <Serial resource_id="brain_uart"/>
//       <Reply window_ms="40" turnaround_guard_us="1000"/>   optional
//   </CommandCollection>
//
// Each run drains the link until a read returns nothing, stamping every read
// with the link clock. Only the newest request a drain completes is
// processed. Its reply may start from its completion read +
// turnaround_guard_us until the last read of the previous drain + window_ms.
// No reply for a request completed in the first drain since construction or
// reset, or followed by more bytes in the same drain; it is still applied.
// A drain with no bytes discards a partial frame. The loop period must be at
// most half the window.

#pragma once
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "common/frame_codec.h"
#include "contracts/commands.h"
#include "resources/serial_link.h"
#include "resources/brain_imu_bench.h"

namespace navigatr
{

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
    void     process(const gatr2::BrainRequest& req, CommandState& c, LinkStats* stats,
                     MonotonicTime now);
    void     hello(const gatr2::BrainRequest& req, CommandState& c, LinkStats* stats);
    void     repeat(const gatr2::BrainRequest& req, BrainReplyContext& r, LinkStats* stats);
    uint32_t randomNonzero(uint32_t differs_from);

    std::shared_ptr<SerialLink> link_;
    std::shared_ptr<BrainImuBench> bench_imu_;
    std::string                 diagnostics_id_;
    int64_t                     window_us_ = 40000;
    int64_t                     guard_us_  = 1000;

    gatr2::FrameReader reader_;
    bool               have_last_read_ = false;   // a previous drain exists
    int64_t            last_read_us_   = 0;       // its final, empty read

    std::mt19937 random_;
    uint32_t     pi_instance_ = 0;

    uint32_t              session_       = 0;
    uint32_t              opening_nonce_ = 0;
    uint16_t              opening_rid_   = 0;
    bool                  accepted_      = false;   // a non-HELLO request in this session
    std::vector<uint32_t> recent_nonces_;           // last opening nonces, newest last

    bool     have_newest_ = false;
    uint16_t newest_rid_  = 0;

    struct SetPoseRecord {
        bool     valid        = false;
        uint16_t rid          = 0;
        int32_t  x_mm         = 0;
        int32_t  y_mm         = 0;
        int32_t  heading_cdeg = 0;
        uint64_t sequence     = 0;   // init_sequence it was recorded under
    };
    struct SelectRecord {
        bool     valid       = false;
        uint16_t rid         = 0;
        uint8_t  landmark_id = 0;
        uint8_t  flags       = 0;
    };
    SetPoseRecord set_pose_;
    SelectRecord  select_;
};

} // namespace navigatr
