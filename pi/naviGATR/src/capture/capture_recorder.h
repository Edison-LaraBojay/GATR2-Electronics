// capture_recorder.h
// The CaptureService implementation (docs/capture.md). One thread drains
// the DiagnosticsHub every pump_ms: while idle it keeps a rolling window of
// the default streams; a trigger (start(), or an enabled fault within its
// cooldown and hourly cap) copies the last pre_s of that window, records the
// selected streams until post_s after the trigger or a limit, then builds
// the ZIP bundle on the same thread and keeps the newest `keep` in memory
// (and, with a directory, on disk).
//
// Producers never wait for it: the hub drops and counts when the recorder
// falls behind, and fault()/noteReset() only queue a note. One capture at a
// time: a start while recording or finalizing is refused as busy, and every
// viewer sees the same status. Viewers disconnecting change nothing.

#pragma once
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "capture/capture_bundle.h"
#include "capture/capture_config.h"
#include "capture/capture_service.h"
#include "diagnostics/hub.h"

namespace navigatr
{

// What the recorder reads from the System it records; any thread.
class CaptureHost
{
public:
    virtual ~CaptureHost() = default;
    virtual std::string sessionId() const  = 0;
    virtual uint64_t    resetCount() const = 0;
    // Keys for metadata.json (configuration, profile, calibration,
    // localization, software), written into the open object. Read on the
    // capture thread once a second while a rolling window runs (for the
    // state at a window start) and when a bundle is built.
    virtual void writeMetadata(JsonWriter& w) const = 0;
};

class CaptureRecorder final : public CaptureService
{
public:
    struct Options {
        bool thread  = true; // false: the caller drives pump() (tests)
        int  pump_ms = 20;
        std::function<int64_t()> clock;   // Pi host us; HostClock::nowUs when empty
        std::function<int64_t()> wall_ms; // unix ms or -1; the system clock when empty
    };

    CaptureRecorder(const CaptureConfig& config, DiagnosticsHub& hub,
                    std::shared_ptr<const CaptureHost> host, Options options);
    ~CaptureRecorder() override;

    CaptureRecorder(const CaptureRecorder&)            = delete;
    CaptureRecorder& operator=(const CaptureRecorder&) = delete;

    bool start(const CaptureRequest& request, std::string& id, std::string& err) override;
    bool cancel(const std::string& id, std::string& err) override;
    void writeStatus(JsonWriter& w) const override;
    uint64_t version() const override { return version_.load(); }
    std::shared_ptr<const std::string> bundle(const std::string& id) const override;
    // Joins the thread, then finishes a capture still recording as truncated
    // (stop_reason "shutdown") so what was recorded stays downloadable.
    void stop() override;

    // From the System, any thread; queue a note and return.
    void fault(CaptureFault fault, const std::string& detail);
    void noteReset(uint64_t reset_count);

    // One step: drain the hub, route records, finish when due. The capture
    // thread's loop body; with Options::thread false the caller drives it.
    // One caller at a time.
    void pump();

    const CaptureConfig& config() const { return config_; }
    std::size_t          recordCap() const { return record_cap_; }
    uint32_t             rollingKinds() const { return rolling_kinds_; }

private:
    enum class State { kIdle, kRecording, kFinalizing };

    struct Trigger {
        std::string id;
        CaptureRequest request; // clamped
        double      pre_s_requested  = 0;
        double      post_s_requested = 0;
        std::string detail;
        int64_t     trigger_us = 0;
        int64_t     unix_ms    = -1;
    };

    struct Active {
        Trigger                 trigger;
        int64_t                 window_start_us = 0;
        int64_t                 end_us          = 0;
        int64_t                 rolling_oldest_us = -1;
        // a deque grows in small blocks: no doubling past max_mb, no copy
        std::deque<DiagRecord>  records;
        DiagKindCounts          not_selected{};
        DiagKindCounts          limit_dropped{};
        DiagKindCounts          hub_posted0{};
        DiagKindCounts          hub_dropped0{};
        uint64_t                faults_dropped0 = 0;
        uint64_t                rolling_evicted = 0; // of this window, by the record bound
        std::string             host_at_start;       // host metadata JSON at the window start
        bool                    limit_hit = false;
    };

    struct Finished {
        std::string id;
        std::string reason, requester, detail, stop_reason;
        int64_t     trigger_us = 0, end_us = 0;
        double      pre_s = 0, post_s = 0;
        uint32_t    kinds = 0;
        bool        truncated = false;
        std::vector<std::string> truncated_by;
        uint64_t    records   = 0;
        DiagKindCounts rows{};
        DiagKindCounts dropped{}; // hub ring full + capture limit + bundle size
        std::shared_ptr<const std::string> zip;
        std::string file;       // written path, empty when none
        std::string file_error; // why the directory write failed
    };

    struct FaultNote {
        CaptureFault fault;
        std::string  detail;
        int64_t      at_us = 0;
    };

    struct HubSample {
        int64_t        at_us = 0;
        DiagKindCounts posted{};
        DiagKindCounts dropped{};
    };

    struct HostSample {
        int64_t     at_us = 0;
        std::string json; // CaptureHost::writeMetadata as one object
    };

    int64_t now() const;
    int64_t wallMs() const;
    void    bump() { version_.fetch_add(1); }
    bool    startLocked(const CaptureRequest& request, const std::string& detail,
                        int64_t trigger_us, std::string& id, std::string& err);
    void    begin(Trigger trigger);
    void    route(const DiagRecord& r);
    void    trimRolling(int64_t now_us);
    void    sampleHub(int64_t now_us);
    void    sampleHost(int64_t now_us);
    std::string hostJson(int64_t at_us) const;
    void    handleFault(const FaultNote& f);
    void    finish(const char* stop_reason, bool truncated);
    bool    writeFile(const std::string& id, const std::string& zip, std::string& path,
                      std::string& err);
    void    writeActive(JsonWriter& w, int64_t now_us) const;

    const CaptureConfig                 config_;
    DiagnosticsHub&                     hub_;
    std::shared_ptr<const CaptureHost>  host_;
    Options                             options_;
    std::size_t                         record_cap_    = 0;
    bool                                memory_bound_  = false; // max_mb binds before max_records
    uint32_t                            rolling_kinds_ = 0;
    std::string                         id_prefix_;

    // shared with start/cancel/status callers
    mutable std::mutex          mutex_;
    std::condition_variable     wake_;
    bool                        woken_   = false;
    bool                        stopped_ = false;
    State                       state_   = State::kIdle;
    std::unique_ptr<Trigger>    pending_;
    std::string                 cancel_id_; // active or pending capture to discard
    std::deque<Finished>        finished_;  // oldest first
    std::vector<FaultNote>      faults_;
    uint64_t                    faults_dropped_ = 0;
    std::vector<std::pair<int64_t, uint64_t>> resets_;
    uint64_t                    next_seq_ = 0;
    std::string                 last_id_, last_outcome_, last_error_;
    // active progress, copied from the capture thread each pump
    std::string                 active_id_;
    Trigger                     active_trigger_;
    int64_t                     active_end_us_  = 0;
    uint64_t                    active_records_ = 0;
    uint64_t                    active_dropped_ = 0;
    int64_t                     last_progress_us_ = 0;
    // automatic triggers
    std::array<uint64_t, kCaptureFaultCount> auto_seen_{};
    uint64_t                    auto_fired_ = 0;
    uint64_t                    auto_busy_ = 0, auto_cooldown_ = 0, auto_capped_ = 0;
    std::deque<int64_t>         auto_times_; // fires in the last hour
    std::string                 auto_last_reason_;
    int64_t                     auto_last_us_ = -1;
    std::atomic<uint64_t>       version_{1};

    // capture thread only
    std::deque<DiagRecord>      rolling_;
    // records the bound evicted from the rolling window: (Pi stamp in ms,
    // count), kept as far back as a window can reach
    std::deque<std::pair<int64_t, uint64_t>> evicted_;
    std::vector<DiagRecord>     batch_;
    std::unique_ptr<Active>     active_;
    std::deque<HubSample>       hub_samples_;
    std::deque<HostSample>      host_samples_; // 1 Hz while a rolling window runs
    std::thread                 thread_;
};

// The recorder for a System's <Capture> configuration.
std::unique_ptr<CaptureRecorder> makeCaptureRecorder(System& system, const CaptureConfig& config);

} // namespace navigatr
