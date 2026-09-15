// system.h
// The coordinator. Owns the resource store, the captured stage executors
// (resources, sensors, localization), and the pipeline slot executables,
// and holds sole execution authority: downstream algorithms read the
// standard result maps and snapshots, never each other's executables, and
// registry lookup grants nothing at runtime.
//
// The semantic pipeline is fixed:
//
//   Resources -> Sensors -> Command Collection -> Localization
//     -> Field Estimation -> Target Resolution -> Publishing
//
// Resources, sensors and localization are aggregate stages built once by
// make_resources, make_sensors and make_localization; each returned
// executor captures its configured collection and owns iteration,
// bookkeeping and map assembly. Construction is atomic: parse, register
// (caller), build resources, build sensors, build localization, build
// slots resolving every reference with payload compatibility, then freeze;
// any failure destroys the candidate and nothing partial ever runs.
//
// Two execution modes share the same stage functions:
//
//   step(now)      every stage inline on the caller's thread, in order;
//                  the synchronous mode tests and replay use
//   start()/stop() two workers. The estimation worker runs resources,
//                  sensors, commands, localization, then target resolution
//                  and publishing against the newest field snapshot, at the
//                  configured loop rate. The field worker runs field
//                  estimation (perception, association, landmark estimate)
//                  on the newest sensor snapshot handed to it, replacing a
//                  pending snapshot it has not reached yet: camera work is
//                  latest-frame, while every motion increment is consumed
//                  by localization on the thread that acquired it.
//
// One writer per mutable state: the estimation worker owns the executors
// and their retained maps, command and target state; the field worker owns
// field estimation and the field snapshot; localization's feed is the only
// history writer. Readers (the other worker, publishing, inspection) get
// immutable shared snapshots or synchronized lookups; no reference into a
// retained map crosses a thread. Shutdown stops the estimation worker
// first, then the field worker, then destroys members in reverse
// dependency order. reset() while running stops both workers, resets every
// stage exactly once, and restarts them, so two workers never race to reset
// one configured resource.

#pragma once
#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "contracts/commands.h"
#include "contracts/field_estimation.h"
#include "contracts/localization.h"
#include "contracts/publishing.h"
#include "contracts/target_resolution.h"
#include "core/diagnostics.h"
#include "core/function_registry.h"
#include "core/records.h"
#include "resources/resource_store.h"
#include "runtime/build_options.h"
#include "runtime/handoff.h"
#include "runtime/inspection_config.h"
#include "runtime/inspection_state.h"
#include "runtime/localization_stage.h"
#include "runtime/resource_stage.h"
#include "runtime/sensor_catalog.h"
#include "runtime/sensor_stage.h"
#include "state/command_state.h"
#include "state/field_state.h"
#include "state/robot_state.h"
#include "state/robot_state_feed.h"
#include "state/target_state.h"

namespace navigatr
{

// What the field stage publishes per invocation: the estimate plus the
// evidence maps that cross its boundary, tagged with the invocation they
// came from.
struct FieldSnapshot {
    FieldState     field;
    ObservationMap observations;
    AssociationMap associations;
    MonotonicTime  at;   // host clock of the invocation
    uint64_t       invocation = 0;
    FunctionStatus status     = FunctionStatus::kOk;
    std::string    diagnostic;
};

// Command and target state as the estimation worker last left them.
struct ReportingSnapshot {
    CommandState  command;
    TargetState   target;
    MonotonicTime at;
    uint64_t      cycle = 0;
};

class System
{
public:
    static std::unique_ptr<System> buildFromFile(const std::string& path,
                                                 const FunctionRegistry& functions,
                                                 std::string&            err,
                                                 const BuildOptions&     options = {});
    // Inline/resolved System XML only. Section file references require
    // buildFromFile, which supplies the containing-file base; string input
    // never implicitly resolves references against the working directory.
    static std::unique_ptr<System> buildFromString(const char*             xml,
                                                   const FunctionRegistry& functions,
                                                   std::string&            err,
                                                   const BuildOptions&     options = {});
    ~System();

    // One full pipeline cycle, every stage inline. now is the host
    // monotonic clock. Not while workers run.
    void step(MonotonicTime now);

    // The estimation cycle alone: resources, sensors, commands,
    // localization. Used by the estimation worker and by step().
    void estimationCycle(MonotonicTime now);

    // The field stage alone against a sensor snapshot; publishes a new
    // field snapshot. Used by the field worker and by step().
    void fieldCycle(const SensorMap& sensors, MonotonicTime now);

    // Target resolution and publishing against the newest field snapshot.
    void reportingCycle(MonotonicTime now);

    // Workers. start() spawns the estimation and field workers; stop()
    // stops and joins them in order. Both are idempotent.
    bool start(std::string& err);
    void stop();
    bool running() const { return running_.load(); }

    // Back to power-on: implementations reset, states cleared, counters
    // kept. Safe while running: workers stop, everything resets once,
    // workers restart. Counted in resetCount() so readers can discard
    // incompatible history.
    void reset();

    double   loopRateHz() const { return loop_rate_hz_; }
    uint64_t cycle() const { return cycle_.load(); }

    // Identity of the running profile: the Configuration id (or the file
    // name for a plain System document) and a content digest over every
    // contributing file.
    const std::string& configurationId() const { return configuration_id_; }
    const std::string& configurationName() const { return configuration_name_; }
    uint64_t           configurationDigest() const { return configuration_digest_; }

    // Identity of this process instance, for inspection clients to detect
    // a restart, plus the count of in-process resets.
    const std::string& sessionId() const { return session_id_; }
    uint64_t           resetCount() const { return reset_count_.load(); }

    // Latest stage results, retained between cycles. Inline mode or
    // stopped workers only: these are the estimation worker's own maps.
    const ResourceMap& resourceMap() const { return execute_resources_.retained(); }
    const SensorMap&   sensorMap() const { return execute_sensors_.retained(); }

    const RobotState&   robot() const { return robot_; }
    const FieldState&   field() const { return field_->field; }
    const CommandState& command() const { return command_; }
    const TargetState&  target() const { return target_; }
    Diagnostics&        diagnostics() { return diagnostics_; }
    Diagnostics&        fieldDiagnostics() { return field_diagnostics_; }

    // Thread-safe readers, for the other worker and inspection.
    std::shared_ptr<RobotStateFeed> robotFeed() const { return execute_localization_.feed(); }
    const LocalizationExecutor&     localization() const { return execute_localization_; }
    std::shared_ptr<const FieldSnapshot>       fieldSnapshot() const;
    std::shared_ptr<const ReportingSnapshot>   reportingSnapshot() const;
    std::shared_ptr<const SourceHealthSnapshot> sourceHealth() const;
    std::shared_ptr<const DiagnosticsSnapshot>  diagnosticsSnapshot() const;
    std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>> detectionFrames() const;
    WorkerStatsSnapshot estimationStats() const { return estimation_stats_.snapshot(); }
    WorkerStatsSnapshot fieldStats() const { return field_stats_.snapshot(); }

    const InspectionConfig& inspection() const { return inspection_; }

    // Non-fatal build notes, e.g. a serial device that failed to open.
    const std::vector<std::string>& warnings() const { return warnings_; }

    // Initialized objects and declared outputs, for tests and tooling.
    const ResourceStore&   resources() const { return resources_; }
    const ResourceCatalog& resourceCatalog() const { return execute_resources_.outputs(); }
    const SensorCatalog&   sensorCatalog() const { return execute_sensors_.outputs(); }

private:
    System() = default;

    bool build(const char* xml, const FunctionRegistry& functions,
               const BuildOptions& options, std::string& err);

    LocalizationRequests requestsFrom(const CommandState& command) const;

    void resetStages();
    bool startWorkers(std::string& err);   // lifecycle mutex held
    void requestStop();
    void estimationWorker();
    void fieldWorker();
    void publishSourceHealth(MonotonicTime now);
    void publishDetectionFrames(const SensorMap& sensors, const FieldSnapshot& snapshot,
                                MonotonicTime now);

    // Drops entries whose id was never declared or whose payload
    // contradicts the declaration, noting a fault against the producer.
    template <typename Map, typename Decls>
    void enforceDeclared(Map& map, const Decls& decls, const std::string& label,
                         Diagnostics& diagnostics) {
        for (auto it = map.begin(); it != map.end();) {
            const auto* decl = [&]() -> const typename Decls::value_type* {
                for (const auto& d : decls) {
                    if (d.id == it->first) {
                        return &d;
                    }
                }
                return nullptr;
            }();
            if (decl == nullptr || !decl->payload.matches(it->second.payload.cppType())) {
                diagnostics.note(label + "/undeclared_output:" + it->first.value,
                                 FunctionStatus::kFault);
                it = map.erase(it);
            } else {
                ++it;
            }
        }
    }

    double loop_rate_hz_ = 100.0;

    std::string configuration_id_     = "inline";
    std::string configuration_name_;
    uint64_t    configuration_digest_ = 0;
    std::string session_id_;

    InspectionConfig inspection_;

    // destruction order: declared first, destroyed last
    ResourceStore            resources_;
    std::vector<std::string> warnings_;

    ResourceExecutor     execute_resources_;
    SensorExecutor       execute_sensors_;
    LocalizationExecutor execute_localization_;

    // Declared outputs, kept for runtime enforcement: a producer cannot
    // place an undeclared id or a payload contradicting its declaration
    // into a standard map.
    std::vector<ObservationOutputDecl> observation_decls_;
    std::vector<AssociationOutputDecl> association_decls_;

    std::unique_ptr<Commands>         commands_;
    std::unique_ptr<FieldEstimation>  field_estimation_;
    std::unique_ptr<TargetResolution> target_resolution_;
    std::unique_ptr<Publishing>       publishing_;
    std::string                       slot_labels_[4];

    // estimation worker state
    std::atomic<uint64_t> cycle_{0};
    RobotState            robot_;
    CommandState          command_;
    TargetState           target_;
    Diagnostics           diagnostics_;

    // field worker state
    Diagnostics field_diagnostics_;
    uint64_t    field_invocations_ = 0;

    // published snapshots
    mutable std::mutex                          snapshot_mutex_;
    std::shared_ptr<const FieldSnapshot>        field_ = std::make_shared<FieldSnapshot>();
    std::shared_ptr<const ReportingSnapshot>    reporting_;
    std::shared_ptr<const SourceHealthSnapshot> source_health_;
    std::shared_ptr<const DiagnosticsSnapshot>  diagnostics_snapshot_;
    std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>> detection_frames_;

    // workers
    struct FieldWork {
        std::shared_ptr<const SensorMap> sensors;
        MonotonicTime                    now;
    };
    LatestSlot<FieldWork> field_handoff_;
    WorkerStats           estimation_stats_{"estimation"};
    WorkerStats           field_stats_{"field"};
    std::thread           estimation_thread_;
    std::thread           field_thread_;
    std::atomic<bool>     stop_{false};
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> reset_count_{0};
    std::mutex            lifecycle_mutex_;   // start, stop, reset
    std::mutex            wake_mutex_;
    std::condition_variable wake_;
};

} // namespace navigatr
