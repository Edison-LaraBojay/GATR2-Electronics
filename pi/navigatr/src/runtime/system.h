// system.h
// The coordinator. Owns the resource store, the captured stage executors
// (resources, sensors, localization, world estimation), and the pipeline
// slot executables, and holds sole execution authority: downstream
// algorithms read the standard result maps and snapshots, never each
// other's executables, and registry lookup grants nothing at runtime.
//
// The semantic pipeline is fixed:
//
//   Resources -> Sensors -> Command Collection -> Localization
//     -> World Estimation -> Target Resolution -> Publishing
//
// Resources, sensors, localization and world estimation are aggregate
// stages built once by make_resources, make_sensors, make_localization and
// make_world_estimation; each returned executor captures its configured
// implementations and owns iteration, bookkeeping and map assembly.
// Construction is atomic: parse, register (caller), build resources, build
// sensors, build localization, build world estimation, build slots
// resolving every reference with payload compatibility, then freeze; any
// failure destroys the candidate and nothing partial ever runs.
//
// Two execution modes share the same stage functions:
//
//   step(now)      every stage inline on the caller's thread, in order;
//                  the synchronous mode tests and replay use
//   start()/stop() two workers. The estimation worker runs resources,
//                  sensors, commands, localization, then target resolution
//                  and publishing against the newest field snapshot, at the
//                  configured loop rate. The field worker runs world
//                  estimation (whatever subpipeline the selected estimator
//                  owns) on the newest sensor snapshot handed to it,
//                  replacing a pending snapshot it has not reached yet:
//                  camera work is latest-frame, while every motion
//                  increment is consumed by localization on the thread that
//                  acquired it.
//
// One writer per mutable state: the estimation worker owns the executors
// and their retained maps, command and target state; the field worker owns
// world estimation and the field snapshot; localization's feed is the only
// history writer. Readers (the other worker, publishing, inspection) get
// immutable shared snapshots or synchronized lookups; no reference into a
// retained map crosses a thread. Shutdown stops the estimation worker
// first, then the field worker, then destroys members in reverse
// dependency order. reset() while running stops both workers, resets every
// stage exactly once, and restarts them, so two workers never race to reset
// one configured resource.
//
// A Brain-profiled configuration (<BrainProfile> under Localization) starts
// waiting: noop localization while resources, commands, world estimation,
// publishing and inspection run. The brain_link commands slot hands a
// validated profile to the System, which checks this Pi's capabilities and
// builds the candidate sensors and localization at once on the estimation
// worker. The swap happens at a controlled boundary, in the reset()
// pattern: applyPendingProfile() on the thread that owns start() in worker
// mode (workers stop, the candidate moves in, workers restart), or the top
// of step() inline. Resources, the commands slot with its session and
// pi_instance, world estimation, publishing and inspection survive. A new
// profile advances the odometry epoch, clears history, leaves the robot
// unplaced and withdraws every earlier placement request; re-applying the
// running profile changes nothing. The latest APPLY wins: a candidate still
// waiting for the boundary when the running profile is applied again, or
// another profile is refused, is dropped.
//
// Recovery. Every cycle the sources the running profile uses (its encoders,
// the Pico IMU or the Brain VEX IMU) are checked for loss (sensor_loss.h):
// stale longer than sensor_loss_ms, restarted, or a used Pico IMU not ready.
// A loss while the robot is placed ends pose continuity like a new profile:
// odometry epoch + 1, unplaced, earlier placement requests withdrawn, an
// event naming the source; on_sensor_loss="warn" only logs it. After
// localization runs, a rise in an observation function's dropped intervals
// (measured motion it had to discard: a gap past its own limit, a restart,
// movement while its bias calibrated) ends continuity the same way, since
// the pose may miss that motion; a placement from that very cycle is
// withdrawn with it. With
// <Pico resource_id=.../> on the brain_link CommandCollection the System
// runs CONTROL 3 (reinitialize the Pico IMU, then recalibrate) and 4
// (restart acquisition, done once the new acquisition epoch arrives)
// through PicoControl, each Pending until the Pico reports completion and
// bounded by kPicoOperationMs; both restart a used source, so both need a
// placement afterwards. Lifecycle events (links, Pico identity, calibration,
// sensor loss) go to events(). Each reporting cycle also publishes what the
// Brain would read (the state block, the wheel readings, the running Pico
// operation) in reportingSnapshot(), for inspection.

#pragma once
#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "contracts/brain_profile.h"
#include "contracts/commands.h"
#include "contracts/field_estimation.h"
#include "contracts/localization.h"
#include "contracts/publishing.h"
#include "contracts/target_resolution.h"
#include "core/diagnostics.h"
#include "core/function_registry.h"
#include "core/records.h"
#include "resources/pico_control.h"
#include "resources/resource_store.h"
#include "runtime/brain_profile_builder.h"
#include "runtime/build_options.h"
#include "runtime/handoff.h"
#include "runtime/inspection_config.h"
#include "runtime/inspection_state.h"
#include "runtime/localization_stage.h"
#include "runtime/resource_stage.h"
#include "runtime/sensor_catalog.h"
#include "runtime/sensor_stage.h"
#include "runtime/sensor_loss.h"
#include "runtime/stationary_precheck.h"
#include "runtime/world_estimation_stage.h"
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

// A CONTROL 3 or 4 running on the Pico, wire codes.
struct PicoOperation {
    bool          active = false;
    uint8_t       action = 0;   // gatr2::ControlAction
    uint32_t      handle = 0;
    MonotonicTime started;
    uint8_t       acq_epoch = 0;   // at submit, RestartAcquisition
    uint64_t      restarts  = 0;
    uint8_t       result    = 0;   // gatr2::BrainResult, Pending until settled
    uint8_t       detail    = 0;   // gatr2::ControlDetail
};

// Command and target state as the estimation worker last left them, and
// what the Brain link would report now.
struct ReportingSnapshot {
    CommandState  command;
    TargetState   target;
    MonotonicTime at;
    uint64_t      cycle = 0;

    std::optional<gatr2::BrainState> brain_state;   // GET_STATE now; brain_link only
    std::vector<gatr2::WheelReading> wheels;        // READ_WHEELS now, profile order
    PicoOperation                    pico_operation;
};

// What the running configuration binds, for readers on any thread. Replaced
// as a whole at a profile boundary.
struct BindingView {
    SensorCatalog                         sensors;
    std::string                           estimator_type;
    std::vector<std::string>              warnings;   // build, then the running profile's
    bool                                  brain_profile = false;   // takes a Brain profile
    std::shared_ptr<const ProfileBinding> profile;   // running profile, null when none
};

// A lifecycle event worth a line in a log or the inspection page.
struct RuntimeEvent {
    uint64_t      sequence = 0;   // counts every event since build
    MonotonicTime at;             // host clock
    std::string   text;
};

class System
{
public:
    static constexpr int64_t kPicoOperationMs = 15000;   // CONTROL 3/4 bound
    static constexpr int64_t kWheelFreshMs    = 150;     // READ_WHEELS fresh flag

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

    // World estimation alone against a sensor snapshot; publishes a new
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
    // incompatible history. A Brain-profiled System waits for a profile
    // again.
    void reset();

    // Worker mode profile boundary, called periodically by the thread that
    // owns start(): swaps in a prepared profile (workers stop, the candidate
    // moves in, workers restart). False when nothing was pending. A
    // candidate a later APPLY superseded is dropped. Inline, step() does
    // this itself.
    bool applyPendingProfile();

    // The running Brain profile, null when none; any thread.
    std::shared_ptr<const ProfileBinding> profileBinding() const;

    // Newest lifecycle events, oldest first, bounded; any thread.
    std::vector<RuntimeEvent> events() const;

    // The Pico link the brain_link CommandCollection names, null when none.
    // Fixed at build; link() is safe on any thread.
    const std::shared_ptr<PicoControl>& picoControl() const { return pico_; }

    double   loopRateHz() const { return loop_rate_hz_; }
    uint64_t cycle() const { return cycle_.load(); }

    // Configured CommandCollection type, e.g. brain_link, which needs the
    // workers: step() runs world estimation between request and reply.
    const std::string& commandsType() const { return commands_type_.value; }

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

    // Thread-safe readers, for the other worker and inspection. The feed
    // outlives profile boundaries.
    std::shared_ptr<RobotStateFeed>    robotFeed() const { return feed_; }
    std::shared_ptr<const BindingView> bindingView() const;
    // Inline mode or stopped workers only: a profile boundary replaces it.
    const LocalizationExecutor&     localization() const { return execute_localization_; }
    const WorldEstimationExecutor&  worldEstimation() const { return execute_world_; }
    std::shared_ptr<const FieldSnapshot>       fieldSnapshot() const;
    std::shared_ptr<const ReportingSnapshot>   reportingSnapshot() const;
    std::shared_ptr<const SourceHealthSnapshot> sourceHealth() const;
    std::shared_ptr<const DiagnosticsSnapshot>  diagnosticsSnapshot() const;
    std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>> detectionFrames() const;
    WorkerStatsSnapshot estimationStats() const { return estimation_stats_.snapshot(); }
    WorkerStatsSnapshot fieldStats() const { return field_stats_.snapshot(); }

    const InspectionConfig& inspection() const { return inspection_; }

    // Non-fatal build notes, e.g. a serial device that failed to open.
    // Warnings of the running profile are in bindingView().
    const std::vector<std::string>& warnings() const { return warnings_; }

    // Initialized objects and declared outputs, for tests and tooling.
    // sensorCatalog() is inline mode or stopped workers only.
    const ResourceStore&   resources() const { return resources_; }
    const ResourceCatalog& resourceCatalog() const { return execute_resources_.outputs(); }
    const SensorCatalog&   sensorCatalog() const { return execute_sensors_.outputs(); }

private:
    class ProfileHost;

    // A built profile waiting for the boundary.
    struct ProfileCandidate {
        std::shared_ptr<ProfileBinding> binding;
        SensorExecutor                  sensors;
        LocalizationExecutor            localization;
        std::vector<std::string>        warnings;
    };

    System();

    bool build(const char* xml, const FunctionRegistry& functions,
               const BuildOptions& options, std::string& err);
    bool buildProfileHost(const ConfigNode& pipeline, const ConfigNode& profile,
                          const FunctionRegistry& functions, std::string& err);
    std::optional<LocalizationExecutor> buildWaitingLocalization(std::string& err);

    LocalizationRequests requestsFrom(const CommandState& command) const;

    // BrainProfileHost, on the estimation worker.
    bool    prepareProfile(const gatr2::RobotProfileDoc& profile, uint32_t id, uint8_t& reason,
                           uint8_t& detail);
    uint8_t controlProfile(uint8_t action, uint8_t arg, MonotonicTime now, uint8_t& detail);
    uint8_t controlProgress(uint8_t action, MonotonicTime now, uint8_t& detail);
    uint8_t readWheels(MonotonicTime now, uint8_t& count, gatr2::WheelReading* wheels) const;

    // Pico commands and recovery bookkeeping, on the estimation worker.
    uint32_t submitPico(uint8_t op, uint8_t arg, MonotonicTime now, uint8_t& detail);
    void     updatePicoOperation(MonotonicTime now);
    void     watchSensors(const SensorMap& sensors, MonotonicTime now);
    void     watchModels(MonotonicTime now);   // after localization
    void     losePlacement(MonotonicTime now, const std::string& why, bool note);
    void     noteRecovery(MonotonicTime now);

    // Boundary work: workers stopped or inline.
    void swapProfile();
    void swapToWaiting();
    bool pauseWorkers();   // lifecycle mutex held; true when they were running
    void resumeWorkers(bool was_running);
    void publishBindingView();
    void noteEvent(MonotonicTime at, std::string text);

    void resetStages();
    bool startWorkers(std::string& err);   // lifecycle mutex held
    void requestStop();
    void estimationWorker();
    void fieldWorker();
    void publishSourceHealth(MonotonicTime now);
    void publishDetectionFrames(const SensorMap& sensors, const FieldSnapshot& snapshot,
                                MonotonicTime now);

    double loop_rate_hz_ = 100.0;

    std::string configuration_id_     = "inline";
    std::string configuration_name_;
    uint64_t    configuration_digest_ = 0;
    std::string session_id_;

    InspectionConfig inspection_;

    // destruction order: declared first, destroyed last
    ResourceStore            resources_;
    std::vector<std::string> warnings_;

    // Brain profile: fixed at build
    bool                         profiled_ = false;
    BrainProfileConfig           profile_config_;
    FunctionRegistry             profile_functions_;   // factories for candidates
    SensorCatalog                base_catalog_;        // configured sensors only
    std::unique_ptr<ProfileHost> profile_host_;

    ResourceExecutor        execute_resources_;
    SensorExecutor          execute_sensors_;
    LocalizationExecutor    execute_localization_;
    WorldEstimationExecutor execute_world_;   // field worker only

    std::unique_ptr<Commands>         commands_;
    std::unique_ptr<TargetResolution> target_resolution_;
    std::unique_ptr<Publishing>       publishing_;
    std::string                       slot_labels_[3];   // commands, targets, publishing
    FunctionKey                       commands_type_;

    std::shared_ptr<RobotStateFeed> feed_;   // the one feed, kept across profile boundaries

    // estimation worker state
    std::atomic<uint64_t>    cycle_{0};
    RobotState               robot_;
    CommandState             command_;
    TargetState              target_;
    Diagnostics              diagnostics_;
    MonotonicTime            last_now_;              // host clock of the latest cycle
    uint64_t                 placement_floor_ = 0;   // init_sequence withdrawn up to here
    StationaryPrecheck       precheck_;
    std::vector<std::string> profile_warnings_;

    // Pico link and recovery, estimation worker
    std::shared_ptr<PicoControl> pico_;   // <Pico resource_id> on the CommandCollection
    PicoOperation     pico_op_;
    SensorLossMonitor sensor_loss_;           // the running profile's used sources
    bool              loss_warned_ = false;   // a level loss was already noted
    bool              sensor_lost_ = false;   // this cycle's watchSensors found a loss
    uint64_t          models_dropped_ = 0;    // dropped intervals seen so far
    struct RecoverySeen {
        bool     pico_known = false;
        bool     pico_fresh = false;
        uint64_t reboots = 0, restarts = 0, imu_restarts = 0;
        uint32_t session           = 0;
        bool     brain_active      = false;
        MonotonicTime last_request;
        std::vector<StillnessStatus> stillness;   // per observation function
    };
    RecoverySeen seen_;

    // a prepared profile, handed from the estimation worker to the boundary
    mutable std::mutex                    profile_mutex_;
    std::unique_ptr<ProfileCandidate>     candidate_;
    std::shared_ptr<const ProfileBinding> applied_;
    std::atomic<bool>                     profile_pending_{false};
    uint64_t                              profile_applies_ = 0;

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
    std::shared_ptr<const BindingView>                                binding_view_;
    std::vector<RuntimeEvent>                                         events_;
    uint64_t                                                          event_count_ = 0;

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
