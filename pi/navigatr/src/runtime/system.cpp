// system.cpp

#include "runtime/system.h"

#include <chrono>
#include <cstdio>
#include <random>
#include <set>
#include <stdexcept>

#include "config/composition.h"
#include "config/config_node.h"
#include "core/host_clock.h"
#include "tinyxml2/tinyxml2.h"

namespace navigatr
{

namespace
{

// Carries a factory signature type into the generic slot builder.
template <typename T>
struct Tag {
    using type = T;
};

// Only the listed children may appear, each at most once for singular ones.
bool checkChildren(const ConfigNode& parent, const std::set<std::string>& allowed,
                   const std::set<std::string>& repeatable, std::string& err) {
    std::set<std::string> seen;
    for (ConfigNode c = parent.child(); c.valid(); c = c.next()) {
        const std::string name = c.name();
        if (allowed.find(name) == allowed.end()) {
            err = parent.path() + " has unknown element " + name;
            return false;
        }
        if (repeatable.find(name) == repeatable.end() && !seen.insert(name).second) {
            err = parent.path() + " has more than one " + name;
            return false;
        }
    }
    return true;
}

std::string newSessionId() {
    std::random_device            rd;
    std::mt19937_64               gen(rd() ^ static_cast<uint64_t>(
                              std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<uint64_t> dist;
    char                                    buf[24];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(dist(gen)));
    return buf;
}

double elapsedMs(std::chrono::steady_clock::time_point since) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - since)
        .count();
}

} // namespace

std::unique_ptr<System> System::buildFromFile(const std::string& path,
                                              const FunctionRegistry& functions,
                                              std::string& err, const BuildOptions& options) {
    ResolvedConfiguration resolved;
    if (!resolveConfiguration(path, resolved, err)) {
        return nullptr;
    }
    auto system = buildFromString(resolved.xml.c_str(), functions, err, options);
    if (system != nullptr) {
        system->configuration_id_     = resolved.id;
        system->configuration_name_   = resolved.name;
        system->configuration_digest_ = resolved.digest;
    }
    return system;
}

std::unique_ptr<System> System::buildFromString(const char*             xml,
                                                const FunctionRegistry& functions,
                                                std::string&            err,
                                                const BuildOptions&     options) {
    std::unique_ptr<System> system(new System());
    if (!system->build(xml, functions, options, err)) {
        return nullptr;   // atomic: the partial candidate dies here
    }
    return system;
}

System::~System() { stop(); }

bool System::build(const char* xml, const FunctionRegistry& functions,
                   const BuildOptions& options, std::string& err) {
    session_id_ = newSessionId();

    tinyxml2::XMLDocument doc;
    if (doc.Parse(xml) != tinyxml2::XML_SUCCESS) {
        err = std::string("cannot parse xml: ") + doc.ErrorStr();
        return false;
    }
    const tinyxml2::XMLElement* root_e = doc.FirstChildElement("System");
    if (root_e == nullptr) {
        err = "root element must be System";
        return false;
    }
    if (!requireResolvedSystem(*root_e, err)) {
        return false;
    }
    const ConfigNode root{root_e};

    if (!checkChildren(root, {"Loop", "Resources", "Sensors", "Pipeline", "Inspection"}, {},
                       err)) {
        return false;
    }

    const ConfigNode loop = root.child("Loop");
    if (loop.valid()) {
        if (!loop.getDouble("rate_hz", loop_rate_hz_, loop_rate_hz_, err)) {
            return false;
        }
        if (loop_rate_hz_ <= 0.0) {
            err = loop.path() + ": rate_hz must be positive";
            return false;
        }
    }
    if (!parseInspectionConfig(root.child("Inspection"), inspection_, err)) {
        return false;
    }

    // Aggregate stages: each builder visits its declarations once and
    // returns one captured executor. Node views die with doc; everything an
    // executor keeps was copied by its factory.
    std::optional<ResourceBuild> resources =
        make_resources(root.child("Resources"), functions, options, &warnings_, err);
    if (!resources.has_value()) {
        return false;
    }
    resources_         = std::move(resources->store);
    execute_resources_ = std::move(resources->execute);

    std::optional<SensorExecutor> sensors =
        make_sensors(root.child("Sensors"), functions, resources_,
                     execute_resources_.outputs(), &warnings_, err);
    if (!sensors.has_value()) {
        return false;
    }
    execute_sensors_ = std::move(*sensors);

    const ConfigNode pipeline = root.child("Pipeline");
    if (!pipeline.valid()) {
        err = root.path() + ": missing Pipeline section";
        return false;
    }
    if (!checkChildren(pipeline,
                       {"CommandCollection", "Localization", "WorldEstimation",
                        "TargetResolution", "Publishing"},
                       {}, err)) {
        return false;
    }

    SlotInitializationContext slot_context;
    slot_context.resources = &resources_;
    slot_context.sensors   = &execute_sensors_.outputs();
    slot_context.functions = &functions;
    slot_context.warnings  = &warnings_;

    // Every slot must be present exactly once and explicitly typed. An
    // intentionally unused slot selects its category noop; omission is an
    // error, never an implicit default.
    const auto slotNode = [&](const char* slot_name, ConfigNode& out_node,
                              FunctionKey& out_type) {
        const ConfigNode node = pipeline.child(slot_name);
        if (!node.valid()) {
            err = pipeline.path() + " is missing " + slot_name +
                  "; select an implementation or the category noop";
            return false;
        }
        out_type = FunctionKey{node.attr("type")};
        if (out_type.empty()) {
            err = node.path() + ": needs an explicit type";
            return false;
        }
        out_node = node;
        return true;
    };

    const auto buildSlot = [&](const char* slot_name, auto& target, auto make_tag,
                               auto& context, std::string& label) {
        using MakeFunction = typename decltype(make_tag)::type;
        ConfigNode  node;
        FunctionKey type;
        if (!slotNode(slot_name, node, type)) {
            return false;
        }
        const MakeFunction* factory = functions.find<MakeFunction>(type, err);
        if (factory == nullptr) {
            err = node.path() + ": " + err;
            return false;
        }
        label  = std::string(slot_name) + "/" + type.value;
        target = (*factory)(node, context, err);
        return target != nullptr;
    };

    if (!buildSlot("CommandCollection", commands_, Tag<CommandsMakeFunction>{},
                   slot_context, slot_labels_[0])) {
        return false;
    }

    // Localization is an aggregate stage, not a typed slot: its observation
    // functions and estimator are selected inside it.
    const ConfigNode localization_node = pipeline.child("Localization");
    if (!localization_node.valid()) {
        err = pipeline.path() + " is missing Localization";
        return false;
    }
    std::optional<LocalizationExecutor> localization =
        make_localization(localization_node, functions, execute_sensors_.outputs(),
                          resources_, &warnings_, err);
    if (!localization.has_value()) {
        return false;
    }
    execute_localization_ = std::move(*localization);
    robot_                = execute_localization_.state();
    for (const ObservationFunctionStatus& f : execute_localization_.functionStatus()) {
        slot_context.observation_functions.push_back(f.id);
    }
    slot_context.robot_observations = execute_localization_.observationOutputs();

    // World estimation is an aggregate stage too: the estimator is selected
    // inside it and owns whatever subpipeline it runs.
    const ConfigNode world_node = pipeline.child("WorldEstimation");
    if (!world_node.valid()) {
        err = pipeline.path() + " is missing WorldEstimation; configure one Estimator, an "
              "implementation or the noop";
        return false;
    }
    std::optional<WorldEstimationExecutor> world = make_world_estimation(
        world_node, functions, execute_sensors_.outputs(), resources_, &warnings_, err);
    if (!world.has_value()) {
        return false;
    }
    execute_world_ = std::move(*world);
    // Later slots reference only what the estimator declares it publishes
    // across the boundary, never an implementation detail.
    slot_context.observations = execute_world_.observationOutputs();
    slot_context.associations = execute_world_.associationOutputs();

    if (!buildSlot("TargetResolution", target_resolution_,
                   Tag<TargetResolutionMakeFunction>{}, slot_context, slot_labels_[1])) {
        return false;
    }
    if (!buildSlot("Publishing", publishing_, Tag<PublishingMakeFunction>{}, slot_context,
                   slot_labels_[2])) {
        return false;
    }

    estimation_stats_.setPeriodTarget(1000.0 / loop_rate_hz_);
    return true;
}

LocalizationRequests System::requestsFrom(const CommandState& command) const {
    LocalizationRequests requests;
    if (command.init_sequence != 0) {
        requests.placement.requested = true;
        requests.placement.origin    = "command";
        requests.placement.sequence  = command.init_sequence;
        requests.placement.pose      = command.init_pose;
    }
    return requests;
}

// ---- snapshot readers --------------------------------------------------

std::shared_ptr<const FieldSnapshot> System::fieldSnapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return field_;
}

std::shared_ptr<const ReportingSnapshot> System::reportingSnapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return reporting_;
}

std::shared_ptr<const SourceHealthSnapshot> System::sourceHealth() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return source_health_;
}

std::shared_ptr<const DiagnosticsSnapshot> System::diagnosticsSnapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return diagnostics_snapshot_;
}

std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>>
System::detectionFrames() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return detection_frames_;
}

// ---- cycles --------------------------------------------------------------

void System::step(MonotonicTime now) {
    estimationCycle(now);
    // the inline cycle lets the same cycle's evidence reach target
    // resolution and publishing, as the synchronous tests expect
    fieldCycle(execute_sensors_.retained(), now);
    reportingCycle(now);
}

void System::estimationCycle(MonotonicTime now) {
    const uint64_t cycle = cycle_.fetch_add(1) + 1;
    ++diagnostics_.cycles;
    const ExecutionContext context{now, cycle, &diagnostics_};

    const ResourceMap& resource_map = execute_resources_(context);
    const SensorMap&   sensor_map   = execute_sensors_(resource_map, context);

    CommandsOutput commands_out = commands_->run({command_, now, &diagnostics_});
    command_                    = commands_out.command;
    diagnostics_.note(slot_labels_[0], commands_out.status);

    robot_ = execute_localization_(sensor_map, requestsFrom(command_), context);

    publishSourceHealth(now);
}

void System::reportingCycle(MonotonicTime now) {
    std::shared_ptr<const FieldSnapshot> field  = fieldSnapshot();
    const LocalizationStatus             status = execute_localization_.feed()->status();
    TargetResolutionOutput               target_out = target_resolution_->run(
        {command_, robot_, *execute_localization_.feed(), field->field, field->observations,
         field->associations, target_, now});
    target_ = target_out.target;
    diagnostics_.note(slot_labels_[1], target_out.status);

    PublishingOutput pub_out = publishing_->run(
        {execute_sensors_.retained(), field->observations, field->associations, robot_,
         status, field->field, command_, target_, now});
    diagnostics_.note(slot_labels_[2], pub_out.status);

    auto reporting     = std::make_shared<ReportingSnapshot>();
    reporting->command = command_;
    reporting->target  = target_;
    reporting->at      = now;
    reporting->cycle   = cycle_.load();
    auto diagnostics   = std::make_shared<DiagnosticsSnapshot>();
    diagnostics->estimation = diagnostics_;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        reporting_ = std::move(reporting);
        if (diagnostics_snapshot_ != nullptr) {
            diagnostics->field = diagnostics_snapshot_->field;
        }
        diagnostics_snapshot_ = std::move(diagnostics);
    }
}

void System::fieldCycle(const SensorMap& sensors, MonotonicTime now) {
    std::shared_ptr<const FieldSnapshot> previous = fieldSnapshot();
    auto                                 next     = std::make_shared<FieldSnapshot>();
    const RobotState                     robot    = execute_localization_.feed()->latest();
    ++field_diagnostics_.cycles;
    const uint64_t         invocation = ++field_invocations_;
    const ExecutionContext context{now, invocation, &field_diagnostics_};

    // One standard call: the whole sensor snapshot, the latest robot state,
    // history lookups, the previous field and the context; the executor
    // owns declared-output enforcement and diagnostics.
    FieldEstimationOutput field_out = execute_world_(
        {sensors, robot, *execute_localization_.feed(), previous->field, context});

    next->field        = std::move(field_out.field);
    next->observations = std::move(field_out.observations);
    next->associations = std::move(field_out.associations);
    next->at           = now;
    next->invocation   = invocation;
    next->status       = field_out.status;
    next->diagnostic   = field_out.diagnostic;

    publishDetectionFrames(sensors, *next, now);

    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        field_ = next;
        auto diagnostics = std::make_shared<DiagnosticsSnapshot>();
        if (diagnostics_snapshot_ != nullptr) {
            diagnostics->estimation = diagnostics_snapshot_->estimation;
        }
        diagnostics->field    = field_diagnostics_;
        diagnostics_snapshot_ = std::move(diagnostics);
    }
}

void System::publishSourceHealth(MonotonicTime now) {
    auto snapshot   = std::make_shared<SourceHealthSnapshot>();
    snapshot->at    = now;
    snapshot->cycle = cycle_.load();
    const auto fill = [](SourceHealthEntry& e, const MeasurementRecord& r) {
        e.state        = r.state;
        e.diagnostic   = r.diagnostic;
        e.lastPolledAt = r.lastPolledAt;
        e.epoch        = r.epoch;
        if (r.latest.has_value()) {
            e.has_sample        = true;
            e.payload           = r.latest->payload.stableTypeName();
            e.measuredAt        = r.latest->measuredAt;
            e.receivedAt        = r.latest->receivedAt;
            e.sequence          = r.latest->sequence;
            e.epoch             = r.latest->epoch;
            e.upstream_source   = r.latest->upstream.source;
            e.upstream_clock    = r.latest->upstream.clock;
            e.upstream_sequence = r.latest->upstream.sequence;
            e.upstream_epoch    = r.latest->upstream.epoch;
        }
    };
    for (const auto& kv : execute_resources_.retained()) {
        if (kv.second.outputs.empty()) {
            SourceHealthEntry e;
            e.kind         = "resource";
            e.id           = kv.first.value;
            e.state        = kv.second.state;
            e.diagnostic   = kv.second.diagnostic;
            e.lastPolledAt = kv.second.lastPolledAt;
            snapshot->entries.push_back(std::move(e));
            continue;
        }
        for (const auto& out : kv.second.outputs) {
            SourceHealthEntry e;
            e.kind = "resource";
            e.id   = kv.first.value + "." + out.first.value;
            fill(e, out.second);
            if (kv.second.state != SourceState::kValid && e.diagnostic.empty()) {
                e.diagnostic = kv.second.diagnostic;
            }
            snapshot->entries.push_back(std::move(e));
        }
    }
    for (const auto& kv : execute_sensors_.retained()) {
        SourceHealthEntry e;
        e.kind = "sensor";
        e.id   = kv.first.value;
        fill(e, kv.second);
        snapshot->entries.push_back(std::move(e));
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    source_health_ = std::move(snapshot);
}

void System::publishDetectionFrames(const SensorMap& sensors, const FieldSnapshot& snapshot,
                                    MonotonicTime now) {
    // Every tag observation set published this invocation is bound to the
    // exact frame it was decoded from: the pixels come from the sensor
    // record whose identity matches, never from "the latest image".
    std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>> produced;
    for (const auto& kv : snapshot.observations) {
        const TagObservationSet* set = kv.second.payload.get<TagObservationSet>();
        if (set == nullptr) {
            continue;
        }
        const auto sensor_it = sensors.find(set->camera);
        if (sensor_it == sensors.end() || !sensor_it->second.latest.has_value()) {
            continue;
        }
        const CameraFramePayload* frame =
            sensor_it->second.latest->payload.get<CameraFramePayload>();
        if (frame == nullptr || frame->frame.epoch != set->frame_epoch ||
            frame->frame.sequence != set->frame_sequence) {
            continue;   // the retained frame is not the one these came from
        }
        auto d                     = std::make_shared<DetectionFrameSnapshot>();
        d->camera                  = set->camera;
        d->engineering_frame       = frame->engineering_frame;
        d->frame_epoch             = set->frame_epoch;
        d->frame_sequence          = set->frame_sequence;
        d->exposureAt              = frame->frame.exposureAt;
        d->receivedAt              = frame->frame.receivedAt;
        d->processedAt             = running_.load() ? HostClock::now() : now;
        d->exposure_uncertainty_ms = frame->frame.exposure_uncertainty_ms;
        d->exposure_time_reliable  = frame->frame.exposure_time_reliable;
        d->width_px                = frame->frame.width_px;
        d->height_px               = frame->frame.height_px;
        d->y8                      = frame->frame.y8;
        d->intrinsics              = frame->intrinsics;
        d->has_observations        = true;
        d->observations            = *set;
        for (const auto& akv : snapshot.associations) {
            const TagAssociationTraceSet* trace =
                akv.second.payload.get<TagAssociationTraceSet>();
            if (trace != nullptr && trace->camera == set->camera &&
                trace->frame_epoch == set->frame_epoch &&
                trace->frame_sequence == set->frame_sequence) {
                d->has_trace = true;
                d->trace     = *trace;
                break;
            }
        }
        if (d->has_trace && d->trace.has_exposure_context) {
            d->pose_at_exposure     = d->trace.exposure_context.pose;
            d->attitude_at_exposure = d->trace.exposure_context.attitude;
            d->field_from_odom      = d->trace.field_from_odom;
            d->anchor_revision      = d->trace.anchor_revision;
        } else {
            const auto exposure = execute_localization_.feed()->sampleSnapshotAt(set->exposureAt);
            d->pose_at_exposure     = exposure.sample.pose;
            d->attitude_at_exposure = exposure.sample.attitude;
            d->field_from_odom      = exposure.current.robot.field_from_odom;
            d->anchor_revision      = exposure.current.robot.anchor_revision;
        }
        d->field_invocation     = snapshot.invocation;
        produced[set->camera]   = std::move(d);
    }
    if (produced.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    for (auto& kv : produced) {
        detection_frames_[kv.first] = std::move(kv.second);
    }
}

// ---- lifecycle -----------------------------------------------------------

void System::resetStages() {
    execute_resources_.reset();   // shared resources reset once, not per consumer
    execute_sensors_.reset();
    execute_localization_.reset();
    execute_world_.reset();
    commands_->reset();
    target_resolution_->reset();
    publishing_->reset();

    robot_   = execute_localization_.state();
    command_ = CommandState{};
    target_  = TargetState{};
    field_invocations_ = 0;
    field_handoff_.clear();

    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    field_ = std::make_shared<FieldSnapshot>();
    detection_frames_.clear();
    reporting_.reset();
    source_health_.reset();
}

void System::reset() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    const bool                  was_running = running_.load();
    if (was_running) {
        // join both workers so exactly one thread touches every stage
        requestStop();
        if (estimation_thread_.joinable()) {
            estimation_thread_.join();
        }
        if (field_thread_.joinable()) {
            field_thread_.join();
        }
        running_.store(false);
    }
    estimation_stats_.setRunning(false);
    field_stats_.setRunning(false);
    resetStages();
    reset_count_.fetch_add(1);
    if (was_running) {
        estimation_stats_.resetCounters();
        field_stats_.resetCounters();
        std::string err;
        if (!startWorkers(err)) {
            throw std::runtime_error(err);
        }
    }
}

bool System::start(std::string& err) {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    return startWorkers(err);
}

bool System::startWorkers(std::string& err) {
    if (running_.load()) {
        return true;
    }
    err.clear();
    stop_.store(false);
    field_handoff_.resume();
    running_.store(true);
    estimation_stats_.setRunning(true);
    field_stats_.setRunning(true);
    try {
        field_thread_      = std::thread([this] { fieldWorker(); });
        estimation_thread_ = std::thread([this] { estimationWorker(); });
    } catch (const std::exception& e) {
        err = std::string("cannot start workers: ") + e.what();
        requestStop();
        if (field_thread_.joinable()) {
            field_thread_.join();
        }
        if (estimation_thread_.joinable()) {
            estimation_thread_.join();
        }
        running_.store(false);
        estimation_stats_.setRunning(false);
        field_stats_.setRunning(false);
        return false;
    }
    return true;
}

void System::requestStop() {
    {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        stop_.store(true);
    }
    wake_.notify_all();
    field_handoff_.stop();
}

void System::stop() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    if (!running_.load()) {
        return;
    }
    // Wake both workers, then join before any stage can be destroyed.
    requestStop();
    if (estimation_thread_.joinable()) {
        estimation_thread_.join();
    }
    if (field_thread_.joinable()) {
        field_thread_.join();
    }
    running_.store(false);
    estimation_stats_.setRunning(false);
    field_stats_.setRunning(false);
}

void System::estimationWorker() {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / loop_rate_hz_));
    auto next = std::chrono::steady_clock::now();
    while (!stop_.load()) {
        const auto          started = std::chrono::steady_clock::now();
        const MonotonicTime now     = HostClock::now();
        estimationCycle(now);
        // the field worker gets its own immutable copy: payloads are shared
        // immutable values, so the copy is the envelopes only
        field_handoff_.put(FieldWork{
            std::make_shared<const SensorMap>(execute_sensors_.retained()), now});
        reportingCycle(now);
        estimation_stats_.cycleDone(elapsedMs(started), now.ms, 0, 0);

        next += period;
        const auto after = std::chrono::steady_clock::now();
        if (next < after - period) {
            next = after;   // fell behind by more than a period: resync, do not burst
        }
        std::unique_lock<std::mutex> lock(wake_mutex_);
        wake_.wait_until(lock, next, [this] { return stop_.load(); });
    }
}

void System::fieldWorker() {
    while (!stop_.load()) {
        FieldWork work;
        if (!field_handoff_.waitTake(work, std::chrono::milliseconds(100))) {
            continue;
        }
        const auto started = std::chrono::steady_clock::now();
        fieldCycle(*work.sensors, work.now);
        field_stats_.cycleDone(elapsedMs(started), HostClock::now().ms,
                               field_handoff_.replaced(),
                               field_handoff_.pending() ? 1 : 0);
    }
}

} // namespace navigatr
