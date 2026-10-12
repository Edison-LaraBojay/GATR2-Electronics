// system.cpp

#include "runtime/system.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <set>
#include <stdexcept>

#include "config/composition.h"
#include "config/config_node.h"
#include "core/host_clock.h"
#include "payloads/pico_telemetry_samples.h"
#include "payloads/sensor_samples.h"
#include "runtime/pico_control_ref.h"
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

constexpr std::size_t kEventsKept = 32;

std::string hexId(uint32_t id) {
    char buf[12];
    std::snprintf(buf, sizeof(buf), "%08x", static_cast<unsigned>(id));
    return buf;
}

} // namespace

// The System as the brain_link commands slot sees it.
class System::ProfileHost : public BrainProfileHost
{
public:
    explicit ProfileHost(System& system) : system_(system) {}

    bool prepare(const translagatr::RobotProfileDoc& profile, uint32_t profile_id, uint8_t& reason,
                 uint8_t& detail) override {
        return system_.prepareProfile(profile, profile_id, reason, detail);
    }

    std::shared_ptr<const ProfileBinding> applied() const override {
        return system_.profileBinding();
    }

    uint8_t control(uint8_t action, uint8_t arg, MonotonicTime now, uint8_t& detail) override {
        return system_.controlProfile(action, arg, now, detail);
    }

    uint8_t controlProgress(uint8_t action, uint8_t, MonotonicTime now,
                            uint8_t& detail) override {
        return system_.controlProgress(action, now, detail);
    }

    uint8_t readWheels(MonotonicTime now, uint8_t& count, translagatr::WheelReading* wheels) override {
        return system_.readWheels(now, count, wheels);
    }

private:
    System& system_;
};

System::System() = default;

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

System::~System() {
    // the recorder first: it closes a running capture while every source of
    // its metadata is still alive
    if (capture_ != nullptr) {
        capture_->stop();
    }
    stop();
}

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

    if (!checkChildren(root, {"Loop", "Resources", "Sensors", "Pipeline", "Inspection", "Capture"},
                       {}, err)) {
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
    if (!parseInspectionConfig(root.child("Inspection"), inspection_, err) ||
        !parseCaptureConfig(root.child("Capture"), capture_config_, err)) {
        return false;
    }
    event_source_ = diag_hub_->sourceId("system");

    // Aggregate stages: each builder visits its declarations once and
    // returns one captured executor. Node views die with doc; everything an
    // executor keeps was copied by its factory.
    BuildOptions resource_options = options;
    resource_options.diagnostics  = diag_hub_.get();
    std::optional<ResourceBuild> resources =
        make_resources(root.child("Resources"), functions, resource_options, &warnings_, err);
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
    slot_context.resources    = &resources_;
    slot_context.sensors      = &execute_sensors_.outputs();
    slot_context.functions    = &functions;
    slot_context.warnings     = &warnings_;
    slot_context.loop_rate_hz = loop_rate_hz_;
    slot_context.diagnostics  = diag_hub_.get();

    // A Brain-profiled Localization: the host exists before Command
    // Collection, which hands it every validated profile.
    const ConfigNode profile_node = pipeline.child("Localization").child("BrainProfile");
    if (profile_node.valid()) {
        if (!buildProfileHost(pipeline, profile_node, functions, err)) {
            return false;
        }
        slot_context.brain_profile = profile_host_.get();
    }

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
                               auto& context, std::string& label, FunctionKey& type) {
        using MakeFunction = typename decltype(make_tag)::type;
        ConfigNode node;
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
                   slot_context, slot_labels_[0], commands_type_)) {
        return false;
    }
    // a publisher answering this collector must read the same link
    slot_context.commands_type = commands_type_;
    slot_context.commands_serial =
        ResourceId{pipeline.child("CommandCollection").child("Serial").attr("resource_id")};

    // Localization is an aggregate stage, not a typed slot: its observation
    // functions and estimator are selected inside it.
    const ConfigNode localization_node = pipeline.child("Localization");
    if (!localization_node.valid()) {
        err = pipeline.path() + " is missing Localization";
        return false;
    }
    std::optional<LocalizationExecutor> localization =
        profiled_ ? buildWaitingLocalization(err)
                  : make_localization(localization_node, functions, execute_sensors_.outputs(),
                                      resources_, &warnings_, err);
    if (!localization.has_value()) {
        return false;
    }
    execute_localization_ = std::move(*localization);
    robot_                = execute_localization_.state();
    feed_                 = execute_localization_.feed();
    // the robot state tap; the feed outlives profile boundaries
    feed_->setDiagnostics(diag_hub_);
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

    FunctionKey target_type;
    if (!buildSlot("TargetResolution", target_resolution_,
                   Tag<TargetResolutionMakeFunction>{}, slot_context, slot_labels_[1],
                   target_type)) {
        return false;
    }
    FunctionKey publishing_type;
    if (!buildSlot("Publishing", publishing_, Tag<PublishingMakeFunction>{}, slot_context,
                   slot_labels_[2], publishing_type)) {
        return false;
    }
    if (commands_type_.value == "brain_link" && publishing_type.value != "brain_link") {
        err = pipeline.path() + ": brain_link CommandCollection needs brain_link Publishing "
                                "to answer its requests";
        return false;
    }

    estimation_stats_.setPeriodTarget(1000.0 / loop_rate_hz_);
    publishBindingView();

    // after the pipeline: every producer already holds the hub
    capture_ = makeCaptureRecorder(*this, capture_config_);
    return capture_ != nullptr;
}

bool System::buildProfileHost(const ConfigNode& pipeline, const ConfigNode& profile,
                              const FunctionRegistry& functions, std::string& err) {
    const ConfigNode localization = pipeline.child("Localization");
    if (!localization.onlyChildren({"BrainProfile"}, err) ||
        !localization.atMostOne("BrainProfile", err)) {
        err += "; a Brain-profiled Localization holds only BrainProfile, the models come from "
               "the Brain robot profile";
        return false;
    }
    const ConfigNode commands = pipeline.child("CommandCollection");
    if (commands.attr("type") != "brain_link") {
        err = profile.path() + ": needs a brain_link CommandCollection; the profile arrives "
                               "over the Brain link";
        return false;
    }
    if (!parseBrainProfileConfig(profile, resources_, execute_resources_.outputs(),
                                 profile_config_, err)) {
        return false;
    }
    if (!parsePicoReference(commands, resources_, pico_, err)) {
        return false;
    }
    const std::string bench = commands.child("BenchImu").attr("resource_id");
    if (!profile_config_.brain_imu.empty() && bench != profile_config_.brain_imu.value) {
        err = profile.path() + ": BrainImu " + profile_config_.brain_imu.value +
              " needs <BenchImu resource_id=\"" + profile_config_.brain_imu.value +
              "\"/> on the brain_link CommandCollection";
        return false;
    }
    const std::string prefix = kProfileSensorPrefix;
    for (const SensorCatalog::Entry& e : execute_sensors_.outputs().entries()) {
        if (e.id.value.compare(0, prefix.size(), prefix) == 0) {
            err = profile.path() + ": sensor id " + e.id.value + " uses the prefix " + prefix +
                  " reserved for sensors built from the Brain profile";
            return false;
        }
    }
    profiled_          = true;
    profile_functions_ = functions;
    base_catalog_      = execute_sensors_.outputs();
    profile_host_      = std::make_unique<ProfileHost>(*this);
    return true;
}

std::optional<LocalizationExecutor> System::buildWaitingLocalization(std::string& err) {
    tinyxml2::XMLDocument doc;
    writeWaitingLocalization(profile_config_, doc);
    return make_localization(ConfigNode{doc.RootElement()}, profile_functions_, base_catalog_,
                             resources_, &warnings_, err);
}

LocalizationRequests System::requestsFrom(const CommandState& command) const {
    LocalizationRequests requests;
    // A new brain session withdraws a placement not applied yet; an applied
    // one stays recorded in the robot state and never re-applies. A profile
    // boundary or a reinitialize withdraws every request up to its floor.
    if (command.init_sequence > placement_floor_ && command.init_session == command.session) {
        requests.placement.requested = true;
        requests.placement.origin    = "command";
        requests.placement.session   = command.init_session;
        requests.placement.sequence  = command.init_sequence;
        requests.placement.pose      = command.init_pose;
    }
    return requests;
}

// ---- Brain profile ---------------------------------------------------------

bool System::prepareProfile(const translagatr::RobotProfileDoc& profile, uint32_t id, uint8_t& reason,
                            uint8_t& detail) {
    if (!checkProfileCapabilities(profile_config_, profile, reason, detail)) {
        noteEvent(last_now_, "profile " + hexId(id) + " refused: " + profileReasonName(reason) +
                                 " (index " + std::to_string(detail) + ")");
        return false;
    }
    auto binding     = std::make_shared<ProfileBinding>();
    binding->id      = id;
    binding->profile = profile;
    tinyxml2::XMLDocument doc;
    writeProfileSubtrees(profile_config_, profile, doc, *binding);
    const ConfigNode root{doc.RootElement()};

    // the ordinary factories, from typed data; nothing running is touched
    std::string                         err;
    std::vector<std::string>            warnings;
    std::optional<LocalizationExecutor> localization;
    std::optional<SensorExecutor>       sensors =
        make_sensors(root.child("Sensors"), profile_functions_, resources_,
                     execute_resources_.outputs(), &warnings, err);
    if (sensors.has_value()) {
        SensorCatalog catalog = base_catalog_;
        for (const SensorCatalog::Entry& e : sensors->outputs().entries()) {
            catalog.add(e.id, e.payload);
        }
        localization = make_localization(root.child("Localization"), profile_functions_, catalog,
                                         resources_, &warnings, err);
    }
    if (!profile_config_.brain_imu.empty() && profile.imu_source == translagatr::kImuSourceBrainVex) {
        binding->bench_imu = resources_.require<BrainImuBench>(profile_config_.brain_imu, err);
    }
    if (!sensors.has_value() || !localization.has_value() ||
        (profile.imu_source == translagatr::kImuSourceBrainVex && binding->bench_imu == nullptr)) {
        reason = translagatr::kProfileReasonBuild;
        detail = 0;
        noteEvent(last_now_, "profile " + hexId(id) + " not built: " + err);
        return false;
    }

    auto candidate          = std::make_unique<ProfileCandidate>();
    candidate->binding      = std::move(binding);
    candidate->sensors      = std::move(*sensors);
    candidate->localization = std::move(*localization);
    candidate->warnings     = std::move(warnings);
    {
        std::lock_guard<std::mutex> lock(profile_mutex_);
        candidate_ = std::move(candidate);
        profile_pending_.store(true);
    }
    noteEvent(last_now_, "profile " + hexId(id) + " built; applies at the next boundary");
    return true;
}

uint8_t System::controlProfile(uint8_t action, uint8_t, MonotonicTime now, uint8_t& detail) {
    detail = translagatr::kControlDetailNone;
    const std::shared_ptr<const ProfileBinding> binding = profileBinding();
    if (binding == nullptr) {
        return translagatr::kResultNotReady;
    }
    const auto stationary = [&](const char* name) {
        std::string why;
        if (precheck_.still(now, &why)) {
            return true;
        }
        noteEvent(now, std::string(name) + " refused, not stationary: " + why);
        return false;
    };
    switch (action) {
    case translagatr::kControlRecalibrate: {
        if (!stationary("recalibrate")) {
            return translagatr::kResultNotStationary;
        }
        const bool any = execute_localization_.recalibrate();
        noteEvent(now, any ? "IMU bias recalibration started; the pose holds while the robot "
                             "stays still"
                           : "recalibrate: nothing to calibrate on the Pi for this profile");
        return translagatr::kResultOk;
    }
    case translagatr::kControlReinitialize:
        if (!stationary("reinitialize")) {
            return translagatr::kResultNotStationary;
        }
        execute_localization_.reset();
        placement_floor_ = command_.init_sequence;
        noteEvent(now, "localization reinitialized: odometry epoch " +
                           std::to_string(execute_localization_.state().odometry_epoch) +
                           ", placement withdrawn, bias recalibrating");
        return translagatr::kResultOk;
    case translagatr::kControlReinitImu: {
        if (binding->profile.imu_source != translagatr::kImuSourcePico) {
            detail = translagatr::kControlDetailImuUnused;
            noteEvent(now, "IMU reinitialization refused: the profile does not use the Pico IMU");
            return translagatr::kResultFailed;
        }
        const uint32_t handle =
            submitPico(translagatr::kPicoOpReinitImu, binding->profile.imu_port, now, detail);
        if (handle == 0) {
            return translagatr::kResultFailed;
        }
        pico_op_         = PicoOperation{};
        pico_op_.active  = true;
        pico_op_.action  = action;
        pico_op_.handle  = handle;
        pico_op_.started = now;
        pico_op_.result  = translagatr::kResultPending;
        noteEvent(now, "Pico IMU reinitialization requested");
        return translagatr::kResultPending;
    }
    case translagatr::kControlRestartAcquisition: {
        if (!stationary("acquisition restart")) {
            return translagatr::kResultNotStationary;
        }
        const PicoLinkState link = pico_ != nullptr ? pico_->link() : PicoLinkState{};
        const uint32_t handle = submitPico(translagatr::kPicoOpRestartAcquisition, 0, now, detail);
        if (handle == 0) {
            return translagatr::kResultFailed;
        }
        pico_op_           = PicoOperation{};
        pico_op_.active    = true;
        pico_op_.action    = action;
        pico_op_.handle    = handle;
        pico_op_.started   = now;
        pico_op_.acq_epoch = link.acq_epoch;
        pico_op_.restarts  = link.restarts;
        pico_op_.result    = translagatr::kResultPending;
        noteEvent(now, "Pico acquisition restart requested");
        return translagatr::kResultPending;
    }
    default: return translagatr::kResultInvalidArgument;
    }
}

uint32_t System::submitPico(uint8_t op, uint8_t arg, MonotonicTime now, uint8_t& detail) {
    detail = translagatr::kControlDetailPicoLink;
    if (pico_ == nullptr) {
        noteEvent(now, "Pico command refused: no Pico link configured");
        return 0;
    }
    const PicoLinkState link = pico_->link();
    if (!link.identity || !link.frames_fresh) {
        noteEvent(now, link.frames_fresh ? "Pico command refused: the Pico firmware takes no "
                                           "commands (no v2 identity)"
                                         : "Pico command refused: no Pico frames");
        return 0;
    }
    const uint32_t handle = pico_->submit(op, arg, now, kPicoOperationMs / 1000.0);
    if (handle == 0) {
        noteEvent(now, "Pico command refused by the Pico link");
        return 0;
    }
    detail = translagatr::kControlDetailNone;
    return handle;
}

uint8_t System::controlProgress(uint8_t action, MonotonicTime now, uint8_t& detail) {
    updatePicoOperation(now);
    if (!pico_op_.active || pico_op_.action != action) {
        detail = translagatr::kControlDetailPicoLink;   // nothing is running under this request
        return translagatr::kResultFailed;
    }
    detail = pico_op_.detail;
    return pico_op_.result;
}

void System::updatePicoOperation(MonotonicTime now) {
    PicoOperation& op = pico_op_;
    if (!op.active || op.result != translagatr::kResultPending || pico_ == nullptr) {
        return;
    }
    const bool reinit = op.action == translagatr::kControlReinitImu;
    const auto settle = [&](uint8_t result, uint8_t detail, const std::string& text) {
        op.result = result;
        op.detail = detail;
        noteEvent(now, text);
    };
    const PicoRequestStatus status = pico_->request(op.handle);
    const PicoLinkState     link   = pico_->link();
    switch (status.state) {
    case PicoRequestState::kFailed:
        settle(translagatr::kResultFailed,
               status.detail != translagatr::kControlDetailNone
                   ? status.detail
                   : static_cast<uint8_t>(translagatr::kControlDetailPicoRefused),
               reinit ? "Pico IMU reinitialization failed" : "Pico acquisition restart failed");
        return;
    case PicoRequestState::kUnknown:
        settle(translagatr::kResultFailed, translagatr::kControlDetailPicoLink,
               reinit ? "Pico IMU reinitialization lost with the Pico link"
                      : "Pico acquisition restart lost with the Pico link");
        return;
    case PicoRequestState::kCompleted:
        // a used source restarted: the pose needs a placement again (8.10)
        if (reinit) {
            execute_localization_.recalibrate();
            settle(translagatr::kResultOk, translagatr::kControlDetailNone,
                   "Pico IMU reinitialized; bias recalibration started");
            losePlacement(now, "sensor lost: pico_imu reinitialized", true);
            return;
        }
        if (link.acq_epoch != op.acq_epoch || link.restarts != op.restarts) {
            settle(translagatr::kResultOk, translagatr::kControlDetailNone,
                   "Pico acquisition restarted (acquisition epoch " +
                       std::to_string(link.acq_epoch) + "); encoders rebased");
            losePlacement(now, "sensor lost: Pico acquisition restarted", true);
            return;
        }
        break;   // completed on the Pico; waiting for frames of the new epoch
    case PicoRequestState::kSending:
    case PicoRequestState::kRunning: break;
    }
    if (sameDomain(now, op.started) && now - op.started > kPicoOperationMs) {
        settle(translagatr::kResultFailed, translagatr::kControlDetailTimedOut,
               reinit ? "Pico IMU reinitialization timed out"
                      : "Pico acquisition restart timed out");
    }
}

uint8_t System::readWheels(MonotonicTime now, uint8_t& count, translagatr::WheelReading* wheels) const {
    count = 0;
    const std::shared_ptr<const ProfileBinding> binding = profileBinding();
    if (binding == nullptr) {
        return translagatr::kResultNotReady;
    }
    const SensorMap&   sensors   = execute_sensors_.retained();
    const ResourceMap& resources = execute_resources_.retained();
    const auto         resource  = resources.find(profile_config_.encoder_resource);
    for (uint8_t i = 0; i < binding->profile.wheel_count && i < translagatr::kWheelReadingsMax; ++i) {
        const translagatr::ProfileWheel& w = binding->profile.wheels[i];
        translagatr::WheelReading        r;
        r.port = w.encoder_port;
        if (resource != resources.end() && w.encoder_port < kProfileEncoderPorts) {
            const auto out = resource->second.outputs.find(profile_config_.ports[w.encoder_port]);
            if (out != resource->second.outputs.end() && out->second.latest.has_value()) {
                const PicoEncoderCounts* raw =
                    out->second.latest->payload.get<PicoEncoderCounts>();
                r.counts = raw != nullptr ? raw->counts : 0;
            }
        }
        const auto it = i < binding->encoders.size() ? sensors.find(binding->encoders[i])
                                                     : sensors.end();
        if (it != sensors.end() && it->second.latest.has_value()) {
            const StoredSample&  stored = *it->second.latest;
            const EncoderSample* sample = stored.payload.get<EncoderSample>();
            if (sample != nullptr) {
                r.flags |= translagatr::kWheelValid;
                // wheel angle already carries counts per revolution, gearing
                // and polarity; the radius makes it raw travel, no travel scale
                const double um = std::round(sample->angle_rad * w.radius_um);
                r.travel_um = static_cast<int32_t>(std::clamp(um, -2147483648.0, 2147483647.0));
                r.discontinuity =
                    static_cast<uint16_t>((sample->discontinuity_epoch + stored.epoch) & 0xFFFF);
                const bool host = stored.receivedAt.domain == ClockDomain::kHost &&
                                  now.domain == ClockDomain::kHost;
                const int64_t age = host ? now - stored.receivedAt : 0;
                r.age_ms = static_cast<uint16_t>(std::clamp<int64_t>(age, 0, 65535));
                if (it->second.state == SourceState::kValid && host && age <= kWheelFreshMs) {
                    r.flags |= translagatr::kWheelFresh;
                }
            }
        }
        wheels[count++] = r;
    }
    return translagatr::kResultOk;
}

void System::watchSensors(const SensorMap& sensors, MonotonicTime now) {
    sensor_lost_ = false;
    if (!sensor_loss_.active()) {
        return;
    }
    PicoLinkState        link;
    const PicoLinkState* pico = nullptr;
    if (pico_ != nullptr) {
        link = pico_->link();
        pico = &link;
    }
    const SensorLossMonitor::Result r = sensor_loss_.update(sensors, pico, now);
    if (!r.edge.empty()) {
        losePlacement(now, "sensor lost: " + r.edge, true);
    } else if (!r.level.empty()) {
        losePlacement(now, "sensor lost: " + r.level, !loss_warned_);
    }
    loss_warned_ = !r.level.empty();
    sensor_lost_ = !r.edge.empty() || !r.level.empty();
}

void System::watchModels(MonotonicTime now) {
    const uint64_t dropped = execute_localization_.droppedIntervals();
    if (dropped > models_dropped_) {
        // a sensor loss found this cycle already explains it
        losePlacement(now, "motion lost: " + execute_localization_.lastDrop(), !sensor_lost_);
    }
    models_dropped_ = dropped;
}

void System::losePlacement(MonotonicTime now, const std::string& why, bool note) {
    if (!robot_.initialized) {
        return;   // nothing placed, nothing to lose
    }
    if (!profile_config_.unplace_on_sensor_loss) {
        if (note) {
            noteEvent(now, why + " (warn only; the pose is kept)");
        }
        return;
    }
    execute_localization_.loseContinuity(why);
    robot_           = execute_localization_.state();
    placement_floor_ = command_.init_sequence;
    noteEvent(now, why + ": place again (odometry epoch " +
                       std::to_string(robot_.odometry_epoch) + ")");
    noteFault(CaptureFault::kContinuityLost, why);
}

void System::noteRecovery(MonotonicTime now) {
    RecoverySeen& s = seen_;

    // Brain link
    if (command_.reply.pending) {
        if (!s.brain_active && s.last_request.isSet()) {
            noteEvent(now, "Brain link requests resumed");
        }
        s.brain_active = true;
        s.last_request = now;
    } else if (s.brain_active && sameDomain(now, s.last_request) &&
               now - s.last_request > 1000) {
        s.brain_active = false;
        noteEvent(now, "Brain link quiet for 1 s");
        noteFault(CaptureFault::kLinkLost, "Brain link quiet for 1 s");
    }
    if (command_.session != s.session) {
        if (command_.session != 0) {
            noteEvent(now, "Brain session opened");
        }
        s.session = command_.session;
    }

    // Pico identity
    if (pico_ != nullptr) {
        const PicoLinkState link = pico_->link();
        if (s.pico_known) {
            if (link.frames_fresh != s.pico_fresh) {
                noteEvent(now, link.frames_fresh ? "Pico frames restored" : "Pico frames lost");
                if (!link.frames_fresh) {
                    noteFault(CaptureFault::kLinkLost, "Pico frames lost");
                }
            }
            if (link.reboots != s.reboots) {
                noteEvent(now, "Pico rebooted (boot " + std::to_string(link.boot_id) +
                                   "); encoders rebased, IMU bias recalibrates");
                noteFault(CaptureFault::kPicoReboot,
                          "Pico rebooted (boot " + std::to_string(link.boot_id) + ")");
            }
            if (link.restarts != s.restarts) {
                noteEvent(now, "Pico acquisition epoch " + std::to_string(link.acq_epoch));
            }
            if (link.imu_restarts != s.imu_restarts) {
                noteEvent(now, "Pico IMU restarted (epoch " + std::to_string(link.imu_epoch) + ")");
            }
        }
        s.pico_known   = true;
        s.pico_fresh   = link.frames_fresh;
        s.reboots      = link.reboots;
        s.restarts     = link.restarts;
        s.imu_restarts = link.imu_restarts;
    }

    // calibration of every function with a window
    const std::vector<ObservationFunctionStatus> functions = feed_->status().functions;
    if (s.stillness.size() != functions.size()) {
        s.stillness.assign(functions.size(), StillnessStatus{});
        for (std::size_t i = 0; i < functions.size(); ++i) {
            s.stillness[i] = functions[i].stillness;
        }
        return;
    }
    for (std::size_t i = 0; i < functions.size(); ++i) {
        const StillnessStatus& now_s = functions[i].stillness;
        StillnessStatus&       was   = s.stillness[i];
        if (now_s.calibration != was.calibration &&
            (now_s.calibration == BiasCalibration::kDone ||
             now_s.calibration == BiasCalibration::kFailed)) {
            noteEvent(now, functions[i].id + ": IMU bias calibration " +
                               toString(now_s.calibration) +
                               (now_s.calibration == BiasCalibration::kFailed
                                    ? " (" + now_s.reason + ")"
                                    : std::string()));
        } else if (now_s.attempts != was.attempts && now_s.attempts > 1) {
            noteEvent(now, functions[i].id + ": IMU bias calibration restarted (" +
                               now_s.reason + ")");
        }
        was = now_s;
    }
}

bool System::applyPendingProfile() {
    if (!profile_pending_.load()) {
        return false;
    }
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    const bool                  was_running = pauseWorkers();
    swapProfile();
    resumeWorkers(was_running);
    return true;
}

void System::swapProfile() {
    std::unique_ptr<ProfileCandidate> c;
    {
        std::lock_guard<std::mutex> lock(profile_mutex_);
        c = std::move(candidate_);
        profile_pending_.store(false);
    }
    if (c == nullptr) {
        return;
    }
    ProfileStatus& status = command_.profile;
    if (status.state != translagatr::kProfileApplying || status.id != c->binding->id) {
        // a later APPLY (the running profile again, or a refused one) won
        noteEvent(last_now_, "profile " + hexId(c->binding->id) +
                                 " dropped: superseded before the boundary");
        return;
    }
    execute_sensors_.replaceProfile(std::move(c->sensors));
    c->localization.continueFrom(execute_localization_);
    execute_localization_ = std::move(c->localization);
    robot_                = execute_localization_.state();
    // placement requests made under the previous odometry never re-apply
    placement_floor_ = command_.init_sequence;

    status.applied_id = c->binding->id;
    status.state      = translagatr::kProfileApplied;
    status.reason     = translagatr::kProfileReasonNone;
    status.detail     = 0;
    c->binding->generation = ++profile_applies_;

    std::vector<StationaryPrecheck::Wheel> wheels;
    for (uint8_t i = 0; i < c->binding->profile.wheel_count; ++i) {
        wheels.push_back({c->binding->encoders[i], c->binding->profile.wheels[i].radius_um * 1e-6});
    }
    precheck_.configure(wheels, c->binding->imu, c->binding->bench_imu);
    std::vector<SensorLossMonitor::Wheel> used;
    for (uint8_t i = 0; i < c->binding->profile.wheel_count; ++i) {
        used.push_back({c->binding->encoders[i], c->binding->profile.wheels[i].encoder_port});
    }
    sensor_loss_.configure(used, c->binding->imu, c->binding->profile.imu_port,
                           c->binding->bench_imu, profile_config_.sensor_loss_ms, last_now_);
    loss_warned_    = false;
    sensor_lost_    = false;
    models_dropped_ = execute_localization_.droppedIntervals();
    seen_.stillness.clear();
    field_handoff_.clear();
    profile_warnings_ = std::move(c->warnings);
    {
        std::lock_guard<std::mutex> lock(profile_mutex_);
        applied_ = c->binding;
    }
    publishBindingView();
    noteEvent(last_now_, "profile " + hexId(c->binding->id) + " applied (" +
                             c->binding->summary + "); odometry epoch " +
                             std::to_string(robot_.odometry_epoch) + ", placement required");
    noteFault(CaptureFault::kProfileChanged, "profile " + hexId(c->binding->id) + " applied");
}

void System::swapToWaiting() {
    {
        std::lock_guard<std::mutex> lock(profile_mutex_);
        candidate_.reset();
        applied_.reset();
        profile_pending_.store(false);
    }
    execute_sensors_.replaceProfile(SensorExecutor{});
    std::string                         err;
    std::optional<LocalizationExecutor> waiting = buildWaitingLocalization(err);
    if (waiting.has_value()) {   // built once at startup already
        waiting->continueFrom(execute_localization_);
        execute_localization_ = std::move(*waiting);
    }
    robot_           = execute_localization_.state();
    placement_floor_ = 0;
    precheck_.configure({}, SensorId{}, nullptr);
    sensor_loss_.clear();
    models_dropped_ = execute_localization_.droppedIntervals();
    seen_.stillness.clear();
    pico_op_        = PicoOperation{};
    profile_warnings_.clear();
    publishBindingView();
}

bool System::pauseWorkers() {
    const bool was_running = running_.load();
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
    return was_running;
}

void System::resumeWorkers(bool was_running) {
    if (!was_running) {
        return;
    }
    std::string err;
    if (!startWorkers(err)) {
        throw std::runtime_error(err);
    }
}

std::shared_ptr<const ProfileBinding> System::profileBinding() const {
    std::lock_guard<std::mutex> lock(profile_mutex_);
    return applied_;
}

void System::publishBindingView() {
    auto view            = std::make_shared<BindingView>();
    view->sensors        = execute_sensors_.outputs();
    view->estimator_type = execute_localization_.estimatorType();
    view->warnings       = warnings_;
    view->warnings.insert(view->warnings.end(), profile_warnings_.begin(),
                          profile_warnings_.end());
    view->brain_profile = profiled_;
    view->profile       = profileBinding();
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    binding_view_ = std::move(view);
}

std::shared_ptr<const BindingView> System::bindingView() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return binding_view_;
}

void System::noteEvent(MonotonicTime at, std::string text) {
    // events are rare: post unconditionally, so hub.latest() has the newest
    DiagRecord record;
    record.kind   = DiagKind::kEvent;
    record.source = event_source_;
    DiagEvent e;
    const std::size_t n = std::min(text.size(), sizeof(e.text) - 1);
    std::memcpy(e.text, text.data(), n);
    e.text[n]      = '\0';
    record.payload = e;
    diag_hub_->post(std::move(record));

    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    events_.push_back(RuntimeEvent{++event_count_, at, std::move(text)});
    if (events_.size() > kEventsKept) {
        events_.erase(events_.begin());
    }
}

void System::noteFault(CaptureFault fault, const std::string& text) {
    // queues a note for the recorder thread; never waits for capture work
    if (capture_ != nullptr) {
        capture_->fault(fault, text);
    }
}

std::vector<RuntimeEvent> System::events() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return events_;
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
    if (profile_pending_.load()) {
        swapProfile();   // the inline boundary
    }
    estimationCycle(now);
    // the inline cycle lets the same cycle's evidence reach target
    // resolution and publishing, as the synchronous tests expect
    fieldCycle(execute_sensors_.retained(), now);
    reportingCycle(now);
}

void System::estimationCycle(MonotonicTime now) {
    const uint64_t cycle = cycle_.fetch_add(1) + 1;
    ++diagnostics_.cycles;
    last_now_ = now;
    const ExecutionContext context{now, cycle, &diagnostics_};

    const ResourceMap& resource_map = execute_resources_(context);
    const SensorMap&   sensor_map   = execute_sensors_(resource_map, context);
    if (profiled_) {
        precheck_.update(sensor_map, now);
    }

    CommandsOutput commands_out = commands_->run({command_, now, &diagnostics_});
    command_                    = commands_out.command;
    diagnostics_.note(slot_labels_[0], commands_out.status);
    if (profiled_) {
        // with this cycle's Brain IMU sample, before any placement request
        // reaches localization
        watchSensors(sensor_map, now);
    }

    robot_ = execute_localization_(sensor_map, requestsFrom(command_), context);
    if (profiled_) {
        watchModels(now);
    }
    noteRecovery(now);
    if (pico_ != nullptr) {
        updatePicoOperation(now);
    }

    publishSourceHealth(now);
}

void System::reportingCycle(MonotonicTime now) {
    std::shared_ptr<const FieldSnapshot> field  = fieldSnapshot();
    const LocalizationStatus             status = feed_->status();
    TargetResolutionOutput               target_out = target_resolution_->run(
        {command_, robot_, *feed_, field->field, field->observations, field->associations,
         target_, now});
    target_ = target_out.target;
    diagnostics_.note(slot_labels_[1], target_out.status);

    PublishingOutput pub_out = publishing_->run(
        {execute_sensors_.retained(), field->observations, field->associations, robot_,
         status, field->field, command_, target_, now, &diagnostics_});
    diagnostics_.note(slot_labels_[2], pub_out.status);

    auto reporting            = std::make_shared<ReportingSnapshot>();
    reporting->command        = command_;
    reporting->target         = target_;
    reporting->at             = now;
    reporting->cycle          = cycle_.load();
    reporting->brain_state    = pub_out.brain_state;
    reporting->pico_operation = pico_op_;
    if (profiled_) {
        std::array<translagatr::WheelReading, translagatr::kWheelReadingsMax> wheels{};
        uint8_t                                                    count = 0;
        if (readWheels(now, count, wheels.data()) == translagatr::kResultOk) {
            reporting->wheels.assign(wheels.begin(), wheels.begin() + count);
        }
    }
    auto diagnostics = std::make_shared<DiagnosticsSnapshot>();
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
    const RobotState                     robot    = feed_->latest();
    ++field_diagnostics_.cycles;
    const uint64_t         invocation = ++field_invocations_;
    const ExecutionContext context{now, invocation, &field_diagnostics_};

    // One standard call: the whole sensor snapshot, the latest robot state,
    // history lookups, the previous field and the context; the executor
    // owns declared-output enforcement and diagnostics.
    FieldEstimationOutput field_out =
        execute_world_({sensors, robot, *feed_, previous->field, context});

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
    const auto frameSnapshot = [&](const SensorId& camera, const CameraFramePayload& frame) {
        auto d                     = std::make_shared<DetectionFrameSnapshot>();
        d->camera                  = camera;
        d->engineering_frame       = frame.engineering_frame;
        d->frame_epoch             = frame.frame.epoch;
        d->frame_sequence          = frame.frame.sequence;
        d->exposureAt              = frame.frame.exposureAt;
        d->receivedAt              = frame.frame.receivedAt;
        d->processedAt             = running_.load() ? HostClock::now() : now;
        d->exposure_uncertainty_ms = frame.frame.exposure_uncertainty_ms;
        d->exposure_time_reliable  = frame.frame.exposure_time_reliable;
        d->width_px                = frame.frame.width_px;
        d->height_px               = frame.frame.height_px;
        d->y8                      = frame.frame.y8;
        d->intrinsics              = frame.intrinsics;
        d->field_invocation        = snapshot.invocation;
        return d;
    };

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
        auto d              = frameSnapshot(set->camera, *frame);
        d->has_observations = true;
        d->observations     = *set;
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
            const auto exposure     = feed_->sampleSnapshotAt(set->exposureAt);
            d->pose_at_exposure     = exposure.sample.pose;
            d->attitude_at_exposure = exposure.sample.attitude;
            d->field_from_odom      = exposure.current.robot.field_from_odom;
            d->anchor_revision      = exposure.current.robot.anchor_revision;
        }
        produced[set->camera] = std::move(d);
    }

    // Preview without detection: a camera frame nothing decoded this
    // invocation is still published under its own identity with
    // has_observations false, so preview works with noop world estimation.
    // A frame already published, decoded or not, is never replaced by a
    // raw copy of itself.
    for (const auto& kv : sensors) {
        if (produced.count(kv.first) != 0 || !kv.second.latest.has_value()) {
            continue;
        }
        const CameraFramePayload* frame = kv.second.latest->payload.get<CameraFramePayload>();
        if (frame == nullptr) {
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            const auto                  it = detection_frames_.find(kv.first);
            if (it != detection_frames_.end() && it->second->frame_epoch == frame->frame.epoch &&
                it->second->frame_sequence == frame->frame.sequence) {
                continue;
            }
        }
        auto       d        = frameSnapshot(kv.first, *frame);
        const auto exposure     = feed_->sampleSnapshotAt(frame->frame.exposureAt);
        d->pose_at_exposure     = exposure.sample.pose;
        d->attitude_at_exposure = exposure.sample.attitude;
        d->field_from_odom      = exposure.current.robot.field_from_odom;
        d->anchor_revision      = exposure.current.robot.anchor_revision;
        produced[kv.first]      = std::move(d);
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
    placement_floor_   = 0;
    if (profiled_) {
        swapToWaiting();   // power-on: no profile
    }

    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    field_ = std::make_shared<FieldSnapshot>();
    detection_frames_.clear();
    reporting_.reset();
    source_health_.reset();
}

void System::reset() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    const bool                  was_running = pauseWorkers();
    // the boundary before the stages publish their power-on state, so
    // capture rows from here on carry the new reset count
    if (capture_ != nullptr) {
        capture_->noteReset(reset_count_.load() + 1);
    }
    resetStages();
    reset_count_.fetch_add(1);
    if (was_running) {
        estimation_stats_.resetCounters();
        field_stats_.resetCounters();
    }
    resumeWorkers(was_running);
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
