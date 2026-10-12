// build.cpp

#include "runtime/system.h"

#include <chrono>
#include <cstdio>
#include <random>
#include <set>

#include "brain_profile/profile_host.h"   // complete ProfileHost: ctor, dtor, build
#include "config/composition.h"
#include "config/config_node.h"
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

} // namespace

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

} // namespace navigatr
