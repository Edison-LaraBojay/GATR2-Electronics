// system.cpp

#include "runtime/system.h"

#include <fstream>
#include <set>
#include <sstream>

#include "config/composition.h"
#include "config/config_node.h"
#include "impl/resources/serial_links.h"
#include "tinyxml2/tinyxml2.h"

namespace navigatr
{

namespace
{

bool readFile(const std::string& path, std::string& out, std::string& err) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        err = "cannot open " + path;
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

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

} // namespace

std::unique_ptr<System> System::buildFromFile(const std::string& path,
                                              const FunctionRegistry& functions,
                                              std::string& err, const BuildOptions& options) {
    bool composed = false;
    if (!isComposedConfiguration(path, composed, err)) {
        return nullptr;
    }
    if (composed) {
        ResolvedConfiguration resolved;
        if (!resolveConfiguration(path, resolved, err)) {
            return nullptr;
        }
        auto system = buildFromString(resolved.xml.c_str(), functions, err, options);
        if (system != nullptr) {
            system->configuration_id_     = resolved.id;
            system->configuration_digest_ = resolved.digest;
        }
        return system;
    }
    std::string xml;
    if (!readFile(path, xml, err)) {
        return nullptr;
    }
    auto system = buildFromString(xml.c_str(), functions, err, options);
    if (system != nullptr) {
        system->configuration_id_     = path;
        system->configuration_digest_ = contentDigest(0, xml);
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

bool System::build(const char* xml, const FunctionRegistry& functions,
                   const BuildOptions& options, std::string& err) {
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
    const ConfigNode root{root_e};

    if (!checkChildren(root, {"Loop", "Resources", "Sensors", "Pipeline"}, {}, err)) {
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

    // Resources: index everything, apply replay overrides, then build with
    // dependency resolution so declaration order never matters.
    ResourceMapBuilder resource_builder(functions, &warnings_);
    resource_builder.setAllowProvisional(options.allow_provisional);
    const ConfigNode     resources_node = root.child("Resources");
    if (resources_node.valid()) {
        if (!checkChildren(resources_node, {"Resource"}, {"Resource"}, err)) {
            return false;
        }
        bool ok = true;
        resources_node.forEach("Resource", [&](const ConfigNode& r) {
            if (ok) {
                ok = resource_builder.index(r, err);
            }
        });
        if (!ok) {
            return false;
        }
    }
    for (const auto& kv : options.replay) {
        if (!resource_builder.overrideFactory(ResourceId{kv.first},
                                              fileReplayFactoryForPath(kv.second), err)) {
            return false;
        }
    }
    if (!resource_builder.buildAll(err)) {
        return false;
    }
    resources_ = resource_builder.take();

    // Sensors: generic builder owns id/type/duplicates; the selected factory
    // owns everything else in its subtree.
    const ConfigNode sensors_node = root.child("Sensors");
    if (sensors_node.valid()) {
        if (!checkChildren(sensors_node, {"Sensor"}, {"Sensor"}, err)) {
            return false;
        }
        SensorInitializationContext context;
        context.resources = &resources_;
        context.functions = &functions;
        context.warnings  = &warnings_;

        bool ok = true;
        sensors_node.forEach("Sensor", [&](const ConfigNode& s) {
            if (!ok) {
                return;
            }
            const SensorId    id{s.attr("id")};
            const FunctionKey type{s.attr("type")};
            if (id.empty() || type.empty()) {
                err = s.path() + ": Sensor needs id and type";
                ok  = false;
                return;
            }
            if (sensors_.find(id) != nullptr) {
                err = s.path() + ": duplicate Sensor id " + id.value;
                ok  = false;
                return;
            }
            const SensorMakeFunction* factory =
                functions.find<SensorMakeFunction>(type, err);
            if (factory == nullptr) {
                err = s.path() + ": " + err;
                ok  = false;
                return;
            }
            std::optional<SensorExecutable> built = (*factory)(s, context, err);
            if (!built.has_value() || !built->execute) {
                if (err.empty()) {
                    err = s.path() + ": factory produced no executable";
                }
                ok = false;
                return;
            }
            catalog_.add(id, built->outputPayload);
            sensor_results_[id] = SensorRecord{};
            sensors_.add(id, std::move(*built), "Sensor/" + id.value);
        });
        if (!ok) {
            return false;
        }
    }

    const ConfigNode pipeline = root.child("Pipeline");
    if (!pipeline.valid()) {
        err = root.path() + ": missing Pipeline section";
        return false;
    }
    if (!checkChildren(pipeline,
                       {"CommandCollection", "Preprocessing", "Localization",
                        "FieldEstimation", "TargetResolution", "Publishing"},
                       {}, err)) {
        return false;
    }

    SlotInitializationContext slot_context;
    slot_context.resources = &resources_;
    slot_context.sensors   = &catalog_;
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

    PreprocessorInitializationContext preprocessing_context;
    preprocessing_context.sensors   = &catalog_;
    preprocessing_context.resources = &resources_;
    preprocessing_context.functions = &functions;
    preprocessing_context.warnings  = &warnings_;
    if (!buildSlot("Preprocessing", preprocessing_, Tag<PreprocessingMakeFunction>{},
                   preprocessing_context, slot_labels_[1])) {
        return false;
    }
    slot_context.artifacts = preprocessing_->produces();
    artifact_decls_        = slot_context.artifacts;

    if (!buildSlot("Localization", localization_, Tag<LocalizationMakeFunction>{},
                   slot_context, slot_labels_[2])) {
        return false;
    }
    if (!buildSlot("FieldEstimation", field_estimation_,
                   Tag<FieldEstimationMakeFunction>{}, slot_context, slot_labels_[3])) {
        return false;
    }
    // Later slots reference only what world estimation declares it
    // publishes across the boundary, never a composite implementation
    // detail.
    slot_context.observations = field_estimation_->producesObservations();
    slot_context.associations = field_estimation_->producesAssociations();
    observation_decls_        = slot_context.observations;
    association_decls_        = slot_context.associations;

    if (!buildSlot("TargetResolution", target_resolution_,
                   Tag<TargetResolutionMakeFunction>{}, slot_context, slot_labels_[4])) {
        return false;
    }
    if (!buildSlot("Publishing", publishing_, Tag<PublishingMakeFunction>{}, slot_context,
                   slot_labels_[5])) {
        return false;
    }
    return true;
}

void System::step(MonotonicTime now) {
    ++cycle_;
    ++diagnostics_.cycles;

    // Sensor Collection: the framework owns record bookkeeping; sensors only
    // report state and publications.
    SensorExecutionInput sensor_input;
    sensor_input.now         = now;
    sensor_input.cycle       = cycle_;
    sensor_input.diagnostics = &diagnostics_;

    for (SensorMap::Entry& entry : sensors_.executionOrder()) {
        SensorPollResult poll   = entry.sensor.execute(sensor_input);
        SensorRecord&    record = sensor_results_[entry.id];
        record.state            = poll.state;
        record.lastPolledAt     = now;
        record.diagnostic       = poll.diagnostic;
        // Declared-output enforcement: a publication contradicting the
        // sensor's declared payload never enters the record.
        if (poll.publication.has_value() &&
            !entry.sensor.outputPayload.matches(poll.publication->payload.cppType())) {
            poll.publication.reset();
            record.state      = SensorState::kFault;
            record.diagnostic = "published a payload contradicting the declared " +
                                std::string(entry.sensor.outputPayload.stable_name);
        }
        if (poll.publication.has_value()) {
            StoredSensorSample stored;
            stored.measuredAt = poll.publication->measuredAt;
            stored.receivedAt = now;
            stored.sequence =
                record.latest.has_value() ? record.latest->sequence + 1 : 1;
            stored.payload = poll.publication->payload;
            record.latest  = std::move(stored);
        }
        FunctionStatus s = FunctionStatus::kOk;
        if (poll.state == SensorState::kFault) {
            s = FunctionStatus::kFault;
        } else if (poll.state != SensorState::kValid) {
            s = FunctionStatus::kNoData;
        }
        diagnostics_.note(entry.label, s);
    }

    CommandsOutput commands_out = commands_->run({command_, now, &diagnostics_});
    command_                    = commands_out.command;
    diagnostics_.note(slot_labels_[0], commands_out.status);

    PreprocessingOutput pre_out =
        preprocessing_->run({sensor_results_, now, cycle_, &diagnostics_});
    enforceDeclared(pre_out.artifacts, artifact_decls_, slot_labels_[1]);
    diagnostics_.note(slot_labels_[1], pre_out.status);

    const uint64_t epoch_before = robot_.odometry_epoch;

    LocalizationOutput loc_out =
        localization_->run({sensor_results_, pre_out.artifacts, command_, robot_, now});
    robot_ = loc_out.robot;
    diagnostics_.note(slot_labels_[2], loc_out.status);

    // An odometry epoch change means the old frame shares nothing with the
    // new one; poses recorded in it are not history, they are garbage.
    if (robot_.odometry_epoch != epoch_before) {
        robot_.history.clear();
    }

    // Framework-owned pose history, so evidence with an exposure timestamp
    // can be evaluated against the pose at exposure. Only valid poses enter
    // history (a startup pose of zeros is not evidence), stamped at the
    // estimator's mapped measurement time when it has one; the loop time is
    // an upper bound fallback. A clock-map revision can move a mapped time
    // at or before the newest entry; the newer pose then explicitly
    // replaces that entry so history stays monotonic without losing the
    // newest estimate.
    if (robot_.valid) {
        MonotonicTime at = robot_.measuredAtHost.isSet() ? robot_.measuredAtHost : now;
        if (!robot_.history.empty() && at < robot_.history.back().at) {
            at = robot_.history.back().at;   // revision: clamp, keep newest pose
        }
        if (robot_.history.empty() || at > robot_.history.back().at) {
            robot_.history.push_back(TimedOdomPose{at, robot_.odom_pose});
        } else {
            robot_.history.back() = TimedOdomPose{at, robot_.odom_pose};
        }
        while (robot_.history.size() > RobotState::kHistoryCapacity) {
            robot_.history.pop_front();
        }
    }

    FieldEstimationOutput field_out = field_estimation_->run(
        {sensor_results_, pre_out.artifacts, robot_, field_, now});
    enforceDeclared(field_out.observations, observation_decls_, slot_labels_[3]);
    enforceDeclared(field_out.associations, association_decls_, slot_labels_[3]);
    field_ = field_out.field;
    diagnostics_.note(slot_labels_[3], field_out.status);

    TargetResolutionOutput target_out = target_resolution_->run(
        {command_, robot_, field_, field_out.observations, field_out.associations,
         target_, now});
    target_ = target_out.target;
    diagnostics_.note(slot_labels_[4], target_out.status);

    PublishingOutput pub_out =
        publishing_->run({sensor_results_, pre_out.artifacts, field_out.observations,
                          field_out.associations, robot_, field_, command_, target_, now});
    diagnostics_.note(slot_labels_[5], pub_out.status);
}

void System::reset() {
    for (SensorMap::Entry& entry : sensors_.executionOrder()) {
        if (entry.sensor.reset) {
            entry.sensor.reset();
        }
        sensor_results_[entry.id] = SensorRecord{};
    }
    resources_.resetAll();   // shared resources reset once, not per consumer
    commands_->reset();
    preprocessing_->reset();
    localization_->reset();
    field_estimation_->reset();
    target_resolution_->reset();
    publishing_->reset();

    // A hard reset is an odometry discontinuity: the new odometry frame
    // shares nothing with the old one, so the epoch moves on.
    const uint64_t next_epoch = robot_.odometry_epoch + 1;

    robot_                = RobotState{};
    robot_.odometry_epoch = next_epoch;
    field_                = FieldState{};
    command_              = CommandState{};
    target_               = TargetState{};
}

} // namespace navigatr
