"""e5: rewrite core/build.cpp so System::build reads as the make_* calls in run order."""
import sys

p = sys.argv[1]
s = open(p, newline='').read()

start = s.index('namespace\n{\n')
end = s.index('} // namespace\n')
anon_new = '''namespace
{

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

// The resolved <System> root, holding only its known sections.
bool parseSystem(const char* xml, tinyxml2::XMLDocument& doc, ConfigNode& root,
                 std::string& err) {
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
    root = ConfigNode{root_e};
    return checkChildren(root, {"Loop", "Resources", "Sensors", "Pipeline", "Inspection", "Capture"},
                         {}, err);
}

// <Loop rate_hz>, <Inspection> and <Capture>, each optional.
bool readSettings(const ConfigNode& root, double& loop_rate_hz, InspectionConfig& inspection,
                  CaptureConfig& capture, std::string& err) {
    const ConfigNode loop = root.child("Loop");
    if (loop.valid()) {
        if (!loop.getDouble("rate_hz", loop_rate_hz, loop_rate_hz, err)) {
            return false;
        }
        if (loop_rate_hz <= 0.0) {
            err = loop.path() + ": rate_hz must be positive";
            return false;
        }
    }
    return parseInspectionConfig(root.child("Inspection"), inspection, err) &&
           parseCaptureConfig(root.child("Capture"), capture, err);
}

// The <Pipeline> section, holding only the stage elements.
bool readPipeline(const ConfigNode& root, ConfigNode& pipeline, std::string& err) {
    pipeline = root.child("Pipeline");
    if (!pipeline.valid()) {
        err = root.path() + ": missing Pipeline section";
        return false;
    }
    return checkChildren(pipeline,
                         {"CommandCollection", "Localization", "WorldEstimation",
                          "TargetResolution", "Publishing"},
                         {}, err);
}

// A stage element the Pipeline must have; hint ends the error.
bool requireStage(const ConfigNode& pipeline, const char* name, const char* hint,
                  std::string& err) {
    if (pipeline.child(name).valid()) {
        return true;
    }
    err = pipeline.path() + " is missing " + name + hint;
    return false;
}

'''
s = s[:start] + anon_new + s[end:]

bstart = s.index('bool System::build(')
bend = s.index('} // namespace navigatr')
build_new = '''bool System::build(const char* xml, const FunctionRegistry& functions,
                   const BuildOptions& options, std::string& err) {
    session_id_ = newSessionId();

    // Node views die with doc; everything a stage keeps was copied by its
    // factory.
    tinyxml2::XMLDocument doc;
    ConfigNode            root;
    ConfigNode            pipeline;
    if (!parseSystem(xml, doc, root, err) ||
        !readSettings(root, loop_rate_hz_, inspection_, capture_config_, err)) {
        return false;
    }
    event_source_ = diag_hub_->sourceId("system");

    BuildOptions resource_options = options;
    resource_options.diagnostics  = diag_hub_.get();
    std::optional<ResourceBuild> resources =
        make_resources(root.child("Resources"), functions, resource_options, &warnings_, err);
    if (!resources.has_value()) {
        return false;
    }
    resources_      = std::move(resources->store);
    read_resources_ = std::move(resources->execute);

    std::optional<SensorExecutor> sensors = make_sensors(
        root.child("Sensors"), functions, resources_, read_resources_.outputs(), &warnings_, err);
    if (!sensors.has_value()) {
        return false;
    }
    read_sensors_ = std::move(*sensors);

    if (!readPipeline(root, pipeline, err)) {
        return false;
    }
    SlotInitializationContext slot_context;
    slot_context.resources    = &resources_;
    slot_context.sensors      = &read_sensors_.outputs();
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

    std::optional<CommandsExecutable> commands =
        make_commands(pipeline, functions, slot_context, err);
    if (!commands.has_value()) {
        return false;
    }
    collect_commands_ = std::move(*commands);
    // a publisher answering this collector must read the same link
    slot_context.commands_type = collect_commands_.type;
    slot_context.commands_serial =
        ResourceId{pipeline.child("CommandCollection").child("Serial").attr("resource_id")};

    if (!requireStage(pipeline, "Localization", "", err)) {
        return false;
    }
    std::optional<LocalizationExecutor> localization =
        profiled_ ? buildWaitingLocalization(err)
                  : make_localization(pipeline.child("Localization"), functions,
                                      read_sensors_.outputs(), resources_, &warnings_, err);
    if (!localization.has_value()) {
        return false;
    }
    localize_ = std::move(*localization);
    robot_    = localize_.state();
    feed_     = localize_.feed();
    // the robot state tap; the feed outlives profile boundaries
    feed_->setDiagnostics(diag_hub_);
    for (const ObservationFunctionStatus& f : localize_.functionStatus()) {
        slot_context.observation_functions.push_back(f.id);
    }
    slot_context.robot_observations = localize_.observationOutputs();

    if (!requireStage(pipeline, "WorldEstimation",
                      "; configure one Estimator, an implementation or the noop", err)) {
        return false;
    }
    std::optional<WorldEstimationExecutor> world =
        make_world_estimation(pipeline.child("WorldEstimation"), functions,
                              read_sensors_.outputs(), resources_, &warnings_, err);
    if (!world.has_value()) {
        return false;
    }
    estimate_world_ = std::move(*world);
    // Later slots reference only what the estimator declares it publishes
    // across the boundary, never an implementation detail.
    slot_context.observations = estimate_world_.observationOutputs();
    slot_context.associations = estimate_world_.associationOutputs();

    std::optional<TargetsExecutable> targets = make_targets(pipeline, functions, slot_context, err);
    if (!targets.has_value()) {
        return false;
    }
    resolve_targets_ = std::move(*targets);

    std::optional<PublishingExecutable> publishing =
        make_publishing(pipeline, functions, slot_context, err);
    if (!publishing.has_value()) {
        return false;
    }
    publish_ = std::move(*publishing);
    if (collect_commands_.type.value == "brain_link" && publish_.type.value != "brain_link") {
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

'''
s = s[:bstart] + build_new + s[bend:]
s = s.replace('// build.cpp\n',
              "// build.cpp\n// System::build: every stage's make_* in run order, each bound to its verb.\n",
              1)
open(p, 'w', newline='').write(s)
