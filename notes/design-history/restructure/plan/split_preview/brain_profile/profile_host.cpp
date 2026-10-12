// profile_host.cpp

#include "brain_profile/profile_host.h"

#include <cstdio>

#include "config/config_node.h"
#include "runtime/pico_control_ref.h"
#include "tinyxml2/tinyxml2.h"

namespace navigatr
{

namespace
{

std::string hexId(uint32_t id) {
    char buf[12];
    std::snprintf(buf, sizeof(buf), "%08x", static_cast<unsigned>(id));
    return buf;
}

} // namespace

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

std::shared_ptr<const ProfileBinding> System::profileBinding() const {
    std::lock_guard<std::mutex> lock(profile_mutex_);
    return applied_;
}

} // namespace navigatr
