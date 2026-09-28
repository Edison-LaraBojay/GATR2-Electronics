// brain_profile_builder.cpp

#include "runtime/brain_profile_builder.h"

#include <cstdio>

#include "payloads/pico_telemetry_samples.h"
#include "resources/brain_imu_bench.h"
#include "tinyxml2/tinyxml2.h"

namespace navigatr
{

namespace
{

bool positive(const ConfigNode& node, const char* what, double v, std::string& err) {
    if (!(v > 0.0)) {
        err = node.path() + ": " + what + " must be positive";
        return false;
    }
    return true;
}

bool nonNegative(const ConfigNode& node, const char* what, double v, std::string& err) {
    if (v < 0.0) {
        err = node.path() + ": " + what + " cannot be negative";
        return false;
    }
    return true;
}

bool parseEncoders(const ConfigNode& node, const ResourceCatalog& outputs,
                   BrainProfileConfig& out, std::string& err) {
    std::string resource;
    if (!node.onlyAttributes({"resource_id", "stale_after_ms"}, err) ||
        !node.onlyChildren({"Port"}, err) || !node.requireAttr("resource_id", resource, err) ||
        !node.getInt("stale_after_ms", 250, out.encoder_stale_after_ms, err) ||
        !nonNegative(node, "stale_after_ms", out.encoder_stale_after_ms, err)) {
        return false;
    }
    out.encoder_resource = ResourceId{resource};
    bool any             = false;
    bool ok              = true;
    node.forEach("Port", [&](const ConfigNode& port) {
        if (!ok) {
            return;
        }
        long        index = -1;
        std::string output;
        ok = port.onlyAttributes({"index", "output_id"}, err) &&
             port.requireInt("index", index, err) && port.requireAttr("output_id", output, err);
        if (!ok) {
            return;
        }
        if (index < 0 || index >= kProfileEncoderPorts) {
            err = port.path() + ": index must be 0..2";
            ok  = false;
            return;
        }
        if (!out.ports[index].empty()) {
            err = port.path() + ": port " + std::to_string(index) + " is declared twice";
            ok  = false;
            return;
        }
        for (const OutputId& seen : out.ports) {
            if (seen.value == output) {
                err = port.path() + ": output " + output + " serves two ports";
                ok  = false;
                return;
            }
        }
        TypedOutputBinding<PicoEncoderCounts> binding;
        ok = outputs.bind<PicoEncoderCounts>(out.encoder_resource, OutputId{output},
                                             port.path(), binding, err);
        out.ports[index] = OutputId{output};
        any              = true;
    });
    if (ok && !any) {
        err = node.path() + ": needs at least one <Port index=... output_id=.../>";
        ok  = false;
    }
    return ok;
}

bool parseImu(const ConfigNode& node, const ResourceCatalog& outputs, BrainProfileConfig& out,
              std::string& err) {
    long        port = -1;
    std::string resource, output;
    if (!node.onlyAttributes({"port", "resource_id", "output_id", "stale_after_ms"}, err) ||
        !node.onlyChildren({}, err) || !node.requireInt("port", port, err) ||
        !node.requireAttr("resource_id", resource, err) ||
        !node.requireAttr("output_id", output, err) ||
        !node.getInt("stale_after_ms", 250, out.imu_stale_after_ms, err) ||
        !nonNegative(node, "stale_after_ms", out.imu_stale_after_ms, err)) {
        return false;
    }
    if (port < 0 || port > 255) {
        err = node.path() + ": port must be 0..255";
        return false;
    }
    out.imu          = true;
    out.imu_port     = static_cast<uint8_t>(port);
    out.imu_resource = ResourceId{resource};
    out.imu_output   = OutputId{output};
    TypedOutputBinding<PicoGyroRate> binding;
    return outputs.bind<PicoGyroRate>(out.imu_resource, out.imu_output, node.path(), binding, err);
}

bool parseFusion(const ConfigNode& node, BrainProfileConfig& out, std::string& err) {
    if (!node.onlyAttributes({"max_wait_ms"}, err) ||
        !node.onlyChildren({"MotionNoise", "HeadingNoise"}, err) ||
        !node.atMostOne("MotionNoise", err) || !node.atMostOne("HeadingNoise", err) ||
        !node.getInt("max_wait_ms", 100, out.max_wait_ms, err) ||
        !positive(node, "max_wait_ms", out.max_wait_ms, err)) {
        return false;
    }
    const ConfigNode motion  = node.child("MotionNoise");
    const ConfigNode heading = node.child("HeadingNoise");
    if (!motion.valid() || !heading.valid()) {
        err = node.path() + ": needs MotionNoise and HeadingNoise";
        return false;
    }
    // no defaults: noise is tuning the configuration states explicitly
    if (!motion.onlyAttributes({"translation_floor_m", "translation_per_m", "rotation_floor_rad",
                                "rotation_per_rad", "rotation_per_m"},
                               err) ||
        !motion.requireDouble("translation_floor_m", out.translation_floor_m, err) ||
        !motion.requireDouble("translation_per_m", out.translation_per_m, err) ||
        !motion.requireDouble("rotation_floor_rad", out.rotation_floor_rad, err) ||
        !motion.requireDouble("rotation_per_rad", out.rotation_per_rad, err) ||
        !motion.requireDouble("rotation_per_m", out.rotation_per_m, err) ||
        !heading.onlyAttributes({"angle_random_walk_rad_per_sqrt_s", "bias_rad_per_s"}, err) ||
        !heading.requireDouble("angle_random_walk_rad_per_sqrt_s",
                               out.angle_random_walk_rad_per_sqrt_s, err) ||
        !heading.requireDouble("bias_rad_per_s", out.bias_rad_per_s, err)) {
        return false;
    }
    if (!positive(motion, "translation_floor_m", out.translation_floor_m, err) ||
        !positive(motion, "rotation_floor_rad", out.rotation_floor_rad, err) ||
        !nonNegative(motion, "translation_per_m", out.translation_per_m, err) ||
        !nonNegative(motion, "rotation_per_rad", out.rotation_per_rad, err) ||
        !nonNegative(motion, "rotation_per_m", out.rotation_per_m, err) ||
        !positive(heading, "angle_random_walk_rad_per_sqrt_s",
                  out.angle_random_walk_rad_per_sqrt_s, err) ||
        !nonNegative(heading, "bias_rad_per_s", out.bias_rad_per_s, err)) {
        return false;
    }
    out.fusion = true;
    return true;
}

tinyxml2::XMLElement* add(tinyxml2::XMLDocument& doc, tinyxml2::XMLNode* parent,
                          const char* name) {
    tinyxml2::XMLElement* e = doc.NewElement(name);
    parent->InsertEndChild(e);
    return e;
}

void addHistory(const BrainProfileConfig& c, tinyxml2::XMLDocument& doc,
                tinyxml2::XMLElement* localization) {
    tinyxml2::XMLElement* h = add(doc, localization, "History");
    h->SetAttribute("retention_s", c.history_retention_s);
    h->SetAttribute("capacity", static_cast<int64_t>(c.history_capacity));
    h->SetAttribute("max_interpolation_gap_ms", static_cast<int64_t>(c.history_gap_ms));
    h->SetAttribute("attitude_gap_ms", static_cast<int64_t>(c.history_attitude_ms));
}

std::string encoderSensorId(uint8_t port) {
    return std::string(kProfileSensorPrefix) + "encoder_" + std::to_string(port);
}

std::string imuSensorId(uint8_t port) {
    return std::string(kProfileSensorPrefix) + "imu_" + std::to_string(port);
}

std::string summarize(const gatr2::RobotProfileDoc& p) {
    char        buf[160];
    std::string s = profileTopologyName(p.topology);
    for (uint8_t i = 0; i < p.wheel_count; ++i) {
        const gatr2::ProfileWheel& w = p.wheels[i];
        std::snprintf(buf, sizeof(buf),
                      "; port %u r %.4f m at (%.4f, %.4f) m %.3f deg cpr %u gear %.6g scale "
                      "%.6g%s",
                      static_cast<unsigned>(w.encoder_port), w.radius_um * 1e-6, w.x_um * 1e-6,
                      w.y_um * 1e-6, w.angle_mdeg / 1000.0, static_cast<unsigned>(w.counts_per_rev),
                      w.gear_micro * 1e-6, w.travel_scale_ppm * 1e-6,
                      (w.flags & gatr2::kWheelReversed) != 0 ? " reversed" : "");
        s += buf;
    }
    switch (p.imu_source) {
    case gatr2::kImuSourcePico:
        std::snprintf(buf, sizeof(buf), "; IMU Pico port %u%s", static_cast<unsigned>(p.imu_port),
                      (p.imu_flags & gatr2::kImuInvert) != 0 ? " inverted" : "");
        break;
    case gatr2::kImuSourceBrainVex:
        std::snprintf(buf, sizeof(buf), "; IMU Brain VEX smart port %u",
                      static_cast<unsigned>(p.vex_smart_port));
        break;
    default: std::snprintf(buf, sizeof(buf), "; no IMU"); break;
    }
    s += buf;
    std::snprintf(buf, sizeof(buf), "; footprint front %.3f back %.3f left %.3f right %.3f m",
                  p.footprint_front_um * 1e-6, p.footprint_back_um * 1e-6,
                  p.footprint_left_um * 1e-6, p.footprint_right_um * 1e-6);
    s += buf;
    return s;
}

} // namespace

const char* profileTopologyName(uint8_t topology) {
    switch (topology) {
    case gatr2::kTopologyTwoWheelImu: return "two wheels + IMU";
    case gatr2::kTopologyTwoForwardWheelImu: return "two forward wheels + IMU";
    case gatr2::kTopologyThreeWheel: return "three wheels";
    default: return "unknown";
    }
}

const char* profileReasonName(uint8_t reason) {
    switch (reason) {
    case gatr2::kProfileReasonNone: return "none";
    case gatr2::kProfileReasonFormat: return "format";
    case gatr2::kProfileReasonTopology: return "topology";
    case gatr2::kProfileReasonWheelCount: return "wheel count";
    case gatr2::kProfileReasonEncoderPort: return "encoder port";
    case gatr2::kProfileReasonWheelGeometry: return "wheel geometry";
    case gatr2::kProfileReasonObservability: return "observability";
    case gatr2::kProfileReasonImuSource: return "IMU source";
    case gatr2::kProfileReasonImuPort: return "IMU port";
    case gatr2::kProfileReasonImuCombination: return "IMU combination";
    case gatr2::kProfileReasonCamera: return "camera";
    case gatr2::kProfileReasonFootprint: return "footprint";
    case gatr2::kProfileReasonBuild: return "build";
    case gatr2::kProfileReasonNotAccepted: return "not accepted";
    case gatr2::kProfileReasonCalibration: return "calibration";
    default: return "unknown";
    }
}

bool parseBrainProfileConfig(const ConfigNode& node, const ResourceStore& resources,
                             const ResourceCatalog& outputs, BrainProfileConfig& out,
                             std::string& err) {
    out = BrainProfileConfig{};
    if (!node.onlyAttributes({}, err) ||
        !node.onlyChildren({"Encoders", "Imu", "BrainImu", "Calibration", "Timing", "Fusion",
                            "History"},
                           err)) {
        return false;
    }
    for (const char* name :
         {"Encoders", "Imu", "BrainImu", "Calibration", "Timing", "Fusion", "History"}) {
        if (!node.atMostOne(name, err)) {
            return false;
        }
    }

    const ConfigNode encoders = node.child("Encoders");
    if (!encoders.valid()) {
        err = node.path() + ": needs <Encoders resource_id=...> with the wired ports";
        return false;
    }
    if (!parseEncoders(encoders, outputs, out, err)) {
        return false;
    }
    const ConfigNode imu = node.child("Imu");
    if (imu.valid() && !parseImu(imu, outputs, out, err)) {
        return false;
    }
    const ConfigNode brain_imu = node.child("BrainImu");
    if (brain_imu.valid()) {
        std::string id;
        if (!brain_imu.onlyAttributes({"resource_id"}, err) || !brain_imu.onlyChildren({}, err) ||
            !brain_imu.requireAttr("resource_id", id, err)) {
            return false;
        }
        std::string inner;
        if (resources.require<BrainImuBench>(ResourceId{id}, inner) == nullptr) {
            err = brain_imu.path() + ": " + inner;
            return false;
        }
        out.brain_imu = ResourceId{id};
    }

    const ConfigNode calibration = node.child("Calibration");
    if (calibration.valid() &&
        (!calibration.onlyAttributes({"bias_samples", "window_ms", "max_gap_ms", "still_travel_m",
                                      "still_rate_dps", "max_rate_dps", "evidence_gap_ms",
                                      "attempt_s"},
                                     err) ||
         !calibration.onlyChildren({}, err))) {
        return false;
    }
    if (!calibration.getInt("bias_samples", 20, out.bias_samples, err) ||
        !calibration.getInt("window_ms", 2000, out.window_ms, err) ||
        !calibration.getInt("max_gap_ms", 250, out.max_gap_ms, err) ||
        !calibration.getDouble("still_travel_m", 0.001, out.still_travel_m, err) ||
        !calibration.getDouble("still_rate_dps", 1.0, out.still_rate_dps, err) ||
        !calibration.getDouble("max_rate_dps", 5.0, out.max_rate_dps, err) ||
        !calibration.getInt("evidence_gap_ms", 100, out.evidence_gap_ms, err) ||
        !calibration.getDouble("attempt_s", 60.0, out.attempt_s, err)) {
        return false;
    }
    if (calibration.valid() &&
        (!nonNegative(calibration, "bias_samples", out.bias_samples, err) ||
         !nonNegative(calibration, "window_ms", out.window_ms, err) ||
         !positive(calibration, "max_gap_ms", out.max_gap_ms, err) ||
         !positive(calibration, "still_travel_m", out.still_travel_m, err) ||
         !positive(calibration, "still_rate_dps", out.still_rate_dps, err) ||
         !positive(calibration, "max_rate_dps", out.max_rate_dps, err) ||
         !positive(calibration, "evidence_gap_ms", out.evidence_gap_ms, err) ||
         !positive(calibration, "attempt_s", out.attempt_s, err))) {
        return false;
    }

    const ConfigNode timing = node.child("Timing");
    if (timing.valid() &&
        (!timing.onlyAttributes(
             {"interval_tolerance_ms", "max_pending_ms", "sensor_loss_ms", "on_sensor_loss"},
             err) ||
         !timing.onlyChildren({}, err))) {
        return false;
    }
    std::string on_loss = "unplace";
    if (!timing.getInt("interval_tolerance_ms", 20, out.interval_tolerance_ms, err) ||
        !timing.getInt("max_pending_ms", 500, out.max_pending_ms, err) ||
        !timing.getInt("sensor_loss_ms", 250, out.sensor_loss_ms, err)) {
        return false;
    }
    if (timing.valid() && !timing.attr("on_sensor_loss").empty()) {
        on_loss = timing.attr("on_sensor_loss");
    }
    if (on_loss != "unplace" && on_loss != "warn") {
        err = timing.path() + ": on_sensor_loss must be unplace or warn";
        return false;
    }
    out.unplace_on_sensor_loss = on_loss == "unplace";
    if (timing.valid() &&
        (!nonNegative(timing, "interval_tolerance_ms", out.interval_tolerance_ms, err) ||
         !positive(timing, "max_pending_ms", out.max_pending_ms, err) ||
         !positive(timing, "sensor_loss_ms", out.sensor_loss_ms, err))) {
        return false;
    }

    const ConfigNode fusion = node.child("Fusion");
    if (fusion.valid() && !parseFusion(fusion, out, err)) {
        return false;
    }

    const ConfigNode history = node.child("History");
    if (history.valid() &&
        (!history.onlyAttributes({"retention_s", "capacity", "max_interpolation_gap_ms",
                                  "attitude_gap_ms"},
                                 err) ||
         !history.onlyChildren({}, err))) {
        return false;
    }
    if (!history.getDouble("retention_s", 5.0, out.history_retention_s, err) ||
        !history.getInt("capacity", 1024, out.history_capacity, err) ||
        !history.getInt("max_interpolation_gap_ms", 100, out.history_gap_ms, err) ||
        !history.getInt("attitude_gap_ms", 100, out.history_attitude_ms, err)) {
        return false;
    }
    if (out.history_retention_s <= 0.0 || out.history_capacity < 2 || out.history_gap_ms <= 0 ||
        out.history_attitude_ms <= 0) {
        err = history.path() + ": retention_s, capacity, max_interpolation_gap_ms and "
                               "attitude_gap_ms must be positive (capacity at least 2)";
        return false;
    }
    return true;
}

bool checkProfileCapabilities(const BrainProfileConfig& config,
                              const gatr2::RobotProfileDoc& profile, uint8_t& reason,
                              uint8_t& detail) {
    reason = gatr2::kProfileReasonNone;
    detail = 0;
    for (uint8_t i = 0; i < profile.wheel_count; ++i) {
        const uint8_t port = profile.wheels[i].encoder_port;
        if (port >= kProfileEncoderPorts || config.ports[port].empty()) {
            reason = gatr2::kProfileReasonEncoderPort;
            detail = i;
            return false;
        }
    }
    switch (profile.imu_source) {
    case gatr2::kImuSourcePico:
        if (!config.imu) {
            reason = gatr2::kProfileReasonImuSource;
            return false;
        }
        if (profile.imu_port != config.imu_port) {
            reason = gatr2::kProfileReasonImuPort;
            return false;
        }
        break;
    case gatr2::kImuSourceBrainVex:
        if (config.brain_imu.empty()) {
            reason = gatr2::kProfileReasonImuSource;
            return false;
        }
        break;
    case gatr2::kImuSourceNone: break;
    default: reason = gatr2::kProfileReasonImuSource; return false;
    }
    // camera mounts from the profile are not applied on the Pi yet
    if (profile.camera_count > 0) {
        reason = gatr2::kProfileReasonCamera;
        return false;
    }
    const bool three = profile.topology == gatr2::kTopologyThreeWheel;
    const bool two   = profile.topology == gatr2::kTopologyTwoWheelImu ||
                     profile.topology == gatr2::kTopologyTwoForwardWheelImu;
    const bool supported =
        (two && (profile.imu_source == gatr2::kImuSourcePico ||
                 profile.imu_source == gatr2::kImuSourceBrainVex)) ||
        (three && profile.imu_source == gatr2::kImuSourceNone) ||
        (three && profile.imu_source == gatr2::kImuSourcePico && config.fusion);
    if (!supported) {
        reason = gatr2::kProfileReasonImuCombination;
        return false;
    }
    return true;
}

void writeWaitingLocalization(const BrainProfileConfig& config, tinyxml2::XMLDocument& doc) {
    doc.Clear();
    tinyxml2::XMLElement* localization = add(doc, &doc, "Localization");
    add(doc, localization, "Estimator")->SetAttribute("type", "noop");
    addHistory(config, doc, localization);
}

void writeProfileSubtrees(const BrainProfileConfig& c, const gatr2::RobotProfileDoc& p,
                          tinyxml2::XMLDocument& doc, ProfileBinding& binding) {
    doc.Clear();
    tinyxml2::XMLElement* root         = add(doc, &doc, "Profile");
    tinyxml2::XMLElement* sensors      = add(doc, root, "Sensors");
    tinyxml2::XMLElement* localization = add(doc, root, "Localization");

    binding.encoders.clear();
    binding.imu           = SensorId{};
    binding.bias_function.clear();
    binding.summary = summarize(p);

    // encoder sensors: counts per revolution, gearing and polarity, once
    for (uint8_t i = 0; i < p.wheel_count; ++i) {
        const gatr2::ProfileWheel& w  = p.wheels[i];
        const std::string          id = encoderSensorId(w.encoder_port);
        tinyxml2::XMLElement*      s  = add(doc, sensors, "Sensor");
        s->SetAttribute("id", id.c_str());
        s->SetAttribute("type", "pico_encoder_channel");
        tinyxml2::XMLElement* source = add(doc, s, "Source");
        source->SetAttribute("resource_id", c.encoder_resource.value.c_str());
        source->SetAttribute("output_id", c.ports[w.encoder_port].value.c_str());
        tinyxml2::XMLElement* cal = add(doc, s, "Calibration");
        cal->SetAttribute("counts_per_revolution", static_cast<int64_t>(w.counts_per_rev));
        cal->SetAttribute("gear", w.gear_micro / static_cast<double>(gatr2::kUnitMicro));
        cal->SetAttribute("invert", (w.flags & gatr2::kWheelReversed) != 0);
        add(doc, s, "Freshness")
            ->SetAttribute("stale_after_ms", static_cast<int64_t>(c.encoder_stale_after_ms));
        binding.encoders.push_back(SensorId{id});
    }
    const bool pico_imu = p.imu_source == gatr2::kImuSourcePico;
    if (pico_imu) {
        const std::string     id = imuSensorId(p.imu_port);
        tinyxml2::XMLElement* s  = add(doc, sensors, "Sensor");
        s->SetAttribute("id", id.c_str());
        s->SetAttribute("type", "pico_imu_channel");
        tinyxml2::XMLElement* source = add(doc, s, "Source");
        source->SetAttribute("resource_id", c.imu_resource.value.c_str());
        source->SetAttribute("output_id", c.imu_output.value.c_str());
        add(doc, s, "Calibration")->SetAttribute("invert", (p.imu_flags & gatr2::kImuInvert) != 0);
        add(doc, s, "Freshness")
            ->SetAttribute("stale_after_ms", static_cast<int64_t>(c.imu_stale_after_ms));
        binding.imu = SensorId{id};
    }

    const long window_ms =
        p.calibration_window_ms != 0 ? static_cast<long>(p.calibration_window_ms) : c.window_ms;
    const double still_travel_m =
        p.still_travel_um != 0 ? p.still_travel_um * 1e-6 : c.still_travel_m;
    const double still_rate_dps =
        p.still_rate_cdps != 0 ? p.still_rate_cdps / 100.0 : c.still_rate_dps;

    // the stationary window every bias path and the stationary status share
    const auto stillness = [&](tinyxml2::XMLElement* e) {
        e->SetAttribute("window_ms", static_cast<int64_t>(window_ms));
        e->SetAttribute("still_rate_dps", still_rate_dps);
        e->SetAttribute("max_rate_dps", c.max_rate_dps);
        e->SetAttribute("evidence_gap_ms", static_cast<int64_t>(c.evidence_gap_ms));
    };

    // wheel geometry and the travel scale, once; direction is always positive
    const auto addWheels = [&](tinyxml2::XMLElement* model) {
        for (uint8_t i = 0; i < p.wheel_count; ++i) {
            const gatr2::ProfileWheel& w     = p.wheels[i];
            tinyxml2::XMLElement*      wheel = add(doc, model, "TrackingWheel");
            wheel->SetAttribute("sensor_id", encoderSensorId(w.encoder_port).c_str());
            wheel->SetAttribute("label", ("port " + std::to_string(w.encoder_port)).c_str());
            wheel->SetAttribute("radius_m", w.radius_um * 1e-6);
            wheel->SetAttribute("position_x_m", w.x_um * 1e-6);
            wheel->SetAttribute("position_y_m", w.y_um * 1e-6);
            wheel->SetAttribute("measurement_angle_deg", w.angle_mdeg / 1000.0);
            wheel->SetAttribute("direction", "positive");
            wheel->SetAttribute("travel_scale",
                                w.travel_scale_ppm / static_cast<double>(gatr2::kUnitMicro));
        }
    };
    const char* motion_id = "profile_motion";
    const auto  output    = [&](tinyxml2::XMLElement* model, const char* id) {
        add(doc, model, "Output")->SetAttribute("observation_id", id);
    };
    const auto timing = [&](tinyxml2::XMLElement* model) {
        tinyxml2::XMLElement* t = add(doc, model, "Timing");
        t->SetAttribute("interval_tolerance_ms", static_cast<int64_t>(c.interval_tolerance_ms));
        t->SetAttribute("max_pending_ms", static_cast<int64_t>(c.max_pending_ms));
    };

    tinyxml2::XMLElement* motion = add(doc, localization, "Observation");
    motion->SetAttribute("id", motion_id);
    const bool bench = p.imu_source == gatr2::kImuSourceBrainVex;
    if (bench) {
        motion->SetAttribute("type", p.topology == gatr2::kTopologyTwoForwardWheelImu
                                         ? "brain_imu_parallel_bench"
                                         : "brain_imu_planar_bench");
        addWheels(motion);
        add(doc, motion, "Imu")->SetAttribute("resource_id", c.brain_imu.value.c_str());
        tinyxml2::XMLElement* still = add(doc, motion, "Stillness");
        stillness(still);
        still->SetAttribute("samples", static_cast<int64_t>(c.bias_samples > 0 ? c.bias_samples
                                                                                : 20));
        still->SetAttribute("still_travel_m", still_travel_m);
        output(motion, motion_id);
    } else {
        motion->SetAttribute("type", "tracking_wheel_motion");
        addWheels(motion);
        if (pico_imu && p.topology != gatr2::kTopologyThreeWheel) {
            // two wheels: the gyro is the heading constraint, counted once
            tinyxml2::XMLElement* hc = add(doc, motion, "HeadingConstraint");
            hc->SetAttribute("sensor_id", binding.imu.value.c_str());
            hc->SetAttribute("bias_samples", static_cast<int64_t>(c.bias_samples));
            hc->SetAttribute("window_ms", static_cast<int64_t>(window_ms));
            hc->SetAttribute("max_gap_ms", static_cast<int64_t>(c.max_gap_ms));
            hc->SetAttribute("max_calibration_travel_m", still_travel_m);
            stillness(hc);
            hc->SetAttribute("attempt_s", c.attempt_s);
            binding.bias_function = motion_id;
            if (p.topology == gatr2::kTopologyTwoForwardWheelImu) {
                add(doc, motion, "LateralMotion")->SetAttribute("assume", "zero");
            }
        }
        timing(motion);
        output(motion, motion_id);
    }

    const bool fused = pico_imu && p.topology == gatr2::kTopologyThreeWheel;
    if (fused) {
        // three wheels see rotation themselves; the gyro is an independent
        // heading observation fused once by the estimator
        const char*           heading_id = "profile_heading";
        tinyxml2::XMLElement* heading    = add(doc, localization, "Observation");
        heading->SetAttribute("id", heading_id);
        heading->SetAttribute("type", "imu_heading_increment");
        add(doc, heading, "Input")->SetAttribute("sensor_id", binding.imu.value.c_str());
        tinyxml2::XMLElement* cal = add(doc, heading, "Calibration");
        cal->SetAttribute("bias_samples", static_cast<int64_t>(c.bias_samples));
        cal->SetAttribute("max_gap_ms", static_cast<int64_t>(c.max_gap_ms));
        cal->SetAttribute("still_travel_m", still_travel_m);
        stillness(cal);
        cal->SetAttribute("attempt_s", c.attempt_s);
        // the profile wheels gate the gyro's stationary window
        for (uint8_t i = 0; i < p.wheel_count; ++i) {
            tinyxml2::XMLElement* w = add(doc, cal, "Wheel");
            w->SetAttribute("sensor_id", encoderSensorId(p.wheels[i].encoder_port).c_str());
            w->SetAttribute("radius_m", p.wheels[i].radius_um * 1e-6);
        }
        output(heading, heading_id);
        binding.bias_function = heading_id;

        tinyxml2::XMLElement* estimator = add(doc, localization, "Estimator");
        estimator->SetAttribute("type", "weighted_planar_fusion");
        tinyxml2::XMLElement* m = add(doc, estimator, "Motion");
        m->SetAttribute("observation_id", motion_id);
        tinyxml2::XMLElement* mn = add(doc, m, "Noise");
        mn->SetAttribute("translation_floor_m", c.translation_floor_m);
        mn->SetAttribute("translation_per_m", c.translation_per_m);
        mn->SetAttribute("rotation_floor_rad", c.rotation_floor_rad);
        mn->SetAttribute("rotation_per_rad", c.rotation_per_rad);
        mn->SetAttribute("rotation_per_m", c.rotation_per_m);
        tinyxml2::XMLElement* h = add(doc, estimator, "Heading");
        h->SetAttribute("observation_id", heading_id);
        h->SetAttribute("max_wait_ms", static_cast<int64_t>(c.max_wait_ms));
        tinyxml2::XMLElement* hn = add(doc, h, "Noise");
        hn->SetAttribute("angle_random_walk_rad_per_sqrt_s", c.angle_random_walk_rad_per_sqrt_s);
        hn->SetAttribute("bias_rad_per_s", c.bias_rad_per_s);
    } else {
        tinyxml2::XMLElement* estimator = add(doc, localization, "Estimator");
        estimator->SetAttribute("type", "planar_motion_integrator");
        add(doc, estimator, "Motion")->SetAttribute("observation_id", motion_id);
    }
    addHistory(c, doc, localization);
}

} // namespace navigatr
