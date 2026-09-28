// capture_config.h
// The optional <Capture> element under <System>:
//
//   <Capture enabled="true" rolling_s="10" max_pre_s="30" max_post_s="60"
//            default_pre_s="10" default_post_s="15" max_records="400000"
//            max_mb="64" keep="3" directory="" auto=""
//            auto_cooldown_s="120" auto_max_per_hour="6"/>
//
// Absent means enabled with these defaults. The pre-trigger interval comes
// from the rolling window, so a capture never holds more than rolling_s
// before its trigger whatever pre_s asks for. directory empty keeps bundles
// in memory only; auto lists fault triggers (link_lost, continuity_lost,
// pico_reboot, profile_changed), empty for none.

#pragma once
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

#include "config/config_node.h"

namespace navigatr
{

enum class CaptureFault : uint8_t {
    kLinkLost       = 0, // Brain requests stopped, or Pico frames lost
    kContinuityLost = 1, // localization ended pose continuity
    kPicoReboot     = 2, // a new Pico boot id
    kProfileChanged = 3, // a Brain robot profile was applied
};
constexpr int kCaptureFaultCount = 4;

inline const char* captureFaultName(CaptureFault f) {
    switch (f) {
    case CaptureFault::kLinkLost: return "link_lost";
    case CaptureFault::kContinuityLost: return "continuity_lost";
    case CaptureFault::kPicoReboot: return "pico_reboot";
    case CaptureFault::kProfileChanged: return "profile_changed";
    }
    return "unknown";
}

constexpr uint32_t captureFaultBit(CaptureFault f) { return 1u << static_cast<uint8_t>(f); }

struct CaptureConfig {
    bool        enabled           = true;
    double      rolling_s         = 10.0;
    double      max_pre_s         = 30.0;
    double      max_post_s        = 60.0;
    double      default_pre_s     = 10.0;
    double      default_post_s    = 15.0;
    long        max_records       = 400000;
    long        max_mb            = 64;
    long        keep              = 3;
    std::string directory;
    uint32_t    auto_faults       = 0; // captureFaultBit mask
    double      auto_cooldown_s   = 120.0;
    long        auto_max_per_hour = 6;
};

inline bool parseCaptureFaults(const std::string& list, uint32_t& mask, std::string& bad) {
    mask = 0;
    std::stringstream in(list);
    std::string       item;
    while (std::getline(in, item, ',')) {
        const auto b = item.find_first_not_of(" \t");
        const auto e = item.find_last_not_of(" \t");
        if (b == std::string::npos) {
            continue;
        }
        item      = item.substr(b, e - b + 1);
        bool seen = false;
        for (int i = 0; i < kCaptureFaultCount; ++i) {
            const auto f = static_cast<CaptureFault>(i);
            if (item == captureFaultName(f)) {
                mask |= captureFaultBit(f);
                seen = true;
            }
        }
        if (!seen) {
            bad = item;
            return false;
        }
    }
    return true;
}

inline bool parseCaptureConfig(const ConfigNode& node, CaptureConfig& out, std::string& err) {
    out = CaptureConfig{};
    if (!node.valid()) {
        return true;
    }
    if (!node.onlyAttributes({"enabled", "rolling_s", "max_pre_s", "max_post_s", "default_pre_s",
                              "default_post_s", "max_records", "max_mb", "keep", "directory",
                              "auto", "auto_cooldown_s", "auto_max_per_hour"},
                             err) ||
        !node.onlyChildren({}, err)) {
        return false;
    }
    if (!node.getBool("enabled", out.enabled, out.enabled, err) ||
        !node.getDouble("rolling_s", out.rolling_s, out.rolling_s, err) ||
        !node.getDouble("max_pre_s", out.max_pre_s, out.max_pre_s, err) ||
        !node.getDouble("max_post_s", out.max_post_s, out.max_post_s, err) ||
        !node.getDouble("default_pre_s", out.default_pre_s, out.default_pre_s, err) ||
        !node.getDouble("default_post_s", out.default_post_s, out.default_post_s, err) ||
        !node.getInt("max_records", out.max_records, out.max_records, err) ||
        !node.getInt("max_mb", out.max_mb, out.max_mb, err) ||
        !node.getInt("keep", out.keep, out.keep, err) ||
        !node.getDouble("auto_cooldown_s", out.auto_cooldown_s, out.auto_cooldown_s, err) ||
        !node.getInt("auto_max_per_hour", out.auto_max_per_hour, out.auto_max_per_hour, err)) {
        return false;
    }
    out.directory = node.attr("directory");
    std::string bad;
    if (!parseCaptureFaults(node.attr("auto"), out.auto_faults, bad)) {
        err = node.path() + ": auto has unknown trigger '" + bad +
              "' (use link_lost, continuity_lost, pico_reboot, profile_changed)";
        return false;
    }
    // bounded on purpose: a capture is a short diagnostic, never a log
    const auto range = [&](const char* name, double v, double lo, double hi) {
        if (v < lo || v > hi) {
            std::ostringstream m;
            m << node.path() << ": " << name << " must be in [" << lo << ", " << hi << "]";
            err = m.str();
            return false;
        }
        return true;
    };
    return range("rolling_s", out.rolling_s, 0.0, 120.0) &&
           range("max_pre_s", out.max_pre_s, 0.0, 120.0) &&
           range("max_post_s", out.max_post_s, 1.0, 600.0) &&
           range("default_pre_s", out.default_pre_s, 0.0, out.max_pre_s) &&
           range("default_post_s", out.default_post_s, 1.0, out.max_post_s) &&
           range("max_records", static_cast<double>(out.max_records), 1000.0, 5000000.0) &&
           range("max_mb", static_cast<double>(out.max_mb), 1.0, 1024.0) &&
           range("keep", static_cast<double>(out.keep), 1.0, 20.0) &&
           range("auto_cooldown_s", out.auto_cooldown_s, 10.0, 86400.0) &&
           range("auto_max_per_hour", static_cast<double>(out.auto_max_per_hour), 1.0, 60.0);
}

} // namespace navigatr
