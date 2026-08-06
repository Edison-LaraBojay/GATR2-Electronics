// calibration.h
// Calibration status policy. Configuration elements that carry measured
// geometry declare calibration_status; a value that was never measured must
// never silently run. UNCONFIGURED is always an error (it marks a template
// placeholder), provisional runs only when the build explicitly allows bench
// use, verified always runs.

#pragma once
#include <string>

#include "config/config_node.h"

namespace navigatr
{

enum class CalibrationStatus {
    kUnconfigured,
    kProvisional,
    kVerified,
};

// Reads the required calibration_status attribute and applies the policy.
// False and err when the attribute is missing, unknown, UNCONFIGURED, or
// provisional without allow_provisional.
inline bool checkCalibration(const ConfigNode& node, bool allow_provisional,
                             std::string& err) {
    std::string raw;
    if (!node.requireAttr("calibration_status", raw, err)) {
        return false;
    }
    if (raw == "verified") {
        return true;
    }
    if (raw == "provisional") {
        if (allow_provisional) {
            return true;
        }
        err = node.path() + ": calibration_status is provisional; run with "
              "--allow-provisional for bench use or verify the values";
        return false;
    }
    if (raw == "UNCONFIGURED") {
        err = node.path() + ": calibration_status is UNCONFIGURED; this is a "
              "template value that must be measured and configured before running";
        return false;
    }
    err = node.path() + ": calibration_status must be UNCONFIGURED, provisional, "
          "or verified, not \"" + raw + "\"";
    return false;
}

} // namespace navigatr
