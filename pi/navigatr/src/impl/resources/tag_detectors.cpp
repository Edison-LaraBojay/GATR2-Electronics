// tag_detectors.cpp

#include "impl/resources/tag_detectors.h"

#include "resources/tag_detector.h"

namespace navigatr
{

ResourceInstance make_apriltag_detector(const ConfigNode& node,
                                        ResourceInitializationContext&,
                                        std::string& err) {
    bool        any = false;
    bool        ok  = true;
    node.forEach("Family", [&](const ConfigNode& fam) {
        if (!ok) {
            return;
        }
        std::string name;
        double      size = 0.0;
        if (!fam.requireAttr("name", name, err) ||
            !fam.requireDouble("detection_size_m", size, err)) {
            ok = false;
            return;
        }
        if (size <= 0.0) {
            err = fam.path() + ": detection_size_m must be positive";
            ok  = false;
            return;
        }
        any = true;
    });
    if (!ok) {
        return ResourceInstance{};
    }
    if (!any) {
        err = node.path() + ": needs at least one <Family name=... detection_size_m=.../>";
        return ResourceInstance{};
    }

    err = node.path() + ": the upstream AprilTag detector is not built into this "
          "binary yet; it lands with camera bring-up";
    return ResourceInstance{};
}

} // namespace navigatr
