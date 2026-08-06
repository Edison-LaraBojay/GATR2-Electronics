// robot_frame_map.cpp

#include "impl/resources/robot_frame_map.h"

#include <memory>
#include <vector>

#include "config/calibration.h"
#include "config/pose3_config.h"
#include "resources/robot_frames.h"

namespace navigatr
{

ResourceInstance make_robot_frame_map(const ConfigNode&              node,
                                      ResourceInitializationContext& context,
                                      std::string&                   err) {
    struct Pending {
        FrameId    id;
        FrameId    parent;
        Transform3 T_parent_child;
        bool       resolved = false;
    };
    std::vector<Pending> pending;

    bool ok = true;
    node.forEach("Frame", [&](const ConfigNode& f) {
        if (!ok) {
            return;
        }
        Pending p;
        std::string id_raw, parent_raw;
        if (!f.requireAttr("id", id_raw, err) ||
            !f.requireAttr("parent_frame_id", parent_raw, err)) {
            ok = false;
            return;
        }
        p.id     = FrameId{id_raw};
        p.parent = FrameId{parent_raw};
        if (p.id == robotBodyFrameId()) {
            err = f.path() + ": robot_body is the root frame and cannot be redefined";
            ok  = false;
            return;
        }
        for (const Pending& seen : pending) {
            if (seen.id == p.id) {
                err = f.path() + ": duplicate Frame id " + p.id.value;
                ok  = false;
                return;
            }
        }
        if (!checkCalibration(f, context.allow_provisional, err)) {
            ok = false;
            return;
        }
        const ConfigNode pose = f.child("PoseOfChildInParent");
        if (!pose.valid()) {
            err = f.path() + ": Frame needs PoseOfChildInParent";
            ok  = false;
            return;
        }
        if (!parseTransform3(pose, p.T_parent_child, err)) {
            ok = false;
            return;
        }
        pending.push_back(std::move(p));
    });
    if (!ok) {
        return ResourceInstance{};
    }

    auto map = std::make_shared<RobotFrameMap>();
    map->frames.emplace(robotBodyFrameId(), Transform3{});

    // Resolve chains to robot_body; a pass that resolves nothing while work
    // remains means a missing parent or a cycle.
    std::size_t remaining = pending.size();
    while (remaining > 0) {
        std::size_t resolved_this_pass = 0;
        for (Pending& p : pending) {
            if (p.resolved) {
                continue;
            }
            const Transform3* parent = map->find(p.parent);
            if (parent == nullptr) {
                continue;
            }
            map->frames.emplace(p.id, compose(*parent, p.T_parent_child));
            p.resolved = true;
            ++resolved_this_pass;
            --remaining;
        }
        if (resolved_this_pass == 0) {
            for (const Pending& p : pending) {
                if (!p.resolved) {
                    err = node.path() + ": Frame " + p.id.value +
                          " cannot resolve parent_frame_id " + p.parent.value +
                          " to robot_body (missing frame or a cycle)";
                    return ResourceInstance{};
                }
            }
        }
    }

    return ResourceInstance::asContract<const RobotFrameMap>(std::move(map));
}

} // namespace navigatr
