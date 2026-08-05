// landmark_map.cpp

#include "impl/world_prediction/landmark_map.h"

#include <typeindex>

#include "payloads/landmark_associations.h"
#include "resources/resource_map.h"

namespace navigatr
{

std::unique_ptr<WorldPrediction> LandmarkMapWorldPrediction::create(
    const ConfigNode& node, SlotInitializationContext& context, std::string& err) {
    auto wp = std::make_unique<LandmarkMapWorldPrediction>();

    const ResourceId map_id{node.child("FieldMap").attr("resource_id")};
    if (map_id.empty()) {
        err = node.path() + ": needs <FieldMap resource_id=.../>";
        return nullptr;
    }
    if (context.resources == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }
    std::string inner;
    wp->map_ = context.resources->require<const FieldMap>(map_id, inner);
    if (wp->map_ == nullptr) {
        err = node.path() + ": " + inner;
        return nullptr;
    }

    const ConfigNode input = node.child("Input");
    if (input.valid() && input.hasAttr("association_id")) {
        wp->association_ref_ = AssociationId{input.attr("association_id")};
        const std::type_index expected(typeid(LandmarkAssociationSet));
        if (!context.requireAssociation(wp->association_ref_, &expected, node.path(),
                                        err)) {
            return nullptr;
        }
    }

    if (!node.getDouble("blend", 1.0, wp->blend_, err)) {
        return nullptr;
    }
    if (wp->blend_ < 0.0 || wp->blend_ > 1.0) {
        err = node.path() + ": blend must be within 0..1";
        return nullptr;
    }
    return wp;
}

WorldPredictionOutput LandmarkMapWorldPrediction::run(const WorldPredictionInput& in) {
    WorldPredictionOutput out;
    out.world = in.previousWorld;

    for (const LandmarkDecl& decl : map_->landmarks) {
        if (out.world.objects.find(decl.id) != out.world.objects.end()) {
            continue;
        }
        WorldObject obj;
        obj.pose.frame      = FrameId{"field"};
        obj.pose.pose       = decl.nominal;
        obj.pose.measuredAt = in.now;
        obj.confidence      = 0.5;
        obj.valid           = true;
        obj.source          = EstimateSource::kFieldMap;
        out.world.objects.emplace(decl.id, obj);
    }

    for (auto& kv : out.world.objects) {
        kv.second.observed = false;
    }

    if (!association_ref_.empty()) {
        const auto it = in.associations.find(association_ref_);
        if (it != in.associations.end()) {
            const LandmarkAssociationSet* set =
                it->second.payload.get<LandmarkAssociationSet>();
            if (set == nullptr) {
                out.status = FunctionStatus::kFault;
                return out;
            }
            for (const LandmarkAssociationEntry& entry : set->entries) {
                WorldObject& obj = out.world.objects[entry.landmark];
                if (!obj.valid) {
                    // first sighting of an object the map did not know
                    obj.pose.frame    = FrameId{"field"};
                    obj.pose.pose.x_m = entry.field_x_m;
                    obj.pose.pose.y_m = entry.field_y_m;
                } else {
                    obj.pose.pose.x_m += (entry.field_x_m - obj.pose.pose.x_m) * blend_;
                    obj.pose.pose.y_m += (entry.field_y_m - obj.pose.pose.y_m) * blend_;
                }
                // bearing-only observation, heading keeps its map value
                obj.pose.measuredAt = in.now;
                obj.valid           = true;
                obj.observed        = true;
                obj.source          = EstimateSource::kObserved;
                obj.confidence      = entry.quality;
            }
        }
    }

    return out;
}

} // namespace navigatr
