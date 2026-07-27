// landmark_map.h
// World estimates anchored to a field map resource. Every landmark starts at
// its nominal map pose; associated observations pull the matched object
// toward where it was actually seen. An object not seen keeps its last
// estimate and stays valid. blend sets how far an estimate moves toward a
// new measurement; out of range is a configuration error, not a clamp.
//
//   <WorldPrediction type="world_prediction/landmark_map" blend="1.0">
//       <FieldMap resource_id="override_field"/>
//       <Input association_id="landmarks"/>   optional
//   </WorldPrediction>

#pragma once
#include <memory>
#include <string>

#include "config/field_map.h"
#include "contracts/world_prediction.h"

namespace navigatr
{

class LandmarkMapWorldPrediction : public WorldPrediction
{
public:
    static std::unique_ptr<WorldPrediction> create(const ConfigNode& node,
                                                   SlotInitializationContext& context,
                                                   std::string& err);

    WorldPredictionOutput run(const WorldPredictionInput& in) override;

private:
    std::shared_ptr<const FieldMap> map_;
    AssociationId                   association_ref_;   // empty means map only
    double                          blend_ = 1.0;
};

} // namespace navigatr
