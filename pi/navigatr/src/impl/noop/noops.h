// noops.h
// The explicit do-nothing implementation for every slot category, each with
// its own registered key and documented semantics; there is no universal
// noop callable. Selecting one in XML is a decision; omitting a slot is a
// configuration error.
//
//   preprocessing/noop     produces no artifacts
//   perception/noop        produces no observations
//   association/noop       produces no associations
//   pose_correction/noop   passes the predicted pose through unchanged
//   world_prediction/noop  preserves the previous world state
//   publishing/noop        publishes nothing, successfully
//   commands/noop          carries the previous command state forward
//   localization/noop      preserves the previous robot state

#pragma once
#include <memory>
#include <string>

#include "contracts/association.h"
#include "contracts/commands.h"
#include "contracts/localization.h"
#include "contracts/perception.h"
#include "contracts/pose_correction.h"
#include "contracts/preprocessing.h"
#include "contracts/publishing.h"
#include "contracts/world_prediction.h"

namespace navigatr
{

std::unique_ptr<Commands>       makeNoopCommands(const ConfigNode&,
                                                 SlotInitializationContext&, std::string&);
std::unique_ptr<Preprocessing>  makeNoopPreprocessing(const ConfigNode&,
                                                      PreprocessorInitializationContext&,
                                                      std::string&);
std::unique_ptr<Localization>   makeNoopLocalization(const ConfigNode&,
                                                     SlotInitializationContext&,
                                                     std::string&);
std::unique_ptr<Perception>     makeNoopPerception(const ConfigNode&,
                                                   SlotInitializationContext&,
                                                   std::string&);
std::unique_ptr<Association>    makeNoopAssociation(const ConfigNode&,
                                                    SlotInitializationContext&,
                                                    std::string&);
std::unique_ptr<PoseCorrection> makeNoopPoseCorrection(const ConfigNode&,
                                                       SlotInitializationContext&,
                                                       std::string&);
std::unique_ptr<WorldPrediction> makeNoopWorldPrediction(const ConfigNode&,
                                                         SlotInitializationContext&,
                                                         std::string&);
std::unique_ptr<Publishing>      makeNoopPublishing(const ConfigNode&,
                                                    SlotInitializationContext&,
                                                    std::string&);

} // namespace navigatr
