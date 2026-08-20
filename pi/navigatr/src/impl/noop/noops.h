// noops.h
// The explicit do-nothing implementation for every slot category, each with
// its own registered key and documented semantics; there is no universal
// noop callable. Selecting one in XML is a decision; omitting a slot is a
// configuration error. Perception and association noops exist for composite
// children as much as for slots.
//
//   commands/noop           carries the previous command state forward
//   preprocessing/noop      produces no artifacts
//   localization/noop       preserves the previous robot state
//   perception/noop         produces no observations
//   association/noop        produces no associations
//   world_estimation/noop   preserves the previous world, publishes no evidence
//   target_resolution/noop  preserves the previous target state
//   publishing/noop         publishes nothing, successfully

#pragma once
#include <memory>
#include <string>

#include "contracts/association.h"
#include "contracts/commands.h"
#include "contracts/localization.h"
#include "contracts/perception.h"
#include "contracts/preprocessing.h"
#include "contracts/publishing.h"
#include "contracts/target_resolution.h"
#include "contracts/world_estimation.h"

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
std::unique_ptr<WorldEstimation>  makeNoopWorldEstimation(const ConfigNode&,
                                                          SlotInitializationContext&,
                                                          std::string&);
std::unique_ptr<TargetResolution> makeNoopTargetResolution(const ConfigNode&,
                                                           SlotInitializationContext&,
                                                           std::string&);
std::unique_ptr<Publishing>       makeNoopPublishing(const ConfigNode&,
                                                     SlotInitializationContext&,
                                                     std::string&);

} // namespace navigatr
