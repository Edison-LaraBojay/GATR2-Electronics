// noops.h
// The explicit do-nothing implementation for every slot category, each with
// its own registered key and documented semantics; there is no universal
// noop callable. Selecting one in XML is a decision; omitting a slot is a
// configuration error. Perception and association noops exist for composite
// children as much as for slots.
//
//   commands/noop           carries the previous command state forward
//   estimator/noop          preserves the previous robot state
//   perception/noop         produces no observations
//   association/noop        produces no associations
//   field_estimation/noop   preserves the previous field, publishes no evidence
//   target_resolution/noop  preserves the previous target state
//   publishing/noop         publishes nothing, successfully

#pragma once
#include <memory>
#include <string>

#include "contracts/association.h"
#include "contracts/commands.h"
#include "contracts/field_estimation.h"
#include "contracts/localization.h"
#include "contracts/perception.h"
#include "contracts/publishing.h"
#include "contracts/target_resolution.h"

namespace navigatr
{

std::unique_ptr<Commands>         makeNoopCommands(const ConfigNode&,
                                                   SlotInitializationContext&, std::string&);
std::unique_ptr<StateEstimator>   makeNoopStateEstimator(const ConfigNode&,
                                                         StateEstimatorInitializationContext&,
                                                         std::string&);
std::unique_ptr<Perception>       makeNoopPerception(const ConfigNode&,
                                                     SlotInitializationContext&,
                                                     std::string&);
std::unique_ptr<Association>      makeNoopAssociation(const ConfigNode&,
                                                      SlotInitializationContext&,
                                                      std::string&);
std::unique_ptr<FieldEstimation>  makeNoopFieldEstimation(const ConfigNode&,
                                                          SlotInitializationContext&,
                                                          std::string&);
std::unique_ptr<TargetResolution> makeNoopTargetResolution(const ConfigNode&,
                                                           SlotInitializationContext&,
                                                           std::string&);
std::unique_ptr<Publishing>       makeNoopPublishing(const ConfigNode&,
                                                     SlotInitializationContext&,
                                                     std::string&);

} // namespace navigatr
