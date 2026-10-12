// noops.h
// The explicit do-nothing implementation for every slot category, each with
// its own registered key and documented semantics; there is no universal
// noop callable. Selecting one in XML is a decision; omitting a slot is a
// configuration error.
//
//   commands/noop           carries the previous command state forward
//   estimator/noop          preserves the previous robot state
//   world_estimation/noop   preserves the previous field, publishes no evidence
//   target_resolution/noop  preserves the previous target state
//   publishing/noop         publishes nothing, successfully

#pragma once
#include <memory>
#include <string>

#include "contracts/commands.h"
#include "contracts/field_estimation.h"
#include "contracts/localization.h"
#include "contracts/publishing.h"
#include "contracts/target_resolution.h"

namespace navigatr
{

std::unique_ptr<Commands>         makeNoopCommands(const ConfigNode&,
                                                   SlotInitializationContext&, std::string&);
std::unique_ptr<StateEstimator>   makeNoopStateEstimator(const ConfigNode&,
                                                         StateEstimatorInitializationContext&,
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
