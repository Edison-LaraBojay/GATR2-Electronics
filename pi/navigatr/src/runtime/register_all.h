// register_all.h
// Explicit startup registration. Every register_* call adds factories to the
// same FunctionRegistry under namespaced keys; none of them create their own
// registry, and nothing registers through static initializers. Adding an
// implementation is one factory line in the right function.

#pragma once
#include "core/function_registry.h"

namespace navigatr
{

void register_resources(FunctionRegistry& functions);
void register_sensors(FunctionRegistry& functions);
void register_commands(FunctionRegistry& functions);
void register_preprocessing(FunctionRegistry& functions);
void register_localization(FunctionRegistry& functions);
void register_perception(FunctionRegistry& functions);
void register_association(FunctionRegistry& functions);
void register_pose_correction(FunctionRegistry& functions);
void register_world_prediction(FunctionRegistry& functions);
void register_publishers(FunctionRegistry& functions);

// All of the above, in order.
void registerAll(FunctionRegistry& functions);

} // namespace navigatr
