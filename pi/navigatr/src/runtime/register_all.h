// register_all.h
// Explicit startup registration. Each register_* function lives beside the
// implementations it registers (impl/resources/register_resources.cpp and so
// on) and adds factories to the same FunctionRegistry under opaque names
// like pico_encoder_channel; none of them create their own registry, and
// nothing registers through static initializers. Every category registers
// its own noop under the name noop, scoped by its factory signature.
//
// A registration collision at startup is a programming error and aborts
// loudly rather than silently keeping the first function.

#pragma once
#include <cstdio>
#include <cstdlib>

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

// Used by the register_* implementations: a duplicate registration is fatal.
template <typename Fn>
inline void registerOrDie(FunctionRegistry& functions, const char* key, Fn fn) {
    if (!functions.add<Fn>(FunctionKey{key}, std::move(fn))) {
        std::fprintf(stderr, "duplicate function registration: %s\n", key);
        std::abort();
    }
}

} // namespace navigatr
