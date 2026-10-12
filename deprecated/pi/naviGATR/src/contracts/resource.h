// resource.h
// The runtime half of a resource: named outputs polled once per cycle by the
// resource stage. A resource factory initializes its device or shared data,
// stores the object under its contract for direct binding, and attaches one
// of these when the resource produces measurements. Configuration-only
// resources (field map, wheel geometry, frames) attach nothing and never
// appear in the ResourceMap; consumers bind them directly at initialization.
//
// One execute call does the shared acquisition and decoding once and reports
// every declared output: a publication when the output has a new sample,
// otherwise its current health. The stage owns sequence, epoch, receipt time
// and retention; the resource only reports.

#pragma once
#include <functional>
#include <string>
#include <vector>

#include "core/execution_context.h"
#include "core/ids.h"
#include "core/payload_descriptor.h"
#include "core/records.h"

namespace navigatr
{

struct ResourceOutputDecl {
    OutputId          id;
    PayloadDescriptor payload;
};

struct OutputPoll {
    OutputId   id;
    PollResult result;
};

struct ResourcePollResult {
    SourceState             state = SourceState::kNoDataYet;   // the source as a whole
    std::string             diagnostic;
    std::vector<OutputPoll> outputs;   // declared outputs, one entry each
};

struct ResourceExecutable {
    std::vector<ResourceOutputDecl> outputs;

    // Empty when the resource has lifecycle hooks but nothing to poll.
    std::function<ResourcePollResult(const ExecutionContext&)> execute;

    // Back to power-on. Shared state resets here exactly once, never per
    // consumer.
    std::function<void()> reset;
};

} // namespace navigatr
