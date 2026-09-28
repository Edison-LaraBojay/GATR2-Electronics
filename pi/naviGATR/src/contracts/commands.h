// commands.h
// Command Collection slot: brain intent in, updated command state out. When
// no update is available the previous state carries forward unchanged; no
// update is not an empty command. Timeout behavior, if wanted, is explicit
// configuration owned by the selected implementation. The no-op is exactly a
// bench with no brain attached.

#pragma once
#include <functional>
#include <memory>

#include "config/config_node.h"
#include "contracts/slot_init.h"
#include "core/diagnostics.h"
#include "core/function_status.h"
#include "core/time.h"
#include "state/command_state.h"

namespace navigatr
{

struct CommandsInput {
    const CommandState& previous;
    MonotonicTime       now;   // host clock
    Diagnostics*        diagnostics = nullptr;
};

struct CommandsOutput {
    CommandState   command;
    FunctionStatus status = FunctionStatus::kOk;
};

class Commands
{
public:
    virtual ~Commands() = default;

    virtual CommandsOutput run(const CommandsInput& in) = 0;

    virtual void reset() {}
};

using CommandsMakeFunction = std::function<std::unique_ptr<Commands>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
