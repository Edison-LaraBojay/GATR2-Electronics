// register_commands.cpp

#include "impl/commands/brain_link_commands.h"
#include "impl/noop/noops.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_commands(FunctionRegistry& functions) {
    registerOrDie<CommandsMakeFunction>(functions, "noop", &makeNoopCommands);
    registerOrDie<CommandsMakeFunction>(functions, "brain_link", &BrainLinkCommands::create);
}

} // namespace navigatr
