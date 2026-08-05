// register_commands.cpp

#include "impl/commands/vex_brain_serial.h"
#include "impl/noop/noops.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_commands(FunctionRegistry& functions) {
    registerOrDie<CommandsMakeFunction>(functions, "noop", &makeNoopCommands);
    registerOrDie<CommandsMakeFunction>(functions, "vex_brain_serial",
                                        &VexBrainSerialCommands::create);
}

} // namespace navigatr
