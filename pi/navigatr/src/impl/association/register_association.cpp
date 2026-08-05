// register_association.cpp
// Tag instance association lands here when perception exists.

#include "impl/noop/noops.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_association(FunctionRegistry& functions) {
    registerOrDie<AssociationMakeFunction>(functions, "noop", &makeNoopAssociation);
}

} // namespace navigatr
