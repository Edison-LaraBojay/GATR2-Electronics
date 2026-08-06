// register_association.cpp

#include "impl/association/tag_mount_association.h"
#include "impl/noop/noops.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_association(FunctionRegistry& functions) {
    registerOrDie<AssociationMakeFunction>(functions, "noop", &makeNoopAssociation);
    registerOrDie<AssociationMakeFunction>(functions, "tag_mount_association",
                                           &TagMountAssociation::create);
}

} // namespace navigatr
