// register_publishers.cpp

#include "impl/noop/noops.h"
#include "impl/publishing/brain_link_publisher.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_publishers(FunctionRegistry& functions) {
    registerOrDie<PublishingMakeFunction>(functions, "noop", &makeNoopPublishing);
    registerOrDie<PublishingMakeFunction>(functions, "brain_link",
                                          &BrainLinkPublisher::create);
}

} // namespace navigatr
