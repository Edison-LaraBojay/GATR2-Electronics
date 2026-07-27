// transport_set.h
// Named transports available to stages. The app owns the streams; stages
// look them up by the id in their config.

#pragma once
#include <map>
#include <string>

#include "transport/byte_stream.h"

namespace navigatr
{

struct TransportSet {
    std::map<std::string, ByteSource*> sources;
    std::map<std::string, ByteSink*>   sinks;

    ByteSource* source(const std::string& id) const {
        auto it = sources.find(id);
        return it == sources.end() ? nullptr : it->second;
    }

    ByteSink* sink(const std::string& id) const {
        auto it = sinks.find(id);
        return it == sinks.end() ? nullptr : it->second;
    }
};

} // namespace navigatr
