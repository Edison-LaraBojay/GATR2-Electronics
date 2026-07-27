// serial_links.cpp

#include "impl/resources/serial_links.h"

namespace navigatr
{

SerialReadResult LinuxSerialLink::readAvailable(MutableByteSpan destination) {
    const int n = port_.read(destination.data, static_cast<int>(destination.size));
    if (n < 0) {
        return SerialReadResult{0, true};
    }
    return SerialReadResult{static_cast<std::size_t>(n), false};
}

SerialWriteResult LinuxSerialLink::write(ByteSpan source) {
    return SerialWriteResult{port_.write(source.data, static_cast<int>(source.size))};
}

SerialReadResult MemoryLink::readAvailable(MutableByteSpan destination) {
    const int n = input_.read(destination.data, static_cast<int>(destination.size));
    return SerialReadResult{static_cast<std::size_t>(n < 0 ? 0 : n), false};
}

SerialWriteResult MemoryLink::write(ByteSpan source) {
    return SerialWriteResult{output_.write(source.data, static_cast<int>(source.size))};
}

SerialReadResult FileReplayLink::readAvailable(MutableByteSpan destination) {
    if (!source_.isOpen()) {
        return SerialReadResult{0, true};
    }
    const int n = source_.read(destination.data, static_cast<int>(destination.size));
    return SerialReadResult{static_cast<std::size_t>(n < 0 ? 0 : n), false};
}

ResourceValue make_linux_serial_link(const ConfigNode& node,
                                     ResourceInitializationContext& context,
                                     std::string& err) {
    const ConfigNode device = node.child("Device");
    const std::string path  = device.attr("path");
    if (path.empty()) {
        err = "linux_serial_link needs <Device path=.../>";
        return ResourceValue{};
    }
    long baud = 115200;
    if (!node.child("Baud").getInt("value", 115200, baud, err)) {
        return ResourceValue{};
    }

    auto        link = std::make_shared<LinuxSerialLink>();
    std::string open_err;
    if (!link->port().open(path, static_cast<int>(baud), open_err)) {
        if (context.warnings != nullptr) {
            context.warnings->push_back(node.path() + ": " + open_err);
        }
        // dead link; reads report closed, statuses show it downstream
    }
    return ResourceValue::asContract<SerialLink>(std::move(link));
}

ResourceValue make_memory_link(const ConfigNode&, ResourceInitializationContext&,
                               std::string&) {
    return ResourceValue::asContract<SerialLink>(std::make_shared<MemoryLink>());
}

ResourceValue make_file_replay_link(const ConfigNode& node,
                                    ResourceInitializationContext&, std::string& err) {
    const std::string path = node.child("File").attr("path");
    if (path.empty()) {
        err = "file_replay_link needs <File path=.../>";
        return ResourceValue{};
    }
    auto link = std::make_shared<FileReplayLink>();
    if (!link->open(path, err)) {
        return ResourceValue{};
    }
    return ResourceValue::asContract<SerialLink>(std::move(link));
}

ResourceMakeFunction fileReplayFactoryForPath(std::string path) {
    return [path = std::move(path)](const ConfigNode&, ResourceInitializationContext&,
                                    std::string& err) -> ResourceValue {
        auto link = std::make_shared<FileReplayLink>();
        if (!link->open(path, err)) {
            return ResourceValue{};
        }
        return ResourceValue::asContract<SerialLink>(std::move(link));
    };
}

} // namespace navigatr
