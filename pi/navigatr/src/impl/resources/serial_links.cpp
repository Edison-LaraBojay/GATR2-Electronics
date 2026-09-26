// serial_links.cpp

#include "impl/resources/serial_links.h"

namespace navigatr
{

namespace
{

SerialWriteResult windowMissed() {
    SerialWriteResult result;
    result.expired = true;
    result.error   = "transmit window missed";
    return result;
}

SerialWriteResult writeFailed(const char* why) {
    SerialWriteResult result;
    result.error = why;
    return result;
}

} // namespace

bool LinuxSerialLink::enableHalfDuplex(int gpio, const HalfDuplexTiming& timing,
                                       std::string& err) {
    half_duplex_ = true;
    timing_      = timing;
    return port_.openDriverEnable(gpio, err);
}

SerialReadResult LinuxSerialLink::readAvailable(MutableByteSpan destination) {
    const int n = port_.read(destination.data, static_cast<int>(destination.size));
    if (n < 0) {
        return SerialReadResult{0, true};
    }
    return SerialReadResult{static_cast<std::size_t>(n), false};
}

SerialWriteResult LinuxSerialLink::write(ByteSpan source) {
    return write(source, TransmitWindow{});
}

SerialWriteResult LinuxSerialLink::write(ByteSpan source, const TransmitWindow& window) {
    if (!port_.isOpen()) {
        return writeFailed("serial port not open");
    }
    if (half_duplex_) {
        if (!port_.driverEnableOpen()) {
            return writeFailed("driver enable gpio unavailable");
        }
        return transmitHalfDuplex(port_, source, window, timing_);
    }
    if (!waitForWindow(port_, window)) {
        return windowMissed();
    }
    if (!port_.write(source.data, static_cast<int>(source.size))) {
        return writeFailed("write failed");
    }
    return SerialWriteResult{true};
}

SerialReadResult MemoryLink::readAvailable(MutableByteSpan destination) {
    const int n = input_.read(destination.data, static_cast<int>(destination.size));
    return SerialReadResult{static_cast<std::size_t>(n < 0 ? 0 : n), false};
}

SerialWriteResult MemoryLink::write(ByteSpan source) {
    return SerialWriteResult{output_.write(source.data, static_cast<int>(source.size))};
}

SerialWriteResult MemoryLink::write(ByteSpan source, const TransmitWindow& window) {
    if (window.missed(nowUs())) {
        return windowMissed();
    }
    if (inputPending()) {
        SerialWriteResult result;
        result.input_pending = true;
        result.error         = "input pending";
        return result;
    }
    return write(source);
}

SerialReadResult FileReplayLink::readAvailable(MutableByteSpan destination) {
    if (!source_.isOpen()) {
        return SerialReadResult{0, true};
    }
    const int n = source_.read(destination.data, static_cast<int>(destination.size));
    return SerialReadResult{static_cast<std::size_t>(n < 0 ? 0 : n), false};
}

ResourceInstance make_linux_serial_link(const ConfigNode& node,
                                     ResourceInitializationContext& context,
                                     std::string& err) {
    const ConfigNode device = node.child("Device");
    const std::string path  = device.attr("path");
    if (path.empty()) {
        err = "linux_serial_link needs <Device path=.../>";
        return ResourceInstance{};
    }
    long baud = 115200;
    if (!node.child("Baud").getInt("value", 115200, baud, err)) {
        return ResourceInstance{};
    }

    auto        link = std::make_shared<LinuxSerialLink>();
    std::string open_err;
    if (!link->port().open(path, static_cast<int>(baud), open_err)) {
        if (context.warnings != nullptr) {
            context.warnings->push_back(node.path() + ": " + open_err);
        }
        // dead link; reads report closed, statuses show it downstream
    }

    const ConfigNode driver_enable = node.child("DriverEnable");
    if (driver_enable.valid()) {
        long gpio = -1;
        if (!driver_enable.getInt("gpio", -1, gpio, err)) {
            return ResourceInstance{};
        }
        if (gpio < 0) {
            err = driver_enable.path() + ": DriverEnable needs gpio";
            return ResourceInstance{};
        }
        if (baud <= 0) {
            err = node.path() + ": half duplex needs a positive Baud";
            return ResourceInstance{};
        }
        HalfDuplexTiming timing;
        timing.baud = static_cast<int>(baud);
        long guard  = static_cast<long>(2 * characterTimeUs(timing.baud));
        long margin = static_cast<long>(timing.margin_us);
        if (!driver_enable.getInt("post_guard_us", guard, guard, err) ||
            !driver_enable.getInt("tx_margin_us", margin, margin, err)) {
            return ResourceInstance{};
        }
        if (guard < 0 || margin < 0) {
            err = driver_enable.path() + ": post_guard_us and tx_margin_us must be >= 0";
            return ResourceInstance{};
        }
        timing.post_guard_us = guard;
        timing.margin_us     = margin;

        std::string gpio_err;
        if (!link->enableHalfDuplex(static_cast<int>(gpio), timing, gpio_err) &&
            context.warnings != nullptr) {
            context.warnings->push_back(node.path() + ": " + gpio_err +
                                        "; half-duplex writes will fail");
        }
    }
    return ResourceInstance::asContract<SerialLink>(std::move(link));
}

ResourceInstance make_memory_link(const ConfigNode&, ResourceInitializationContext&,
                               std::string&) {
    return ResourceInstance::asContract<SerialLink>(std::make_shared<MemoryLink>());
}

ResourceInstance make_file_replay_link(const ConfigNode& node,
                                    ResourceInitializationContext&, std::string& err) {
    const std::string path = node.child("File").attr("path");
    if (path.empty()) {
        err = "file_replay_link needs <File path=.../>";
        return ResourceInstance{};
    }
    auto link = std::make_shared<FileReplayLink>();
    if (!link->open(path, err)) {
        return ResourceInstance{};
    }
    return ResourceInstance::asContract<SerialLink>(std::move(link));
}

ResourceMakeFunction fileReplayFactoryForPath(std::string path) {
    return [path = std::move(path)](const ConfigNode&, ResourceInitializationContext&,
                                    std::string& err) -> ResourceInstance {
        auto link = std::make_shared<FileReplayLink>();
        if (!link->open(path, err)) {
            return ResourceInstance{};
        }
        return ResourceInstance::asContract<SerialLink>(std::move(link));
    };
}

} // namespace navigatr
