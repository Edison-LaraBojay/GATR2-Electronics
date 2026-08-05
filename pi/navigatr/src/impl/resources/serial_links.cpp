// serial_links.cpp

#include "impl/resources/serial_links.h"

#include <cstdio>

namespace navigatr
{

namespace
{

// sysfs GPIO write; simple and dependency free, good enough for one static
// enable line.
bool writeSysfs(const std::string& path, const std::string& value) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        return false;
    }
    const bool ok = std::fwrite(value.c_str(), 1, value.size(), f) == value.size();
    std::fclose(f);
    return ok;
}

bool setGpio(int gpio, bool high, std::string& err) {
    const std::string base = "/sys/class/gpio/gpio" + std::to_string(gpio);
    // export is allowed to fail when the pin is already exported
    writeSysfs("/sys/class/gpio/export", std::to_string(gpio));
    if (!writeSysfs(base + "/direction", "out") ||
        !writeSysfs(base + "/value", high ? "1" : "0")) {
        err = "cannot drive gpio " + std::to_string(gpio) + " via sysfs";
        return false;
    }
    return true;
}

} // namespace

LinuxSerialLink::~LinuxSerialLink() {
    if (driver_enable_gpio_ >= 0) {
        // release the bus; the pulldown keeps it idle when nobody drives it
        std::string ignored;
        setGpio(driver_enable_gpio_, false, ignored);
    }
}

bool LinuxSerialLink::enableDriver(int gpio, std::string& err) {
    if (!setGpio(gpio, true, err)) {
        return false;
    }
    driver_enable_gpio_ = gpio;
    return true;
}

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
        std::string gpio_err;
        if (!link->enableDriver(static_cast<int>(gpio), gpio_err) &&
            context.warnings != nullptr) {
            context.warnings->push_back(node.path() + ": " + gpio_err);
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
