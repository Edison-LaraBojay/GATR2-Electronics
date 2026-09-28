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
    // a hung-up tty reads 0 like an idle one; only poll tells them apart
    if (n == 0 && !port_.waitWritable(0)) {
        return SerialReadResult{0, true};
    }
    return SerialReadResult{static_cast<std::size_t>(n), false};
}

ReopeningLink::ReopeningLink(Opener open, std::shared_ptr<SerialLink> device, int64_t retry_us)
    : open_(std::move(open)), device_(std::move(device)), retry_us_(retry_us) {
    last_attempt_us_ = retryNowUs();
}

int64_t ReopeningLink::retryNowUs() const { return clock_ ? clock_() : steadyNowUs(); }

void ReopeningLink::setClock(std::function<int64_t()> now_us) {
    clock_           = std::move(now_us);
    last_attempt_us_ = retryNowUs();
}

int64_t ReopeningLink::nowUs() { return device_ != nullptr ? device_->nowUs() : retryNowUs(); }

bool ReopeningLink::retry() {
    const int64_t now = retryNowUs();
    if (now - last_attempt_us_ < retry_us_) {
        return false;
    }
    last_attempt_us_ = now;
    ++attempts_;
    std::string err;
    device_     = open_(err);
    last_error_ = err;   // with a device: a driver enable warning, if any
    if (device_ == nullptr) {
        return false;
    }
    ++reopens_;
    return true;
}

SerialReadResult ReopeningLink::readAvailable(MutableByteSpan destination) {
    if (device_ == nullptr && !retry()) {
        return SerialReadResult{0, true};
    }
    const SerialReadResult read = device_->readAvailable(destination);
    if (read.closed) {
        device_.reset();   // closes the descriptor now; reopening follows the schedule
        ++closes_;
        return SerialReadResult{0, true};
    }
    return read;
}

SerialWriteResult ReopeningLink::write(ByteSpan source) {
    if (device_ == nullptr) {
        return writeFailed("serial device not open");
    }
    return device_->write(source);
}

SerialWriteResult ReopeningLink::write(ByteSpan source, const TransmitWindow& window) {
    if (device_ == nullptr) {
        return writeFailed("serial device not open");
    }
    return device_->write(source, window);
}

bool ReopeningLink::inputPending() { return device_ != nullptr && device_->inputPending(); }

SerialWriteResult LinuxSerialLink::write(ByteSpan source) {
    return write(source, TransmitWindow{});
}

SerialWriteResult LinuxSerialLink::write(ByteSpan source, const TransmitWindow& window) {
    if (half_duplex_ && !port_.driverEnableOpen()) {
        return writeFailed("driver enable gpio unavailable");
    }
    if (!port_.isOpen()) {
        return writeFailed("serial port not open");
    }
    if (half_duplex_) {
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
    bool required = false;
    long baud = 115200;
    if (!device.getBool("required", false, required, err) ||
        !node.child("Baud").getInt("value", 115200, baud, err)) {
        return ResourceInstance{};
    }

    auto        link = std::make_shared<LinuxSerialLink>();
    std::string open_err;
    const bool  opened = link->port().open(path, static_cast<int>(baud), open_err);
    if (!opened) {
        if (required) {
            err = device.path() + ": required serial device unavailable: " + open_err;
            return ResourceInstance{};
        }
        if (context.warnings != nullptr) {
            context.warnings->push_back(node.path() + ": " + open_err);
        }
        // closed link; reads report closed until a retry opens the device
    }

    bool             half_duplex = false;
    long             gpio        = -1;
    HalfDuplexTiming timing;
    const ConfigNode driver_enable = node.child("DriverEnable");
    if (driver_enable.valid()) {
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
        half_duplex          = true;

        // driven low even when the device did not open: the transceiver listens
        std::string gpio_err;
        if (!link->enableHalfDuplex(static_cast<int>(gpio), timing, gpio_err)) {
            if (required) {
                err = driver_enable.path() + ": required serial driver enable unavailable: " +
                      gpio_err;
                return ResourceInstance{};
            }
            if (context.warnings != nullptr) {
                context.warnings->push_back(node.path() + ": " + gpio_err +
                                            "; half-duplex writes will fail");
            }
        }
    }

    // every reopen repeats the device and driver enable setup
    ReopeningLink::Opener reopen = [path, baud, half_duplex, gpio,
                                    timing](std::string& why) -> std::shared_ptr<SerialLink> {
        auto next = std::make_shared<LinuxSerialLink>();
        if (!next->port().open(path, static_cast<int>(baud), why)) {
            return nullptr;
        }
        std::string gpio_err;
        if (half_duplex && !next->enableHalfDuplex(static_cast<int>(gpio), timing, gpio_err)) {
            why = gpio_err + "; half-duplex writes will fail";
        }
        return next;
    };
    return ResourceInstance::asContract<SerialLink>(std::make_shared<ReopeningLink>(
        std::move(reopen), opened ? std::shared_ptr<SerialLink>(std::move(link)) : nullptr));
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
