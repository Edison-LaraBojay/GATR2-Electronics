// fake_port.cpp

#include "receiver/sources/fakes/fake_port.h"

#include <algorithm>
#include <utility>

namespace spectagatr::receiver::sources::fakes
{

namespace
{

DurationUs microsecondsOf(Milliseconds duration) { return duration.count() * 1000; }

DurationUs& waitOf(PortStep& step) {
    if (Arrive* arrive = std::get_if<Arrive>(&step)) {
        return arrive->after_us;
    }
    return std::get<Unplug>(step).after_us;
}

} // namespace

ReceiverClock FakeClock::clock() {
    return [this] { return now_us_; };
}

ScriptedPort::ScriptedPort(FakeClock& clock, PortSteps steps, std::string name, SourceKind kind)
    : clock_(clock), steps_(std::move(steps)), name_(std::move(name)), kind_(kind) {}

Port ScriptedPort::port() {
    Port port;
    port.read = [this](Byte* destination, std::size_t capacity, Milliseconds wait) {
        return read(destination, capacity, wait);
    };
    port.write = [this](const Byte* source, std::size_t size, Milliseconds) {
        return write(source, size);
    };
    port.kind = kind_;
    port.path = name_;
    port.name = name_;
    return port;
}

IoResult ScriptedPort::read(Byte* destination, std::size_t capacity, Milliseconds wait) {
    ++reads_;
    IoResult result;
    if (closed_) {
        result.status = IoStatus::kClosed;
        result.error  = "closed";
        return result;
    }
    if (steps_.empty()) {
        if (on_end_) {
            on_end_();
        }
        clock_.advance(microsecondsOf(wait));
        result.status = IoStatus::kTimedOut;
        return result;
    }
    DurationUs& after_us = waitOf(steps_.front());
    if (after_us > microsecondsOf(wait)) {
        after_us -= microsecondsOf(wait);
        clock_.advance(microsecondsOf(wait));
        result.status = IoStatus::kTimedOut;
        return result;
    }
    clock_.advance(after_us);
    after_us = 0;
    if (Unplug* unplug = std::get_if<Unplug>(&steps_.front())) {
        closed_       = true;
        result.status = IoStatus::kClosed;
        result.error  = unplug->reason;
        steps_.pop_front();
        return result;
    }
    Bytes&            bytes = std::get<Arrive>(steps_.front()).bytes;
    const std::size_t count = std::min(capacity, bytes.size());
    std::copy_n(bytes.begin(), count, destination);
    bytes.erase(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(count));
    if (bytes.empty()) {
        steps_.pop_front();
    }
    result.bytes = count;
    return result;
}

IoResult ScriptedPort::write(const Byte* source, std::size_t size) {
    IoResult result;
    if (closed_) {
        result.status = IoStatus::kClosed;
        result.error  = "closed";
        return result;
    }
    written_.insert(written_.end(), source, source + size);
    result.bytes = size;
    return result;
}

} // namespace spectagatr::receiver::sources::fakes
