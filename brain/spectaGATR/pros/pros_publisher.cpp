// pros_publisher.cpp
// PROS only.
//
// PROS 4.2.2 stdout: write(STDOUT_FILENO) goes straight to the serial driver
// (no stdio FILE buffer), which sends one call as one COBS packet "sout" +
// bytes, or the raw bytes with COBS off, under one write mutex; a blocking
// write waits until the whole packet fits in the 2047 B output buffer
// (src/system/dev/ser_driver.c ser_write_r). Packet = line + 6 B (stream id,
// COBS code byte, 0x00); lines from this publisher are 88 B at most (the
// protocol allows 120).

#include "spectagatr/pros_publisher.h"

#include <algorithm>
#include <cstring>
#include <unistd.h>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

// VEXos SDK, exported by libpros.a.
extern "C" int32_t vexSerialWriteFree(uint32_t channel);

namespace spectagatr
{
namespace
{

constexpr uint32_t kPollMs     = 5;
constexpr uint32_t kPriority   = TASK_PRIORITY_DEFAULT - 1;
constexpr char     kTaskName[] = "spectagatr";

// VEXos stdio FIFO (channel 1). A line is written only while at most about two
// packets wait in it, so a slow reader (controller radio) gets the newest pose
// instead of a queue of old ones, and with no reader the task stops writing
// instead of blocking in write(). The empty size is the largest free count seen,
// not assumed: PROS expects 2048, but a smaller FIFO must not close the gate for
// good. A count that never moves leaves the gate open, as before the gate.
constexpr uint32_t kStdioChannel   = 1;
constexpr int32_t  kMaxQueuedBytes = 128;

bool outputHasRoom(int32_t& empty_size) {
    const int32_t free = vexSerialWriteFree(kStdioChannel);
    if (free < 0) {
        return true;
    }
    empty_size = std::max(empty_size, free);
    return empty_size - free <= kMaxQueuedBytes;
}

Milliseconds prosMillis() { return pros::millis(); }

uint64_t bits(double value) {
    uint64_t out = 0;
    std::memcpy(&out, &value, sizeof(out));
    return out;
}

// Best effort: the boot timing in microseconds and battery readings differ
// from run to run; nothing guarantees two runs never share an id.
SessionId newSessionId() {
    const uint64_t inputs[] = {
        pros::micros(),
        static_cast<uint64_t>(pros::battery::get_voltage()),
        static_cast<uint64_t>(pros::battery::get_current()),
        bits(pros::battery::get_temperature()),
        bits(pros::battery::get_capacity()),
        pros::micros(),
    };
    return makeSessionId(inputs, sizeof(inputs) / sizeof(inputs[0]));
}

} // namespace

ProsPublisher::ProsPublisher(Hertz rate) : publisher_(&prosMillis, rate) {}

bool ProsPublisher::start() {
    if (!task_created_.exchange(true)) {
        publisher_.begin(newSessionId());
        if (pros::c::task_create(&ProsPublisher::run, this, kPriority, TASK_STACK_DEPTH_DEFAULT,
                                 kTaskName) == nullptr) {
            task_created_.store(false);
            return false;
        }
    }
    enabled_.store(true);
    return true;
}

void ProsPublisher::stop() { enabled_.store(false); }

bool ProsPublisher::running() const { return enabled_.load() && task_created_.load(); }

void ProsPublisher::updatePose(Meters x, Meters y, Radians heading) {
    publisher_.updatePose(x, y, heading);
}

void ProsPublisher::updatePose(Meters x, Meters y, const Attitude& attitude) {
    publisher_.updatePose(x, y, attitude);
}

void ProsPublisher::resetTrail() { publisher_.resetTrail(); }

void ProsPublisher::setRate(Hertz rate) { publisher_.setRate(rate); }

Hertz ProsPublisher::rate() const { return publisher_.rate(); }

PublisherStats ProsPublisher::stats() const { return publisher_.stats(); }

SessionId ProsPublisher::session() const { return publisher_.session(); }

void ProsPublisher::run(void* self) {
    ProsPublisher& p = *static_cast<ProsPublisher*>(self);
    char           line[kLineCapacity];
    int32_t        empty_size = 0;
    while (true) {
        if (p.enabled_.load() && outputHasRoom(empty_size)) {
            const std::size_t length = p.publisher_.nextLine(pros::millis(), line, sizeof(line));
            if (length > 0) {
                const ssize_t written = ::write(STDOUT_FILENO, line, length);
                p.publisher_.recordWrite(written == static_cast<ssize_t>(length));
            }
        }
        pros::delay(kPollMs);
    }
}

} // namespace spectagatr
