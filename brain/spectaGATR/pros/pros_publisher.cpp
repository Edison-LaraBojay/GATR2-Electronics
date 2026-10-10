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

#include <cstring>
#include <unistd.h>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace spectagatr
{
namespace
{

constexpr uint32_t kPollMs     = 5;
constexpr uint32_t kPriority   = TASK_PRIORITY_DEFAULT - 1;
constexpr char     kTaskName[] = "spectagatr";

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
    while (true) {
        if (p.enabled_.load()) {
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
