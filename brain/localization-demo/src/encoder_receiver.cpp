// encoder_receiver.cpp
// PROS only.

#include "encoder_receiver.h"

#include <unistd.h>

#include "pros/rtos.hpp"

namespace localization
{

namespace
{

constexpr uint32_t kNoInputMs = 5; // after an empty or failed read

} // namespace

bool EncoderReceiver::start() {
    return pros::c::task_create(&EncoderReceiver::taskEntry, this, TASK_PRIORITY_DEFAULT,
                                TASK_STACK_DEPTH_DEFAULT, "encoder-receiver") != nullptr;
}

ReceiverStats EncoderReceiver::stats() const {
    ReceiverStats s;
    s.records       = records_.load(std::memory_order_relaxed);
    s.skipped_lines = skipped_.load(std::memory_order_relaxed);
    s.read_errors   = read_errors_.load(std::memory_order_relaxed);
    return s;
}

void EncoderReceiver::run() {
    encoder_record::LineScanner scanner;
    uint32_t                    number = 0;
    char                        input[96];
    while (true) {
        // Blocks this task only. Kernel 4.2.2 appends a NUL after the data,
        // so one byte stays free.
        const ssize_t count = ::read(STDIN_FILENO, input, sizeof(input) - 1);
        if (count <= 0) {
            if (count < 0) {
                read_errors_.fetch_add(1, std::memory_order_relaxed);
            }
            pros::delay(kNoInputMs);
            continue;
        }
        for (ssize_t i = 0; i < count; ++i) {
            if (!scanner.push(input[i])) {
                continue;
            }
            if (++number == 0) {
                number = 1;
            }
            ReceivedRecord received;
            received.record     = scanner.record();
            received.arrived_ms = pros::millis();
            received.number     = number;
            newest_.store(received);
        }
        records_.store(scanner.stats().records, std::memory_order_relaxed);
        skipped_.store(scanner.stats().skipped, std::memory_order_relaxed);
    }
}

} // namespace localization
