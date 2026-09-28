// pros_usb_port.cpp
// PROS only.

#include "communigatr/pros_usb_port.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include "pros/apix.h"
#include "pros/rtos.hpp"

namespace communigatr
{

namespace
{

constexpr uint32_t kIdleMs = 2;       // transmit task with nothing queued
constexpr uint32_t kNoInputMs = 5;    // receive task after an empty or failed read
constexpr uint32_t kStopWaitMs = 50;  // destructor wait for the tasks

struct UsbFrame {
    uint8_t  bytes[gatr2::kMaxFrameLen] = {};
    uint16_t size                       = 0;
    uint32_t queued_at_ms               = 0;
};

// One producer task and one consumer task.
template <std::size_t Capacity> class FrameQueue {
public:
    bool push(const UsbFrame& frame) {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        if (head - tail_.load(std::memory_order_acquire) >= Capacity) {
            return false;
        }
        slots_[head % Capacity] = frame;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    bool pop(UsbFrame& frame) {
        const uint32_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        frame = slots_[tail % Capacity];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

private:
    UsbFrame              slots_[Capacity];
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
};

void bump(std::atomic<uint32_t>& counter) {
    counter.fetch_add(1, std::memory_order_relaxed);
}

} // namespace

// Everything the I/O tasks touch. Heap allocated so it can outlive the port
// while a task is still inside a USB call.
struct ProsUsbPort::Io {
    uint32_t          tx_lifetime_ms = 1;
    std::atomic<int>  output_fd{-1};
    bool              configured = false; // stream activated, output COBS off
    std::atomic<bool> stopping{false};
    std::atomic<bool> tx_running{false};
    std::atomic<bool> rx_running{false};

    FrameQueue<1> outgoing; // poll task to transmit task
    FrameQueue<4> incoming; // receive task to poll task
    UsbFrame      reading;  // poll task only
    std::size_t   read_at = 0;

    UsbLineDecoder decoder; // receive task only

    std::atomic<uint32_t> frames_out{0};
    std::atomic<uint32_t> expired{0};
    std::atomic<uint32_t> short_writes{0};
    std::atomic<uint32_t> frames_in{0};
    std::atomic<uint32_t> input_full{0};
    std::atomic<uint32_t> read_errors{0};
    std::atomic<uint32_t> lines_ignored{0};
    std::atomic<uint32_t> lines_dropped{0};

    static void transmitEntry(void* self) { static_cast<Io*>(self)->transmit(); }
    static void receiveEntry(void* self) { static_cast<Io*>(self)->receive(); }

    // A failed or partial write is left to the client's bounded retries; no
    // USB backlog builds up behind it.
    void transmit() {
        char line[kUsbLineMax];
        while (!stopping.load()) {
            UsbFrame frame;
            if (!outgoing.pop(frame)) {
                pros::delay(kIdleMs);
                continue;
            }
            if (pros::millis() - frame.queued_at_ms >= tx_lifetime_ms) {
                bump(expired);
                continue;
            }
            const std::size_t n = encodeUsbLine(frame.bytes, frame.size, line, sizeof(line));
            if (n == 0) {
                continue;
            }
            // One write per line: the kernel serial mutex keeps it whole
            // between other console output.
            const ssize_t written = ::write(output_fd.load(), line, n);
            if (written == static_cast<ssize_t>(n)) {
                bump(frames_out);
            } else {
                bump(short_writes);
            }
        }
        tx_running.store(false);
    }

    void receive() {
        char input[96];
        while (!stopping.load()) {
            // Kernel 4.2.2 ser_read_r appends a NUL after the returned data:
            // one byte stays free, and only the count is used.
            const ssize_t count = ::read(STDIN_FILENO, input, sizeof(input) - 1);
            if (stopping.load()) {
                break;
            }
            if (count <= 0) {
                if (count < 0) {
                    bump(read_errors);
                }
                pros::delay(kNoInputMs);
                continue;
            }
            for (ssize_t i = 0; i < count; ++i) {
                if (!decoder.push(input[i])) {
                    continue;
                }
                UsbFrame frame;
                frame.size = static_cast<uint16_t>(decoder.length());
                std::memcpy(frame.bytes, decoder.bytes(), frame.size);
                // Queue full: the whole frame is dropped, never a part of it.
                if (incoming.push(frame)) {
                    bump(frames_in);
                } else {
                    bump(input_full);
                }
            }
            lines_ignored.store(decoder.stats().ignored, std::memory_order_relaxed);
            lines_dropped.store(decoder.stats().dropped, std::memory_order_relaxed);
        }
        rx_running.store(false);
    }
};

ProsUsbPort::ProsUsbPort(uint32_t tx_lifetime_ms) : io_(new Io) {
    io_->tx_lifetime_ms = std::max<uint32_t>(1, tx_lifetime_ms);
}

ProsUsbPort::~ProsUsbPort() {
    io_->stopping.store(true);
    for (uint32_t waited = 0; waited < kStopWaitMs && (io_->tx_running || io_->rx_running);
         waited += kIdleMs) {
        pros::delay(kIdleMs);
    }
    if (io_->tx_running || io_->rx_running) {
        return; // a task may own a kernel serial mutex: never pull its state away
    }
    if (io_->output_fd >= 0) {
        ::close(io_->output_fd);
    }
    delete io_;
}

bool ProsUsbPort::open() {
    Io& io = *io_;
    if (io.stopping.load()) {
        errno = EINVAL;
        return false;
    }
    if (io.output_fd < 0) {
        io.output_fd = ::open("/ser/ngtr", O_WRONLY);
        if (io.output_fd < 0) {
            return false;
        }
    }
    if (!io.configured) {
        // COBS off applies to all PROS output, so console text arrives plain
        // and the Pi skips it; SERCTL_NOBLKWRITE does nothing in 4.2.2.
        if (pros::c::fdctl(io.output_fd, SERCTL_ACTIVATE, nullptr) != 0 ||
            pros::c::serctl(SERCTL_DISABLE_COBS, nullptr) != 0) {
            return false;
        }
        io.configured = true;
    }
    if (!io.tx_running) {
        io.tx_running = true;
        if (pros::c::task_create(&Io::transmitEntry, io_, TASK_PRIORITY_DEFAULT,
                                 TASK_STACK_DEPTH_DEFAULT, "communigatr-usb-tx") == nullptr) {
            io.tx_running = false;
            errno         = ENOMEM;
            return false;
        }
    }
    if (!io.rx_running) {
        io.rx_running = true;
        if (pros::c::task_create(&Io::receiveEntry, io_, TASK_PRIORITY_DEFAULT,
                                 TASK_STACK_DEPTH_DEFAULT, "communigatr-usb-rx") == nullptr) {
            io.rx_running = false;
            errno         = ENOMEM;
            return false;
        }
    }
    return true;
}

bool ProsUsbPort::isOpen() const {
    return io_->configured && io_->tx_running && io_->rx_running;
}

int ProsUsbPort::read(uint8_t* buf, int max) {
    if (max <= 0) {
        return 0;
    }
    Io& io = *io_;
    if (io.read_at == io.reading.size) {
        if (!io.incoming.pop(io.reading)) {
            return 0;
        }
        io.read_at = 0;
    }
    const std::size_t n = std::min<std::size_t>(static_cast<std::size_t>(max),
                                                 io.reading.size - io.read_at);
    std::memcpy(buf, io.reading.bytes + io.read_at, n);
    io.read_at += n;
    return static_cast<int>(n);
}

bool ProsUsbPort::write(const uint8_t* data, int len) {
    if (len < 0 || len > static_cast<int>(gatr2::kMaxFrameLen) || !isOpen()) {
        return false;
    }
    if (len == 0) {
        return true;
    }
    UsbFrame frame;
    frame.size         = static_cast<uint16_t>(len);
    frame.queued_at_ms = pros::millis();
    std::memcpy(frame.bytes, data, frame.size);
    return io_->outgoing.push(frame); // full: the client counts a write error
}

ProsUsbStats ProsUsbPort::stats() const {
    const Io&    io = *io_;
    ProsUsbStats s;
    s.frames_out    = io.frames_out.load(std::memory_order_relaxed);
    s.expired       = io.expired.load(std::memory_order_relaxed);
    s.short_writes  = io.short_writes.load(std::memory_order_relaxed);
    s.frames_in     = io.frames_in.load(std::memory_order_relaxed);
    s.input_full    = io.input_full.load(std::memory_order_relaxed);
    s.read_errors   = io.read_errors.load(std::memory_order_relaxed);
    s.lines_ignored = io.lines_ignored.load(std::memory_order_relaxed);
    s.lines_dropped = io.lines_dropped.load(std::memory_order_relaxed);
    return s;
}

} // namespace communigatr
