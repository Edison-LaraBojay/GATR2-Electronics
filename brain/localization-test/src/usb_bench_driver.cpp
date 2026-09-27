#include "usb_bench_driver.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <unistd.h>

#include "pros/apix.h"
#include "pros/misc.hpp"

namespace communigatr {
namespace {

constexpr char kPrefix[] = "NG1:";
constexpr std::size_t kFrameCapacity = gatr2::kMaxFrameLen;
constexpr std::size_t kLineCapacity = 4 + 2 * kFrameCapacity + 1;

struct UsbFrame {
    std::array<uint8_t, kFrameCapacity> bytes{};
    std::size_t size = 0;
    uint32_t queued_at_ms = 0;
};

// One producer and one consumer per queue. USB I/O never owns the client mutex.
template <std::size_t Capacity>
class FrameQueue {
public:
    bool push(const UsbFrame& frame) {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        if (head - tail_.load(std::memory_order_acquire) >= Capacity) return false;
        slots_[head % Capacity] = frame;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    bool pop(UsbFrame& frame) {
        const uint32_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return false;
        frame = slots_[tail % Capacity];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

private:
    std::array<UsbFrame, Capacity> slots_{};
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
};

int hexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

uint32_t mix(uint32_t value) {
    value ^= value >> 16;
    value *= 0x85EBCA6Bu;
    value ^= value >> 13;
    value *= 0xC2B2AE35u;
    return value ^ (value >> 16);
}

} // namespace

struct UsbBenchDriver::Impl final : BytePort, std::enable_shared_from_this<Impl> {
    explicit Impl(const ProsDriverConfig& settings)
        : config(settings), client(*this, [this] { return nonce(); }, settings.client),
          driver(client, settings.driver) {
        // The queue must not replay expired requests after a slow/disconnected
        // USB writer catches up. A request already inside the kernel can still
        // arrive late; the normal protocol correlation rejects its stale reply.
        const double lifetime = settings.client.response_timeout * 800.0;
        tx_lifetime_ms = static_cast<uint32_t>(std::clamp(lifetime, 1.0, 10000.0));
    }

    ~Impl() {
        if (output_fd >= 0) ::close(output_fd);
    }

    uint32_t nonce() {
        const uint64_t clock = pros::micros();
        return mix(static_cast<uint32_t>(clock) ^ static_cast<uint32_t>(clock >> 32) ^
                   static_cast<uint32_t>(reinterpret_cast<uintptr_t>(this)) ^
                   static_cast<uint32_t>(pros::battery::get_voltage()) ^
                   (++nonce_count * 0x9E3779B9u));
    }

    bool start() {
        std::lock_guard<pros::Mutex> lock(mutex);
        if (started) return true;
        if (stopping.load()) {
            errno = ENOMEM;
            return false;
        }
        if (output_fd < 0) output_fd = ::open("/ser/ngtr", O_WRONLY);
        if (output_fd < 0) return false;
        if (pros::c::fdctl(output_fd, SERCTL_ACTIVATE, nullptr) != 0 ||
            pros::c::serctl(SERCTL_DISABLE_COBS, nullptr) != 0) {
            return false;
        }

        // The named stream prevents accidental use of stdio buffering. Kernel
        // diagnostics may still share USB; each NG1 line is one atomic write
        // under the PROS serial driver's mutex and receivers discard other text.
        // SERCTL_NOBLKWRITE is ineffective in kernel 4.2.2, so isolate all USB
        // calls in workers instead of depending on that flag.
        const auto self = shared_from_this();
        const auto tx = pros::Task::create([self] { self->transmitUsb(); },
                                           TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                                           "bench-usb-tx");
        const auto rx = tx ? pros::Task::create([self] { self->receiveUsb(); },
                                               TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                                               "bench-usb-rx") : nullptr;
        const auto poll = rx ? pros::Task::create([self] { self->pollClient(); },
                                                 config.task_priority, TASK_STACK_DEPTH_DEFAULT,
                                                 "bench-usb-link") : nullptr;
        if (!poll) {
            // Never cancel a task inside a kernel serial call: it may own a
            // kernel mutex. Workers retain shared state until their I/O returns.
            stopping.store(true);
            errno = ENOMEM;
            return false;
        }
        started = true;
        return true;
    }

    int read(uint8_t* destination, int maximum) override {
        if (maximum <= 0) return 0;
        if (rx_offset == rx_current.size) {
            if (!received.pop(rx_current)) return 0;
            rx_offset = 0;
        }
        const std::size_t count = std::min<std::size_t>(maximum, rx_current.size - rx_offset);
        std::memcpy(destination, rx_current.bytes.data() + rx_offset, count);
        rx_offset += count;
        return static_cast<int>(count);
    }

    bool write(const uint8_t* data, int length) override {
        if (stopping.load() || length < 0 || static_cast<std::size_t>(length) > kFrameCapacity)
            return false;
        if (length == 0) return true;
        UsbFrame frame;
        frame.size = static_cast<std::size_t>(length);
        frame.queued_at_ms = pros::millis();
        std::memcpy(frame.bytes.data(), data, frame.size);
        return outgoing.push(frame);
    }

    void pollClient() {
        const uint32_t period = std::max<uint32_t>(1, config.poll_period_ms);
        uint32_t wake = pros::millis();
        while (!stopping.load()) {
            {
                std::lock_guard<pros::Mutex> lock(mutex);
                client.poll(UsbBenchDriver::now());
            }
            pros::Task::delay_until(&wake, period);
        }
    }

    void transmitUsb() {
        static constexpr char digits[] = "0123456789ABCDEF";
        std::array<char, kLineCapacity> line{};
        std::memcpy(line.data(), kPrefix, 4);
        while (!stopping.load()) {
            UsbFrame frame;
            if (!outgoing.pop(frame)) {
                pros::delay(2);
                continue;
            }
            if (pros::millis() - frame.queued_at_ms >= tx_lifetime_ms) continue;
            for (std::size_t i = 0; i < frame.size; ++i) {
                line[4 + 2 * i] = digits[frame.bytes[i] >> 4];
                line[5 + 2 * i] = digits[frame.bytes[i] & 15];
            }
            const std::size_t length = 4 + 2 * frame.size + 1;
            line[length - 1] = '\n';
            // A failed/partial write is left to the client's bounded retry
            // rules; do not accumulate a USB retry backlog or block its poll.
            (void)::write(output_fd, line.data(), length);
        }
    }

    void receiveUsb() {
        std::array<char, 96> input{};
        std::array<char, kLineCapacity> line{};
        std::size_t used = 0;
        bool overflow = false;
        while (!stopping.load()) {
            // Kernel 4.2.2 ser_read_r appends a NUL after the returned data.
            // Reserve that extra byte, and use the count (not strlen).
            const ssize_t count = ::read(STDIN_FILENO, input.data(), input.size() - 1);
            if (stopping.load()) break;
            if (count <= 0) {
                pros::delay(5);
                continue;
            }
            for (ssize_t i = 0; i < count; ++i) {
                const char c = input[static_cast<std::size_t>(i)];
                if (c == '\n') {
                    if (!overflow) acceptLine(line.data(), used);
                    used = 0;
                    overflow = false;
                } else if (used < line.size()) {
                    line[used++] = c;
                } else {
                    overflow = true;
                }
            }
        }
    }

    void acceptLine(const char* line, std::size_t length) {
        if (length > 0 && line[length - 1] == '\r') --length;
        // Ignore unrelated text, including a diagnostic prefix before NG1.
        std::size_t start = 0;
        while (start + 4 <= length && std::memcmp(line + start, kPrefix, 4) != 0) ++start;
        if (start + 4 > length) return;
        const std::size_t hex_length = length - start - 4;
        if (hex_length == 0 || hex_length % 2 != 0 || hex_length > 2 * kFrameCapacity) return;
        UsbFrame frame;
        frame.size = hex_length / 2;
        for (std::size_t i = 0; i < frame.size; ++i) {
            const int high = hexDigit(line[start + 4 + 2 * i]);
            const int low = hexDigit(line[start + 5 + 2 * i]);
            if (high < 0 || low < 0) return;
            frame.bytes[i] = static_cast<uint8_t>((high << 4) | low);
        }
        // The ordinary Client frame reader validates type, length and CRC.
        // On overflow drop a whole frame; never feed a truncated frame.
        (void)received.push(frame);
    }

    ProsDriverConfig config;
    Client client;
    Driver driver;
    mutable pros::Mutex mutex;
    bool started = false;
    std::atomic<bool> stopping{false};
    int output_fd = -1;
    uint32_t nonce_count = 0;
    uint32_t tx_lifetime_ms = 1;
    FrameQueue<1> outgoing;
    FrameQueue<4> received;
    UsbFrame rx_current;
    std::size_t rx_offset = 0;
};

UsbBenchDriver::UsbBenchDriver(const ProsDriverConfig& config)
    : impl_(std::make_shared<Impl>(config)) {}

UsbBenchDriver::~UsbBenchDriver() {
    // Used for the application's lifetime. If USB is blocked on disconnect,
    // workers keep their state alive and exit after I/O resumes; never remove
    // them while they might own a kernel mutex.
    impl_->stopping.store(true);
}

bool UsbBenchDriver::start() { return impl_->start(); }
Seconds UsbBenchDriver::now() { return static_cast<double>(pros::micros()) * 1e-6; }

void UsbBenchDriver::request(const investigatr::InputRequest& request) {
    std::lock_guard<pros::Mutex> lock(impl_->mutex);
    impl_->driver.request(request);
}

investigatr::InputSnapshot UsbBenchDriver::latest(Seconds time) {
    std::lock_guard<pros::Mutex> lock(impl_->mutex);
    return impl_->driver.latest(time);
}

PlacementTicket UsbBenchDriver::submitPlacement(const investigatr::Pose& pose) {
    std::lock_guard<pros::Mutex> lock(impl_->mutex);
    return impl_->driver.submitPlacement(pose);
}

PlacementResult UsbBenchDriver::placementResult(PlacementTicket ticket) const {
    std::lock_guard<pros::Mutex> lock(impl_->mutex);
    return impl_->driver.placementResult(ticket);
}

PlacementStatus UsbBenchDriver::placementStatus(PlacementTicket ticket) const {
    std::lock_guard<pros::Mutex> lock(impl_->mutex);
    return impl_->driver.placementStatus(ticket);
}

ProsDriverStatus UsbBenchDriver::status() const {
    std::lock_guard<pros::Mutex> lock(impl_->mutex);
    ProsDriverStatus result;
    const Seconds time = now();
    result.started = impl_->started;
    result.ready = impl_->client.ready();
    result.connected = impl_->client.connected(time);
    result.link_age = impl_->client.linkAge(time);
    result.session = impl_->client.session();
    result.pi_instance = impl_->client.piInstance();
    result.error = impl_->client.error();
    result.peer_version = impl_->client.peerVersion();
    result.selection = impl_->client.selection();
    result.stats = impl_->client.stats();
    return result;
}

} // namespace communigatr
