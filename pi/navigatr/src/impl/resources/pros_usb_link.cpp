#include "impl/resources/pros_usb_link.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "common/frames.h"

#if defined(__unix__) || defined(__APPLE__)
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace navigatr
{
namespace
{
constexpr std::size_t kPrefixLength = 4;
constexpr std::size_t kLineLimit = kPrefixLength + 2 * gatr2::kMaxFrameLen + 256;
constexpr int64_t kRetryUs = 1'000'000;

int hexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

std::string attribute(const std::filesystem::path& path) {
    std::ifstream file(path);
    std::string value;
    file >> value;
    return value;
}

// Owns only the bench USB device. No GPIO, console commands, or changes to the
// production serial drivers. All reads are nonblocking; disconnection closes
// the old descriptor and discovery is repeated to allow a changed ACM number.
class ReconnectingUsbPort final : public SerialLink
{
public:
    explicit ReconnectingUsbPort(std::string path) : path_(std::move(path)) {}
    ~ReconnectingUsbPort() override { disconnect(); }

    SerialReadResult readAvailable(MutableByteSpan destination) override {
#if defined(__unix__) || defined(__APPLE__)
        if (!connect()) return {0, true};
        pollfd event{fd_, POLLIN, 0};
        const int polled = ::poll(&event, 1, 0);
        if (polled < 0 && errno == EINTR) return {};
        if (polled < 0 || (event.revents & (POLLHUP | POLLERR | POLLNVAL))) {
            disconnect();
            return {0, true};
        }
        if (!(event.revents & POLLIN) || destination.size == 0) return {};
        const ssize_t count = ::read(fd_, destination.data, destination.size);
        if (count > 0) return {static_cast<std::size_t>(count), false};
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
            return {};
        disconnect();
        return {0, true};
#else
        (void)destination;
        return {0, true};
#endif
    }

    SerialWriteResult write(ByteSpan source) override {
        return write(source, TransmitWindow{});
    }

    SerialWriteResult write(ByteSpan source, const TransmitWindow& window) override {
        if (window.missed(nowUs())) return {false, true, false, false, "USB reply expired"};
#if defined(__unix__) || defined(__APPLE__)
        if (!connect()) return {false, false, false, false, "Brain USB unavailable"};
        const int64_t now = nowUs();
        if (window.not_before_us > now) {
            std::this_thread::sleep_for(std::chrono::microseconds(window.not_before_us - now));
        }
        if (window.missed(nowUs())) return {false, true, false, false, "USB reply expired"};
        // A USB frame is at most 261 bytes. Bound any host backpressure to 5 ms
        // so a detached/unresponsive Brain cannot stall the localization loop.
        const int64_t end = std::min(window.deadline_us, nowUs() + 5'000);
        std::size_t offset = 0;
        while (offset < source.size) {
            const ssize_t count = ::write(fd_, source.data + offset, source.size - offset);
            if (count > 0) {
                offset += static_cast<std::size_t>(count);
                continue;
            }
            if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                disconnect();
                return {false, false, false, false, "Brain USB write failed"};
            }
            const int64_t left = end - nowUs();
            if (left <= 0) {
                disconnect();
                return {false, false, false, false, "Brain USB write timed out"};
            }
            pollfd event{fd_, POLLOUT, 0};
            if (::poll(&event, 1, static_cast<int>((left + 999) / 1000)) < 0 &&
                errno != EINTR) {
                disconnect();
                return {false, false, false, false, "Brain USB poll failed"};
            }
            if (event.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                disconnect();
                return {false, false, false, false, "Brain USB disconnected"};
            }
        }
        return {true};
#else
        (void)source;
        return {false, false, false, false, "Brain USB is supported on POSIX hosts only"};
#endif
    }

    bool inputPending() override {
#if defined(__unix__) || defined(__APPLE__)
        if (fd_ < 0) return false;
        pollfd event{fd_, POLLIN, 0};
        return ::poll(&event, 1, 0) > 0 && (event.revents & POLLIN);
#else
        return false;
#endif
    }

private:
    void disconnect() {
#if defined(__unix__) || defined(__APPLE__)
        if (fd_ >= 0) ::close(fd_);
#endif
        fd_ = -1;
    }

#if defined(__unix__) || defined(__APPLE__)
    bool connect() {
        if (fd_ >= 0) return true;
        if (nowUs() < retry_at_) return false;
        retry_at_ = nowUs() + kRetryUs;
        const std::string device = path_ == "auto" ? findProsUsbUserPort() : path_;
        if (device.empty()) return false;
        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) return false;
        termios settings{};
        if (::tcgetattr(fd_, &settings) != 0) {
            disconnect();
            return false;
        }
        ::cfmakeraw(&settings);
        ::cfsetispeed(&settings, B115200);
        ::cfsetospeed(&settings, B115200);
        settings.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
        settings.c_cflag &= ~CRTSCTS;
#endif
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        if (::tcsetattr(fd_, TCSANOW, &settings) != 0) {
            disconnect();
            return false;
        }
        ::tcflush(fd_, TCIOFLUSH);
        return true;
    }
#endif

    std::string path_;
    int fd_ = -1;
    [[maybe_unused]] int64_t retry_at_ = 0;
};
} // namespace

std::string findProsUsbUserPort(const std::string& tty_root, const std::string& device_root) {
    namespace fs = std::filesystem;
    std::error_code error;
    fs::directory_iterator it(tty_root, error), end;
    std::string selected;
    for (; !error && it != end; it.increment(error)) {
        const std::string name = it->path().filename().string();
        if (name.compare(0, 6, "ttyACM") != 0) continue;
        fs::path path = fs::canonical(it->path() / "device", error);
        if (error) {
            error.clear();
            continue;
        }
        std::string interface_number, vendor, product;
        for (unsigned depth = 0; depth < 12 && !path.empty(); ++depth) {
            if (interface_number.empty()) interface_number = attribute(path / "bInterfaceNumber");
            if (vendor.empty()) vendor = attribute(path / "idVendor");
            if (product.empty()) product = attribute(path / "idProduct");
            const fs::path parent = path.parent_path();
            if (parent == path) break;
            path = parent;
        }
        if (interface_number != "02" || vendor != "2888" || product != "0501") continue;
        if (!selected.empty()) return {}; // multiple Brains: require explicit device
        selected = (fs::path(device_root) / name).string();
    }
    return error ? std::string{} : selected;
}

void ProsUsbLink::finishLine() {
    if (!line_.empty() && line_.back() == '\r') line_.pop_back();
    const std::size_t marker = line_.rfind("NG1:");
    if (marker == std::string::npos) return;
    const std::size_t start = marker + kPrefixLength;
    const std::size_t digits = line_.size() - start;
    if (digits == 0 || digits % 2 != 0 || digits > 2 * gatr2::kMaxFrameLen) return;
    std::array<uint8_t, gatr2::kMaxFrameLen> bytes{};
    for (std::size_t i = 0; i < digits; i += 2) {
        const int high = hexDigit(line_[start + i]);
        const int low = hexDigit(line_[start + i + 1]);
        if (high < 0 || low < 0) return;
        bytes[i / 2] = static_cast<uint8_t>((high << 4) | low);
    }
    decoded_.insert(decoded_.end(), bytes.begin(), bytes.begin() + digits / 2);
}

void ProsUsbLink::acceptByte(uint8_t byte) {
    if (byte == '\n') {
        if (!discarding_) finishLine();
        line_.clear();
        discarding_ = false;
    } else if (!discarding_) {
        if (line_.size() >= kLineLimit) {
            line_.clear();
            discarding_ = true;
        } else {
            line_.push_back(static_cast<char>(byte));
        }
    }
}

SerialReadResult ProsUsbLink::readAvailable(MutableByteSpan destination) {
    if (destination.size == 0) return {};
    // Never drain an unbounded diagnostic stream in one localization step.
    for (unsigned batch = 0; decoded_.empty() && batch < 4; ++batch) {
        std::array<uint8_t, 256> input{};
        const auto result = transport_->readAvailable({input.data(), input.size()});
        if (result.closed) {
            line_.clear();
            discarding_ = false;
            return {0, true};
        }
        for (std::size_t i = 0; i < result.bytes; ++i) acceptByte(input[i]);
        if (result.bytes == 0) break;
    }
    std::size_t count = 0;
    while (count < destination.size && !decoded_.empty()) {
        destination.data[count++] = decoded_.front();
        decoded_.pop_front();
    }
    return {count, false};
}

SerialWriteResult ProsUsbLink::write(ByteSpan source) { return write(source, TransmitWindow{}); }

SerialWriteResult ProsUsbLink::write(ByteSpan source, const TransmitWindow& window) {
    if (source.size == 0 || source.size > gatr2::kMaxFrameLen)
        return {false, false, false, false, "USB frame size invalid"};
    static constexpr char hex[] = "0123456789ABCDEF";
    std::string line = "NG1:";
    line.reserve(kPrefixLength + source.size * 2 + 1);
    for (std::size_t i = 0; i < source.size; ++i) {
        line.push_back(hex[source.data[i] >> 4]);
        line.push_back(hex[source.data[i] & 15]);
    }
    line.push_back('\n');
    return transport_->write({reinterpret_cast<const uint8_t*>(line.data()), line.size()}, window);
}

bool ProsUsbLink::inputPending() { return !decoded_.empty() || transport_->inputPending(); }

ResourceInstance make_pros_usb_link(const ConfigNode& node, ResourceInitializationContext&,
                                    std::string& err) {
    const std::string path = node.child("Device").attr("path");
    if (path.empty()) {
        err = node.path() + ": pros_usb_link needs <Device path=\"auto\"/> or an explicit USB user port";
        return {};
    }
    if (node.child("DriverEnable").valid()) {
        err = node.path() + ": pros_usb_link is USB and cannot use DriverEnable";
        return {};
    }
    auto transport = std::make_shared<ReconnectingUsbPort>(path);
    return ResourceInstance::asContract<SerialLink>(std::make_shared<ProsUsbLink>(std::move(transport)));
}
} // namespace navigatr
