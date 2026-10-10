// serial_port_gtest.cpp
// Opens that fail cleanly on every OS; a pty loopback on POSIX.

#include "receiver/platform/serial_port.h"

#include <array>
#include <gtest/gtest.h>
#include <string>
#include <utility>

#if !defined(_WIN32)
#include <unistd.h>
#if defined(__APPLE__)
#include <util.h>
#else
#include <pty.h>
#endif
#endif

namespace spectagatr::receiver::platform
{
namespace
{

using std::chrono_literals::operator""ms;

#if defined(_WIN32)
const std::string kMissingPort = "COM250";
const std::string kNullDevice  = "NUL";
#else
const std::string kMissingPort = "/dev/spectagatr-no-such-port";
const std::string kNullDevice  = "/dev/null";
#endif

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

TEST(SerialPort, MissingPortFailsWithItsName) {
    std::string                     err;
    const std::optional<SerialPort> port = openSerialPort(kMissingPort, {}, err);
    EXPECT_FALSE(port);
    EXPECT_TRUE(contains(err, kMissingPort + ": no such port")) << err;
}

#if defined(_WIN32)
TEST(SerialPort, FullDevicePathFailsTheSameWay) {
    std::string err;
    EXPECT_FALSE(openSerialPort("\\\\.\\COM250", {}, err));
    EXPECT_TRUE(contains(err, "no such port")) << err;
}
#endif

TEST(SerialPort, NonSerialDeviceIsRefusedAndReleased) {
    for (int attempt = 0; attempt < 2; ++attempt) {
        std::string err;
        EXPECT_FALSE(openSerialPort(kNullDevice, {}, err));
        EXPECT_TRUE(contains(err, "not a serial port")) << err;
    }
}

TEST(SerialPort, EmptyPathAndBadBaud) {
    std::string err;
    EXPECT_FALSE(openSerialPort("", {}, err));
    EXPECT_EQ(err, "no serial port given");

    SerialSettings settings;
    settings.baud = 0;
    err.clear();
    EXPECT_FALSE(openSerialPort(kMissingPort, settings, err));
    EXPECT_FALSE(err.empty());
}

TEST(SerialPort, ClosedPortReportsClosed) {
    SerialPort          port(kNoHandle, "none");
    std::array<Byte, 8> buffer{};
    const IoResult      read  = port.read(buffer.data(), buffer.size(), 0ms);
    const IoResult      write = port.writeAll(buffer.data(), buffer.size(), 0ms);
    EXPECT_FALSE(port.open());
    EXPECT_EQ(read.status, IoStatus::kClosed);
    EXPECT_EQ(write.status, IoStatus::kClosed);
    EXPECT_EQ(read.error, "none: not open");

    SerialPort moved(std::move(port));
    EXPECT_FALSE(moved.open());
    EXPECT_EQ(moved.path(), "none");
    EXPECT_STREQ(ioStatusName(IoStatus::kTimedOut), "timed_out");
}

#if !defined(_WIN32)
// Untested on the Windows build host; runs on the Linux and macOS CI.
class PtyLoopback : public ::testing::Test {
  protected:
    int         master = -1;
    int         slave  = -1;
    std::string path;

    void SetUp() override {
        ASSERT_EQ(::openpty(&master, &slave, nullptr, nullptr, nullptr), 0);
        path = ::ttyname(slave);
    }

    void TearDown() override {
        closeMaster();
        ::close(slave);
    }

    void closeMaster() {
        if (master >= 0) {
            ::close(master);
            master = -1;
        }
    }
};

TEST_F(PtyLoopback, ReadsWakeOnDataAndWritesArrive) {
    std::string               err;
    std::optional<SerialPort> port = openSerialPort(path, {}, err);
    ASSERT_TRUE(port) << err;

    std::array<Byte, 16> buffer{};
    EXPECT_EQ(port->read(buffer.data(), buffer.size(), 0ms).status, IoStatus::kTimedOut);

    ASSERT_EQ(::write(master, "SGP1", 4), 4);
    const IoResult read = port->read(buffer.data(), buffer.size(), 5000ms);
    ASSERT_EQ(read.status, IoStatus::kOk) << read.error;
    EXPECT_EQ(std::string(buffer.begin(), buffer.begin() + read.bytes), "SGP1");

    const std::array<Byte, 3> poll{0xC9, 0x36, 0x00};
    const IoResult            write = port->writeAll(poll.data(), poll.size(), 1000ms);
    ASSERT_EQ(write.status, IoStatus::kOk) << write.error;
    std::array<Byte, 8> echoed{};
    EXPECT_EQ(::read(master, echoed.data(), echoed.size()), 3);
    EXPECT_EQ(echoed[0], 0xC9);
    EXPECT_EQ(echoed[2], 0x00);
}

TEST_F(PtyLoopback, HangUpClosesThePort) {
    std::string               err;
    std::optional<SerialPort> port = openSerialPort(path, {}, err);
    ASSERT_TRUE(port) << err;
    closeMaster();
    std::array<Byte, 8> buffer{};
    const IoResult      read = port->read(buffer.data(), buffer.size(), 5000ms);
    EXPECT_EQ(read.status, IoStatus::kClosed);
    EXPECT_FALSE(port->open());
}

TEST_F(PtyLoopback, SecondOpenIsRefused) {
    if (::geteuid() == 0) {
        GTEST_SKIP() << "root bypasses TIOCEXCL";
    }
    std::string                     err;
    const std::optional<SerialPort> first = openSerialPort(path, {}, err);
    ASSERT_TRUE(first) << err;
    EXPECT_FALSE(openSerialPort(path, {}, err));
    EXPECT_TRUE(contains(err, "in use")) << err;
}
#endif

} // namespace
} // namespace spectagatr::receiver::platform
