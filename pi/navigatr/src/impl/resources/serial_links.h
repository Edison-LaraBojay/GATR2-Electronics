// serial_links.h
// SerialLink implementations and their resource factories.
//
//   linux_serial_link   live serial device
//       <Device path="/dev/ttyAMA5"/>
//       <Baud value="115200"/>
//       <DriverEnable gpio="6"/>        optional, RS-485 DE and /RE
//           post_guard_us="174"         optional, default 2 characters
//           tx_margin_us="2000"         optional, transmit deadline slack
//
//   memory_link         in-memory link for tests and loopback rigs
//
//   file_replay_link    read-only capture replay
//       <File path="capture.bin"/>
//
// DriverEnable makes the link half duplex: the GPIO (sysfs) is driven low at
// open so the transceiver listens, high only while write() sends a frame,
// and low again once the transmitter is empty and on destruction. If the GPIO
// cannot be reached that is a build warning and every write fails.
//
// A live device that fails to open is a build warning and a dead link at
// runtime (an unplugged cable must not stop the robot); a named capture file
// that fails to open is a build error, because a capture is not
// hot-pluggable. The memory link keeps its input and output streams
// separate, so a bidirectional link never loops back on itself in a test.

#pragma once
#include <functional>
#include <memory>
#include <string>

#include "resources/resource_store.h"
#include "resources/serial_link.h"
#include "transport/byte_stream.h"
#include "transport/file_stream.h"
#include "transport/half_duplex.h"
#include "transport/serial_port.h"

namespace navigatr
{

class LinuxSerialLink : public SerialLink
{
public:
    SerialReadResult  readAvailable(MutableByteSpan destination) override;
    SerialWriteResult write(ByteSpan source) override;
    SerialWriteResult write(ByteSpan source, const TransmitWindow& window) override;
    bool              inputPending() override { return port_.inputPending(); }
    int64_t           nowUs() override { return port_.nowUs(); }

    SerialPort& port() { return port_; }

    // Half duplex with the GPIO as transceiver DE. False when the GPIO
    // cannot be reached; the link stays half duplex and writes fail.
    bool enableHalfDuplex(int gpio, const HalfDuplexTiming& timing, std::string& err);
    bool halfDuplex() const { return half_duplex_; }

private:
    SerialPort       port_;
    bool             half_duplex_ = false;
    HalfDuplexTiming timing_;
};

class MemoryLink : public SerialLink
{
public:
    SerialReadResult  readAvailable(MutableByteSpan destination) override;
    SerialWriteResult write(ByteSpan source) override;
    // Like a half-duplex link, but never waits: a future not_before_us counts
    // as reached. Expired when the window is missed at nowUs(), input_pending
    // while input() holds bytes; nothing is written in either case.
    SerialWriteResult write(ByteSpan source, const TransmitWindow& window) override;
    bool              inputPending() override { return input_.size() > 0; }
    int64_t           nowUs() override { return clock_ ? clock_() : steadyNowUs(); }

    // Test clock in microseconds; empty restores the steady clock.
    void setClock(std::function<int64_t()> now_us) { clock_ = std::move(now_us); }

    // Test access: feed input(), drain output().
    MemoryStream& input() { return input_; }
    MemoryStream& output() { return output_; }

private:
    MemoryStream             input_;
    MemoryStream             output_;
    std::function<int64_t()> clock_;
};

class FileReplayLink : public SerialLink
{
public:
    SerialReadResult  readAvailable(MutableByteSpan destination) override;
    SerialWriteResult write(ByteSpan) override { return SerialWriteResult{false}; }

    bool open(const std::string& path, std::string& err) {
        return source_.open(path, err);
    }

private:
    FileByteSource source_;
};

ResourceInstance make_linux_serial_link(const ConfigNode& node,
                                     ResourceInitializationContext& context,
                                     std::string& err);
ResourceInstance make_memory_link(const ConfigNode& node,
                               ResourceInitializationContext& context, std::string& err);
ResourceInstance make_file_replay_link(const ConfigNode& node,
                                    ResourceInitializationContext& context,
                                    std::string& err);

// Replay override used by the app: a factory bound to a specific capture
// path, replacing whatever the resource id declared.
ResourceMakeFunction fileReplayFactoryForPath(std::string path);

} // namespace navigatr
