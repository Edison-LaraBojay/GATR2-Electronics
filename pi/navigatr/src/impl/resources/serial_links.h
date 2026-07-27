// serial_links.h
// SerialLink implementations and their resource factories.
//
//   resource/linux_serial_link   live serial device
//       <Device path="/dev/ttyAMA0"/>
//       <Baud value="921600"/>
//
//   resource/memory_link         in-memory link for tests and loopback rigs
//
//   resource/file_replay_link    read-only capture replay
//       <File path="capture.bin"/>
//
// A live device that fails to open is a build warning and a dead link at
// runtime (an unplugged cable must not stop the robot); a named capture file
// that fails to open is a build error, because a capture is not
// hot-pluggable. The memory link keeps its input and output streams
// separate, so a bidirectional link never loops back on itself in a test.

#pragma once
#include <memory>
#include <string>

#include "resources/resource_store.h"
#include "resources/serial_link.h"
#include "transport/byte_stream.h"
#include "transport/file_stream.h"
#include "transport/serial_port.h"

namespace navigatr
{

class LinuxSerialLink : public SerialLink
{
public:
    SerialReadResult  readAvailable(MutableByteSpan destination) override;
    SerialWriteResult write(ByteSpan source) override;

    SerialPort& port() { return port_; }

private:
    SerialPort port_;
};

class MemoryLink : public SerialLink
{
public:
    SerialReadResult  readAvailable(MutableByteSpan destination) override;
    SerialWriteResult write(ByteSpan source) override;

    // Test access: feed input(), drain output().
    MemoryStream& input() { return input_; }
    MemoryStream& output() { return output_; }

private:
    MemoryStream input_;
    MemoryStream output_;
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

ResourceValue make_linux_serial_link(const ConfigNode& node,
                                     ResourceInitializationContext& context,
                                     std::string& err);
ResourceValue make_memory_link(const ConfigNode& node,
                               ResourceInitializationContext& context, std::string& err);
ResourceValue make_file_replay_link(const ConfigNode& node,
                                    ResourceInitializationContext& context,
                                    std::string& err);

// Replay override used by the app: a factory bound to a specific capture
// path, replacing whatever the resource id declared.
ResourceMakeFunction fileReplayFactoryForPath(std::string path);

} // namespace navigatr
