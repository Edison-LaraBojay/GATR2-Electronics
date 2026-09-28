// Optional bench transport for the V5 Brain's USB user interface. Existing
// brain_link framing is wrapped in NG1:<uppercase hex>\n to avoid PROS stdin
// terminal control sequences. The Brain app must disable its output COBS mode.
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "resources/resource_store.h"
#include "resources/serial_link.h"

namespace navigatr
{

class LinkMonitor;

// Finds exactly one VEX V5 Brain (2888:0501), USB interface 02. Never guesses
// ttyACM numbering or opens the system/upload interface. Empty means absent or
// ambiguous; Device path=... can select an explicit user-interface symlink.
std::string findProsUsbUserPort(const std::string& tty_root = "/sys/class/tty",
                                const std::string& device_root = "/dev");

class ProsUsbLink : public SerialLink
{
public:
    explicit ProsUsbLink(std::shared_ptr<SerialLink> transport)
        : transport_(std::move(transport)) {}

    SerialReadResult readAvailable(MutableByteSpan destination) override;
    SerialWriteResult write(ByteSpan source) override;
    SerialWriteResult write(ByteSpan source, const TransmitWindow& window) override;
    bool inputPending() override;
    int64_t nowUs() override { return transport_->nowUs(); }

    // Dropped NG1 lines are reported to the Brain link's monitor as
    // rejections; frame bytes are monitored by the brain_link slots at the
    // decoded frame layer, not here. Lines without an NG1: marker are Brain
    // console text and are only counted.
    void attachMonitor(std::shared_ptr<LinkMonitor> monitor) { monitor_ = std::move(monitor); }

    uint64_t framedLines() const { return framed_lines_; }     // NG1 lines decoded
    uint64_t rejectedLines() const { return rejected_lines_; } // NG1 lines dropped
    uint64_t consoleLines() const { return console_lines_; }   // no NG1: marker

private:
    void acceptByte(uint8_t byte);
    void finishLine();
    void reject(const char* reason);
    std::shared_ptr<SerialLink> transport_;
    std::shared_ptr<LinkMonitor> monitor_;
    std::string line_;
    std::deque<uint8_t> decoded_;
    bool discarding_ = false;
    uint64_t framed_lines_ = 0, rejected_lines_ = 0, console_lines_ = 0;
};

// <Resource id="brain_uart" type="pros_usb_link"><Device path="auto"/></Resource>
// Optional USB is retried once per second; absence never prevents startup.
ResourceInstance make_pros_usb_link(const ConfigNode& node,
                                    ResourceInitializationContext& context,
                                    std::string& err);

} // namespace navigatr
