// fake_usb.h
// Host-only V5 USB user console between the Brain's BytePort and a FakePi:
// every frame travels as an NG1 line through the real line codec in both
// directions, with fixed latencies, console text mixed in, and a cable that
// can be pulled. The Pi side follows pros_usb_link: after the port reopens,
// the first request is applied but not answered.

#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "communigatr/byte_port.h"
#include "communigatr/client.h"
#include "communigatr/usb_line.h"
#include "sim/fake_pi.h"

namespace communigatr
{

struct FakeUsbConfig {
    Seconds to_pi      = 0.002; // Brain write to the Pi reading the line
    Seconds turnaround = 0.004; // Pi read to its reply write
    Seconds to_brain   = 0.002; // Pi write to the Brain reading the line
};

class FakeUsb {
public:
    explicit FakeUsb(FakePi& pi, const FakeUsbConfig& config = {});
    FakeUsb(const FakeUsb&)            = delete;
    FakeUsb& operator=(const FakeUsb&) = delete;

    BytePort& brainPort() { return port_; }

    // Moves the clock forward and delivers every line due by now.
    void    advanceTo(Seconds now);
    Seconds now() const { return now_; }

    // Unplugged: lines in flight and new writes are lost; the Brain port
    // still accepts writes, as the PROS transmit queue does.
    void setPlugged(bool plugged);
    bool plugged() const { return plugged_; }

    // Console text on the wire, delivered with the next lines.
    void textToPi(const std::string& text);
    void textToBrain(const std::string& text);

    const UsbLineStats& piLines() const { return pi_decoder_.stats(); }
    const UsbLineStats& brainLines() const { return brain_decoder_.stats(); }
    int                 unansweredAfterReopen() const { return unanswered_; }
    int                 linesToPi() const { return lines_to_pi_; }

private:
    class Port : public BytePort {
    public:
        explicit Port(FakeUsb& usb) : usb_(usb) {}
        int  read(uint8_t* buf, int max) override;
        bool write(const uint8_t* data, int len) override;

    private:
        FakeUsb& usb_;
    };

    int  brainRead(uint8_t* buf, int max);
    bool brainWrite(const uint8_t* data, int len);
    void deliverToPi(const std::string& chars, Seconds at);

    FakePi&        pi_;
    FakeUsbConfig  config_;
    Port           port_;
    Seconds        now_     = 0;
    bool           plugged_ = true;
    bool           reopened_ = false;
    int            unanswered_  = 0;
    int            lines_to_pi_ = 0;
    UsbLineDecoder pi_decoder_;
    UsbLineDecoder brain_decoder_;

    std::multimap<Seconds, std::string> to_pi_;    // arrival time, characters
    std::multimap<Seconds, std::string> to_brain_; // arrival time, characters
    std::vector<uint8_t>                brain_rx_; // decoded, readable now
};

} // namespace communigatr
