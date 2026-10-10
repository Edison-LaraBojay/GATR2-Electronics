// fake_port.h
// Test time and a scripted Port (a Brain user port unless told otherwise). A read with nothing due
// moves the clock by its wait, so a source loop runs in fake time without sleeping.

#pragma once
#include <deque>
#include <functional>
#include <string>
#include <variant>

#include "receiver/sources/source.h"

namespace spectagatr::receiver::sources::fakes
{

class FakeClock {
  public:
    ReceiverUs    now() const { return now_us_; }
    void          advance(DurationUs us) { now_us_ += us; }
    ReceiverClock clock();

  private:
    ReceiverUs now_us_ = 0;
};

// Bytes that arrive after_us after the previous step.
struct Arrive {
    DurationUs after_us = 0;
    Bytes      bytes;
};

// The device goes away after_us after the previous step.
struct Unplug {
    DurationUs  after_us = 0;
    std::string reason   = "unplugged";
};

using PortStep    = std::variant<Arrive, Unplug>;
using PortSteps   = std::deque<PortStep>;
using EndCallback = std::function<void()>;

class ScriptedPort {
  public:
    ScriptedPort(FakeClock& clock, PortSteps steps, std::string name = "fake",
                 SourceKind kind = SourceKind::kBrainUsb);

    // Reads and writes this object, which outlives the Port.
    Port port();

    // Called at each read once the script is used up.
    void onEnd(EndCallback callback) { on_end_ = std::move(callback); }

    const Bytes& written() const { return written_; }
    std::size_t  reads() const { return reads_; }

  private:
    IoResult read(Byte* destination, std::size_t capacity, Milliseconds wait);
    IoResult write(const Byte* source, std::size_t size);

    FakeClock&  clock_;
    PortSteps   steps_;
    std::string name_;
    SourceKind  kind_;
    EndCallback on_end_;
    Bytes       written_;
    std::size_t reads_  = 0;
    bool        closed_ = false;
};

} // namespace spectagatr::receiver::sources::fakes
