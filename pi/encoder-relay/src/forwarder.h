// forwarder.h
// Pico UART bytes in, Brain USB text lines out. Only the newest record waits
// to be sent: counts are cumulative, so a newer record replaces an unsent
// older one and no backlog builds up. A line already partly written is
// finished first so the Brain never sees a torn line. Portable, no I/O.

#pragma once
#include <cstddef>
#include <cstdint>

#include "encoder_record.h"

namespace relay
{

struct ForwarderStats {
    uint64_t records    = 0; // valid records from the Pico
    uint64_t lines      = 0; // lines fully written to the Brain
    uint64_t superseded = 0; // records replaced by a newer one before sending
    uint64_t discarded  = 0; // unsent lines dropped by dropPending()
};

struct Pending {
    const char* data = nullptr;
    std::size_t size = 0;
};

class Forwarder {
  public:
    // Feeds UART bytes. Returns the number of valid records found.
    std::size_t receive(const uint8_t* data, std::size_t size);

    // After the UART reopens: forget a partial record.
    void resetInput() { scanner_.reset(); }

    // After the Brain port reopens or stalls: forget unsent output.
    void dropPending();

    // Bytes to write next; size 0 when nothing is waiting.
    Pending pending();

    // count bytes of pending() were written.
    void wrote(std::size_t count);

    const encoder_record::Record& lastRecord() const { return scanner_.record(); }
    const ForwarderStats&         stats() const { return stats_; }
    const encoder_record::ScanStats& scanStats() const { return scanner_.stats(); }

  private:
    encoder_record::RecordScanner scanner_;
    encoder_record::Line          next_{};
    bool                          has_next_ = false;
    encoder_record::Line          current_{};
    std::size_t                   current_at_ = 0;
    bool                          has_current_ = false;
    ForwarderStats                stats_;
};

} // namespace relay
