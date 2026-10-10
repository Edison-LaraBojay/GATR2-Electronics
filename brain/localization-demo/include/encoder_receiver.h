// encoder_receiver.h
// Encoder records from the Pi relay on stdin ("ENC1:" lines, encoder_record.h).
// One task reads stdin and keeps only the newest record; counts are
// cumulative, so skipping older ones loses no travel. latest() never waits
// for the Pi or for the reader task. PROS only.
//
// This is the program's only stdin reader. stdout and COBS are left alone
// for the spectaGATR publisher.

#pragma once
#include <atomic>
#include <cstdint>

#include "latest_record.h"

namespace localization
{

struct ReceiverStats {
    uint32_t records       = 0; // valid lines
    uint32_t skipped_lines = 0; // other text, bad hex, bad checksum
    uint32_t read_errors   = 0;
};

class EncoderReceiver {
  public:
    // Creates the reader task. Call once.
    bool start();

    // Copies the newest record. False if none yet.
    bool latest(ReceivedRecord& out) const { return newest_.load(out); }

    ReceiverStats stats() const;

  private:
    static void taskEntry(void* self) { static_cast<EncoderReceiver*>(self)->run(); }
    void        run();

    LatestRecord newest_;

    std::atomic<uint32_t> records_{0};
    std::atomic<uint32_t> skipped_{0};
    std::atomic<uint32_t> read_errors_{0};
};

} // namespace localization
