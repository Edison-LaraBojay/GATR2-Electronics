// pico_telemetry.h
// Shared decoder for one Pico telemetry stream. The Pico packs several
// physical sensors into one packet; this resource drains and decodes the
// serial link at most once per runtime cycle and keeps a coherent snapshot
// of the latest value per channel, so every logical channel sensor reads
// from the same decoded packet and nobody re-drains the UART. All Pico wire
// knowledge lives here and in common/, nowhere else in navigatr.
//
//   <Resource id="pico_telemetry" type="resource/pico_telemetry">
//       <Serial resource_id="pico_uart"/>
//   </Resource>
//
// Thread safety: single-threaded; refresh and channel reads happen on the
// pipeline thread. Cycle-snapshot based for consumers.

#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "common/frame_codec.h"
#include "core/time.h"
#include "resources/resource_store.h"
#include "resources/serial_link.h"

namespace navigatr
{

struct Diagnostics;

class PicoTelemetry
{
public:
    PicoTelemetry(std::shared_ptr<SerialLink> link, std::string diagnostics_id);

    // Drains and decodes at most once per cycle; later calls in the same
    // cycle see the identical snapshot.
    void refresh(uint64_t cycle, Diagnostics* diagnostics);

    struct Channel {
        bool          present = false;
        int32_t       value[2] = {0, 0};
        MonotonicTime measuredAt;   // device clock
        uint64_t      updates = 0;  // counts distinct decoded publications
    };

    static constexpr int kEncoderChannels = 3;

    const Channel& encoder(int channel) const { return encoders_[channel]; }
    const Channel& gyro() const { return gyro_; }
    const Channel& accel() const { return accel_; }

    bool linkDead() const { return link_dead_; }

    void reset();

private:
    void applyPacket(const gatr2::SensorSample& s, Diagnostics* diagnostics);

    std::shared_ptr<SerialLink> link_;
    std::string                 diagnostics_id_;
    gatr2::FrameReader          reader_;

    Channel encoders_[kEncoderChannels];
    Channel gyro_;
    Channel accel_;

    bool     have_seq_        = false;
    uint8_t  last_seq_        = 0;
    uint64_t last_poll_cycle_ = 0;
    bool     polled_once_     = false;
    bool     link_dead_       = false;
};

ResourceValue make_pico_telemetry(const ConfigNode& node,
                                  ResourceInitializationContext& context,
                                  std::string& err);

} // namespace navigatr
