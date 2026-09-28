// sensor_loss.h
// The sources a Brain profile localizes with, watched for loss (spec 8.10):
// the profile encoders, the Pico IMU when the profile uses it, and the Brain
// VEX IMU mailbox when the profile uses that. Sources the profile does not
// use are never watched.
//
// A source is lost while it is stale longer than loss_ms (host receipt of
// its newest sample; for the VEX IMU its newest valid sample, an invalid one
// counts as missing) and while a used Pico IMU reports itself anything but
// ready. A change of its identity is a loss at that moment: a record epoch,
// an encoder discontinuity, a source epoch, the Brain IMU epoch (moved by an
// invalid sample or a new Brain session too), a Pico reboot or acquisition
// restart, or a restart of a used Pico IMU. A
// physically disconnected quadrature encoder keeps its last count and reads
// as standing still; that cannot be seen here. Estimation worker only.

#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "core/ids.h"
#include "core/records.h"
#include "resources/brain_imu_bench.h"
#include "resources/pico_control.h"

namespace navigatr
{

class SensorLossMonitor
{
public:
    struct Wheel {
        SensorId sensor;   // EncoderSample
        uint8_t  port = 0;
    };

    // Replaces the sources; staleness counts from now until each delivers.
    void configure(const std::vector<Wheel>& wheels, const SensorId& pico_imu, uint8_t imu_port,
                   std::shared_ptr<const BrainImuBench> vex, int64_t loss_ms, MonotonicTime now);
    void clear();
    bool active() const { return !sources_.empty(); }

    struct Result {
        std::string edge;    // a source restarted this cycle
        std::string level;   // a source is stale or not ready now
    };

    // pico is the link state when a Pico link is configured, else null.
    Result update(const SensorMap& sensors, const PicoLinkState* pico, MonotonicTime now);

private:
    enum class Kind : uint8_t { kEncoder, kPicoImu, kBrainImu };
    struct Source {
        Kind          kind = Kind::kEncoder;
        std::string   name;
        SensorId      sensor;
        bool          seen     = false;
        uint64_t      epoch    = 0;
        uint64_t      identity = 0;   // discontinuity, source epoch or mailbox epoch
        MonotonicTime last;           // newest usable receipt, or configure time
    };

    std::vector<Source>                  sources_;
    std::shared_ptr<const BrainImuBench> vex_;
    int64_t                              loss_ms_  = 250;
    bool                                 uses_pico_imu_ = false;
    bool                                 pico_seen_     = false;
    uint64_t                             reboots_ = 0, restarts_ = 0, imu_restarts_ = 0;
};

} // namespace navigatr
