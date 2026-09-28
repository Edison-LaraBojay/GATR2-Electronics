// brain_profile.h
// The System side of a Brain robot profile. The brain_link commands slot
// stages the document, decodes it and runs the shared semantic check; the
// host runs this Pi's capability checks (wired encoder ports, IMU, camera
// slots), builds the candidate at once and swaps it in at the next
// controlled boundary. No host means this configuration takes no profile.
//
// The host also answers what depends on the running profile: the sensors
// its health reports on, and CONTROL. Called on the estimation worker,
// except applied(), which any thread may call.

#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "common/link_documents.h"
#include "core/ids.h"
#include "core/time.h"
#include "resources/brain_imu_bench.h"

namespace navigatr
{

// What the running profile uses, fixed from one apply to the next.
struct ProfileBinding {
    uint32_t               id = 0;
    gatr2::RobotProfileDoc profile;
    uint64_t               generation = 0;   // counts applies

    std::vector<SensorId>          encoders;    // one per profile wheel, profile order
    SensorId                       imu;         // Pico IMU channel; empty unless IMU source pico
    std::shared_ptr<BrainImuBench> bench_imu;   // Brain VEX IMU mailbox; brain_vex only
    std::string bias_function;   // observation function owning the IMU bias; empty = none
    std::string summary;         // human readable, for inspection and logs
};

class BrainProfileHost
{
public:
    virtual ~BrainProfileHost() = default;

    // Capability checks and candidate build. False: reason is a
    // gatr2::ProfileReason, detail the wheel or camera index.
    virtual bool prepare(const gatr2::RobotProfileDoc& profile, uint32_t profile_id,
                         uint8_t& reason, uint8_t& detail) = 0;

    // The running profile; null before the first apply.
    virtual std::shared_ptr<const ProfileBinding> applied() const = 0;

    // CONTROL action (gatr2::ControlAction). Returns a gatr2::BrainResult;
    // detail is a gatr2::ControlDetail.
    virtual uint8_t control(uint8_t action, uint8_t arg, MonotonicTime now,
                            uint8_t& detail) = 0;
};

} // namespace navigatr
