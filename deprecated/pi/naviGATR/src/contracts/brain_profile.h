// brain_profile.h
// The System side of a Brain robot profile. The brain_link commands slot
// stages the document, decodes it and runs the shared semantic check; the
// host runs this Pi's capability checks (wired encoder ports, IMU, camera
// slots), builds the candidate at once and swaps it in at the next
// controlled boundary. No host means this configuration takes no profile.
//
// The host also answers what depends on the running profile: the sensors
// its health reports on, CONTROL and READ_WHEELS. Called on the estimation
// worker, except applied(), which any thread may call.

#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "translaGATR/frames.h"
#include "translaGATR/link_documents.h"
#include "core/ids.h"
#include "core/time.h"
#include "resources/brain_imu_bench.h"

namespace navigatr
{

// What the running profile uses, fixed from one apply to the next.
struct ProfileBinding {
    uint32_t               id = 0;
    translagatr::RobotProfileDoc profile;
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
    // translagatr::ProfileReason, detail the wheel or camera index.
    virtual bool prepare(const translagatr::RobotProfileDoc& profile, uint32_t profile_id,
                         uint8_t& reason, uint8_t& detail) = 0;

    // The running profile; null before the first apply.
    virtual std::shared_ptr<const ProfileBinding> applied() const = 0;

    // CONTROL action (translagatr::ControlAction). Returns a translagatr::BrainResult;
    // detail is a translagatr::ControlDetail. Pending: the Pico is still working.
    virtual uint8_t control(uint8_t action, uint8_t arg, MonotonicTime now,
                            uint8_t& detail) = 0;

    // Progress of the CONTROL a duplicate request names, whose recorded
    // result was Pending. Reports only; never starts or resubmits anything.
    virtual uint8_t controlProgress(uint8_t action, uint8_t arg, MonotonicTime now,
                                    uint8_t& detail) = 0;

    // READ_WHEELS: one reading per profile wheel, profile order, into
    // wheels[0..count). A translagatr::BrainResult; NotReady before an apply.
    virtual uint8_t readWheels(MonotonicTime now, uint8_t& count,
                               translagatr::WheelReading* wheels) = 0;
};

} // namespace navigatr
