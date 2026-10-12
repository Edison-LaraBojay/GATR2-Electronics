// profile_host.h

#pragma once
#include "runtime/system.h"

namespace navigatr
{

// The System as the brain_link commands slot sees it.
class System::ProfileHost : public BrainProfileHost
{
public:
    explicit ProfileHost(System& system) : system_(system) {}

    bool prepare(const translagatr::RobotProfileDoc& profile, uint32_t profile_id, uint8_t& reason,
                 uint8_t& detail) override {
        return system_.prepareProfile(profile, profile_id, reason, detail);
    }

    std::shared_ptr<const ProfileBinding> applied() const override {
        return system_.profileBinding();
    }

    uint8_t control(uint8_t action, uint8_t arg, MonotonicTime now, uint8_t& detail) override {
        return system_.controlProfile(action, arg, now, detail);
    }

    uint8_t controlProgress(uint8_t action, uint8_t, MonotonicTime now,
                            uint8_t& detail) override {
        return system_.controlProgress(action, now, detail);
    }

    uint8_t readWheels(MonotonicTime now, uint8_t& count, translagatr::WheelReading* wheels) override {
        return system_.readWheels(now, count, wheels);
    }

private:
    System& system_;
};

} // namespace navigatr
