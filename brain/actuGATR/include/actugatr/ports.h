// ports.h
// V5 Smart Port bookkeeping: every active device on its own port 1..21.
// Applications list the motors, the VEX IMU when it is the IMU source, and
// the Smart Port link when RS-485 is selected.

#pragma once
#include <cstddef>
#include <cstdint>

#include "actugatr/drivetrain.h"

namespace actugatr
{

class PortMap {
public:
    // False, with error() naming both devices, when the port is outside 1..21
    // or already taken. The first problem is kept.
    bool add(uint8_t port, const char* device);
    bool add(const MotorGroup& group, const char* device);
    bool add(const TankConfig& config);
    bool add(const MecanumConfig& config);

    bool        ok() const { return error_[0] == '\0'; }
    const char* error() const { return error_; }

    // Device on a port, nullptr when free.
    const char* owner(uint8_t port) const;

private:
    const char* owners_[22] = {};
    char        error_[80]  = {};
};

// For static_assert over constant port lists: every port in 1..21, none twice.
template <std::size_t N> constexpr bool distinctPorts(const uint8_t (&ports)[N]) {
    for (std::size_t i = 0; i < N; ++i) {
        if (ports[i] < 1 || ports[i] > 21) {
            return false;
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (ports[i] == ports[j]) {
                return false;
            }
        }
    }
    return true;
}

} // namespace actugatr
