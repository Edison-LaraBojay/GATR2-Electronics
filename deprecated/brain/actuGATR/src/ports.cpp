// ports.cpp

#include "actugatr/ports.h"

#include <cstdio>

namespace actugatr
{

bool PortMap::add(uint8_t port, const char* device) {
    if (port < 1 || port > 21) {
        if (ok()) {
            std::snprintf(error_, sizeof(error_), "%s: port %u outside 1..21", device,
                          static_cast<unsigned>(port));
        }
        return false;
    }
    if (owners_[port] != nullptr) {
        if (ok()) {
            std::snprintf(error_, sizeof(error_), "port %u: %s and %s", static_cast<unsigned>(port),
                          owners_[port], device);
        }
        return false;
    }
    owners_[port] = device;
    return true;
}

bool PortMap::add(const MotorGroup& group, const char* device) {
    bool all = true;
    for (std::size_t i = 0; i < group.count && i < kMaxMotorsPerGroup; ++i) {
        all = add(group.motors[i].port, device) && all;
    }
    return all;
}

bool PortMap::add(const TankConfig& c) {
    const bool left = add(c.left, "left drive");
    return add(c.right, "right drive") && left;
}

bool PortMap::add(const MecanumConfig& c) {
    bool all = add(c.front_left, "front left drive");
    all      = add(c.front_right, "front right drive") && all;
    all      = add(c.rear_left, "rear left drive") && all;
    return add(c.rear_right, "rear right drive") && all;
}

const char* PortMap::owner(uint8_t port) const {
    return port >= 1 && port <= 21 ? owners_[port] : nullptr;
}

} // namespace actugatr
