// brain_port.h
// Finds the V5 Brain's USB user port: USB 2888:0501, interface 02. Interface
// 00 is the system/upload port and is never chosen, whatever ttyACM number
// either one gets. Portable (std::filesystem over sysfs).

#pragma once
#include <string>

namespace relay
{

struct BrainPortSearch {
    std::string path;       // empty unless exactly one Brain was found
    int         brains = 0; // user ports seen
};

BrainPortSearch findBrainUserPort(const std::string& tty_root   = "/sys/class/tty",
                                  const std::string& device_root = "/dev");

} // namespace relay
