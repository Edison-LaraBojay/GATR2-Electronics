// brain_port.cpp
// Same rules as the archived naviGATR pros_usb_link discovery, which worked
// on the robot.

#include "brain_port.h"

#include <filesystem>
#include <fstream>

namespace relay
{

namespace
{

namespace fs = std::filesystem;

std::string attribute(const fs::path& path) {
    std::ifstream file(path);
    std::string   value;
    file >> value;
    return value;
}

} // namespace

BrainPortSearch findBrainUserPort(const std::string& tty_root, const std::string& device_root) {
    BrainPortSearch       result;
    std::string           found;
    std::error_code       error;
    fs::directory_iterator it(tty_root, error), end;
    for (; !error && it != end; it.increment(error)) {
        const std::string name = it->path().filename().string();
        if (name.compare(0, 6, "ttyACM") != 0) {
            continue;
        }
        std::error_code canonical_error;
        fs::path        path = fs::canonical(it->path() / "device", canonical_error);
        if (canonical_error) {
            continue;
        }
        // The interface number sits on the interface directory, vendor and
        // product on the USB device directory above it.
        std::string interface_number, vendor, product;
        for (int depth = 0; depth < 12 && !path.empty(); ++depth) {
            if (interface_number.empty()) {
                interface_number = attribute(path / "bInterfaceNumber");
            }
            if (vendor.empty()) {
                vendor = attribute(path / "idVendor");
            }
            if (product.empty()) {
                product = attribute(path / "idProduct");
            }
            const fs::path parent = path.parent_path();
            if (parent == path) {
                break;
            }
            path = parent;
        }
        if (interface_number != "02" || vendor != "2888" || product != "0501") {
            continue;
        }
        ++result.brains;
        found = (fs::path(device_root) / name).string();
    }
    if (!error && result.brains == 1) {
        result.path = found;
    }
    return result;
}

} // namespace relay
