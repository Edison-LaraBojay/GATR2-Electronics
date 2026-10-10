// usb_ports.cpp
// Windows device id parsing and the sysfs walk. The OS listings are in usb_ports_<os>.cpp.

#include "receiver/platform/usb_ports.h"

#include <algorithm>
#include <fstream>
#include <system_error>

namespace spectagatr::receiver::platform
{
namespace
{

constexpr int          kMaxSysfsDepth = 12;
constexpr std::int64_t kCdcDataClass  = 0x0A;

char upper(char c) { return (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c; }

bool startsWithNoCase(std::string_view text, std::string_view prefix) {
    if (text.size() < prefix.size()) {
        return false;
    }
    for (std::size_t i = 0; i < prefix.size(); ++i) {
        if (upper(text[i]) != upper(prefix[i])) {
            return false;
        }
    }
    return true;
}

std::optional<unsigned> hexDigit(char c) {
    if (c >= '0' && c <= '9') {
        return static_cast<unsigned>(c - '0');
    }
    const char u = upper(c);
    if (u >= 'A' && u <= 'F') {
        return static_cast<unsigned>(u - 'A' + 10);
    }
    return std::nullopt;
}

// 1 to 4 hex digits, nothing else.
std::optional<unsigned> parseHex(std::string_view digits) {
    if (digits.empty() || digits.size() > 4) {
        return std::nullopt;
    }
    unsigned value = 0;
    for (char c : digits) {
        const std::optional<unsigned> digit = hexDigit(c);
        if (!digit) {
            return std::nullopt;
        }
        value = value * 16 + *digit;
    }
    return value;
}

std::optional<UsbId> usbIdOf(std::string_view digits) {
    const std::optional<unsigned> value = parseHex(digits);
    if (!value) {
        return std::nullopt;
    }
    return static_cast<UsbId>(*value);
}

std::optional<UsbInterfaceNumber> interfaceNumberOf(std::string_view digits) {
    const std::optional<unsigned> value = parseHex(digits);
    if (!value || *value > 0xFF) {
        return std::nullopt;
    }
    return static_cast<UsbInterfaceNumber>(*value);
}

// One line of a sysfs attribute without trailing whitespace. Absent when the file is.
std::optional<std::string> readAttribute(const Path& dir, const char* name) {
    std::ifstream file(dir / name);
    if (!file) {
        return std::nullopt;
    }
    std::string value;
    std::getline(file, value);
    while (!value.empty() && (value.back() == '\r' || value.back() == ' ')) {
        value.pop_back();
    }
    return value;
}

bool usbSerialName(const std::string& name) {
    return name.rfind("ttyACM", 0) == 0 || name.rfind("ttyUSB", 0) == 0;
}

// Walks up from the tty's device: the first bInterfaceNumber is the interface, the first
// idVendor is the USB device. Absent when no USB device is above.
std::optional<UsbSerialPort> describeSysfsDevice(const Path& device_dir) {
    UsbSerialPort port;
    Path          dir = device_dir;
    for (int depth = 0; depth < kMaxSysfsDepth && !dir.empty(); ++depth) {
        if (!port.interface_number) {
            const std::optional<std::string> number = readAttribute(dir, "bInterfaceNumber");
            if (number) {
                port.interface_number = interfaceNumberOf(*number);
            }
        }
        const std::optional<std::string> vendor = readAttribute(dir, "idVendor");
        if (vendor) {
            const std::optional<std::string> product_id = readAttribute(dir, "idProduct");
            const std::optional<UsbId>       vid        = usbIdOf(*vendor);
            const std::optional<UsbId>       pid = product_id ? usbIdOf(*product_id) : std::nullopt;
            if (!vid || !pid) {
                return std::nullopt;
            }
            port.vendor_id  = *vid;
            port.product_id = *pid;
            port.serial     = readAttribute(dir, "serial").value_or("");
            port.product    = readAttribute(dir, "product").value_or("");
            port.location   = dir.filename().string();
            return port;
        }
        const Path parent = dir.parent_path();
        if (parent == dir) {
            break;
        }
        dir = parent;
    }
    return std::nullopt;
}

} // namespace

std::optional<WindowsDeviceId> parseWindowsDeviceId(std::string_view id) {
    const std::size_t first = id.find('\\');
    const std::size_t last  = id.rfind('\\');
    if (first == std::string_view::npos || last == first || !startsWithNoCase(id, "USB\\")) {
        return std::nullopt;
    }
    const std::string_view hardware = id.substr(first + 1, last - first - 1);

    WindowsDeviceId      result;
    std::optional<UsbId> vendor;
    std::optional<UsbId> product;
    std::size_t          start = 0;
    while (start <= hardware.size()) {
        const std::size_t      end   = std::min(hardware.find('&', start), hardware.size());
        const std::string_view token = hardware.substr(start, end - start);
        if (startsWithNoCase(token, "VID_") && token.size() == 8) {
            vendor = usbIdOf(token.substr(4));
        } else if (startsWithNoCase(token, "PID_") && token.size() == 8) {
            product = usbIdOf(token.substr(4));
        } else if (startsWithNoCase(token, "MI_") && token.size() == 5) {
            result.interface_number = interfaceNumberOf(token.substr(3));
        }
        start = end + 1;
    }
    if (!vendor || !product) {
        return std::nullopt;
    }
    result.vendor_id  = *vendor;
    result.product_id = *product;
    result.instance   = std::string(id.substr(last + 1));
    return result;
}

std::string serialFromWindowsInstance(std::string_view instance) {
    // Windows generates ids with '&' for devices without a serial number.
    if (instance.empty() || instance.find('&') != std::string_view::npos) {
        return {};
    }
    return std::string(instance);
}

UsbSerialPorts listSysfsSerialPorts(const Path& class_root, const Path& device_root) {
    namespace fs = std::filesystem;
    UsbSerialPorts         ports;
    std::error_code        error;
    fs::directory_iterator it(class_root, error);
    for (; !error && it != fs::directory_iterator(); it.increment(error)) {
        const std::string name = it->path().filename().string();
        if (!usbSerialName(name)) {
            continue;
        }
        std::error_code resolve_error;
        const Path      device_dir = fs::canonical(it->path() / "device", resolve_error);
        if (resolve_error) {
            continue;
        }
        std::optional<UsbSerialPort> port = describeSysfsDevice(device_dir);
        if (!port) {
            continue;
        }
        port->path = (device_root / name).generic_string();
        ports.push_back(std::move(*port));
    }
    return ports;
}

std::optional<UsbInterfaceNumber>
cdcFunctionInterface(std::int64_t interface_number, std::optional<std::int64_t> interface_class) {
    if (interface_number < 0 || interface_number > 0xFF) {
        return std::nullopt;
    }
    const bool data_interface = interface_class == kCdcDataClass && interface_number > 0;
    return static_cast<UsbInterfaceNumber>(data_interface ? interface_number - 1
                                                          : interface_number);
}

} // namespace spectagatr::receiver::platform
