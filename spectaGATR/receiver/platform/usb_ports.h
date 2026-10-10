// usb_ports.h
// USB serial ports present now, with the USB facts that identify them. Never opens a port.

#pragma once
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace spectagatr::receiver::platform
{

using UsbId              = std::uint16_t;
using UsbInterfaceNumber = std::uint8_t;
using Path               = std::filesystem::path;

struct UsbSerialPort {
    std::string                       path; // COM7, /dev/cu.usbmodem1103, /dev/ttyACM1
    UsbId                             vendor_id  = 0;
    UsbId                             product_id = 0;
    std::optional<UsbInterfaceNumber> interface_number; // first interface of the CDC function
    std::string                       serial;           // iSerialNumber, empty when none
    std::string                       product;          // iProduct, empty when unknown
    std::string                       location;         // bus position, empty when unknown
};
using UsbSerialPorts = std::vector<UsbSerialPort>;

// One implementation per OS. Empty when listing fails.
UsbSerialPorts listUsbSerialPorts();

// Portable parts of the OS listings, tested on every OS.

struct WindowsDeviceId {
    UsbId                             vendor_id  = 0;
    UsbId                             product_id = 0;
    std::optional<UsbInterfaceNumber> interface_number; // MI_xx
    std::string                       instance;         // after the last backslash
};

// USB\VID_2888&PID_0501&MI_02\6&A694CA9&0&0002. Absent for non-USB or malformed ids.
[[nodiscard]] std::optional<WindowsDeviceId> parseWindowsDeviceId(std::string_view id);

// The USB serial number when Windows used it as the instance id, else empty.
std::string serialFromWindowsInstance(std::string_view instance);

// Linux sysfs: ttyACM* and ttyUSB* under class_root (/sys/class/tty), named under device_root.
UsbSerialPorts listSysfsSerialPorts(const Path& class_root, const Path& device_root);

// macOS: the interface the tty hangs under. A CDC data interface (class 0x0A) gives the one
// before it. Absent when out of range.
[[nodiscard]] std::optional<UsbInterfaceNumber>
cdcFunctionInterface(std::int64_t interface_number, std::optional<std::int64_t> interface_class);

} // namespace spectagatr::receiver::platform
