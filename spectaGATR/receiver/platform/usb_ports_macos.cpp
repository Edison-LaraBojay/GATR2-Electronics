// usb_ports_macos.cpp
// macOS: IOKit serial services (callout /dev/cu.* names) with a USB device above them.

#if defined(__APPLE__)

#include "receiver/platform/usb_ports.h"

#include <array>
#include <cstdio>

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>

namespace spectagatr::receiver::platform
{
namespace
{

using RegistryEntry = io_registry_entry_t;
using PropertyValue = std::int64_t;

constexpr int kMaxRegistryDepth = 16;

// The caller releases the value.
CFTypeRef copyProperty(RegistryEntry entry, const char* key) {
    CFStringRef name  = CFStringCreateWithCString(kCFAllocatorDefault, key, kCFStringEncodingUTF8);
    CFTypeRef   value = IORegistryEntryCreateCFProperty(entry, name, kCFAllocatorDefault, 0);
    CFRelease(name);
    return value;
}

std::string stringProperty(RegistryEntry entry, const char* key) {
    CFTypeRef   value = copyProperty(entry, key);
    std::string text;
    if (value != nullptr && CFGetTypeID(value) == CFStringGetTypeID()) {
        std::array<char, 256> buffer{};
        if (CFStringGetCString(static_cast<CFStringRef>(value), buffer.data(),
                               static_cast<CFIndex>(buffer.size()), kCFStringEncodingUTF8)) {
            text = buffer.data();
        }
    }
    if (value != nullptr) {
        CFRelease(value);
    }
    return text;
}

std::optional<PropertyValue> numberProperty(RegistryEntry entry, const char* key) {
    CFTypeRef                    value = copyProperty(entry, key);
    std::optional<PropertyValue> number;
    if (value != nullptr && CFGetTypeID(value) == CFNumberGetTypeID()) {
        PropertyValue out = 0;
        if (CFNumberGetValue(static_cast<CFNumberRef>(value), kCFNumberSInt64Type, &out)) {
            number = out;
        }
    }
    if (value != nullptr) {
        CFRelease(value);
    }
    return number;
}

// Nearest ancestor of either class, retained for the caller; 0 when none.
RegistryEntry ancestorOf(RegistryEntry entry, const char* first_class, const char* second_class) {
    RegistryEntry current = entry;
    IOObjectRetain(current);
    for (int depth = 0; depth < kMaxRegistryDepth; ++depth) {
        RegistryEntry       parent = 0;
        const kern_return_t result =
            IORegistryEntryGetParentEntry(current, kIOServicePlane, &parent);
        IOObjectRelease(current);
        if (result != KERN_SUCCESS) {
            return 0;
        }
        if (IOObjectConformsTo(parent, first_class) || IOObjectConformsTo(parent, second_class)) {
            return parent;
        }
        current = parent;
    }
    IOObjectRelease(current);
    return 0;
}

std::optional<UsbInterfaceNumber> functionInterface(RegistryEntry service) {
    const RegistryEntry usb_interface = ancestorOf(service, "IOUSBHostInterface", "IOUSBInterface");
    if (usb_interface == 0) {
        return std::nullopt;
    }
    const std::optional<PropertyValue> number = numberProperty(usb_interface, "bInterfaceNumber");
    const std::optional<PropertyValue> type   = numberProperty(usb_interface, "bInterfaceClass");
    IOObjectRelease(usb_interface);
    if (!number) {
        return std::nullopt;
    }
    return cdcFunctionInterface(*number, type);
}

std::optional<UsbSerialPort> describeService(RegistryEntry service) {
    UsbSerialPort port;
    port.path = stringProperty(service, kIOCalloutDeviceKey);
    if (port.path.empty()) {
        return std::nullopt;
    }
    const RegistryEntry device = ancestorOf(service, "IOUSBHostDevice", "IOUSBDevice");
    if (device == 0) {
        return std::nullopt; // not USB
    }
    const std::optional<PropertyValue> vendor   = numberProperty(device, "idVendor");
    const std::optional<PropertyValue> product  = numberProperty(device, "idProduct");
    const std::optional<PropertyValue> location = numberProperty(device, "locationID");
    port.serial                                 = stringProperty(device, "USB Serial Number");
    port.product                                = stringProperty(device, "USB Product Name");
    IOObjectRelease(device);
    if (!vendor || !product) {
        return std::nullopt;
    }
    port.vendor_id  = static_cast<UsbId>(*vendor);
    port.product_id = static_cast<UsbId>(*product);
    if (location) {
        std::array<char, 16> text{};
        std::snprintf(text.data(), text.size(), "0x%08llx",
                      static_cast<unsigned long long>(*location & 0xFFFFFFFF));
        port.location = text.data();
    }
    port.interface_number = functionInterface(service);
    return port;
}

} // namespace

UsbSerialPorts listUsbSerialPorts() {
    UsbSerialPorts         ports;
    CFMutableDictionaryRef matching = IOServiceMatching(kIOSerialBSDServiceValue);
    if (matching == nullptr) {
        return ports;
    }
    io_iterator_t services = 0;
    // Default main port; consumes matching.
    if (IOServiceGetMatchingServices(MACH_PORT_NULL, matching, &services) != KERN_SUCCESS) {
        return ports;
    }
    for (io_object_t service = IOIteratorNext(services); service != 0;
         service             = IOIteratorNext(services)) {
        std::optional<UsbSerialPort> port = describeService(service);
        IOObjectRelease(service);
        if (port) {
            ports.push_back(std::move(*port));
        }
    }
    IOObjectRelease(services);
    return ports;
}

} // namespace spectagatr::receiver::platform

#endif
