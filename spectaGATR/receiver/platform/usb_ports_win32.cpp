// usb_ports_win32.cpp
// Windows: Ports class devices from SetupAPI; ids, product and location from CfgMgr32.

#if defined(_WIN32)

#include "receiver/platform/usb_ports.h"

#include <array>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

// Before the headers below, so their GUIDs and property keys are defined here.
#include <initguid.h>

#include <cfgmgr32.h>
#include <devguid.h>
#include <devpkey.h>
#include <setupapi.h>

#include "receiver/platform/win32_text.h"

namespace spectagatr::receiver::platform
{
namespace
{

using WideText = std::array<wchar_t, 256>;

std::string portName(HDEVINFO devices, SP_DEVINFO_DATA& device) {
    HKEY key = SetupDiOpenDevRegKey(devices, &device, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
    if (key == INVALID_HANDLE_VALUE) {
        return {};
    }
    WideText   name{};
    DWORD      bytes  = sizeof(name) - sizeof(wchar_t); // room for the terminator
    DWORD      type   = 0;
    const LONG status = RegQueryValueExW(key, L"PortName", nullptr, &type,
                                         reinterpret_cast<BYTE*>(name.data()), &bytes);
    RegCloseKey(key);
    if (status != ERROR_SUCCESS || type != REG_SZ) {
        return {};
    }
    return toUtf8(name.data());
}

std::string deviceId(DEVINST node) {
    std::array<wchar_t, MAX_DEVICE_ID_LEN + 1> id{};
    if (CM_Get_Device_IDW(node, id.data(), MAX_DEVICE_ID_LEN, 0) != CR_SUCCESS) {
        return {};
    }
    return toUtf8(id.data());
}

// Port_#0002.Hub_#0001
std::string locationOf(DEVINST node) {
    WideText text{};
    ULONG    bytes = sizeof(text) - sizeof(wchar_t);
    ULONG    type  = 0;
    if (CM_Get_DevNode_Registry_PropertyW(node, CM_DRP_LOCATION_INFORMATION, &type, text.data(),
                                          &bytes, 0) != CR_SUCCESS ||
        type != REG_SZ) {
        return {};
    }
    return toUtf8(text.data());
}

// The device's iProduct string.
std::string productOf(DEVINST node) {
    WideText    text{};
    ULONG       bytes = sizeof(text) - sizeof(wchar_t);
    DEVPROPTYPE type  = 0;
    if (CM_Get_DevNode_PropertyW(node, &DEVPKEY_Device_BusReportedDeviceDesc, &type,
                                 reinterpret_cast<PBYTE>(text.data()), &bytes, 0) != CR_SUCCESS ||
        type != DEVPROP_TYPE_STRING) {
        return {};
    }
    return toUtf8(text.data());
}

std::optional<UsbSerialPort> describePort(HDEVINFO devices, SP_DEVINFO_DATA& device) {
    UsbSerialPort port;
    port.path = portName(devices, device);
    if (port.path.rfind("COM", 0) != 0) {
        return std::nullopt; // LPT and unnamed ports
    }
    const std::optional<WindowsDeviceId> id = parseWindowsDeviceId(deviceId(device.DevInst));
    if (!id) {
        return std::nullopt; // not USB
    }
    port.vendor_id        = id->vendor_id;
    port.product_id       = id->product_id;
    port.interface_number = id->interface_number;

    // An interface node's USB device is its composite parent.
    DEVINST     usb_device = device.DevInst;
    std::string instance   = id->instance;
    if (id->interface_number) {
        if (CM_Get_Parent(&usb_device, device.DevInst, 0) != CR_SUCCESS) {
            return port;
        }
        const std::optional<WindowsDeviceId> parent = parseWindowsDeviceId(deviceId(usb_device));
        instance                                    = parent ? parent->instance : std::string();
    }
    port.serial   = serialFromWindowsInstance(instance);
    port.product  = productOf(usb_device);
    port.location = locationOf(usb_device);
    return port;
}

} // namespace

UsbSerialPorts listUsbSerialPorts() {
    UsbSerialPorts ports;
    HDEVINFO devices = SetupDiGetClassDevsW(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (devices == INVALID_HANDLE_VALUE) {
        return ports;
    }
    SP_DEVINFO_DATA device{};
    device.cbSize = sizeof(device);
    for (DWORD index = 0; SetupDiEnumDeviceInfo(devices, index, &device); ++index) {
        std::optional<UsbSerialPort> port = describePort(devices, device);
        if (port) {
            ports.push_back(std::move(*port));
        }
    }
    SetupDiDestroyDeviceInfoList(devices);
    return ports;
}

} // namespace spectagatr::receiver::platform

#endif
