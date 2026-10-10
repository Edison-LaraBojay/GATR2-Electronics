// usb_ports_linux.cpp

#if defined(__linux__)

#include "receiver/platform/usb_ports.h"

namespace spectagatr::receiver::platform
{

UsbSerialPorts listUsbSerialPorts() { return listSysfsSerialPorts("/sys/class/tty", "/dev"); }

} // namespace spectagatr::receiver::platform

#endif
