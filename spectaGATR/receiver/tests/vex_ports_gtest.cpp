// vex_ports_gtest.cpp
// Roles and identities from USB facts: Windows, macOS and Linux shaped fixtures, picking rules.

#include "receiver/platform/vex_ports.h"

#include <gtest/gtest.h>
#include <string>

namespace spectagatr::receiver::platform
{
namespace
{

UsbSerialPort usbPort(const std::string& path, UsbId pid,
                      std::optional<UsbInterfaceNumber> interface_number,
                      const std::string&                location = "Port_#0002.Hub_#0001") {
    UsbSerialPort port;
    port.path             = path;
    port.vendor_id        = kVexVendorId;
    port.product_id       = pid;
    port.interface_number = interface_number;
    port.location         = location;
    return port;
}

// As listUsbSerialPorts gives them on Windows: COM numbers in no useful order.
UsbSerialPorts windowsPorts() {
    UsbSerialPort ftdi;
    ftdi.path       = "COM3";
    ftdi.vendor_id  = 0x0403;
    ftdi.product_id = 0x6001;
    return {
        usbPort("COM9", kV5BrainProductId, 0),
        ftdi,
        usbPort("COM4", kV5BrainProductId, 2),
        usbPort("COM12", kV5ControllerProductId, 1, "Port_#0003.Hub_#0001"),
    };
}

TEST(VexPorts, RolesComeFromProductAndInterface) {
    EXPECT_EQ(vexRoleOf(usbPort("COM9", kV5BrainProductId, 0)), VexRole::kBrainSystem);
    EXPECT_EQ(vexRoleOf(usbPort("COM4", kV5BrainProductId, 2)), VexRole::kBrainUser);
    EXPECT_EQ(vexRoleOf(usbPort("COM12", kV5ControllerProductId, 1)), VexRole::kController);
    EXPECT_EQ(vexRoleOf(usbPort("COM13", kV5ControllerProductId, std::nullopt)),
              VexRole::kController);
}

TEST(VexPorts, UnknownInterfaceIsNeverGuessed) {
    EXPECT_EQ(vexRoleOf(usbPort("COM4", kV5BrainProductId, std::nullopt)), VexRole::kUnknown);
    EXPECT_EQ(vexRoleOf(usbPort("COM4", kV5BrainProductId, 1)), VexRole::kUnknown);
    EXPECT_EQ(vexRoleOf(usbPort("COM4", kV5BrainProductId, 3)), VexRole::kUnknown);
    EXPECT_EQ(vexRoleOf(usbPort("COM4", 0x0600, 2)), VexRole::kUnknown);

    UsbSerialPort other = usbPort("COM4", kV5BrainProductId, 2);
    other.vendor_id     = 0x1234;
    EXPECT_EQ(vexRoleOf(other), VexRole::kUnknown);
}

TEST(VexPorts, WindowsListKeepsVexOnlyAndSharesBrainIdentity) {
    const VexPorts ports = classifyVexPorts(windowsPorts());
    ASSERT_EQ(ports.size(), 3u);
    EXPECT_EQ(ports[0].port.path, "COM9");
    EXPECT_EQ(ports[0].role, VexRole::kBrainSystem);
    EXPECT_EQ(ports[1].port.path, "COM4");
    EXPECT_EQ(ports[1].role, VexRole::kBrainUser);
    EXPECT_EQ(ports[2].role, VexRole::kController);
    EXPECT_EQ(ports[0].identity, ports[1].identity);
    EXPECT_NE(ports[1].identity, ports[2].identity);
    EXPECT_EQ(ports[1].identity, "vex:0501@Port_#0002.Hub_#0001");
}

TEST(VexPorts, MacNamesAreIgnored) {
    // pros-cli reads the last digit of the name (3 user, 1 system); these names would mislead it.
    const UsbSerialPorts listed = {
        usbPort("/dev/cu.usbmodem14203", kV5BrainProductId, 0, "0x14200000"),
        usbPort("/dev/cu.usbmodem14201", kV5BrainProductId, 2, "0x14200000"),
    };
    const VexPorts ports = classifyVexPorts(listed);
    ASSERT_EQ(ports.size(), 2u);
    EXPECT_EQ(ports[0].role, VexRole::kBrainSystem);
    EXPECT_EQ(ports[1].role, VexRole::kBrainUser);
    EXPECT_EQ(ports[1].identity, "vex:0501@0x14200000");
}

TEST(VexPorts, IdentityPrefersSerialThenProductSuffixThenLocationThenPath) {
    UsbSerialPort port = usbPort("/dev/ttyACM1", kV5BrainProductId, 2, "1-1.2");
    port.serial        = "SN123";
    port.product       = "V5 Brain - A9EC2700";
    EXPECT_EQ(vexIdentityOf(port), "vex:0501:SN123");
    port.serial.clear();
    EXPECT_EQ(vexIdentityOf(port), "vex:0501:A9EC2700");
    port.product = "V5 Controller";
    EXPECT_EQ(vexIdentityOf(port), "vex:0501@1-1.2");
    port.location.clear();
    EXPECT_EQ(vexIdentityOf(port), "vex:0501@/dev/ttyACM1");
}

TEST(VexPorts, PicksTheOnePortWithTheRole) {
    const VexPorts               ports = classifyVexPorts(windowsPorts());
    std::string                  err;
    const std::optional<VexPort> user = pickVexPort(ports, VexRole::kBrainUser, std::nullopt, err);
    ASSERT_TRUE(user) << err;
    EXPECT_EQ(user->port.path, "COM4");

    const std::optional<VexPort> controller =
        pickVexPort(ports, VexRole::kController, std::nullopt, err);
    ASSERT_TRUE(controller) << err;
    EXPECT_EQ(controller->port.path, "COM12");
}

TEST(VexPorts, RefusesTwoBrainsAndListsThem) {
    UsbSerialPorts listed = windowsPorts();
    listed.push_back(usbPort("COM20", kV5BrainProductId, 2, "Port_#0004.Hub_#0002"));
    const VexPorts ports = classifyVexPorts(listed);

    std::string err;
    EXPECT_FALSE(pickVexPort(ports, VexRole::kBrainUser, std::nullopt, err));
    EXPECT_NE(err.find("2 VEX brain_user ports"), std::string::npos) << err;
    EXPECT_NE(err.find("COM4"), std::string::npos) << err;
    EXPECT_NE(err.find("COM20"), std::string::npos) << err;
    EXPECT_NE(err.find("--port"), std::string::npos) << err;

    err.clear();
    const std::optional<VexPort> second =
        pickVexPort(ports, VexRole::kBrainUser, std::string("vex:0501@Port_#0004.Hub_#0002"), err);
    ASSERT_TRUE(second) << err;
    EXPECT_EQ(second->port.path, "COM20");
}

TEST(VexPorts, NoMatchNamesWhatIsPresent) {
    std::string err;
    EXPECT_FALSE(pickVexPort({}, VexRole::kBrainUser, std::nullopt, err));
    EXPECT_EQ(err, "no VEX brain_user port found");

    const VexPorts unknown = classifyVexPorts({usbPort("COM5", kV5BrainProductId, std::nullopt)});
    err.clear();
    EXPECT_FALSE(pickVexPort(unknown, VexRole::kBrainUser, std::nullopt, err));
    EXPECT_NE(err.find("COM5 (unknown, vex:0501@Port_#0002.Hub_#0001)"), std::string::npos) << err;

    err.clear();
    EXPECT_FALSE(pickVexPort(classifyVexPorts(windowsPorts()), VexRole::kBrainUser,
                             std::string("vex:0501:OTHER"), err));
    EXPECT_NE(err.find("no VEX brain_user (vex:0501:OTHER) port found"), std::string::npos) << err;
}

TEST(VexPorts, UnknownRoleIsNeverPicked) {
    const VexPorts unknown = classifyVexPorts({usbPort("COM5", kV5BrainProductId, std::nullopt)});
    std::string    err;
    EXPECT_FALSE(pickVexPort(unknown, VexRole::kUnknown, std::nullopt, err));
    EXPECT_EQ(err, "a VEX port with an unknown role is never picked");
}

TEST(VexPorts, RoleNames) {
    EXPECT_STREQ(vexRoleName(VexRole::kBrainSystem), "brain_system");
    EXPECT_STREQ(vexRoleName(VexRole::kBrainUser), "brain_user");
    EXPECT_STREQ(vexRoleName(VexRole::kController), "controller");
    EXPECT_STREQ(vexRoleName(VexRole::kUnknown), "unknown");
}

} // namespace
} // namespace spectagatr::receiver::platform
