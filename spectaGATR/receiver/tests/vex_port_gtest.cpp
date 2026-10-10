// vex_port_gtest.cpp
// Which port each --source opens: auto prefers a controller, the Brain system port is never
// opened, a forced port takes its kind from the listing.

#include "receiver/sources/vex_port.h"

#include <gtest/gtest.h>

namespace spectagatr::receiver::sources
{
namespace
{

platform::UsbSerialPort usbPort(const std::string& path, platform::UsbId pid,
                                std::optional<platform::UsbInterfaceNumber> interface_number,
                                const std::string& location = "Port_#0002.Hub_#0001") {
    platform::UsbSerialPort port;
    port.path             = path;
    port.vendor_id        = platform::kVexVendorId;
    port.product_id       = pid;
    port.interface_number = interface_number;
    port.location         = location;
    return port;
}

platform::VexPorts brainOnly() {
    return platform::classifyVexPorts({usbPort("COM9", platform::kV5BrainProductId, 0),
                                       usbPort("COM4", platform::kV5BrainProductId, 2)});
}

platform::VexPorts brainAndController() {
    platform::VexPorts       ports      = brainOnly();
    const platform::VexPorts controller = platform::classifyVexPorts(
        {usbPort("COM12", platform::kV5ControllerProductId, 1, "Port_#0003.Hub_#0001")});
    ports.insert(ports.end(), controller.begin(), controller.end());
    return ports;
}

TEST(PickPort, AutoPrefersTheController) {
    std::string                   err;
    const std::optional<PortPick> pick =
        pickPort(brainAndController(), PortChoice::kAuto, std::nullopt, err);
    ASSERT_TRUE(pick) << err;
    EXPECT_EQ(pick->path, "COM12");
    EXPECT_EQ(pick->kind, SourceKind::kController);
}

TEST(PickPort, AutoFallsBackToTheBrainUserPort) {
    std::string                   err;
    const std::optional<PortPick> pick =
        pickPort(brainOnly(), PortChoice::kAuto, std::nullopt, err);
    ASSERT_TRUE(pick) << err;
    EXPECT_EQ(pick->path, "COM4"); // interface 2, never the system port COM9
    EXPECT_EQ(pick->kind, SourceKind::kBrainUsb);
}

TEST(PickPort, NothingFoundSaysSo) {
    std::string err;
    EXPECT_FALSE(pickPort({}, PortChoice::kAuto, std::nullopt, err));
    EXPECT_EQ(err, "no VEX controller or Brain user port found");
    EXPECT_FALSE(pickPort(brainOnly(), PortChoice::kController, std::nullopt, err));
    EXPECT_NE(err.find("no VEX controller port found"), std::string::npos) << err;
}

TEST(PickPort, TwoControllersAreRefused) {
    platform::VexPorts       ports  = brainAndController();
    const platform::VexPorts second = platform::classifyVexPorts(
        {usbPort("COM13", platform::kV5ControllerProductId, 1, "Port_#0004.Hub_#0001")});
    ports.insert(ports.end(), second.begin(), second.end());
    std::string err;
    EXPECT_FALSE(pickPort(ports, PortChoice::kAuto, std::nullopt, err));
    EXPECT_NE(err.find("--port"), std::string::npos) << err;
}

TEST(PickPort, ForcedPortTakesItsKindFromTheListing) {
    std::string                   err;
    const std::optional<PortPick> controller =
        pickPort(brainAndController(), PortChoice::kAuto, std::string("COM12"), err);
    ASSERT_TRUE(controller) << err;
    EXPECT_EQ(controller->kind, SourceKind::kController);
#if defined(_WIN32)
    const std::optional<PortPick> spelled =
        pickPort(brainAndController(), PortChoice::kAuto, std::string("\\\\.\\com12"), err);
    ASSERT_TRUE(spelled) << err;
    EXPECT_EQ(spelled->kind, SourceKind::kController); // COM names ignore case and \\.\ prefix
#endif
    const std::optional<PortPick> unknown =
        pickPort(brainAndController(), PortChoice::kAuto, std::string("COM30"), err);
    ASSERT_TRUE(unknown) << err;
    EXPECT_EQ(unknown->kind, SourceKind::kBrainUsb); // read raw: never writes
    const std::optional<PortPick> told =
        pickPort(brainAndController(), PortChoice::kController, std::string("COM30"), err);
    ASSERT_TRUE(told) << err;
    EXPECT_EQ(told->kind, SourceKind::kController);
}

// The controller source writes CDC2 queries: never to a port listed as a Brain.
TEST(PickPort, ForcedControllerRefusesABrainUserPort) {
    std::string err;
    EXPECT_FALSE(pickPort(brainOnly(), PortChoice::kController, std::string("COM4"), err));
    EXPECT_NE(err.find("--source brain"), std::string::npos) << err;
    const std::optional<PortPick> brain =
        pickPort(brainOnly(), PortChoice::kBrain, std::string("COM4"), err);
    ASSERT_TRUE(brain) << err;
    EXPECT_EQ(brain->kind, SourceKind::kBrainUsb);
}

TEST(PickPort, TheBrainSystemPortIsNeverOpened) {
    std::string err;
    EXPECT_FALSE(pickPort(brainOnly(), PortChoice::kBrain, std::string("COM9"), err));
    EXPECT_NE(err.find("system port"), std::string::npos) << err;
}

} // namespace
} // namespace spectagatr::receiver::sources
