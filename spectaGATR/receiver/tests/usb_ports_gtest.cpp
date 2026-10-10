// usb_ports_gtest.cpp
// Windows device ids, sysfs trees under a temp dir, and the live listing on this OS.

#include "receiver/platform/usb_ports.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

#include "receiver/platform/vex_ports.h"

namespace spectagatr::receiver::platform
{
namespace
{

namespace fs = std::filesystem;

TEST(WindowsDeviceId, InterfaceNode) {
    const std::optional<WindowsDeviceId> id =
        parseWindowsDeviceId("USB\\VID_2888&PID_0501&MI_02\\6&A694CA9&0&0002");
    ASSERT_TRUE(id);
    EXPECT_EQ(id->vendor_id, 0x2888);
    EXPECT_EQ(id->product_id, 0x0501);
    ASSERT_TRUE(id->interface_number);
    EXPECT_EQ(*id->interface_number, 2);
    EXPECT_EQ(id->instance, "6&A694CA9&0&0002");
    EXPECT_EQ(serialFromWindowsInstance(id->instance), "");
}

TEST(WindowsDeviceId, CompositeParentWithSerial) {
    const std::optional<WindowsDeviceId> id =
        parseWindowsDeviceId("USB\\VID_2888&PID_0501\\A9EC2700");
    ASSERT_TRUE(id);
    EXPECT_FALSE(id->interface_number);
    EXPECT_EQ(serialFromWindowsInstance(id->instance), "A9EC2700");
}

TEST(WindowsDeviceId, LowerCaseAndHexInterface) {
    const std::optional<WindowsDeviceId> id =
        parseWindowsDeviceId("usb\\vid_2888&pid_0503&mi_01\\7&1b2c3d4e&0&0001");
    ASSERT_TRUE(id);
    EXPECT_EQ(id->product_id, 0x0503);
    ASSERT_TRUE(id->interface_number);
    EXPECT_EQ(*id->interface_number, 1);

    const std::optional<WindowsDeviceId> high =
        parseWindowsDeviceId("USB\\VID_1D50&PID_6018&MI_0A\\6&1&0&000A");
    ASSERT_TRUE(high);
    ASSERT_TRUE(high->interface_number);
    EXPECT_EQ(*high->interface_number, 10);
}

TEST(WindowsDeviceId, BadInterfaceIsUnknownNotFatal) {
    const std::optional<WindowsDeviceId> id =
        parseWindowsDeviceId("USB\\VID_2888&PID_0501&MI_ZZ\\6&A694CA9&0&0002");
    ASSERT_TRUE(id);
    EXPECT_FALSE(id->interface_number);
}

TEST(WindowsDeviceId, RejectsNonUsbAndMalformed) {
    EXPECT_FALSE(parseWindowsDeviceId(""));
    EXPECT_FALSE(parseWindowsDeviceId("ACPI\\PNP0501\\0"));
    // A Bluetooth serial port as listed on the build laptop.
    EXPECT_FALSE(
        parseWindowsDeviceId("BTHENUM\\{00001101-0000-1000-8000-00805F9B34FB}_VID&00010094_"
                             "PID&0004\\7&2B886E8&0&8C0DD9A837D1_C00000000"));
    EXPECT_FALSE(parseWindowsDeviceId("FTDIBUS\\VID_0403+PID_6001+A50285BIA\\0000"));
    EXPECT_FALSE(parseWindowsDeviceId("USB\\ROOT_HUB30\\4&1234&0&0"));
    EXPECT_FALSE(parseWindowsDeviceId("USB\\VID_28&PID_0501\\1"));
    EXPECT_FALSE(parseWindowsDeviceId("USB\\VID_2888\\1"));
    EXPECT_FALSE(parseWindowsDeviceId("USB\\VID_2888&PID_05G1\\1"));
    EXPECT_FALSE(parseWindowsDeviceId("USB\\VID_2888&PID_0501"));
    EXPECT_EQ(serialFromWindowsInstance(""), "");
}

class SysfsTree : public ::testing::Test {
  protected:
    fs::path root;

    void SetUp() override {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        root = fs::temp_directory_path() /
               ("spectagatr-sysfs-" +
                std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + "-" +
                std::to_string(stamp));
        fs::create_directories(root / "class");
    }

    void TearDown() override {
        std::error_code error;
        fs::remove_all(root, error);
    }

    static void write(const fs::path& file, const std::string& text) {
        std::ofstream(file) << text << '\n';
    }

    // Flat stand-in for the symlinked layout: device/ is the interface, the tty dir the device.
    void tty(const std::string& name, const std::string& interface_number, const std::string& vid,
             const std::string& pid, const std::string& serial = "") {
        const fs::path device = root / "class" / name / "device";
        fs::create_directories(device);
        if (!interface_number.empty()) {
            write(device / "bInterfaceNumber", interface_number);
        }
        if (!vid.empty()) {
            write(device.parent_path() / "idVendor", vid);
            write(device.parent_path() / "idProduct", pid);
        }
        if (!serial.empty()) {
            write(device.parent_path() / "serial", serial);
        }
    }

    UsbSerialPorts sorted() const {
        UsbSerialPorts ports = listSysfsSerialPorts(root / "class", "/dev");
        std::sort(ports.begin(), ports.end(),
                  [](const UsbSerialPort& a, const UsbSerialPort& b) { return a.path < b.path; });
        return ports;
    }
};

TEST_F(SysfsTree, ListsUsbTtysWithIds) {
    tty("ttyACM0", "00", "2888", "0501");
    tty("ttyACM1", "02", "2888", "0501");
    tty("ttyACM2", "01", "2888", "0503", "CTRL01");
    tty("ttyUSB0", "00", "0403", "6001");
    tty("ttyS0", "", "", "");
    tty("console", "00", "2888", "0501");

    const UsbSerialPorts ports = sorted();
    ASSERT_EQ(ports.size(), 4u);
    EXPECT_EQ(ports[0].path, "/dev/ttyACM0");
    EXPECT_EQ(ports[1].path, "/dev/ttyACM1");
    EXPECT_EQ(ports[1].vendor_id, 0x2888);
    EXPECT_EQ(ports[1].product_id, 0x0501);
    ASSERT_TRUE(ports[1].interface_number);
    EXPECT_EQ(*ports[1].interface_number, 2);
    EXPECT_EQ(ports[1].serial, "");
    EXPECT_EQ(ports[2].serial, "CTRL01");
    EXPECT_EQ(ports[3].path, "/dev/ttyUSB0");
    EXPECT_EQ(ports[3].vendor_id, 0x0403);
}

TEST_F(SysfsTree, WalksUpFromTheTtyToInterfaceAndDevice) {
    // Nested as under /sys/devices: the tty below its interface, the interface below the device.
    const fs::path usb_device = root / "devices" / "usb1" / "1-1" / "1-1.2";
    const fs::path interface  = usb_device / "1-1.2_1.2"; // 1-1.2:1.2 on Linux, no ':' on Windows
    const fs::path class_root = interface / "tty";
    fs::create_directories(class_root / "ttyACM1" / "device");
    write(interface / "bInterfaceNumber", "02");
    write(usb_device / "idVendor", "2888");
    write(usb_device / "idProduct", "0501");
    write(usb_device / "product", "V5 Brain - A9EC2700");

    const UsbSerialPorts ports = listSysfsSerialPorts(class_root, "/dev");
    ASSERT_EQ(ports.size(), 1u);
    ASSERT_TRUE(ports[0].interface_number);
    EXPECT_EQ(*ports[0].interface_number, 2);
    EXPECT_EQ(ports[0].location, "1-1.2");
    EXPECT_EQ(ports[0].product, "V5 Brain - A9EC2700");
    EXPECT_EQ(vexIdentityOf(ports[0]), "vex:0501:A9EC2700");
}

TEST_F(SysfsTree, BadIdsSkipAndMissingInterfaceIsUnknown) {
    tty("ttyACM0", "", "2888", "0501");
    tty("ttyACM1", "02", "28G8", "0501");
    const UsbSerialPorts ports = sorted();
    ASSERT_EQ(ports.size(), 1u);
    EXPECT_EQ(ports[0].path, "/dev/ttyACM0");
    EXPECT_FALSE(ports[0].interface_number);
}

TEST_F(SysfsTree, UnplugAndMissingRoot) {
    tty("ttyACM3", "02", "2888", "0501");
    tty("ttyACM5", "02", "2888", "0501");
    EXPECT_EQ(sorted().size(), 2u);
    fs::remove_all(root / "class" / "ttyACM3");
    ASSERT_EQ(sorted().size(), 1u);
    EXPECT_EQ(sorted()[0].path, "/dev/ttyACM5");
    EXPECT_TRUE(listSysfsSerialPorts(root / "absent", "/dev").empty());
}

TEST_F(SysfsTree, ClassifiesIntoOneBrainUserPortWithoutAssumingAcmNumbers) {
    tty("ttyACM0", "00", "2888", "0501");
    tty("ttyACM7", "02", "2888", "0501");
    tty("ttyACM1", "02", "1234", "0501");
    tty("ttyACM2", "01", "2888", "0503");
    std::string                  err;
    const std::optional<VexPort> user =
        pickVexPort(classifyVexPorts(sorted()), VexRole::kBrainUser, std::nullopt, err);
    ASSERT_TRUE(user) << err;
    EXPECT_EQ(user->port.path, "/dev/ttyACM7");
}

TEST(CdcFunctionInterface, MacDataInterfacesGiveTheirFunction) {
    // Brain system and user data interfaces 1 and 3; the controller's 2 (pros-cli name suffixes).
    EXPECT_EQ(cdcFunctionInterface(1, 0x0A), UsbInterfaceNumber{0});
    EXPECT_EQ(cdcFunctionInterface(3, 0x0A), UsbInterfaceNumber{2});
    EXPECT_EQ(cdcFunctionInterface(2, 0x0A), UsbInterfaceNumber{1});
    EXPECT_EQ(cdcFunctionInterface(0, 0x0A), UsbInterfaceNumber{0});
}

TEST(CdcFunctionInterface, OtherClassesKeepTheirNumber) {
    EXPECT_EQ(cdcFunctionInterface(2, 0x02), UsbInterfaceNumber{2});
    EXPECT_EQ(cdcFunctionInterface(2, std::nullopt), UsbInterfaceNumber{2});
    EXPECT_EQ(cdcFunctionInterface(255, 0xFF), UsbInterfaceNumber{255});
    EXPECT_FALSE(cdcFunctionInterface(-1, 0x0A));
    EXPECT_FALSE(cdcFunctionInterface(256, 0x02));
}

TEST(UsbPorts, LiveListingOnThisHost) {
    const UsbSerialPorts ports = listUsbSerialPorts();
    for (const UsbSerialPort& port : ports) {
        EXPECT_FALSE(port.path.empty());
    }
    RecordProperty("ports", static_cast<int>(ports.size()));
}

} // namespace
} // namespace spectagatr::receiver::platform
