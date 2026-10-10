// brain_port_gtest.cpp
// Fake sysfs trees: tty/<name>/device is the USB interface directory, its
// parent holds the device's idVendor and idProduct.

#include "brain_port.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace relay
{
namespace
{

namespace fs = std::filesystem;

class BrainPort : public ::testing::Test {
  protected:
    void SetUp() override {
        root_ = fs::temp_directory_path() /
                ("brain_port_" + std::string(::testing::UnitTest::GetInstance()
                                                 ->current_test_info()
                                                 ->name()));
        fs::remove_all(root_);
        fs::create_directories(root_ / "tty");
    }
    void TearDown() override { fs::remove_all(root_); }

    void write(const fs::path& file, const std::string& value) {
        std::ofstream(file) << value << "\n";
    }

    void addAcm(const std::string& name, const std::string& vendor, const std::string& product,
                const std::string& interface_number) {
        const fs::path usb_device = root_ / "tty" / name;
        fs::create_directories(usb_device / "device");
        write(usb_device / "idVendor", vendor);
        write(usb_device / "idProduct", product);
        write(usb_device / "device" / "bInterfaceNumber", interface_number);
    }

    BrainPortSearch search() {
        return findBrainUserPort((root_ / "tty").string(), "/dev");
    }

    fs::path root_;
};

TEST_F(BrainPort, PicksInterfaceTwoNotTheSystemPort) {
    addAcm("ttyACM0", "2888", "0501", "00"); // system/upload
    addAcm("ttyACM1", "2888", "0501", "02"); // user
    const BrainPortSearch s = search();
    EXPECT_EQ(s.brains, 1);
    EXPECT_EQ(fs::path(s.path).filename(), "ttyACM1");
}

TEST_F(BrainPort, IgnoresOtherDevices) {
    addAcm("ttyACM0", "2e8a", "000a", "00"); // a Pico on USB
    addAcm("ttyACM1", "2888", "0503", "02"); // a V5 controller
    addAcm("ttyUSB0", "2888", "0501", "02"); // not ttyACM
    const BrainPortSearch s = search();
    EXPECT_EQ(s.brains, 0);
    EXPECT_TRUE(s.path.empty());
}

TEST_F(BrainPort, TwoBrainsNeedAnExplicitPort) {
    addAcm("ttyACM1", "2888", "0501", "02");
    addAcm("ttyACM3", "2888", "0501", "02");
    const BrainPortSearch s = search();
    EXPECT_EQ(s.brains, 2);
    EXPECT_TRUE(s.path.empty());
}

TEST_F(BrainPort, MissingTtyRootIsNotFound) {
    const BrainPortSearch s = findBrainUserPort((root_ / "nope").string(), "/dev");
    EXPECT_EQ(s.brains, 0);
    EXPECT_TRUE(s.path.empty());
}

} // namespace
} // namespace relay
