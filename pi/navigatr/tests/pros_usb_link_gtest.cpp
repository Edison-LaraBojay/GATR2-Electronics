#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "common/frame_codec.h"
#include "common/frames.h"
#include "config/composition.h"
#include "impl/resources/pros_usb_link.h"
#include "impl/resources/serial_links.h"
#include "inspection/inspection_document.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{
void feed(MemoryLink& raw, const std::string& text) {
    raw.input().feed(std::vector<uint8_t>(text.begin(), text.end()));
}

std::vector<uint8_t> read(ProsUsbLink& link, std::size_t capacity = 128) {
    std::vector<uint8_t> bytes(capacity);
    const auto result = link.readAvailable({bytes.data(), bytes.size()});
    EXPECT_FALSE(result.closed);
    bytes.resize(result.bytes);
    return bytes;
}

class ClosableMemoryLink : public MemoryLink
{
public:
    bool closed = false;
    SerialReadResult readAvailable(MutableByteSpan destination) override {
        return closed ? SerialReadResult{0, true} : MemoryLink::readAvailable(destination);
    }
};

class UsbDiscovery : public ::testing::Test
{
protected:
    std::filesystem::path root;

    void SetUp() override {
        root = std::filesystem::temp_directory_path() /
               ("navigatr-usb-test-" + std::to_string(steadyNowUs()));
        std::filesystem::create_directories(root);
    }
    void TearDown() override {
        std::error_code error;
        std::filesystem::remove_all(root, error);
    }
    void device(const char* tty, const char* interface_number, const char* vendor = "2888",
                const char* product = "0501") {
        const auto path = root / tty / "device";
        std::filesystem::create_directories(path);
        std::ofstream(path / "bInterfaceNumber") << interface_number << '\n';
        std::ofstream(path / "idVendor") << vendor << '\n';
        std::ofstream(path / "idProduct") << product << '\n';
    }
};
} // namespace

TEST(ProsUsbLink, EncodesBinaryIncludingProsControlSequenceWithoutTerminalEscapes) {
    auto raw = std::make_shared<MemoryLink>();
    ProsUsbLink link(raw);
    const std::vector<uint8_t> frame{0, 'p', 'R', '\n', '\r', 0xFF, 0x80};
    ASSERT_TRUE(link.write({frame.data(), frame.size()}).ok);
    const auto output = raw->output().takeAll();
    EXPECT_EQ(std::string(output.begin(), output.end()), "NG1:0070520A0DFF80\n");
    raw->input().feed(output);
    EXPECT_EQ(read(link), frame);
}

TEST(ProsUsbLink, RetainsFragmentedLinesAndPartialCallerReads) {
    auto raw = std::make_shared<MemoryLink>();
    ProsUsbLink link(raw);
    feed(*raw, "N");
    EXPECT_TRUE(read(link).empty());
    feed(*raw, "G1:0012");
    EXPECT_TRUE(read(link).empty());
    feed(*raw, "AB\r\nNG1:FF\n");
    EXPECT_EQ(read(link, 2), (std::vector<uint8_t>{0, 0x12}));
    EXPECT_TRUE(link.inputPending());
    EXPECT_EQ(read(link), (std::vector<uint8_t>{0xAB, 0xFF}));
    EXPECT_FALSE(link.inputPending());
}

TEST(ProsUsbLink, IgnoresLogNoiseMalformedHexAndOversizedLinesThenRecovers) {
    auto raw = std::make_shared<MemoryLink>();
    ProsUsbLink link(raw);
    feed(*raw, "boot complete\nNG1:\nNG1:012\nNG1:abcd\nNG1:00ZZ\nNG1:00 garbage\n");
    feed(*raw, "NG1:" + std::string(2 * gatr2::kMaxFrameLen + 2, '0') + "\n");
    feed(*raw, std::string(2000, 'X') + "\n");
    feed(*raw, "kernel prefix: NG1:1200FF\n");
    std::vector<uint8_t> received;
    for (int attempt = 0; attempt < 5; ++attempt) {
        const auto part = read(link);
        received.insert(received.end(), part.begin(), part.end());
    }
    EXPECT_EQ(received, (std::vector<uint8_t>{0x12, 0, 0xFF}));
}

TEST(ProsUsbLink, TransmitWindowAndInputPendingPropagateThroughEnvelope) {
    auto raw = std::make_shared<MemoryLink>();
    raw->setClock([] { return 100; });
    ProsUsbLink link(raw);
    const uint8_t byte = 1;
    const auto expired = link.write({&byte, 1}, {0, 99});
    EXPECT_TRUE(expired.expired);
    EXPECT_FALSE(expired.ok);
    EXPECT_TRUE(raw->output().takeAll().empty());
    feed(*raw, "NG1:12\n");
    EXPECT_TRUE(link.write({&byte, 1}, {0, 200}).input_pending);
    EXPECT_TRUE(raw->output().takeAll().empty());
    EXPECT_EQ(read(link), (std::vector<uint8_t>{0x12}));
    EXPECT_TRUE(link.write({&byte, 1}, {0, 200}).ok);
}

TEST(ProsUsbLink, DisconnectDropsPartialEnvelopeBeforeReconnection) {
    auto raw = std::make_shared<ClosableMemoryLink>();
    ProsUsbLink link(raw);
    feed(*raw, "NG1:12");
    EXPECT_TRUE(read(link).empty());
    raw->closed = true;
    std::array<uint8_t, 128> bytes{};
    EXPECT_TRUE(link.readAvailable({bytes.data(), bytes.size()}).closed);
    raw->closed = false;
    feed(*raw, "34\nNG1:AB\n");
    EXPECT_EQ(read(link), (std::vector<uint8_t>{0xAB}));
}

TEST(ProsUsbLink, AcceptsMaximumFrameAndRejectsOversizedWrite) {
    auto raw = std::make_shared<MemoryLink>();
    ProsUsbLink link(raw);
    std::vector<uint8_t> frame(gatr2::kMaxFrameLen, 0xAC);
    ASSERT_TRUE(link.write({frame.data(), frame.size()}).ok);
    raw->input().feed(raw->output().takeAll());
    EXPECT_EQ(read(link), frame);
    frame.push_back(0);
    EXPECT_FALSE(link.write({frame.data(), frame.size()}).ok);
    EXPECT_TRUE(raw->output().takeAll().empty());
}

TEST_F(UsbDiscovery, SelectsOnlyBrainUserInterfaceWithoutAssumingAcmNumber) {
    device("ttyACM0", "00");
    device("ttyACM1", "02", "1234");
    device("ttyACM2", "02", "2888", "0503");
    device("ttyACM7", "02");
    EXPECT_EQ(findProsUsbUserPort(root.string(), "/dev"),
              (std::filesystem::path("/dev") / "ttyACM7").string());
}

TEST_F(UsbDiscovery, RefusesAmbiguousBrainsAndHandlesUnplug) {
    EXPECT_TRUE(findProsUsbUserPort(root.string()).empty());
    device("ttyACM3", "02");
    device("ttyACM5", "02");
    EXPECT_TRUE(findProsUsbUserPort(root.string()).empty());
    std::filesystem::remove_all(root / "ttyACM3");
    EXPECT_EQ(findProsUsbUserPort(root.string(), "/dev"),
              (std::filesystem::path("/dev") / "ttyACM5").string());
    std::filesystem::remove_all(root / "ttyACM5");
    EXPECT_TRUE(findProsUsbUserPort(root.string()).empty());
}

TEST(ProsUsbLink, FactoryBuildsWithoutConnectedUsbAndRejectsDriverEnable) {
    FunctionRegistry functions;
    register_resources(functions);
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(R"(<Resource id="brain_uart" type="pros_usb_link"><Device path="auto"/></Resource>)"),
              tinyxml2::XML_SUCCESS);
    std::vector<std::string> warnings;
    ResourceStoreBuilder builder(functions, &warnings);
    std::string error;
    ASSERT_TRUE(builder.index(ConfigNode{doc.RootElement()}, error)) << error;
    ASSERT_TRUE(builder.buildAll(error)) << error;
    auto store = builder.take();
    EXPECT_NE(store.require<SerialLink>(ResourceId{"brain_uart"}, error), nullptr);
    ASSERT_EQ(doc.Parse(R"(<Resource><Device path="auto"/><DriverEnable gpio="6"/></Resource>)"),
              tinyxml2::XML_SUCCESS);
    ResourceInitializationContext context;
    make_pros_usb_link(ConfigNode{doc.RootElement()}, context, error);
    EXPECT_NE(error.find("cannot use DriverEnable"), std::string::npos);
}

TEST(ProsUsbLink, UsbProfileHandshakeImuPlacementMotionAndInspectionUseExistingPipeline) {
    FunctionRegistry functions;
    registerAll(functions);
    int64_t now = 1000;
    auto pi_raw = std::make_shared<MemoryLink>();
    auto brain_raw = std::make_shared<MemoryLink>();
    pi_raw->setClock([&] { return now * 1000; });
    ProsUsbLink brain(brain_raw);
    ASSERT_TRUE(functions.add<ResourceMakeFunction>(FunctionKey{"test_usb"},
        [pi_raw](const ConfigNode&, ResourceInitializationContext&, std::string&) {
            return ResourceInstance::asContract<SerialLink>(std::make_shared<ProsUsbLink>(pi_raw));
        }));
    ResolvedConfiguration config;
    std::string error;
    ASSERT_TRUE(resolveConfiguration(std::string(NAVIGATR_CONFIG_DIR) +
        "/override/diagnostics/bench_vex_imu_usb.xml", config, error)) << error;
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(config.xml.c_str()), tinyxml2::XML_SUCCESS);
    auto* resources = doc.RootElement()->FirstChildElement("Resources");
    ASSERT_NE(resources, nullptr);
    for (auto* resource = resources->FirstChildElement("Resource"); resource;
         resource = resource->NextSiblingElement("Resource")) {
        const std::string id = ConfigNode{resource}.attr("id");
        if (id == "brain_usb") resource->SetAttribute("type", "test_usb");
        if (id == "pico_uart") resource->SetAttribute("type", "memory_link");
    }
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    auto system = System::buildFromString(printer.CStr(), functions, error);
    ASSERT_NE(system, nullptr) << error;
    auto pico = std::dynamic_pointer_cast<MemoryLink>(
        system->resources().require<SerialLink>(ResourceId{"pico_uart"}, error));
    ASSERT_NE(pico, nullptr) << error;
    system->step(hostTime(now));

    const auto request = [&](const gatr2::BrainRequest& request) {
        std::array<uint8_t, gatr2::kMaxFrameLen> bytes{};
        const auto length = gatr2::encodeBrainRequest(request, bytes.data(), bytes.size());
        EXPECT_GT(length, 0);
        EXPECT_TRUE(brain.write({bytes.data(), length}).ok);
        pi_raw->input().feed(brain_raw->output().takeAll());
        now += 20;
        system->step(hostTime(now));
        brain_raw->input().feed(pi_raw->output().takeAll());
        const auto frame = read(brain);
        gatr2::BrainReply reply;
        EXPECT_TRUE(gatr2::decodeBrainReply(frame.data(), static_cast<uint16_t>(frame.size()), reply));
        return reply;
    };
    uint8_t sequence = 0;
    const auto wheels = [&](int count) {
        gatr2::SensorSample sample{};
        sample.seq = sequence++;
        sample.stamp_ms = static_cast<uint32_t>(now);
        sample.mask = gatr2::kSensorEnc0 | gatr2::kSensorEnc1;
        sample.enc[0] = sample.enc[1] = count;
        std::vector<uint8_t> frame(gatr2::kMaxFrameLen);
        frame.resize(gatr2::encodeSensorFrame(sample, frame.data(), gatr2::kMaxFrameLen));
        pico->input().feed(frame);
    };
    gatr2::BrainRequest hello;
    hello.op = gatr2::kOpHello;
    hello.request_id = 1;
    hello.nonce = 0x00005270; // little-endian payload includes the pR terminal escape
    const uint32_t session = request(hello).session;
    ASSERT_NE(session, 0u);
    gatr2::BrainRequest imu;
    imu.op = gatr2::kOpGetStateWithImu;
    imu.request_id = 2;
    imu.session = session;
    imu.imu_flags = gatr2::kBenchImuValid;
    imu.imu_stamp_ms = 100;
    wheels(0);
    EXPECT_EQ(request(imu).result, gatr2::kResultOk);
    gatr2::BrainRequest place;
    place.op = gatr2::kOpSetPose;
    place.request_id = 3;
    place.session = session;
    place.x_mm = 1000;
    request(place);
    wheels(0);
    imu.request_id = 4;
    imu.imu_stamp_ms = 120;
    request(imu);
    wheels(4000);
    imu.request_id = 5;
    imu.imu_stamp_ms = 140;
    const auto state = request(imu);
    EXPECT_EQ(state.result, gatr2::kResultOk);
    EXPECT_NE(state.state.robot_flags & gatr2::kRobotPoseValid, 0);
    EXPECT_GT(state.state.x_mm, 1100);
    const auto snapshot = snapshotDocument(*system, InspectionServiceStats{}, hostTime(now));
    EXPECT_NE(snapshot.find("arrival-time"), std::string::npos);
    EXPECT_NE(snapshot.find("brain_usb"), std::string::npos);
}
