// resources_gtest.cpp
// ResourceMap construction, typed retrieval, sharing, dependency
// resolution, lifetimes, and atomic SPI transactions through a fake bus.
// Fake implementations register through the same explicit startup calls as
// real ones, which is itself the extension path under test.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "resources/resource_map.h"
#include "resources/spi_bus.h"
#include "runtime/register_all.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

// Fake SPI bus: counts initializations and asserts that every transfer is
// one atomic transaction with the owning device's settings applied.
struct FakeSpiState {
    int  init_count     = 0;
    bool in_transaction = false;

    struct AppliedSettings {
        int     chip_select = -1;
        SpiMode mode        = SpiMode::kMode0;
    };
    std::vector<AppliedSettings> transfers;
};

class FakeSpiDevice : public SpiDevice
{
public:
    FakeSpiDevice(std::shared_ptr<FakeSpiState> state, SpiDeviceConfig config)
        : state_(std::move(state)), config_(config) {}

    bool transfer(ByteSpan transmit, MutableByteSpan receive) override {
        EXPECT_EQ(transmit.size, receive.size);
        EXPECT_FALSE(state_->in_transaction);   // no interleaving
        state_->in_transaction = true;
        state_->transfers.push_back({config_.chip_select, config_.mode});
        for (std::size_t i = 0; i < transmit.size; ++i) {
            receive.data[i] = transmit.data[i];
        }
        state_->in_transaction = false;
        return true;
    }

private:
    std::shared_ptr<FakeSpiState> state_;
    SpiDeviceConfig               config_;
};

class FakeSpiBus : public SpiBus
{
public:
    explicit FakeSpiBus(std::shared_ptr<FakeSpiState> state) : state_(std::move(state)) {}

    std::shared_ptr<SpiDevice> createDevice(const SpiDeviceConfig& config) override {
        return std::make_shared<FakeSpiDevice>(state_, config);
    }

private:
    std::shared_ptr<FakeSpiState> state_;
};

// Destruction-order probe for lifetime tests.
struct LifetimeLog {
    std::vector<std::string> events;
};

class ProbeResource
{
public:
    ProbeResource(std::shared_ptr<LifetimeLog> log) : log_(std::move(log)) {}
    ~ProbeResource() { log_->events.push_back("resource_destroyed"); }

private:
    std::shared_ptr<LifetimeLog> log_;
};

struct Fixture {
    tinyxml2::XMLDocument    doc;
    FunctionRegistry         functions;
    std::vector<std::string> warnings;

    std::shared_ptr<FakeSpiState> spi_state = std::make_shared<FakeSpiState>();
    std::shared_ptr<LifetimeLog>  log       = std::make_shared<LifetimeLog>();

    Fixture() {
        register_resources(functions);

        functions.add<ResourceMakeFunction>(
            FunctionKey{"fake_spi_bus"},
            [state = spi_state](const ConfigNode&, ResourceInitializationContext&,
                                std::string&) -> ResourceInstance {
                ++state->init_count;
                return ResourceInstance::asContract<SpiBus>(
                    std::make_shared<FakeSpiBus>(state));
            });

        functions.add<ResourceMakeFunction>(
            FunctionKey{"probe"},
            [log = log](const ConfigNode&, ResourceInitializationContext&,
                        std::string&) -> ResourceInstance {
                return ResourceInstance::asContract<ProbeResource>(
                    std::make_shared<ProbeResource>(log));
            });

        // a resource that depends on another declared resource
        functions.add<ResourceMakeFunction>(
            FunctionKey{"needs_bus"},
            [](const ConfigNode& node, ResourceInitializationContext& context,
               std::string& err) -> ResourceInstance {
                const ResourceId bus_id{node.child("Bus").attr("resource_id")};
                auto             bus = context.require<SpiBus>(bus_id, err);
                if (bus == nullptr) {
                    return ResourceInstance{};
                }
                return ResourceInstance::asContract<SpiBus>(
                    std::static_pointer_cast<FakeSpiBus>(bus));
            });

        // mutual dependency pair for cycle detection
        const auto depend_on = [](const char* attr) {
            return [attr](const ConfigNode& node, ResourceInitializationContext& context,
                          std::string& err) -> ResourceInstance {
                const ResourceId other{node.child("Dep").attr(attr)};
                auto             dep = context.require<SpiBus>(other, err);
                if (dep == nullptr) {
                    return ResourceInstance{};
                }
                return ResourceInstance{};
            };
        };
        functions.add<ResourceMakeFunction>(FunctionKey{"cyclic"},
                                            depend_on("resource_id"));
    }

    bool build(const char* xml, ResourceMap& out, std::string& err) {
        doc.Clear();
        if (doc.Parse(xml) != tinyxml2::XML_SUCCESS) {
            err = "parse";
            return false;
        }
        ResourceMapBuilder builder(functions, &warnings);
        bool                 ok = true;
        ConfigNode{doc.RootElement()}.forEach("Resource", [&](const ConfigNode& r) {
            if (ok) {
                ok = builder.index(r, err);
            }
        });
        if (!ok || !builder.buildAll(err)) {
            return false;
        }
        out = builder.take();
        return true;
    }
};

} // namespace

TEST(Resources, SharedBusInitializedOnceAndShared) {
    Fixture       f;
    ResourceMap store;
    std::string   err;
    ASSERT_TRUE(f.build(R"(
<Resources>
    <Resource id="bus" type="fake_spi_bus"/>
</Resources>)",
                        store, err))
        << err;

    auto a = store.require<SpiBus>(ResourceId{"bus"}, err);
    auto b = store.require<SpiBus>(ResourceId{"bus"}, err);
    auto c = store.require<SpiBus>(ResourceId{"bus"}, err);
    ASSERT_NE(a, nullptr);
    EXPECT_EQ(a.get(), b.get());
    EXPECT_EQ(b.get(), c.get());
    EXPECT_EQ(f.spi_state->init_count, 1);
}

TEST(Resources, UnknownIdAndWrongContractFail) {
    Fixture       f;
    ResourceMap store;
    std::string   err;
    ASSERT_TRUE(f.build(R"(
<Resources>
    <Resource id="bus" type="fake_spi_bus"/>
</Resources>)",
                        store, err))
        << err;

    EXPECT_EQ(store.require<SpiBus>(ResourceId{"does_not_exist"}, err), nullptr);
    EXPECT_NE(err.find("does_not_exist"), std::string::npos);

    err.clear();
    EXPECT_EQ(store.require<SerialLink>(ResourceId{"bus"}, err), nullptr);
    EXPECT_NE(err.find("contract"), std::string::npos);
}

TEST(Resources, DuplicateIdAndUnknownTypeFail) {
    Fixture       f;
    ResourceMap store;
    std::string   err;

    EXPECT_FALSE(f.build(R"(
<Resources>
    <Resource id="bus" type="fake_spi_bus"/>
    <Resource id="bus" type="fake_spi_bus"/>
</Resources>)",
                         store, err));
    EXPECT_NE(err.find("duplicate Resource id"), std::string::npos);

    EXPECT_FALSE(f.build(R"(
<Resources>
    <Resource id="x" type="carrier_pigeon"/>
</Resources>)",
                         store, err));
    EXPECT_NE(err.find("carrier_pigeon"), std::string::npos);
}

TEST(Resources, DependenciesResolveRegardlessOfDeclarationOrder) {
    Fixture       f;
    ResourceMap store;
    std::string   err;
    // the dependent is declared before the bus it needs
    ASSERT_TRUE(f.build(R"(
<Resources>
    <Resource id="wrapper" type="needs_bus">
        <Bus resource_id="bus"/>
    </Resource>
    <Resource id="bus" type="fake_spi_bus"/>
</Resources>)",
                        store, err))
        << err;
    EXPECT_EQ(f.spi_state->init_count, 1);
}

TEST(Resources, DependencyCycleReportsTheCycle) {
    Fixture       f;
    ResourceMap store;
    std::string   err;
    EXPECT_FALSE(f.build(R"(
<Resources>
    <Resource id="a" type="cyclic"><Dep resource_id="b"/></Resource>
    <Resource id="b" type="cyclic"><Dep resource_id="a"/></Resource>
</Resources>)",
                         store, err));
    EXPECT_NE(err.find("cycle"), std::string::npos);
    EXPECT_NE(err.find("a -> b -> a"), std::string::npos);
}

TEST(Resources, ResourcesOutliveCapturersAndCleanUpLast) {
    Fixture     f;
    std::string err;
    {
        ResourceMap store;
        ASSERT_TRUE(f.build(R"(
<Resources>
    <Resource id="probe" type="probe"/>
</Resources>)",
                            store, err))
            << err;

        {
            // a consumer capturing the shared handle, destroyed first
            auto handle = store.require<ProbeResource>(ResourceId{"probe"}, err);
            ASSERT_NE(handle, nullptr);
            struct Consumer {
                std::shared_ptr<ProbeResource> resource;
                std::shared_ptr<LifetimeLog>   log;
                ~Consumer() { log->events.push_back("consumer_destroyed"); }
            } consumer{handle, f.log};
            EXPECT_TRUE(f.log->events.empty());
        }
        EXPECT_EQ(f.log->events,
                  (std::vector<std::string>{"consumer_destroyed"}));   // resource alive
    }
    EXPECT_EQ(f.log->events, (std::vector<std::string>{"consumer_destroyed",
                                                       "resource_destroyed"}));
}

TEST(Resources, SpiTransactionsApplyDeviceSettingsAtomically) {
    Fixture       f;
    ResourceMap store;
    std::string   err;
    ASSERT_TRUE(f.build(R"(
<Resources>
    <Resource id="bus" type="fake_spi_bus"/>
</Resources>)",
                        store, err))
        << err;

    auto bus = store.require<SpiBus>(ResourceId{"bus"}, err);
    ASSERT_NE(bus, nullptr);

    SpiDeviceConfig config_a;
    config_a.chip_select = 8;
    config_a.mode        = SpiMode::kMode1;
    SpiDeviceConfig config_b;
    config_b.chip_select = 7;
    config_b.mode        = SpiMode::kMode3;

    auto device_a = bus->createDevice(config_a);
    auto device_b = bus->createDevice(config_b);

    uint8_t         tx[2] = {0xAB, 0xCD};
    uint8_t         rx[2] = {0, 0};
    MutableByteSpan receive{rx, 2};

    device_a->transfer(ByteSpan{tx, 2}, receive);
    device_b->transfer(ByteSpan{tx, 2}, receive);
    device_a->transfer(ByteSpan{tx, 2}, receive);

    ASSERT_EQ(f.spi_state->transfers.size(), 3u);
    EXPECT_EQ(f.spi_state->transfers[0].chip_select, 8);
    EXPECT_EQ(f.spi_state->transfers[0].mode, SpiMode::kMode1);
    EXPECT_EQ(f.spi_state->transfers[1].chip_select, 7);
    EXPECT_EQ(f.spi_state->transfers[1].mode, SpiMode::kMode3);
    EXPECT_EQ(f.spi_state->transfers[2].chip_select, 8);
    EXPECT_EQ(rx[0], 0xAB);
}
