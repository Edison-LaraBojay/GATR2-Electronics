// core_gtest.cpp
// FunctionRegistry, TypedPayload, strong ids, time domains.

#include <gtest/gtest.h>

#include "contracts/sensor.h"
#include "core/function_registry.h"
#include "core/records.h"
#include "core/time.h"
#include "resources/resource_store.h"
#include "runtime/sensor_stage.h"

using namespace navigatr;

namespace
{

struct AlphaPayload {
    int value = 0;
};

struct BetaPayload {
    double value = 0.0;
};

} // namespace

TEST(FunctionRegistry, RegisterAndRetrieveTypedSignatures) {
    FunctionRegistry functions;

    // one opaque name may exist once per signature; this is what lets every
    // category register its own noop
    ASSERT_TRUE(functions.add<SensorMakeFunction>(
        FunctionKey{"fake"},
        [](const ConfigNode&, SensorInitializationContext&,
           std::string&) -> std::optional<SensorExecutable> { return std::nullopt; }));
    ASSERT_TRUE(functions.add<ResourceMakeFunction>(
        FunctionKey{"fake"},
        [](const ConfigNode&, ResourceInitializationContext&,
           std::string&) -> ResourceInstance { return ResourceInstance{}; }));

    std::string err;
    EXPECT_NE(functions.find<SensorMakeFunction>(FunctionKey{"fake"}, err), nullptr);
    EXPECT_NE(functions.find<ResourceMakeFunction>(FunctionKey{"fake"}, err),
              nullptr);
}

TEST(FunctionRegistry, UnknownKeyFailsLoudly) {
    FunctionRegistry functions;
    std::string      err;
    EXPECT_EQ(functions.find<SensorMakeFunction>(FunctionKey{"missing"}, err),
              nullptr);
    EXPECT_NE(err.find("missing"), std::string::npos);
}

TEST(FunctionRegistry, WrongSignatureFailsLoudly) {
    FunctionRegistry functions;
    functions.add<ResourceMakeFunction>(
        FunctionKey{"fake"},
        [](const ConfigNode&, ResourceInitializationContext&,
           std::string&) -> ResourceInstance { return ResourceInstance{}; });

    std::string err;
    EXPECT_EQ(functions.find<SensorMakeFunction>(FunctionKey{"fake"}, err),
              nullptr);
    EXPECT_NE(err.find("different signature"), std::string::npos);
}

TEST(FunctionRegistry, DuplicateKeyWithinSignatureFails) {
    FunctionRegistry functions;
    const auto       factory =
        [](const ConfigNode&, SensorInitializationContext&,
           std::string&) -> std::optional<SensorExecutable> { return std::nullopt; };
    EXPECT_TRUE(functions.add<SensorMakeFunction>(FunctionKey{"fake"}, factory));
    EXPECT_FALSE(functions.add<SensorMakeFunction>(FunctionKey{"fake"}, factory));
    EXPECT_FALSE(functions.add<SensorMakeFunction>(FunctionKey{""}, factory));
}

TEST(FunctionRegistry, ExplicitRegistrationNoStatics) {
    // a fresh registry is empty until register_* calls run
    FunctionRegistry functions;
    EXPECT_TRUE(functions.keys().empty());
}

TEST(TypedPayload, StoreAndGetExactTypeWithStableName) {
    const TypedPayload p = TypedPayload::store(AlphaPayload{42}, "test.alpha");
    const AlphaPayload* a = p.get<AlphaPayload>();
    ASSERT_NE(a, nullptr);
    EXPECT_EQ(a->value, 42);
    EXPECT_EQ(p.stableTypeName(), "test.alpha");
    EXPECT_EQ(p.cppType(), std::type_index(typeid(AlphaPayload)));
}

TEST(TypedPayload, WrongTypeIsNull) {
    const TypedPayload p = TypedPayload::store(AlphaPayload{1}, "test.alpha");
    EXPECT_EQ(p.get<BetaPayload>(), nullptr);
    EXPECT_FALSE(p.empty());
}

TEST(TypedPayload, EmptyIsNull) {
    const TypedPayload p;
    EXPECT_TRUE(p.empty());
    EXPECT_EQ(p.get<AlphaPayload>(), nullptr);
}

TEST(TypedIds, DistinctTypesDoNotMix) {
    SensorMap results;
    results[SensorId{"tracking"}] = MeasurementRecord{};
    EXPECT_EQ(results.count(SensorId{"tracking"}), 1u);
    // ResourceId{"tracking"} would not compile as a key here.
    EXPECT_NE(SensorId{"a"}, SensorId{"b"});
}

TEST(SensorFunctionsClass, IdKeyedDeterministicOrder) {
    const auto executable = [] {
        SensorExecutable e;
        e.outputPayload = PayloadDescriptor::of<AlphaPayload>("test.alpha");
        e.execute = [](const ResourceMap&, const ExecutionContext&) { return PollResult{}; };
        return e;
    };

    SensorFunctions sensors;
    EXPECT_TRUE(sensors.add(SensorId{"b"}, executable(), "Sensor/b"));
    EXPECT_TRUE(sensors.add(SensorId{"a"}, executable(), "Sensor/a"));
    EXPECT_FALSE(sensors.add(SensorId{"a"}, executable(), "Sensor/a"));   // duplicate

    // declaration order, not hash or lexical order
    ASSERT_EQ(sensors.size(), 2u);
    EXPECT_EQ(sensors.executionOrder()[0].id, SensorId{"b"});
    EXPECT_EQ(sensors.executionOrder()[1].id, SensorId{"a"});

    const SensorFunctions::Entry* found = sensors.find(SensorId{"a"});
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->label, "Sensor/a");
    EXPECT_EQ(sensors.find(SensorId{"missing"}), nullptr);
}

TEST(Time, DomainsStaySeparate) {
    const MonotonicTime a = deviceTime(1000);
    const MonotonicTime b = deviceTime(1250);
    EXPECT_EQ(b - a, 250);
    EXPECT_NEAR(secondsBetween(b, a), 0.25, 1e-12);
    EXPECT_TRUE(sameDomain(a, MonotonicTime{}));   // unset tolerated
    EXPECT_FALSE(sameDomain(a, hostTime(1000)));
    EXPECT_FALSE(MonotonicTime{}.isSet());
}
