// core_gtest.cpp
// FunctionRegistry, TypedPayload, strong ids, time domains.

#include <gtest/gtest.h>

#include "contracts/sensor.h"
#include "core/function_registry.h"
#include "core/records.h"
#include "core/time.h"
#include "resources/resource_store.h"

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

    ASSERT_TRUE(functions.add<SensorMakeFunction>(
        FunctionKey{"sensor/fake"},
        [](const ConfigNode&, SensorInitializationContext&,
           std::string&) -> std::unique_ptr<Sensor> { return nullptr; }));
    ASSERT_TRUE(functions.add<ResourceMakeFunction>(
        FunctionKey{"resource/fake"},
        [](const ConfigNode&, ResourceInitializationContext&,
           std::string&) -> ResourceValue { return ResourceValue{}; }));

    std::string err;
    EXPECT_NE(functions.find<SensorMakeFunction>(FunctionKey{"sensor/fake"}, err), nullptr);
    EXPECT_NE(functions.find<ResourceMakeFunction>(FunctionKey{"resource/fake"}, err),
              nullptr);
}

TEST(FunctionRegistry, UnknownKeyFailsLoudly) {
    FunctionRegistry functions;
    std::string      err;
    EXPECT_EQ(functions.find<SensorMakeFunction>(FunctionKey{"sensor/missing"}, err),
              nullptr);
    EXPECT_NE(err.find("sensor/missing"), std::string::npos);
}

TEST(FunctionRegistry, WrongSignatureFailsLoudly) {
    FunctionRegistry functions;
    functions.add<ResourceMakeFunction>(
        FunctionKey{"resource/fake"},
        [](const ConfigNode&, ResourceInitializationContext&,
           std::string&) -> ResourceValue { return ResourceValue{}; });

    std::string err;
    EXPECT_EQ(functions.find<SensorMakeFunction>(FunctionKey{"resource/fake"}, err),
              nullptr);
    EXPECT_NE(err.find("different signature"), std::string::npos);
}

TEST(FunctionRegistry, DuplicateKeyFails) {
    FunctionRegistry functions;
    const auto       factory = [](const ConfigNode&, SensorInitializationContext&,
                            std::string&) -> std::unique_ptr<Sensor> { return nullptr; };
    EXPECT_TRUE(functions.add<SensorMakeFunction>(FunctionKey{"sensor/fake"}, factory));
    EXPECT_FALSE(functions.add<SensorMakeFunction>(FunctionKey{"sensor/fake"}, factory));
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
    SensorResultsMap results;
    results[SensorId{"tracking"}] = SensorRecord{};
    EXPECT_EQ(results.count(SensorId{"tracking"}), 1u);
    // ResourceId{"tracking"} would not compile as a key here.
    EXPECT_NE(SensorId{"a"}, SensorId{"b"});
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
