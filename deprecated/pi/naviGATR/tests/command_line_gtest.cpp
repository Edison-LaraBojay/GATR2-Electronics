#include <gtest/gtest.h>

#include <initializer_list>
#include <string>
#include <vector>

#include "app/command_line.h"

namespace
{

bool parse(std::initializer_list<const char*> args, navigatr::CommandLine& result,
           std::string& err) {
    const std::vector<const char*> argv(args);
    return navigatr::parseCommandLine(static_cast<int>(argv.size()), argv.data(),
                                     "C:/robot config/default.xml", result, err);
}

TEST(CommandLine, NoFilenameUsesCompiledDefaultEvenWithRunOptions) {
    navigatr::CommandLine result;
    std::string err;
    ASSERT_TRUE(parse({"navigatr"}, result, err)) << err;
    EXPECT_EQ(result.config_path, "C:/robot config/default.xml");
    ASSERT_TRUE(parse({"navigatr", "--inline", "--cycles", "0"}, result, err)) << err;
    EXPECT_EQ(result.config_path, "C:/robot config/default.xml");
    EXPECT_TRUE(result.inline_mode);
    EXPECT_EQ(result.max_cycles, 0);
}

TEST(CommandLine, ExplicitFileSupportsEqualsSeparatedSpacesAndPositionalForm) {
    for (const auto args : {
             std::initializer_list<const char*>{"navigatr", "--config_file=trial configs/main.xml"},
             {"navigatr", "--config_file", "trial configs/main.xml"},
             {"navigatr", "--config-file", "trial configs/main.xml"},
             {"navigatr", "trial configs/main.xml"}}) {
        navigatr::CommandLine result;
        std::string err;
        ASSERT_TRUE(parse(args, result, err)) << err;
        EXPECT_EQ(result.config_path, "trial configs/main.xml");
    }
}

TEST(CommandLine, OverridesPreserveReplayAndRuntimeOptions) {
    navigatr::CommandLine result;
    std::string err;
    ASSERT_TRUE(parse({"navigatr", "--cycles=12", "--replay", "pico_uart=log files/run.bin",
                       "--config_file=experiment.xml", "--inspect-port", "8799", "--inline"},
                      result, err)) << err;
    EXPECT_EQ(result.config_path, "experiment.xml");
    EXPECT_EQ(result.max_cycles, 12);
    EXPECT_EQ(result.inspect_port, 8799);
    EXPECT_TRUE(result.inline_mode);
    EXPECT_EQ(result.build.replay.at("pico_uart"), "log files/run.bin");
}

TEST(CommandLine, MissingEmptyUnknownAndDuplicateArgumentsFail) {
    for (const auto args : {
             std::initializer_list<const char*>{"navigatr", "--config_file"},
             {"navigatr", "--config_file="},
             {"navigatr", "--config_file", "--inline"},
             {"navigatr", "one.xml", "--config_file=two.xml"},
             {"navigatr", "--config_file=one.xml", "two.xml"},
             {"navigatr", "--config_flie=typo.xml"},
             {"navigatr", "--replay", "missing_separator"},
             {"navigatr", "--inspect-port"}}) {
        navigatr::CommandLine result;
        std::string err;
        EXPECT_FALSE(parse(args, result, err));
        EXPECT_FALSE(err.empty());
    }
}

TEST(CommandLine, NumericOptionsRejectGarbageOverflowAndInvalidRanges) {
    for (const auto args : {
             std::initializer_list<const char*>{"navigatr", "--cycles", "not-a-number"},
             {"navigatr", "--cycles", "2suffix"},
             {"navigatr", "--cycles", "999999999999999999999999"},
             {"navigatr", "--cycles", "-2"},
             {"navigatr", "--inspect-port", "0"},
             {"navigatr", "--inspect-port", "65536"}}) {
        navigatr::CommandLine result;
        std::string err;
        EXPECT_FALSE(parse(args, result, err));
        EXPECT_FALSE(err.empty());
    }
}

TEST(CommandLine, HelpDoesNotRequireAnExplicitConfiguration) {
    navigatr::CommandLine result;
    std::string err;
    ASSERT_TRUE(parse({"navigatr", "--help"}, result, err)) << err;
    EXPECT_TRUE(result.help);
}

} // namespace
