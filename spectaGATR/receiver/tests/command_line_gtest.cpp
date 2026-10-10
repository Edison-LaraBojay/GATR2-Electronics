// command_line_gtest.cpp
// Options: defaults from the contract, values, implied sources, and refusals.

#include "receiver/app/command_line.h"

#include <gtest/gtest.h>

namespace spectagatr::receiver::app
{
namespace
{

ParsedCommandLine parse(std::vector<std::string> args) { return parseCommandLine(args); }

TEST(CommandLine, Defaults) {
    const ParsedCommandLine parsed = parse({});
    ASSERT_TRUE(parsed.options) << parsed.error;
    const Options& o = *parsed.options;
    EXPECT_EQ(o.source, SourceChoice::kAuto);
    EXPECT_EQ(o.http_port, 8790);
    EXPECT_EQ(o.bind, "127.0.0.1");
    EXPECT_EQ(o.field, "override");
    EXPECT_EQ(o.stale_ms, 750.0);
    EXPECT_EQ(o.rate_hz, 10.0);
    EXPECT_EQ(o.sim_mode, sim::SimMode::kFigure8);
    EXPECT_TRUE(o.open_browser);
    EXPECT_FALSE(o.port);
    EXPECT_EQ(o.robot.front, 0.1685);
    EXPECT_EQ(o.robot.left, 0.1465);
    EXPECT_EQ(o.robot.height, 0.25);
}

TEST(CommandLine, EveryValue) {
    const ParsedCommandLine parsed =
        parse({"--source", "controller", "--port", "COM7", "--http-port=9000", "--bind", "0.0.0.0",
               "--field", "empty", "--web", "w", "--fields", "f", "--stale-ms", "500", "--robot",
               "0.2,0.1,0.15,0.15", "--no-browser"});
    ASSERT_TRUE(parsed.options) << parsed.error;
    const Options& o = *parsed.options;
    EXPECT_EQ(o.source, SourceChoice::kController);
    EXPECT_EQ(*o.port, "COM7");
    EXPECT_EQ(o.http_port, 9000);
    EXPECT_EQ(o.bind, "0.0.0.0");
    EXPECT_EQ(o.field, "empty");
    EXPECT_EQ(*o.web_dir, "w");
    EXPECT_EQ(*o.fields_dir, "f");
    EXPECT_EQ(o.stale_ms, 500.0);
    EXPECT_EQ(o.robot.front, 0.2);
    EXPECT_EQ(o.robot.back, 0.1);
    EXPECT_EQ(o.robot.height, 0.25); // not given: default
    EXPECT_FALSE(o.open_browser);
}

TEST(CommandLine, SimOptionsAndTheImpliedSource) {
    const ParsedCommandLine parsed = parse({"--sim", "attitude", "--rate", "25", "--seed", "9"});
    ASSERT_TRUE(parsed.options) << parsed.error;
    EXPECT_EQ(parsed.options->source, SourceChoice::kSim);
    EXPECT_EQ(parsed.options->sim_mode, sim::SimMode::kAttitude);
    EXPECT_EQ(parsed.options->rate_hz, 25.0);
    EXPECT_EQ(parsed.options->seed, 9u);
    const ParsedCommandLine robot = parse({"--source", "sim", "--robot", "0.2,0.2,0.2,0.2,0.4"});
    ASSERT_TRUE(robot.options) << robot.error;
    EXPECT_EQ(robot.options->robot.height, 0.4);
}

TEST(CommandLine, ReplayOptions) {
    const ParsedCommandLine parsed =
        parse({"--source", "file", "--replay", "-", "--replay-rate-bps", "4000", "--loop"});
    ASSERT_TRUE(parsed.options) << parsed.error;
    EXPECT_EQ(parsed.options->replay_path, "-");
    EXPECT_EQ(*parsed.options->replay_rate_bps, 4000.0);
    EXPECT_TRUE(parsed.options->loop);
    EXPECT_EQ(parse({"--replay", "capture.bin"}).options->source, SourceChoice::kFile);
}

TEST(CommandLine, Refusals) {
    EXPECT_FALSE(parse({"--source", "wifi"}).options);
    EXPECT_FALSE(parse({"--sim", "spiral"}).options);
    EXPECT_FALSE(parse({"--rate", "0"}).options);
    EXPECT_FALSE(parse({"--rate", "51"}).options);
    EXPECT_FALSE(parse({"--rate", "abc"}).options);
    EXPECT_FALSE(parse({"--http-port", "70000"}).options);
    EXPECT_FALSE(parse({"--stale-ms", "nan"}).options);
    EXPECT_FALSE(parse({"--robot", "0.2,0.2,0.2"}).options);
    EXPECT_FALSE(parse({"--robot", "0.2,0.2,0.2,-1"}).options);
    EXPECT_FALSE(parse({"--robot", "0.2,0.2,,0.2"}).options);
    EXPECT_FALSE(parse({"--port"}).options);
    EXPECT_FALSE(parse({"--frobnicate"}).options);
    EXPECT_FALSE(parse({"--no-browser=1"}).options);
    EXPECT_FALSE(parse({"--source", "file"}).options); // needs --replay
    EXPECT_FALSE(parse({"--source", "brain", "--sim", "circle"}).options);
    EXPECT_FALSE(parse({"--source", "sim", "--port", "COM7"}).options);
    EXPECT_FALSE(parse({"--loop"}).options);
    const ParsedCommandLine bad = parse({"--rate", "0"});
    EXPECT_NE(bad.error.find("--rate"), std::string::npos);
}

TEST(CommandLine, HelpAndListPorts) {
    EXPECT_TRUE(parse({"--help"}).options->help);
    EXPECT_TRUE(parse({"-h"}).options->help);
    EXPECT_TRUE(parse({"--list-ports"}).options->list_ports);
    EXPECT_NE(usageText().find("--source auto|controller|brain|sim|file"), std::string::npos);
}

} // namespace
} // namespace spectagatr::receiver::app
