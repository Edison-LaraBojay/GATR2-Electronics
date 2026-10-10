// feed_json_gtest.cpp
// Browser messages: exact hello and status text, finite numbers only, escaping to plain ASCII.

#include "receiver/feed/feed_json.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace spectagatr::receiver::feed
{
namespace
{

std::string number(double value, int decimals) {
    std::string out;
    appendJsonNumber(out, value, decimals);
    return out;
}

std::string quoted(std::string_view text) {
    std::string out;
    appendJsonString(out, text);
    return out;
}

TEST(FeedJson, NumbersAreFiniteAndTrimmed) {
    EXPECT_EQ(number(1.5, 4), "1.5");
    EXPECT_EQ(number(-0.25, 4), "-0.25");
    EXPECT_EQ(number(2.0, 4), "2");
    EXPECT_EQ(number(-0.00001, 4), "0"); // no "-0"
    EXPECT_EQ(number(0.0, 1), "0");
    EXPECT_EQ(number(98765.43, 1), "98765.4");
    EXPECT_EQ(number(std::numeric_limits<double>::quiet_NaN(), 3), "null");
    EXPECT_EQ(number(std::numeric_limits<double>::infinity(), 3), "null");
    EXPECT_EQ(number(-std::numeric_limits<double>::infinity(), 3), "null");
}

TEST(FeedJson, StringsAreEscapedToAscii) {
    EXPECT_EQ(quoted("plain"), "\"plain\"");
    EXPECT_EQ(quoted("a\"b\\c"), "\"a\\\"b\\\\c\"");
    EXPECT_EQ(quoted("line\nfeed\ttab"), "\"line\\u000Afeed\\u0009tab\"");
    EXPECT_EQ(quoted("\x7f\xc3\xa9"), "\"\\u007F\\u00C3\\u00A9\""); // never raw non-ASCII
    EXPECT_EQ(quoted(std::string_view("a\0b", 3)), "\"a\\u0000b\"");
}

TEST(FeedJson, HelloMatchesTheProtocol) {
    HelloInfo hello;
    hello.fields = {"empty", "override"};
    EXPECT_EQ(helloJson(hello),
              "{\"type\":\"hello\",\"protocol\":1,\"receiver\":\"spectagatr 1.0\","
              "\"stale_ms\":750,\"field\":\"override\",\"fields\":[\"empty\",\"override\"],"
              "\"robot\":{\"front_m\":0.1685,\"back_m\":0.1685,\"left_m\":0.1465,"
              "\"right_m\":0.1465,\"height_m\":0.25}}");
}

TEST(FeedJson, StatusMatchesTheProtocol) {
    FeedStatus status;
    status.rx_ms           = 98800.0;
    status.source          = "controller";
    status.port            = "COM7";
    status.link            = LinkState::kOpen;
    status.detail          = "controller: Brain linked by radio";
    status.stale           = false;
    status.pose_age_ms     = 35;
    status.counts.poses    = 1234;
    status.counts.sessions = 1;
    status.counts.bytes    = 56789;
    EXPECT_EQ(statusJson(status),
              "{\"type\":\"status\",\"rx_ms\":98800,\"source\":\"controller\",\"port\":\"COM7\","
              "\"link\":\"open\",\"detail\":\"controller: Brain linked by radio\","
              "\"stale\":false,\"pose_age_ms\":35,\"counts\":{\"poses\":1234,\"bad_crc\":0,"
              "\"malformed\":0,\"out_of_range\":0,\"repeated\":0,\"out_of_order\":0,"
              "\"truncated\":0,\"too_long\":0,\"garbage\":0,\"sessions\":1,\"bytes\":56789}}");
}

TEST(FeedJson, StatusBeforeTheFirstPoseHasANullAge) {
    FeedStatus        status;
    const std::string json = statusJson(status);
    EXPECT_NE(json.find("\"pose_age_ms\":null"), std::string::npos);
    EXPECT_NE(json.find("\"link\":\"searching\""), std::string::npos);
    EXPECT_NE(json.find("\"stale\":true"), std::string::npos);
    EXPECT_NE(json.find("\"source\":\"none\""), std::string::npos);
}

TEST(FeedJson, PoseValuesAreSi) {
    TrackedPose pose;
    pose.record.session    = 0x0000ABCD;
    pose.record.x_mm       = -2000000;
    pose.record.y_mm       = 1;
    pose.record.yaw_cdeg   = 18000;
    pose.rx_ms             = 0.05;
    const std::string json = poseJson(pose, "sim");
    EXPECT_NE(json.find("\"session\":\"0000ABCD\""), std::string::npos);
    EXPECT_NE(json.find("\"x\":-2000,"), std::string::npos);
    EXPECT_NE(json.find("\"y\":0.001,"), std::string::npos);
    EXPECT_NE(json.find("\"yaw\":3.141593,"), std::string::npos);
    EXPECT_NE(json.find("\"roll\":0,"), std::string::npos);
    EXPECT_NE(json.find("\"attitude\":false"), std::string::npos);
    EXPECT_NE(json.find("\"rx_ms\":0.1"), std::string::npos);
}

TEST(FeedJson, FieldsListing) {
    EXPECT_EQ(fieldsJson({"empty", "override"}, "override"),
              "{\"fields\":[\"empty\",\"override\"],\"default\":\"override\"}");
    EXPECT_EQ(fieldsJson({}, "override"), "{\"fields\":[],\"default\":\"override\"}");
}

} // namespace
} // namespace spectagatr::receiver::feed
