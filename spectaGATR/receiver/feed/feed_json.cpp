// feed_json.cpp

#include "receiver/feed/feed_json.h"

#include <cmath>
#include <cstdio>

#include "spectagatr/protocol/units.h"

namespace spectagatr::receiver::feed
{

namespace
{

constexpr int kMeterDecimals  = 4; // wire mm, exact at 3
constexpr int kRadianDecimals = 6; // wire centidegrees, 1.7e-4 rad
constexpr int kMillisDecimals = 1;

// One JSON object written key by key; commas go in by themselves.
class Object {
  public:
    explicit Object(std::string& out) : out_(out) { out_ += '{'; }

    Object& text(const char* key, std::string_view value) {
        name(key);
        appendJsonString(out_, value);
        return *this;
    }
    Object& number(const char* key, double value, int decimals) {
        name(key);
        appendJsonNumber(out_, value, decimals);
        return *this;
    }
    Object& integer(const char* key, int64_t value) {
        name(key);
        out_ += std::to_string(value);
        return *this;
    }
    Object& count(const char* key, uint64_t value) {
        name(key);
        out_ += std::to_string(value);
        return *this;
    }
    Object& boolean(const char* key, bool value) {
        name(key);
        out_ += value ? "true" : "false";
        return *this;
    }
    Object& null(const char* key) {
        name(key);
        out_ += "null";
        return *this;
    }
    Object& names(const char* key, const FieldNames& values) {
        name(key);
        out_ += '[';
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i > 0) {
                out_ += ',';
            }
            appendJsonString(out_, values[i]);
        }
        out_ += ']';
        return *this;
    }
    // The caller writes the nested value right after.
    std::string& raw(const char* key) {
        name(key);
        return out_;
    }
    void close() { out_ += '}'; }

  private:
    void name(const char* key) {
        if (!first_) {
            out_ += ',';
        }
        first_ = false;
        appendJsonString(out_, key);
        out_ += ':';
    }

    std::string& out_;
    bool         first_ = true;
};

std::string hex8(uint32_t value) {
    char text[16];
    std::snprintf(text, sizeof(text), "%08lX", static_cast<unsigned long>(value));
    return text;
}

void appendRobot(std::string& out, const RobotBox& robot) {
    Object box(out);
    box.number("front_m", robot.front, kMeterDecimals)
        .number("back_m", robot.back, kMeterDecimals)
        .number("left_m", robot.left, kMeterDecimals)
        .number("right_m", robot.right, kMeterDecimals)
        .number("height_m", robot.height, kMeterDecimals);
    box.close();
}

void appendCounts(std::string& out, const FeedCounts& counts) {
    Object object(out);
    object.count("poses", counts.poses)
        .count("bad_crc", counts.bad_crc)
        .count("malformed", counts.malformed)
        .count("out_of_range", counts.out_of_range)
        .count("repeated", counts.repeated)
        .count("out_of_order", counts.out_of_order)
        .count("truncated", counts.truncated)
        .count("too_long", counts.too_long)
        .count("garbage", counts.garbage)
        .count("sessions", counts.sessions)
        .count("bytes", counts.bytes);
    object.close();
}

} // namespace

void appendJsonString(std::string& out, std::string_view text) {
    out += '"';
    for (const char c : text) {
        const auto byte = static_cast<unsigned char>(c);
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (byte >= 0x20 && byte < 0x7F) {
            out += c;
        } else {
            char escaped[8];
            std::snprintf(escaped, sizeof(escaped), "\\u%04X", static_cast<unsigned>(byte));
            out += escaped;
        }
    }
    out += '"';
}

void appendJsonNumber(std::string& out, double value, int decimals) {
    if (!std::isfinite(value)) {
        out += "null";
        return;
    }
    char      text[64];
    const int length = std::snprintf(text, sizeof(text), "%.*f", decimals, value);
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(text)) {
        out += "null"; // beyond any value this feed carries
        return;
    }
    std::string number(text, static_cast<std::size_t>(length));
    if (number.find('.') != std::string::npos) {
        while (number.back() == '0') {
            number.pop_back();
        }
        if (number.back() == '.') {
            number.pop_back();
        }
    }
    if (number == "-0") {
        number = "0";
    }
    out += number;
}

std::string helloJson(const HelloInfo& hello) {
    std::string out;
    Object      object(out);
    object.text("type", "hello")
        .integer("protocol", kFeedProtocol)
        .text("receiver", kReceiverName)
        .integer("stale_ms", hello.stale_ms)
        .text("field", hello.field)
        .names("fields", hello.fields);
    appendRobot(object.raw("robot"), hello.robot);
    object.close();
    return out;
}

std::string poseJson(const TrackedPose& pose, std::string_view source) {
    const protocol::PoseRecord& r = pose.record;
    std::string                 out;
    Object                      object(out);
    object.text("type", "pose")
        .text("session", hex8(r.session))
        .integer("epoch", r.epoch)
        .count("seq", r.seq)
        .count("t_ms", r.t_ms)
        .number("x", protocol::toMeters(r.x_mm), kMeterDecimals)
        .number("y", protocol::toMeters(r.y_mm), kMeterDecimals)
        .number("yaw", protocol::toRadians(r.yaw_cdeg), kRadianDecimals)
        .number("roll", protocol::toRadians(r.roll_cdeg), kRadianDecimals)
        .number("pitch", protocol::toRadians(r.pitch_cdeg), kRadianDecimals)
        .boolean("attitude", (r.flags & protocol::kFullAttitude) != 0)
        .boolean("restart", pose.restart)
        .number("rx_ms", pose.rx_ms, kMillisDecimals)
        .text("source", source);
    object.close();
    return out;
}

std::string statusJson(const FeedStatus& status) {
    std::string out;
    Object      object(out);
    object.text("type", "status")
        .number("rx_ms", status.rx_ms, kMillisDecimals)
        .text("source", status.source)
        .text("port", status.port)
        .text("link", linkStateName(status.link))
        .text("detail", status.detail)
        .boolean("stale", status.stale);
    if (status.pose_age_ms) {
        object.integer("pose_age_ms", *status.pose_age_ms);
    } else {
        object.null("pose_age_ms");
    }
    appendCounts(object.raw("counts"), status.counts);
    object.close();
    return out;
}

std::string fieldsJson(const FieldNames& fields, const std::string& default_field) {
    std::string out;
    Object      object(out);
    object.names("fields", fields).text("default", default_field);
    object.close();
    return out;
}

} // namespace spectagatr::receiver::feed
