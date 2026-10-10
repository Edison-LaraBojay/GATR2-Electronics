// concurrency_gtest.cpp
// Writers hammer updatePose while the transport pulls lines. Every value of
// a pose is derived from one counter k, so a torn pose shows as a mismatch.

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "spectagatr/publisher.h"

namespace
{

using namespace spectagatr;
using protocol::PoseRecord;

thread_local Milliseconds t_current_k = 0;

Milliseconds counterClock() { return t_current_k; }

int32_t yawOf(uint32_t k) { return static_cast<int32_t>(k % 35999) - 17999; }

int32_t rollOf(uint32_t k) { return static_cast<int32_t>(k % 9000) - 4500; }

int32_t pitchOf(uint32_t k) { return static_cast<int32_t>(k % 7000) - 3500; }

// Odd k: full attitude; even k: heading only.
void publishK(Publisher& publisher, uint32_t k) {
    t_current_k       = k;
    const Meters  x   = protocol::toMeters(static_cast<int32_t>(k));
    const Radians yaw = protocol::toRadians(yawOf(k));
    if (k % 2 == 1) {
        publisher.updatePose(
            x, -x, Attitude{protocol::toRadians(rollOf(k)), protocol::toRadians(pitchOf(k)), yaw});
    } else {
        publisher.updatePose(x, -x, yaw);
    }
}

// Empty string when consistent, else what is wrong.
std::string check(const PoseRecord& r) {
    const uint32_t k = static_cast<uint32_t>(r.x_mm);
    if (r.y_mm != -r.x_mm) {
        return "y";
    }
    if (r.t_ms != k) {
        return "t_ms";
    }
    if (r.yaw_cdeg != yawOf(k)) {
        return "yaw";
    }
    const bool full = (r.flags & protocol::kFullAttitude) != 0;
    if (full != (k % 2 == 1)) {
        return "flags";
    }
    if (full ? (r.roll_cdeg != rollOf(k) || r.pitch_cdeg != pitchOf(k))
             : (r.roll_cdeg != 0 || r.pitch_cdeg != 0)) {
        return "roll/pitch";
    }
    return "";
}

void run(int writer_count, uint32_t updates_per_writer) {
    Publisher publisher(&counterClock, kMaxRateHz);
    publisher.begin(0xC0FFEE01u);

    std::atomic<int>         running{writer_count};
    std::vector<std::thread> writers;
    for (int w = 0; w < writer_count; ++w) {
        writers.emplace_back([&publisher, &running, w, updates_per_writer] {
            const uint32_t base = 1 + static_cast<uint32_t>(w) * 1'000'000u;
            for (uint32_t i = 0; i < updates_per_writer; ++i) {
                publishK(publisher, base + i % 999'999u);
            }
            running.fetch_sub(1);
        });
    }

    Milliseconds now      = 0;
    uint32_t     lines    = 0;
    uint32_t     bad      = 0;
    Sequence     expected = 0;
    char         buffer[kLineCapacity];
    while (running.load() > 0) {
        now += 20; // always due at 50 Hz
        const std::size_t length = publisher.nextLine(now, buffer, sizeof(buffer));
        if (length == 0) {
            continue;
        }
        const protocol::DecodeResult result =
            protocol::decodeLine(std::string_view(buffer, length - 1));
        ASSERT_EQ(result.error, protocol::DecodeError::kNone);
        EXPECT_EQ(result.record.seq, expected++);
        const std::string problem = check(result.record);
        if (!problem.empty()) {
            ++bad;
            ADD_FAILURE() << "torn " << problem << ": " << std::string_view(buffer, length);
        }
        ++lines;
    }
    for (std::thread& t : writers) {
        t.join();
    }
    EXPECT_EQ(bad, 0u);
    EXPECT_GT(lines, 100u);
    EXPECT_EQ(publisher.stats().updates, writer_count * updates_per_writer);
    EXPECT_EQ(publisher.stats().rejected, 0u);
}

TEST(SpectaConcurrency, OneWriterOneReader) { run(1, 2'000'000); }

TEST(SpectaConcurrency, TwoWritersOneReader) { run(2, 1'000'000); }

} // namespace
