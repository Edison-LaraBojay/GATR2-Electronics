// pose_tracker_gtest.cpp
// PROTOCOL.md receiver rules on decoded lines: restart on a new session or epoch, repeated and
// out-of-order drops, the seq wrap, drop counts, and the receiver's own rx stamp.

#include "receiver/feed/pose_tracker.h"

#include <gtest/gtest.h>

namespace spectagatr::receiver::feed
{
namespace
{

protocol::DecodeResult decoded(protocol::SessionId session, protocol::Epoch epoch,
                               protocol::Sequence seq, protocol::SourceMillis t_ms = 0) {
    protocol::DecodeResult result;
    result.error          = protocol::DecodeError::kNone;
    result.record.session = session;
    result.record.epoch   = epoch;
    result.record.seq     = seq;
    result.record.t_ms    = t_ms;
    return result;
}

protocol::DecodeResult failed(protocol::DecodeError error) {
    protocol::DecodeResult result;
    result.error = error;
    return result;
}

TEST(PoseTracker, FirstPoseIsARestartAndCountsASession) {
    PoseTracker                      tracker;
    const std::optional<TrackedPose> pose = tracker.offer(decoded(0xAB, 0, 7), 12.5);
    ASSERT_TRUE(pose);
    EXPECT_TRUE(pose->restart);
    EXPECT_EQ(pose->rx_ms, 12.5);
    EXPECT_EQ(pose->record.seq, 7u);
    EXPECT_EQ(tracker.counts().sessions, 1u);
    EXPECT_EQ(tracker.counts().poses, 1u);
    ASSERT_TRUE(tracker.lastRx());
    EXPECT_EQ(*tracker.lastRx(), 12.5);
}

TEST(PoseTracker, NewerSeqContinuesWithoutRestart) {
    PoseTracker tracker;
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 10), 0));
    const std::optional<TrackedPose> next = tracker.offer(decoded(1, 0, 12), 100);
    ASSERT_TRUE(next);
    EXPECT_FALSE(next->restart); // a gap is not a restart
    EXPECT_EQ(tracker.counts().poses, 2u);
    EXPECT_EQ(tracker.counts().sessions, 1u);
}

TEST(PoseTracker, NewSessionRestartsEvenWithAnOlderSeq) {
    PoseTracker tracker;
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 5000), 0));
    const std::optional<TrackedPose> pose = tracker.offer(decoded(2, 0, 0), 10);
    ASSERT_TRUE(pose);
    EXPECT_TRUE(pose->restart);
    EXPECT_EQ(tracker.counts().sessions, 2u);
    // Back to the first session id is a new run as well.
    const std::optional<TrackedPose> back = tracker.offer(decoded(1, 0, 0), 20);
    ASSERT_TRUE(back);
    EXPECT_TRUE(back->restart);
    EXPECT_EQ(tracker.counts().sessions, 3u);
}

TEST(PoseTracker, NewEpochRestartsTheTrailInTheSameSession) {
    PoseTracker tracker;
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 10), 0));
    const std::optional<TrackedPose> reset = tracker.offer(decoded(1, 1, 11), 10);
    ASSERT_TRUE(reset);
    EXPECT_TRUE(reset->restart);
    EXPECT_EQ(tracker.counts().sessions, 1u);
    const std::optional<TrackedPose> after = tracker.offer(decoded(1, 1, 12), 20);
    ASSERT_TRUE(after);
    EXPECT_FALSE(after->restart);
}

TEST(PoseTracker, RepeatedSeqIsDroppedAndCounted) {
    PoseTracker tracker;
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 10), 0));
    EXPECT_FALSE(tracker.offer(decoded(1, 0, 10), 5));
    EXPECT_EQ(tracker.lastVerdict(), Verdict::kRepeated);
    EXPECT_EQ(tracker.counts().repeated, 1u);
    EXPECT_EQ(tracker.counts().poses, 1u);
    EXPECT_EQ(*tracker.lastRx(), 0.0); // a dropped line does not refresh the age
}

TEST(PoseTracker, OlderSeqIsOutOfOrder) {
    PoseTracker tracker;
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 100), 0));
    EXPECT_FALSE(tracker.offer(decoded(1, 0, 99), 5));
    EXPECT_EQ(tracker.lastVerdict(), Verdict::kOutOfOrder);
    EXPECT_FALSE(tracker.offer(decoded(1, 0, 100 - 64), 6)); // still within the window
    EXPECT_EQ(tracker.counts().out_of_order, 2u);
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 101), 7)); // ordering continues from 100
}

// Session ids are best effort: a restarted publisher that drew the same id
// starts again near seq 0. A serial stream does not reorder lines, so a jump
// back past the window is a restart, not dozens of minutes of drops.
TEST(PoseTracker, LargeSeqJumpBackIsARestartedPublisherWithTheSameId) {
    PoseTracker tracker;
    ASSERT_TRUE(tracker.offer(decoded(7, 0, 5000), 0));
    const std::optional<TrackedPose> again = tracker.offer(decoded(7, 0, 0), 10);
    ASSERT_TRUE(again);
    EXPECT_TRUE(again->restart);
    EXPECT_EQ(tracker.counts().sessions, 2u);
    EXPECT_EQ(tracker.counts().out_of_order, 0u);
    ASSERT_TRUE(tracker.offer(decoded(7, 0, 1), 20));
}

TEST(PoseTracker, SeqWrapsModulo32Bits) {
    PoseTracker tracker;
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 0xFFFFFFFEu), 0));
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 0xFFFFFFFFu), 1));
    const std::optional<TrackedPose> wrapped = tracker.offer(decoded(1, 0, 0), 2);
    ASSERT_TRUE(wrapped);
    EXPECT_FALSE(wrapped->restart);
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 5), 3));
    EXPECT_FALSE(tracker.offer(decoded(1, 0, 0xFFFFFFFFu), 4)); // before the wrap: older
    EXPECT_EQ(tracker.counts().out_of_order, 1u);
}

TEST(PoseTracker, PublisherClockIsPassedOnNeverCompared) {
    PoseTracker tracker;
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 1, 0xFFFFFFF0u), 1000));
    // t_ms wrapped and went "backwards": still newer by seq, so accepted.
    const std::optional<TrackedPose> pose = tracker.offer(decoded(1, 0, 2, 5), 1100);
    ASSERT_TRUE(pose);
    EXPECT_EQ(pose->record.t_ms, 5u);
    EXPECT_EQ(pose->rx_ms, 1100);
}

TEST(PoseTracker, DecodeErrorsAreCountedPerReason) {
    PoseTracker tracker;
    EXPECT_FALSE(tracker.offer(failed(protocol::DecodeError::kBadCrc), 0));
    EXPECT_EQ(tracker.lastVerdict(), Verdict::kBadCrc);
    EXPECT_FALSE(tracker.offer(failed(protocol::DecodeError::kMalformed), 0));
    EXPECT_FALSE(tracker.offer(failed(protocol::DecodeError::kMalformed), 0));
    EXPECT_FALSE(tracker.offer(failed(protocol::DecodeError::kOutOfRange), 0));
    EXPECT_EQ(tracker.counts().bad_crc, 1u);
    EXPECT_EQ(tracker.counts().malformed, 2u);
    EXPECT_EQ(tracker.counts().out_of_range, 1u);
    EXPECT_EQ(tracker.counts().poses, 0u);
    EXPECT_FALSE(tracker.lastRx());
}

TEST(PoseTracker, ForgetOrderingAcceptsAReplayedStreamAsARestart) {
    PoseTracker tracker;
    ASSERT_TRUE(tracker.offer(decoded(1, 0, 10), 0));
    tracker.forgetOrdering();
    const std::optional<TrackedPose> again = tracker.offer(decoded(1, 0, 0), 10);
    ASSERT_TRUE(again);
    EXPECT_TRUE(again->restart);
}

TEST(SequenceDistance, SignedModulo32) {
    EXPECT_EQ(sequenceDistance(5, 3), 2);
    EXPECT_EQ(sequenceDistance(3, 5), -2);
    EXPECT_EQ(sequenceDistance(0, 0xFFFFFFFFu), 1);
    EXPECT_EQ(sequenceDistance(0xFFFFFFFFu, 0), -1);
    EXPECT_EQ(sequenceDistance(0x80000000u, 0), INT32_MIN);
    EXPECT_EQ(sequenceDistance(0x7FFFFFFFu, 0), INT32_MAX);
}

} // namespace
} // namespace spectagatr::receiver::feed
