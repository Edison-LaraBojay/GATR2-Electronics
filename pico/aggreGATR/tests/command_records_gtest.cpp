// command_records_gtest.cpp

#include <gtest/gtest.h>

#include "command_records.h"
#include "frames.h"

using namespace pilink;

namespace
{

CommandRecord make(uint16_t id, uint8_t op, uint8_t body) {
    CommandRecord r;
    r.request_id = id;
    r.op         = op;
    r.body       = body;
    r.status     = translagatr::kPicoCommandCompleted;
    return r;
}

} // namespace

TEST(CommandRecords, FindsOnlyTheExactCommand) {
    CommandRecords records;
    records.add(make(7, translagatr::kPicoOpConfigure, 1));
    EXPECT_NE(records.find(7, translagatr::kPicoOpConfigure, 1), nullptr);
    EXPECT_EQ(records.find(7, translagatr::kPicoOpConfigure, 0), nullptr);
    EXPECT_EQ(records.find(7, translagatr::kPicoOpReinitImu, 1), nullptr);
    EXPECT_EQ(records.find(8, translagatr::kPicoOpConfigure, 1), nullptr);
}

TEST(CommandRecords, EmptySlotsNeverMatch) {
    CommandRecords records;
    EXPECT_EQ(records.find(0, 0, 0), nullptr);
    records.add(make(1, translagatr::kPicoOpRestartAcquisition, 0));
    EXPECT_EQ(records.find(0, 0, 0), nullptr);
}

TEST(CommandRecords, KeepsTheLastFour) {
    CommandRecords records;
    for (uint16_t id = 1; id <= 5; ++id) {
        records.add(make(id, translagatr::kPicoOpRestartAcquisition, 0));
    }
    EXPECT_EQ(records.find(1, translagatr::kPicoOpRestartAcquisition, 0), nullptr);
    for (uint16_t id = 2; id <= 5; ++id) {
        EXPECT_NE(records.find(id, translagatr::kPicoOpRestartAcquisition, 0), nullptr) << id;
    }
}

TEST(CommandRecords, SameIdReplacesItsRecordAndBecomesNewest) {
    CommandRecords records;
    records.add(make(1, translagatr::kPicoOpConfigure, 1));
    records.add(make(2, translagatr::kPicoOpConfigure, 1));
    records.add(make(3, translagatr::kPicoOpConfigure, 1));
    records.add(make(4, translagatr::kPicoOpConfigure, 1));

    records.add(make(1, translagatr::kPicoOpReinitImu, 0));
    EXPECT_EQ(records.find(1, translagatr::kPicoOpConfigure, 1), nullptr);
    EXPECT_NE(records.find(1, translagatr::kPicoOpReinitImu, 0), nullptr);

    // Id 2 is now the oldest.
    records.add(make(5, translagatr::kPicoOpConfigure, 0));
    EXPECT_EQ(records.find(2, translagatr::kPicoOpConfigure, 1), nullptr);
    EXPECT_NE(records.find(1, translagatr::kPicoOpReinitImu, 0), nullptr);
    EXPECT_NE(records.find(3, translagatr::kPicoOpConfigure, 1), nullptr);
}

TEST(CommandRecords, RecordsAreUpdatedInPlace) {
    CommandRecords records;
    CommandRecord& r           = records.add(make(9, translagatr::kPicoOpReinitImu, 0));
    r.status                   = translagatr::kPicoCommandFailed;
    r.detail                   = translagatr::kPicoDetailImuAbsent;
    const CommandRecord* found = records.find(9, translagatr::kPicoOpReinitImu, 0);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->status, translagatr::kPicoCommandFailed);
    EXPECT_EQ(found->detail, translagatr::kPicoDetailImuAbsent);
}

TEST(CommandRecords, ClearForgetsEverything) {
    CommandRecords records;
    records.add(make(3, translagatr::kPicoOpConfigure, 1));
    records.clear();
    EXPECT_EQ(records.find(3, translagatr::kPicoOpConfigure, 1), nullptr);
    for (uint8_t i = 0; i < CommandRecords::kCapacity; ++i) {
        EXPECT_EQ(records.at(i).request_id, 0);
    }
}
