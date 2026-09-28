// field_sync_gtest.cpp
// Field map and estimate transfer against the fake Pi: different object
// counts, chunking, atomic publication, malformed and inconsistent
// documents, replaced estimates and Stale, backoff, interrupted transfers,
// map changes and the map cache across sessions and Pi restarts.

#include "communigatr/client.h"

#include <gtest/gtest.h>
#include <vector>

#include "common/link_documents.h"
#include "sim/link_rig.h"

using namespace communigatr;

namespace
{

constexpr Seconds kLimit = 5.0;

std::vector<gatr2::BrainRequest> reads(const FakeBus& bus, uint8_t kind) {
    std::vector<gatr2::BrainRequest> out;
    for (const gatr2::BrainRequest& r : bus.brainRequests()) {
        if (r.op == gatr2::kOpReadDoc && r.doc_kind == kind) {
            out.push_back(r);
        }
    }
    return out;
}

// The publication is a validated pair or nothing.
void expectConsistent(const FieldPublication& p) {
    if (p.generation == 0) {
        EXPECT_TRUE(p.map.empty());
        EXPECT_TRUE(p.estimate.empty());
        return;
    }
    const uint16_t map_len = static_cast<uint16_t>(p.map.size());
    ASSERT_EQ(gatr2::crc32(p.map.data(), map_len), p.map_id);
    ASSERT_EQ(gatr2::validateFieldMap(p.map.data(), map_len), gatr2::DocError::kNone);
    ASSERT_EQ(gatr2::validateFieldEstimate(p.estimate.data(),
                                           static_cast<uint16_t>(p.estimate.size()),
                                           p.map.data(), map_len, p.map_id),
              gatr2::DocError::kNone);
    gatr2::FieldEstimateHeader header;
    ASSERT_TRUE(gatr2::decodeFieldEstimateHeader(
        p.estimate.data(), static_cast<uint16_t>(p.estimate.size()), header));
    EXPECT_EQ(header.estimate_id, p.estimate_id);
}

bool published(LinkRig& rig) {
    return rig.runUntil([&] { return rig.client().field().generation != 0; }, kLimit);
}

} // namespace

TEST(FieldSync, EveryObjectCountArrivesWholeAndChecked) {
    for (uint16_t count : {0, 1, 2, 7, 37, 128}) {
        SCOPED_TRACE(count);
        LinkRig rig;
        rig.pi.setField(makeFakeField(count, 3));
        Client& client = rig.client();
        const Seconds end = rig.now() + 10.0;
        while (client.field().generation == 0 && rig.now() < end) {
            rig.step();
            expectConsistent(client.field()); // never partial
        }
        const FieldPublication& p = client.field();
        ASSERT_NE(p.generation, 0u);
        expectConsistent(p);
        EXPECT_EQ(p.map, rig.pi.mapDocument());
        EXPECT_EQ(p.map_id, rig.pi.mapId());
        EXPECT_EQ(p.estimate_id, rig.pi.newestEstimate());
        EXPECT_EQ(p.pi_instance, rig.pi.piInstance());
        EXPECT_EQ(client.fieldSync().map_id, rig.pi.mapId());

        const std::size_t map_chunks =
            (gatr2::fieldMapLen(count) + gatr2::kDocChunkMax - 1) / gatr2::kDocChunkMax;
        const std::size_t est_chunks =
            (gatr2::fieldEstimateLen(count) + gatr2::kDocChunkMax - 1) / gatr2::kDocChunkMax;
        EXPECT_EQ(reads(rig.bus, gatr2::kDocFieldMap).size(), map_chunks);
        EXPECT_EQ(reads(rig.bus, gatr2::kDocFieldEstimate).size(), est_chunks);
        EXPECT_EQ(client.stats().doc_rejects, 0u);
        EXPECT_EQ(client.stats().maps, 1u);
        EXPECT_EQ(client.stats().estimates, 1u);

        // The same generation stays; nothing is read again without a change.
        rig.run(1.0);
        EXPECT_EQ(client.field().generation, p.generation);
        EXPECT_EQ(reads(rig.bus, gatr2::kDocFieldMap).size(), map_chunks);
    }
}

TEST(FieldSync, NoFieldOnThePiMeansNoReads) {
    LinkRig rig;
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
    rig.run(1.0);
    EXPECT_EQ(rig.client().field().generation, 0u);
    EXPECT_TRUE(reads(rig.bus, gatr2::kDocFieldMap).empty());
    EXPECT_TRUE(reads(rig.bus, gatr2::kDocFieldEstimate).empty());
    investigatr::Field field;
    EXPECT_FALSE(rig.driver().field(field));
}

TEST(FieldSync, InconsistentChunksAreDroppedThenTheTransferRecovers) {
    struct Case {
        const char*                             name;
        std::function<void(gatr2::BrainReply&)> edit;
    };
    const std::vector<Case> cases = {
        {"total_len", [](gatr2::BrainReply& r) { r.doc_total_len += 4; }},
        {"crc", [](gatr2::BrainReply& r) { r.doc_crc32 ^= 0x10; }},
        {"doc id", [](gatr2::BrainReply& r) { r.doc_id += 1; }},
        {"offset", [](gatr2::BrainReply& r) { r.doc_offset += 1; }},
        {"data", [](gatr2::BrainReply& r) { r.data[5] ^= 0x01; }},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.name);
        LinkRig rig;
        rig.pi.setField(makeFakeField(20));
        int edits      = 0;
        rig.pi.doc_hook = [&](gatr2::BrainReply& r) {
            if (r.doc_offset == gatr2::kDocChunkMax && edits < 2) {
                ++edits; // second chunk, twice
                c.edit(r);
            }
        };
        Client& client = rig.client();
        const Seconds end = rig.now() + 10.0;
        while (client.field().generation == 0 && rig.now() < end) {
            rig.step();
            expectConsistent(client.field());
        }
        ASSERT_NE(client.field().generation, 0u);
        expectConsistent(client.field());
        EXPECT_GE(client.stats().doc_rejects, 1u);
        EXPECT_EQ(client.field().map, rig.pi.mapDocument());
    }
}

TEST(FieldSync, MalformedMapIsNeverPublishedAndBacksOff) {
    ClientConfig config;
    config.transfer_backoff = 1.0;
    LinkRig rig(config);
    // Header says 5 objects, the document holds 4 records.
    const FakeField      field = makeFakeField(5);
    std::vector<uint8_t> doc(gatr2::fieldMapLen(5));
    gatr2::FieldMapHeader header;
    header.revision     = 1;
    header.object_count = 5;
    header.max_x_mm     = 3658;
    header.max_y_mm     = 3658;
    const uint16_t cap  = static_cast<uint16_t>(doc.size());
    gatr2::encodeFieldMapHeader(header, doc.data(), cap);
    for (uint16_t i = 0; i < 5; ++i) {
        gatr2::encodeFieldObjectRecord(field.objects[i], i, doc.data(), cap);
    }
    doc.resize(doc.size() - gatr2::kFieldMapRecordLen);
    rig.pi.setMapDocument(doc);

    rig.run(2.5);
    const Client& client = rig.client();
    EXPECT_EQ(client.field().generation, 0u);
    EXPECT_EQ(client.fieldSync().map_id, 0u);
    EXPECT_GE(client.stats().doc_rejects, 3u);
    // Three failures, a pause, three more: bounded, not a storm.
    const std::size_t map_reads = reads(rig.bus, gatr2::kDocFieldMap).size();
    EXPECT_LE(map_reads, 3u * 2u * 3u);
    EXPECT_TRUE(reads(rig.bus, gatr2::kDocFieldEstimate).empty());
}

TEST(FieldSync, EstimateForAnotherMapOrWithAnotherCountIsRejected) {
    LinkRig rig;
    rig.pi.setField(makeFakeField(6));
    ASSERT_TRUE(published(rig));
    const uint32_t generation = rig.client().field().generation;

    rig.pi.setEstimateMapId(0x12345678);
    rig.pi.publishNominalEstimate();
    rig.run(1.5);
    EXPECT_EQ(rig.client().field().generation, generation);
    EXPECT_GE(rig.client().stats().doc_rejects, 1u);

    rig.pi.setEstimateMapId(0);
    std::vector<gatr2::FieldEstimateRecord> records = rig.pi.nominalRecords();
    records.pop_back(); // count no longer the map's
    rig.pi.publishEstimate(records);
    rig.run(1.5);
    EXPECT_EQ(rig.client().field().generation, generation);

    rig.pi.publishNominalEstimate();
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().field().generation != generation; }, kLimit));
    expectConsistent(rig.client().field());
}

TEST(FieldSync, ReplacedEstimateIsStaleAndTheNewestIsReadInstead) {
    LinkRig rig;
    rig.pi.setField(makeFakeField(60)); // estimate spans 13 chunks
    ASSERT_TRUE(published(rig));
    const uint32_t first = rig.client().field().estimate_id;

    // A new estimate; while it is read, three more replace it.
    const uint32_t target = rig.pi.publishNominalEstimate();
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().fieldSync().estimate_reading == target; }, kLimit));
    rig.pi.publishNominalEstimate();
    rig.pi.publishNominalEstimate();
    const uint32_t newest = rig.pi.publishNominalEstimate();
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().field().estimate_id == newest; }, kLimit));
    EXPECT_GE(rig.client().stats().doc_stale, 1u);
    EXPECT_GT(newest, first);
    expectConsistent(rig.client().field());
}

TEST(FieldSync, StaleThreeTimesInARowBacksOff) {
    ClientConfig config;
    config.transfer_backoff = 1.0;
    LinkRig rig(config);
    rig.pi.setField(makeFakeField(10));
    ASSERT_TRUE(published(rig));
    // Every chunk served is followed by three replacements: the next chunk
    // of that estimate is always Stale.
    rig.pi.doc_hook = [&](gatr2::BrainReply& r) {
        if (r.doc_kind == gatr2::kDocFieldEstimate) {
            rig.pi.publishNominalEstimate();
            rig.pi.publishNominalEstimate();
            rig.pi.publishNominalEstimate();
        }
    };
    rig.pi.publishNominalEstimate();
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().doc_stale == 3; }, kLimit));
    const std::size_t reads_then = reads(rig.bus, gatr2::kDocFieldEstimate).size();
    rig.run(0.9);
    EXPECT_EQ(reads(rig.bus, gatr2::kDocFieldEstimate).size(), reads_then);
    rig.pi.doc_hook = nullptr;
    const uint32_t newest = rig.pi.publishNominalEstimate();
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().field().estimate_id == newest; }, kLimit));
}

TEST(FieldSync, EstimatesRefreshAtFieldPeriod) {
    LinkRig rig;
    rig.pi.setField(makeFakeField(4));
    ASSERT_TRUE(published(rig));
    // A new estimate every 50 ms; the Brain reads one per field_period.
    const uint32_t before = rig.client().stats().estimates;
    Seconds        next   = rig.now();
    const Seconds  end    = rig.now() + 2.0;
    while (rig.now() < end) {
        if (rig.now() >= next) {
            rig.pi.publishNominalEstimate();
            next = rig.now() + 0.05;
        }
        rig.step();
    }
    const uint32_t read = rig.client().stats().estimates - before;
    EXPECT_GE(read, 3u);
    EXPECT_LE(read, 5u);
}

TEST(FieldSync, MapCacheSurvivesSessionsAndPiRestarts) {
    LinkRig rig;
    rig.pi.setField(makeFakeField(30));
    ASSERT_TRUE(published(rig));
    const std::size_t map_reads = reads(rig.bus, gatr2::kDocFieldMap).size();
    const uint32_t    gen       = rig.client().field().generation;

    rig.rebootBrain(); // a new Brain boot starts empty
    ASSERT_TRUE(published(rig));
    const std::size_t after_boot = reads(rig.bus, gatr2::kDocFieldMap).size();
    EXPECT_EQ(after_boot, 2 * map_reads);

    rig.pi.restart(0xABCD0001); // same map, new instance: only a new estimate
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().pi_restarts == 1; }, kLimit));
    const uint32_t gen_before = rig.client().field().generation;
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().field().generation != gen_before; }, kLimit));
    EXPECT_EQ(reads(rig.bus, gatr2::kDocFieldMap).size(), after_boot);
    EXPECT_EQ(rig.client().field().pi_instance, 0xABCD0001u);
    EXPECT_EQ(rig.client().field().estimate_id, 1u);
    expectConsistent(rig.client().field());
    (void)gen;
}

TEST(FieldSync, NewMapIsReadThenItsEstimateAndPublishedTogether) {
    LinkRig rig;
    rig.pi.setField(makeFakeField(8, 1));
    ASSERT_TRUE(published(rig));
    const uint32_t old_map = rig.client().field().map_id;
    const uint32_t gen     = rig.client().field().generation;

    rig.pi.setField(makeFakeField(12, 2));
    const Seconds end = rig.now() + kLimit;
    while (rig.client().field().map_id == old_map && rig.now() < end) {
        rig.step();
        const FieldPublication& p = rig.client().field();
        expectConsistent(p);
        EXPECT_TRUE(p.map_id == old_map || p.map_id == rig.pi.mapId());
    }
    EXPECT_EQ(rig.client().field().map_id, rig.pi.mapId());
    EXPECT_GT(rig.client().field().generation, gen);
    EXPECT_EQ(rig.client().stats().maps, 2u);
    expectConsistent(rig.client().field());
}

TEST(FieldSync, InterruptedTransfersRestartInTheNewSession) {
    LinkRig rig;
    rig.pi.setField(makeFakeField(100));
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().fieldSync().map_received >= 3 * gatr2::kDocChunkMax; },
        kLimit));
    rig.pi.restart(0x77770001); // mid map
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().pi_restarts == 1; }, kLimit));
    EXPECT_FALSE(rig.client().fieldSync().map_reading);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().fieldSync().estimate_reading != 0; }, 3 * kLimit));
    rig.pi.restart(0x77770002); // mid estimate
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().pi_restarts == 2; }, kLimit));
    ASSERT_TRUE(published(rig));
    EXPECT_EQ(rig.client().field().pi_instance, 0x77770002u);
    expectConsistent(rig.client().field());
    EXPECT_EQ(rig.client().stats().maps, 1u);
}

TEST(FieldSync, UnavailableWaitsAFieldPeriod) {
    LinkRig rig;
    rig.pi.setField(makeFakeField(3));
    ASSERT_TRUE(published(rig));
    // The Pi drops its estimates but still names one in the state block.
    rig.pi.doc_hook = [](gatr2::BrainReply& r) {
        if (r.doc_kind == gatr2::kDocFieldEstimate) {
            r.result = gatr2::kResultUnavailable;
        }
    };
    rig.pi.publishNominalEstimate();
    rig.run(1.0);
    const std::size_t n = reads(rig.bus, gatr2::kDocFieldEstimate).size();
    EXPECT_LE(n, 1u + 3u); // one plus at most one per field_period
}
