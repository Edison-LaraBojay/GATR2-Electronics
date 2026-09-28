// doc_assembly_gtest.cpp
// Chunk assembly rules: one kind and doc_id, constant total_len and crc32,
// contiguous offsets inside total_len and the capacity, crc32 checked at the
// end, and nothing readable before a complete, checked document.

#include "communigatr/doc_assembly.h"

#include <gtest/gtest.h>
#include <vector>

#include "common/link_documents.h"

using namespace communigatr;

namespace
{

constexpr uint32_t kDoc = 0xD0C0FFEE;

std::vector<uint8_t> document(std::size_t len) {
    std::vector<uint8_t> doc(len);
    for (std::size_t i = 0; i < len; ++i) {
        doc[i] = static_cast<uint8_t>(i * 7 + 3);
    }
    return doc;
}

// READ_DOC Ok reply carrying doc[offset, offset + n).
gatr2::BrainReply chunk(const std::vector<uint8_t>& doc, uint16_t offset, uint8_t n,
                        uint8_t kind = gatr2::kDocFieldMap, uint32_t id = kDoc) {
    gatr2::BrainReply r;
    r.op            = gatr2::kOpReadDoc;
    r.doc_kind      = kind;
    r.doc_id        = id;
    r.doc_total_len = static_cast<uint16_t>(doc.size());
    r.doc_crc32     = gatr2::crc32(doc.data(), static_cast<uint32_t>(doc.size()));
    r.doc_offset    = offset;
    r.data_len      = n;
    std::copy(doc.begin() + offset, doc.begin() + offset + n, r.data);
    return r;
}

// Feeds the whole document in maxLen() chunks. Returns the last step.
DocAssembly::Step feed(DocAssembly& a, const std::vector<uint8_t>& doc) {
    DocAssembly::Step step = DocAssembly::Step::kMore;
    while (step == DocAssembly::Step::kMore) {
        const uint16_t offset = a.offset();
        const std::size_t left = doc.size() - offset;
        const uint8_t  n = static_cast<uint8_t>(std::min<std::size_t>(a.maxLen(), left));
        step = a.accept(chunk(doc, offset, n));
    }
    return step;
}

} // namespace

TEST(DocAssembly, AssemblesDocumentsOfEverySizeUpToCapacity) {
    for (std::size_t len : {1u, 24u, 95u, 96u, 97u, 192u, 1000u, 3608u}) {
        DocAssembly a(3608);
        a.begin(gatr2::kDocFieldMap, kDoc);
        EXPECT_TRUE(a.active());
        EXPECT_EQ(a.data(), nullptr);
        const std::vector<uint8_t> doc = document(len);
        ASSERT_EQ(feed(a, doc), DocAssembly::Step::kComplete) << len;
        EXPECT_FALSE(a.active());
        EXPECT_TRUE(a.complete());
        ASSERT_EQ(a.length(), len);
        EXPECT_TRUE(std::equal(doc.begin(), doc.end(), a.data()));
        EXPECT_EQ(a.crc(), gatr2::crc32(doc.data(), static_cast<uint32_t>(len)));
    }
}

TEST(DocAssembly, MaxLenIsAChunkUntilTheTail) {
    DocAssembly                a(1000);
    const std::vector<uint8_t> doc = document(200);
    a.begin(gatr2::kDocFieldEstimate, 7);
    EXPECT_EQ(a.maxLen(), gatr2::kDocChunkMax);
    ASSERT_EQ(a.accept(chunk(doc, 0, 96, gatr2::kDocFieldEstimate, 7)), DocAssembly::Step::kMore);
    ASSERT_EQ(a.accept(chunk(doc, 96, 96, gatr2::kDocFieldEstimate, 7)), DocAssembly::Step::kMore);
    EXPECT_EQ(a.maxLen(), 8);
    EXPECT_EQ(a.accept(chunk(doc, 192, 8, gatr2::kDocFieldEstimate, 7)),
              DocAssembly::Step::kComplete);
}

TEST(DocAssembly, InconsistentChunksEndTheAssembly) {
    const std::vector<uint8_t> doc = document(300);
    struct Case {
        const char*                            name;
        std::function<void(gatr2::BrainReply&)> edit;
    };
    const std::vector<Case> cases = {
        {"kind", [](gatr2::BrainReply& r) { r.doc_kind = gatr2::kDocFieldEstimate; }},
        {"doc id", [](gatr2::BrainReply& r) { r.doc_id += 1; }},
        {"offset gap", [](gatr2::BrainReply& r) { r.doc_offset += 1; }},
        {"total_len", [](gatr2::BrainReply& r) { r.doc_total_len += 1; }},
        {"crc", [](gatr2::BrainReply& r) { r.doc_crc32 ^= 1; }},
        {"empty data", [](gatr2::BrainReply& r) { r.data_len = 0; }},
        {"past total", [](gatr2::BrainReply& r) { r.doc_total_len = r.doc_offset + 10; }},
    };
    for (const Case& c : cases) {
        DocAssembly a(1000);
        a.begin(gatr2::kDocFieldMap, kDoc);
        ASSERT_EQ(a.accept(chunk(doc, 0, 96)), DocAssembly::Step::kMore);
        gatr2::BrainReply second = chunk(doc, 96, 96);
        c.edit(second);
        EXPECT_EQ(a.accept(second), DocAssembly::Step::kInvalid) << c.name;
        EXPECT_FALSE(a.active()) << c.name;
        EXPECT_FALSE(a.complete()) << c.name;
        EXPECT_EQ(a.data(), nullptr) << c.name;
        EXPECT_EQ(a.accept(chunk(doc, 192, 96)), DocAssembly::Step::kInvalid) << c.name;
    }
}

TEST(DocAssembly, FirstChunkBoundsAndWrongStart) {
    const std::vector<uint8_t> doc = document(300);
    DocAssembly                small(200);
    small.begin(gatr2::kDocFieldMap, kDoc);
    EXPECT_EQ(small.accept(chunk(doc, 0, 96)), DocAssembly::Step::kInvalid); // over capacity

    DocAssembly a(1000);
    a.begin(gatr2::kDocFieldMap, kDoc);
    EXPECT_EQ(a.accept(chunk(doc, 96, 96)), DocAssembly::Step::kInvalid); // not from 0

    gatr2::BrainReply zero = chunk(doc, 0, 96);
    zero.doc_total_len     = 0;
    a.begin(gatr2::kDocFieldMap, kDoc);
    EXPECT_EQ(a.accept(zero), DocAssembly::Step::kInvalid);

    DocAssembly idle(1000);
    EXPECT_EQ(idle.accept(chunk(doc, 0, 96)), DocAssembly::Step::kInvalid); // never begun
}

TEST(DocAssembly, CorruptBytesFailTheCrcAtTheEnd) {
    const std::vector<uint8_t> doc = document(150);
    DocAssembly                a(1000);
    a.begin(gatr2::kDocFieldMap, kDoc);
    gatr2::BrainReply first = chunk(doc, 0, 96);
    first.data[10] ^= 0x40; // crc field still claims the true document
    ASSERT_EQ(a.accept(first), DocAssembly::Step::kMore);
    EXPECT_EQ(a.accept(chunk(doc, 96, 54)), DocAssembly::Step::kInvalid);
    EXPECT_EQ(a.data(), nullptr);
}

TEST(DocAssembly, BeginRestartsAndClearHides) {
    const std::vector<uint8_t> doc = document(120);
    DocAssembly                a(1000);
    a.begin(gatr2::kDocFieldMap, kDoc);
    ASSERT_EQ(feed(a, doc), DocAssembly::Step::kComplete);
    a.begin(gatr2::kDocFieldMap, kDoc);
    EXPECT_EQ(a.offset(), 0);
    EXPECT_EQ(a.data(), nullptr);
    ASSERT_EQ(feed(a, doc), DocAssembly::Step::kComplete);
    a.clear();
    EXPECT_FALSE(a.complete());
    EXPECT_EQ(a.length(), 0);
}
