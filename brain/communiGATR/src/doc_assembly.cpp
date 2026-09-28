// doc_assembly.cpp

#include "communigatr/doc_assembly.h"

#include <cstring>

#include "common/link_documents.h"

namespace communigatr
{

DocAssembly::DocAssembly(uint16_t capacity) : buffer_(capacity), capacity_(capacity) {}

void DocAssembly::begin(uint8_t kind, uint32_t doc_id) {
    clear();
    active_ = true;
    kind_   = kind;
    doc_id_ = doc_id;
}

void DocAssembly::clear() {
    active_    = false;
    complete_  = false;
    kind_      = 0;
    doc_id_    = 0;
    total_len_ = 0;
    crc_       = 0;
    offset_    = 0;
}

uint8_t DocAssembly::maxLen() const {
    if (total_len_ == 0 || total_len_ - offset_ >= gatr2::kDocChunkMax) {
        return gatr2::kDocChunkMax;
    }
    return static_cast<uint8_t>(total_len_ - offset_);
}

DocAssembly::Step DocAssembly::accept(const gatr2::BrainReply& reply) {
    if (!active_ || complete_) {
        return invalid();
    }
    if (reply.doc_kind != kind_ || reply.doc_id != doc_id_ || reply.doc_offset != offset_ ||
        reply.data_len == 0) {
        return invalid();
    }
    if (total_len_ == 0) {
        if (reply.doc_total_len == 0 || reply.doc_total_len > capacity_) {
            return invalid();
        }
        total_len_ = reply.doc_total_len;
        crc_       = reply.doc_crc32;
    } else if (reply.doc_total_len != total_len_ || reply.doc_crc32 != crc_) {
        return invalid();
    }
    if (static_cast<uint32_t>(offset_) + reply.data_len > total_len_) {
        return invalid();
    }
    std::memcpy(buffer_.data() + offset_, reply.data, reply.data_len);
    offset_ = static_cast<uint16_t>(offset_ + reply.data_len);
    if (offset_ < total_len_) {
        return Step::kMore;
    }
    if (gatr2::crc32(buffer_.data(), total_len_) != crc_) {
        return invalid();
    }
    complete_ = true;
    active_   = false;
    return Step::kComplete;
}

DocAssembly::Step DocAssembly::invalid() {
    clear();
    return Step::kInvalid;
}

} // namespace communigatr
