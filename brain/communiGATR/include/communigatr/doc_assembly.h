// doc_assembly.h
// One document read with READ_DOC chunks into a buffer allocated once.
// Every chunk must name the same kind and doc_id, repeat the total_len and
// crc32 of the first, start where the previous one ended and stay inside
// total_len; a complete document must match its crc32. Anything else ends
// the assembly, and a partial document is never readable.

#pragma once
#include <cstdint>
#include <vector>

#include "common/frame_codec.h"

namespace communigatr
{

class DocAssembly {
public:
    enum class Step : uint8_t {
        kMore,     // chunk taken, more to read
        kComplete, // whole document held, crc32 checked
        kInvalid,  // inconsistent chunk or crc32; assembly cleared
    };

    explicit DocAssembly(uint16_t capacity);

    // Starts reading doc_id of kind from offset 0.
    void begin(uint8_t kind, uint32_t doc_id);
    void clear();

    bool     active() const { return active_; }
    uint8_t  kind() const { return kind_; }
    uint32_t docId() const { return doc_id_; }

    // Next chunk to request.
    uint16_t offset() const { return offset_; }
    uint8_t  maxLen() const;

    // A READ_DOC Ok reply to the chunk request made from offset().
    Step accept(const gatr2::BrainReply& reply);

    // The document, only after kComplete and until begin() or clear().
    bool           complete() const { return complete_; }
    const uint8_t* data() const { return complete_ ? buffer_.data() : nullptr; }
    uint16_t       length() const { return complete_ ? total_len_ : 0; }
    uint32_t       crc() const { return crc_; }

private:
    Step invalid();

    std::vector<uint8_t> buffer_;
    uint16_t             capacity_  = 0;
    bool                 active_    = false;
    bool                 complete_  = false;
    uint8_t              kind_      = 0;
    uint32_t             doc_id_    = 0;
    uint16_t             total_len_ = 0; // 0 until the first chunk
    uint32_t             crc_       = 0;
    uint16_t             offset_    = 0;
};

} // namespace communigatr
