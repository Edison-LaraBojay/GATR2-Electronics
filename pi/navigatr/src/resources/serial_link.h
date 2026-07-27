// serial_link.h
// Typed contract for a byte link. There is deliberately no universal DataBus
// abstraction; different hardware concepts get different contracts.
//
// Thread safety: SerialLink implementations are single-threaded unless their
// own documentation says otherwise. A shared_ptr does not make hardware safe.

#pragma once
#include <cstddef>
#include <cstdint>

namespace navigatr
{

struct ByteSpan {
    const uint8_t* data = nullptr;
    std::size_t    size = 0;
};

struct MutableByteSpan {
    uint8_t*    data = nullptr;
    std::size_t size = 0;
};

struct SerialReadResult {
    std::size_t bytes  = 0;
    bool        closed = false;   // link is dead, not merely idle
};

struct SerialWriteResult {
    bool ok = false;
};

class SerialLink
{
public:
    virtual ~SerialLink() = default;

    // Nonblocking. Copies what is available, up to destination.size.
    virtual SerialReadResult readAvailable(MutableByteSpan destination) = 0;

    virtual SerialWriteResult write(ByteSpan source) = 0;
};

} // namespace navigatr
