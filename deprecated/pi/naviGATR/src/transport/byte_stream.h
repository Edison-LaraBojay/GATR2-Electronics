// byte_stream.h
// The transport seam. Anything that yields bytes is a source, anything that
// takes bytes is a sink. Serial, replay files, and in-memory test streams
// all sit behind these two, so the pipeline never knows which one it has.

#pragma once
#include <cstdint>
#include <deque>
#include <vector>

namespace navigatr
{

class ByteSource
{
public:
    virtual ~ByteSource() = default;

    // Nonblocking. Bytes copied into dst, 0 when nothing is available,
    // negative when the source is dead.
    virtual int read(uint8_t* dst, int cap) = 0;
};

class ByteSink
{
public:
    virtual ~ByteSink() = default;

    virtual bool write(const uint8_t* src, int len) = 0;
};

// In-memory stream, test double and loopback. write appends, read consumes.
class MemoryStream : public ByteSource, public ByteSink
{
public:
    int read(uint8_t* dst, int cap) override {
        int n = 0;
        while (n < cap && !buf_.empty()) {
            dst[n++] = buf_.front();
            buf_.pop_front();
        }
        return n;
    }

    bool write(const uint8_t* src, int len) override {
        buf_.insert(buf_.end(), src, src + len);
        return true;
    }

    void feed(const std::vector<uint8_t>& bytes) {
        buf_.insert(buf_.end(), bytes.begin(), bytes.end());
    }

    std::vector<uint8_t> takeAll() {
        std::vector<uint8_t> out(buf_.begin(), buf_.end());
        buf_.clear();
        return out;
    }

    std::size_t size() const { return buf_.size(); }

private:
    std::deque<uint8_t> buf_;
};

} // namespace navigatr
