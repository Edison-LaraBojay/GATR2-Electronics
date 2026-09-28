// file_stream.h
// Raw frame bytes from disk. Replays a capture through the real parsers.

#pragma once
#include <cstdio>
#include <string>

#include "transport/byte_stream.h"

namespace navigatr
{

class FileByteSource : public ByteSource
{
public:
    ~FileByteSource() override;

    bool open(const std::string& path, std::string& err);
    bool isOpen() const { return f_ != nullptr; }

    // Reads at most chunk_ bytes per call so a replay feeds the pipeline in
    // serial-sized bites instead of one giant read.
    int read(uint8_t* dst, int cap) override;

private:
    std::FILE* f_     = nullptr;
    int        chunk_ = 64;
};

} // namespace navigatr
