// zip_writer.h
// Store-only ZIP archives (no compression, no ZIP64, no data descriptors),
// so any unzip tool and the viewer's few-line reader can open them. CRC-32
// is translagatr::crc32 (the zlib CRC).

#pragma once
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace navigatr
{

struct ZipEntry {
    ZipEntry(std::string n, std::string d, std::vector<std::string> c = {})
        : name(std::move(n)), data(std::move(d)), chunks(std::move(c)) {}

    std::string name; // ASCII path inside the archive
    std::string data;
    // Appended after data, in order: text built in pieces goes into the
    // archive without first being joined into one more copy.
    std::vector<std::string> chunks;
};

// DOS date and time for the entries; 1980-01-01 00:00 when unix_ms < 0.
struct ZipTime {
    uint16_t time = 0;
    uint16_t date = (1u << 5) | 1u;
};
ZipTime zipTimeFromUnixMs(int64_t unix_ms);

// The archive bytes, or false with err when an entry or the whole archive
// exceeds the classic ZIP limits (4 GiB, 65535 entries).
bool buildStoredZip(const std::vector<ZipEntry>& entries, ZipTime when, std::string& out,
                    std::string& err);

} // namespace navigatr
