// zip_writer.cpp

#include "capture/zip_writer.h"

#include <ctime>
#include <limits>

#include "translaGATR/link_documents.h"

namespace navigatr
{
namespace
{

void put16(std::string& s, uint32_t v) {
    s += static_cast<char>(v & 0xFF);
    s += static_cast<char>((v >> 8) & 0xFF);
}

void put32(std::string& s, uint32_t v) {
    put16(s, v & 0xFFFF);
    put16(s, (v >> 16) & 0xFFFF);
}

void patch32(std::string& s, std::size_t at, uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        s[at + static_cast<std::size_t>(i)] = static_cast<char>((v >> (8 * i)) & 0xFF);
    }
}

uint64_t entrySize(const ZipEntry& e) {
    uint64_t n = e.data.size();
    for (const std::string& c : e.chunks) {
        n += c.size();
    }
    return n;
}

} // namespace

ZipTime zipTimeFromUnixMs(int64_t unix_ms) {
    ZipTime t;
    if (unix_ms < 0) {
        return t;
    }
    const std::time_t secs = static_cast<std::time_t>(unix_ms / 1000);
    std::tm           tm{};
#if defined(_WIN32)
    if (gmtime_s(&tm, &secs) != 0) {
        return t;
    }
#else
    if (gmtime_r(&secs, &tm) == nullptr) {
        return t;
    }
#endif
    const int year = tm.tm_year + 1900;
    if (year < 1980 || year > 2107) {
        return t;
    }
    t.time = static_cast<uint16_t>((tm.tm_hour << 11) | (tm.tm_min << 5) | (tm.tm_sec / 2));
    t.date = static_cast<uint16_t>(((year - 1980) << 9) | ((tm.tm_mon + 1) << 5) | tm.tm_mday);
    return t;
}

bool buildStoredZip(const std::vector<ZipEntry>& entries, ZipTime when, std::string& out,
                    std::string& err) {
    constexpr uint64_t kMax = std::numeric_limits<uint32_t>::max();
    if (entries.size() > 0xFFFF) {
        err = "too many zip entries";
        return false;
    }
    uint64_t total = 22;
    for (const ZipEntry& e : entries) {
        if (e.name.empty() || e.name.size() > 0xFFFF || entrySize(e) > kMax) {
            err = "zip entry too large or unnamed: " + e.name;
            return false;
        }
        total += 30 + 46 + 2 * e.name.size() + entrySize(e);
    }
    if (total > kMax) {
        err = "zip archive over 4 GiB";
        return false;
    }
    out.clear();
    out.reserve(static_cast<std::size_t>(total));
    std::string central;
    for (const ZipEntry& e : entries) {
        const uint32_t size   = static_cast<uint32_t>(entrySize(e));
        const uint32_t offset = static_cast<uint32_t>(out.size());

        put32(out, 0x04034b50); // local file header
        put16(out, 10);         // version needed: 1.0, stored
        put16(out, 0);          // flags
        put16(out, 0);          // method: stored
        put16(out, when.time);
        put16(out, when.date);
        put32(out, 0);    // CRC, patched once the data is in place
        put32(out, size); // compressed
        put32(out, size); // uncompressed
        put16(out, static_cast<uint32_t>(e.name.size()));
        put16(out, 0); // extra
        out += e.name;
        // the data is contiguous here, so one CRC pass covers the chunks
        const std::size_t data_at = out.size();
        out += e.data;
        for (const std::string& c : e.chunks) {
            out += c;
        }
        const uint32_t crc =
            translagatr::crc32(reinterpret_cast<const uint8_t*>(out.data() + data_at), size);
        patch32(out, offset + 14, crc);

        put32(central, 0x02014b50); // central directory header
        put16(central, 20);         // made by: MS-DOS, 2.0
        put16(central, 10);
        put16(central, 0);
        put16(central, 0);
        put16(central, when.time);
        put16(central, when.date);
        put32(central, crc);
        put32(central, size);
        put32(central, size);
        put16(central, static_cast<uint32_t>(e.name.size()));
        put16(central, 0); // extra
        put16(central, 0); // comment
        put16(central, 0); // disk
        put16(central, 0); // internal attributes
        put32(central, 0); // external attributes
        put32(central, offset);
        central += e.name;
    }
    const uint32_t cd_offset = static_cast<uint32_t>(out.size());
    out += central;
    put32(out, 0x06054b50); // end of central directory
    put16(out, 0);
    put16(out, 0);
    put16(out, static_cast<uint32_t>(entries.size()));
    put16(out, static_cast<uint32_t>(entries.size()));
    put32(out, static_cast<uint32_t>(central.size()));
    put32(out, cd_offset);
    put16(out, 0); // comment
    return true;
}

} // namespace navigatr
