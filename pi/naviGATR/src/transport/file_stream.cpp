// file_stream.cpp

#include "transport/file_stream.h"

namespace navigatr
{

FileByteSource::~FileByteSource() {
    if (f_ != nullptr) {
        std::fclose(f_);
    }
}

bool FileByteSource::open(const std::string& path, std::string& err) {
    f_ = std::fopen(path.c_str(), "rb");
    if (f_ == nullptr) {
        err = "cannot open " + path;
        return false;
    }
    return true;
}

int FileByteSource::read(uint8_t* dst, int cap) {
    if (f_ == nullptr) {
        return -1;
    }
    const int want = cap < chunk_ ? cap : chunk_;
    if (want <= 0) {
        return 0;
    }
    return static_cast<int>(std::fread(dst, 1, static_cast<size_t>(want), f_));
}

} // namespace navigatr
