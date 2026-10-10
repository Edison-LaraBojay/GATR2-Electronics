// replay.cpp

#include "receiver/sources/replay.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <utility>

#include "receiver/platform/process.h"

namespace spectagatr::receiver::sources
{

namespace
{

struct FileCloser {
    void operator()(std::FILE* file) const {
        if (file != nullptr) {
            std::fclose(file);
        }
    }
};
using File = std::unique_ptr<std::FILE, FileCloser>;

// One open input: a file, or stdin.
class Input {
  public:
    static std::optional<Input> open(const std::string& path, std::string& err) {
        Input input;
        if (path == kStandardInputPath) {
            platform::useBinaryStandardInput();
            input.stdin_ = true;
            return input;
        }
        input.file_.reset(std::fopen(path.c_str(), "rb"));
        if (!input.file_) {
            err = "file: cannot open " + path + ": " + std::strerror(errno);
            return std::nullopt;
        }
        return input;
    }

    // Bytes read, 0 at the end, -1 on an error.
    long read(Byte* destination, std::size_t capacity) {
        if (stdin_) {
            return platform::readStandardInput(destination, capacity);
        }
        const std::size_t count = std::fread(destination, 1, capacity, file_.get());
        if (count > 0) {
            return static_cast<long>(count);
        }
        return std::ferror(file_.get()) != 0 ? -1 : 0;
    }

    bool rewindable() const { return !stdin_; }

  private:
    File file_;
    bool stdin_ = false;
};

void report(const SourceHooks& hooks, const ReplayConfig& config, PortState state,
            std::string detail, bool rewound = false) {
    SourceStatus status;
    status.src              = SourceKind::kFile;
    status.state            = state;
    status.at_us            = hooks.clock();
    status.port             = config.path == kStandardInputPath ? "stdin" : config.path;
    status.detail           = std::move(detail);
    status.stream_restarted = rewound;
    hooks.on_status(status);
}

// True at the end of the input, false on stop. any: some bytes came.
// Paced against elapsed time, not per wait: a wait can last much longer than
// asked (about 15.6 ms on Windows for a 5 ms wait), so each wake sends what
// the rate owes since the start, up to a bounded catch-up chunk.
bool replay(Input& input, const ReplayConfig& config, const SourceHooks& hooks,
            platform::StopSignal& stop, bool& any) {
    using Clock                   = std::chrono::steady_clock;
    const std::size_t chunk_bytes = config.chunk_bytes > 0 ? config.chunk_bytes : 1;
    const double interval_ms = static_cast<double>(std::max<long long>(config.interval.count(), 1));
    const double bytes_per_ms = static_cast<double>(chunk_bytes) / interval_ms;
    const std::size_t capacity =
        std::min<std::size_t>(std::max<std::size_t>(chunk_bytes * 16, 256), std::size_t(1) << 20);
    Bytes      buffer(std::max(capacity, chunk_bytes));
    const auto start = Clock::now();
    uint64_t   sent  = 0;
    while (!stop.requested()) {
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        const uint64_t owed = static_cast<uint64_t>(elapsed_ms * bytes_per_ms) + chunk_bytes;
        if (owed > sent) {
            const std::size_t want =
                static_cast<std::size_t>(std::min<uint64_t>(owed - sent, buffer.size()));
            const long count = input.read(buffer.data(), want);
            if (count <= 0) {
                return true;
            }
            any = true;
            sent += static_cast<uint64_t>(count);
            SourceChunk chunk;
            chunk.src   = SourceKind::kFile;
            chunk.rx_us = hooks.clock();
            chunk.bytes.assign(buffer.begin(), buffer.begin() + count);
            hooks.on_chunk(chunk);
            if (owed > sent) {
                continue; // still behind: no wait
            }
        }
        if (stop.waitFor(config.interval)) {
            return false;
        }
    }
    return false;
}

} // namespace

std::size_t replayChunkBytes(double bytes_per_second, Milliseconds interval) {
    const double bytes = bytes_per_second * static_cast<double>(interval.count()) / 1000.0;
    return bytes >= 1.0 ? static_cast<std::size_t>(std::floor(bytes)) : 1;
}

void runReplaySource(const ReplayConfig& config, const SourceHooks& hooks,
                     platform::StopSignal& stop) {
    const std::string name    = config.path == kStandardInputPath ? "stdin" : config.path;
    bool              rewound = false;
    while (!stop.requested()) {
        std::string          err;
        std::optional<Input> input = Input::open(config.path, err);
        if (!input) {
            report(hooks, config, PortState::kSearching, err);
            if (stop.waitFor(config.retry_wait)) {
                break;
            }
            continue;
        }
        report(hooks, config, PortState::kOpen, "file: replaying " + name, rewound);
        bool any = false;
        if (!replay(*input, config, hooks, stop, any)) {
            break;
        }
        if (config.loop && input->rewindable()) {
            rewound = true;
            if (!any && stop.waitFor(config.retry_wait)) {
                break; // an empty file: no busy loop
            }
            continue;
        }
        report(hooks, config, PortState::kSearching, "file: end of " + name);
        while (!stop.waitFor(Milliseconds(1000))) {
        }
    }
    report(hooks, config, PortState::kStopped, "");
}

} // namespace spectagatr::receiver::sources
