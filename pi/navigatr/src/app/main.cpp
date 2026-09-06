// main.cpp
// Register the compiled factories, build the configured system, loop. A
// device that fails to open is a build warning and a runtime status; a
// configuration error exits before the loop starts.
//
//   navigatr <config.xml> [--replay <resource_id>=<capture.bin>] [--cycles <n>]

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "runtime/register_all.h"
#include "runtime/system.h"

namespace
{

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

void printDiagnostics(const navigatr::Diagnostics& d) {
    std::printf("cycles %llu\n", static_cast<unsigned long long>(d.cycles));
    for (const auto& kv : d.links) {
        std::printf("link %-14s %u bytes, %u packets, %u decode errors, %u seq gaps\n",
                    kv.first.c_str(), kv.second.bytes, kv.second.packets,
                    kv.second.decode_errors, kv.second.seq_gaps);
    }
    for (const auto& kv : d.functions) {
        const navigatr::FunctionStats& f = kv.second;
        std::printf("%-50s runs %-8u ok %-8u no_data %-8u fault %-6u\n", kv.first.c_str(),
                    f.runs, f.ok, f.no_data, f.fault);
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string            config_path;
    navigatr::BuildOptions options;
    long                   max_cycles = -1;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--replay" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const std::size_t eq   = spec.find('=');
            if (eq == std::string::npos || eq == 0 || eq + 1 >= spec.size()) {
                std::fprintf(stderr, "--replay wants <resource_id>=<capture.bin>\n");
                return 2;
            }
            options.replay[spec.substr(0, eq)] = spec.substr(eq + 1);
        } else if (arg == "--cycles" && i + 1 < argc) {
            max_cycles = std::strtol(argv[++i], nullptr, 10);
        } else if (arg == "--allow-provisional") {
            options.allow_provisional = true;
        } else if (config_path.empty()) {
            config_path = arg;
        } else {
            std::fprintf(stderr, "unexpected argument %s\n", arg.c_str());
            return 2;
        }
    }
    if (config_path.empty()) {
        std::fprintf(stderr,
                     "usage: navigatr <config.xml> "
                     "[--replay <resource_id>=<capture.bin>] [--cycles <n>] "
                     "[--allow-provisional]\n");
        return 2;
    }

    navigatr::FunctionRegistry functions;
    navigatr::registerAll(functions);

    std::string                       err;
    std::unique_ptr<navigatr::System> system =
        navigatr::System::buildFromFile(config_path, functions, err, options);
    if (system == nullptr) {
        std::fprintf(stderr, "config error: %s\n", err.c_str());
        return 1;
    }
    std::fprintf(stderr, "profile %s digest %016llx\n",
                 system->configurationId().c_str(),
                 static_cast<unsigned long long>(system->configurationDigest()));
    for (const std::string& warning : system->warnings()) {
        std::fprintf(stderr, "warning: %s\n", warning.c_str());
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const auto start  = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / system->loopRateHz()));
    auto next = start;

    long cycles = 0;
    while (g_stop == 0 && (max_cycles < 0 || cycles < max_cycles)) {
        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - start)
                                .count();
        system->step(navigatr::hostTime(static_cast<int64_t>(now_ms) + 1));
        ++cycles;

        next += period;
        std::this_thread::sleep_until(next);
    }

    printDiagnostics(system->diagnostics());
    return 0;
}
