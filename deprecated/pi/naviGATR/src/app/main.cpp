// main.cpp
// Register the compiled factories, build the configured system, run. A
// device that fails to open is a build warning and a runtime status; a
// configuration error exits before anything starts.
//
//   navigatr [--config_file <config.xml>] [--replay <resource_id>=<capture.bin>] [--cycles <n>]
//            [--inline] [--inspect-port <port>]
//
// Default run mode starts the estimation and field workers and, when the
// configuration enables it, the inspection service; SIGINT/SIGTERM stop the
// service first, then the capture recorder, then the workers, then the
// system. --inline runs every
// stage on this thread at the loop rate instead (deterministic replay and
// bench use; warned with the brain link, which needs the workers). --cycles
// stops after n estimation cycles in either mode. --inspect-port enables
// the inspection service on loopback at that port without editing the
// profile.

#include <chrono>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "app/command_line.h"
#include "navigatr_build_config.h"
#include "core/host_clock.h"
#include "inspection/inspection_service.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

namespace
{

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

void printDiagnostics(const char* title, const navigatr::Diagnostics& d) {
    std::printf("%s: cycles %llu\n", title, static_cast<unsigned long long>(d.cycles));
    for (const auto& kv : d.links) {
        const navigatr::LinkStats& l = kv.second;
        std::printf("link %-14s %u bytes, %u packets, %u decode errors, %u seq gaps\n",
                    kv.first.c_str(), l.bytes, l.packets, l.decode_errors, l.seq_gaps);
        if (l.requests != 0) {
            std::printf("     %-14s %u requests, %u replies, %u duplicates, %u superseded, "
                        "%u stale, %u unknown session, %u unanswered, %u expired, "
                        "%u input pending, %u late release, %u tx errors\n",
                        "", l.requests, l.replies, l.duplicates, l.superseded, l.stale,
                        l.unknown_session, l.unanswered, l.expired, l.input_pending,
                        l.late_release, l.tx_errors);
        }
    }
    for (const auto& kv : d.functions) {
        const navigatr::FunctionStats& f = kv.second;
        std::printf("%-50s runs %-8u ok %-8u no_data %-8u fault %-6u\n", kv.first.c_str(),
                    f.runs, f.ok, f.no_data, f.fault);
    }
}

void printWorker(const navigatr::WorkerStatsSnapshot& s) {
    std::printf("worker %-11s cycles %-8llu rate %6.1f Hz  mean %6.2f ms  max %7.2f ms  "
                "overruns %llu  dropped %llu\n",
                s.name.c_str(), static_cast<unsigned long long>(s.cycles), s.rate_hz,
                s.mean_cycle_ms, s.max_cycle_ms, static_cast<unsigned long long>(s.overruns),
                static_cast<unsigned long long>(s.dropped));
}

} // namespace

int main(int argc, char** argv) {
    navigatr::CommandLine command;
    std::string err;
    if (!navigatr::parseCommandLine(argc, argv, navigatr::kDefaultConfigFile, command, err)) {
        std::fprintf(stderr, "%s\nRun navigatr --help for usage.\n", err.c_str());
        return 2;
    }
    if (command.help) {
        std::printf("usage: navigatr [--config_file <config.xml>] "
                    "[--replay <resource_id>=<capture.bin>] [--cycles <n>] "
                    "[--inline] [--inspect-port <port>]\n\n"
                    "--config_file=<path>, --config-file, and a positional filename are accepted.\n"
                    "Without a filename, the compiled default is:\n  %s\n"
                    "Set it at build time with -DNAVIGATR_DEFAULT_CONFIG=<path>.\n",
                    navigatr::kDefaultConfigFile);
        return 0;
    }
    const long max_cycles = command.max_cycles;
    const bool inline_mode = command.inline_mode;
    const long inspect_port = command.inspect_port;

    navigatr::FunctionRegistry functions;
    navigatr::registerAll(functions);

    std::fprintf(stderr, "config %s\n", command.config_path.c_str());
    std::unique_ptr<navigatr::System> system =
        navigatr::System::buildFromFile(command.config_path, functions, err, command.build);
    if (system == nullptr) {
        std::fprintf(stderr, "config error: %s\n", err.c_str());
        return 1;
    }
    std::fprintf(stderr, "profile %s digest %016llx session %s\n",
                 system->configurationId().c_str(),
                 static_cast<unsigned long long>(system->configurationDigest()),
                 system->sessionId().c_str());
    for (const std::string& warning : system->warnings()) {
        std::fprintf(stderr, "warning: %s\n", warning.c_str());
    }
    const navigatr::CaptureConfig& capture = system->captureConfig();
    if (capture.enabled) {
        std::fprintf(stderr, "capture on: rolling %.0f s, keep %ld, %s%s\n", capture.rolling_s,
                     capture.keep, capture.directory.empty() ? "memory only" : "files in ",
                     capture.directory.c_str());
    } else {
        std::fprintf(stderr, "capture off\n");
    }
    if (inline_mode && system->commandsType() == "brain_link") {
        std::fprintf(stderr, "warning: --inline runs world estimation between a brain_link "
                             "request and its reply; replies may miss the reply window\n");
    }

    navigatr::InspectionConfig inspection = system->inspection();
    if (inspect_port != 0) {
        inspection.enabled = true;
        inspection.port    = inspect_port;
    }
    std::unique_ptr<navigatr::InspectionService> service;
    if (inspection.enabled) {
        service = navigatr::InspectionService::create(*system, inspection, err);
        if (service == nullptr || !service->start(err)) {
            std::fprintf(stderr, "inspection error: %s\n", err.c_str());
            return 1;
        }
        std::fprintf(stderr, "inspection at %s (forward with: ssh -N -L %d:127.0.0.1:%d "
                             "<user>@<pi-host>)\n",
                     service->url().c_str(), service->port(), service->port());
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // lifecycle events (profile applies and refusals, controls) as log lines
    uint64_t   events_seen = 0;
    const auto printEvents = [&] {
        for (const navigatr::RuntimeEvent& e : system->events()) {
            if (e.sequence > events_seen) {
                std::fprintf(stderr, "event: %s\n", e.text.c_str());
                events_seen = e.sequence;
            }
        }
    };

    if (inline_mode) {
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / system->loopRateHz()));
        auto next   = std::chrono::steady_clock::now();
        long cycles = 0;
        while (g_stop == 0 && (max_cycles < 0 || cycles < max_cycles)) {
            system->step(navigatr::HostClock::now());
            printEvents();
            ++cycles;
            next += period;
            std::this_thread::sleep_until(next);
        }
    } else {
        if (!system->start(err)) {
            std::fprintf(stderr, "start error: %s\n", err.c_str());
            return 1;
        }
        while (g_stop == 0 &&
               (max_cycles < 0 || system->cycle() < static_cast<uint64_t>(max_cycles))) {
            // the Brain profile boundary belongs to this thread
            system->applyPendingProfile();
            printEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    // orderly: browsers first, then the capture recorder (a capture still
    // recording is closed as truncated), then the workers, then the system
    if (service != nullptr) {
        service->stop();
    }
    if (system->capture() != nullptr) {
        system->capture()->stop();
    }
    system->stop();

    printWorker(system->estimationStats());
    printWorker(system->fieldStats());
    printDiagnostics("estimation", system->diagnostics());
    printDiagnostics("field", system->fieldDiagnostics());
    return 0;
}
