"""e5: main.cpp becomes parse, build, run, report (F11)."""
import sys

p = sys.argv[1]
s = open(p, newline='').read()
head_end = s.index('#include <chrono>')
head = s[:head_end]
assert head.startswith('// main.cpp\n// Register the compiled factories, build the configured system, run. A\n')
head = head.replace(
    '// main.cpp\n// Register the compiled factories, build the configured system, run. A\n',
    '// main.cpp\n// Parse, build, run, report: register the compiled factories, build the\n'
    '// configured system, run it (core/run.h), print (cli/report.h). A\n', 1)

body = '''#include <csignal>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

#include "cli/command_line.h"
#include "cli/report.h"
#include "core/run.h"
#include "core/system.h"
#include "debug/inspection/inspection_service.h"
#include "navigatr_build_config.h"
#include "pipeline/register_all.h"

namespace
{

using InspectionServicePtr = std::unique_ptr<navigatr::InspectionService>;

navigatr::StopFlag g_stop = 0;

void onSignal(int) { g_stop = 1; }

// Started when the profile or --inspect-port enables it, else left null.
bool startInspection(navigatr::System& system, long inspect_port, InspectionServicePtr& service,
                     std::string& err) {
    navigatr::InspectionConfig inspection = system.inspection();
    if (inspect_port != 0) {
        inspection.enabled = true;
        inspection.port    = inspect_port;
    }
    if (!inspection.enabled) {
        return true;
    }
    service = navigatr::InspectionService::create(system, inspection, err);
    return service != nullptr && service->start(err);
}

// Browsers first, then the capture recorder (a capture still recording is
// closed as truncated), then the workers, then the system.
void stopInOrder(navigatr::System& system, navigatr::InspectionService* service) {
    if (service != nullptr) {
        service->stop();
    }
    if (system.capture() != nullptr) {
        system.capture()->stop();
    }
    system.stop();
}

} // namespace

int main(int argc, char** argv) {
    navigatr::CommandLine command;
    std::string           err;
    if (!navigatr::parseCommandLine(argc, argv, navigatr::kDefaultConfigFile, command, err)) {
        std::fprintf(stderr, "%s\\nRun navigatr --help for usage.\\n", err.c_str());
        return 2;
    }
    if (command.help) {
        navigatr::printUsage(navigatr::kDefaultConfigFile);
        return 0;
    }

    navigatr::FunctionRegistry functions;
    navigatr::registerAll(functions);
    std::fprintf(stderr, "config %s\\n", command.config_path.c_str());
    std::unique_ptr<navigatr::System> system =
        navigatr::System::buildFromFile(command.config_path, functions, err, command.build);
    if (system == nullptr) {
        std::fprintf(stderr, "config error: %s\\n", err.c_str());
        return 1;
    }
    navigatr::printStartup(*system, command.inline_mode);

    InspectionServicePtr service;
    if (!startInspection(*system, command.inspect_port, service, err)) {
        std::fprintf(stderr, "inspection error: %s\\n", err.c_str());
        return 1;
    }
    if (service != nullptr) {
        navigatr::printInspection(*service);
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    navigatr::EventSequence       events_seen  = 0;
    const navigatr::CycleCallback print_events = [&] {
        navigatr::printNewEvents(*system, events_seen);
    };
    if (command.inline_mode) {
        navigatr::runInline(*system, command.max_cycles, g_stop, print_events);
    } else if (!navigatr::runThreaded(*system, command.max_cycles, g_stop, print_events, err)) {
        std::fprintf(stderr, "start error: %s\\n", err.c_str());
        return 1;
    }

    stopInOrder(*system, service.get());
    navigatr::printSummary(*system);
    return 0;
}
'''
open(p, 'w', newline='').write(head + body)
