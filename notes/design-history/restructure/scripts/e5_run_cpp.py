"""e5: core/run.cpp calls the stages by their verbs and gains runInline/runThreaded."""
import sys

p = sys.argv[1]
s = open(p, newline='').read()


def rep(a, b):
    global s
    assert s.count(a) == 1, a
    s = s.replace(a, b)


rep('''// run.cpp

#include "core/system.h"

#include <array>
#include <chrono>
#include <stdexcept>

#include "time/host_clock.h"
''', '''// run.cpp
// The cycles, each stage called by its verb, the workers, and the two
// process run modes.

#include "core/run.h"

#include <array>
#include <chrono>
#include <stdexcept>
#include <thread>

#include "core/system.h"
#include "time/host_clock.h"
''')

rep('''    CommandsOutput commands_out = commands_->run({command_, now, &diagnostics_});
    command_                    = commands_out.command;
    diagnostics_.note(slot_labels_[0], commands_out.status);
''', '''    CommandsOutput commands_out = collect_commands_.run({command_, now, &diagnostics_});
    command_                    = commands_out.command;
    diagnostics_.note(collect_commands_.label, commands_out.status);
''')

rep('''    TargetResolutionOutput               target_out = target_resolution_->run(
        {command_, robot_, *feed_, field->field, field->observations, field->associations,
         target_, now});
    target_ = target_out.target;
    diagnostics_.note(slot_labels_[1], target_out.status);

    PublishingOutput pub_out = publishing_->run(
        {read_sensors_.retained(), field->observations, field->associations, robot_,
         status, field->field, command_, target_, now, &diagnostics_});
    diagnostics_.note(slot_labels_[2], pub_out.status);
''', '''    TargetResolutionOutput               target_out = resolve_targets_.run(
        {command_, robot_, *feed_, field->field, field->observations, field->associations,
         target_, now});
    target_ = target_out.target;
    diagnostics_.note(resolve_targets_.label, target_out.status);

    PublishingOutput pub_out = publish_.run(
        {read_sensors_.retained(), field->observations, field->associations, robot_,
         status, field->field, command_, target_, now, &diagnostics_});
    diagnostics_.note(publish_.label, pub_out.status);
''')

rep('''    commands_->reset();
    target_resolution_->reset();
    publishing_->reset();
''', '''    collect_commands_.reset();
    resolve_targets_.reset();
    publish_.reset();
''')

rep('''std::shared_ptr<const BindingView> System::bindingView() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return binding_view_;
}
''', '''std::shared_ptr<const BindingView> System::bindingView() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return binding_view_;
}

// ---- process run modes -------------------------------------------------

void runInline(System& system, long max_cycles, const StopFlag& stop,
               const CycleCallback& after_cycle) {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / system.loopRateHz()));
    auto next   = std::chrono::steady_clock::now();
    long cycles = 0;
    while (stop == 0 && (max_cycles < 0 || cycles < max_cycles)) {
        system.step(HostClock::now());
        after_cycle();
        ++cycles;
        next += period;
        std::this_thread::sleep_until(next);
    }
}

bool runThreaded(System& system, long max_cycles, const StopFlag& stop,
                 const CycleCallback& after_poll, std::string& err) {
    if (!system.start(err)) {
        return false;
    }
    while (stop == 0 &&
           (max_cycles < 0 || system.cycle() < static_cast<uint64_t>(max_cycles))) {
        // the Brain profile boundary belongs to this thread
        system.applyPendingProfile();
        after_poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return true;
}
''')

open(p, 'w', newline='').write(s)
