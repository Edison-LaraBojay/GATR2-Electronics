// run.cpp

#include "runtime/system.h"

#include <array>
#include <chrono>
#include <stdexcept>

#include "core/host_clock.h"

namespace navigatr
{

namespace
{

double elapsedMs(std::chrono::steady_clock::time_point since) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - since)
        .count();
}

} // namespace

// ---- cycles --------------------------------------------------------------

void System::step(MonotonicTime now) {
    if (profile_pending_.load()) {
        swapProfile();   // the inline boundary
    }
    estimationCycle(now);
    // the inline cycle lets the same cycle's evidence reach target
    // resolution and publishing, as the synchronous tests expect
    fieldCycle(execute_sensors_.retained(), now);
    reportingCycle(now);
}

void System::estimationCycle(MonotonicTime now) {
    const uint64_t cycle = cycle_.fetch_add(1) + 1;
    ++diagnostics_.cycles;
    last_now_ = now;
    const ExecutionContext context{now, cycle, &diagnostics_};

    const ResourceMap& resource_map = execute_resources_(context);
    const SensorMap&   sensor_map   = execute_sensors_(resource_map, context);
    if (profiled_) {
        precheck_.update(sensor_map, now);
    }

    CommandsOutput commands_out = commands_->run({command_, now, &diagnostics_});
    command_                    = commands_out.command;
    diagnostics_.note(slot_labels_[0], commands_out.status);
    if (profiled_) {
        // with this cycle's Brain IMU sample, before any placement request
        // reaches localization
        watchSensors(sensor_map, now);
    }

    robot_ = execute_localization_(sensor_map, requestsFrom(command_), context);
    if (profiled_) {
        watchModels(now);
    }
    noteRecovery(now);
    if (pico_ != nullptr) {
        updatePicoOperation(now);
    }

    publishSourceHealth(now);
}

LocalizationRequests System::requestsFrom(const CommandState& command) const {
    LocalizationRequests requests;
    // A new brain session withdraws a placement not applied yet; an applied
    // one stays recorded in the robot state and never re-applies. A profile
    // boundary or a reinitialize withdraws every request up to its floor.
    if (command.init_sequence > placement_floor_ && command.init_session == command.session) {
        requests.placement.requested = true;
        requests.placement.origin    = "command";
        requests.placement.session   = command.init_session;
        requests.placement.sequence  = command.init_sequence;
        requests.placement.pose      = command.init_pose;
    }
    return requests;
}

void System::reportingCycle(MonotonicTime now) {
    std::shared_ptr<const FieldSnapshot> field  = fieldSnapshot();
    const LocalizationStatus             status = feed_->status();
    TargetResolutionOutput               target_out = target_resolution_->run(
        {command_, robot_, *feed_, field->field, field->observations, field->associations,
         target_, now});
    target_ = target_out.target;
    diagnostics_.note(slot_labels_[1], target_out.status);

    PublishingOutput pub_out = publishing_->run(
        {execute_sensors_.retained(), field->observations, field->associations, robot_,
         status, field->field, command_, target_, now, &diagnostics_});
    diagnostics_.note(slot_labels_[2], pub_out.status);

    auto reporting            = std::make_shared<ReportingSnapshot>();
    reporting->command        = command_;
    reporting->target         = target_;
    reporting->at             = now;
    reporting->cycle          = cycle_.load();
    reporting->brain_state    = pub_out.brain_state;
    reporting->pico_operation = pico_op_;
    if (profiled_) {
        std::array<translagatr::WheelReading, translagatr::kWheelReadingsMax> wheels{};
        uint8_t                                                    count = 0;
        if (readWheels(now, count, wheels.data()) == translagatr::kResultOk) {
            reporting->wheels.assign(wheels.begin(), wheels.begin() + count);
        }
    }
    auto diagnostics = std::make_shared<DiagnosticsSnapshot>();
    diagnostics->estimation = diagnostics_;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        reporting_ = std::move(reporting);
        if (diagnostics_snapshot_ != nullptr) {
            diagnostics->field = diagnostics_snapshot_->field;
        }
        diagnostics_snapshot_ = std::move(diagnostics);
    }
}

void System::fieldCycle(const SensorMap& sensors, MonotonicTime now) {
    std::shared_ptr<const FieldSnapshot> previous = fieldSnapshot();
    auto                                 next     = std::make_shared<FieldSnapshot>();
    const RobotState                     robot    = feed_->latest();
    ++field_diagnostics_.cycles;
    const uint64_t         invocation = ++field_invocations_;
    const ExecutionContext context{now, invocation, &field_diagnostics_};

    // One standard call: the whole sensor snapshot, the latest robot state,
    // history lookups, the previous field and the context; the executor
    // owns declared-output enforcement and diagnostics.
    FieldEstimationOutput field_out =
        execute_world_({sensors, robot, *feed_, previous->field, context});

    next->field        = std::move(field_out.field);
    next->observations = std::move(field_out.observations);
    next->associations = std::move(field_out.associations);
    next->at           = now;
    next->invocation   = invocation;
    next->status       = field_out.status;
    next->diagnostic   = field_out.diagnostic;

    publishDetectionFrames(sensors, *next, now);

    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        field_ = next;
        auto diagnostics = std::make_shared<DiagnosticsSnapshot>();
        if (diagnostics_snapshot_ != nullptr) {
            diagnostics->estimation = diagnostics_snapshot_->estimation;
        }
        diagnostics->field    = field_diagnostics_;
        diagnostics_snapshot_ = std::move(diagnostics);
    }
}

// ---- lifecycle -----------------------------------------------------------

void System::resetStages() {
    execute_resources_.reset();   // shared resources reset once, not per consumer
    execute_sensors_.reset();
    execute_localization_.reset();
    execute_world_.reset();
    commands_->reset();
    target_resolution_->reset();
    publishing_->reset();

    robot_   = execute_localization_.state();
    command_ = CommandState{};
    target_  = TargetState{};
    field_invocations_ = 0;
    field_handoff_.clear();
    placement_floor_   = 0;
    if (profiled_) {
        swapToWaiting();   // power-on: no profile
    }

    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    field_ = std::make_shared<FieldSnapshot>();
    detection_frames_.clear();
    reporting_.reset();
    source_health_.reset();
}

void System::reset() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    const bool                  was_running = pauseWorkers();
    // the boundary before the stages publish their power-on state, so
    // capture rows from here on carry the new reset count
    if (capture_ != nullptr) {
        capture_->noteReset(reset_count_.load() + 1);
    }
    resetStages();
    reset_count_.fetch_add(1);
    if (was_running) {
        estimation_stats_.resetCounters();
        field_stats_.resetCounters();
    }
    resumeWorkers(was_running);
}

bool System::start(std::string& err) {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    return startWorkers(err);
}

bool System::startWorkers(std::string& err) {
    if (running_.load()) {
        return true;
    }
    err.clear();
    stop_.store(false);
    field_handoff_.resume();
    running_.store(true);
    estimation_stats_.setRunning(true);
    field_stats_.setRunning(true);
    try {
        field_thread_      = std::thread([this] { fieldWorker(); });
        estimation_thread_ = std::thread([this] { estimationWorker(); });
    } catch (const std::exception& e) {
        err = std::string("cannot start workers: ") + e.what();
        requestStop();
        if (field_thread_.joinable()) {
            field_thread_.join();
        }
        if (estimation_thread_.joinable()) {
            estimation_thread_.join();
        }
        running_.store(false);
        estimation_stats_.setRunning(false);
        field_stats_.setRunning(false);
        return false;
    }
    return true;
}

void System::requestStop() {
    {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        stop_.store(true);
    }
    wake_.notify_all();
    field_handoff_.stop();
}

void System::stop() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    if (!running_.load()) {
        return;
    }
    // Wake both workers, then join before any stage can be destroyed.
    requestStop();
    if (estimation_thread_.joinable()) {
        estimation_thread_.join();
    }
    if (field_thread_.joinable()) {
        field_thread_.join();
    }
    running_.store(false);
    estimation_stats_.setRunning(false);
    field_stats_.setRunning(false);
}

void System::estimationWorker() {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / loop_rate_hz_));
    auto next = std::chrono::steady_clock::now();
    while (!stop_.load()) {
        const auto          started = std::chrono::steady_clock::now();
        const MonotonicTime now     = HostClock::now();
        estimationCycle(now);
        // the field worker gets its own immutable copy: payloads are shared
        // immutable values, so the copy is the envelopes only
        field_handoff_.put(FieldWork{
            std::make_shared<const SensorMap>(execute_sensors_.retained()), now});
        reportingCycle(now);
        estimation_stats_.cycleDone(elapsedMs(started), now.ms, 0, 0);

        next += period;
        const auto after = std::chrono::steady_clock::now();
        if (next < after - period) {
            next = after;   // fell behind by more than a period: resync, do not burst
        }
        std::unique_lock<std::mutex> lock(wake_mutex_);
        wake_.wait_until(lock, next, [this] { return stop_.load(); });
    }
}

void System::fieldWorker() {
    while (!stop_.load()) {
        FieldWork work;
        if (!field_handoff_.waitTake(work, std::chrono::milliseconds(100))) {
            continue;
        }
        const auto started = std::chrono::steady_clock::now();
        fieldCycle(*work.sensors, work.now);
        field_stats_.cycleDone(elapsedMs(started), HostClock::now().ms,
                               field_handoff_.replaced(),
                               field_handoff_.pending() ? 1 : 0);
    }
}

bool System::pauseWorkers() {
    const bool was_running = running_.load();
    if (was_running) {
        // join both workers so exactly one thread touches every stage
        requestStop();
        if (estimation_thread_.joinable()) {
            estimation_thread_.join();
        }
        if (field_thread_.joinable()) {
            field_thread_.join();
        }
        running_.store(false);
    }
    estimation_stats_.setRunning(false);
    field_stats_.setRunning(false);
    return was_running;
}

void System::resumeWorkers(bool was_running) {
    if (!was_running) {
        return;
    }
    std::string err;
    if (!startWorkers(err)) {
        throw std::runtime_error(err);
    }
}

// ---- snapshot readers --------------------------------------------------

std::shared_ptr<const FieldSnapshot> System::fieldSnapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return field_;
}

std::shared_ptr<const ReportingSnapshot> System::reportingSnapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return reporting_;
}

std::shared_ptr<const DiagnosticsSnapshot> System::diagnosticsSnapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return diagnostics_snapshot_;
}

void System::publishBindingView() {
    auto view            = std::make_shared<BindingView>();
    view->sensors        = execute_sensors_.outputs();
    view->estimator_type = execute_localization_.estimatorType();
    view->warnings       = warnings_;
    view->warnings.insert(view->warnings.end(), profile_warnings_.begin(),
                          profile_warnings_.end());
    view->brain_profile = profiled_;
    view->profile       = profileBinding();
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    binding_view_ = std::move(view);
}

std::shared_ptr<const BindingView> System::bindingView() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return binding_view_;
}

} // namespace navigatr
