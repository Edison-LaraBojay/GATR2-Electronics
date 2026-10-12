// events.cpp

#include "debug/diagnostics/events.h"

#include <algorithm>
#include <cstring>

#include "runtime/system.h"

namespace navigatr
{

namespace
{

constexpr std::size_t kEventsKept = 32;

} // namespace

void System::noteEvent(MonotonicTime at, std::string text) {
    // events are rare: post unconditionally, so hub.latest() has the newest
    DiagRecord record;
    record.kind   = DiagKind::kEvent;
    record.source = event_source_;
    DiagEvent e;
    const std::size_t n = std::min(text.size(), sizeof(e.text) - 1);
    std::memcpy(e.text, text.data(), n);
    e.text[n]      = '\0';
    record.payload = e;
    diag_hub_->post(std::move(record));

    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    events_.push_back(RuntimeEvent{++event_count_, at, std::move(text)});
    if (events_.size() > kEventsKept) {
        events_.erase(events_.begin());
    }
}

void System::noteFault(CaptureFault fault, const std::string& text) {
    // queues a note for the recorder thread; never waits for capture work
    if (capture_ != nullptr) {
        capture_->fault(fault, text);
    }
}

std::vector<RuntimeEvent> System::events() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return events_;
}

} // namespace navigatr
