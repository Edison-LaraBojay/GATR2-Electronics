// record_bookkeeping.h
// How a poll lands in a retained record, shared by the resource and sensor
// stages so both boundaries keep the same rules: health and diagnostic
// update every poll; a sample is stored only for a Valid publication whose
// payload matches the declaration; sequence counts publications within the
// record's epoch; receipt is the upstream receipt the producer carried, or
// the stage clock when the producer is the acquisition point; nothing else
// ever touches the retained sample. A reset clears the record and moves it
// to the next epoch.

#pragma once
#include <string>
#include <utility>

#include "core/function_status.h"
#include "core/payload_descriptor.h"
#include "core/records.h"

namespace navigatr
{

inline void storePoll(MeasurementRecord& record, PollResult&& poll,
                      const PayloadDescriptor& declared, MonotonicTime now) {
    record.state        = poll.state;
    record.lastPolledAt = now;
    record.diagnostic   = std::move(poll.diagnostic);
    if (!poll.publication.has_value()) {
        return;
    }
    if (poll.state != SourceState::kValid) {
        record.state      = SourceState::kFault;
        record.diagnostic = "published a sample while reporting " +
                            std::string(sourceStateName(poll.state));
        return;
    }
    if (!declared.matches(poll.publication->payload.cppType())) {
        record.state      = SourceState::kFault;
        record.diagnostic = "published a payload contradicting the declared " +
                            declared.stable_name;
        return;
    }
    StoredSample stored;
    stored.measuredAt = poll.publication->measuredAt;
    stored.receivedAt = poll.publication->receivedAt.isSet() ? poll.publication->receivedAt : now;
    stored.sequence   = record.latest.has_value() ? record.latest->sequence + 1 : 1;
    stored.epoch      = record.epoch;
    stored.upstream   = std::move(poll.publication->upstream);
    stored.payload    = std::move(poll.publication->payload);
    record.latest     = std::move(stored);
}

inline void resetRecord(MeasurementRecord& record) {
    const uint64_t next_epoch = record.epoch + 1;
    record                    = MeasurementRecord{};
    record.epoch              = next_epoch;
}

inline FunctionStatus statusOf(SourceState state) {
    if (state == SourceState::kFault) {
        return FunctionStatus::kFault;
    }
    return state == SourceState::kValid ? FunctionStatus::kOk : FunctionStatus::kNoData;
}

} // namespace navigatr
