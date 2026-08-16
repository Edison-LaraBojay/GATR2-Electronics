// records.h
// Standard record envelopes and map aliases for the pipeline boundaries.
// Envelopes are fixed; payloads are open.
//
// Sensor semantics: a sensor poll reports health separately from data. A
// healthy sensor with no new sample this cycle is Valid without a
// publication; its stored record keeps the last sample, and an unchanged
// sequence says no new publication occurred. A slow camera does not
// disappear between frames, and fault states do not erase history. A missing
// id in SensorResultsMap means not configured, nothing else.

#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include "core/ids.h"
#include "core/time.h"
#include "core/typed_payload.h"

namespace navigatr
{

enum class SensorState : uint8_t {
    kNoDataYet = 0,   // initialized, never produced a sample
    kValid,           // healthy; publication says whether a new sample exists
    kUnavailable,     // temporarily cannot provide data
    kFault,           // detected an actual error
};

inline const char* sensorStateName(SensorState s) {
    switch (s) {
    case SensorState::kNoDataYet: return "no_data_yet";
    case SensorState::kValid: return "valid";
    case SensorState::kUnavailable: return "unavailable";
    case SensorState::kFault: return "fault";
    }
    return "unknown";
}

// What a sensor hands back from one poll.
struct SensorPublication {
    MonotonicTime measuredAt;   // device clock
    TypedPayload  payload;
};

struct SensorPollResult {
    SensorState                      state = SensorState::kNoDataYet;
    std::optional<SensorPublication> publication;
    std::string                      diagnostic;   // empty when nothing to report
};

// What Sensor Collection stores. receivedAt and sequence are assigned by the
// collection step, never by the sensor.
struct StoredSensorSample {
    MonotonicTime measuredAt;   // device clock
    MonotonicTime receivedAt;   // host clock
    uint64_t      sequence = 0; // increments only for a new publication
    TypedPayload  payload;
};

struct SensorRecord {
    SensorState                       state = SensorState::kNoDataYet;
    MonotonicTime                     lastPolledAt;   // host clock
    std::optional<StoredSensorSample> latest;
    std::string                       diagnostic;
};

using SensorResultsMap = std::unordered_map<SensorId, SensorRecord, SensorId::Hash>;

struct ArtifactRecord {
    MonotonicTime measuredAt;
    MonotonicTime receivedAt;   // host receipt of the newest consumed sample
    TypedPayload  payload;
};

struct ObservationRecord {
    MonotonicTime measuredAt;
    TypedPayload  payload;
};

struct AssociationRecord {
    MonotonicTime measuredAt;
    TypedPayload  payload;
};

using ArtifactMap    = std::unordered_map<ArtifactId, ArtifactRecord, ArtifactId::Hash>;
using ObservationMap = std::unordered_map<ObservationId, ObservationRecord, ObservationId::Hash>;
using AssociationMap = std::unordered_map<AssociationId, AssociationRecord, AssociationId::Hash>;

} // namespace navigatr
