// records.h
// Standard record envelopes and the result maps for the pipeline boundaries.
// Envelopes are fixed; payloads are open.
//
// Source semantics: a poll reports health separately from data. A healthy
// source with no new sample this cycle is Valid without a publication; its
// record keeps the last sample, and an unchanged sequence says no new
// publication occurred. A slow camera does not disappear between frames,
// and fault states do not erase history. A missing id in a result map means
// not configured, nothing else.
//
//   ResourceMap: resource id -> ResourceRecord (status + named outputs)
//   SensorMap:   sensor id   -> MeasurementRecord
//
// The stage that owns a map assigns sequence and epoch. Receipt is the host
// time the upstream data actually arrived: a producer that forwards or
// derives from an earlier record carries that receipt along, and only a
// producer that is itself the acquisition point leaves it unset for the
// stage clock. Provenance names the upstream source, its clock, and its
// own sequence and restart epoch, so a derived record never loses where
// its evidence came from.

#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/ids.h"
#include "core/time.h"
#include "core/typed_payload.h"

namespace navigatr
{

enum class SourceState : uint8_t {
    kNoDataYet = 0,   // initialized, never produced a sample
    kValid,           // healthy; publication says whether a new sample exists
    kUnavailable,     // temporarily cannot provide data
    kFault,           // detected an actual error
};

inline const char* sourceStateName(SourceState s) {
    switch (s) {
    case SourceState::kNoDataYet: return "no_data_yet";
    case SourceState::kValid: return "valid";
    case SourceState::kUnavailable: return "unavailable";
    case SourceState::kFault: return "fault";
    }
    return "unknown";
}

// Where a sample came from: the producing output or sensor, the physical
// measurement behind it, the clock its measuredAt is on, and the
// producer's own sequence and restart epoch. source names the layer that
// published this record (a sensor id, an output); measurement names the
// acquisition output it was derived from and is carried unchanged through
// every derived record, so two sensors reading one Pico output share a
// measurement while two outputs of one Pico do not.
struct Provenance {
    std::string source;        // "robot_imu", "pico_telemetry.imu", ...
    std::string measurement;   // "pico_telemetry.imu"; empty when unknown
    std::string clock;         // "pico_uart" device clock, "host", ...
    uint64_t    sequence = 0;
    uint64_t    epoch    = 0;   // bumps on a source restart or discontinuity
};

// What a producer hands back for one output from one poll.
struct Publication {
    MonotonicTime measuredAt;   // source clock
    MonotonicTime receivedAt;   // host receipt of the upstream data; unset = stage clock
    Provenance    upstream;     // what this was derived from, empty at acquisition
    TypedPayload  payload;
};

struct PollResult {
    SourceState                state = SourceState::kNoDataYet;
    std::optional<Publication> publication;   // only with state Valid
    std::string                diagnostic;    // empty when nothing to report
};

// What the owning stage stores. sequence and epoch are assigned by the
// stage, never by the producer.
struct StoredSample {
    MonotonicTime measuredAt;   // source clock
    MonotonicTime receivedAt;   // host clock, actual upstream receipt
    uint64_t      sequence = 0; // increments only for a new publication
    uint64_t      epoch    = 0; // bumps on reset; sequences restart with it
    Provenance    upstream;
    TypedPayload  payload;
};

struct MeasurementRecord {
    SourceState                 state = SourceState::kNoDataYet;
    MonotonicTime               lastPolledAt;   // host clock
    std::optional<StoredSample> latest;
    std::string                 diagnostic;
    uint64_t                    epoch = 0;   // current epoch, copied into samples
};

using SensorMap = std::unordered_map<SensorId, MeasurementRecord, SensorId::Hash>;

// One executed resource: its own health plus every declared output, each
// with its own timing and health. A single-output resource uses the same
// shape, so consumers address every source the same way.
struct ResourceRecord {
    SourceState   state = SourceState::kNoDataYet;
    MonotonicTime lastPolledAt;   // host clock
    std::string   diagnostic;

    std::unordered_map<OutputId, MeasurementRecord, OutputId::Hash> outputs;
};

using ResourceMap = std::unordered_map<ResourceId, ResourceRecord, ResourceId::Hash>;

struct ObservationRecord {
    MonotonicTime measuredAt;
    TypedPayload  payload;
};

struct AssociationRecord {
    MonotonicTime measuredAt;
    TypedPayload  payload;
};

using ObservationMap = std::unordered_map<ObservationId, ObservationRecord, ObservationId::Hash>;
using AssociationMap = std::unordered_map<AssociationId, AssociationRecord, AssociationId::Hash>;

} // namespace navigatr
