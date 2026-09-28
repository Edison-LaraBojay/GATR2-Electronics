// inspection_document.h
// The versioned inspection contract, navigatr.inspect/2, produced from the
// runtime's published snapshots. Live feed messages (docs/inspection.md):
//
//   hello     static identity: session, configuration, features, field
//             definition with display and planning data, cameras,
//             localization layout
//   state     small and frequent: the robot and localization summary
//   history   the bounded trail, after hello and on request
//   diag      the snapshot without its trail, plus per-client queue stats,
//             document costs and hub counters
//   event     one runtime lifecycle event
//   telemetry one Brain TELEMETRY report, decoded
//   instrumentation, capture, pong
//   frame     the header that precedes one JPEG preview, naming exactly
//             which frame the bytes belong to
//
// GET /api/snapshot keeps the inspect/1 snapshot document (with trail).
//
// Every time in a document is the Pi host monotonic clock in milliseconds
// since process start ("host_ms"); a document carries the host time it was
// produced at so clients compute ages against it, never against browser
// wall time. Documents are read-only descriptions: nothing in them feeds
// an estimate.

#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

#include "core/time.h"
#include "diagnostics/records.h"
#include "inspection/inspection_stats.h"
#include "runtime/inspection_state.h"
#include "runtime/system.h"
#include "state/attitude.h"

namespace navigatr
{

class CaptureService;
struct InstrumentationOptions;

constexpr const char* kInspectionContract = "navigatr.inspect/2";
constexpr const char* kSnapshotContract   = "navigatr.inspect/1";   // GET /api/snapshot shape
constexpr std::size_t kHistoryMaxEntries  = 300;
// Measured attitude older than this is reported stale.
constexpr int64_t kAttitudeFreshMs = 250;

// measured, stale, assumed_level or unavailable.
const char* attitudeStatus(const Attitude& a, MonotonicTime now);

// The capture service reports itself available.
bool captureAvailable(const System& system);

// client_id, when not 0, names the feed client the hello goes to.
std::string helloDocument(const System& system, MonotonicTime now, uint64_t client_id = 0);

// Snapshot, state, history and diag label their data with the reset count
// it was read under: the count is read before the data and checked after,
// so no document carries data of an earlier session than its label says.

std::string snapshotDocument(const System& system, const InspectionServiceStats& service,
                             MonotonicTime now, std::size_t trail_max_entries = 300);

// publication receives the feed publication the state describes. Empty
// (nothing to send) when resets kept landing during the build.
std::string stateDocument(const System& system, uint64_t seq, MonotonicTime now, int64_t now_us,
                          uint64_t* publication = nullptr);

std::string historyDocument(const System& system, uint64_t seq, MonotonicTime now,
                            std::size_t max_entries = kHistoryMaxEntries);

// content_hash receives a hash of the content that is not varying: seq,
// host_ms, clocks, ages, counters, timings, the robot block (state carries
// it) and the inspection transport counters are left out, for the
// per-client identical-diag skip. Empty as stateDocument.
std::string diagDocument(const System& system, const InspectionServiceStats& service,
                         const InspectionFeedStats& feed, uint64_t seq, MonotonicTime now,
                         uint64_t* content_hash = nullptr);

// seq is the System event sequence, the same value as diag.events[].sequence.
std::string eventDocument(const RuntimeEvent& event);

// id and client_ms are echoed as the client sent them (checked number
// tokens, or empty for null).
std::string pongDocument(const std::string& id_token, const std::string& client_ms_token,
                         MonotonicTime now, int64_t now_us);

// Decodes a hub TELEMETRY record with the translaGATR codec. False when the
// body is not a valid TELEMETRY body.
bool decodeTelemetryRecord(const DiagBrainTelemetry& rec, translagatr::BrainTelemetry& out);

std::string telemetryDocument(uint64_t seq, MonotonicTime now, const DiagRecord& record,
                              const DiagBrainTelemetry& rec,
                              const translagatr::BrainTelemetry& telemetry);

std::string instrumentationDocument(const System& system, uint64_t seq, MonotonicTime now,
                                    const InstrumentationOptions& options);

std::string captureDocument(const CaptureService& capture, uint64_t seq, MonotonicTime now);

std::string frameHeaderDocument(const DetectionFrameSnapshot& frame, int preview_width_px,
                                int preview_height_px, long quality, double encode_ms,
                                MonotonicTime now, const System* system = nullptr);

} // namespace navigatr
