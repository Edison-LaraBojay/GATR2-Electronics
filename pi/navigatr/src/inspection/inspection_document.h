// inspection_document.h
// The versioned inspection contract, navigatr.inspect/1, produced from the
// runtime's published snapshots. Three documents:
//
//   hello     static identity: session, configuration, field definition
//             with display data, cameras, localization layout
//   snapshot  the live state: robot, localization, trail, field objects,
//             detection frames with overlays bound to their image identity,
//             sources, workers, diagnostics
//   frame     the header that precedes one JPEG preview, naming exactly
//             which frame the bytes belong to
//
// Every time in a document is the Pi host monotonic clock in milliseconds
// since process start ("host_ms"); a document carries the host time it was
// produced at so clients compute ages against it, never against browser
// wall time. Documents are read-only descriptions: nothing in them feeds
// an estimate. See docs/inspection.md for the field-by-field description.

#pragma once
#include <cstddef>
#include <string>

#include "core/time.h"
#include "inspection/inspection_stats.h"
#include "runtime/inspection_state.h"
#include "runtime/system.h"

namespace navigatr
{

constexpr const char* kInspectionContract = "navigatr.inspect/1";

std::string helloDocument(const System& system, MonotonicTime now);

std::string snapshotDocument(const System& system, const InspectionServiceStats& service,
                             MonotonicTime now, std::size_t trail_max_entries = 300);

std::string frameHeaderDocument(const DetectionFrameSnapshot& frame, int preview_width_px,
                                int preview_height_px, long quality, double encode_ms,
                                MonotonicTime now);

} // namespace navigatr
