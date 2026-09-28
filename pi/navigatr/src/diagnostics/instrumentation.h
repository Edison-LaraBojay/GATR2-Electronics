// instrumentation.h
// Live hardware and transport instrumentation for the viewer: per-link byte
// and frame counters, rates, decode rejections, optional raw bytes and
// decoded summaries, the Pico diagnostic frame, and the newest Brain
// telemetry. Written as one JSON object by the inspection service when a
// viewer has the instrumentation panel open.

#pragma once
#include <cstddef>

namespace navigatr
{

class JsonWriter;
class System;

struct InstrumentationOptions {
    bool        raw           = false; // include raw bytes (hex) when captured
    std::size_t max_raw_bytes = 512;   // per link, newest
    std::size_t max_decoded   = 32;    // per link, newest
};

// One JSON object; fields and meanings in docs/pico_link.md (Instrumentation),
// the viewer message that carries it in docs/inspection.md.
void writeInstrumentation(JsonWriter& w, const System& system, const InstrumentationOptions& options);

} // namespace navigatr
