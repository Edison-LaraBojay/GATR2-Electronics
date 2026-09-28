// instrumentation.cpp
// The viewer's instrumentation object (spec section 7, docs/inspection.md):
// per-link counters and rings from the LinkMonitors, the Pico link state and
// diagnostic frame, encoder activity, the newest Brain telemetry and bench
// IMU sample, and the hub counters. Ages are Pi host clock at write time.
// Everything is read through thread-safe views; nothing here touches a link.

#include "diagnostics/instrumentation.h"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

#include "core/host_clock.h"
#include "diagnostics/hub.h"
#include "impl/resources/pico_telemetry.h"
#include "inspection/json_writer.h"
#include "runtime/system.h"
#include "translaGATR/frame_codec.h"

namespace navigatr
{
namespace
{

void msAgo(JsonWriter& w, const char* key, int64_t at_us, int64_t now_us) {
    if (at_us < 0) {
        w.fieldNull(key);
    } else {
        w.field(key, (now_us - at_us) / 1000);
    }
}

std::string hex(const std::vector<uint8_t>& bytes) {
    static constexpr char kDigits[] = "0123456789ABCDEF";
    std::string out;
    out.reserve(bytes.size() * 2);
    for (uint8_t b : bytes) {
        out.push_back(kDigits[b >> 4]);
        out.push_back(kDigits[b & 15]);
    }
    return out;
}

const char* dirName(DiagDirection d) {
    switch (d) {
    case DiagDirection::kRx: return "rx";
    case DiagDirection::kTxAccepted: return "tx";
    case DiagDirection::kTxAttempted: return "tx_attempted";
    }
    return "rx";
}

const char* imuStateName(uint8_t s) {
    switch (s) {
    case translagatr::kPicoImuDisabled: return "disabled";
    case translagatr::kPicoImuInitializing: return "initializing";
    case translagatr::kPicoImuAligning: return "aligning";
    case translagatr::kPicoImuReady: return "ready";
    case translagatr::kPicoImuRetrying: return "retrying";
    case translagatr::kPicoImuFailed: return "failed";
    default: return "unknown";
    }
}

const char* imuReasonName(uint8_t r) {
    switch (r) {
    case translagatr::kPicoImuReasonNone: return "none";
    case translagatr::kPicoImuReasonNoResponse: return "no response";
    case translagatr::kPicoImuReasonBoot: return "boot timeout";
    case translagatr::kPicoImuReasonFeatures: return "reports not acknowledged";
    case translagatr::kPicoImuReasonStream: return "reports stopped";
    default: return "unknown";
    }
}

const char* firmwareName(uint8_t f) {
    switch (f) {
    case translagatr::kPicoFirmwareBno08x: return "bno08x";
    case translagatr::kPicoFirmwareAsm330: return "asm330";
    default: return "unknown";
    }
}

const char* picoOpName(uint8_t op) {
    switch (op) {
    case 0: return "none";
    case translagatr::kPicoOpConfigure: return "CONFIGURE";
    case translagatr::kPicoOpReinitImu: return "REINIT_IMU";
    case translagatr::kPicoOpRestartAcquisition: return "RESTART_ACQUISITION";
    case translagatr::kPicoOpDiagnostics: return "DIAGNOSTICS";
    default: return "unknown";
    }
}

const char* picoCommandStatusName(uint8_t s) {
    switch (s) {
    case translagatr::kPicoCommandNone: return "none";
    case translagatr::kPicoCommandRunning: return "running";
    case translagatr::kPicoCommandCompleted: return "completed";
    case translagatr::kPicoCommandFailed: return "failed";
    default: return "unknown";
    }
}

const char* picoDetailName(uint8_t d) {
    switch (d) {
    case translagatr::kPicoDetailNone: return "none";
    case translagatr::kPicoDetailWrongTarget: return "wrong target";
    case translagatr::kPicoDetailUnknownOp: return "unknown op";
    case translagatr::kPicoDetailBadBody: return "bad body";
    case translagatr::kPicoDetailImuAbsent: return "IMU absent";
    case translagatr::kPicoDetailImuDisabled: return "IMU disabled";
    case translagatr::kPicoDetailNoSuchPort: return "no such port";
    default: return "unknown";
    }
}

std::string hex16(uint16_t v) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%04X", static_cast<unsigned>(v));
    return buf;
}

int64_t hostAgeMs(MonotonicTime t, MonotonicTime now) {
    return t.isSet() && t.domain == ClockDomain::kHost ? now.ms - t.ms : -1;
}

void ageField(JsonWriter& w, const char* key, int64_t age_ms) {
    if (age_ms < 0) {
        w.fieldNull(key);
    } else {
        w.field(key, age_ms);
    }
}

struct PinInfo {
    uint16_t    bit;
    const char* name;
    bool        driven; // the Pico drives it as an output; else it reads an input
};

constexpr PinInfo kPins[] = {
    {translagatr::kPicoPinImuInt, "imu_int", false},
    {translagatr::kPicoPinImuRst, "imu_rst", true},
    {translagatr::kPicoPinImuWake, "imu_wake", true},
    {translagatr::kPicoPinImuCs, "imu_cs", true},
    {translagatr::kPicoPinEnc0A, "enc0_a", false},
    {translagatr::kPicoPinEnc0B, "enc0_b", false},
    {translagatr::kPicoPinEnc1A, "enc1_a", false},
    {translagatr::kPicoPinEnc1B, "enc1_b", false},
    {translagatr::kPicoPinEnc2A, "enc2_a", false},
    {translagatr::kPicoPinEnc2B, "enc2_b", false},
    {translagatr::kPicoPinPiRx, "pi_uart_rx", false},
};

void pinLevel(JsonWriter& w, const char* key, const translagatr::PicoDiag& d, uint16_t bit) {
    if ((d.pins_known & bit) == 0) {
        w.fieldNull(key);
    } else {
        w.field(key, (d.pins & bit) != 0 ? "HIGH" : "LOW");
    }
}

void writeLink(JsonWriter& w, const LinkMonitorSnapshot& s, const InstrumentationOptions& options,
               int64_t now_us) {
    w.beginObject();
    w.field("id", s.id);
    w.field("kind", s.kind);
    w.field("rx_bytes", s.rx_bytes);
    w.field("tx_attempted", s.tx_attempted);
    w.field("tx_accepted", s.tx_accepted);
    w.field("rx_frames", s.rx_frames);
    w.field("tx_frames", s.tx_frames);
    w.field("rejected", s.rejected);
    w.key("reader");
    w.beginObject();
    w.field("bytes", s.reader.bytes);
    w.field("frames", s.reader.frames);
    w.field("sync_dropped", s.reader.sync_dropped);
    w.field("length_errors", s.reader.length_errors);
    w.field("check_errors", s.reader.check_errors);
    w.endObject();
    w.key("rates");
    w.beginObject();
    w.field("rx_bytes_s", s.rx_bytes_per_s);
    w.field("tx_bytes_s", s.tx_bytes_per_s);
    w.field("rx_frames_s", s.rx_frames_per_s);
    w.endObject();
    msAgo(w, "last_rx_ms_ago", s.last_rx_us, now_us);
    msAgo(w, "last_tx_ms_ago", s.last_tx_us, now_us);
    msAgo(w, "last_valid_rx_ms_ago", s.last_valid_rx_us, now_us);
    w.field("raw_on", s.raw_on);
    w.key("raw");
    w.beginArray();
    if (options.raw) {
        for (const LinkRawChunk& c : s.raw) {
            w.beginObject();
            w.field("ms_ago", (now_us - c.host_us) / 1000);
            w.field("dir", dirName(c.dir));
            w.field("hex", hex(c.bytes));
            w.endObject();
        }
    }
    w.endArray();
    w.key("decoded");
    w.beginArray();
    for (const LinkDecodedEntry& e : s.decoded) {
        w.beginObject();
        w.field("ms_ago", (now_us - e.host_us) / 1000);
        w.field("dir", e.rx ? "rx" : "tx");
        w.field("name", e.name);
        w.field("fields", e.fields);
        w.endObject();
    }
    w.endArray();
    w.key("errors");
    w.beginArray();
    for (const LinkErrorEntry& e : s.errors) {
        w.beginObject();
        w.field("ms_ago", (now_us - e.host_us) / 1000);
        w.field("reason", e.reason);
        w.endObject();
    }
    w.endArray();
    w.endObject();
}

// The Pico link the Brain link names, else the first pico_telemetry resource.
std::shared_ptr<PicoTelemetry> findPico(const System& system) {
    if (auto named = std::dynamic_pointer_cast<PicoTelemetry>(system.picoControl())) {
        return named;
    }
    for (const ResourceStore::Record& r : system.resources().records()) {
        std::string err;
        if (auto p = r.value.require<PicoTelemetry>(err)) {
            return p;
        }
    }
    return nullptr;
}

void writePicoDiag(JsonWriter& w, const PicoInstrumentation& v, const PicoLinkState& link,
                   int64_t now_us) {
    const translagatr::PicoDiag& d = v.diag;
    w.beginObject();
    msAgo(w, "age_ms", v.diag_host_us, now_us);
    w.field("seq", d.seq);
    w.field("boot_id", hex16(d.boot_id));
    w.field("current_boot", link.identity && link.boot_id == d.boot_id);
    w.field("firmware", firmwareName(d.firmware));
    w.field("frames", v.diag_frames);
    w.field("pins_note", "logic levels read back from the pads when the frame was built; not "
                         "voltages, and one sample per frame misses fast transitions. UART RX "
                         "idles HIGH, so HIGH there does not mean connected or disconnected.");
    w.key("pins");
    w.beginArray();
    for (const PinInfo& p : kPins) {
        w.beginObject();
        w.field("name", p.name);
        const bool known = (d.pins_known & p.bit) != 0;
        if (known) {
            w.field("level", (d.pins & p.bit) != 0 ? "HIGH" : "LOW");
        } else {
            w.fieldNull("level");
        }
        w.field("known", known);
        w.field("driven", p.driven);
        w.endObject();
    }
    w.endArray();
    const bool asm330 = d.firmware == translagatr::kPicoFirmwareAsm330;
    w.key("imu");
    w.beginObject();
    w.field("present", (d.flags & translagatr::kPicoDiagImuPresent) != 0);
    w.field("rx", d.imu_rx);
    w.field("bad", d.imu_bad);
    w.field("resets", d.imu_resets);
    w.field("error", d.imu_error);
    w.field("reports_ok", d.reports_ok);
    if (asm330) {
        w.fieldNull("reports_rejected");   // no report validation on this driver
    } else {
        w.field("reports_rejected", d.reports_rejected);
    }
    if (d.report_age_ms == 0xFFFF) {
        w.fieldNull("report_age_ms");
    } else {
        w.field("report_age_ms", d.report_age_ms);   // 65534 means at least that
    }
    w.field("meaning", asm330 ? "rx gyro samples read; bad failed probes, configurations and "
                                "checks; resets software resets; error last failure reason; "
                                "counters wrap at 16 bits"
                              : "rx SHTP packets; bad headers without a length; resets hub "
                                "resets; error last sh2 result (negative SH2_ERR); counters "
                                "wrap at 16 bits");
    w.endObject();
    w.field("link_rx_bad", d.link_rx_bad);
    w.field("ticks_skipped", d.ticks_skipped);
    w.endObject();
}

void writePico(JsonWriter& w, const PicoTelemetry& pico, int64_t now_us) {
    const PicoInstrumentation v    = pico.instrumentation();
    const PicoLinkState       link = pico.link();
    const MonotonicTime       now  = HostClock::now();
    // fresh: within three frame periods and at least three seconds
    const int64_t period_ms = v.diag_hz > 0 ? 1000 / v.diag_hz : 1000;
    const int64_t fresh_ms  = std::max<int64_t>(3000, 3 * period_ms);
    const bool    available = v.have_diag && v.diag_host_us >= 0 &&
                           (now_us - v.diag_host_us) / 1000 <= fresh_ms;

    w.beginObject();
    w.field("resource", pico.clockId());
    w.field("available", available);
    w.field("reason", picoDiagStateName(v.diag_state));
    w.field("diag_hz", v.diag_hz);

    w.key("status");
    w.beginObject();
    w.field("frames_fresh", link.frames_fresh);
    w.field("identity", link.identity);
    w.field("boot_id", hex16(link.boot_id));
    w.field("acq_epoch", link.acq_epoch);
    w.field("imu_epoch", link.imu_epoch);
    w.field("reboots", link.reboots);
    w.field("restarts", link.restarts);
    w.field("imu_restarts", link.imu_restarts);
    w.field("sensor_frames", v.sensor_frames);
    w.field("status_frames", v.status_frames);
    ageField(w, "last_frame_ms_ago", hostAgeMs(link.last_frame, now));
    ageField(w, "last_status_ms_ago", hostAgeMs(link.last_status, now));
    if (v.serial_reopening) {
        w.key("serial");
        w.beginObject();
        w.field("open", v.serial_open);
        w.field("reopen_attempts", v.serial_attempts);
        w.field("reopens", v.serial_reopens);
        w.field("closes", v.serial_closes);
        w.field("last_error", v.serial_error);
        w.endObject();
    }
    w.field("status_known", link.status_known);
    if (link.status_known) {
        const translagatr::PicoStatus& s = link.status;
        w.field("uptime_ms", s.uptime_ms);
        w.field("imu_enabled", (s.flags & translagatr::kPicoImuEnabled) != 0);
        w.field("imu_state", imuStateName(s.imu_state));
        w.field("imu_reason", imuReasonName(s.imu_reason));
        w.field("imu_attempts", s.imu_attempts);
        w.field("last_request_id", s.last_request_id);
        w.field("last_op", picoOpName(s.last_op));
        w.field("last_status", picoCommandStatusName(s.last_status));
        w.field("last_detail", picoDetailName(s.last_detail));
        w.field("firmware", firmwareName(s.firmware));
    }
    w.endObject();

    w.key("diag");
    if (v.have_diag) {
        writePicoDiag(w, v, link, now_us);
    } else {
        w.null();
    }

    w.key("encoders");
    w.beginArray();
    for (int port = 0; port < 3; ++port) {
        const PicoEncoderView& e = v.encoders[port];
        w.beginObject();
        w.field("port", port);
        w.field("present", e.present);
        w.field("counts", e.counts);
        msAgo(w, "updated_ms_ago", e.updated_host_us, now_us);
        w.field("fresh", e.fresh);
        if (e.delta_known) {
            w.field("delta_1s", e.delta);
            w.field("span_ms", e.span_ms);
            w.field("direction", e.delta > 0 ? "fwd" : e.delta < 0 ? "rev" : "still");
        } else {
            w.fieldNull("delta_1s");
            w.fieldNull("span_ms");
            w.fieldNull("direction");
        }
        const uint16_t a = static_cast<uint16_t>(translagatr::kPicoPinEnc0A << (2 * port));
        const uint16_t b = static_cast<uint16_t>(translagatr::kPicoPinEnc0B << (2 * port));
        if (available) {
            pinLevel(w, "a", v.diag, a);
            pinLevel(w, "b", v.diag, b);
        } else {
            w.fieldNull("a");
            w.fieldNull("b");
        }
        const char* note = !e.present      ? "no counts received for this port"
                           : !e.fresh       ? "no recent counts: Pico frames for this port stopped"
                           : !e.delta_known ? "not enough samples yet"
                           : e.delta == 0   ? "no change: stationary or disconnected, cannot tell"
                                            : "counts changing: edges are arriving";
        w.field("note", note);
        w.endObject();
    }
    w.endArray();
    w.field("encoders_note", "raw Pico counts before profile polarity and gearing; fwd means "
                             "counts increasing. Fresh Pico frames do not prove an encoder is "
                             "connected.");
    w.endObject();
}

void writeBrain(JsonWriter& w, const System& system, int64_t now_us) {
    const DiagnosticsHub& hub = system.diagHub();
    w.beginObject();
    DiagRecord telemetry;
    uint64_t   telemetry_seq = 0;
    const bool have_telemetry = hub.latest(DiagKind::kBrainTelemetry, telemetry, &telemetry_seq);
    if (have_telemetry) {
        w.field("telemetry_age_ms", (now_us - telemetry.host_us) / 1000);
        w.field("telemetry_supported", true);
    } else {
        w.fieldNull("telemetry_age_ms");
        w.fieldNull("telemetry_supported");   // unknown until a report arrives
    }
    w.field("telemetry_reports", telemetry_seq);
    w.key("telemetry");
    const auto* body = have_telemetry ? std::get_if<DiagBrainTelemetry>(&telemetry.payload) : nullptr;
    translagatr::BrainTelemetry t;
    if (body != nullptr && translagatr::decodeTelemetryBody(body->body, body->len, t)) {
        w.beginObject();
        w.field("session", body->session);
        w.field("stamp_ms", t.stamp_ms);
        w.field("flags", t.flags);
        w.key("attitude");
        if ((t.flags & translagatr::kTelemetryAttitude) != 0) {
            w.beginObject();
            w.field("roll_deg", t.roll_cdeg / 100.0);
            w.field("pitch_deg", t.pitch_cdeg / 100.0);
            w.endObject();
        } else {
            w.null();
        }
        w.field("motion", (t.flags & translagatr::kTelemetryMotion) != 0);
        w.field("wheels", (t.flags & translagatr::kTelemetryWheels) != 0);
        w.endObject();
    } else {
        w.null();
    }

    DiagRecord imu;
    uint64_t   imu_seq = 0;
    w.key("vex_imu");
    const DiagVexImu* v = hub.latest(DiagKind::kVexImu, imu, &imu_seq)
                              ? std::get_if<DiagVexImu>(&imu.payload)
                              : nullptr;
    if (v != nullptr) {
        w.beginObject();
        w.field("age_ms", (now_us - imu.host_us) / 1000);
        w.field("samples", imu_seq);
        w.field("valid", (v->flags & translagatr::kBenchImuValid) != 0);
        w.field("accepted", v->accepted);
        w.field("session", v->session);
        w.field("stamp_ms", v->stamp_ms);
        w.field("rotation_deg", v->rotation_mdeg / 1000.0);
        w.endObject();
    } else {
        w.null();
    }
    w.endObject();
}

void writeHub(JsonWriter& w, const DiagnosticsHub& hub) {
    const DiagHubStats s = hub.stats();
    w.beginObject();
    w.key("posted");
    w.beginObject();
    for (int k = 0; k < kDiagKindCount; ++k) {
        w.field(diagKindName(static_cast<DiagKind>(k)), s.posted[k]);
    }
    w.endObject();
    w.key("dropped");
    w.beginObject();
    for (int k = 0; k < kDiagKindCount; ++k) {
        w.field(diagKindName(static_cast<DiagKind>(k)), s.dropped[k]);
    }
    w.endObject();
    w.field("queued", static_cast<uint64_t>(s.queued));
    w.field("capacity", static_cast<uint64_t>(s.capacity));
    w.endObject();
}

} // namespace

void writeInstrumentation(JsonWriter& w, const System& system, const InstrumentationOptions& options) {
    const int64_t now_us = HostClock::nowUs();
    w.beginObject();
    w.key("links");
    w.beginArray();
    for (const auto& m : system.diagHub().links().all()) {
        writeLink(w, m->snapshot(options.raw ? options.max_raw_bytes : 0, options.max_decoded),
                  options, now_us);
    }
    w.endArray();
    w.key("pico");
    if (const auto pico = findPico(system)) {
        writePico(w, *pico, now_us);
    } else {
        w.null();
    }
    w.key("brain");
    writeBrain(w, system, now_us);
    w.key("hub");
    writeHub(w, system.diagHub());
    w.endObject();
}

} // namespace navigatr
