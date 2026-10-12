// capture_bundle.cpp

#include "capture/capture_bundle.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <sstream>

#include "capture/zip_writer.h"
#include "inspection/json_writer.h"
#include "translaGATR/frame_codec.h"

namespace navigatr
{
namespace
{

constexpr double kRadToDeg = 57.295779513082320876798;

double wrapDeg(double d) {
    d = std::fmod(d, 360.0);
    if (d <= -180.0) {
        d += 360.0;
    } else if (d > 180.0) {
        d -= 360.0;
    }
    return d;
}

// ---- names ------------------------------------------------------------------

const char* brainOpName(uint8_t op) {
    switch (op) {
    case translagatr::kOpHello: return "HELLO";
    case translagatr::kOpSetPose: return "SET_POSE";
    case translagatr::kOpGetState: return "GET_STATE";
    case translagatr::kOpProfileWrite: return "PROFILE_WRITE";
    case translagatr::kOpProfileApply: return "PROFILE_APPLY";
    case translagatr::kOpReadDoc: return "READ_DOC";
    case translagatr::kOpControl: return "CONTROL";
    case translagatr::kOpPathReport: return "PATH_REPORT";
    case translagatr::kOpReadWheels: return "READ_WHEELS";
    case translagatr::kOpTelemetry: return "TELEMETRY";
    default: return "";
    }
}

const char* brainResultName(uint8_t r) {
    switch (r) {
    case translagatr::kResultOk: return "ok";
    case translagatr::kResultPending: return "pending";
    case translagatr::kResultUnknownSession: return "unknown_session";
    case translagatr::kResultUnsupportedVersion: return "unsupported_version";
    case translagatr::kResultUnsupportedOp: return "unsupported_op";
    case translagatr::kResultInvalidArgument: return "invalid_argument";
    case translagatr::kResultStale: return "stale";
    case translagatr::kResultNotReady: return "not_ready";
    case translagatr::kResultProfileRejected: return "profile_rejected";
    case translagatr::kResultUnavailable: return "unavailable";
    case translagatr::kResultNotStationary: return "not_stationary";
    case translagatr::kResultFailed: return "failed";
    default: return "";
    }
}

const char* pathModeName(uint8_t m) {
    switch (m) {
    case translagatr::kPathNone: return "none";
    case translagatr::kPathDirect: return "direct";
    case translagatr::kPathAvoiding: return "avoiding";
    default: return "";
    }
}

const char* picoImuStateName(uint8_t s) {
    switch (s) {
    case translagatr::kPicoImuDisabled: return "disabled";
    case translagatr::kPicoImuInitializing: return "initializing";
    case translagatr::kPicoImuAligning: return "aligning";
    case translagatr::kPicoImuReady: return "ready";
    case translagatr::kPicoImuRetrying: return "retrying";
    case translagatr::kPicoImuFailed: return "failed";
    default: return "";
    }
}

const char* picoImuReasonName(uint8_t r) {
    switch (r) {
    case translagatr::kPicoImuReasonNone: return "none";
    case translagatr::kPicoImuReasonNoResponse: return "no_response";
    case translagatr::kPicoImuReasonBoot: return "boot";
    case translagatr::kPicoImuReasonFeatures: return "features";
    case translagatr::kPicoImuReasonStream: return "stream";
    default: return "";
    }
}

const char* picoOpName(uint8_t op) {
    switch (op) {
    case 0: return "none";
    case translagatr::kPicoOpConfigure: return "configure";
    case translagatr::kPicoOpReinitImu: return "reinit_imu";
    case translagatr::kPicoOpRestartAcquisition: return "restart_acquisition";
    case translagatr::kPicoOpDiagnostics: return "diagnostics";
    default: return "";
    }
}

const char* picoCommandStatusName(uint8_t s) {
    switch (s) {
    case translagatr::kPicoCommandNone: return "none";
    case translagatr::kPicoCommandRunning: return "running";
    case translagatr::kPicoCommandCompleted: return "completed";
    case translagatr::kPicoCommandFailed: return "failed";
    default: return "";
    }
}

const char* picoDetailName(uint8_t d) {
    switch (d) {
    case translagatr::kPicoDetailNone: return "none";
    case translagatr::kPicoDetailWrongTarget: return "wrong_target";
    case translagatr::kPicoDetailUnknownOp: return "unknown_op";
    case translagatr::kPicoDetailBadBody: return "bad_body";
    case translagatr::kPicoDetailImuAbsent: return "imu_absent";
    case translagatr::kPicoDetailImuDisabled: return "imu_disabled";
    case translagatr::kPicoDetailNoSuchPort: return "no_such_port";
    default: return "";
    }
}

const char* picoFirmwareName(uint8_t f) {
    switch (f) {
    case translagatr::kPicoFirmwareUnknown: return "unknown";
    case translagatr::kPicoFirmwareBno08x: return "bno08x";
    case translagatr::kPicoFirmwareAsm330: return "asm330";
    default: return "";
    }
}

const char* clockName(DiagClock c) {
    switch (c) {
    case DiagClock::kNone: return "";
    case DiagClock::kPiHost: return "pi_host";
    case DiagClock::kPico: return "pico";
    case DiagClock::kBrain: return "brain";
    }
    return "";
}

const char* directionName(DiagDirection d) {
    switch (d) {
    case DiagDirection::kRx: return "rx";
    case DiagDirection::kTxAttempted: return "tx_attempted";
    case DiagDirection::kTxAccepted: return "tx_accepted";
    }
    return "";
}

struct PinColumn {
    uint16_t    bit;
    const char* column;
};
constexpr PinColumn kPins[] = {
    {translagatr::kPicoPinImuInt, "pin_imu_int_level"},
    {translagatr::kPicoPinImuRst, "pin_imu_rst_level"},
    {translagatr::kPicoPinImuWake, "pin_imu_wake_level"},
    {translagatr::kPicoPinImuCs, "pin_imu_cs_level"},
    {translagatr::kPicoPinEnc0A, "pin_enc0_a_level"},
    {translagatr::kPicoPinEnc0B, "pin_enc0_b_level"},
    {translagatr::kPicoPinEnc1A, "pin_enc1_a_level"},
    {translagatr::kPicoPinEnc1B, "pin_enc1_b_level"},
    {translagatr::kPicoPinEnc2A, "pin_enc2_a_level"},
    {translagatr::kPicoPinEnc2B, "pin_enc2_b_level"},
    {translagatr::kPicoPinPiRx, "pin_pi_rx_level"},
};

// ---- columns ------------------------------------------------------------------

const char* const kEventsHeader = "pi_host_us,pi_session,reset_count,kind,source,text";

const char* const kRobotStateHeader =
    "pi_host_us,pi_session,reset_count,source,publication,segment,valid,placed,source_clock,"
    "source_ms,measured_pi_host_ms,source_age_ms,odometry_epoch,anchor_revision,"
    "placement_session,placement_sequence,odom_x_m,odom_y_m,odom_heading_deg,"
    "odom_heading_unwrapped_deg,field_x_m,field_y_m,field_heading_deg,"
    "field_heading_unwrapped_deg,odom_vx_m_s,odom_vy_m_s,yaw_rate_deg_s,confidence,"
    "attitude_status,roll_deg,pitch_deg,stationary,new_measurement";

const char* const kPicoSensorHeader =
    "pi_host_us,pi_session,reset_count,source,source_clock,source_ms,frame_version,boot_id,"
    "acq_epoch,imu_epoch,seq,mask,enc0_counts,enc1_counts,enc2_counts,enc0_delta_counts,"
    "enc1_delta_counts,enc2_delta_counts,gyro_z_deg_s,seq_gap_frames";

const char* const kPicoStatusHeader =
    "pi_host_us,pi_session,reset_count,source,source_clock,source_ms,version,boot_id,acq_epoch,"
    "imu_epoch,imu_state,imu_state_name,imu_reason,imu_reason_name,imu_attempts,imu_enabled,"
    "last_request_id,last_op,last_op_name,last_status,last_status_name,last_detail,"
    "last_detail_name,firmware,firmware_name";

std::string picoDiagHeader() {
    std::string h = "pi_host_us,pi_session,reset_count,source,decoded,version,boot_id,seq,"
                    "firmware,firmware_name";
    for (const PinColumn& p : kPins) {
        h += ',';
        h += p.column;
    }
    h += ",imu_rx,imu_bad,imu_resets,imu_error,reports_ok,reports_rejected,report_age_ms,"
         "link_rx_bad,ticks_skipped,imu_present,flags,payload_hex";
    return h;
}

const char* const kVexImuHeader =
    "pi_host_us,pi_session,reset_count,source,brain_session,request_id,source_clock,source_ms,"
    "valid,flags,rotation_deg,accepted";

const char* const kBrainRequestsHeader =
    "pi_host_us,pi_session,reset_count,source,brain_session,request_id,op,op_name,replied,result,"
    "result_name,request_len_bytes,reply_len_bytes,duplicate";

std::string brainTelemetryHeader() {
    std::string h =
        "pi_host_us,pi_session,reset_count,source,brain_session,decoded,source_clock,source_ms,"
        "flags,attitude_present,roll_deg,pitch_deg,motion_present,command_id,motion_state,"
        "motion_reason,plan_mode,path_segment,path_segment_count,target_valid,target_field_x_m,"
        "target_field_y_m,target_field_heading_deg,cmd_body_vx_m_s,cmd_body_vy_m_s,"
        "cmd_omega_deg_s,cross_track_m,distance_error_m,heading_error_deg,drive_fault,"
        "wheels_present,wheel_count";
    for (int i = 0; i < translagatr::kTelemetryWheelsMax; ++i) {
        h += ",wheel" + std::to_string(i) + "_rpm";
    }
    h += ",payload_hex";
    return h;
}

const char* const kPathsHeader =
    "pi_host_us,pi_session,reset_count,source,brain_session,command_id,mode,mode_name,"
    "point_count,point_index,field_x_m,field_y_m";

const char* const kBytesHeader =
    "pi_host_us,pi_session,reset_count,source,direction,len_bytes,hex";

std::string streamHeader(DiagKind k) {
    switch (k) {
    case DiagKind::kRobotState: return kRobotStateHeader;
    case DiagKind::kPicoSensor: return kPicoSensorHeader;
    case DiagKind::kPicoStatus: return kPicoStatusHeader;
    case DiagKind::kPicoDiag: return picoDiagHeader();
    case DiagKind::kVexImu: return kVexImuHeader;
    case DiagKind::kBrainRequest: return kBrainRequestsHeader;
    case DiagKind::kBrainTelemetry: return brainTelemetryHeader();
    case DiagKind::kPath: return kPathsHeader;
    case DiagKind::kEvent: return kEventsHeader;
    case DiagKind::kBytes: return kBytesHeader;
    }
    return "";
}

// ---- one CSV row --------------------------------------------------------------

class Row
{
public:
    void clear() {
        s_.clear();
        first_ = true;
    }
    Row& none() {
        sep();
        return *this;
    }
    Row& i(int64_t v) {
        sep();
        s_ += std::to_string(v);
        return *this;
    }
    Row& u(uint64_t v) {
        sep();
        s_ += std::to_string(v);
        return *this;
    }
    Row& b(bool v) {
        sep();
        s_ += v ? '1' : '0';
        return *this;
    }
    // Fixed decimals; a non-finite value is missing, and a value that
    // rounds to zero prints without a minus sign.
    Row& f(double v, int decimals) {
        sep();
        if (!std::isfinite(v)) {
            return *this;
        }
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
        const char* p = buf;
        if (buf[0] == '-' && std::strspn(buf + 1, "0.") == std::strlen(buf + 1)) {
            ++p;
        }
        s_ += p;
        return *this;
    }
    Row& s(const std::string& v) {
        sep();
        s_ += csvEscape(v);
        return *this;
    }
    Row& hex(const uint8_t* d, std::size_t n) {
        sep();
        static const char kHex[] = "0123456789ABCDEF";
        for (std::size_t k = 0; k < n; ++k) {
            s_ += kHex[d[k] >> 4];
            s_ += kHex[d[k] & 0x0F];
        }
        return *this;
    }
    const std::string& str() const { return s_; }

private:
    void sep() {
        if (!first_) {
            s_ += ',';
        }
        first_ = false;
    }
    std::string s_;
    bool        first_ = true;
};

// ---- segments -----------------------------------------------------------------

struct Segment {
    std::string domain;
    std::string source;
    std::size_t index = 0; // within its domain and source
    int         nkeys = 0;
    const char* names[4] = {};
    int64_t     values[4] = {};
    int64_t     start_us = 0;
    int64_t     end_us   = 0;
    uint64_t    rows     = 0;
};

class Segments
{
public:
    // The segment index (per domain and source) of a row with these keys.
    std::size_t note(const char* domain, const std::string& source, int nkeys,
                     const char* const* names, const int64_t* values, int64_t t) {
        const std::string key = std::string(domain) + '\x1f' + source;
        const auto        it  = current_.find(key);
        if (it != current_.end()) {
            Segment& s    = segments_[it->second];
            bool     same = true;
            for (int k = 0; k < nkeys; ++k) {
                same = same && s.values[k] == values[k];
            }
            if (same) {
                s.end_us = std::max(s.end_us, t);
                ++s.rows;
                return s.index;
            }
        }
        Segment s;
        s.domain = domain;
        s.source = source;
        s.index  = it == current_.end() ? 0 : segments_[it->second].index + 1;
        s.nkeys  = nkeys;
        for (int k = 0; k < nkeys; ++k) {
            s.names[k]  = names[k];
            s.values[k] = values[k];
        }
        s.start_us = s.end_us = t;
        s.rows                = 1;
        current_[key]         = segments_.size();
        segments_.push_back(s);
        return s.index;
    }
    const std::vector<Segment>& all() const { return segments_; }

private:
    std::map<std::string, std::size_t> current_;
    std::vector<Segment>               segments_;
};

// ---- decoding with the translaGATR codec ----------------------------------------

bool isFrame(const uint8_t* p, std::size_t n) {
    return n >= 2 && p[0] == translagatr::kSync0 && p[1] == translagatr::kSync1;
}

// The Pico telemetry resource keeps the 26-byte payload; a whole frame is
// accepted too.
bool decodeDiag(const DiagPicoDiag& d, translagatr::PicoDiag& out) {
    const auto n = static_cast<uint16_t>(std::min<std::size_t>(d.len, sizeof(d.payload)));
    return isFrame(d.payload, n) ? translagatr::decodePicoDiag(d.payload, n, out)
                                 : translagatr::decodePicoDiagPayload(d.payload, n, out);
}

// The Brain link keeps the 54-byte body; a whole request frame is accepted too.
bool decodeTelemetry(const DiagBrainTelemetry& t, translagatr::BrainTelemetry& out) {
    const auto n = static_cast<uint16_t>(std::min<std::size_t>(t.len, sizeof(t.body)));
    if (!isFrame(t.body, n)) {
        return translagatr::decodeTelemetryBody(t.body, n, out);
    }
    translagatr::BrainRequest req;
    if (!translagatr::decodeBrainRequest(t.body, n, req) ||
        req.version != translagatr::kBrainLinkVersion || req.op != translagatr::kOpTelemetry) {
        return false;
    }
    out = req.telemetry;
    return true;
}

// ---- the writer ---------------------------------------------------------------

// CSV text in chunks: one growing string would copy itself as it doubles and
// hold up to half its capacity unused, the memory peak of a large bundle.
class Text
{
public:
    void append(const char* p, std::size_t n) {
        if (chunks_.empty() || chunks_.back().size() + n > chunks_.back().capacity()) {
            // grow with the text (little slack for a small stream), 1 MiB at most
            const std::size_t cap = std::min<std::size_t>(std::max<std::size_t>(size_, 4096), 1u << 20);
            chunks_.emplace_back();
            chunks_.back().reserve(std::max(n, cap));
        }
        chunks_.back().append(p, n);
        size_ += n;
    }
    void append(const std::string& s) { append(s.data(), s.size()); }
    std::size_t              size() const { return size_; }
    std::vector<std::string> take() {
        size_ = 0;
        return std::move(chunks_);
    }

private:
    std::vector<std::string> chunks_;
    std::size_t              size_ = 0;
};

struct Stream {
    Text        text;
    uint64_t    rows     = 0;
    uint64_t    omitted  = 0;
    int64_t     first_us = -1;
    int64_t     last_us  = -1;
};

struct PicoPrev {
    bool    set = false;
    int64_t key[3] = {};
    uint8_t seq = 0;         // of the previous recorded frame
    bool    has[3] = {};
    int32_t enc[3] = {};
    uint8_t enc_seq[3] = {}; // of the frame that carried enc[e]
};

struct HeadingUnwrap {
    std::size_t segment = static_cast<std::size_t>(-1);
    bool        odom_set = false, field_set = false;
    double      odom_wrapped = 0, odom_unwrapped = 0;
    double      field_wrapped = 0, field_unwrapped = 0;
};

class Writer
{
public:
    explicit Writer(const CaptureBundleInput& in) : in_(in) {
        for (int k = 0; k < kDiagKindCount; ++k) {
            if ((in_.kinds & (1u << k)) != 0) {
                streams_[k].text.append(streamHeader(static_cast<DiagKind>(k)) + "\n");
                used_ += streams_[k].text.size();
            }
        }
    }

    void run() {
        std::vector<const DiagRecord*> sorted;
        if (in_.records != nullptr) {
            sorted.reserve(in_.records->size());
            for (const DiagRecord& r : *in_.records) {
                sorted.push_back(&r);
            }
        }
        // producers stamp before they post, so arrival order can differ by a
        // few microseconds across threads; stable keeps equal stamps in order
        std::stable_sort(sorted.begin(), sorted.end(),
                         [](const DiagRecord* a, const DiagRecord* b) {
                             return a->host_us < b->host_us;
                         });
        std::size_t m = 0;
        for (const DiagRecord* r : sorted) {
            // capture markers go to events.csv in time order with the records
            while (m < in_.markers.size() && in_.markers[m].host_us <= r->host_us) {
                marker(in_.markers[m++]);
            }
            record(*r);
        }
        while (m < in_.markers.size()) {
            marker(in_.markers[m++]);
        }
    }

    Stream&        stream(int k) { return streams_[k]; }
    const DiagKindCounts& mismatched() const { return mismatched_; }
    const Segments& segments() const { return segments_; }
    std::size_t    used() const { return used_; }

private:
    uint64_t resetAt(int64_t t) const {
        uint64_t count = in_.resets.empty() ? 0 : in_.resets.front().second;
        for (const auto& r : in_.resets) {
            if (r.first <= t) {
                count = r.second;
            } else {
                break;
            }
        }
        return count;
    }

    std::string sourceName(uint16_t id) const {
        return id >= 1 && id <= in_.source_names.size() ? in_.source_names[id - 1] : std::string();
    }

    void lead(int64_t t, uint64_t reset) {
        row_.clear();
        row_.i(t).s(in_.pi_session).u(reset);
    }

    // A record row counts toward the stream and the size bound; a capture
    // marker (a handful per bundle) is always written and counted apart.
    void commit(DiagKind kind, int64_t t, bool record = true) {
        Stream& s = streams_[static_cast<int>(kind)];
        if (record && used_ + row_.str().size() + 1 > in_.max_bytes) {
            ++s.omitted;
            return;
        }
        s.text.append(row_.str());
        s.text.append("\n", 1);
        used_ += row_.str().size() + 1;
        if (!record) {
            return;
        }
        ++s.rows;
        if (s.first_us < 0) {
            s.first_us = t;
        }
        s.last_us = t;
    }

    void marker(const CaptureMarker& mk) {
        if ((in_.kinds & diagBit(DiagKind::kEvent)) == 0) {
            return;
        }
        lead(mk.host_us, resetAt(mk.host_us));
        row_.s("capture").s("capture").s(mk.text);
        commit(DiagKind::kEvent, mk.host_us, false);
    }

    void record(const DiagRecord& r) {
        const int k = static_cast<int>(r.kind);
        if (k < 0 || k >= kDiagKindCount || (in_.kinds & (1u << k)) == 0) {
            return;
        }
        if (!payloadMatches(r)) {
            ++mismatched_[k]; // a producer bug must never crash the recorder
            return;
        }
        const uint64_t    reset  = resetAt(r.host_us);
        const std::string source = sourceName(r.source);
        {
            const char*   names[1]  = {"reset_count"};
            const int64_t values[1] = {static_cast<int64_t>(reset)};
            segments_.note("pi", "", 1, names, values, r.host_us);
        }
        const DiagPayload& p = r.payload;
        switch (r.kind) {
        case DiagKind::kRobotState: robotState(r, *std::get_if<DiagRobotState>(&p), reset, source); break;
        case DiagKind::kPicoSensor: picoSensor(r, *std::get_if<DiagPicoSensor>(&p), reset, source); break;
        case DiagKind::kPicoStatus:
            picoStatus(r, std::get_if<DiagPicoStatus>(&p)->status, reset, source);
            break;
        case DiagKind::kPicoDiag: picoDiag(r, *std::get_if<DiagPicoDiag>(&p), reset, source); break;
        case DiagKind::kVexImu: vexImu(r, *std::get_if<DiagVexImu>(&p), reset, source); break;
        case DiagKind::kBrainRequest:
            brainRequest(r, *std::get_if<DiagBrainRequest>(&p), reset, source);
            break;
        case DiagKind::kBrainTelemetry:
            brainTelemetry(r, *std::get_if<DiagBrainTelemetry>(&p), reset, source);
            break;
        case DiagKind::kPath: path(r, *std::get_if<DiagPath>(&p), reset, source); break;
        case DiagKind::kEvent: {
            const DiagEvent& e = *std::get_if<DiagEvent>(&p);
            lead(r.host_us, reset);
            const char*      end = std::find(e.text, e.text + sizeof(e.text), '\0');
            row_.s("runtime").s(source).s(std::string(e.text, end));
            commit(r.kind, r.host_us);
            break;
        }
        case DiagKind::kBytes: {
            const DiagBytes&  b = *std::get_if<DiagBytes>(&p);
            const std::size_t n = std::min<std::size_t>(b.len, sizeof(b.data));
            lead(r.host_us, reset);
            row_.s(source).s(directionName(b.dir)).u(n).hex(b.data, n);
            commit(r.kind, r.host_us);
            break;
        }
        }
    }

    static bool payloadMatches(const DiagRecord& r) {
        const DiagPayload& p = r.payload;
        switch (r.kind) {
        case DiagKind::kRobotState: return std::holds_alternative<DiagRobotState>(p);
        case DiagKind::kPicoSensor: return std::holds_alternative<DiagPicoSensor>(p);
        case DiagKind::kPicoStatus: return std::holds_alternative<DiagPicoStatus>(p);
        case DiagKind::kPicoDiag: return std::holds_alternative<DiagPicoDiag>(p);
        case DiagKind::kVexImu: return std::holds_alternative<DiagVexImu>(p);
        case DiagKind::kBrainRequest: return std::holds_alternative<DiagBrainRequest>(p);
        case DiagKind::kBrainTelemetry: return std::holds_alternative<DiagBrainTelemetry>(p);
        case DiagKind::kPath: return std::holds_alternative<DiagPath>(p);
        case DiagKind::kEvent: return std::holds_alternative<DiagEvent>(p);
        case DiagKind::kBytes: return std::holds_alternative<DiagBytes>(p);
        }
        return false;
    }

    void brainSegment(uint32_t session, const std::string& source, int64_t t) {
        const char*   names[1]  = {"brain_session"};
        const int64_t values[1] = {static_cast<int64_t>(session)};
        segments_.note("brain", source, 1, names, values, t);
    }

    void robotState(const DiagRecord& r, const DiagRobotState& s, uint64_t reset,
                    const std::string& source) {
        const char*   names[3]  = {"reset_count", "odometry_epoch", "anchor_revision"};
        const int64_t values[3] = {static_cast<int64_t>(reset),
                                   static_cast<int64_t>(s.odometry_epoch),
                                   static_cast<int64_t>(s.anchor_revision)};
        const std::size_t seg = segments_.note("robot", source, 3, names, values, r.host_us);
        if (seg != unwrap_.segment) {
            unwrap_         = HeadingUnwrap{};
            unwrap_.segment = seg;
        }
        const bool   placed = s.valid && s.initialized;
        const double odom_h = wrapDeg(s.odom_heading_rad * kRadToDeg);
        const double field_h = wrapDeg(s.field_heading_rad * kRadToDeg);
        if (s.valid) {
            unwrap_.odom_unwrapped =
                unwrap_.odom_set ? unwrap_.odom_unwrapped + wrapDeg(odom_h - unwrap_.odom_wrapped)
                                 : odom_h;
            unwrap_.odom_wrapped = odom_h;
            unwrap_.odom_set     = true;
        }
        if (placed) {
            unwrap_.field_unwrapped =
                unwrap_.field_set
                    ? unwrap_.field_unwrapped + wrapDeg(field_h - unwrap_.field_wrapped)
                    : field_h;
            unwrap_.field_wrapped = field_h;
            unwrap_.field_set     = true;
        }

        lead(r.host_us, reset);
        row_.s(source).u(s.publication).u(seg).b(s.valid).b(s.initialized);
        if (s.measured_clock != DiagClock::kNone) {
            row_.s(clockName(s.measured_clock)).i(s.measured_ms);
        } else {
            row_.none().none();
        }
        if (s.measured_host_ms >= 0) {
            row_.i(s.measured_host_ms).f(static_cast<double>(r.host_us) / 1000.0 -
                                             static_cast<double>(s.measured_host_ms),
                                         3);
        } else {
            row_.none().none();
        }
        row_.u(s.odometry_epoch).u(s.anchor_revision);
        if (s.initialized) {
            row_.u(s.placement_session).u(s.placement_sequence);
        } else {
            row_.none().none();
        }
        if (s.valid) {
            row_.f(s.odom_x_m, 6).f(s.odom_y_m, 6).f(odom_h, 4).f(unwrap_.odom_unwrapped, 4);
        } else {
            row_.none().none().none().none();
        }
        if (placed) {
            row_.f(s.field_x_m, 6).f(s.field_y_m, 6).f(field_h, 4).f(unwrap_.field_unwrapped, 4);
        } else {
            row_.none().none().none().none();
        }
        if (s.valid) {
            row_.f(s.vx_m_s, 5).f(s.vy_m_s, 5).f(s.yaw_rate_rad_s * kRadToDeg, 4).f(s.confidence, 4);
        } else {
            row_.none().none().none().none();
        }
        if (s.attitude_valid) {
            row_.s("measured").f(s.roll_rad * kRadToDeg, 4).f(s.pitch_rad * kRadToDeg, 4);
        } else {
            row_.s(s.attitude_assumed_level ? "assumed_level" : "unavailable").none().none();
        }
        // localization publishes every cycle; the estimator's own flag says
        // which rows carry a new measurement and which repeat the held pose
        row_.b(s.stationary).b(s.advanced);
        commit(r.kind, r.host_us);
    }

    void picoSensor(const DiagRecord& r, const DiagPicoSensor& p, uint64_t reset,
                    const std::string& source) {
        const bool    v2        = p.version >= 2;
        const char*   names[4]  = {"identity", "boot_id", "acq_epoch", "imu_epoch"};
        const int64_t values[4] = {v2 ? 1 : 0, v2 ? p.boot_id : 0, v2 ? p.acq_epoch : 0,
                                   v2 ? p.imu_epoch : 0};
        segments_.note("pico", source, 4, names, values, r.host_us);

        PicoPrev&  prev     = pico_prev_[r.source];
        const bool same_acq = prev.set && prev.key[0] == values[0] && prev.key[1] == values[1] &&
                              prev.key[2] == values[2];
        lead(r.host_us, reset);
        row_.s(source).s("pico").u(p.stamp_ms).u(p.version);
        if (v2) {
            row_.u(p.boot_id).u(p.acq_epoch).u(p.imu_epoch);
        } else {
            row_.none().none().none();
        }
        row_.u(p.seq).u(p.mask);
        bool has[3];
        for (int e = 0; e < 3; ++e) {
            has[e] = (p.mask & (1u << e)) != 0;
            if (has[e]) {
                row_.i(p.enc[e]);
            } else {
                row_.none();
            }
        }
        // a change per frame: only against the port's value in the frame
        // right before (seq + 1) of the same acquisition (boot and
        // acq_epoch). Across a restart the counters were rebased, and across
        // lost frames the step is not one frame's change; the counts stay.
        for (int e = 0; e < 3; ++e) {
            if (has[e] && same_acq && prev.has[e] &&
                static_cast<uint8_t>(p.seq - prev.enc_seq[e]) == 1) {
                row_.i(static_cast<int64_t>(p.enc[e]) - static_cast<int64_t>(prev.enc[e]));
            } else {
                row_.none();
            }
        }
        if ((p.mask & translagatr::kSensorGyroZ) != 0) {
            row_.f(p.gyro_z_mdps / 1000.0, 3);
        } else {
            row_.none();
        }
        // frames sent since the previous recorded one of this acquisition
        // that are not in the file, modulo 256 (seq is u8)
        if (same_acq) {
            row_.u(static_cast<uint8_t>(p.seq - prev.seq - 1));
        } else {
            row_.none();
        }
        commit(r.kind, r.host_us);
        prev.set    = true;
        prev.key[0] = values[0];
        prev.key[1] = values[1];
        prev.key[2] = values[2];
        prev.seq    = p.seq;
        for (int e = 0; e < 3; ++e) {
            if (has[e]) {
                prev.enc[e]     = p.enc[e];
                prev.enc_seq[e] = p.seq;
            }
            prev.has[e] = has[e] || (same_acq && prev.has[e]);
        }
    }

    void picoStatus(const DiagRecord& r, const translagatr::PicoStatus& s, uint64_t reset,
                    const std::string& source) {
        lead(r.host_us, reset);
        row_.s(source).s("pico").u(s.uptime_ms).u(s.version).u(s.boot_id).u(s.acq_epoch).u(s.imu_epoch);
        row_.u(s.imu_state).s(picoImuStateName(s.imu_state));
        row_.u(s.imu_reason).s(picoImuReasonName(s.imu_reason));
        row_.u(s.imu_attempts).b((s.flags & translagatr::kPicoImuEnabled) != 0);
        if (s.last_request_id != 0) {
            row_.u(s.last_request_id).u(s.last_op).s(picoOpName(s.last_op));
            row_.u(s.last_status).s(picoCommandStatusName(s.last_status));
            row_.u(s.last_detail).s(picoDetailName(s.last_detail));
        } else {
            row_.none().none().none().none().none().none().none();
        }
        row_.u(s.firmware).s(picoFirmwareName(s.firmware));
        commit(r.kind, r.host_us);
    }

    void picoDiag(const DiagRecord& r, const DiagPicoDiag& d, uint64_t reset,
                  const std::string& source) {
        translagatr::PicoDiag diag;
        const bool            ok = decodeDiag(d, diag);
        lead(r.host_us, reset);
        row_.s(source).b(ok);
        if (ok) {
            row_.u(diag.version).u(diag.boot_id).u(diag.seq).u(diag.firmware);
            row_.s(picoFirmwareName(diag.firmware));
            for (const PinColumn& p : kPins) {
                if ((diag.pins_known & p.bit) != 0) {
                    row_.b((diag.pins & p.bit) != 0);
                } else {
                    row_.none();
                }
            }
            row_.u(diag.imu_rx).u(diag.imu_bad).u(diag.imu_resets).i(diag.imu_error);
            row_.u(diag.reports_ok).u(diag.reports_rejected);
            if (diag.report_age_ms != 0xFFFF) {
                row_.u(diag.report_age_ms);
            } else {
                row_.none();
            }
            row_.u(diag.link_rx_bad).u(diag.ticks_skipped);
            row_.b((diag.flags & translagatr::kPicoDiagImuPresent) != 0).u(diag.flags);
        } else {
            for (int c = 0; c < 5 + static_cast<int>(sizeof(kPins) / sizeof(kPins[0])) + 11; ++c) {
                row_.none();
            }
        }
        row_.hex(d.payload, std::min<std::size_t>(d.len, sizeof(d.payload)));
        commit(r.kind, r.host_us);
    }

    void vexImu(const DiagRecord& r, const DiagVexImu& v, uint64_t reset, const std::string& source) {
        brainSegment(v.session, source, r.host_us);
        const bool valid = (v.flags & translagatr::kBenchImuValid) != 0;
        lead(r.host_us, reset);
        row_.s(source).u(v.session).u(v.request_id);
        if (valid) {
            row_.s("brain").u(v.stamp_ms);
        } else {
            row_.none().none();
        }
        row_.b(valid).u(v.flags);
        if (valid) {
            row_.f(v.rotation_mdeg / 1000.0, 3);
        } else {
            row_.none();
        }
        row_.b(v.accepted);
        commit(r.kind, r.host_us);
    }

    void brainRequest(const DiagRecord& r, const DiagBrainRequest& q, uint64_t reset,
                      const std::string& source) {
        brainSegment(q.session, source, r.host_us);
        const bool replied = q.reply_len != 0;
        lead(r.host_us, reset);
        row_.s(source).u(q.session).u(q.request_id).u(q.op).s(brainOpName(q.op)).b(replied);
        if (replied) {
            row_.u(q.result).s(brainResultName(q.result));
        } else {
            row_.none().none();
        }
        row_.u(q.request_len);
        if (replied) {
            row_.u(q.reply_len);
        } else {
            row_.none();
        }
        row_.b(q.duplicate);
        commit(r.kind, r.host_us);
    }

    void brainTelemetry(const DiagRecord& r, const DiagBrainTelemetry& t, uint64_t reset,
                        const std::string& source) {
        brainSegment(t.session, source, r.host_us);
        translagatr::BrainTelemetry v;
        const bool                  ok = decodeTelemetry(t, v);
        lead(r.host_us, reset);
        row_.s(source).u(t.session).b(ok);
        if (ok) {
            const bool att    = (v.flags & translagatr::kTelemetryAttitude) != 0;
            const bool motion = (v.flags & translagatr::kTelemetryMotion) != 0;
            const bool wheels = (v.flags & translagatr::kTelemetryWheels) != 0;
            row_.s("brain").u(v.stamp_ms).u(v.flags).b(att);
            if (att) {
                row_.f(v.roll_cdeg / 100.0, 2).f(v.pitch_cdeg / 100.0, 2);
            } else {
                row_.none().none();
            }
            row_.b(motion);
            if (motion) {
                // target_* holds a destination only with its own flag bit
                const bool target = (v.flags & translagatr::kTelemetryTarget) != 0;
                row_.u(v.command_id).u(v.motion_state).u(v.motion_reason).u(v.plan_mode);
                row_.u(v.segment).u(v.segment_count).b(target);
                if (target) {
                    row_.f(v.target_x_mm / 1000.0, 3).f(v.target_y_mm / 1000.0, 3);
                    row_.f(v.target_heading_cdeg / 100.0, 2);
                } else {
                    row_.none().none().none();
                }
                row_.f(v.cmd_vx_mm_s / 1000.0, 3).f(v.cmd_vy_mm_s / 1000.0, 3);
                row_.f(v.cmd_omega_cdeg_s / 100.0, 2);
                row_.f(v.cross_track_mm / 1000.0, 3).f(v.distance_error_mm / 1000.0, 3);
                row_.f(v.heading_error_cdeg / 100.0, 2).u(v.drive_fault);
            } else {
                for (int c = 0; c < 17; ++c) {
                    row_.none();
                }
            }
            row_.b(wheels);
            if (wheels) {
                row_.u(v.wheel_count);
            } else {
                row_.none();
            }
            for (int w = 0; w < translagatr::kTelemetryWheelsMax; ++w) {
                if (wheels && w < v.wheel_count) {
                    row_.f(v.wheel_rpm_x10[w] / 10.0, 1);
                } else {
                    row_.none();
                }
            }
        } else {
            for (int c = 0; c < 26 + translagatr::kTelemetryWheelsMax; ++c) {
                row_.none();
            }
        }
        row_.hex(t.body, std::min<std::size_t>(t.len, sizeof(t.body)));
        commit(r.kind, r.host_us);
    }

    void path(const DiagRecord& r, const DiagPath& p, uint64_t reset, const std::string& source) {
        brainSegment(p.session, source, r.host_us);
        const std::size_t n = std::min<std::size_t>(p.count, translagatr::kPathReportMaxPoints);
        const auto        head = [&] {
            lead(r.host_us, reset);
            row_.s(source).u(p.session).u(p.command_id).u(p.mode).s(pathModeName(p.mode)).u(p.count);
        };
        if (n == 0) {
            head();
            row_.none().none().none();
            commit(r.kind, r.host_us);
            return;
        }
        for (std::size_t i = 0; i < n; ++i) {
            head();
            row_.u(i).f(p.points[i].x_mm / 1000.0, 3).f(p.points[i].y_mm / 1000.0, 3);
            commit(r.kind, r.host_us);
        }
    }

    const CaptureBundleInput& in_;
    Stream                    streams_[kDiagKindCount];
    Segments                  segments_;
    Row                       row_;
    std::size_t               used_ = 0;
    DiagKindCounts            mismatched_{}; // kind and payload disagree: skipped
    HeadingUnwrap             unwrap_;
    std::map<uint16_t, PicoPrev> pico_prev_;
};

// ---- metadata ------------------------------------------------------------------

void writeCounts(JsonWriter& w, const char* key, const DiagKindCounts& c) {
    w.key(key);
    w.beginObject();
    for (int k = 0; k < kDiagKindCount; ++k) {
        w.field(diagKindName(static_cast<DiagKind>(k)), c[k]);
    }
    w.endObject();
}

void writeConventions(JsonWriter& w) {
    w.key("conventions");
    w.beginObject();
    w.key("frames");
    w.beginObject();
    w.field("odometry", "local odometry frame O: continuous within one odometry_epoch; its "
                        "origin is where localization started or last reinitialized");
    w.field("field", "field frame F: odometry re-anchored by a placement; changes with "
                     "anchor_revision; empty while the robot is not placed");
    w.field("body", "robot body frame: origin the reported robot point, +x forward, +y left, "
                    "+z up");
    w.endObject();
    w.field("axes", "+x forward, +y left, +z up, right handed");
    w.field("headings", "degrees, counterclockwise positive from +x, wrapped to (-180, 180]; "
                        "*_unwrapped_deg is continuous within one robot segment");
    w.field("attitude", "roll about +x, positive left side up; pitch about +y, positive nose "
                        "down; degrees; only when attitude_status is measured");
    w.field("height", "not estimated: the robot is on the floor plane");
    w.key("clocks");
    w.beginObject();
    w.field("pi_host_us", "Pi steady clock, microseconds since the naviGATR process started; "
                          "the only clock shared by every file");
    w.field("pi_host_ms", "the same Pi clock in milliseconds");
    w.field("pico", "Pico milliseconds since its boot (per boot_id); never subtract from the "
                    "Pi or Brain clocks");
    w.field("brain", "Brain program milliseconds since its start (per brain_session); never "
                     "subtract from the Pi or Pico clocks");
    w.endObject();
    w.field("missing", "an empty CSV field is a missing value, never zero");
    w.field("booleans", "0 or 1");
    w.field("line_end", "\\n");
    w.endObject();
}

void writeSegments(JsonWriter& w, const Segments& segments) {
    w.key("segments");
    w.beginArray();
    for (const Segment& s : segments.all()) {
        w.beginObject();
        w.field("domain", s.domain);
        if (!s.source.empty()) {
            w.field("source", s.source);
        }
        w.field("index", static_cast<uint64_t>(s.index));
        for (int k = 0; k < s.nkeys; ++k) {
            w.field(s.names[k], s.values[k]);
        }
        w.field("first_pi_host_us", s.start_us);
        w.field("last_pi_host_us", s.end_us);
        w.field("rows", s.rows);
        w.endObject();
    }
    w.endArray();
}

std::string metadataJson(const CaptureBundleInput& in, Writer& writer,
                         const CaptureBundleOutput& out) {
    const DiagKindCounts& rows    = out.rows;
    const DiagKindCounts& omitted = out.rows_omitted;
    JsonWriter w;
    w.beginObject();
    w.field("schema", kCaptureSchema);
    w.field("id", in.id);
    w.key("created");
    w.beginObject();
    w.field("pi_host_us", in.trigger_us);
    if (in.created_unix_ms >= 0) {
        w.field("unix_ms", in.created_unix_ms);
        w.field("unix_ms_note", "Pi system clock at the trigger; wrong if the Pi has no "
                                "network time or RTC");
    } else {
        w.fieldNull("unix_ms");
    }
    w.endObject();

    w.key("pi");
    w.beginObject();
    w.field("session", in.pi_session);
    w.key("resets");
    w.beginArray();
    for (const auto& r : in.resets) {
        w.beginObject();
        w.field("from_pi_host_us", r.first);
        w.field("reset_count", r.second);
        w.endObject();
    }
    w.endArray();
    w.endObject();

    // the host keys when the bundle was built, and as they were at the window
    // start: a profile, calibration or Pico boot that changed during the
    // capture shows in both
    if (in.write_host) {
        in.write_host(w);
        w.field("host_sampled_pi_host_us", in.host_end_us);
    }
    w.key("host_at_start");
    if (!in.host_at_start.empty()) {
        w.raw(in.host_at_start);
    } else {
        w.null();
    }

    w.key("trigger");
    w.beginObject();
    w.field("reason", in.reason);
    w.field("requester", in.requester);
    if (!in.detail.empty()) {
        w.field("detail", in.detail);
    }
    w.field("pre_s_requested", in.pre_s_requested);
    w.field("post_s_requested", in.post_s_requested);
    w.field("pre_s", in.pre_s);
    w.field("post_s", in.post_s);
    // what the rolling window actually held before the trigger
    const int64_t pre_from =
        in.rolling_oldest_us >= 0 ? std::max(in.window_start_us, in.rolling_oldest_us)
                                  : in.trigger_us;
    w.field("pre_s_actual", static_cast<double>(in.trigger_us - pre_from) * 1e-6);
    w.field("post_s_actual", static_cast<double>(in.end_us - in.trigger_us) * 1e-6);
    w.field("window_start_pi_host_us", in.window_start_us);
    w.field("trigger_pi_host_us", in.trigger_us);
    w.field("end_pi_host_us", in.end_us);
    // truncated: a capture bound left records out, each cause in
    // truncated_by; stop_reason says only why recording stopped. Hub ring
    // drops are in drops, as are the counts per stream.
    w.field("truncated", out.truncated);
    w.key("truncated_by");
    w.beginArray();
    for (const std::string& why : out.truncated_by) {
        w.value(why);
    }
    w.endArray();
    w.field("stop_reason", in.stop_reason);
    w.endObject();

    w.key("streams");
    w.beginObject();
    for (int k = 0; k < kDiagKindCount; ++k) {
        const auto kind     = static_cast<DiagKind>(k);
        const bool selected = (in.kinds & (1u << k)) != 0;
        Stream&    s        = writer.stream(k);
        w.key(diagKindName(kind));
        w.beginObject();
        w.field("selected", selected);
        if (selected) {
            w.field("file", captureStreamFile(kind));
        } else {
            w.fieldNull("file");
        }
        w.field("rows", rows[k]);
        if (selected && s.first_us >= 0) {
            w.field("first_pi_host_us", s.first_us);
            w.field("last_pi_host_us", s.last_us);
        } else {
            w.fieldNull("first_pi_host_us");
            w.fieldNull("last_pi_host_us");
        }
        w.endObject();
    }
    w.endObject();

    w.key("drops");
    w.beginObject();
    writeCounts(w, "hub_ring_full", in.hub_dropped);
    writeCounts(w, "capture_limit", in.limit_dropped);
    writeCounts(w, "not_selected", in.not_selected);
    writeCounts(w, "bundle_size_limit", omitted);
    writeCounts(w, "malformed", writer.mismatched());
    writeCounts(w, "hub_posted", in.hub_posted);
    w.field("rolling_evicted", in.rolling_evicted);
    w.field("fault_notices_dropped", in.faults_dropped);
    w.field("hub_capacity", static_cast<uint64_t>(in.hub_capacity));
    w.field("record_cap", static_cast<uint64_t>(in.record_cap));
    w.field("notes",
            "hub_ring_full: records a producer posted while the hub ring was full (the "
            "recorder fell behind), counted from the window start; capture_limit: records "
            "refused because the capture reached max_records or max_mb; not_selected: records "
            "seen during the window of streams not chosen for this capture; bundle_size_limit: "
            "rows left out to keep the CSV text under max_mb; malformed: records whose payload "
            "did not match their stream (a producer bug), skipped; hub_posted: everything producers "
            "posted during the window, selected or not; rolling_evicted: records stamped inside "
            "this capture's window that the record bound pushed out of the rolling window before "
            "the trigger, so the pre-window misses them (counted per millisecond of their stamp: "
            "the window's first millisecond counts whole). Raw bytes are produced only while a "
            "capture selects them.");
    w.endObject();

    writeSegments(w, writer.segments());
    writeConventions(w);

    w.key("markers");
    w.beginArray();
    for (const CaptureMarker& m : in.markers) {
        w.beginObject();
        w.field("pi_host_us", m.host_us);
        w.field("text", m.text);
        w.endObject();
    }
    w.endArray();
    w.endObject();
    return w.take();
}

std::string readme(const CaptureBundleInput& in) {
    std::ostringstream o;
    o << "gatr2.capture/1 bundle " << in.id << "\n"
      << "=============================================\n\n"
      << "A short diagnostic recording from the naviGATR Pi runtime. Every CSV holds the\n"
      << "records as they happened: nothing is resampled onto a common time grid, so\n"
      << "join files on pi_host_us (Pi steady clock, microseconds) when you need to.\n"
      << "metadata.json holds the trigger, window, streams, drops, segments, the\n"
      << "configuration and robot profile, and the conventions below.\n"
      << "configuration, profile, localization and pico_link at its top level were\n"
      << "read when the bundle was built (host_sampled_pi_host_us, after the post\n"
      << "window); host_at_start holds the same keys read at or before the window\n"
      << "start (its sampled_pi_host_us). A profile, calibration or Pico boot that\n"
      << "changed during the capture shows in both; events.csv says when.\n\n"
      << "Conventions\n"
      << "- Units are in the column names: _m metres, _deg degrees, _deg_s degrees per\n"
      << "  second, _m_s metres per second, _ms milliseconds, _us microseconds.\n"
      << "- An empty field means missing (not measured, not valid, not applicable). It\n"
      << "  never means zero. Booleans are 0/1. Line ends are \\n.\n"
      << "- +x forward, +y left, headings counterclockwise from +x, wrapped to\n"
      << "  (-180, 180]. *_unwrapped_deg is continuous within one robot segment only.\n"
      << "- Roll about +x (left side up positive), pitch about +y (nose down\n"
      << "  positive), only when attitude_status is measured. Height is never estimated.\n"
      << "- Clocks: pi_host_us/_ms is the Pi clock shared by every file. source_ms with\n"
      << "  source_clock pico is Pico milliseconds since its boot (per boot_id); with\n"
      << "  source_clock brain it is Brain milliseconds since the program started (per\n"
      << "  brain_session). Never subtract timestamps of different clocks.\n"
      << "- Identifiers on every row say which segment it belongs to: pi_session and\n"
      << "  reset_count (Pi process and in-process resets), odometry_epoch and\n"
      << "  anchor_revision (pose continuity and placement), boot_id, acq_epoch and\n"
      << "  imu_epoch (Pico), brain_session (Brain link). A change in any of them is a\n"
      << "  discontinuity: do not join a trajectory across it. metadata.json lists the\n"
      << "  segments.\n"
      << "- Data loss is in metadata.json drops. Rows missing there were not recorded.\n"
      << "  trigger.truncated is true when a capture bound left records out;\n"
      << "  trigger.truncated_by names each (record_limit, memory_limit, shutdown,\n"
      << "  bundle_size_limit). stop_reason only says why recording stopped.\n\n"
      << "Files (present only when the stream was selected)\n\n"
      << "events.csv: runtime lifecycle events (kind runtime, text truncated to 119\n"
      << "  bytes) and capture markers (kind capture: trigger, end, Pi resets).\n  "
      << kEventsHeader << "\n\n"
      << "robot_state.csv: every localization publication. segment counts robot\n"
      << "  segments (reset_count, odometry_epoch, anchor_revision). Pose fields are\n"
      << "  empty while not valid; field_* are empty while not placed. source_age_ms\n"
      << "  is pi_host_us/1000 minus measured_pi_host_ms (both Pi clock). odom_v* are\n"
      << "  in the odometry frame. confidence is a producer score, not a covariance.\n"
      << "  Localization publishes every loop cycle, so rows repeat the pose between\n"
      << "  measurements: new_measurement is 1 when the estimator took a new\n"
      << "  measurement for this publication, 0 when the row repeats the held pose\n"
      << "  (loop cycle without one, reset, continuity loss).\n  "
      << kRobotStateHeader << "\n\n"
      << "pico_sensor.csv: every decoded Pico sensor frame. enc*_counts are raw\n"
      << "  counts, empty when the frame did not carry that port. enc*_delta_counts is\n"
      << "  one frame's change: against the frame right before (seq - 1) of the same\n"
      << "  boot and acquisition epoch, empty otherwise. seq_gap_frames counts the\n"
      << "  frames between the previous recorded frame of that acquisition and this\n"
      << "  one that are not in the file (0 consecutive; modulo 256, so a loss of a\n"
      << "  multiple of 256 cannot be seen; empty for its first frame). Across a gap, use\n"
      << "  the enc*_counts difference: it spans more than one frame.\n"
      << "  Zero change cannot tell a stationary encoder from a disconnected one.\n"
      << "  gyro_z_deg_s is raw (bias not removed). frame_version 1 has no identity.\n  "
      << kPicoSensorHeader << "\n\n"
      << "pico_status.csv: every Pico status frame (IMU state, last command).\n  "
      << kPicoStatusHeader << "\n\n"
      << "pico_diag.csv: every Pico diagnostic frame. pin_*_level are digital logic\n"
      << "  levels read back from the pad when the frame was built (1 HIGH, 0 LOW,\n"
      << "  empty when not sampled). They are not voltages, a sample can miss fast\n"
      << "  transitions, and UART RX idles HIGH. decoded 0 means the payload could not\n"
      << "  be decoded; payload_hex always holds the raw bytes.\n  "
      << picoDiagHeader() << "\n\n"
      << "vex_imu.csv: every Brain VEX IMU sample carried by GET_STATE. rotation_deg is\n"
      << "  continuous (not wrapped), counterclockwise positive. accepted 1 means the\n"
      << "  Pi took it as a new sample.\n  "
      << kVexImuHeader << "\n\n"
      << "brain_requests.csv: every Brain request with the result the Pi answered.\n"
      << "  replied 0 means nothing was sent back.\n  "
      << kBrainRequestsHeader << "\n\n"
      << "brain_telemetry.csv: every Brain TELEMETRY report (display and recording\n"
      << "  only; localization never reads it). Groups whose *_present is 0 are empty.\n"
      << "  target_valid is flags bit 3: target_field_* hold a resolved destination\n"
      << "  only when it is 1 and are empty when it is 0.\n"
      << "  motion_state, motion_reason, plan_mode and drive_fault are the actuGATR and\n"
      << "  investiGATR enum values (docs/actugatr.md). cmd_body_* is the commanded\n"
      << "  chassis velocity in the body frame; wheel*_rpm are motor velocity targets.\n  "
      << brainTelemetryHeader() << "\n\n"
      << "paths.csv: every PATH_REPORT, one row per vertex (field frame); a report\n"
      << "  with no points is one row with empty point columns.\n  "
      << kPathsHeader << "\n\n"
      << "transport_bytes.csv: raw link bytes in hex, only when selected. direction\n"
      << "  rx is bytes read, tx_accepted bytes the link reported written, tx_attempted\n"
      << "  bytes handed to write() that were not reported written.\n  "
      << kBytesHeader << "\n";
    return o.str();
}

} // namespace

const char* captureStreamFile(DiagKind k) {
    switch (k) {
    case DiagKind::kRobotState: return "robot_state.csv";
    case DiagKind::kPicoSensor: return "pico_sensor.csv";
    case DiagKind::kPicoStatus: return "pico_status.csv";
    case DiagKind::kPicoDiag: return "pico_diag.csv";
    case DiagKind::kVexImu: return "vex_imu.csv";
    case DiagKind::kBrainRequest: return "brain_requests.csv";
    case DiagKind::kBrainTelemetry: return "brain_telemetry.csv";
    case DiagKind::kPath: return "paths.csv";
    case DiagKind::kEvent: return "events.csv";
    case DiagKind::kBytes: return "transport_bytes.csv";
    }
    return "";
}

bool parseCaptureStreams(const std::string& text, uint32_t& kinds, std::string& bad) {
    kinds = 0;
    std::stringstream in(text);
    std::string       item;
    bool              any = false;
    while (std::getline(in, item, ',')) {
        const auto b = item.find_first_not_of(" \t");
        const auto e = item.find_last_not_of(" \t");
        if (b == std::string::npos) {
            continue;
        }
        item = item.substr(b, e - b + 1);
        any  = true;
        if (item == "default") {
            kinds |= kCaptureDefaultKinds;
            continue;
        }
        if (item == "all") {
            kinds |= kDiagAllKinds;
            continue;
        }
        bool seen = false;
        for (int k = 0; k < kDiagKindCount; ++k) {
            if (item == diagKindName(static_cast<DiagKind>(k))) {
                kinds |= 1u << k;
                seen = true;
            }
        }
        if (!seen) {
            bad = item;
            return false;
        }
    }
    if (!any) {
        kinds = kCaptureDefaultKinds;
    }
    return true;
}

std::string csvEscape(const std::string& s) {
    if (s.find_first_of(",\"\r\n") == std::string::npos) {
        return s;
    }
    std::string out = "\"";
    for (const char c : s) {
        if (c == '"') {
            out += '"';
        }
        out += c;
    }
    out += '"';
    return out;
}

bool buildCaptureBundle(const CaptureBundleInput& in, CaptureBundleOutput& out, std::string& err) {
    out = CaptureBundleOutput{};
    Writer writer(in);
    writer.run();
    if (in.rows_written) {
        in.rows_written(); // nothing below reads the records
    }
    for (int k = 0; k < kDiagKindCount; ++k) {
        out.rows[k]         = writer.stream(k).rows;
        out.rows_omitted[k] = writer.stream(k).omitted;
    }
    out.csv_bytes = writer.used();
    // the metadata and the status report one verdict
    bool size_limited = false;
    for (uint64_t n : out.rows_omitted) {
        size_limited = size_limited || n != 0;
    }
    if (in.truncated) {
        out.truncated_by.push_back(in.stop_reason.empty() ? "unknown" : in.stop_reason);
    }
    if (size_limited) {
        out.truncated_by.push_back("bundle_size_limit");
    }
    out.truncated = !out.truncated_by.empty();

    std::vector<ZipEntry> entries;
    entries.push_back({"metadata.json", metadataJson(in, writer, out)});
    entries.push_back({"README.txt", readme(in)});
    for (int k = 0; k < kDiagKindCount; ++k) {
        if ((in.kinds & (1u << k)) != 0) {
            entries.push_back({captureStreamFile(static_cast<DiagKind>(k)), std::string(),
                               writer.stream(k).text.take()});
        }
    }
    for (const ZipEntry& e : entries) {
        out.files.push_back(e.name);
    }
    return buildStoredZip(entries, zipTimeFromUnixMs(in.created_unix_ms), out.zip, err);
}

} // namespace navigatr
