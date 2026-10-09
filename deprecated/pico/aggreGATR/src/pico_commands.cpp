// pico_commands.cpp

#include "pico_commands.h"

#include "frame_codec.h"

namespace pilink
{
namespace
{

uint16_t rd16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

// Header of a command frame the codec refused because its body length is
// wrong for the op. False unless the envelope, CRC and version are good.
bool readHeader(const uint8_t* frame, uint16_t len, translagatr::PicoCommand& out) {
    if (len < translagatr::kLinkEnvelopeLen + translagatr::kPicoCommandHeaderLen) {
        return false;
    }
    if (frame[0] != translagatr::kSync0 || frame[1] != translagatr::kSync1 ||
        frame[2] != translagatr::kFramePicoCommand || frame[3] + translagatr::kLinkEnvelopeLen != len) {
        return false;
    }
    if (translagatr::crc16(frame + 2, static_cast<uint16_t>(len - 4)) != rd16(frame + len - 2)) {
        return false;
    }
    const uint8_t* p = frame + 4;
    if (p[0] != translagatr::kPicoLinkVersion) {
        return false;
    }
    out                = translagatr::PicoCommand{};
    out.version        = p[0];
    out.op             = p[1];
    out.request_id     = rd16(p + 2);
    out.target_boot_id = rd16(p + 4);
    return true;
}

uint8_t bodyOf(const translagatr::PicoCommand& c) {
    switch (c.op) {
    case translagatr::kPicoOpConfigure:
        return c.imu_enabled;
    case translagatr::kPicoOpReinitImu:
        return c.imu_port;
    case translagatr::kPicoOpDiagnostics:
        return c.diag_hz;
    default:
        return 0;
    }
}

} // namespace

Outcome Commands::receive(const uint8_t* frame, uint16_t len, uint16_t boot_id, bool imu_enabled) {
    translagatr::PicoCommand c;
    const bool         whole = translagatr::decodePicoCommand(frame, len, c);
    if ((!whole && !readHeader(frame, len, c)) || c.request_id == 0) {
        ++ignored_;
        return {};
    }
    ++received_;
    if (c.target_boot_id != boot_id) {
        return answer(c, translagatr::kPicoCommandFailed, translagatr::kPicoDetailWrongTarget);
    }
    if (translagatr::picoCommandLen(c.op) == 0) {
        return answer(c, translagatr::kPicoCommandFailed, translagatr::kPicoDetailUnknownOp);
    }
    if (!whole) {
        return answer(c, translagatr::kPicoCommandFailed, translagatr::kPicoDetailBadBody);
    }
    const uint8_t body = bodyOf(c);
    if (const CommandRecord* r = records_.find(c.request_id, c.op, body)) {
        ++duplicates_;
        last_ = Last{r->request_id, r->op, r->status, r->detail, true};
        Outcome out;
        out.answered = true;
        return out;
    }
    return run(c, body, imu_enabled);
}

Outcome Commands::run(const translagatr::PicoCommand& c, uint8_t body, bool imu_enabled) {
    switch (c.op) {
    case translagatr::kPicoOpConfigure: {
        if (body > 1) {
            return record(c, body, translagatr::kPicoCommandFailed, translagatr::kPicoDetailBadBody,
                          Effect::None);
        }
        const bool   want   = body != 0;
        const Effect effect = want == imu_enabled ? Effect::None
                              : want              ? Effect::EnableImu
                                                  : Effect::DisableImu;
        return record(c, body, translagatr::kPicoCommandCompleted, translagatr::kPicoDetailNone, effect);
    }
    case translagatr::kPicoOpReinitImu:
        if (body != 0) {
            return record(c, body, translagatr::kPicoCommandFailed, translagatr::kPicoDetailNoSuchPort,
                          Effect::None);
        }
        if (!imu_enabled) {
            return record(c, body, translagatr::kPicoCommandFailed, translagatr::kPicoDetailImuDisabled,
                          Effect::None);
        }
        return record(c, body, translagatr::kPicoCommandRunning, translagatr::kPicoDetailNone,
                      Effect::ReinitImu);
    case translagatr::kPicoOpRestartAcquisition:
        return record(c, body, translagatr::kPicoCommandCompleted, translagatr::kPicoDetailNone,
                      Effect::RestartAcquisition);
    case translagatr::kPicoOpDiagnostics: {
        if (body > kMaxDiagHz) {
            return record(c, body, translagatr::kPicoCommandFailed, translagatr::kPicoDetailBadBody,
                          Effect::None);
        }
        Outcome out = record(c, body, translagatr::kPicoCommandCompleted,
                             translagatr::kPicoDetailNone, Effect::SetDiagnostics);
        out.diag_hz = body;
        return out;
    }
    default:
        return answer(c, translagatr::kPicoCommandFailed, translagatr::kPicoDetailUnknownOp);
    }
}

Outcome Commands::answer(const translagatr::PicoCommand& c, uint8_t status, uint8_t detail) {
    last_ = Last{c.request_id, c.op, status, detail, false};
    Outcome out;
    out.answered = true;
    return out;
}

Outcome Commands::record(const translagatr::PicoCommand& c, uint8_t body, uint8_t status, uint8_t detail,
                         Effect effect) {
    CommandRecord r;
    r.request_id = c.request_id;
    r.op         = c.op;
    r.body       = body;
    r.status     = status;
    r.detail     = detail;
    records_.add(r);
    last_ = Last{c.request_id, c.op, status, detail, true};
    Outcome out;
    out.answered = true;
    out.effect   = effect;
    return out;
}

bool Commands::imuProgress(uint8_t imu_state) {
    uint8_t status = translagatr::kPicoCommandRunning;
    uint8_t detail = translagatr::kPicoDetailNone;
    switch (imu_state) {
    case translagatr::kPicoImuReady:
        status = translagatr::kPicoCommandCompleted;
        break;
    case translagatr::kPicoImuFailed:
        status = translagatr::kPicoCommandFailed;
        detail = translagatr::kPicoDetailImuAbsent;
        break;
    case translagatr::kPicoImuDisabled:
        status = translagatr::kPicoCommandFailed;
        detail = translagatr::kPicoDetailImuDisabled;
        break;
    default:
        return false;
    }
    bool changed = false;
    for (uint8_t i = 0; i < CommandRecords::kCapacity; ++i) {
        CommandRecord& r = records_.at(i);
        if (r.request_id == 0 || r.op != translagatr::kPicoOpReinitImu ||
            r.status != translagatr::kPicoCommandRunning) {
            continue;
        }
        r.status = status;
        r.detail = detail;
        changed  = true;
        if (last_.recorded && last_.request_id == r.request_id && last_.op == r.op) {
            last_.status = status;
            last_.detail = detail;
        }
    }
    return changed;
}

void Commands::fill(translagatr::PicoStatus& status) const {
    status.last_request_id = last_.request_id;
    status.last_op         = last_.op;
    status.last_status     = last_.status;
    status.last_detail     = last_.detail;
}

} // namespace pilink
