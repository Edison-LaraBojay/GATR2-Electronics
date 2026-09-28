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
bool readHeader(const uint8_t* frame, uint16_t len, gatr2::PicoCommand& out) {
    if (len < gatr2::kLinkEnvelopeLen + gatr2::kPicoCommandHeaderLen) {
        return false;
    }
    if (frame[0] != gatr2::kSync0 || frame[1] != gatr2::kSync1 ||
        frame[2] != gatr2::kFramePicoCommand || frame[3] + gatr2::kLinkEnvelopeLen != len) {
        return false;
    }
    if (gatr2::crc16(frame + 2, static_cast<uint16_t>(len - 4)) != rd16(frame + len - 2)) {
        return false;
    }
    const uint8_t* p = frame + 4;
    if (p[0] != gatr2::kPicoLinkVersion) {
        return false;
    }
    out                = gatr2::PicoCommand{};
    out.version        = p[0];
    out.op             = p[1];
    out.request_id     = rd16(p + 2);
    out.target_boot_id = rd16(p + 4);
    return true;
}

uint8_t bodyOf(const gatr2::PicoCommand& c) {
    switch (c.op) {
    case gatr2::kPicoOpConfigure:
        return c.imu_enabled;
    case gatr2::kPicoOpReinitImu:
        return c.imu_port;
    default:
        return 0;
    }
}

} // namespace

Outcome Commands::receive(const uint8_t* frame, uint16_t len, uint16_t boot_id, bool imu_enabled) {
    gatr2::PicoCommand c;
    const bool         whole = gatr2::decodePicoCommand(frame, len, c);
    if ((!whole && !readHeader(frame, len, c)) || c.request_id == 0) {
        ++ignored_;
        return {};
    }
    ++received_;
    if (c.target_boot_id != boot_id) {
        return answer(c, gatr2::kPicoCommandFailed, gatr2::kPicoDetailWrongTarget);
    }
    if (gatr2::picoCommandLen(c.op) == 0) {
        return answer(c, gatr2::kPicoCommandFailed, gatr2::kPicoDetailUnknownOp);
    }
    if (!whole) {
        return answer(c, gatr2::kPicoCommandFailed, gatr2::kPicoDetailBadBody);
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

Outcome Commands::run(const gatr2::PicoCommand& c, uint8_t body, bool imu_enabled) {
    switch (c.op) {
    case gatr2::kPicoOpConfigure: {
        if (body > 1) {
            return record(c, body, gatr2::kPicoCommandFailed, gatr2::kPicoDetailBadBody,
                          Effect::None);
        }
        const bool   want   = body != 0;
        const Effect effect = want == imu_enabled ? Effect::None
                              : want              ? Effect::EnableImu
                                                  : Effect::DisableImu;
        return record(c, body, gatr2::kPicoCommandCompleted, gatr2::kPicoDetailNone, effect);
    }
    case gatr2::kPicoOpReinitImu:
        if (body != 0) {
            return record(c, body, gatr2::kPicoCommandFailed, gatr2::kPicoDetailNoSuchPort,
                          Effect::None);
        }
        if (!imu_enabled) {
            return record(c, body, gatr2::kPicoCommandFailed, gatr2::kPicoDetailImuDisabled,
                          Effect::None);
        }
        return record(c, body, gatr2::kPicoCommandRunning, gatr2::kPicoDetailNone,
                      Effect::ReinitImu);
    case gatr2::kPicoOpRestartAcquisition:
        return record(c, body, gatr2::kPicoCommandCompleted, gatr2::kPicoDetailNone,
                      Effect::RestartAcquisition);
    default:
        return answer(c, gatr2::kPicoCommandFailed, gatr2::kPicoDetailUnknownOp);
    }
}

Outcome Commands::answer(const gatr2::PicoCommand& c, uint8_t status, uint8_t detail) {
    last_ = Last{c.request_id, c.op, status, detail, false};
    Outcome out;
    out.answered = true;
    return out;
}

Outcome Commands::record(const gatr2::PicoCommand& c, uint8_t body, uint8_t status, uint8_t detail,
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
    uint8_t status = gatr2::kPicoCommandRunning;
    uint8_t detail = gatr2::kPicoDetailNone;
    switch (imu_state) {
    case gatr2::kPicoImuReady:
        status = gatr2::kPicoCommandCompleted;
        break;
    case gatr2::kPicoImuFailed:
        status = gatr2::kPicoCommandFailed;
        detail = gatr2::kPicoDetailImuAbsent;
        break;
    case gatr2::kPicoImuDisabled:
        status = gatr2::kPicoCommandFailed;
        detail = gatr2::kPicoDetailImuDisabled;
        break;
    default:
        return false;
    }
    bool changed = false;
    for (uint8_t i = 0; i < CommandRecords::kCapacity; ++i) {
        CommandRecord& r = records_.at(i);
        if (r.request_id == 0 || r.op != gatr2::kPicoOpReinitImu ||
            r.status != gatr2::kPicoCommandRunning) {
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

void Commands::fill(gatr2::PicoStatus& status) const {
    status.last_request_id = last_.request_id;
    status.last_op         = last_.op;
    status.last_status     = last_.status;
    status.last_detail     = last_.detail;
}

} // namespace pilink
