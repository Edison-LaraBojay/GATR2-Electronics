// frame_codec_gtest.cpp
// Byte-level tests of the sensor frame and brain link v4. The known vectors
// were packed by an independent generator; every one is also checked here
// against refCrc16, a CRC-16/CCITT-FALSE unrelated to the codec's bit loop.

#include <gtest/gtest.h>

#include <stddef.h>
#include <stdint.h>

#include <initializer_list>
#include <vector>

#include "frame_codec.h"

using namespace gatr2;

namespace
{

using Bytes = std::vector<uint8_t>;

// CRC-16/CCITT-FALSE, byte-wise shift form.
uint16_t refCrc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        uint8_t x = static_cast<uint8_t>((crc >> 8) ^ data[i]);
        x         = static_cast<uint8_t>(x ^ (x >> 4));
        crc       = static_cast<uint16_t>((crc << 8) ^ (x << 12) ^ (x << 5) ^ x);
    }
    return crc;
}

// Little endian writer, independent of the codec.
struct Le {
    Bytes b;
    Le&   u8(uint32_t v) {
        b.push_back(static_cast<uint8_t>(v));
        return *this;
    }
    Le& u16(uint32_t v) { return u8(v).u8(v >> 8); }
    Le& u32(uint32_t v) { return u16(v).u16(v >> 16); }
    Le& i32(int32_t v) { return u32(static_cast<uint32_t>(v)); }
    Le& raw(const Bytes& r) {
        b.insert(b.end(), r.begin(), r.end());
        return *this;
    }
};

// Link frame around any payload, crc from refCrc16.
Bytes linkFrame(uint8_t type, const Bytes& payload) {
    Bytes f = {0xAA, 0x55, type, static_cast<uint8_t>(payload.size())};
    f.insert(f.end(), payload.begin(), payload.end());
    const uint16_t crc = refCrc16(f.data() + 2, f.size() - 2);
    f.push_back(static_cast<uint8_t>(crc));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    return f;
}

constexpr uint32_t kSession   = 0xA1B2C3D4;
constexpr uint32_t kInstance  = 0x0BADF00D;
constexpr uint32_t kNonce     = 0x12345678;
constexpr uint32_t kProfileId = 0xCAFEBABE;
constexpr uint32_t kMapId     = 0x89ABCDEF;

Le requestHead(uint8_t op, uint32_t session, uint16_t rid) {
    Le h;
    h.u8(kBrainLinkVersion).u8(op).u32(session).u16(rid);
    return h;
}

Le replyHead(uint8_t op, uint32_t session, uint16_t rid, uint8_t result) {
    Le h;
    h.u8(kBrainLinkVersion).u8(op).u32(session).u16(rid).u8(result).u32(kInstance);
    return h;
}

// Sensor frame: seq 7, stamp 1000, mask 0x000B, enc0 1000, enc1 -500,
// gyro_z 2500.
const Bytes kSensorVector = {
    0xAA, 0x55, 0x01, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x0B, 0x00, 0xE8, 0x03,
    0x00, 0x00, 0x0C, 0xFE, 0xFF, 0xFF, 0xC4, 0x09, 0x00, 0x00, 0xCD,
};

// HELLO: session 0, rid 1, nonce 0x12345678.
const Bytes kHelloRequest = {
    0xAA, 0x55, 0x10, 0x0C, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x78, 0x56, 0x34, 0x12, 0xAD, 0x21,
};

// SET_POSE: rid 2, (610, -457, -9000).
const Bytes kSetPoseRequest = {
    0xAA, 0x55, 0x10, 0x14, 0x04, 0x02, 0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00,
    0x62, 0x02, 0x00, 0x00, 0x37, 0xFE, 0xFF, 0xFF, 0xD8, 0xDC, 0xFF, 0xFF,
    0xDC, 0x0D,
};

// GET_STATE without a bench sample: rid 65535.
const Bytes kGetStateRequest = {
    0xAA, 0x55, 0x10, 0x11, 0x04, 0x04, 0xD4, 0xC3, 0xB2, 0xA1, 0xFF, 0xFF,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x0F,
};

// GET_STATE with a bench sample: rid 42, stamp 0x12345678, -450123 mdeg.
const Bytes kGetStateImuRequest = {
    0xAA, 0x55, 0x10, 0x11, 0x04, 0x04, 0xD4, 0xC3, 0xB2, 0xA1, 0x2A, 0x00,
    0x01, 0x78, 0x56, 0x34, 0x12, 0xB5, 0x21, 0xF9, 0xFF, 0x18, 0x5F,
};

// PROFILE_WRITE: rid 5, id 0xCAFEBABE, total 208, offset 106, one byte 0x5A.
const Bytes kProfileWriteRequest = {
    0xAA, 0x55, 0x10, 0x11, 0x04, 0x06, 0xD4, 0xC3, 0xB2, 0xA1, 0x05, 0x00,
    0xBE, 0xBA, 0xFE, 0xCA, 0xD0, 0x00, 0x6A, 0x00, 0x5A, 0x73, 0xB5,
};

// PROFILE_APPLY: rid 6, id 0xCAFEBABE, total 208.
const Bytes kProfileApplyRequest = {
    0xAA, 0x55, 0x10, 0x0E, 0x04, 0x07, 0xD4, 0xC3, 0xB2, 0xA1, 0x06, 0x00,
    0xBE, 0xBA, 0xFE, 0xCA, 0xD0, 0x00, 0x79, 0x49,
};

// READ_DOC: rid 7, field map 0x89ABCDEF, offset 192, max 96.
const Bytes kReadDocRequest = {
    0xAA, 0x55, 0x10, 0x10, 0x04, 0x08, 0xD4, 0xC3, 0xB2, 0xA1, 0x07, 0x00,
    0x01, 0xEF, 0xCD, 0xAB, 0x89, 0xC0, 0x00, 0x60, 0x73, 0xFC,
};

// CONTROL: rid 8, recalibrate.
const Bytes kControlRequest = {
    0xAA, 0x55, 0x10, 0x0A, 0x04, 0x09, 0xD4, 0xC3, 0xB2, 0xA1, 0x08, 0x00,
    0x01, 0x00, 0x49, 0x0F,
};

// PATH_REPORT: rid 9, command 0x01020304, mode none, no points.
const Bytes kPathClearRequest = {
    0xAA, 0x55, 0x10, 0x0E, 0x04, 0x0A, 0xD4, 0xC3, 0xB2, 0xA1, 0x09, 0x00,
    0x04, 0x03, 0x02, 0x01, 0x00, 0x00, 0x9D, 0x33,
};

// PATH_REPORT: rid 10, command 77, avoiding, (1000, -2000) (-1, 32767).
const Bytes kPathRequest = {
    0xAA, 0x55, 0x10, 0x1E, 0x04, 0x0A, 0xD4, 0xC3, 0xB2, 0xA1, 0x0A, 0x00,
    0x4D, 0x00, 0x00, 0x00, 0x02, 0x02, 0xE8, 0x03, 0x00, 0x00, 0x30, 0xF8,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x9F, 0xCB,
};

// HELLO Ok: new session, nonce echo.
const Bytes kHelloOkReply = {
    0xAA, 0x55, 0x11, 0x11, 0x04, 0x01, 0xD4, 0xC3, 0xB2, 0xA1, 0x01, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x78, 0x56, 0x34, 0x12, 0x15, 0x1F,
};

// HELLO Stale: session 0, nonce echo.
const Bytes kHelloStaleReply = {
    0xAA, 0x55, 0x11, 0x11, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x08, 0x0D, 0xF0, 0xAD, 0x0B, 0x78, 0x56, 0x34, 0x12, 0x65, 0xAF,
};

// SET_POSE Ok: epoch 2, anchor revision 7.
const Bytes kSetPoseOkReply = {
    0xAA, 0x55, 0x11, 0x15, 0x04, 0x02, 0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x02, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00,
    0x00, 0x70, 0x2F,
};

// SET_POSE Pending: epoch 2, anchor revision 6.
const Bytes kSetPosePendingReply = {
    0xAA, 0x55, 0x11, 0x15, 0x04, 0x02, 0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00,
    0x01, 0x0D, 0xF0, 0xAD, 0x0B, 0x02, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
    0x00, 0xA7, 0x1C,
};

// SET_POSE NotReady: header only.
const Bytes kSetPoseNotReadyReply = {
    0xAA, 0x55, 0x11, 0x0D, 0x04, 0x02, 0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00,
    0x09, 0x0D, 0xF0, 0xAD, 0x0B, 0x56, 0xAA,
};

// GET_STATE Ok: see makeState().
const Bytes kGetStateOkReply = {
    0xAA, 0x55, 0x11, 0x35, 0x04, 0x04, 0xD4, 0xC3, 0xB2, 0xA1, 0xFF, 0xFF,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x0F, 0xDC, 0x05, 0x00, 0x00, 0x06, 0xFF,
    0xFF, 0xFF, 0x50, 0x46, 0x00, 0x00, 0x23, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x0B, 0x03, 0x06, 0xBE, 0xBA, 0xFE, 0xCA, 0xEF,
    0xCD, 0xAB, 0x89, 0x02, 0x01, 0x00, 0x00, 0x01, 0x01, 0x71, 0xB2,
};

// PROFILE_WRITE Ok: id 0xCAFEBABE, 107 bytes held.
const Bytes kProfileWriteOkReply = {
    0xAA, 0x55, 0x11, 0x13, 0x04, 0x06, 0xD4, 0xC3, 0xB2, 0xA1, 0x05, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0xBE, 0xBA, 0xFE, 0xCA, 0x6B, 0x00, 0xDB,
    0x8C,
};

// PROFILE_APPLY Pending: applying.
const Bytes kProfileApplyPendingReply = {
    0xAA, 0x55, 0x11, 0x14, 0x04, 0x07, 0xD4, 0xC3, 0xB2, 0xA1, 0x06, 0x00,
    0x01, 0x0D, 0xF0, 0xAD, 0x0B, 0xBE, 0xBA, 0xFE, 0xCA, 0x01, 0x00, 0x00,
    0xE7, 0x8E,
};

// PROFILE_APPLY ProfileRejected: observability, wheel 1.
const Bytes kProfileRejectedReply = {
    0xAA, 0x55, 0x11, 0x14, 0x04, 0x07, 0xD4, 0xC3, 0xB2, 0xA1, 0x06, 0x00,
    0x0A, 0x0D, 0xF0, 0xAD, 0x0B, 0xBE, 0xBA, 0xFE, 0xCA, 0x03, 0x06, 0x01,
    0x37, 0x44,
};

// READ_DOC Ok: estimate 5, total 44, crc 0x11223344, offset 43, one byte 0xEE.
const Bytes kReadDocOkReply = {
    0xAA, 0x55, 0x11, 0x1B, 0x04, 0x08, 0xD4, 0xC3, 0xB2, 0xA1, 0x07, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x02, 0x05, 0x00, 0x00, 0x00, 0x2C, 0x00,
    0x44, 0x33, 0x22, 0x11, 0x2B, 0x00, 0xEE, 0x1A, 0x33,
};

// READ_DOC Unavailable: header only.
const Bytes kReadDocUnavailableReply = {
    0xAA, 0x55, 0x11, 0x0D, 0x04, 0x08, 0xD4, 0xC3, 0xB2, 0xA1, 0x07, 0x00,
    0x0B, 0x0D, 0xF0, 0xAD, 0x0B, 0x30, 0x8A,
};

// CONTROL Ok: reinitialize, calibration running, no detail.
const Bytes kControlOkReply = {
    0xAA, 0x55, 0x11, 0x10, 0x04, 0x09, 0xD4, 0xC3, 0xB2, 0xA1, 0x08, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x02, 0x01, 0x00, 0xC6, 0x40,
};

// CONTROL Pending: reinit IMU, calibration waiting still, no detail.
const Bytes kControlPendingReply = {
    0xAA, 0x55, 0x11, 0x10, 0x04, 0x09, 0xD4, 0xC3, 0xB2, 0xA1, 0x0C, 0x00,
    0x01, 0x0D, 0xF0, 0xAD, 0x0B, 0x03, 0x03, 0x00, 0x72, 0xFB,
};

// CONTROL Failed: restart acquisition, calibration failed, detail timed out.
const Bytes kControlFailedReply = {
    0xAA, 0x55, 0x11, 0x10, 0x04, 0x09, 0xD4, 0xC3, 0xB2, 0xA1, 0x0D, 0x00,
    0x0D, 0x0D, 0xF0, 0xAD, 0x0B, 0x04, 0x05, 0x05, 0x13, 0xFA,
};

// CONTROL: rid 12, reinit IMU, arg 0.
const Bytes kControlReinitImuRequest = {
    0xAA, 0x55, 0x10, 0x0A, 0x04, 0x09, 0xD4, 0xC3, 0xB2, 0xA1, 0x0C, 0x00,
    0x03, 0x00, 0xDA, 0xA3,
};

// READ_WHEELS: rid 11.
const Bytes kReadWheelsRequest = {
    0xAA, 0x55, 0x10, 0x08, 0x04, 0x0B, 0xD4, 0xC3, 0xB2, 0xA1, 0x0B, 0x00,
    0x82, 0x2D,
};

// READ_WHEELS Ok: two wheels, see makeWheels().
const Bytes kReadWheelsOkReply = {
    0xAA, 0x55, 0x11, 0x2E, 0x04, 0x0B, 0xD4, 0xC3, 0xB2, 0xA1, 0x0B, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x02, 0x00, 0x03, 0x34, 0x12, 0xC0, 0x1D,
    0xFE, 0xFF, 0x06, 0x12, 0x0F, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x01,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF,
    0x00, 0x00, 0x5E, 0x0E,
};

// READ_WHEELS Ok: no wheels.
const Bytes kReadWheelsEmptyReply = {
    0xAA, 0x55, 0x11, 0x0E, 0x04, 0x0B, 0xD4, 0xC3, 0xB2, 0xA1, 0x0B, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x00, 0x57, 0xFE,
};

// READ_WHEELS NotReady: header only.
const Bytes kReadWheelsNotReadyReply = {
    0xAA, 0x55, 0x11, 0x0D, 0x04, 0x0B, 0xD4, 0xC3, 0xB2, 0xA1, 0x0B, 0x00,
    0x09, 0x0D, 0xF0, 0xAD, 0x0B, 0x47, 0x99,
};

// Sensor v2: seq 9, stamp 0x01020304, boot 0xBEEF, acq 2, imu 5, mask
// 0x000B, enc0 1000, enc1 -500, gyro_z 2500.
const Bytes kSensorV2Vector = {
    0xAA, 0x55, 0x04, 0x09, 0x04, 0x03, 0x02, 0x01, 0xEF, 0xBE, 0x02, 0x05,
    0x0B, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x0C, 0xFE, 0xFF, 0xFF, 0xC4, 0x09,
    0x00, 0x00, 0x7F,
};

// Sensor v2: seq 0, stamp 0, boot 1, acq 0, imu 0, no sensors.
const Bytes kSensorV2EmptyVector = {
    0xAA, 0x55, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA,
};

// Pico CONFIGURE: rid 0x0102, boot 0xBEEF, IMU enabled.
const Bytes kPicoConfigureCommand = {
    0xAA, 0x55, 0x12, 0x07, 0x01, 0x01, 0x02, 0x01, 0xEF, 0xBE, 0x01, 0x26,
    0x24,
};

// Pico REINIT_IMU: rid 0xFFFF, boot 0x0001, port 0.
const Bytes kPicoReinitImuCommand = {
    0xAA, 0x55, 0x12, 0x07, 0x01, 0x02, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0xEE,
    0x5F,
};

// Pico RESTART_ACQUISITION: rid 7, boot 0xBEEF.
const Bytes kPicoRestartCommand = {
    0xAA, 0x55, 0x12, 0x06, 0x01, 0x03, 0x07, 0x00, 0xEF, 0xBE, 0xCF, 0x9E,
};

// Pico status: see makePicoStatus().
const Bytes kPicoStatusVector = {
    0xAA, 0x55, 0x13, 0x14, 0x01, 0xEF, 0xBE, 0x02, 0x05, 0x56, 0x34, 0x12,
    0x00, 0x04, 0x03, 0x02, 0x01, 0x01, 0x02, 0x01, 0x02, 0x01, 0x04, 0x01,
    0x09, 0xBF,
};

// CONTROL NotStationary: header only.
const Bytes kControlNotStationaryReply = {
    0xAA, 0x55, 0x11, 0x0D, 0x04, 0x09, 0xD4, 0xC3, 0xB2, 0xA1, 0x08, 0x00,
    0x0C, 0x0D, 0xF0, 0xAD, 0x0B, 0x78, 0x64,
};

// PATH_REPORT Ok: header only.
const Bytes kPathOkReply = {
    0xAA, 0x55, 0x11, 0x0D, 0x04, 0x0A, 0xD4, 0xC3, 0xB2, 0xA1, 0x0A, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x2F, 0x8A,
};

const std::vector<const Bytes*> kLinkVectors = {
    &kHelloRequest,       &kSetPoseRequest,          &kGetStateRequest,
    &kGetStateImuRequest, &kProfileWriteRequest,     &kProfileApplyRequest,
    &kReadDocRequest,     &kControlRequest,          &kPathClearRequest,
    &kPathRequest,        &kHelloOkReply,            &kHelloStaleReply,
    &kSetPoseOkReply,     &kSetPosePendingReply,     &kSetPoseNotReadyReply,
    &kGetStateOkReply,    &kProfileWriteOkReply,     &kProfileApplyPendingReply,
    &kProfileRejectedReply, &kReadDocOkReply,        &kReadDocUnavailableReply,
    &kControlOkReply,     &kControlNotStationaryReply, &kPathOkReply,
    &kControlPendingReply, &kControlFailedReply,     &kControlReinitImuRequest,
    &kReadWheelsRequest,  &kReadWheelsOkReply,       &kReadWheelsEmptyReply,
    &kReadWheelsNotReadyReply, &kPicoConfigureCommand, &kPicoReinitImuCommand,
    &kPicoRestartCommand, &kPicoStatusVector,
};

const uint8_t kKnownOps[] = {kOpHello,        kOpSetPose, kOpGetState,   kOpProfileWrite,
                             kOpProfileApply, kOpReadDoc, kOpControl,    kOpPathReport,
                             kOpReadWheels};

const uint8_t kKnownResults[] = {kResultOk,
                                 kResultPending,
                                 kResultUnknownSession,
                                 kResultUnsupportedVersion,
                                 kResultUnsupportedOp,
                                 kResultInvalidArgument,
                                 kResultStale,
                                 kResultNotReady,
                                 kResultProfileRejected,
                                 kResultUnavailable,
                                 kResultNotStationary,
                                 kResultFailed};

bool isKnownResult(uint8_t result) {
    for (uint8_t r : kKnownResults) {
        if (r == result) {
            return true;
        }
    }
    return false;
}

SensorSample makeSample() {
    SensorSample s{};
    s.seq      = 7;
    s.stamp_ms = 1000;
    s.mask     = kSensorEnc0 | kSensorEnc1 | kSensorGyroZ;
    s.enc[0]   = 1000;
    s.enc[1]   = -500;
    s.gyro_z   = 2500;
    return s;
}

BrainRequest makeRequest(uint8_t op, uint32_t session, uint16_t rid) {
    BrainRequest r{};
    r.op         = op;
    r.session    = session;
    r.request_id = rid;
    return r;
}

BrainReply makeReply(uint8_t op, uint32_t session, uint16_t rid, uint8_t result) {
    BrainReply r{};
    r.op          = op;
    r.session     = session;
    r.request_id  = rid;
    r.result      = result;
    r.pi_instance = kInstance;
    return r;
}

BrainState makeState() {
    BrainState s{};
    s.robot_flags     = kRobotPoseValid | kRobotLocalized | kRobotAgeKnown | kRobotAnchorCommand;
    s.x_mm            = 1500;
    s.y_mm            = -250;
    s.heading_cdeg    = 18000;
    s.robot_age_ms    = 35;
    s.odometry_epoch  = 2;
    s.anchor_revision = 7;
    s.health          = kHealthEncodersFresh | kHealthGyroFresh | kHealthBiasCalibrated;
    s.profile_state   = kProfileRejected;
    s.profile_reason  = kProfileReasonObservability;
    s.profile_detail  = 1;
    s.profile_id      = kProfileId;
    s.map_id          = kMapId;
    s.estimate_id     = 0x102;
    s.calibration     = kCalibrationRunning;
    return s;
}

// The two READ_WHEELS records of kReadWheelsOkReply.
BrainReply makeWheels() {
    BrainReply r        = makeReply(kOpReadWheels, kSession, 11, kResultOk);
    r.wheel_count       = 2;
    r.wheels[0]         = {0, kWheelFresh | kWheelValid, 0x1234, -123456, 987654, 12};
    r.wheels[1]         = {1, kWheelFresh, 0xFFFF, INT32_MAX, INT32_MIN, 65535};
    return r;
}

SensorSample makeSensorV2() {
    SensorSample s = makeSample();
    s.seq          = 9;
    s.stamp_ms     = 0x01020304;
    s.identity     = true;
    s.boot_id      = 0xBEEF;
    s.acq_epoch    = 2;
    s.imu_epoch    = 5;
    return s;
}

PicoStatus makePicoStatus() {
    PicoStatus s{};
    s.boot_id         = 0xBEEF;
    s.acq_epoch       = 2;
    s.imu_epoch       = 5;
    s.uptime_ms       = 0x00123456;
    s.imu_state       = kPicoImuRetrying;
    s.imu_reason      = kPicoImuReasonFeatures;
    s.imu_attempts    = 0x0102;
    s.flags           = kPicoImuEnabled;
    s.last_request_id = 0x0102;
    s.last_op         = kPicoOpReinitImu;
    s.last_status     = kPicoCommandRunning;
    s.last_detail     = kPicoDetailImuAbsent;
    s.firmware        = kPicoFirmwareBno08x;
    return s;
}

PicoCommand makePicoCommand(uint8_t op, uint16_t rid, uint16_t boot) {
    PicoCommand c{};
    c.op             = op;
    c.request_id     = rid;
    c.target_boot_id = boot;
    return c;
}

Bytes encode(const PicoCommand& c) {
    Bytes buf(kMaxFrameLen);
    buf.resize(encodePicoCommand(c, buf.data(), kMaxFrameLen));
    return buf;
}

Bytes encode(const PicoStatus& s) {
    Bytes buf(kMaxFrameLen);
    buf.resize(encodePicoStatus(s, buf.data(), kMaxFrameLen));
    return buf;
}

Bytes encode(const SensorSample& s) {
    Bytes buf(kMaxFrameLen);
    buf.resize(encodeSensorFrame(s, buf.data(), kMaxFrameLen));
    return buf;
}

bool decodes(const Bytes& f, PicoCommand& out) {
    return decodePicoCommand(f.data(), static_cast<uint16_t>(f.size()), out);
}

bool decodes(const Bytes& f, PicoStatus& out) {
    return decodePicoStatus(f.data(), static_cast<uint16_t>(f.size()), out);
}

bool decodes(const Bytes& f, SensorSample& out) {
    return decodeSensorFrame(f.data(), static_cast<uint16_t>(f.size()), out);
}

// XOR over the whole frame, independent of gatr2::checksum.
uint8_t refXor(const Bytes& f) {
    uint8_t x = 0;
    for (uint8_t b : f) {
        x = static_cast<uint8_t>(x ^ b);
    }
    return x;
}

// Deterministic chunk content that includes sync bytes and zeros.
uint8_t pattern(size_t i) {
    return static_cast<uint8_t>(i * 37 + 0xAA);
}

Bytes encode(const BrainRequest& r) {
    Bytes buf(kMaxFrameLen);
    buf.resize(encodeBrainRequest(r, buf.data(), kMaxFrameLen));
    return buf;
}

Bytes encode(const BrainReply& r) {
    Bytes buf(kMaxFrameLen);
    buf.resize(encodeBrainReply(r, buf.data(), kMaxFrameLen));
    return buf;
}

// Request payload: header plus zero body bytes.
Bytes requestPayload(uint8_t version, uint8_t op, size_t len) {
    Bytes p(len, 0);
    p[0] = version;
    p[1] = op;
    p[6] = 0x09;
    return p;
}

// Reply payload: header plus zero body bytes.
Bytes replyPayload(uint8_t version, uint8_t op, uint8_t result, size_t len) {
    Bytes p(len, 0);
    p[0] = version;
    p[1] = op;
    p[6] = 0x09;
    p[8] = result;
    return p;
}

bool decodes(const Bytes& f, BrainRequest& out) {
    return decodeBrainRequest(f.data(), static_cast<uint16_t>(f.size()), out);
}

bool decodes(const Bytes& f, BrainReply& out) {
    return decodeBrainReply(f.data(), static_cast<uint16_t>(f.size()), out);
}

// PROFILE_WRITE with a full 106 byte chunk: 128 byte frame.
BrainRequest maxProfileWrite() {
    BrainRequest r = makeRequest(kOpProfileWrite, kSession, 11);
    r.profile_id   = kProfileId;
    r.total_len    = 208;
    r.offset       = 0;
    r.data_len     = kProfileChunkMax;
    for (size_t i = 0; i < kProfileChunkMax; ++i) {
        r.data[i] = pattern(i);
    }
    return r;
}

Bytes maxProfileWriteBytes() {
    Le p = requestHead(kOpProfileWrite, kSession, 11);
    p.u32(kProfileId).u16(208).u16(0);
    for (size_t i = 0; i < kProfileChunkMax; ++i) {
        p.u8(pattern(i));
    }
    return linkFrame(kFrameBrainRequest, p.b);
}

// READ_DOC Ok with a full 96 byte chunk: 128 byte frame.
BrainReply maxReadDoc() {
    BrainReply r    = makeReply(kOpReadDoc, kSession, 12, kResultOk);
    r.doc_kind      = kDocFieldMap;
    r.doc_id        = kMapId;
    r.doc_total_len = 3608;
    r.doc_crc32     = kMapId;
    r.doc_offset    = 3456;
    r.data_len      = kDocChunkMax;
    for (size_t i = 0; i < kDocChunkMax; ++i) {
        r.data[i] = pattern(i + 1);
    }
    return r;
}

Bytes maxReadDocBytes() {
    Le p = replyHead(kOpReadDoc, kSession, 12, kResultOk);
    p.u8(kDocFieldMap).u32(kMapId).u16(3608).u32(kMapId).u16(3456);
    for (size_t i = 0; i < kDocChunkMax; ++i) {
        p.u8(pattern(i + 1));
    }
    return linkFrame(kFrameBrainReply, p.b);
}

Bytes frameOf(const FrameReader& r) {
    return Bytes(r.frame(), r.frame() + r.frameLen());
}

// Frames completed while pushing bytes one at a time.
std::vector<Bytes> readAll(FrameReader& r, const Bytes& stream) {
    std::vector<Bytes> frames;
    for (uint8_t b : stream) {
        if (!r.push(b)) {
            continue;
        }
        do {
            frames.push_back(frameOf(r));
        } while (r.next());
    }
    return frames;
}

Bytes concat(std::initializer_list<Bytes> parts) {
    Bytes out;
    for (const Bytes& p : parts) {
        out.insert(out.end(), p.begin(), p.end());
    }
    return out;
}

} // namespace

// ---------------------------------------------------------------------------
// CRC-16
// ---------------------------------------------------------------------------

TEST(Crc, CheckValue) {
    const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(refCrc16(text, sizeof(text)), 0x29B1);
    EXPECT_EQ(crc16(text, sizeof(text)), 0x29B1);
}

TEST(Crc, EmptyIsInit) {
    EXPECT_EQ(crc16(nullptr, 0), 0xFFFF);
}

TEST(Crc, MatchesReferenceOnPseudoRandomData) {
    uint32_t seed = 12345;
    Bytes    data;
    for (size_t len = 0; len <= 300; ++len) {
        EXPECT_EQ(crc16(data.data(), static_cast<uint16_t>(data.size())),
                  refCrc16(data.data(), data.size()))
            << "len " << len;
        seed = seed * 1103515245u + 12345u;
        data.push_back(static_cast<uint8_t>(seed >> 16));
    }
}

TEST(Crc, EveryKnownVectorCarriesItsReferenceCrc) {
    for (const Bytes* v : kLinkVectors) {
        const size_t n = v->size();
        ASSERT_GE(n, kLinkEnvelopeLen);
        EXPECT_EQ((*v)[3] + kLinkEnvelopeLen, n);
        const uint16_t stored = static_cast<uint16_t>((*v)[n - 2] | ((*v)[n - 1] << 8));
        EXPECT_EQ(stored, refCrc16(v->data() + 2, n - 4));
    }
}

TEST(Crc, CoversTypeLenAndPayloadNotSync) {
    const uint16_t crc = crc16(kGetStateRequest.data() + 2, 19);
    EXPECT_EQ(crc, 0x0F28);

    Bytes other_sync = kGetStateRequest;
    other_sync[0]    = 0x00;
    other_sync[1]    = 0x00;
    EXPECT_EQ(crc16(other_sync.data() + 2, 19), crc);
}

// ---------------------------------------------------------------------------
// Length arithmetic
// ---------------------------------------------------------------------------

TEST(Lengths, SensorMaskValidity) {
    EXPECT_TRUE(sensorMaskValid(0x0000));
    EXPECT_TRUE(sensorMaskValid(kSensorEnc0 | kSensorAccelXY));
    EXPECT_FALSE(sensorMaskValid(1u << kSensorBitCount));
}

TEST(Lengths, SensorPayloadSumsWidths) {
    EXPECT_EQ(sensorPayloadLen(0), 0);
    EXPECT_EQ(sensorPayloadLen(kSensorEnc0), 4);
    EXPECT_EQ(sensorPayloadLen(kSensorEnc0 | kSensorEnc1 | kSensorGyroZ), 12);
    EXPECT_EQ(sensorPayloadLen(kSensorAccelXY), 8);
    EXPECT_EQ(sensorFrameLen(kSensorEnc0 | kSensorEnc1 | kSensorGyroZ), kSensorVector.size());
}

TEST(Lengths, LinkConstants) {
    EXPECT_EQ(kLinkEnvelopeLen, 6);
    EXPECT_EQ(kBrainPayloadMax, 122);
    EXPECT_EQ(kBrainRequestHeaderLen, 8);
    EXPECT_EQ(kBrainRequestMaxLen, 122);
    EXPECT_EQ(kBrainReplyHeaderLen, 13);
    EXPECT_EQ(kBrainReplyMaxLen, 122);
    EXPECT_EQ(kBrainStateLen, 40);
    EXPECT_EQ(kProfileWriteHeaderLen, 8);
    EXPECT_EQ(kReadDocReplyHeaderLen, 13);
    EXPECT_EQ(kPathReportHeaderLen, 6);
    EXPECT_EQ(kWheelReadingLen, 16);
    EXPECT_EQ(kSensorHeaderLen, 10);
    EXPECT_EQ(kSensorV2HeaderLen, 14);
    EXPECT_EQ(kPicoCommandHeaderLen, 6);
    EXPECT_EQ(kPicoCommandMaxLen, 8);
    EXPECT_EQ(kPicoStatusLen, 20);
}

TEST(Lengths, SensorFrameLengthsForEveryMask) {
    for (uint32_t mask = 0; mask < (1u << kSensorBitCount); ++mask) {
        size_t payload = 0;
        for (uint8_t bit = 0; bit < kSensorBitCount; ++bit) {
            if (mask & (1u << bit)) {
                payload += bit == 4 ? 8 : 4;
            }
        }
        const uint16_t m = static_cast<uint16_t>(mask);
        EXPECT_EQ(sensorPayloadLen(m), payload) << "mask " << mask;
        EXPECT_EQ(sensorFrameLen(m), 10 + payload + 1) << "mask " << mask;
        EXPECT_EQ(sensorV2FrameLen(m), 14 + payload + 1) << "mask " << mask;
    }
    EXPECT_EQ(sensorV2FrameLen(kSensorEnc0 | kSensorEnc1 | kSensorGyroZ), kSensorV2Vector.size());
    EXPECT_EQ(sensorV2FrameLen(0), kSensorV2EmptyVector.size());
}

TEST(Lengths, PicoCommandLengthForEveryOp) {
    for (int op = 0; op < 256; ++op) {
        uint8_t want = 0;
        if (op == kPicoOpConfigure || op == kPicoOpReinitImu) {
            want = 7;
        } else if (op == kPicoOpRestartAcquisition) {
            want = 6;
        }
        EXPECT_EQ(picoCommandLen(static_cast<uint8_t>(op)), want) << "op " << op;
    }
}

TEST(Lengths, ChunksFillButNeverExceedTheFrame) {
    EXPECT_LE(sensorFrameLen((1u << kSensorBitCount) - 1u), kMaxFrameLen);
    EXPECT_LE(sensorV2FrameLen((1u << kSensorBitCount) - 1u), kMaxFrameLen);
    EXPECT_EQ(brainReplyMaxLen(kOpReadWheels, kResultOk), 13 + 1 + 16 * kWheelReadingsMax);
    EXPECT_LE(brainReplyMaxLen(kOpReadWheels, kResultOk) + kLinkEnvelopeLen, kMaxFrameLen);
    EXPECT_LE(kPicoCommandMaxLen + kLinkEnvelopeLen, kMaxFrameLen);
    EXPECT_LE(kPicoStatusLen + kLinkEnvelopeLen, kMaxFrameLen);
    EXPECT_EQ(brainRequestMaxLen(kOpProfileWrite) + kLinkEnvelopeLen, kMaxFrameLen);
    EXPECT_EQ(brainReplyMaxLen(kOpReadDoc, kResultOk) + kLinkEnvelopeLen, kMaxFrameLen);
    EXPECT_LE(brainRequestMaxLen(kOpPathReport) + kLinkEnvelopeLen, kMaxFrameLen);
    // One more point would not fit.
    EXPECT_GT(kBrainRequestHeaderLen + kPathReportHeaderLen + 8 * (kPathReportMaxPoints + 1),
              kBrainPayloadMax);
}

TEST(Lengths, RequestRanges) {
    struct Range {
        uint8_t op, lo, hi;
    };
    const Range known[] = {
        {kOpHello, 12, 12},        {kOpSetPose, 20, 20},   {kOpGetState, 17, 17},
        {kOpProfileWrite, 17, 122}, {kOpProfileApply, 14, 14}, {kOpReadDoc, 16, 16},
        {kOpControl, 10, 10},      {kOpPathReport, 14, 118}, {kOpReadWheels, 8, 8},
    };
    for (const Range& r : known) {
        EXPECT_EQ(brainRequestMinLen(r.op), r.lo) << "op " << int(r.op);
        EXPECT_EQ(brainRequestMaxLen(r.op), r.hi) << "op " << int(r.op);
    }
    for (int op = 0; op < 256; ++op) {
        bool is_known = false;
        for (const Range& r : known) {
            is_known = is_known || r.op == op;
        }
        if (!is_known) {
            EXPECT_EQ(brainRequestMinLen(static_cast<uint8_t>(op)), 0) << "op " << op;
            EXPECT_EQ(brainRequestMaxLen(static_cast<uint8_t>(op)), 0) << "op " << op;
        }
    }
    EXPECT_EQ(brainRequestMinLen(3), 0);
    EXPECT_EQ(brainRequestMinLen(5), 0);
}

TEST(Lengths, ReplyRangesForEveryOpAndResult) {
    for (int op = 0; op < 256; ++op) {
        for (int result = 0; result < 256; ++result) {
            const uint8_t o  = static_cast<uint8_t>(op);
            const uint8_t r  = static_cast<uint8_t>(result);
            const bool    ok = r == kResultOk;
            uint8_t       lo = 0;
            uint8_t       hi = 0;
            if (o == kOpHello) {
                lo = hi = 17;
            } else if (isKnownResult(r)) {
                switch (o) {
                case kOpSetPose: lo = hi = (ok || r == kResultPending) ? 21 : 13; break;
                case kOpGetState: lo = hi = ok ? 53 : 13; break;
                case kOpProfileWrite: lo = hi = ok ? 19 : 13; break;
                case kOpProfileApply:
                    lo = hi = (ok || r == kResultPending || r == kResultProfileRejected) ? 20 : 13;
                    break;
                case kOpReadDoc:
                    lo = ok ? 27 : 13;
                    hi = ok ? 122 : 13;
                    break;
                case kOpControl:
                    lo = hi = (ok || r == kResultPending || r == kResultFailed) ? 16 : 13;
                    break;
                case kOpPathReport: lo = hi = 13; break;
                case kOpReadWheels:
                    lo = ok ? 14 : 13;
                    hi = ok ? 78 : 13;
                    break;
                default: break;
                }
            }
            ASSERT_EQ(brainReplyMinLen(o, r), lo) << "op " << op << " result " << result;
            ASSERT_EQ(brainReplyMaxLen(o, r), hi) << "op " << op << " result " << result;
        }
    }
    // Retired results and ops are unknown, and so is the first unused result.
    EXPECT_EQ(brainReplyMinLen(kOpGetState, 6), 0);
    EXPECT_EQ(brainReplyMinLen(kOpGetState, 7), 0);
    EXPECT_EQ(brainReplyMinLen(kOpGetState, 14), 0);
    EXPECT_EQ(brainReplyMinLen(3, kResultOk), 0);
    EXPECT_EQ(brainReplyMinLen(5, kResultOk), 0);
    EXPECT_EQ(brainReplyMinLen(12, kResultOk), 0);
}

// ---------------------------------------------------------------------------
// Sensor frame
// ---------------------------------------------------------------------------

TEST(Sensor, EncodeMatchesKnownBytes) {
    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodeSensorFrame(makeSample(), buf, sizeof(buf));
    ASSERT_EQ(n, kSensorVector.size());
    EXPECT_EQ(Bytes(buf, buf + n), kSensorVector);
}

TEST(Sensor, DecodeFromKnownBytes) {
    SensorSample s{};
    ASSERT_TRUE(decodeSensorFrame(kSensorVector.data(),
                                  static_cast<uint16_t>(kSensorVector.size()), s));
    EXPECT_EQ(s.seq, 7);
    EXPECT_EQ(s.stamp_ms, 1000u);
    EXPECT_EQ(s.mask, kSensorEnc0 | kSensorEnc1 | kSensorGyroZ);
    EXPECT_EQ(s.enc[0], 1000);
    EXPECT_EQ(s.enc[1], -500);
    EXPECT_EQ(s.gyro_z, 2500);
    // Absent sensors cost no bytes and stay zero.
    EXPECT_EQ(s.enc[2], 0);
    EXPECT_EQ(s.accel[0], 0);
    EXPECT_EQ(s.accel[1], 0);
}

TEST(Sensor, RoundTripAllSensors) {
    SensorSample in{};
    in.seq      = 255;
    in.stamp_ms = 0xDEADBEEF;
    in.mask     = kSensorEnc0 | kSensorEnc1 | kSensorEnc2 | kSensorGyroZ | kSensorAccelXY;
    in.enc[0]   = 2147483647;
    in.enc[1]   = -2147483647 - 1;
    in.enc[2]   = 0;
    in.gyro_z   = -1;
    in.accel[0] = 123;
    in.accel[1] = -456;

    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodeSensorFrame(in, buf, sizeof(buf));
    ASSERT_GT(n, 0);

    SensorSample out{};
    ASSERT_TRUE(decodeSensorFrame(buf, n, out));
    EXPECT_EQ(out.seq, in.seq);
    EXPECT_EQ(out.stamp_ms, in.stamp_ms);
    EXPECT_EQ(out.mask, in.mask);
    EXPECT_EQ(out.enc[0], in.enc[0]);
    EXPECT_EQ(out.enc[1], in.enc[1]);
    EXPECT_EQ(out.enc[2], in.enc[2]);
    EXPECT_EQ(out.gyro_z, in.gyro_z);
    EXPECT_EQ(out.accel[0], in.accel[0]);
    EXPECT_EQ(out.accel[1], in.accel[1]);
}

TEST(Sensor, RoundTripEmptyMask) {
    SensorSample in{};
    in.seq      = 1;
    in.stamp_ms = 42;

    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodeSensorFrame(in, buf, sizeof(buf));
    ASSERT_EQ(n, kSensorHeaderLen + 1);

    SensorSample out{};
    ASSERT_TRUE(decodeSensorFrame(buf, n, out));
    EXPECT_EQ(out.mask, 0);
    EXPECT_EQ(out.stamp_ms, 42u);
}

TEST(Sensor, CorruptionRejected) {
    SensorSample s{};
    for (size_t i = 0; i < kSensorVector.size(); ++i) {
        Bytes bad = kSensorVector;
        bad[i] ^= 0x01;
        EXPECT_FALSE(decodeSensorFrame(bad.data(), static_cast<uint16_t>(bad.size()), s))
            << "byte " << i;
    }
}

TEST(Sensor, TruncatedRejectedWithoutCrash) {
    SensorSample s{};
    for (uint16_t n = 0; n < kSensorVector.size(); ++n) {
        EXPECT_FALSE(decodeSensorFrame(kSensorVector.data(), n, s));
    }
}

TEST(Sensor, WrongTypeRejected) {
    SensorSample s{};
    EXPECT_FALSE(decodeSensorFrame(kGetStateRequest.data(),
                                   static_cast<uint16_t>(kGetStateRequest.size()), s));
    BrainRequest req{};
    EXPECT_FALSE(decodes(kSensorVector, req));
    EXPECT_FALSE(decodes(kHelloOkReply, req));
    BrainReply rep{};
    EXPECT_FALSE(decodes(kHelloRequest, rep));
}

TEST(Sensor, UnknownMaskBitNotEncodable) {
    SensorSample in{};
    in.mask = static_cast<uint16_t>(1u << kSensorBitCount);
    uint8_t buf[kMaxFrameLen];
    EXPECT_EQ(encodeSensorFrame(in, buf, sizeof(buf)), 0);
}

TEST(Sensor, EncodeRespectsCapacity) {
    uint8_t buf[kMaxFrameLen];
    EXPECT_EQ(encodeSensorFrame(makeSample(), buf, 22), 0);
    EXPECT_EQ(encodeSensorFrame(makeSample(), buf, 23), 23);
}

TEST(Sensor, V1DecodesWithoutIdentity) {
    SensorSample s{};
    s.identity = true;
    s.boot_id  = 5;
    ASSERT_TRUE(decodes(kSensorVector, s));
    EXPECT_FALSE(s.identity);
    EXPECT_EQ(s.boot_id, 0);
    EXPECT_EQ(s.acq_epoch, 0);
    EXPECT_EQ(s.imu_epoch, 0);
}

TEST(Sensor, V1EncodeIgnoresIdentityFields) {
    SensorSample s = makeSample();
    s.boot_id      = 0xBEEF;
    s.acq_epoch    = 3;
    EXPECT_EQ(encode(s), kSensorVector);
}

// ---------------------------------------------------------------------------
// Sensor frame v2
// ---------------------------------------------------------------------------

TEST(SensorV2, KnownVectorsCarryTheirXor) {
    for (const Bytes* v : {&kSensorVector, &kSensorV2Vector, &kSensorV2EmptyVector}) {
        EXPECT_EQ(refXor(*v), 0);
    }
}

TEST(SensorV2, EncodeMatchesKnownBytes) {
    EXPECT_EQ(encode(makeSensorV2()), kSensorV2Vector);

    SensorSample empty{};
    empty.identity = true;
    empty.boot_id  = 1;
    EXPECT_EQ(encode(empty), kSensorV2EmptyVector);
}

TEST(SensorV2, DecodeFromKnownBytes) {
    SensorSample s{};
    ASSERT_TRUE(decodes(kSensorV2Vector, s));
    EXPECT_TRUE(s.identity);
    EXPECT_EQ(s.seq, 9);
    EXPECT_EQ(s.stamp_ms, 0x01020304u);
    EXPECT_EQ(s.boot_id, 0xBEEF);
    EXPECT_EQ(s.acq_epoch, 2);
    EXPECT_EQ(s.imu_epoch, 5);
    EXPECT_EQ(s.mask, kSensorEnc0 | kSensorEnc1 | kSensorGyroZ);
    EXPECT_EQ(s.enc[0], 1000);
    EXPECT_EQ(s.enc[1], -500);
    EXPECT_EQ(s.enc[2], 0);
    EXPECT_EQ(s.gyro_z, 2500);
    EXPECT_EQ(s.accel[0], 0);

    ASSERT_TRUE(decodes(kSensorV2EmptyVector, s));
    EXPECT_TRUE(s.identity);
    EXPECT_EQ(s.boot_id, 1);
    EXPECT_EQ(s.mask, 0);
}

TEST(SensorV2, SamePayloadAsV1AfterTheLongerHeader) {
    // Only the header differs: identity sits before the mask.
    const Bytes v2 = encode(makeSensorV2());
    ASSERT_EQ(v2.size(), kSensorVector.size() + 4);
    EXPECT_EQ(Bytes(v2.begin() + 14, v2.end() - 1),
              Bytes(kSensorVector.begin() + 10, kSensorVector.end() - 1));
}

TEST(SensorV2, RoundTripEveryMaskAndExtremes) {
    for (uint32_t mask = 0; mask < (1u << kSensorBitCount); ++mask) {
        SensorSample in{};
        in.identity  = true;
        in.seq       = static_cast<uint8_t>(mask * 7);
        in.stamp_ms  = 0xFFFFFFFFu - mask;
        in.boot_id   = static_cast<uint16_t>(0xFFFF - mask);
        in.acq_epoch = 255;
        in.imu_epoch = static_cast<uint8_t>(mask);
        in.mask      = static_cast<uint16_t>(mask);
        in.enc[0]    = INT32_MIN;
        in.enc[1]    = INT32_MAX;
        in.enc[2]    = -7;
        in.gyro_z    = -123456;
        in.accel[0]  = 1000;
        in.accel[1]  = -1000;
        const Bytes f = encode(in);
        ASSERT_EQ(f.size(), sensorV2FrameLen(in.mask)) << "mask " << mask;
        EXPECT_EQ(refXor(f), 0);

        SensorSample out{};
        ASSERT_TRUE(decodes(f, out)) << "mask " << mask;
        EXPECT_TRUE(out.identity);
        EXPECT_EQ(out.seq, in.seq);
        EXPECT_EQ(out.stamp_ms, in.stamp_ms);
        EXPECT_EQ(out.boot_id, in.boot_id);
        EXPECT_EQ(out.acq_epoch, in.acq_epoch);
        EXPECT_EQ(out.imu_epoch, in.imu_epoch);
        EXPECT_EQ(out.mask, in.mask);
        EXPECT_EQ(out.enc[0], (mask & kSensorEnc0) ? INT32_MIN : 0);
        EXPECT_EQ(out.enc[1], (mask & kSensorEnc1) ? INT32_MAX : 0);
        EXPECT_EQ(out.enc[2], (mask & kSensorEnc2) ? -7 : 0);
        EXPECT_EQ(out.gyro_z, (mask & kSensorGyroZ) ? -123456 : 0);
        EXPECT_EQ(out.accel[0], (mask & kSensorAccelXY) ? 1000 : 0);
        EXPECT_EQ(out.accel[1], (mask & kSensorAccelXY) ? -1000 : 0);
    }
}

TEST(SensorV2, CorruptionRejected) {
    SensorSample s{};
    for (size_t i = 0; i < kSensorV2Vector.size(); ++i) {
        for (uint8_t flip : {0x01, 0x80}) {
            Bytes bad = kSensorV2Vector;
            bad[i] ^= flip;
            EXPECT_FALSE(decodes(bad, s)) << "byte " << i << " flip " << int(flip);
        }
    }
}

TEST(SensorV2, TruncatedAndPaddedRejected) {
    SensorSample s{};
    for (const Bytes& frame : {kSensorV2Vector, kSensorV2EmptyVector}) {
        for (uint16_t n = 0; n < frame.size(); ++n) {
            EXPECT_FALSE(decodeSensorFrame(frame.data(), n, s)) << "len " << n;
        }
        Bytes padded = frame;
        padded.push_back(0x00); // keeps the XOR at zero
        EXPECT_FALSE(decodes(padded, s));
    }
}

TEST(SensorV2, TypeSelectsTheHeader) {
    // A v2 frame relabeled v1 reads the boot id as the mask and is refused.
    Bytes as_v1 = kSensorV2Vector;
    as_v1[2]    = kFrameSensor;
    as_v1.back() ^= kFrameSensor ^ kFrameSensorV2;
    ASSERT_EQ(refXor(as_v1), 0);
    SensorSample s{};
    EXPECT_FALSE(decodes(as_v1, s));

    Bytes as_v2 = kSensorVector;
    as_v2[2]    = kFrameSensorV2;
    as_v2.back() ^= kFrameSensor ^ kFrameSensorV2;
    ASSERT_EQ(refXor(as_v2), 0);
    EXPECT_FALSE(decodes(as_v2, s));
}

TEST(SensorV2, UnknownMaskBitRejected) {
    SensorSample in = makeSensorV2();
    in.mask         = static_cast<uint16_t>(1u << kSensorBitCount);
    EXPECT_TRUE(encode(in).empty());

    // A hand built frame with bit 5 set and a valid XOR.
    Bytes f = {0xAA, 0x55, 0x04, 0x01, 0, 0, 0, 0, 0x01, 0x00, 0, 0, 0x20, 0x00};
    f.push_back(refXor(f));
    SensorSample s{};
    EXPECT_FALSE(decodes(f, s));
}

TEST(SensorV2, EncodeRespectsCapacity) {
    uint8_t buf[kMaxFrameLen];
    EXPECT_EQ(encodeSensorFrame(makeSensorV2(), buf, 26), 0);
    EXPECT_EQ(encodeSensorFrame(makeSensorV2(), buf, 27), 27);
}

TEST(SensorV2, OtherFrameTypesRejected) {
    SensorSample s{};
    for (const Bytes* f : {&kPicoStatusVector, &kPicoConfigureCommand, &kGetStateRequest}) {
        EXPECT_FALSE(decodes(*f, s));
    }
    BrainRequest req{};
    EXPECT_FALSE(decodes(kSensorV2Vector, req));
    PicoStatus status{};
    EXPECT_FALSE(decodes(kSensorV2Vector, status));
}

// ---------------------------------------------------------------------------
// Brain requests, known bytes
// ---------------------------------------------------------------------------

TEST(BrainRequest, HelloKnownBytes) {
    BrainRequest r = makeRequest(kOpHello, 0, 1);
    r.nonce        = kNonce;
    EXPECT_EQ(encode(r), kHelloRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kHelloRequest, out));
    EXPECT_EQ(out.version, 4);
    EXPECT_EQ(out.op, kOpHello);
    EXPECT_EQ(out.session, 0u);
    EXPECT_EQ(out.request_id, 1);
    EXPECT_EQ(out.nonce, kNonce);
}

TEST(BrainRequest, SetPoseKnownBytes) {
    BrainRequest r = makeRequest(kOpSetPose, kSession, 2);
    r.x_mm         = 610;
    r.y_mm         = -457;
    r.heading_cdeg = -9000;
    EXPECT_EQ(encode(r), kSetPoseRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kSetPoseRequest, out));
    EXPECT_EQ(out.op, kOpSetPose);
    EXPECT_EQ(out.session, kSession);
    EXPECT_EQ(out.request_id, 2);
    EXPECT_EQ(out.x_mm, 610);
    EXPECT_EQ(out.y_mm, -457);
    EXPECT_EQ(out.heading_cdeg, -9000);
}

TEST(BrainRequest, GetStateWithoutBenchSampleKnownBytes) {
    EXPECT_EQ(encode(makeRequest(kOpGetState, kSession, 0xFFFF)), kGetStateRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kGetStateRequest, out));
    EXPECT_EQ(out.op, kOpGetState);
    EXPECT_EQ(out.session, kSession);
    EXPECT_EQ(out.request_id, 0xFFFF);
    EXPECT_EQ(out.imu_flags, 0);
    EXPECT_EQ(out.imu_stamp_ms, 0u);
    EXPECT_EQ(out.imu_rotation_mdeg, 0);
}

TEST(BrainRequest, GetStateWithBenchSampleKnownBytes) {
    BrainRequest r      = makeRequest(kOpGetState, kSession, 42);
    r.imu_flags         = kBenchImuValid;
    r.imu_stamp_ms      = 0x12345678u;
    r.imu_rotation_mdeg = -450123;
    EXPECT_EQ(encode(r), kGetStateImuRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kGetStateImuRequest, out));
    EXPECT_EQ(out.op, kOpGetState);
    EXPECT_EQ(out.request_id, 42);
    EXPECT_EQ(out.imu_flags, kBenchImuValid);
    EXPECT_EQ(out.imu_stamp_ms, 0x12345678u);
    EXPECT_EQ(out.imu_rotation_mdeg, -450123);
}

TEST(BrainRequest, GetStateCarriesUnknownImuFlagsForThePiToRefuse) {
    BrainRequest r = makeRequest(kOpGetState, kSession, 3);
    r.imu_flags    = 0xFE;
    BrainRequest out{};
    ASSERT_TRUE(decodes(encode(r), out));
    EXPECT_EQ(out.imu_flags, 0xFE);
}

TEST(BrainRequest, ProfileWriteMinChunkKnownBytes) {
    BrainRequest r = makeRequest(kOpProfileWrite, kSession, 5);
    r.profile_id   = kProfileId;
    r.total_len    = 208;
    r.offset       = 106;
    r.data_len     = 1;
    r.data[0]      = 0x5A;
    r.data[1]      = 0x77; // beyond data_len, not sent
    EXPECT_EQ(encode(r), kProfileWriteRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kProfileWriteRequest, out));
    EXPECT_EQ(out.op, kOpProfileWrite);
    EXPECT_EQ(out.request_id, 5);
    EXPECT_EQ(out.profile_id, kProfileId);
    EXPECT_EQ(out.total_len, 208);
    EXPECT_EQ(out.offset, 106);
    EXPECT_EQ(out.data_len, 1);
    EXPECT_EQ(out.data[0], 0x5A);
    EXPECT_EQ(out.data[1], 0);
}

TEST(BrainRequest, ProfileWriteMaxChunkFillsTheFrame) {
    const Bytes expected = maxProfileWriteBytes();
    ASSERT_EQ(expected.size(), kMaxFrameLen);
    EXPECT_EQ(encode(maxProfileWrite()), expected);

    BrainRequest out{};
    ASSERT_TRUE(decodes(expected, out));
    EXPECT_EQ(out.profile_id, kProfileId);
    EXPECT_EQ(out.total_len, 208);
    EXPECT_EQ(out.offset, 0);
    ASSERT_EQ(out.data_len, kProfileChunkMax);
    for (size_t i = 0; i < kProfileChunkMax; ++i) {
        EXPECT_EQ(out.data[i], pattern(i)) << "byte " << i;
    }
}

TEST(BrainRequest, ProfileApplyKnownBytes) {
    BrainRequest r = makeRequest(kOpProfileApply, kSession, 6);
    r.profile_id   = kProfileId;
    r.total_len    = 208;
    EXPECT_EQ(encode(r), kProfileApplyRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kProfileApplyRequest, out));
    EXPECT_EQ(out.op, kOpProfileApply);
    EXPECT_EQ(out.profile_id, kProfileId);
    EXPECT_EQ(out.total_len, 208);
}

TEST(BrainRequest, ReadDocKnownBytes) {
    BrainRequest r = makeRequest(kOpReadDoc, kSession, 7);
    r.doc_kind     = kDocFieldMap;
    r.doc_id       = kMapId;
    r.doc_offset   = 192;
    r.max_len      = kDocChunkMax;
    EXPECT_EQ(encode(r), kReadDocRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kReadDocRequest, out));
    EXPECT_EQ(out.op, kOpReadDoc);
    EXPECT_EQ(out.doc_kind, kDocFieldMap);
    EXPECT_EQ(out.doc_id, kMapId);
    EXPECT_EQ(out.doc_offset, 192);
    EXPECT_EQ(out.max_len, kDocChunkMax);
}

TEST(BrainRequest, ControlKnownBytes) {
    BrainRequest r = makeRequest(kOpControl, kSession, 8);
    r.action       = kControlRecalibrate;
    EXPECT_EQ(encode(r), kControlRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kControlRequest, out));
    EXPECT_EQ(out.op, kOpControl);
    EXPECT_EQ(out.action, kControlRecalibrate);
    EXPECT_EQ(out.action_arg, 0);
}

TEST(BrainRequest, ControlReinitImuKnownBytes) {
    BrainRequest r = makeRequest(kOpControl, kSession, 12);
    r.action       = kControlReinitImu;
    EXPECT_EQ(encode(r), kControlReinitImuRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kControlReinitImuRequest, out));
    EXPECT_EQ(out.action, kControlReinitImu);
}

TEST(BrainRequest, ControlCarriesAnyActionAndArg) {
    // Unknown actions are the Pi's to refuse, so the codec passes them.
    for (int action = 0; action < 256; action += 17) {
        BrainRequest r = makeRequest(kOpControl, kSession, 3);
        r.action       = static_cast<uint8_t>(action);
        r.action_arg   = static_cast<uint8_t>(255 - action);
        BrainRequest out{};
        ASSERT_TRUE(decodes(encode(r), out));
        EXPECT_EQ(out.action, action);
        EXPECT_EQ(out.action_arg, 255 - action);
    }
}

TEST(BrainRequest, ReadWheelsKnownBytes) {
    BrainRequest r = makeRequest(kOpReadWheels, kSession, 11);
    r.action       = 4; // other ops' fields are not sent
    r.nonce        = 1;
    EXPECT_EQ(encode(r), kReadWheelsRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kReadWheelsRequest, out));
    EXPECT_EQ(out.op, kOpReadWheels);
    EXPECT_EQ(out.session, kSession);
    EXPECT_EQ(out.request_id, 11);
    EXPECT_EQ(out.action, 0);
}

TEST(BrainRequest, PathReportEmptyKnownBytes) {
    BrainRequest r = makeRequest(kOpPathReport, kSession, 9);
    r.command_id   = 0x01020304;
    r.path_mode    = kPathNone;
    r.points[0]    = {5, 5}; // beyond point_count, not sent
    EXPECT_EQ(encode(r), kPathClearRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kPathClearRequest, out));
    EXPECT_EQ(out.op, kOpPathReport);
    EXPECT_EQ(out.command_id, 0x01020304u);
    EXPECT_EQ(out.path_mode, kPathNone);
    EXPECT_EQ(out.point_count, 0);
    EXPECT_EQ(out.points[0].x_mm, 0);
}

TEST(BrainRequest, PathReportKnownBytes) {
    BrainRequest r = makeRequest(kOpPathReport, kSession, 10);
    r.command_id   = 77;
    r.path_mode    = kPathAvoiding;
    r.point_count  = 2;
    r.points[0]    = {1000, -2000};
    r.points[1]    = {-1, 32767};
    EXPECT_EQ(encode(r), kPathRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kPathRequest, out));
    EXPECT_EQ(out.command_id, 77u);
    EXPECT_EQ(out.path_mode, kPathAvoiding);
    ASSERT_EQ(out.point_count, 2);
    EXPECT_EQ(out.points[0].x_mm, 1000);
    EXPECT_EQ(out.points[0].y_mm, -2000);
    EXPECT_EQ(out.points[1].x_mm, -1);
    EXPECT_EQ(out.points[1].y_mm, 32767);
}

TEST(BrainRequest, PathReportMaxPoints) {
    BrainRequest r = makeRequest(kOpPathReport, kSession, 13);
    r.command_id   = 0xFFFFFFFF;
    r.path_mode    = kPathDirect;
    r.point_count  = kPathReportMaxPoints;
    Le p           = requestHead(kOpPathReport, kSession, 13);
    p.u32(0xFFFFFFFF).u8(kPathDirect).u8(kPathReportMaxPoints);
    for (int i = 0; i < kPathReportMaxPoints; ++i) {
        r.points[i] = {i * 1000 - 6000, -i * 7 + INT32_MAX - 100};
        p.i32(r.points[i].x_mm).i32(r.points[i].y_mm);
    }
    const Bytes expected = linkFrame(kFrameBrainRequest, p.b);
    ASSERT_EQ(expected.size(), 124u);
    EXPECT_EQ(encode(r), expected);

    BrainRequest out{};
    ASSERT_TRUE(decodes(expected, out));
    ASSERT_EQ(out.point_count, kPathReportMaxPoints);
    for (int i = 0; i < kPathReportMaxPoints; ++i) {
        EXPECT_EQ(out.points[i].x_mm, r.points[i].x_mm);
        EXPECT_EQ(out.points[i].y_mm, r.points[i].y_mm);
    }
}

TEST(BrainRequest, BodyFieldsOfOtherOpsAreNotEncoded) {
    BrainRequest r = makeRequest(kOpGetState, kSession, 0xFFFF);
    r.nonce        = 99;
    r.x_mm         = 99;
    r.profile_id   = 99;
    r.data_len     = 5;
    r.doc_kind     = 1;
    r.action       = 2;
    r.point_count  = 3;
    EXPECT_EQ(encode(r), kGetStateRequest);
}

TEST(BrainRequest, RoundTripExtremes) {
    BrainRequest r = makeRequest(kOpSetPose, 0xFFFFFFFF, 0x8000);
    r.x_mm         = INT32_MIN;
    r.y_mm         = INT32_MAX;
    r.heading_cdeg = 18000;

    BrainRequest out{};
    ASSERT_TRUE(decodes(encode(r), out));
    EXPECT_EQ(out.session, 0xFFFFFFFFu);
    EXPECT_EQ(out.request_id, 0x8000);
    EXPECT_EQ(out.x_mm, INT32_MIN);
    EXPECT_EQ(out.y_mm, INT32_MAX);
    EXPECT_EQ(out.heading_cdeg, 18000);
}

TEST(BrainRequest, RequestIdZeroDecodes) {
    // The Pi answers it with kResultInvalidArgument, so it must reach the Pi.
    BrainRequest out{};
    ASSERT_TRUE(decodes(encode(makeRequest(kOpGetState, kSession, 0)), out));
    EXPECT_EQ(out.request_id, 0);
}

TEST(BrainRequest, EncoderRefusesVariableBodiesOutOfRange) {
    uint8_t      buf[kMaxFrameLen];
    BrainRequest w = maxProfileWrite();
    w.data_len     = 0;
    EXPECT_EQ(encodeBrainRequest(w, buf, sizeof(buf)), 0);
    w.data_len = kProfileChunkMax + 1;
    EXPECT_EQ(encodeBrainRequest(w, buf, sizeof(buf)), 0);

    BrainRequest p = makeRequest(kOpPathReport, kSession, 1);
    p.point_count  = kPathReportMaxPoints + 1;
    EXPECT_EQ(encodeBrainRequest(p, buf, sizeof(buf)), 0);
    p.point_count = 255;
    EXPECT_EQ(encodeBrainRequest(p, buf, sizeof(buf)), 0);
}

TEST(BrainRequest, EncodeRespectsCapacity) {
    uint8_t      buf[kMaxFrameLen];
    BrainRequest req = makeRequest(kOpGetState, kSession, 1);
    EXPECT_EQ(encodeBrainRequest(req, buf, 22), 0);
    EXPECT_EQ(encodeBrainRequest(req, buf, 23), 23);
    EXPECT_EQ(encodeBrainRequest(maxProfileWrite(), buf, 127), 0);
    EXPECT_EQ(encodeBrainRequest(maxProfileWrite(), buf, 128), 128);
}

// ---------------------------------------------------------------------------
// Brain requests, length ranges, unknown ops and versions
// ---------------------------------------------------------------------------

TEST(BrainRequest, KnownOpsAcceptOnlyTheirLengthRange) {
    const uint8_t ops[] = {kOpHello,        kOpSetPose, kOpGetState, kOpProfileWrite,
                           kOpProfileApply, kOpReadDoc, kOpControl,  kOpReadWheels};
    for (uint8_t op : ops) {
        const uint8_t lo = brainRequestMinLen(op);
        const uint8_t hi = brainRequestMaxLen(op);
        for (size_t n = kBrainRequestHeaderLen; n <= kBrainPayloadMax; ++n) {
            BrainRequest out{};
            const bool   in_range = n >= lo && n <= hi;
            EXPECT_EQ(decodes(linkFrame(kFrameBrainRequest, requestPayload(4, op, n)), out),
                      in_range)
                << "op " << int(op) << " len " << n;
            if (in_range && op == kOpProfileWrite) {
                EXPECT_EQ(out.data_len, n - kBrainRequestHeaderLen - kProfileWriteHeaderLen);
            }
        }
    }
}

TEST(BrainRequest, PathReportLengthMustMatchCount) {
    for (int count = 0; count <= 255; ++count) {
        for (size_t n = kBrainRequestHeaderLen; n <= kBrainPayloadMax; ++n) {
            Bytes p = requestPayload(4, kOpPathReport, n);
            if (n > kBrainRequestHeaderLen + 5) {
                p[kBrainRequestHeaderLen + 5] = static_cast<uint8_t>(count);
            }
            const bool expected = count <= kPathReportMaxPoints &&
                                  n == kBrainRequestHeaderLen + kPathReportHeaderLen + 8u * count;
            BrainRequest out{};
            ASSERT_EQ(decodes(linkFrame(kFrameBrainRequest, p), out), expected)
                << "count " << count << " len " << n;
        }
    }
}

TEST(BrainRequest, UnknownAndRetiredOpsDecodeHeaderOnly) {
    // Retired v4 op numbers with their old v3 bodies, and never used ones.
    const struct {
        uint8_t op;
        size_t  len;
    } cases[] = {{0, 8}, {3, 10}, {5, 17}, {12, 40}, {0x7F, 122}, {0xFF, 13}};
    for (const auto& c : cases) {
        Bytes p = requestPayload(4, c.op, c.len);
        for (size_t i = kBrainRequestHeaderLen; i < c.len; ++i) {
            p[i] = 0x11;
        }
        BrainRequest out{};
        ASSERT_TRUE(decodes(linkFrame(kFrameBrainRequest, p), out)) << "op " << int(c.op);
        EXPECT_EQ(out.version, 4);
        EXPECT_EQ(out.op, c.op);
        EXPECT_EQ(out.request_id, 9);
        EXPECT_EQ(out.imu_flags, 0);
        EXPECT_EQ(out.nonce, 0u);
        EXPECT_EQ(out.data_len, 0);
        EXPECT_EQ(out.point_count, 0);
    }
}

TEST(BrainRequest, EncodingUnknownOpWritesHeaderOnly) {
    BrainRequest r = makeRequest(0x7F, kSession, 4);
    r.nonce        = 1;
    const Bytes f  = encode(r);
    ASSERT_EQ(f.size(), 14u);
    EXPECT_EQ(f[3], 8);
    EXPECT_EQ(encode(makeRequest(3, kSession, 4)).size(), 14u);
    EXPECT_EQ(encode(makeRequest(5, kSession, 4)).size(), 14u);
    EXPECT_EQ(encode(makeRequest(12, kSession, 4)).size(), 14u);
}

TEST(BrainRequest, OtherVersionDecodesHeaderOnly) {
    // A v3 Brain's GET_STATE (no body) and GET_STATE_WITH_IMU.
    BrainRequest out{};
    ASSERT_TRUE(decodes(linkFrame(kFrameBrainRequest, requestPayload(3, kOpGetState, 8)), out));
    EXPECT_EQ(out.version, 3);
    EXPECT_EQ(out.op, kOpGetState);
    EXPECT_EQ(out.request_id, 9);

    Bytes v3imu = requestPayload(3, 5, 17);
    v3imu[8]    = kBenchImuValid;
    ASSERT_TRUE(decodes(linkFrame(kFrameBrainRequest, v3imu), out));
    EXPECT_EQ(out.version, 3);
    EXPECT_EQ(out.op, 5);
    EXPECT_EQ(out.imu_flags, 0);

    // A later version with a v4 op.
    BrainRequest r = makeRequest(kOpSetPose, kSession, 7);
    r.version      = 5;
    r.x_mm         = 1234;
    ASSERT_TRUE(decodes(encode(r), out));
    EXPECT_EQ(out.version, 5);
    EXPECT_EQ(out.op, kOpSetPose);
    EXPECT_EQ(out.session, kSession);
    EXPECT_EQ(out.request_id, 7);
    EXPECT_EQ(out.x_mm, 0);

    // A length that is wrong for v4 is fine for another version.
    ASSERT_TRUE(decodes(linkFrame(kFrameBrainRequest, requestPayload(2, kOpHello, 30)), out));
    EXPECT_EQ(out.version, 2);
    EXPECT_EQ(out.nonce, 0u);
}

TEST(BrainRequest, EnvelopeLenBounds) {
    BrainRequest out{};
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainRequest, Bytes(7, 0x04)), out));
    EXPECT_TRUE(decodes(linkFrame(kFrameBrainRequest, requestPayload(4, 0x7F, 8)), out));
    EXPECT_TRUE(decodes(linkFrame(kFrameBrainRequest, requestPayload(4, 0x7F, 122)), out));
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainRequest, requestPayload(4, 0x7F, 123)), out));
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainRequest, requestPayload(4, 0x7F, 255)), out));
}

TEST(BrainRequest, CorruptionRejected) {
    for (const Bytes& frame : {kSetPoseRequest, kPathRequest, maxProfileWriteBytes(),
                               kReadWheelsRequest, kControlReinitImuRequest}) {
        for (size_t i = 0; i < frame.size(); ++i) {
            Bytes bad = frame;
            bad[i] ^= 0x01;
            BrainRequest out{};
            EXPECT_FALSE(decodes(bad, out)) << "size " << frame.size() << " byte " << i;
        }
    }
}

TEST(BrainRequest, TruncatedAndPaddedRejected) {
    for (const Bytes& frame : {kSetPoseRequest, maxProfileWriteBytes()}) {
        BrainRequest out{};
        for (uint16_t n = 0; n < frame.size(); ++n) {
            EXPECT_FALSE(decodeBrainRequest(frame.data(), n, out));
        }
        Bytes padded = frame;
        padded.push_back(0x00);
        EXPECT_FALSE(decodes(padded, out));
    }
}

// ---------------------------------------------------------------------------
// Brain replies, known bytes
// ---------------------------------------------------------------------------

TEST(BrainReply, HelloOkKnownBytes) {
    BrainReply r = makeReply(kOpHello, kSession, 1, kResultOk);
    r.nonce      = kNonce;
    EXPECT_EQ(encode(r), kHelloOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kHelloOkReply, out));
    EXPECT_EQ(out.version, 4);
    EXPECT_EQ(out.op, kOpHello);
    EXPECT_EQ(out.session, kSession);
    EXPECT_EQ(out.request_id, 1);
    EXPECT_EQ(out.result, kResultOk);
    EXPECT_EQ(out.pi_instance, kInstance);
    EXPECT_EQ(out.nonce, kNonce);
}

TEST(BrainReply, HelloStaleKnownBytes) {
    BrainReply r = makeReply(kOpHello, 0, 1, kResultStale);
    r.nonce      = kNonce;
    EXPECT_EQ(encode(r), kHelloStaleReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kHelloStaleReply, out));
    EXPECT_EQ(out.result, kResultStale);
    EXPECT_EQ(out.session, 0u);
    EXPECT_EQ(out.nonce, kNonce);
}

TEST(BrainReply, HelloCarriesTheNonceForEveryResult) {
    for (int result = 0; result < 256; ++result) {
        BrainReply r = makeReply(kOpHello, 0, 1, static_cast<uint8_t>(result));
        r.nonce      = kNonce;
        const Bytes f = encode(r);
        ASSERT_EQ(f.size(), 23u) << "result " << result;
        BrainReply out{};
        ASSERT_TRUE(decodes(f, out));
        EXPECT_EQ(out.nonce, kNonce);
    }
}

TEST(BrainReply, SetPoseOkKnownBytes) {
    BrainReply r      = makeReply(kOpSetPose, kSession, 2, kResultOk);
    r.odometry_epoch  = 2;
    r.anchor_revision = 7;
    EXPECT_EQ(encode(r), kSetPoseOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kSetPoseOkReply, out));
    EXPECT_EQ(out.op, kOpSetPose);
    EXPECT_EQ(out.result, kResultOk);
    EXPECT_EQ(out.odometry_epoch, 2u);
    EXPECT_EQ(out.anchor_revision, 7u);
}

TEST(BrainReply, SetPosePendingKnownBytes) {
    BrainReply r      = makeReply(kOpSetPose, kSession, 2, kResultPending);
    r.odometry_epoch  = 2;
    r.anchor_revision = 6;
    EXPECT_EQ(encode(r), kSetPosePendingReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kSetPosePendingReply, out));
    EXPECT_EQ(out.result, kResultPending);
    EXPECT_EQ(out.odometry_epoch, 2u);
    EXPECT_EQ(out.anchor_revision, 6u);
}

TEST(BrainReply, SetPoseNotReadyHasNoBody) {
    BrainReply r      = makeReply(kOpSetPose, kSession, 2, kResultNotReady);
    r.odometry_epoch  = 2;
    r.anchor_revision = 7;
    EXPECT_EQ(encode(r), kSetPoseNotReadyReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kSetPoseNotReadyReply, out));
    EXPECT_EQ(out.result, kResultNotReady);
    EXPECT_EQ(out.pi_instance, kInstance);
    EXPECT_EQ(out.odometry_epoch, 0u);
    EXPECT_EQ(out.anchor_revision, 0u);
}

TEST(BrainReply, GetStateOkKnownBytes) {
    BrainReply r = makeReply(kOpGetState, kSession, 0xFFFF, kResultOk);
    r.state      = makeState();
    EXPECT_EQ(encode(r), kGetStateOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kGetStateOkReply, out));
    EXPECT_EQ(out.op, kOpGetState);
    EXPECT_EQ(out.request_id, 0xFFFF);
    const BrainState& s = out.state;
    EXPECT_EQ(s.robot_flags, 0x0F);
    EXPECT_EQ(s.x_mm, 1500);
    EXPECT_EQ(s.y_mm, -250);
    EXPECT_EQ(s.heading_cdeg, 18000);
    EXPECT_EQ(s.robot_age_ms, 35);
    EXPECT_EQ(s.odometry_epoch, 2u);
    EXPECT_EQ(s.anchor_revision, 7u);
    EXPECT_EQ(s.health, 0x0B);
    EXPECT_EQ(s.profile_state, kProfileRejected);
    EXPECT_EQ(s.profile_reason, kProfileReasonObservability);
    EXPECT_EQ(s.profile_detail, 1);
    EXPECT_EQ(s.profile_id, kProfileId);
    EXPECT_EQ(s.map_id, kMapId);
    EXPECT_EQ(s.estimate_id, 0x102u);
    EXPECT_EQ(s.calibration, kCalibrationRunning);
}

TEST(BrainReply, StateBlockOffsets) {
    BrainReply  r = makeReply(kOpGetState, kSession, 1, kResultOk);
    BrainState& s = r.state;
    s.robot_flags     = 0xA1;
    s.x_mm            = 0x04030201;
    s.y_mm            = 0x08070605;
    s.heading_cdeg    = 0x0C0B0A09;
    s.robot_age_ms    = 0x0E0D;
    s.odometry_epoch  = 0x1211100F;
    s.anchor_revision = 0x16151413;
    s.health          = 0xA2;
    s.profile_state   = 0xA3;
    s.profile_reason  = 0xA4;
    s.profile_id      = 0x1A191817;
    s.map_id          = 0x1E1D1C1B;
    s.estimate_id     = 0x2221201F;
    s.calibration     = 0xA5;
    s.profile_detail  = 0xA6;

    const Bytes f = encode(r);
    ASSERT_EQ(f.size(), kLinkEnvelopeLen + kBrainReplyHeaderLen + kBrainStateLen);
    const Bytes body(f.begin() + 4 + kBrainReplyHeaderLen, f.end() - 2);
    const Bytes expected = {
        0xA1,                   // 0 robot_flags
        0x01, 0x02, 0x03, 0x04, // 1 x_mm
        0x05, 0x06, 0x07, 0x08, // 5 y_mm
        0x09, 0x0A, 0x0B, 0x0C, // 9 heading_cdeg
        0x0D, 0x0E,             // 13 robot_age_ms
        0x0F, 0x10, 0x11, 0x12, // 15 odometry_epoch
        0x13, 0x14, 0x15, 0x16, // 19 anchor_revision
        0xA2,                   // 23 health
        0xA3,                   // 24 profile_state
        0xA4,                   // 25 profile_reason
        0x17, 0x18, 0x19, 0x1A, // 26 profile_id
        0x1B, 0x1C, 0x1D, 0x1E, // 30 map_id
        0x1F, 0x20, 0x21, 0x22, // 34 estimate_id
        0xA5,                   // 38 calibration
        0xA6,                   // 39 profile_detail
    };
    EXPECT_EQ(body, expected);

    BrainReply out{};
    ASSERT_TRUE(decodes(f, out));
    EXPECT_EQ(out.state.heading_cdeg, 0x0C0B0A09);
    EXPECT_EQ(out.state.calibration, 0xA5);
    EXPECT_EQ(out.state.profile_detail, 0xA6);
}

TEST(BrainReply, GetStateRoundTripExtremes) {
    BrainReply r            = makeReply(kOpGetState, 1, 1, kResultOk);
    r.state.x_mm            = INT32_MIN;
    r.state.y_mm            = INT32_MAX;
    r.state.heading_cdeg    = -17999;
    r.state.robot_age_ms    = 65535;
    r.state.odometry_epoch  = 0xFFFFFFFF;
    r.state.anchor_revision = 0x80000000;
    r.state.robot_flags     = 0xFF;
    r.state.estimate_id     = 0xFFFFFFFF;

    BrainReply out{};
    ASSERT_TRUE(decodes(encode(r), out));
    EXPECT_EQ(out.state.x_mm, INT32_MIN);
    EXPECT_EQ(out.state.y_mm, INT32_MAX);
    EXPECT_EQ(out.state.heading_cdeg, -17999);
    EXPECT_EQ(out.state.robot_age_ms, 65535);
    EXPECT_EQ(out.state.odometry_epoch, 0xFFFFFFFFu);
    EXPECT_EQ(out.state.anchor_revision, 0x80000000u);
    EXPECT_EQ(out.state.robot_flags, 0xFF);
    EXPECT_EQ(out.state.estimate_id, 0xFFFFFFFFu);
}

TEST(BrainReply, ProfileWriteOkKnownBytes) {
    BrainReply r = makeReply(kOpProfileWrite, kSession, 5, kResultOk);
    r.profile_id = kProfileId;
    r.received   = 107;
    EXPECT_EQ(encode(r), kProfileWriteOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kProfileWriteOkReply, out));
    EXPECT_EQ(out.op, kOpProfileWrite);
    EXPECT_EQ(out.profile_id, kProfileId);
    EXPECT_EQ(out.received, 107);
}

TEST(BrainReply, ProfileApplyPendingKnownBytes) {
    BrainReply r    = makeReply(kOpProfileApply, kSession, 6, kResultPending);
    r.profile_id    = kProfileId;
    r.profile_state = kProfileApplying;
    EXPECT_EQ(encode(r), kProfileApplyPendingReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kProfileApplyPendingReply, out));
    EXPECT_EQ(out.result, kResultPending);
    EXPECT_EQ(out.profile_id, kProfileId);
    EXPECT_EQ(out.profile_state, kProfileApplying);
    EXPECT_EQ(out.profile_reason, kProfileReasonNone);
}

TEST(BrainReply, ProfileRejectedKnownBytes) {
    BrainReply r     = makeReply(kOpProfileApply, kSession, 6, kResultProfileRejected);
    r.profile_id     = kProfileId;
    r.profile_state  = kProfileRejected;
    r.profile_reason = kProfileReasonObservability;
    r.profile_detail = 1;
    EXPECT_EQ(encode(r), kProfileRejectedReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kProfileRejectedReply, out));
    EXPECT_EQ(out.result, kResultProfileRejected);
    EXPECT_EQ(out.profile_id, kProfileId);
    EXPECT_EQ(out.profile_state, kProfileRejected);
    EXPECT_EQ(out.profile_reason, kProfileReasonObservability);
    EXPECT_EQ(out.profile_detail, 1);
}

TEST(BrainReply, ProfileApplyOkMatchesBuiltBytes) {
    BrainReply r    = makeReply(kOpProfileApply, kSession, 6, kResultOk);
    r.profile_id    = kProfileId;
    r.profile_state = kProfileApplied;
    Le p            = replyHead(kOpProfileApply, kSession, 6, kResultOk);
    p.u32(kProfileId).u8(kProfileApplied).u8(0).u8(0);
    EXPECT_EQ(encode(r), linkFrame(kFrameBrainReply, p.b));
}

TEST(BrainReply, ReadDocMinChunkKnownBytes) {
    BrainReply r    = makeReply(kOpReadDoc, kSession, 7, kResultOk);
    r.doc_kind      = kDocFieldEstimate;
    r.doc_id        = 5;
    r.doc_total_len = 44;
    r.doc_crc32     = 0x11223344;
    r.doc_offset    = 43;
    r.data_len      = 1;
    r.data[0]       = 0xEE;
    EXPECT_EQ(encode(r), kReadDocOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kReadDocOkReply, out));
    EXPECT_EQ(out.doc_kind, kDocFieldEstimate);
    EXPECT_EQ(out.doc_id, 5u);
    EXPECT_EQ(out.doc_total_len, 44);
    EXPECT_EQ(out.doc_crc32, 0x11223344u);
    EXPECT_EQ(out.doc_offset, 43);
    ASSERT_EQ(out.data_len, 1);
    EXPECT_EQ(out.data[0], 0xEE);
}

TEST(BrainReply, ReadDocMaxChunkFillsTheFrame) {
    const Bytes expected = maxReadDocBytes();
    ASSERT_EQ(expected.size(), kMaxFrameLen);
    EXPECT_EQ(encode(maxReadDoc()), expected);

    BrainReply out{};
    ASSERT_TRUE(decodes(expected, out));
    EXPECT_EQ(out.doc_kind, kDocFieldMap);
    EXPECT_EQ(out.doc_id, kMapId);
    EXPECT_EQ(out.doc_total_len, 3608);
    EXPECT_EQ(out.doc_crc32, kMapId);
    EXPECT_EQ(out.doc_offset, 3456);
    ASSERT_EQ(out.data_len, kDocChunkMax);
    for (size_t i = 0; i < kDocChunkMax; ++i) {
        EXPECT_EQ(out.data[i], pattern(i + 1)) << "byte " << i;
    }
}

TEST(BrainReply, ReadDocEveryDataLength) {
    for (uint8_t n = 1; n <= kDocChunkMax; ++n) {
        BrainReply r = maxReadDoc();
        r.data_len   = n;
        const Bytes f = encode(r);
        ASSERT_EQ(f.size(), kLinkEnvelopeLen + kBrainReplyHeaderLen + kReadDocReplyHeaderLen + n);
        BrainReply out{};
        ASSERT_TRUE(decodes(f, out));
        EXPECT_EQ(out.data_len, n);
        EXPECT_EQ(out.data[n - 1], pattern(n));
    }
}

TEST(BrainReply, ReadDocUnavailableHasNoBody) {
    BrainReply r = maxReadDoc();
    r.request_id = 7;
    r.result     = kResultUnavailable;
    EXPECT_EQ(encode(r), kReadDocUnavailableReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kReadDocUnavailableReply, out));
    EXPECT_EQ(out.result, kResultUnavailable);
    EXPECT_EQ(out.data_len, 0);
    EXPECT_EQ(out.doc_id, 0u);
}

TEST(BrainReply, ControlOkKnownBytes) {
    BrainReply r  = makeReply(kOpControl, kSession, 8, kResultOk);
    r.action      = kControlReinitialize;
    r.calibration = kCalibrationRunning;
    EXPECT_EQ(encode(r), kControlOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kControlOkReply, out));
    EXPECT_EQ(out.result, kResultOk);
    EXPECT_EQ(out.action, kControlReinitialize);
    EXPECT_EQ(out.calibration, kCalibrationRunning);
    EXPECT_EQ(out.control_detail, kControlDetailNone);
}

TEST(BrainReply, ControlPendingKnownBytes) {
    BrainReply r  = makeReply(kOpControl, kSession, 12, kResultPending);
    r.action      = kControlReinitImu;
    r.calibration = kCalibrationWaitingStill;
    EXPECT_EQ(encode(r), kControlPendingReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kControlPendingReply, out));
    EXPECT_EQ(out.result, kResultPending);
    EXPECT_EQ(out.action, kControlReinitImu);
    EXPECT_EQ(out.calibration, kCalibrationWaitingStill);
    EXPECT_EQ(out.control_detail, kControlDetailNone);
}

TEST(BrainReply, ControlFailedKnownBytes) {
    BrainReply r     = makeReply(kOpControl, kSession, 13, kResultFailed);
    r.action         = kControlRestartAcquisition;
    r.calibration    = kCalibrationFailed;
    r.control_detail = kControlDetailTimedOut;
    EXPECT_EQ(encode(r), kControlFailedReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kControlFailedReply, out));
    EXPECT_EQ(out.result, kResultFailed);
    EXPECT_EQ(out.action, kControlRestartAcquisition);
    EXPECT_EQ(out.calibration, kCalibrationFailed);
    EXPECT_EQ(out.control_detail, kControlDetailTimedOut);
}

TEST(BrainReply, ControlBodyByteOrder) {
    for (uint8_t result : {kResultOk, kResultPending, kResultFailed}) {
        BrainReply r     = makeReply(kOpControl, kSession, 1, result);
        r.action         = 0xA1;
        r.calibration    = 0xA2;
        r.control_detail = 0xA3;
        const Bytes f    = encode(r);
        ASSERT_EQ(f.size(), kLinkEnvelopeLen + kBrainReplyHeaderLen + 3u);
        EXPECT_EQ(f[17], 0xA1);
        EXPECT_EQ(f[18], 0xA2);
        EXPECT_EQ(f[19], 0xA3);
        BrainReply out{};
        ASSERT_TRUE(decodes(f, out));
        EXPECT_EQ(out.action, 0xA1);
        EXPECT_EQ(out.calibration, 0xA2);
        EXPECT_EQ(out.control_detail, 0xA3);
    }
}

TEST(BrainReply, ControlNotStationaryHasNoBody) {
    BrainReply r     = makeReply(kOpControl, kSession, 8, kResultNotStationary);
    r.action         = kControlReinitialize;
    r.calibration    = kCalibrationRunning;
    r.control_detail = kControlDetailCalibration;
    EXPECT_EQ(encode(r), kControlNotStationaryReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kControlNotStationaryReply, out));
    EXPECT_EQ(out.result, kResultNotStationary);
    EXPECT_EQ(out.action, 0);
    EXPECT_EQ(out.control_detail, 0);
}

TEST(BrainReply, ReadWheelsKnownBytes) {
    EXPECT_EQ(encode(makeWheels()), kReadWheelsOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kReadWheelsOkReply, out));
    EXPECT_EQ(out.op, kOpReadWheels);
    EXPECT_EQ(out.result, kResultOk);
    ASSERT_EQ(out.wheel_count, 2);
    EXPECT_EQ(out.wheels[0].port, 0);
    EXPECT_EQ(out.wheels[0].flags, kWheelFresh | kWheelValid);
    EXPECT_EQ(out.wheels[0].discontinuity, 0x1234);
    EXPECT_EQ(out.wheels[0].counts, -123456);
    EXPECT_EQ(out.wheels[0].travel_um, 987654);
    EXPECT_EQ(out.wheels[0].age_ms, 12);
    EXPECT_EQ(out.wheels[1].port, 1);
    EXPECT_EQ(out.wheels[1].flags, kWheelFresh);
    EXPECT_EQ(out.wheels[1].discontinuity, 0xFFFF);
    EXPECT_EQ(out.wheels[1].counts, INT32_MAX);
    EXPECT_EQ(out.wheels[1].travel_um, INT32_MIN);
    EXPECT_EQ(out.wheels[1].age_ms, 65535);
    EXPECT_EQ(out.wheels[2].counts, 0);
}

TEST(BrainReply, ReadWheelsEmptyAndNotReady) {
    BrainReply r  = makeWheels();
    r.wheel_count = 0;
    EXPECT_EQ(encode(r), kReadWheelsEmptyReply);
    BrainReply out{};
    ASSERT_TRUE(decodes(kReadWheelsEmptyReply, out));
    EXPECT_EQ(out.result, kResultOk);
    EXPECT_EQ(out.wheel_count, 0);

    BrainReply nr = makeWheels();
    nr.result     = kResultNotReady;
    EXPECT_EQ(encode(nr), kReadWheelsNotReadyReply);
    ASSERT_TRUE(decodes(kReadWheelsNotReadyReply, out));
    EXPECT_EQ(out.result, kResultNotReady);
    EXPECT_EQ(out.wheel_count, 0);
    EXPECT_EQ(out.wheels[0].counts, 0);
}

TEST(BrainReply, ReadWheelsRecordOffsetsAndReservedBytes) {
    BrainReply r = makeReply(kOpReadWheels, kSession, 1, kResultOk);
    r.wheel_count = kWheelReadingsMax;
    Le p          = replyHead(kOpReadWheels, kSession, 1, kResultOk);
    p.u8(kWheelReadingsMax);
    for (uint8_t i = 0; i < kWheelReadingsMax; ++i) {
        WheelReading& w = r.wheels[i];
        w.port          = static_cast<uint8_t>(0x10 + i);
        w.flags         = static_cast<uint8_t>(0x20 + i);
        w.discontinuity = static_cast<uint16_t>(0x3132 + i);
        w.counts        = 0x41424344 + i;
        w.travel_um     = -0x51525354 - i;
        w.age_ms        = static_cast<uint16_t>(0x6162 + i);
        p.u8(w.port).u8(w.flags).u16(w.discontinuity).i32(w.counts).i32(w.travel_um);
        p.u16(w.age_ms).u16(0);
    }
    const Bytes expected = linkFrame(kFrameBrainReply, p.b);
    ASSERT_EQ(expected.size(), 6u + 13u + 1u + 64u);
    EXPECT_EQ(encode(r), expected);

    BrainReply out{};
    ASSERT_TRUE(decodes(expected, out));
    ASSERT_EQ(out.wheel_count, kWheelReadingsMax);
    for (uint8_t i = 0; i < kWheelReadingsMax; ++i) {
        EXPECT_EQ(out.wheels[i].port, r.wheels[i].port);
        EXPECT_EQ(out.wheels[i].flags, r.wheels[i].flags);
        EXPECT_EQ(out.wheels[i].discontinuity, r.wheels[i].discontinuity);
        EXPECT_EQ(out.wheels[i].counts, r.wheels[i].counts);
        EXPECT_EQ(out.wheels[i].travel_um, r.wheels[i].travel_um);
        EXPECT_EQ(out.wheels[i].age_ms, r.wheels[i].age_ms);
    }

    // Reserved record bytes are written as zero and ignored when read.
    Le q = replyHead(kOpReadWheels, kSession, 1, kResultOk);
    q.u8(1).u8(2).u8(kWheelValid).u16(9).i32(-1).i32(1).u16(3).u16(0xFFFF);
    ASSERT_TRUE(decodes(linkFrame(kFrameBrainReply, q.b), out));
    ASSERT_EQ(out.wheel_count, 1);
    EXPECT_EQ(out.wheels[0].port, 2);
    EXPECT_EQ(out.wheels[0].age_ms, 3);
}

TEST(BrainReply, ReadWheelsLengthMustMatchCount) {
    for (int count = 0; count <= 255; ++count) {
        for (size_t n = kBrainReplyHeaderLen; n <= kBrainPayloadMax; ++n) {
            Bytes p = replyPayload(4, kOpReadWheels, kResultOk, n);
            if (n > kBrainReplyHeaderLen) {
                p[kBrainReplyHeaderLen] = static_cast<uint8_t>(count);
            }
            const bool expected =
                n > kBrainReplyHeaderLen && count <= kWheelReadingsMax &&
                n == kBrainReplyHeaderLen + 1u + kWheelReadingLen * static_cast<size_t>(count);
            BrainReply out{};
            ASSERT_EQ(decodes(linkFrame(kFrameBrainReply, p), out), expected)
                << "count " << count << " len " << n;
        }
    }
}

TEST(BrainReply, EncoderRefusesTooManyWheels) {
    uint8_t    buf[kMaxFrameLen];
    BrainReply r = makeWheels();
    r.wheel_count = kWheelReadingsMax + 1;
    EXPECT_EQ(encodeBrainReply(r, buf, sizeof(buf)), 0);
    r.wheel_count = 255;
    EXPECT_EQ(encodeBrainReply(r, buf, sizeof(buf)), 0);
    r.wheel_count = 2;
    EXPECT_EQ(encodeBrainReply(r, buf, 51), 0);
    EXPECT_EQ(encodeBrainReply(r, buf, 52), 52);
}

TEST(BrainReply, PathReportOkHasNoBody) {
    EXPECT_EQ(encode(makeReply(kOpPathReport, kSession, 10, kResultOk)), kPathOkReply);
    BrainReply out{};
    ASSERT_TRUE(decodes(kPathOkReply, out));
    EXPECT_EQ(out.op, kOpPathReport);
    EXPECT_EQ(out.result, kResultOk);
}

TEST(BrainReply, EveryHeaderOnlyPairEncodesHeaderOnly) {
    for (uint8_t op : kKnownOps) {
        for (uint8_t result : kKnownResults) {
            if (brainReplyMinLen(op, result) != kBrainReplyHeaderLen) {
                continue;
            }
            BrainReply r      = maxReadDoc();
            r.op              = op;
            r.result          = result;
            r.odometry_epoch  = 1;
            r.profile_id      = 1;
            r.action          = 1;
            r.state.x_mm      = 1;
            const Bytes f     = encode(r);
            ASSERT_EQ(f.size(), kLinkEnvelopeLen + kBrainReplyHeaderLen)
                << "op " << int(op) << " result " << int(result);
            BrainReply out{};
            ASSERT_TRUE(decodes(f, out));
            EXPECT_EQ(out.result, result);
        }
    }
}

TEST(BrainReply, EncoderRefusesReadDocDataOutOfRange) {
    uint8_t    buf[kMaxFrameLen];
    BrainReply r = maxReadDoc();
    r.data_len   = 0;
    EXPECT_EQ(encodeBrainReply(r, buf, sizeof(buf)), 0);
    r.data_len = kDocChunkMax + 1;
    EXPECT_EQ(encodeBrainReply(r, buf, sizeof(buf)), 0);
    r.data_len = kDocChunkMax;
    EXPECT_EQ(encodeBrainReply(r, buf, 127), 0);
    EXPECT_EQ(encodeBrainReply(r, buf, 128), 128);
}

// ---------------------------------------------------------------------------
// Brain replies, length ranges, unknown pairs and versions
// ---------------------------------------------------------------------------

TEST(BrainReply, KnownPairsAcceptOnlyTheirLengthRange) {
    for (uint8_t op : kKnownOps) {
        for (uint8_t result : kKnownResults) {
            const uint8_t lo = brainReplyMinLen(op, result);
            const uint8_t hi = brainReplyMaxLen(op, result);
            // READ_WHEELS Ok: the count byte names the whole records present, so
            // only lengths of whole records decode (exhaustive in
            // ReadWheelsLengthMustMatchCount).
            const bool wheels = op == kOpReadWheels && result == kResultOk;
            for (size_t n = kBrainReplyHeaderLen; n <= kBrainPayloadMax; ++n) {
                Bytes p        = replyPayload(4, op, result, n);
                bool  in_range = n >= lo && n <= hi;
                if (wheels && n > kBrainReplyHeaderLen) {
                    const size_t body       = n - kBrainReplyHeaderLen - 1;
                    p[kBrainReplyHeaderLen] = static_cast<uint8_t>(body / kWheelReadingLen);
                    in_range                = in_range && body % kWheelReadingLen == 0;
                }
                BrainReply out{};
                ASSERT_EQ(decodes(linkFrame(kFrameBrainReply, p), out), in_range)
                    << "op " << int(op) << " result " << int(result) << " len " << n;
            }
        }
    }
}

TEST(BrainReply, UnknownOpOrResultDecodesHeaderOnly) {
    BrainReply out{};
    ASSERT_TRUE(decodes(encode(makeReply(0x7F, kSession, 5, kResultUnsupportedOp)), out));
    EXPECT_EQ(out.op, 0x7F);
    EXPECT_EQ(out.result, kResultUnsupportedOp);
    EXPECT_EQ(out.request_id, 5);

    // Any length for an unknown pair, the body ignored.
    const struct {
        uint8_t op, result;
        size_t  len;
    } cases[] = {
        {kOpGetState, 6, 53},       {kOpGetState, 7, 53},  {kOpGetState, 99, 53},
        {kOpReadDoc, 200, 122},     {kOpSetPose, 14, 21},  {3, kResultOk, 15},
        {5, kResultOk, 53},         {0x7F, kResultOk, 13}, {0, kResultOk, 30},
        {12, kResultOk, 78},        {kOpControl, 14, 16},  {kOpReadWheels, 14, 14},
    };
    for (const auto& c : cases) {
        Bytes p = replyPayload(4, c.op, c.result, c.len);
        for (size_t i = kBrainReplyHeaderLen; i < c.len; ++i) {
            p[i] = 0x22;
        }
        ASSERT_TRUE(decodes(linkFrame(kFrameBrainReply, p), out))
            << "op " << int(c.op) << " result " << int(c.result);
        EXPECT_EQ(out.op, c.op);
        EXPECT_EQ(out.result, c.result);
        EXPECT_EQ(out.state.x_mm, 0);
        EXPECT_EQ(out.odometry_epoch, 0u);
        EXPECT_EQ(out.data_len, 0);
        EXPECT_EQ(out.action, 0);
        EXPECT_EQ(out.wheel_count, 0);
    }

    // Encoding an unknown pair writes the header only.
    BrainReply r = makeReply(kOpGetState, kSession, 5, 7);
    r.state      = makeState();
    EXPECT_EQ(encode(r).size(), kLinkEnvelopeLen + kBrainReplyHeaderLen);
}

TEST(BrainReply, OtherVersionDecodesHeaderOnly) {
    // A v3 Pi's GET_STATE Ok with its landmark block.
    Bytes v3 = replyPayload(3, kOpGetState, kResultOk, 53);
    for (size_t i = kBrainReplyHeaderLen; i < v3.size(); ++i) {
        v3[i] = 0x33;
    }
    BrainReply out{};
    ASSERT_TRUE(decodes(linkFrame(kFrameBrainReply, v3), out));
    EXPECT_EQ(out.version, 3);
    EXPECT_EQ(out.result, kResultOk);
    EXPECT_EQ(out.state.profile_id, 0u);
    EXPECT_EQ(out.state.map_id, 0u);

    // A later Pi answering UnsupportedVersion with its own version and body.
    Bytes p = replyPayload(9, kOpGetState, kResultUnsupportedVersion, 40);
    p[2]    = 0x44;
    p[9]    = 0x0D;
    p[20]   = 0x77;
    ASSERT_TRUE(decodes(linkFrame(kFrameBrainReply, p), out));
    EXPECT_EQ(out.version, 9);
    EXPECT_EQ(out.op, kOpGetState);
    EXPECT_EQ(out.session, 0x44u);
    EXPECT_EQ(out.request_id, 9);
    EXPECT_EQ(out.result, kResultUnsupportedVersion);
    EXPECT_EQ(out.pi_instance, 0x0Du);
    EXPECT_EQ(out.state.robot_flags, 0);

    // A v3 HELLO reply: header only, so the nonce is not read.
    Bytes hello = replyPayload(3, kOpHello, kResultUnsupportedVersion, 17);
    hello[13]   = 0x78;
    ASSERT_TRUE(decodes(linkFrame(kFrameBrainReply, hello), out));
    EXPECT_EQ(out.version, 3);
    EXPECT_EQ(out.nonce, 0u);

    BrainReply r     = makeReply(kOpSetPose, kSession, 2, kResultOk);
    r.version        = 2;
    r.odometry_epoch = 5;
    ASSERT_TRUE(decodes(encode(r), out));
    EXPECT_EQ(out.version, 2);
    EXPECT_EQ(out.odometry_epoch, 0u);
}

TEST(BrainReply, EnvelopeLenBounds) {
    BrainReply out{};
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainReply, Bytes(12, 0x04)), out));
    EXPECT_TRUE(decodes(linkFrame(kFrameBrainReply, replyPayload(4, 0x7F, 0, 13)), out));
    EXPECT_TRUE(decodes(linkFrame(kFrameBrainReply, replyPayload(4, 0x7F, 0, 122)), out));
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainReply, replyPayload(4, 0x7F, 0, 123)), out));
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainReply, replyPayload(4, 0x7F, 0, 255)), out));
}

TEST(BrainReply, CorruptionRejected) {
    for (const Bytes& frame : {kGetStateOkReply, kReadDocOkReply, maxReadDocBytes(),
                               kReadWheelsOkReply, kControlFailedReply}) {
        for (size_t i = 0; i < frame.size(); ++i) {
            Bytes bad = frame;
            bad[i] ^= 0x80;
            BrainReply out{};
            EXPECT_FALSE(decodes(bad, out)) << "size " << frame.size() << " byte " << i;
        }
    }
}

TEST(BrainReply, TruncatedRejected) {
    for (const Bytes& frame : {kGetStateOkReply, maxReadDocBytes(), kReadWheelsOkReply,
                               kControlPendingReply}) {
        BrainReply out{};
        for (uint16_t n = 0; n < frame.size(); ++n) {
            EXPECT_FALSE(decodeBrainReply(frame.data(), n, out));
        }
        Bytes padded = frame;
        padded.push_back(0x00);
        EXPECT_FALSE(decodes(padded, out));
    }
}

// ---------------------------------------------------------------------------
// Pico link
// ---------------------------------------------------------------------------

TEST(PicoCommand, KnownBytes) {
    PicoCommand configure = makePicoCommand(kPicoOpConfigure, 0x0102, 0xBEEF);
    configure.imu_enabled = 1;
    configure.imu_port    = 9; // not CONFIGURE's field
    EXPECT_EQ(encode(configure), kPicoConfigureCommand);

    PicoCommand reinit = makePicoCommand(kPicoOpReinitImu, 0xFFFF, 0x0001);
    reinit.imu_enabled = 1; // not REINIT_IMU's field
    EXPECT_EQ(encode(reinit), kPicoReinitImuCommand);

    PicoCommand restart = makePicoCommand(kPicoOpRestartAcquisition, 7, 0xBEEF);
    restart.imu_enabled = 1;
    restart.imu_port    = 1;
    EXPECT_EQ(encode(restart), kPicoRestartCommand);
}

TEST(PicoCommand, DecodeFromKnownBytes) {
    PicoCommand out{};
    ASSERT_TRUE(decodes(kPicoConfigureCommand, out));
    EXPECT_EQ(out.version, kPicoLinkVersion);
    EXPECT_EQ(out.op, kPicoOpConfigure);
    EXPECT_EQ(out.request_id, 0x0102);
    EXPECT_EQ(out.target_boot_id, 0xBEEF);
    EXPECT_EQ(out.imu_enabled, 1);
    EXPECT_EQ(out.imu_port, 0);

    ASSERT_TRUE(decodes(kPicoReinitImuCommand, out));
    EXPECT_EQ(out.op, kPicoOpReinitImu);
    EXPECT_EQ(out.request_id, 0xFFFF);
    EXPECT_EQ(out.target_boot_id, 0x0001);
    EXPECT_EQ(out.imu_port, 0);
    EXPECT_EQ(out.imu_enabled, 0);

    ASSERT_TRUE(decodes(kPicoRestartCommand, out));
    EXPECT_EQ(out.op, kPicoOpRestartAcquisition);
    EXPECT_EQ(out.request_id, 7);
    EXPECT_EQ(out.target_boot_id, 0xBEEF);
}

TEST(PicoCommand, BodyByteRoundTrips) {
    for (int v = 0; v < 256; ++v) {
        PicoCommand c = makePicoCommand(kPicoOpConfigure, 1, 2);
        c.imu_enabled = static_cast<uint8_t>(v);
        PicoCommand out{};
        ASSERT_TRUE(decodes(encode(c), out));
        EXPECT_EQ(out.imu_enabled, v);

        PicoCommand r = makePicoCommand(kPicoOpReinitImu, 1, 2);
        r.imu_port    = static_cast<uint8_t>(v);
        ASSERT_TRUE(decodes(encode(r), out));
        EXPECT_EQ(out.imu_port, v);
    }
}

TEST(PicoCommand, EveryOpAndLength) {
    // Known ops take exactly their length; unknown ops decode the header so
    // the Pico can report them. The envelope bounds the length to 6..8.
    for (int op = 0; op < 256; ++op) {
        for (size_t n = 0; n <= kBrainPayloadMax; ++n) {
            Bytes p(n, 0);
            if (n >= 1) {
                p[0] = kPicoLinkVersion;
            }
            if (n >= 2) {
                p[1] = static_cast<uint8_t>(op);
            }
            if (n >= 3) {
                p[2] = 0x21;
            }
            const uint8_t want = picoCommandLen(static_cast<uint8_t>(op));
            const bool    envelope = n >= kPicoCommandHeaderLen && n <= kPicoCommandMaxLen;
            const bool    expected = envelope && (want == 0 || n == want);
            PicoCommand   out{};
            ASSERT_EQ(decodes(linkFrame(kFramePicoCommand, p), out), expected)
                << "op " << op << " len " << n;
            if (expected) {
                EXPECT_EQ(out.op, op);
                EXPECT_EQ(out.request_id, 0x21);
            }
        }
    }
}

TEST(PicoCommand, UnknownOpEncodesHeaderOnly) {
    PicoCommand c = makePicoCommand(0x7F, 3, 4);
    c.imu_enabled = 1;
    c.imu_port    = 1;
    const Bytes f = encode(c);
    Le p;
    p.u8(kPicoLinkVersion).u8(0x7F).u16(3).u16(4);
    EXPECT_EQ(f, linkFrame(kFramePicoCommand, p.b));

    PicoCommand out{};
    ASSERT_TRUE(decodes(f, out));
    EXPECT_EQ(out.op, 0x7F);
    EXPECT_EQ(out.imu_enabled, 0);
}

TEST(PicoCommand, OtherVersionRejected) {
    for (int version : {0, 2, 4, 255}) {
        Bytes p = Bytes(kPicoConfigureCommand.begin() + 4, kPicoConfigureCommand.end() - 2);
        p[0]    = static_cast<uint8_t>(version);
        PicoCommand out{};
        EXPECT_FALSE(decodes(linkFrame(kFramePicoCommand, p), out)) << version;
    }
    PicoCommand c = makePicoCommand(kPicoOpRestartAcquisition, 1, 1);
    c.version     = 2;
    PicoCommand out{};
    EXPECT_FALSE(decodes(encode(c), out));
}

TEST(PicoCommand, CorruptTruncatedAndPaddedRejected) {
    for (const Bytes& frame : {kPicoConfigureCommand, kPicoReinitImuCommand, kPicoRestartCommand}) {
        PicoCommand out{};
        for (size_t i = 0; i < frame.size(); ++i) {
            Bytes bad = frame;
            bad[i] ^= 0x01;
            EXPECT_FALSE(decodes(bad, out)) << "byte " << i;
        }
        for (uint16_t n = 0; n < frame.size(); ++n) {
            EXPECT_FALSE(decodePicoCommand(frame.data(), n, out)) << "len " << n;
        }
        Bytes padded = frame;
        padded.push_back(0);
        EXPECT_FALSE(decodes(padded, out));
    }
}

TEST(PicoCommand, EncodeRespectsCapacity) {
    uint8_t buf[kMaxFrameLen];
    EXPECT_EQ(encodePicoCommand(makePicoCommand(kPicoOpConfigure, 1, 1), buf, 12), 0);
    EXPECT_EQ(encodePicoCommand(makePicoCommand(kPicoOpConfigure, 1, 1), buf, 13), 13);
    EXPECT_EQ(encodePicoCommand(makePicoCommand(kPicoOpRestartAcquisition, 1, 1), buf, 11), 0);
    EXPECT_EQ(encodePicoCommand(makePicoCommand(kPicoOpRestartAcquisition, 1, 1), buf, 12), 12);
}

TEST(PicoStatus, KnownBytes) {
    EXPECT_EQ(encode(makePicoStatus()), kPicoStatusVector);

    PicoStatus out{};
    ASSERT_TRUE(decodes(kPicoStatusVector, out));
    EXPECT_EQ(out.version, kPicoLinkVersion);
    EXPECT_EQ(out.boot_id, 0xBEEF);
    EXPECT_EQ(out.acq_epoch, 2);
    EXPECT_EQ(out.imu_epoch, 5);
    EXPECT_EQ(out.uptime_ms, 0x00123456u);
    EXPECT_EQ(out.imu_state, kPicoImuRetrying);
    EXPECT_EQ(out.imu_reason, kPicoImuReasonFeatures);
    EXPECT_EQ(out.imu_attempts, 0x0102);
    EXPECT_EQ(out.flags, kPicoImuEnabled);
    EXPECT_EQ(out.last_request_id, 0x0102);
    EXPECT_EQ(out.last_op, kPicoOpReinitImu);
    EXPECT_EQ(out.last_status, kPicoCommandRunning);
    EXPECT_EQ(out.last_detail, kPicoDetailImuAbsent);
    EXPECT_EQ(out.firmware, kPicoFirmwareBno08x);
}

TEST(PicoStatus, FieldOffsets) {
    PicoStatus s{};
    s.boot_id         = 0x0201;
    s.acq_epoch       = 0x03;
    s.imu_epoch       = 0x04;
    s.uptime_ms       = 0x08070605;
    s.imu_state       = 0x09;
    s.imu_reason      = 0x0A;
    s.imu_attempts    = 0x0C0B;
    s.flags           = 0x0D;
    s.last_request_id = 0x0F0E;
    s.last_op         = 0x10;
    s.last_status     = 0x11;
    s.last_detail     = 0x12;
    s.firmware        = 0x13;
    const Bytes f     = encode(s);
    ASSERT_EQ(f.size(), kLinkEnvelopeLen + kPicoStatusLen);
    Bytes expected = {kPicoLinkVersion};
    for (uint8_t b = 0x01; b <= 0x13; ++b) {
        expected.push_back(b);
    }
    EXPECT_EQ(Bytes(f.begin() + 4, f.end() - 2), expected);

    PicoStatus out{};
    ASSERT_TRUE(decodes(f, out));
    EXPECT_EQ(out.uptime_ms, 0x08070605u);
    EXPECT_EQ(out.firmware, 0x13);
}

TEST(PicoStatus, OnlyTheExactLengthAndVersion) {
    for (size_t n = 0; n <= kBrainPayloadMax; ++n) {
        Bytes p(n, 0);
        if (n >= 1) {
            p[0] = kPicoLinkVersion;
        }
        PicoStatus out{};
        EXPECT_EQ(decodes(linkFrame(kFramePicoStatus, p), out), n == kPicoStatusLen) << n;
    }
    for (int version : {0, 2, 255}) {
        PicoStatus s = makePicoStatus();
        s.version    = static_cast<uint8_t>(version);
        PicoStatus out{};
        EXPECT_FALSE(decodes(encode(s), out)) << version;
    }
}

TEST(PicoStatus, CorruptTruncatedAndPaddedRejected) {
    PicoStatus out{};
    for (size_t i = 0; i < kPicoStatusVector.size(); ++i) {
        Bytes bad = kPicoStatusVector;
        bad[i] ^= 0x40;
        EXPECT_FALSE(decodes(bad, out)) << "byte " << i;
    }
    for (uint16_t n = 0; n < kPicoStatusVector.size(); ++n) {
        EXPECT_FALSE(decodePicoStatus(kPicoStatusVector.data(), n, out)) << "len " << n;
    }
    Bytes padded = kPicoStatusVector;
    padded.push_back(0);
    EXPECT_FALSE(decodes(padded, out));
}

TEST(PicoStatus, EncodeRespectsCapacity) {
    uint8_t buf[kMaxFrameLen];
    EXPECT_EQ(encodePicoStatus(makePicoStatus(), buf, 25), 0);
    EXPECT_EQ(encodePicoStatus(makePicoStatus(), buf, 26), 26);
}

TEST(PicoLink, TypesAreNotInterchangeable) {
    PicoCommand  cmd{};
    PicoStatus   status{};
    BrainRequest req{};
    BrainReply   rep{};
    EXPECT_FALSE(decodes(kPicoStatusVector, cmd));
    EXPECT_FALSE(decodes(kPicoConfigureCommand, status));
    EXPECT_FALSE(decodes(kPicoConfigureCommand, req));
    EXPECT_FALSE(decodes(kPicoStatusVector, rep));
    EXPECT_FALSE(decodes(kControlRequest, cmd));
    EXPECT_FALSE(decodes(kHelloOkReply, status));

    // Same payload, other type byte: the crc covers the type.
    Bytes relabeled = kPicoConfigureCommand;
    relabeled[2]    = kFrameBrainRequest;
    EXPECT_FALSE(decodes(relabeled, req));
}

// ---------------------------------------------------------------------------
// Stream reader
// ---------------------------------------------------------------------------

TEST(Reader, ReadsCleanFrame) {
    FrameReader r;
    EXPECT_EQ(r.frameLen(), 0);
    EXPECT_EQ(r.frameType(), 0);
    bool got = false;
    for (uint8_t b : kSensorVector) {
        got = r.push(b);
    }
    ASSERT_TRUE(got);
    EXPECT_EQ(r.frameLen(), kSensorVector.size());
    EXPECT_EQ(r.frameType(), kFrameSensor);
}

TEST(Reader, IgnoresLeadingGarbage) {
    FrameReader r;
    for (uint8_t b : {0x00, 0x12, 0xFF, 0xAA, 0x99, 0x55}) {
        EXPECT_FALSE(r.push(b));
    }
    bool got = false;
    for (uint8_t b : kSensorVector) {
        got = r.push(b);
    }
    EXPECT_TRUE(got);
}

TEST(Reader, RelocksAfterCorruptFrame) {
    FrameReader r;
    Bytes       bad = kSensorVector;
    bad.back() ^= 0xFF;
    for (uint8_t b : bad) {
        EXPECT_FALSE(r.push(b));
    }
    bool got = false;
    for (uint8_t b : kSensorVector) {
        got = r.push(b);
    }
    EXPECT_TRUE(got);
}

TEST(Reader, HandlesBackToBackFrames) {
    FrameReader r;
    int         frames = 0;
    for (int i = 0; i < 3; ++i) {
        for (uint8_t b : kSensorVector) {
            if (r.push(b)) {
                ++frames;
            }
        }
    }
    EXPECT_EQ(frames, 3);
}

TEST(Reader, HandlesRepeatedSyncBytes) {
    FrameReader r;
    for (int i = 0; i < 5; ++i) {
        EXPECT_FALSE(r.push(kSync0));
    }
    bool got = false;
    for (size_t i = 1; i < kSensorVector.size(); ++i) {
        got = r.push(kSensorVector[i]);
    }
    EXPECT_TRUE(got);
}

TEST(Reader, RejectsUnknownFrameType) {
    FrameReader r;
    Bytes       bad = kSensorVector;
    bad[2]          = 0x7F;
    for (uint8_t b : bad) {
        EXPECT_FALSE(r.push(b));
    }
}

TEST(Reader, RetiredTypesRejected) {
    FrameReader r;
    for (uint8_t type : {0x02, 0x03}) {
        Bytes bad = kSensorVector;
        bad[2]    = type;
        EXPECT_TRUE(readAll(r, bad).empty());
    }
}

TEST(Reader, LinkFrameCompletesOnLastByteOnly) {
    FrameReader r;
    for (size_t i = 0; i + 1 < kGetStateOkReply.size(); ++i) {
        EXPECT_FALSE(r.push(kGetStateOkReply[i]));
    }
    ASSERT_TRUE(r.push(kGetStateOkReply.back()));
    EXPECT_EQ(r.frameType(), kFrameBrainReply);
    EXPECT_EQ(r.frameLen(), kGetStateOkReply.size());

    BrainReply out{};
    ASSERT_TRUE(decodeBrainReply(r.frame(), r.frameLen(), out));
    EXPECT_EQ(out.state.x_mm, 1500);
    EXPECT_FALSE(r.next());
}

TEST(Reader, MaxLengthFramesCompleteOnLastByteOnly) {
    for (const Bytes& frame : {maxProfileWriteBytes(), maxReadDocBytes()}) {
        ASSERT_EQ(frame.size(), kMaxFrameLen);
        FrameReader r;
        for (size_t i = 0; i + 1 < frame.size(); ++i) {
            ASSERT_FALSE(r.push(frame[i])) << "byte " << i;
        }
        ASSERT_TRUE(r.push(frame.back()));
        EXPECT_EQ(frameOf(r), frame);
        EXPECT_FALSE(r.next());
    }
}

TEST(Reader, BackToBackMaxLengthFrames) {
    const Bytes stream = concat({maxProfileWriteBytes(), maxReadDocBytes(), kSensorVector,
                                 maxProfileWriteBytes(), kGetStateRequest, maxReadDocBytes()});
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, stream);
    ASSERT_EQ(frames.size(), 6u);
    EXPECT_EQ(frames[0], maxProfileWriteBytes());
    EXPECT_EQ(frames[1], maxReadDocBytes());
    EXPECT_EQ(frames[2], kSensorVector);
    EXPECT_EQ(frames[3], maxProfileWriteBytes());
    EXPECT_EQ(frames[4], kGetStateRequest);
    EXPECT_EQ(frames[5], maxReadDocBytes());
}

TEST(Reader, MaxLengthFrameAfterAFullBufferOfGarbage) {
    const Bytes stream = concat({Bytes(3 * kMaxFrameLen, 0x13), maxReadDocBytes()});
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, stream);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], maxReadDocBytes());
}

TEST(Reader, FramesInsideAValidChunkAreNotSplitOut) {
    // Document bytes that happen to hold a whole link frame stay chunk data.
    BrainReply r = maxReadDoc();
    for (size_t i = 0; i < kControlRequest.size(); ++i) {
        r.data[10 + i] = kControlRequest[i];
    }
    const Bytes outer = encode(r);
    ASSERT_EQ(outer.size(), kMaxFrameLen);
    FrameReader reader;
    const std::vector<Bytes> frames = readAll(reader, outer);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], outer);
}

TEST(Reader, CorruptMaxLengthFrameThenRelock) {
    Bytes bad = maxProfileWriteBytes();
    bad[60] ^= 0x04;
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, concat({bad, maxReadDocBytes()}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], maxReadDocBytes());
}

TEST(Reader, InterleavedSensorAndLinkFrames) {
    const Bytes stream = concat({kSensorVector, kHelloRequest, {0x00, 0xAA, 0x13},
                                 kGetStateOkReply, kSensorVector, kSetPoseRequest,
                                 kControlOkReply, {0x55}, kGetStateImuRequest});
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, stream);
    ASSERT_EQ(frames.size(), 7u);
    EXPECT_EQ(frames[0], kSensorVector);
    EXPECT_EQ(frames[1], kHelloRequest);
    EXPECT_EQ(frames[2], kGetStateOkReply);
    EXPECT_EQ(frames[3], kSensorVector);
    EXPECT_EQ(frames[4], kSetPoseRequest);
    EXPECT_EQ(frames[5], kControlOkReply);
    EXPECT_EQ(frames[6], kGetStateImuRequest);
}

TEST(Reader, EveryFrameTypeInOneStream) {
    const Bytes stream =
        concat({kSensorV2Vector, kPicoStatusVector, {0x13, 0xAA}, kSensorVector,
                kPicoConfigureCommand, kReadWheelsOkReply, kSensorV2EmptyVector,
                kPicoRestartCommand, kControlFailedReply, {0xAA, 0x55}, kPicoReinitImuCommand,
                kReadWheelsRequest});
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, stream);
    ASSERT_EQ(frames.size(), 10u);
    EXPECT_EQ(frames[0], kSensorV2Vector);
    EXPECT_EQ(frames[1], kPicoStatusVector);
    EXPECT_EQ(frames[2], kSensorVector);
    EXPECT_EQ(frames[3], kPicoConfigureCommand);
    EXPECT_EQ(frames[4], kReadWheelsOkReply);
    EXPECT_EQ(frames[5], kSensorV2EmptyVector);
    EXPECT_EQ(frames[6], kPicoRestartCommand);
    EXPECT_EQ(frames[7], kControlFailedReply);
    EXPECT_EQ(frames[8], kPicoReinitImuCommand);
    EXPECT_EQ(frames[9], kReadWheelsRequest);
}

TEST(Reader, NewTypesCompleteOnLastByteOnly) {
    for (const Bytes& frame : {kSensorV2Vector, kSensorV2EmptyVector, kPicoStatusVector,
                               kPicoConfigureCommand, kPicoRestartCommand}) {
        FrameReader r;
        for (size_t i = 0; i + 1 < frame.size(); ++i) {
            ASSERT_FALSE(r.push(frame[i])) << "byte " << i;
        }
        ASSERT_TRUE(r.push(frame.back()));
        EXPECT_EQ(frameOf(r), frame);
        EXPECT_EQ(r.frameType(), frame[2]);
        EXPECT_FALSE(r.next());
    }
}

TEST(Reader, LargestSensorV2Frame) {
    SensorSample s = makeSensorV2();
    s.mask         = static_cast<uint16_t>((1u << kSensorBitCount) - 1u);
    const Bytes f  = encode(s);
    ASSERT_EQ(f.size(), 39u);
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, concat({f, f}));
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], f);
    EXPECT_EQ(frames[1], f);
}

TEST(Reader, SensorV2InvalidMaskRejectedAtOnce) {
    // The mask is known after 14 bytes; the frame after it is not lost.
    const Bytes head = {0xAA, 0x55, 0x04, 0x01, 0, 0, 0, 0, 0x01, 0x00, 0, 0, 0x00, 0x80};
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, concat({head, kPicoStatusVector}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kPicoStatusVector);
}

TEST(Reader, SensorV2BadXorThenRelock) {
    Bytes bad = kSensorV2Vector;
    bad[16] ^= 0x02;
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, concat({bad, kSensorV2Vector}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kSensorV2Vector);
}

TEST(Reader, CutPicoFramesDoNotLoseTheNext) {
    for (const Bytes& cut : {Bytes(kPicoStatusVector.begin(), kPicoStatusVector.begin() + 10),
                             Bytes(kPicoConfigureCommand.begin(), kPicoConfigureCommand.end() - 1),
                             Bytes(kSensorV2Vector.begin(), kSensorV2Vector.begin() + 13)}) {
        FrameReader r;
        const std::vector<Bytes> frames = readAll(r, concat({cut, kPicoStatusVector}));
        ASSERT_EQ(frames.size(), 1u) << "cut " << cut.size();
        EXPECT_EQ(frames[0], kPicoStatusVector);
    }
}

TEST(Reader, CrcErrorThenRelock) {
    Bytes bad = kSetPoseRequest;
    bad[14] ^= 0x10;
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, concat({bad, kGetStateRequest}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kGetStateRequest);
}

TEST(Reader, OutOfRangeLenRejectedAtOnce) {
    const Bytes heads[] = {
        {0xAA, 0x55, 0x10, 7},  {0xAA, 0x55, 0x10, 123}, {0xAA, 0x55, 0x10, 255},
        {0xAA, 0x55, 0x11, 12}, {0xAA, 0x55, 0x11, 123}, {0xAA, 0x55, 0x11, 255},
        {0xAA, 0x55, 0x12, 5},  {0xAA, 0x55, 0x12, 9},   {0xAA, 0x55, 0x12, 0},
        {0xAA, 0x55, 0x13, 19}, {0xAA, 0x55, 0x13, 21},  {0xAA, 0x55, 0x13, 255},
    };
    for (const Bytes& head : heads) {
        FrameReader r;
        const std::vector<Bytes> frames = readAll(r, concat({head, kControlRequest}));
        ASSERT_EQ(frames.size(), 1u);
        EXPECT_EQ(frames[0], kControlRequest);
    }
}

TEST(Reader, RescansAfterCorruptedLen) {
    // CONTROL with len 10 corrupted to 32 claims 38 bytes and swallows most
    // of the next GET_STATE request. Both following frames must still be read.
    Bytes bad = kControlRequest;
    bad[3]    = 32;
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, concat({bad, kGetStateRequest, kHelloRequest}));
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], kGetStateRequest);
    EXPECT_EQ(frames[1], kHelloRequest);
}

TEST(Reader, RescanFindsFrameInsideRejectedCandidate) {
    // Cut-off sensor header: the reply after it is read as an invalid mask.
    FrameReader              r;
    const std::vector<Bytes> frames = readAll(r, concat({{0xAA, 0x55, 0x01}, kHelloOkReply}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kHelloOkReply);
}

TEST(Reader, ShortenedLenDoesNotLoseNextFrame) {
    // HELLO with len 12 corrupted to 8: the crc check lands inside the body.
    Bytes bad = kHelloRequest;
    bad[3]    = 8;
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, concat({bad, kSetPoseOkReply}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kSetPoseOkReply);
}

TEST(Reader, OneRescanCanBufferSeveralFrames) {
    // A reply len of 122 claims a whole 128 byte buffer and swallows two
    // complete requests.
    const Bytes stream = concat({{0xAA, 0x55, 0x11, 122}, kGetStateRequest, kControlRequest,
                                 Bytes(128 - 4 - 23 - 16, 0x00)});
    ASSERT_EQ(stream.size(), kMaxFrameLen);

    FrameReader r;
    for (size_t i = 0; i + 1 < stream.size(); ++i) {
        EXPECT_FALSE(r.push(stream[i]));
    }
    ASSERT_TRUE(r.push(stream.back()));
    EXPECT_EQ(frameOf(r), kGetStateRequest);
    ASSERT_TRUE(r.next());
    EXPECT_EQ(frameOf(r), kControlRequest);
    EXPECT_FALSE(r.next());
    EXPECT_FALSE(r.next());

    const std::vector<Bytes> frames = readAll(r, kHelloRequest);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kHelloRequest);
}

TEST(Reader, NextKeepsPartialFrame) {
    FrameReader r;
    EXPECT_FALSE(r.next());
    for (size_t i = 0; i + 1 < kHelloRequest.size(); ++i) {
        EXPECT_FALSE(r.push(kHelloRequest[i]));
        EXPECT_FALSE(r.next());
    }
    ASSERT_TRUE(r.push(kHelloRequest.back()));
    EXPECT_EQ(frameOf(r), kHelloRequest);
}

TEST(Reader, ResetDropsPartialFrame) {
    FrameReader r;
    for (size_t i = 0; i < 100; ++i) {
        r.push(maxReadDocBytes()[i]);
    }
    r.reset();
    EXPECT_EQ(r.frameLen(), 0);
    const std::vector<Bytes> frames = readAll(r, kControlRequest);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kControlRequest);
}
