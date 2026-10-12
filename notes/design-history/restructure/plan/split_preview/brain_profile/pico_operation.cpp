// pico_operation.cpp

#include "brain_profile/pico_operation.h"

#include "runtime/system.h"

namespace navigatr
{

uint32_t System::submitPico(uint8_t op, uint8_t arg, MonotonicTime now, uint8_t& detail) {
    detail = translagatr::kControlDetailPicoLink;
    if (pico_ == nullptr) {
        noteEvent(now, "Pico command refused: no Pico link configured");
        return 0;
    }
    const PicoLinkState link = pico_->link();
    if (!link.identity || !link.frames_fresh) {
        noteEvent(now, link.frames_fresh ? "Pico command refused: the Pico firmware takes no "
                                           "commands (no v2 identity)"
                                         : "Pico command refused: no Pico frames");
        return 0;
    }
    const uint32_t handle = pico_->submit(op, arg, now, kPicoOperationMs / 1000.0);
    if (handle == 0) {
        noteEvent(now, "Pico command refused by the Pico link");
        return 0;
    }
    detail = translagatr::kControlDetailNone;
    return handle;
}

uint8_t System::controlProgress(uint8_t action, MonotonicTime now, uint8_t& detail) {
    updatePicoOperation(now);
    if (!pico_op_.active || pico_op_.action != action) {
        detail = translagatr::kControlDetailPicoLink;   // nothing is running under this request
        return translagatr::kResultFailed;
    }
    detail = pico_op_.detail;
    return pico_op_.result;
}

void System::updatePicoOperation(MonotonicTime now) {
    PicoOperation& op = pico_op_;
    if (!op.active || op.result != translagatr::kResultPending || pico_ == nullptr) {
        return;
    }
    const bool reinit = op.action == translagatr::kControlReinitImu;
    const auto settle = [&](uint8_t result, uint8_t detail, const std::string& text) {
        op.result = result;
        op.detail = detail;
        noteEvent(now, text);
    };
    const PicoRequestStatus status = pico_->request(op.handle);
    const PicoLinkState     link   = pico_->link();
    switch (status.state) {
    case PicoRequestState::kFailed:
        settle(translagatr::kResultFailed,
               status.detail != translagatr::kControlDetailNone
                   ? status.detail
                   : static_cast<uint8_t>(translagatr::kControlDetailPicoRefused),
               reinit ? "Pico IMU reinitialization failed" : "Pico acquisition restart failed");
        return;
    case PicoRequestState::kUnknown:
        settle(translagatr::kResultFailed, translagatr::kControlDetailPicoLink,
               reinit ? "Pico IMU reinitialization lost with the Pico link"
                      : "Pico acquisition restart lost with the Pico link");
        return;
    case PicoRequestState::kCompleted:
        // a used source restarted: the pose needs a placement again (8.10)
        if (reinit) {
            execute_localization_.recalibrate();
            settle(translagatr::kResultOk, translagatr::kControlDetailNone,
                   "Pico IMU reinitialized; bias recalibration started");
            losePlacement(now, "sensor lost: pico_imu reinitialized", true);
            return;
        }
        if (link.acq_epoch != op.acq_epoch || link.restarts != op.restarts) {
            settle(translagatr::kResultOk, translagatr::kControlDetailNone,
                   "Pico acquisition restarted (acquisition epoch " +
                       std::to_string(link.acq_epoch) + "); encoders rebased");
            losePlacement(now, "sensor lost: Pico acquisition restarted", true);
            return;
        }
        break;   // completed on the Pico; waiting for frames of the new epoch
    case PicoRequestState::kSending:
    case PicoRequestState::kRunning: break;
    }
    if (sameDomain(now, op.started) && now - op.started > kPicoOperationMs) {
        settle(translagatr::kResultFailed, translagatr::kControlDetailTimedOut,
               reinit ? "Pico IMU reinitialization timed out"
                      : "Pico acquisition restart timed out");
    }
}

} // namespace navigatr
