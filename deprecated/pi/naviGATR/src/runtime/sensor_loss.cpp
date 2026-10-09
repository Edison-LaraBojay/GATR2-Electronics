// sensor_loss.cpp

#include "runtime/sensor_loss.h"

#include "payloads/sensor_samples.h"

namespace navigatr
{

namespace
{

const char* imuStateName(uint8_t state) {
    switch (state) {
    case translagatr::kPicoImuDisabled: return "disabled";
    case translagatr::kPicoImuInitializing: return "initializing";
    case translagatr::kPicoImuAligning: return "aligning";
    case translagatr::kPicoImuReady: return "ready";
    case translagatr::kPicoImuRetrying: return "retrying";
    case translagatr::kPicoImuFailed: return "failed";
    default: return "in an unknown state";
    }
}

} // namespace

void SensorLossMonitor::configure(const std::vector<Wheel>& wheels, const SensorId& pico_imu,
                                  uint8_t imu_port, std::shared_ptr<const BrainImuBench> vex,
                                  int64_t loss_ms, MonotonicTime now) {
    clear();
    loss_ms_       = loss_ms;
    uses_pico_imu_ = !pico_imu.empty();
    vex_           = std::move(vex);
    for (const Wheel& w : wheels) {
        Source s;
        s.name   = "encoder port " + std::to_string(w.port);
        s.sensor = w.sensor;
        s.last   = now;
        sources_.push_back(s);
    }
    if (uses_pico_imu_) {
        Source s;
        s.kind   = Kind::kPicoImu;
        s.name   = "pico_imu port " + std::to_string(imu_port);
        s.sensor = pico_imu;
        s.last   = now;
        sources_.push_back(s);
    }
    if (vex_ != nullptr) {
        Source s;
        s.kind = Kind::kBrainImu;
        s.name = "brain_vex_imu";
        s.last = now;
        sources_.push_back(s);
    }
}

void SensorLossMonitor::clear() {
    sources_.clear();
    vex_.reset();
    uses_pico_imu_ = false;
    pico_seen_     = false;
}

SensorLossMonitor::Result SensorLossMonitor::update(const SensorMap& sensors,
                                                    const PicoLinkState* pico, MonotonicTime now) {
    Result     r;
    const auto edge = [&](const std::string& why) {
        if (r.edge.empty()) {
            r.edge = why;
        }
    };
    for (Source& s : sources_) {
        if (s.kind == Kind::kBrainImu) {
            // an invalid sample, or a new Brain session, resets the mailbox:
            // a missing sample and a new epoch, a restart at that moment
            const BrainImuBench& vex = *vex_;
            if (s.seen && vex.epoch != s.identity) {
                edge(s.name + (vex.valid ? " restarted" : " restarted (sample invalid or new "
                                                          "session)"));
            }
            if (s.seen || vex.valid) {
                s.identity = vex.epoch;
            }
            if (vex.valid) {
                s.seen = true;
                if (vex.received.domain == ClockDomain::kHost && vex.received.ms > s.last.ms) {
                    s.last = vex.received;
                }
            }
        } else {
            const auto it = sensors.find(s.sensor);
            if (it != sensors.end() && it->second.latest.has_value()) {
                const StoredSample& stored   = *it->second.latest;
                uint64_t            identity = stored.upstream.epoch;
                if (s.kind == Kind::kEncoder) {
                    const EncoderSample* e = stored.payload.get<EncoderSample>();
                    identity               = e != nullptr ? e->discontinuity_epoch : 0;
                }
                if (s.seen && (stored.epoch != s.epoch || identity != s.identity)) {
                    edge(s.name + " restarted");
                }
                s.seen     = true;
                s.epoch    = stored.epoch;
                s.identity = identity;
                if (stored.receivedAt.domain == ClockDomain::kHost &&
                    stored.receivedAt.ms > s.last.ms) {
                    s.last = stored.receivedAt;
                }
            }
        }
        if (r.level.empty() && now.domain == ClockDomain::kHost && now.ms - s.last.ms > loss_ms_) {
            r.level = s.name + " stale " + std::to_string(now.ms - s.last.ms) + " ms";
        }
    }
    if (pico != nullptr && !sources_.empty()) {
        if (pico_seen_) {
            if (pico->reboots != reboots_) {
                edge("Pico rebooted");
            } else if (pico->restarts != restarts_) {
                edge("Pico acquisition restarted");
            } else if (uses_pico_imu_ && pico->imu_restarts != imu_restarts_) {
                edge("Pico IMU restarted");
            }
        }
        pico_seen_    = true;
        reboots_      = pico->reboots;
        restarts_     = pico->restarts;
        imu_restarts_ = pico->imu_restarts;
        if (uses_pico_imu_ && pico->status_known && pico->status.imu_state != translagatr::kPicoImuReady &&
            r.level.empty()) {
            r.level = std::string("pico_imu ") + imuStateName(pico->status.imu_state);
        }
    }
    return r;
}

} // namespace navigatr
