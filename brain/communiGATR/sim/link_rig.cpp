// link_rig.cpp

#include "sim/link_rig.h"

namespace communigatr
{

LinkRig::LinkRig(const ClientConfig& client_config, const FakeBusConfig& bus_config)
    : bus(pi, bus_config), client_config_(client_config) {
    rebootBrain();
}

void LinkRig::step(Seconds dt) {
    now_ += dt;
    bus.advanceTo(now_);
    client_->poll(now_);
}

bool LinkRig::runUntil(const std::function<bool()>& done, Seconds limit, Seconds dt) {
    const Seconds end = now_ + limit;
    while (!done() && now_ < end) {
        step(dt);
    }
    return done();
}

void LinkRig::run(Seconds duration, Seconds dt) {
    const Seconds end = now_ + duration;
    while (now_ < end) {
        step(dt);
    }
}

void LinkRig::rebootBrain() {
    client_.reset(new Client(bus.brainPort(), [this] { return nextNonce(); }, client_config_));
}

uint32_t LinkRig::nextNonce() {
    if (!nonces.empty()) {
        const uint32_t nonce = nonces.front();
        nonces.pop_front();
        return nonce;
    }
    return nonce_counter_++;
}

} // namespace communigatr
