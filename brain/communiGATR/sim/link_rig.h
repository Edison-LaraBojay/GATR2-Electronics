// link_rig.h
// Host-only test rig: a FakePi on a FakeBus, a Client on the bus's
// Brain port, and a clock stepped by hand. rebootBrain() replaces the client
// as a Brain power cycle does, with the Pi and bus untouched.

#pragma once
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>

#include "communigatr/client.h"
#include "sim/fake_bus.h"
#include "sim/fake_pi.h"

namespace communigatr
{

class LinkRig {
public:
    explicit LinkRig(const ClientConfig& client_config = {}, const FakeBusConfig& bus_config = {});

    // Advances the bus to now + dt, then polls the client.
    void step(Seconds dt = 0.001);

    // Steps until done() or limit seconds pass. Returns done().
    bool runUntil(const std::function<bool()>& done, Seconds limit, Seconds dt = 0.001);
    void run(Seconds duration, Seconds dt = 0.001);

    // New client on the same port: new boot, request ids from 1.
    void rebootBrain();

    Client& client() { return *client_; }
    Seconds now() const { return now_; }

    // Nonces handed out before the counter takes over.
    std::deque<uint32_t> nonces;

    FakePi  pi;
    FakeBus bus;

private:
    uint32_t nextNonce();

    ClientConfig            client_config_;
    std::unique_ptr<Client> client_;
    Seconds                 now_           = 0;
    uint32_t                nonce_counter_ = 0x1000;
};

} // namespace communigatr
