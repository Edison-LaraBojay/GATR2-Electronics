// link_rig.h
// Host-only test rig: a FakePi behind a FakeBus (RS-485) or a FakeUsb, a
// Client and LinkDriver on the chosen Brain port, and a clock stepped by
// hand. rebootBrain() replaces the client and driver as a Brain power cycle
// does, with the Pi and transport untouched.

#pragma once
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>

#include "communigatr/client.h"
#include "communigatr/link_driver.h"
#include "sim/fake_bus.h"
#include "sim/fake_pi.h"
#include "sim/fake_usb.h"

namespace communigatr
{

enum class RigTransport : uint8_t { kRs485, kUsb };

class LinkRig {
public:
    explicit LinkRig(const ClientConfig& client_config = {}, const FakeBusConfig& bus_config = {},
                     RigTransport transport = RigTransport::kRs485,
                     const FakeUsbConfig& usb_config = {});

    // Advances the transport to now + dt, then polls the client.
    void step(Seconds dt = 0.001);

    // Steps until done() or limit seconds pass. Returns done().
    bool runUntil(const std::function<bool()>& done, Seconds limit, Seconds dt = 0.001);
    void run(Seconds duration, Seconds dt = 0.001);

    // New client and driver on the same port: new boot, request ids from 1.
    // The config applies to this and later boots.
    void rebootBrain();
    void rebootBrain(const ClientConfig& client_config);

    Client&      client() { return *client_; }
    LinkDriver&  driver() { return *driver_; }
    Seconds      now() const { return now_; }
    RigTransport transport() const { return transport_; }

    // Nonces handed out before the counter takes over.
    std::deque<uint32_t> nonces;

    FakePi  pi;
    FakeBus bus;
    FakeUsb usb;

private:
    uint32_t nextNonce();

    RigTransport                transport_;
    ClientConfig                client_config_;
    std::unique_ptr<Client>     client_;
    std::unique_ptr<LinkDriver> driver_;
    Seconds                     now_           = 0;
    uint32_t                    nonce_counter_ = 0x1000;
};

} // namespace communigatr
