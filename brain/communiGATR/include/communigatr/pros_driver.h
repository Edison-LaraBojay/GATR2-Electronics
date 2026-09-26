// pros_driver.h
// Navigatr link for a PROS program. Owns the serial port, Client and
// Driver, polls the client from its own task, and takes one mutex
// around every call. A Navigator uses it directly as its InputSource.
// Source in pros/, PROS builds only.

#pragma once
#include <cstdint>
#include <memory>

#include "communigatr/client.h"
#include "communigatr/driver.h"
#include "communigatr/pros_serial_port.h"
#include "investigatr/input.h"
#include "pros/rtos.hpp"

namespace communigatr
{

struct ProsDriverConfig {
    uint8_t      port           = 0;      // V5 smart port 1..21 wired to the RS-485 link
    int32_t      baud           = 115200; // must match the Pi serial resource
    uint32_t     poll_period_ms = 2;      // at least 1
    uint32_t     task_priority  = TASK_PRIORITY_DEFAULT + 1;
    ClientConfig client;
    DriverConfig driver;
};

// Link state for display and bring-up.
struct ProsDriverStatus {
    bool           started      = false;
    bool           ready        = false;
    bool           connected    = false;
    Seconds        link_age     = 0; // infinity without a session
    uint32_t       session      = 0;
    uint32_t       pi_instance  = 0;
    LinkError      error        = LinkError::kNone;
    uint8_t        peer_version = 0;
    SelectionState selection    = SelectionState::kNotRequested;
    ClientStats    stats;
};

class ProsDriver : public investigatr::InputSource {
public:
    explicit ProsDriver(const ProsDriverConfig& config);
    ~ProsDriver() override;
    ProsDriver(const ProsDriver&)            = delete;
    ProsDriver& operator=(const ProsDriver&) = delete;

    // Opens the port and starts the poll task. False when the port could not
    // be opened; call again to retry. True once started.
    bool start();

    // Clock of the poll task: seconds since PROS started. Pass it to
    // Navigator::update so link and navigation times share one origin.
    static Seconds now();

    void                       request(const investigatr::InputRequest& request) override;
    investigatr::InputSnapshot latest(Seconds now) override;

    PlacementTicket submitPlacement(const investigatr::Pose& pose);
    PlacementResult placementResult(PlacementTicket ticket) const;
    PlacementStatus placementStatus(PlacementTicket ticket) const;

    ProsDriverStatus status() const;

private:
    void     run();
    uint32_t nonce();

    ProsDriverConfig            config_;
    ProsSerialPort              port_;
    Client                      client_;
    Driver                      driver_;
    mutable pros::Mutex         mutex_;
    std::unique_ptr<pros::Task> task_;
    uint32_t                    entropy_ = 0;
    uint32_t                    nonces_  = 0;
};

} // namespace communigatr
