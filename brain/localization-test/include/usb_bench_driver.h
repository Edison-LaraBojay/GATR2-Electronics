#pragma once

#include <memory>

#include "communigatr/pros_driver.h"

namespace communigatr {

// Temporary localization-test transport over the Brain's USB user console.
// Keeps the ordinary Client/Driver and bench IMU protocol. Each binary frame
// travels as NG1:<uppercase hex>\n so PROS cannot consume binary pR commands.
// This app owns USB input and disables PROS output COBS while it runs.
class UsbBenchDriver final : public investigatr::InputSource {
public:
    explicit UsbBenchDriver(const ProsDriverConfig& config);
    ~UsbBenchDriver() override;
    UsbBenchDriver(const UsbBenchDriver&) = delete;
    UsbBenchDriver& operator=(const UsbBenchDriver&) = delete;

    bool start();
    static Seconds now();

    void request(const investigatr::InputRequest& request) override;
    investigatr::InputSnapshot latest(Seconds now) override;
    PlacementTicket submitPlacement(const investigatr::Pose& pose);
    PlacementResult placementResult(PlacementTicket ticket) const;
    PlacementStatus placementStatus(PlacementTicket ticket) const;
    ProsDriverStatus status() const;

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

} // namespace communigatr
