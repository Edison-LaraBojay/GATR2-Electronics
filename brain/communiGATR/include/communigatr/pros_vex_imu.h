// pros_vex_imu.h
// VEX V5 IMU on a Brain smart port as the bench IMU source of GET_STATE
// (profile IMU source kBrainVex). The sample is the continuous rotation in
// CCW millidegrees, stamped with the Brain clock at the read (an arrival
// time approximation; it does not synchronize the Brain and the Pico). It is
// invalid while the IMU calibrates, is missing or reports an error. VEX
// firmware owns the calibration; the Pi applies no bias to this source.
// Source in pros/, PROS builds only.

#pragma once
#include <cstdint>

#include "communigatr/client.h"
#include "pros/imu.hpp"

namespace communigatr
{

class ProsVexImu {
public:
    explicit ProsVexImu(uint8_t smart_port);

    // Safe from any task; the link calls it in its poll task.
    BenchImuSample sample() const;

    // Starts a VEX firmware calibration (pros::Imu::reset without waiting for
    // it to finish). Blocks until the IMU shows it calibrating, about 1 s at
    // most. The robot must be still. Samples are invalid until it is done.
    // False when PROS refuses (errno set), for example while calibrating.
    bool recalibrate();

    bool    calibrating() const;
    uint8_t port() const { return port_; }

private:
    uint8_t   port_;
    pros::Imu imu_;
};

} // namespace communigatr
