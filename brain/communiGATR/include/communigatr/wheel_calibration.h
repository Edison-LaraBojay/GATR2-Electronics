// wheel_calibration.h
// Measured-travel calibration of one tracking wheel assembly. A trial pushes
// the robot a known, independently measured distance along that wheel's
// measuring direction without turning it. The wheel's raw travel over the
// trial (READ_WHEELS: counts per revolution, gearing, polarity and radius
// applied, travel scale not applied) against the reference gives the wheel's
// combined distance scale, reference / raw travel.
//
// The reference is the travel of the robot origin. The small rotation a trial
// allows moves the wheel by its lever times the heading change; that part is
// removed using the wheel's position. Measure the reference at a point near
// the origin: a tape point L away from it adds up to L * max_rotation.
//
// That scale lumps radius error, gearing error, tire compression and slip
// together; a trial cannot tell them apart, and a constant scale does not
// fix slip, missed encoder transitions, changing compression or magnet
// eccentricity. It becomes the profile's travel_scale only when repeated
// trials agree.

#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "common/frames.h"
#include "communigatr/robot_profile.h"
#include "investigatr/geometry.h"

namespace communigatr
{

using investigatr::Meters;
using investigatr::Radians;
using investigatr::Seconds;

// One READ_WHEELS record in SI units.
struct WheelSample {
    uint8_t  port          = 0;
    bool     fresh         = false;
    bool     valid         = false;
    uint16_t discontinuity = 0;
    int32_t  counts        = 0;
    Meters   travel        = 0; // raw, no travel scale
    Seconds  age           = 0; // at the time the snapshot was taken
};

// READ_WHEELS result plus the robot heading at the same time.
struct CalibrationSnapshot {
    uint32_t                 profile_id = 0; // profile the readings were computed under
    std::vector<WheelSample> wheels;
    bool                     heading_valid = false;
    Radians                  heading       = 0;
};

WheelSample fromReading(const gatr2::WheelReading& reading);

struct WheelCalibrationConfig {
    Meters  min_reference  = 0.5;   // shorter pushes are too sensitive to the tape measure
    Meters  max_reference  = 5.0;
    double  max_correction = 0.1;   // |scale - 1| beyond this points at radius, CPR or gearing
    Radians max_rotation   = 0.0087; // heading change allowed over a trial (0.5 deg)
    double  max_cross      = 0.05;  // cross wheel travel as a share of the reference
    Seconds max_age        = 0.25;  // start and end readings at most this old
    int     min_trials     = 3;
    double  max_spread     = 0.01;  // (max - min) / mean over accepted trials
};

enum class TrialStatus : uint8_t {
    kAccepted,
    kNotStarted,       // finish without a start
    kReadingMissing,   // the wheel under test is not in the snapshot
    kReadingStale,     // a reading is not fresh or older than max_age
    kReadingInvalid,   // the encoder had no value
    kDiscontinuity,    // a source restart between start and end; motion may be lost
    kProfileChanged,   // readings from two different profiles
    kReferenceInvalid, // reference not finite or outside min..max
    kTooShort,         // the wheel measured almost nothing
    kWrongDirection,   // travel against the measuring direction: push the other way or check polarity
    kRotated,          // heading changed, or no heading to check
    kNotStraight,      // a cross wheel moved: the push was not along the wheel
    kOutOfRange,       // scale beyond max_correction: check radius, CPR and gearing first
};

const char* toString(TrialStatus status);

struct WheelTrial {
    Meters reference = 0;
    Meters measured  = 0; // raw wheel travel less the rotation part
    double scale     = 1; // reference / measured
};

struct CalibrationProposal {
    bool        ready  = false; // enough trials that agree
    double      scale  = 1;     // proposed travel_scale, mean of the trials
    double      spread = 0;     // (max - min) / mean
    std::size_t trials = 0;
};

class WheelCalibration {
public:
    // wheel: the wheel under test, its port and position. cross_ports: wheels
    // that must not move during a push along it (the perpendicular wheel of a
    // two-wheel setup).
    WheelCalibration(const TrackingWheel& wheel, std::vector<uint8_t> cross_ports,
                     const WheelCalibrationConfig& config = {});

    TrialStatus start(const CalibrationSnapshot& snapshot);
    TrialStatus finish(const CalibrationSnapshot& snapshot, Meters reference);

    bool started() const { return started_; }
    void clear();

    uint8_t                        port() const { return port_; }
    const std::vector<WheelTrial>& trials() const { return trials_; }
    CalibrationProposal            proposal() const;

private:
    const WheelSample* find(const CalibrationSnapshot& snapshot, uint8_t port) const;
    TrialStatus        check(const WheelSample* sample) const;

    uint8_t                 port_;
    Meters                  lever_; // wheel travel per radian of rotation
    std::vector<uint8_t>    cross_;
    WheelCalibrationConfig  config_;
    bool                    started_ = false;
    CalibrationSnapshot     start_;
    std::vector<WheelTrial> trials_;
};

} // namespace communigatr
