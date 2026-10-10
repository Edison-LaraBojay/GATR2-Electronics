// localization.cpp

#include "localization.h"

namespace localization
{

int32_t channelCount(const encoder_record::Record& record, int channel) {
    return channel == 0 ? record.channel0 : record.channel1;
}

int32_t channelDelta(const Step& step, int channel) {
    return channel == 0 ? step.channel0 : step.channel1;
}

StepKind applyRecord(State& state, const odometry::WheelSetup& wheels,
                     const encoder_record::Record& record, uint32_t arrived_ms,
                     const ImuReading& imu) {
    // No heading, no update. The next good reading starts a new baseline.
    if (!imu.valid) {
        state.encoders.rebase();
        ++state.health.imu_faults;
        return StepKind::kImuInvalid;
    }

    const Step step = state.encoders.accept(record, arrived_ms);
    switch (step.kind) {
    case StepKind::kBaseline:
        // Ties the IMU to the current heading: the initial heading at start,
        // the last known heading after an IMU fault.
        state.imu           = {imu.rotation_degrees, state.pose.heading};
        state.have_baseline = true;
        break;

    case StepKind::kDuplicate:
        ++state.health.duplicates;
        break;

    case StepKind::kAdvance: {
        const odometry::Radians heading = odometry::imuHeading(state.imu, imu.rotation_degrees);
        const odometry::Radians turn    = odometry::wrapAngle(heading - state.pose.heading);
        const odometry::BodyMotion motion =
            odometry::bodyMotion(wheels, channelDelta(step, wheels.forward.channel),
                                 channelDelta(step, wheels.sideways.channel), turn);
        state.pose         = odometry::advance(state.pose, motion);
        state.pose.heading = heading;
        ++state.health.advances;
        break;
    }

    // Rebased: x and y stay where they were, the IMU still knows the heading.
    case StepKind::kRestart:
    case StepKind::kResume:
    case StepKind::kJump:
        state.pose.heading = odometry::imuHeading(state.imu, imu.rotation_degrees);
        if (step.kind == StepKind::kRestart) {
            ++state.health.restarts;
        } else if (step.kind == StepKind::kResume) {
            ++state.health.gaps;
        } else {
            ++state.health.jumps;
        }
        break;

    case StepKind::kImuInvalid:
        break;
    }
    return step.kind;
}

} // namespace localization
