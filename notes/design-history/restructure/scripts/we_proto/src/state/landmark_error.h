// landmark_error.h
// A landmark's error from its map nominal pose, in field axes. (0, 0, 0) is
// the field as built; the map's heading zero is only an authoring convention.

#pragma once
#include <cmath>

#include "math/angles.h"
#include "math/transforms.h"
#include "math/units.h"

namespace navigatr
{

struct LandmarkError {
    Meters  dx     = 0.0;
    Meters  dy     = 0.0;
    Radians dtheta = 0.0;
};

// estimated minus nominal
inline LandmarkError landmarkError(const Pose2D& nominal, const Pose2D& estimated) {
    return LandmarkError{estimated.x_m - nominal.x_m, estimated.y_m - nominal.y_m,
                         wrapAngle(estimated.heading_rad - nominal.heading_rad)};
}

// A nominal target relative to a landmark, moved with the landmark's error:
// its offset from the nominal landmark rotates by dtheta, then shifts by
// (dx, dy). Written as target plus correction so zero error is exact.
inline Pose2D applyLandmarkError(const LandmarkError& error, const Pose2D& landmark_nominal,
                                 const Pose2D& target_nominal) {
    const Meters ox = target_nominal.x_m - landmark_nominal.x_m;
    const Meters oy = target_nominal.y_m - landmark_nominal.y_m;
    const double c  = std::cos(error.dtheta);
    const double s  = std::sin(error.dtheta);
    return Pose2D{target_nominal.x_m + error.dx + (c - 1.0) * ox - s * oy,
                  target_nominal.y_m + error.dy + s * ox + (c - 1.0) * oy,
                  wrapAngle(target_nominal.heading_rad + error.dtheta)};
}

} // namespace navigatr
