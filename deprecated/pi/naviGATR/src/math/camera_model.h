// camera_model.h
// Pinhole projection with the Brown-Conrady distortion model in the OpenCV
// convention, on the detector-native optical frame (+x image-right, +y
// image-down, +z forward). Pixel coordinates always refer to the captured
// image: distortion is applied when projecting into it and removed when
// solving from it, so detections, pose solves and overlays agree on one
// coordinate system.
//
// The engineering camera frame (+x looking, +y left, +z up) maps to the
// optical frame as x_o = -y_e, y_o = -z_e, z_o = x_e.

#pragma once
#include <cmath>

#include "resources/camera.h"

namespace navigatr
{

inline void opticalFromEngineering(double x_e, double y_e, double z_e, double& x_o,
                                   double& y_o, double& z_o) {
    x_o = -y_e;
    y_o = -z_e;
    z_o = x_e;
}

inline void engineeringFromOptical(double x_o, double y_o, double z_o, double& x_e,
                                   double& y_e, double& z_e) {
    x_e = z_o;
    y_e = -x_o;
    z_e = -y_o;
}

// Normalized optical coordinates (x/z, y/z) through the distortion model.
inline void distortNormalized(const CameraIntrinsics& K, double x, double y, double& xd,
                              double& yd) {
    const double r2     = x * x + y * y;
    const double radial = 1.0 + K.k1 * r2 + K.k2 * r2 * r2 + K.k3 * r2 * r2 * r2;
    xd = x * radial + 2.0 * K.p1 * x * y + K.p2 * (r2 + 2.0 * x * x);
    yd = y * radial + K.p1 * (r2 + 2.0 * y * y) + 2.0 * K.p2 * x * y;
}

// Captured-image pixel of an optical-frame point in front of the camera.
inline bool projectOptical(const CameraIntrinsics& K, double x_o, double y_o, double z_o,
                           double& u, double& v) {
    if (z_o <= 1e-9) {
        return false;
    }
    double xd = x_o / z_o;
    double yd = y_o / z_o;
    if (K.hasDistortion()) {
        distortNormalized(K, x_o / z_o, y_o / z_o, xd, yd);
    }
    u = K.fx_px * xd + K.cx_px;
    v = K.fy_px * yd + K.cy_px;
    return true;
}

inline bool projectEngineering(const CameraIntrinsics& K, double x_e, double y_e, double z_e,
                               double& u, double& v) {
    double x_o = 0.0, y_o = 0.0, z_o = 0.0;
    opticalFromEngineering(x_e, y_e, z_e, x_o, y_o, z_o);
    return projectOptical(K, x_o, y_o, z_o, u, v);
}

// Inverts the distortion for one captured pixel: the normalized optical
// coordinates of its ray. Iterative fixed point, exact for the pinhole.
inline void undistortPixel(const CameraIntrinsics& K, double u, double v, double& xn,
                           double& yn) {
    const double xd = (u - K.cx_px) / K.fx_px;
    const double yd = (v - K.cy_px) / K.fy_px;
    xn              = xd;
    yn              = yd;
    if (!K.hasDistortion()) {
        return;
    }
    for (int i = 0; i < 20; ++i) {
        const double r2     = xn * xn + yn * yn;
        const double radial = 1.0 + K.k1 * r2 + K.k2 * r2 * r2 + K.k3 * r2 * r2 * r2;
        const double dx     = 2.0 * K.p1 * xn * yn + K.p2 * (r2 + 2.0 * xn * xn);
        const double dy     = K.p1 * (r2 + 2.0 * yn * yn) + 2.0 * K.p2 * xn * yn;
        xn                  = (xd - dx) / radial;
        yn                  = (yd - dy) / radial;
    }
}

// The pixel the same ray would have in an ideal pinhole image with the same
// fx, fy, cx, cy: what a distortion-free pose solver wants.
inline void undistortedPixel(const CameraIntrinsics& K, double u, double v, double& u_ideal,
                             double& v_ideal) {
    double xn = 0.0, yn = 0.0;
    undistortPixel(K, u, v, xn, yn);
    u_ideal = K.fx_px * xn + K.cx_px;
    v_ideal = K.fy_px * yn + K.cy_px;
}

} // namespace navigatr
