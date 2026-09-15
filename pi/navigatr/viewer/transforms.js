// transforms.js
// Re-expression helpers over the document transform shapes: planar poses are
// {x_m, y_m, heading_deg}; 3D transforms {x_m, y_m, z_m, roll_deg, pitch_deg,
// yaw_deg, R[9]} with R row-major and R = Rz(yaw) Ry(pitch) Rx(roll), the
// convention of docs/coordinates.md. Nothing here estimates anything: it
// composes what a document already says, and projects points through the
// camera model the runtime itself uses (src/math/camera_model.h).

export const DEG = Math.PI / 180;

export function wrapDeg(d) {
    let x = ((((d + 180) % 360) + 360) % 360) - 180;
    if (x === -180) {
        x = 180;
    }
    return x;
}

// a then b: the pose of b, given in a's frame, expressed in a's parent frame.
export function composePlanar(a, b) {
    const h = a.heading_deg * DEG;
    const c = Math.cos(h);
    const s = Math.sin(h);
    return {
        x_m: a.x_m + c * b.x_m - s * b.y_m,
        y_m: a.y_m + s * b.x_m + c * b.y_m,
        heading_deg: wrapDeg(a.heading_deg + b.heading_deg),
    };
}

// Row-major Rz(yaw) Ry(pitch) Rx(roll).
export function rotationRpy(roll_deg, pitch_deg, yaw_deg) {
    const r = roll_deg * DEG;
    const p = pitch_deg * DEG;
    const y = yaw_deg * DEG;
    const cr = Math.cos(r), sr = Math.sin(r);
    const cp = Math.cos(p), sp = Math.sin(p);
    const cy = Math.cos(y), sy = Math.sin(y);
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    ];
}

// The document's R when present, else rebuilt from its angles.
export function rotationOf(T) {
    if (T && Array.isArray(T.R) && T.R.length === 9) {
        return T.R;
    }
    return rotationRpy(T.roll_deg || 0, T.pitch_deg || 0, T.yaw_deg || 0);
}

export function transformPoint(T, p) {
    const R = rotationOf(T);
    return [
        R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + (T.x_m || 0),
        R[3] * p[0] + R[4] * p[1] + R[5] * p[2] + (T.y_m || 0),
        R[6] * p[0] + R[7] * p[1] + R[8] * p[2] + (T.z_m || 0),
    ];
}

// Captured-image pixel of a point given in the engineering camera frame
// (+x looking through the lens, +y camera-left, +z camera-up), the model
// of src/math/camera_model.h so overlays land where the detector looked:
//
//   optical     x_o = -y_e, y_o = -z_e, z_o = x_e
//   normalized  x = x_o / z_o, y = y_o / z_o, r2 = x^2 + y^2
//   radial      1 + k1 r2 + k2 r2^2 + k3 r2^3
//   xd = x radial + 2 p1 x y + p2 (r2 + 2 x^2)
//   yd = y radial + p1 (r2 + 2 y^2) + 2 p2 x y
//   u = fx xd + cx,  v = fy yd + cy
//
// The optical frame is +x image-right, +y image-down, +z forward:
// x_o = -y_e, y_o = -z_e, z_o = x_e. Returns null behind the camera.
export function projectEngineering(K, p) {
    const x_o = -p[1];
    const y_o = -p[2];
    const z_o = p[0];
    if (!(z_o > 1e-9)) {
        return null;
    }
    const x = x_o / z_o;
    const y = y_o / z_o;
    const k1 = K.k1 || 0, k2 = K.k2 || 0, k3 = K.k3 || 0, p1 = K.p1 || 0, p2 = K.p2 || 0;
    const r2 = x * x + y * y;
    const radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    const yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    return [K.fx_px * xd + K.cx_px, K.fy_px * yd + K.cy_px];
}

export function fmt(v, digits = 3) {
    return typeof v === 'number' && Number.isFinite(v) ? v.toFixed(digits) : 'n/a';
}

export function fmtMs(ms) {
    if (typeof ms !== 'number' || !Number.isFinite(ms)) {
        return 'n/a';
    }
    if (ms < 1000) {
        return ms.toFixed(0) + ' ms';
    }
    return (ms / 1000).toFixed(1) + ' s';
}
