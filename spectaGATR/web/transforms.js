// transforms.js
// Pure math and state helpers for the viewer: no three.js, no DOM, so the
// test page checks them directly. Conventions: ../docs/PROTOCOL.md.
//
// Scene axes are field axes one to one: scene x = field +X (right on the
// diagram), scene y = field +Y (up on the diagram), scene z = up. The camera
// up vector is +Z. Nothing is mirrored or swapped.
//
// Quaternions are [x, y, z, w] (three.js order). Angles are radians.

export const DEG = Math.PI / 180;
export const M_TO_IN = 1 / 0.0254;
export const TRAIL_CAPACITY = 4000;
export const TRAIL_MIN_STEP_M = 0.001;
export const TRAIL_Z_M = 0.015;
export const SMOOTH_MS = 100;
export const SMOOTH_SNAP_M = 0.5;
export const DEFAULT_STALE_MS = 750;
export const DEFAULT_ROBOT = {
    front_m: 0.1685, back_m: 0.1685, left_m: 0.1465, right_m: 0.1465, height_m: 0.25,
};

export const SCENE_UP = [0, 0, 1];

// Field point to scene point: the identity, kept as a function so the
// mapping is stated once and tested.
export function fieldToScene(x, y, z = 0) {
    return [x, y, z];
}

// Wraps to (-pi, pi].
export function wrapPi(a) {
    let r = a % (2 * Math.PI);
    if (r <= -Math.PI) {
        r += 2 * Math.PI;
    } else if (r > Math.PI) {
        r -= 2 * Math.PI;
    }
    return r;
}

// Shortest signed turn from one heading to another.
export function angleDelta(from, to) {
    return wrapPi(to - from);
}

// R = Rz(yaw) * Ry(pitch) * Rx(roll), robot to field.
export function attitudeQuaternion(roll, pitch, yaw) {
    const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
    const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
    const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ];
}

// Row-major 3x3 of the same rotation, for cross-checks.
export function attitudeMatrix(roll, pitch, yaw) {
    const cr = Math.cos(roll), sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    ];
}

// q * v * conj(q).
export function rotateVector(q, v) {
    const [qx, qy, qz, qw] = q;
    const [vx, vy, vz] = v;
    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);
    return [
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    ];
}

// Yaw of a quaternion: the heading of its rotated +x projected on the floor.
export function quaternionYaw(q) {
    const f = rotateVector(q, [1, 0, 0]);
    return Math.atan2(f[1], f[0]);
}

// Shortest-path slerp.
export function slerp(a, b, t) {
    let [bx, by, bz, bw] = b;
    let dot = a[0] * bx + a[1] * by + a[2] * bz + a[3] * bw;
    if (dot < 0) {
        bx = -bx; by = -by; bz = -bz; bw = -bw;
        dot = -dot;
    }
    let ka, kb;
    if (dot > 0.999999) {
        ka = 1 - t;
        kb = t;
    } else {
        const th = Math.acos(Math.min(1, dot));
        const s = Math.sin(th);
        ka = Math.sin((1 - t) * th) / s;
        kb = Math.sin(t * th) / s;
    }
    const r = [ka * a[0] + kb * bx, ka * a[1] + kb * by, ka * a[2] + kb * bz, ka * a[3] + kb * bw];
    const n = Math.hypot(r[0], r[1], r[2], r[3]) || 1;
    return [r[0] / n, r[1] / n, r[2] / n, r[3] / n];
}

// Drawn pose of a received pose: position plus attitude quaternion. A
// heading-only pose has roll = pitch = 0. Never a height.
export function presentPose(p) {
    const roll = p.attitude ? p.roll : 0;
    const pitch = p.attitude ? p.pitch : 0;
    return { x: p.x, y: p.y, q: attitudeQuaternion(roll, pitch, p.yaw) };
}

export function blendPose(a, b, t) {
    return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, q: slerp(a.q, b.q, t) };
}

// Presentation-only smoothing: blends from the drawn pose to each new pose
// over at most SMOOTH_MS, then holds it. Snaps when disabled, on restart, on
// a jump, or when the feed is not live (a stale pose is frozen exactly).
export class Smoother {
    constructor(durationMs = SMOOTH_MS) {
        this.durationMs = durationMs;
        this.from = null;
        this.to = null;
        this.t0 = 0;
        this.serial = -1;
    }

    update(target, serial, now, enabled, snap) {
        if (serial !== this.serial) {
            const jump = this.to ? Math.hypot(target.x - this.to.x, target.y - this.to.y) : Infinity;
            this.from = enabled && !snap && this.to && jump < SMOOTH_SNAP_M ? this.current(now) : target;
            this.to = target;
            this.t0 = now;
            this.serial = serial;
        }
        if (!enabled || snap) {
            this.from = this.to;
        }
        return this.current(now);
    }

    current(now) {
        if (!this.to) {
            return null;
        }
        const t = Math.min(1, Math.max(0, (now - this.t0) / this.durationMs));
        return t >= 1 || this.from === this.to ? this.to : blendPose(this.from, this.to, t);
    }
}

// --- trail: a preallocated ring of line segments ---
//
// Each slot holds one segment (two points, six floats), so the buffer draws
// as THREE.LineSegments in any order and a break simply writes no segment.
// The oldest segment is overwritten once the ring is full.

export class TrailRing {
    constructor(capacity = TRAIL_CAPACITY, z = TRAIL_Z_M) {
        this.capacity = capacity;
        this.z = z;
        this.positions = new Float32Array(capacity * 6);
        this.head = 0;
        this.count = 0;
        this.hasLast = false;
        this.lastX = 0;
        this.lastY = 0;
        this.version = 0;
        this.breaks = 0;
    }

    clear() {
        this.head = 0;
        this.count = 0;
        this.hasLast = false;
        this.version += 1;
    }

    // Adds a point. It joins the previous point unless breakBefore is set
    // or there is none. True when a segment was written.
    push(x, y, breakBefore = false) {
        if (!Number.isFinite(x) || !Number.isFinite(y)) {
            return false;
        }
        if (breakBefore || !this.hasLast) {
            if (breakBefore && this.hasLast) {
                this.breaks += 1;
            }
            this.hasLast = true;
            this.lastX = x;
            this.lastY = y;
            return false;
        }
        const dx = x - this.lastX, dy = y - this.lastY;
        if (dx * dx + dy * dy < TRAIL_MIN_STEP_M * TRAIL_MIN_STEP_M) {
            return false;
        }
        const p = this.positions;
        const o = this.head * 6;
        p[o] = this.lastX;
        p[o + 1] = this.lastY;
        p[o + 2] = this.z;
        p[o + 3] = x;
        p[o + 4] = y;
        p[o + 5] = this.z;
        this.head = this.head + 1 === this.capacity ? 0 : this.head + 1;
        if (this.count < this.capacity) {
            this.count += 1;
        }
        this.lastX = x;
        this.lastY = y;
        this.version += 1;
        return true;
    }

    // The k-th oldest stored segment as [x0, y0, x1, y1].
    segment(k) {
        let i = this.head - this.count + k;
        if (i < 0) {
            i += this.capacity;
        }
        const o = i * 6;
        const p = this.positions;
        return [p[o], p[o + 1], p[o + 3], p[o + 4]];
    }
}

// --- received messages ---

function finite(v) {
    return typeof v === 'number' && Number.isFinite(v);
}

// A pose message as the viewer uses it, or null when unusable. Heading-only
// poses get roll = pitch = 0 whatever the message carries.
export function normalizePose(m) {
    if (!m || !finite(m.x) || !finite(m.y) || !finite(m.yaw)) {
        return null;
    }
    const attitude = m.attitude === true;
    if (attitude && (!finite(m.roll) || !finite(m.pitch))) {
        return null;
    }
    return {
        x: m.x,
        y: m.y,
        yaw: m.yaw,
        roll: attitude ? m.roll : 0,
        pitch: attitude ? m.pitch : 0,
        attitude,
        session: typeof m.session === 'string' ? m.session : '',
        epoch: finite(m.epoch) ? m.epoch : null,
        seq: finite(m.seq) ? m.seq : null,
        t_ms: finite(m.t_ms) ? m.t_ms : null,
        rx_ms: finite(m.rx_ms) ? m.rx_ms : null,
        restart: m.restart === true,
        source: typeof m.source === 'string' ? m.source : '',
    };
}

// Robot box dimensions from hello.robot; each missing or bad value falls
// back to the default.
export function normalizeRobot(r) {
    const out = { ...DEFAULT_ROBOT };
    if (r && typeof r === 'object') {
        for (const k of Object.keys(DEFAULT_ROBOT)) {
            if (finite(r[k]) && r[k] >= 0 && r[k] < 5) {
                out[k] = r[k];
            }
        }
    }
    if (out.height_m <= 0) {
        out.height_m = DEFAULT_ROBOT.height_m;
    }
    return out;
}

// Receiver-local receive age of the pose being shown. When a status from
// the same connection was sent after that pose (receiver clock), its age is
// the receiver's difference plus the local time since that status arrived;
// otherwise the local time since the pose message arrived. Never compares
// the publisher clock (t_ms) with any laptop clock.
export function poseAgeMs(f, now) {
    if (!f.pose || f.poseAt === null) {
        return null;
    }
    const s = f.status;
    if (s && f.statusConn === f.poseConn && f.statusAt !== null && finite(s.rx_ms) &&
        finite(f.pose.rx_ms) && s.rx_ms >= f.pose.rx_ms) {
        return (s.rx_ms - f.pose.rx_ms) + (now - f.statusAt);
    }
    return now - f.poseAt;
}

// What the page shows: 'disconnected' (socket closed), 'connecting' (no
// hello yet), 'searching' (receiver has no open VEX link), 'waiting' (no
// pose yet), 'stale' or 'live'.
export function feedState(f, now) {
    const ageMs = poseAgeMs(f, now);
    let state;
    if (!f.wsOpen) {
        state = 'disconnected';
    } else if (!f.hello) {
        state = 'connecting';
    } else if (f.status && f.status.link !== 'open') {
        state = 'searching';
    } else if (!f.pose) {
        state = 'waiting';
    } else {
        const s = f.status;
        const statusSaysStale = !!(s && s.stale && f.statusConn === f.poseConn &&
            (!finite(s.rx_ms) || !finite(f.pose.rx_ms) || s.rx_ms >= f.pose.rx_ms));
        state = statusSaysStale || ageMs > f.staleMs ? 'stale' : 'live';
    }
    return { state, ageMs };
}

// --- cameras ---

// Top-down camera over the field center that fits the whole field. A hair
// toward -Y keeps the orbit math regular and puts +Y at the top of the
// screen, +X to the right.
export function overheadCamera(dims, fovDeg, aspect, margin = 0.45) {
    const cx = dims.inside_x_m / 2, cy = dims.inside_y_m / 2;
    const t = Math.tan((fovDeg * DEG) / 2);
    const h = Math.max((dims.inside_y_m / 2 + margin) / t,
        (dims.inside_x_m / 2 + margin) / (t * Math.max(aspect, 1e-3)));
    return { position: [cx, cy - h * 1e-4, h], target: [cx, cy, 0] };
}

// Oblique view from the audience side (-Y), so +X stays to the right.
export function orbitCamera(dims) {
    const s = Math.max(dims.inside_x_m, dims.inside_y_m);
    const cx = dims.inside_x_m / 2, cy = dims.inside_y_m / 2;
    return { position: [cx, cy - 1.05 * s, 0.85 * s], target: [cx, cy, 0] };
}

export const FOLLOW_OFFSET = [0, -1.4, 1.2];

// --- formatting ---

export function fmt(v, digits = 3) {
    return finite(v) ? v.toFixed(digits) : '--';
}

export function fmtAge(ms) {
    if (!finite(ms)) {
        return '--';
    }
    if (ms < 1000) {
        return Math.max(0, ms).toFixed(0) + ' ms';
    }
    if (ms < 60000) {
        return (ms / 1000).toFixed(1) + ' s';
    }
    return Math.floor(ms / 60000) + ' min ' + Math.floor((ms % 60000) / 1000) + ' s';
}
