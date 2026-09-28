// smoothing.js
// Presentation-only smoothing of the drawn robot pose. It interpolates: a
// newly received pose is reached by a linear blend from the pose on screen
// over one measured state period, capped at SMOOTH_MAX_MS. The drawn robot
// therefore trails the newest received pose by at most that period (about
// 33 ms at 30 Hz), never extrapolates, and never shows a pose outside the
// segment between two received poses.
//
// It snaps with no blend on any identity change (session, reset, odometry
// epoch, anchor revision, placement, validity, a new connection), on a jump
// over SNAP_DIST_M or SNAP_HEADING_DEG, and whenever the pose is stale or
// not measured (poseFreshness): the newest received pose is then drawn
// exactly and the caller shows that state. Readouts, graphs, the trail and
// exports never read from here.

export const SMOOTH_MAX_MS = 100;
export const SNAP_DIST_M = 0.2;
export const SNAP_HEADING_DEG = 15;
export const STALE_MS = 250;

// The page's verdict on a valid pose, shared by live and replay: 'stale'
// when no new state came for STALE_MS (sinceMs) or the pose was already
// older than that, 'unmeasured' when the runtime gave no Pi-clock
// measurement time (see measuredOnSourceClock for which case), else ''.
// Unknown age is never taken as fresh.
export function poseFreshness(ageMs, sinceMs) {
    if (!(sinceMs <= STALE_MS)) {
        return 'stale';
    }
    if (typeof ageMs !== 'number' || !Number.isFinite(ageMs)) {
        return 'unmeasured';
    }
    return ageMs + Math.max(0, sinceMs) > STALE_MS ? 'stale' : '';
}

// Which 'unmeasured' case a pose is: true when it has a measurement on its
// source clock that is not mapped to the Pi clock yet (or was just reset),
// so its age is unknown; false when it has no measurement at all (a
// configured placement before the first sensor reading).
export function measuredOnSourceClock(robot) {
    const m = robot ? robot.measured_at : null;
    return !!m && typeof m === 'object' && typeof m.ms === 'number';
}

function wrap180(d) {
    let x = ((((d + 180) % 360) + 360) % 360) - 180;
    if (x === -180) {
        x = 180;
    }
    return x;
}

export class PoseSmoother {
    constructor() {
        this.enabled = true;
        this.out = { x_m: 0, y_m: 0, heading_deg: 0 };
        this.from = { x_m: 0, y_m: 0, heading_deg: 0 };
        this.to = { x_m: 0, y_m: 0, heading_deg: 0 };
        this.id = { session: '', reset: -1, epoch: -1, anchor: -1, placement: -1, placed: false, valid: false, connection: -1 };
        this.snaps = 0;
        this.blends = 0;
        this.lastSnapReason = '';
        this.reset();
    }

    reset() {
        this.has = false;
        this.t0 = 0;
        this.dur = 0;
        this.periodMs = 33;
        this.lastArrival = -1;
    }

    setEnabled(on) {
        this.enabled = !!on;
        this.dur = 0; // finish any blend now
    }

    // True when the identity differs from the stored one; stores the new one.
    identityChanged(ident) {
        const id = this.id;
        const changed = id.session !== ident.session || id.reset !== ident.reset || id.epoch !== ident.epoch ||
            id.anchor !== ident.anchor || id.placement !== ident.placement || id.placed !== ident.placed ||
            id.valid !== ident.valid || id.connection !== ident.connection;
        if (changed) {
            id.session = ident.session;
            id.reset = ident.reset;
            id.epoch = ident.epoch;
            id.anchor = ident.anchor;
            id.placement = ident.placement;
            id.placed = ident.placed;
            id.valid = ident.valid;
            id.connection = ident.connection;
        }
        return changed;
    }

    snap(pose, reason) {
        this.to.x_m = pose.x_m;
        this.to.y_m = pose.y_m;
        this.to.heading_deg = pose.heading_deg;
        this.from.x_m = pose.x_m;
        this.from.y_m = pose.y_m;
        this.from.heading_deg = pose.heading_deg;
        this.dur = 0;
        this.has = true;
        this.snaps += 1;
        this.lastSnapReason = reason;
    }

    // The newest received pose {x_m, y_m, heading_deg} with its identity
    // {session, reset, epoch, anchor, placement, placed, valid, connection};
    // nowMs is its browser receipt time. Returns the snap reason or ''.
    // Every received new pose, even one a frame never draws: the blend
    // spans the state period, not the drawing period.
    noteArrival(nowMs) {
        if (this.lastArrival >= 0) {
            const dt = nowMs - this.lastArrival;
            // only the steady stream sets the period; pauses do not
            if (dt > 0 && dt <= STALE_MS) {
                this.periodMs = this.periodMs * 0.8 + dt * 0.2;
            }
        }
        this.lastArrival = nowMs;
    }

    push(pose, ident, nowMs, stale) {
        const changed = this.identityChanged(ident);
        let reason = '';
        if (!this.has) {
            reason = 'first';
        } else if (changed) {
            reason = 'identity';
        } else if (!this.enabled) {
            reason = 'off';
        } else if (stale) {
            reason = 'stale';
        } else {
            const cur = this.sample(nowMs, false);
            const dx = pose.x_m - cur.x_m;
            const dy = pose.y_m - cur.y_m;
            if (dx * dx + dy * dy > SNAP_DIST_M * SNAP_DIST_M) {
                reason = 'jump';
            } else if (Math.abs(wrap180(pose.heading_deg - cur.heading_deg)) > SNAP_HEADING_DEG) {
                reason = 'turn';
            }
        }
        if (reason) {
            this.snap(pose, reason);
            return reason;
        }
        const cur = this.out; // filled by sample() above
        this.from.x_m = cur.x_m;
        this.from.y_m = cur.y_m;
        this.from.heading_deg = cur.heading_deg;
        this.to.x_m = pose.x_m;
        this.to.y_m = pose.y_m;
        this.to.heading_deg = pose.heading_deg;
        this.t0 = nowMs;
        this.dur = Math.min(SMOOTH_MAX_MS, this.periodMs);
        this.blends += 1;
        return '';
    }

    // The pose to draw at nowMs, in this.out (reused, never reallocated).
    sample(nowMs, stale) {
        const o = this.out;
        if (!this.enabled || stale || this.dur <= 0 || nowMs >= this.t0 + this.dur) {
            o.x_m = this.to.x_m;
            o.y_m = this.to.y_m;
            o.heading_deg = this.to.heading_deg;
            return o;
        }
        const a = Math.max(0, (nowMs - this.t0) / this.dur);
        o.x_m = this.from.x_m + (this.to.x_m - this.from.x_m) * a;
        o.y_m = this.from.y_m + (this.to.y_m - this.from.y_m) * a;
        o.heading_deg = wrap180(this.from.heading_deg + wrap180(this.to.heading_deg - this.from.heading_deg) * a);
        return o;
    }

    // How far the drawn pose still trails the newest received one, in ms.
    lagMs(nowMs) {
        if (!this.enabled || this.dur <= 0) {
            return 0;
        }
        return Math.max(0, this.t0 + this.dur - nowMs);
    }

    // True while a blend is in progress (the scene must redraw).
    moving(nowMs) {
        return this.enabled && this.dur > 0 && nowMs < this.t0 + this.dur;
    }
}
