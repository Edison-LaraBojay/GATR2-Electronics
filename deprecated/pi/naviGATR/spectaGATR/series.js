// series.js
// Time-series storage for the graphs, live and replay. A channel is one
// message stream (state, telemetry, diag, instrumentation, or one CSV of a
// capture) with a bounded ring of sample times; each series of that channel
// has a value ring with the same slots. Missing values are NaN and stay
// missing: nothing is filled with zero. A slot can carry a break flag (a
// discontinuity such as a new odometry epoch) and an invalid flag (the
// source said the value is not valid); both are drawn, never hidden.
//
// Times are milliseconds on one clock per store (the Pi host clock). A
// sample older than the newest one in its channel is dropped and counted:
// the rings stay sorted so a draw can binary-search its window.

export const FLAG_VALID = 1;   // series slot: the source said valid
export const FLAG_BREAK = 2;   // channel slot: a discontinuity before it
export const FLAG_INVALID = 4; // series slot: the source said not valid

export class Channel {
    constructor(name, capacity) {
        this.name = name;
        this.cap = capacity;
        this.t = new Float64Array(capacity);
        this.flags = new Uint8Array(capacity);
        this.head = 0;   // next slot
        this.n = 0;
        this.series = [];
        this.pendingBreak = false;
        this.outOfOrder = 0;
        this.version = 0;
        // recent intervals, for the gap threshold
        this.iv = new Float64Array(16);
        this.ivN = 0;
        this.ivHead = 0;
        this.gapMs = Infinity;
        this.minGapMs = 0;
    }

    clear() {
        this.head = 0;
        this.n = 0;
        this.pendingBreak = false;
        this.ivN = 0;
        this.ivHead = 0;
        this.gapMs = Infinity;
        this.version += 1;
        for (const s of this.series) {
            s.v.fill(NaN);
        }
    }

    // Physical slot of logical index k (0 = oldest).
    slot(k) {
        const s = this.head - this.n + k;
        return s < 0 ? s + this.cap : (s >= this.cap ? s - this.cap : s);
    }

    newest() {
        return this.n ? this.t[this.slot(this.n - 1)] : -Infinity;
    }

    oldest() {
        return this.n ? this.t[this.slot(0)] : Infinity;
    }

    // Starts a sample at time t (ms); returns the slot or -1 when dropped.
    // A sample at the newest time replaces it (the newer message wins); an
    // earlier one is dropped and counted.
    begin(t) {
        if (!(typeof t === 'number' && Number.isFinite(t))) {
            return -1;
        }
        const last = this.newest();
        if (this.n && t === last) {
            const i = this.slot(this.n - 1);
            for (const s of this.series) {
                s.v[i] = NaN;
                s.f[i] = 0;
            }
            this.version += 1;
            return i;
        }
        if (this.n && t < last) {
            this.outOfOrder += 1;
            return -1;
        }
        if (this.n) {
            this.noteInterval(t - last);
        }
        const i = this.head;
        this.t[i] = t;
        this.flags[i] = this.pendingBreak ? FLAG_BREAK : 0;
        this.pendingBreak = false;
        for (const s of this.series) {
            s.v[i] = NaN;
            s.f[i] = 0;
        }
        this.head = i + 1 === this.cap ? 0 : i + 1;
        if (this.n < this.cap) {
            this.n += 1;
        }
        this.version += 1;
        return i;
    }

    // The next sample starts a new segment (no line drawn across).
    markBreak() {
        this.pendingBreak = true;
    }

    noteInterval(dt) {
        this.iv[this.ivHead] = dt;
        this.ivHead = (this.ivHead + 1) & 15;
        if (this.ivN < 16) {
            this.ivN += 1;
        }
        // 3x the median recent interval: robust to the gap being measured;
        // recomputed every 8 samples to keep the sort off the common path
        if ((this.ivHead & 7) === 0 || this.gapMs === Infinity) {
            const tmp = Channel.tmp || (Channel.tmp = new Float64Array(16));
            for (let k = 0; k < this.ivN; ++k) {
                tmp[k] = this.iv[k];
            }
            const view = tmp.subarray(0, this.ivN);
            view.sort();
            this.gapMs = Math.max(3 * view[this.ivN >> 1], this.minGapMs);
        }
    }
}

export class Series {
    constructor(id, channel, meta) {
        this.id = id;
        this.channel = channel;
        this.label = meta.label || id;
        this.unit = meta.unit || '';
        this.source = meta.source || channel.name;
        this.group = meta.group || 'other';
        this.wrap = !!meta.wrap;          // degrees, (-180, 180]
        this.note = meta.note || '';
        this.v = new Float32Array(channel.cap).fill(NaN);
        this.f = new Uint8Array(channel.cap);
        channel.series.push(this);
    }

    // value may be null/undefined/NaN for missing; valid false marks the slot.
    set(slot, value, valid) {
        if (slot < 0) {
            return;
        }
        this.v[slot] = typeof value === 'number' && Number.isFinite(value) ? value : NaN;
        this.f[slot] = valid === false ? FLAG_INVALID : FLAG_VALID;
    }
}

export class SeriesStore {
    constructor() {
        this.channels = new Map();
        this.series = new Map();
        this.version = 0;
    }

    channel(name, capacity, minGapMs) {
        let c = this.channels.get(name);
        if (!c) {
            c = new Channel(name, capacity || 2048);
            c.minGapMs = minGapMs || 0;
            this.channels.set(name, c);
            this.version += 1;
        }
        return c;
    }

    // The series for id on channel, created on first use.
    get(id, channel, meta) {
        let s = this.series.get(id);
        if (!s) {
            s = new Series(id, channel, meta || {});
            this.series.set(id, s);
            this.version += 1;
        }
        return s;
    }

    clear() {
        for (const c of this.channels.values()) {
            c.clear();
        }
        this.version += 1;
    }

    // Newest sample time over all channels.
    newest() {
        let t = -Infinity;
        for (const c of this.channels.values()) {
            t = Math.max(t, c.newest());
        }
        return t;
    }

    oldest() {
        let t = Infinity;
        for (const c of this.channels.values()) {
            t = Math.min(t, c.oldest());
        }
        return t;
    }
}

// First logical index with t >= t0 (n when none).
export function lowerBound(channel, t0) {
    let lo = 0, hi = channel.n;
    while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (channel.t[channel.slot(mid)] < t0) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

// Value of a series at or before t, with its time: {t, v, valid} or null.
export function valueAt(series, t, out) {
    const c = series.channel;
    const k = lowerBound(c, t + 1e-9) - 1;
    if (k < 0) {
        return null;
    }
    const s = c.slot(k);
    const o = out || {};
    o.t = c.t[s];
    o.v = series.v[s];
    o.valid = (series.f[s] & FLAG_INVALID) === 0;
    return o;
}

function colOf(t, t0, span, cols) {
    const col = Math.floor(((t - t0) / span) * cols);
    return col < 0 ? 0 : (col >= cols ? cols - 1 : col);
}

function wrapDeg(d) {
    let x = ((((d + 180) % 360) + 360) % 360) - 180;
    if (x === -180) {
        x = 180;
    }
    return x;
}

// Min/max decimation of one series over [t0, t1] into cols pixel columns.
// Each column keeps min, max, first and last of its samples plus flags:
// a spike inside a column survives as its min or max. A column is marked
// broken-before when a gap (dt > gapMs), a break flag or a NaN separates it
// from the previous drawn sample; invalid samples mark the column invalid.
// Heading series are unwrapped inside each segment first (continuous), so
// a column never spans a wrap; drawWrapped maps back and splits at +-180.
// Outputs are preallocated Float32Arrays of length >= cols; returns the
// value range {lo, hi} of what was decimated, or null when empty.
export class Decimator {
    constructor() {
        this.cap = 0;
        this.ensure(256);
    }

    ensure(cols) {
        if (cols <= this.cap) {
            return;
        }
        this.cap = Math.max(cols, this.cap * 2);
        this.min = new Float32Array(this.cap);
        this.max = new Float32Array(this.cap);
        this.first = new Float32Array(this.cap);
        this.last = new Float32Array(this.cap);
        this.has = new Uint8Array(this.cap);
        this.brk = new Uint8Array(this.cap);   // gap before this column's first sample
        this.bad = new Uint8Array(this.cap);   // an invalid sample in this column
        this.inner = new Uint8Array(this.cap); // a gap inside the column
    }

    run(series, t0, t1, cols) {
        this.ensure(cols);
        const c = series.channel;
        this.has.fill(0, 0, cols);
        this.brk.fill(0, 0, cols);
        this.bad.fill(0, 0, cols);
        this.inner.fill(0, 0, cols);
        if (!c.n || !(t1 > t0)) {
            return null;
        }
        const gap = c.gapMs;
        const span = t1 - t0;
        let k = lowerBound(c, t0);
        // the sample before the window carries continuity and the unwrap
        let prevT = NaN, prevV = NaN, prevRaw = NaN;
        if (k > 0) {
            const s = c.slot(k - 1);
            if (!Number.isNaN(series.v[s])) {
                prevT = c.t[s];
                prevV = prevRaw = series.v[s];
            }
        }
        let lo = Infinity, hi = -Infinity;
        for (; k < c.n; ++k) {
            const s = c.slot(k);
            const t = c.t[s];
            if (t > t1) {
                break;
            }
            const raw = series.v[s];
            if (Number.isNaN(raw)) {
                if (series.f[s] & FLAG_INVALID) {
                    this.bad[colOf(t, t0, span, cols)] = 1;
                }
                prevT = t;
                prevV = prevRaw = NaN;
                continue;
            }
            const broken = Number.isNaN(prevV) || (c.flags[s] & FLAG_BREAK) !== 0 || t - prevT > gap;
            const v = series.wrap && !broken ? prevV + wrapDeg(raw - prevRaw) : raw;
            const col = colOf(t, t0, span, cols);
            if (!this.has[col]) {
                this.has[col] = 1;
                this.min[col] = v;
                this.max[col] = v;
                this.first[col] = v;
                this.brk[col] = broken ? 1 : 0;
            } else {
                if (broken) {
                    this.inner[col] = 1;
                }
                if (v < this.min[col]) {
                    this.min[col] = v;
                }
                if (v > this.max[col]) {
                    this.max[col] = v;
                }
            }
            this.last[col] = v;
            if (series.f[s] & FLAG_INVALID) {
                this.bad[col] = 1;
            }
            if (v < lo) {
                lo = v;
            }
            if (v > hi) {
                hi = v;
            }
            prevT = t;
            prevV = v;
            prevRaw = raw;
        }
        if (lo === Infinity) {
            return null;
        }
        this.lo = lo;
        this.hi = hi;
        return this;
    }
}

// Draws one decimated series. y(v) maps a value to a pixel row; x(col) the
// column to a pixel x. Wrapped series are drawn in (-180, 180] with the
// line split where it crosses +-180.
export function drawDecimated(ctx, d, cols, x, y, wrap) {
    const map = wrap ? wrapDeg : null;
    let penDown = false;
    let lastX = 0, lastV = 0;
    ctx.beginPath();
    for (let col = 0; col < cols; ++col) {
        if (!d.has[col]) {
            continue;
        }
        const px = x(col);
        const first = d.first[col];
        if (d.brk[col] || !penDown) {
            penDown = false;
        }
        // connect from the previous column's last value
        if (penDown) {
            segment(ctx, lastX, lastV, px, first, y, map);
        } else {
            moveTo(ctx, px, first, y, map);
        }
        // the column's full extent: a vertical stroke from min to max
        if (d.max[col] !== d.min[col]) {
            vertical(ctx, px, d.min[col], d.max[col], y, map);
            moveTo(ctx, px, d.last[col], y, map);
        }
        penDown = true;
        lastX = px;
        lastV = d.last[col];
    }
    ctx.stroke();
}

function moveTo(ctx, px, v, y, map) {
    ctx.moveTo(px, y(map ? map(v) : v));
}

// A line between two (unwrapped) values; wrapped: split at the boundary.
function segment(ctx, x0, v0, x1, v1, y, map) {
    if (!map) {
        ctx.lineTo(x1, y(v1));
        return;
    }
    const w0 = map(v0);
    const k0 = Math.round((v0 - w0) / 360);
    const w1 = map(v1);
    const k1 = Math.round((v1 - w1) / 360);
    if (k0 === k1) {
        ctx.lineTo(x1, y(w1));
        return;
    }
    // crosses a wrap: stop at the edge, restart on the other side
    const edge = k1 > k0 ? 180 : -180;
    const tb = (k0 * 360 + edge - v0) / (v1 - v0);
    const xb = x0 + (x1 - x0) * tb;
    ctx.lineTo(xb, y(edge));
    ctx.moveTo(xb, y(-edge));
    ctx.lineTo(x1, y(w1));
}

function vertical(ctx, px, lo, hi, y, map) {
    if (!map) {
        ctx.moveTo(px, y(lo));
        ctx.lineTo(px, y(hi));
        return;
    }
    if (hi - lo >= 360) {
        ctx.moveTo(px, y(-180));
        ctx.lineTo(px, y(180));
        return;
    }
    const wl = map(lo);
    const wh = map(hi);
    if (wl <= wh) {
        ctx.moveTo(px, y(wl));
        ctx.lineTo(px, y(wh));
    } else {
        ctx.moveTo(px, y(wl));
        ctx.lineTo(px, y(180));
        ctx.moveTo(px, y(-180));
        ctx.lineTo(px, y(wh));
    }
}
