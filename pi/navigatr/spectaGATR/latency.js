// latency.js
// The three latency terms of the inspect/2 contract, each on one clock:
//
//   source age at publish  state.host_ms - robot.measured_at_host_ms   Pi clock only
//   publish to receive     receipt mapped to the Pi clock through the
//                          ping/pong offset, minus state.host_us       estimate, +-RTT/2
//   receive to render      message arrival to the frame that drew it  browser clock only
//
// A Pi timestamp is never subtracted from a browser timestamp directly.
// The offset comes from the minimum-RTT ping of the last 20: that ping
// waited least in any queue, so its midpoint is the best estimate of when
// the Pi stamped it. The error bound is that ping's RTT/2.

export const PING_KEEP = 20;

export class ClockSync {
    constructor() {
        // ids keep counting across resets: a pong still in flight from
        // before a reset can never match a newer ping's send time
        this.nextId = 1;
        this.reset();
    }

    reset() {
        this.samples = [];   // {rtt, offset, at}
        this.best = null;
        this.pending = new Map();
        this.sent = 0;
        this.received = 0;
    }

    // Records a ping send; returns the message to send.
    ping(nowMs) {
        const id = this.nextId++;
        this.pending.set(id, nowMs);
        if (this.pending.size > 64) {
            this.pending.delete(this.pending.keys().next().value);
        }
        this.sent += 1;
        return { type: 'ping', id, client_ms: nowMs };
    }

    // A pong {id, client_ms, host_ms, host_us} received at nowMs. The
    // server echoes client_ms verbatim, so the send time is the ping's own;
    // a pending entry that disagrees belongs to another ping: dropped.
    pong(msg, nowMs) {
        const pending = this.pending.get(msg.id);
        this.pending.delete(msg.id);
        const echoed = typeof msg.client_ms === 'number' ? msg.client_ms : undefined;
        if (echoed !== undefined && pending !== undefined && Math.abs(pending - echoed) > 1e-6) {
            return null;
        }
        const sentAt = echoed !== undefined ? echoed : pending;
        if (typeof sentAt !== 'number') {
            return null;
        }
        const host = typeof msg.host_us === 'number' ? msg.host_us / 1000 : msg.host_ms;
        if (typeof host !== 'number') {
            return null;
        }
        const rtt = nowMs - sentAt;
        if (!(rtt >= 0)) {
            return null;
        }
        this.received += 1;
        const s = { rtt, offset: host - (sentAt + nowMs) / 2, at: nowMs };
        this.samples.push(s);
        if (this.samples.length > PING_KEEP) {
            this.samples.shift();
        }
        this.best = this.samples.reduce((a, b) => (b.rtt < a.rtt ? b : a));
        return s;
    }

    // Browser time mapped to the Pi clock, or NaN before the first pong.
    toPi(browserMs) {
        return this.best ? browserMs + this.best.offset : NaN;
    }

    // Half the best RTT: the error bound of any mapped time.
    uncertaintyMs() {
        return this.best ? this.best.rtt / 2 : NaN;
    }
}

// A bounded window of numbers with order statistics on demand.
export class Stat {
    constructor(size) {
        this.buf = new Float64Array(size || 256);
        this.n = 0;
        this.head = 0;
        this.last = NaN;
        this.sorted = new Float64Array(this.buf.length);
    }

    push(v) {
        if (!Number.isFinite(v)) {
            return;
        }
        this.buf[this.head] = v;
        this.head = (this.head + 1) % this.buf.length;
        if (this.n < this.buf.length) {
            this.n += 1;
        }
        this.last = v;
    }

    clear() {
        this.n = 0;
        this.head = 0;
        this.last = NaN;
    }

    // {p50, p95, max, n} over the window; NaN fields when empty.
    summary(out) {
        const o = out || {};
        o.n = this.n;
        if (!this.n) {
            o.p50 = o.p95 = o.max = o.mean = NaN;
            return o;
        }
        const s = this.sorted.subarray(0, this.n);
        s.set(this.buf.subarray(0, this.n));
        s.sort();
        o.p50 = s[Math.floor((this.n - 1) * 0.5)];
        o.p95 = s[Math.floor((this.n - 1) * 0.95)];
        o.max = s[this.n - 1];
        let sum = 0;
        for (let i = 0; i < this.n; ++i) {
            sum += s[i];
        }
        o.mean = sum / this.n;
        return o;
    }
}

// Messages per second over the last whole second, per key.
export class Rates {
    constructor() {
        this.counts = new Map();
        this.rates = new Map();
        this.windowStart = -1;
    }

    add(key, nowMs) {
        this.roll(nowMs);
        this.counts.set(key, (this.counts.get(key) || 0) + 1);
    }

    roll(nowMs) {
        if (this.windowStart < 0) {
            this.windowStart = nowMs;
            return;
        }
        const dt = nowMs - this.windowStart;
        if (dt >= 1000) {
            for (const k of this.rates.keys()) {
                if (!this.counts.has(k)) {
                    this.rates.set(k, 0);
                }
            }
            for (const [k, c] of this.counts) {
                this.rates.set(k, (c * 1000) / dt);
            }
            this.counts.clear();
            this.windowStart = nowMs;
        }
    }

    rate(key) {
        return this.rates.get(key) || 0;
    }
}
