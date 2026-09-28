// graphs.js
// Live and replay time-series plots on canvas. The collector turns each
// received message into samples in bounded rings (series.js); the panel
// draws the selected plots at most GRAPH_HZ while its tab is visible, with
// min/max decimation per pixel column so spikes survive, gaps (no sample
// for 3x the stream's usual interval, or a discontinuity) drawn as gaps,
// invalid spans marked in red under the plot, and event markers. Nothing
// is interpolated, filled or smoothed: a value is what a message said.
//
// Clock: every live series is on the Pi host clock (ms). Robot series are
// placed at their measurement time (measured_at_host_ms); telemetry,
// diag and instrumentation series at the time the Pi built the message.
// Replay series are at the capture's pi_host_us.

import { Decimator, drawDecimated, valueAt } from './series.js';
import { el, setText, setHidden, setTitle } from './dom.js';
import { attitudeStatus } from './transforms.js';

export const GRAPH_HZ = 15;
const COLORS = ['#4fc3f7', '#ffb74d', '#81c784', '#e57373', '#ba68c8', '#fff176', '#4db6ac', '#f06292', '#a1887f', '#90a4ae'];

// Plot definitions: fixed series ids, or a prefix for per-wheel/per-source
// series created as they appear. Units are the axis units.
export const PLOTS = [
    { id: 'xy', title: 'Position x, y', unit: 'm', ids: ['robot.x', 'robot.y'] },
    { id: 'heading', title: 'Heading', unit: 'deg', ids: ['robot.heading'], wrap: true },
    { id: 'vel', title: 'Body velocity: actual vs commanded', unit: 'm/s', ids: ['robot.vbx', 'robot.vby', 'tel.cmd_vx', 'tel.cmd_vy'] },
    { id: 'velodom', title: 'Velocity, odometry frame', unit: 'm/s', ids: ['robot.vx', 'robot.vy'] },
    { id: 'yawrate', title: 'Yaw rate: actual vs commanded', unit: 'deg/s', ids: ['robot.yaw_rate', 'tel.cmd_omega'] },
    { id: 'attitude', title: 'Roll / pitch', unit: 'deg', ids: ['robot.roll', 'robot.pitch', 'tel.roll', 'tel.pitch'] },
    { id: 'track', title: 'Tracking errors (distances)', unit: 'm', ids: ['tel.cross_track', 'tel.distance_error'] },
    { id: 'headerr', title: 'Heading error (destination minus robot)', unit: 'deg', ids: ['tel.heading_error'], wrap: true },
    { id: 'rpm', title: 'Wheel rpm targets (Brain)', unit: 'rpm', prefix: 'tel.rpm.' },
    { id: 'wcounts', title: 'Wheel counts (Brain link wheels)', unit: 'counts', prefix: 'wheel.counts.' },
    { id: 'wrate', title: 'Wheel count change rate', unit: 'counts/s', prefix: 'wheel.rate.' },
    { id: 'wtravel', title: 'Wheel travel (raw, before travel scale)', unit: 'm', prefix: 'wheel.travel.' },
    { id: 'enc', title: 'Pico encoder counts (instrumentation)', unit: 'counts', prefix: 'enc.counts.' },
    { id: 'imu', title: 'IMU gyro bias (calibration)', unit: 'deg/s', prefix: 'imu.bias.' },
    { id: 'vexrot', title: 'VEX IMU rotation (bench sample)', unit: 'deg', ids: ['imu.vex_rotation'] },
    { id: 'ages', title: 'Ages: robot pose at publish, sources at receipt', unit: 'ms', prefix: 'age.' },
    { id: 'lrate', title: 'Link packet rate', unit: 'packets/s', prefix: 'link.rate.' },
    { id: 'lerr', title: 'Link errors (cumulative counts)', unit: 'count', prefix: 'link.err.' },
    { id: 'workers', title: 'Worker loop rates', unit: 'Hz', prefix: 'worker.' },
];
const DEFAULT_PLOTS = ['xy', 'heading', 'vel', 'yawrate'];

const META = {
    'robot.x': { label: 'x', unit: 'm', source: 'state robot.field (odom before placement)' },
    'robot.y': { label: 'y', unit: 'm', source: 'state robot.field (odom before placement)' },
    'robot.heading': { label: 'heading', unit: 'deg', source: 'state robot.field', wrap: true },
    'robot.vx': { label: 'vx (odometry frame)', unit: 'm/s', source: 'state robot.vx_m_s' },
    'robot.vy': { label: 'vy (odometry frame)', unit: 'm/s', source: 'state robot.vy_m_s' },
    'robot.vbx': { label: 'vx actual (body)', unit: 'm/s', source: 'state robot.vx/vy_m_s (odometry frame) rotated into the body frame by the same state\'s odometry heading' },
    'robot.vby': { label: 'vy actual (body)', unit: 'm/s', source: 'state robot.vx/vy_m_s (odometry frame) rotated into the body frame by the same state\'s odometry heading' },
    'imu.vex_rotation': { label: 'rotation', unit: 'deg', source: 'instrumentation brain.vex_imu.rotation_deg (continuous, CCW positive)' },
    'robot.yaw_rate': { label: 'yaw rate actual', unit: 'deg/s', source: 'state robot.yaw_rate_deg_s' },
    'robot.roll': { label: 'roll (Pi attitude)', unit: 'deg', source: 'state robot.attitude, measured only' },
    'robot.pitch': { label: 'pitch (Pi attitude)', unit: 'deg', source: 'state robot.attitude, measured only' },
    'tel.cmd_vx': { label: 'vx commanded', unit: 'm/s', source: 'Brain telemetry cmd (body)' },
    'tel.cmd_vy': { label: 'vy commanded', unit: 'm/s', source: 'Brain telemetry cmd (body)' },
    'tel.cmd_omega': { label: 'omega commanded', unit: 'deg/s', source: 'Brain telemetry cmd' },
    'tel.roll': { label: 'roll (Brain VEX)', unit: 'deg', source: 'Brain telemetry attitude' },
    'tel.pitch': { label: 'pitch (Brain VEX)', unit: 'deg', source: 'Brain telemetry attitude' },
    'tel.cross_track': { label: 'cross-track', unit: 'm', source: 'Brain telemetry' },
    'tel.distance_error': { label: 'distance to destination', unit: 'm', source: 'Brain telemetry' },
    'tel.heading_error': { label: 'heading error', unit: 'deg', source: 'Brain telemetry', wrap: true },
    'age.robot': { label: 'robot pose age at publish', unit: 'ms', source: 'state host_ms - measured_at_host_ms' },
};

function metaFor(id) {
    if (META[id]) {
        return META[id];
    }
    const parts = id.split('.');
    const key = parts.slice(2).join('.') || parts[1];
    switch (parts[0] + '.' + parts[1]) {
    case 'tel.rpm': return { label: `wheel ${key} rpm target`, unit: 'rpm', source: 'Brain telemetry wheels' };
    case 'wheel.counts': return { label: `port ${key} counts`, unit: 'counts', source: 'diag brain_link.wheels' };
    case 'wheel.rate': return { label: `port ${key} change`, unit: 'counts/s', source: 'diag brain_link.wheels, difference of consecutive samples' };
    case 'wheel.travel': return { label: `port ${key} travel`, unit: 'm', source: 'diag brain_link.wheels travel_m' };
    case 'enc.counts': return { label: `encoder ${key}`, unit: 'counts', source: 'instrumentation pico.encoders' };
    case 'imu.bias': return { label: `${key} bias`, unit: 'deg/s', source: 'diag localization stillness' };
    case 'link.rate': return { label: `${key}`, unit: 'packets/s', source: /\(instr\)$/.test(key)
        ? 'instrumentation rates.rx_frames_s (the Pi\'s own one-second rate)' : 'diag link counters, difference of consecutive samples' };
    case 'link.err': return { label: `${key}`, unit: 'count', source: 'diag/instrumentation link counters' };
    case 'worker.rate': return { label: `${key}`, unit: 'Hz', source: 'diag workers' };
    default: break;
    }
    if (parts[0] === 'age') {
        return { label: `${parts.slice(1).join('.')} receipt age`, unit: 'ms', source: 'diag sources receipt_age_ms' };
    }
    if (parts[0] === 'csv') {
        return { label: parts.slice(2).join('.'), unit: '', source: parts[1] + '.csv' };
    }
    return { label: id, unit: '', source: '' };
}

function num(v) {
    return typeof v === 'number' && Number.isFinite(v) ? v : NaN;
}

// Live collector: messages in, samples out. Keeps the previous values it
// needs for differences and discontinuity breaks.
export class LiveCollector {
    constructor(store) {
        this.store = store;
        this.state = store.channel('state', 4096, 0);
        this.tel = store.channel('telemetry', 2048, 0);
        this.diag = store.channel('diag', 1024, 0);
        this.inst = store.channel('instrumentation', 1024, 0);
        const s = (id) => store.get(id, this.state, metaFor(id));
        this.sx = s('robot.x');
        this.sy = s('robot.y');
        this.sh = s('robot.heading');
        this.svx = s('robot.vx');
        this.svy = s('robot.vy');
        this.svbx = s('robot.vbx');
        this.svby = s('robot.vby');
        this.swz = s('robot.yaw_rate');
        this.sroll = s('robot.roll');
        this.spitch = s('robot.pitch');
        this.sage = s('age.robot');
        const t = (id) => store.get(id, this.tel, metaFor(id));
        this.tcvx = t('tel.cmd_vx');
        this.tcvy = t('tel.cmd_vy');
        this.tcw = t('tel.cmd_omega');
        this.troll = t('tel.roll');
        this.tpitch = t('tel.pitch');
        this.tct = t('tel.cross_track');
        this.tde = t('tel.distance_error');
        this.the = t('tel.heading_error');
        this.lastPublication = null;
        this.ident = { session: null, reset: null, epoch: null, anchor: null, placed: null, placement: null };
        this.prevWheels = new Map();
        this.prevLinks = new Map();
        this.prevDiagT = NaN;
    }

    clear() {
        this.store.clear();
        this.lastPublication = null;
        this.ident.session = null;
        this.prevWheels.clear();
        this.prevLinks.clear();
        this.prevDiagT = NaN;
    }

    // Marks a discontinuity on every channel (session reset, reconnect).
    breakAll() {
        for (const c of this.store.channels.values()) {
            c.markBreak();
        }
        this.prevWheels.clear();
        this.prevLinks.clear();
    }

    onState(msg) {
        const r = msg.robot;
        if (!r) {
            return;
        }
        // one sample per new publication; keepalives repeat the last one
        const pub = typeof msg.publication === 'number' ? msg.publication : msg.host_ms;
        if (pub === this.lastPublication) {
            return;
        }
        this.lastPublication = pub;
        const id = this.ident;
        const placed = !!r.initialized;
        const sess = msg.session || {};
        if (id.session !== null && (id.session !== sess.id || id.reset !== sess.reset_count || id.epoch !== r.odometry_epoch ||
            id.anchor !== r.anchor_revision || id.placed !== placed || id.placement !== r.placement_sequence)) {
            this.state.markBreak();
        }
        id.session = sess.id;
        id.reset = sess.reset_count;
        id.epoch = r.odometry_epoch;
        id.anchor = r.anchor_revision;
        id.placed = placed;
        id.placement = r.placement_sequence;
        const t = typeof r.measured_at_host_ms === 'number' ? r.measured_at_host_ms : msg.host_ms;
        const i = this.state.begin(t);
        if (i < 0) {
            return;
        }
        const valid = !!r.valid;
        const f = (placed ? r.field : r.odom) || {};
        this.sx.set(i, valid ? num(f.x_m) : NaN, valid);
        this.sy.set(i, valid ? num(f.y_m) : NaN, valid);
        this.sh.set(i, valid ? num(f.heading_deg) : NaN, valid);
        this.svx.set(i, valid ? num(r.vx_m_s) : NaN, valid);
        this.svy.set(i, valid ? num(r.vy_m_s) : NaN, valid);
        // body frame = odometry-frame velocity rotated by minus the odometry heading
        const oh = r.odom ? num(r.odom.heading_deg) * Math.PI / 180 : NaN;
        const vx = num(r.vx_m_s), vy = num(r.vy_m_s);
        this.svbx.set(i, valid ? Math.cos(oh) * vx + Math.sin(oh) * vy : NaN, valid);
        this.svby.set(i, valid ? -Math.sin(oh) * vx + Math.cos(oh) * vy : NaN, valid);
        this.swz.set(i, valid ? num(r.yaw_rate_deg_s) : NaN, valid);
        const att = r.attitude;
        const measured = attitudeStatus(att) === 'measured';
        this.sroll.set(i, measured ? num(att.roll_deg) : NaN, measured);
        this.spitch.set(i, measured ? num(att.pitch_deg) : NaN, measured);
        this.sage.set(i, num(r.age_ms), valid);
    }

    onTelemetry(msg) {
        const i = this.tel.begin(msg.host_ms);
        if (i < 0) {
            return;
        }
        const m = msg.motion;
        const c = m && m.cmd ? m.cmd : null;
        this.tcvx.set(i, c ? num(c.vx_m_s) : NaN, !!c);
        this.tcvy.set(i, c ? num(c.vy_m_s) : NaN, !!c);
        this.tcw.set(i, c ? num(c.omega_deg_s) : NaN, !!c);
        this.tct.set(i, m ? num(m.cross_track_m) : NaN, !!m);
        this.tde.set(i, m ? num(m.distance_error_m) : NaN, !!m);
        this.the.set(i, m ? num(m.heading_error_deg) : NaN, !!m);
        const a = msg.attitude;
        this.troll.set(i, a ? num(a.roll_deg) : NaN, !!a);
        this.tpitch.set(i, a ? num(a.pitch_deg) : NaN, !!a);
        const rpm = msg.wheels && Array.isArray(msg.wheels.rpm) ? msg.wheels.rpm : null;
        if (rpm) {
            for (let k = 0; k < rpm.length; ++k) {
                this.store.get('tel.rpm.' + k, this.tel, metaFor('tel.rpm.' + k)).set(i, num(rpm[k]), true);
            }
        }
    }

    onDiag(msg) {
        const t = msg.host_ms;
        const i = this.diag.begin(t);
        if (i < 0) {
            return;
        }
        const dt = (t - this.prevDiagT) / 1000;
        const broken = !(dt > 0) || (this.diag.flags[i] & 2) !== 0;
        this.prevDiagT = t;
        const store = this.store;
        const ch = this.diag;
        const link = msg.brain_link;
        for (const w of (link && link.wheels) || []) {
            const k = String(w.port);
            store.get('wheel.counts.' + k, ch, metaFor('wheel.counts.' + k)).set(i, num(w.counts), w.valid !== false);
            store.get('wheel.travel.' + k, ch, metaFor('wheel.travel.' + k)).set(i, num(w.travel_m), w.valid !== false);
            const prev = this.prevWheels.get(k);
            const rate = !broken && prev !== undefined && typeof w.counts === 'number' ? (w.counts - prev) / dt : NaN;
            store.get('wheel.rate.' + k, ch, metaFor('wheel.rate.' + k)).set(i, rate, true);
            this.prevWheels.set(k, w.counts);
        }
        const loc = msg.localization || {};
        for (const f of loc.functions || []) {
            const s = f.stillness;
            if (s && s.monitored) {
                store.get('imu.bias.' + f.id, ch, metaFor('imu.bias.' + f.id)).set(i, num(s.bias_dps), typeof s.bias_dps === 'number');
            }
        }
        for (const s of msg.sources || []) {
            store.get('age.' + s.id, ch, metaFor('age.' + s.id)).set(i, num(s.receipt_age_ms), s.state === 'valid');
        }
        const d = msg.diagnostics;
        if (d) {
            for (const part of [d.estimation, d.field]) {
                for (const l of (part && part.links) || []) {
                    const prev = this.prevLinks.get(l.id);
                    const rate = !broken && prev !== undefined ? (l.packets - prev) / dt : NaN;
                    this.prevLinks.set(l.id, l.packets);
                    store.get('link.rate.' + l.id, ch, metaFor('link.rate.' + l.id)).set(i, rate, true);
                    store.get('link.err.' + l.id + '.decode', ch, metaFor('link.err.' + l.id + ' decode')).set(i, num(l.decode_errors), true);
                    store.get('link.err.' + l.id + '.gaps', ch, metaFor('link.err.' + l.id + ' seq gaps')).set(i, num(l.seq_gaps), true);
                }
            }
        }
        const w = msg.workers || {};
        for (const name of ['estimation', 'field']) {
            if (w[name]) {
                store.get('worker.rate.' + name, ch, metaFor('worker.rate.' + name)).set(i, num(w[name].rate_hz), !!w[name].running);
            }
        }
    }

    onInstrumentation(msg, t) {
        const i = this.inst.begin(t);
        if (i < 0) {
            return;
        }
        const ch = this.inst;
        const pico = msg.pico;
        for (const e of (pico && pico.encoders) || []) {
            const k = String(e.port);
            this.store.get('enc.counts.' + k, ch, metaFor('enc.counts.' + k)).set(i, num(e.counts), true);
        }
        const vex = msg.brain && msg.brain.vex_imu;
        if (vex) {
            this.store.get('imu.vex_rotation', ch, metaFor('imu.vex_rotation')).set(i, num(vex.rotation_deg), vex.valid !== false);
        }
        for (const l of msg.links || []) {
            if (l.rates) {
                this.store.get('link.rate.' + l.id + ' (instr)', ch, metaFor('link.rate.' + l.id + ' (instr)')).set(i, num(l.rates.rx_frames_s), true);
            }
            const reader = l.reader || {};
            const errs = num(l.rejected) + num(reader.sync_dropped) + num(reader.length_errors) + num(reader.check_errors);
            this.store.get('link.err.' + l.id + '.rejected', ch, metaFor('link.err.' + l.id + ' rejected+reader')).set(i, errs, true);
        }
    }
}

// One plot: a header with the title and legend, and a canvas.
class Plot {
    constructor(def, panel) {
        this.def = def;
        this.panel = panel;
        this.root = el('div', 'plot');
        const head = el('div', 'plot-head');
        this.title = el('span', 'plot-title', `${def.title} [${def.unit}]`);
        head.appendChild(this.title);
        this.legend = el('span', 'plot-legend');
        head.appendChild(this.legend);
        this.note = el('div', 'plot-note muted');
        this.canvas = el('canvas', 'plot-canvas');
        this.root.appendChild(head);
        this.root.appendChild(this.canvas);
        this.root.appendChild(this.note);
        this.ctx = this.canvas.getContext('2d');
        this.entries = new Map(); // series id -> {span, value}
        this.w = 0;
        this.h = 0;
        this.cursorX = -1;
        this.lastEventNote = '';
        this.canvas.addEventListener('pointermove', (e) => {
            const r = this.canvas.getBoundingClientRect();
            this.cursorX = e.clientX - r.left;
            panel.dirty = true;
        });
        this.canvas.addEventListener('pointerleave', () => {
            this.cursorX = -1;
            panel.dirty = true;
        });
        this.canvas.addEventListener('click', (e) => {
            const r = this.canvas.getBoundingClientRect();
            panel.onPlotClick(this, e.clientX - r.left);
        });
    }

    seriesList(store) {
        const d = this.def;
        const out = [];
        if (d.ids) {
            for (const id of d.ids) {
                const s = store.series.get(id);
                if (s) {
                    out.push(s);
                }
            }
        }
        if (d.prefix) {
            for (const [id, s] of store.series) {
                if (id.startsWith(d.prefix)) {
                    out.push(s);
                }
            }
        }
        return out;
    }

    size() {
        const w = Math.max(100, this.canvas.clientWidth | 0);
        const h = Math.max(60, this.canvas.clientHeight | 0);
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        if (w !== this.w || h !== this.h || dpr !== this.dpr) {
            this.w = w;
            this.h = h;
            this.dpr = dpr;
            this.canvas.width = Math.round(w * dpr);
            this.canvas.height = Math.round(h * dpr);
        }
    }

    legendEntry(s, color) {
        let e = this.entries.get(s.id);
        if (!e) {
            const span = el('span', 'legend-item');
            const sw = el('span', 'swatch');
            span.appendChild(sw);
            const label = el('span');
            span.appendChild(label);
            const value = el('span', 'legend-value');
            span.appendChild(value);
            this.legend.appendChild(span);
            e = { span, sw, label, value };
            this.entries.set(s.id, e);
        }
        if (e.color !== color) {
            e.color = color;
            e.sw.style.background = color;
        }
        setText(e.label, ` ${s.label}${s.unit && s.unit !== this.def.unit ? ' [' + s.unit + ']' : ''} `);
        setTitle(e.span, `${s.label} [${s.unit || this.def.unit}] from ${s.source}${s.note ? '; ' + s.note : ''}`);
        return e;
    }

    // Draws the window [t0, t1]; cursorT/playT in the same clock.
    draw(store, t0, t1, opts) {
        this.size();
        const ctx = this.ctx;
        const dpr = this.dpr;
        const W = this.w, H = this.h;
        const padL = 46, padR = 6, padT = 4, padB = 16;
        const cols = Math.max(10, Math.floor(W - padL - padR));
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.fillStyle = '#161a20';
        ctx.fillRect(0, 0, W, H);
        const list = this.seriesList(store);
        const dec = this.panel.decimators;
        while (dec.length < list.length) {
            dec.push(new Decimator());
        }
        const wrapMode = this.def.wrap && !opts.unwrap;
        let lo = Infinity, hi = -Infinity;
        const got = [];
        for (let k = 0; k < list.length; ++k) {
            const r = dec[k].run(list[k], t0, t1, cols);
            got.push(r);
            if (r) {
                lo = Math.min(lo, r.lo);
                hi = Math.max(hi, r.hi);
            }
        }
        if (wrapMode) {
            lo = -180;
            hi = 180;
        } else if (lo === Infinity) {
            lo = -1;
            hi = 1;
        } else if (hi - lo < 1e-9) {
            const m = Math.max(Math.abs(lo) * 0.05, 1e-3);
            lo -= m;
            hi += m;
        } else {
            const m = (hi - lo) * 0.06;
            lo -= m;
            hi += m;
        }
        const plotH = H - padT - padB;
        const y = (v) => padT + (1 - (v - lo) / (hi - lo)) * plotH;
        const x = (col) => padL + col + 0.5;
        const xt = (t) => padL + ((t - t0) / (t1 - t0)) * cols;
        // grid and y labels
        ctx.strokeStyle = '#262c35';
        ctx.fillStyle = '#8a93a2';
        ctx.font = '10px system-ui, sans-serif';
        ctx.lineWidth = 1;
        ctx.textBaseline = 'middle';
        const ticks = niceTicks(lo, hi, 4);
        ctx.beginPath();
        for (const v of ticks) {
            const py = Math.round(y(v)) + 0.5;
            ctx.moveTo(padL, py);
            ctx.lineTo(W - padR, py);
        }
        ctx.stroke();
        for (const v of ticks) {
            ctx.fillText(tickLabel(v, ticks), 2, y(v));
        }
        // x labels
        ctx.textBaseline = 'alphabetic';
        const span = t1 - t0;
        const step = niceStep(span / 6);
        ctx.beginPath();
        for (let tt = Math.ceil(t0 / step) * step; tt <= t1; tt += step) {
            const px = Math.round(xt(tt)) + 0.5;
            ctx.moveTo(px, padT);
            ctx.lineTo(px, H - padB);
        }
        ctx.stroke();
        for (let tt = Math.ceil(t0 / step) * step; tt <= t1; tt += step) {
            ctx.fillText(opts.timeLabel(tt), xt(tt) + 2, H - 4);
        }
        // invalid spans: a red band along the bottom of the plot
        ctx.fillStyle = 'rgba(255, 82, 82, 0.55)';
        for (let k = 0; k < list.length; ++k) {
            if (!got[k]) {
                continue;
            }
            const d = dec[k];
            for (let col = 0; col < cols; ++col) {
                if (d.bad[col]) {
                    ctx.fillRect(x(col) - 0.5, H - padB - 3, 1, 3);
                }
            }
        }
        // events
        const events = opts.events || [];
        ctx.strokeStyle = 'rgba(255, 183, 77, 0.6)';
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        let nearEvent = null;
        const lb = lowerBoundEvents(events, t0);
        for (let k = lb; k < events.length && events[k].t <= t1; ++k) {
            const px = Math.round(xt(events[k].t)) + 0.5;
            ctx.moveTo(px, padT);
            ctx.lineTo(px, H - padB);
            if (this.cursorX >= 0 && Math.abs(this.cursorX - px) <= 4) {
                nearEvent = events[k];
            }
        }
        ctx.stroke();
        ctx.setLineDash([]);
        // series
        ctx.lineWidth = 1.25;
        for (let k = 0; k < list.length; ++k) {
            const color = COLORS[k % COLORS.length];
            this.legendEntry(list[k], color);
            if (!got[k]) {
                continue;
            }
            ctx.strokeStyle = color;
            drawDecimated(ctx, dec[k], cols, x, y, wrapMode);
        }
        // playhead and cursor
        if (typeof opts.playT === 'number' && opts.playT >= t0 && opts.playT <= t1) {
            ctx.strokeStyle = '#ff5252';
            ctx.beginPath();
            const px = Math.round(xt(opts.playT)) + 0.5;
            ctx.moveTo(px, padT);
            ctx.lineTo(px, H - padB);
            ctx.stroke();
        }
        let cursorT = NaN;
        if (this.cursorX >= padL && this.cursorX <= W - padR) {
            cursorT = t0 + ((this.cursorX - padL) / cols) * (t1 - t0);
            ctx.strokeStyle = 'rgba(230, 233, 239, 0.5)';
            ctx.beginPath();
            ctx.moveTo(Math.round(this.cursorX) + 0.5, padT);
            ctx.lineTo(Math.round(this.cursorX) + 0.5, H - padB);
            ctx.stroke();
        }
        // legend values: at the cursor or the playhead (n/a inside a gap),
        // else the series' newest sample, whatever its age
        const pinned = Number.isFinite(cursorT) ? cursorT : (typeof opts.playT === 'number' ? opts.playT : NaN);
        const tmp = this.panel.tmp;
        const seen = new Set();
        for (const s of list) {
            seen.add(s.id);
            const e = this.entries.get(s.id);
            const at = Number.isFinite(pinned) ? pinned : s.channel.newest();
            const v = valueAt(s, at, tmp);
            let text = 'n/a';
            if (v && !Number.isNaN(v.v) && at - v.t <= Math.max(s.channel.gapMs, 1)) {
                text = fmtValue(v.v) + (v.valid ? '' : ' (invalid)');
            } else if (v && !v.valid) {
                text = 'invalid';
            }
            setText(e.value, text);
        }
        for (const [id, e] of this.entries) {
            setHidden(e.span, !seen.has(id));
        }
        const note = nearEvent ? `event: ${nearEvent.text}` :
            (list.length === 0 ? 'no data for this plot from this source' : '');
        setText(this.note, note);
        this.cursorT = cursorT;
        this.window = [t0, t1, padL, cols];
    }

    timeAt(px) {
        if (!this.window) {
            return NaN;
        }
        const [t0, t1, padL, cols] = this.window;
        return t0 + ((px - padL) / cols) * (t1 - t0);
    }
}

function lowerBoundEvents(events, t) {
    let lo = 0, hi = events.length;
    while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (events[mid].t < t) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

function fmtValue(v) {
    const a = Math.abs(v);
    if (a >= 1000) {
        return v.toFixed(0);
    }
    if (a >= 10) {
        return v.toFixed(2);
    }
    return v.toFixed(3);
}

function niceStep(raw) {
    const p = Math.pow(10, Math.floor(Math.log10(Math.max(raw, 1e-9))));
    const m = raw / p;
    return (m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10) * p;
}

function niceTicks(lo, hi, n) {
    const step = niceStep((hi - lo) / n);
    const out = [];
    for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) {
        out.push(Math.abs(v) < step * 1e-6 ? 0 : v);
    }
    return out;
}

function tickLabel(v, ticks) {
    const step = ticks.length > 1 ? Math.abs(ticks[1] - ticks[0]) : 1;
    const digits = step >= 1 ? 0 : Math.min(4, Math.ceil(-Math.log10(step)));
    return v.toFixed(digits);
}

// The graphs tab: plot picker, window options and the plot stack.
export class GraphPanel {
    constructor(root, opts) {
        this.root = root;
        this.onSeek = opts.onSeek || (() => {});
        this.picker = root.querySelector('#graph-picker');
        this.stack = root.querySelector('#graph-stack');
        this.spanSelect = root.querySelector('#graph-span');
        this.unwrapBox = root.querySelector('#graph-unwrap');
        this.freezeBtn = root.querySelector('#graph-freeze');
        this.columnSelect = root.querySelector('#graph-column');
        this.status = root.querySelector('#graph-status');
        this.decimators = [];
        this.tmp = {};
        this.plots = new Map();   // plot id -> Plot
        this.defs = new Map(PLOTS.map((d) => [d.id, d]));
        this.selected = new Set(loadSelection());
        this.frozen = false;
        this.frozenT1 = 0;
        this.dirty = true;
        this.lastDraw = 0;
        this.store = null;
        this.mode = 'live';
        this.buildPicker();
        this.spanSelect.addEventListener('change', () => {
            this.dirty = true;
        });
        this.unwrapBox.addEventListener('change', () => {
            this.dirty = true;
        });
        this.freezeBtn.addEventListener('click', () => this.setFrozen(!this.frozen));
        this.columnSelect.addEventListener('change', () => {
            const v = this.columnSelect.value;
            if (v) {
                this.addColumnPlot(v);
                this.columnSelect.value = '';
            }
        });
    }

    buildPicker() {
        this.picker.textContent = '';
        this.checks = new Map();
        for (const d of this.defs.values()) {
            const label = el('label', 'pick');
            const box = el('input');
            box.type = 'checkbox';
            box.checked = this.selected.has(d.id);
            box.addEventListener('change', () => {
                if (box.checked) {
                    this.selected.add(d.id);
                } else {
                    this.selected.delete(d.id);
                }
                saveSelection(this.selected);
                this.syncPlots();
            });
            label.appendChild(box);
            label.appendChild(el('span', undefined, ` ${d.title}`));
            this.picker.appendChild(label);
            this.checks.set(d.id, box);
        }
        this.syncPlots();
    }

    syncPlots() {
        for (const [id, p] of this.plots) {
            if (!this.selected.has(id)) {
                p.root.remove();
                this.plots.delete(id);
            }
        }
        for (const d of this.defs.values()) {
            if (this.selected.has(d.id) && !this.plots.has(d.id)) {
                this.plots.set(d.id, new Plot(d, this));
            }
        }
        // keep the picker order
        for (const d of this.defs.values()) {
            const p = this.plots.get(d.id);
            if (p) {
                this.stack.appendChild(p.root);
            }
        }
        this.dirty = true;
    }

    // Replay: every numeric column of every CSV can be plotted by itself.
    setColumns(columns) {
        this.columnSelect.textContent = '';
        const first = el('option', undefined, columns.length ? 'plot a capture column...' : 'no capture loaded');
        first.value = '';
        this.columnSelect.appendChild(first);
        for (const c of columns) {
            const o = el('option', undefined, c.label);
            o.value = c.id;
            this.columnSelect.appendChild(o);
        }
        this.columnSelect.disabled = columns.length === 0;
    }

    addColumnPlot(seriesId) {
        const id = 'col:' + seriesId;
        if (!this.defs.has(id)) {
            const s = this.store ? this.store.series.get(seriesId) : null;
            this.defs.set(id, { id, title: s ? `${s.source} ${s.label}` : seriesId, unit: s ? s.unit : '', ids: [seriesId], custom: true });
        }
        this.selected.add(id);
        this.buildPicker();
    }

    removeColumnPlots() {
        let changed = false;
        for (const [id, d] of this.defs) {
            if (d.custom) {
                this.defs.delete(id);
                this.selected.delete(id);
                changed = true;
            }
        }
        if (changed) {
            this.buildPicker();
        }
    }

    setFrozen(on) {
        this.frozen = on;
        this.frozenT1 = this.lastT1 || 0;
        this.freezeBtn.classList.toggle('active', on);
        this.freezeBtn.textContent = on ? 'resume' : 'hold';
        this.dirty = true;
    }

    setSource(store, mode) {
        this.store = store;
        this.mode = mode;
        this.dirty = true;
        this.freezeBtn.disabled = mode === 'replay';
        if (mode === 'replay' && this.frozen) {
            this.setFrozen(false);
        }
    }

    onPlotClick(plot, px) {
        const t = plot.timeAt(px);
        if (!Number.isFinite(t)) {
            return;
        }
        if (this.mode === 'replay') {
            this.onSeek(t);
        } else {
            this.setFrozen(!this.frozen);
        }
    }

    spanMs() {
        return Number(this.spanSelect.value || 60) * 1000;
    }

    // opts: {nowT (live right edge, Pi ms), playT (replay), events [{t, text}], timeLabel}
    draw(nowMs, opts) {
        if (!this.store) {
            return;
        }
        if (nowMs - this.lastDraw < 1000 / GRAPH_HZ && !this.dirty) {
            return;
        }
        this.lastDraw = nowMs;
        this.dirty = false;
        const span = this.spanMs();
        let t1;
        if (this.mode === 'replay') {
            t1 = Math.min(opts.endT, Math.max(opts.startT + span, opts.playT + span * 0.25));
        } else {
            t1 = this.frozen ? this.frozenT1 : opts.nowT;
        }
        if (!Number.isFinite(t1)) {
            setText(this.status, 'waiting for data');
            return;
        }
        this.lastT1 = t1;
        const t0 = t1 - span;
        const drawOpts = {
            unwrap: this.unwrapBox.checked, events: opts.events, playT: opts.playT, timeLabel: opts.timeLabel,
        };
        for (const p of this.plots.values()) {
            p.draw(this.store, t0, t1, drawOpts);
        }
        let dropped = 0;
        for (const c of this.store.channels.values()) {
            dropped += c.outOfOrder;
        }
        setText(this.status, (this.mode === 'replay' ? 'replay data, ' : (this.frozen ? 'held, ' : 'live, ')) +
            `window ${span / 1000} s, Pi host clock` + (dropped ? `, ${dropped} samples older than the newest dropped` : ''));
    }
}

function loadSelection() {
    try {
        const v = JSON.parse(localStorage.getItem('spectagatr.graphs') || 'null');
        if (Array.isArray(v)) {
            return v.filter((id) => PLOTS.some((p) => p.id === id));
        }
    } catch (e) {
        // storage can be unavailable; defaults then
    }
    return DEFAULT_PLOTS;
}

function saveSelection(set) {
    try {
        localStorage.setItem('spectagatr.graphs', JSON.stringify([...set].filter((id) => PLOTS.some((p) => p.id === id))));
    } catch (e) {
        // per-viewer convenience only
    }
}
