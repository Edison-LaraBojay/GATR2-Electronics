// replay.js
// Replay of a capture bundle (gatr2.capture/1): the CSVs become a shared
// timeline on the Pi host clock (pi_host_us). At a playhead time the model
// answers what was recorded at or before it: the robot state row, the
// trail of the same segment, the newest path and telemetry, the events.
// Rows of different streams are never merged onto one grid, and a trail
// never crosses a session, reset, odometry epoch or Pico boot boundary.
//
// Replay is read-only: it holds no socket, sends nothing and changes no
// live state. The page shows it behind a REPLAY banner.

import { readZip, CsvTable, unitOf } from './bundle.js';
import { SeriesStore } from './series.js';

// Column names per gatr2.capture/1 (docs/capture.md, the bundle's
// README.txt); the aliases tolerate small naming differences.
const COL = {
    t: ['pi_host_us'],
    session: ['pi_session', 'session'],
    reset: ['reset_count'],
    epoch: ['odometry_epoch'],
    anchor: ['anchor_revision'],
    placement: ['placement_sequence'],
    valid: ['valid'],
    placed: ['placed', 'initialized'],
    fx: ['field_x_m'],
    fy: ['field_y_m'],
    fh: ['field_heading_deg'],
    ox: ['odom_x_m'],
    oy: ['odom_y_m'],
    oh: ['odom_heading_deg'],
    vx: ['odom_vx_m_s', 'vx_m_s'],
    vy: ['odom_vy_m_s', 'vy_m_s'],
    wz: ['yaw_rate_deg_s'],
    roll: ['roll_deg'],
    pitch: ['pitch_deg'],
    attValid: ['attitude_valid'],
    attLevel: ['attitude_assumed_level'],
    attStatus: ['attitude_status'],
    confidence: ['confidence'],
    age: ['source_age_ms'],
    measuredHost: ['measured_pi_host_ms', 'measured_host_ms'],
    sourceClock: ['source_clock'],
    sourceMs: ['source_ms'],
};

// Enum names for recorded telemetry, as the live server names them
// (inspection_document.cpp; actuGATR motion.h and drive.h, investiGATR
// path.h). An unknown value stays a number.
const MOTION_STATE = ['idle', 'waiting', 'running', 'settling', 'completed', 'cancelled', 'failed'];
const MOTION_REASON = ['none', 'invalid_command', 'invalid_config', 'input_unavailable', 'no_profile', 'calibrating',
    'placement_required', 'input_lost', 'frame_changed', 'field_unavailable', 'map_mismatch', 'unknown_reference',
    'not_reference', 'reference_unavailable', 'unsupported_model', 'start_out_of_bounds', 'start_blocked',
    'goal_out_of_bounds', 'goal_blocked', 'no_path', 'tracking_error', 'plan_limit', 'timed_out', 'source_changed',
    'cancelled_by_caller'];
const PLAN_MODE = ['direct', 'avoiding'];
const DRIVE_FAULT = ['none', 'wrong_frame', 'non_finite', 'unsupported_motion', 'stale'];

function nameOf(table, v) {
    return Number.isInteger(v) && v >= 0 && v < table.length ? table[v] : undefined;
}

function pick(table, key) {
    return table ? table.pick(...COL[key]) : null;
}

function upperIndex(times, t) {
    // last index with times[i] <= t, or -1
    let lo = 0, hi = times.length;
    while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (times[mid] <= t) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo - 1;
}

// Rows sorted by time: the index order and the times in ms.
function sortedTimes(table) {
    const us = table.num('pi_host_us');
    if (!us) {
        return null;
    }
    const order = Array.from({ length: table.n }, (_, i) => i).filter((i) => Number.isFinite(us[i]));
    order.sort((a, b) => us[a] - us[b]);
    const t = new Float64Array(order.length);
    for (let k = 0; k < order.length; ++k) {
        t[k] = us[order[k]] / 1000;
    }
    return { order, t };
}

export class ReplayModel {
    // entries: Map name -> Uint8Array (from readZip); label: where it came from.
    constructor(entries, label) {
        this.label = label;
        this.entries = entries;
        this.tables = new Map();
        this.warnings = [];
        const decoder = new TextDecoder();
        const meta = entries.get('metadata.json');
        this.metadata = null;
        if (meta) {
            try {
                this.metadata = JSON.parse(decoder.decode(meta));
            } catch (e) {
                this.warnings.push('metadata.json is not valid JSON: ' + e.message);
            }
        } else {
            this.warnings.push('no metadata.json in the bundle');
        }
        for (const [name, data] of entries) {
            if (name.endsWith('.csv')) {
                this.tables.set(name.slice(0, -4), new CsvTable(name, decoder.decode(data)));
            }
        }
        if (this.metadata && this.metadata.schema && this.metadata.schema !== 'gatr2.capture/1') {
            this.warnings.push(`schema ${this.metadata.schema}, this viewer reads gatr2.capture/1`);
        }
        this.buildTimes();
        this.buildRobot();
        this.buildTelemetry();
        this.buildPaths();
        this.buildEvents();
        this.buildStore();
    }

    static async fromZip(buffer, label) {
        return new ReplayModel(await readZip(buffer), label);
    }

    buildTimes() {
        let t0 = Infinity, t1 = -Infinity;
        for (const table of this.tables.values()) {
            const us = table.num('pi_host_us');
            if (!us) {
                continue;
            }
            for (let i = 0; i < us.length; ++i) {
                if (Number.isFinite(us[i])) {
                    t0 = Math.min(t0, us[i] / 1000);
                    t1 = Math.max(t1, us[i] / 1000);
                }
            }
        }
        if (t0 === Infinity) {
            throw new Error('the bundle has no rows with pi_host_us');
        }
        this.startT = t0;
        this.endT = Math.max(t1, t0 + 1);
    }

    buildRobot() {
        const tb = this.tables.get('robot_state');
        this.robot = null;
        if (!tb || !tb.n) {
            this.warnings.push('no robot_state.csv rows: no 3D robot in replay');
            return;
        }
        const s = sortedTimes(tb);
        const col = (key) => {
            const name = pick(tb, key);
            return name ? tb.num(name) : null;
        };
        const sessions = tb.str(pick(tb, 'session'));
        const r = {
            order: s.order, t: s.t, sessions,
            reset: col('reset'), epoch: col('epoch'), anchor: col('anchor'), placement: col('placement'),
            valid: col('valid'), placed: col('placed'),
            fx: col('fx'), fy: col('fy'), fh: col('fh'), ox: col('ox'), oy: col('oy'), oh: col('oh'),
            vx: col('vx'), vy: col('vy'), wz: col('wz'), roll: col('roll'), pitch: col('pitch'),
            attValid: col('attValid'), attLevel: col('attLevel'), confidence: col('confidence'),
            measuredHost: col('measuredHost'), age: col('age'),
            sourceMs: col('sourceMs'), sourceClock: tb.str(pick(tb, 'sourceClock')),
            attStatus: tb.str(pick(tb, 'attStatus')),
        };
        if (!(r.fx && r.fy && r.fh) && !(r.ox && r.oy && r.oh)) {
            this.warnings.push('robot_state.csv has neither field_* nor odom_* pose columns: no 3D robot in replay');
            return;
        }
        // segment id per sorted row: a new one at every identity change
        const seg = new Int32Array(s.order.length);
        let id = 0;
        let prevKey = null;
        for (let k = 0; k < s.order.length; ++k) {
            const i = s.order[k];
            const key = `${sessions ? sessions[i] : ''}/${r.reset ? r.reset[i] : ''}/${r.epoch ? r.epoch[i] : ''}`;
            if (prevKey !== null && key !== prevKey) {
                id += 1;
            }
            prevKey = key;
            seg[k] = id;
        }
        r.segment = seg;
        r.segments = id + 1;
        this.robot = r;
    }

    buildTelemetry() {
        const tb = this.tables.get('brain_telemetry');
        this.telemetry = null;
        if (!tb || !tb.n) {
            return;
        }
        const s = sortedTimes(tb);
        const n = (...names) => tb.num(tb.pick(...names));
        const wheels = [];
        for (let k = 0; k < 8; ++k) {
            const name = tb.pick(`wheel${k}_rpm`, `wheel_${k}_rpm`);
            if (name) {
                wheels.push(tb.num(name));
            }
        }
        this.telemetry = {
            order: s.order, t: s.t,
            command: n('command_id'), state: n('motion_state'), stateName: tb.str(tb.pick('motion_state_name')),
            reason: n('motion_reason'), mode: n('plan_mode'),
            segment: n('path_segment', 'segment'), segmentCount: n('path_segment_count', 'segment_count'),
            tx: n('target_field_x_m', 'target_x_m'), ty: n('target_field_y_m', 'target_y_m'),
            th: n('target_field_heading_deg', 'target_heading_deg'),
            cvx: n('cmd_body_vx_m_s', 'cmd_vx_m_s'), cvy: n('cmd_body_vy_m_s', 'cmd_vy_m_s'), cw: n('cmd_omega_deg_s'),
            ct: n('cross_track_m'), de: n('distance_error_m'), he: n('heading_error_deg'),
            fault: n('drive_fault'), roll: n('roll_deg'), pitch: n('pitch_deg'), wheels,
            motionPresent: n('motion_present'), attitudePresent: n('attitude_present'),
        };
    }

    // paths.csv: one row per point, grouped by (pi_host_us, command_id).
    buildPaths() {
        const tb = this.tables.get('paths');
        this.paths = [];
        if (!tb || !tb.n) {
            return;
        }
        const us = tb.num('pi_host_us');
        const cmd = tb.num(tb.pick('command_id'));
        const idx = tb.num(tb.pick('point_index', 'index'));
        const x = tb.num(tb.pick('field_x_m', 'x_m'));
        const y = tb.num(tb.pick('field_y_m', 'y_m'));
        const mode = tb.str(tb.pick('mode_name', 'mode'));
        const session = tb.str(tb.pick('brain_session', 'session'));
        if (!us || !x || !y) {
            this.warnings.push('paths.csv lacks pi_host_us/field_x_m/field_y_m: paths not replayed');
            return;
        }
        const groups = new Map();
        for (let i = 0; i < tb.n; ++i) {
            if (!Number.isFinite(us[i])) {
                continue;
            }
            const key = `${us[i]}/${cmd ? cmd[i] : ''}`;
            let g = groups.get(key);
            if (!g) {
                const m = mode ? mode[i] : null;
                g = {
                    t: us[i] / 1000, command_id: cmd ? cmd[i] : 0, session: session ? session[i] : '',
                    mode: m === '2' ? 'avoiding' : m === '1' ? 'direct' : (m || 'unknown'), rows: [],
                };
                groups.set(key, g);
            }
            // an empty report is one row with empty point columns
            if (Number.isFinite(x[i]) && Number.isFinite(y[i])) {
                g.rows.push({ k: idx ? idx[i] : g.rows.length, x_m: x[i], y_m: y[i] });
            }
        }
        for (const g of groups.values()) {
            g.rows.sort((a, b) => a.k - b.k);
            g.points = g.rows.map((r) => ({ x_m: r.x_m, y_m: r.y_m }));
            g.received_host_ms = g.t;
            delete g.rows;
            this.paths.push(g);
        }
        this.paths.sort((a, b) => a.t - b.t);
        this.pathT = Float64Array.from(this.paths.map((p) => p.t));
    }

    buildEvents() {
        const tb = this.tables.get('events');
        this.events = [];
        if (tb && tb.n) {
            const us = tb.num('pi_host_us');
            const text = tb.str(tb.pick('text', 'event', 'message'));
            for (let i = 0; i < tb.n; ++i) {
                if (us && Number.isFinite(us[i])) {
                    this.events.push({ t: us[i] / 1000, text: text ? text[i] || '' : '' });
                }
            }
        }
        // segment starts are markers too: the viewer never joins across them
        const segs = this.metadata && Array.isArray(this.metadata.segments) ? this.metadata.segments : [];
        for (const s of segs) {
            if (typeof s.first_pi_host_us === 'number' && s.index > 0) {
                const keys = Object.entries(s).filter(([k]) => !/pi_host_us|^rows$|^index$|^domain$|^source$/.test(k))
                    .map(([k, v]) => `${k} ${v}`).join(', ');
                this.events.push({ t: s.first_pi_host_us / 1000, text: `segment: ${s.domain}${s.source ? ' ' + s.source : ''} #${s.index} (${keys})` });
            }
        }
        this.events.sort((a, b) => a.t - b.t);
    }

    // Graph data: known columns under the live series ids, and every
    // numeric column of every CSV as csv.<stream>.<column>.
    buildStore() {
        const store = new SeriesStore();
        this.store = store;
        this.columns = [];
        for (const [name, tb] of this.tables) {
            const s = sortedTimes(tb);
            if (!s || !s.t.length) {
                continue;
            }
            const ch = store.channel(name, s.t.length, 0);
            const slots = new Int32Array(s.order.length);
            // equal times (one path's points) keep only the first slot
            let last = -Infinity;
            for (let k = 0; k < s.order.length; ++k) {
                slots[k] = s.t[k] > last ? ch.begin(s.t[k]) : -1;
                last = Math.max(last, s.t[k]);
            }
            // breaks at identity changes of the stream's own identifiers
            const idCols = ['pi_session', 'reset_count', 'odometry_epoch', 'anchor_revision', 'placed', 'boot_id', 'acq_epoch', 'imu_epoch', 'brain_session']
                .filter((c) => tb.has(c)).map((c) => tb.str(c));
            let prev = null;
            for (let k = 0; k < s.order.length; ++k) {
                const i = s.order[k];
                const key = idCols.map((a) => a[i]).join('/');
                if (prev !== null && key !== prev && slots[k] >= 0) {
                    ch.flags[slots[k]] |= 2;
                }
                prev = key;
            }
            for (const c of tb.numericColumns()) {
                if (c === 'pi_host_us') {
                    continue;
                }
                const id = `csv.${name}.${c}`;
                const series = store.get(id, ch, { label: c, unit: unitOf(c), source: name + '.csv', wrap: /heading_deg$|yaw_deg$/.test(c) && !/unwrapped/.test(c) });
                const v = tb.num(c);
                for (let k = 0; k < s.order.length; ++k) {
                    series.set(slots[k], v[s.order[k]], true);
                }
                this.columns.push({ id, label: `${name}.csv: ${c}${unitOf(c) ? ' [' + unitOf(c) + ']' : ''}` });
            }
            this.aliasKnown(name, tb, ch, s, slots);
        }
    }

    // Known columns under the live plot ids, so the standard plots work.
    aliasKnown(name, tb, ch, s, slots) {
        const map = [];
        if (name === 'robot_state') {
            const valid = tb.num(pick(tb, 'valid'));
            const placed = tb.num(pick(tb, 'placed'));
            const att = tb.num(pick(tb, 'attValid'));
            const fx = tb.num(pick(tb, 'fx')), fy = tb.num(pick(tb, 'fy')), fh = tb.num(pick(tb, 'fh'));
            const ox = tb.num(pick(tb, 'ox')), oy = tb.num(pick(tb, 'oy')), oh = tb.num(pick(tb, 'oh'));
            // the live plots show the field pose once placed, odometry before
            const choose = (f, o) => (i) => (f && Number.isFinite(f[i]) ? f[i] : (o ? o[i] : NaN));
            map.push(['robot.x', choose(fx, ox), valid, { label: 'x', unit: 'm', source: 'robot_state.csv field (odom before placement)' }]);
            map.push(['robot.y', choose(fy, oy), valid, { label: 'y', unit: 'm', source: 'robot_state.csv field (odom before placement)' }]);
            map.push(['robot.heading', choose(fh, oh), valid, { label: 'heading', unit: 'deg', source: 'robot_state.csv', wrap: true }]);
            const col = (key) => tb.num(pick(tb, key));
            const vx = col('vx'), vy = col('vy'), wz = col('wz'), roll = col('roll'), pitch = col('pitch');
            const status = tb.str(pick(tb, 'attStatus'));
            const measured = (i) => (status ? status[i] === 'measured' : !!(att && att[i] === 1));
            map.push(['robot.vx', (i) => (vx ? vx[i] : NaN), valid, { label: 'vx (odometry frame)', unit: 'm/s', source: 'robot_state.csv odom_vx_m_s' }]);
            map.push(['robot.vy', (i) => (vy ? vy[i] : NaN), valid, { label: 'vy (odometry frame)', unit: 'm/s', source: 'robot_state.csv odom_vy_m_s' }]);
            // body frame = odometry-frame velocity rotated by minus the odometry heading
            const ohd = col('oh');
            const body = (i, axis) => {
                if (!vx || !vy || !ohd) {
                    return NaN;
                }
                const h = ohd[i] * Math.PI / 180;
                return axis === 0 ? Math.cos(h) * vx[i] + Math.sin(h) * vy[i] : -Math.sin(h) * vx[i] + Math.cos(h) * vy[i];
            };
            const bodyMeta = (l) => ({ label: l, unit: 'm/s', source: 'robot_state.csv odom_v*_m_s rotated by odom_heading_deg of the same row' });
            map.push(['robot.vbx', (i) => body(i, 0), valid, bodyMeta('vx actual (body)')]);
            map.push(['robot.vby', (i) => body(i, 1), valid, bodyMeta('vy actual (body)')]);
            map.push(['robot.yaw_rate', (i) => (wz ? wz[i] : NaN), valid, { label: 'yaw rate actual', unit: 'deg/s', source: 'robot_state.csv' }]);
            map.push(['robot.roll', (i) => (measured(i) && roll ? roll[i] : NaN), null, { label: 'roll (Pi attitude)', unit: 'deg', source: 'robot_state.csv, measured only' }]);
            map.push(['robot.pitch', (i) => (measured(i) && pitch ? pitch[i] : NaN), null, { label: 'pitch (Pi attitude)', unit: 'deg', source: 'robot_state.csv, measured only' }]);
            const age = col('age');
            if (age) {
                map.push(['age.robot', (i) => age[i], valid, { label: 'robot pose age at publish', unit: 'ms', source: 'robot_state.csv source_age_ms' }]);
            }
        } else if (name === 'brain_telemetry') {
            const n = (c) => tb.num(tb.pick(c));
            const pairs = [
                ['tel.cmd_vx', 'cmd_body_vx_m_s', 'vx commanded', 'm/s'], ['tel.cmd_vy', 'cmd_body_vy_m_s', 'vy commanded', 'm/s'],
                ['tel.cmd_omega', 'cmd_omega_deg_s', 'omega commanded', 'deg/s'], ['tel.cross_track', 'cross_track_m', 'cross-track', 'm'],
                ['tel.distance_error', 'distance_error_m', 'distance to destination', 'm'],
                ['tel.heading_error', 'heading_error_deg', 'heading error', 'deg'],
                ['tel.roll', 'roll_deg', 'roll (Brain VEX)', 'deg'], ['tel.pitch', 'pitch_deg', 'pitch (Brain VEX)', 'deg'],
            ];
            for (const [id, c, label, unit] of pairs) {
                const a = n(c) || n(c.replace('cmd_body_', 'cmd_'));
                if (a) {
                    map.push([id, (i) => a[i], null, { label, unit, source: 'brain_telemetry.csv', wrap: id === 'tel.heading_error' }]);
                }
            }
            (this.telemetry ? this.telemetry.wheels : []).forEach((a, k) => {
                map.push([`tel.rpm.${k}`, (i) => a[i], null, { label: `wheel ${k} rpm target`, unit: 'rpm', source: 'brain_telemetry.csv' }]);
            });
        } else if (name === 'pico_sensor') {
            for (let k = 0; k < 3; ++k) {
                const c = tb.pick(`enc${k}_counts`, `enc_${k}_counts`, `enc${k}`, `encoder${k}_counts`);
                if (c) {
                    const a = tb.num(c);
                    map.push([`enc.counts.${k}`, (i) => a[i], null, { label: `encoder ${k}`, unit: 'counts', source: 'pico_sensor.csv' }]);
                }
            }
        }
        for (const [id, get, valid, meta] of map) {
            const series = this.store.get(id, ch, meta);
            for (let k = 0; k < s.order.length; ++k) {
                const i = s.order[k];
                series.set(slots[k], get(i), valid ? valid[i] !== 0 : true);
            }
        }
    }

    // --- at a playhead time (ms, Pi host clock) ---

    robotIndex(t) {
        return this.robot ? upperIndex(this.robot.t, t) : -1;
    }

    // A state.robot-shaped object for the scene, or null.
    robotAt(t) {
        const r = this.robot;
        const k = this.robotIndex(t);
        if (!r || k < 0) {
            return null;
        }
        const i = r.order[k];
        const v = (a) => (a ? a[i] : NaN);
        const flag = (a, dflt) => (a && Number.isFinite(a[i]) ? a[i] !== 0 : dflt);
        const valid = flag(r.valid, true);
        const placed = flag(r.placed, true);
        const attValid = flag(r.attValid, false);
        const status = r.attStatus && r.attStatus[i] ? r.attStatus[i]
            : (attValid ? 'measured' : (flag(r.attLevel, false) ? 'assumed_level' : 'unavailable'));
        const hasOdom = r.ox && r.oy && r.oh && Number.isFinite(v(r.ox));
        const odom = hasOdom ? { x_m: v(r.ox), y_m: v(r.oy), heading_deg: v(r.oh) } : null;
        // field_* is empty while not placed; the robot is then drawn at its
        // odometry pose, as the live view does
        const hasField = !!(r.fx && r.fy && r.fh) && Number.isFinite(v(r.fx));
        const field = hasField ? { x_m: v(r.fx), y_m: v(r.fy), heading_deg: v(r.fh) } : odom;
        const poseValid = valid && !!field && Number.isFinite(field.x_m);
        return {
            index: k, t: r.t[k], segment: r.segment[k],
            valid: poseValid, initialized: placed && hasField, field: field || { x_m: NaN, y_m: NaN, heading_deg: NaN }, odom,
            odometry_epoch: v(r.epoch), anchor_revision: v(r.anchor), placement_sequence: v(r.placement),
            vx_m_s: v(r.vx), vy_m_s: v(r.vy), yaw_rate_deg_s: v(r.wz), confidence: v(r.confidence),
            // empty in a row = no measurement time; a bundle without the
            // columns says nothing about it (age_known false)
            age_known: !!(r.age || r.measuredHost),
            age_ms: Number.isFinite(v(r.age)) ? v(r.age) : (Number.isFinite(v(r.measuredHost)) ? r.t[k] - v(r.measuredHost) : NaN),
            // the source-clock stamp, as the live measured_at: set with an
            // empty Pi time means measured, clock not mapped
            measured_at: { clock: r.sourceClock ? r.sourceClock[i] : null, ms: Number.isFinite(v(r.sourceMs)) ? v(r.sourceMs) : null },
            attitude: { status, valid: status === 'measured', assumed_level: status === 'assumed_level', roll_deg: v(r.roll), pitch_deg: v(r.pitch), source: 'recorded' },
            placement_origin: 'recorded', session: r.sessions ? r.sessions[i] : '',
        };
    }

    // field_from_odom of a row: field pose composed with the inverse odom pose.
    anchorOf(robot) {
        if (!robot || !robot.odom || !robot.initialized) {
            return null;   // not placed: odometry drawn as is
        }
        const f = robot.field;
        const o = robot.odom;
        const h = (f.heading_deg - o.heading_deg) * Math.PI / 180;
        const c = Math.cos(h), s = Math.sin(h);
        return { x_m: f.x_m - (c * o.x_m - s * o.y_m), y_m: f.y_m - (s * o.x_m + c * o.y_m), heading_deg: f.heading_deg - o.heading_deg };
    }

    // Trail points of the playhead's segment up to row k (newest max),
    // in the odometry frame when the bundle has it, else in the field frame.
    trailPoints(k, max) {
        const r = this.robot;
        if (!r || k < 0) {
            return { points: [], odom: false };
        }
        const seg = r.segment[k];
        let a = k;
        while (a > 0 && r.segment[a - 1] === seg && k - a < max - 1) {
            a -= 1;
        }
        const odom = !!(r.ox && r.oy);
        const points = [];
        for (let j = a; j <= k; ++j) {
            const i = r.order[j];
            if (r.valid && r.valid[i] === 0) {
                continue;
            }
            const x = odom ? r.ox[i] : (r.fx ? r.fx[i] : NaN);
            const y = odom ? r.oy[i] : (r.fy ? r.fy[i] : NaN);
            if (Number.isFinite(x) && Number.isFinite(y)) {
                points.push([x, y, r.t[j]]);
            }
        }
        return { points, odom };
    }

    // Trail points of sorted rows a..b (inclusive), same frame choice as
    // trailPoints; the caller keeps both ends inside one segment.
    trailSlice(a, b) {
        const r = this.robot;
        const out = [];
        if (!r) {
            return out;
        }
        const odom = !!(r.ox && r.oy);
        for (let j = Math.max(0, a); j <= b && j < r.order.length; ++j) {
            const i = r.order[j];
            if (r.valid && r.valid[i] === 0) {
                continue;
            }
            const x = odom ? r.ox[i] : (r.fx ? r.fx[i] : NaN);
            const y = odom ? r.oy[i] : (r.fy ? r.fy[i] : NaN);
            if (Number.isFinite(x) && Number.isFinite(y)) {
                out.push([x, y, r.t[j]]);
            }
        }
        return out;
    }

    pathAt(t) {
        if (!this.paths.length) {
            return null;
        }
        const k = upperIndex(this.pathT, t);
        return k >= 0 ? this.paths[k] : null;
    }

    // A telemetry-message-shaped object at t with its age, or null.
    telemetryAt(t) {
        const m = this.telemetry;
        if (!m) {
            return null;
        }
        const k = upperIndex(m.t, t);
        if (k < 0) {
            return null;
        }
        const i = m.order[k];
        const v = (a) => (a ? a[i] : NaN);
        const hasMotion = (m.motionPresent ? v(m.motionPresent) === 1 : true) && Number.isFinite(v(m.command)) && Number.isFinite(v(m.tx));
        return {
            age_ms: t - m.t[k],
            host_ms: m.t[k],
            attitude: Number.isFinite(v(m.roll)) && (m.attitudePresent ? v(m.attitudePresent) === 1 : true)
                ? { roll_deg: v(m.roll), pitch_deg: v(m.pitch) } : null,
            motion: hasMotion ? {
                command_id: v(m.command), state: v(m.state),
                state_name: (m.stateName ? m.stateName[i] : undefined) || nameOf(MOTION_STATE, v(m.state)),
                reason: v(m.reason), reason_name: nameOf(MOTION_REASON, v(m.reason)),
                mode: v(m.mode), mode_name: nameOf(PLAN_MODE, v(m.mode)),
                segment: v(m.segment), segment_count: v(m.segmentCount),
                target: { x_m: v(m.tx), y_m: v(m.ty), heading_deg: v(m.th) },
                cmd: { vx_m_s: v(m.cvx), vy_m_s: v(m.cvy), omega_deg_s: v(m.cw) },
                cross_track_m: v(m.ct), distance_error_m: v(m.de), heading_error_deg: v(m.he),
                drive_fault: v(m.fault), drive_fault_name: nameOf(DRIVE_FAULT, v(m.fault)),
            } : null,
        };
    }

    summary() {
        const m = this.metadata || {};
        const streams = [...this.tables.entries()].map(([n, t]) => `${n} ${t.n}`).join(', ');
        return { id: m.id || this.label, streams, duration_s: (this.endT - this.startT) / 1000, metadata: m };
    }
}

// Playback: a playhead on the timeline advanced by the page's animation
// loop at speed x real time. Seeking never plays through the skipped span.
export class ReplayClock {
    constructor(model) {
        this.model = model;
        this.t = model.startT;
        this.playing = false;
        this.speed = 1;
        this.lastNow = -1;
        this.version = 0;
        this.jumped = true;  // the next frame rebuilds derived state
    }

    play(on) {
        this.playing = on;
        this.lastNow = -1;
        if (on && this.t >= this.model.endT) {
            this.seek(this.model.startT);
        }
    }

    seek(t) {
        this.t = Math.max(this.model.startT, Math.min(this.model.endT, t));
        this.jumped = true;
        this.version += 1;
    }

    tick(nowMs) {
        if (!this.playing) {
            this.lastNow = -1;
            return;
        }
        if (this.lastNow >= 0) {
            const dt = Math.min(250, nowMs - this.lastNow) * this.speed;
            this.t = Math.min(this.model.endT, this.t + dt);
            this.version += 1;
            if (this.t >= this.model.endT) {
                this.playing = false;
            }
        }
        this.lastNow = nowMs;
    }
}
