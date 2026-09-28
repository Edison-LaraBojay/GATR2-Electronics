// unit_tests.js
// Unit tests of the viewer's pure modules, run in a real browser by
// viewer_tests.py (unit). Results go to window.__unit = {done, passed,
// failed, results:[{name, ok, detail}]}. The fixture bundle (written by
// capture_fixture.py) and its closed-form expectations are fetched from the
// URL in ?fixture= when given.

import { PoseSmoother, SMOOTH_MAX_MS, SNAP_DIST_M } from '../../spectaGATR/smoothing.js';
import { SeriesStore, Decimator, valueAt, lowerBound, FLAG_BREAK } from '../../spectaGATR/series.js';
import { crc32, readZip, parseCsvRows, CsvTable, unitOf } from '../../spectaGATR/bundle.js';
import { ClockSync, Stat } from '../../spectaGATR/latency.js';
import { KeyedTable } from '../../spectaGATR/dom.js';
import { Feed } from '../../spectaGATR/feed.js';
import { LiveCollector } from '../../spectaGATR/graphs.js';
import { ReplayModel, ReplayClock } from '../../spectaGATR/replay.js';
import { FieldScene, TRAIL_MAX } from '../../spectaGATR/field_scene.js';

const results = [];
window.__unit = { done: false, passed: 0, failed: 0, results };

function check(cond, msg) {
    if (!cond) {
        throw new Error(msg);
    }
}

function near(a, b, tol, msg) {
    check(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b} (tol ${tol})`);
}

async function test(name, fn) {
    try {
        await fn();
        results.push({ name, ok: true });
        window.__unit.passed += 1;
    } catch (e) {
        results.push({ name, ok: false, detail: String(e && e.stack ? e.stack : e) });
        window.__unit.failed += 1;
    }
}

const ident = (o) => Object.assign({ session: 's', reset: 0, epoch: 1, anchor: 1, placement: 1, placed: true, valid: true, connection: 1 }, o);
const pose = (x, y, h) => ({ x_m: x, y_m: y, heading_deg: h });

// --- smoothing ---

await test('smoothing: first pose snaps', () => {
    const s = new PoseSmoother();
    check(s.push(pose(1, 2, 3), ident(), 0, false) === 'first', 'first');
    const o = s.sample(5, false);
    check(o.x_m === 1 && o.y_m === 2 && o.heading_deg === 3, 'snapped to the first pose');
});

await test('smoothing: blends over one period, never past the newest pose', () => {
    const s = new PoseSmoother();
    for (let t = 0; t <= 330; t += 33) {
        s.noteArrival(t);
    }
    s.push(pose(0, 0, 0), ident(), 0, false);
    check(s.push(pose(0.05, 0, 0), ident(), 33, false) === '', 'no snap for a small step');
    check(s.dur > 0 && s.dur <= SMOOTH_MAX_MS, `blend duration ${s.dur}`);
    near(s.dur, 33, 3, 'duration follows the state period');
    near(s.sample(33, false).x_m, 0, 1e-9, 'starts at the drawn pose');
    near(s.sample(33 + s.dur / 2, false).x_m, 0.025, 1e-6, 'halfway');
    near(s.sample(33 + s.dur, false).x_m, 0.05, 1e-9, 'reaches the newest pose');
    near(s.sample(10000, false).x_m, 0.05, 1e-9, 'no extrapolation after the blend');
    check(s.lagMs(33) <= SMOOTH_MAX_MS, 'lag bounded');
});

await test('smoothing: period capped at 100 ms, pauses ignored', () => {
    const s = new PoseSmoother();
    s.noteArrival(0);
    s.noteArrival(2000);   // a pause: ignored
    for (let t = 2000; t < 4000; t += 200) {
        s.noteArrival(t);  // 200 ms is above the stale bound: ignored too
    }
    for (let t = 4000; t < 6000; t += 150) {
        s.noteArrival(t);
    }
    s.push(pose(0, 0, 0), ident(), 6000, false);
    s.push(pose(0.01, 0, 0), ident(), 6150, false);
    check(s.dur <= SMOOTH_MAX_MS, `duration ${s.dur} capped`);
});

await test('smoothing: jump and turn snap', () => {
    const s = new PoseSmoother();
    s.push(pose(0, 0, 0), ident(), 0, false);
    check(s.push(pose(SNAP_DIST_M + 0.01, 0, 0), ident(), 33, false) === 'jump', 'jump');
    near(s.sample(34, false).x_m, SNAP_DIST_M + 0.01, 1e-9, 'shown at once');
    check(s.push(pose(SNAP_DIST_M + 0.01, 0, 16), ident(), 66, false) === 'turn', 'turn over 15 deg');
});

await test('smoothing: identity, reconnect, placement, validity changes snap', () => {
    for (const change of [{ session: 't' }, { reset: 1 }, { epoch: 2 }, { anchor: 2 }, { placement: 2 }, { placed: false }, { valid: false }, { connection: 2 }]) {
        const s = new PoseSmoother();
        s.push(pose(0, 0, 0), ident(), 0, false);
        check(s.push(pose(0.01, 0, 0), ident(change), 33, false) === 'identity', 'snap on ' + JSON.stringify(change));
    }
});

await test('smoothing: stale draws the newest pose exactly', () => {
    const s = new PoseSmoother();
    s.push(pose(0, 0, 0), ident(), 0, false);
    s.push(pose(0.05, 0, 0), ident(), 33, false);
    near(s.sample(40, true).x_m, 0.05, 1e-12, 'stale: exact');
    check(s.push(pose(0.06, 0, 0), ident(), 400, true) === 'stale', 'push while stale snaps');
});

await test('smoothing: off draws exact values', () => {
    const s = new PoseSmoother();
    s.setEnabled(false);
    s.push(pose(0, 0, 0), ident(), 0, false);
    s.push(pose(0.05, 0, 0), ident(), 33, false);
    near(s.sample(34, false).x_m, 0.05, 1e-12, 'exact');
});

await test('smoothing: heading blends the short way across +-180', () => {
    const s = new PoseSmoother();
    s.push(pose(0, 0, 175), ident(), 0, false);
    s.push(pose(0, 0, -175), ident(), 33, false);
    const h = s.sample(33 + s.dur / 2, false).heading_deg;
    check(Math.abs(Math.abs(h) - 180) < 1e-6, `mid heading ${h}`);
});

// --- series ---

await test('series: ring is bounded and ordered', () => {
    const st = new SeriesStore();
    const c = st.channel('c', 8);
    const s = st.get('a', c, {});
    for (let i = 0; i < 20; ++i) {
        s.set(c.begin(i * 10), i, true);
    }
    check(c.n === 8, 'n');
    check(c.oldest() === 120 && c.newest() === 190, `oldest ${c.oldest()} newest ${c.newest()}`);
    check(c.begin(150) === -1 && c.outOfOrder === 1, 'out of order dropped and counted');
    check(lowerBound(c, 155) === 4, 'lower bound');
    const v = valueAt(s, 175);
    check(v.t === 170 && v.v === 17, 'value at or before');
});

await test('series: min/max decimation keeps a one-sample spike', () => {
    const st = new SeriesStore();
    const c = st.channel('c', 4096);
    const s = st.get('a', c, {});
    for (let i = 0; i < 2000; ++i) {
        s.set(c.begin(i), i === 1234 ? 100 : 0, true);
    }
    const d = new Decimator().run(s, 0, 2000, 50);
    let max = -Infinity;
    for (let k = 0; k < 50; ++k) {
        max = Math.max(max, d.max[k]);
    }
    check(max === 100, 'spike survives');
    check(d.hi === 100 && d.lo === 0, 'range');
});

await test('series: gaps and missing values break the line, never zero', () => {
    const st = new SeriesStore();
    const c = st.channel('c', 4096);
    const s = st.get('a', c, {});
    let t = 0;
    for (let i = 0; i < 100; ++i) {
        s.set(c.begin(t), 5, true);
        t += 10;
    }
    t += 1000;  // a hole of 100x the interval
    for (let i = 0; i < 100; ++i) {
        s.set(c.begin(t), 5, true);
        t += 10;
    }
    s.set(c.begin(t), null, true);  // missing
    t += 10;
    s.set(c.begin(t), 5, true);
    check(c.gapMs < 100, `gap threshold ${c.gapMs}`);
    const d = new Decimator().run(s, 0, t + 10, 400);
    let breaks = 0;
    for (let k = 0; k < 400; ++k) {
        if (d.has[k] && d.brk[k]) {
            breaks += 1;
        }
        if (d.has[k]) {
            check(d.min[k] === 5, 'no zero filled in');
        }
    }
    check(breaks === 3, `breaks: start, after the hole, after the missing value (got ${breaks})`);
});

await test('series: discontinuity and invalid marks', () => {
    const st = new SeriesStore();
    const c = st.channel('c', 64);
    const s = st.get('a', c, {});
    for (let i = 0; i < 10; ++i) {
        if (i === 5) {
            c.markBreak();
        }
        s.set(c.begin(i * 10), i, i !== 7);
    }
    check((c.flags[c.slot(5)] & FLAG_BREAK) !== 0, 'break flag');
    const d = new Decimator().run(s, 0, 100, 10);
    check(d.brk[5] === 1, 'break drawn');
    check(d.bad[7] === 1, 'invalid marked');
    check(valueAt(s, 70).valid === false, 'invalid reported');
});

await test('series: heading unwraps within a segment', () => {
    const st = new SeriesStore();
    const c = st.channel('c', 64);
    const s = st.get('h', c, { wrap: true });
    const hs = [170, 175, 179, -179, -175, -170];
    hs.forEach((h, i) => s.set(c.begin(i * 10), h, true));
    const d = new Decimator().run(s, 0, 60, 6);
    near(d.last[5], 190, 1e-4, 'continuous through +-180');
    check(d.hi - d.lo < 30, 'no column spans the wrap');
});

await test('series: a sample at the newest time replaces it', () => {
    const st = new SeriesStore();
    const c = st.channel('c', 8);
    const s = st.get('a', c, {});
    s.set(c.begin(10), 1, true);
    s.set(c.begin(20), 2, true);
    s.set(c.begin(20), 3, true);
    check(c.n === 2 && c.outOfOrder === 0, 'same time reuses the slot');
    check(valueAt(s, 25).v === 3, 'the newer value wins');
});

// --- scene trail ring (needs WebGL: swiftshader in headless Chrome) ---

const sceneEl = document.getElementById('scene');
const scene = new FieldScene(sceneEl);

await test('scene: trail ring wraps into one contiguous draw range', () => {
    check(scene.available, 'WebGL available for the scene tests');
    scene.clearTrail();
    const n = TRAIL_MAX + 10;
    for (let i = 0; i < n; ++i) {
        scene.trailPush(i * 0.01, 0, i);
    }
    scene.flushTrail();
    check(scene.trailCount === TRAIL_MAX, `count ${scene.trailCount}`);
    const dr = scene.trail.geometry.drawRange;
    check(dr.start === 10 && dr.count === TRAIL_MAX, `draw range ${dr.start}+${dr.count}`);
    const buf = scene.trailBuf;
    for (const k of [0, 1, TRAIL_MAX - 1]) {
        near(buf[(dr.start + k) * 3], (10 + k) * 0.01, 1e-6, `point ${k} in order`);
    }
});

await test('scene: a new point uploads only its own bytes, twice', () => {
    scene.clearTrail();
    for (let i = 0; i < 100; ++i) {
        scene.trailPush(i * 0.01, 0, i);
    }
    scene.flushTrail();
    const before = scene.trailUploadedBytes;
    scene.trailPush(5, 5, 1000);
    scene.trailPush(5.1, 5, 1001);
    scene.flushTrail();
    const ranges = scene.trailAttr.updateRanges;
    check(ranges.length === 2 && ranges[0].count === 6 && ranges[1].start === ranges[0].start + TRAIL_MAX * 3,
        `ranges ${JSON.stringify(ranges)}`);
    check(scene.trailUploadedBytes - before === 2 * 2 * 12, 'bytes accounted');
});

await test('scene: near-duplicate points only move the time', () => {
    scene.clearTrail();
    scene.trailPush(1, 1, 1);
    scene.trailPush(1.0005, 1, 2);
    check(scene.trailCount === 1 && scene.trailLastT === 2, 'kept one point');
});

await test('scene: history replace keeps what arrived after it; a new epoch clears', () => {
    scene.clearTrail();
    scene.trailEpoch = 3;
    scene.trailPush(9, 9, 500);   // arrived from a state after the history snapshot
    const hist = [];
    for (let i = 0; i < 50; ++i) {
        hist.push({ host_ms: i * 10, x_m: i * 0.02, y_m: 0, epoch: i < 10 ? 2 : 3 });
    }
    scene.trailFromHistory(hist, true, 3);
    check(scene.trailCount === 41, `40 history points of epoch 3 plus the newer one: ${scene.trailCount}`);
    check(scene.trailLastT === 500, 'newest kept');
    scene.trailFromState({ valid: true, measured_at_host_ms: 600, odometry_epoch: 4, odom: { x_m: 0, y_m: 0 },
        field_from_odom: { x_m: 1, y_m: 2, heading_deg: 90 } });
    check(scene.trailCount === 1 && scene.trailEpoch === 4, 'new epoch starts over');
    near(scene.trail.rotation.z, Math.PI / 2, 1e-9, 'trail carries field_from_odom');
});

await test('scene: robot label and stale look follow the exact state', () => {
    const r = { valid: true, initialized: true, field: { x_m: 1, y_m: 1, heading_deg: 0 },
        attitude: { status: 'measured', roll_deg: 5, pitch_deg: 0 } };
    scene.applyRobotState(r, false);
    check(scene.robotLabelText === 'robot' && scene.tiltRoll > 0, 'measured attitude tilts');
    scene.applyRobotState(Object.assign({}, r, { attitude: { status: 'stale', roll_deg: 5, pitch_deg: 0 } }), false);
    check(scene.robotLabelText === 'robot (attitude stale)' && scene.tiltRoll === 0, 'stale attitude is not drawn as tilt');
    scene.applyRobotState(r, true);
    check(scene.robotLabelText === 'robot (pose stale)' && scene.robotMaterial.opacity < 0.3, 'stale pose looks stale');
    scene.applyRobotState(Object.assign({}, r, { valid: false }), false);
    check(!scene.robot.visible, 'invalid pose hidden');
});

await test('scene: telemetry overlays only for a running command, fresh, never a (0,0,0) guess', () => {
    const tel = (state, flags, target) => ({ flags, motion: { command_id: 7, state, target, segment: 0, segment_count: 1 } });
    const t = { x_m: 2, y_m: 1, heading_deg: 90 };
    scene.applyTelemetry(tel(2, 3, t), true);
    check(scene.dest.visible, 'running: destination drawn');
    near(scene.desired.rotation.z, Math.PI / 2, 1e-9, 'desired heading is the destination heading');
    scene.applyTelemetry(tel(2, 3, t), false);
    check(!scene.dest.visible, 'stale telemetry: hidden');
    for (const st of [0, 1, 4]) {
        scene.applyTelemetry(tel(st, 3, { x_m: 0, y_m: 0, heading_deg: 0 }), true);
        check(!scene.dest.visible, `state ${st} without a target flag: hidden`);
    }
    scene.applyTelemetry(tel(4, 3 | 8, t), true);
    check(scene.dest.visible && !scene.desired.visible, 'target flag: drawn, dimmed, no desired heading after the end');
    scene.applyTelemetry(null, true);
    check(!scene.dest.visible && !scene.followTarget.visible, 'no telemetry: hidden');
});

// --- bundle ---

function zipStore(files) {
    const enc = new TextEncoder();
    const parts = [];
    const central = [];
    let offset = 0;
    for (const [name, text] of files) {
        const data = typeof text === 'string' ? enc.encode(text) : text;
        const nameB = enc.encode(name);
        const crc = crc32(data);
        const local = new DataView(new ArrayBuffer(30));
        local.setUint32(0, 0x04034b50, true);
        local.setUint16(4, 20, true);
        local.setUint32(14, crc, true);
        local.setUint32(18, data.length, true);
        local.setUint32(22, data.length, true);
        local.setUint16(26, nameB.length, true);
        parts.push(new Uint8Array(local.buffer), nameB, data);
        const cd = new DataView(new ArrayBuffer(46));
        cd.setUint32(0, 0x02014b50, true);
        cd.setUint16(4, 20, true);
        cd.setUint16(6, 20, true);
        cd.setUint32(16, crc, true);
        cd.setUint32(20, data.length, true);
        cd.setUint32(24, data.length, true);
        cd.setUint16(28, nameB.length, true);
        cd.setUint32(42, offset, true);
        central.push(new Uint8Array(cd.buffer), nameB);
        offset += 30 + nameB.length + data.length;
    }
    const cdSize = central.reduce((a, b) => a + b.length, 0);
    const end = new DataView(new ArrayBuffer(22));
    end.setUint32(0, 0x06054b50, true);
    end.setUint16(8, files.length, true);
    end.setUint16(10, files.length, true);
    end.setUint32(12, cdSize, true);
    end.setUint32(16, offset, true);
    const all = [...parts, ...central, new Uint8Array(end.buffer)];
    const out = new Uint8Array(all.reduce((a, b) => a + b.length, 0));
    let p = 0;
    for (const a of all) {
        out.set(a, p);
        p += a.length;
    }
    return out;
}

await test('bundle: CRC-32 check value', () => {
    check(crc32(new TextEncoder().encode('123456789')) === 0xcbf43926, 'zlib CRC-32');
});

await test('bundle: store-only ZIP round trip, CRC mismatch refused', async () => {
    const z = zipStore([['a.csv', 'x,y\n1,2\n'], ['dir/b.txt', 'hello']]);
    const m = await readZip(z.buffer);
    check(new TextDecoder().decode(m.get('a.csv')) === 'x,y\n1,2\n', 'a.csv');
    check(new TextDecoder().decode(m.get('b.txt')) === 'hello', 'folder stripped');
    const bad = z.slice();
    bad[30 + 5 + 2] ^= 0xff;   // a byte of a.csv's data
    let threw = false;
    try {
        await readZip(bad.buffer);
    } catch (e) {
        threw = /CRC/.test(String(e));
    }
    check(threw, 'CRC mismatch is an error');
    let notZip = false;
    try {
        await readZip(new Uint8Array(100).buffer);
    } catch (e) {
        notZip = true;
    }
    check(notZip, 'not a ZIP');
});

await test('bundle: CSV quoting, missing values, line ends', () => {
    const text = 'a,b,c\r\n1,"x, ""y""",\n"",2,"line\nbreak"\n\n3,,4';
    const { header, rows } = parseCsvRows(text);
    check(header.join('|') === 'a|b|c', 'header');
    check(rows.length === 3, `rows ${rows.length}`);
    check(rows[0][1] === 'x, "y"' && rows[0][2] === '', 'quoted comma and quote, empty last');
    check(rows[1][2] === 'line\nbreak', 'newline inside quotes');
    const t = new CsvTable('t.csv', text);
    const a = t.num('a');
    check(a[0] === 1 && Number.isNaN(a[1]) && a[2] === 3, 'missing is NaN, not zero');
    check(t.str('b')[2] === null, 'missing text is null');
    check(t.numericColumns().join() === 'a', `numeric columns: ${t.numericColumns().join()}`);
    check(unitOf('yaw_rate_deg_s') === 'deg/s' && unitOf('x_m') === 'm' && unitOf('pi_host_us') === 'us', 'units from names');
});

// --- latency ---

await test('latency: offset from the minimum-RTT ping', () => {
    const c = new ClockSync();
    // Pi clock = browser + 1000; one-way delays vary
    const pings = [[0, 30, 50], [100, 5, 5], [200, 80, 10]];
    for (const [t, up, down] of pings) {
        const m = c.ping(t);
        c.pong({ id: m.id, client_ms: t, host_ms: t + up + 1000, host_us: (t + up + 1000) * 1000 }, t + up + down);
    }
    near(c.best.rtt, 10, 1e-9, 'best rtt');
    near(c.toPi(500), 1500, 1e-9, 'symmetric best ping: exact offset');
    near(c.uncertaintyMs(), 5, 1e-9, 'bound is RTT/2');
    c.reset();
    check(Number.isNaN(c.toPi(1)), 'no estimate after reset');
    const st = new Stat(10);
    for (let i = 1; i <= 100; ++i) {
        st.push(i);
    }
    const s = st.summary();
    check(s.n === 10 && s.max === 100 && s.p50 === 95, `window stats ${JSON.stringify(s)}`);
});

// --- dom ---

await test('dom: keyed rows are reused, reordered and removed in place', () => {
    const t = new KeyedTable(document.getElementById('keyed'), ['k', 'v']);
    t.begin();
    const a = t.row('a', ['a', 1]);
    const b = t.row('b', ['b', 2]);
    t.end();
    t.begin();
    const b2 = t.row('b', ['b', 3]);
    const a2 = t.row('a', ['a', 1]);
    t.end();
    check(a === a2 && b === b2, 'same elements');
    check(t.table.children[1] === b && t.table.children[2] === a, 'reordered');
    check(b.children[1].textContent === '3', 'text updated');
    t.begin();
    t.row('a', ['a', 1]);
    t.end();
    check(t.table.children.length === 2 && !b.isConnected, 'removed');
});

// --- feed (no socket) ---

await test('feed: stores latest, splits inspect/1, dedupes events, counts seq gaps', () => {
    let sessions = 0;
    const stored = [];
    const f = new Feed({ onSession: () => { sessions += 1; }, onStored: (type) => stored.push(type) });
    const sess = { id: 'A', reset_count: 0 };
    f.onMessage(JSON.stringify({ type: 'hello', contract: 'navigatr.inspect/2', session: sess, features: { state_hz: 30 } }));
    check(f.contract2 && sessions === 1, 'hello');
    f.onMessage(JSON.stringify({ type: 'state', seq: 1, host_ms: 10, session: sess, publication: 1, robot: { valid: true } }));
    f.onMessage(JSON.stringify({ type: 'state', seq: 4, host_ms: 20, session: sess, publication: 2, robot: { valid: true } }));
    check(f.state.host_ms === 20 && f.stateVersion === 2, 'latest state');
    check(f.seqGaps.get('state') === 2, 'seq gap counted');
    f.onMessage(JSON.stringify({ type: 'event', seq: 7, host_ms: 5, text: 'x' }));
    f.onMessage(JSON.stringify({ type: 'diag', host_ms: 30, session: sess, events: [{ host_ms: 5, text: 'x' }, { host_ms: 6, text: 'y' }] }));
    check(f.events.length === 2, `events deduped: ${f.events.length}`);
    f.onMessage(JSON.stringify({ type: 'hello', contract: 'navigatr.inspect/2', session: { id: 'A', reset_count: 1 } }));
    check(sessions === 2 && f.events.length === 0 && f.state === null, 'reset clears');
    f.onMessage(JSON.stringify({ type: 'snapshot', host_ms: 40, session: { id: 'A', reset_count: 1 }, cycle: 3,
        robot: { valid: true }, localization: { publication: 9 }, trail: [{ host_ms: 1, x_m: 0, y_m: 0, epoch: 1 }] }));
    check(f.state.compat && f.state.publication === 9 && f.diag.cycle === 3, 'inspect/1 split');
    check(f.history.length === 1 && f.history[0].replace === false, 'inspect/1 trail merges');
    check(stored.includes('state') && stored.includes('diag'), 'store hooks');
    f.onMessage('{bad json');
});

// --- graphs collector ---

await test('graphs: one sample per publication, breaks, invalid and unmeasured values', () => {
    const st = new SeriesStore();
    const col = new LiveCollector(st);
    const sess = { id: 'A', reset_count: 0 };
    const robot = (x, epoch, valid, att) => ({
        valid, initialized: true, odometry_epoch: epoch, anchor_revision: 1, placement_sequence: 1,
        field: { x_m: x, y_m: 0, heading_deg: 0 }, odom: { x_m: x, y_m: 0, heading_deg: 0 },
        vx_m_s: 0, vy_m_s: 0, yaw_rate_deg_s: 0, measured_at_host_ms: 0, age_ms: 5, attitude: att,
    });
    const put = (t, pub, r) => col.onState({ host_ms: t, publication: pub, session: sess, robot: Object.assign(r, { measured_at_host_ms: t - 5 }) });
    put(100, 1, robot(1, 1, true, { status: 'measured', roll_deg: 1, pitch_deg: 2 }));
    put(1100, 1, robot(1, 1, true, { status: 'measured', roll_deg: 1, pitch_deg: 2 }));   // keepalive
    put(200, 2, robot(2, 1, true, { status: 'assumed_level', valid: false, assumed_level: true }));
    put(300, 3, robot(3, 2, true, null));   // new epoch
    put(400, 4, robot(4, 2, false, null));  // not valid
    const c = st.channels.get('state');
    check(c.n === 4, `samples ${c.n}`);
    const x = st.series.get('robot.x');
    const roll = st.series.get('robot.roll');
    check(Number.isNaN(roll.v[c.slot(1)]), 'assumed level is not a measurement');
    check((c.flags[c.slot(2)] & FLAG_BREAK) !== 0, 'epoch change breaks the line');
    check(Number.isNaN(x.v[c.slot(3)]) && valueAt(x, 395).valid === false, 'invalid pose is missing and marked');
});

// --- replay against the fixture bundle ---

const params = new URLSearchParams(location.search);
const fixtureUrl = params.get('fixture');
if (fixtureUrl) {
    const buf = await (await fetch(fixtureUrl)).arrayBuffer();
    const expected = await (await fetch(params.get('expected'))).json();
    await test('replay: fixture loads with every stream', async () => {
        const m = await ReplayModel.fromZip(buf, 'fixture');
        check(m.warnings.length === 0, 'warnings: ' + m.warnings.join('; '));
        check(m.robot && m.robot.segments === 2, `segments ${m.robot && m.robot.segments}`);
        check(m.paths.length === 1 && m.paths[0].points.length === 5, 'path');
        check(m.events.length >= 4, 'events');
        check(m.store.series.has('robot.x') && m.store.series.has('tel.cmd_vx') && m.store.series.has('enc.counts.0'), 'standard series');
        check(m.columns.length > 20, 'capture columns');
    });
    await test('replay: robot state at a time is the recorded row at or before it', async () => {
        const m = await ReplayModel.fromZip(buf, 'fixture');
        for (const e of expected.points) {
            const r = m.robotAt(e.t_ms);
            near(r.field.x_m, e.x, 2e-6, `x at ${e.t_ms}`);
            near(r.field.y_m, e.y, 2e-6, `y at ${e.t_ms}`);
            check(r.initialized === e.placed, `placed at ${e.t_ms}`);
        }
        check(m.robotAt(m.startT - 1) === null, 'nothing before the first row');
    });
    await test('replay: trails never cross a segment; anchor re-expresses odometry', async () => {
        const m = await ReplayModel.fromZip(buf, 'fixture');
        const k = m.robotIndex(expected.epoch_t_ms + 500);
        const tr = m.trailPoints(k, 100000);
        check(tr.points[0][2] >= expected.epoch_t_ms, 'trail starts at the new epoch');
        const r = m.robotAt(expected.epoch_t_ms + 500);
        const a = m.anchorOf(r);
        const h = a.heading_deg * Math.PI / 180;
        near(a.x_m + Math.cos(h) * r.odom.x_m - Math.sin(h) * r.odom.y_m, r.field.x_m, 1e-5, 'anchor x');
        near(a.y_m + Math.sin(h) * r.odom.x_m + Math.cos(h) * r.odom.y_m, r.field.y_m, 1e-5, 'anchor y');
        const before = m.robotAt(expected.place_t_ms - 100);
        check(m.anchorOf(before) === null, 'no anchor before placement');
    });
    await test('replay: telemetry and path at a time; clock bounds', async () => {
        const m = await ReplayModel.fromZip(buf, 'fixture');
        const tel = m.telemetryAt(m.startT + 3500);
        check(tel && tel.motion && tel.motion.command_id === 7 && tel.motion.target.x_m === 2.5, 'telemetry');
        check(tel.age_ms >= 0 && tel.age_ms < 100, 'telemetry age');
        check(m.pathAt(m.startT + 500) === null && m.pathAt(m.startT + 1500).command_id === 7, 'path appears when reported');
        const c = new ReplayClock(m);
        c.speed = 4;
        c.play(true);
        c.tick(0);
        c.tick(100);
        near(c.t - m.startT, 400, 1e-6, 'speed');
        c.seek(m.endT + 1000);
        check(c.t === m.endT, 'seek clamps');
        c.tick(200);
        c.tick(300);
        check(!c.playing, 'stops at the end');
    });
}

window.__unit.done = true;
const out = document.getElementById('out');
out.innerHTML = results.map((r) => `<span class="${r.ok ? 'pass' : 'fail'}">${r.ok ? 'PASS' : 'FAIL'} ${r.name}</span>${r.ok ? '' : '\n    ' + r.detail.replace(/</g, '&lt;')}`).join('\n') +
    `\n\n${window.__unit.passed} passed, ${window.__unit.failed} failed`;
