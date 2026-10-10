// tests.js
// Viewer tests without Node: open tests/index.html in a browser (or headless
// Chrome with --dump-dom). Each test writes PASS or FAIL into #log; #summary
// ends as "PASS n/n" or "FAIL k of n" with data-result PASS or FAIL.

import {
    DEG, Smoother, TrailRing, angleDelta, attitudeMatrix, attitudeQuaternion, feedState,
    fieldToScene, fmtAge, normalizePose, overheadCamera, presentPose, quaternionYaw,
    rotateVector, slerp, wrapPi,
} from '../transforms.js';
import { FeedModel } from '../feed.js';

const tests = [];
function test(name, fn) {
    tests.push([name, fn]);
}

function assert(cond, msg) {
    if (!cond) {
        throw new Error(msg || 'assertion failed');
    }
}

function near(a, b, tol, msg) {
    if (!(Math.abs(a - b) <= tol)) {
        throw new Error(`${msg ? msg + ': ' : ''}expected ${b}, got ${a} (tol ${tol})`);
    }
}

function nearVec(a, b, tol, msg) {
    for (let i = 0; i < b.length; ++i) {
        near(a[i], b[i], tol, `${msg || 'vector'}[${i}]`);
    }
}

// --- mapping and angles ---

test('field to scene is the identity (+X -> scene x, +Y -> scene y, up -> z)', () => {
    nearVec(fieldToScene(1.5, -0.25), [1.5, -0.25, 0], 0);
    nearVec(fieldToScene(0, 2, 0.3), [0, 2, 0.3], 0);
});

test('wrapPi wraps into (-pi, pi]', () => {
    near(wrapPi(181 * DEG) / DEG, -179, 1e-9);
    near(wrapPi(-180 * DEG) / DEG, 180, 1e-9);
    near(wrapPi(540 * DEG) / DEG, 180, 1e-9);
    near(wrapPi(-190 * DEG) / DEG, 170, 1e-9);
});

test('heading wrap 179 -> -179 deg is a +2 deg turn (and back is -2)', () => {
    near(angleDelta(179 * DEG, -179 * DEG) / DEG, 2, 1e-9);
    near(angleDelta(-179 * DEG, 179 * DEG) / DEG, -2, 1e-9);
});

test('heading 0 faces +X, heading +90 faces +Y (CCW positive)', () => {
    nearVec(rotateVector(attitudeQuaternion(0, 0, 0), [1, 0, 0]), [1, 0, 0], 1e-12);
    nearVec(rotateVector(attitudeQuaternion(0, 0, 90 * DEG), [1, 0, 0]), [0, 1, 0], 1e-12);
    nearVec(rotateVector(attitudeQuaternion(0, 0, -90 * DEG), [1, 0, 0]), [0, -1, 0], 1e-12);
});

test('roll +10 deg raises the left side (robot +y), lowers the right', () => {
    const q = attitudeQuaternion(10 * DEG, 0, 0);
    const left = rotateVector(q, [0, 0.1465, 0]);
    const right = rotateVector(q, [0, -0.1465, 0]);
    assert(left[2] > 0, 'left z ' + left[2]);
    assert(right[2] < 0, 'right z ' + right[2]);
    near(left[2], 0.1465 * Math.sin(10 * DEG), 1e-12);
});

test('pitch +10 deg lowers the nose (robot +x), raises the back', () => {
    const q = attitudeQuaternion(0, 10 * DEG, 0);
    const nose = rotateVector(q, [0.1685, 0, 0]);
    const back = rotateVector(q, [-0.1685, 0, 0]);
    assert(nose[2] < 0, 'nose z ' + nose[2]);
    assert(back[2] > 0, 'back z ' + back[2]);
    near(nose[2], -0.1685 * Math.sin(10 * DEG), 1e-12);
});

test('composition is Rz(yaw) Ry(pitch) Rx(roll): yaw 90 + pitch 10 noses down toward +Y', () => {
    const f = rotateVector(attitudeQuaternion(0, 10 * DEG, 90 * DEG), [1, 0, 0]);
    nearVec(f, [0, Math.cos(10 * DEG), -Math.sin(10 * DEG)], 1e-12);
    const l = rotateVector(attitudeQuaternion(10 * DEG, 0, 90 * DEG), [0, 1, 0]);
    nearVec(l, [-Math.cos(10 * DEG), 0, Math.sin(10 * DEG)], 1e-12);
});

test('quaternion matches the row-major matrix for mixed angles', () => {
    const cases = [[0.3, -0.2, 2.5], [-1.0, 0.7, -3.0], [0.05, 0.4, 0.9]];
    for (const [r, p, y] of cases) {
        const q = attitudeQuaternion(r, p, y);
        const R = attitudeMatrix(r, p, y);
        for (let c = 0; c < 3; ++c) {
            const e = [0, 0, 0];
            e[c] = 1;
            nearVec(rotateVector(q, e), [R[c], R[3 + c], R[6 + c]], 1e-12, `column ${c}`);
        }
    }
});

test('slerp across +-180 deg takes the short way (179 -> -179 passes 180)', () => {
    const a = attitudeQuaternion(0, 0, 179 * DEG);
    const b = attitudeQuaternion(0, 0, -179 * DEG);
    near(Math.abs(quaternionYaw(slerp(a, b, 0.5))) / DEG, 180, 1e-6);
    near(quaternionYaw(slerp(a, b, 0.25)) / DEG, 179.5, 1e-6);
});

// --- received poses ---

test('heading-only pose has roll = pitch = 0 even if the message carries them', () => {
    const p = normalizePose({ x: 1, y: 2, yaw: 0.5, roll: 0.3, pitch: 0.2, attitude: false });
    assert(p && p.roll === 0 && p.pitch === 0 && p.attitude === false);
    const f = rotateVector(presentPose(p).q, [0, 1, 0]);
    near(f[2], 0, 1e-12, 'left side height');
});

test('non-finite or incomplete poses are rejected', () => {
    assert(normalizePose({ x: NaN, y: 0, yaw: 0 }) === null, 'NaN x');
    assert(normalizePose({ x: 0, y: 0, yaw: Infinity }) === null, 'inf yaw');
    assert(normalizePose({ x: 0, y: 0 }) === null, 'missing yaw');
    assert(normalizePose({ x: 0, y: 0, yaw: 0, attitude: true, roll: null, pitch: 0 }) === null, 'null roll');
    assert(normalizePose({ x: '1', y: 0, yaw: 0 }) === null, 'string x');
});

// --- trail ---

test('trail ring stays bounded (10000 points into 100 slots)', () => {
    const t = new TrailRing(100);
    const n = t.positions.length;
    for (let i = 0; i < 10000; ++i) {
        t.push(i * 0.01, Math.sin(i), false);
    }
    assert(t.count === 100, 'count ' + t.count);
    assert(t.positions.length === n && n === 600, 'buffer length ' + t.positions.length);
    const last = t.segment(99);
    near(last[2], 9999 * 0.01, 1e-4, 'newest end x');
    const first = t.segment(0);
    near(first[0], 9899 * 0.01, 1e-4, 'oldest start x');
});

test('trail breaks on restart: no segment joins across the break', () => {
    const t = new TrailRing(100);
    t.push(0, 0);
    t.push(1, 0);
    t.push(2, 0);
    t.push(5, 5, true);
    t.push(6, 5);
    assert(t.count === 3, 'count ' + t.count);
    assert(t.breaks === 1, 'breaks ' + t.breaks);
    for (let k = 0; k < t.count; ++k) {
        const [x0, y0, x1, y1] = t.segment(k);
        assert(!(x0 === 2 && y0 === 0 && x1 === 5 && y1 === 5), 'segment across the break');
    }
});

test('trail skips sub-millimeter repeats and non-finite points', () => {
    const t = new TrailRing(10);
    t.push(1, 1);
    t.push(1, 1);
    t.push(1.0004, 1);
    t.push(NaN, 1);
    assert(t.count === 0, 'count ' + t.count);
});

function pose(over) {
    return {
        type: 'pose', session: '5A3C19E2', epoch: 0, seq: 1, t_ms: 1000, x: 0, y: 0, yaw: 0,
        roll: 0, pitch: 0, attitude: false, restart: false, rx_ms: 5000, source: 'sim', ...over,
    };
}

function liveModel() {
    const m = new FeedModel(100);
    m.opened();
    m.handle({ type: 'hello', protocol: 1, stale_ms: 750, field: 'override', fields: ['override'] }, 0);
    m.handle({ type: 'status', rx_ms: 4990, link: 'open', stale: true, pose_age_ms: null }, 0);
    return m;
}

test('feed: trail joins a normal sequence and breaks on restart, new session, gap, reconnect', () => {
    const m = liveModel();
    m.handle(pose({ seq: 1, x: 0, restart: true, rx_ms: 5000 }), 10);
    m.handle(pose({ seq: 2, x: 0.1, rx_ms: 5100 }), 110);
    m.handle(pose({ seq: 3, x: 0.2, rx_ms: 5200 }), 210);
    assert(m.trail.count === 2, 'joined count ' + m.trail.count);
    m.handle(pose({ seq: 4, x: 1.0, epoch: 1, restart: true, rx_ms: 5300 }), 310);
    m.handle(pose({ seq: 5, x: 1.1, epoch: 1, rx_ms: 5400 }), 410);
    assert(m.trail.count === 3 && m.trail.breaks === 1, 'restart break');
    m.handle(pose({ seq: 1, x: 2.0, epoch: 1, session: '0BADF00D', rx_ms: 5500 }), 510);
    assert(m.trail.count === 3 && m.trail.breaks === 2, 'session break without restart flag');
    m.handle(pose({ seq: 2, x: 2.1, epoch: 1, session: '0BADF00D', rx_ms: 5600 }), 610);
    m.handle(pose({ seq: 3, x: 2.5, epoch: 1, session: '0BADF00D', rx_ms: 7000 }), 2010);
    assert(m.trail.count === 4 && m.trail.breaks === 3, 'gap break');
    m.closed();
    m.opened();
    m.handle(pose({ seq: 4, x: 2.6, epoch: 1, session: '0BADF00D', rx_ms: 7100 }), 2110);
    assert(m.trail.count === 4 && m.trail.breaks === 4, 'reconnect break');
});

// --- stale logic ---

test('feed state: disconnected, connecting, searching, waiting', () => {
    const m = new FeedModel(10);
    assert(m.view(0).state === 'disconnected', 'closed socket');
    m.opened();
    assert(m.view(0).state === 'connecting', 'no hello');
    m.handle({ type: 'hello', stale_ms: 750 }, 0);
    m.handle({ type: 'status', rx_ms: 100, link: 'searching', stale: true, pose_age_ms: null }, 0);
    assert(m.view(0).state === 'searching', 'link searching');
    m.handle({ type: 'status', rx_ms: 200, link: 'error', stale: true, pose_age_ms: null }, 0);
    assert(m.view(0).state === 'searching', 'link error');
    m.handle({ type: 'status', rx_ms: 300, link: 'open', stale: true, pose_age_ms: null }, 0);
    assert(m.view(0).state === 'waiting', 'no pose');
    m.closed();
    assert(m.view(0).state === 'disconnected', 'closed again');
});

test('feed state: live, then stale by age, by receiver status, and recovery', () => {
    const m = liveModel();
    m.handle(pose({ rx_ms: 5000, restart: true }), 1000);
    let v = m.view(1050);
    assert(v.state === 'live', 'fresh pose ' + v.state);
    near(v.ageMs, 50, 1e-9, 'local age before any newer status');
    m.handle({ type: 'status', rx_ms: 5100, link: 'open', stale: false, pose_age_ms: 100 }, 1100);
    v = m.view(1200);
    near(v.ageMs, 200, 1e-9, 'receiver age plus local time since status');
    assert(v.state === 'live');
    v = m.view(1900);
    assert(v.state === 'stale' && v.ageMs > 750, 'age past stale_ms ' + v.state + ' ' + v.ageMs);
    m.handle({ type: 'status', rx_ms: 5300, link: 'open', stale: true, pose_age_ms: 300 }, 1300);
    v = m.view(1310);
    assert(v.state === 'stale', 'receiver says stale');
    m.handle(pose({ seq: 2, rx_ms: 5400 }), 1400);
    v = m.view(1410);
    assert(v.state === 'live', 'a pose newer than the stale status is live: ' + v.state);
});

test('feed state: a stale old pose sent on connect stays stale', () => {
    const m = liveModel();
    m.handle(pose({ rx_ms: 1000 }), 100);
    m.handle({ type: 'status', rx_ms: 9000, link: 'open', stale: true, pose_age_ms: 8000 }, 101);
    const v = m.view(102);
    assert(v.state === 'stale', v.state);
    near(v.ageMs, 8001, 1e-9);
});

// --- smoothing ---

test('smoothing: off returns the received pose exactly; on blends within 100 ms', () => {
    const s = new Smoother(100);
    const a = presentPose(normalizePose({ x: 0, y: 0, yaw: 179 * DEG }));
    const b = presentPose(normalizePose({ x: 0.1, y: 0, yaw: -179 * DEG }));
    assert(s.update(a, 1, 0, false, false) === a, 'first pose');
    assert(s.update(b, 2, 10, false, false) === b, 'disabled');
    const t = new Smoother(100);
    t.update(a, 1, 0, true, false);
    const mid = t.update(b, 2, 1000, true, false);
    near(mid.x, 0, 1e-12, 'starts at the drawn pose');
    const half = t.current(1050);
    near(half.x, 0.05, 1e-12, 'halfway x');
    near(Math.abs(quaternionYaw(half.q)) / DEG, 180, 1e-6, 'halfway yaw short way');
    assert(t.current(1100) === b, 'done at 100 ms');
    const u = new Smoother(100);
    u.update(a, 1, 0, true, false);
    assert(u.update(b, 2, 1000, true, true) === b, 'snap');
});

test('overhead camera sits above the field center', () => {
    const c = overheadCamera({ inside_x_m: 3.5664, inside_y_m: 3.5664 }, 30, 1.6);
    near(c.target[0], 1.7832, 1e-12);
    near(c.target[1], 1.7832, 1e-12);
    assert(c.position[2] > 5, 'height ' + c.position[2]);
    assert(c.position[1] < c.target[1], 'a hair toward -Y');
});

test('age formatting', () => {
    assert(fmtAge(35) === '35 ms');
    assert(fmtAge(1234) === '1.2 s');
    assert(fmtAge(null) === '--');
});

// --- with three.js (the real scene code) ---

let THREE = null;
let sceneMod = null;
let OrbitControls = null;
let loadError = null;
try {
    THREE = await import('three');
    sceneMod = await import('../scene.js');
    ({ OrbitControls } = await import('../vendor/three/OrbitControls.js'));
} catch (e) {
    loadError = e;
}

function needThree() {
    if (loadError) {
        throw new Error('three.js or scene.js failed to load: ' + loadError);
    }
}

function overheadSetup(aspect) {
    const dims = { inside_x_m: 3.5664, inside_y_m: 3.5664 };
    const cam = new THREE.PerspectiveCamera(30, aspect, 0.02, 200);
    const controls = new OrbitControls(cam, document.createElement('div'));
    sceneMod.applyCameraPose(cam, controls, overheadCamera(dims, 30, aspect));
    cam.updateMatrixWorld(true);
    const ndc = (x, y, z = 0) => new THREE.Vector3(x, y, z).project(cam);
    return { cam, ndc, dims };
}

test('three: overhead view maps field +X to screen right and +Y to screen up', () => {
    needThree();
    const { ndc } = overheadSetup(1.6);
    const c = ndc(1.7832, 1.7832);
    const px = ndc(2.7832, 1.7832);
    const py = ndc(1.7832, 2.7832);
    near(c.x, 0, 1e-3, 'center x');
    near(c.y, 0, 1e-3, 'center y');
    assert(px.x - c.x > 0.1, '+X right: ' + px.x);
    near(px.y, c.y, 2e-3, '+X keeps screen y');
    assert(py.y - c.y > 0.1, '+Y up: ' + py.y);
    near(py.x, c.x, 2e-3, '+Y keeps screen x');
    const o = ndc(0, 0);
    assert(o.x < 0 && o.y < 0, 'origin bottom-left');
    for (const [x, y] of [[0, 0], [3.5664, 0], [0, 3.5664], [3.5664, 3.5664]]) {
        const p = ndc(x, y);
        assert(Math.abs(p.x) < 1 && Math.abs(p.y) < 1, `corner ${x},${y} on screen`);
    }
});

test('three: attitude quaternion equals Euler(roll, pitch, yaw, ZYX)', () => {
    needThree();
    for (const [r, p, y] of [[0.3, -0.2, 2.5], [-1.0, 0.7, -3.0], [0.1, 0.1, 0.1]]) {
        const q = attitudeQuaternion(r, p, y);
        const e = new THREE.Quaternion().setFromEuler(new THREE.Euler(r, p, y, 'ZYX'));
        const dot = Math.abs(q[0] * e.x + q[1] * e.y + q[2] * e.z + q[3] * e.w);
        near(dot, 1, 1e-12, 'same rotation');
    }
});

function robotAt(x, y, roll, pitch, yaw) {
    const dims = { front_m: 0.1685, back_m: 0.1685, left_m: 0.1465, right_m: 0.1465, height_m: 0.25 };
    const robot = sceneMod.buildRobot(dims);
    sceneMod.applyRobotPose(robot, x, y, attitudeQuaternion(roll, pitch, yaw));
    robot.updateMatrixWorld(true);
    const w = (px, py, pz = 0) => robot.localToWorld(new THREE.Vector3(px, py, pz));
    return { robot, dims, w };
}

test('three: robot model, roll +10 raises the left side, no vertical translation', () => {
    needThree();
    const { w, dims } = robotAt(1, 2, 10 * DEG, 0, 0);
    const o = w(0, 0, 0);
    nearVec([o.x, o.y, o.z], [1, 2, 0], 1e-12, 'origin stays on the floor');
    assert(w(0, dims.left_m).z > 0, 'left up');
    assert(w(0, -dims.right_m).z < 0, 'right down');
});

test('three: robot model, pitch +10 lowers the nose', () => {
    needThree();
    const { w, dims } = robotAt(1, 2, 0, 10 * DEG, 0);
    assert(w(dims.front_m, 0).z < 0, 'nose down');
    assert(w(-dims.back_m, 0).z > 0, 'back up');
    near(w(0, 0, 0).z, 0, 1e-12, 'no lift');
});

test('three: robot heading +90 points its nose up the overhead screen', () => {
    needThree();
    const { ndc } = overheadSetup(1.6);
    const { w, dims } = robotAt(1.7832, 1.7832, 0, 0, 90 * DEG);
    const nose = w(dims.front_m, 0);
    const tail = w(-dims.back_m, 0);
    const n = ndc(nose.x, nose.y, nose.z), t = ndc(tail.x, tail.y, tail.z);
    assert(n.y > t.y, 'nose above tail on screen');
    near(n.x, t.x, 1e-3, 'vertical on screen');
    const left = w(0, dims.left_m);
    assert(ndc(left.x, left.y).x < n.x, 'left side is screen left when facing +Y');
});

test('three: robot box size comes from front/back/left/right/height', () => {
    needThree();
    const dims = { front_m: 0.3, back_m: 0.1, left_m: 0.2, right_m: 0.05, height_m: 0.4 };
    const robot = sceneMod.buildRobot(dims);
    const box = robot.children.find((c) => c.isMesh && c.geometry.type === 'BoxGeometry');
    const p = box.geometry.parameters;
    near(p.width, 0.4, 1e-12, 'length along x');
    near(p.height, 0.25, 1e-12, 'width along y');
    near(p.depth, 0.4, 1e-12, 'height along z');
    near(box.position.x, 0.1, 1e-12, 'center x offset');
    near(box.position.y, 0.075, 1e-12, 'center y offset');
});

// --- run ---

const log = document.getElementById('log');
let failed = 0;
for (const [name, fn] of tests) {
    const line = document.createElement('div');
    try {
        await fn();
        line.textContent = 'PASS ' + name;
    } catch (e) {
        failed += 1;
        line.textContent = 'FAIL ' + name + ': ' + (e && e.message ? e.message : e);
        line.className = 'fail';
    }
    log.appendChild(line);
}
const summary = document.getElementById('summary');
summary.dataset.result = failed ? 'FAIL' : 'PASS';
summary.textContent = failed ? `FAIL ${failed} of ${tests.length}` : `PASS ${tests.length}/${tests.length}`;
