// app.js
// The inspector: one WebSocket to the runtime, documents dispatched to the
// 3D field, the camera panel and the diagnostics panel, and the status bar
// that says how much to trust what is on screen. The page never estimates:
// it draws hello, snapshot and frame documents and ages them by their own
// host_ms; elapsed browser receipt time advances displayed ages between
// documents and notices when updates stop arriving.
//
// For headless checks and scripts, #status carries data-state, data-snapshots,
// data-frames, data-session, data-cycle and data-errors, and window.__navigatr
// exposes counters, the last documents and the error list.

import { FieldScene } from './field_scene.js';
import { CameraPanel } from './camera_panel.js';
import { Diagnostics } from './diagnostics.js';
import { fmt, fmtMs } from './transforms.js';

const STALE_MS = 1000;
const RECONNECT_MIN_MS = 500;
const RECONNECT_MAX_MS = 5000;

const state = {
    snapshots: 0,
    frames: 0,
    lastSnapshot: null,
    lastFrameHeader: null,
    session: null,
    errors: [],
    hello: null,
    connection: 'connecting',
    lastReceiptMs: 0,
    webgl: false,
    reachedLive: false,   // true once the feed was live at least once
};
window.__navigatr = state;

const statusEl = document.getElementById('status');
const connEl = document.getElementById('conn');
const badgesEl = document.getElementById('badges');
const readoutEl = document.getElementById('readout');
const viewNote = document.getElementById('view-note');
const landmarkTable = document.getElementById('landmark-table');

function recordError(msg) {
    state.errors.push(String(msg));
    if (state.errors.length > 50) {
        state.errors.shift();
    }
    statusEl.dataset.errors = state.errors.join(' | ');
}
window.addEventListener('error', (e) => recordError(e.message || e));
window.addEventListener('unhandledrejection', (e) => recordError('unhandled rejection: ' + (e.reason && e.reason.message ? e.reason.message : e.reason)));

const scene = new FieldScene(document.getElementById('view'));
state.webgl = scene.available;
statusEl.dataset.webgl = scene.available ? '1' : '0';
if (!scene.available) {
    viewNote.hidden = false;
    viewNote.textContent = scene.error + ' (3D view disabled; documents still flow)';
}
const cameraPanel = new CameraPanel(document.getElementById('camera'));
const diagnostics = new Diagnostics(document.getElementById('diag'), sendJson);

document.getElementById('btn-reset').addEventListener('click', () => {
    scene.resetView();
    setFollowButton(false);
});
document.getElementById('btn-top').addEventListener('click', () => {
    scene.topDown();
    setFollowButton(false);
});
document.getElementById('btn-follow').addEventListener('click', () => {
    setFollowButton(!scene.followRobot);
});

function setFollowButton(on) {
    scene.setFollow(on);
    document.getElementById('btn-follow').classList.toggle('active', on);
}

// --- connection ---

let ws = null;
let reconnectAttempts = 0;
let reconnectTimer = null;

function wsUrl() {
    const proto = location.protocol === 'https:' ? 'wss://' : 'ws://';
    return proto + location.host + '/ws';
}

function connect() {
    reconnectTimer = null;
    setConnection('connecting');
    try {
        ws = new WebSocket(wsUrl());
    } catch (e) {
        recordError('websocket: ' + e);
        scheduleReconnect();
        return;
    }
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => {
        reconnectAttempts = 0;
    };
    ws.onmessage = (ev) => {
        if (typeof ev.data === 'string') {
            let doc;
            try {
                doc = JSON.parse(ev.data);
            } catch (e) {
                recordError('bad json: ' + e);
                return;
            }
            onDocument(doc);
        } else {
            onBinary(ev.data);
        }
    };
    ws.onclose = () => {
        ws = null;
        setConnection('disconnected');
        scheduleReconnect();
    };
    ws.onerror = () => {
        // the close event that follows carries the state change
    };
}

function scheduleReconnect() {
    if (reconnectTimer !== null) {
        return;
    }
    const delay = Math.min(RECONNECT_MAX_MS, RECONNECT_MIN_MS * Math.pow(2, reconnectAttempts));
    reconnectAttempts += 1;
    reconnectTimer = setTimeout(connect, delay);
}

function sendJson(obj) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        return false;
    }
    ws.send(JSON.stringify(obj));
    return true;
}

// --- documents ---

function sessionChanged(session) {
    return !state.session || state.session.id !== session.id || state.session.reset_count !== session.reset_count;
}

function clearIncompatible() {
    scene.clearSession();
    cameraPanel.clear();
}

function onDocument(doc) {
    if (doc.type === 'hello') {
        onHello(doc);
    } else if (doc.type === 'snapshot') {
        onSnapshot(doc);
    }
}

function onHello(doc) {
    if (doc.contract && state.hello && state.hello.contract !== doc.contract) {
        recordError(`contract changed: ${state.hello.contract} -> ${doc.contract}`);
    }
    const rebuild = !state.hello || sessionChanged(doc.session) || state.hello.configuration.digest !== doc.configuration.digest;
    if (sessionChanged(doc.session)) {
        clearIncompatible();
    }
    state.hello = doc;
    state.session = doc.session;
    statusEl.dataset.session = doc.session.id;
    if (rebuild) {
        scene.buildField(doc);
    }
    cameraPanel.setHello(doc);
    diagnostics.setHello(doc);
    updateStatus();
}

function onSnapshot(doc) {
    if (sessionChanged(doc.session)) {
        clearIncompatible();
        state.session = doc.session;
        statusEl.dataset.session = doc.session.id;
    }
    state.lastSnapshot = doc;
    state.snapshots += 1;
    state.lastReceiptMs = performance.now();
    statusEl.dataset.snapshots = String(state.snapshots);
    statusEl.dataset.cycle = String(doc.cycle);
    scene.updateSnapshot(doc, state.hello);
    cameraPanel.updateSnapshot(doc);
    diagnostics.updateSnapshot(doc);
    renderLandmarks(doc);
    setConnection('live');
    updateStatus();
}

function onBinary(buffer) {
    const view = new DataView(buffer);
    if (buffer.byteLength < 4) {
        recordError('short binary message');
        return;
    }
    const n = view.getUint32(0, true);
    if (4 + n > buffer.byteLength) {
        recordError('binary header length out of range');
        return;
    }
    let header;
    try {
        header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, n)));
    } catch (e) {
        recordError('bad frame header: ' + e);
        return;
    }
    const jpeg = new Uint8Array(buffer, 4 + n);
    state.frames += 1;
    state.lastFrameHeader = header;
    statusEl.dataset.frames = String(state.frames);
    cameraPanel.onFrame(header, jpeg);
}

// --- status ---

function setConnection(s) {
    state.connection = s;
    statusEl.dataset.state = s;
    connEl.textContent = s;
    connEl.className = 'badge ' + ({ live: 'ok', stale: 'warn', disconnected: 'bad', connecting: 'info' }[s] || 'info');
    if (s === 'live' && !state.reachedLive) {
        // sticky proof the feed reached live; a headless capture under
        // virtual time can dump the DOM during the fast-forward tail (when
        // the accelerated clock has outrun the real socket and the bar reads
        // stale), so scripts check this instead of the instantaneous state
        state.reachedLive = true;
        statusEl.dataset.liveSeen = '1';
    }
}

function badge(text, cls, key) {
    const b = document.createElement('span');
    b.className = 'badge ' + cls;
    b.textContent = text;
    if (key) {
        b.dataset.badge = key;
    }
    return b;
}

function updateStatus() {
    const snap = state.lastSnapshot;
    badgesEl.innerHTML = '';
    if (!snap) {
        readoutEl.textContent = state.hello ? `${state.hello.configuration.name || state.hello.configuration.id}` : '';
        return;
    }
    const r = snap.robot || {};
    const elapsed = Math.max(0, performance.now() - state.lastReceiptMs);
    const age = (value) => typeof value === 'number' ? value + elapsed : value;
    const att = r.attitude || {};
    if (!r.valid) {
        badgesEl.appendChild(badge('localization unavailable', 'bad', 'localization-unavailable'));
    }
    if (snap.localization && !snap.localization.clock_mapped) {
        badgesEl.appendChild(badge('clock unmapped', 'warn', 'clock-unmapped'));
    }
    if (att.valid) {
        badgesEl.appendChild(badge(`attitude ${fmt(att.roll_deg, 1)} / ${fmt(att.pitch_deg, 1)} deg, age ${fmtMs(age(att.age_ms))}`, 'ok', 'attitude'));
        statusEl.dataset.attitude = 'valid';
    } else if (att.assumed_level) {
        badgesEl.appendChild(badge('attitude assumed level', 'warn', 'attitude-assumed'));
        statusEl.dataset.attitude = 'assumed_level';
    } else {
        badgesEl.appendChild(badge('attitude unavailable', 'warn', 'attitude-unavailable'));
        statusEl.dataset.attitude = 'unavailable';
    }
    const fs = snap.field_snapshot || {};
    badgesEl.appendChild(badge(`field snapshot ${fs.status || 'n/a'}, age ${fmtMs(age(fs.age_ms))}`,
        fs.status === 'ok' ? 'info' : 'warn', 'field-snapshot'));
    const w = snap.workers || {};
    badgesEl.appendChild(badge(
        `est ${fmt((w.estimation || {}).rate_hz, 1)} Hz  field ${fmt((w.field || {}).rate_hz, 1)} Hz  ` +
        `insp ${fmt((w.inspection || {}).snapshot_rate_hz, 1)} snap/s ${fmt((w.inspection || {}).frame_rate_hz, 1)} img/s`,
        'info', 'rates'));
    if (state.hello && (state.hello.warnings || []).length) {
        badgesEl.appendChild(badge(`${state.hello.warnings.length} build warnings`, 'warn', 'warnings'));
    }
    const f = r.field || {};
    readoutEl.innerHTML = '';
    const parts = [
        `robot x <b>${fmt(f.x_m, 3)}</b> m  y <b>${fmt(f.y_m, 3)}</b> m  heading <b>${fmt(f.heading_deg, 1)}</b> deg`,
        `pose age ${fmtMs(age(r.age_ms))}`,
        `cycle ${snap.cycle}`,
        `session ${snap.session.id.slice(0, 8)}`,
    ];
    readoutEl.innerHTML = parts.join(' &middot; ');
}

function renderLandmarks(snap) {
    const objects = snap.field_objects || [];
    landmarkTable.innerHTML = '';
    const head = document.createElement('tr');
    for (const h of ['landmark', 'source', 'x m', 'y m', 'disp m', 'head err deg', 'age', 'valid']) {
        const th = document.createElement('th');
        th.textContent = h;
        head.appendChild(th);
    }
    landmarkTable.appendChild(head);
    for (const o of objects) {
        const tr = document.createElement('tr');
        const cells = [
            o.id, o.source, fmt(o.pose.x_m, 3), fmt(o.pose.y_m, 3),
            o.source === 'observed' ? fmt(o.displacement_m, 3) : '-',
            o.source === 'observed' ? fmt(o.heading_error_deg, 1) : '-',
            o.source === 'observed' ? fmtMs(o.age_ms) : 'nominal',
            o.valid ? 'yes' : 'no',
        ];
        for (const c of cells) {
            const td = document.createElement('td');
            td.textContent = c;
            tr.appendChild(td);
        }
        tr.title = `${o.id}: ${o.source}${o.observed ? ', observed this cycle' : ''}; last ${o.last_source || ''} ${o.last_feature || ''}`;
        tr.addEventListener('pointerenter', () => scene.setHighlight(o.id));
        tr.addEventListener('pointerleave', () => scene.setHighlight(null));
        landmarkTable.appendChild(tr);
    }
}

// staleness by the browser's receipt clock, never by document host_ms
setInterval(() => {
    updateStatus();
    cameraPanel.updateAge();
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        return;
    }
    if (state.lastSnapshot && performance.now() - state.lastReceiptMs > STALE_MS && state.connection === 'live') {
        setConnection('stale');
    }
}, 250);

connect();

// Hold the document's load event until the first documents arrived, bounded:
// a module script is deferred, so this top-level await delays load, and a
// headless --dump-dom (which captures at load) then sees a live page with
// snapshots and a frame instead of the empty shell. People see the page
// render regardless; only the tab's loading indicator lasts a moment longer.
await new Promise((resolve) => {
    const started = performance.now();
    const tick = () => {
        const ready = state.snapshots >= 10 && state.frames >= 1;
        if (ready || performance.now() - started > 6000) {
            resolve();
        } else {
            setTimeout(tick, 100);
        }
    };
    tick();
});
