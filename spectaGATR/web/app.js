// app.js
// Page wiring. Messages only update FeedModel (feed.js); this file reads the
// model once per animation frame and updates the scene and the readouts.
// Readouts show received values; smoothing (off by default) changes only
// where the robot is drawn.
//
// URL options (for testing): ?view=overhead|orbit|follow, ?field=<name>,
// ?smooth=1.

import { FeedModel, connectFeed } from './feed.js';
import { FieldScene } from './scene.js';
import { DEG, M_TO_IN, Smoother, fmt, fmtAge, presentPose } from './transforms.js';

const STORE_KEY = 'spectagatr.field';
const VIEWS = ['overhead', 'orbit', 'follow'];
const params = new URLSearchParams(location.search);

function storeGet(k) {
    try {
        return localStorage.getItem(k);
    } catch (e) {
        return null;
    }
}

function storeSet(k, v) {
    try {
        localStorage.setItem(k, v);
    } catch (e) {
        // private mode or blocked storage: the choice is just not remembered
    }
}

const $ = (id) => document.getElementById(id);
const shown = new Map();

function setText(id, text) {
    if (shown.get(id) !== text) {
        shown.set(id, text);
        $(id).textContent = text;
    }
}

function setData(key, value) {
    const v = String(value);
    if (document.body.dataset[key] !== v) {
        document.body.dataset[key] = v;
    }
}

const model = new FeedModel();
const scene = new FieldScene($('view'));
const smoother = new Smoother();
setData('webgl', scene.available ? 'yes' : 'no');
window.spectagatr = { model, scene };   // for the console and automated checks
scene.setRobotDims(model.robot);

// --- fields ---

let fieldNames = [];
let apiDefault = null;
let loadedField = null;
let loadedTitle = '';
let loadingField = null;
let failedField = null;
let fieldNote = '';

function knownNames() {
    const names = new Set(fieldNames);
    if (model.hello && Array.isArray(model.hello.fields)) {
        for (const n of model.hello.fields) {
            if (typeof n === 'string') {
                names.add(n);
            }
        }
    }
    return [...names];
}

function chooseField() {
    const names = knownNames();
    const cands = [params.get('field'), storeGet(STORE_KEY), model.hello && model.hello.field, apiDefault, names[0]];
    for (const c of cands) {
        if (typeof c === 'string' && c && (names.length === 0 || names.includes(c))) {
            return c;
        }
    }
    return null;
}

function fillSelector() {
    const sel = $('field-select');
    const names = knownNames();
    const want = loadingField || loadedField || chooseField() || '';
    const key = names.join('|') + '#' + want;
    if (sel.dataset.key === key) {
        return;
    }
    sel.dataset.key = key;
    sel.textContent = '';
    if (names.length === 0) {
        sel.add(new Option('(no fields)', ''));
        return;
    }
    for (const n of names) {
        sel.add(new Option(n, n, false, n === want));
    }
}

async function loadFieldList() {
    try {
        const r = await fetch('api/fields', { cache: 'no-store' });
        if (!r.ok) {
            throw new Error('HTTP ' + r.status);
        }
        const j = await r.json();
        fieldNames = Array.isArray(j.fields) ? j.fields.filter((s) => typeof s === 'string') : [];
        apiDefault = typeof j.default === 'string' ? j.default : null;
        if (fieldNote.startsWith('field list unavailable')) {
            fieldNote = '';
        }
    } catch (e) {
        fieldNote = 'field list unavailable (' + (e && e.message ? e.message : e) + ')';
    }
    fillSelector();
    ensureField();
}

function validField(f) {
    const d = f && f.dimensions;
    return d && Number.isFinite(d.inside_x_m) && Number.isFinite(d.inside_y_m) &&
        d.inside_x_m > 0 && d.inside_y_m > 0 && (!f.objects || Array.isArray(f.objects));
}

async function loadField(name) {
    loadingField = name;
    fillSelector();
    try {
        const r = await fetch('fields/' + encodeURIComponent(name) + '.json', { cache: 'no-store' });
        if (!r.ok) {
            throw new Error('HTTP ' + r.status);
        }
        const f = await r.json();
        if (!validField(f)) {
            throw new Error('not a spectagatr-field/1 document');
        }
        if (loadingField !== name) {
            return;   // another choice replaced this one meanwhile
        }
        scene.setField(f);
        loadedField = name;
        loadedTitle = typeof f.title === 'string' ? f.title : name;
        fieldNote = '';
    } catch (e) {
        failedField = name;
        fieldNote = `field "${name}" failed to load (${e && e.message ? e.message : e})`;
    }
    if (loadingField === name) {
        loadingField = null;
    }
    fillSelector();
}

function ensureField() {
    const want = chooseField();
    if (want && want !== loadedField && want !== loadingField && want !== failedField) {
        loadField(want);
    }
}

$('field-select').addEventListener('change', (e) => {
    const name = e.target.value;
    if (name) {
        storeSet(STORE_KEY, name);
        failedField = null;
        loadField(name);
    }
});

// --- camera and options ---

function setView(mode) {
    scene.setView(mode);
    for (const b of document.querySelectorAll('button[data-view]')) {
        b.classList.toggle('active', b.dataset.view === mode);
    }
    setData('view', mode);
}

for (const b of document.querySelectorAll('button[data-view]')) {
    b.addEventListener('click', () => setView(b.dataset.view));
}
$('btn-reset').addEventListener('click', () => scene.resetView());
$('btn-clear').addEventListener('click', () => model.trail.clear());
const smoothBox = $('smooth');
smoothBox.checked = params.get('smooth') === '1';

if (VIEWS.includes(params.get('view'))) {
    setView(params.get('view'));
}

// --- banner and readouts ---

function banner(v) {
    const s = model.status;
    switch (v.state) {
    case 'disconnected':
        return ['bad', 'Receiver disconnected, retrying',
            `No connection to the spectaGATR receiver at ${location.host || '(no host)'}.` +
            (model.pose ? ' Last pose shown dimmed.' : '')];
    case 'connecting':
        return ['', 'Connecting to receiver', ''];
    case 'searching':
        return ['warn', s && s.link === 'error' ? 'VEX link error, searching for VEX device' : 'Searching for VEX device',
            (s && s.detail ? s.detail : '') + (model.pose ? ' Last pose shown dimmed.' : '')];
    case 'waiting':
        return ['', 'Waiting for first pose', s && s.detail ? s.detail : ''];
    case 'stale':
        return ['warn', 'STALE ' + fmtAge(v.ageMs),
            `No new pose within ${model.staleMs} ms. Robot frozen at its last received pose.`];
    default:
        return null;
    }
}

function statusCounts(c) {
    if (!c || typeof c !== 'object') {
        return '';
    }
    let dropped = 0;
    for (const k of ['bad_crc', 'malformed', 'out_of_range', 'repeated', 'out_of_order', 'truncated', 'too_long']) {
        if (typeof c[k] === 'number') {
            dropped += c[k];
        }
    }
    return `receiver: ${c.poses ?? '--'} poses, ${dropped} lines dropped, ${c.sessions ?? '--'} sessions`;
}

function updateDom(v) {
    setData('state', v.state);
    setData('poses', model.poses);
    setData('field', loadedField || '');
    setText('pill', v.state.toUpperCase());

    const b = banner(v);
    const el = $('banner');
    if (b) {
        el.hidden = false;
        if (el.dataset.kind !== b[0]) {
            el.dataset.kind = b[0];
        }
        setText('banner-title', b[1]);
        setText('banner-detail', b[2]);
    } else if (!el.hidden) {
        el.hidden = true;
    }

    const p = model.pose;
    const s = model.status;
    if (p) {
        setText('r-x', `${fmt(p.x, 3)} m   ${fmt(p.x * M_TO_IN, 2)} in`);
        setText('r-y', `${fmt(p.y, 3)} m   ${fmt(p.y * M_TO_IN, 2)} in`);
        setText('r-heading', `${fmt(p.yaw / DEG, 2)} deg`);
        setText('r-roll', p.attitude ? `${fmt(p.roll / DEG, 2)} deg` : 'heading only');
        setText('r-pitch', p.attitude ? `${fmt(p.pitch / DEG, 2)} deg` : 'heading only');
        setText('r-session', p.session || '--');
        setText('r-seq', `${p.epoch ?? '--'} / ${p.seq ?? '--'}`);
        setText('r-tms', p.t_ms === null ? '--' : `${p.t_ms} (publisher clock)`);
    }
    setText('r-source', (s && s.source) || (p && p.source) || '--');
    setText('r-port', (s && s.port) || '--');
    setText('r-link', !model.wsOpen ? 'receiver disconnected' : (s ? `${s.link}${s.detail ? ': ' + s.detail : ''}` : '--'));
    setText('r-age', v.ageMs === null ? '--' : fmtAge(v.ageMs) + ' (receiver-local)');
    setText('r-counts', `${model.poses} poses here, ${model.rejected} rejected here` +
        (s && s.counts ? '; ' + statusCounts(s.counts) : ''));
    setText('r-field', `field: ${loadedField ? loadedField + ' - ' + loadedTitle : '--'}${fieldNote ? '; ' + fieldNote : ''}`);
    const r = model.robot;
    setText('r-robot', `robot box (m): front ${r.front_m} back ${r.back_m} left ${r.left_m} right ${r.right_m} height ${r.height_m}`);
}

// --- frame loop ---

let lastConn = -1;
let lastHello = null;

function frame() {
    const now = performance.now();
    if (model.conn !== lastConn) {
        lastConn = model.conn;
        failedField = null;
        if (lastConn > 0) {
            loadFieldList();
        }
    }
    if (model.hello !== lastHello) {
        lastHello = model.hello;
        if (model.hello) {
            scene.setRobotDims(model.robot);
            fillSelector();
            ensureField();
        }
    }
    const v = model.view(now);
    const live = v.state === 'live';
    let drawn = null;
    if (model.pose) {
        drawn = smoother.update(presentPose(model.pose), model.poseSerial, now, smoothBox.checked,
            !live || model.pose.breakBefore);
    }
    scene.showRobot(drawn, live);
    scene.setTrail(model.trail);
    scene.render(now);
    updateDom(v);
    requestAnimationFrame(frame);
}

if (!scene.available) {
    $('view-note').hidden = false;
    $('view-note').textContent = '3D view unavailable (' + (scene.error || 'WebGL') +
        '). Readouts and states still work.';
}

if (location.protocol === 'file:') {
    $('banner').hidden = false;
    $('banner').dataset.kind = 'bad';
    $('banner-title').textContent = 'Open this page through the receiver';
    $('banner-detail').textContent = 'Start the spectaGATR receiver (or tools/mock_feed.py) and open the address it prints, default 127.0.0.1:8790.';
} else {
    loadFieldList();
    connectFeed(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`, model);
    requestAnimationFrame(frame);
}
