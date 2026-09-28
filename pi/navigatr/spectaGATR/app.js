// app.js
// spectaGATR, the inspector page. One WebSocket (feed.js) whose handlers
// only parse and store; one requestAnimationFrame loop that applies the
// newest stored messages to the 3D scene and renders it, independent of
// when messages arrive; DOM panels refreshed at most PANEL_HZ times a
// second and only while visible (a hidden tab also unsubscribes from what
// only it shows). The page never estimates: it draws what the runtime
// reported, ages it on the right clock, and says when it is stale.
//
// Replay (replay.js) swaps the scene and graphs over to a capture bundle
// behind a REPLAY banner; the live connection keeps running unrendered and
// replay has no path to send anything.
//
// For headless checks and scripts, #status carries data-state,
// data-snapshots (state messages received), data-frames, data-session,
// data-cycle, data-errors, data-readiness, data-attitude (valid,
// assumed_level, stale or unavailable), data-attitude-status (the inspect/2
// status), data-contract, data-mode (live or replay), data-trail (trail
// points) and data-live-seen; window.__navigatr exposes counters, the
// newest documents, the error list and metrics(). ?perf adds
// performance.mark/measure entries per message and per frame.

import { FieldScene, TRAIL_MAX } from './field_scene.js';
import { CameraPanel } from './camera_panel.js';
import { Diagnostics } from './diagnostics.js';
import { LinkPanel, readinessOf } from './link_panel.js';
import { Feed } from './feed.js';
import { PoseSmoother, STALE_MS as POSE_STALE_MS } from './smoothing.js';
import { SeriesStore } from './series.js';
import { LiveCollector, GraphPanel } from './graphs.js';
import { InstrumentationPanel } from './instrumentation_panel.js';
import { LatencyPanel } from './latency_panel.js';
import { CapturePanel } from './capture_panel.js';
import { ReplayModel, ReplayClock } from './replay.js';
import { Stat } from './latency.js';
import { el, BadgeSet, KeyedTable, setText, setClass } from './dom.js';
import { fmt, fmtMs, attitudeStatus } from './transforms.js';

const PANEL_HZ = 10;
const TELEMETRY_FRESH_MS = 1000;
const TRAIL_QUEUE = 1024;
const PERF = new URLSearchParams(location.search).has('perf');

// --- measurements (window.__navigatr.metrics()) ---

const m = {
    frameInterval: new Stat(600),
    frameWork: new Stat(600),
    panelWork: new Stat(200),
    receiveToRender: new Stat(300),
    publishToReceive: new Stat(300),
    sourceAge: new Stat(300),
    smoothLag: new Stat(300),
    rtt: new Stat(60),
    frames: 0,
    renders: 0,
    skipped: 0,   // vsyncs left out by the render budget
    smoothing: true,
};

const state = {
    snapshots: 0,
    frames: 0,
    lastSnapshot: null,
    lastDiag: null,
    lastFrameHeader: null,
    session: null,
    errors: [],
    hello: null,
    connection: 'connecting',
    webgl: false,
    reachedLive: false,
    mode: 'live',
};
window.__navigatr = state;

const statusEl = document.getElementById('status');
const connEl = document.getElementById('conn');
const readoutEl = document.getElementById('readout');
const viewNote = document.getElementById('view-note');

function recordError(msg) {
    state.errors.push(String(msg));
    if (state.errors.length > 50) {
        state.errors.shift();
    }
    statusEl.dataset.errors = state.errors.join(' | ');
}
window.addEventListener('error', (e) => recordError(e.message || e));
window.addEventListener('unhandledrejection', (e) => recordError('unhandled rejection: ' + (e.reason && e.reason.message ? e.reason.message : e.reason)));

// --- parts ---

const scene = new FieldScene(document.getElementById('view'));
state.webgl = scene.available;
statusEl.dataset.webgl = scene.available ? '1' : '0';
if (!scene.available) {
    viewNote.hidden = false;
    viewNote.textContent = scene.error + ' (3D view disabled; documents still flow)';
}
const smoother = new PoseSmoother();
const cameraPanel = new CameraPanel(document.getElementById('camera'));
const liveStore = new SeriesStore();
const collector = new LiveCollector(liveStore);

// trail points of every received state, drained each frame: x, y, t, epoch
const trailQueue = new Float64Array(TRAIL_QUEUE * 4);
let trailQueueN = 0;

const feed = new Feed({
    perf: PERF,
    onError: recordError,
    onFrame: (header, jpeg) => {
        state.frames = feed.frames;
        state.lastFrameHeader = header;
        cameraPanel.onFrame(header, jpeg);
    },
    onSession: (newProcess) => {
        // the Pi host clock restarts with a new process: old samples cannot
        // share an axis with new ones; a reset only breaks the lines
        if (newProcess) {
            collector.clear();
        } else {
            collector.breakAll();
        }
        trailQueueN = 0;
        sessionDirty = true;
    },
    onStored: (type, doc, now) => {
        if (type === 'state') {
            storedState(doc, now);
        } else if (type === 'diag') {
            collector.onDiag(doc);
        } else if (type === 'telemetry') {
            collector.onTelemetry(doc);
        } else if (type === 'instrumentation') {
            collector.onInstrumentation(doc, typeof doc.host_ms === 'number' ? doc.host_ms : piNow(now));
        } else if (type === 'pong') {
            const s = feed.clock.samples[feed.clock.samples.length - 1];
            m.rtt.push(s.rtt);
        }
    },
});
state.feed = feed;
state.sent = feed.sentTypes;

let lastPublication = null;

// Per state message, in the handler: numbers only.
function storedState(doc, now) {
    collector.onState(doc);
    const r = doc.robot;
    const pub = typeof doc.publication === 'number' ? doc.publication : doc.host_ms;
    const advanced = pub !== lastPublication;
    lastPublication = pub;
    if (!r || !advanced) {
        return;   // a keepalive repeats the last measurement
    }
    smoother.noteArrival(now);
    if (typeof r.age_ms === 'number') {
        m.sourceAge.push(r.age_ms);
    }
    const sent = typeof doc.host_us === 'number' ? doc.host_us / 1000 : doc.host_ms;
    const recvPi = feed.clock.toPi(now);
    if (Number.isFinite(recvPi) && typeof sent === 'number') {
        m.publishToReceive.push(recvPi - sent);
    }
    if (r.valid && typeof r.measured_at_host_ms === 'number' && r.odom && trailQueueN < TRAIL_QUEUE) {
        const k = trailQueueN * 4;
        trailQueue[k] = r.odom.x_m;
        trailQueue[k + 1] = r.odom.y_m;
        trailQueue[k + 2] = r.measured_at_host_ms;
        trailQueue[k + 3] = r.odometry_epoch;
        trailQueueN += 1;
    }
}

// The Pi host clock now: through the ping offset when known, else from
// the newest state's build time plus the browser time since it arrived.
function piNow(nowMs) {
    const t = feed.clock.toPi(nowMs);
    if (Number.isFinite(t)) {
        return t;
    }
    const s = feed.state;
    return s && typeof s.host_ms === 'number' ? s.host_ms + (nowMs - feed.stateArrival) : NaN;
}

const diagnostics = new Diagnostics(document.getElementById('tab-diag'), (msg) => feed.send(msg));
const linkPanel = new LinkPanel(document.getElementById('link'));
const graphPanel = new GraphPanel(document.getElementById('tab-graphs'), { onSeek: (t) => replay && replay.clock.seek(t) });
graphPanel.setSource(liveStore, 'live');
graphPanel.setColumns([]);
const instPanel = new InstrumentationPanel(document.getElementById('tab-inst'), { onOptions: () => updateSubscription() });
const latencyPanel = new LatencyPanel(document.getElementById('tab-latency'));
const capturePanel = new CapturePanel(document.getElementById('tab-capture'), { onReplay: (buf, label) => startReplay(buf, label) });
cameraPanel.piNow = () => piNow(performance.now());
const badges = new BadgeSet(document.getElementById('badges'));
const landmarkTable = new KeyedTable(document.getElementById('landmark-table'),
    ['landmark', 'source', 'x m', 'y m', 'disp m', 'head err deg', 'age', 'valid']);

// --- view controls ---

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
document.getElementById('btn-planning').addEventListener('click', (e) => {
    scene.setPlanningVisible(!scene.showPlanning);
    e.target.classList.toggle('active', scene.showPlanning);
});
document.getElementById('btn-smooth').addEventListener('click', (e) => {
    smoother.setEnabled(!smoother.enabled);
    m.smoothing = smoother.enabled;
    e.target.classList.toggle('active', smoother.enabled);
    statusEl.dataset.smoothing = smoother.enabled ? '1' : '0';
});

function setFollowButton(on) {
    scene.setFollow(on);
    document.getElementById('btn-follow').classList.toggle('active', on);
}

// --- dock: tabs, size, collapse, and what each visible part subscribes to ---

const dock = document.getElementById('diag');
let activeTab = 'diag';
const tabButtons = [...document.querySelectorAll('#dock-tabs button')];
for (const b of tabButtons) {
    b.addEventListener('click', () => {
        activeTab = b.dataset.tab;
        for (const x of tabButtons) {
            x.classList.toggle('active', x === b);
        }
        for (const t of document.querySelectorAll('#diag-body > .tab')) {
            t.hidden = t.id !== 'tab-' + activeTab;
        }
        if (dock.classList.contains('collapsed')) {
            setCollapsed(false);
        }
        graphPanel.dirty = true;
        panelsDue = true;
        if (activeTab === 'capture') {
            capturePanel.fetchStatus();
        }
        updateSubscription();
    });
}
document.getElementById('diag-toggle').addEventListener('click', () => setCollapsed(!dock.classList.contains('collapsed')));
document.getElementById('dock-size').addEventListener('click', (e) => {
    dock.classList.toggle('tall');
    e.target.textContent = dock.classList.contains('tall') ? 'shorter' : 'taller';
    graphPanel.dirty = true;
});

function setCollapsed(on) {
    dock.classList.toggle('collapsed', on);
    document.getElementById('diag-toggle').textContent = on ? 'show' : 'hide';
    updateSubscription();
}

function tabVisible(tab) {
    return activeTab === tab && !dock.classList.contains('collapsed') && !document.hidden;
}

function updateSubscription() {
    const inst = tabVisible('inst');
    const opts = instPanel.options();
    feed.subscribe({
        // live graphs plot encoder counts from it; replay graphs do not
        instrumentation: inst || (tabVisible('graphs') && mode === 'live'),
        raw: inst && opts.raw,
        decoded: inst && opts.decoded,
    });
}

// side panels scrolled out of view are not refreshed
const sideVisible = new Map([['link', true], ['camera', true], ['landmarks', true]]);
if (typeof IntersectionObserver !== 'undefined') {
    const io = new IntersectionObserver((entries) => {
        for (const e of entries) {
            sideVisible.set(e.target.id, e.isIntersecting);
        }
    }, { root: document.getElementById('side') });
    for (const id of sideVisible.keys()) {
        io.observe(document.getElementById(id));
    }
}

document.addEventListener('visibilitychange', () => {
    updateSubscription();
    // back from a hidden tab: refill the trail the paused frames missed
    if (!document.hidden && mode === 'live') {
        feed.requestHistory();
    }
});

// --- applying stored messages, once per animation frame ---

let mode = 'live';
let sessionDirty = false;
let panelsDue = true;
const seen = { hello: 0, state: 0, diag: 0, telemetry: -1, history: 0, connection: 0, epochRequested: null };
const ident = { session: '', reset: 0, epoch: 0, anchor: 0, placement: 0, placed: false, valid: false, connection: 0 };
let renderArrival = 0;
let lastStale = null;
let telFresh = false;
let telPathKey = '';

function poseStale(now) {
    const s = feed.state;
    if (!s || !s.robot) {
        return true;
    }
    const since = now - feed.lastStateArrival;
    if (since > POSE_STALE_MS) {
        return true;
    }
    const age = s.robot.age_ms;
    return typeof age === 'number' && age + since > POSE_STALE_MS;
}

function applyFeed(now) {
    if (sessionDirty) {
        sessionDirty = false;
        if (mode === 'live') {
            scene.clearSession();
        }
        cameraPanel.clear();
        smoother.reset();
        landmarkTable.clear();
        lastStale = null;
        state.session = feed.session;
        statusEl.dataset.session = feed.session ? feed.session.id : '';
    }
    if (feed.connectionEpoch !== seen.connection) {
        seen.connection = feed.connectionEpoch;
        collector.breakAll();
    }
    if (feed.helloVersion !== seen.hello) {
        seen.hello = feed.helloVersion;
        onHello(feed.hello);
    }
    if (mode !== 'live') {
        feed.history.length = 0;
        trailQueueN = 0;
        return;
    }
    const s = feed.state;
    const robot = s ? s.robot : null;
    const epoch = robot ? robot.odometry_epoch : null;
    if (feed.history.length) {
        for (const b of feed.history.splice(0)) {
            if (scene.available) {
                scene.trailFromHistory(b.trail, b.replace, epoch);
            }
        }
    }
    for (let k = 0; k < trailQueueN && scene.available; ++k) {
        const q = k * 4;
        if (scene.trailEpoch !== trailQueue[q + 3]) {
            scene.clearTrail();
            scene.trailEpoch = trailQueue[q + 3];
        }
        if (trailQueue[q + 2] > scene.trailLastT) {
            scene.trailPush(trailQueue[q], trailQueue[q + 1], trailQueue[q + 2]);
        }
    }
    trailQueueN = 0;
    if (feed.stateVersion !== seen.state && s) {
        seen.state = feed.stateVersion;
        onState(s, now);
    }
    if (feed.diagVersion !== seen.diag && feed.diag) {
        seen.diag = feed.diagVersion;
        state.lastDiag = feed.diag;
        scene.applyDiag(feed.diag, feed.hello, now);
        cameraPanel.updateDiag(feed.diag);
        panelsDue = true;
    }
    const fresh = telemetryAge(now) <= TELEMETRY_FRESH_MS;
    if (feed.telemetryVersion !== seen.telemetry || fresh !== telFresh || scene.pathKey !== telPathKey) {
        seen.telemetry = feed.telemetryVersion;
        telFresh = fresh;
        telPathKey = scene.pathKey;
        scene.applyTelemetry(feed.telemetry, fresh);
    }
}

// Age of the newest telemetry report: its age when the Pi built the
// message (Pi clock) plus the time since this browser received it.
function telemetryAge(now) {
    const t = feed.telemetry;
    if (!t) {
        return Infinity;
    }
    return (typeof t.age_ms === 'number' ? t.age_ms : 0) + (now - feed.telemetryArrival);
}

function onHello(doc) {
    const prev = state.hello;
    const rebuild = !prev || !prev.session || prev.session.id !== doc.session.id ||
        prev.session.reset_count !== doc.session.reset_count ||
        (prev.configuration || {}).digest !== (doc.configuration || {}).digest;
    state.hello = doc;
    statusEl.dataset.session = doc.session.id;
    statusEl.dataset.contract = doc.contract || '';
    if (rebuild) {
        scene.buildField(doc);
    }
    cameraPanel.setHello(doc);
    diagnostics.setHello(doc);
    updateSubscription();
    panelsDue = true;
}

function onState(s, now) {
    const r = s.robot;
    state.lastSnapshot = s;
    state.snapshots = feed.stateCount;
    if (!r) {
        return;
    }
    const stale = poseStale(now);
    lastStale = stale;
    scene.applyRobotState(r, stale);
    if (!r.valid || !r.field) {
        smoother.reset();   // the next valid pose is drawn exactly, never blended into
        return;
    }
    const sess = s.session || feed.session || {};
    ident.session = sess.id;
    ident.reset = sess.reset_count;
    ident.epoch = r.odometry_epoch;
    ident.anchor = r.anchor_revision;
    ident.placement = r.placement_sequence;
    ident.placed = !!r.initialized;
    ident.valid = !!r.valid;
    ident.connection = feed.connectionEpoch;
    if (smoother.push(r.field, ident, feed.stateArrival, stale) === '') {
        m.smoothLag.push(smoother.dur);
    } else {
        m.smoothLag.push(0);
    }
    if (r.field_from_odom) {
        scene.setTrailFrame(r.field_from_odom);
    }
    // a new odometry epoch: backfill what coalescing dropped, once per epoch
    if (seen.epochRequested !== r.odometry_epoch) {
        if (seen.epochRequested !== null) {
            feed.requestHistory();
        }
        seen.epochRequested = r.odometry_epoch;
    }
    renderArrival = feed.stateArrival;
}

function liveRobot(now) {
    const s = feed.state;
    const r = s ? s.robot : null;
    if (!r || !r.valid || !r.field) {
        return;
    }
    const stale = poseStale(now);
    if (stale !== lastStale) {
        lastStale = stale;
        scene.applyRobotState(r, stale);
    }
    scene.placeRobot(smoother.sample(now, stale));
}

// --- replay ---

let replay = null;
const replayBar = document.getElementById('replay-bar');
const replayPlay = document.getElementById('replay-play');
const replaySpeed = document.getElementById('replay-speed');
const replayScrub = document.getElementById('replay-scrub');
const replayTime = document.getElementById('replay-time');
const replayMarks = document.getElementById('replay-marks');
let scrubbing = false;

async function startReplay(buffer, label) {
    let model;
    try {
        model = await ReplayModel.fromZip(buffer, label);
    } catch (e) {
        capturePanel.say('cannot replay: ' + (e && e.message ? e.message : e), true);
        return;
    }
    replay = { model, clock: new ReplayClock(model), seenVersion: -1, lastIndex: -1, lastSegment: -1, lastPath: undefined };
    mode = 'replay';
    state.mode = 'replay';
    statusEl.dataset.mode = 'replay';
    document.body.classList.add('replaying');
    scene.setMode('replay');
    smoother.reset();
    graphPanel.setSource(model.store, 'replay');
    graphPanel.setColumns(model.columns);
    diagnostics.setLiveControls(false);
    capturePanel.setLive(false);
    replayBar.hidden = false;
    const sum = model.summary();
    setText(document.getElementById('replay-label'), `${label}: ${fmt(sum.duration_s, 1)} s`);
    setText(document.getElementById('replay-info'), `replaying ${label}: ${sum.streams}` +
        (model.warnings.length ? `. Warnings: ${model.warnings.join('; ')}` : ''));
    replayPlay.textContent = 'play';
    replaySpeed.value = '1';
    drawReplayMarks();
    updateSubscription();
    panelsDue = true;
}

function exitReplay() {
    if (!replay) {
        return;
    }
    replay = null;
    mode = 'live';
    state.mode = 'live';
    statusEl.dataset.mode = 'live';
    document.body.classList.remove('replaying');
    scene.setMode('live');
    smoother.reset();
    graphPanel.setSource(liveStore, 'live');
    graphPanel.removeColumnPlots();
    graphPanel.setColumns([]);
    diagnostics.setLiveControls(true);
    capturePanel.setLive(true);
    replayBar.hidden = true;
    setText(document.getElementById('replay-info'), '');
    // re-apply the newest live documents and refill the trail
    seen.state = -1;
    seen.diag = -1;
    seen.telemetry = -1;
    lastStale = null;
    scene.trailEpoch = null;
    feed.requestHistory();
    updateSubscription();
    panelsDue = true;
}

replayPlay.addEventListener('click', () => {
    if (replay) {
        replay.clock.play(!replay.clock.playing);
    }
});
replaySpeed.addEventListener('change', () => {
    if (replay) {
        replay.clock.speed = Number(replaySpeed.value) || 1;
    }
});
replayScrub.addEventListener('input', () => {
    scrubbing = true;
    if (replay) {
        const mdl = replay.model;
        replay.clock.seek(mdl.startT + (Number(replayScrub.value) / 10000) * (mdl.endT - mdl.startT));
    }
});
replayScrub.addEventListener('change', () => {
    scrubbing = false;
});
document.getElementById('replay-exit').addEventListener('click', exitReplay);
replayMarks.addEventListener('click', (e) => {
    if (!replay) {
        return;
    }
    const rect = replayMarks.getBoundingClientRect();
    const mdl = replay.model;
    const span = mdl.endT - mdl.startT;
    let best = null;
    let bestPx = 7;
    for (const ev of mdl.events) {
        const px = ((ev.t - mdl.startT) / span) * rect.width;
        const d = Math.abs(px - (e.clientX - rect.left));
        if (d < bestPx) {
            bestPx = d;
            best = ev;
        }
    }
    replay.clock.seek(best ? best.t : mdl.startT + ((e.clientX - rect.left) / rect.width) * span);
});
replayMarks.addEventListener('pointermove', (e) => {
    if (!replay) {
        return;
    }
    const rect = replayMarks.getBoundingClientRect();
    const mdl = replay.model;
    const span = mdl.endT - mdl.startT;
    const near = mdl.events.find((ev) => Math.abs(((ev.t - mdl.startT) / span) * rect.width - (e.clientX - rect.left)) < 7);
    replayMarks.title = near ? `${fmt((near.t - mdl.startT) / 1000, 2)} s: ${near.text}` : 'events: click a marker to go there';
});
document.addEventListener('keydown', (e) => {
    if (replay && e.code === 'Space' && !/INPUT|SELECT|TEXTAREA|BUTTON/.test(document.activeElement.tagName)) {
        e.preventDefault();
        replay.clock.play(!replay.clock.playing);
    }
});

function drawReplayMarks() {
    const c = replayMarks;
    const w = Math.max(100, c.clientWidth | 0);
    const h = 10;
    c.width = w;
    c.height = h;
    const ctx = c.getContext('2d');
    ctx.clearRect(0, 0, w, h);
    if (!replay) {
        return;
    }
    const mdl = replay.model;
    const span = mdl.endT - mdl.startT;
    for (const ev of mdl.events) {
        const x = Math.round(((ev.t - mdl.startT) / span) * (w - 1)) + 0.5;
        ctx.fillStyle = /segment/.test(ev.text) ? '#ff5252' : '#ffb74d';
        ctx.fillRect(x - 1, 0, 2, h);
    }
}

function replayFrame(now) {
    const r = replay;
    r.clock.tick(now);
    if (r.clock.version === r.seenVersion) {
        return;
    }
    r.seenVersion = r.clock.version;
    const mdl = r.model;
    const t = r.clock.t;
    const robot = mdl.robotAt(t);
    if (robot) {
        scene.applyRobotState(robot, false);
        if (robot.valid) {
            scene.placeRobot(robot.field);
        }
        if (r.clock.jumped || robot.segment !== r.lastSegment || robot.index < r.lastIndex) {
            scene.clearTrail();
            for (const [x, y, tt] of mdl.trailPoints(robot.index, TRAIL_MAX).points) {
                scene.trailPush(x, y, tt);
            }
        } else if (robot.index > r.lastIndex) {
            for (const [x, y, tt] of mdl.trailSlice(r.lastIndex + 1, robot.index)) {
                scene.trailPush(x, y, tt);
            }
        }
        scene.setTrailFrame(mdl.robot.ox ? mdl.anchorOf(robot) : null);
        r.lastIndex = robot.index;
        r.lastSegment = robot.segment;
    } else {
        scene.applyRobotState(null, false);
        scene.clearTrail();
        r.lastIndex = -1;
    }
    r.clock.jumped = false;
    state.replayRobot = robot;
    const p = mdl.pathAt(t);
    if (p !== r.lastPath) {
        r.lastPath = p;
        scene.updatePath(p);
    }
    const tel = mdl.telemetryAt(t);
    scene.applyTelemetry(tel, !!tel && tel.age_ms <= TELEMETRY_FRESH_MS);
    if (!scrubbing) {
        replayScrub.value = String(Math.round(((t - mdl.startT) / (mdl.endT - mdl.startT)) * 10000));
    }
    replayPlay.textContent = r.clock.playing ? 'pause' : 'play';
    setText(replayTime, `${fmt((t - mdl.startT) / 1000, 2)} / ${fmt((mdl.endT - mdl.startT) / 1000, 2)} s`);
    graphPanel.dirty = true;
}

// --- panels, at most PANEL_HZ and only what is visible ---

let lastPanels = 0;
let liveEvents = [];
let liveEventsVersion = -1;

function graphEvents() {
    if (replay) {
        return replay.model.events;
    }
    if (feed.eventsVersion !== liveEventsVersion) {
        liveEventsVersion = feed.eventsVersion;
        liveEvents = feed.events.map((e) => ({ t: e.host_ms, text: e.text }));
    }
    return liveEvents;
}

function panels(now) {
    const diag = feed.diag;
    renderStatus(now);
    if (!document.hidden) {
        const linkEl = linkPanel.root;
        const wanted = linkPanel.wanted(diag, feed.events);
        if (linkEl.hidden === wanted) {
            linkEl.hidden = !wanted;
        }
        if (wanted && sideVisible.get('link')) {
            linkPanel.render({ robot: feed.state ? feed.state.robot : null, brain_link: diag ? diag.brain_link : null },
                diag, feed.hello, feed.events, piNow(now));
        }
        if (sideVisible.get('camera')) {
            cameraPanel.render();
        }
        if (sideVisible.get('landmarks') && diag) {
            renderLandmarks(diag);
        }
        diagnostics.renderSummary(diag);
        if (tabVisible('diag') && diag) {
            diagnostics.render(diag);
        }
        if (tabVisible('inst')) {
            instPanel.render(feed.instrumentation, now - feed.instrumentationArrival,
                feed.hello ? feed.contract2 : null);
        }
        if (tabVisible('latency')) {
            m.renders = scene.renders;
            latencyPanel.render(m, feed, diag);
        }
        if (tabVisible('capture')) {
            capturePanel.poll(now);
            capturePanel.render();
        }
    }
}

function renderLandmarks(diag) {
    const t = landmarkTable;
    t.begin();
    const objects = diag.field_objects || [];
    const onCreate = (id) => (tr) => {
        tr.addEventListener('pointerenter', () => scene.setHighlight(id));
        tr.addEventListener('pointerleave', () => scene.setHighlight(null));
    };
    const estimated = new Set();
    for (const o of objects) {
        estimated.add(o.id);
        t.row('o:' + o.id, [
            o.id, o.source, fmt(o.pose.x_m, 3), fmt(o.pose.y_m, 3),
            o.source === 'observed' ? fmt(o.displacement_m, 3) : '-',
            o.source === 'observed' ? fmt(o.heading_error_deg, 1) : '-',
            o.source === 'observed' ? fmtMs(typeof o.age_ms === 'number' ? Math.round(o.age_ms / 100) * 100 : o.age_ms) : 'nominal',
            o.valid ? 'yes' : 'no',
        ], {
            onCreate: onCreate(o.id),
            title: `${o.id}: ${o.source}${o.observed ? ', observed this cycle' : ''}; last ${o.last_source || ''} ${o.last_feature || ''}`,
        });
    }
    // configured geometry with no estimate at all, e.g. world estimation off
    for (const field of (feed.hello && feed.hello.fields) || []) {
        for (const lm of field.landmarks || []) {
            if (estimated.has(lm.id)) {
                continue;
            }
            t.row('n:' + lm.id, [lm.id, 'nominal (map)', fmt(lm.nominal.x_m, 3), fmt(lm.nominal.y_m, 3), '-', '-', 'no estimate', 'no'],
                { onCreate: onCreate(lm.id), title: `${lm.id}: configured map pose only, not an estimate` });
        }
    }
    t.end();
}

// readout spans, built once
const ro = {};
(() => {
    readoutEl.textContent = '';
    ro.label = el('span');
    ro.x = el('b');
    ro.y = el('b');
    ro.h = el('b');
    ro.rest = el('span');
    readoutEl.append(ro.label, ' x ', ro.x, ' m  y ', ro.y, ' m  heading ', ro.h, ' deg', ro.rest);
})();

const CONN_CLS = { live: 'ok', stale: 'warn', disconnected: 'bad', connecting: 'info' };
const DOT = ' ' + String.fromCharCode(0xb7) + ' ';   // a middle dot between readout fields

function renderStatus(now) {
    const conn = feed.connection;
    state.connection = conn;
    state.reachedLive = feed.reachedLive;
    statusEl.dataset.state = conn;
    if (feed.reachedLive && statusEl.dataset.liveSeen !== '1') {
        // sticky proof the feed reached live; a headless capture under
        // virtual time can dump the DOM during the fast-forward tail (when
        // the accelerated clock has outrun the real socket and the bar reads
        // stale), so scripts check this instead of the instantaneous state
        statusEl.dataset.liveSeen = '1';
    }
    setText(connEl, mode === 'replay' ? `live feed ${conn}` : conn);
    setClass(connEl, 'badge ' + (CONN_CLS[conn] || 'info'));
    statusEl.dataset.snapshots = String(feed.stateCount);
    statusEl.dataset.frames = String(feed.frames);
    statusEl.dataset.diags = String(feed.counts.get('diag') || feed.counts.get('snapshot') || 0);
    statusEl.dataset.trail = String(scene.trailCount || 0);
    const diag = feed.diag;
    const cycle = feed.state && feed.state.cycle !== undefined ? feed.state.cycle : (diag ? diag.cycle : '');
    statusEl.dataset.cycle = String(cycle);

    badges.begin();
    if (mode === 'replay') {
        badges.badge('replay', 'REPLAY: recorded data, not live', 'replay');
        renderReplayReadout();
        badges.end();
        return;
    }
    const s = feed.state;
    const r = s && s.robot ? s.robot : null;
    if (!r) {
        badges.end();
        setText(ro.label, feed.hello ? `${feed.hello.configuration.name || feed.hello.configuration.id}` : '');
        setText(ro.x, '');
        setText(ro.y, '');
        setText(ro.h, '');
        setText(ro.rest, '');
        return;
    }
    const since = Math.max(0, now - feed.stateArrival);
    const age = (value) => (typeof value === 'number' ? value + since : value);
    const view = { robot: r, brain_link: diag ? diag.brain_link : null };
    const ready = readinessOf(view, feed.hello);
    statusEl.dataset.readiness = ready ? ready.overall.state : '';
    if (ready) {
        badges.badge('readiness', ready.overall.state === 'ready' ? 'Brain link ready' : ready.overall.text, ready.overall.cls);
    }
    if (!r.valid) {
        badges.badge('localization-unavailable', 'localization unavailable', 'bad');
    } else if (!r.initialized) {
        badges.badge('unplaced', 'not placed: odometry frame, not a field position', 'warn');
    }
    statusEl.dataset.placement = r.valid && r.initialized ? 'placed' : 'unplaced';
    const stale = poseStale(now);
    if (r.valid && stale) {
        badges.badge('pose-stale', `pose stale: ${fmtMs(age(r.age_ms))} old, last state ${fmtMs(since)} ago`, 'bad');
    }
    const loc = s.localization || (diag ? diag.localization : null);
    if (loc && loc.clock_mapped === false) {
        badges.badge('clock-unmapped', 'clock unmapped', 'warn');
    }
    const att = r.attitude || {};
    const ast = attitudeStatus(att);
    statusEl.dataset.attitudeStatus = ast;
    if (ast === 'measured') {
        badges.badge('attitude', `attitude ${fmt(att.roll_deg, 1)} / ${fmt(att.pitch_deg, 1)} deg, age ${fmtMs(age(att.age_ms))}`, 'ok',
            `roll / pitch measured by ${att.source || 'the attitude source'}`);
        statusEl.dataset.attitude = 'valid';
    } else if (ast === 'assumed_level') {
        badges.badge('attitude-assumed', 'attitude assumed level', 'warn', 'no attitude source: the robot is drawn level by assumption, not measurement');
        statusEl.dataset.attitude = 'assumed_level';
    } else if (ast === 'stale') {
        badges.badge('attitude-stale', `attitude stale, age ${fmtMs(age(att.age_ms))}`, 'warn', 'the last measured attitude is too old to draw as current');
        statusEl.dataset.attitude = 'stale';
    } else {
        badges.badge('attitude-unavailable', 'attitude unavailable', 'warn');
        statusEl.dataset.attitude = 'unavailable';
    }
    if (diag) {
        const fs = diag.field_snapshot || {};
        const fsAge = typeof fs.age_ms === 'number' ? fs.age_ms + (now - feed.diagArrival) : fs.age_ms;
        badges.badge('field-snapshot', `field snapshot ${fs.status || 'n/a'}, age ${fmtMs(fsAge)}`, fs.status === 'ok' ? 'info' : 'warn');
        const w = diag.workers || {};
        badges.badge('rates',
            `est ${fmt((w.estimation || {}).rate_hz, 1)} Hz  field ${fmt((w.field || {}).rate_hz, 1)} Hz  ` +
            `recv ${fmt(feed.rates.rate(s.compat ? 'snapshot' : 'state'), 1)} state/s ${fmt(feed.rates.rate('frame'), 1)} img/s`,
            'info', 'estimation and field worker rates reported by the Pi; state and image rates measured in this browser');
    }
    const tel = feed.telemetry;
    if (tel && telemetryAge(now) > TELEMETRY_FRESH_MS * 3) {
        badges.badge('telemetry', `Brain telemetry stale ${fmtMs(telemetryAge(now))}`, 'warn', 'overlays from telemetry are hidden while it is stale');
    }
    const cap = feed.capture;
    if (cap && cap.active && cap.active.id && /record|trigger|final/.test(String(cap.active.state || cap.state || ''))) {
        badges.badge('capture', `capture ${cap.active.id} ${cap.active.state || cap.state}`, 'info');
    }
    if (feed.hello && (feed.hello.warnings || []).length) {
        badges.badge('warnings', `${feed.hello.warnings.length} build warnings`, 'warn');
    }
    badges.end();
    const placed = !!r.initialized;
    const f = (placed ? r.field : r.odom) || {};
    setText(ro.label, placed ? 'robot' : 'odometry (not placed)');
    setText(ro.x, fmt(f.x_m, 3));
    setText(ro.y, fmt(f.y_m, 3));
    setText(ro.h, fmt(f.heading_deg, 1));
    setText(ro.rest, `${DOT}pose age ${fmtMs(age(r.age_ms))}${DOT}${cycle !== '' ? 'cycle ' + cycle + DOT : ''}session ${String((feed.session || {}).id || '').slice(0, 8)}`);
}

function renderReplayReadout() {
    const r = state.replayRobot;
    const t = replay ? replay.clock.t : NaN;
    if (!r) {
        setText(ro.label, 'REPLAY: no robot state at this time');
        setText(ro.x, '');
        setText(ro.y, '');
        setText(ro.h, '');
        setText(ro.rest, '');
        return;
    }
    const f = (r.initialized ? r.field : r.odom || r.field) || {};
    setText(ro.label, `REPLAY ${r.initialized ? 'robot' : 'odometry (not placed)'}`);
    setText(ro.x, fmt(f.x_m, 3));
    setText(ro.y, fmt(f.y_m, 3));
    setText(ro.h, fmt(f.heading_deg, 1));
    setText(ro.rest, `${DOT}recorded at Pi ${fmt(t / 1000, 3)} s${DOT}segment ${r.segment + 1}${r.valid ? '' : DOT + 'NOT VALID'}`);
}

// --- the frame loop ---

// reused every frame: no per-frame allocation for the graph pass
const graphOpts = { nowT: NaN, playT: undefined, startT: undefined, endT: undefined, events: [], timeLabel: null };
const liveTimeLabel = (tt) => `${fmt(tt / 1000, 0)} s`;
const replayTimeLabel = (tt) => `${fmt((tt - replay.model.startT) / 1000, 1)} s`;

let lastFrameT = -1;
let workEma = 0;

// On a machine too slow for the scene, drawing every vsync starves the
// message handlers and the newest state waits behind frames. Frames that
// cost more than RENDER_BUDGET_MS are spaced so drawing takes at most about
// a third of the main thread; while the view is dragged every vsync is
// drawn. A fast machine never hits the cap.
const RENDER_BUDGET_MS = new URLSearchParams(location.search).has('nobudget') ? Infinity : 12;

function frame(ts) {
    requestAnimationFrame(frame);
    const gap = workEma > RENDER_BUDGET_MS && !scene.dragging ? Math.min(250, 3 * workEma) : 0;
    if (lastFrameT >= 0 && ts - lastFrameT < gap) {
        m.skipped += 1;
        return;
    }
    const t0 = performance.now();
    if (lastFrameT >= 0) {
        m.frameInterval.push(ts - lastFrameT);
    }
    lastFrameT = ts;
    m.frames += 1;
    feed.tick(t0);
    applyFeed(t0);
    if (mode === 'replay' && replay) {
        if (scene.available) {
            replayFrame(t0);
        } else {
            replay.clock.tick(t0);
        }
    } else {
        liveRobot(t0);
    }
    const rendered = scene.render();
    if (rendered) {
        m.renders = scene.renders;
        if (renderArrival) {
            m.receiveToRender.push(performance.now() - renderArrival);
            renderArrival = 0;
        }
    }
    if (panelsDue || t0 - lastPanels >= 1000 / PANEL_HZ) {
        panelsDue = false;
        lastPanels = t0;
        const p0 = performance.now();
        panels(t0);
        m.panelWork.push(performance.now() - p0);
    }
    if (tabVisible('graphs')) {
        const mdl = replay ? replay.model : null;
        const g = graphOpts;
        g.nowT = piNow(t0);
        g.playT = replay ? replay.clock.t : undefined;
        g.startT = mdl ? mdl.startT : undefined;
        g.endT = mdl ? mdl.endT : undefined;
        g.events = graphEvents();
        g.timeLabel = mdl ? replayTimeLabel : liveTimeLabel;
        graphPanel.draw(t0, g);
    }
    const dt = performance.now() - t0;
    m.frameWork.push(dt);
    workEma = workEma * 0.8 + dt * 0.2;
    if (PERF) {
        performance.measure('navigatr:frame', { start: t0, duration: dt });
    }
}

// Plain numbers for scripts (perf harness, browser tests).
state.metrics = () => {
    const out = {};
    for (const k of ['frameInterval', 'frameWork', 'panelWork', 'receiveToRender', 'publishToReceive', 'sourceAge', 'smoothLag', 'rtt']) {
        out[k] = m[k].summary({});
    }
    out.frames = m.frames;
    out.skipped = m.skipped;
    out.renders = scene.renders;
    out.trailPoints = scene.trailCount || 0;
    out.trailUploadedBytes = scene.trailUploadedBytes || 0;
    out.smoother = { snaps: smoother.snaps, blends: smoother.blends, last: smoother.lastSnapReason, enabled: smoother.enabled };
    out.counts = Object.fromEntries(feed.counts);
    out.seqGaps = Object.fromEntries(feed.seqGaps);
    out.sent = Object.fromEntries(feed.sentTypes);
    out.handler = {};
    for (const [k, st] of feed.handlerMs) {
        out.handler[k] = st.summary({});
    }
    out.clock = { offset: feed.clock.best ? feed.clock.best.offset : null, uncertainty_ms: feed.clock.uncertaintyMs() };
    out.connection = feed.connection;
    out.connectionEpoch = feed.connectionEpoch;
    out.sessionEpoch = feed.sessionEpoch;
    out.mode = mode;
    out.graphSeries = liveStore.series.size;
    return out;
};
state.replayState = () => (replay ? {
    t: replay.clock.t, start: replay.model.startT, end: replay.model.endT, playing: replay.clock.playing,
    robot: state.replayRobot ? { x: state.replayRobot.field.x_m, y: state.replayRobot.field.y_m, segment: state.replayRobot.segment } : null,
    events: replay.model.events.length, warnings: replay.model.warnings, columns: replay.model.columns.length,
} : null);
state.seek = (t) => replay && replay.clock.seek(t);

feed.connect();
requestAnimationFrame(frame);

// Hold the document's load event until the first documents arrived, bounded:
// a module script is deferred, so this top-level await delays load, and a
// headless --dump-dom (which captures at load) then sees a live page with
// states and a frame instead of the empty shell. People see the page render
// regardless; only the tab's loading indicator lasts a moment longer. A
// configuration without cameras never waits for a frame.
await new Promise((resolve) => {
    const started = performance.now();
    const tick = () => {
        const cameras = feed.hello ? (feed.hello.camera_sensors || []).length : 1;
        const ready = feed.stateCount >= 10 && (feed.frames >= 1 || cameras === 0);
        if (ready || performance.now() - started > 6000) {
            // the DOM a headless dump reads is written by the panel pass
            panels(performance.now());
            resolve();
        } else {
            setTimeout(tick, 100);
        }
    };
    tick();
});
