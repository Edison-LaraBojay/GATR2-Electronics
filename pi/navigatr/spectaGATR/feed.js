// feed.js
// The one WebSocket to the runtime. Handlers only parse and store: each
// message replaces the newest of its kind (state, diag, telemetry,
// instrumentation, capture) or is queued (history, events), a version
// counter is bumped, and the receipt time is kept. The render loop and the
// panels read from here on their own schedule; nothing in onmessage touches
// the DOM or the scene.
//
// Speaks navigatr.inspect/2 (state/diag/history/event/telemetry/...), and
// still reads an inspect/1 'snapshot' by splitting it into the same parts,
// so the page also works against an older runtime.
//
// The viewer sends only preview budgets, pings, subscriptions and history
// requests. Nothing here can command the robot, and replay never calls it.

import { ClockSync, Stat, Rates } from './latency.js';

const RECONNECT_MIN_MS = 500;
const RECONNECT_MAX_MS = 5000;
const CONNECTION_STALE_MS = 1000;
const PING_MS = 1000;
const EVENTS_KEEP = 200;

export const CONTRACT_2 = 'navigatr.inspect/2';

function sameSession(a, b) {
    return !!a && !!b && a.id === b.id && a.reset_count === b.reset_count;
}

export class Feed {
    constructor(opts) {
        this.onError = opts.onError || (() => {});
        this.onFrame = opts.onFrame || (() => {});
        // Synchronous store hooks: numbers into bounded rings only (graphs,
        // trail points, latency statistics), so no message is lost to the
        // latest-wins slots when the page draws slower than messages come.
        this.onStored = opts.onStored || (() => {});
        this.onSession = opts.onSession || (() => {});
        this.perf = !!opts.perf;
        this.ws = null;
        this.reconnectAttempts = 0;
        this.reconnectTimer = null;
        this.pingTimer = null;
        this.decoder = new TextDecoder();

        this.connection = 'connecting';
        this.connectionEpoch = 0;   // bumps on every socket open
        this.sessionEpoch = 0;      // bumps on every Pi session or reset change
        this.session = null;        // {id, reset_count}
        this.sessionIdEpoch = 0;    // bumps only when the Pi process changed (new clock)
        this.reachedLive = false;

        this.hello = null;
        this.helloVersion = 0;
        this.contract2 = false;
        this.features = {};

        this.state = null;
        this.stateVersion = 0;
        this.stateArrival = 0;
        this.stateCount = 0;
        this.lastStateArrival = 0;
        this.diag = null;
        this.diagVersion = 0;
        this.diagArrival = 0;
        this.telemetry = null;
        this.telemetryVersion = 0;
        this.telemetryArrival = 0;
        this.instrumentation = null;
        this.instrumentationVersion = 0;
        this.instrumentationArrival = 0;
        this.capture = null;
        this.captureVersion = 0;
        this.events = [];
        this.eventsVersion = 0;
        this.eventKeys = new Set();
        this.history = [];          // queued history batches for the scene
        this.historyVersion = 0;
        this.frames = 0;

        this.clock = new ClockSync();
        this.rates = new Rates();
        this.counts = new Map();    // type -> messages received
        this.bytes = new Map();     // type -> bytes received
        this.seqGaps = new Map();   // type -> messages the server's seq skipped
        this.lastSeq = new Map();
        this.handlerMs = new Map(); // type -> Stat of handler cost
        this.sentTypes = new Map(); // type -> messages sent (the tests read this)

        this.sub = { state_hz: 30, diag: true, telemetry: true, instrumentation: false, raw: false, decoded: false };
    }

    url() {
        const proto = location.protocol === 'https:' ? 'wss://' : 'ws://';
        return proto + location.host + '/ws';
    }

    connect() {
        this.reconnectTimer = null;
        this.setConnection('connecting');
        let ws;
        try {
            ws = new WebSocket(this.url());
        } catch (e) {
            this.onError('websocket: ' + e);
            this.scheduleReconnect();
            return;
        }
        this.ws = ws;
        ws.binaryType = 'arraybuffer';
        ws.onopen = () => {
            this.reconnectAttempts = 0;
            this.connectionEpoch += 1;
            this.clock.reset();
            this.sendPing();
            clearInterval(this.pingTimer);
            this.pingTimer = setInterval(() => this.sendPing(), PING_MS);
        };
        ws.onmessage = (ev) => this.onMessage(ev.data);
        ws.onclose = () => {
            if (this.ws === ws) {
                this.ws = null;
            }
            clearInterval(this.pingTimer);
            this.pingTimer = null;
            this.setConnection('disconnected');
            this.scheduleReconnect();
        };
        ws.onerror = () => {
            // the close event that follows carries the state change
        };
    }

    scheduleReconnect() {
        if (this.reconnectTimer !== null) {
            return;
        }
        const delay = Math.min(RECONNECT_MAX_MS, RECONNECT_MIN_MS * Math.pow(2, this.reconnectAttempts));
        this.reconnectAttempts += 1;
        this.reconnectTimer = setTimeout(() => this.connect(), delay);
    }

    open() {
        return !!this.ws && this.ws.readyState === WebSocket.OPEN;
    }

    send(obj) {
        if (!this.open()) {
            return false;
        }
        this.ws.send(JSON.stringify(obj));
        this.sentTypes.set(obj.type, (this.sentTypes.get(obj.type) || 0) + 1);
        return true;
    }

    sendPing() {
        if (this.open()) {
            this.send(this.clock.ping(performance.now()));
        }
    }

    // Merges into the subscription and sends it when the server speaks inspect/2.
    subscribe(changes) {
        let changed = false;
        for (const k of Object.keys(changes || {})) {
            if (this.sub[k] !== changes[k]) {
                this.sub[k] = changes[k];
                changed = true;
            }
        }
        if (changed || !changes) {
            this.sendSubscription();
        }
    }

    sendSubscription() {
        if (this.contract2) {
            this.send({ type: 'subscribe', ...this.sub });
        }
    }

    requestHistory() {
        return this.contract2 ? this.send({ type: 'history' }) : false;
    }

    setConnection(s) {
        this.connection = s;
        if (s === 'live') {
            this.reachedLive = true;
        }
    }

    // Stale by the browser receipt clock, never by a Pi timestamp.
    tick(nowMs) {
        this.rates.roll(nowMs);
        if (this.open() && this.connection === 'live' && this.stateCount && nowMs - this.lastStateArrival > CONNECTION_STALE_MS) {
            this.setConnection('stale');
        }
    }

    count(type, bytes, seq) {
        this.counts.set(type, (this.counts.get(type) || 0) + 1);
        this.bytes.set(type, (this.bytes.get(type) || 0) + bytes);
        if (typeof seq === 'number') {
            const last = this.lastSeq.get(type);
            if (typeof last === 'number' && seq > last + 1) {
                this.seqGaps.set(type, (this.seqGaps.get(type) || 0) + (seq - last - 1));
            }
            this.lastSeq.set(type, seq);
        }
    }

    onMessage(data) {
        const t0 = performance.now();
        let type;
        if (typeof data === 'string') {
            let doc;
            try {
                doc = JSON.parse(data);
            } catch (e) {
                this.onError('bad json: ' + e);
                return;
            }
            type = doc && doc.type ? doc.type : 'unknown';
            if (this.perf) {
                performance.mark('navigatr:' + type);
            }
            this.count(type, data.length, doc.seq);
            this.rates.add(type, t0);
            this.dispatch(doc, t0);
        } else {
            type = 'frame';
            this.count(type, data.byteLength);
            this.rates.add(type, t0);
            this.onBinary(data);
        }
        const dt = performance.now() - t0;
        let st = this.handlerMs.get(type);
        if (!st) {
            st = new Stat(128);
            this.handlerMs.set(type, st);
        }
        st.push(dt);
        if (this.perf) {
            performance.measure('navigatr:handle:' + type, { start: t0, duration: dt });
        }
    }

    dispatch(doc, now) {
        switch (doc.type) {
        case 'hello': this.onHello(doc, now); break;
        case 'state': this.onState(doc, now); break;
        case 'diag': this.onDiag(doc, now); break;
        case 'history': this.onHistory(doc, true); break;
        case 'event': this.addEvent(doc); break;
        case 'telemetry':
            this.telemetry = doc;
            this.telemetryVersion += 1;
            this.telemetryArrival = now;
            this.onStored('telemetry', doc, now);
            break;
        case 'instrumentation':
            this.instrumentation = doc;
            this.instrumentationVersion += 1;
            this.instrumentationArrival = now;
            this.onStored('instrumentation', doc, now);
            break;
        case 'capture':
            this.capture = doc;
            this.captureVersion += 1;
            break;
        case 'pong':
            if (this.clock.pong(doc, now)) {
                this.onStored('pong', doc, now);
            }
            break;
        case 'snapshot': this.onSnapshot(doc, now); break;
        default: break; // unknown types are ignored by contract
        }
    }

    noteSession(session) {
        if (!session || typeof session !== 'object' || session.id === undefined || sameSession(session, this.session)) {
            return;
        }
        const newProcess = !this.session || this.session.id !== session.id;
        if (newProcess) {
            this.sessionIdEpoch += 1;
            this.clock.reset();   // a new Pi process: its clock restarted
            this.sendPing();
        }
        this.session = { id: session.id, reset_count: session.reset_count };
        this.sessionEpoch += 1;
        this.events.length = 0;
        this.eventKeys.clear();
        this.eventsVersion += 1;
        this.history.length = 0;
        this.state = null;
        this.onSession(newProcess);
    }

    onHello(doc) {
        if (this.hello && doc.contract && this.hello.contract !== doc.contract) {
            this.onError(`contract changed: ${this.hello.contract} -> ${doc.contract}`);
        }
        this.noteSession(doc.session);
        this.hello = doc;
        this.helloVersion += 1;
        this.contract2 = doc.contract === CONTRACT_2;
        this.features = doc.features || {};
        if (typeof this.features.state_hz === 'number' && this.features.state_hz > 0) {
            this.sub.state_hz = this.features.state_hz;   // the server's configured rate
        }
        if (this.contract2) {
            this.sendSubscription();
        }
    }

    onState(doc, now) {
        this.noteSession(doc.session);
        this.state = doc;
        this.stateVersion += 1;
        this.stateArrival = now;
        this.stateCount += 1;
        this.lastStateArrival = now;
        if (this.connection !== 'live') {
            this.setConnection('live');
        }
        this.onStored('state', doc, now);
    }

    onDiag(doc, now) {
        this.noteSession(doc.session);
        this.diag = doc;
        this.diagVersion += 1;
        this.diagArrival = now;
        this.onStored('diag', doc, now);
        const events = doc.events;
        if (Array.isArray(events) && events.length) {
            for (const e of events) {
                this.addEvent(e);
            }
        }
    }

    // History batch: replace rebuilds the trail (inspect/2), merge appends
    // what is newer (inspect/1 snapshots carry overlapping trails).
    onHistory(doc, replace) {
        if (doc.session) {
            this.noteSession(doc.session);
        }
        this.history.push({ trail: Array.isArray(doc.trail) ? doc.trail : [], replace });
        if (this.history.length > 4) {
            // a replace supersedes everything queued before it
            const lastReplace = this.history.map((h) => h.replace).lastIndexOf(true);
            if (lastReplace > 0) {
                this.history.splice(0, lastReplace);
            }
        }
        this.historyVersion += 1;
    }

    // event messages carry seq, diag.events entries sequence: the same number
    addEvent(e) {
        if (!e || typeof e.text !== 'string') {
            return;
        }
        const seq = typeof e.seq === 'number' ? e.seq : (typeof e.sequence === 'number' ? e.sequence : null);
        const key = seq !== null ? 's' + seq : 'h' + e.host_ms + '|' + e.text;
        if (this.eventKeys.has(key)) {
            return;
        }
        // diag lists and event messages overlap; one key per event
        const alt = 'h' + e.host_ms + '|' + e.text;
        if (key !== alt && this.eventKeys.has(alt)) {
            this.eventKeys.add(key);
            return;
        }
        this.eventKeys.add(key);
        this.eventKeys.add(alt);
        const ev = { seq, host_ms: e.host_ms, text: e.text };
        let i = this.events.length;
        while (i > 0 && typeof ev.host_ms === 'number' && this.events[i - 1].host_ms > ev.host_ms) {
            i -= 1;
        }
        this.events.splice(i, 0, ev);
        if (this.events.length > EVENTS_KEEP) {
            this.events.shift();
        }
        if (this.eventKeys.size > EVENTS_KEEP * 8) {
            this.eventKeys.clear();
            for (const x of this.events) {
                this.eventKeys.add(x.seq !== null && x.seq !== undefined ? 's' + x.seq : 'h' + x.host_ms + '|' + x.text);
                this.eventKeys.add('h' + x.host_ms + '|' + x.text);
            }
        }
        this.eventsVersion += 1;
    }

    // inspect/1: one document carries state, diagnostics and a trail tail.
    onSnapshot(doc, now) {
        this.noteSession(doc.session);
        const loc = doc.localization || {};
        this.onState({
            type: 'state', compat: true, host_ms: doc.host_ms, session: doc.session,
            publication: loc.publication, robot: doc.robot, localization: loc,
        }, now);
        this.onDiag(doc, now);
        if (Array.isArray(doc.trail) && doc.trail.length) {
            this.history.push({ trail: doc.trail, replace: false });
            if (this.history.length > 8) {
                this.history.shift();
            }
            this.historyVersion += 1;
        }
    }

    onBinary(buffer) {
        if (buffer.byteLength < 4) {
            this.onError('short binary message');
            return;
        }
        const n = new DataView(buffer).getUint32(0, true);
        if (4 + n > buffer.byteLength) {
            this.onError('binary header length out of range');
            return;
        }
        let header;
        try {
            header = JSON.parse(this.decoder.decode(new Uint8Array(buffer, 4, n)));
        } catch (e) {
            this.onError('bad frame header: ' + e);
            return;
        }
        this.frames += 1;
        this.onFrame(header, new Uint8Array(buffer, 4 + n));
    }
}
