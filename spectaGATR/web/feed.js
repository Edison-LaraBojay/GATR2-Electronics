// feed.js
// The receiver feed: one WebSocket to <host>/ws (PROTOCOL.md, "Receiver to
// browser") with automatic reconnect, and FeedModel, which only records
// what arrived. Nothing here touches the DOM or the scene; the page reads
// the model once per animation frame.

import {
    DEFAULT_STALE_MS, TRAIL_CAPACITY, TrailRing, feedState, normalizePose, normalizeRobot,
} from './transforms.js';

const RECONNECT_MIN_MS = 250;
const RECONNECT_MAX_MS = 2000;

export class FeedModel {
    constructor(trailCapacity = TRAIL_CAPACITY) {
        this.trail = new TrailRing(trailCapacity);
        this.wsOpen = false;
        this.conn = 0;              // bumps on every socket open
        this.hello = null;
        this.robot = normalizeRobot(null);
        this.staleMs = DEFAULT_STALE_MS;
        this.status = null;
        this.statusAt = null;       // local ms
        this.statusConn = -1;
        this.pose = null;           // newest normalized pose, kept across reconnects
        this.poseAt = null;         // local ms
        this.poseConn = -1;
        this.poseSerial = 0;        // bumps per accepted pose
        this.poses = 0;
        this.rejected = 0;
        this.breakNext = true;
    }

    opened() {
        this.wsOpen = true;
        this.conn += 1;
        this.hello = null;
        this.status = null;
        this.statusAt = null;
        this.breakNext = true;      // never join the trail across a reconnect
    }

    closed() {
        this.wsOpen = false;
    }

    // One parsed message; now is the local clock in ms.
    handle(m, now) {
        if (!m || typeof m !== 'object') {
            this.rejected += 1;
            return;
        }
        if (m.type === 'hello') {
            this.hello = m;
            this.robot = normalizeRobot(m.robot);
            this.staleMs = typeof m.stale_ms === 'number' && m.stale_ms > 0 ? m.stale_ms : DEFAULT_STALE_MS;
            this.breakNext = true;
        } else if (m.type === 'status') {
            this.status = m;
            this.statusAt = now;
            this.statusConn = this.conn;
        } else if (m.type === 'pose') {
            this.acceptPose(m, now);
        }
    }

    acceptPose(m, now) {
        const p = normalizePose(m);
        if (!p) {
            this.rejected += 1;
            return;
        }
        const prev = this.pose;
        let brk = p.restart || this.breakNext || !prev;
        if (!brk) {
            // a new session or epoch breaks even when the restart pose was missed
            brk = p.session !== prev.session || p.epoch !== prev.epoch;
        }
        if (!brk) {
            // a gap longer than the stale threshold breaks: the robot may
            // have moved anywhere meanwhile
            const gap = this.poseConn === this.conn && p.rx_ms !== null && prev.rx_ms !== null
                ? p.rx_ms - prev.rx_ms : now - this.poseAt;
            brk = !(gap >= 0 && gap <= this.staleMs);
        }
        this.trail.push(p.x, p.y, brk);
        this.breakNext = false;
        p.breakBefore = brk;
        this.pose = p;
        this.poseAt = now;
        this.poseConn = this.conn;
        this.poseSerial += 1;
        this.poses += 1;
    }

    view(now) {
        return feedState(this, now);
    }
}

// Opens url and keeps it open: on close it retries with a backoff from
// RECONNECT_MIN_MS doubling to RECONNECT_MAX_MS, reset by a successful open.
export function connectFeed(url, model, clock = () => performance.now()) {
    let delay = RECONNECT_MIN_MS;
    let ws = null;
    const open = () => {
        try {
            ws = new WebSocket(url);
        } catch (e) {
            setTimeout(open, delay);
            delay = Math.min(RECONNECT_MAX_MS, delay * 2);
            return;
        }
        ws.onopen = () => {
            delay = RECONNECT_MIN_MS;
            model.opened();
        };
        ws.onmessage = (ev) => {
            if (typeof ev.data !== 'string') {
                return;
            }
            let m;
            try {
                m = JSON.parse(ev.data);
            } catch (e) {
                model.rejected += 1;
                return;
            }
            model.handle(m, clock());
        };
        ws.onclose = () => {
            model.closed();
            ws = null;
            setTimeout(open, delay);
            delay = Math.min(RECONNECT_MAX_MS, delay * 2);
        };
        ws.onerror = () => {
            // onclose follows and schedules the retry
        };
    };
    open();
}
