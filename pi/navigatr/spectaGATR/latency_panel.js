// latency_panel.js
// Where the time goes between a measurement and the screen, per the
// inspect/2 latency terms (latency.js), plus the browser's own frame times,
// message rates per channel, gaps in the server's per-channel sequence
// numbers (one counter shared by all clients, so a gap also counts messages
// built for other viewers), this page's own replaced counts from the
// server's per-client statistics, and those statistics for every client.
// Each figure names its clock; nothing subtracts a Pi time from a browser
// time.

import { fmt } from './transforms.js';
import { KeyedTable, setText } from './dom.js';

function kvText(obj) {
    if (!obj || typeof obj !== 'object') {
        return String(obj);
    }
    return Object.entries(obj).map(([k, v]) => (v !== null && typeof v === 'object'
        ? `${k} {${kvText(v)}}` : `${k} ${typeof v === 'number' && !Number.isInteger(v) ? fmt(v, 2) : v}`)).join(', ');
}

// This page's replaced count for a message type, from its own entry in
// diag.inspection.clients (binary frames sum the per-camera preview
// channels); '-' for reliable types, 'n/a' without per-client statistics.
export function replacedFor(me, type) {
    const ch = me && me.channels;
    if (!ch) {
        return 'n/a';
    }
    if (type === 'frame') {
        let n = 0;
        let any = false;
        for (const k of Object.keys(ch)) {
            if (k.startsWith('preview')) {
                any = true;
                n += ch[k].replaced || 0;
            }
        }
        return any ? n : '-';
    }
    const c = ch[type];
    return c && c.replaceable !== false ? (c.replaced || 0) : '-';
}

export class LatencyPanel {
    constructor(root) {
        this.terms = new KeyedTable(root.querySelector('#lat-terms'), ['term', 'last', 'p50', 'p95', 'max', 'n', 'clock / how']);
        this.channels = new KeyedTable(root.querySelector('#lat-channels'),
            ['channel', 'msg/s', 'received', 'KB', 'seq gaps (replaced, skipped or built for other clients)',
                'replaced for this page (server)', 'handler p50 / p95 / max ms']);
        this.clients = new KeyedTable(root.querySelector('#lat-clients'),
            ['client', 'queued B', 'in flight B', 'reliable', 'slots', 'diag same, skipped', 'per channel: sent / replaced / refused, last enqueue-to-written ms']);
        this.builds = new KeyedTable(root.querySelector('#lat-builds'), ['document', 'count', 'last us', 'mean us', 'max us', 'last B', 'mean B']);
        this.closed = root.querySelector('#lat-closed');
        this.note = root.querySelector('#lat-note');
        this.tmp = {};
    }

    // m: the page's measurement set (app.js); feed: the Feed; diag: newest.
    render(m, feed, diag) {
        const s = this.tmp;
        const row = (key, label, stat, how) => {
            stat.summary(s);
            this.terms.row(key, [label, fmt(stat.last, 1), fmt(s.p50, 1), fmt(s.p95, 1), fmt(s.max, 1), s.n, how]);
        };
        const unc = feed.clock.uncertaintyMs();
        this.terms.begin();
        row('src', 'source age at publish (ms)', m.sourceAge, 'Pi clock only: state.host_ms - robot.measured_at_host_ms');
        row('net', 'publish to receive (ms)', m.publishToReceive, Number.isFinite(unc)
            ? `estimate via ping/pong offset, +-${fmt(unc, 1)} ms (min RTT/2 of last 20 pings)`
            : 'no pong yet: not estimated (never Pi minus browser time)');
        row('rtt', 'ping round trip (ms)', m.rtt, 'browser clock only');
        row('render', 'receive to drawn (ms)', m.receiveToRender, 'browser clock only: message arrival to the frame that drew it');
        row('smooth', 'display smoothing lag (ms)', m.smoothLag, m.smoothing ? 'presentation only, at most one state period (<= 100 ms)' : 'smoothing off');
        row('interval', 'frame interval (ms)', m.frameInterval, 'browser: requestAnimationFrame to requestAnimationFrame');
        row('work', 'frame work (ms)', m.frameWork, 'browser: JS time inside the frame (scene, panels, render call)');
        row('panels', 'panel update (ms)', m.panelWork, 'browser: visible DOM panels, at most 10 per second');
        this.terms.end();

        const insp = diag && diag.inspection ? diag.inspection : null;
        const mine = feed.hello ? feed.hello.client_id : undefined;
        const clientsAll = insp && Array.isArray(insp.clients) ? insp.clients : [];
        const me = mine !== undefined ? clientsAll.find((c) => c.id === mine) : undefined;
        this.channels.begin();
        const types = [...feed.counts.keys()].sort();
        for (const t of types) {
            const h = feed.handlerMs.get(t);
            let hs = 'n/a';
            if (h) {
                h.summary(s);
                hs = `${fmt(s.p50, 2)} / ${fmt(s.p95, 2)} / ${fmt(s.max, 2)}`;
            }
            this.channels.row(t, [t, fmt(feed.rates.rate(t), 1), feed.counts.get(t), fmt((feed.bytes.get(t) || 0) / 1024, 1),
                feed.seqGaps.get(t) || 0, replacedFor(me, t), hs]);
        }
        this.channels.end();

        this.clients.begin();
        const clients = clientsAll;
        for (const c of clients) {
            const ch = c.channels || {};
            const per = Object.keys(ch).sort().map((k) => {
                const x = ch[k];
                return `${k} ${x.sent}/${x.replaced}/${x.refused || 0}` + (x.last_latency_ms ? ` ${fmt(x.last_latency_ms, 0)} ms` : '');
            }).join('; ');
            this.clients.row(String(c.id), [String(c.id) + (c.id === mine ? ' (this page)' : ''), c.queued_bytes, c.in_flight_bytes,
                c.reliable_backlog, c.slots_pending, c.diag_skipped, per]);
        }
        if (!clients.length) {
            this.clients.row('none', ['-', '', '', '', '', '', diag && diag.contract === 'navigatr.inspect/1'
                ? 'inspect/1 server: no per-client queue statistics' : 'no per-client statistics in the newest diag message']);
        }
        this.clients.end();
        this.builds.begin();
        for (const [k, b] of Object.entries((insp && insp.build) || {})) {
            this.builds.row(k, [k, b.count, fmt(b.last_us, 0), fmt(b.mean_us, 0), fmt(b.max_us, 0), fmt(b.last_bytes, 0), fmt(b.mean_bytes, 0)]);
        }
        this.builds.end();
        setText(this.closed, insp ? `clients closed: ${kvText(insp.closed || {})}; messages replaced ${insp.messages_replaced ?? 'n/a'}, ` +
            `refused ${insp.messages_refused ?? 'n/a'} (server totals; values are from the newest diag message)` : '');
        setText(this.note, `${m.renders} scene renders, ${m.frames} animation frames, ${m.skipped} vsyncs skipped by the render budget; ` +
            `pings sent ${feed.clock.sent}, pongs ${feed.clock.received}; connection ${feed.connection}`);
    }
}
