// capture_panel.js
// Diagnostic capture on the Pi: start (pre/post window and streams),
// cancel, status, the list of finished captures, download, and replay.
// The recording itself runs on the Pi, so what is kept never depends on
// this browser or its network; closing the page changes nothing. Status
// arrives as 'capture' messages; the panel also asks GET
// /api/capture/status when it opens and while a capture runs, so it works
// when those messages are late. Start and cancel are live-only controls:
// disabled during replay.

import { el, setText, setClass, setHidden, KeyedTable } from './dom.js';
import { fmt } from './transforms.js';

// Stream kinds (DiagnosticsHub kinds). bytes is raw transport bytes: not
// part of the default set and only kept while a capture that selected it runs.
export const CAPTURE_KINDS = [
    ['robot_state', 'robot state (every localization publication)'],
    ['pico_sensor', 'Pico sensor frames (encoders, gyro)'],
    ['pico_status', 'Pico status frames'],
    ['pico_diag', 'Pico diagnostic frames'],
    ['vex_imu', 'VEX IMU samples (Brain GET_STATE)'],
    ['brain_request', 'Brain requests and replies'],
    ['brain_telemetry', 'Brain telemetry (commands, errors, attitude)'],
    ['path', 'Brain path reports'],
    ['event', 'lifecycle and calibration events'],
    ['bytes', 'raw transport bytes (hex, bounded)'],
];

function sizeText(b) {
    if (typeof b !== 'number') {
        return 'n/a';
    }
    return b >= 1048576 ? `${fmt(b / 1048576, 2)} MB` : `${fmt(b / 1024, 1)} KB`;
}

export class CapturePanel {
    constructor(root, opts) {
        this.root = root;
        this.onReplay = opts.onReplay || (() => {});
        this.pre = root.querySelector('#cap-pre');
        this.post = root.querySelector('#cap-post');
        this.startBtn = root.querySelector('#cap-start');
        this.cancelBtn = root.querySelector('#cap-cancel');
        this.msg = root.querySelector('#cap-msg');
        this.statusEl = root.querySelector('#cap-status');
        this.activeEl = root.querySelector('#cap-active');
        this.list = new KeyedTable(root.querySelector('#cap-list'), ['capture', 'state', 'trigger', 'window', 'size', 'loss', '']);
        this.fileInput = root.querySelector('#cap-file');
        this.streamsEl = root.querySelector('#cap-streams');
        this.boxes = new Map();
        for (const [kind, label] of CAPTURE_KINDS) {
            const l = el('label', 'pick');
            const b = el('input');
            b.type = 'checkbox';
            b.checked = kind !== 'bytes';
            l.appendChild(b);
            l.appendChild(el('span', undefined, ` ${kind}`));
            l.title = label + (kind === 'bytes' ? '. Off by default: while a capture with bytes runs, the link monitors copy raw bytes to the recorder.' : '');
            this.streamsEl.appendChild(l);
            this.boxes.set(kind, b);
        }
        this.status = null;
        this.statusAt = 0;
        this.live = true;
        this.polling = false;
        this.lastPoll = 0;
        this.startBtn.addEventListener('click', () => this.start());
        this.cancelBtn.addEventListener('click', () => this.cancel());
        this.fileInput.addEventListener('change', () => {
            const f = this.fileInput.files && this.fileInput.files[0];
            if (f) {
                f.arrayBuffer().then((buf) => this.onReplay(buf, `local file ${f.name}`))
                    .catch((e) => this.say('could not read the file: ' + e, true));
                this.fileInput.value = '';
            }
        });
    }

    say(text, bad) {
        setText(this.msg, text);
        setClass(this.msg, bad ? 'state-fault' : 'muted');
    }

    setLive(on) {
        this.live = on;
        this.updateButtons();
    }

    // A 'capture' message or an HTTP status object.
    setStatus(status, nowMs) {
        this.status = status;
        this.statusAt = nowMs;
        this.updateButtons();
    }

    active() {
        const s = this.status;
        if (!s) {
            return null;
        }
        const a = s.active;
        return a && typeof a === 'object' && a.id ? a : null;
    }

    updateButtons() {
        const s = this.status;
        const available = !!(s && s.available);
        const a = this.active();
        const busy = !!a && /record|trigger|final|pre/.test(String(a.state || s.state || ''));
        this.startBtn.disabled = !this.live || !available || busy;
        this.cancelBtn.disabled = !this.live || !a || !busy;
        this.startBtn.title = !this.live ? 'replay: capture start is a live-only control' :
            (!available ? 'capture is not available on this runtime' : (busy ? 'a capture is already recording' : ''));
    }

    async fetchStatus() {
        try {
            const r = await fetch('/api/capture/status', { cache: 'no-store' });
            if (!r.ok) {
                this.setStatus({ available: false, reason: `status ${r.status}` }, performance.now());
                return;
            }
            this.setStatus(await r.json(), performance.now());
        } catch (e) {
            this.setStatus({ available: false, reason: String(e) }, performance.now());
        }
    }

    // Panel tick while visible: poll while recording or when no status
    // message came for a while (an older runtime has no 'capture' messages).
    poll(nowMs) {
        const a = this.active();
        const due = !this.status || (a && nowMs - this.lastPoll > 1000) || nowMs - this.statusAt > 5000;
        if (due && !this.polling && nowMs - this.lastPoll > 900) {
            this.polling = true;
            this.lastPoll = nowMs;
            this.fetchStatus().finally(() => {
                this.polling = false;
            });
        }
    }

    streams() {
        const picked = [...this.boxes].filter(([, b]) => b.checked).map(([k]) => k);
        return picked.length === this.boxes.size ? 'all' : picked.join(',');
    }

    async start() {
        if (!this.live) {
            return;
        }
        const q = new URLSearchParams({ pre_s: String(Number(this.pre.value)), post_s: String(Number(this.post.value)), streams: this.streams() });
        this.say('starting...');
        try {
            const r = await fetch('/api/capture/start?' + q.toString(), { method: 'POST' });
            const body = await r.json().catch(() => ({}));
            if (r.ok && body.ok) {
                this.say(`capture ${body.id} started`);
            } else {
                this.say(`not started: ${body.error || 'HTTP ' + r.status}`, true);
            }
        } catch (e) {
            this.say('not started: ' + e, true);
        }
        this.fetchStatus();
    }

    async cancel() {
        const a = this.active();
        if (!this.live || !a) {
            return;
        }
        try {
            const r = await fetch('/api/capture/cancel?id=' + encodeURIComponent(a.id), { method: 'POST' });
            const body = await r.json().catch(() => ({}));
            this.say(r.ok && body.ok !== false ? `capture ${a.id} cancelled` : `cancel failed: ${body.error || 'HTTP ' + r.status}`, !(r.ok && body.ok !== false));
        } catch (e) {
            this.say('cancel failed: ' + e, true);
        }
        this.fetchStatus();
    }

    async replayFromPi(id) {
        this.say(`downloading ${id}...`);
        try {
            const r = await fetch(`/api/capture/${encodeURIComponent(id)}.zip`, { cache: 'no-store' });
            if (!r.ok) {
                this.say(`capture ${id} not available (HTTP ${r.status})`, true);
                return;
            }
            const buf = await r.arrayBuffer();
            this.say(`loaded ${id}, ${sizeText(buf.byteLength)}`);
            this.onReplay(buf, `Pi capture ${id}`);
        } catch (e) {
            this.say('download failed: ' + e, true);
        }
    }

    render() {
        const s = this.status;
        if (!s) {
            setText(this.statusEl, 'capture status not known yet');
            return;
        }
        if (!s.available) {
            setText(this.statusEl, `capture unavailable on this runtime${s.reason ? ': ' + s.reason : ''}`);
        } else {
            const lim = s.limits || {};
            const auto = s.auto || {};
            setText(this.statusEl, `state ${s.state || 'idle'}; limits: pre <= ${lim.max_pre_s ?? 'n/a'} s, post <= ${lim.max_post_s ?? 'n/a'} s, ` +
                `${lim.max_records ?? 'n/a'} records, ${lim.max_mb ?? 'n/a'} MB, keep ${lim.keep ?? 'n/a'}` +
                (Array.isArray(auto.triggers) && auto.triggers.length ? `; auto triggers ${auto.triggers.join(', ')}` +
                    ` (cooldown ${auto.cooldown_s ?? 'n/a'} s, at most ${auto.max_per_hour ?? 'n/a'} per hour)` : '; auto triggers off'));
            if (typeof lim.max_pre_s === 'number') {
                this.pre.max = String(lim.max_pre_s);
            }
            if (typeof lim.max_post_s === 'number') {
                this.post.max = String(lim.max_post_s);
            }
        }
        const a = this.active();
        setHidden(this.activeEl, !a);
        if (a) {
            const parts = [`capture ${a.id}: ${a.state || s.state}`];
            if (a.reason) {
                parts.push(`trigger ${a.reason}${a.requester ? ' by ' + a.requester : ''}`);
            }
            if (typeof a.elapsed_s === 'number' || typeof a.post_s === 'number') {
                parts.push(`${fmt(a.elapsed_s, 1)} / ${fmt(a.post_s, 1)} s after trigger`);
            }
            if (typeof a.records === 'number') {
                parts.push(`${a.records} records`);
            }
            if (a.truncated) {
                parts.push('TRUNCATED (limit reached)');
            }
            setText(this.activeEl, parts.join(', '));
        }
        this.list.begin();
        if (!(s.captures || []).length) {
            this.list.row('none', ['none yet', '', '', '', '', '', '']);
        }
        for (const c of s.captures || []) {
            const ready = c.ready === true || c.state === 'ready';
            const loss = [];
            if (c.truncated) {
                loss.push('truncated');
            }
            const drops = c.drops || c.dropped;
            if (drops && typeof drops === 'object') {
                const total = Object.values(drops).reduce((acc, v) => acc + (typeof v === 'number' ? v : 0), 0);
                if (total) {
                    loss.push(`${total} dropped`);
                }
            } else if (typeof drops === 'number' && drops) {
                loss.push(`${drops} dropped`);
            }
            if (c.write_error) {
                loss.push('file write failed: in memory only');
            }
            const tr = this.list.row(c.id, [c.id, c.state || (ready ? 'ready' : ''), c.reason || '',
                `${fmt(c.pre_s, 1)} + ${fmt(c.post_s, 1)} s`, sizeText(c.bytes ?? c.size_bytes), loss.join(', ') || 'none', ''],
            { title: JSON.stringify(c) });
            const cell = tr.lastChild;
            if (cell.__ready !== ready) {
                cell.__ready = ready;
                cell.textContent = '';
                if (ready) {
                    const a2 = el('a', undefined, 'download');
                    a2.href = `/api/capture/${encodeURIComponent(c.id)}.zip`;
                    a2.download = `${c.id}.zip`;
                    cell.appendChild(a2);
                    cell.appendChild(document.createTextNode(' '));
                    const b = el('button', undefined, 'replay');
                    b.type = 'button';
                    b.dataset.capture = c.id;
                    b.addEventListener('click', () => this.replayFromPi(c.id));
                    cell.appendChild(b);
                }
            }
        }
        this.list.end();
    }
}
