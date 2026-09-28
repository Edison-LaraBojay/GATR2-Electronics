// diagnostics.js
// The diagnostics tab: sources, workers, localization function readiness
// with each function's stationary window and IMU bias calibration,
// diagnostics function counters and link counters from the newest diag
// message, plus the preview budget control. The form's budget goes to the
// page (onPreview), which sends {"type":"preview",...} while the camera
// panel is in view and hz 0 while it is not. Rows are updated in place by
// key, at the page's panel rate, and only while the tab is visible.

import { fmt, fmtMs } from './transforms.js';
import { KeyedTable, setText } from './dom.js';

export class Diagnostics {
    // onPreview(msg) -> 'sent', 'deferred' (sent once the camera panel is
    // in view) or 'offline'.
    constructor(root, onPreview) {
        this.root = root;
        this.sources = new KeyedTable(root.querySelector('#sources-table'), ['id', 'kind', 'state', 'age', 'seq', 'epoch', 'diagnostic']);
        this.workers = new KeyedTable(root.querySelector('#workers-table'),
            ['worker', 'run', 'cycles', 'Hz', 'last ms', 'mean ms', 'max ms', 'target ms', 'overruns', 'dropped', 'pending']);
        this.loc = new KeyedTable(root.querySelector('#loc-table'), ['id', 'type', 'ready', 'still', 'calibration', 'bias dps', 'note']);
        this.fn = new KeyedTable(root.querySelector('#diagfn-table'), ['worker', 'function', 'runs', 'ok', 'no data', 'fault', 'last']);
        this.links = new KeyedTable(root.querySelector('#links-table'), ['worker', 'link', 'bytes', 'packets', 'decode err', 'seq gaps']);
        this.session = new KeyedTable(root.querySelector('#session-table'), ['key', 'value']);
        this.summary = document.querySelector('#diag-summary');
        this.note = root.querySelector('#preview-note');
        this.form = root.querySelector('#preview-form');
        this.hello = null;
        this.userPreview = false;   // the person set a budget: a new hello keeps it
        this.form.addEventListener('submit', (e) => {
            e.preventDefault();
            const msg = this.formPreview();
            this.userPreview = true;
            const r = onPreview(msg);
            const what = `hz ${msg.hz}, quality ${msg.quality}, max width ${msg.max_width}`;
            this.note.textContent = r === 'sent' ? `sent ${what}`
                : (r === 'deferred' ? `kept ${what}: sent while the camera panel is in view` : 'not connected');
        });
    }

    // The budget the form holds (the server defaults until edited).
    formPreview() {
        return {
            type: 'preview',
            hz: Number(this.root.querySelector('#preview-hz').value),
            quality: Number(this.root.querySelector('#preview-quality').value),
            max_width: Number(this.root.querySelector('#preview-width').value),
        };
    }

    // Live-only controls are off during replay.
    setLiveControls(on) {
        for (const e of this.form.elements) {
            e.disabled = !on;
        }
        if (!on) {
            this.note.textContent = 'replay: preview budget is a live-only control';
        }
    }

    setHello(hello) {
        this.hello = hello;
        const i = hello.inspection || {};
        if (this.userPreview) {
            return;
        }
        if (typeof i.preview_hz === 'number') {
            this.root.querySelector('#preview-hz').value = i.preview_hz;
        }
        if (typeof i.preview_quality === 'number') {
            this.root.querySelector('#preview-quality').value = i.preview_quality;
        }
        if (typeof i.preview_max_width === 'number') {
            this.root.querySelector('#preview-width').value = i.preview_max_width;
        }
        this.note.textContent = `server defaults: hz ${i.preview_hz}, quality ${i.preview_quality}, max width ${i.preview_max_width}`;
    }

    // The one-line summary in the dock header (always visible).
    renderSummary(diag) {
        if (!diag) {
            return;
        }
        const w = diag.workers || {};
        const est = w.estimation || {};
        const fld = w.field || {};
        const insp = w.inspection || {};
        const rate = typeof insp.snapshot_rate_hz === 'number' ? `, inspection ${fmt(insp.snapshot_rate_hz, 1)} snap/s` : '';
        const frames = typeof insp.frame_rate_hz === 'number' ? ` ${fmt(insp.frame_rate_hz, 1)} frames/s` : '';
        setText(this.summary, `estimation ${fmt(est.rate_hz, 1)} Hz, field ${fmt(fld.rate_hz, 1)} Hz${rate}${frames}, ` +
            `${insp.clients} clients, ${(diag.sources || []).filter((s) => s.state !== 'valid').length} sources not valid`);
    }

    render(diag) {
        if (!diag) {
            return;
        }
        const w = diag.workers || {};
        const est = w.estimation || {};
        const fld = w.field || {};
        const insp = w.inspection || {};

        this.sources.begin();
        for (const s of diag.sources || []) {
            this.sources.row(s.id, [s.id, s.kind, [s.state, `state-${s.state}`], fmtMs(s.receipt_age_ms), s.sequence, s.epoch, s.diagnostic || '']);
        }
        this.sources.end();

        const worker = (name, s) => [
            name, s.running ? 'yes' : 'no', s.cycles, fmt(s.rate_hz, 1), fmt(s.last_cycle_ms, 2), fmt(s.mean_cycle_ms, 2),
            fmt(s.max_cycle_ms, 1), s.period_target_ms, s.overruns, s.dropped, s.pending,
        ];
        this.workers.begin();
        this.workers.row('estimation', worker('estimation', est));
        this.workers.row('field', worker('field', fld));
        this.workers.row('inspection', ['inspection', [inspectionText(insp, diag.inspection), '', 10]]);
        this.workers.end();

        const loc = diag.localization || {};
        const still = (f) => {
            const s = f.stillness;
            if (!s || !s.monitored) {
                return ['-', '-', '-'];
            }
            return [
                s.stationary ? 'yes' : 'no',
                `${s.calibration} ${fmt(s.progress_ms / 1000, 1)}/${fmt(s.window_ms / 1000, 1)} s, ${s.attempts} att`,
                typeof s.bias_dps === 'number' ? fmt(s.bias_dps, 3) : '-',
            ];
        };
        this.loc.begin();
        for (const f of loc.functions || []) {
            this.loc.row('f:' + f.id, [f.id, f.type, [f.ready ? 'ready' : 'not ready', f.ready ? 'state-ok' : 'state-no_data'], ...still(f), f.note || '']);
        }
        this.loc.row('summary', [[`${loc.estimator_type}: ${loc.updates} updates, history ${loc.history_size}, publication ${loc.publication}, ` +
            `clock ${loc.clock_mapped ? 'mapped' : 'unmapped'}, ${loc.all_ready ? 'all ready' : 'not all ready'}, ` +
            `${loc.stationary ? 'stationary' : 'moving or unknown'}, ${loc.continuity_breaks || 0} continuity breaks` +
            (loc.last_break ? ` (last: ${loc.last_break})` : ''), '', 7]]);
        this.loc.end();

        const d = diag.diagnostics;
        this.fn.begin();
        this.links.begin();
        if (d) {
            for (const [name, part] of [['estimation', d.estimation], ['field', d.field]]) {
                for (const f of (part && part.functions) || []) {
                    this.fn.row(`${name}/${f.label}`, [name, f.label, f.runs, f.ok, f.no_data,
                        [f.fault, f.fault > 0 ? 'state-fault' : ''], [f.last, `state-${f.last}`]]);
                }
                for (const l of (part && part.links) || []) {
                    this.links.row(`${name}/${l.id}`, [name, l.id, l.bytes, l.packets,
                        [l.decode_errors, l.decode_errors > 0 ? 'state-fault' : ''], l.seq_gaps]);
                }
            }
        }
        this.fn.end();
        this.links.end();

        const h = this.hello || {};
        const cfg = h.configuration || {};
        const fs = diag.field_snapshot || {};
        const target = diag.target;
        const link = diag.brain_link;
        const rows = [
            ['session', `${diag.session.id} reset ${diag.session.reset_count}`],
            ['contract', `${h.contract || 'n/a'}`],
            ['configuration', `${cfg.id || ''} ${cfg.digest ? 'digest ' + cfg.digest : ''}`],
            ['loop rate', `${cfg.loop_rate_hz !== undefined ? cfg.loop_rate_hz + ' Hz' : 'n/a'}`],
            ['cycle', `${diag.cycle} (${diag.running ? 'running' : 'stopped'}) host ${diag.host_ms} ms`],
            ['field snapshot', `${fs.status || 'n/a'} inv ${fs.invocation} age ${fmtMs(fs.age_ms)} ${fs.diagnostic || ''}`],
            ['target', target ? `${target.active ? target.target_id + ' ' + target.status : 'none'}` : 'n/a'],
            ['brain link', link ? `${link.link_open ? 'open' : 'closed'}, session ${link.session}, pi ${link.pi_instance}, ` +
                `last request ${fmtMs(link.last_request_age_ms)}` : 'none'],
            ['path', link && link.path ? `${link.path.mode} #${link.path.command_id}, ${link.path.points.length} points` : 'none'],
            ['warnings', `${(h.warnings || []).length}`],
        ];
        this.session.begin();
        for (const [k, v] of rows) {
            this.session.row(k, [k, v]);
        }
        this.session.end();
    }
}

// The inspection worker line: inspect/1 counters when present, inspect/2
// build timings when present, whichever the server sent.
function inspectionText(insp, inspection) {
    const parts = [];
    if (insp.snapshot_rate_hz !== undefined) {
        parts.push(`${fmt(insp.snapshot_rate_hz, 1)} snap/s`);
    }
    if (insp.frame_rate_hz !== undefined) {
        parts.push(`${fmt(insp.frame_rate_hz, 1)} frames/s`);
    }
    if (insp.snapshots_sent !== undefined) {
        parts.push(`sent ${insp.snapshots_sent} / skipped ${insp.snapshots_skipped} snapshots, ${insp.frames_sent} / ${insp.frames_skipped} frames`);
    }
    if (insp.last_encode_ms !== undefined) {
        parts.push(`encode ${fmt(insp.last_encode_ms, 1)} ms (mean ${fmt(insp.mean_encode_ms, 1)}), ${insp.encodes} encodes`);
    }
    if (typeof insp.bytes_sent === 'number') {
        parts.push(`${(insp.bytes_sent / 1e6).toFixed(2)} MB`);
    }
    if (insp.clients !== undefined) {
        parts.push(`clients ${insp.clients}` + (insp.clients_total !== undefined
            ? ` (${insp.clients_total} total, ${insp.client_disconnects} disconnects)` : ''));
    }
    const build = (inspection && inspection.build) || {};
    for (const key of ['state', 'diag']) {
        const b = build[key];
        if (b && typeof b === 'object') {
            parts.push(`${key} build ${fmt(b.last_us, 0)} us (mean ${fmt(b.mean_us, 0)}), ${fmt(b.last_bytes, 0)} B`);
        }
    }
    return parts.join(', ') || 'n/a';
}
