// diagnostics.js
// The diagnostics panel: sources, workers, localization function readiness,
// diagnostics function counters and link counters from each snapshot, plus
// the preview budget control that sends {"type":"preview",...} to the
// server. Tables are rebuilt at most a few times a second; the snapshot
// rate is higher than a person can read.

import { fmt, fmtMs } from './transforms.js';

const REFRESH_MS = 250;

function el(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) {
        e.className = cls;
    }
    if (text !== undefined) {
        e.textContent = text;
    }
    return e;
}

function fill(table, header, rows) {
    table.innerHTML = '';
    const tr = el('tr');
    for (const h of header) {
        tr.appendChild(el('th', undefined, h));
    }
    table.appendChild(tr);
    for (const r of rows) {
        const row = el('tr');
        for (const c of r) {
            const cell = Array.isArray(c) ? c : [c];
            row.appendChild(el('td', cell[1], String(cell[0])));
        }
        table.appendChild(row);
    }
}

export class Diagnostics {
    constructor(root, sendPreview) {
        this.root = root;
        this.sources = root.querySelector('#sources-table');
        this.workers = root.querySelector('#workers-table');
        this.loc = root.querySelector('#loc-table');
        this.fn = root.querySelector('#diagfn-table');
        this.links = root.querySelector('#links-table');
        this.session = root.querySelector('#session-table');
        this.summary = root.querySelector('#diag-summary');
        this.note = root.querySelector('#preview-note');
        this.lastRender = 0;
        this.hello = null;
        root.querySelector('#diag-toggle').addEventListener('click', (e) => {
            root.classList.toggle('collapsed');
            e.target.textContent = root.classList.contains('collapsed') ? 'show' : 'hide';
        });
        const form = root.querySelector('#preview-form');
        form.addEventListener('submit', (e) => {
            e.preventDefault();
            const msg = {
                type: 'preview',
                hz: Number(root.querySelector('#preview-hz').value),
                quality: Number(root.querySelector('#preview-quality').value),
                max_width: Number(root.querySelector('#preview-width').value),
            };
            const ok = sendPreview(msg);
            this.note.textContent = ok ? `sent hz ${msg.hz}, quality ${msg.quality}, max width ${msg.max_width}` : 'not connected';
        });
    }

    setHello(hello) {
        this.hello = hello;
        const i = hello.inspection || {};
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
        this.lastRender = 0;
    }

    updateSnapshot(snap) {
        const now = performance.now();
        if (now - this.lastRender < REFRESH_MS) {
            return;
        }
        this.lastRender = now;
        const w = snap.workers || {};
        const est = w.estimation || {};
        const fld = w.field || {};
        const insp = w.inspection || {};
        this.summary.textContent = `estimation ${fmt(est.rate_hz, 1)} Hz, field ${fmt(fld.rate_hz, 1)} Hz, ` +
            `inspection ${fmt(insp.snapshot_rate_hz, 1)} snap/s ${fmt(insp.frame_rate_hz, 1)} frames/s, ` +
            `${insp.clients} clients, ${(snap.sources || []).filter((s) => s.state !== 'valid').length} sources not valid`;

        fill(this.sources, ['id', 'kind', 'state', 'age', 'seq', 'epoch', 'diagnostic'],
            (snap.sources || []).map((s) => [
                s.id, s.kind, [s.state, `state-${s.state}`], fmtMs(s.receipt_age_ms), s.sequence, s.epoch, s.diagnostic || '',
            ]));

        const worker = (name, s) => [
            name, s.running ? 'yes' : 'no', s.cycles, fmt(s.rate_hz, 1), fmt(s.last_cycle_ms, 2), fmt(s.mean_cycle_ms, 2),
            fmt(s.max_cycle_ms, 1), s.period_target_ms, s.overruns, s.dropped, s.pending,
        ];
        const rows = [worker('estimation', est), worker('field', fld)];
        fill(this.workers, ['worker', 'run', 'cycles', 'Hz', 'last ms', 'mean ms', 'max ms', 'target ms', 'overruns', 'dropped', 'pending'], rows);
        const ir = el('tr');
        ir.appendChild(el('td', undefined, 'inspection'));
        const it = el('td');
        it.colSpan = 10;
        it.textContent = `${insp.snapshot_rate_hz !== undefined ? fmt(insp.snapshot_rate_hz, 1) : 'n/a'} snap/s, ${fmt(insp.frame_rate_hz, 1)} frames/s, ` +
            `sent ${insp.snapshots_sent} / skipped ${insp.snapshots_skipped} snapshots, ${insp.frames_sent} / ${insp.frames_skipped} frames, ` +
            `encode ${fmt(insp.last_encode_ms, 1)} ms (mean ${fmt(insp.mean_encode_ms, 1)}), ${insp.encodes} encodes, ` +
            `${(insp.bytes_sent / 1e6).toFixed(2)} MB, clients ${insp.clients} (${insp.clients_total} total, ${insp.client_disconnects} disconnects)`;
        ir.appendChild(it);
        this.workers.appendChild(ir);

        const loc = snap.localization || {};
        fill(this.loc, ['id', 'type', 'ready', 'note'],
            (loc.functions || []).map((f) => [f.id, f.type, [f.ready ? 'ready' : 'not ready', f.ready ? 'state-ok' : 'state-no_data'], f.note || '']));
        const lr = el('tr');
        const lt = el('td');
        lt.colSpan = 4;
        lt.textContent = `${loc.estimator_type}: ${loc.updates} updates, history ${loc.history_size}, publication ${loc.publication}, ` +
            `clock ${loc.clock_mapped ? 'mapped' : 'unmapped'}, ${loc.all_ready ? 'all ready' : 'not all ready'}`;
        lr.appendChild(lt);
        this.loc.appendChild(lr);

        const d = snap.diagnostics;
        const fnRows = [];
        const linkRows = [];
        if (d) {
            for (const [name, part] of [['estimation', d.estimation], ['field', d.field]]) {
                for (const f of (part && part.functions) || []) {
                    fnRows.push([name, f.label, f.runs, f.ok, f.no_data, [f.fault, f.fault > 0 ? 'state-fault' : undefined], [f.last, `state-${f.last}`]]);
                }
                for (const l of (part && part.links) || []) {
                    linkRows.push([name, l.id, l.bytes, l.packets, [l.decode_errors, l.decode_errors > 0 ? 'state-fault' : undefined], l.seq_gaps]);
                }
            }
        }
        fill(this.fn, ['worker', 'function', 'runs', 'ok', 'no data', 'fault', 'last'], fnRows);
        fill(this.links, ['worker', 'link', 'bytes', 'packets', 'decode err', 'seq gaps'], linkRows);

        const h = this.hello || {};
        const cfg = h.configuration || {};
        const fs = snap.field_snapshot || {};
        const target = snap.target;
        fill(this.session, ['key', 'value'], [
            ['session', `${snap.session.id} reset ${snap.session.reset_count}`],
            ['configuration', `${cfg.id || ''} ${cfg.digest ? 'digest ' + cfg.digest : ''}`],
            ['loop rate', `${cfg.loop_rate_hz !== undefined ? cfg.loop_rate_hz + ' Hz' : 'n/a'}`],
            ['cycle', `${snap.cycle} (${snap.running ? 'running' : 'stopped'}) host ${snap.host_ms} ms`],
            ['field snapshot', `${fs.status || 'n/a'} inv ${fs.invocation} age ${fmtMs(fs.age_ms)} ${fs.diagnostic || ''}`],
            ['target', target ? `${target.active ? target.target_id + ' ' + target.status : 'none'}` : 'n/a'],
            ['warnings', `${(h.warnings || []).length}`],
        ]);
    }
}
