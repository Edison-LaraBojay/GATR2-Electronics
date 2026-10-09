// instrumentation_panel.js
// Transport and hardware instrumentation: per-link byte and frame counters,
// rates, last-valid ages and decode rejections, optional raw bytes (hex)
// and decoded message lists, the Pico diagnostic frame and the Brain
// telemetry state, from the server's instrumentation message. The panel
// subscribes only while its tab is visible; raw and decoded capture on the
// Pi run only while someone asked for them (the checkboxes here).
//
// Wording is deliberate. Pin levels are logic levels the Pico read back
// with gpio_get, not voltages or signal quality; software sampling can miss
// fast edges; a UART line idles HIGH, so a steady HIGH proves nothing about
// a connection. Unchanging encoder counts cannot tell a stationary wheel
// from a disconnected encoder. Fresh Pico frames do not prove every
// attached sensor works.

import { fmt, fmtMs } from './transforms.js';
import { el, setText, setClass, setHidden, KeyedTable, Lines } from './dom.js';

const PIN_NOTE = 'Logic level read back by the Pico (gpio_get), not a voltage or signal-quality measurement. ' +
    'Sampled in software: fast transitions can be missed. UART lines idle HIGH, so a steady HIGH does not show a connection or a disconnection.';
const ENCODER_NOTE = 'No change in counts cannot tell a stationary wheel from a disconnected encoder.';

function ms(v) {
    return typeof v === 'number' ? fmtMs(v) : 'never';
}

function n(v) {
    return typeof v === 'number' ? String(v) : 'n/a';
}

function kvText(obj, skip) {
    if (!obj || typeof obj !== 'object') {
        return 'n/a';
    }
    const parts = [];
    for (const [k, v] of Object.entries(obj)) {
        if (skip && skip.includes(k)) {
            continue;
        }
        if (v !== null && typeof v === 'object') {
            parts.push(`${k} {${kvText(v)}}`);
        } else {
            parts.push(`${k} ${typeof v === 'number' && !Number.isInteger(v) ? fmt(v, 3) : v}`);
        }
    }
    return parts.join(', ');
}

// Hex bytes spaced in pairs for reading: "AA55 0102" -> "AA 55 01 02".
function spaced(hex) {
    return String(hex || '').replace(/\s+/g, '').replace(/(..)(?!$)/g, '$1 ').toUpperCase();
}

export class InstrumentationPanel {
    constructor(root, opts) {
        this.root = root;
        this.onOptions = opts.onOptions || (() => {});
        this.rawBox = root.querySelector('#inst-raw');
        this.decodedBox = root.querySelector('#inst-decoded');
        this.status = root.querySelector('#inst-status');
        this.links = new KeyedTable(root.querySelector('#inst-links'),
            ['link', 'kind', 'rx B', 'tx tried B', 'tx ok B', 'rx fr', 'tx fr', 'rejected',
                'reader B/fr/sync/len/crc', 'rx B/s', 'tx B/s', 'rx fr/s', 'last rx', 'last valid', 'last tx']);
        this.linksNote = root.querySelector('#inst-links-note');
        this.errors = new KeyedTable(root.querySelector('#inst-errors'), ['link', 'ago', 'rejection reason']);
        this.decoded = new KeyedTable(root.querySelector('#inst-decoded-table'), ['link', 'ago', 'dir', 'message', 'fields']);
        this.rawPre = root.querySelector('#inst-raw-view');
        this.decodedHead = root.querySelector('#inst-decoded-head');
        this.pico = new Lines(root.querySelector('#inst-pico'));
        this.pins = new KeyedTable(root.querySelector('#inst-pins'), ['pin', 'logic level', 'known', 'driven by Pico']);
        this.encoders = new KeyedTable(root.querySelector('#inst-encoders'),
            ['port', 'counts', 'change', 'over ms', 'direction', 'A level', 'B level', 'note']);
        this.serverNotes = root.querySelector('#inst-server-notes');
        this.brain = new Lines(root.querySelector('#inst-brain'));
        this.hub = new Lines(root.querySelector('#inst-hub'));
        root.querySelector('#inst-pin-note').textContent = PIN_NOTE;
        root.querySelector('#inst-encoder-note').textContent = ENCODER_NOTE;
        this.rawBox.addEventListener('change', () => this.optionsChanged());
        this.decodedBox.addEventListener('change', () => this.optionsChanged());
        this.rawLines = [];
    }

    options() {
        return { raw: this.rawBox.checked, decoded: this.decodedBox.checked };
    }

    optionsChanged() {
        this.onOptions(this.options());
        setHidden(this.rawPre, !this.rawBox.checked);
        setHidden(this.decoded.table, !this.decodedBox.checked);
        setHidden(this.decodedHead, !this.decodedBox.checked);
    }

    // msg: newest instrumentation message or null; age: ms since receipt.
    render(msg, ageMs, supported) {
        if (!msg) {
            setText(this.status, supported === false
                ? 'this runtime sends no instrumentation (inspect/1 server): counters are in the Diagnostics tab'
                : 'waiting for instrumentation...');
            return;
        }
        setText(this.status, `updated ${fmtMs(ageMs)} ago; ages computed on the Pi when the message was built` +
            (ageMs > 2000 ? ' (STALE: no instrumentation update)' : ''));
        setClass(this.status, ageMs > 2000 ? 'state-fault' : 'muted');
        const links = msg.links || [];
        this.links.begin();
        this.errors.begin();
        this.decoded.begin();
        const rawOut = [];
        for (const l of links) {
            const r = l.reader || {};
            const rates = l.rates || {};
            this.links.row(l.id, [
                l.id, l.kind, n(l.rx_bytes), n(l.tx_attempted), n(l.tx_accepted), n(l.rx_frames), n(l.tx_frames),
                [n(l.rejected), l.rejected > 0 ? 'state-fault' : ''],
                `${n(r.bytes)} / ${n(r.frames)} / ${n(r.sync_dropped)} / ${n(r.length_errors)} / ${n(r.check_errors)}`,
                fmt(rates.rx_bytes_s, 0), fmt(rates.tx_bytes_s, 0), fmt(rates.rx_frames_s, 1),
                ms(l.last_rx_ms_ago), ms(l.last_valid_rx_ms_ago), ms(l.last_tx_ms_ago),
            ], { title: 'tx attempted: bytes handed to write(); tx accepted: bytes the link reported written. Only accepted bytes can reach the other side.' });
            (l.errors || []).slice(-12).forEach((e, k) => {
                this.errors.row(`${l.id}#${k}`, [l.id, fmtMs(e.ms_ago), e.reason]);
            });
            if (this.decodedBox.checked) {
                (l.decoded || []).slice(-24).forEach((d, k) => {
                    this.decoded.row(`${l.id}#${k}`, [l.id, fmtMs(d.ms_ago), d.dir, d.name,
                        typeof d.fields === 'string' ? d.fields : kvText(d.fields)]);
                });
            }
            if (this.rawBox.checked) {
                rawOut.push(`${l.id} (${l.kind})${l.raw_on ? '' : ': raw capture off on the Pi'}`);
                for (const c of l.raw || []) {
                    const dir = c.dir === 'rx' ? 'RX        ' : c.dir === 'tx' ? 'TX ok     ' : 'TX tried  ';
                    rawOut.push(`  -${String(Math.round(c.ms_ago)).padStart(6)} ms ${dir} ${spaced(c.hex)}`);
                }
            }
        }
        this.links.end();
        this.errors.end();
        this.decoded.end();
        setHidden(this.links.table, links.length === 0);
        setText(this.linksNote, links.length ? '' : 'no serial links in this configuration');
        setHidden(this.errors.table, this.errors.rows.size === 0);
        setHidden(this.decoded.table, !this.decodedBox.checked || this.decoded.rows.size === 0);
        setHidden(this.decodedHead, !this.decodedBox.checked);
        if (this.rawBox.checked) {
            const text = rawOut.join('\n') || 'no raw bytes yet';
            setText(this.rawPre, text + '\n\nTX ok = accepted by the link; TX tried = handed to write() (attempted), may not have been sent.');
        }
        this.renderPico(msg.pico);
        this.renderBrain(msg.brain);
        this.renderHub(msg.hub);
    }

    renderPico(p) {
        const out = this.pico;
        out.begin();
        this.pins.begin();
        this.encoders.begin();
        if (!p) {
            out.line('no Pico in this configuration', 'muted');
        } else {
            out.line(p.available ? 'Pico diagnostics available' : `Pico diagnostics unavailable: ${p.reason || 'unknown'}`,
                p.available ? '' : 'state-no_data');
            if (p.status) {
                out.line('status: ' + kvText(p.status));
            }
            const d = p.diag;
            if (d) {
                out.line(`diag frame seq ${n(d.seq)}, age ${ms(d.age_ms)}, firmware ${d.firmware}` +
                    (d.boot_id !== undefined ? `, boot ${d.boot_id}${d.current_boot === false ? ' (an EARLIER boot)' : ''}` : '') +
                    `, link rx bad ${n(d.link_rx_bad)}, sensor ticks skipped (TX FIFO busy) ${n(d.ticks_skipped)}`);
                if (d.imu) {
                    out.line('IMU: ' + kvText(d.imu, ['meaning']), '');
                    if (d.imu.meaning) {
                        out.line(d.imu.meaning, 'muted');
                    }
                }
                for (const pin of d.pins || []) {
                    this.pins.row(pin.name, [pin.name, pin.known && pin.level ? pin.level : 'not sampled', pin.known ? 'yes' : 'no',
                        pin.driven ? 'yes (output)' : 'no (input)'], { title: PIN_NOTE });
                }
            } else if (p.available) {
                out.line('no diagnostic frame received yet', 'muted');
            }
            for (const e of p.encoders || []) {
                const lvl = (v) => (v === 'HIGH' || v === 'LOW' ? v : 'n/a');
                this.encoders.row('p' + e.port, [e.port, e.present === false ? 'none' : n(e.counts), n(e.delta_1s), n(e.span_ms),
                    e.direction || 'n/a', lvl(e.a), lvl(e.b), e.note || ''], { title: ENCODER_NOTE });
            }
        }
        this.pins.end();
        this.encoders.end();
        setHidden(this.pins.table, this.pins.rows.size === 0);
        setHidden(this.encoders.table, this.encoders.rows.size === 0);
        const notes = [];
        if (p && p.diag && p.diag.pins_note) {
            notes.push('Pins (from the Pi): ' + p.diag.pins_note);
        }
        if (p && p.encoders_note) {
            notes.push('Encoders (from the Pi): ' + p.encoders_note);
        }
        setText(this.serverNotes, notes.join(' '));
        out.end();
    }

    renderBrain(b) {
        const out = this.brain;
        out.begin();
        if (!b) {
            out.line('no Brain link in this configuration', 'muted');
        } else {
            const sup = b.telemetry_supported;
            out.line(`telemetry ${sup === null || sup === undefined ? 'support not known yet (no report received)' : (sup ? 'reports arriving' : 'not supported by this Brain program')}` +
                (typeof b.telemetry_age_ms === 'number' ? `, last ${fmtMs(b.telemetry_age_ms)} ago` : '') +
                (b.telemetry_reports !== undefined ? `, ${b.telemetry_reports} reports` : ''));
            if (b.telemetry) {
                out.line('newest report: ' + kvText(b.telemetry), 'muted');
            }
            if (b.vex_imu) {
                out.line('VEX IMU (bench sample via GET_STATE): ' + kvText(b.vex_imu));
            }
        }
        out.end();
    }

    renderHub(h) {
        const out = this.hub;
        out.begin();
        if (h) {
            out.line(`hub posted: ${kvText(h.posted)}`);
            out.line(`hub dropped (ring full): ${kvText(h.dropped)}; queued ${n(h.queued)}` +
                (h.capacity !== undefined ? ` of ${h.capacity}` : ''),
                h.dropped && Object.values(h.dropped).some((v) => v > 0) ? 'state-fault' : '');
        }
        out.end();
    }
}
