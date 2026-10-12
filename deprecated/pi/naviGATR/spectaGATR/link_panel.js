// link_panel.js
// The Brain link panel: a readiness line (link, profile, sensors, map,
// calibration, placement), the running Brain profile with its active wheel
// corrections, the raw wheel readings, IMU bias calibration, the Pico link
// and the bounded recovery event log. brain_link and pico come from the
// newest diag message, the robot from the newest state, events from the
// event stream. Nothing here estimates or decides anything the Pi did not
// report; readiness is a reading of those documents, in the order a Brain
// program would wait for them. Rows update in place.

import { fmt, fmtMs } from './transforms.js';
import { el, setText, setClass, setHidden, BadgeSet, Lines, KeyedTable } from './dom.js';

const EVENT_ROWS = 60;

// Up to six significant digits, no trailing zeros.
function compact(v) {
    return typeof v === 'number' && Number.isFinite(v) ? String(Number(v.toPrecision(6))) : 'n/a';
}

function hex32(v) {
    return typeof v === 'number' ? (v >>> 0).toString(16).padStart(8, '0') : String(v);
}

// Readiness items {key, text, cls} and an overall {state, text, cls}, or
// null when this configuration has no Brain link. cls is ok, info, warn or
// bad; the overall state is the first item that is not ok or info. snap is
// {robot, brain_link}: the state's robot and the diag's brain_link.
export function readinessOf(snap, hello) {
    const link = snap && snap.brain_link;
    if (!link) {
        return null;
    }
    const items = [];
    const add = (key, text, cls) => items.push({ key, text, cls });
    const st = link.state;
    const robot = snap.robot || {};
    const profile = link.profile || {};
    const running = profile.running;

    if (!link.link_open) {
        add('link', 'link closed', 'bad');
    } else if (!link.session) {
        add('link', 'no Brain session', 'warn');
    } else if (typeof link.last_request_age_ms === 'number' && link.last_request_age_ms > 1000) {
        add('link', `link quiet ${fmtMs(link.last_request_age_ms)}`, 'warn');
    } else {
        add('link', `link session ${hex32(link.session)}`, 'ok');
    }

    if (link.takes_profile) {
        switch (profile.state) {
        case 'applied': add('profile', `profile ${profile.applied_id}`, 'ok'); break;
        case 'applying': add('profile', `profile ${profile.id} applying`, 'info'); break;
        case 'rejected':
            add('profile', `profile ${profile.id} rejected: ${profile.reason} ${profile.detail}` +
                (profile.applied_id ? `; ${profile.applied_id} runs` : ''), 'bad');
            break;
        default: add('profile', 'waiting for a Brain profile', 'warn'); break;
        }
    } else {
        add('profile', 'XML localization', 'info');
    }

    if (running && st) {
        const stale = [];
        if (!st.health.encoders_fresh) {
            stale.push('encoders');
        }
        if (running.imu && running.imu.source !== 'none' && !st.health.gyro_fresh) {
            stale.push(running.imu.source === 'brain_vex' ? 'VEX IMU' : 'Pico IMU');
        }
        add('sensors', stale.length ? `stale: ${stale.join(', ')}` : 'sensors fresh', stale.length ? 'bad' : 'ok');
    }

    if (st && st.map_id) {
        let revision = '?';
        for (const f of (hello && hello.fields) || []) {
            if (f.map_id === st.map_id) {
                revision = f.revision;
            }
        }
        add('map', `map ${st.map_id} rev ${revision}, estimate ${st.estimate_id}`, 'ok');
    } else {
        add('map', 'no field served', 'info');
    }

    if (st) {
        const cal = link.calibration ? link.calibration.stillness : null;
        const progress = cal && cal.window_ms > 0 ? ` ${fmt(cal.progress_ms / 1000, 1)} / ${fmt(cal.window_ms / 1000, 1)} s` : '';
        switch (st.calibration) {
        case 'none': add('calibration', running && running.imu.source === 'brain_vex'
            ? 'IMU calibrated on the Brain' : 'no Pi IMU calibration', 'info'); break;
        case 'done': add('calibration', 'IMU bias calibrated', 'ok'); break;
        case 'collecting': add('calibration', `calibrating, hold still${progress}`, 'warn'); break;
        case 'waiting for stillness': add('calibration', 'calibration waits for stillness', 'warn'); break;
        case 'waiting for data': add('calibration', 'calibration waits for IMU and wheel data', 'warn'); break;
        case 'failed': add('calibration', 'calibration failed: recalibrate when still', 'bad'); break;
        default: add('calibration', `calibration ${st.calibration}`, 'info'); break;
        }
    }

    if (robot.valid && robot.initialized) {
        add('placement', 'placed', 'ok');
    } else if (!link.takes_profile || profile.state === 'applied') {
        add('placement', 'needs placement', 'warn');
    }

    const blocking = items.find((i) => i.cls === 'bad' || i.cls === 'warn');
    const overall = blocking
        ? { state: blocking.key, text: blocking.text, cls: blocking.cls }
        : { state: 'ready', text: 'ready', cls: 'ok' };
    return { items, overall };
}

export class LinkPanel {
    constructor(root) {
        this.root = root;
        this.summary = root.querySelector('#link-summary');
        this.readiness = new BadgeSet(root.querySelector('#readiness'));
        this.profile = new Lines(root.querySelector('#link-profile'));
        this.calibration = new Lines(root.querySelector('#link-calibration'));
        this.wheels = new KeyedTable(root.querySelector('#wheel-table'),
            ['port', 'counts', 'travel m', 'fresh', 'disc', 'cpr', 'gear', 'r m', 'scale', 'rev']);
        this.pico = new Lines(root.querySelector('#link-pico'));
        this.events = root.querySelector('#event-log');
        this.eventRows = [];
    }

    // Whether the panel has anything to show (else it stays hidden).
    wanted(diag, events) {
        return !!(diag && (diag.brain_link || diag.pico)) || events.length > 0;
    }

    // view: {robot, brain_link}; diag: newest diag; events: feed events;
    // piNow: the Pi clock now (estimated) for event ages.
    render(view, diag, hello, events, piNow) {
        const link = diag ? diag.brain_link : null;
        this.renderReadiness(view, hello);
        this.renderProfile(link);
        this.renderCalibration(link);
        this.renderWheels(link);
        this.renderPico(diag ? diag.pico : null, link);
        this.renderEvents(events, piNow);
    }

    renderReadiness(view, hello) {
        const r = readinessOf(view, hello);
        this.readiness.begin();
        if (!r) {
            setText(this.summary, 'no Brain link in this configuration');
            this.readiness.end();
            return;
        }
        setText(this.summary, r.overall.text);
        for (const i of r.items) {
            this.readiness.badge(i.key, i.text, i.cls).dataset.readiness = i.key;
        }
        this.readiness.end();
    }

    renderProfile(link) {
        const out = this.profile;
        out.begin();
        if (link) {
            const p = link.profile || {};
            const running = p.running;
            if (!running) {
                out.line(link.takes_profile ? 'no profile applied: localization waits' : 'this configuration localizes from its XML', 'muted');
            } else {
                const imu = running.imu || {};
                const imuText = imu.source === 'pico' ? `Pico IMU port ${imu.port}${imu.inverted ? ' inverted' : ''}`
                    : imu.source === 'brain_vex' ? `Brain VEX IMU, Smart Port ${imu.vex_smart_port}` : 'no IMU';
                const f = running.footprint || {};
                out.line(`${running.topology}, ${imuText}`);
                out.line(`footprint front ${fmt(f.front_m, 3)} back ${fmt(f.back_m, 3)} left ${fmt(f.left_m, 3)} right ${fmt(f.right_m, 3)} m`);
                out.line(`profile ${running.id}, apply ${running.generation}`);
            }
        }
        out.end();
    }

    renderCalibration(link) {
        const out = this.calibration;
        out.begin();
        const c = link && link.calibration;
        if (c) {
            const s = c.stillness || {};
            const bias = typeof s.bias_dps === 'number' ? `${fmt(s.bias_dps, 3)} deg/s` : 'none';
            out.line(`IMU bias (${c.function}): ${s.calibration}, window ${fmt(s.progress_ms / 1000, 1)} / ${fmt(s.window_ms / 1000, 1)} s, ` +
                `bias ${bias}, attempts ${s.attempts}, windows ${s.windows}, steps ${s.steps}`);
            if (s.reason) {
                out.line(s.reason, 'muted');
            }
        }
        out.end();
    }

    renderWheels(link) {
        const wheels = (link && link.wheels) || [];
        const t = this.wheels;
        setHidden(t.table, wheels.length === 0);
        t.begin();
        for (const w of wheels) {
            t.row('p' + w.port, [
                w.port, w.counts, fmt(w.travel_m, 4),
                [w.fresh ? 'yes' : `no (${fmtMs(w.age_ms)})`, w.fresh ? 'state-ok' : 'state-fault'],
                w.discontinuity, w.counts_per_rev, compact(w.gear), compact(w.radius_m),
                compact(w.travel_scale), w.reversed ? 'yes' : 'no',
            ], { title: 'raw travel: counts per revolution, gear, polarity and radius applied, never the travel scale. ' +
                'Unchanging counts cannot tell a stationary wheel from a disconnected encoder.' });
        }
        t.end();
    }

    renderPico(pico, link) {
        const out = this.pico;
        out.begin();
        if (pico) {
            let text = `Pico ${pico.resource_id}: frames ${pico.frames_fresh ? 'fresh' : 'lost'}`;
            if (!pico.identity) {
                text += pico.frames_fresh ? ', v1 firmware (no identity, no commands)' : '';
            } else {
                text += `, boot ${pico.boot_id}, acquisition epoch ${pico.acq_epoch}, IMU epoch ${pico.imu_epoch}` +
                    `, ${pico.reboots} reboots`;
            }
            out.line(text);
            if (pico.status_known && pico.imu) {
                const imu = pico.imu;
                const used = link && link.profile && link.profile.running && link.profile.running.imu.source === 'pico';
                out.line(`${pico.firmware} IMU ${imu.enabled ? imu.state : 'disabled'}` +
                    (imu.reason !== 'none' ? ` (${imu.reason})` : '') + `, attempts ${imu.attempts}` +
                    (used ? '' : ', not used by the profile'));
                const c = pico.last_command;
                if (c && c.request_id) {
                    out.line(`last command ${c.op} #${c.request_id}: ${c.status}${c.detail !== 'none' ? ' ' + c.detail : ''}`, 'muted');
                }
            }
        }
        const op = link && link.operation;
        if (op) {
            out.line(`CONTROL ${op.action}: ${op.result}${op.detail !== 'none' ? ' ' + op.detail : ''}, ${fmtMs(op.age_ms)}`,
                op.result === 'failed' ? 'state-fault' : '');
        }
        out.end();
    }

    // Newest first, a fixed pool of rows; ages from the Pi clock estimate.
    renderEvents(events, piNow) {
        const n = Math.min(EVENT_ROWS, events.length);
        while (this.eventRows.length < n) {
            const row = el('div', 'event');
            const ago = el('span', 'muted');
            const text = el('span');
            row.appendChild(ago);
            row.appendChild(text);
            this.events.appendChild(row);
            this.eventRows.push({ row, ago, text });
        }
        for (let k = 0; k < this.eventRows.length; ++k) {
            const r = this.eventRows[k];
            if (k >= n) {
                setHidden(r.row, true);
                continue;
            }
            const e = events[events.length - 1 - k];
            setHidden(r.row, false);
            const ago = typeof e.host_ms === 'number' && Number.isFinite(piNow)
                ? fmtMs(Math.max(0, Math.round((piNow - e.host_ms) / 100) * 100)) : 'n/a';
            setText(r.ago, `-${ago} `);
            setText(r.text, e.text);
            setClass(r.text, /lost|failed|refused|place again/.test(e.text) ? 'state-fault' : '');
        }
    }
}
