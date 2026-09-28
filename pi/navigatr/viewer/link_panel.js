// link_panel.js
// The Brain link panel: a readiness line (link, profile, sensors, map,
// calibration, placement), the running Brain profile with its active wheel
// corrections, the raw wheel readings, IMU bias calibration, the Pico link
// and the bounded recovery event log, all from snapshot.brain_link,
// snapshot.pico and snapshot.events. Nothing here estimates or decides
// anything the Pi did not report; readiness is a reading of the snapshot,
// in the order a Brain program would wait for it.

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

// Up to six significant digits, no trailing zeros.
function compact(v) {
    return typeof v === 'number' && Number.isFinite(v) ? String(Number(v.toPrecision(6))) : 'n/a';
}

function hex32(v) {
    return typeof v === 'number' ? (v >>> 0).toString(16).padStart(8, '0') : String(v);
}

// Readiness items {key, text, cls} and an overall {state, text, cls}, or
// null when this configuration has no Brain link. cls is ok, info, warn or
// bad; the overall state is the first item that is not ok or info.
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
        this.readiness = root.querySelector('#readiness');
        this.profile = root.querySelector('#link-profile');
        this.calibration = root.querySelector('#link-calibration');
        this.wheels = root.querySelector('#wheel-table');
        this.pico = root.querySelector('#link-pico');
        this.events = root.querySelector('#event-log');
        this.lastRender = 0;
    }

    update(snap, hello) {
        const now = performance.now();
        if (now - this.lastRender < REFRESH_MS) {
            return;
        }
        this.lastRender = now;
        const link = snap.brain_link;
        this.root.hidden = !link && !snap.pico && !(snap.events || []).length;
        this.renderReadiness(snap, hello);
        this.renderProfile(link);
        this.renderCalibration(link);
        this.renderWheels(link);
        this.renderPico(snap.pico, link);
        this.renderEvents(snap);
    }

    renderReadiness(snap, hello) {
        const r = readinessOf(snap, hello);
        this.readiness.innerHTML = '';
        if (!r) {
            this.summary.textContent = 'no Brain link in this configuration';
            return;
        }
        this.summary.textContent = r.overall.text;
        for (const i of r.items) {
            const b = el('span', 'badge ' + i.cls, i.text);
            b.dataset.readiness = i.key;
            this.readiness.appendChild(b);
        }
    }

    renderProfile(link) {
        this.profile.innerHTML = '';
        if (!link) {
            return;
        }
        const p = link.profile || {};
        const running = p.running;
        if (!running) {
            this.profile.appendChild(el('div', 'muted', link.takes_profile
                ? 'no profile applied: localization waits'
                : 'this configuration localizes from its XML'));
            return;
        }
        const imu = running.imu || {};
        const imuText = imu.source === 'pico' ? `Pico IMU port ${imu.port}${imu.inverted ? ' inverted' : ''}`
            : imu.source === 'brain_vex' ? `Brain VEX IMU, Smart Port ${imu.vex_smart_port}` : 'no IMU';
        const f = running.footprint || {};
        const lines = [
            `${running.topology}, ${imuText}`,
            `footprint front ${fmt(f.front_m, 3)} back ${fmt(f.back_m, 3)} left ${fmt(f.left_m, 3)} right ${fmt(f.right_m, 3)} m`,
            `profile ${running.id}, apply ${running.generation}`,
        ];
        for (const l of lines) {
            this.profile.appendChild(el('div', undefined, l));
        }
    }

    renderCalibration(link) {
        this.calibration.innerHTML = '';
        const c = link && link.calibration;
        if (!c) {
            return;
        }
        const s = c.stillness || {};
        const bias = typeof s.bias_dps === 'number' ? `${fmt(s.bias_dps, 3)} deg/s` : 'none';
        this.calibration.appendChild(el('div', undefined,
            `IMU bias (${c.function}): ${s.calibration}, window ${fmt(s.progress_ms / 1000, 1)} / ${fmt(s.window_ms / 1000, 1)} s, ` +
            `bias ${bias}, attempts ${s.attempts}, windows ${s.windows}, steps ${s.steps}`));
        if (s.reason) {
            this.calibration.appendChild(el('div', 'muted', s.reason));
        }
    }

    renderWheels(link) {
        const table = this.wheels;
        table.innerHTML = '';
        const wheels = (link && link.wheels) || [];
        if (!wheels.length) {
            return;
        }
        const head = el('tr');
        for (const h of ['port', 'counts', 'travel m', 'fresh', 'disc', 'cpr', 'gear', 'r m', 'scale', 'rev']) {
            head.appendChild(el('th', undefined, h));
        }
        table.appendChild(head);
        for (const w of wheels) {
            const tr = el('tr');
            const cells = [
                w.port, w.counts, fmt(w.travel_m, 4),
                [w.fresh ? 'yes' : `no (${fmtMs(w.age_ms)})`, w.fresh ? 'state-ok' : 'state-fault'],
                w.discontinuity, w.counts_per_rev, compact(w.gear), compact(w.radius_m),
                compact(w.travel_scale), w.reversed ? 'yes' : 'no',
            ];
            for (const c of cells) {
                const cell = Array.isArray(c) ? c : [c];
                tr.appendChild(el('td', cell[1], String(cell[0])));
            }
            tr.title = 'raw travel: counts per revolution, gear, polarity and radius applied, never the travel scale';
            table.appendChild(tr);
        }
    }

    renderPico(pico, link) {
        this.pico.innerHTML = '';
        if (!pico) {
            return;
        }
        let text = `Pico ${pico.resource_id}: frames ${pico.frames_fresh ? 'fresh' : 'lost'}`;
        if (!pico.identity) {
            text += pico.frames_fresh ? ', v1 firmware (no identity, no commands)' : '';
        } else {
            text += `, boot ${pico.boot_id}, acquisition epoch ${pico.acq_epoch}, IMU epoch ${pico.imu_epoch}` +
                `, ${pico.reboots} reboots`;
        }
        this.pico.appendChild(el('div', undefined, text));
        if (pico.status_known && pico.imu) {
            const imu = pico.imu;
            const used = link && link.profile && link.profile.running && link.profile.running.imu.source === 'pico';
            this.pico.appendChild(el('div', undefined,
                `${pico.firmware} IMU ${imu.enabled ? imu.state : 'disabled'}` +
                (imu.reason !== 'none' ? ` (${imu.reason})` : '') + `, attempts ${imu.attempts}` +
                (used ? '' : ', not used by the profile')));
            const c = pico.last_command;
            if (c && c.request_id) {
                this.pico.appendChild(el('div', 'muted',
                    `last command ${c.op} #${c.request_id}: ${c.status}${c.detail !== 'none' ? ' ' + c.detail : ''}`));
            }
        }
        const op = link && link.operation;
        if (op) {
            this.pico.appendChild(el('div', op.result === 'failed' ? 'state-fault' : undefined,
                `CONTROL ${op.action}: ${op.result}${op.detail !== 'none' ? ' ' + op.detail : ''}, ${fmtMs(op.age_ms)}`));
        }
    }

    renderEvents(snap) {
        this.events.innerHTML = '';
        const events = snap.events || [];
        for (let i = events.length - 1; i >= 0; --i) {
            const e = events[i];
            const ago = typeof e.host_ms === 'number' ? fmtMs(snap.host_ms - e.host_ms) : 'n/a';
            const row = el('div', 'event');
            row.appendChild(el('span', 'muted', `-${ago} `));
            row.appendChild(el('span', /lost|failed|refused|place again/.test(e.text) ? 'state-fault' : undefined, e.text));
            this.events.appendChild(row);
        }
    }
}
