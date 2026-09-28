// camera_panel.js
// The camera preview: the JPEG from a binary WebSocket message drawn at its
// preview size, with tag overlays taken only from a detection_frames entry
// whose (camera, epoch, sequence) is the image's own identity: the entry
// carried in the frame header when the server sends one, else one from the
// last few diag messages kept per camera. An image with no matching entry
// gets no overlay and says so. Corners are scaled from captured-image
// pixels by preview_width_px / width_px. Metric axes are projected through
// the entry's intrinsics (transforms.js) and only when the tag has a pose.
//
// Messages only store; the canvas, badges and tag list are redrawn by
// render() at the page's panel rate while the panel is visible.

import { transformPoint, projectEngineering, fmt, fmtMs } from './transforms.js';
import { BadgeSet, KeyedTable, setText, setClass } from './dom.js';

const ENTRY_KEEP = 12;
const COLORS = { accepted: '#3ddc84', rejected: '#ff5252', none: '#ffd54f' };

function associationState(tag) {
    const a = tag.association;
    if (!a) {
        return 'none';
    }
    if (a.accepted) {
        return 'accepted';
    }
    return a.rejection ? 'rejected' : 'none';
}

function associationText(tag) {
    const a = tag.association;
    if (!a) {
        return 'no association';
    }
    if (a.accepted) {
        return `accepted: ${a.object} / ${a.mount}`;
    }
    return `rejected: ${a.rejection || 'no reason given'}`;
}

function pointInPolygon(p, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        const [xi, yi] = poly[i];
        const [xj, yj] = poly[j];
        const cross = ((yi > p[1]) !== (yj > p[1])) && (p[0] < (xj - xi) * (p[1] - yi) / (yj - yi) + xi);
        if (cross) {
            inside = !inside;
        }
    }
    return inside;
}

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

function row(table, cells, cls) {
    const tr = el('tr', cls);
    for (const c of cells) {
        const td = el('td', undefined, c);
        tr.appendChild(td);
    }
    table.appendChild(tr);
    return tr;
}

export class CameraPanel {
    constructor(root) {
        this.select = root.querySelector('#camera-select');
        this.canvas = root.querySelector('#camera-canvas');
        this.ctx = this.canvas.getContext('2d');
        this.empty = root.querySelector('#camera-empty');
        this.badges = root.querySelector('#camera-badges');
        this.frameInfo = root.querySelector('#camera-frame-info');
        this.overlayNote = root.querySelector('#camera-overlay-note');
        this.tagList = root.querySelector('#tag-list');
        this.tagDetails = root.querySelector('#tag-details');
        this.cameras = [];
        this.current = null;
        this.images = new Map();     // camera -> {header, bitmap, serial}
        this.entries = new Map();    // camera -> Map(identity -> {entry, cycle})
        this.latest = new Map();     // camera -> newest detection entry
        this.selected = null;        // tag index in the drawn entry
        this.hover = null;
        this.serial = 0;
        this.generation = 0;
        this.hello = null;
        this.dirty = true;
        this.axisLengths = new Map();
        this.piNow = () => NaN;
        this.badgeSet = new BadgeSet(this.badges);
        this.tagRows = new Map();
        this.select.addEventListener('change', () => {
            this.current = this.select.value;
            this.selected = null;
            this.redraw();
        });
        this.canvas.addEventListener('click', (e) => this.onClick(e));
        this.canvas.addEventListener('pointermove', (e) => this.onHover(e));
        this.canvas.addEventListener('pointerleave', () => {
            this.hover = null;
            this.canvas.title = '';
            this.redraw();
        });
    }

    setHello(hello) {
        this.hello = hello;
        this.axisLengths.clear();
        this.addCameras(hello.camera_sensors || []);
        this.dirty = true;
    }

    clear() {
        this.generation += 1;
        for (const img of this.images.values()) {
            if (img.bitmap && img.bitmap.close) {
                img.bitmap.close();
            }
        }
        this.images.clear();
        this.entries.clear();
        this.latest.clear();
        this.selected = null;
        this.dirty = true;
    }

    addCameras(ids) {
        let changed = false;
        for (const id of ids) {
            if (!this.cameras.includes(id)) {
                this.cameras.push(id);
                changed = true;
            }
        }
        if (changed) {
            this.select.innerHTML = '';
            for (const id of this.cameras) {
                const o = el('option', undefined, id);
                o.value = id;
                this.select.appendChild(o);
            }
            if (!this.current || !this.cameras.includes(this.current)) {
                this.current = this.cameras[0] || null;
            }
            this.select.value = this.current;
        }
    }

    keepEntry(f, hostMs) {
        let map = this.entries.get(f.camera);
        if (!map) {
            map = new Map();
            this.entries.set(f.camera, map);
        }
        const key = `${f.epoch}:${f.sequence}`;
        if (!map.has(key)) {
            map.set(key, { entry: f, host_ms: hostMs });
            while (map.size > ENTRY_KEEP) {
                map.delete(map.keys().next().value);
            }
        }
        const prev = this.latest.get(f.camera);
        if (!prev || prev.epoch !== f.epoch || prev.sequence <= f.sequence) {
            this.latest.set(f.camera, f);
        }
    }

    // detection_frames of a diag message (or an inspect/1 snapshot).
    updateDiag(diag) {
        const frames = diag.detection_frames || [];
        if (frames.length) {
            this.addCameras(frames.map((f) => f.camera));
        }
        for (const f of frames) {
            this.keepEntry(f, diag.host_ms);
        }
        this.dirty = true;
    }

    onFrame(header, jpeg) {
        this.addCameras([header.camera]);
        // a server that binds the detection to the image sends it along
        if (header.detection && typeof header.detection === 'object') {
            this.keepEntry(header.detection, header.host_ms);
        }
        const serial = ++this.serial;
        const generation = this.generation;
        const blob = new Blob([jpeg], { type: header.format || 'image/jpeg' });
        const store = (bitmap) => {
            const prev = this.images.get(header.camera);
            if (generation !== this.generation || (prev && prev.serial > serial)) {
                if (bitmap.close) {
                    bitmap.close();
                }
                return;   // a newer frame already landed
            }
            if (prev && prev.bitmap && prev.bitmap.close) {
                prev.bitmap.close();
            }
            this.images.set(header.camera, { header, bitmap, serial, receivedAt: performance.now() });
            if (header.camera === this.current) {
                this.dirty = true;
            }
        };
        if (typeof createImageBitmap === 'function') {
            createImageBitmap(blob).then(store).catch((e) => {
                if (window.__navigatr) {
                    window.__navigatr.errors.push('preview decode failed: ' + e);
                }
            });
        } else {
            const url = URL.createObjectURL(blob);
            const img = new Image();
            img.onload = () => {
                URL.revokeObjectURL(url);
                store(img);
            };
            img.src = url;
        }
    }

    // The detection entry with exactly the image's identity, if known.
    matchingEntry(header) {
        const map = this.entries.get(header.camera);
        if (!map) {
            return null;
        }
        const hit = map.get(`${header.epoch}:${header.sequence}`);
        return hit ? hit.entry : null;
    }

    drawnTags() {
        const img = this.current ? this.images.get(this.current) : null;
        if (!img) {
            return { img: null, entry: null, tags: [], sx: 1, sy: 1 };
        }
        const entry = this.matchingEntry(img.header);
        if (!entry) {
            return { img, entry: null, tags: [], sx: 1, sy: 1 };
        }
        const sx = img.header.preview_width_px / entry.width_px;
        const sy = img.header.preview_height_px / entry.height_px;
        const tags = (entry.tags || []).map((t) => ({
            tag: t,
            poly: t.corners_px.map(([u, v]) => [u * sx, v * sy]),
            center: [t.center_px[0] * sx, t.center_px[1] * sy],
        }));
        return { img, entry, tags, sx, sy };
    }

    canvasPoint(e) {
        const r = this.canvas.getBoundingClientRect();
        return [(e.clientX - r.left) * this.canvas.width / r.width, (e.clientY - r.top) * this.canvas.height / r.height];
    }

    onClick(e) {
        const p = this.canvasPoint(e);
        const { tags } = this.drawnTags();
        const hit = tags.find((t) => pointInPolygon(p, t.poly));
        this.selected = hit ? hit.tag.index : null;
        this.redraw();
    }

    onHover(e) {
        const p = this.canvasPoint(e);
        const { tags } = this.drawnTags();
        const hit = tags.find((t) => pointInPolygon(p, t.poly));
        const idx = hit ? hit.tag.index : null;
        this.canvas.title = hit ? `${hit.tag.family} id ${hit.tag.id}: ${associationText(hit.tag)}` : '';
        if (idx !== this.hover) {
            this.hover = idx;
            this.redraw();
        }
    }

    selectTag(index) {
        this.selected = index;
        this.redraw();
    }

    // Axis length in meters: twice the detection size of the tag's family
    // when a mount of that family is configured, else 3 cm. Cached per hello.
    axisLength(family) {
        const cached = this.axisLengths.get(family);
        if (cached !== undefined) {
            return cached;
        }
        let len = 0.03;
        const fields = (this.hello && this.hello.fields) || [];
        search:
        for (const f of fields) {
            for (const lm of f.landmarks || []) {
                for (const m of lm.mounts || []) {
                    if (m.family === family && m.detection_size_m > 0) {
                        len = m.detection_size_m * 2;
                        break search;
                    }
                }
            }
        }
        this.axisLengths.set(family, len);
        return len;
    }

    // Panel tick: redraw when something changed; the age line always.
    render() {
        if (this.dirty) {
            this.redraw();
        } else {
            this.updateAge();
        }
    }

    redraw() {
        this.dirty = false;
        const { img, entry, tags, sx, sy } = this.drawnTags();
        const latest = this.current ? this.latest.get(this.current) : null;
        const badges = [];
        const ctx = this.ctx;

        if (!img) {
            if (this.canvas.width !== 320 || this.canvas.height !== 240) {
                this.canvas.width = 320;
                this.canvas.height = 240;
            }
            ctx.fillStyle = '#000';
            ctx.fillRect(0, 0, 320, 240);
            this.empty.hidden = false;
            badges.push(['no image', 'warn']);
        } else {
            this.empty.hidden = true;
            const h = img.header;
            if (this.canvas.width !== h.preview_width_px || this.canvas.height !== h.preview_height_px) {
                this.canvas.width = h.preview_width_px;
                this.canvas.height = h.preview_height_px;
            }
            ctx.drawImage(img.bitmap, 0, 0, this.canvas.width, this.canvas.height);
            const fontPx = Math.max(11, Math.min(18, this.canvas.width / 32));
            ctx.font = `${fontPx}px system-ui, sans-serif`;
            ctx.lineJoin = 'round';
            if (entry) {
                const K = entry.intrinsics;
                for (const t of tags) {
                    const state = associationState(t.tag);
                    const color = COLORS[state];
                    const isSel = this.selected === t.tag.index;
                    const isHover = this.hover === t.tag.index;
                    ctx.strokeStyle = color;
                    ctx.lineWidth = isSel ? 3 : (isHover ? 2.5 : 1.5);
                    ctx.beginPath();
                    t.poly.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
                    ctx.closePath();
                    ctx.stroke();
                    // corner 0 marked so the corner order is visible
                    ctx.fillStyle = color;
                    ctx.beginPath();
                    ctx.arc(t.poly[0][0], t.poly[0][1], isSel ? 5 : 3.5, 0, Math.PI * 2);
                    ctx.fill();
                    if (t.tag.has_pose && K) {
                        const T = t.tag.T_camera_tag;
                        const L = this.axisLength(t.tag.family);
                        const o = projectEngineering(K, transformPoint(T, [0, 0, 0]));
                        const axes = [[[L, 0, 0], '#ff5050'], [[0, L, 0], '#50ff50'], [[0, 0, L], '#5080ff']];
                        for (const [p, c] of axes) {
                            const q = projectEngineering(K, transformPoint(T, p));
                            if (o && q) {
                                ctx.strokeStyle = c;
                                ctx.lineWidth = 2;
                                ctx.beginPath();
                                ctx.moveTo(o[0] * sx, o[1] * sy);
                                ctx.lineTo(q[0] * sx, q[1] * sy);
                                ctx.stroke();
                            }
                        }
                    }
                    const label = `${t.tag.id}` + (isSel || isHover ? ` ${state}${state === 'rejected' ? ': ' + t.tag.association.rejection : ''}` : '');
                    const tw = ctx.measureText(label).width + 8;
                    const top = Math.min(...t.poly.map((p) => p[1]));
                    const lx = Math.max(0, Math.min(this.canvas.width - tw, t.center[0] - tw / 2));
                    const ly = Math.max(fontPx + 4, top - 4);
                    ctx.fillStyle = 'rgba(0,0,0,0.7)';
                    ctx.fillRect(lx, ly - fontPx - 3, tw, fontPx + 5);
                    ctx.fillStyle = color;
                    ctx.fillText(label, lx + 4, ly - 2);
                }
            }
        }

        const ref = entry || latest;
        if (img && !entry) {
            badges.push(['overlay identity mismatch, waiting', 'warn']);
        }
        const cameras = (this.hello && this.hello.camera_sensors) || [];
        if (this.hello && cameras.length === 0) {
            badges.push(['no camera configured', 'info']);
        }
        if (ref) {
            if (!ref.has_image) {
                badges.push(['no image in frame', 'warn']);
            }
            if (ref.has_observations === false) {
                badges.push(['preview only, detection not run', 'info']);
            } else if (!ref.tags || ref.tags.length === 0) {
                badges.push(['no tags', 'info']);
            }
            if (!ref.intrinsics) {
                badges.push(['metric unavailable (no intrinsics)', 'warn']);
            }
            if (!ref.exposure_time_reliable) {
                badges.push(['exposure timing not reliable', 'warn']);
            }
            if (!ref.mounted) {
                badges.push(['camera not mounted', 'warn']);
            }
            if (ref.pose_at_exposure && ref.pose_at_exposure.status !== 'ok') {
                badges.push([`pose at exposure ${ref.pose_at_exposure.status}`, 'warn']);
            }
        } else if (!img && cameras.length > 0) {
            badges.push(['no detection frame yet', 'info']);
        }
        this.renderBadges(badges);

        this.updateAge();
        if (entry && entry.has_observations === false) {
            setText(this.overlayNote, `preview seq ${entry.sequence}: detection not run, no overlay`);
        } else if (entry) {
            setText(this.overlayNote, `overlay bound to seq ${entry.sequence}: ${(entry.tags || []).length} tags, detector ${fmt(entry.detector_processing_ms, 1)} ms` +
                (latest && latest.sequence !== entry.sequence ? `; newest processed seq ${latest.sequence}` : '') +
                (entry.frame_note ? `; ${entry.frame_note}` : ''));
        } else if (img && latest) {
            setText(this.overlayNote, `image seq ${img.header.sequence}, newest processed seq ${latest.sequence}: no overlay until they match`);
        } else {
            setText(this.overlayNote, '');
        }
        this.renderTagList(entry);
    }

    updateAge() {
        const img = this.current ? this.images.get(this.current) : null;
        if (!img) {
            this.frameInfo.textContent = '';
            return;
        }
        const h = img.header;
        // exposure age on the Pi clock: the Pi-now estimate minus exposure
        const piNow = this.piNow();
        const age = Number.isFinite(piNow) && typeof h.exposure_host_ms === 'number'
            ? fmtMs(Math.max(0, Math.round((piNow - h.exposure_host_ms) / 10) * 10)) : 'n/a';
        setText(this.frameInfo, `${h.camera} seq ${h.sequence} epoch ${h.epoch}, ${h.preview_width_px}x${h.preview_height_px} of ${h.width_px}x${h.height_px}, q${h.quality}, exposure age ${age}`);
    }

    renderBadges(list) {
        this.badgeSet.begin();
        for (const [text, cls] of list) {
            this.badgeSet.badge(text, text, cls);
        }
        this.badgeSet.end();
    }

    // Tag rows by index, updated in place; details rebuilt only when the
    // selected tag or its entry changed.
    renderTagList(entry) {
        const tags = entry ? entry.tags || [] : [];
        const seen = new Set();
        let position = 0;
        for (const t of tags) {
            seen.add(t.index);
            let r = this.tagRows.get(t.index);
            if (!r) {
                const row = el('div', 'tag-row');
                const dot = el('span', 'dot');
                const name = el('span');
                const assoc = el('span', 'muted');
                row.appendChild(dot);
                row.appendChild(name);
                row.appendChild(assoc);
                const index = t.index;
                row.addEventListener('click', () => this.selectTag(index));
                r = { row, dot, name, assoc };
                this.tagRows.set(t.index, r);
            }
            const state = associationState(t);
            setClass(r.row, 'tag-row' + (this.selected === t.index ? ' selected' : ''));
            if (r.color !== COLORS[state]) {
                r.color = COLORS[state];
                r.dot.style.background = r.color;
            }
            setText(r.name, `#${t.index} ${t.family} id ${t.id}`);
            setText(r.assoc, associationText(t));
            const at = this.tagList.children[position];
            if (at !== r.row) {
                this.tagList.insertBefore(r.row, at || null);
            }
            position += 1;
        }
        for (const [index, r] of this.tagRows) {
            if (!seen.has(index)) {
                r.row.remove();
                this.tagRows.delete(index);
            }
        }
        const sel = tags.find((t) => t.index === this.selected);
        const detailsKey = sel && entry ? `${entry.camera}:${entry.epoch}:${entry.sequence}:${sel.index}` : (tags.length ? 'hint' : '');
        if (detailsKey === this.detailsKey) {
            return;
        }
        this.detailsKey = detailsKey;
        this.tagDetails.textContent = '';
        if (!sel) {
            if (tags.length) {
                this.tagDetails.appendChild(el('div', 'muted', 'click a tag for details'));
            }
            return;
        }
        const table = el('table');
        row(table, ['family', sel.family]);
        row(table, ['id', String(sel.id)]);
        row(table, ['hamming', String(sel.hamming)]);
        row(table, ['decision margin', fmt(sel.decision_margin, 1)]);
        row(table, ['reprojection error', sel.reprojection_error_px === null || sel.reprojection_error_px === undefined ? 'n/a' : fmt(sel.reprojection_error_px, 3) + ' px']);
        row(table, ['ambiguity', sel.alternate_pose_ambiguity === null || sel.alternate_pose_ambiguity === undefined ? 'n/a' : fmt(sel.alternate_pose_ambiguity, 3)]);
        row(table, ['center', `${fmt(sel.center_px[0], 1)}, ${fmt(sel.center_px[1], 1)} px`]);
        if (sel.has_pose && sel.T_camera_tag) {
            const T = sel.T_camera_tag;
            row(table, ['pose (camera)', `x ${fmt(T.x_m)} y ${fmt(T.y_m)} z ${fmt(T.z_m)} m`]);
            row(table, ['', `roll ${fmt(T.roll_deg, 1)} pitch ${fmt(T.pitch_deg, 1)} yaw ${fmt(T.yaw_deg, 1)} deg`]);
        } else {
            row(table, ['pose', entry.intrinsics ? '2D decode only' : 'metric unavailable (no intrinsics)']);
        }
        row(table, ['association', associationText(sel)]);
        this.tagDetails.appendChild(table);
        const cands = sel.association ? sel.association.candidates || [] : [];
        if (cands.length) {
            this.tagDetails.appendChild(el('div', 'muted', 'candidates'));
            const ct = el('table');
            const head = el('tr');
            for (const h of ['object', 'mount', 'trans err m', 'head err deg', 'score', 'implied field']) {
                head.appendChild(el('th', undefined, h));
            }
            ct.appendChild(head);
            for (const c of cands) {
                const f = c.implied_field;
                row(ct, [c.object, c.mount.replace(c.object + '_', ''), fmt(c.translation_error_m, 3), fmt(c.heading_error_deg, 1), fmt(c.score, 3),
                    f ? `${fmt(f.x_m, 2)}, ${fmt(f.y_m, 2)}, ${fmt(f.heading_deg, 1)}` : '']);
            }
            this.tagDetails.appendChild(ct);
        }
    }
}
