// field_scene.js
// The 3D field. Static geometry, nominal landmarks and tag mounts come from
// hello.fields; the robot, its trail, the camera mount and the estimated
// landmark bodies from each snapshot. Scene axes are the field axes one to
// one: THREE.Object3D.DEFAULT_UP is +z, so field x is scene x (audience
// right), field y is scene y (toward the 0-degree wall), field z is scene z
// (up). No mirroring anywhere, so the right-handed field stays right-handed
// on screen; the wall labels come from the landmark ids in hello (blue goals
// at large x), which makes a handedness mistake visible instead of silent.
//
// Transform chains are applied explicitly with Object3D nesting:
//   field <- landmark (nominal or estimate, planar) <- tag surface (mount)
//   field <- robot origin (planar pose, tilt when attitude is valid)
//         <- camera engineering frame (T_robot_camera)
// Mesh origins never stand in for landmark or robot origins: bodies are
// translated inside their group so the group origin is the configured one.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { DEG, composePlanar, rotationOf, fmt, fmtMs } from './transforms.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const TRAIL_MAX = 4000;
const FRUSTUM_DEPTH_M = 0.45;

function matrixFromTransform(T) {
    const R = rotationOf(T);
    return new THREE.Matrix4().set(
        R[0], R[1], R[2], T.x_m || 0,
        R[3], R[4], R[5], T.y_m || 0,
        R[6], R[7], R[8], T.z_m || 0,
        0, 0, 0, 1);
}

function disposeObject(root) {
    root.traverse((o) => {
        if (o.geometry) {
            o.geometry.dispose();
        }
        const mats = Array.isArray(o.material) ? o.material : (o.material ? [o.material] : []);
        for (const m of mats) {
            if (m.map) {
                m.map.dispose();
            }
            m.dispose();
        }
    });
}

// Text as a sprite drawn on a 2D canvas: OS font, no network. height_m is
// the world height of one text line.
function textSprite(text, opts = {}) {
    const px = 28;
    const lineH = px + 6;
    const lines = String(text).split('\n');
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');
    ctx.font = `${px}px system-ui, sans-serif`;
    const w = Math.ceil(Math.max(...lines.map((l) => ctx.measureText(l).width))) + 16;
    const h = lines.length * lineH + 8;
    canvas.width = w;
    canvas.height = h;
    ctx.font = `${px}px system-ui, sans-serif`;
    ctx.fillStyle = opts.bg || 'rgba(0,0,0,0.65)';
    ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = opts.color || '#ffffff';
    ctx.textBaseline = 'top';
    lines.forEach((l, i) => ctx.fillText(l, 8, 4 + i * lineH));
    const tex = new THREE.CanvasTexture(canvas);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.minFilter = THREE.LinearFilter;
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
    const height_m = opts.height_m || 0.075;
    sprite.scale.set(height_m * w / lineH, height_m * h / lineH, 1);
    sprite.renderOrder = 10;
    sprite.userData.text = String(text);
    return sprite;
}

// Replaces a label sprite when its text changed; returns the current one.
function setLabel(parent, current, text, opts) {
    if (current && current.userData.text === String(text)) {
        return current;
    }
    if (current) {
        parent.remove(current);
        disposeObject(current);
    }
    const s = textSprite(text, opts);
    if (opts && opts.position) {
        s.position.copy(opts.position);
    }
    parent.add(s);
    return s;
}

const tagFaceCache = new Map();

// A printed tag look-alike: white carrier, black ring, the id in the middle.
function tagFaceMaterial(family, id, ghost) {
    const key = `${family}:${id}:${ghost ? 'g' : 's'}`;
    if (tagFaceCache.has(key)) {
        return tagFaceCache.get(key);
    }
    const c = document.createElement('canvas');
    c.width = 128;
    c.height = 128;
    const ctx = c.getContext('2d');
    ctx.fillStyle = '#f4f4f4';
    ctx.fillRect(0, 0, 128, 128);
    ctx.strokeStyle = '#111';
    ctx.lineWidth = 12;
    ctx.beginPath();
    ctx.arc(64, 64, 50, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = '#111';
    ctx.font = 'bold 44px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(String(id), 64, 66);
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    const mat = new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide, transparent: ghost, opacity: ghost ? 0.35 : 1 });
    tagFaceCache.set(key, mat);
    return mat;
}

// Body geometry with the landmark origin at (0, 0, 0) on the floor.
function landmarkGeometry(visual) {
    if (visual && visual.shape === 'octagonal_prism') {
        // across-flats to circumradius; rotateZ(pi/8) puts flats on the axes
        const rb = visual.base_across_flats_m / 2 / Math.cos(Math.PI / 8);
        const rt = visual.top_across_flats_m / 2 / Math.cos(Math.PI / 8);
        const g = new THREE.CylinderGeometry(rt, rb, visual.height_m, 8, 1, false);
        g.rotateX(Math.PI / 2);
        g.rotateZ(Math.PI / 8);
        g.translate(0, 0, visual.height_m / 2);
        return { geometry: g, height: visual.height_m, known: true };
    }
    if (visual && visual.shape === 'box') {
        const g = new THREE.BoxGeometry(visual.size_x_m, visual.size_y_m, visual.size_z_m);
        g.translate(0, 0, visual.size_z_m / 2);
        return { geometry: g, height: visual.size_z_m, known: true };
    }
    // unknown or absent visual: a labeled stand-in so the rest still loads
    const g = new THREE.CylinderGeometry(0.06, 0.06, 0.12, 16, 1, false);
    g.rotateX(Math.PI / 2);
    g.translate(0, 0, 0.06);
    return { geometry: g, height: 0.12, known: false };
}

// One tag mount in its own surface frame: +x outward normal, +z printed
// top, +y right-handed completion. The carrier plate sits behind the
// surface (-x), the printed face just in front of it.
function buildTagPlate(mount, visual, families, ghost) {
    const fam = families ? families[mount.family] : null;
    const cells = fam && fam.width_at_border > 0 ? fam.total_width / fam.width_at_border : 2;
    const side = mount.detection_size_m * cells;
    const w = visual && visual.tag_plate_width_m > 0 ? visual.tag_plate_width_m : side;
    const h = visual && visual.tag_plate_height_m > 0 ? visual.tag_plate_height_m : side;
    const t = visual && visual.tag_plate_thickness_m > 0 ? visual.tag_plate_thickness_m : 0.002;
    const group = new THREE.Group();
    const plateGeom = new THREE.BoxGeometry(t, w, h);
    plateGeom.translate(-t / 2, 0, 0);
    const plateMat = new THREE.MeshStandardMaterial({
        color: 0xdadada, transparent: true, opacity: ghost ? 0.3 : 1, depthWrite: !ghost,
    });
    group.add(new THREE.Mesh(plateGeom, plateMat));
    // plane u -> tag +y, v -> tag +z, normal -> tag +x (a cyclic permutation)
    const faceGeom = new THREE.PlaneGeometry(side, side);
    faceGeom.applyMatrix4(new THREE.Matrix4().set(
        0, 0, 1, 0.0006,
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 1));
    group.add(new THREE.Mesh(faceGeom, tagFaceMaterial(mount.family, mount.observed_id, ghost)));
    const label = textSprite(`id ${mount.observed_id}`, { height_m: 0.035, bg: ghost ? 'rgba(0,0,0,0.35)' : 'rgba(0,0,0,0.7)' });
    label.position.set(0.02, 0, h / 2 + 0.02);
    group.add(label);
    group.userData.info = `${mount.instance_id}\n${mount.family} id ${mount.observed_id}\ndetection size ${fmt(mount.detection_size_m, 4)} m`;
    return group;
}

// A landmark body plus its mounts, ghost (nominal) or solid (estimate).
function buildLandmarkBody(lm, families, ghost) {
    const group = new THREE.Group();
    const { geometry, height, known } = landmarkGeometry(lm.visual);
    const colorHex = lm.visual && lm.visual.color ? lm.visual.color : (known ? '#888888' : '#ff40ff');
    const color = new THREE.Color(colorHex);
    const material = new THREE.MeshStandardMaterial({
        color, transparent: true, opacity: ghost ? 0.22 : 1, depthWrite: !ghost, roughness: 0.7,
    });
    const mesh = new THREE.Mesh(geometry, material);
    group.add(mesh);
    const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(geometry, 25),
        new THREE.LineBasicMaterial({ color: ghost ? 0xcfd6e0 : 0xffffff, transparent: true, opacity: ghost ? 0.55 : 0.9 }));
    group.add(edges);
    for (const m of lm.mounts || []) {
        const plate = buildTagPlate(m, lm.visual, families, ghost);
        plate.matrixAutoUpdate = false;
        plate.matrix.copy(matrixFromTransform(m.T_landmark_tag));
        group.add(plate);
    }
    return { group, mesh, edges, material, height, known };
}

// Frustum lines in the engineering frame from the intrinsics: the four
// image corners back-projected (undistorted, so an approximation of the
// true field of view) to FRUSTUM_DEPTH_M, plus a tick on the image-top edge.
function frustumLines(K, color) {
    const d = FRUSTUM_DEPTH_M;
    const dir = (u, v) => {
        const xn = (u - K.cx_px) / K.fx_px;
        const yn = (v - K.cy_px) / K.fy_px;
        return new THREE.Vector3(d, -xn * d, -yn * d);
    };
    const w = K.width_px, h = K.height_px;
    const c = [dir(0, 0), dir(w, 0), dir(w, h), dir(0, h)];
    const o = new THREE.Vector3();
    const pts = [];
    for (let i = 0; i < 4; ++i) {
        pts.push(o, c[i], c[i], c[(i + 1) % 4]);
    }
    const topMid = c[0].clone().add(c[1]).multiplyScalar(0.5);
    const tick = topMid.clone().add(new THREE.Vector3(0, 0, 0.04));
    pts.push(c[0], tick, tick, c[1]);
    return new THREE.LineSegments(
        new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color }));
}

function shortId(id) {
    return String(id).replace(/^neutral_goal_/, 'neutral ').replace(/^blue_goal_/, 'blue ').replace(/^red_goal_/, 'red ');
}

export class FieldScene {
    constructor(container) {
        this.container = container;
        this.available = false;
        this.followRobot = false;
        this.fieldGroup = null;
        this.fieldCenter = new THREE.Vector3(1.8, 1.8, 0);
        this.fieldSpan = 3.6;
        this.nominal = new Map();      // landmark id -> {group, ...}
        this.estimates = new Map();    // landmark id -> {group, line, label, ...}
        this.landmarkDecls = new Map();
        this.families = {};
        this.pickables = [];
        this.trailPoints = 0;
        this.trailLastHostMs = -Infinity;
        this.trailKey = '';
        this.cameraMounts = new Map(); // camera id -> group
        this.highlighted = null;

        try {
            this.renderer = new THREE.WebGLRenderer({ antialias: true });
        } catch (e) {
            this.error = 'WebGL unavailable: ' + (e && e.message ? e.message : String(e));
            return;
        }
        this.available = true;
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        this.renderer.setClearColor(0x1b1f26, 1);
        container.appendChild(this.renderer.domElement);

        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(50, 1, 0.02, 200);
        this.camera.position.set(1.8, -3.2, 2.6);
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.12;
        this.controls.target.copy(this.fieldCenter);
        this.controls.update();

        this.scene.add(new THREE.HemisphereLight(0xffffff, 0x3a3f4a, 1.1));
        const sun = new THREE.DirectionalLight(0xffffff, 1.4);
        sun.position.set(2, -3, 5);
        this.scene.add(sun);

        this.dynamic = new THREE.Group();
        this.scene.add(this.dynamic);
        this.buildRobot({ length_m: 0.45, width_m: 0.45, height_m: 0.3, origin_x_m: 0, origin_y_m: 0 });
        this.buildTrail();

        this.raycaster = new THREE.Raycaster();
        this.pointer = new THREE.Vector2();
        this.tooltip = container.querySelector('#tooltip');
        this.renderer.domElement.addEventListener('pointermove', (e) => this.onPointerMove(e));
        this.renderer.domElement.addEventListener('pointerleave', () => this.hideTooltip());

        this.resize();
        // resize on the next frame: sizing the canvas inside the observer
        // callback would re-trigger it (undelivered-notifications error)
        const later = () => requestAnimationFrame(() => this.resize());
        if (typeof ResizeObserver !== 'undefined') {
            new ResizeObserver(later).observe(container);
        } else {
            window.addEventListener('resize', later);
        }
        this.renderer.setAnimationLoop(() => this.frame());
    }

    resize() {
        if (!this.available) {
            return;
        }
        const w = Math.max(64, this.container.clientWidth);
        const h = Math.max(64, this.container.clientHeight);
        this.renderer.setSize(w, h, false);
        this.renderer.domElement.style.width = w + 'px';
        this.renderer.domElement.style.height = h + 'px';
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
    }

    frame() {
        if (this.followRobot && this.robot.visible) {
            const target = this.robot.position.clone();
            const delta = target.sub(this.controls.target);
            this.controls.target.add(delta);
            this.camera.position.add(delta);
        }
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }

    // --- views ---

    resetView() {
        this.followRobot = false;
        const s = this.fieldSpan;
        this.controls.target.copy(this.fieldCenter);
        this.camera.position.set(this.fieldCenter.x, this.fieldCenter.y - 0.95 * s, 0.75 * s);
        this.controls.update();
    }

    topDown() {
        this.followRobot = false;
        const s = this.fieldSpan;
        this.controls.target.copy(this.fieldCenter);
        // a hair off the vertical keeps OrbitControls' spherical math regular
        this.camera.position.set(this.fieldCenter.x, this.fieldCenter.y - 0.001, 1.25 * s);
        this.controls.update();
    }

    setFollow(on) {
        this.followRobot = on;
    }

    // --- static field from hello ---

    buildField(hello) {
        if (!this.available) {
            return;
        }
        if (this.fieldGroup) {
            this.scene.remove(this.fieldGroup);
            disposeObject(this.fieldGroup);
        }
        for (const e of this.estimates.values()) {
            this.dynamic.remove(e.group);
            this.dynamic.remove(e.line);
            disposeObject(e.group);
            disposeObject(e.line);
        }
        this.estimates.clear();
        this.nominal.clear();
        this.landmarkDecls.clear();
        this.pickables = [];
        this.families = hello.tag_families || {};
        this.fieldGroup = new THREE.Group();
        this.scene.add(this.fieldGroup);

        const fields = hello.fields || [];
        let span = 3.6;
        let cx = 1.8, cy = 1.8;
        for (const field of fields) {
            const d = field.dimensions;
            if (d) {
                span = Math.max(d.inside_x_m, d.inside_y_m);
                cx = d.inside_x_m / 2;
                cy = d.inside_y_m / 2;
                this.buildFloor(d);
            } else {
                this.buildFallbackFloor(field);
            }
            for (const f of field.features || []) {
                this.buildFeature(f);
            }
            for (const lm of field.landmarks || []) {
                this.landmarkDecls.set(lm.id, lm);
                const body = buildLandmarkBody(lm, this.families, true);
                body.group.position.set(lm.nominal.x_m, lm.nominal.y_m, 0);
                body.group.rotation.z = lm.nominal.heading_deg * DEG;
                body.group.userData.info = `${lm.id} (nominal placement)\n` +
                    `x ${fmt(lm.nominal.x_m)} m  y ${fmt(lm.nominal.y_m)} m  heading ${fmt(lm.nominal.heading_deg, 1)} deg\n` +
                    (lm.visual ? `${lm.visual.shape}` : 'no visual declared: stand-in drawn') +
                    `\n${(lm.mounts || []).length} tag mounts`;
                body.label = textSprite(shortId(lm.id) + (body.known ? '' : ' (no visual)'), { height_m: 0.07, bg: 'rgba(0,0,0,0.45)', color: '#d8dde6' });
                body.label.position.set(0, 0, body.height + 0.09);
                body.group.add(body.label);
                this.fieldGroup.add(body.group);
                this.nominal.set(lm.id, body);
                this.pickables.push(body.group);
            }
            this.buildWallLabels(field);
        }
        this.fieldCenter.set(cx, cy, 0);
        this.fieldSpan = span;
        this.buildAxes();
        this.resetView();
    }

    buildFloor(d) {
        const floor = new THREE.Mesh(
            new THREE.PlaneGeometry(d.inside_x_m, d.inside_y_m),
            new THREE.MeshStandardMaterial({ color: 0x4a5058, roughness: 0.95 }));
        floor.position.set(d.inside_x_m / 2, d.inside_y_m / 2, -0.001);
        floor.userData.info = `field floor ${fmt(d.inside_x_m, 3)} x ${fmt(d.inside_y_m, 3)} m\ntile ${fmt(d.tile_m, 4)} m\n${d.source || ''}`;
        this.fieldGroup.add(floor);
        this.pickables.push(floor);
        if (d.tile_m > 0) {
            const pts = [];
            for (let x = 0; x <= d.inside_x_m + 1e-6; x += d.tile_m) {
                pts.push(new THREE.Vector3(x, 0, 0.001), new THREE.Vector3(x, d.inside_y_m, 0.001));
            }
            for (let y = 0; y <= d.inside_y_m + 1e-6; y += d.tile_m) {
                pts.push(new THREE.Vector3(0, y, 0.001), new THREE.Vector3(d.inside_x_m, y, 0.001));
            }
            this.fieldGroup.add(new THREE.LineSegments(
                new THREE.BufferGeometry().setFromPoints(pts),
                new THREE.LineBasicMaterial({ color: 0x6b727c })));
        }
        const t = d.wall_thickness_m, h = d.wall_height_m;
        const wallMat = new THREE.MeshStandardMaterial({ color: 0x8d949e, transparent: true, opacity: 0.45, roughness: 0.6 });
        const walls = [
            [d.inside_x_m + 2 * t, t, h, d.inside_x_m / 2, -t / 2],
            [d.inside_x_m + 2 * t, t, h, d.inside_x_m / 2, d.inside_y_m + t / 2],
            [t, d.inside_y_m, h, -t / 2, d.inside_y_m / 2],
            [t, d.inside_y_m, h, d.inside_x_m + t / 2, d.inside_y_m / 2],
        ];
        for (const [sx, sy, sz, x, y] of walls) {
            const wall = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), wallMat);
            wall.position.set(x, y, sz / 2);
            wall.userData.info = `perimeter wall ${fmt(h, 3)} m high, ${fmt(t, 4)} m thick`;
            this.fieldGroup.add(wall);
        }
    }

    buildFallbackFloor(field) {
        // no declared dimensions: a plain plane around the landmarks
        let maxx = 1, maxy = 1;
        for (const lm of field.landmarks || []) {
            maxx = Math.max(maxx, lm.nominal.x_m + 0.5);
            maxy = Math.max(maxy, lm.nominal.y_m + 0.5);
        }
        const floor = new THREE.Mesh(new THREE.PlaneGeometry(maxx, maxy),
            new THREE.MeshStandardMaterial({ color: 0x3e444c }));
        floor.position.set(maxx / 2, maxy / 2, -0.001);
        floor.userData.info = 'field dimensions not declared';
        this.fieldGroup.add(floor);
        this.fieldGroup.add(textSprite('no field dimensions declared', { height_m: 0.1 }));
    }

    buildFeature(f) {
        const isTape = f.kind === 'tape';
        const sz = isTape ? Math.max(f.size_z_m, 0.003) : f.size_z_m;
        const mesh = new THREE.Mesh(
            new THREE.BoxGeometry(f.size_x_m, f.size_y_m, sz),
            new THREE.MeshStandardMaterial({ color: new THREE.Color(f.color || '#999999'), roughness: 0.8 }));
        mesh.position.set(f.x_m, f.y_m, isTape ? f.z_m + sz / 2 : f.z_m);
        mesh.rotation.z = (f.yaw_deg || 0) * DEG;
        mesh.userData.info = `${f.id} (${f.kind}, static display geometry)\n` +
            `${fmt(f.size_x_m)} x ${fmt(f.size_y_m)} x ${fmt(f.size_z_m)} m at ${fmt(f.x_m)}, ${fmt(f.y_m)}, ${fmt(f.z_m)}\n${f.note || ''}`;
        this.fieldGroup.add(mesh);
        this.pickables.push(mesh);
    }

    buildWallLabels(field) {
        const d = field.dimensions;
        if (!d) {
            return;
        }
        const mean = (prefix) => {
            const xs = (field.landmarks || []).filter((l) => l.id.startsWith(prefix)).map((l) => l.nominal.x_m);
            return xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : null;
        };
        const red = mean('red_'), blue = mean('blue_');
        // wall names follow the landmark ids, never an assumption
        let low = 'x = 0 wall', high = `x = ${fmt(d.inside_x_m, 2)} wall`;
        if (red !== null && blue !== null) {
            low = blue > red ? 'red wall (x = 0)' : 'blue wall (x = 0)';
            high = blue > red ? `blue wall (x = ${fmt(d.inside_x_m, 2)})` : `red wall (x = ${fmt(d.inside_x_m, 2)})`;
        }
        const l1 = textSprite(low, { height_m: 0.11, color: blue !== null && blue > red ? '#ff8a8a' : '#8ad0ff' });
        l1.position.set(-0.15, d.inside_y_m / 2, d.wall_height_m + 0.15);
        const l2 = textSprite(high, { height_m: 0.11, color: blue !== null && blue > red ? '#8ad0ff' : '#ff8a8a' });
        l2.position.set(d.inside_x_m + 0.15, d.inside_y_m / 2, d.wall_height_m + 0.15);
        const l3 = textSprite('audience (y = 0)', { height_m: 0.1 });
        l3.position.set(d.inside_x_m / 2, -0.2, d.wall_height_m + 0.1);
        this.fieldGroup.add(l1, l2, l3);
    }

    buildAxes() {
        const len = 0.5;
        const ax = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.01), len, 0xff4040, 0.1, 0.05);
        const ay = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0.01), len, 0x40ff40, 0.1, 0.05);
        const az = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0.01), len * 0.6, 0x4080ff, 0.1, 0.05);
        const lx = textSprite('+x', { height_m: 0.08, color: '#ff8080' });
        lx.position.set(len + 0.08, 0, 0.03);
        const ly = textSprite('+y', { height_m: 0.08, color: '#80ff80' });
        ly.position.set(0, len + 0.08, 0.03);
        const lz = textSprite('+z', { height_m: 0.08, color: '#80a0ff' });
        lz.position.set(0, 0, len * 0.6 + 0.08);
        this.fieldGroup.add(ax, ay, az, lx, ly, lz);
    }

    // --- robot ---

    buildRobot(body) {
        if (this.robot) {
            this.dynamic.remove(this.robot);
            disposeObject(this.robot);
            this.pickables = this.pickables.filter((p) => p !== this.robot);
        }
        const robot = new THREE.Group();
        robot.visible = false;
        const box = new THREE.Mesh(
            new THREE.BoxGeometry(body.length_m, body.width_m, body.height_m),
            new THREE.MeshStandardMaterial({ color: 0xff9f43, transparent: true, opacity: 0.85, roughness: 0.6 }));
        // body center relative to the robot origin, which stays the group origin
        box.position.set(body.origin_x_m, body.origin_y_m, body.height_m / 2);
        robot.add(box);
        robot.add(new THREE.LineSegments(new THREE.EdgesGeometry(box.geometry),
            new THREE.LineBasicMaterial({ color: 0xffffff })).translateX(body.origin_x_m).translateY(body.origin_y_m).translateZ(body.height_m / 2));
        // the exact robot origin and its forward direction
        const origin = new THREE.Mesh(new THREE.SphereGeometry(0.025, 16, 12),
            new THREE.MeshBasicMaterial({ color: 0xffffff }));
        origin.position.set(0, 0, 0.01);
        robot.add(origin);
        robot.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.012),
            body.length_m * 0.9, 0xff3030, 0.08, 0.05));
        robot.add(new THREE.AxesHelper(0.2));
        this.robotLabel = textSprite('robot', { height_m: 0.07, position: new THREE.Vector3(0, 0, body.height_m + 0.1) });
        this.robotLabel.position.set(0, 0, body.height_m + 0.1);
        robot.add(this.robotLabel);
        robot.userData.info = 'robot';
        this.robot = robot;
        this.robotBody = body;
        this.dynamic.add(robot);
        this.cameraMounts.clear();
        this.pickables.push(robot);
    }

    buildTrail() {
        this.trailBuffer = new Float32Array(TRAIL_MAX * 3);
        this.trailAttr = new THREE.BufferAttribute(this.trailBuffer, 3).setUsage(THREE.DynamicDrawUsage);
        const geom = new THREE.BufferGeometry().setAttribute('position', this.trailAttr);
        geom.setDrawRange(0, 0);
        this.trail = new THREE.Line(geom, new THREE.LineBasicMaterial({ color: 0xffd166 }));
        this.trail.frustumCulled = false;
        this.dynamic.add(this.trail);
    }

    clearTrail() {
        this.trailPoints = 0;
        this.trailLastHostMs = -Infinity;
        this.trail.geometry.setDrawRange(0, 0);
    }

    pushTrailPoint(x, y, z) {
        if (this.trailPoints >= TRAIL_MAX) {
            this.trailBuffer.copyWithin(0, 3);
            this.trailPoints = TRAIL_MAX - 1;
        }
        this.trailBuffer.set([x, y, z], this.trailPoints * 3);
        this.trailPoints += 1;
    }

    // --- per snapshot ---

    updateSnapshot(snap, hello) {
        if (!this.available) {
            return;
        }
        const robot = snap.robot;
        if (hello && hello.robot_body && this.robotBody !== hello.robot_body) {
            this.buildRobot(hello.robot_body);
        }
        if (robot && robot.valid) {
            const f = robot.field;
            this.robot.visible = true;
            this.robot.position.set(f.x_m, f.y_m, 0);
            const att = robot.attitude;
            const tilt = att && att.valid;
            // heading from the planar pose; roll and pitch only when measured
            const R = rotationOf({
                roll_deg: tilt ? att.roll_deg : 0,
                pitch_deg: tilt ? att.pitch_deg : 0,
                yaw_deg: f.heading_deg,
            });
            this.robot.quaternion.setFromRotationMatrix(new THREE.Matrix4().set(
                R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0, 0, 0, 0, 1));
            const attText = tilt
                ? `roll ${fmt(att.roll_deg, 1)} pitch ${fmt(att.pitch_deg, 1)} deg (${att.source}, age ${fmtMs(att.age_ms)})`
                : (att && att.assumed_level ? 'attitude assumed level' : 'attitude unavailable');
            this.robot.userData.info = `robot origin\nx ${fmt(f.x_m)} m  y ${fmt(f.y_m)} m  heading ${fmt(f.heading_deg, 1)} deg\n` +
                `pose age ${fmtMs(robot.age_ms)}  confidence ${fmt(robot.confidence, 2)}\n${attText}`;
            this.robotLabel = setLabel(this.robot, this.robotLabel,
                tilt ? 'robot' : (att && att.assumed_level ? 'robot (level assumed)' : 'robot (no attitude)'),
                { height_m: 0.07, position: new THREE.Vector3(0, 0, this.robotBody.height_m + 0.1) });
        } else {
            this.robot.visible = false;
        }

        // trail: odometry frame -> field through this snapshot's anchor
        const key = `${snap.session.id}/${snap.session.reset_count}/${robot ? robot.odometry_epoch : ''}/${robot ? robot.anchor_revision : ''}`;
        if (key !== this.trailKey) {
            this.trailKey = key;
            this.clearTrail();
        }
        if (robot && Array.isArray(snap.trail)) {
            const anchor = robot.field_from_odom;
            for (const e of snap.trail) {
                if (e.epoch !== robot.odometry_epoch || e.host_ms <= this.trailLastHostMs) {
                    continue;
                }
                const p = composePlanar(anchor, e);
                this.pushTrailPoint(p.x_m, p.y_m, 0.015);
                this.trailLastHostMs = e.host_ms;
            }
            this.trailAttr.needsUpdate = true;
            this.trail.geometry.setDrawRange(0, this.trailPoints);
        }

        this.updateCameraMounts(snap, hello);
        this.updateEstimates(snap);
    }

    updateCameraMounts(snap, hello) {
        const seen = new Set();
        const frames = snap.detection_frames || [];
        const fromHello = (hello && hello.cameras) || [];
        const specs = [];
        for (const f of frames) {
            specs.push({ id: f.camera, frame_id: f.frame_id, T: f.T_robot_camera, K: f.intrinsics, mounted: f.mounted });
        }
        for (const c of fromHello) {
            if (!specs.some((s) => s.id === c.resource_id || s.frame_id === c.frame_id)) {
                specs.push({ id: c.resource_id, T: c.T_robot_camera, K: c.intrinsics, mounted: c.mounted });
            }
        }
        for (const s of specs) {
            seen.add(s.id);
            const sig = `${s.mounted}|${JSON.stringify(s.K)}|${s.T ? [s.T.x_m, s.T.y_m, s.T.z_m, s.T.roll_deg, s.T.pitch_deg, s.T.yaw_deg].join(',') : ''}`;
            const existing = this.cameraMounts.get(s.id);
            if (existing && existing.userData.sig === sig) {
                continue;
            }
            if (existing) {
                this.robot.remove(existing);
                disposeObject(existing);
            }
            const g = new THREE.Group();
            g.userData.sig = sig;
            if (s.T) {
                g.matrixAutoUpdate = false;
                g.matrix.copy(matrixFromTransform(s.T));
            }
            const calibrated = !!s.K;
            const color = calibrated && s.mounted ? 0x4fc3f7 : 0xffb347;
            if (calibrated) {
                g.add(frustumLines(s.K, color));
            }
            const marker = new THREE.Mesh(new THREE.SphereGeometry(0.018, 12, 8), new THREE.MeshBasicMaterial({ color }));
            g.add(marker);
            g.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.12, color, 0.03, 0.02));
            const label = textSprite(`${s.id}${calibrated ? '' : ' (no intrinsics)'}${s.mounted ? '' : ' (unmounted)'}`,
                { height_m: 0.05, color: '#bfe9ff' });
            label.position.set(0, 0, 0.06);
            g.add(label);
            g.userData.info = `camera ${s.id}\n${s.mounted ? 'T_robot_camera ' + fmt(s.T.x_m) + ', ' + fmt(s.T.y_m) + ', ' + fmt(s.T.z_m) + ' m rpy ' + fmt(s.T.roll_deg, 1) + ', ' + fmt(s.T.pitch_deg, 1) + ', ' + fmt(s.T.yaw_deg, 1) + ' deg' : 'not mounted: drawn at the robot origin'}\n` +
                (calibrated ? 'frustum from intrinsics' : 'no intrinsics: marker only');
            this.robot.add(g);
            this.cameraMounts.set(s.id, g);
        }
        for (const [id, g] of this.cameraMounts) {
            if (!seen.has(id)) {
                this.robot.remove(g);
                disposeObject(g);
                this.cameraMounts.delete(id);
            }
        }
    }

    updateEstimates(snap) {
        const seen = new Set();
        for (const o of snap.field_objects || []) {
            if (o.source !== 'observed') {
                continue;   // field_map entries are nominal only: the ghost already shows them
            }
            seen.add(o.id);
            let e = this.estimates.get(o.id);
            if (!e) {
                const decl = this.landmarkDecls.get(o.id) || { id: o.id, visual: null, mounts: [] };
                const body = buildLandmarkBody(decl, this.families, false);
                const line = new THREE.Line(
                    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
                    new THREE.LineBasicMaterial({ color: 0xffd166 }));
                line.frustumCulled = false;
                e = { ...body, line, label: null, id: o.id };
                this.dynamic.add(e.group, line);
                this.estimates.set(o.id, e);
                this.pickables.push(e.group);
            }
            e.group.position.set(o.pose.x_m, o.pose.y_m, 0);
            e.group.rotation.z = o.pose.heading_deg * DEG;
            const age = o.age_ms;
            // fresh estimates solid, old ones fade; never below a visible floor
            const opacity = typeof age === 'number' ? Math.max(0.3, 1 - Math.max(0, age - 2000) / 8000) : 0.3;
            e.material.opacity = o.valid ? opacity : 0.3;
            e.edges.material.opacity = e.material.opacity;
            const nominal = o.nominal;
            const pos = e.line.geometry.attributes.position;
            if (nominal) {
                pos.setXYZ(0, nominal.x_m, nominal.y_m, 0.01);
                pos.setXYZ(1, o.pose.x_m, o.pose.y_m, 0.01);
                e.line.visible = true;
            } else {
                e.line.visible = false;
            }
            pos.needsUpdate = true;
            const ageText = typeof age === 'number' ? fmtMs(Math.round(age / 100) * 100) : 'never';
            const text = `${shortId(o.id)} est.\n` +
                (nominal ? `d ${fmt(o.displacement_m, 3)} m  dh ${fmt(o.heading_error_deg, 1)} deg\n` : 'no nominal\n') +
                `age ${ageText}${o.valid ? '' : ' (invalid)'}`;
            e.label = setLabel(e.group, e.label, text, {
                height_m: 0.06, bg: 'rgba(40,30,0,0.75)', color: '#ffe9a8',
                position: new THREE.Vector3(0, 0, e.height + 0.22),
            });
            e.group.userData.info = `${o.id} (${o.source}${o.observed ? ', observed this cycle' : ''})\n` +
                `x ${fmt(o.pose.x_m)} m  y ${fmt(o.pose.y_m)} m  heading ${fmt(o.pose.heading_deg, 1)} deg\n` +
                (nominal ? `nominal ${fmt(nominal.x_m)}, ${fmt(nominal.y_m)}, ${fmt(nominal.heading_deg, 1)} deg\n` +
                    `displacement ${fmt(o.displacement_m, 3)} m  heading error ${fmt(o.heading_error_deg, 2)} deg\n` : '') +
                `age ${fmtMs(age)}  confidence ${fmt(o.confidence, 2)}  valid ${o.valid}\n` +
                `last ${o.last_source} / ${o.last_feature} seq ${o.last_source_sequence}\n` +
                `anchor rev ${o.anchor_revision}  odometry epoch ${o.odometry_epoch}`;
        }
        for (const [id, e] of this.estimates) {
            if (!seen.has(id)) {
                this.dynamic.remove(e.group, e.line);
                disposeObject(e.group);
                disposeObject(e.line);
                this.estimates.delete(id);
                this.pickables = this.pickables.filter((p) => p !== e.group);
            }
        }
    }

    // Everything bound to a session: estimates, trail, camera mounts.
    clearSession() {
        if (!this.available) {
            return;
        }
        for (const e of this.estimates.values()) {
            this.dynamic.remove(e.group, e.line);
            disposeObject(e.group);
            disposeObject(e.line);
        }
        this.estimates.clear();
        this.clearTrail();
        this.trailKey = '';
        this.robot.visible = false;
    }

    setHighlight(id) {
        const apply = (e, on) => {
            if (e && e.material && e.material.emissive) {
                e.material.emissive.setHex(on ? 0x335577 : 0x000000);
            }
        };
        if (this.highlighted) {
            apply(this.nominal.get(this.highlighted), false);
            apply(this.estimates.get(this.highlighted), false);
        }
        this.highlighted = id;
        if (id) {
            apply(this.nominal.get(id), true);
            apply(this.estimates.get(id), true);
        }
    }

    // --- hover ---

    onPointerMove(e) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        this.pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
        this.raycaster.setFromCamera(this.pointer, this.camera);
        const hits = this.raycaster.intersectObjects(this.pickables, true);
        let info = null;
        for (const h of hits) {
            let o = h.object;
            while (o && !o.userData.info) {
                o = o.parent;
            }
            if (o && o.userData.info) {
                info = o.userData.info;
                break;
            }
        }
        if (!info) {
            this.hideTooltip();
            return;
        }
        this.tooltip.textContent = info;
        this.tooltip.style.display = 'block';
        const x = e.clientX - rect.left + 14;
        const y = e.clientY - rect.top + 14;
        this.tooltip.style.left = Math.min(x, rect.width - this.tooltip.offsetWidth - 8) + 'px';
        this.tooltip.style.top = Math.min(y, rect.height - this.tooltip.offsetHeight - 8) + 'px';
    }

    hideTooltip() {
        if (this.tooltip) {
            this.tooltip.style.display = 'none';
        }
    }
}
